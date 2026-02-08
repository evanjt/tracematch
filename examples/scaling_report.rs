//! Scaling report: runs predefined scenarios and outputs a markdown table.
//!
//! Run with: `cargo run --example scaling_report --features synthetic --release`
//!
//! Prints performance characteristics at various scales to identify
//! where section detection breaks down (the "cliff edge").

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::Instant;
use tracematch::synthetic::{CorridorConfig, CorridorPattern, SyntheticScenario};
use tracematch::{
    GpsPoint, NoopProgress, SectionConfig,
    compute_grid_cells, detect_sections_incremental, detect_sections_multiscale,
    detect_sections_optimized, grid_filtered_pairs,
};

// ============================================================================
// Peak Memory Tracking Allocator
// ============================================================================

struct TrackingAllocator;

static CURRENT_BYTES: AtomicUsize = AtomicUsize::new(0);
static PEAK_BYTES: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for TrackingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { System.alloc(layout) };
        if !ptr.is_null() {
            let current = CURRENT_BYTES.fetch_add(layout.size(), Ordering::Relaxed) + layout.size();
            PEAK_BYTES.fetch_max(current, Ordering::Relaxed);
        }
        ptr
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        CURRENT_BYTES.fetch_sub(layout.size(), Ordering::Relaxed);
        unsafe { System.dealloc(ptr, layout) };
    }
}

#[global_allocator]
static ALLOCATOR: TrackingAllocator = TrackingAllocator;

fn reset_peak_memory() {
    CURRENT_BYTES.store(0, Ordering::Relaxed);
    PEAK_BYTES.store(0, Ordering::Relaxed);
}

fn get_peak_memory_mb() -> f64 {
    PEAK_BYTES.load(Ordering::Relaxed) as f64 / (1024.0 * 1024.0)
}

// ============================================================================
// Scenario Runner
// ============================================================================

struct ScenarioResult {
    #[allow(dead_code)]
    name: String,
    activities: usize,
    pairs: usize,
    total_points: usize,
    sections_found: usize,
    overlaps_found: u32,
    time_ms: u128,
    estimated_memory_mb: f64,
    peak_memory_mb: f64,
}

fn run_scenario(name: &str, scenario: &SyntheticScenario) -> ScenarioResult {
    let dataset = scenario.generate();
    let groups = dataset.route_groups();
    let config = SectionConfig::default();
    let estimated_memory_mb = dataset.metadata.estimated_memory_bytes as f64 / (1024.0 * 1024.0);

    reset_peak_memory();
    let start = Instant::now();
    let result = detect_sections_multiscale(
        &dataset.tracks,
        &dataset.sport_types,
        &groups,
        &config,
    );
    let elapsed = start.elapsed();
    let peak_memory_mb = get_peak_memory_mb();

    ScenarioResult {
        name: name.to_string(),
        activities: dataset.tracks.len(),
        pairs: dataset.metadata.total_pairs,
        total_points: dataset.metadata.total_points,
        sections_found: result.sections.len(),
        overlaps_found: result.stats.overlaps_found,
        time_ms: elapsed.as_millis(),
        estimated_memory_mb,
        peak_memory_mb,
    }
}

fn run_scenario_optimized(name: &str, scenario: &SyntheticScenario) -> ScenarioResult {
    let dataset = scenario.generate();
    let config = SectionConfig::default();
    let estimated_memory_mb = dataset.metadata.estimated_memory_bytes as f64 / (1024.0 * 1024.0);

    reset_peak_memory();
    let start = Instant::now();
    let result = detect_sections_optimized(
        &dataset.tracks,
        &dataset.sport_types,
        &config,
    );
    let elapsed = start.elapsed();
    let peak_memory_mb = get_peak_memory_mb();

    ScenarioResult {
        name: name.to_string(),
        activities: dataset.tracks.len(),
        pairs: dataset.metadata.total_pairs,
        total_points: dataset.metadata.total_points,
        sections_found: result.len(),
        overlaps_found: 0,
        time_ms: elapsed.as_millis(),
        estimated_memory_mb,
        peak_memory_mb,
    }
}

fn main() {
    println!("# Section Detection Scaling Report\n");
    println!("Config: `SectionConfig::default()` (presets: short, medium, long, extra_long, ultra_long)\n");

    // ---- Scaling curve (multiscale) ----
    println!("## 1. Scaling Curve — Multiscale (10km corridor, 80% overlap)\n");
    println!(
        "| Activities | Pairs      | Points    | Overlaps | Sections | Time       | Est. Input | Peak Alloc |"
    );
    println!(
        "|------------|------------|-----------|----------|----------|------------|------------|------------|"
    );

    for count in [50, 100, 250, 500, 1000] {
        let scenario = SyntheticScenario::with_activity_count(count, 10_000.0, 0.8);
        let r = run_scenario("scaling", &scenario);
        println!(
            "| {:>10} | {:>10} | {:>9} | {:>8} | {:>8} | {:>8}ms | {:>7.1} MB | {:>7.1} MB |",
            r.activities,
            format_number(r.pairs),
            format_number(r.total_points),
            r.overlaps_found,
            r.sections_found,
            r.time_ms,
            r.estimated_memory_mb,
            r.peak_memory_mb,
        );
    }

    // ---- Scaling curve (optimized) ----
    println!("\n## 2. Scaling Curve — Optimized (10km corridor, 80% overlap)\n");
    println!(
        "| Activities | Points    | Sections | Time       | Peak Alloc |"
    );
    println!(
        "|------------|-----------|----------|------------|------------|"
    );

    for count in [50, 100, 250, 500, 1000] {
        let scenario = SyntheticScenario::with_activity_count(count, 10_000.0, 0.8);
        let r = run_scenario_optimized("scaling_opt", &scenario);
        println!(
            "| {:>10} | {:>9} | {:>8} | {:>8}ms | {:>7.1} MB |",
            r.activities,
            format_number(r.total_points),
            r.sections_found,
            r.time_ms,
            r.peak_memory_mb,
        );
    }

    // ---- Route length impact ----
    println!("\n## 3. Route Length Impact (50 activities, 60% overlap)\n");
    println!(
        "| Corridor  | Points    | Overlaps | Sections | Time       | Peak Alloc |"
    );
    println!(
        "|-----------|-----------|----------|----------|------------|------------|"
    );

    for length_km in [10, 50, 100] {
        let length_m = length_km as f64 * 1000.0;
        let scenario = SyntheticScenario::with_activity_count(50, length_m, 0.6);
        let r = run_scenario("route_length", &scenario);
        println!(
            "| {:>5}km  | {:>9} | {:>8} | {:>8} | {:>8}ms | {:>7.1} MB |",
            length_km,
            format_number(r.total_points),
            r.overlaps_found,
            r.sections_found,
            r.time_ms,
            r.peak_memory_mb,
        );
    }

    // ---- Overlap density ----
    println!("\n## 4. Overlap Density (100 activities, 10km corridor)\n");
    println!(
        "| Overlap | Overlaps | Sections | Time       | Peak Alloc |"
    );
    println!(
        "|---------|----------|----------|------------|------------|"
    );

    for overlap_pct in [10, 30, 50, 70, 90] {
        let overlap_frac = overlap_pct as f64 / 100.0;
        let scenario = SyntheticScenario::with_activity_count(100, 10_000.0, overlap_frac);
        let r = run_scenario("overlap", &scenario);
        println!(
            "| {:>5}%  | {:>8} | {:>8} | {:>8}ms | {:>7.1} MB |",
            overlap_pct, r.overlaps_found, r.sections_found, r.time_ms, r.peak_memory_mb,
        );
    }

    // ---- No overlap worst case ----
    println!("\n## 5. No Overlap Worst Case (pure overhead)\n");
    println!(
        "| Activities | Pairs      | Time       | Peak Alloc |"
    );
    println!(
        "|------------|------------|------------|------------|"
    );

    for count in [50, 100, 250, 500] {
        let scenario = SyntheticScenario::with_no_overlap(count);
        let r = run_scenario("no_overlap", &scenario);
        println!(
            "| {:>10} | {:>10} | {:>8}ms | {:>7.1} MB |",
            r.activities,
            format_number(r.pairs),
            r.time_ms,
            r.peak_memory_mb,
        );
    }

    // ---- Predefined scenarios ----
    println!("\n## 6. Predefined Scenarios\n");
    println!(
        "| Scenario         | Activities | Corridors | Sections | Time       | Peak Alloc |"
    );
    println!(
        "|------------------|------------|-----------|----------|------------|------------|"
    );

    let scenarios: Vec<(&str, SyntheticScenario)> = vec![
        ("standard_cycling", SyntheticScenario::standard_cycling()),
        ("long_sections", SyntheticScenario::long_sections()),
        ("no_overlap", SyntheticScenario::no_overlap()),
        ("multi_corridor", SyntheticScenario::multi_corridor()),
        ("urban_commute", SyntheticScenario::urban_commute()),
    ];

    for (name, scenario) in &scenarios {
        let corridors = scenario.corridors.len();
        let r = run_scenario(name, scenario);
        println!(
            "| {:>16} | {:>10} | {:>9} | {:>8} | {:>8}ms | {:>7.1} MB |",
            name, r.activities, corridors, r.sections_found, r.time_ms, r.peak_memory_mb,
        );
    }

    // ---- Grid filtering impact ----
    println!("\n## 7. Grid Filtering Impact (pair reduction)\n");
    println!(
        "| Activities | Exhaustive | Grid-filt. | Reduction | Speedup |"
    );
    println!(
        "|------------|------------|------------|-----------|---------|"
    );

    for count in [50, 100, 250, 500] {
        let scenario = SyntheticScenario::with_activity_count(count, 10_000.0, 0.8);
        let dataset = scenario.generate();
        let n = dataset.tracks.len();
        let exhaustive = n * (n - 1) / 2;

        let track_cells: Vec<_> = dataset
            .tracks
            .iter()
            .map(|(_, pts)| compute_grid_cells(pts))
            .collect();
        let filtered = grid_filtered_pairs(&track_cells).len();
        let reduction = (1.0 - filtered as f64 / exhaustive as f64) * 100.0;
        let speedup = exhaustive as f64 / filtered.max(1) as f64;

        println!(
            "| {:>10} | {:>10} | {:>10} | {:>8.1}% | {:>5.1}x |",
            count,
            format_number(exhaustive),
            format_number(filtered),
            reduction,
            speedup,
        );
    }

    println!("\n_Note: Grid filtering eliminates cross-region pairs. Single-corridor synthetic data");
    println!("shows 0% reduction (all activities nearby). Real-world data with activities across");
    println!("multiple cities shows 60-90% reduction._\n");

    // ---- Incremental detection ----
    println!("## 8. Incremental Detection (add N new to existing)\n");
    println!(
        "| Existing | New | Sections | Incr. time | Full time  | Speedup | Matched |"
    );
    println!(
        "|----------|-----|----------|------------|------------|---------|---------|"
    );

    let progress = Arc::new(NoopProgress) as Arc<dyn tracematch::DetectionProgressCallback>;

    for (existing_count, new_count) in [(50, 5), (100, 10), (250, 25)] {
        let scenario = SyntheticScenario::with_activity_count(existing_count, 10_000.0, 0.8);
        let dataset = scenario.generate();
        let config = SectionConfig::default();
        let groups = dataset.route_groups();

        // Full detection to establish existing sections
        let full_result = detect_sections_multiscale(
            &dataset.tracks,
            &dataset.sport_types,
            &groups,
            &config,
        );

        if full_result.sections.is_empty() {
            continue;
        }

        // Generate new activities
        let new_scenario = SyntheticScenario {
            origin: GpsPoint::new(47.37, 8.55),
            activity_count: new_count,
            corridors: vec![CorridorConfig {
                length_meters: 10_000.0,
                overlap_fraction: 1.0,
                pattern: CorridorPattern::Winding,
                approach_length: 500.0,
            }],
            gps_noise_sigma_meters: 3.0,
            seed: existing_count as u64 * 7 + new_count as u64,
        };
        let new_dataset = new_scenario.generate();
        let new_tracks: Vec<(String, Vec<GpsPoint>)> = new_dataset
            .tracks
            .iter()
            .map(|(id, pts)| (format!("new_{}", id), pts.clone()))
            .collect();

        let mut all_tracks = dataset.tracks.clone();
        all_tracks.extend(new_tracks.clone());
        let mut all_sport_types = dataset.sport_types.clone();
        for (id, _) in &new_tracks {
            all_sport_types.insert(id.clone(), "Ride".to_string());
        }

        // Time incremental
        reset_peak_memory();
        let start_incr = Instant::now();
        let incr_result = detect_sections_incremental(
            &new_tracks,
            &full_result.sections,
            &all_tracks,
            &all_sport_types,
            &groups,
            &config,
            progress.clone(),
        );
        let incr_ms = start_incr.elapsed().as_millis();

        // Time full re-detection for comparison
        let all_groups: Vec<_> = all_tracks
            .iter()
            .enumerate()
            .map(|(i, (id, _))| tracematch::RouteGroup {
                group_id: format!("g_{}", i),
                representative_id: id.clone(),
                activity_ids: vec![id.clone()],
                sport_type: "Ride".to_string(),
                bounds: None,
                custom_name: None,
                best_time: None,
                avg_time: None,
                best_pace: None,
                best_activity_id: None,
            })
            .collect();

        reset_peak_memory();
        let start_full = Instant::now();
        let _ = detect_sections_multiscale(
            &all_tracks,
            &all_sport_types,
            &all_groups,
            &config,
        );
        let full_ms = start_full.elapsed().as_millis();

        let speedup = full_ms as f64 / incr_ms.max(1) as f64;

        println!(
            "| {:>8} | {:>3} | {:>8} | {:>8}ms | {:>8}ms | {:>5.1}x | {:>3}/{:<3} |",
            existing_count,
            new_count,
            full_result.sections.len(),
            incr_ms,
            full_ms,
            speedup,
            incr_result.matched_activity_ids.len(),
            new_count,
        );
    }

    // ---- Realistic chaos scenario ----
    println!("\n## 9. Realistic Chaos (~12% corridor overlap)\n");
    println!(
        "| Activities | Corridor | Sections | Overlaps | Time       | Peak Alloc |"
    );
    println!(
        "|------------|----------|----------|----------|------------|------------|"
    );

    for count in [100, 200, 500] {
        let scenario = SyntheticScenario {
            origin: GpsPoint::new(47.37, 8.55),
            activity_count: count,
            corridors: vec![CorridorConfig {
                length_meters: 5_000.0,
                overlap_fraction: 0.12,
                pattern: CorridorPattern::Winding,
                approach_length: 500.0,
            }],
            gps_noise_sigma_meters: 5.0,
            seed: count as u64 * 31,
        };

        let dataset = scenario.generate();
        let corridor_users = dataset.expected_sections[0].activity_ids.len();
        let r = run_scenario("chaos", &scenario);

        println!(
            "| {:>10} | {:>5}/{:<3} | {:>8} | {:>8} | {:>8}ms | {:>7.1} MB |",
            r.activities,
            corridor_users,
            count,
            r.sections_found,
            r.overlaps_found,
            r.time_ms,
            r.peak_memory_mb,
        );
    }

    println!("\n_Generated with `cargo run --example scaling_report --features synthetic --release`_");
}

fn format_number(n: usize) -> String {
    if n >= 1_000_000 {
        format!("{:.1}M", n as f64 / 1_000_000.0)
    } else if n >= 1_000 {
        format!("{:.1}k", n as f64 / 1_000.0)
    } else {
        format!("{}", n)
    }
}
