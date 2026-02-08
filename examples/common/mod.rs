//! Shared infrastructure for scaling benchmarks.
//!
//! Provides the memory-tracking allocator, scenario runners, and individual
//! benchmark functions that can be composed into the full scaling report or
//! run independently.

// Each example binary only uses a subset of this module's items.
#![allow(dead_code)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

use tracematch::synthetic::{CorridorConfig, CorridorPattern, SyntheticScenario};
use tracematch::{
    GpsPoint, NoopProgress, SectionConfig, compute_grid_cells, detect_sections_incremental,
    detect_sections_multiscale, detect_sections_optimized, grid_filtered_pairs,
};

// ============================================================================
// Peak Memory Tracking Allocator
// ============================================================================

pub struct TrackingAllocator;

pub static CURRENT_BYTES: AtomicUsize = AtomicUsize::new(0);
pub static PEAK_BYTES: AtomicUsize = AtomicUsize::new(0);

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

pub fn reset_peak_memory() {
    CURRENT_BYTES.store(0, Ordering::Relaxed);
    PEAK_BYTES.store(0, Ordering::Relaxed);
}

pub fn get_peak_memory_mb() -> f64 {
    PEAK_BYTES.load(Ordering::Relaxed) as f64 / (1024.0 * 1024.0)
}

// ============================================================================
// Scenario Runner
// ============================================================================

pub struct ScenarioResult {
    pub name: String,
    pub activities: usize,
    pub pairs: usize,
    pub total_points: usize,
    pub sections_found: usize,
    pub overlaps_found: u32,
    pub time_ms: u128,
    pub estimated_memory_mb: f64,
    pub peak_memory_mb: f64,
}

pub fn run_scenario(name: &str, scenario: &SyntheticScenario) -> ScenarioResult {
    let dataset = scenario.generate();
    let groups = dataset.route_groups();
    let config = SectionConfig::default();
    let estimated_memory_mb = dataset.metadata.estimated_memory_bytes as f64 / (1024.0 * 1024.0);

    reset_peak_memory();
    let start = Instant::now();
    let result =
        detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);
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

pub fn run_scenario_optimized(name: &str, scenario: &SyntheticScenario) -> ScenarioResult {
    let dataset = scenario.generate();
    let config = SectionConfig::default();
    let estimated_memory_mb = dataset.metadata.estimated_memory_bytes as f64 / (1024.0 * 1024.0);

    reset_peak_memory();
    let start = Instant::now();
    let result = detect_sections_optimized(&dataset.tracks, &dataset.sport_types, &config);
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

pub fn format_number(n: usize) -> String {
    if n >= 1_000_000 {
        format!("{:.1}M", n as f64 / 1_000_000.0)
    } else if n >= 1_000 {
        format!("{:.1}k", n as f64 / 1_000.0)
    } else {
        format!("{}", n)
    }
}

// ============================================================================
// Individual Benchmarks
// ============================================================================

pub fn bench_scaling_multiscale() {
    println!("## Scaling Curve — Multiscale (10km corridor, 80% overlap)\n");
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
}

pub fn bench_scaling_optimized() {
    println!("## Scaling Curve — Optimized (10km corridor, 80% overlap)\n");
    println!("| Activities | Points    | Sections | Time       | Peak Alloc |");
    println!("|------------|-----------|----------|------------|------------|");

    for count in [50, 100, 250, 500, 1000, 2000] {
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
}

pub fn bench_route_length() {
    println!("## Route Length Impact (50 activities, 60% overlap)\n");
    println!("| Corridor  | Points    | Overlaps | Sections | Time       | Peak Alloc |");
    println!("|-----------|-----------|----------|----------|------------|------------|");

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
}

pub fn bench_overlap_density() {
    println!("## Overlap Density (100 activities, 10km corridor)\n");
    println!("| Overlap | Overlaps | Sections | Time       | Peak Alloc |");
    println!("|---------|----------|----------|------------|------------|");

    for overlap_pct in [10, 30, 50, 70, 90] {
        let overlap_frac = overlap_pct as f64 / 100.0;
        let scenario = SyntheticScenario::with_activity_count(100, 10_000.0, overlap_frac);
        let r = run_scenario("overlap", &scenario);
        println!(
            "| {:>5}%  | {:>8} | {:>8} | {:>8}ms | {:>7.1} MB |",
            overlap_pct, r.overlaps_found, r.sections_found, r.time_ms, r.peak_memory_mb,
        );
    }
}

pub fn bench_no_overlap() {
    println!("## No Overlap Worst Case (pure overhead)\n");
    println!("| Activities | Pairs      | Time       | Peak Alloc |");
    println!("|------------|------------|------------|------------|");

    for count in [50, 100, 250, 500, 1000, 2000] {
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
}

pub fn bench_predefined_scenarios() {
    println!("## Predefined Scenarios\n");
    println!("| Scenario         | Activities | Corridors | Sections | Time       | Peak Alloc |");
    println!("|------------------|------------|-----------|----------|------------|------------|");

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
}

pub fn bench_grid_filtering() {
    println!("## Grid Filtering Impact (pair reduction)\n");
    println!("| Activities | Exhaustive | Grid-filt. | Reduction | Speedup |");
    println!("|------------|------------|------------|-----------|---------|");

    for count in [50, 100, 250, 500, 1000, 2000] {
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

    println!(
        "\n_Note: Grid filtering eliminates cross-region pairs. Single-corridor synthetic data"
    );
    println!("shows 0% reduction (all activities nearby). Real-world data with activities across");
    println!("multiple cities shows 60-90% reduction._");
}

pub fn bench_incremental() {
    println!("## Incremental Detection (add N new to existing)\n");
    println!("| Existing | New | Sections | Incr. time | Full time  | Speedup | Matched |");
    println!("|----------|-----|----------|------------|------------|---------|---------|");

    let progress = Arc::new(NoopProgress) as Arc<dyn tracematch::DetectionProgressCallback>;

    for (existing_count, new_count) in [(50, 5), (100, 10), (250, 25), (500, 50), (1000, 100)] {
        let scenario = SyntheticScenario::with_activity_count(existing_count, 10_000.0, 0.8);
        let dataset = scenario.generate();
        let config = SectionConfig::default();
        let groups = dataset.route_groups();

        // Full detection to establish existing sections
        let full_result =
            detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);

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
        let _ = detect_sections_multiscale(&all_tracks, &all_sport_types, &all_groups, &config);
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
}

pub fn bench_chaos() {
    println!("## Realistic Chaos (~12% corridor overlap)\n");
    println!("| Activities | Corridor | Sections | Overlaps | Time       | Peak Alloc |");
    println!("|------------|----------|----------|----------|------------|------------|");

    for count in [100, 200, 500, 1000, 2000] {
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
}

pub fn bench_predefined_scenarios_large() {
    println!("## Predefined Scenarios — Large Scale (optimized)\n");
    println!("| Scenario            | Activities | Corridors | Sections | Time       | Peak Alloc |");
    println!("|---------------------|------------|-----------|----------|------------|------------|");

    let scenarios: Vec<(&str, SyntheticScenario)> = vec![
        ("extreme_scale", SyntheticScenario::extreme_scale()),
        ("extreme_scale_2000", SyntheticScenario::extreme_scale_2000()),
    ];

    for (name, scenario) in &scenarios {
        let corridors = scenario.corridors.len();
        let r = run_scenario_optimized(name, scenario);
        println!(
            "| {:>19} | {:>10} | {:>9} | {:>8} | {:>8}ms | {:>7.1} MB |",
            name, r.activities, corridors, r.sections_found, r.time_ms, r.peak_memory_mb,
        );
    }
}

pub fn bench_mobile_estimates() {
    println!("## Mobile Performance Estimates (optimized detection)\n");

    struct DeviceProfile {
        name: &'static str,
        sustained_factor: f64,
    }

    let devices = [
        DeviceProfile { name: "Desktop (baseline)", sustained_factor: 1.0 },
        DeviceProfile { name: "iPhone 15 Pro (A17)", sustained_factor: 0.65 },
        DeviceProfile { name: "Flagship Android (SD 8 Elite)", sustained_factor: 0.55 },
        DeviceProfile { name: "Mid-range Android (SD 7+ Gen 2)", sustained_factor: 0.35 },
    ];

    println!(
        "| Device                          | {:>8} | {:>8} | {:>8} |",
        "500", "1000", "2000"
    );
    println!(
        "|---------------------------------|----------|----------|----------|"
    );

    // Run desktop benchmarks first
    let mut desktop_times = Vec::new();
    for count in [500, 1000, 2000] {
        let scenario = SyntheticScenario::with_activity_count(count, 10_000.0, 0.8);
        let r = run_scenario_optimized("mobile_est", &scenario);
        desktop_times.push(r.time_ms);
    }

    for device in &devices {
        let times: Vec<String> = desktop_times
            .iter()
            .map(|&t| {
                let estimated = (t as f64 / device.sustained_factor) as u128;
                if estimated >= 1000 {
                    format!("{:.1}s", estimated as f64 / 1000.0)
                } else {
                    format!("{}ms", estimated)
                }
            })
            .collect();

        println!(
            "| {:>31} | {:>8} | {:>8} | {:>8} |",
            device.name, times[0], times[1], times[2],
        );
    }

    println!("\n_Estimates use sustained performance factors (not peak burst).");
    println!("Actual mobile performance depends on thermal state, memory bandwidth, and OS scheduling._");
}
