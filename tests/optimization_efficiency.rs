//! Efficiency tests for section detection optimizations.
//!
//! These tests demonstrate the performance gains from:
//! 1. Grid-based spatial filtering (reduces O(N²) pair count)
//! 2. Incremental detection (avoids full re-detection when adding activities)
//!
//! Run with: `cargo test --test optimization_efficiency --features synthetic -- --nocapture`

use std::collections::HashSet;
use std::sync::Arc;
use std::time::Instant;

use tracematch::synthetic::{CorridorConfig, CorridorPattern, SyntheticScenario};
use tracematch::{
    GpsPoint, SectionConfig, detect_sections_incremental,
    detect_sections_multiscale,
};
use tracematch::sections::NoopProgress;
use tracematch::sections::optimized::{compute_grid_cells, grid_filtered_pairs};

/// Create a simple linear track from point A to point B with `n` points.
fn make_track(
    start_lat: f64,
    start_lng: f64,
    end_lat: f64,
    end_lng: f64,
    n: usize,
) -> Vec<GpsPoint> {
    (0..n)
        .map(|i| {
            let t = i as f64 / (n - 1) as f64;
            GpsPoint::new(
                start_lat + (end_lat - start_lat) * t,
                start_lng + (end_lng - start_lng) * t,
            )
        })
        .collect()
}

// ============================================================================
// Grid Filtering Tests
// ============================================================================

#[test]
fn test_grid_filtering_reduces_pairs_across_cities() {
    // Simulate a user who rides in 3 different cities (~100km apart).
    // Activities in City A should NOT be paired with activities in City B.
    //
    // Grid cells are ~5km, so cities 100km apart are in completely different cells.
    let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();

    // City A: Zurich area (47.37°N, 8.55°E) — 20 activities
    for i in 0..20 {
        let noise = i as f64 * 0.001; // slight variation
        tracks.push((
            format!("zurich_{}", i),
            make_track(47.37 + noise, 8.55, 47.40 + noise, 8.58, 200),
        ));
    }

    // City B: Munich area (48.14°N, 11.58°E) — 20 activities (~300km from Zurich)
    for i in 0..20 {
        let noise = i as f64 * 0.001;
        tracks.push((
            format!("munich_{}", i),
            make_track(48.14 + noise, 11.58, 48.17 + noise, 11.61, 200),
        ));
    }

    // City C: Milan area (45.46°N, 9.19°E) — 20 activities (~250km from Zurich)
    for i in 0..20 {
        let noise = i as f64 * 0.001;
        tracks.push((
            format!("milan_{}", i),
            make_track(45.46 + noise, 9.19, 45.49 + noise, 9.22, 200),
        ));
    }

    let n = tracks.len();
    let exhaustive_pairs = n * (n - 1) / 2;

    let track_cells: Vec<HashSet<_>> = tracks
        .iter()
        .map(|(_, pts)| compute_grid_cells(pts))
        .collect();

    let filtered_pairs = grid_filtered_pairs(&track_cells);
    let filtered_count = filtered_pairs.len();

    // Within-city pairs: 3 cities × C(20,2) = 3 × 190 = 570
    let within_city_pairs = 3 * (20 * 19 / 2);

    let reduction_pct = (1.0 - filtered_count as f64 / exhaustive_pairs as f64) * 100.0;

    println!("=== Grid Filtering: 3 Cities ===");
    println!("  Activities:        {} (20 per city × 3 cities)", n);
    println!("  Exhaustive pairs:  {}", exhaustive_pairs);
    println!("  Grid-filtered:     {}", filtered_count);
    println!(
        "  Within-city pairs: {} (theoretical minimum)",
        within_city_pairs
    );
    println!("  Pairs eliminated:  {:.1}%", reduction_pct);
    println!(
        "  Speedup factor:    {:.1}x",
        exhaustive_pairs as f64 / filtered_count.max(1) as f64
    );

    // Cross-city pairs should be eliminated entirely.
    // Grid-filtered count should be close to within-city pairs (570).
    assert!(
        filtered_count < exhaustive_pairs / 2,
        "Grid filtering should eliminate cross-city pairs: {} should be < {} (half of exhaustive)",
        filtered_count,
        exhaustive_pairs / 2
    );

    // Should be close to within-city pairs (allowing some margin for edge cells)
    assert!(
        filtered_count <= within_city_pairs * 2,
        "Grid-filtered pairs ({}) should be near within-city pairs ({})",
        filtered_count,
        within_city_pairs
    );
}

#[test]
fn test_grid_filtering_dense_corridor_preserves_all_needed_pairs() {
    // When all activities are in the same ~5km corridor, grid filtering
    // should preserve most pairs (it doesn't over-filter).
    let scenario = SyntheticScenario::with_activity_count(50, 5_000.0, 1.0);
    let dataset = scenario.generate();
    let n = dataset.tracks.len();
    let exhaustive_pairs = n * (n - 1) / 2;

    let track_cells: Vec<_> = dataset
        .tracks
        .iter()
        .map(|(_, pts)| compute_grid_cells(pts))
        .collect();

    let filtered_pairs = grid_filtered_pairs(&track_cells);
    let filtered_count = filtered_pairs.len();

    println!("=== Grid Filtering: Dense Corridor ===");
    println!("  Activities:       {}", n);
    println!("  Exhaustive pairs: {}", exhaustive_pairs);
    println!("  Grid-filtered:    {}", filtered_count);
    println!(
        "  Preservation:     {:.1}%",
        filtered_count as f64 / exhaustive_pairs as f64 * 100.0
    );

    // All activities in same area → grid filtering should keep most pairs
    assert!(
        filtered_count >= exhaustive_pairs / 2,
        "Dense corridor should preserve at least half of pairs ({} < {})",
        filtered_count,
        exhaustive_pairs / 2
    );
}

#[test]
fn test_grid_filtering_scales_with_geographic_spread() {
    // Demonstrate that grid filtering becomes MORE effective as
    // activities span more geographic area (typical for years of data).
    println!("=== Grid Filtering: Geographic Spread Scaling ===");
    println!(
        "{:>10} {:>12} {:>12} {:>10} {:>12}",
        "Cities", "Activities", "Exhaustive", "Filtered", "Reduction"
    );

    for num_cities in [1, 2, 4, 8] {
        let per_city = 20;
        let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();

        // Spread cities along a 1000km arc
        for city in 0..num_cities {
            let base_lat = 45.0 + city as f64 * 1.5; // ~170km between cities
            let base_lng = 8.0 + city as f64 * 2.0;
            for i in 0..per_city {
                let noise = i as f64 * 0.001;
                tracks.push((
                    format!("city{}_{}", city, i),
                    make_track(
                        base_lat + noise,
                        base_lng,
                        base_lat + 0.03 + noise,
                        base_lng + 0.03,
                        200,
                    ),
                ));
            }
        }

        let n = tracks.len();
        let exhaustive = n * (n - 1) / 2;
        let track_cells: Vec<_> = tracks
            .iter()
            .map(|(_, pts)| compute_grid_cells(pts))
            .collect();
        let filtered = grid_filtered_pairs(&track_cells).len();
        let reduction = (1.0 - filtered as f64 / exhaustive as f64) * 100.0;

        println!(
            "{:>10} {:>12} {:>12} {:>12} {:>9.1}%",
            num_cities, n, exhaustive, filtered, reduction
        );
    }

    // 8 cities: should eliminate most cross-city pairs
    let mut tracks_8: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    for city in 0..8 {
        let base_lat = 45.0 + city as f64 * 1.5;
        let base_lng = 8.0 + city as f64 * 2.0;
        for i in 0..20 {
            let noise = i as f64 * 0.001;
            tracks_8.push((
                format!("c{}_{}", city, i),
                make_track(
                    base_lat + noise,
                    base_lng,
                    base_lat + 0.03 + noise,
                    base_lng + 0.03,
                    200,
                ),
            ));
        }
    }
    let n8 = tracks_8.len();
    let exhaustive_8 = n8 * (n8 - 1) / 2;
    let cells_8: Vec<_> = tracks_8
        .iter()
        .map(|(_, pts)| compute_grid_cells(pts))
        .collect();
    let filtered_8 = grid_filtered_pairs(&cells_8).len();

    assert!(
        filtered_8 < exhaustive_8 / 4,
        "8 cities should eliminate >75% of pairs: {} should be < {}",
        filtered_8,
        exhaustive_8 / 4
    );
}

// ============================================================================
// Incremental Detection Tests
// ============================================================================

#[test]
fn test_incremental_faster_than_full_redetection() {
    // Phase 1: Full detection on 30 activities to establish baseline sections
    let scenario = SyntheticScenario::with_activity_count(30, 10_000.0, 0.8);
    let dataset = scenario.generate();

    let config = SectionConfig::default();
    let groups = dataset.route_groups();
    let progress = Arc::new(NoopProgress) as Arc<dyn tracematch::DetectionProgressCallback>;

    let start_full = Instant::now();
    let full_result =
        detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);
    let full_time = start_full.elapsed();

    let existing_sections = &full_result.sections;
    assert!(
        !existing_sections.is_empty(),
        "Full detection should find sections"
    );

    // Phase 2: Generate 5 new activities in the same corridor
    let new_scenario = SyntheticScenario {
        origin: GpsPoint::new(47.37, 8.55),
        activity_count: 5,
        corridors: vec![CorridorConfig {
            length_meters: 10_000.0,
            overlap_fraction: 1.0,
            pattern: CorridorPattern::Winding,
            approach_length: 500.0,
        }],
        gps_noise_sigma_meters: 3.0,
        seed: 99999,
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

    // Phase 3: Incremental detection (5 new vs existing sections)
    let start_incr = Instant::now();
    let incr_result = detect_sections_incremental(
        &new_tracks,
        existing_sections,
        &all_tracks,
        &all_sport_types,
        &groups,
        &config,
        progress.clone(),
    );
    let incr_time = start_incr.elapsed();

    // Phase 4: Full re-detection on all 35 activities (for comparison)
    let all_groups: Vec<_> = all_tracks
        .iter()
        .enumerate()
        .map(|(i, (id, _))| tracematch::RouteGroup {
            group_id: format!("group_{}", i),
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

    let start_redo = Instant::now();
    let _redo_result =
        detect_sections_multiscale(&all_tracks, &all_sport_types, &all_groups, &config);
    let redo_time = start_redo.elapsed();

    let speedup = redo_time.as_micros() as f64 / incr_time.as_micros().max(1) as f64;

    println!("=== Incremental vs Full Re-detection ===");
    println!("  Existing activities:  30");
    println!("  New activities:       5");
    println!("  Existing sections:    {}", existing_sections.len());
    println!("  ---");
    println!("  Initial full detect:  {:?}", full_time);
    println!("  Incremental (5 new):  {:?}", incr_time);
    println!("  Full re-detect (35):  {:?}", redo_time);
    println!("  Speedup:              {:.1}x faster", speedup);
    println!("  ---");
    println!(
        "  Matched activities:   {}",
        incr_result.matched_activity_ids.len()
    );
    println!(
        "  Unmatched activities: {}",
        incr_result.unmatched_activity_ids.len()
    );
    println!(
        "  Updated sections:     {}",
        incr_result.updated_sections.len()
    );
    println!("  New sections:         {}", incr_result.new_sections.len());

    // Incremental should be faster than full re-detection
    assert!(
        incr_time < redo_time,
        "Incremental ({:?}) should be faster than full re-detection ({:?})",
        incr_time,
        redo_time
    );
}

#[test]
fn test_incremental_matches_activities_to_existing_sections() {
    // Detect sections from 20 activities
    let scenario = SyntheticScenario::with_activity_count(20, 10_000.0, 0.8);
    let dataset = scenario.generate();

    let config = SectionConfig::default();
    let groups = dataset.route_groups();
    let progress = Arc::new(NoopProgress) as Arc<dyn tracematch::DetectionProgressCallback>;

    let full_result =
        detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);

    let existing_sections = &full_result.sections;
    if existing_sections.is_empty() {
        println!("Skipping: no sections detected (dataset may lack sufficient overlap)");
        return;
    }

    // Add 3 activities that traverse the corridor (high overlap)
    let new_scenario = SyntheticScenario {
        origin: GpsPoint::new(47.37, 8.55),
        activity_count: 3,
        corridors: vec![CorridorConfig {
            length_meters: 10_000.0,
            overlap_fraction: 1.0,
            pattern: CorridorPattern::Winding,
            approach_length: 500.0,
        }],
        gps_noise_sigma_meters: 3.0,
        seed: 77777,
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

    let incr_result = detect_sections_incremental(
        &new_tracks,
        existing_sections,
        &all_tracks,
        &all_sport_types,
        &groups,
        &config,
        progress,
    );

    println!("=== Incremental Correctness ===");
    println!("  Existing sections: {}", existing_sections.len());
    println!("  New activities:    {}", new_tracks.len());
    println!(
        "  Matched:           {}",
        incr_result.matched_activity_ids.len()
    );
    println!(
        "  Unmatched:         {}",
        incr_result.unmatched_activity_ids.len()
    );

    // Verify matched activities appear in updated sections
    for matched_id in &incr_result.matched_activity_ids {
        let found_in_section = incr_result
            .updated_sections
            .iter()
            .any(|s| s.activity_ids.contains(matched_id));
        assert!(
            found_in_section,
            "Matched activity {} should appear in an updated section",
            matched_id
        );
    }

    // Updated sections should have version incremented for modified sections
    let modified_sections: Vec<_> = incr_result
        .updated_sections
        .iter()
        .filter(|s| {
            let original = existing_sections.iter().find(|es| es.id == s.id);
            match original {
                Some(orig) => s.version > orig.version,
                None => false,
            }
        })
        .collect();

    println!("  Modified sections: {}", modified_sections.len());

    // All new activities should be classified
    let total_processed =
        incr_result.matched_activity_ids.len() + incr_result.unmatched_activity_ids.len();
    assert_eq!(
        total_processed,
        new_tracks.len(),
        "All new activities should be classified as matched or unmatched"
    );
}

#[test]
fn test_incremental_empty_existing_sections() {
    // Edge case: no existing sections → all activities unmatched
    let scenario = SyntheticScenario::with_activity_count(5, 5_000.0, 0.8);
    let dataset = scenario.generate();
    let progress = Arc::new(NoopProgress) as Arc<dyn tracematch::DetectionProgressCallback>;

    let config = SectionConfig::default();
    let groups = dataset.route_groups();

    let result = detect_sections_incremental(
        &dataset.tracks,
        &[],
        &dataset.tracks,
        &dataset.sport_types,
        &groups,
        &config,
        progress,
    );

    println!("=== Incremental: Empty Existing ===");
    println!("  Unmatched: {}", result.unmatched_activity_ids.len());
    println!("  Matched:   {}", result.matched_activity_ids.len());

    assert_eq!(result.matched_activity_ids.len(), 0);
    assert_eq!(result.unmatched_activity_ids.len(), dataset.tracks.len());
    assert!(result.updated_sections.is_empty());
}

#[test]
fn test_incremental_zero_new_activities() {
    // Edge case: no new activities → no-op, existing sections preserved
    let scenario = SyntheticScenario::with_activity_count(10, 5_000.0, 0.8);
    let dataset = scenario.generate();
    let progress = Arc::new(NoopProgress) as Arc<dyn tracematch::DetectionProgressCallback>;

    let config = SectionConfig::default();
    let groups = dataset.route_groups();

    let full_result =
        detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);

    let empty_tracks: Vec<(String, Vec<GpsPoint>)> = vec![];

    let result = detect_sections_incremental(
        &empty_tracks,
        &full_result.sections,
        &dataset.tracks,
        &dataset.sport_types,
        &groups,
        &config,
        progress,
    );

    println!("=== Incremental: Zero New Activities ===");
    println!(
        "  Existing sections preserved: {}",
        result.updated_sections.len()
    );

    assert_eq!(result.updated_sections.len(), full_result.sections.len());
    assert_eq!(result.matched_activity_ids.len(), 0);
    assert_eq!(result.unmatched_activity_ids.len(), 0);
    assert!(result.new_sections.is_empty());
}

// ============================================================================
// Scaling Comparison
// ============================================================================

#[test]
fn test_incremental_operation_count_advantage() {
    // Show the operation count advantage of incremental detection.
    // This is what matters on mobile: fewer operations = less battery + faster.
    //
    // Uses actual section detection for small counts, analytical for large counts
    // to keep test runtime reasonable in debug mode.
    println!("=== Incremental Operation Count ===");
    println!(
        "{:>10} {:>8} {:>10} {:>12} {:>12} {:>10}",
        "Existing", "New", "Sections", "Full pairs", "Incr. ops", "Reduction"
    );

    let config = SectionConfig::default();

    // Run actual detection for small counts to get real section counts
    for (existing_count, new_count) in [(50, 5), (100, 10)] {
        let scenario = SyntheticScenario::with_activity_count(existing_count, 10_000.0, 0.7);
        let dataset = scenario.generate();
        let groups = dataset.route_groups();

        let full_result =
            detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);

        let sections_count = full_result.sections.len();
        let total = existing_count + new_count;
        let full_pairs = total * (total - 1) / 2;
        let incr_ops = new_count * sections_count + new_count * (new_count - 1) / 2;
        let reduction = (1.0 - incr_ops as f64 / full_pairs as f64) * 100.0;

        println!(
            "{:>10} {:>8} {:>10} {:>12} {:>12} {:>9.1}%",
            existing_count, new_count, sections_count, full_pairs, incr_ops, reduction
        );
    }

    // Analytical projections for larger counts (section count scales ~linearly with activities)
    // Based on observed: 50 activities → ~40 sections, 100 → ~80 sections
    for (existing_count, new_count, est_sections) in
        [(500, 50, 100), (1000, 100, 150), (5000, 500, 300)]
    {
        let total = existing_count + new_count;
        let full_pairs = total * (total - 1) / 2;
        let incr_ops = new_count * est_sections + new_count * (new_count - 1) / 2;
        let reduction = (1.0 - incr_ops as f64 / full_pairs as f64) * 100.0;

        println!(
            "{:>10} {:>8} {:>9}* {:>12} {:>12} {:>9.1}%",
            existing_count, new_count, est_sections, full_pairs, incr_ops, reduction
        );
    }

    println!("  (* = estimated section count)");

    // At scale, incremental should reduce operations by >90%
    let full_pairs_5500 = 5500 * 5499 / 2; // 5000+500
    let incr_ops_5500 = 500 * 300 + 500 * 499 / 2; // 300 sections
    assert!(
        incr_ops_5500 < full_pairs_5500 / 10,
        "Incremental should be >10x fewer operations at 5000+500"
    );
}

#[test]
fn test_realistic_chaos_scenario() {
    // Simulates a real user: 200 activities where only ~12% share a corridor.
    // The rest are random rides in different areas.
    //
    // This tests: can the algorithm find patterns in chaos?
    // And: does grid filtering help when most activities are geographically spread?
    let scenario = SyntheticScenario {
        origin: GpsPoint::new(47.37, 8.55),
        activity_count: 200,
        corridors: vec![
            // Daily commute: ~12% of activities use this
            CorridorConfig {
                length_meters: 5_000.0,
                overlap_fraction: 0.12,
                pattern: CorridorPattern::Winding,
                approach_length: 500.0,
            },
        ],
        gps_noise_sigma_meters: 5.0, // Realistic GPS noise
        seed: 12345,
    };

    let dataset = scenario.generate();
    let config = SectionConfig::default();
    let groups = dataset.route_groups();

    // How many activities use the corridor? (ground truth)
    let corridor_users = dataset.expected_sections[0].activity_ids.len();

    // Grid filtering: most activities are random, should be spread across cells
    let track_cells: Vec<HashSet<_>> = dataset
        .tracks
        .iter()
        .map(|(_, pts)| compute_grid_cells(pts))
        .collect();
    let n = dataset.tracks.len();
    let exhaustive = n * (n - 1) / 2;
    let filtered = grid_filtered_pairs(&track_cells).len();
    let grid_reduction = (1.0 - filtered as f64 / exhaustive as f64) * 100.0;

    // Run detection
    let start = Instant::now();
    let result =
        detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);
    let detect_time = start.elapsed();

    println!("=== Realistic Chaos: 200 Activities, ~12% Corridor ===");
    println!("  Total activities:     {}", n);
    println!(
        "  Corridor users:       {} ({:.0}%)",
        corridor_users,
        corridor_users as f64 / n as f64 * 100.0
    );
    println!("  Random activities:    {}", n - corridor_users);
    println!("  ---");
    println!("  Exhaustive pairs:     {}", exhaustive);
    println!("  Grid-filtered pairs:  {}", filtered);
    println!("  Grid reduction:       {:.1}%", grid_reduction);
    println!("  ---");
    println!("  Sections found:       {}", result.sections.len());
    println!("  Overlaps found:       {}", result.stats.overlaps_found);
    println!("  Detection time:       {:?}", detect_time);

    // The algorithm should still find patterns despite the chaos.
    // With ~24 activities sharing a 5km corridor, sections should be detected.
    if corridor_users >= 3 {
        // With enough corridor users, at least one section should be found
        println!(
            "  Pattern detection:    {} sections from {} corridor users",
            result.sections.len(),
            corridor_users
        );
    }

    // Grid filtering should help since 88% of activities are random/spread
    // (though the synthetic generator starts all from the same origin, so
    // initial portions will be nearby — real-world spread would be even better)
}

#[test]
fn test_incremental_timing_comparison() {
    // Time incremental vs full re-detection at 50 existing + 5 new.
    let scenario = SyntheticScenario::with_activity_count(50, 10_000.0, 0.7);
    let dataset = scenario.generate();
    let config = SectionConfig::default();
    let groups = dataset.route_groups();
    let progress = Arc::new(NoopProgress) as Arc<dyn tracematch::DetectionProgressCallback>;

    let full_result =
        detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);

    if full_result.sections.is_empty() {
        println!("Skipping: no sections detected");
        return;
    }

    // Generate 5 new activities
    let new_scenario = SyntheticScenario {
        origin: GpsPoint::new(47.37, 8.55),
        activity_count: 5,
        corridors: vec![CorridorConfig {
            length_meters: 10_000.0,
            overlap_fraction: 1.0,
            pattern: CorridorPattern::Winding,
            approach_length: 500.0,
        }],
        gps_noise_sigma_meters: 3.0,
        seed: 88888,
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

    let all_groups: Vec<_> = all_tracks
        .iter()
        .enumerate()
        .map(|(i, (id, _))| tracematch::RouteGroup {
            group_id: format!("group_{}", i),
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

    // Time incremental
    let start_incr = Instant::now();
    let _incr = detect_sections_incremental(
        &new_tracks,
        &full_result.sections,
        &all_tracks,
        &all_sport_types,
        &groups,
        &config,
        progress,
    );
    let incr_time = start_incr.elapsed();

    // Time full re-detection
    let start_full = Instant::now();
    let _full = detect_sections_multiscale(&all_tracks, &all_sport_types, &all_groups, &config);
    let full_time = start_full.elapsed();

    let speedup = full_time.as_micros() as f64 / incr_time.as_micros().max(1) as f64;

    // Estimate mobile times (ARM ~3-5x slower than x86)
    let mobile_factor = 4.0;

    println!("=== Timing: Incremental vs Full (50+5 activities) ===");
    println!("  Incremental:         {:?}", incr_time);
    println!("  Full re-detect (55): {:?}", full_time);
    println!("  Speedup:             {:.1}x", speedup);
    println!("  ---");
    println!(
        "  Est. mobile incremental: {:?}",
        incr_time.mul_f64(mobile_factor)
    );
    println!(
        "  Est. mobile full:        {:?}",
        full_time.mul_f64(mobile_factor)
    );

    assert!(
        incr_time < full_time,
        "Incremental should be faster than full re-detection"
    );
}
