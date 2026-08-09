//! Efficiency and correctness tests for incremental section detection.
//!
//! `detect_sections_multiscale` is the live density-grid path (see
//! `veloqrs/src/persistence/sections/detection.rs` and
//! `veloqrs/src/objects/detection.rs`), so these tests must exercise it for
//! real rather than measure a detector that returned nothing.
//!
//! Run with: `cargo test --test optimization_efficiency --features synthetic -- --nocapture`

use std::sync::Arc;
use std::time::Instant;

use tracematch::sections::NoopProgress;
use tracematch::synthetic::{CorridorConfig, CorridorPattern, SyntheticDataset, SyntheticScenario};
use tracematch::{
    GpsPoint, RouteGroup, SectionConfig, detect_sections_incremental, detect_sections_multiscale,
};

/// Route groups the density grid can actually seed from.
///
/// `SyntheticDataset::route_groups` emits one singleton group per activity, and
/// the density grid only takes a representative from groups with >= 2 members,
/// so a singleton list leaves it with zero representatives and it returns no
/// sections at all. Pair up the activities the generator placed on the shared
/// corridor instead, which is what upstream grouping produces in production.
fn corridor_route_groups(dataset: &SyntheticDataset) -> Vec<RouteGroup> {
    dataset.expected_sections[0]
        .activity_ids
        .chunks(2)
        .enumerate()
        .map(|(i, ids)| RouteGroup {
            group_id: format!("corridor_{}", i),
            representative_id: ids[0].clone(),
            activity_ids: ids.to_vec(),
            sport_type: "Ride".to_string(),
            bounds: None,
            custom_name: None,
            best_time: None,
            avg_time: None,
            best_pace: None,
            best_activity_id: None,
        })
        .collect()
}

/// Extra activities laid over the same corridor as `with_activity_count`.
fn later_corridor_tracks(count: usize, seed: u64) -> Vec<(String, Vec<GpsPoint>)> {
    let scenario = SyntheticScenario {
        origin: GpsPoint::new(47.37, 8.55),
        activity_count: count,
        corridors: vec![CorridorConfig {
            length_meters: 10_000.0,
            overlap_fraction: 1.0,
            pattern: CorridorPattern::Winding,
            approach_length: 500.0,
        }],
        gps_noise_sigma_meters: 3.0,
        seed,
    };
    scenario
        .generate()
        .tracks
        .iter()
        .map(|(id, pts)| (format!("new_{}", id), pts.clone()))
        .collect()
}

#[test]
fn full_detection_finds_the_generated_corridor() {
    // Anchor for every other test in this file: if this fails, the ones below
    // are comparing empty result sets and prove nothing.
    let dataset = SyntheticScenario::with_activity_count(30, 10_000.0, 0.8).generate();
    let config = SectionConfig::default();
    let groups = corridor_route_groups(&dataset);

    let result =
        detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);

    assert!(
        !result.sections.is_empty(),
        "multiscale detection found no sections in a 30-activity corridor dataset"
    );
    assert!(
        result.stats.overlaps_found > 0,
        "detection reported {} sections but 0 overlaps",
        result.sections.len()
    );

    let corridor_users: std::collections::HashSet<&String> =
        dataset.expected_sections[0].activity_ids.iter().collect();
    let best = result
        .sections
        .iter()
        .max_by_key(|s| s.activity_ids.len())
        .unwrap();
    assert!(
        best.activity_ids
            .iter()
            .all(|id| corridor_users.contains(id)),
        "section {} pulled in activities that never used the corridor",
        best.id
    );
}

#[test]
fn incremental_matches_activities_to_existing_sections() {
    let dataset = SyntheticScenario::with_activity_count(30, 10_000.0, 0.8).generate();
    let config = SectionConfig::default();
    let groups = corridor_route_groups(&dataset);
    let progress = Arc::new(NoopProgress) as Arc<dyn tracematch::DetectionProgressCallback>;

    let full_result =
        detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);
    let existing_sections = &full_result.sections;
    assert!(
        !existing_sections.is_empty(),
        "fixture must produce sections for the incremental pass to mean anything"
    );

    let new_tracks = later_corridor_tracks(3, 77777);
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

    for matched_id in &incr_result.matched_activity_ids {
        let found_in_section = incr_result
            .updated_sections
            .iter()
            .any(|s| s.activity_ids.contains(matched_id));
        assert!(
            found_in_section,
            "matched activity {} should appear in an updated section",
            matched_id
        );
    }

    // Matching is not just bookkeeping: the section detail screen reads
    // portions, so a matched activity must gain one and lift the visit count.
    for matched_id in &incr_result.matched_activity_ids {
        let section = incr_result
            .updated_sections
            .iter()
            .find(|s| s.activity_ids.contains(matched_id))
            .unwrap();
        assert!(
            section
                .activity_portions
                .iter()
                .any(|p| &p.activity_id == matched_id && p.distance_meters > 0.0),
            "activity {} joined section {} with no portion covering any distance",
            matched_id,
            section.id
        );

        let before = existing_sections
            .iter()
            .find(|es| es.id == section.id)
            .expect("updated section must come from an existing one");
        assert!(
            section.visit_count > before.visit_count,
            "section {} gained activity {} without raising its visit count ({})",
            section.id,
            matched_id,
            before.visit_count
        );
    }

    let total_processed =
        incr_result.matched_activity_ids.len() + incr_result.unmatched_activity_ids.len();
    assert_eq!(
        total_processed,
        new_tracks.len(),
        "all new activities should be classified as matched or unmatched"
    );

    // Existing sections are never dropped by an incremental pass.
    for existing in existing_sections {
        assert!(
            incr_result
                .updated_sections
                .iter()
                .any(|s| s.id == existing.id),
            "incremental detection dropped existing section {}",
            existing.id
        );
    }
}

#[test]
fn incremental_with_no_existing_sections_leaves_everything_unmatched() {
    let dataset = SyntheticScenario::with_activity_count(5, 5_000.0, 0.8).generate();
    let progress = Arc::new(NoopProgress) as Arc<dyn tracematch::DetectionProgressCallback>;
    let config = SectionConfig::default();
    let groups = corridor_route_groups(&dataset);

    let result = detect_sections_incremental(
        &dataset.tracks,
        &[],
        &dataset.tracks,
        &dataset.sport_types,
        &groups,
        &config,
        progress,
    );

    assert_eq!(result.matched_activity_ids.len(), 0);
    assert_eq!(result.unmatched_activity_ids.len(), dataset.tracks.len());
    assert!(result.updated_sections.is_empty());
}

#[test]
fn incremental_with_no_new_activities_preserves_existing_sections() {
    let dataset = SyntheticScenario::with_activity_count(30, 10_000.0, 0.8).generate();
    let progress = Arc::new(NoopProgress) as Arc<dyn tracematch::DetectionProgressCallback>;
    let config = SectionConfig::default();
    let groups = corridor_route_groups(&dataset);

    let full_result =
        detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);
    assert!(
        !full_result.sections.is_empty(),
        "a no-op pass over zero sections proves nothing"
    );

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

    assert_eq!(result.updated_sections.len(), full_result.sections.len());
    assert_eq!(result.matched_activity_ids.len(), 0);
    assert_eq!(result.unmatched_activity_ids.len(), 0);
    assert!(result.new_sections.is_empty());

    // A no-op must not rewrite geometry either.
    for (before, after) in full_result
        .sections
        .iter()
        .zip(result.updated_sections.iter())
    {
        assert_eq!(before.id, after.id, "section order changed");
        assert_eq!(
            before.polyline.len(),
            after.polyline.len(),
            "section {} polyline changed on a no-op pass",
            before.id
        );
        assert_eq!(
            before.visit_count, after.visit_count,
            "section {} visit count changed on a no-op pass",
            before.id
        );
    }
}

#[test]
fn incremental_compares_against_far_fewer_candidates_than_full_detection() {
    // What matters on mobile is the candidate count, not wall clock: full
    // detection is all-pairs over activities, incremental is new activities
    // against known sections.
    println!("=== Incremental Operation Count ===");
    println!(
        "{:>10} {:>8} {:>10} {:>12} {:>12} {:>10}",
        "Existing", "New", "Sections", "Full pairs", "Incr. ops", "Reduction"
    );

    let config = SectionConfig::default();

    for (existing_count, new_count) in [(30, 5), (50, 5)] {
        let dataset =
            SyntheticScenario::with_activity_count(existing_count, 10_000.0, 0.7).generate();
        let groups = corridor_route_groups(&dataset);

        let full_result =
            detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);

        let sections_count = full_result.sections.len();
        assert!(
            sections_count > 0,
            "no sections at {} activities, so the operation count below is meaningless",
            existing_count
        );

        let total = existing_count + new_count;
        let full_pairs = total * (total - 1) / 2;
        let incr_ops = new_count * sections_count + new_count * (new_count - 1) / 2;
        let reduction = (1.0 - incr_ops as f64 / full_pairs as f64) * 100.0;

        println!(
            "{:>10} {:>8} {:>10} {:>12} {:>12} {:>9.1}%",
            existing_count, new_count, sections_count, full_pairs, incr_ops, reduction
        );

        assert!(
            incr_ops * 2 < full_pairs,
            "incremental should compare against less than half the pair count at {} activities ({} vs {})",
            existing_count,
            incr_ops,
            full_pairs
        );
    }
}

#[test]
fn corridor_is_found_among_mostly_unrelated_activities() {
    // 200 activities where only ~12% share a corridor: can the density grid
    // still pull the commute out of the noise?
    let scenario = SyntheticScenario {
        origin: GpsPoint::new(47.37, 8.55),
        activity_count: 200,
        corridors: vec![CorridorConfig {
            length_meters: 5_000.0,
            overlap_fraction: 0.12,
            pattern: CorridorPattern::Winding,
            approach_length: 500.0,
        }],
        gps_noise_sigma_meters: 5.0,
        seed: 12345,
    };

    let dataset = scenario.generate();
    let config = SectionConfig::default();
    let groups = corridor_route_groups(&dataset);

    let corridor_users = dataset.expected_sections[0].activity_ids.len();
    assert!(
        corridor_users >= 3,
        "fixture should place at least 3 activities on the corridor, got {}",
        corridor_users
    );

    let start = Instant::now();
    let result =
        detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);
    let detect_time = start.elapsed();

    println!("=== 200 Activities, ~12% Corridor ===");
    println!("  Corridor users:  {} of 200", corridor_users);
    println!("  Sections found:  {}", result.sections.len());
    println!("  Overlaps found:  {}", result.stats.overlaps_found);
    println!("  Detection time:  {:?}", detect_time);

    assert!(
        !result.sections.is_empty(),
        "{} activities share a 5 km corridor but detection found nothing",
        corridor_users
    );

    // The corridor is 5 km, so the section that represents it must be a
    // substantial stretch of it rather than a stray fragment.
    let longest = result
        .sections
        .iter()
        .map(|s| s.distance_meters)
        .fold(0.0_f64, f64::max);
    assert!(
        longest > 500.0,
        "longest section is only {:.0} m of a 5 km corridor",
        longest
    );
}

#[test]
fn incremental_and_full_redetection_agree_on_the_new_activities() {
    let dataset = SyntheticScenario::with_activity_count(50, 10_000.0, 0.7).generate();
    let config = SectionConfig::default();
    let groups = corridor_route_groups(&dataset);
    let progress = Arc::new(NoopProgress) as Arc<dyn tracematch::DetectionProgressCallback>;

    let full_result =
        detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config);
    assert!(
        !full_result.sections.is_empty(),
        "timing an empty detection tells us nothing"
    );

    let new_tracks = later_corridor_tracks(5, 88888);
    let mut all_tracks = dataset.tracks.clone();
    all_tracks.extend(new_tracks.clone());
    let mut all_sport_types = dataset.sport_types.clone();
    for (id, _) in &new_tracks {
        all_sport_types.insert(id.clone(), "Ride".to_string());
    }

    let mut all_groups = groups.clone();
    all_groups.push(RouteGroup {
        group_id: "corridor_new".to_string(),
        representative_id: new_tracks[0].0.clone(),
        activity_ids: new_tracks.iter().map(|(id, _)| id.clone()).collect(),
        sport_type: "Ride".to_string(),
        bounds: None,
        custom_name: None,
        best_time: None,
        avg_time: None,
        best_pace: None,
        best_activity_id: None,
    });

    let start_incr = Instant::now();
    let incr = detect_sections_incremental(
        &new_tracks,
        &full_result.sections,
        &all_tracks,
        &all_sport_types,
        &groups,
        &config,
        progress,
    );
    let incr_time = start_incr.elapsed();

    let start_full = Instant::now();
    let redo = detect_sections_multiscale(&all_tracks, &all_sport_types, &all_groups, &config);
    let redo_time = start_full.elapsed();

    println!("=== Timing: Incremental vs Full (50+5 activities) ===");
    println!("  Incremental:         {:?}", incr_time);
    println!("  Full re-detect (55): {:?}", redo_time);
    println!("  Incremental matched: {}", incr.matched_activity_ids.len());
    println!("  Full re-detect saw:  {} sections", redo.sections.len());

    assert!(
        !redo.sections.is_empty(),
        "full re-detection over 55 activities found nothing to compare against"
    );

    // The two paths must not disagree about where the new activities went:
    // anything incremental matched has to sit in a section of the full run too,
    // or the cheap path is inventing membership the expensive one denies.
    for matched_id in &incr.matched_activity_ids {
        assert!(
            redo.sections
                .iter()
                .any(|s| s.activity_ids.contains(matched_id)),
            "incremental matched {} but full re-detection put it in no section",
            matched_id
        );
    }

    // Wall-clock comparison only means anything on a quiet host, so gate it.
    // At 50+5 the two paths are roughly level here, so this is a report, not a
    // guarantee: the win only shows up once the existing corpus dwarfs the batch.
    if std::env::var("VELOQRS_TIMING_TESTS").is_ok() {
        assert!(
            incr_time < redo_time,
            "incremental ({:?}) should beat full re-detection ({:?})",
            incr_time,
            redo_time
        );
    }
}
