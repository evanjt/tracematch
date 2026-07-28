//! Ingestion-lifecycle scenarios for the unified detector.
//!
//! The app never hands the engine one tidy batch. It syncs a window at
//! install, backfills history behind that window, then adds one activity
//! at a time for years. These tests run those orders over the same
//! activity set and hold each of them to the batch answer at every step,
//! on both the quantity (section count) and the shape (endpoints and
//! length of each section).
//!
//! The adversarial case is [`scenario_late_branch_recuts_the_trunk`]: a
//! junction that does not exist until half the library has arrived. An
//! incremental that carried its previous sections forward would keep the
//! uncut through-line and drift from the batch permanently, so this is
//! the scenario that separates a converging design from an accumulating
//! one.

mod shapes;

use std::collections::HashMap;
use tracematch::geo_utils::haversine_distance;
use tracematch::{
    FrequentSection, GpsPoint, SectionConfig, SectionEvidenceCache, detect_sections_unified,
    detect_sections_unified_incremental_cached,
};

/// Endpoints must agree this closely for two sections to count as the
/// same cut. One coverage cell is ~30 m, so 25 m is sub-cell: a genuine
/// re-cut moves an endpoint by hundreds of metres and cannot hide here.
const ENDPOINT_TOL_M: f64 = 25.0;

fn polyline_length(pts: &[GpsPoint]) -> f64 {
    pts.windows(2)
        .map(|w| haversine_distance(&w[0], &w[1]))
        .sum()
}

/// Distance between two sections' endpoint pairs, taking the better of
/// the two orientations (a section carries a direction, the ground does
/// not).
fn endpoint_distance(a: &FrequentSection, b: &FrequentSection) -> f64 {
    let (a0, a1) = (a.polyline.first(), a.polyline.last());
    let (b0, b1) = (b.polyline.first(), b.polyline.last());
    let (Some(a0), Some(a1), Some(b0), Some(b1)) = (a0, a1, b0, b1) else {
        return f64::INFINITY;
    };
    let forward = haversine_distance(a0, b0).max(haversine_distance(a1, b1));
    let reversed = haversine_distance(a0, b1).max(haversine_distance(a1, b0));
    forward.min(reversed)
}

/// Every section in `left` has a partner in `right` with the same
/// endpoints and length, and the two catalogues are the same size.
/// Stricter than the ground-overlap metric the convergence gates use:
/// this fails on a re-cut that still shares most of its ground.
fn assert_same_catalogue(left: &[FrequentSection], right: &[FrequentSection], ctx: &str) {
    assert_eq!(
        left.len(),
        right.len(),
        "{ctx}: section COUNT differs (drip {} vs batch {})",
        left.len(),
        right.len()
    );
    let mut used = vec![false; right.len()];
    for a in left {
        let best = right
            .iter()
            .enumerate()
            .filter(|(j, _)| !used[*j])
            .map(|(j, b)| (j, endpoint_distance(a, b)))
            .min_by(|x, y| x.1.total_cmp(&y.1));
        let Some((j, dist)) = best else {
            panic!("{ctx}: no batch partner left for section {}", a.id);
        };
        assert!(
            dist <= ENDPOINT_TOL_M,
            "{ctx}: section SHAPE differs — {} endpoints are {dist:.0} m from the nearest \
             unmatched batch section (tolerance {ENDPOINT_TOL_M} m)",
            a.id
        );
        let (la, lb) = (
            polyline_length(&a.polyline),
            polyline_length(&right[j].polyline),
        );
        assert!(
            (la - lb).abs() <= 0.05 * la.max(lb).max(1.0),
            "{ctx}: section LENGTH differs — {} is {la:.0} m, batch partner {lb:.0} m",
            a.id
        );
        used[j] = true;
    }
}

/// Ingest `tracks` in the given arrival order, handing the engine
/// `chunks` of new activities at a time, and hold the cached incremental
/// to a from-scratch batch over the same prefix at EVERY step. Returns
/// the per-step section count so a scenario can assert how the catalogue
/// moved, not just that it agreed.
fn replay(tracks: &[(String, Vec<GpsPoint>)], chunks: &[usize], label: &str) -> Vec<usize> {
    let cfg = SectionConfig::default();
    let sports: HashMap<String, String> = shapes::pooled(tracks);
    let mut cache = SectionEvidenceCache::new();
    let mut pool: Vec<(String, Vec<GpsPoint>)> = Vec::with_capacity(tracks.len());
    let mut catalogue: Vec<FrequentSection> = Vec::new();
    let mut counts = Vec::new();
    let mut next = 0usize;

    for (step, take) in chunks.iter().enumerate() {
        let arriving: Vec<String> = tracks[next..next + take]
            .iter()
            .map(|(id, pts)| {
                pool.push((id.clone(), pts.clone()));
                id.clone()
            })
            .collect();
        next += take;
        let new_ids: Vec<&str> = arriving.iter().map(|s| s.as_str()).collect();

        let result = detect_sections_unified_incremental_cached(
            &mut cache,
            &catalogue,
            &pool,
            &new_ids,
            &[],
            &sports,
            &cfg,
        );
        catalogue = result.catalogue;

        let batch = detect_sections_unified(&pool, &[], &sports, &cfg);
        assert_same_catalogue(
            &catalogue,
            &batch,
            &format!("[{label}] step {step} (N={})", pool.len()),
        );
        counts.push(catalogue.len());
    }
    assert_eq!(
        next,
        tracks.len(),
        "[{label}] replay must consume the corpus"
    );
    counts
}

/// THE adversarial scenario. Six outings run a junction straight through,
/// then six more peel east at its midpoint. The batch over all twelve
/// sees a fork and cuts the trunk there; a drip does not see the junction
/// exist until activity seven. If incrementality were an accumulation,
/// the through-line detected in the first half would survive and the drip
/// would end up with different shapes and a different count from the
/// batch — the exact drift this design is claimed to avoid.
#[test]
fn scenario_late_branch_recuts_the_trunk() {
    let tracks = shapes::late_fork(6, 6);
    let counts = replay(&tracks, &[1; 12], "late-fork drip");

    println!("\n=========== late-emerging junction: catalogue per activity ===========");
    println!("(activities 1-6 run straight through; 7-12 peel east at the midpoint)");
    for (i, n) in counts.iter().enumerate() {
        let phase = if i < 6 { "straight" } else { "branch  " };
        println!("  after activity {:>2} ({phase})  sections = {n}", i + 1);
    }

    let before = counts[5];
    let after = *counts.last().expect("twelve steps");
    assert!(before > 0, "the straight through-line must be found first");
    assert!(
        after > before,
        "the late branch must change the catalogue: {before} sections before it arrived, \
         {after} after — if these are equal the scenario is not exercising a re-cut"
    );
    println!(
        "-----------------------------------------------------------------------\n\
         READ: the catalogue went {before} -> {after} sections when the junction\n\
         appeared, and matched a from-scratch batch on count, endpoints and\n\
         length at every one of the 12 steps.\n"
    );
}

/// The app's real pattern: a sync window lands as one bulk call, then
/// activities arrive one at a time. The bulk half and the drip half must
/// land on the same answer a single batch would give.
#[test]
fn scenario_install_window_then_daily_drip() {
    let tracks = shapes::late_fork(6, 6);
    let mut chunks = vec![6];
    chunks.extend(std::iter::repeat_n(1, 6));
    replay(&tracks, &chunks, "install-then-drip");
}

/// Backfill: history arrives AFTER the recent window, so the engine sees
/// the branch outings first and the straight ones second. Reverse
/// arrival must reach the same catalogue as forward arrival.
#[test]
fn scenario_backfill_arrives_after_the_recent_window() {
    let mut tracks = shapes::late_fork(6, 6);
    tracks.reverse();
    replay(&tracks, &[6, 1, 1, 1, 1, 1, 1], "backfill");
}

/// Bulk is the benchmark, not a different algorithm: the whole library in
/// one call must equal the same library dripped one at a time, on the
/// corpus whose answer changes mid-life.
#[test]
fn scenario_one_shot_bulk_equals_full_drip() {
    let tracks = shapes::late_fork(6, 6);
    let bulk = replay(&tracks, &[12], "one-shot");
    let drip = replay(&tracks, &[1; 12], "full-drip");
    assert_eq!(
        bulk.last().copied(),
        drip.last().copied(),
        "one-shot bulk and full drip must agree on the final catalogue size"
    );
}
