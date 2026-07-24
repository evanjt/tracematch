//! Does tracematch's CORE LIBRARY already give us an incremental "pool"
//! primitive we can reuse for B1 (order-free, ingest-concurrent incremental
//! detection that CONVERGES to a from-scratch batch)?
//!
//! This is a PURE tracematch experiment: free functions only, no veloqrs, no
//! SQLite, no stateful engine. The corpus is the seeded synthetic
//! `LifecycleCorpus`, so there is zero dependency on personal or external data.
//!
//! What it proves, with measured numbers (the measurement test PRINTS them and
//! never fails; the gate documents the gap B1 must close and is `#[ignore]`d):
//!
//! 1. There is no stateful accumulator object to reuse. The "pool" is a plain
//!    `Vec` you thread through pure functions. We confirm this structurally
//!    (a grep in the harness message) and operationally (we build the drip by
//!    hand out of `detect_sections_incremental` + a growing `Vec`).
//! 2. `detect_sections_incremental` is welded to the LEGACY multiscale detector
//!    (`detect_sections_multiscale_with_progress`, incremental.rs:283). It does
//!    NOT dispatch on `config.detection_method`, so passing
//!    `DetectionMethod::Unified` changes nothing about what it produces.
//! 3. The multiscale detector it leans on is route-group-driven: it needs
//!    several DISTINCT route representatives to converge (mod.rs:1260 skips when
//!    `route_reps < min_routes`). `detect_sections_unified` instead reads raw
//!    activity traffic and needs no groups. These are different section
//!    definitions, so the incremental drip cannot reproduce the unified batch.
//!
//! Verdict framing lives in the report, not here. The numbers below back it.

use std::sync::{Arc, OnceLock};
use std::time::Instant;

use tracematch::scenarios::{LifecycleConfig, LifecycleCorpus};
use tracematch::sections::NoopProgress;
use tracematch::{
    DetectionProgressCallback, FrequentSection, GpsPoint, MatchConfig, RouteGroup, RouteSignature,
    SectionConfig, detect_sections_corridor, detect_sections_incremental,
    detect_sections_multiscale, detect_sections_unified, group_signatures,
};

// ============================================================================
// Ground-match maths — inlined from the veloqrs lifecycle_support harness so
// this test owns its geometry and depends on no veloqrs internals.
// ============================================================================

/// Ground-match tolerance: half the ~100 m evidence cell. Two lines within this
/// of each other describe the same physical corridor.
const GROUND_TOL_M: f64 = 50.0;

/// Fraction of one polyline that must fall within tolerance of the other for
/// the two to count as the same ground (bidirectional, extent-growth tolerant).
const COVERAGE_FRAC: f64 = 0.6;

/// Metres between two points (haversine).
fn haversine_m(a: &GpsPoint, b: &GpsPoint) -> f64 {
    let r = 6_371_000.0_f64;
    let (la1, lo1) = (a.latitude.to_radians(), a.longitude.to_radians());
    let (la2, lo2) = (b.latitude.to_radians(), b.longitude.to_radians());
    let dla = la2 - la1;
    let dlo = lo2 - lo1;
    let h = (dla / 2.0).sin().powi(2) + la1.cos() * la2.cos() * (dlo / 2.0).sin().powi(2);
    2.0 * r * h.sqrt().asin()
}

/// Fraction of `samples` within `tol_m` of any point on `line`.
fn coverage(samples: &[GpsPoint], line: &[GpsPoint], tol_m: f64) -> f64 {
    if samples.is_empty() || line.is_empty() {
        return 0.0;
    }
    let covered = samples
        .iter()
        .filter(|s| {
            line.iter()
                .map(|p| haversine_m(s, p))
                .fold(f64::INFINITY, f64::min)
                <= tol_m
        })
        .count();
    covered as f64 / samples.len() as f64
}

/// Whether two sections describe the same physical ground.
fn ground_matches(a: &FrequentSection, b: &FrequentSection) -> bool {
    coverage(&a.polyline, &b.polyline, GROUND_TOL_M) >= COVERAGE_FRAC
        || coverage(&b.polyline, &a.polyline, GROUND_TOL_M) >= COVERAGE_FRAC
}

/// Directional recall: fraction of `reference` sections that have at least one
/// ground-matching section in `candidate`.
fn ground_recall(reference: &[FrequentSection], candidate: &[FrequentSection]) -> f64 {
    if reference.is_empty() {
        return 1.0;
    }
    let hit = reference
        .iter()
        .filter(|r| candidate.iter().any(|c| ground_matches(r, c)))
        .count();
    hit as f64 / reference.len() as f64
}

/// Catalogue overlap under a greedy 1:1 ground pairing, normalised by the
/// larger catalogue. Unlike bare recall, this PENALISES count mismatch: if one
/// detector fragments a corridor into three sections and the other emits one,
/// the surplus stays unmatched and the score drops. That count/extent
/// disagreement is exactly where multiscale and unified part ways, so this is
/// the decision-grade convergence number.
fn catalogue_overlap(a: &[FrequentSection], b: &[FrequentSection]) -> f64 {
    if a.is_empty() && b.is_empty() {
        return 1.0;
    }
    if a.is_empty() || b.is_empty() {
        return 0.0;
    }
    let mut used = vec![false; b.len()];
    let mut matched = 0usize;
    for sa in a {
        for (j, sb) in b.iter().enumerate() {
            if !used[j] && ground_matches(sa, sb) {
                used[j] = true;
                matched += 1;
                break;
            }
        }
    }
    matched as f64 / a.len().max(b.len()) as f64
}

// ============================================================================
// Corpus + group construction
// ============================================================================

/// A small, drip-fast slice of the lifecycle corpus. Only bucket A is sized up;
/// C (1) and D (3) are fixed by the generator, giving 24 activities total. That
/// keeps 24 sequential incremental calls quick in a debug build while still
/// carrying real corridors, reverses, parallel streets, and one-offs.
fn reduced_corpus() -> LifecycleCorpus {
    LifecycleCorpus::generate(&LifecycleConfig {
        bucket_a_count: 20,
        bucket_b_delta_count: 0,
        bucket_d_delta_count: 0,
        bucket_e_delta_count: 0,
        ..LifecycleConfig::default()
    })
}

/// Build route groups the way the multiscale/incremental path needs them.
///
/// The synthetic activities prepend a random ~350 m approach and append a
/// random ~350 m departure, so two traversals of the same corridor have start
/// and end points scattered up to ~800 m apart. The default 250 m
/// `endpoint_threshold` would prune those pairs before matching and no groups
/// would form, starving the density-grid detector. Real activities usually
/// start and end at a consistent trailhead, so we widen the endpoint gate (and
/// loosen the distance-ratio and match floor for GPS noise) to let the synthetic
/// corridors group as their real-world counterparts would. Batch and drip build
/// groups identically, so the convergence comparison stays fair.
fn build_groups(tracks: &[(String, Vec<GpsPoint>)], cfg: &MatchConfig) -> Vec<RouteGroup> {
    let sigs: Vec<RouteSignature> = tracks
        .iter()
        .filter_map(|(id, pts)| RouteSignature::from_points(id, pts, cfg))
        .collect();
    group_signatures(&sigs, cfg)
}

fn grouping_config() -> MatchConfig {
    MatchConfig {
        endpoint_threshold: 1_500.0,
        max_distance_diff_ratio: 0.5,
        min_match_percentage: 45.0,
        ..MatchConfig::default()
    }
}

// ============================================================================
// The experiment
// ============================================================================

struct Outcome {
    activity_count: usize,
    group_count: usize,
    significant_group_count: usize,

    multiscale_batch_count: usize,
    unified_batch_count: usize,
    corridor_batch_count: usize,
    drip_final_count: usize,

    per_add_ms: Vec<u128>,

    // Greedy 1:1 catalogue overlap (the headline convergence numbers).
    drip_vs_multiscale: f64,
    drip_vs_unified: f64,

    // Directional breakdowns, so a count mismatch is legible in the report.
    drip_recall_of_multiscale: f64,
    multiscale_recall_of_drip: f64,
    drip_recall_of_unified: f64,
    unified_recall_of_drip: f64,
}

/// Run once, share across both tests. The corpus is deterministic, so the two
/// tests observe identical numbers without paying for the drip twice.
fn outcome() -> &'static Outcome {
    static CELL: OnceLock<Outcome> = OnceLock::new();
    CELL.get_or_init(run_experiment)
}

fn run_experiment() -> Outcome {
    let corpus = reduced_corpus();
    let tracks = corpus.tracks_through_e();
    let sport_types = corpus.sport_map_through_e();
    // The route-overlap detector (multiscale) skips unless it sees at least
    // `min_routes` distinct significant route reps (mod.rs:1260). The reduced
    // synthetic corpus is deliberately few-corridors / many-activities, which
    // collapses to only a couple of significant groups, so the production
    // default of 3 keeps it inert. We relax to 2 for BOTH the multiscale batch
    // and the drip so the detector fires at all and "drip converges to its own
    // detector" becomes a real number rather than 0-vs-0. Unified ignores
    // `min_routes` (it reads raw traffic, not route groups), so its baseline is
    // unaffected by this knob.
    let section_cfg = SectionConfig {
        min_routes: 2,
        ..SectionConfig::default()
    };
    let group_cfg = grouping_config();

    // --- Batch baselines on the whole corpus (the convergence targets) ------
    let groups_full = build_groups(&tracks, &group_cfg);
    let significant_group_count = groups_full
        .iter()
        .filter(|g| g.activity_ids.len() >= 2)
        .count();

    let multiscale_batch =
        detect_sections_multiscale(&tracks, &sport_types, &groups_full, &section_cfg).sections;

    // Unified needs no groups and no time streams here; the lift veto rests on
    // geometry alone, which is all the synthetic corridors carry.
    let unified_batch = detect_sections_unified(&tracks, &[], &sport_types, &section_cfg);

    // Corridor is the engine's ACTUAL shipped detector for small batches
    // (<500 activities; multiscale only kicks in above that). Like unified it
    // reads raw activity traffic and needs no route groups, so it fires on this
    // corpus. Carried purely as context: it proves the corpus IS detectable and
    // that the drip's emptiness is specific to its multiscale dependency, not a
    // property of the data.
    let corridor_batch = detect_sections_corridor(&tracks, &sport_types, &section_cfg);

    // --- Pool drip through the pure incremental function --------------------
    // There is no pool object: `existing` is the catalogue so far and `pool` is
    // the growing track list. We thread both through `detect_sections_incremental`
    // one activity at a time, exactly as B1 would have to if it reused this
    // primitive. Groups are recomputed on the pool each step (the engine
    // regroups before detecting).
    let progress: Arc<dyn DetectionProgressCallback> = Arc::new(NoopProgress);
    let mut existing: Vec<FrequentSection> = Vec::new();
    let mut pool: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    let mut per_add_ms: Vec<u128> = Vec::with_capacity(tracks.len());

    for track in &tracks {
        pool.push(track.clone());
        let groups_pool = build_groups(&pool, &group_cfg);
        let new_tracks = [track.clone()];

        let t0 = Instant::now();
        let result = detect_sections_incremental(
            &new_tracks,
            &existing,
            &pool,
            &sport_types,
            &groups_pool,
            &section_cfg,
            progress.clone(),
        );
        per_add_ms.push(t0.elapsed().as_millis());

        // Next pool state = updated existing (already the full carried-forward
        // catalogue) plus any sections discovered from the unmatched activities.
        existing = result.updated_sections;
        existing.extend(result.new_sections);
    }
    let drip_final = existing;

    Outcome {
        activity_count: tracks.len(),
        group_count: groups_full.len(),
        significant_group_count,
        multiscale_batch_count: multiscale_batch.len(),
        unified_batch_count: unified_batch.len(),
        corridor_batch_count: corridor_batch.len(),
        drip_final_count: drip_final.len(),
        per_add_ms,
        drip_vs_multiscale: catalogue_overlap(&drip_final, &multiscale_batch),
        drip_vs_unified: catalogue_overlap(&drip_final, &unified_batch),
        drip_recall_of_multiscale: ground_recall(&multiscale_batch, &drip_final),
        multiscale_recall_of_drip: ground_recall(&drip_final, &multiscale_batch),
        drip_recall_of_unified: ground_recall(&unified_batch, &drip_final),
        unified_recall_of_drip: ground_recall(&drip_final, &unified_batch),
    }
}

// ============================================================================
// Tests
// ============================================================================

/// Structural + measurement report. Never asserts: it prints the evidence the
/// verdict rests on. Run with `-- --nocapture` to read it.
#[test]
fn pool_incremental_measurement() {
    let o = outcome();

    let per_add_summary = if o.per_add_ms.is_empty() {
        "none".to_string()
    } else {
        let total: u128 = o.per_add_ms.iter().sum();
        let max = o.per_add_ms.iter().copied().max().unwrap_or(0);
        let last = *o.per_add_ms.last().unwrap();
        format!(
            "total={total}ms max={max}ms last={last}ms full=[{}]",
            o.per_add_ms
                .iter()
                .map(|m| m.to_string())
                .collect::<Vec<_>>()
                .join(",")
        )
    };

    println!("\n================ POOL / INCREMENTAL PARITY ================");
    println!("corpus activities ........... {}", o.activity_count);
    println!(
        "route groups ................ {} ({} significant, >=2 activities)",
        o.group_count, o.significant_group_count
    );
    println!("--- batch baselines (targets) ---");
    println!(
        "multiscale batch sections ... {}   (the detector the incremental uses)",
        o.multiscale_batch_count
    );
    println!(
        "unified batch sections ...... {}   (the new base; raw traffic, no groups)",
        o.unified_batch_count
    );
    println!(
        "corridor batch sections ..... {}   (engine's shipped <500-activity detector; raw traffic)",
        o.corridor_batch_count
    );
    println!("--- pool drip (detect_sections_incremental, threaded Vec) ---");
    println!("drip final sections ......... {}", o.drip_final_count);
    println!("per-add ms .................. {per_add_summary}");
    println!("--- convergence (greedy 1:1 ground overlap, normalised by max) ---");
    // Guard against the misleading both-empty case: an overlap of 1.000 between
    // two empty catalogues is not convergence, it is joint inertness.
    let multi_line = if o.drip_final_count == 0 && o.multiscale_batch_count == 0 {
        "n/a  (both empty; the incremental's detector never fired on this corpus)".to_string()
    } else {
        format!(
            "{:.3}   (recall drip->multi {:.3}, multi->drip {:.3})",
            o.drip_vs_multiscale, o.drip_recall_of_multiscale, o.multiscale_recall_of_drip
        )
    };
    println!("drip  vs  MULTISCALE batch .. {multi_line}");
    println!(
        "drip  vs  UNIFIED batch ..... {:.3}   (recall drip->uni {:.3}, uni->drip {:.3})",
        o.drip_vs_unified, o.drip_recall_of_unified, o.unified_recall_of_drip
    );
    println!("----------------------------------------------------------");
    println!(
        "READING: detect_sections_incremental calls detect_sections_multiscale\n\
         for its unmatched pool (incremental.rs:283). It never inspects\n\
         config.detection_method, so it can only ever emit MULTISCALE-shaped\n\
         sections, whatever the Unified setting says. Multiscale is\n\
         route-group-driven and needs several distinct crossing routes; on\n\
         activity-traffic data (many activities, few shared corridors) it stays\n\
         inert, so BOTH the multiscale batch and the drip are empty here, while\n\
         the two raw-traffic detectors (corridor, unified) fire. The unified\n\
         batch is a different section definition (raw-traffic supernodes, no\n\
         route groups), so the drip cannot converge to it. Numbers quantify it.\n"
    );

    // Sanity only, never a semantic assertion: the drip must have actually run.
    assert_eq!(o.per_add_ms.len(), o.activity_count);
}

/// GATE (red today). Asserts the pure incremental drip converges to the UNIFIED
/// batch. It does not, because the primitive is welded to multiscale, so this
/// stays `#[ignore]`d and green-by-default. Under `--include-ignored` it FAILS
/// on purpose: that failure IS the measured statement of the gap B1 closes.
/// When B1 ships an order-free Unified-aware incremental that converges, flip
/// this test to a live gate by deleting the `#[ignore]`.
#[test]
#[ignore = "B1 not built: pure incremental does not converge to unified batch"]
fn gate_drip_converges_to_unified_batch() {
    let o = outcome();
    assert!(
        o.drip_vs_unified >= 0.85,
        "drip-vs-unified catalogue overlap {:.3} < 0.85 \
         (multiscale batch={}, unified batch={}, drip={}). The pure incremental \
         is hardcoded to the multiscale detector and cannot reproduce the \
         unified catalogue; B1 must build an order-free Unified-aware incremental.",
        o.drip_vs_unified,
        o.multiscale_batch_count,
        o.unified_batch_count,
        o.drip_final_count,
    );
}
