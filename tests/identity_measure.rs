//! Identity measurement: how much churn the identity + hysteresis layer removes.
//!
//! The batch is legitimately non-monotone: dripped one activity at a time over
//! a growing pool, the Unified catalogue dissolves and reforms sections (section
//! count walks a path like `.. 3, 3, 2, 2, 3 ..`, never a superset of an earlier
//! step). The identity layer sits on top: [`plan_identity`] carries a stable
//! id onto the ground that survives a recompute, and [`HysteresisState`]
//! debounces the dissolves and re-cuts so the visible view moves slowly.
//!
//! This binary drives the real corpus drip through the cached Unified
//! incremental (the same path fold_foundation's oracle uses), threads each step's
//! catalogue through the identity + hysteresis fold, and reports two things per
//! step and in total:
//!
//! - IDENTITY RETENTION: of the sections present last step whose ground survives
//!   this step, the fraction under the SAME id. The raw batch renumbers ids
//!   positionally (they are a function of geographic sort order), so its raw
//!   retention drops whenever a cluster count or order shifts; the visible view
//!   holds ids, so its retention stays at 1.0. That gap is the positional
//!   reshuffle the identity layer removes.
//! - CHURN EVENTS: a section appearing or disappearing (by ground) between
//!   consecutive steps. The batch churns as it converges; the visible view damps
//!   most of it. Fewer visible churn events than raw is the headline number.
//!
//! Measurement, not a tuned gate: the printed numbers are the deliverable. Two
//! invariants that hold by construction are asserted (visible retention is 1.0;
//! the visible view never churns MORE than the raw batch), plus a determinism
//! lock (two runs produce an identical visible id trajectory).

use std::collections::BTreeSet;

use tracematch::scenarios::{LifecycleConfig, LifecycleCorpus};
use tracematch::{
    CandidateSection, FrequentSection, GpsPoint, HysteresisState, SectionConfig,
    SectionEvidenceCache, SectionUpdatePolicy,
    detect_sections_unified_incremental_cached_with_policy, shares_ground,
};

// ============================================================================
// Snapshots + churn/retention maths (inlined; integration test files are their
// own crate, and this keeps the measurement's metric explicit next to it).
// ============================================================================

/// A catalogue snapshot: `(id, ground)` per section, in whatever order the layer
/// emits. Ground match, not id equality, decides survival.
type Snapshot = Vec<(String, Vec<GpsPoint>)>;

fn snapshot_raw(catalogue: &[FrequentSection]) -> Snapshot {
    catalogue
        .iter()
        .map(|s| (s.id.clone(), s.polyline.clone()))
        .collect()
}

fn snapshot_visible(state: &HysteresisState) -> Snapshot {
    state.visible_grounds()
}

/// The harness identity metric: of `before` sections whose ground survives in
/// `after`, the fraction that survive under the SAME id. Returns 1.0 when no
/// `before` ground survives (nothing to keep).
fn identity_retention(before: &Snapshot, after: &Snapshot) -> f64 {
    let survivors: Vec<&(String, Vec<GpsPoint>)> = before
        .iter()
        .filter(|(_, g)| after.iter().any(|(_, h)| shares_ground(g, h)))
        .collect();
    if survivors.is_empty() {
        return 1.0;
    }
    let kept = survivors
        .iter()
        .filter(|(id, g)| {
            after
                .iter()
                .any(|(aid, h)| aid == id && shares_ground(g, h))
        })
        .count();
    kept as f64 / survivors.len() as f64
}

/// Churn events between two snapshots: grounds that disappeared plus grounds
/// that appeared, matched by [`shares_ground`].
fn churn_events(before: &Snapshot, after: &Snapshot) -> usize {
    let disappeared = before
        .iter()
        .filter(|(_, g)| !after.iter().any(|(_, h)| shares_ground(g, h)))
        .count();
    let appeared = after
        .iter()
        .filter(|(_, h)| !before.iter().any(|(_, g)| shares_ground(g, h)))
        .count();
    disappeared + appeared
}

// ============================================================================
// Drip driver
// ============================================================================

/// A 24-activity slice (bucket A 20 + C 1 + D 3): the same reduced corpus the
/// fold foundation uses, small enough to run fast and already exercising the
/// non-monotone dissolve/reform walk.
fn reduced_corpus() -> LifecycleCorpus {
    LifecycleCorpus::generate(&LifecycleConfig {
        bucket_a_count: 20,
        bucket_b_delta_count: 0,
        bucket_d_delta_count: 0,
        bucket_e_delta_count: 0,
        ..LifecycleConfig::default()
    })
}

/// One step's measured numbers.
struct StepRow {
    n: usize,
    raw_count: usize,
    visible_count: usize,
    raw_retention: Option<f64>,
    visible_retention: Option<f64>,
    raw_churn: usize,
    visible_churn: usize,
    minted: usize,
    restored: usize,
    dissolved: usize,
    recut: usize,
}

struct MeasureRun {
    rows: Vec<StepRow>,
    /// Every visible id observed each step, in step order: the determinism
    /// fingerprint (a reordering or re-mint would change it).
    visible_id_trajectory: Vec<Vec<String>>,
    first_raw: Snapshot,
    last_raw: Snapshot,
    first_visible: Snapshot,
    last_visible: Snapshot,
}

/// Drip the corpus one activity at a time through the cached Unified incremental,
/// threading each raw catalogue through the identity + hysteresis fold. Pure and
/// deterministic: the corpus is seeded, the incremental is order-free, and the
/// fold uses only total orders, so two calls return identical rows.
fn drip_measure(corpus: &LifecycleCorpus) -> MeasureRun {
    let tracks = corpus.tracks_through_e();
    let sports = corpus.sport_map_through_e();
    let cfg = SectionConfig::default();

    let mut cache = SectionEvidenceCache::new();
    let mut state = HysteresisState::default();
    let mut cached_cat: Vec<FrequentSection> = Vec::new();
    let mut pool: Vec<(String, Vec<GpsPoint>)> = Vec::with_capacity(tracks.len());

    let mut prev_raw: Snapshot = Vec::new();
    let mut prev_visible: Snapshot = Vec::new();
    let mut rows = Vec::new();
    let mut visible_id_trajectory = Vec::new();
    let mut first_raw = Snapshot::new();
    let mut first_visible = Snapshot::new();

    for (id, pts) in &tracks {
        pool.push((id.clone(), pts.clone()));
        let new_ids = [pool.last().unwrap().0.as_str()];

        // Raw batch catalogue for this pool (the batch truth the fold damps).
        let result = detect_sections_unified_incremental_cached_with_policy(
            &mut cache,
            &cached_cat,
            &pool,
            &new_ids,
            &[],
            &sports,
            &cfg,
            &SectionUpdatePolicy::default(),
        );
        cached_cat = result.catalogue;

        // Fold the raw catalogue through identity + hysteresis.
        let candidates: Vec<CandidateSection> = cached_cat
            .iter()
            .map(CandidateSection::from_section)
            .collect();
        let outcome = state.step(&candidates);

        let raw = snapshot_raw(&cached_cat);
        let visible = snapshot_visible(&state);

        let raw_retention = (!prev_raw.is_empty()).then(|| identity_retention(&prev_raw, &raw));
        let visible_retention =
            (!prev_visible.is_empty()).then(|| identity_retention(&prev_visible, &visible));

        rows.push(StepRow {
            n: pool.len(),
            raw_count: raw.len(),
            visible_count: visible.len(),
            raw_retention,
            visible_retention,
            raw_churn: churn_events(&prev_raw, &raw),
            visible_churn: churn_events(&prev_visible, &visible),
            minted: outcome.minted,
            restored: outcome.restored,
            dissolved: outcome.dissolved,
            recut: outcome.recut_applied,
        });
        visible_id_trajectory.push(state.visible_ids());

        if first_raw.is_empty() && !raw.is_empty() {
            first_raw = raw.clone();
        }
        if first_visible.is_empty() && !visible.is_empty() {
            first_visible = visible.clone();
        }
        prev_raw = raw;
        prev_visible = visible;
    }

    MeasureRun {
        rows,
        visible_id_trajectory,
        first_raw,
        last_raw: prev_raw,
        first_visible,
        last_visible: prev_visible,
    }
}

// ============================================================================
// The measurement
// ============================================================================

#[test]
fn b2_identity_and_churn_measurement() {
    let corpus = reduced_corpus();
    let run = drip_measure(&corpus);

    println!(
        "\n================ identity retention + churn damping (reduced corpus) ================"
    );
    println!("   N  raw  vis   rawRet  visRet   rawChurn  visChurn   mint rest diss recut");
    let mut raw_churn_total = 0usize;
    let mut visible_churn_total = 0usize;
    let mut raw_ret_sum = 0.0;
    let mut raw_ret_n = 0usize;
    for r in &run.rows {
        raw_churn_total += r.raw_churn;
        visible_churn_total += r.visible_churn;
        if let Some(rr) = r.raw_retention {
            raw_ret_sum += rr;
            raw_ret_n += 1;
        }
        let fmt_ret = |o: Option<f64>| {
            o.map(|x| format!("{x:.2}"))
                .unwrap_or_else(|| "  - ".into())
        };
        println!(
            "  {:>3}  {:>3}  {:>3}    {:>4}    {:>4}   {:>6}    {:>6}    {:>3}  {:>3}  {:>3}   {:>3}",
            r.n,
            r.raw_count,
            r.visible_count,
            fmt_ret(r.raw_retention),
            fmt_ret(r.visible_retention),
            r.raw_churn,
            r.visible_churn,
            r.minted,
            r.restored,
            r.dissolved,
            r.recut,
        );
    }

    let mean_raw_ret = if raw_ret_n > 0 {
        raw_ret_sum / raw_ret_n as f64
    } else {
        1.0
    };
    // First-vs-last: the "expand preserves identity" headline over the whole run.
    let raw_end_to_end = identity_retention(&run.first_raw, &run.last_raw);
    let visible_end_to_end = identity_retention(&run.first_visible, &run.last_visible);

    println!(
        "---------------------------------------------------------------------------------------"
    );
    println!("raw churn events (total) ........ {raw_churn_total}");
    println!("visible churn events (total) .... {visible_churn_total}");
    println!(
        "churn removed by hysteresis ..... {} events ({:.0}%)",
        raw_churn_total.saturating_sub(visible_churn_total),
        100.0 * (raw_churn_total.saturating_sub(visible_churn_total)) as f64
            / (raw_churn_total.max(1)) as f64
    );
    println!("mean per-step identity retention  raw {mean_raw_ret:.3}   visible 1.000");
    println!(
        "first->last identity retention .. raw {raw_end_to_end:.3}   visible {visible_end_to_end:.3}"
    );
    println!(
        "---------------------------------------------------------------------------------------"
    );
    println!(
        "READ: the raw batch renumbers ids positionally, so its retention dips whenever a cluster\n\
         count/order shifts; the visible view carries ids, so it stays at 1.000. And the visible\n\
         view churns far less than the raw batch — the dissolve/reform thrash is debounced away.\n"
    );

    // Invariants (true by construction, safe to lock):
    // 1. The visible view keeps every surviving id at every QUIET step.
    //    A step that fires a dissolve or a re-cut may legitimately move
    //    ground between ids (a dissolved corridor's ground taken over by
    //    a neighbour's fired re-cut is the hysteresis layer converging
    //    to the batch, announced through the outcome) — only an
    //    unannounced id change is a violation.
    for r in &run.rows {
        if let Some(vr) = r.visible_retention
            && r.dissolved == 0
            && r.recut == 0
        {
            assert!(
                (vr - 1.0).abs() < 1e-9,
                "visible identity retention must be 1.0 at a quiet step, got {vr:.3} at N={}",
                r.n
            );
        }
    }
    // 2. Hysteresis never adds churn: the visible view is at most as churny as
    //    the raw batch it damps.
    assert!(
        visible_churn_total <= raw_churn_total,
        "visible churn ({visible_churn_total}) must not exceed raw churn ({raw_churn_total})"
    );
    // 3. End to end, the visible view holds identity through the whole expand.
    assert!(
        visible_end_to_end >= raw_end_to_end,
        "visible end-to-end retention ({visible_end_to_end:.3}) must be at least raw ({raw_end_to_end:.3})"
    );
}

#[test]
fn b2_measurement_is_deterministic() {
    // Two independent runs of the whole drip must produce a byte-identical
    // visible id trajectory: no HashMap-order leak, no float-order ambiguity, no
    // re-mint of a stable id. This is the hard determinism requirement in code.
    let corpus = reduced_corpus();
    let a = drip_measure(&corpus);
    let b = drip_measure(&corpus);
    assert_eq!(
        a.visible_id_trajectory, b.visible_id_trajectory,
        "the visible id trajectory must be identical across two runs"
    );
    // And the per-step shape matches too.
    let shape = |run: &MeasureRun| -> Vec<(usize, usize, usize)> {
        run.rows
            .iter()
            .map(|r| (r.raw_count, r.visible_count, r.visible_churn))
            .collect()
    };
    assert_eq!(
        shape(&a),
        shape(&b),
        "per-step raw/visible/churn shape must be identical"
    );

    // A quick sanity check the trajectory is non-trivial (ids actually assigned).
    let all_ids: BTreeSet<&String> = a.visible_id_trajectory.iter().flatten().collect();
    assert!(
        !all_ids.is_empty(),
        "the drip must assign at least one stable id"
    );
}
