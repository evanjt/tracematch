//! Split and merge inheritance stress scenarios.
//!
//! The reduced growth corpus that identity_measure drives barely exercises
//! GENUINE inheritance tangles: it is mostly monotone growth, so carry / mint /
//! dissolve dominate and real splits and merges are rare. This file builds the
//! tangles directly on hand-drawn synthetic ground and drives each one through
//! BOTH `plan_identity` (the instantaneous decision) AND `HysteresisState` (the
//! k-step visible behaviour), so a decision rule's dynamic consequences show up,
//! not just its one-shot output.
//!
//! Four scenarios:
//! (a) a long held section splits into two as a branch gains support,
//! (b) two long held neighbours fuse into one corridor,
//! (c) the marginal-capture tangle: a candidate edged to a MARGINAL senior and
//!     a DOMINANT junior. The agreement tier keeps an extent-agreeing match's
//!     id at every floor; the hysteresis run and the opt-in
//!     `merge_mutual_floor` cover the shapes without an exact match,
//! (d) a 3-prior/3-candidate braid, asserting the plan is byte-identical across
//!     runs and stable under input permutation.
//!
//! Pure tracematch, deterministic, synthetic only. No `synthetic` feature: every
//! polyline is hand-built, so run with `--test inheritance_stress` (which does
//! not compile the corpus-backed suites).

use tracematch::{
    CandidateSection, Decision, GpsPoint, HysteresisParams, HysteresisState, IdentityParams,
    PriorSection, RetireReason, Retirement, mutual_overlap, plan_identity, plan_identity_tuned,
};

// ============================================================================
// Ground builders
// ============================================================================

/// `n` points from `(lat0, lng0)` stepping by `(dlat, dlng)` each point. A step
/// of 1e-4 deg latitude is ~11 m, well inside the 50 m match tolerance, so a
/// prefix/suffix of one line stays "the same corridor" as the whole.
fn seg(lat0: f64, lng0: f64, dlat: f64, dlng: f64, n: usize) -> Vec<GpsPoint> {
    (0..n)
        .map(|i| GpsPoint::new(lat0 + dlat * i as f64, lng0 + dlng * i as f64))
        .collect()
}

/// A straight north-heading line of `n` ~11 m-spaced points.
fn north(lat0: f64, lng0: f64, n: usize) -> Vec<GpsPoint> {
    seg(lat0, lng0, 1.0e-4, 0.0, n)
}

fn cand(polyline: Vec<GpsPoint>, visit_count: u32) -> CandidateSection {
    CandidateSection {
        polyline,
        visit_count,
    }
}

fn prior(id: &str, polyline: Vec<GpsPoint>, first_seen: u64, visit_count: u32) -> PriorSection {
    PriorSection {
        id: id.to_string(),
        polyline,
        first_seen,
        visit_count,
    }
}

// ============================================================================
// Hysteresis driver
// ============================================================================

/// Feed `batch` into `state`, returning the outcome plus the visible id set and
/// the visible `(id, ground)` pairs after the step. Printed by the scenarios so
/// the k-step trajectory is legible.
fn step(
    state: &mut HysteresisState,
    batch: &[CandidateSection],
) -> (usize, usize, usize, usize, Vec<String>) {
    let out = state.step(batch);
    (
        out.visible_count,
        out.minted,
        out.dissolved,
        out.restored,
        state.visible_ids(),
    )
}

/// The stable id a given ground currently carries in the visible view, if any
/// visible section shares ground with it. Used to check "did this ground keep
/// its id" without depending on the emit order.
fn id_on_ground(state: &HysteresisState, ground: &[GpsPoint]) -> Option<String> {
    state
        .visible_grounds()
        .into_iter()
        .find(|(_, g)| mutual_overlap(ground, g) >= 0.5)
        .map(|(id, _)| id)
}

/// How many visible sections share ground with `ground`. More than one means a
/// duplicate section on the same corridor.
fn count_on_ground(state: &HysteresisState, ground: &[GpsPoint]) -> usize {
    state
        .visible_grounds()
        .into_iter()
        .filter(|(_, g)| mutual_overlap(ground, g) >= 0.5)
        .count()
}

// ============================================================================
// (a) Fork becomes section-worthy: one held section splits into two
// ============================================================================

#[test]
fn fork_split_larger_piece_inherits_and_is_order_free() {
    // A 100-point corridor P, held as one section. A branch gains support and
    // the corridor is re-cut into a longer piece C1 (70%) and a shorter carved
    // piece C2 (30%). Both are sub-portions of P, so both share its ground; C1
    // overlaps P more, so C1 inherits and C2 mints.
    let p = north(46.0, 7.0, 100);
    let c1 = north(46.0, 7.0, 70);
    let c2 = north(46.0 + 70.0e-4, 7.0, 30);

    let held = vec![prior("s_A", p.clone(), 1, 9)];
    let plan = plan_identity(&held, &[cand(c1.clone(), 5), cand(c2.clone(), 4)]);
    assert_eq!(
        plan.decisions,
        vec![
            Decision::SplitInherit { id: "s_A".into() },
            Decision::Mint {
                split_from: Some("s_A".into())
            }
        ],
        "the larger-overlap piece must inherit, the carved piece must mint from it"
    );
    assert!(plan.retired.is_empty(), "a split retires no prior");

    // Order-free: reverse the candidate order, the same piece still inherits.
    let rev = plan_identity(&held, &[cand(c2.clone(), 4), cand(c1.clone(), 5)]);
    assert_eq!(
        rev.decisions[1],
        Decision::SplitInherit { id: "s_A".into() }
    );
    assert_eq!(
        rev.decisions[0],
        Decision::Mint {
            split_from: Some("s_A".into())
        }
    );

    // Through the hysteresis. The larger piece keeps P's id the whole way, and
    // the carved piece resolves to exactly ONE section. The fold fix (a section
    // mid re-cut competes on its target footprint, not its stale full one) stops
    // the retained piece capturing the carved candidate on its held geometry, so
    // it no longer mints phantom duplicates each detect. This holds at the
    // default floor 0.0 — the fold fix, not the merge floor, resolves it.
    let run = |floor: f64| -> (String, usize, Vec<(usize, usize)>) {
        let mut state = HysteresisState::new(HysteresisParams {
            merge_mutual_floor: floor,
            ..HysteresisParams::default()
        });
        state.step(&[cand(p.clone(), 9)]);
        let a_id = id_on_ground(&state, &p).expect("P visible");
        let mut rows = Vec::new();
        for _ in 0..5 {
            let out = state.step(&[cand(c1.clone(), 5), cand(c2.clone(), 4)]);
            rows.push((out.visible_count, out.minted));
            // The larger piece always keeps P's id, floor or not.
            assert_eq!(id_on_ground(&state, &c1).as_ref(), Some(&a_id));
        }
        (a_id, count_on_ground(&state, &c2), rows)
    };

    println!("\n================ (a) fork split ================");
    let (a_id, default_dups, default_rows) = run(0.0);
    println!(
        "default (floor 0.0): P held as {a_id}; per-step (visible, mint) = {default_rows:?}; carved-ground sections = {default_dups}"
    );
    let (_, floored_dups, floored_rows) = run(0.4);
    println!(
        "floored (0.4):       per-step (visible, mint) = {floored_rows:?}; carved-ground sections = {floored_dups}"
    );
    println!(
        "READ: the larger piece keeps its id and the carved piece is a single section at floor 0.0 - the fold fix, not the floor, resolves the phantom duplicates."
    );

    // The fold fix makes the split clean at the DEFAULT floor: exactly one
    // section on the carved ground, no phantom duplicates. The floor does not
    // change this.
    assert_eq!(
        default_dups, 1,
        "the split leaves one section on the carved ground at floor 0.0 (phantoms resolved by the fold fix)"
    );
    assert_eq!(floored_dups, 1, "and the merge floor does not change that");
}

// ============================================================================
// (b) Corridor fusion: two long held neighbours merge into one
// ============================================================================

#[test]
fn corridor_fusion_senior_survives_junior_merges() {
    // A (southern half, senior) and B (northern half, junior) are two held
    // neighbours. Bridging evidence fuses them into one corridor Z = A ++ B. The
    // senior A inherits Z; the junior B retires MergedInto A (not Dissolved), so
    // a later re-split can restore it.
    let a = north(46.0, 7.0, 50);
    let b = north(46.0 + 50.0e-4, 7.0, 50);
    let z = north(46.0, 7.0, 100);

    let held = vec![prior("s_A", a.clone(), 1, 6), prior("s_B", b.clone(), 2, 6)];
    let plan = plan_identity(&held, &[cand(z.clone(), 12)]);
    assert_eq!(
        plan.decisions,
        vec![Decision::MergeInherit { id: "s_A".into() }],
        "the fused corridor inherits the senior prior"
    );
    assert_eq!(
        plan.retired,
        vec![Retirement {
            id: "s_B".into(),
            reason: RetireReason::MergedInto { id: "s_A".into() },
        }],
        "the junior retires MergedInto the senior, retained for a later re-split"
    );

    // The floor must NOT disturb this legitimate merge: both parties cover half
    // the corridor (mutual 0.5 each), which clears a 0.4 floor, so seniority
    // still decides and A still inherits.
    let floored = plan_identity_tuned(
        &held,
        &[cand(z.clone(), 12)],
        &IdentityParams {
            merge_mutual_floor: 0.4,
        },
    );
    assert_eq!(
        floored.decisions,
        vec![Decision::MergeInherit { id: "s_A".into() }],
        "a 0.4 floor must leave a genuine half-and-half merge resolving by seniority"
    );

    // Through the hysteresis: the merge is a re-cut of A (extent grows to the
    // union) plus a debounced dissolve of B. After k sustained detects, A holds
    // the union and B is tombstoned.
    println!("\n================ (b) corridor fusion ================");
    let mut state = HysteresisState::default();
    state.step(&[cand(a.clone(), 6)]);
    state.step(&[cand(b.clone(), 6)]);
    let a_id = id_on_ground(&state, &a).expect("A visible");
    let b_id = id_on_ground(&state, &b).expect("B visible");
    println!("held: A={a_id} B={b_id}");
    assert_ne!(a_id, b_id);
    for s in 0..4 {
        let (vis, mint, diss, _rest, ids) = step(&mut state, &[cand(z.clone(), 12)]);
        println!("  step {s}: visible={vis} mint={mint} diss={diss} ids={ids:?}");
    }
    // A survived holding the fused ground; B is gone from the visible view but
    // retained as a tombstone (MergedInto, reversible).
    assert_eq!(
        id_on_ground(&state, &z),
        Some(a_id.clone()),
        "senior A must hold the fused corridor"
    );
    assert!(
        !state.visible_ids().contains(&b_id),
        "junior B must leave the visible view after k"
    );
    assert!(
        state.is_tombstoned(&b_id),
        "junior B must be tombstoned, not forgotten"
    );
}

// ============================================================================
// (c) The marginal-capture tangle: seniority beats overlap in merge candidacy
// ============================================================================

/// Build the tangle grounds. A corridor at lng 7.0: `long` is the whole 120-pt
/// corridor, `short` is its first 20 pts. A short senior prior sits on `short`
/// (marginally inside `long`); a dominant junior prior sits on `long`.
fn marginal_capture_grounds() -> (Vec<GpsPoint>, Vec<GpsPoint>) {
    (north(46.0, 7.0, 120), north(46.0, 7.0, 20))
}

#[test]
fn marginal_capture_corridor_keeps_its_exact_match_at_every_floor() {
    let (long, short) = marginal_capture_grounds();
    // A (senior, first_seen 1) sits on `short` and is marginally inside `long`.
    // B (junior, first_seen 2) sits on `long` itself: an extent-agreeing 1:1
    // match for the fused candidate.
    let held = vec![
        prior("s_A_short", short.clone(), 1, 3),
        prior("s_B_long", long.clone(), 2, 9),
    ];

    // Single fused candidate = the long corridor (realistic disjoint batch).
    let mo_a = mutual_overlap(&short, &long);
    let mo_b = 1.0;
    println!("\n================ (c) marginal-capture plan ================");
    println!(
        "mutual(short senior A, long Z) = {mo_a:.3}   mutual(long junior B, long Z) = {mo_b:.3}"
    );

    // The agreement tier outranks containment-plus-seniority, so the corridor
    // stays under its exact match's id and the marginal senior folds into it,
    // with or without the merge floor. Seniority alone would hand Z to the
    // marginal senior A and re-mint churn ids on B's ground.
    for floor in [0.0, 0.4] {
        let plan = plan_identity_tuned(
            &held,
            &[cand(long.clone(), 12)],
            &IdentityParams {
                merge_mutual_floor: floor,
            },
        );
        println!(
            "floor {floor}: plan {:?} | retired {:?}",
            plan.decisions, plan.retired
        );
        assert_eq!(
            plan.decisions,
            vec![Decision::MergeInherit {
                id: "s_B_long".into()
            }],
            "floor {floor}: the corridor keeps its exact match's id"
        );
        assert_eq!(
            plan.retired,
            vec![Retirement {
                id: "s_A_short".into(),
                reason: RetireReason::MergedInto {
                    id: "s_B_long".into()
                },
            }],
            "floor {floor}: the marginal senior folds into the corridor"
        );
    }
}

/// The (c2) variant WITH an alternative home for the senior, driven through a
/// full hysteresis run. Historically this shape accreted duplicate corridors
/// at floor 0.0 (the displaced sections were held by the debounce while Z
/// re-minted) and only the merge floor kept it bounded. The containment-tier
/// nomination settles it without the floor: the view converges to two stable
/// sections with no churn on unchanged input, at both floors, with exactly
/// one section on the long corridor.
#[test]
fn marginal_capture_settles_two_stable_without_the_floor() {
    let (long, short) = marginal_capture_grounds();
    let z = long.clone(); // the long corridor re-detected
    let w = short.clone(); // the senior's short section re-emitted

    // Drive N steps of the SAME batch [Z, W] (an unchanged catalogue) and report
    // the visible-section count and mint/dissolve activity. A stable identity
    // layer should settle to two sections and never churn on unchanged input.
    let run = |floor: f64| -> Vec<(usize, usize, usize)> {
        let mut state = HysteresisState::new(HysteresisParams {
            merge_mutual_floor: floor,
            ..HysteresisParams::default()
        });
        // Establish A (short, senior) then B (long, junior) as two held sections.
        state.step(&[cand(short.clone(), 3)]);
        state.step(&[cand(long.clone(), 9)]);
        let mut rows = Vec::new();
        for _ in 0..6 {
            let out = state.step(&[cand(z.clone(), 12), cand(w.clone(), 3)]);
            rows.push((out.visible_count, out.minted, out.dissolved));
        }
        // Final: how many visible sections sit on the long corridor's ground.
        let dups = count_on_ground(&state, &long);
        rows.push((state.visible_len(), dups, 0));
        rows
    };

    println!("\n================ (c2) marginal-capture hysteresis ================");
    println!(
        "(feeding the UNCHANGED batch [long Z, short W] six times; last row = (visible_total, dups_on_long, -))"
    );
    let default_rows = run(0.0);
    println!("default (floor 0.0): per-step (visible, mint, diss) = {default_rows:?}");
    let floored_rows = run(0.4);
    println!("floored (0.4):       per-step (visible, mint, diss) = {floored_rows:?}");

    let default_dups = default_rows.last().unwrap().1;
    let default_visible = default_rows.last().unwrap().0;
    let floored_dups = floored_rows.last().unwrap().1;
    let floored_visible = floored_rows.last().unwrap().0;
    println!("default: {default_visible} visible, {default_dups} on the long corridor");
    println!("floored: {floored_visible} visible, {floored_dups} on the long corridor");
    println!("-----------------------------------------------------------------------------");
    println!(
        "READ: the containment-tier nomination settles this shape without the floor. The senior's\n\
         ground is contained in the long corridor, so its inheritance is the genuine merge shape;\n\
         the short ground re-mints one junior id, and from there the view is two-stable with no\n\
         churn on unchanged input. Pre-fix this grew 2->7 unbounded and needed the floor to bound.\n"
    );

    // The view settles to two sections at floor 0.0 with no churn after the
    // first settle step, and no duplicate accretion on the long corridor.
    let default_visible: Vec<usize> = default_rows[..6].iter().map(|r| r.0).collect();
    assert!(
        default_visible.iter().all(|&v| v == 2),
        "the view must be two-stable at floor 0.0 (visible per step = {default_visible:?})"
    );
    let settled = default_rows[1..6].iter().all(|&(_, m, d)| m == 0 && d == 0);
    assert!(
        settled,
        "an unchanged batch must not mint or dissolve at floor 0.0 after the settle step \
         (rows = {default_rows:?})"
    );
    assert_eq!(
        default_dups, 1,
        "the long corridor holds exactly one section at floor 0.0 (got {default_dups})"
    );
    // The floor additionally prevents the capture: exactly the two priors, one
    // section on the corridor, no churn at all.
    assert_eq!(
        floored_dups, 1,
        "with the floor, the long corridor holds exactly one section"
    );
    assert_eq!(
        floored_visible, 2,
        "with the floor, the view is stable at two sections"
    );
    let stable_after_settle = floored_rows[1..6].iter().all(|&(_, m, d)| m == 0 && d == 0);
    assert!(
        stable_after_settle,
        "with the floor, an unchanged batch must not mint or dissolve"
    );
}

// ============================================================================
// (d) M:N tangle determinism
// ============================================================================

#[test]
fn mn_braid_plan_is_deterministic_and_permutation_stable() {
    // Three overlapping thirds of one 150-point line, as both priors and
    // candidates, so every candidate edges two or three priors: a genuine 3x3
    // braid. The plan must be byte-identical run to run, and the id landing on
    // each ground must be invariant to input order.
    let l = north(46.0, 7.0, 150);
    let grounds = [
        l[0..70].to_vec(),   // south third
        l[40..110].to_vec(), // middle third
        l[80..150].to_vec(), // north third
    ];
    let priors: Vec<PriorSection> = grounds
        .iter()
        .enumerate()
        .map(|(i, g)| prior(&format!("s_{i}"), g.clone(), i as u64 + 1, 5 + i as u32))
        .collect();
    // Candidates: slightly different cuts of the same thirds, so each dominantly
    // matches its own prior and marginally edges the neighbours.
    let cands = vec![
        cand(l[0..75].to_vec(), 5),
        cand(l[35..115].to_vec(), 6),
        cand(l[75..150].to_vec(), 7),
    ];

    // Deterministic: the same inputs give a byte-identical plan.
    let plan_a = plan_identity(&priors, &cands);
    let plan_b = plan_identity(&priors, &cands);
    assert_eq!(
        plan_a, plan_b,
        "identical inputs must give a byte-identical plan"
    );

    // The ground->carried-id map is the semantic result; it must be invariant to
    // the order priors and candidates are presented in.
    let ground_id_map = |priors: &[PriorSection],
                         cands: &[CandidateSection]|
     -> Vec<(usize, Option<String>)> {
        let plan = plan_identity(priors, cands);
        // Key each candidate by which fixed ground third it matches best.
        cands
            .iter()
            .zip(plan.decisions.iter())
            .map(|(c, d)| {
                let third = grounds
                    .iter()
                    .enumerate()
                    .max_by(|(_, a), (_, b)| {
                        mutual_overlap(&c.polyline, a).total_cmp(&mutual_overlap(&c.polyline, b))
                    })
                    .map(|(i, _)| i)
                    .unwrap();
                (third, d.carried_id().map(|s| s.to_string()))
            })
            .collect::<std::collections::BTreeMap<_, _>>()
            .into_iter()
            .collect()
    };

    let baseline = ground_id_map(&priors, &cands);
    println!("\n================ (d) 3x3 braid ================");
    println!("ground third -> carried id: {baseline:?}");

    // Permute both inputs (reverse order) and confirm the same ground keeps the
    // same id. Reversing is a deterministic permutation of the SAME sets.
    let priors_rev: Vec<PriorSection> = priors.iter().rev().cloned().collect();
    let cands_rev: Vec<CandidateSection> = cands.iter().rev().cloned().collect();
    let permuted = ground_id_map(&priors_rev, &cands_rev);
    assert_eq!(
        baseline, permuted,
        "the ground->id map must be invariant to input permutation"
    );

    // And the plan on the reversed candidate order is itself deterministic.
    assert_eq!(
        plan_identity(&priors_rev, &cands_rev),
        plan_identity(&priors_rev, &cands_rev),
        "the permuted plan must also be deterministic"
    );
}
