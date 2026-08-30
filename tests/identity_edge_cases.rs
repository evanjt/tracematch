//! Identity-layer edge cases: degenerate inputs, debounce boundaries, state
//! round-trips, and the pinning pathologies the corpus replay measured.
//! Every scenario is hand-drawn synthetic ground driven through
//! `plan_identity` and `HysteresisState`; no corpus data is required.
//!
//! The pinning pathologies the corpus replay quantified on both corpora
//! (stale visible-only sections that never retire: sion 10 of 66 visible,
//! fullcorpus 22 of 217; re-cut/dissolve kind flips: sion 85 across 44 ids,
//! fullcorpus 685 across 208) are fixed by the streak ledger: both
//! debounce directions accumulate through each other's steps and only a
//! decisive continuation clears them. Every test here is a green contract
//! the veloqrs registry and the change emitter build on.

use tracematch::geo_utils::haversine_distance;
use tracematch::sections::{CARRY_COVERAGE, DEFAULT_K, GROUND_TOL_M, RECUT_AGREEMENT};
use tracematch::{
    CandidateFate, CandidateSection, Decision, GpsPoint, HysteresisParams, HysteresisState,
    PriorSection, RetireReason, Retirement, StepOutcome, mutual_overlap, plan_identity,
    shares_ground,
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

/// A straight north-heading line of `n` ~111 m-spaced points. The 50 m
/// tolerance never bridges neighbouring points at this spacing, so coverage is
/// an exact shared-point count and threshold fractions are exact f64 values.
fn sparse(lat0: f64, lng0: f64, n: usize) -> Vec<GpsPoint> {
    seg(lat0, lng0, 1.0e-3, 0.0, n)
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

/// The marginal spur: the first 70 points of `ground` extended onto a distant
/// corridor. One-way coverage of the ground is 0.7 (an edge), mutual ~0.54
/// (below RECUT_AGREEMENT), so it captures the id as a re-cut carry.
fn spur_of(ground: &[GpsPoint]) -> Vec<GpsPoint> {
    let mut spur: Vec<GpsPoint> = ground[..70].to_vec();
    spur.extend(north(46.007, 7.02, 60));
    spur
}

// ============================================================================
// Pinning pathologies measured by the corpus replay, fixed by the streak ledger
// ============================================================================

/// Scenario: a section's ground stops being ridden. The batch is empty for two
/// detects, then the marginal spur captures the id as a re-cut carry on the
/// third, and the cycle repeats: the ground is gone from 8 of 12 detects.
/// Expected behaviour: the id retires within the run. The dissolve streak
/// survives the marginal capture (a frozen carry preserves it), so the
/// rotation cannot reset the debounce forever. The corpus replay measured
/// the pinning this fixes: stale visible-only sections, sion 10 of 66,
/// fullcorpus 22 of 217, settle trajectories flat.
#[test]
fn capture_rotation_must_not_pin_a_dead_section() {
    let ground = north(46.0, 7.0, 100);
    let spur = spur_of(&ground);

    let mut state = HysteresisState::default();
    let (_, r) = state.step_assign(&[cand(ground.clone(), 8)]);
    let id = r[0].id.clone();

    for _ in 0..4 {
        state.step_assign(&[]);
        state.step_assign(&[]);
        state.step_assign(&[cand(spur.clone(), 3)]);
    }
    assert!(
        !state.visible_ids().contains(&id),
        "a ground gone from 8 of 12 detects must retire; a rotating marginal \
         capture must not reset the dissolve debounce forever"
    );
}

/// Scenario: after minting a full corridor, the batch alternates between the
/// marginal spur and nothing for 30 detects.
/// Expected behaviour: the view converges: the id either tombstones or adopts
/// the spur geometry. Both streaks accumulate through each other's steps
/// (only a decisive continuation clears the ledger), so one of the two fires
/// within 2k detects. The corpus replay measured the oscillation this
/// fixes: 85 kind flips across 44 ids on sion (max 7 per id), 685 across 208
/// on fullcorpus (max 15).
#[test]
fn kind_flip_oscillation_converges() {
    let ground = north(46.0, 7.0, 100);
    let spur = spur_of(&ground);

    let mut state = HysteresisState::default();
    let (_, r) = state.step_assign(&[cand(ground.clone(), 8)]);
    let id = r[0].id.clone();

    for step in 0..30 {
        if step % 2 == 0 {
            state.step_assign(&[cand(spur.clone(), 3)]);
        } else {
            state.step_assign(&[]);
        }
    }
    let converged = state.is_tombstoned(&id) || state.ground_of(&id) == Some(spur.as_slice());
    assert!(
        converged,
        "30 alternating detects must converge: tombstone the id or adopt the \
         spur, not hold the original geometry under an unfired debounce"
    );
}

/// Scenario: the detector emits one degenerate section with an empty polyline
/// for six consecutive detects.
/// Expected behaviour: at most one id is ever minted for it. Two empty
/// grounds are indistinguishable, so they share a corridor vacuously
/// (`coverage` is 1.0 when both sides are empty) and the held copy carries
/// the id instead of dissolving and re-minting every detect.
#[test]
fn empty_candidate_does_not_mint_per_step() {
    let mut state = HysteresisState::default();
    let mut total_minted = 0;
    for _ in 0..6 {
        let out = state.step(&[cand(vec![], 1)]);
        total_minted += out.minted;
    }
    assert!(
        total_minted <= 1,
        "an empty polyline must not mint a fresh id every detect \
         (minted {total_minted} over 6 steps)"
    );
}

// ============================================================================
// Debounce boundaries
// ============================================================================

#[test]
fn k_zero_and_one_disable_debounce() {
    let full = north(46.0, 7.0, 100);
    let half: Vec<GpsPoint> = full[..50].to_vec();

    for k in [0u8, 1] {
        let mut state = HysteresisState::new(HysteresisParams {
            k,
            ..HysteresisParams::default()
        });
        state.step(&[cand(full.clone(), 5)]);

        let (out, r) = state.step_assign(&[cand(half.clone(), 6)]);
        assert_eq!(
            r[0].fate,
            CandidateFate::CarriedAdopted,
            "k={k}: a below-agreement carry must adopt on the first plan"
        );
        assert_eq!(out.recut_applied, 1, "k={k}");
        assert_eq!(state.pending_len(), 0, "k={k}: no debounce residue");

        let out = state.step(&[]);
        assert_eq!(
            out.dissolved, 1,
            "k={k}: the first retiring plan must fire the dissolve"
        );
    }

    // k = 255: the streak walks to the u8 ceiling without overflowing, and the
    // dissolve fires exactly on the 255th sustained detect.
    let mut state = HysteresisState::new(HysteresisParams {
        k: 255,
        ..HysteresisParams::default()
    });
    state.step(&[cand(full, 5)]);
    for step in 1..255u32 {
        let out = state.step(&[]);
        assert_eq!(out.dissolved, 0, "step {step}: below k the id must hold");
    }
    let out = state.step(&[]);
    assert_eq!(out.dissolved, 1, "the 255th sustained detect fires");
}

#[test]
fn recut_agreement_boundary_is_inclusive() {
    // 85 of 100 sparse points: coverage is exactly 85.0/100.0, the same f64 as
    // the RECUT_AGREEMENT literal, so this pins the boundary as inclusive.
    let full = sparse(46.0, 7.0, 100);
    let at: Vec<GpsPoint> = full[..85].to_vec();
    let below: Vec<GpsPoint> = full[..84].to_vec();
    assert_eq!(mutual_overlap(&full, &at), RECUT_AGREEMENT);
    assert!(mutual_overlap(&full, &below) < RECUT_AGREEMENT);

    let mut state = HysteresisState::default();
    let (_, r) = state.step_assign(&[cand(full.clone(), 5)]);
    let id = r[0].id.clone();
    let (out, r) = state.step_assign(&[cand(at.clone(), 6)]);
    assert_eq!(
        r[0].fate,
        CandidateFate::CarriedAdopted,
        "mutual exactly at the plateau adopts immediately"
    );
    assert_eq!(out.recut_applied, 0, "agreement adoption is not a re-cut");
    assert_eq!(state.pending_len(), 0);
    assert_eq!(state.ground_of(&id), Some(at.as_slice()));

    let mut state = HysteresisState::default();
    let (_, r) = state.step_assign(&[cand(full.clone(), 5)]);
    let id = r[0].id.clone();
    let (out, r) = state.step_assign(&[cand(below, 6)]);
    assert_eq!(
        r[0].fate,
        CandidateFate::CarriedFrozen,
        "one point under the plateau debounces"
    );
    assert_eq!(out.recut_applied, 0);
    assert_eq!(state.pending_len(), 1);
    assert_eq!(
        state.ground_of(&id),
        Some(full.as_slice()),
        "the prior geometry is held during the debounce"
    );
}

#[test]
fn flicker_keeps_ground_visible() {
    let g = north(46.0, 7.0, 100);
    let mut state = HysteresisState::default();
    state.step(&[cand(g.clone(), 5)]);
    let id = state.visible_ids()[0].clone();

    for step in 0..12 {
        let batch = if step % 2 == 0 {
            vec![]
        } else {
            vec![cand(g.clone(), 5)]
        };
        let out = state.step(&batch);
        assert_eq!(out.dissolved, 0, "step {step}: flicker must never dissolve");
        assert!(
            state.visible_ids().contains(&id),
            "step {step}: the id must stay visible"
        );
        if step % 2 == 1 {
            assert_eq!(
                state.pending_len(),
                0,
                "step {step}: a full-agreement carry clears the pending debounce"
            );
        }
    }
}

/// Scenario: step_assign is deterministic but NOT idempotent: applying the
/// same batch twice advances the debounce twice.
/// Expected behaviour: the double-stepped clone fires the re-cut one step
/// earlier than the original, so "exactly one apply per detect" is an
/// obligation on the engine, not something the layer forgives.
#[test]
fn double_step_advances_streaks() {
    let full = north(46.0, 7.0, 100);
    let half: Vec<GpsPoint> = full[..50].to_vec();

    let mut original = HysteresisState::default();
    original.step(&[cand(full.clone(), 5)]);
    original.step(&[cand(half.clone(), 5)]);
    assert_eq!(original.pending_len(), 1);
    let id = original.visible_ids()[0].clone();
    let mut doubled = original.clone();

    let (out_once, _) = original.step_assign(&[cand(half.clone(), 5)]);
    assert_eq!(out_once.recut_applied, 0);
    assert_eq!(
        original.ground_of(&id),
        Some(full.as_slice()),
        "one apply: still frozen below k"
    );

    doubled.step(&[cand(half.clone(), 5)]);
    let (out_twice, _) = doubled.step_assign(&[cand(half.clone(), 5)]);
    assert_eq!(
        out_twice.recut_applied, 1,
        "a double apply fires the re-cut a step early"
    );
    assert_eq!(doubled.ground_of(&id), Some(half.as_slice()));
}

// ============================================================================
// Degenerate and duplicate inputs
// ============================================================================

#[test]
fn empty_batch_first_step_is_inert() {
    let mut state = HysteresisState::default();
    let (out, resolutions) = state.step_assign(&[]);
    assert_eq!(out, StepOutcome::default());
    assert!(resolutions.is_empty());
    assert!(state.visible_grounds().is_empty());
    assert_eq!(state.pending_len(), 0);
    assert!(state.tombstone_ids().is_empty());

    let plan = plan_identity(&[], &[]);
    assert!(plan.decisions.is_empty());
    assert!(plan.retired.is_empty());
}

/// Scenario: the detector emits the same section twice, byte-identical.
/// Expected behaviour (current contract, documented deliberately): the first
/// copy inherits, the second mints, so one ground carries two ids. This is
/// the tripwire to flip if candidate dedupe is ever chosen instead.
#[test]
fn duplicate_candidates_resolve_deterministically() {
    let g = north(46.0, 7.0, 100);
    let held = vec![prior("s_1", g.clone(), 1, 5)];
    let next = vec![cand(g.clone(), 5), cand(g.clone(), 5)];

    let plan = plan_identity(&held, &next);
    assert_eq!(
        plan.decisions,
        vec![
            Decision::SplitInherit { id: "s_1".into() },
            Decision::Mint {
                split_from: Some("s_1".into())
            }
        ]
    );
    assert!(plan.retired.is_empty());
    assert_eq!(plan, plan_identity(&held, &next));

    let mut state = HysteresisState::default();
    let (_, r) = state.step_assign(&[cand(g.clone(), 5)]);
    let first = r[0].id.clone();
    let (out, r) = state.step_assign(&next);
    assert_eq!(r[0].id, first);
    assert_eq!(r[0].fate, CandidateFate::CarriedAdopted);
    assert_eq!(r[1].fate, CandidateFate::Minted);
    assert_eq!(out.minted, 1);
    assert_eq!(
        state.visible_len(),
        2,
        "two ids over one ground: the duplicate-mint contract"
    );
}

#[test]
fn identical_priors_tie_break_is_total() {
    let g = north(46.0, 7.0, 100);
    let a = prior("s_a", g.clone(), 7, 5);
    let b = prior("s_b", g.clone(), 7, 5);
    let next = vec![cand(g, 6)];

    let plan = plan_identity(&[a.clone(), b.clone()], &next);
    assert_eq!(
        plan.decisions,
        vec![Decision::MergeInherit { id: "s_a".into() }],
        "priors equal on every seniority key resolve by smaller id"
    );
    assert_eq!(
        plan.retired,
        vec![Retirement {
            id: "s_b".into(),
            reason: RetireReason::MergedInto { id: "s_a".into() },
        }]
    );

    let reversed = plan_identity(&[b, a], &next);
    assert_eq!(
        plan, reversed,
        "the id tie-break must be invariant under input order"
    );
}

// ============================================================================
// Tombstones and restore
// ============================================================================

#[test]
fn restore_enters_clean_then_needs_full_k() {
    let g = north(46.0, 7.0, 100);
    let mut state = HysteresisState::default();
    state.step(&[cand(g.clone(), 5)]);
    let id = state.visible_ids()[0].clone();
    for _ in 0..DEFAULT_K {
        state.step(&[]);
    }
    assert!(state.is_tombstoned(&id));

    let (out, r) = state.step_assign(&[cand(g, 5)]);
    assert_eq!(out.restored, 1);
    assert_eq!(out.minted, 0);
    assert_eq!(r[0].fate, CandidateFate::Restored);
    assert_eq!(
        state.pending_len(),
        0,
        "a restore enters with no debounce residue"
    );

    for step in 1..DEFAULT_K {
        let out = state.step(&[]);
        assert_eq!(
            out.dissolved, 0,
            "step {step}: the pre-dissolve streak must not resume"
        );
    }
    let out = state.step(&[]);
    assert_eq!(out.dissolved, 1, "a fresh full k dissolves it again");
    assert!(state.is_tombstoned(&id));
}

#[test]
fn grown_ground_restores_under_old_id() {
    let g = north(46.0, 7.0, 100);
    let grown = north(46.0, 7.0, 160);
    let mut state = HysteresisState::default();
    let (_, r) = state.step_assign(&[cand(g, 5)]);
    let id = r[0].id.clone();
    for _ in 0..DEFAULT_K {
        state.step(&[]);
    }
    assert!(state.is_tombstoned(&id));

    // The tombstoned 100-point ground lies entirely inside the grown line and
    // covers 100/160 of it, so the mutual overlap clears CARRY_COVERAGE: the
    // old id restores and adopts the grown extent.
    let (out, r) = state.step_assign(&[cand(grown.clone(), 6)]);
    assert_eq!(out.restored, 1);
    assert_eq!(out.minted, 0);
    assert_eq!(r[0].id, id);
    assert_eq!(state.ground_of(&id), Some(grown.as_slice()));
}

/// Scenario: a dead section's tombstoned ground is one-way covered 70% by a
/// spur that continues onto a distant corridor, so most of the spur is foreign
/// ground.
/// Expected behaviour: the spur mints its own id. Restoring the dead id onto
/// it would resurrect a section the athlete stopped riding under a corridor
/// that is mostly somewhere else; a restore needs MUTUAL coverage, not the
/// one-way tolerance extent growth enjoys.
#[test]
fn foreign_extension_does_not_restore_a_tombstone() {
    let ground = north(46.0, 7.0, 100);
    let spur = spur_of(&ground);
    assert!(
        shares_ground(&ground, &spur),
        "precondition: the spur passes the one-way ground test"
    );
    assert!(
        mutual_overlap(&ground, &spur) < CARRY_COVERAGE,
        "precondition: the spur fails the mutual test"
    );

    let mut state = HysteresisState::default();
    let (_, r) = state.step_assign(&[cand(ground, 8)]);
    let id = r[0].id.clone();
    for _ in 0..DEFAULT_K {
        state.step(&[]);
    }
    assert!(state.is_tombstoned(&id));

    let (out, r) = state.step_assign(&[cand(spur, 3)]);
    assert_eq!(out.restored, 0, "a mostly-foreign spur must not restore");
    assert_eq!(out.minted, 1);
    assert_eq!(r[0].fate, CandidateFate::Minted);
    assert_ne!(r[0].id, id);
    assert!(
        state.is_tombstoned(&id),
        "the tombstone stays for real ground"
    );
}

/// Scenario: a candidate covers both a live prior's ground and a tombstoned
/// ground.
/// Expected behaviour (current semantics, documented deliberately): the live
/// prior captures it as a mutual-best carry, the mint path is never reached,
/// so the tombstone is never consulted and stays. There is no expiry surface.
#[test]
fn carried_candidate_never_consults_tombstones() {
    let full = north(46.0, 7.0, 100);
    let south: Vec<GpsPoint> = full[..50].to_vec();
    let north_half: Vec<GpsPoint> = full[50..].to_vec();

    let mut state = HysteresisState::default();
    let (_, r) = state.step_assign(&[cand(south.clone(), 5), cand(north_half.clone(), 5)]);
    let south_id = r[0].id.clone();
    let north_id = r[1].id.clone();
    for _ in 0..DEFAULT_K {
        state.step(&[cand(south.clone(), 5)]);
    }
    assert!(state.is_tombstoned(&north_id));

    assert!(
        shares_ground(&full, &north_half),
        "the full line does share the tombstoned ground"
    );
    let (out, r) = state.step_assign(&[cand(full, 6)]);
    assert_eq!(out.restored, 0, "a carried candidate never restores");
    assert_eq!(r[0].id, south_id);
    assert_eq!(r[0].fate, CandidateFate::CarriedFrozen);
    assert!(
        state.is_tombstoned(&north_id),
        "the tombstone stays: no expiry surface"
    );
}

/// Scenario: a tombstoned senior restores, then a candidate spans its ground
/// plus a younger neighbour's.
/// Expected behaviour: the restored id wins the merge, which is only possible
/// if the restore reused its original first_seen rather than re-minting a
/// younger ordinal.
#[test]
fn restored_id_keeps_seniority() {
    let full = north(46.0, 7.0, 100);
    let south: Vec<GpsPoint> = full[..50].to_vec();
    let north_half: Vec<GpsPoint> = full[50..].to_vec();

    let mut state = HysteresisState::default();
    let (_, r) = state.step_assign(&[cand(south.clone(), 5)]);
    let senior = r[0].id.clone();
    for _ in 0..DEFAULT_K {
        state.step(&[]);
    }
    assert!(state.is_tombstoned(&senior));

    let (out, r) = state.step_assign(&[cand(south, 5), cand(north_half, 4)]);
    assert_eq!(out.restored, 1);
    assert_eq!(r[0].id, senior);
    let junior = r[1].id.clone();

    let (_, r) = state.step_assign(&[cand(full, 9)]);
    assert_eq!(r.len(), 1);
    assert_eq!(r[0].id, senior, "the restored senior wins the merge");
    assert_ne!(r[0].id, junior);
}

#[test]
fn forget_releases_id_forever() {
    let g = north(46.0, 7.0, 100);
    let g_half: Vec<GpsPoint> = g[..50].to_vec();
    let mut state = HysteresisState::default();
    let (_, r) = state.step_assign(&[cand(g.clone(), 5)]);
    let first = r[0].id.clone();
    state.step(&[cand(g_half, 5)]);
    assert_eq!(state.pending_len(), 1);

    state.forget(&first);
    assert_eq!(state.visible_len(), 0);
    assert_eq!(state.pending_len(), 0);
    assert!(!state.is_tombstoned(&first));
    state.forget(&first);
    assert_eq!(state.visible_len(), 0, "forget is idempotent");

    let (_, r) = state.step_assign(&[cand(g.clone(), 5)]);
    assert_eq!(
        r[0].fate,
        CandidateFate::Minted,
        "forgotten ground mints, never restores"
    );
    let second = r[0].id.clone();
    assert_ne!(second, first, "a forgotten id is never re-issued");

    for _ in 0..DEFAULT_K {
        state.step(&[]);
    }
    assert!(state.is_tombstoned(&second));
    state.forget(&second);
    assert!(state.tombstone_ids().is_empty());

    let (_, r) = state.step_assign(&[cand(g, 5)]);
    assert_eq!(r[0].fate, CandidateFate::Minted);
    assert_ne!(r[0].id, first);
    assert_ne!(r[0].id, second);
}

// ============================================================================
// Per-id fire-time reporting (the change emitter contract)
// ============================================================================

/// Scenario: four held corridors; the batch then sustains a union of the
/// first two (a merge), drops the third (a dissolve), and halves the fourth
/// (a re-cut) for k detects.
/// Expected behaviour: nothing is reported per-id until the debounce fires;
/// on the k-th detect the outcome names each fired id with the reason the
/// plan gave at fire time. The counts are the lengths of the id lists.
#[test]
fn fired_changes_report_ids_and_reasons() {
    let z = north(46.0, 7.0, 100);
    let a_half: Vec<GpsPoint> = z[..50].to_vec();
    let b_half: Vec<GpsPoint> = z[50..].to_vec();
    let c = north(46.0, 7.02, 100);
    let d = north(46.0, 7.04, 100);
    let d_half: Vec<GpsPoint> = d[..50].to_vec();

    let mut state = HysteresisState::default();
    let (_, r) = state.step_assign(&[cand(a_half, 5), cand(b_half, 5), cand(c, 5), cand(d, 5)]);
    let (a_id, b_id, c_id, d_id) = (
        r[0].id.clone(),
        r[1].id.clone(),
        r[2].id.clone(),
        r[3].id.clone(),
    );

    let batch = vec![cand(z, 10), cand(d_half.clone(), 6)];
    for step in 0..(DEFAULT_K - 1) {
        let out = state.step(&batch);
        assert!(out.retired.is_empty(), "step {step}: nothing fired yet");
        assert!(out.recut_ids.is_empty(), "step {step}: nothing fired yet");
    }

    let out = state.step(&batch);
    assert_eq!(
        out.retired,
        vec![
            Retirement {
                id: b_id,
                reason: RetireReason::MergedInto { id: a_id.clone() },
            },
            Retirement {
                id: c_id,
                reason: RetireReason::Dissolved,
            },
        ],
        "each fired retirement carries its fire-time reason"
    );
    assert_eq!(out.dissolved, out.retired.len());
    assert_eq!(
        out.recut_ids,
        vec![a_id.clone(), d_id.clone()],
        "the union adoption and the halving both fire as re-cuts"
    );
    assert_eq!(out.recut_applied, out.recut_ids.len());
    assert_eq!(state.ground_of(&d_id), Some(d_half.as_slice()));
}

// ============================================================================
// Mid-debounce competition
// ============================================================================

/// Scenario: a prior mid re-cut debounce sees a batch containing both its
/// pending target (the half) and the carved-off remainder.
/// Expected behaviour: the prior competes on its target footprint, not the
/// frozen full one, so the remainder is not captured: it mints exactly once
/// and the view settles at two sections with no churn.
#[test]
fn midrecut_prior_competes_on_target() {
    let full = north(46.0, 7.0, 100);
    let half: Vec<GpsPoint> = full[..50].to_vec();
    let other: Vec<GpsPoint> = full[50..].to_vec();

    let mut state = HysteresisState::default();
    let (_, r) = state.step_assign(&[cand(full, 5)]);
    let id = r[0].id.clone();
    let (_, r) = state.step_assign(&[cand(half.clone(), 6)]);
    assert_eq!(r[0].fate, CandidateFate::CarriedFrozen);

    let (out, r) = state.step_assign(&[cand(half.clone(), 6), cand(other.clone(), 4)]);
    assert_eq!(r[0].id, id);
    assert_eq!(r[1].fate, CandidateFate::Minted);
    assert_eq!(out.minted, 1, "the remainder mints exactly once");
    let other_id = r[1].id.clone();

    for step in 0..5 {
        let (out, r) = state.step_assign(&[cand(half.clone(), 6), cand(other.clone(), 4)]);
        assert_eq!(out.minted, 0, "step {step}: no churn mints");
        assert_eq!(out.visible_count, 2, "step {step}");
        assert_eq!(r[0].id, id, "step {step}");
        assert_eq!(r[1].id, other_id, "step {step}");
    }
    assert_eq!(
        state.ground_of(&id),
        Some(half.as_slice()),
        "the sustained re-cut fired and adopted the half"
    );
}

// ============================================================================
// Determinism and persistence
// ============================================================================

#[test]
fn step_is_deterministic_from_any_state() {
    let a = north(46.0, 7.0, 100);
    let b = north(46.0, 7.02, 100);
    let b_half: Vec<GpsPoint> = b[..50].to_vec();
    let c = north(46.0, 7.04, 100);
    let d = north(46.0, 7.06, 100);

    // A rich state: A stable, B mid re-cut, C tombstoned, ordinal advanced.
    let mut state = HysteresisState::default();
    state.step(&[cand(a.clone(), 5), cand(b.clone(), 5), cand(c.clone(), 5)]);
    for _ in 0..DEFAULT_K {
        state.step(&[cand(a.clone(), 5), cand(b.clone(), 5)]);
    }
    assert_eq!(state.tombstone_ids().len(), 1);
    state.step(&[cand(a.clone(), 5), cand(b_half.clone(), 5)]);
    assert_eq!(state.pending_len(), 1);

    let batch = vec![cand(a, 6), cand(b_half, 6), cand(c, 6), cand(d, 3)];
    let mut s1 = state.clone();
    let mut s2 = state.clone();
    let (out1, r1) = s1.step_assign(&batch);
    let (out2, r2) = s2.step_assign(&batch);
    assert_eq!(out1, out2);
    assert_eq!(r1.len(), r2.len());
    for (x, y) in r1.iter().zip(r2.iter()) {
        assert_eq!(x.id, y.id);
        assert_eq!(x.fate, y.fate);
    }
    assert_eq!(s1.visible_grounds(), s2.visible_grounds());
    assert_eq!(s1.tombstone_ids(), s2.tombstone_ids());
    assert_eq!(s1.pending_len(), s2.pending_len());
}

/// Scenario: the persistence contract: a state holding a stable id, a mid
/// re-cut debounce, a mid dissolve debounce, a tombstone, and an advanced
/// ordinal survives a serde round-trip.
/// Expected behaviour: both copies step identically, and both in-flight
/// debounces fire on the same later step, proving the streaks round-tripped.
#[test]
fn serde_roundtrip_preserves_streaks() {
    let a = north(46.0, 7.0, 100);
    let b = north(46.0, 7.02, 100);
    let b_half: Vec<GpsPoint> = b[..50].to_vec();
    let c = north(46.0, 7.04, 100);
    let d = north(46.0, 7.06, 100);

    let mut state = HysteresisState::default();
    state.step(&[
        cand(a.clone(), 5),
        cand(b.clone(), 5),
        cand(c.clone(), 5),
        cand(d, 5),
    ]);
    for _ in 0..DEFAULT_K {
        state.step(&[cand(a.clone(), 5), cand(b.clone(), 5), cand(c.clone(), 5)]);
    }
    assert_eq!(state.tombstone_ids().len(), 1);
    // One step arms both debounces: B mid re-cut, C mid dissolve.
    state.step(&[cand(a.clone(), 5), cand(b_half.clone(), 5)]);
    assert_eq!(state.pending_len(), 2);

    let json = serde_json::to_string(&state).expect("serialise");
    let mut restored: HysteresisState = serde_json::from_str(&json).expect("deserialise");

    let batch = vec![cand(a, 6), cand(b_half, 6)];
    let (out_a, r_a) = state.step_assign(&batch);
    let (out_b, r_b) = restored.step_assign(&batch);
    assert_eq!(out_a, out_b);
    for (x, y) in r_a.iter().zip(r_b.iter()) {
        assert_eq!(x.id, y.id);
        assert_eq!(x.fate, y.fate);
    }
    assert_eq!(out_a.dissolved, 0);
    assert_eq!(out_a.recut_applied, 0);

    let (out_a, _) = state.step_assign(&batch);
    let (out_b, _) = restored.step_assign(&batch);
    assert_eq!(out_a, out_b);
    assert_eq!(out_a.dissolved, 1, "the dissolve fires on the same step");
    assert_eq!(out_a.recut_applied, 1, "the re-cut fires on the same step");
    assert_eq!(state.visible_grounds(), restored.visible_grounds());
    assert_eq!(state.tombstone_ids(), restored.tombstone_ids());
    assert_eq!(state.pending_len(), restored.pending_len());
}

// ============================================================================
// The fate contract the veloqrs reconcile leans on
// ============================================================================

/// Scenario: a scripted mix of mints, splits, dissolves, and restores.
/// Expected behaviour, before/after every step: Carried* resolves to a
/// pre-step visible id, Restored to a pre-step tombstone, Minted to neither;
/// resolutions parallel the batch and ids are distinct within a step.
#[test]
fn fate_membership_property() {
    let p = north(46.0, 7.0, 100);
    let p_half: Vec<GpsPoint> = p[..50].to_vec();
    let p_other: Vec<GpsPoint> = p[50..].to_vec();
    let q = north(46.0, 7.02, 100);
    let r_line = north(46.0, 7.04, 100);

    let script: Vec<Vec<CandidateSection>> = vec![
        vec![cand(p.clone(), 3), cand(q.clone(), 3)],
        vec![
            cand(p.clone(), 4),
            cand(q.clone(), 4),
            cand(r_line.clone(), 2),
        ],
        vec![cand(p_half.clone(), 4), cand(q.clone(), 5)],
        vec![
            cand(p_half.clone(), 5),
            cand(p_other.clone(), 2),
            cand(q.clone(), 5),
        ],
        vec![],
        vec![],
        vec![],
        vec![cand(p.clone(), 5), cand(r_line.clone(), 3)],
        vec![
            cand(p.clone(), 5),
            cand(q.clone(), 5),
            cand(r_line.clone(), 3),
        ],
        vec![cand(p_half.clone(), 6), cand(q.clone(), 6)],
        vec![],
        vec![cand(p.clone(), 6)],
    ];

    let mut state = HysteresisState::default();
    for (step, batch) in script.iter().enumerate() {
        let pre_visible = state.visible_ids();
        let pre_tombs = state.tombstone_ids();
        let (out, resolutions) = state.step_assign(batch);
        assert_eq!(out.raw_count, batch.len(), "step {step}");
        assert_eq!(
            resolutions.len(),
            batch.len(),
            "step {step}: resolutions parallel the batch"
        );
        let mut seen = std::collections::BTreeSet::new();
        for res in &resolutions {
            assert!(!res.id.is_empty(), "step {step}: no blank ids");
            assert!(
                seen.insert(res.id.clone()),
                "step {step}: id {} resolved twice in one step",
                res.id
            );
            match res.fate {
                CandidateFate::CarriedAdopted | CandidateFate::CarriedFrozen => assert!(
                    pre_visible.contains(&res.id),
                    "step {step}: a carry must target a pre-step visible id ({})",
                    res.id
                ),
                CandidateFate::Restored => assert!(
                    pre_tombs.contains(&res.id),
                    "step {step}: a restore must target a pre-step tombstone ({})",
                    res.id
                ),
                CandidateFate::Minted => assert!(
                    !pre_visible.contains(&res.id) && !pre_tombs.contains(&res.id),
                    "step {step}: a mint must be a fresh id ({})",
                    res.id
                ),
            }
        }
    }
}

// ============================================================================
// Metric canaries
// ============================================================================

/// Scenario: user-facing suppression rides on shares_ground, so its two
/// constants get behavioural tripwires: retuning GROUND_TOL_M or
/// CARRY_COVERAGE moves these flips and this test says so loudly.
#[test]
fn shares_ground_canary() {
    assert_eq!(GROUND_TOL_M, 50.0);
    assert_eq!(CARRY_COVERAGE, 0.6);

    // Sparse spacing: coverage is an exact shared-point count, so 59 and 61
    // shared points of 100 sit exactly either side of CARRY_COVERAGE.
    let base = sparse(46.0, 7.0, 141);
    let a: Vec<GpsPoint> = base[..100].to_vec();
    let just_below: Vec<GpsPoint> = base[41..141].to_vec();
    let just_above: Vec<GpsPoint> = base[39..139].to_vec();
    assert!(
        !shares_ground(&a, &just_below),
        "coverage 0.59 both ways sits under CARRY_COVERAGE"
    );
    assert!(
        shares_ground(&a, &just_above),
        "coverage 0.61 both ways clears CARRY_COVERAGE"
    );

    // Lateral offsets either side of GROUND_TOL_M, verified with the same
    // haversine the metric uses so the canary cannot drift from it.
    let m_per_deg_lng =
        haversine_distance(&GpsPoint::new(46.0, 7.0), &GpsPoint::new(46.0, 7.001)) / 0.001;
    let inside = seg(46.0, 7.0 + 45.0 / m_per_deg_lng, 1.0e-3, 0.0, 100);
    let outside = seg(46.0, 7.0 + 55.0 / m_per_deg_lng, 1.0e-3, 0.0, 100);
    let off_in = haversine_distance(&a[0], &inside[0]);
    let off_out = haversine_distance(&a[0], &outside[0]);
    assert!(
        off_in > 40.0 && off_in < GROUND_TOL_M,
        "offset {off_in:.1} m must sit just inside the tolerance"
    );
    assert!(
        off_out > GROUND_TOL_M && off_out < 60.0,
        "offset {off_out:.1} m must sit just outside the tolerance"
    );
    assert!(shares_ground(&a, &inside));
    assert!(!shares_ground(&a, &outside));
}

// ----------------------------------------------------------- ground anchors

/// The heart of a line sits half way along it by arc length, and the cell it
/// falls in is the same for every point of the same cell and different one
/// cell over, so an id anchored on it is stable to jitter and unique to ground.
#[test]
fn a_heart_anchors_to_a_global_cell() {
    use tracematch::sections::ANCHOR_CELL_M;
    use tracematch::{GpsPoint, earth_cell, section_heart};
    let line: Vec<GpsPoint> = (0..=20)
        .map(|i| GpsPoint::new(46.0, 7.0 + i as f64 * 0.001))
        .collect();
    let heart = section_heart(&line).expect("a heart");
    assert!(
        (heart.longitude - 7.01).abs() < 1e-6,
        "half way along: {heart:?}"
    );
    assert!(section_heart(&[]).is_none());

    let cell = earth_cell(&heart);
    let jitter = GpsPoint::new(heart.latitude + 0.00005, heart.longitude);
    let far = GpsPoint::new(
        heart.latitude + 2.0 * ANCHOR_CELL_M / 111_132.0,
        heart.longitude,
    );
    assert!(
        earth_cell(&jitter) == cell || (earth_cell(&jitter).0 - cell.0).abs() <= 1,
        "a 5 m jitter stays within one cell of the heart"
    );
    assert_ne!(earth_cell(&far), cell, "two cells north is another cell");
}
