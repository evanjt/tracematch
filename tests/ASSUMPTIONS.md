# Assumption register

Every load-bearing assumption the previous detection stack broke on, mapped
to the executable contract that now pins it. The contracts run on
deterministic synthetic ground (`tests/shapes/mod.rs`, no RNG, no clock, no
private data) under plain `cargo test`, so CI holds the spec with no corpus
present. The algorithm and its rules live in `src/sections/unified.rs`.

| Broken assumption (v1 failure mode) | Contract |
|---|---|
| Whole activities were pooled, so a shared stretch inside otherwise unique rides never became a section | `deviation_emerges_from_unique_outings` |
| Geometry was a consensus average, stitched from many activities | real-trace check in `assert_catalogue_invariants` (every test), sharpened by `braid_variants_read_as_real_lines_not_a_midline` and `separated_variants_stay_distinct_ground` |
| Near-duplicates within 2x proximity were merged into one blob | corridor-disjointness in `assert_catalogue_invariants`; never-merge semantics in both `*_variants_*` tests |
| Boundaries fell at arbitrary places, or nowhere the athlete could see | `oval_and_stem_split_at_the_usage_change`, `lollipop_splits_at_the_mouth`, `persona_racer_hill_is_single_passed`, `persona_commuter_is_one_unfragmented_corridor` |
| Laps and return legs inflated section geometry | single-pass length bounds in the oval, lollipop, and racer tests; `self_overlap_frac` in `assert_catalogue_invariants` |
| Switchback climbs were cut into legs by re-pass logic | `switchback_climb_survives_as_one_section` |
| Cable cars and funiculars became sections | `lift_ground_forms_no_section_but_the_piste_does` |
| Support floors could be bypassed, or one-off ground invented sections | `grid_city_respects_floors_and_forks`, `persona_casual_produces_no_sections`, `persona_weekender_finds_only_the_favourite_loop` |
| The catalogue depended on arrival order (the install-window precedent) | `catalogue_is_a_pure_function_of_the_activity_set` |
| Evidence semantics were unclear on deletion | `deleting_activities_is_the_only_way_evidence_leaves` |
| One global projection plane serves every continent | `far_ground_changes_nothing_local` |
| A partly represented candidate was emitted whole if it slipped under the backoff share, re-emitting represented ground | trim semantics in `assert_catalogue_invariants` (both-direction corridor share < 60%); `crossing_islands_bridge_but_long_overlap_splits`, `probe_beside_accepted_line_is_represented` (unit). The mask is deliberately planimetric: plan cells are partition-atomic so stacked ground never becomes a separate candidate here, and cross-day absolute elevation comparison is barometric drift, not signal (an elevation guard was probed and rejected 2026-07-21) |
| Chain members each picked their own reference, so one physical line rendered as splices from different days | `chain_members_share_one_reference_and_meet_exactly`. Unification changes the source trace only, never the span: member extents stay medoid-anchored (re-deriving spans from the cover activity's portions was probed 2026-07-21 and sheared jackknife core persistence 92% → 70%) |
| A rendered representative was a real trace yet not a single pass: a mid-line spin too tight for the cell-event cut to catch, or a junction where every visit mills | single-pass render guard (`straight_line_has_no_self_pass`, `out_and_back_revisits_its_outbound_leg`, `closed_loop_join_is_exempt`, unit). The render takes the cleanest contributing pass; a candidate with no pass under the floor backs off. Verified 2026-07-24: two spins re-render straight and the 178-visit junction blob backs off, every other section holds. Section-level `self_overlap` was blind to a tight spin (read 0.00 on a 40%-revisiting line), so the render metric is now the reported one |
| A forward-and-reverse over the same ground rendered as a section: a closed out-and-back whose short spurs slip the self-pass arc gap | `out_and_back_scores_high_but_a_through_line_does_not`, `switchback_climb_is_not_an_out_and_back` (unit). Only CLOSED lines are scored (endpoints within a fifth of the length), so a switchback climbs away and keeps its antiparallel hairpins (`switchback_climb_survives_as_one_section`). Verified 2026-07-24: the flagged out-and-back and one 4-visit twin re-render one-way, every loop and climb byte-identical, no out-and-back survives on either corpus |
| A section rendered a minority end-branch that most of its own traffic skips: a wrong-way turn at a junction (short, well-supported, but off the through-flow) drawn as if it were the section's extent | `a_minority_branch_behind_a_cliff_clips_but_a_taper_does_not` (unit). The rendered line is clipped, display only, where metre-resolution support falls under half the line's median across a short end run behind a cliff (the body sample abutting it carries at least double), so a genuine wrong-turn branch is dropped from the drawing while a gradual taper or a legitimately lower-traffic half is left whole. Counts, extent, and occupied footprint keep the full portion. The clean signal is directed flow continuation through a junction, which the coarse coverage grid cannot express (its one-ring tolerance leaks through-traffic onto the branch); the metre-resolution end clip is the interim, and the exact query lands with B1's per-portion alignment. Verified 2026-07-24: sec_all_113's SE tail is clipped so it connects to its neighbour, six more wrong-turn branches drop elsewhere, gradients and tapers are untouched, no new display overlaps on either corpus |
| Support held only at the supernode total, so a near-private spur welded onto a busy corridor through the traffic gradient at their junction (adjacent thin cells always pass the one-missing-track rule), inherited its visit count, and could be rendered | `one_off_tail_is_cut_where_its_own_support_ends` (contract), `support_counts_contributor_passes_not_strangers_or_clips` (unit). Support binds along the length: every cell must be traversed by the section's own floor of its OWN contributors' qualifying passes (all runs count, so a fragmented loop traversal is not read as absent; stranger traffic and corner clips lend nothing). The floor is fixed pre-trim (re-deriving it as portions shorten ratchets the length tier and spirals short sections to death) and ordering uses the pre-trim score (the trim corrects extent, never priority — score-reordering was probed 2026-07-24 and reshuffled a whole city's catalogue). Verified 2026-07-24: the flagged welded section re-renders as its uniformly-traversed corridor under a new representative, four 3-visit stitched lines whose middles were one outing's private ground dissolve, one inflated count de-inflates (11 to 3 on identical geometry), jackknife improves on the full corpus and holds on Sion, disjointness 0 violations both corpora |

## Identity and hysteresis contracts (2026-07-29)

The identity layer (`src/sections/identity.rs`) and its veloqrs registry
mirror, pinned after the D2 corpus gate. Pure-layer contracts live in
`tests/identity_edge_cases.rs`; seam contracts in veloqrs
`tests/identity_seam.rs` (synthetic feature); detector explanations in
`tests/boundary_records.rs` and `tests/tunables_neighbourhood.rs`.

| Assumption | Contract |
|---|---|
| Same state and candidates give byte-identical outcomes, from any mid-flight state | `step_is_deterministic_from_any_state`, `mn_braid_plan_is_deterministic_and_permutation_stable` |
| Exactly one apply per detect: a step advances streaks, so re-applying the same batch would fire debounced changes early | `double_step_advances_streaks`; registry side `double_apply_is_a_no_op` |
| Mid-flight debounce state survives serialisation, or hysteresis silently resets each restart | `serde_roundtrip_preserves_streaks` (pure layer holds); veloqrs `grave_restore_survives_restart` is RED (see defects below) |
| A dissolved corridor that re-forms comes back under its old id, with its old seniority, and needs a full fresh k to dissolve again | `grown_ground_restores_under_old_id`, `restored_id_keeps_seniority`, `restore_enters_clean_then_needs_full_k`, `carried_candidate_never_consults_tombstones` |
| The k debounce is well defined at its edges (0, 1, 255) | `k_zero_and_one_disable_debounce` |
| The agreement plateau boundary is inclusive: mutual exactly 0.85 adopts, 0.84 freezes | `recut_agreement_boundary_is_inclusive` |
| Fates are membership-honest: Carried* was visible, Restored was tombstoned, Minted was neither; resolutions parallel, ids distinct (the D3 fate-verbatim reconcile spec) | `fate_membership_property` |
| `forget()` transfers ownership forever: idempotent, never re-issues the id | `forget_releases_id_forever` |
| The suppression metric cannot be silently retuned | `shares_ground_canary` (0.59/0.61 coverage, 45/55 m offsets) |
| Flickering-but-real ground stays visible; a full-agreement carry clears the pending | `flicker_keeps_ground_visible`, `flip_flop_damps_to_a_stable_view` |
| During a re-cut debounce the prior competes on its pending target, so the frozen footprint cannot capture the carved-off piece | `midrecut_prior_competes_on_target` |
| Duplicate candidates and identical priors resolve deterministically (duplicate-mint is a conscious contract, the tripwire to flip if dedupe is chosen) | `duplicate_candidates_resolve_deterministically`, `identical_priors_tie_break_is_total` |
| Every refused or reshaped candidate explains itself as data: Backoff, Trim, NoSinglePass | `near_duplicate_corridor_backs_off_with_a_record`, `partly_represented_candidate_is_trimmed_to_its_own_run`, `milling_ground_with_no_single_pass_is_refused_with_a_record` |
| Unpin releases convergence immediately: the first fold after clearing a pin emits the withheld re-cut with no new activity needed | `unpin_releases_the_withheld_recut` |
| Constants sit on plateaus: one-step tunables neighbours keep the catalogue invariant-clean and the count flat | `commuter_corridor_holds_across_pass_neighbours`, `oval_stem_holds_across_pass_neighbours` |
| The registry mirrors the pure layer mechanically: rows equal pure grounds after every apply, graves track tombstones, every promotion relinquishes, durable rows never collide | veloqrs `mirror_rows_equal_pure_grounds`, `graves_track_tombstones_exactly`, `*_relinquishes_and_survives_resync` (six mutations), `durable_rows_never_collide` |

## Red gates (known defects, `#[ignore]`, fixes ride D5)

Written red on purpose so the fix ungates them instead of rediscovering
the defect. Corpus evidence from the D2 gate replay is quoted in each
ignore reason.

| Gate | Defect it pins |
|---|---|
| `capture_rotation_must_not_pin_a_dead_section` | A marginal capture flips the pending kind and resets the dissolve streak, so a dead section never retires while neighbours keep adopting (sion 10/66 visible pinned, fullcorpus 22/217) |
| `kind_flip_oscillation_converges` | ReCut and Dissolve pendings alternate without progress (corpus max 15 flips per id); neither ever fires |
| `empty_candidate_does_not_mint_per_step` | An empty polyline edges with nothing, including its own held copy, so one degenerate detector output mints a fresh id every step |
| veloqrs `durable_claim_mid_tombstone_clears_the_grave` | Relinquish scans visible rows only, so a durable claim on tombstoned ground pins the grave forever |

Ungated: veloqrs `grave_restore_survives_restart` is green — the identity
blob is rmp-encoded (blob version 2), whose length-prefixed arrays recover
the trailing skip-if-None fields that desynced postcard's positional
stream. A v1 blob reseeds by version tag, the same outcome it always got.

## Contracts that land with later tranches

| Assumption | Where its contract lands |
|---|---|
| One engine, no cosmetic method selector | C4 deletion, compiler-verified |
| Split lineage: a mint carries the prior it was carved from | D3; `fate_membership_property` is the reconcile spec it extends |
| Per-id retire reasons at fire time | D5, ungating the red gates above |

## Defects these contracts caught at birth (2026-07-21)

Written before the engine was adjusted, the suite immediately found two
real defects, which is the register working as intended:

1. Excluded lift ground re-entered sections through geometry: the portion
   cut walked the raw track and bridged across a lift span whose base and
   summit both touched the component. Fixed by walking geometry per
   lift-free keep range (`CoverageGrid::keep`).
2. The catalogue depended on arrival order: portions followed input index
   order, so the anchor, tie-breaks, and the representative changed with
   input permutation, and `ref_lat` summed floats in input order. Fixed by
   canonical portion order (activity id) and order-stable accumulation.

## Defects the identity battery caught at birth (2026-07-29)

Same pattern, one tranche later: the seam contracts found two live
registry defects before any fix was designed.

1. The section identity blob is written byte-correctly inside the save
   transaction but fails its own postcard decode: `skip_serializing_if`
   fields inside the serialised graph (`GpsPoint.elevation`,
   `FrequentSection.consensus_state`) desync postcard's positional
   stream, which `codec.rs`'s own doc forbids. `load()` falls back to
   reseed, so every app restart silently drops graves, tombstones, and
   debounce streaks. The pre-existing restart gate
   (`identity_registries_survive_restart`) passed vacuously because its
   grave-free state is one a reseed reproduces byte-for-byte.
2. No path clears a grave on durable-claimed ground: mutation-time and
   apply-time relinquish iterate visible rows only, and suppression only
   drops candidates, so a custom claim on a tombstoned corridor pins the
   grave and tombstone across every later apply. The grave-side sibling
   of the corpus replay's pinned stale sections.
