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

## Contracts that land with later tranches

| Assumption | Where its contract lands |
|---|---|
| Incremental equals batch (order-free parity on the ingest path) | B1; `catalogue_is_a_pure_function_of_the_activity_set` is the spec it must meet |
| Extent evolution converges without flip-flopping | B2 hysteresis tests |
| A user-disabled section's corridor stays hidden | B-tranche intent-record tests (veloqrs) |
| A config change never silently no-ops | B1 invalidation contract (veloqrs) |
| One engine, no cosmetic method selector | C4 deletion, compiler-verified |

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
