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
