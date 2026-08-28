# tracematch

GPS route matching, grouping, and section detection for fitness activity data.

Given a collection of GPS tracks (running, cycling, etc.), tracematch can compare routes for similarity, cluster activities by route, and detect frequently-traveled sections across your activity history. Written in Rust. Runs on mobile via UniFFI, in the browser via WASM, or standalone.

## Section detection

The papers each rule rests on are listed in `REFERENCES.md`.

Finds recurring **sections**, corridors that many activities travel, across your whole history at once. One rule defines a section: *a maximal corridor stretch of near-constant traffic composition* (who passes, and how often). Every section's polyline is a real single pass of one activity, never an average and never stitched from several. Entry point: `detect_sections_unified`.

### The pipeline

`N` = activities, `P` = mean points each, `C` = occupied grid cells, `t̄` = mean tracks per cell, `S` = candidate corridors.

| # | Step | What it does | Cost |
|---|------|--------------|------|
| 1 | Evidence grid | Rasterise every track into ~100 m cells; per cell record the unique track set and each track's pass class (1 / 2 / 3+, elevation-aware so switchback legs never inflate the count). Additive and order-free. | O(N·P) |
| 2 | Hot cells | Keep cells visited by at least the support floor. | O(C) |
| 3 | Traffic partition | Union adjacent hot cells whose track sets are mutually near-identical and agree on pass class (union-find). Braided lanes on one road collapse into one corridor; forks and usage changes split it. | O(C·t̄) |
| 4 | Visible boundaries | Keep a cut only where the diverging branch is itself section-worthy; iterate to a fixed point. Sections come out disjoint, no post-hoc overlap removal. | O(k·S²) |
| 5 | Support | Adaptive visit floor by length and corpus size, plus a minimum length. An along-length trim drops any stretch fewer than the floor of the candidate's *own* contributors travel. | O(S·t̄·P) |
| 6 | Ineligible ground | Exclude lift / cable-car spans (sustained grade, one-way, near-straight) before they can form sections. | O(S·P) |
| 7 | Geometry | Reduce each contributor to a single pass; pick the medoid; the polyline is that activity's real points (consensus kept only as metadata: confidence, spread, density). | O(S·t̄·P) |
| 8 | Render guards | Drop laps and spins (single-pass penalty), control forward-and-reverse out-and-backs, and clip a minority end-branch a wrong-way turn leaves behind, display only, at metre resolution. | O(S·P) |
| 9 | Selection backoff | Emit best-first by real metres represented; ground within one cell of an accepted line is never re-emitted, a partial overlap trims to its longest free run. | O(S log S + S·P) |
| 10 | Chain references | Sections tiling one physical line share ONE covering activity's trace, so joints meet on a real point instead of splices from different days. | O(S²) |

Far-apart regions each get their own reference latitude, so one lat/lng scale never distorts distant ground.

Total is **O(N·P)**, linear in the data, dominated by the grid build and the per-contributor portions. Measured on desktop: 426 activities in ~3 s, 1188 activities in ~8.6 s (linear). The rules and their evidence live in `src/sections/unified.rs`; the executable spec is `tests/unified_contracts.rs`, with the failure-mode map in `tests/ASSUMPTIONS.md`.

Older methods (`detect_sections_corridor`, `detect_sections_multiscale`, `detect_sections_flow_graph`) remain callable via `DetectionMethod` but are superseded by the unified detector.

## Scaling: batch vs incremental

The pipeline above is a **batch** over the whole set. Re-running it fresh whenever an activity arrives is wasteful, but naive incremental updates can drift from the batch answer. Three separate questions decide the design, geometry, identity, and cost, and they are orthogonal.

### 1. Geometry, does incremental match batch?

The batch is a pure function of the activity *set*, not its arrival order: the grid is additive, everything sorts by activity id, and the partition is deterministic. So any order yields the same catalogue *if you re-run the batch* (contract: `catalogue_is_a_pure_function_of_the_activity_set`).

An **incremental** update converges to the same geometry only if each new activity re-detects the region it touches:

- Grid, gains the new track in O(P); always exact, order-free.
- Hot cells, monotonic; a few cells may newly cross the floor.
- Partition, the new track changes its cells' track sets, so the connected component around them can merge, split, or bridge two components; re-partition that region.
- Support / geometry / render / backoff, re-run for the affected candidates only. Backoff is globally best-first by score, but a new track only perturbs candidates whose ground overlaps it, so the affected set is spatially local.

Ground the activity never touches stays untouched. **Add-and-forget does not converge**, it misses merges, newly-hot cells, and medoid shifts. Done right, incremental and batch agree; incremental just bounds the work to the changed region.

### 2. Identity, do section ids survive an update?

Determinism is not persistence. The `sec_all_N` ids are a positional index (geographic order): reproducible for a fixed set, but renumbered the moment the set changes. A spatial object has **no intrinsic key**, a section's identity ("my hill climb") is a product notion, not a geometric invariant, and its ground evolves (trims, re-renders), so no geometry hash is stable.

Stable identity must be **assigned once and maintained by matching**, not derived:

- A monotonic id counter; ids are never reused.
- On each update, match new sections to the stored catalogue by corridor overlap (IoU on ground) and carry the matched id forward; an unmatched section gets a new id.
- On a merge, the older / busier id wins and histories concatenate; on a split, the child holding the most original ground keeps the id and the other gets a new one.
- User records, pins, hides, renames, PR history, notifications, hang off that persistent id.

This id layer sits *above* detection and runs whether the geometry came from batch or incremental, so even a full re-detect preserves what the user attached. It carries hysteresis: a visible re-cut needs a decisive margin sustained over several new activities, so the catalogue debounces instead of flip-flopping while still converging to the batch answer.

### 3. Cost, the phone's real operations

Let `B(N)` = O(N·P) be a batch over N activities and `I` = O(P + local) an incremental add.

| Operation | Re-batch each time | Incremental | Notes |
|-----------|--------------------|-------------|-------|
| Cold start, ~90 days | B(90) | (first run is a batch) | one-off |
| +1 activity / day for 3 years | Σ B(N) = **O(N²·P)** | Σ I = **O(N·P)** | the drip: incremental ~1000× cheaper on the tail |
| +100 (expand the window) | B(N+100) | 100·I + region merges | ≈ batch if the 100 spread over old ground |
| +1 to a large corpus | B(N), grows with N | I ≈ independent of N | incremental strongly wins |
| Config change | B(N) | full re-detect, grid reused | rare, background |
| Delete an activity | B(N) | drop from grid, re-detect touched region | evidence is removed, never hidden |

The **drip** is the whole reason to build incremental: re-batching every day for years is quadratic in the library size (each of ~1000 days reprocesses a growing corpus), while incremental stays linear. The 90-day cold start and the occasional window-expansion are fine as background batches, their work is bounded.

**Storage.** GPS tracks dominate: O(N·P), tens of MB for a multi-year library, and already persisted (needed to re-render medoids). The incremental *state* is the coverage grid, small, O(C·t̄), a fraction of a MB with delta-encoded track ids and pass classes. The section catalogue itself is tiny.

**What can be skipped per add.** Never rebuild the grid. Skip partition, support, and geometry for components the new track never enters; skip render and backoff for sections whose ground is untouched. The unavoidable floor is: rasterise the new track (O(P)), test its cells for new hotness, re-detect the connected region. An activity in never-before-seen geography touches nothing downstream, it just accumulates evidence until a cell goes hot.

**Design.** Incremental for the ingest-concurrent drip (a per-activity budget, re-detecting only the touched region); batch for cold start and large expansions as a background, checkpointed job; a persistent identity layer over both, so ids and history outlive any recompute.

## Route matching

Compares two GPS tracks by Average Minimum Distance (AMD). For each point on route A, find the nearest point on route B and average the distances. Runs both directions to detect subsets.

```rust
use tracematch::{GpsPoint, RouteSignature, MatchConfig, compare_routes};

let track = vec![
    GpsPoint::new(51.5074, -0.1278),
    GpsPoint::new(51.5080, -0.1290),
    GpsPoint::new(51.5090, -0.1300),
];

let config = MatchConfig::default();
let sig1 = RouteSignature::from_points("run-1", &track, &config).unwrap();
let sig2 = RouteSignature::from_points("run-2", &track, &config).unwrap();

compare_routes(&sig1, &sig2, &config); // Some(100% match, same direction)
```

```sh
cargo run --example route_matching
```

## Route grouping

Clusters activities by similarity using Union-Find. Each activity starts in its own group. When two routes match, their groups merge.

```rust
use tracematch::{GpsPoint, RouteSignature, MatchConfig, group_signatures};

let track: Vec<GpsPoint> = (0..10)
    .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
    .collect();

let config = MatchConfig::default();
let sigs = vec![
    RouteSignature::from_points("monday", &track, &config).unwrap(),
    RouteSignature::from_points("wednesday", &track, &config).unwrap(),
];

let groups = group_signatures(&sigs, &config);
// One group: ["monday", "wednesday"]
```

```sh
cargo run --example route_grouping
```

## Performance

Benchmarked on real GPS traces (140-490 points per track):

| Operation | Time |
|-----------|------|
| Create signature | 10-16 us |
| Compare two routes | 20-28 us |
| Group 20 routes | 750 us |
| Unified section detection (426 tracks) | ~3 s |
| Unified section detection (1188 tracks) | ~8.6 s |

Detection scales linearly with the corpus; the unified detector trades the older methods' millisecond speed for correctness (pass-class partitioning, along-length support, single-pass geometry, disjoint-by-construction boundaries). At library scale, incremental detection (above) replaces repeated full batches.

R-tree spatial indexing gives O(log n) nearest-neighbour queries. Rayon parallelism is optional via the `parallel` feature.

Run benchmarks: `cargo bench --bench route_matching`

## Install

```sh
cargo add tracematch
```

## References

**Implemented algorithms:**

- Beckmann, N., Kriegel, H.-P., Schneider, R., & Seeger, B. (1990). [The R\*-tree: An efficient and robust access method for points and rectangles](https://doi.org/10.1145/93597.98741). _SIGMOD_, 322-331.

- Tarjan, R. E. (1975). [Efficiency of a good but not linear set union algorithm](https://doi.org/10.1145/321879.321884). _JACM_, 22(2), 215-225.

- Douglas, D. H., & Peucker, T. K. (1973). [Algorithms for the reduction of the number of points required to represent a digitized line](https://doi.org/10.3138/FM57-6770-U75U-7727). _Cartographica_, 10(2), 112-122.

- Kaufman, L., & Rousseeuw, P. J. (1987). Clustering by means of medoids. _Statistical Data Analysis Based on the L1-Norm_, 405-416.

- Zhang, T. Y. & Suen, C. Y. (1984). A fast parallel algorithm for thinning digital patterns. _Communications of the ACM_, 27(3), 236-239.

**Conceptual inspiration:**

- Lee, J.-G., Han, J., & Whang, K.-Y. (2007). [Trajectory clustering: A partition-and-group framework](https://doi.org/10.1145/1247480.1247546). _SIGMOD_, 593-604.

- Xu, W., & Dong, S. (2022). [Unsupervised trajectory segmentation based on multiple motion features](https://doi.org/10.1155/2022/9540944). _Wireless Comm. and Mobile Computing_, 2022.

- Yang, J., Mariescu-Istodor, R., & Fränti, P. (2019). [Three rapid methods for averaging GPS segments](https://doi.org/10.3390/app9224899). _Applied Sciences_, 9(22), 4899.

- Zygouras, N., et al. Discovering corridors from GPS trajectories.

## License

Apache-2.0
