# tracematch

GPS route matching, grouping, and section detection for fitness activity data.

Given a collection of GPS tracks (running, cycling, etc.), tracematch can compare routes for similarity, cluster activities by route, and detect frequently-traveled sections across your activity history. Written in Rust. Runs on mobile via UniFFI, in the browser via WASM, or standalone.

## Section detection

Three methods for finding recurring sections in GPS traces. All produce `Vec<FrequentSection>` with polylines from actual GPS tracks.

### Corridor (default)

Finds corridors where many activities converge. Works directly on raw GPS traces without route grouping. Best coverage for most use cases.

1. Rasterise all tracks into grid cells, count unique tracks per cell
2. Threshold to "hot" cells (cells visited by N or more unique activities)
3. Jaccard-gated union-find: adjacent hot cells merge only if their track sets overlap sufficiently. This prevents over-merging at intersections where runners diverge.
4. For each connected component, find each track's longest contiguous run through it
5. Select the median-distance track as the representative polyline
6. Postprocess: merge nearby sections, remove overlapping

### Density grid

Detects sections where distinct route groups overlap. Requires pre-computed route grouping. Best for finding shared stretches between different routes.

1. Rasterise route representatives into cells (one rep per route group)
2. Build inverted index mapping cells to route IDs
3. 4-connected union-find with Jaccard gate on route sets
4. Extract per-track portions from each connected component
5. Select medoid trace, compute consensus polyline via weighted averaging
6. Postprocess: fold splitting, heading and gradient splits, merge, dedup

### Flow graph

Models GPS data as directed traffic flow to identify road junctions. Sections are edges between divergence points. Network-topology approach.

1. Rasterise tracks, record cell-to-cell transitions with directional flow
2. Identify junctions: cells where 3+ exit directions each carry 15%+ of traffic
3. Merge nearby junctions within 2 cells
4. Trace edges between junctions via BFS along highest-traffic cells
5. Merge pass-through junctions where stub edges are shorter than 5 cells
6. Select median-distance track for each edge as the section polyline

### Choosing a method

```rust
use tracematch::{SectionConfig, DetectionMethod};

let config = SectionConfig {
    detection_method: DetectionMethod::Corridor, // default
    proximity_threshold: 150.0,
    min_section_length: 200.0,
    ..SectionConfig::default()
};

let sections = tracematch::detect_sections(&tracks, &sport_types, &groups, &config);
```

Individual methods are also available directly: `detect_sections_corridor()`, `detect_sections_flow_graph()`, `detect_sections_multiscale()`.

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

## Corpus report

Compare all three detection methods on your own GPS data. Place GPX files in a directory and run:

```sh
cargo run --release --example corpus_report --features synthetic -- ./my_runs
```

Example output from a 426-track running corpus:

```
  ┌──────────────────────┬────────────┬────────────┬────────────┐
  │                      │  Density   │   Flow     │  Corridor  │
  ├──────────────────────┼────────────┼────────────┼────────────┤
  │ Sections             │          6 │          4 │         32 │
  │ Time                 │      52 ms │     130 ms │     178 ms │
  │ Total visits         │        412 │        606 │        821 │
  │ Median visits        │         63 │        123 │          6 │
  │ Median distance (m)  │        890 │        526 │       1059 │
  │ Max distance (m)     │       2960 │        905 │      10206 │
  └──────────────────────┴────────────┴────────────┴────────────┘
```

## Performance

Benchmarked on real GPS traces (140-490 points per track):

| Operation | Time |
|-----------|------|
| Create signature | 10-16 us |
| Compare two routes | 20-28 us |
| Group 20 routes | 750 us |
| Corridor detection (426 tracks) | 178 ms |

R-tree spatial indexing gives O(log n) nearest-neighbor queries. Rayon parallelism is optional via the `parallel` feature.

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
