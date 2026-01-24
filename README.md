# tracematch

High-performance GPS route matching library.

## Features

- **Route Matching** — AMD-based polyline similarity with bidirectional detection
- **Route Grouping** — Cluster activities by route using Union-Find
- **Section Detection** — Multi-scale detection of frequently-traveled sections
- **Spatial Indexing** — R-tree for O(log n) viewport queries
- **Parallel Processing** — Optional rayon-based parallelism

## Performance

Fast enough for real-time use on mobile devices. Benchmarked on real GPS traces (140-490 points per track):

| Operation | Time | What it means |
|-----------|------|---------------|
| Create signature | 10-16 µs | Process a 200-point track in 0.016ms |
| Compare two routes | 20-28 µs | Check similarity in 0.028ms |
| Group 20 routes | 750 µs | Cluster all routes in under 1ms |

**Why it's fast:**

- **R-tree spatial indexing** — O(log n) nearest-neighbor queries instead of O(n) linear scan
- **Early exit** — Dissimilar routes (different regions) detected in ~3 nanoseconds
- **Rayon parallelism** — Optional multi-core processing for large datasets
- **Zero-copy design** — Minimal allocations in hot paths

Run benchmarks: `cargo bench --bench route_matching`

## Installation

```sh
cargo add tracematch
```

## How It Works

**Route Matching** compares two GPS tracks by measuring how far apart they are. For each point on route A, find the nearest point on route B and measure that distance. Average all these distances to get the AMD (Average Minimum Distance). Lower AMD means more similar routes. The comparison runs both directions (A→B and B→A) to detect if one route is a subset of another.

**Route Grouping** clusters multiple activities by similarity. Each activity starts in its own group. When two routes match, their groups merge using Union-Find (a fast algorithm for tracking connected components). The result: activities on the same route end up in the same group.

**Section Detection** finds frequently-traveled portions of tracks across many activities using spatial indexing (R-tree) and consensus algorithms.

## Usage

### Route matching

```rust
use tracematch::{GpsPoint, RouteSignature, MatchConfig, compare_routes};

let london = vec![
    GpsPoint::new(51.5074, -0.1278),
    GpsPoint::new(51.5080, -0.1290),
    GpsPoint::new(51.5090, -0.1300),
];
let nyc = vec![
    GpsPoint::new(40.7128, -74.0060),
    GpsPoint::new(40.7138, -74.0070),
    GpsPoint::new(40.7148, -74.0080),
];

let config = MatchConfig::default();
let sig1 = RouteSignature::from_points("london-1", &london, &config).unwrap();
let sig2 = RouteSignature::from_points("london-2", &london, &config).unwrap();
let sig3 = RouteSignature::from_points("nyc", &nyc, &config).unwrap();

compare_routes(&sig1, &sig2, &config);  // Some(100% match, same)
compare_routes(&sig1, &sig3, &config);  // None (different routes)
```

```sh
cargo run --example route_matching
```

```
london-1 vs london-2:
  100% match (same)
london-1 vs nyc:
  no match
```

### Route grouping

```rust
use tracematch::{GpsPoint, RouteSignature, MatchConfig, group_signatures};

// Two activities on the same route (London, ~1km)
let london: Vec<GpsPoint> = (0..10)
    .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
    .collect();

// One activity on a different route (NYC, ~1km)
let nyc: Vec<GpsPoint> = (0..10)
    .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
    .collect();

let config = MatchConfig::default();
let sigs = vec![
    RouteSignature::from_points("monday", &london, &config).unwrap(),
    RouteSignature::from_points("wednesday", &london, &config).unwrap(),
    RouteSignature::from_points("friday", &nyc, &config).unwrap(),
];

for group in group_signatures(&sigs, &config) {
    println!("{}: {:?}", group.group_id, group.activity_ids);
}
// Output:
// monday: ["monday", "wednesday"]
// friday: ["friday"]
```

```sh
cargo run --example route_grouping
```

### Configuration

All fields have sensible defaults. Adjust thresholds based on GPS accuracy and use case.

```rust
use tracematch::MatchConfig;

let config = MatchConfig {
    // Matching thresholds
    perfect_threshold: 15.0,       // AMD ≤ 15m → 100% match
    zero_threshold: 100.0,         // AMD ≥ 100m → 0% match
    min_match_percentage: 50.0,    // Below this, compare_routes returns None

    // Route filtering
    min_route_distance: 500.0,     // Ignore routes shorter than 500m
    max_distance_diff_ratio: 0.30, // Routes must be within 30% length of each other
    endpoint_threshold: 300.0,     // Start/end points must be within 300m

    // Resampling (normalizes point density before comparison)
    resample_spacing_meters: 50.0, // One point every 50m
    min_resample_points: 20,       // At least 20 points
    max_resample_points: 200,      // At most 200 points

    // Simplification (Douglas-Peucker)
    simplification_tolerance: 0.0001,
    max_simplified_points: 100,

    ..Default::default()
};
```

## References

**Implemented algorithms:**

- Beckmann, N., Kriegel, H.-P., Schneider, R., & Seeger, B. (1990). [The R\*-tree: An efficient and robust access method for points and rectangles](https://doi.org/10.1145/93597.98741). *Proceedings of the 1990 ACM SIGMOD International Conference on Management of Data*, 322–331.
  — Spatial indexing via [rstar](https://crates.io/crates/rstar) crate

- Tarjan, R. E. (1975). [Efficiency of a good but not linear set union algorithm](https://doi.org/10.1145/321879.321884). *Journal of the ACM*, 22(2), 215–225.
  — Route grouping with path compression and union by rank

- Douglas, D. H., & Peucker, T. K. (1973). [Algorithms for the reduction of the number of points required to represent a digitized line or its caricature](https://doi.org/10.3138/FM57-6770-U75U-7727). *Cartographica*, 10(2), 112–122.
  — Polyline simplification via [geo](https://crates.io/crates/geo) crate

- Kaufman, L., & Rousseeuw, P. J. (1987). Clustering by means of medoids. *Statistical Data Analysis Based on the L₁-Norm and Related Methods*, 405–416.
  — Representative route selection (medoid concept; not full PAM)

**Conceptual inspiration:**

- Lee, J.-G., Han, J., & Whang, K.-Y. (2007). [Trajectory clustering: A partition-and-group framework](https://doi.org/10.1145/1247480.1247546). *Proceedings of the 2007 ACM SIGMOD International Conference on Management of Data*, 593–604.
  — MDL principle for avoiding over-segmentation

- Xu, W., & Dong, S. (2022). [Application of artificial intelligence in an unsupervised algorithm for trajectory segmentation based on multiple motion features](https://doi.org/10.1155/2022/9540944). *Wireless Communications and Mobile Computing*, 2022, 9540944.
  — Two-phase segmentation-then-mergence pipeline

- Yang, J., Mariescu-Istodor, R., & Fränti, P. (2019). [Three rapid methods for averaging GPS segments](https://doi.org/10.3390/app9224899). *Applied Sciences*, 9(22), 4899.
  — Consensus polyline computation concepts

## License

Apache-2.0
