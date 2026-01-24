# tracematch

High-performance GPS route matching library.

## Features

- **Route Matching** — AMD-based polyline similarity with bidirectional detection
- **Route Grouping** — Cluster activities by route using Union-Find
- **Section Detection** — Multi-scale detection of frequently-traveled sections
- **Spatial Indexing** — R-tree for O(log n) viewport queries
- **Parallel Processing** — Optional rayon-based parallelism

## Installation

```sh
cargo add tracematch
```

## Usage

### Route matching

```rust
use tracematch::{GpsPoint, RouteSignature, MatchConfig, compare_routes};

let route = vec![
    GpsPoint::new(51.5074, -0.1278),
    GpsPoint::new(51.5080, -0.1290),
    GpsPoint::new(51.5090, -0.1300),
];

let config = MatchConfig::default();
let sig1 = RouteSignature::from_points("route-1", &route, &config).unwrap();
let sig2 = RouteSignature::from_points("route-2", &route, &config).unwrap();

if let Some(result) = compare_routes(&sig1, &sig2, &config) {
    println!("{}% match ({})", result.match_percentage, result.direction);
}
// Output: 100% match (same)
```

### Route grouping

```rust
use tracematch::{GpsPoint, RouteSignature, MatchConfig, group_signatures};

let config = MatchConfig::default();
let sigs = vec![
    RouteSignature::from_points("monday", &london_route, &config).unwrap(),
    RouteSignature::from_points("wednesday", &london_route, &config).unwrap(),
    RouteSignature::from_points("friday", &nyc_route, &config).unwrap(),
];

for group in group_signatures(&sigs, &config) {
    println!("{}: {:?}", group.group_id, group.activity_ids);
}
// Output:
// route_0: ["monday", "wednesday"]
// route_1: ["friday"]
```

## References

**Section detection:**

- [TRACLUS](https://hanj.cs.illinois.edu/pdf/sigmod07_jglee.pdf) — MDL-based trajectory partitioning (Lee et al., SIGMOD 2007)
- [TS-MF](https://www.hindawi.com/journals/wcmc/2022/9540944/) — Trajectory segmentation and mergence (Xu et al., 2022)
- [Graph-based clustering](https://arxiv.org/abs/1310.5249) — Network-constrained clustering (Hwang et al., 2013)
- [GPS segment averaging](https://mdpi.com/2076-3417/9/22/4899/htm) — Consensus polyline computation (MDPI, 2019)

**Foundational algorithms:**

- [R\*-tree](https://dl.acm.org/doi/10.1145/93597.98741) — Spatial indexing (Beckmann et al., SIGMOD 1990)
- [Union-Find](https://dl.acm.org/doi/10.1145/321879.321884) — Disjoint set clustering (Tarjan, 1975)
- [Douglas-Peucker](https://doi.org/10.3138/FM57-6770-U75U-7727) — Line simplification (1973)
- [Medoid selection](https://en.wikipedia.org/wiki/K-medoids) — Representative route selection (Kaufman & Rousseeuw, 1987)
- [Haversine formula](https://en.wikipedia.org/wiki/Haversine_formula) — Great-circle distance

## License

Apache-2.0
