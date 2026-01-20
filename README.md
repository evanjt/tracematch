# tracematch

High-performance GPS route matching library for fitness applications.

## Features

- **AMD Route Matching** - Average Minimum Distance for robust polyline similarity
- **Bidirectional Detection** - Detects forward and reverse route matches
- **R-tree Spatial Indexing** - O(log n) pre-filtering for batch operations
- **Section Detection** - Multi-scale detection of frequently-traveled sections
- **Parallel Processing** - Rayon-based parallel grouping
- **Mobile FFI** - UniFFI bindings for iOS and Android

## Installation

```toml
[dependencies]
tracematch = "0.1"
```

## Quick Start

```rust
use tracematch::{GpsPoint, RouteSignature, MatchConfig, compare_routes};

let route = vec![
    GpsPoint::new(51.5074, -0.1278),
    GpsPoint::new(51.5080, -0.1290),
    GpsPoint::new(51.5090, -0.1300),
];

let config = MatchConfig::default();
let sig1 = RouteSignature::from_points("route-1", &route, &config);
let sig2 = RouteSignature::from_points("route-2", &route, &config);

if let (Some(s1), Some(s2)) = (sig1, sig2) {
    if let Some(result) = compare_routes(&s1, &s2, &config) {
        println!("Match: {}% ({})", result.match_percentage, result.direction);
    }
}
```

## References

Section detection:
- **TRACLUS** (Lee, Han, Whang, SIGMOD 2007) - MDL-based trajectory partitioning ([paper](https://hanj.cs.illinois.edu/pdf/sigmod07_jglee.pdf))
- **TS-MF** (Xu et al., 2022) - Trajectory segmentation and mergence ([paper](https://www.hindawi.com/journals/wcmc/2022/9540944/))
- **Graph-based trajectory clustering** (Hwang et al., 2013) - Network-constrained clustering ([paper](https://arxiv.org/abs/1310.5249))
- **GPS segment averaging** (MDPI 2019) - Consensus polyline computation ([paper](https://mdpi.com/2076-3417/9/22/4899/htm))

Foundational algorithms:
- **R\*-tree** (Beckmann et al., SIGMOD 1990) - Spatial indexing ([paper](https://dl.acm.org/doi/10.1145/93597.98741))
- **Union-Find** (Tarjan, 1975) - Disjoint set clustering ([paper](https://dl.acm.org/doi/10.1145/321879.321884))
- **Douglas-Peucker** (Douglas & Peucker, 1973) - Line simplification ([paper](https://doi.org/10.3138/FM57-6770-U75U-7727))
- **Medoid/k-medoids** (Kaufman & Rousseeuw, 1987) - Representative selection
- **Haversine formula** - Great-circle distance ([Wikipedia](https://en.wikipedia.org/wiki/Haversine_formula))

## License

Apache-2.0
