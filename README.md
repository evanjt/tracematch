# tracematch

High-performance GPS route matching using Fréchet distance and spatial indexing.

## Features

- **Fréchet Distance Matching** - Accurate polyline similarity
- **Bidirectional Detection** - Detects forward and reverse route matches
- **R-tree Spatial Indexing** - O(log n) pre-filtering for batch operations
- **Parallel Processing** - Rayon-based parallel grouping
- **Mobile FFI** - UniFFI bindings for iOS and Android

## Installation

```toml
[dependencies]
tracematch = "0.0.1"
```

## Quick Start

```rust
use tracematch::{GpsPoint, RouteSignature, MatchConfig, compare_routes};

let route1 = vec![
    GpsPoint::new(51.5074, -0.1278),
    GpsPoint::new(51.5080, -0.1290),
    GpsPoint::new(51.5090, -0.1300),
];

let config = MatchConfig::default();
let sig1 = RouteSignature::from_points("route-1", &route1, &config).unwrap();
let sig2 = RouteSignature::from_points("route-2", &route1, &config).unwrap();

if let Some(result) = compare_routes(&sig1, &sig2, &config) {
    println!("Match: {}%", result.match_percentage);
}
```

## License

Apache-2.0
