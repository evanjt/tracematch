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

## License

Apache-2.0
