//! Instruction-counting benchmarks using iai-callgrind.
//!
//! These benchmarks count CPU instructions for deterministic CI results.
//! Run with: `cargo bench --bench iai_benchmarks`
//!
//! Requires valgrind to be installed.

use iai_callgrind::{LibraryBenchmarkConfig, library_benchmark, library_benchmark_group, main};
use std::hint::black_box;
use tracematch::{GpsPoint, MatchConfig, RouteSignature};

#[cfg(feature = "parallel")]
use tracematch::group_signatures_parallel;

#[cfg(not(feature = "parallel"))]
use tracematch::group_signatures;

// ============================================================================
// Test Data Generation (deterministic, no RNG)
// ============================================================================

/// Generate a deterministic route for benchmarking.
/// Uses fixed coordinates to ensure reproducible instruction counts.
fn generate_route(start_lat: f64, start_lng: f64, points: usize) -> Vec<GpsPoint> {
    (0..points)
        .map(|i| {
            let progress = i as f64 / points as f64;
            GpsPoint::new(start_lat + progress * 0.01, start_lng + progress * 0.01)
        })
        .collect()
}

/// Generate a slightly different route (for comparison benchmarks).
fn generate_similar_route(start_lat: f64, start_lng: f64, points: usize) -> Vec<GpsPoint> {
    (0..points)
        .map(|i| {
            let progress = i as f64 / points as f64;
            GpsPoint::new(
                start_lat + progress * 0.01 + 0.00001,
                start_lng + progress * 0.01 + 0.00001,
            )
        })
        .collect()
}

// ============================================================================
// Signature Creation Benchmarks
// ============================================================================

#[library_benchmark]
fn bench_signature_creation_100_points() -> Option<RouteSignature> {
    let route = generate_route(51.5, -0.1, 100);
    black_box(RouteSignature::from_points(
        "test",
        black_box(&route),
        &MatchConfig::default(),
    ))
}

#[library_benchmark]
fn bench_signature_creation_500_points() -> Option<RouteSignature> {
    let route = generate_route(51.5, -0.1, 500);
    black_box(RouteSignature::from_points(
        "test",
        black_box(&route),
        &MatchConfig::default(),
    ))
}

#[library_benchmark]
fn bench_signature_creation_1000_points() -> Option<RouteSignature> {
    let route = generate_route(51.5, -0.1, 1000);
    black_box(RouteSignature::from_points(
        "test",
        black_box(&route),
        &MatchConfig::default(),
    ))
}

// ============================================================================
// Route Comparison Benchmarks
// ============================================================================

#[library_benchmark]
fn bench_compare_identical_routes() -> Option<tracematch::MatchResult> {
    let route = generate_route(51.5, -0.1, 100);
    let sig1 = RouteSignature::from_points("a", &route, &MatchConfig::default()).unwrap();
    let sig2 = RouteSignature::from_points("b", &route, &MatchConfig::default()).unwrap();
    black_box(tracematch::compare_routes(
        black_box(&sig1),
        black_box(&sig2),
        &MatchConfig::default(),
    ))
}

#[library_benchmark]
fn bench_compare_similar_routes() -> Option<tracematch::MatchResult> {
    let route1 = generate_route(51.5, -0.1, 100);
    let route2 = generate_similar_route(51.5, -0.1, 100);
    let sig1 = RouteSignature::from_points("a", &route1, &MatchConfig::default()).unwrap();
    let sig2 = RouteSignature::from_points("b", &route2, &MatchConfig::default()).unwrap();
    black_box(tracematch::compare_routes(
        black_box(&sig1),
        black_box(&sig2),
        &MatchConfig::default(),
    ))
}

#[library_benchmark]
fn bench_compare_different_routes() -> Option<tracematch::MatchResult> {
    let route1 = generate_route(51.5, -0.1, 100);
    let route2 = generate_route(40.7, -74.0, 100); // NYC vs London
    let sig1 = RouteSignature::from_points("a", &route1, &MatchConfig::default()).unwrap();
    let sig2 = RouteSignature::from_points("b", &route2, &MatchConfig::default()).unwrap();
    black_box(tracematch::compare_routes(
        black_box(&sig1),
        black_box(&sig2),
        &MatchConfig::default(),
    ))
}

// ============================================================================
// Grouping Benchmarks
// ============================================================================

#[library_benchmark]
fn bench_group_10_identical_routes() -> Vec<tracematch::RouteGroup> {
    let route = generate_route(51.5, -0.1, 100);
    let signatures: Vec<RouteSignature> = (0..10)
        .filter_map(|i| {
            RouteSignature::from_points(&format!("route-{}", i), &route, &MatchConfig::default())
        })
        .collect();

    #[cfg(feature = "parallel")]
    {
        black_box(group_signatures_parallel(
            black_box(&signatures),
            &MatchConfig::default(),
        ))
    }

    #[cfg(not(feature = "parallel"))]
    {
        black_box(group_signatures(
            black_box(&signatures),
            &MatchConfig::default(),
        ))
    }
}

#[library_benchmark]
fn bench_group_10_different_routes() -> Vec<tracematch::RouteGroup> {
    let signatures: Vec<RouteSignature> = (0..10)
        .filter_map(|i| {
            let route = generate_route(51.5 + i as f64 * 0.5, -0.1 + i as f64 * 0.5, 100);
            RouteSignature::from_points(&format!("route-{}", i), &route, &MatchConfig::default())
        })
        .collect();

    #[cfg(feature = "parallel")]
    {
        black_box(group_signatures_parallel(
            black_box(&signatures),
            &MatchConfig::default(),
        ))
    }

    #[cfg(not(feature = "parallel"))]
    {
        black_box(group_signatures(
            black_box(&signatures),
            &MatchConfig::default(),
        ))
    }
}

// ============================================================================
// Benchmark Groups
// ============================================================================

library_benchmark_group!(
    name = signature_creation;
    benchmarks =
        bench_signature_creation_100_points,
        bench_signature_creation_500_points,
        bench_signature_creation_1000_points
);

library_benchmark_group!(
    name = route_comparison;
    benchmarks =
        bench_compare_identical_routes,
        bench_compare_similar_routes,
        bench_compare_different_routes
);

library_benchmark_group!(
    name = route_grouping;
    benchmarks =
        bench_group_10_identical_routes,
        bench_group_10_different_routes
);

main!(
    config = LibraryBenchmarkConfig::default();
    library_benchmark_groups =
        signature_creation,
        route_comparison,
        route_grouping
);
