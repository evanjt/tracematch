//! Benchmarks for route matching and grouping with real GPS data.
//!
//! GPS trace data (c) 2026 Evan Thomas. All rights reserved.
//! See tests/fixtures/demo/LICENSE for terms.
//!
//! Run with: `cargo bench --bench route_matching`

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use serde::Deserialize;
use std::fs;
use tracematch::{GpsPoint, MatchConfig, RouteSignature, compare_routes, group_signatures};

#[derive(Deserialize)]
struct DemoRoute {
    id: String,
    coordinates: Vec<[f64; 2]>,
}

fn load_demo_routes() -> Vec<(String, Vec<GpsPoint>)> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/demo/realRoutes.json");
    let content = fs::read_to_string(path).expect("Failed to load demo routes");
    let routes: Vec<DemoRoute> = serde_json::from_str(&content).expect("Failed to parse JSON");

    routes
        .into_iter()
        .map(|r| {
            let points = r.coordinates
                .into_iter()
                .map(|[lat, lng]| GpsPoint::new(lat, lng))
                .collect();
            (r.id, points)
        })
        .collect()
}

fn bench_signature_creation(c: &mut Criterion) {
    let routes = load_demo_routes();
    let config = MatchConfig::default();

    let mut group = c.benchmark_group("signature_creation");

    // Benchmark different route sizes
    for (id, points) in routes.iter().take(5) {
        group.bench_with_input(
            BenchmarkId::new("from_points", format!("{}_{}pts", id, points.len())),
            points,
            |b, pts| {
                b.iter(|| RouteSignature::from_points("test", pts, &config));
            },
        );
    }

    group.finish();
}

fn bench_route_comparison(c: &mut Criterion) {
    let routes = load_demo_routes();
    let config = MatchConfig::default();

    // Pre-compute signatures
    let signatures: Vec<_> = routes
        .iter()
        .filter_map(|(id, pts)| RouteSignature::from_points(id, pts, &config).map(|s| (id.clone(), s)))
        .collect();

    let mut group = c.benchmark_group("route_comparison");

    // Compare first signature against others
    if let Some((_, sig1)) = signatures.first() {
        for (id, sig2) in signatures.iter().skip(1).take(5) {
            group.bench_with_input(
                BenchmarkId::new("compare_routes", id),
                sig2,
                |b, s2| {
                    b.iter(|| compare_routes(sig1, s2, &config));
                },
            );
        }
    }

    group.finish();
}

fn bench_route_grouping(c: &mut Criterion) {
    let routes = load_demo_routes();
    let config = MatchConfig::default();

    // Pre-compute all signatures
    let signatures: Vec<_> = routes
        .iter()
        .filter_map(|(id, pts)| RouteSignature::from_points(id, pts, &config))
        .collect();

    let mut group = c.benchmark_group("route_grouping");

    // Group subsets of increasing size
    for count in [5, 10, 15, 20] {
        let subset: Vec<_> = signatures.iter().take(count).cloned().collect();
        group.bench_with_input(
            BenchmarkId::new("group_signatures", count),
            &subset,
            |b, sigs| {
                b.iter(|| group_signatures(sigs, &config));
            },
        );
    }

    group.finish();
}

criterion_group!(
    benches,
    bench_signature_creation,
    bench_route_comparison,
    bench_route_grouping
);
criterion_main!(benches);
