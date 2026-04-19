//! Benchmarks for route matching and grouping on synthetic GPS data.
//!
//! Run with: `cargo bench --bench route_matching --features synthetic`
//!
//! Synthetic tracks are generated via [`SyntheticScenario`] so these benches
//! are self-contained and do not depend on external fixture files.

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use tracematch::synthetic::{CorridorConfig, CorridorPattern, SyntheticScenario};
use tracematch::{GpsPoint, MatchConfig, RouteSignature, compare_routes, group_signatures};

/// Generate a deterministic batch of synthetic tracks for the matching benches.
///
/// Produces `count` activities that share a 10km corridor with 80% overlap so
/// that most routes are genuinely similar (exercises the matching path rather
/// than trivially rejecting on bounds).
fn synthetic_routes(count: usize) -> Vec<(String, Vec<GpsPoint>)> {
    let scenario = SyntheticScenario {
        origin: GpsPoint::new(47.37, 8.55),
        activity_count: count,
        corridors: vec![CorridorConfig {
            length_meters: 10_000.0,
            overlap_fraction: 0.8,
            pattern: CorridorPattern::Winding,
            approach_length: 500.0,
        }],
        gps_noise_sigma_meters: 3.0,
        seed: 42,
    };
    scenario.generate().tracks
}

fn bench_signature_creation(c: &mut Criterion) {
    let routes = synthetic_routes(5);
    let config = MatchConfig::default();

    let mut group = c.benchmark_group("signature_creation");

    // Benchmark across different track sizes. Synthetic tracks already vary in
    // length because the random approach/departure portions differ per activity.
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
    let routes = synthetic_routes(6);
    let config = MatchConfig::default();

    // Pre-compute signatures once; comparison is what we want to measure.
    let signatures: Vec<_> = routes
        .iter()
        .filter_map(|(id, pts)| {
            RouteSignature::from_points(id, pts, &config).map(|s| (id.clone(), s))
        })
        .collect();

    let mut group = c.benchmark_group("route_comparison");

    if let Some((_, sig1)) = signatures.first() {
        for (id, sig2) in signatures.iter().skip(1).take(5) {
            group.bench_with_input(BenchmarkId::new("compare_routes", id), sig2, |b, s2| {
                b.iter(|| compare_routes(sig1, s2, &config));
            });
        }
    }

    group.finish();
}

fn bench_route_grouping(c: &mut Criterion) {
    // Need enough routes to support the largest grouping size (20).
    let routes = synthetic_routes(20);
    let config = MatchConfig::default();

    // Pre-compute all signatures so grouping is isolated from signature building.
    let signatures: Vec<_> = routes
        .iter()
        .filter_map(|(id, pts)| RouteSignature::from_points(id, pts, &config))
        .collect();

    let mut group = c.benchmark_group("route_grouping");

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
