//! Performance benchmarks for tracematch library.
//!
//! Run with: `cargo bench`
//!
//! These benchmarks use synthetic GPS data to measure performance under
//! realistic conditions for mobile fitness apps.

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rand::Rng;
use tracematch::{GpsPoint, MatchConfig, RouteSignature};

#[cfg(feature = "parallel")]
use tracematch::{group_incremental, group_signatures_parallel};

#[cfg(not(feature = "parallel"))]
use tracematch::group_signatures;

// ============================================================================
// Synthetic Route Generation
// ============================================================================

/// Generate a realistic GPS route with noise.
///
/// # Arguments
/// * `start_lat` - Starting latitude
/// * `start_lng` - Starting longitude
/// * `distance_km` - Total route distance in kilometers
/// * `points_per_km` - Number of GPS points per kilometer
/// * `noise_meters` - GPS noise amplitude in meters
fn generate_synthetic_route(
    start_lat: f64,
    start_lng: f64,
    distance_km: f64,
    points_per_km: usize,
    noise_meters: f64,
) -> Vec<GpsPoint> {
    let mut rng = rand::thread_rng();
    let total_points = (distance_km * points_per_km as f64) as usize;
    let bearing: f64 = rng.gen_range(0.0..360.0_f64).to_radians();

    (0..total_points)
        .map(|i| {
            let progress = i as f64 / total_points.max(1) as f64;
            let distance_m = progress * distance_km * 1000.0;

            // Move along bearing with some wobble
            let noise_deg = noise_meters / 111_000.0;
            let wobble_lat = rng.gen_range(-noise_deg..noise_deg);
            let wobble_lng = rng.gen_range(-noise_deg..noise_deg);

            let lat = start_lat + (distance_m / 111_000.0) * bearing.cos() + wobble_lat;
            let lng = start_lng
                + (distance_m / (111_000.0 * start_lat.to_radians().cos().max(0.1))) * bearing.sin()
                + wobble_lng;

            GpsPoint::new(lat, lng)
        })
        .collect()
}

/// Generate N routes with variation (some matching, some different).
///
/// # Arguments
/// * `count` - Total number of routes to generate
/// * `similar_percent` - Percentage (0.0-1.0) of routes that should be similar
fn generate_route_set(count: usize, similar_percent: f64) -> Vec<(String, Vec<GpsPoint>)> {
    let mut rng = rand::thread_rng();
    let similar_count = (count as f64 * similar_percent) as usize;

    // Generate "template" routes for similar ones
    let template_count = ((similar_count as f64).sqrt() as usize).max(1);
    let templates: Vec<Vec<GpsPoint>> = (0..template_count)
        .map(|_| {
            let lat = rng.gen_range(45.0..55.0);
            let lng = rng.gen_range(-5.0..15.0);
            let dist = rng.gen_range(5.0..50.0);
            generate_synthetic_route(lat, lng, dist, 100, 0.0) // Base template without noise
        })
        .collect();

    let mut routes = Vec::with_capacity(count);

    // Generate similar routes (same template with noise)
    for i in 0..similar_count {
        let template = &templates[i % template_count];
        let noisy = add_gps_noise(template, 10.0); // 10m noise
        routes.push((format!("similar-{}", i), noisy));
    }

    // Generate unique routes
    for i in similar_count..count {
        let lat = rng.gen_range(45.0..55.0);
        let lng = rng.gen_range(-5.0..15.0);
        let dist = rng.gen_range(5.0..50.0);
        routes.push((
            format!("unique-{}", i),
            generate_synthetic_route(lat, lng, dist, 100, 5.0),
        ));
    }

    routes
}

/// Add GPS noise to a route.
fn add_gps_noise(points: &[GpsPoint], noise_meters: f64) -> Vec<GpsPoint> {
    let mut rng = rand::thread_rng();
    let noise_deg = noise_meters / 111_000.0;

    points
        .iter()
        .map(|p| {
            GpsPoint::new(
                p.latitude + rng.gen_range(-noise_deg..noise_deg),
                p.longitude + rng.gen_range(-noise_deg..noise_deg),
            )
        })
        .collect()
}

// ============================================================================
// Benchmarks
// ============================================================================

/// Benchmark single route comparison.
fn bench_single_comparison(c: &mut Criterion) {
    let route1 = generate_synthetic_route(51.5, -0.1, 10.0, 100, 0.0);
    let route2 = add_gps_noise(&route1, 10.0);

    let sig1 = RouteSignature::from_points("a", &route1, &MatchConfig::default()).unwrap();
    let sig2 = RouteSignature::from_points("b", &route2, &MatchConfig::default()).unwrap();

    c.bench_function("compare_two_10km_routes", |b| {
        b.iter(|| {
            tracematch::compare_routes(black_box(&sig1), black_box(&sig2), &MatchConfig::default())
        })
    });
}

/// Benchmark signature creation for different route lengths.
fn bench_signature_creation(c: &mut Criterion) {
    let mut group = c.benchmark_group("signature_creation");

    for km in [5, 20, 50, 100].iter() {
        let route = generate_synthetic_route(51.5, -0.1, *km as f64, 100, 5.0);

        group.bench_with_input(
            BenchmarkId::new("route_km", format!("{}", km)),
            &route,
            |b, r| {
                b.iter(|| {
                    RouteSignature::from_points("test", black_box(r), &MatchConfig::default())
                })
            },
        );
    }

    group.finish();
}

/// Benchmark grouping scaling with different numbers of routes.
fn bench_grouping_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("grouping_scaling");
    group.sample_size(10); // Fewer samples for slow operations

    // Mobile target: <500ms for 100 routes, <2s for 500 routes
    for count in [50, 100, 200].iter() {
        let routes = generate_route_set(*count, 0.5); // 50% similar
        let signatures: Vec<RouteSignature> = routes
            .iter()
            .filter_map(|(id, pts)| RouteSignature::from_points(id, pts, &MatchConfig::default()))
            .collect();

        #[cfg(feature = "parallel")]
        group.bench_with_input(
            BenchmarkId::new("group_parallel", count),
            &signatures,
            |b, sigs| {
                b.iter(|| group_signatures_parallel(black_box(sigs), &MatchConfig::default()))
            },
        );

        #[cfg(not(feature = "parallel"))]
        group.bench_with_input(
            BenchmarkId::new("group_sequential", count),
            &signatures,
            |b, sigs| b.iter(|| group_signatures(black_box(sigs), &MatchConfig::default())),
        );
    }

    group.finish();
}

/// Benchmark incremental grouping (adding new activities to existing groups).
#[cfg(feature = "parallel")]
fn bench_incremental_grouping(c: &mut Criterion) {
    let mut group = c.benchmark_group("incremental_grouping");
    group.sample_size(10);

    // Simulate adding 1, 10, 50 new activities to existing 200
    let all_routes = generate_route_set(250, 0.5);
    let existing_routes: Vec<_> = all_routes[..200].to_vec();
    let existing_sigs: Vec<_> = existing_routes
        .iter()
        .filter_map(|(id, pts)| RouteSignature::from_points(id, pts, &MatchConfig::default()))
        .collect();
    let existing_groups = group_signatures_parallel(&existing_sigs, &MatchConfig::default());

    for add_count in [1, 10, 50].iter() {
        let new_routes: Vec<_> = all_routes[200..(200 + add_count)].to_vec();
        let new_sigs: Vec<_> = new_routes
            .iter()
            .filter_map(|(id, pts)| RouteSignature::from_points(id, pts, &MatchConfig::default()))
            .collect();

        group.bench_with_input(
            BenchmarkId::new("add_to_200", add_count),
            &(new_sigs.clone(), existing_groups.clone(), existing_sigs.clone()),
            |b, (new, groups, existing)| {
                b.iter(|| {
                    group_incremental(
                        black_box(new),
                        black_box(groups),
                        black_box(existing),
                        &MatchConfig::default(),
                    )
                })
            },
        );
    }

    group.finish();
}

/// Benchmark for routes with varying point densities.
fn bench_point_density(c: &mut Criterion) {
    let mut group = c.benchmark_group("point_density");

    // Same 10km route with different point densities
    for points_per_km in [10, 50, 100, 500].iter() {
        let route = generate_synthetic_route(51.5, -0.1, 10.0, *points_per_km, 5.0);

        group.bench_with_input(
            BenchmarkId::new("signature_10km", format!("{}_pts_per_km", points_per_km)),
            &route,
            |b, r| {
                b.iter(|| {
                    RouteSignature::from_points("test", black_box(r), &MatchConfig::default())
                })
            },
        );
    }

    group.finish();
}

/// Benchmark memory-intensive operations.
fn bench_batch_processing(c: &mut Criterion) {
    let mut group = c.benchmark_group("batch_processing");
    group.sample_size(10);

    // Test batch signature creation
    let routes: Vec<Vec<GpsPoint>> = (0..100)
        .map(|i| generate_synthetic_route(51.5 + i as f64 * 0.01, -0.1, 5.0, 100, 5.0))
        .collect();

    group.bench_function("create_100_signatures", |b| {
        b.iter(|| {
            routes
                .iter()
                .enumerate()
                .filter_map(|(i, r)| {
                    RouteSignature::from_points(&format!("route-{}", i), r, &MatchConfig::default())
                })
                .collect::<Vec<_>>()
        })
    });

    group.finish();
}

/// Benchmark worst-case scenarios.
fn bench_worst_cases(c: &mut Criterion) {
    let mut group = c.benchmark_group("worst_cases");

    // Very long route (100km)
    let long_route = generate_synthetic_route(51.5, -0.1, 100.0, 100, 5.0);
    group.bench_function("signature_100km_route", |b| {
        b.iter(|| {
            RouteSignature::from_points("long", black_box(&long_route), &MatchConfig::default())
        })
    });

    // Very dense route (10k points for 10km)
    let dense_route = generate_synthetic_route(51.5, -0.1, 10.0, 1000, 5.0);
    group.bench_function("signature_dense_10k_points", |b| {
        b.iter(|| {
            RouteSignature::from_points("dense", black_box(&dense_route), &MatchConfig::default())
        })
    });

    // Many nearly-identical routes (worst case for grouping)
    let template = generate_synthetic_route(51.5, -0.1, 5.0, 100, 0.0);
    let sigs: Vec<RouteSignature> = (0..50)
        .filter_map(|i| {
            let noisy = add_gps_noise(&template, 5.0);
            RouteSignature::from_points(&format!("clone-{}", i), &noisy, &MatchConfig::default())
        })
        .collect();

    #[cfg(feature = "parallel")]
    group.bench_function("group_50_identical_routes", |b| {
        b.iter(|| group_signatures_parallel(black_box(&sigs), &MatchConfig::default()))
    });

    #[cfg(not(feature = "parallel"))]
    group.bench_function("group_50_identical_routes", |b| {
        b.iter(|| group_signatures(black_box(&sigs), &MatchConfig::default()))
    });

    group.finish();
}

// ============================================================================
// Criterion Configuration
// ============================================================================

#[cfg(feature = "parallel")]
criterion_group!(
    benches,
    bench_single_comparison,
    bench_signature_creation,
    bench_grouping_scaling,
    bench_incremental_grouping,
    bench_point_density,
    bench_batch_processing,
    bench_worst_cases,
);

#[cfg(not(feature = "parallel"))]
criterion_group!(
    benches,
    bench_single_comparison,
    bench_signature_creation,
    bench_grouping_scaling,
    bench_point_density,
    bench_batch_processing,
    bench_worst_cases,
);

criterion_main!(benches);
