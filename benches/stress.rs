//! Stress benchmarks for section detection at scale.
//!
//! Run with: `cargo bench --bench stress --features synthetic`
//!
//! These benchmarks use synthetic data to find performance cliff edges
//! at scale (100-1000 activities, 10km-100km corridors).

use criterion::{BenchmarkId, Criterion, SamplingMode, criterion_group, criterion_main};
use std::time::Duration;
use tracematch::synthetic::SyntheticScenario;
use tracematch::{SectionConfig, detect_sections_multiscale, detect_sections_optimized};

// ============================================================================
// 1. Scaling Curve (Multiscale) — Find the N^2 cliff
// ============================================================================

fn bench_scaling_curve(c: &mut Criterion) {
    let mut group = c.benchmark_group("scaling_curve");
    group.sampling_mode(SamplingMode::Flat);
    group.warm_up_time(Duration::from_secs(5));

    for count in [50, 100, 250, 500, 1000] {
        // Reduce sample size and measurement time for large counts
        if count >= 500 {
            group.sample_size(2);
            group.measurement_time(Duration::from_secs(30));
        } else {
            group.sample_size(5);
            group.measurement_time(Duration::from_secs(60));
        }

        let scenario = SyntheticScenario::with_activity_count(count, 10_000.0, 0.8);
        let dataset = scenario.generate();
        let groups = dataset.route_groups();
        let config = SectionConfig::default();

        group.bench_with_input(BenchmarkId::new("activities", count), &count, |b, _| {
            b.iter(|| {
                detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config)
            });
        });
    }

    group.finish();
}

// ============================================================================
// 2. Scaling Curve (Optimized) — Comparison with multiscale
// ============================================================================

fn bench_scaling_curve_optimized(c: &mut Criterion) {
    let mut group = c.benchmark_group("scaling_curve_optimized");
    group.sampling_mode(SamplingMode::Flat);
    group.warm_up_time(Duration::from_secs(5));

    for count in [50, 100, 250, 500, 1000, 2000] {
        if count >= 2000 {
            group.sample_size(2);
            group.measurement_time(Duration::from_secs(120));
        } else if count >= 500 {
            group.sample_size(2);
            group.measurement_time(Duration::from_secs(30));
        } else {
            group.sample_size(5);
            group.measurement_time(Duration::from_secs(60));
        }

        let scenario = SyntheticScenario::with_activity_count(count, 10_000.0, 0.8);
        let dataset = scenario.generate();
        let config = SectionConfig::default();

        group.bench_with_input(BenchmarkId::new("activities", count), &count, |b, _| {
            b.iter(|| detect_sections_optimized(&dataset.tracks, &dataset.sport_types, &config));
        });
    }

    group.finish();
}

// ============================================================================
// 3. Route Length Impact — Does downsampling hold for long routes?
// ============================================================================

fn bench_route_length_impact(c: &mut Criterion) {
    let mut group = c.benchmark_group("route_length_impact");
    group.sample_size(5);
    group.sampling_mode(SamplingMode::Flat);
    group.measurement_time(Duration::from_secs(60));
    group.warm_up_time(Duration::from_secs(5));

    for length_km in [10, 50, 100] {
        let length_m = length_km as f64 * 1000.0;
        let scenario = SyntheticScenario::with_activity_count(50, length_m, 0.6);
        let dataset = scenario.generate();
        let groups = dataset.route_groups();
        let config = SectionConfig::default();

        group.bench_with_input(
            BenchmarkId::new("corridor_km", length_km),
            &length_km,
            |b, _| {
                b.iter(|| {
                    detect_sections_multiscale(
                        &dataset.tracks,
                        &dataset.sport_types,
                        &groups,
                        &config,
                    )
                });
            },
        );
    }

    group.finish();
}

// ============================================================================
// 4. Overlap Density — How does sparsity affect detection?
// ============================================================================

fn bench_overlap_density(c: &mut Criterion) {
    let mut group = c.benchmark_group("overlap_density");
    group.sample_size(5);
    group.sampling_mode(SamplingMode::Flat);
    group.measurement_time(Duration::from_secs(30));
    group.warm_up_time(Duration::from_secs(3));

    for overlap_pct in [10, 30, 50, 70, 90] {
        let overlap_frac = overlap_pct as f64 / 100.0;
        let scenario = SyntheticScenario::with_activity_count(100, 10_000.0, overlap_frac);
        let dataset = scenario.generate();
        let groups = dataset.route_groups();
        let config = SectionConfig::default();

        group.bench_with_input(
            BenchmarkId::new("overlap_pct", overlap_pct),
            &overlap_pct,
            |b, _| {
                b.iter(|| {
                    detect_sections_multiscale(
                        &dataset.tracks,
                        &dataset.sport_types,
                        &groups,
                        &config,
                    )
                });
            },
        );
    }

    group.finish();
}

// ============================================================================
// 5. No Overlap Worst Case — Pure overhead measurement
// ============================================================================

fn bench_no_overlap_worst_case(c: &mut Criterion) {
    let mut group = c.benchmark_group("no_overlap_worst_case");
    group.sampling_mode(SamplingMode::Flat);
    group.warm_up_time(Duration::from_secs(5));

    for count in [50, 100, 250, 500, 1000, 2000] {
        if count >= 2000 {
            group.sample_size(2);
            group.measurement_time(Duration::from_secs(120));
        } else if count >= 500 {
            group.sample_size(2);
            group.measurement_time(Duration::from_secs(30));
        } else {
            group.sample_size(5);
            group.measurement_time(Duration::from_secs(60));
        }

        let scenario = SyntheticScenario::with_no_overlap(count);
        let dataset = scenario.generate();
        let groups = dataset.route_groups();
        let config = SectionConfig::default();

        group.bench_with_input(BenchmarkId::new("activities", count), &count, |b, _| {
            b.iter(|| {
                detect_sections_multiscale(&dataset.tracks, &dataset.sport_types, &groups, &config)
            });
        });
    }

    group.finish();
}

// ============================================================================
// 6. Component Breakdown — Profile each stage
// ============================================================================

fn bench_component_breakdown(c: &mut Criterion) {
    let mut group = c.benchmark_group("component_breakdown");
    group.sample_size(5);
    group.sampling_mode(SamplingMode::Flat);
    group.measurement_time(Duration::from_secs(30));
    group.warm_up_time(Duration::from_secs(3));

    for count in [100, 500] {
        let scenario = SyntheticScenario::with_activity_count(count, 10_000.0, 0.8);
        let dataset = scenario.generate();
        let groups = dataset.route_groups();

        // Benchmark R-tree construction
        {
            use tracematch::sections::build_rtree;
            let tracks_for_rtree: Vec<(&str, &[GpsPoint])> = dataset
                .tracks
                .iter()
                .map(|(id, pts)| (id.as_str(), pts.as_slice()))
                .collect();

            group.bench_with_input(BenchmarkId::new("rtree_build", count), &count, |b, _| {
                b.iter(|| {
                    for (_, track) in &tracks_for_rtree {
                        build_rtree(track);
                    }
                });
            });
        }

        // Benchmark full pipeline (for comparison with component times)
        {
            let config = SectionConfig::default();
            group.bench_with_input(BenchmarkId::new("full_pipeline", count), &count, |b, _| {
                b.iter(|| {
                    detect_sections_multiscale(
                        &dataset.tracks,
                        &dataset.sport_types,
                        &groups,
                        &config,
                    )
                });
            });
        }
    }

    group.finish();
}

// ============================================================================
// Main
// ============================================================================

use tracematch::GpsPoint;

criterion_group!(
    benches,
    bench_scaling_curve,
    bench_scaling_curve_optimized,
    bench_route_length_impact,
    bench_overlap_density,
    bench_no_overlap_worst_case,
    bench_component_breakdown,
);
criterion_main!(benches);
