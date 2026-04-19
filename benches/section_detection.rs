//! Criterion benchmarks for section detection performance.
//!
//! Run with: `cargo bench --bench section_detection --features synthetic`
//!
//! These benchmarks measure the real-world performance of section detection
//! with varying numbers of activities. Synthetic data is generated via
//! [`SyntheticScenario`] so there are no fixture files to keep in sync.

use criterion::{BenchmarkId, Criterion, SamplingMode, criterion_group, criterion_main};
use std::time::Duration;
use tracematch::synthetic::SyntheticScenario;
use tracematch::{SectionConfig, detect_sections_multiscale};

/// Build a deterministic dataset of the given size with a single shared corridor.
///
/// Matches the shape used by the legacy fixture-based benches: ~10km corridor,
/// 80% overlap, so that roughly four out of five activities share the same route
/// and produce meaningful section detection work.
fn make_dataset(
    activity_count: usize,
) -> (
    Vec<(String, Vec<tracematch::GpsPoint>)>,
    std::collections::HashMap<String, String>,
    Vec<tracematch::RouteGroup>,
) {
    let scenario = SyntheticScenario::with_activity_count(activity_count, 10_000.0, 0.8);
    let dataset = scenario.generate();
    let groups = dataset.route_groups();
    (dataset.tracks, dataset.sport_types, groups)
}

fn bench_section_detection(c: &mut Criterion) {
    let mut group = c.benchmark_group("section_detection");

    // Section detection at these sizes takes seconds, not microseconds —
    // use Flat sampling and a generous measurement window.
    group.sampling_mode(SamplingMode::Flat);
    group.sample_size(10);
    group.measurement_time(Duration::from_secs(30));
    group.warm_up_time(Duration::from_secs(3));

    for activity_count in [5usize, 20, 50] {
        let (activities, sport_types, route_groups) = make_dataset(activity_count);
        let config = SectionConfig::default();

        group.bench_with_input(
            BenchmarkId::new("multiscale", activity_count),
            &activity_count,
            |b, _| {
                b.iter(|| {
                    detect_sections_multiscale(&activities, &sport_types, &route_groups, &config)
                });
            },
        );
    }

    group.finish();
}

fn bench_postprocessing_heavy(c: &mut Criterion) {
    // This benchmark focuses on the post-processing heavy case
    // where many sections are detected and need to be merged/deduplicated.
    let mut group = c.benchmark_group("postprocessing");
    group.sampling_mode(SamplingMode::Flat);
    group.sample_size(10);
    group.measurement_time(Duration::from_secs(60));
    group.warm_up_time(Duration::from_secs(3));

    let (activities, sport_types, route_groups) = make_dataset(50);
    let config = SectionConfig::default();

    group.bench_function("50_activities_full_pipeline", |b| {
        b.iter(|| detect_sections_multiscale(&activities, &sport_types, &route_groups, &config));
    });

    group.finish();
}

criterion_group!(benches, bench_section_detection, bench_postprocessing_heavy);
criterion_main!(benches);
