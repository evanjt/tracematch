//! Criterion benchmarks for section detection performance.
//!
//! Run with: `cargo bench --bench section_detection`
//!
//! These benchmarks measure the real-world performance of section detection
//! with varying numbers of activities.

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use std::collections::HashMap;
use std::fs;
use std::path::Path;
use tracematch::{GpsPoint, RouteGroup, SectionConfig, detect_sections_multiscale};

/// Load GPS points from a trimmed activity JSON file.
fn load_trimmed_activity(path: &Path) -> Option<Vec<GpsPoint>> {
    let content = fs::read_to_string(path).ok()?;
    let coords: Vec<[f64; 2]> = serde_json::from_str(&content).ok()?;
    Some(
        coords
            .into_iter()
            .map(|[lat, lng]| GpsPoint::new(lat, lng))
            .collect(),
    )
}

/// Load multiple activities from the fixtures directory.
fn load_test_activities(count: usize) -> Vec<(String, Vec<GpsPoint>)> {
    let fixtures_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("raw_traces");

    let mut activities = Vec::new();

    // Load the 5 trimmed activities
    for i in 1..=5 {
        let path = fixtures_dir.join(format!("activity_{}_trimmed.json", i));
        if let Some(points) = load_trimmed_activity(&path) {
            activities.push((format!("activity_{}", i), points));
        }
    }

    // If we need more activities, duplicate and slightly offset the existing ones
    let base_count = activities.len();
    let mut idx = 0;
    while activities.len() < count && base_count > 0 {
        let (base_id, base_points) = &activities[idx % base_count];
        // Slightly offset the coordinates to simulate different runs of similar routes
        let offset = ((activities.len() - base_count + 1) as f64) * 0.00001;
        let offset_points: Vec<GpsPoint> = base_points
            .iter()
            .map(|p| GpsPoint::new(p.latitude + offset, p.longitude + offset))
            .collect();
        activities.push((
            format!("{}_dup_{}", base_id, activities.len()),
            offset_points,
        ));
        idx += 1;
    }

    activities
}

/// Prepare data for section detection benchmark.
fn prepare_benchmark_data(
    activity_count: usize,
) -> (
    Vec<(String, Vec<GpsPoint>)>,
    HashMap<String, String>,
    Vec<RouteGroup>,
) {
    let activities = load_test_activities(activity_count);

    // All activities are "Run" type
    let sport_types: HashMap<String, String> = activities
        .iter()
        .map(|(id, _)| (id.clone(), "Run".to_string()))
        .collect();

    // Create simple route groups (each activity is its own group)
    let route_groups: Vec<RouteGroup> = activities
        .iter()
        .enumerate()
        .map(|(i, (id, _))| RouteGroup {
            group_id: format!("group_{}", i),
            representative_id: id.clone(),
            activity_ids: vec![id.clone()],
            sport_type: "Run".to_string(),
            bounds: None,
            custom_name: None,
            best_time: None,
            avg_time: None,
            best_pace: None,
            best_activity_id: None,
        })
        .collect();

    (activities, sport_types, route_groups)
}

fn bench_section_detection(c: &mut Criterion) {
    let mut group = c.benchmark_group("section_detection");

    // Configure for longer benchmarks (section detection is slow)
    group.sample_size(10);
    group.measurement_time(std::time::Duration::from_secs(30));

    for activity_count in [5, 20, 50] {
        let (activities, sport_types, route_groups) = prepare_benchmark_data(activity_count);
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
    // where many sections are detected and need to be merged/deduplicated
    let mut group = c.benchmark_group("postprocessing");
    group.sample_size(10);
    group.measurement_time(std::time::Duration::from_secs(20));

    // Use more activities to generate more sections for post-processing
    let (activities, sport_types, route_groups) = prepare_benchmark_data(50);
    let config = SectionConfig::default();

    group.bench_function("50_activities_full_pipeline", |b| {
        b.iter(|| detect_sections_multiscale(&activities, &sport_types, &route_groups, &config));
    });

    group.finish();
}

criterion_group!(benches, bench_section_detection, bench_postprocessing_heavy);
criterion_main!(benches);
