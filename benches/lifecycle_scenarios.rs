//! Criterion benches aligned with the lifecycle scenarios A–E.
//!
//! Run with: `cargo bench --bench lifecycle_scenarios --features synthetic`
//!
//! These benches measure pure section-detection cost (no SQLite, no
//! apply_sections) at the corpus size of each user-flow checkpoint. The
//! complementary apply-tail cost is measured by the
//! `lifecycle_scenarios` integration test in the veloqrs crate.
//!
//! Each bench's name maps to the user-visible scenario in
//! `docs/perf/baselines-2026-04-19.md` so optimization tiers track against
//! the same labels users feel.

use criterion::{BenchmarkId, Criterion, SamplingMode, criterion_group, criterion_main};
use std::time::Duration;
use tracematch::scenarios::{LifecycleConfig, LifecycleCorpus};
use tracematch::{RouteGroup, SectionConfig, detect_sections_multiscale};

/// Build the same corpus the integration test uses, then expose tracks +
/// sport map for a given scenario checkpoint.
fn corpus_for_default() -> LifecycleCorpus {
    LifecycleCorpus::generate(&LifecycleConfig::default())
}

fn one_group_per_activity(
    tracks: &[(String, Vec<tracematch::GpsPoint>)],
    sport_types: &std::collections::HashMap<String, String>,
) -> Vec<RouteGroup> {
    tracks
        .iter()
        .enumerate()
        .map(|(i, (id, _))| RouteGroup {
            group_id: format!("group_{i}"),
            representative_id: id.clone(),
            activity_ids: vec![id.clone()],
            sport_type: sport_types
                .get(id)
                .cloned()
                .unwrap_or_else(|| "Ride".to_string()),
            bounds: None,
            custom_name: None,
            best_time: None,
            avg_time: None,
            best_pace: None,
            best_activity_id: None,
        })
        .collect()
}

fn checkpoint_tracks(
    corpus: &LifecycleCorpus,
    label: &str,
) -> (
    Vec<(String, Vec<tracematch::GpsPoint>)>,
    std::collections::HashMap<String, String>,
) {
    let activities: Vec<_> = match label {
        "A_cold_60" => corpus.through_a(),
        "B_expand_150" => corpus.through_b(),
        "C_add1_151" => corpus.through_c(),
        "D_add3_154" => corpus.through_d(),
        "E_year_550" => corpus.through_e(),
        other => panic!("unknown lifecycle checkpoint label: {other}"),
    };

    let tracks = activities
        .iter()
        .map(|a| (a.id.clone(), a.gps_points.clone()))
        .collect::<Vec<_>>();
    let sport = activities
        .iter()
        .map(|a| (a.id.clone(), a.sport_type.clone()))
        .collect::<std::collections::HashMap<_, _>>();
    (tracks, sport)
}

fn bench_lifecycle_full_detection(c: &mut Criterion) {
    let mut group = c.benchmark_group("lifecycle_full_detection");
    group.sampling_mode(SamplingMode::Flat);
    group.warm_up_time(Duration::from_secs(2));

    let corpus = corpus_for_default();

    // Criterion enforces sample_size >= 10. Larger corpora use longer
    // measurement windows so the per-iteration count stays sensible.
    for (label, sample_size, measurement_secs) in [
        ("A_cold_60", 10, 30u64),
        ("B_expand_150", 10, 60),
        ("C_add1_151", 10, 60),
        ("D_add3_154", 10, 60),
        ("E_year_550", 10, 180),
    ] {
        group.sample_size(sample_size);
        group.measurement_time(Duration::from_secs(measurement_secs));

        let (tracks, sport_types) = checkpoint_tracks(&corpus, label);
        let groups = one_group_per_activity(&tracks, &sport_types);
        let config = SectionConfig::default();

        group.bench_with_input(BenchmarkId::from_parameter(label), &label, |b, _| {
            b.iter(|| detect_sections_multiscale(&tracks, &sport_types, &groups, &config));
        });
    }

    group.finish();
}

criterion_group!(benches, bench_lifecycle_full_detection);
criterion_main!(benches);
