//! Targeted benchmark for `group_incremental` vs `group_signatures_parallel_with_matches`.
//!
//! Run with: `cargo bench --bench grouping_incremental --features synthetic`
//!
//! Background: profiling scenario E (550-activity year expansion) put
//! `recompute_groups` at ~4 s of the ~9 s total, and veloqrs always took the
//! full path even though tracematch had a `group_incremental` (O(N×M) where
//! M = new). This bench measures the delta between the two paths at realistic
//! corpus sizes.
//!
//! **The 4 s is `recompute_groups`, not the fold this file benchmarks.** That
//! call also builds signatures and writes tables, and the figure predates the
//! endpoint-grid filter in `grouping_filter.rs`. The fold itself, on this
//! machine on a quiet run: 7.7 ms at 60 activities, 22.7 ms at 150, 123.7 ms
//! at 550. Read a number off your own run rather than off this comment, and
//! do not quote the 4 s as the cost of grouping.

use criterion::{Criterion, SamplingMode, criterion_group, criterion_main};
use std::time::Duration;
use tracematch::scenarios::{LifecycleConfig, LifecycleCorpus};
use tracematch::{
    MatchConfig, RouteSignature, group_incremental, group_signatures_parallel_with_matches,
};

fn build_signatures(
    corpus_through_e: &[(String, Vec<tracematch::GpsPoint>)],
) -> Vec<RouteSignature> {
    let cfg = MatchConfig::default();
    corpus_through_e
        .iter()
        .filter_map(|(id, pts)| RouteSignature::from_points(id, pts, &cfg))
        .collect()
}

fn make_corpus_signatures() -> Vec<RouteSignature> {
    let cfg = LifecycleConfig::default();
    let corpus = LifecycleCorpus::generate(&cfg);
    let tracks: Vec<(String, Vec<tracematch::GpsPoint>)> = corpus
        .through_e()
        .iter()
        .map(|a| (a.id.clone(), a.gps_points.clone()))
        .collect();
    build_signatures(&tracks)
}

fn bench_full_grouping(c: &mut Criterion) {
    let mut group = c.benchmark_group("grouping_full");
    group.sampling_mode(SamplingMode::Flat);
    group.sample_size(10);
    group.warm_up_time(Duration::from_secs(2));
    group.measurement_time(Duration::from_secs(60));

    let signatures = make_corpus_signatures();
    let cfg = MatchConfig::default();

    // Full grouping at the corpus sizes that matter for scenario E:
    // 60 (post-A), 150 (post-B), 154 (post-D), 550 (post-E).
    for size in [60usize, 150, 550] {
        let slice: Vec<RouteSignature> = signatures.iter().take(size).cloned().collect();
        group.bench_function(format!("full_{size}"), |b| {
            b.iter(|| group_signatures_parallel_with_matches(&slice, &cfg));
        });
    }

    group.finish();
}

fn bench_incremental_grouping(c: &mut Criterion) {
    let mut group = c.benchmark_group("grouping_incremental");
    group.sampling_mode(SamplingMode::Flat);
    group.sample_size(10);
    group.warm_up_time(Duration::from_secs(2));
    group.measurement_time(Duration::from_secs(60));

    let signatures = make_corpus_signatures();
    let cfg = MatchConfig::default();

    // Pre-build "existing" state at three sizes by running full grouping
    // once. The bench then measures only the incremental call.
    // Includes large-M cases (60+90, 154+396) to test whether the 50%
    // FULL-fallback threshold could safely be raised.
    for (existing_n, new_n) in [
        (60usize, 1usize),
        (150, 1),
        (150, 3),
        (547, 3),
        (60, 90),   // analogous to scenario B step 2 (60% new)
        (154, 396), // analogous to scenario E step 5 (72% new)
    ] {
        let existing: Vec<RouteSignature> = signatures.iter().take(existing_n).cloned().collect();
        let new: Vec<RouteSignature> = signatures
            .iter()
            .skip(existing_n)
            .take(new_n)
            .cloned()
            .collect();
        let existing_groups = group_signatures_parallel_with_matches(&existing, &cfg).groups;

        group.bench_function(format!("incremental_{existing_n}+{new_n}"), |b| {
            b.iter(|| group_incremental(&new, &existing_groups, &existing, &cfg))
        });
    }

    group.finish();
}

criterion_group!(benches, bench_full_grouping, bench_incremental_grouping);
criterion_main!(benches);
