//! Timings of the incremental fold against the naive re-batch it replaces.
//!
//! Run with: `cargo bench --bench fold_cost --features synthetic`
//!
//! The scaling claims are gated in `tests/fold_foundation.rs` by counting the
//! work each add does. These are the wall-clock curves behind them, kept out of
//! the test lane because a timing ratio fails under load:
//!
//! - `naive_rebatch`: one `detect_sections` over N tracks, the cost a single add
//!   pays when the fold re-batches the pool. Summed over a drip it is O(N²).
//! - `cached_add_by_depth`: one cached add into a library of many far-apart
//!   clusters, shallow and deep, beside a naive re-batch of the same pool. The
//!   add stays flat while the re-batch grows.
//! - `cached_single_cluster_add`: one cached add when the whole library is one
//!   cluster. This grows with N, the honest O(cluster) = O(N) case.
//! - `cold`: a cold cache given N new activities in one call, beside the plain
//!   batch over the same N. The two track each other.

use std::collections::HashMap;
use std::time::Duration;

use criterion::{BatchSize, BenchmarkId, Criterion, criterion_group, criterion_main};
use tracematch::scenarios::{LifecycleConfig, LifecycleCorpus};
use tracematch::{
    FrequentSection, GpsPoint, SectionConfig, SectionEvidenceCache, SectionUpdatePolicy,
    detect_sections, detect_sections_incremental_cached_with_policy,
};

type Tracks = Vec<(String, Vec<GpsPoint>)>;

fn corpus_with_bucket_a(n: usize) -> LifecycleCorpus {
    LifecycleCorpus::generate(&LifecycleConfig {
        bucket_a_count: n,
        bucket_b_delta_count: 0,
        bucket_d_delta_count: 0,
        bucket_e_delta_count: 0,
        ..LifecycleConfig::default()
    })
}

/// The same library `gate_cached_incremental_cost_is_flat` drips: clusters 3°
/// apart with no one-offs or parallel streets, so none of them bridge.
fn multi_cluster_library(n_clusters: usize, bucket_a: usize) -> (Tracks, HashMap<String, String>) {
    let mut tracks = Vec::new();
    let mut sports = HashMap::new();
    for c in 0..n_clusters {
        let corpus = LifecycleCorpus::generate(&LifecycleConfig {
            origin: GpsPoint::with_elevation(44.0 + c as f64 * 3.0, 8.0, 400.0),
            seed: 0x51D + c as u64,
            bucket_a_count: bucket_a,
            bucket_b_delta_count: 0,
            bucket_d_delta_count: 0,
            bucket_e_delta_count: 0,
            one_off_fraction: 0.0,
            parallel_street_count: 0,
            ..LifecycleConfig::default()
        });
        for (id, pts) in corpus.tracks_through_e() {
            let id = format!("c{c}_{id}");
            sports.insert(id.clone(), "Ride".to_string());
            tracks.push((id, pts));
        }
    }
    (tracks, sports)
}

fn fold(
    cache: &mut SectionEvidenceCache,
    existing: &[FrequentSection],
    pool: &[(String, Vec<GpsPoint>)],
    new_ids: &[&str],
    sports: &HashMap<String, String>,
    cfg: &SectionConfig,
) -> Vec<FrequentSection> {
    detect_sections_incremental_cached_with_policy(
        cache,
        existing,
        pool,
        new_ids,
        &[],
        sports,
        cfg,
        &SectionUpdatePolicy::default(),
    )
    .catalogue
}

/// Drip `tracks[..n]` into a fresh cache, one add at a time.
fn dripped(
    tracks: &[(String, Vec<GpsPoint>)],
    n: usize,
    sports: &HashMap<String, String>,
    cfg: &SectionConfig,
) -> (SectionEvidenceCache, Vec<FrequentSection>) {
    let mut cache = SectionEvidenceCache::new();
    let mut catalogue = Vec::new();
    for i in 0..n {
        catalogue = fold(
            &mut cache,
            &catalogue,
            &tracks[..=i],
            &[tracks[i].0.as_str()],
            sports,
            cfg,
        );
    }
    (cache, catalogue)
}

fn configure(group: &mut criterion::BenchmarkGroup<'_, criterion::measurement::WallTime>) {
    group.sample_size(10);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(10));
}

fn bench_naive_rebatch(c: &mut Criterion) {
    let cfg = SectionConfig::default();
    let corpus = corpus_with_bucket_a(80);
    let tracks = corpus.tracks_through_e();
    let sports = corpus.sport_map_through_e();

    let mut group = c.benchmark_group("naive_rebatch");
    configure(&mut group);
    for n in [20usize, 40, 80] {
        let prefix = &tracks[..n];
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| detect_sections(prefix, &[], &sports, &cfg));
        });
    }
    group.finish();
}

/// One cached add of the next track, timed against a clone of the cache as it
/// stood after the first `n` adds, so every iteration sees the same state.
fn bench_add_after(
    group: &mut criterion::BenchmarkGroup<'_, criterion::measurement::WallTime>,
    id: BenchmarkId,
    tracks: &[(String, Vec<GpsPoint>)],
    n: usize,
    sports: &HashMap<String, String>,
    cfg: &SectionConfig,
) {
    let (cache, catalogue) = dripped(tracks, n, sports, cfg);
    let pool = &tracks[..=n];
    let new_ids = [tracks[n].0.as_str()];
    group.bench_function(id, |b| {
        b.iter_batched(
            || cache.clone(),
            |mut cache| fold(&mut cache, &catalogue, pool, &new_ids, sports, cfg),
            BatchSize::LargeInput,
        );
    });
}

fn bench_cached_add_by_depth(c: &mut Criterion) {
    let cfg = SectionConfig::default();
    let n_clusters = 9;
    let (tracks, sports) = multi_cluster_library(n_clusters, 8);
    let cluster_size = tracks.len() / n_clusters;

    let mut group = c.benchmark_group("cached_add_by_depth");
    configure(&mut group);
    // The first add into the second cluster and into the last: the touched
    // cluster is the same size at both depths, the pool is not.
    for depth in [1usize, n_clusters - 1] {
        let n = depth * cluster_size;
        bench_add_after(
            &mut group,
            BenchmarkId::new("cached", n + 1),
            &tracks,
            n,
            &sports,
            &cfg,
        );
        let pool = &tracks[..=n];
        group.bench_with_input(BenchmarkId::new("naive", n + 1), &n, |b, _| {
            b.iter(|| detect_sections(pool, &[], &sports, &cfg));
        });
    }
    group.finish();
}

fn bench_cached_single_cluster_add(c: &mut Criterion) {
    let cfg = SectionConfig::default();
    let corpus = corpus_with_bucket_a(36);
    let tracks = corpus.tracks_through_e();
    let sports = corpus.sport_map_through_e();

    let mut group = c.benchmark_group("cached_single_cluster_add");
    configure(&mut group);
    for n in [10usize, 20, 39] {
        bench_add_after(
            &mut group,
            BenchmarkId::from_parameter(n + 1),
            &tracks,
            n,
            &sports,
            &cfg,
        );
    }
    group.finish();
}

fn bench_cold(c: &mut Criterion) {
    let cfg = SectionConfig::default();
    let corpus = corpus_with_bucket_a(80);
    let tracks = corpus.tracks_through_e();
    let sports = corpus.sport_map_through_e();

    let mut group = c.benchmark_group("cold");
    configure(&mut group);
    for n in [20usize, 40, 80] {
        let prefix = &tracks[..n];
        let new_ids: Vec<&str> = prefix.iter().map(|(id, _)| id.as_str()).collect();
        group.bench_with_input(BenchmarkId::new("cached", n), &n, |b, _| {
            b.iter(|| {
                let mut cache = SectionEvidenceCache::new();
                fold(&mut cache, &[], prefix, &new_ids, &sports, &cfg)
            });
        });
        group.bench_with_input(BenchmarkId::new("batch", n), &n, |b, _| {
            b.iter(|| detect_sections(prefix, &[], &sports, &cfg));
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_naive_rebatch,
    bench_cached_add_by_depth,
    bench_cached_single_cluster_add,
    bench_cold
);
criterion_main!(benches);
