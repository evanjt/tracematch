//! Wall-clock cost of the three scenarios the bitwise gate folds over the
//! private corpus: a cold fold of every track, one warm add into a cache
//! warmed over the rest, and a bulk backfill over a base.
//!
//! Run with: `cargo bench --bench corpus_cost --features real-corpus`, with
//! `TRACEMATCH_CORPUS` naming the directory that holds `fullcorpus`.
//!
//! The folds, their order and their shape are the gate's: `bitwise::fold` over
//! `bitwise::FULL_CORPUS_SHAPE`, so the clocks time the folds whose digests the
//! golden holds. This is where a time is read. A timing ratio fails under load
//! in the test lane, so the gate does not judge one here; note the load average
//! beside any median quoted from a run.
//!
//! Not built without `real-corpus`, so `cargo check --benches` on a machine
//! with no corpus skips it.

// The gate's harness and corpus loader, compiled into this bench. The counting
// allocator the harness declares is installed here too, as it is in the gate,
// so the clocks carry the same per-allocation overhead the golden's were taken
// under.
#[path = "../tests/bitwise/mod.rs"]
mod bitwise;
#[path = "../tests/corpus/mod.rs"]
mod corpus;

use std::cell::Cell;
use std::time::Duration;

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use tracematch::{FrequentSection, SectionConfig, SectionEvidenceCache};

use bitwise::{Corpus, FULL_CORPUS_SHAPE, Shape};

const CORPUS: &str = "fullcorpus";

/// The cache and the catalogue after the head and some number of warm adds.
type Warm = (SectionEvidenceCache, Vec<FrequentSection>);

/// The gate's warm sequence, folded once outside the measurement. State `i` is
/// the cache and catalogue after the head and `i` adds, so add `i` starts from
/// the state the gate's own add `i` starts from.
fn warm_states(c: &Corpus, config: &SectionConfig, shape: &Shape) -> Vec<Warm> {
    let split = c.tracks.len() - shape.warm_adds;
    let mut cache = SectionEvidenceCache::new();
    let head_ids: Vec<&str> = c.tracks[..split]
        .iter()
        .map(|(id, _)| id.as_str())
        .collect();
    let mut catalogue = bitwise::fold(&mut cache, &[], &c.tracks[..split], &head_ids, c, config);
    let mut states = Vec::with_capacity(shape.warm_adds);
    for i in 0..shape.warm_adds {
        states.push((cache.clone(), catalogue.clone()));
        let pool = &c.tracks[..split + i + 1];
        let new_id = [c.tracks[split + i].0.as_str()];
        catalogue = bitwise::fold(&mut cache, &catalogue, pool, &new_id, c, config);
    }
    states
}

fn corpus_cost(criterion: &mut Criterion) {
    let (tracks, starts, seconds) = corpus::load_tracks(CORPUS, 1000);
    let corpus = bitwise::single_sport(tracks, starts, seconds);
    let config = SectionConfig::default();
    let shape = &FULL_CORPUS_SHAPE;
    let split = corpus.tracks.len() - shape.warm_adds;

    let all_ids: Vec<&str> = corpus.tracks.iter().map(|(id, _)| id.as_str()).collect();
    let base_ids: Vec<&str> = corpus.tracks[..shape.bulk_base]
        .iter()
        .map(|(id, _)| id.as_str())
        .collect();
    let rest_ids: Vec<&str> = corpus.tracks[shape.bulk_base..]
        .iter()
        .map(|(id, _)| id.as_str())
        .collect();

    let warm = warm_states(&corpus, &config, shape);
    let mut base_cache = SectionEvidenceCache::new();
    let base = bitwise::fold(
        &mut base_cache,
        &[],
        &corpus.tracks[..shape.bulk_base],
        &base_ids,
        &corpus,
        &config,
    );

    let mut group = criterion.benchmark_group("corpus_cost");
    group.sample_size(10);
    group.measurement_time(Duration::from_secs(60));

    group.bench_function("cold", |b| {
        b.iter_with_large_drop(|| {
            bitwise::fold(
                &mut SectionEvidenceCache::new(),
                &[],
                &corpus.tracks,
                &all_ids,
                &corpus,
                &config,
            )
        });
    });

    group.bench_function("warm_add", |b| {
        let next = Cell::new(0usize);
        b.iter_batched(
            || {
                let i = next.get();
                next.set((i + 1) % shape.warm_adds);
                let (cache, catalogue) = &warm[i];
                (i, cache.clone(), catalogue.clone())
            },
            |(i, mut cache, existing)| {
                let pool = &corpus.tracks[..split + i + 1];
                let new_id = [corpus.tracks[split + i].0.as_str()];
                let catalogue =
                    bitwise::fold(&mut cache, &existing, pool, &new_id, &corpus, &config);
                (catalogue, cache, existing)
            },
            BatchSize::PerIteration,
        );
    });

    group.bench_function("bulk", |b| {
        b.iter_batched(
            || (base_cache.clone(), base.clone()),
            |(mut cache, base)| {
                let catalogue = bitwise::fold(
                    &mut cache,
                    &base,
                    &corpus.tracks,
                    &rest_ids,
                    &corpus,
                    &config,
                );
                (catalogue, cache, base)
            },
            BatchSize::PerIteration,
        );
    });

    group.finish();
}

criterion_group!(benches, corpus_cost);
criterion_main!(benches);
