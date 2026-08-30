//! The per-add budget: after a cold fold over the rest of the private
//! corpus, each of the last activities is added on its own and timed. The
//! median must fit the instant tier a sync pays per downloaded activity.
//!
//! The cold fold itself has no hard budget, so it is bounded the other way:
//! recorded as a baseline beside the corpus and compared with a band, which
//! catches a cold-path regression without inventing a target for it. Rebase
//! with `TRACEMATCH_BITWISE_REBASE=1`, the same switch the digests use.
//!
//! Local-only: gated behind `real-corpus`, corpus resolved via
//! `TRACEMATCH_CORPUS`.
//!
//! Run: `cargo test --release --features real-corpus --test drip_budget -- --nocapture`

mod bitwise;
mod corpus;

use std::time::Instant;

use bitwise::baseline::{self, Band};
use bitwise::{Corpus, fold, median_and_p95};
use tracematch::{SectionConfig, SectionEvidenceCache};

const CORPUS: &str = "fullcorpus";
const BASELINE: &str = "_drip_baseline.txt";
const WARM_ADDS: usize = 50;
const BUDGET_MS: u64 = 500;

/// One machine holds this corpus, so the clock only has to absorb run-to-run
/// scheduling noise. The heap is the code's own and gets a tighter band.
const BAND: Band = Band {
    time_factor: 1.35,
    time_floor_ms: 50,
    bytes_factor: 1.20,
    bytes_floor: 16 * 1024 * 1024,
};

#[test]
fn a_warm_add_stays_inside_the_budget() {
    let (tracks, starts) = corpus::load_tracks(CORPUS, 1000);
    let sports = tracks
        .iter()
        .map(|(id, _)| (id.clone(), "All".to_string()))
        .collect();
    let c = Corpus {
        tracks,
        sports,
        starts,
    };
    let config = SectionConfig::default();
    let split = c.tracks.len() - WARM_ADDS;
    let head_ids: Vec<&str> = c.tracks[..split]
        .iter()
        .map(|(id, _)| id.as_str())
        .collect();
    // The corpus is loaded and stays loaded, so the peak below reports what
    // the fold costs on top of it.
    baseline::anchor_peak();
    let mut cache = SectionEvidenceCache::new();
    let t = Instant::now();
    let mut catalogue = fold(&mut cache, &[], &c.tracks[..split], &head_ids, &c, &config);
    let cold_ms = t.elapsed().as_millis() as u64;
    println!(
        "cold {} activities: {} sections, {cold_ms} ms",
        split,
        catalogue.len()
    );
    let mut add_ms: Vec<u64> = Vec::with_capacity(WARM_ADDS);
    for i in 0..WARM_ADDS {
        let pool = &c.tracks[..split + i + 1];
        let new_id = [c.tracks[split + i].0.as_str()];
        let existing = catalogue.clone();
        let t = Instant::now();
        catalogue = fold(&mut cache, &existing, pool, &new_id, &c, &config);
        add_ms.push(t.elapsed().as_millis() as u64);
    }
    let (median, p95) = median_and_p95(&add_ms);
    println!(
        "warm adds over {}: median {median} ms, p95 {p95} ms, max {} ms, {} sections",
        split,
        add_ms.iter().max().copied().unwrap_or_default(),
        catalogue.len()
    );
    assert!(
        median <= BUDGET_MS,
        "warm-add median {median} ms is over the {BUDGET_MS} ms budget"
    );

    let measured = [
        ("perf_cold_ms", cold_ms),
        ("perf_add_median_ms", median),
        ("perf_add_p95_ms", p95),
        ("perf_peak_bytes", baseline::peak_rise_bytes()),
    ];
    baseline::check(&corpus::dir(CORPUS).join(BASELINE), &[], &measured, &BAND);
}
