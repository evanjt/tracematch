//! The per-add budget: after a cold fold over the rest of the private
//! corpus, each of the last activities is added on its own and timed. The
//! median must fit the instant tier a sync pays per downloaded activity.
//!
//! Local-only: gated behind `real-corpus`, corpus resolved via
//! `TRACEMATCH_CORPUS`.
//!
//! Run: `cargo test --release --features real-corpus --test drip_budget -- --nocapture`

mod bitwise;
mod corpus;

use std::time::Instant;

use bitwise::{Corpus, fold};
use tracematch::{SectionConfig, SectionEvidenceCache};

const CORPUS: &str = "fullcorpus";
const WARM_ADDS: usize = 50;
const BUDGET_MS: u128 = 500;

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
    let mut cache = SectionEvidenceCache::new();
    let t = Instant::now();
    let mut catalogue = fold(&mut cache, &[], &c.tracks[..split], &head_ids, &c, &config);
    println!(
        "cold {} activities: {} sections, {} ms",
        split,
        catalogue.len(),
        t.elapsed().as_millis()
    );
    let mut add_ms: Vec<u128> = Vec::with_capacity(WARM_ADDS);
    for i in 0..WARM_ADDS {
        let pool = &c.tracks[..split + i + 1];
        let new_id = [c.tracks[split + i].0.as_str()];
        let existing = catalogue.clone();
        let t = Instant::now();
        catalogue = fold(&mut cache, &existing, pool, &new_id, &c, &config);
        add_ms.push(t.elapsed().as_millis());
    }
    let mut sorted = add_ms.clone();
    sorted.sort_unstable();
    let median = sorted[sorted.len() / 2];
    let p95 = sorted[(sorted.len() * 95 / 100).min(sorted.len() - 1)];
    println!(
        "warm adds over {}: median {median} ms, p95 {p95} ms, max {} ms, {} sections",
        split,
        sorted[sorted.len() - 1],
        catalogue.len()
    );
    assert!(
        median <= BUDGET_MS,
        "warm-add median {median} ms is over the {BUDGET_MS} ms budget"
    );
}
