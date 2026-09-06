//! The per-add budget: after a cold fold over the rest of the private
//! corpus, each of the last activities is added on its own and timed. The
//! median must fit the instant tier a sync pays per downloaded activity.
//!
//! Two numbers judge it and they judge different things. The run is compared
//! with the baseline recorded beside the corpus, banded, which is the only
//! comparison a wall clock can survive: a run reads 700 or 900 on the same
//! code depending on what else the machine is doing, so a hard bound on the
//! run reports the machine. The recorded baseline is what the ceiling bounds,
//! and it is deterministic: it moves only when the corpus changes or a rebase
//! is asked for, and neither may carry it past `BUDGET_MS`. So a slowdown
//! fails the band, a rebase that tries to absorb one fails the ceiling, and
//! neither verdict depends on the run's noise.
//!
//! The baseline is re-derived when the corpus changes shape. The golden
//! records the track and point counts it was measured on, and a run over a
//! different corpus re-records rather than comparing, so growth is reported
//! as growth and never as a regression. The cold fold has no ceiling and is
//! bounded by the band alone. Rebase by hand with the same switch the digests
//! use, which carries the reason it moved: see `bitwise::baseline::REBASE_ENV`.
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
/// The ceiling on the recorded warm-add median, in milliseconds.
///
/// It bounds the baseline, not the run. A run is judged by `BAND` against the
/// baseline, and the baseline may not be recorded or rebased past this, which
/// is the stop on the ratchet: a regression cannot be absorbed by rebasing,
/// and corpus growth re-derives the baseline only while it still fits.
///
/// 900 is not a measurement. The baseline was 730 when `Q55` set this over
/// the elevated corpus and 702 once `B154` pinned the build profile, so the
/// headroom is real and the number states what the sync is prepared to pay
/// per activity on a corpus of this size. It is re-derived on one occasion
/// only: when a production per-activity budget is set (`Q2` deferred one),
/// this becomes that number.
const BUDGET_MS: u64 = 900;

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
    let (tracks, starts, seconds) = corpus::load_tracks(CORPUS, 1000);
    let sports = tracks
        .iter()
        .map(|(id, _)| (id.clone(), "All".to_string()))
        .collect();
    let c = Corpus {
        tracks,
        sports,
        starts,
        seconds,
    };
    let config = SectionConfig::default();

    // A golden recorded on another corpus is re-derived, not compared.
    let path = corpus::dir(CORPUS).join(BASELINE);
    let points: usize = c.tracks.iter().map(|(_, t)| t.len()).sum();
    let shape = format!("C {} {points}", c.tracks.len());
    let mut golden = std::fs::read_to_string(&path).ok();
    if let Some(existing) = golden.as_deref()
        && baseline::digests_differ(existing, std::slice::from_ref(&shape))
    {
        println!(
            "corpus changed shape, was {:?}, now {shape:?}: re-deriving the baseline",
            baseline::digest_lines(existing)
        );
        std::fs::remove_file(&path).expect("remove stale golden baseline");
        golden = None;
    }
    let recording = golden.is_none() || baseline::rebase_asked().is_some();

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

    // The ceiling judges the number that will be the baseline after this run:
    // the recorded one, or this run's when it is about to be recorded. Its
    // verdict is printed before the band's so a failing run reports both.
    let recorded = match (&golden, recording) {
        (Some(golden), false) => baseline::recorded(golden)
            .into_iter()
            .find(|(name, _)| name == "perf_add_median_ms")
            .map(|(_, value)| value)
            .expect("the baseline records perf_add_median_ms"),
        _ => median,
    };
    let over_ceiling = recorded > BUDGET_MS;
    println!(
        "recorded warm-add median {recorded} ms against the {BUDGET_MS} ms ceiling: {}",
        if over_ceiling { "OVER" } else { "within" }
    );

    let measured = [
        ("perf_cold_ms", cold_ms),
        ("perf_add_median_ms", median),
        ("perf_add_p95_ms", p95),
        ("perf_peak_bytes", baseline::peak_rise_bytes()),
    ];
    baseline::check(&path, &[shape], &measured, &BAND);
    assert!(
        !over_ceiling,
        "the recorded warm-add median {recorded} ms is past the {BUDGET_MS} ms ceiling. \
         The band cannot absorb this by rebasing: make the add cheaper, or raise the \
         ceiling with the reasoning beside it"
    );
}
