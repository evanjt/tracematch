//! Full-corpus bitwise regression gate for the unified fold.
//!
//! The `bitwise` harness over the private corpus, with the golden baseline
//! stored NEXT TO THE CORPUS, never in the repository. `geolife_bitwise.rs`
//! is the same gate over public data and is the one CI runs.
//!
//! Local-only: gated behind `real-corpus`, corpus resolved via
//! `TRACEMATCH_CORPUS`. A change that moves the output attaches its
//! before-and-after report to the item and stops there; the rebase is Evan's
//! call and carries its reason. See `bitwise::baseline::REBASE_ENV`.
//!
//! Run: `cargo test --release --features real-corpus --test full_corpus_bitwise -- --nocapture`

mod bitwise;
mod corpus;

use bitwise::{Corpus, Shape, baseline::Band};
use tracematch::GpsPoint;

const CORPUS: &str = "fullcorpus";
const GOLDEN: &str = "_bitwise_golden.txt";

/// One machine holds this corpus, so the clock only has to absorb run-to-run
/// scheduling noise. The heap is the code's own and gets a tighter band.
const BAND: Band = Band {
    time_factor: 1.35,
    time_floor_ms: 50,
    bytes_factor: 1.20,
    bytes_floor: 16 * 1024 * 1024,
};

fn load_corpus() -> Corpus {
    let (tracks, starts, seconds) = corpus::load_tracks(CORPUS, 1000);
    let sports = tracks
        .iter()
        .map(|(id, _)| (id.clone(), "All".to_string()))
        .collect();
    Corpus {
        tracks,
        sports,
        starts,
        seconds,
    }
}

#[test]
fn full_corpus_output_is_bitwise_stable() {
    let c = load_corpus();
    bitwise::run(
        &c,
        Shape {
            warm_adds: 50,
            bulk_base: 100,
        },
        BAND,
        &corpus::dir(CORPUS).join(GOLDEN),
    );
}

/// The veto that keeps chairlift ground out of the catalogue reads elevation
/// first and per-point time second: `lift_spans_tuned` returns empty below two
/// elevated points, and without seconds it judges a candidate by GPS jitter
/// rather than by speed. A corpus loaded without both makes every digest below
/// silent about it, the way an empty corpus makes them vacuous.
#[test]
fn the_corpus_this_gate_folds_can_exercise_the_lift_veto() {
    let c = load_corpus();

    let elevated = c
        .tracks
        .iter()
        .filter(|(_, pts)| pts.iter().filter(|p| p.elevation.is_some()).count() >= 2)
        .count();
    assert!(
        elevated * 2 > c.tracks.len(),
        "only {elevated} of {} tracks carry elevation, so the lift veto is inert \
         over most of this corpus",
        c.tracks.len()
    );

    let timed = c
        .tracks
        .iter()
        .filter(|(id, pts)| c.seconds.get(id).is_some_and(|s| s.len() == pts.len()))
        .count();
    assert!(
        timed * 2 > c.tracks.len(),
        "only {timed} of {} tracks carry per-point time, so most of this corpus \
         reaches the jitter fallback rather than the velocity veto the tunables \
         were measured on",
        c.tracks.len()
    );

    // Both readings, because the difference is the whole reason the streams
    // are loaded. The untimed one is what the gate measured before the streams
    // reached it; a corpus where they agree is not testing the velocity veto.
    let view: Vec<(&str, &[GpsPoint])> = c
        .tracks
        .iter()
        .map(|(id, pts)| (id.as_str(), pts.as_slice()))
        .collect();
    let untimed = tracematch::sections::confirmed_lift_spans_tuned(
        &view,
        &[],
        &tracematch::Tunables::DEFAULT,
    );
    let untimed_spans: usize = untimed.iter().map(Vec::len).sum();
    let (spans, points) = bitwise::lift_reach(&c);
    println!(
        "lift veto over {} tracks: {untimed_spans} spans untimed, {spans} timed, \
         {points} points excluded",
        c.tracks.len()
    );
    assert!(
        spans > 0,
        "the lift veto excludes no ground anywhere in {} tracks, so nothing \
         below this line measures it",
        c.tracks.len()
    );
}
