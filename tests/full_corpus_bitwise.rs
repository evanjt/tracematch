//! Full-corpus bitwise regression gate for the unified fold.
//!
//! The `bitwise` harness over the private corpus, with the golden baseline
//! stored NEXT TO THE CORPUS, never in the repository. `geolife_bitwise.rs`
//! is the same gate over public data and is the one CI runs.
//!
//! Local-only: gated behind `real-corpus`, corpus resolved via
//! `TRACEMATCH_CORPUS`. After an INTENTIONAL output change, re-baseline with
//! `TRACEMATCH_BITWISE_REBASE=1`.
//!
//! Run: `cargo test --release --features real-corpus --test full_corpus_bitwise -- --nocapture`

mod bitwise;
mod corpus;

use bitwise::{Corpus, Shape};

const CORPUS: &str = "fullcorpus";
const GOLDEN: &str = "_bitwise_golden.txt";

fn load_corpus() -> Corpus {
    let (tracks, starts) = corpus::load_tracks(CORPUS, 1000);
    let sports = tracks
        .iter()
        .map(|(id, _)| (id.clone(), "All".to_string()))
        .collect();
    Corpus {
        tracks,
        sports,
        starts,
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
        &corpus::dir(CORPUS).join(GOLDEN),
    );
}
