//! Answer, for named (line, activity) pairs, the passes the detector counts
//! over the public GeoLife corpus. `section_passes.rs` over the public data:
//! same request and answer lines, read by `section_spatial.py`. A scratch
//! measurement, not a gate.
//!
//! Run: `SECTION_PASSES_IN=/tmp/req.jsonl SECTION_PASSES_OUT=/tmp/ans.jsonl \
//!       cargo test --release --features public-corpus --test section_passes_geolife -- --nocapture`
#![cfg(feature = "public-corpus")]

mod bitwise;
mod geolife_corpus;
mod section_rows;

use std::collections::HashMap;
use tracematch::GpsPoint;

#[test]
fn answer_passes() {
    let input =
        std::env::var("SECTION_PASSES_IN").expect("SECTION_PASSES_IN names the request file");
    let output =
        std::env::var("SECTION_PASSES_OUT").expect("SECTION_PASSES_OUT names the answer file");
    let c = geolife_corpus::load_busiest();
    let by_id: HashMap<&str, &[GpsPoint]> = c
        .tracks
        .iter()
        .map(|(id, t)| (id.as_str(), t.as_slice()))
        .collect();
    let n = section_rows::answer_passes(&by_id, &input, &output);
    println!("answered {n} requests into {output}");
}
