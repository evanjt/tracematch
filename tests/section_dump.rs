//! Dump every section of a cold fold over the private corpus, one JSON line
//! each, so two detector pins can be compared section by section. A scratch
//! measurement, not a gate.
//!
//! Run: `SECTION_DUMP=/tmp/x.jsonl cargo test --release --features real-corpus \
//!       --test section_dump -- --nocapture`
#![cfg(feature = "real-corpus")]

mod bitwise;
mod corpus;
mod section_rows;

use tracematch::{SectionConfig, SectionEvidenceCache};

#[test]
fn dump_cold_fold() {
    let out = std::env::var("SECTION_DUMP").expect("SECTION_DUMP names the output file");
    let (tracks, starts, seconds) = corpus::load_tracks("fullcorpus", 1000);
    let sports = tracks
        .iter()
        .map(|(id, _)| (id.clone(), "All".to_string()))
        .collect();
    let c = bitwise::Corpus {
        tracks,
        sports,
        starts,
        seconds,
    };
    let config = SectionConfig::default();
    let all_ids: Vec<&str> = c.tracks.iter().map(|(id, _)| id.as_str()).collect();
    let result = tracematch::sections::detect_sections_incremental_dated(
        &mut SectionEvidenceCache::new(),
        &[],
        &c.tracks,
        &all_ids,
        &bitwise::seconds_view(&c.tracks, &c),
        &c.sports,
        &c.starts,
        &config,
        &tracematch::SectionUpdatePolicy::default(),
    );
    section_rows::write_dump(
        &result.catalogue,
        &result.boundaries,
        &out,
        &corpus::dir("fullcorpus").display().to_string(),
        bitwise::catalogue_digest(&result.catalogue),
    );
    println!("dumped {} sections to {out}", result.catalogue.len());
}
