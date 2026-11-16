//! Dump every section of a cold fold over the public GeoLife corpus, as
//! `geolife_bitwise.rs` folds it, one JSON line each, so two detector pins can
//! be compared section by section. A scratch measurement, not a gate.
//!
//! Run: `SECTION_DUMP=/tmp/x.jsonl [SECTION_DUMP_GPX_DIR=/tmp/gpx] cargo test \
//!       --release --features public-corpus --test section_dump_geolife -- --nocapture`
//!
//! `SECTION_DUMP_GPX_DIR`, when set, receives the folded tracks as GPX so
//! `section_spatial.py` can draw them.
#![cfg(feature = "public-corpus")]

mod bitwise;
mod geolife_corpus;
mod section_rows;

use tracematch::{SectionConfig, SectionEvidenceCache};

#[test]
fn dump_cold_fold() {
    let out = std::env::var("SECTION_DUMP").expect("SECTION_DUMP names the output file");
    let c = geolife_corpus::load_busiest();
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
        &geolife_corpus::data_dir().display().to_string(),
        bitwise::catalogue_digest(&result.catalogue),
    );
    if let Ok(dir) = std::env::var("SECTION_DUMP_GPX_DIR")
        && !dir.trim().is_empty()
    {
        section_rows::write_gpx_corpus(&c.tracks, std::path::Path::new(&dir));
    }
    println!(
        "dumped {} sections of {} activities to {out}",
        result.catalogue.len(),
        c.tracks.len()
    );
}
