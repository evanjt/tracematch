//! Bitwise regression harness for the unified fold, shared by the corpus gates.
//!
//! Hashes every field of the catalogue after a full cold fold, a progressive
//! cold-to-warm run, and a bulk backfill, and compares the digests against a
//! golden file. A code change that alters one bit of any output, or the emitted
//! order, fails. Run-to-run determinism is asserted in-process, so a
//! parallelism regression surfaces without a baseline.
//!
//! What each scenario cost is recorded in the same golden and bounded by a
//! band, so a cold-path slowdown fails the same gate the output does. See
//! `baseline`.
//!
//! The golden is a tripwire, not a correctness record. An agent whose change
//! moves it attaches the before-and-after report to the item and stops there:
//! the rebase is Evan's call. When it is made it carries its reason, which the
//! file keeps as a `# rebased` line. See `baseline::REBASE_ENV`.

#![allow(dead_code)]

pub mod baseline;

use std::collections::HashMap;
use std::path::Path;
use std::time::Instant;

use baseline::Band;

use tracematch::sections::FrequentSection;
use tracematch::{
    GpsPoint, SectionConfig, SectionEvidenceCache, SectionUpdatePolicy, Tunables,
    sections::{confirmed_lift_spans_tuned, detect_sections_unified_incremental_dated},
};

pub struct Corpus {
    pub tracks: Vec<(String, Vec<GpsPoint>)>,
    pub sports: HashMap<String, String>,
    pub starts: HashMap<String, i64>,
    /// Per-point seconds by activity id, for the tracks that carry a stream
    /// covering every point. The lift veto judges a timed candidate by speed
    /// and an untimed one by GPS jitter, so a corpus folded without these
    /// measures a detector the app does not run.
    pub seconds: HashMap<String, Vec<f64>>,
}

/// How the three scenarios slice the corpus.
pub struct Shape {
    /// Activities added one at a time after a cold fold over the rest.
    pub warm_adds: usize,
    /// Activities folded cold before the bulk backfill of everything else.
    pub bulk_base: usize,
}

// ---------------------------------------------------------------------------
// Canonical bitwise digest: FNV-1a 64 over every deterministic field, floats
// as raw bits, in emitted order. `updated_at`/`created_at` are wall-clock and
// engine-owned, so they are excluded.
// ---------------------------------------------------------------------------

struct Fnv(u64);

impl Fnv {
    fn new() -> Self {
        Fnv(0xcbf29ce484222325)
    }
    fn bytes(&mut self, b: &[u8]) {
        for &x in b {
            self.0 ^= x as u64;
            self.0 = self.0.wrapping_mul(0x100000001b3);
        }
    }
    fn u64(&mut self, v: u64) {
        self.bytes(&v.to_le_bytes());
    }
    fn f64(&mut self, v: f64) {
        self.u64(v.to_bits());
    }
    fn str(&mut self, s: &str) {
        self.u64(s.len() as u64);
        self.bytes(s.as_bytes());
    }
    fn point(&mut self, p: &GpsPoint) {
        self.f64(p.latitude);
        self.f64(p.longitude);
        match p.elevation {
            Some(e) => {
                self.u64(1);
                self.f64(e);
            }
            None => self.u64(0),
        }
    }
}

pub fn catalogue_digest(sections: &[FrequentSection]) -> u64 {
    let mut h = Fnv::new();
    h.u64(sections.len() as u64);
    for s in sections {
        h.str(&s.id);
        match &s.name {
            Some(n) => {
                h.u64(1);
                h.str(n);
            }
            None => h.u64(0),
        }
        h.str(&s.sport_type);
        h.u64(s.polyline.len() as u64);
        for p in &s.polyline {
            h.point(p);
        }
        h.str(&s.representative_activity_id);
        h.u64(s.activity_ids.len() as u64);
        for id in &s.activity_ids {
            h.str(id);
        }
        h.u64(s.activity_portions.len() as u64);
        for ap in &s.activity_portions {
            h.str(&ap.activity_id);
            h.u64(ap.start_index as u64);
            h.u64(ap.end_index as u64);
            h.f64(ap.distance_meters);
            h.str(&format!("{:?}", ap.direction));
        }
        h.u64(s.visit_count as u64);
        h.f64(s.distance_meters);
        // activity_traces: HashMap, so sort keys for a stable walk.
        let mut trace_ids: Vec<&String> = s.activity_traces.keys().collect();
        trace_ids.sort();
        h.u64(trace_ids.len() as u64);
        for id in trace_ids {
            h.str(id);
            let pts = &s.activity_traces[id];
            h.u64(pts.len() as u64);
            for p in pts {
                h.point(p);
            }
        }
        h.f64(s.confidence);
        h.u64(s.observation_count as u64);
        h.f64(s.average_spread);
        h.u64(s.point_density.len() as u64);
        for &d in &s.point_density {
            h.u64(d as u64);
        }
        h.str(&format!("{:?}", s.scale));
        h.u64(s.is_user_defined as u64);
        h.f64(s.stability);
        h.u64(s.version as u64);
    }
    h.0
}

/// The corpus seconds as the detector takes them: one slice per pool entry, in
/// pool order, empty where a track has no stream. Built by id rather than by
/// index because the scenarios fold prefixes of the corpus, not all of it.
pub fn seconds_view<'a>(pool: &[(String, Vec<GpsPoint>)], c: &'a Corpus) -> Vec<&'a [f64]> {
    pool.iter()
        .map(|(id, _)| c.seconds.get(id).map_or(&[][..], |s| s.as_slice()))
        .collect()
}

/// What the lift veto actually removes from `c`, as `(spans, points)` over the
/// whole corpus after the cross-track descent rescue. This is the ground the
/// catalogue below never sees. Recorded in the golden and compared for
/// equality, not banded: it is an exact count of a pure function, so any drift
/// is a change in the veto's reach and worth failing on.
pub fn lift_reach(c: &Corpus) -> (usize, usize) {
    let view: Vec<(&str, &[GpsPoint])> = c
        .tracks
        .iter()
        .map(|(id, pts)| (id.as_str(), pts.as_slice()))
        .collect();
    let spans = confirmed_lift_spans_tuned(&view, &seconds_view(&c.tracks, c), &Tunables::DEFAULT);
    (
        spans.iter().map(Vec::len).sum(),
        spans
            .iter()
            .flatten()
            .map(|(start, end)| end - start + 1)
            .sum(),
    )
}

pub fn fold(
    cache: &mut SectionEvidenceCache,
    existing: &[FrequentSection],
    pool: &[(String, Vec<GpsPoint>)],
    new_ids: &[&str],
    c: &Corpus,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    detect_sections_unified_incremental_dated(
        cache,
        existing,
        pool,
        new_ids,
        &seconds_view(pool, c),
        &c.sports,
        &c.starts,
        config,
        &SectionUpdatePolicy::default(),
    )
    .catalogue
}

/// Run the three scenarios and compare their digests and costs with
/// `golden_path`, or record them there when it is absent or a rebase is
/// requested.
pub fn run(c: &Corpus, shape: Shape, band: Band, golden_path: &Path) {
    let config = SectionConfig::default();
    let n = c.tracks.len();
    assert!(
        n > shape.warm_adds && n > shape.bulk_base,
        "{n} activities cannot carry {} warm adds over a {}-activity base",
        shape.warm_adds,
        shape.bulk_base
    );
    let mut lines: Vec<String> = Vec::new();

    // The veto's reach, before any scenario. A corpus that excludes nothing
    // makes every digest below silent about it, so the number is recorded
    // rather than assumed: GeoLife reads zero because it carries no elevation
    // to raise a candidate from.
    let (lift_spans, lift_points) = lift_reach(c);
    println!("lift veto: {lift_spans} confirmed spans, {lift_points} points excluded");
    lines.push(format!("L {lift_spans} {lift_points}"));

    // The corpus is already in memory and stays there, so anchor here and let
    // the peak report what folding it costs on top.
    baseline::anchor_peak();

    // A: full cold fold, twice in-process. Equal digests prove run-to-run
    // determinism (including under rayon) with no baseline needed. The first
    // catalogue is released before the second fold, so holding both does not
    // show up as the fold's own memory.
    let all_ids: Vec<&str> = c.tracks.iter().map(|(id, _)| id.as_str()).collect();
    let t = Instant::now();
    let cold = fold(
        &mut SectionEvidenceCache::new(),
        &[],
        &c.tracks,
        &all_ids,
        c,
        &config,
    );
    let cold_ms = t.elapsed().as_millis() as u64;
    let cold_digest = catalogue_digest(&cold);
    let cold_len = cold.len();
    drop(cold);
    let again = fold(
        &mut SectionEvidenceCache::new(),
        &[],
        &c.tracks,
        &all_ids,
        c,
        &config,
    );
    assert_eq!(
        cold_digest,
        catalogue_digest(&again),
        "two identical cold folds diverged within one process: \
         nondeterminism, look at parallel or iteration order first"
    );
    drop(again);
    println!("A cold {n} activities: {cold_len} sections, {cold_ms} ms");
    lines.push(format!("A {cold_digest:016x}"));

    // B: progressive cold-to-warm. Cold over the head, then the tail one at
    // a time, digest after every add.
    let split = n - shape.warm_adds;
    let mut cache = SectionEvidenceCache::new();
    let head_ids: Vec<&str> = c.tracks[..split]
        .iter()
        .map(|(id, _)| id.as_str())
        .collect();
    let mut catalogue = fold(&mut cache, &[], &c.tracks[..split], &head_ids, c, &config);
    lines.push(format!("B_cold {:016x}", catalogue_digest(&catalogue)));
    let mut add_ms: Vec<u64> = Vec::new();
    for i in 0..shape.warm_adds {
        let pool = &c.tracks[..split + i + 1];
        let new_id = [c.tracks[split + i].0.as_str()];
        let existing = catalogue.clone();
        let t = Instant::now();
        catalogue = fold(&mut cache, &existing, pool, &new_id, c, &config);
        add_ms.push(t.elapsed().as_millis() as u64);
        lines.push(format!(
            "B_add_{:02} {:016x}",
            i + 1,
            catalogue_digest(&catalogue)
        ));
    }
    let (add_median_ms, add_p95_ms) = median_and_p95(&add_ms);
    println!("B warm adds: median {add_median_ms} ms, p95 {add_p95_ms} ms");

    // C: bulk backfill, the second-user shape. Cold over the base, everything
    // else in one fold.
    let mut cache = SectionEvidenceCache::new();
    let base_ids: Vec<&str> = c.tracks[..shape.bulk_base]
        .iter()
        .map(|(id, _)| id.as_str())
        .collect();
    let base = fold(
        &mut cache,
        &[],
        &c.tracks[..shape.bulk_base],
        &base_ids,
        c,
        &config,
    );
    let rest_ids: Vec<&str> = c.tracks[shape.bulk_base..]
        .iter()
        .map(|(id, _)| id.as_str())
        .collect();
    let t = Instant::now();
    let bulk = fold(&mut cache, &base, &c.tracks, &rest_ids, c, &config);
    let bulk_ms = t.elapsed().as_millis() as u64;
    println!(
        "C bulk {} over {}: {} sections, {bulk_ms} ms",
        rest_ids.len(),
        shape.bulk_base,
        bulk.len()
    );
    lines.push(format!("C {:016x}", catalogue_digest(&bulk)));

    let measured = [
        ("perf_cold_ms", cold_ms),
        ("perf_add_median_ms", add_median_ms),
        ("perf_add_p95_ms", add_p95_ms),
        ("perf_bulk_ms", bulk_ms),
        ("perf_peak_bytes", baseline::peak_rise_bytes()),
    ];
    baseline::check(golden_path, &lines, &measured, &band);
}

/// Median and p95 of a set of timings, both by nearest rank.
pub fn median_and_p95(ms: &[u64]) -> (u64, u64) {
    assert!(!ms.is_empty(), "no timings to summarise");
    let mut sorted = ms.to_vec();
    sorted.sort_unstable();
    let p95 = sorted[(sorted.len() * 95 / 100).min(sorted.len() - 1)];
    (sorted[sorted.len() / 2], p95)
}
