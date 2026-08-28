//! Bitwise regression harness for the unified fold, shared by the corpus gates.
//!
//! Hashes every field of the catalogue after a full cold fold, a progressive
//! cold-to-warm run, and a bulk backfill, and compares the digests against a
//! golden file. A code change that alters one bit of any output, or the emitted
//! order, fails. Run-to-run determinism is asserted in-process, so a
//! parallelism regression surfaces without a baseline.
//!
//! After an INTENTIONAL output change, re-baseline with
//! `TRACEMATCH_BITWISE_REBASE=1`.

#![allow(dead_code)]

use std::collections::HashMap;
use std::path::Path;
use std::time::Instant;

use tracematch::sections::FrequentSection;
use tracematch::{
    GpsPoint, SectionConfig, SectionEvidenceCache, SectionUpdatePolicy,
    sections::detect_sections_unified_incremental_dated,
};

pub struct Corpus {
    pub tracks: Vec<(String, Vec<GpsPoint>)>,
    pub sports: HashMap<String, String>,
    pub starts: HashMap<String, i64>,
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
        h.u64(s.route_ids.len() as u64);
        for id in &s.route_ids {
            h.str(id);
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
        &[],
        &c.sports,
        &c.starts,
        config,
        &SectionUpdatePolicy::default(),
    )
    .catalogue
}

/// Run the three scenarios and compare their digests with `golden_path`, or
/// record them there when it is absent or a rebase is requested.
pub fn run(c: &Corpus, shape: Shape, golden_path: &Path) {
    let config = SectionConfig::default();
    let n = c.tracks.len();
    assert!(
        n > shape.warm_adds && n > shape.bulk_base,
        "{n} activities cannot carry {} warm adds over a {}-activity base",
        shape.warm_adds,
        shape.bulk_base
    );
    let mut lines: Vec<String> = Vec::new();

    // A: full cold fold, twice in-process. Equal digests prove run-to-run
    // determinism (including under rayon) with no baseline needed.
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
    let cold_ms = t.elapsed().as_millis();
    let again = fold(
        &mut SectionEvidenceCache::new(),
        &[],
        &c.tracks,
        &all_ids,
        c,
        &config,
    );
    assert_eq!(
        catalogue_digest(&cold),
        catalogue_digest(&again),
        "two identical cold folds diverged within one process: \
         nondeterminism, look at parallel or iteration order first"
    );
    println!(
        "A cold {} activities: {} sections, {} ms",
        n,
        cold.len(),
        cold_ms
    );
    lines.push(format!("A {:016x}", catalogue_digest(&cold)));

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
    let mut add_ms: Vec<u128> = Vec::new();
    for i in 0..shape.warm_adds {
        let pool = &c.tracks[..split + i + 1];
        let new_id = [c.tracks[split + i].0.as_str()];
        let existing = catalogue.clone();
        let t = Instant::now();
        catalogue = fold(&mut cache, &existing, pool, &new_id, c, &config);
        add_ms.push(t.elapsed().as_millis());
        lines.push(format!(
            "B_add_{:02} {:016x}",
            i + 1,
            catalogue_digest(&catalogue)
        ));
    }
    let mut sorted = add_ms.clone();
    sorted.sort_unstable();
    println!(
        "B warm adds: mean {} ms, p95 {} ms",
        add_ms.iter().sum::<u128>() / add_ms.len() as u128,
        sorted[(sorted.len() * 95 / 100).min(sorted.len() - 1)]
    );

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
    println!(
        "C bulk {} over {}: {} sections, {} ms",
        rest_ids.len(),
        shape.bulk_base,
        bulk.len(),
        t.elapsed().as_millis()
    );
    lines.push(format!("C {:016x}", catalogue_digest(&bulk)));

    let current = lines.join("\n") + "\n";
    let rebase = std::env::var("TRACEMATCH_BITWISE_REBASE").is_ok_and(|v| v == "1");
    match std::fs::read_to_string(golden_path) {
        Ok(golden) if !rebase => {
            let want: Vec<&str> = golden.lines().collect();
            let got: Vec<&str> = current.lines().collect();
            for (w, g) in want.iter().zip(got.iter()) {
                assert_eq!(
                    w, g,
                    "bitwise divergence from the golden baseline. If this \
                     change is INTENTIONAL, rerun with \
                     TRACEMATCH_BITWISE_REBASE=1"
                );
            }
            assert_eq!(want.len(), got.len(), "scenario count changed");
            println!("golden baseline matched: {} digests", got.len());
        }
        _ => {
            std::fs::write(golden_path, &current).expect("write golden baseline");
            println!(
                "golden baseline {} at {}",
                if rebase { "rewritten" } else { "recorded" },
                golden_path.display()
            );
        }
    }
}
