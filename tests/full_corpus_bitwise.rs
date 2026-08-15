//! Full-corpus bitwise regression gate for the unified fold.
//!
//! Hashes every field of the catalogue after a full cold fold, a progressive
//! cold-to-warm run, and a bulk backfill, and compares against a golden
//! baseline stored NEXT TO THE CORPUS, never in the repository. A code change
//! that alters one bit of any output, or the emitted order, fails here.
//! Run-to-run determinism is asserted in-process, so parallelism regressions
//! surface without a baseline.
//!
//! Local-only: gated behind `real-corpus`, corpus resolved via
//! `TRACEMATCH_CORPUS`. After an INTENTIONAL output change, re-baseline with
//! `TRACEMATCH_BITWISE_REBASE=1`.
//!
//! Run: `cargo test --release --features real-corpus --test full_corpus_bitwise -- --nocapture`

mod corpus;

use std::collections::HashMap;
use std::path::Path;
use std::time::Instant;

use tracematch::sections::FrequentSection;
use tracematch::{
    GpsPoint, SectionConfig, SectionEvidenceCache, SectionUpdatePolicy,
    sections::detect_sections_unified_incremental_dated,
};

const CORPUS: &str = "fullcorpus";
const GOLDEN: &str = "_bitwise_golden.txt";
const WARM_ADDS: usize = 50;

/// Track points from a GPX, tolerating a `<trkpt` whose attributes wrap onto
/// following lines.
fn load_points(path: &Path) -> Vec<GpsPoint> {
    let Ok(content) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let mut points = Vec::new();
    let bytes = content.as_bytes();
    let mut from = 0usize;
    while let Some(hit) = content[from..].find("<trkpt") {
        let start = from + hit;
        let end = (start + 200).min(bytes.len());
        let window = &content[start..end];
        let coord = |key: &str| -> Option<f64> {
            let at = window.find(key)? + key.len();
            let close = window[at..].find('"')?;
            window[at..at + close].parse().ok()
        };
        if let (Some(lat), Some(lon)) = (coord("lat=\""), coord("lon=\"")) {
            points.push(GpsPoint::new(lat, lon));
        }
        from = start + 6;
    }
    points
}

fn epoch_of(date: &str) -> Option<i64> {
    let (y, rest) = date.split_once('-')?;
    let (m, rest) = rest.split_once('-')?;
    let d = rest.get(0..2)?;
    let (y, m, d): (i64, i64, i64) = (y.parse().ok()?, m.parse().ok()?, d.parse().ok()?);
    let a = (14 - m) / 12;
    let y2 = y + 4800 - a;
    let m2 = m + 12 * a - 3;
    let jdn = d + (153 * m2 + 2) / 5 + 365 * y2 + y2 / 4 - y2 / 100 + y2 / 400 - 32045;
    Some((jdn - 2_440_588) * 86_400)
}

fn load_dates(dir: &Path) -> HashMap<String, String> {
    let Ok(content) = std::fs::read_to_string(dir.join("_meta.json")) else {
        return HashMap::new();
    };
    let Ok(v) = serde_json::from_str::<serde_json::Value>(&content) else {
        return HashMap::new();
    };
    v.as_object()
        .map(|map| {
            map.iter()
                .filter_map(|(id, m)| Some((id.clone(), m.get("date")?.as_str()?.to_string())))
                .collect()
        })
        .unwrap_or_default()
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

fn catalogue_digest(sections: &[FrequentSection]) -> u64 {
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

// ---------------------------------------------------------------------------
// The scenarios
// ---------------------------------------------------------------------------

struct Corpus {
    tracks: Vec<(String, Vec<GpsPoint>)>,
    sports: HashMap<String, String>,
    starts: HashMap<String, i64>,
}

fn load_corpus() -> Corpus {
    let dir = corpus::dir(CORPUS);
    let dates = load_dates(&dir);
    let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    for path in corpus::gpx_files(CORPUS, usize::MAX) {
        let id = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or_default()
            .split('_')
            .next()
            .unwrap_or_default()
            .to_string();
        let pts = load_points(&path);
        if pts.len() > 1 {
            tracks.push((id, pts));
        }
    }
    assert!(
        tracks.len() >= 1000,
        "expected the full corpus, got {} tracks",
        tracks.len()
    );
    let sports = tracks
        .iter()
        .map(|(id, _)| (id.clone(), "All".to_string()))
        .collect();
    let starts = tracks
        .iter()
        .filter_map(|(id, _)| {
            dates
                .get(id)
                .and_then(|d| epoch_of(d))
                .map(|e| (id.clone(), e))
        })
        .collect();
    Corpus {
        tracks,
        sports,
        starts,
    }
}

fn fold(
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

#[test]
fn full_corpus_output_is_bitwise_stable() {
    let c = load_corpus();
    let config = SectionConfig::default();
    let n = c.tracks.len();
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
        &c,
        &config,
    );
    let cold_ms = t.elapsed().as_millis();
    let again = fold(
        &mut SectionEvidenceCache::new(),
        &[],
        &c.tracks,
        &all_ids,
        &c,
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

    // B: progressive cold-to-warm. Cold over the head, then the last
    // WARM_ADDS activities one at a time, digest after every add.
    let split = n - WARM_ADDS;
    let mut cache = SectionEvidenceCache::new();
    let head_ids: Vec<&str> = c.tracks[..split].iter().map(|(id, _)| id.as_str()).collect();
    let mut catalogue = fold(
        &mut cache,
        &[],
        &c.tracks[..split],
        &head_ids,
        &c,
        &config,
    );
    lines.push(format!("B_cold {:016x}", catalogue_digest(&catalogue)));
    let mut add_ms: Vec<u128> = Vec::new();
    for i in 0..WARM_ADDS {
        let pool = &c.tracks[..split + i + 1];
        let new_id = [c.tracks[split + i].0.as_str()];
        let existing = catalogue.clone();
        let t = Instant::now();
        catalogue = fold(&mut cache, &existing, pool, &new_id, &c, &config);
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

    // C: bulk backfill, the second-user shape. Cold over the first 100,
    // everything else in one fold.
    let mut cache = SectionEvidenceCache::new();
    let base_ids: Vec<&str> = c.tracks[..100].iter().map(|(id, _)| id.as_str()).collect();
    let base = fold(&mut cache, &[], &c.tracks[..100], &base_ids, &c, &config);
    let rest_ids: Vec<&str> = c.tracks[100..].iter().map(|(id, _)| id.as_str()).collect();
    let t = Instant::now();
    let bulk = fold(&mut cache, &base, &c.tracks, &rest_ids, &c, &config);
    println!(
        "C bulk {} over 100: {} sections, {} ms",
        rest_ids.len(),
        bulk.len(),
        t.elapsed().as_millis()
    );
    lines.push(format!("C {:016x}", catalogue_digest(&bulk)));

    // Compare with the golden baseline, or record it. The file lives inside
    // the corpus directory, so it never enters any repository.
    let golden_path = corpus::dir(CORPUS).join(GOLDEN);
    let current = lines.join("\n") + "\n";
    let rebase = std::env::var("TRACEMATCH_BITWISE_REBASE").is_ok_and(|v| v == "1");
    match std::fs::read_to_string(&golden_path) {
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
            std::fs::write(&golden_path, &current).expect("write golden baseline");
            println!(
                "golden baseline {} at {}",
                if rebase { "rewritten" } else { "recorded" },
                golden_path.display()
            );
        }
    }
}
