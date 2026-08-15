//! Unified fold benchmark over a GPX corpus: cold fold, progressive warm
//! adds, and bulk backfill, timed with peak resident memory and a full-field
//! catalogue digest per phase (mirrors tests/full_corpus_bitwise.rs). Builds
//! for the host or, via cargo-ndk, for a device.
//!
//!     cargo run --release --example arm_bench -- <CORPUS_DIR> \
//!         [--limit N] [--repeat N] [--split N] [--adds N] [--bulk]

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::Instant;

use tracematch::{
    GpsPoint, SectionConfig, SectionEvidenceCache, SectionUpdatePolicy, Tunables,
    sections::{detect_sections_unified_dated, detect_sections_unified_incremental_dated},
};

const POOLED: &str = "All";

/// Track points from a GPX, tolerating a `<trkpt` whose attributes wrap
/// onto following lines.
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

fn peak_rss_kb() -> u64 {
    std::fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("VmHWM:"))
                .and_then(|l| l.split_whitespace().nth(1).map(str::to_string))
        })
        .and_then(|v| v.parse().ok())
        .unwrap_or(0)
}

/// FNV-1a 64 over every deterministic catalogue field, floats as raw bits,
/// emitted order included. Mirrors tests/full_corpus_bitwise.rs.
fn catalogue_digest(sections: &[tracematch::FrequentSection]) -> u64 {
    struct Fnv(u64);
    impl Fnv {
        fn bytes(&mut self, b: &[u8]) {
            for &x in b {
                self.0 ^= x as u64;
                self.0 = self.0.wrapping_mul(0x100000001b3);
            }
        }
        fn u64v(&mut self, v: u64) {
            self.bytes(&v.to_le_bytes());
        }
        fn f64v(&mut self, v: f64) {
            self.u64v(v.to_bits());
        }
        fn strv(&mut self, s: &str) {
            self.u64v(s.len() as u64);
            self.bytes(s.as_bytes());
        }
        fn point(&mut self, p: &GpsPoint) {
            self.f64v(p.latitude);
            self.f64v(p.longitude);
            match p.elevation {
                Some(e) => {
                    self.u64v(1);
                    self.f64v(e);
                }
                None => self.u64v(0),
            }
        }
    }
    let mut f = Fnv(0xcbf29ce484222325);
    f.u64v(sections.len() as u64);
    for s in sections {
        f.strv(&s.id);
        match &s.name {
            Some(n) => {
                f.u64v(1);
                f.strv(n);
            }
            None => f.u64v(0),
        }
        f.strv(&s.sport_type);
        f.u64v(s.polyline.len() as u64);
        for p in &s.polyline {
            f.point(p);
        }
        f.strv(&s.representative_activity_id);
        f.u64v(s.activity_ids.len() as u64);
        for id in &s.activity_ids {
            f.strv(id);
        }
        f.u64v(s.activity_portions.len() as u64);
        for ap in &s.activity_portions {
            f.strv(&ap.activity_id);
            f.u64v(ap.start_index as u64);
            f.u64v(ap.end_index as u64);
            f.f64v(ap.distance_meters);
            f.strv(&format!("{:?}", ap.direction));
        }
        f.u64v(s.route_ids.len() as u64);
        for id in &s.route_ids {
            f.strv(id);
        }
        f.u64v(s.visit_count as u64);
        f.f64v(s.distance_meters);
        let mut trace_ids: Vec<&String> = s.activity_traces.keys().collect();
        trace_ids.sort();
        f.u64v(trace_ids.len() as u64);
        for id in trace_ids {
            f.strv(id);
            let pts = &s.activity_traces[id];
            f.u64v(pts.len() as u64);
            for p in pts {
                f.point(p);
            }
        }
        f.f64v(s.confidence);
        f.u64v(s.observation_count as u64);
        f.f64v(s.average_spread);
        f.u64v(s.point_density.len() as u64);
        for &d in &s.point_density {
            f.u64v(d as u64);
        }
        f.strv(&format!("{:?}", s.scale));
        f.u64v(s.is_user_defined as u64);
        f.f64v(s.stability);
        f.u64v(s.version as u64);
    }
    f.0
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
                .filter_map(|(id, m)| {
                    let d = m.get("date")?.as_str()?.to_string();
                    Some((id.clone(), d))
                })
                .collect()
        })
        .unwrap_or_default()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let dir = PathBuf::from(args.first().expect("corpus directory"));
    let mut limit = usize::MAX;
    let mut repeat = 1usize;
    let mut split_arg: Option<usize> = None;
    let mut adds = 30usize;
    let mut bulk = false;
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--limit" => limit = args[i + 1].parse().unwrap_or(usize::MAX),
            "--repeat" => repeat = args[i + 1].parse().unwrap_or(1),
            "--split" => split_arg = args[i + 1].parse().ok(),
            "--adds" => adds = args[i + 1].parse().unwrap_or(30),
            "--bulk" => {
                bulk = true;
                i += 1;
                continue;
            }
            _ => {}
        }
        i += 2;
    }

    let dates = load_dates(&dir);
    let mut files: Vec<PathBuf> = std::fs::read_dir(&dir)
        .expect("corpus directory")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e == "gpx"))
        .collect();
    files.sort();
    files.truncate(limit);

    let load_start = Instant::now();
    let mut unreadable = 0usize;
    let mut empty = 0usize;
    let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    for path in &files {
        if std::fs::read_to_string(path).is_err() {
            unreadable += 1;
        }
        let id = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or_default()
            .split('_')
            .next()
            .unwrap_or_default()
            .to_string();
        let pts = load_points(path);
        if pts.len() > 1 {
            tracks.push((id, pts));
        } else {
            empty += 1;
        }
    }
    let load_ms = load_start.elapsed().as_millis();
    let points: usize = tracks.iter().map(|(_, p)| p.len()).sum();

    let sports: HashMap<String, String> = tracks
        .iter()
        .map(|(id, _)| (id.clone(), POOLED.to_string()))
        .collect();
    let starts: HashMap<String, i64> = tracks
        .iter()
        .filter_map(|(id, _)| dates.get(id).and_then(|d| epoch_of(d)).map(|e| (id.clone(), e)))
        .collect();

    println!(
        "corpus {} of {} files, {} points, loaded in {} ms, {} dated, {} unreadable, {} empty",
        tracks.len(),
        files.len(),
        points,
        load_ms,
        starts.len(),
        unreadable,
        empty
    );
    println!("rss after load {} MB", peak_rss_kb() / 1024);

    let config = SectionConfig::default();

    // Warm fold: build the catalogue over all but the last `FOLD` activities,
    // then add them one at a time, the shape of a sync on a device that has
    // already detected once.
    let fold = adds.min(tracks.len().saturating_sub(1));
    if tracks.len() > fold {
        let split = split_arg
            .unwrap_or(tracks.len() - fold)
            .min(tracks.len() - 1);
        let mut cache = SectionEvidenceCache::new();
        let seed: Vec<&str> = tracks[..split].iter().map(|(id, _)| id.as_str()).collect();
        let policy = SectionUpdatePolicy::default();
        let build = Instant::now();
        let mut result = detect_sections_unified_incremental_dated(
            &mut cache,
            &[],
            &tracks[..split],
            &seed,
            &[],
            &sports,
            &starts,
            &config,
            &policy,
        );
        println!(
            "cold fold over {} activities {} ms, {} sections",
            split,
            build.elapsed().as_millis(),
            result.catalogue.len()
        );
        println!("digest cold {:016x}", catalogue_digest(&result.catalogue));
        let mut times: Vec<u128> = Vec::new();
        let steps = if bulk { 0 } else { fold.min(tracks.len() - split) };
        for i in 0..steps {
            let pool = &tracks[..split + i + 1];
            let new_id = [tracks[split + i].0.as_str()];
            let existing = result.catalogue.clone();
            let start = Instant::now();
            result = detect_sections_unified_incremental_dated(
                &mut cache,
                &existing,
                pool,
                &new_id,
                &[],
                &sports,
                &starts,
                &config,
                &policy,
            );
            let ms = start.elapsed().as_millis();
            times.push(ms);
            println!("  add {} {} ms, {} sections", i + 1, ms, result.catalogue.len());
        }
        if !times.is_empty() {
            println!("digest warm_final {:016x}", catalogue_digest(&result.catalogue));
        }
        if bulk && tracks.len() > split {
            let new_ids: Vec<&str> = tracks[split..].iter().map(|(id, _)| id.as_str()).collect();
            let existing = result.catalogue.clone();
            let start = Instant::now();
            result = detect_sections_unified_incremental_dated(
                &mut cache,
                &existing,
                &tracks,
                &new_ids,
                &[],
                &sports,
                &starts,
                &config,
                &policy,
            );
            println!("digest bulk {:016x}", catalogue_digest(&result.catalogue));
            println!(
                "bulk fold of {} new over {} existing: {} ms, {} sections, peak rss {} MB",
                new_ids.len(),
                split,
                start.elapsed().as_millis(),
                result.catalogue.len(),
                peak_rss_kb() / 1024
            );
        }
        if !times.is_empty() {
            let mut sorted = times.clone();
            sorted.sort_unstable();
            let n = times.len();
            let mean: u128 = times.iter().sum::<u128>() / n as u128;
            println!(
                "warm add mean {} ms, p50 {} ms, p95 {} ms, worst {} ms, peak rss {} MB",
                mean,
                sorted[n / 2],
                sorted[((n * 95) / 100).min(n - 1)],
                sorted[n - 1],
                peak_rss_kb() / 1024
            );
        }
    }

    for run in 1..=repeat {
        let start = Instant::now();
        let out = detect_sections_unified_dated(
            &tracks,
            &[],
            &sports,
            &starts,
            &config,
            &Tunables::DEFAULT,
        );
        let ms = start.elapsed().as_millis();
        println!(
            "run {} detect {} ms, {} sections, peak rss {} MB",
            run,
            ms,
            out.sections.len(),
            peak_rss_kb() / 1024
        );
    }
}
