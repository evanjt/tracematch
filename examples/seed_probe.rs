//! Measures the engine's per-detect consensus seed against the fold it rides on.
//!
//! `veloqrs::persistence::sections::detection::seed_consensus_state` rebuilds a
//! `ConsensusAccumulator` for every catalogue section carrying none, on every
//! detect. Under the unified detector nothing reads the result back: the only
//! consumer is `sections::incremental`, which the unified arm never calls. This
//! replays that loop verbatim over a real catalogue so the cost is a number.
//!
//!     cargo run --release --example seed_probe -- <CORPUS_DIR> [--limit N] [--adds N]

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::Instant;

use tracematch::{
    FrequentSection, GpsPoint, SectionConfig, SectionEvidenceCache, SectionUpdatePolicy,
    sections::{
        build_accumulator_from_traces, detect_sections_unified_incremental_dated,
        extract_all_activity_traces,
    },
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

/// The body of the engine's `seed_consensus_state`, reproduced so its cost can
/// be attributed. Returns (elapsed ms, sections seeded, member-track scans).
fn seed(
    sections: &mut [FrequentSection],
    tracks: &[(String, Vec<GpsPoint>)],
    proximity_threshold: f64,
) -> (u128, usize, usize) {
    let track_map: HashMap<&str, &[GpsPoint]> = tracks
        .iter()
        .map(|(id, pts)| (id.as_str(), pts.as_slice()))
        .collect();

    let start = Instant::now();
    let (mut seeded, mut scans) = (0usize, 0usize);
    for section in sections.iter_mut() {
        if section.consensus_state.is_some() {
            continue;
        }
        if section.polyline.len() < 2 || section.activity_ids.is_empty() {
            continue;
        }
        scans += section.activity_ids.len();
        let traces_map =
            extract_all_activity_traces(&section.activity_ids, &section.polyline, &track_map);
        if traces_map.is_empty() {
            continue;
        }
        let traces: Vec<(String, Vec<GpsPoint>)> = traces_map.into_iter().collect();
        let acc = build_accumulator_from_traces(&section.polyline, &traces, proximity_threshold);
        section.consensus_state = Some(acc);
        seeded += 1;
    }
    (start.elapsed().as_millis(), seeded, scans)
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let dir = PathBuf::from(args.first().expect("corpus directory"));
    let mut limit = usize::MAX;
    let mut adds = 10usize;
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--limit" => limit = args[i + 1].parse().unwrap_or(usize::MAX),
            "--adds" => adds = args[i + 1].parse().unwrap_or(10),
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

    let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    for path in &files {
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
        }
    }

    let sports: HashMap<String, String> = tracks
        .iter()
        .map(|(id, _)| (id.clone(), POOLED.to_string()))
        .collect();
    let starts: HashMap<String, i64> = tracks
        .iter()
        .filter_map(|(id, _)| {
            dates
                .get(id)
                .and_then(|d| epoch_of(d))
                .map(|e| (id.clone(), e))
        })
        .collect();

    println!("corpus {} tracks", tracks.len());
    let config = SectionConfig::default();
    let policy = SectionUpdatePolicy::default();

    // Cold: the first detect on a fresh install, where every section is new and
    // therefore every section is seeded.
    let mut cache = SectionEvidenceCache::new();
    let ids: Vec<&str> = tracks.iter().map(|(id, _)| id.as_str()).collect();
    let t = Instant::now();
    let mut cold = detect_sections_unified_incremental_dated(
        &mut cache,
        &[],
        &tracks,
        &ids,
        &[],
        &sports,
        &starts,
        &config,
        &policy,
    )
    .catalogue;
    let fold_ms = t.elapsed().as_millis();
    let (seed_ms, seeded, scans) = seed(&mut cold, &tracks, config.proximity_threshold);
    println!(
        "cold  fold {} ms, seed {} ms ({:.0}% on top), {} sections, {} seeded, {} member scans",
        fold_ms,
        seed_ms,
        100.0 * seed_ms as f64 / fold_ms.max(1) as f64,
        cold.len(),
        seeded,
        scans
    );

    // Warm: a daily single add. The engine seeds the whole returned catalogue,
    // not just the sections the fold touched, so the cost does not shrink.
    let split = tracks.len().saturating_sub(adds);
    let mut cache = SectionEvidenceCache::new();
    let seed_ids: Vec<&str> = tracks[..split].iter().map(|(id, _)| id.as_str()).collect();
    let mut result = detect_sections_unified_incremental_dated(
        &mut cache,
        &[],
        &tracks[..split],
        &seed_ids,
        &[],
        &sports,
        &starts,
        &config,
        &policy,
    );
    let (mut fold_tot, mut seed_tot) = (0u128, 0u128);
    for n in 1..=adds.min(tracks.len() - split) {
        let upto = split + n;
        let new = [tracks[upto - 1].0.as_str()];
        let t = Instant::now();
        result = detect_sections_unified_incremental_dated(
            &mut cache,
            &result.catalogue,
            &tracks[..upto],
            &new,
            &[],
            &sports,
            &starts,
            &config,
            &policy,
        );
        fold_tot += t.elapsed().as_millis();
        let mut cat = result.catalogue.clone();
        let (ms, _, _) = seed(&mut cat, &tracks[..upto], config.proximity_threshold);
        seed_tot += ms;
    }
    let n = adds.min(tracks.len() - split).max(1) as u128;
    println!(
        "warm  fold {} ms/add, seed {} ms/add ({:.0}% on top) over {} adds at pool {}",
        fold_tot / n,
        seed_tot / n,
        100.0 * seed_tot as f64 / fold_tot.max(1) as f64,
        n,
        tracks.len()
    );
}
