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

use std::collections::HashMap;
use std::path::Path;

use bitwise::{Corpus, Shape};
use tracematch::GpsPoint;

const CORPUS: &str = "fullcorpus";
const GOLDEN: &str = "_bitwise_golden.txt";

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
