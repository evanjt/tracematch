//! Locates the local GPX corpora.
//!
//! The traces are personal activity history and never enter the repository.
//! They are gitignored, and `scripts/check-no-private-data.sh` refuses to stage
//! them. Set `TRACEMATCH_CORPUS` to keep them outside the working tree
//! entirely, which is the arrangement no gitignore mistake can undo.
//!
//! Targets that need a corpus are gated behind the `real-corpus` feature, so a
//! machine without the data does not build them. Once built, a missing or empty
//! corpus is a hard failure: a test that quietly returns reports the same green
//! as one that ran.

// Each test target compiles this module separately and uses a subset of it.
#![allow(dead_code)]

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use tracematch::GpsPoint;

pub const ENV: &str = "TRACEMATCH_CORPUS";

/// Directory holding the corpus subdirectories.
pub fn root() -> PathBuf {
    match std::env::var(ENV) {
        Ok(p) if !p.trim().is_empty() => PathBuf::from(p),
        _ => PathBuf::from(env!("CARGO_MANIFEST_DIR")),
    }
}

/// Absolute path to one corpus, verified to exist and hold GPX files.
pub fn dir(name: &str) -> PathBuf {
    let path = root().join(name);

    assert!(
        path.is_dir(),
        "corpus {name:?} not found at {}\n\
         Set {ENV} to the directory holding the corpora, or drop them in the \
         crate root. Building this target without the data is what the \
         `real-corpus` feature exists to prevent.",
        path.display()
    );

    let count = gpx_paths(&path).len();
    assert!(
        count > 0,
        "corpus {name:?} at {} holds no .gpx files. An empty corpus makes every \
         assertion below vacuous.",
        path.display()
    );

    path
}

fn gpx_paths(dir: &Path) -> Vec<PathBuf> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read {}: {e}", dir.display()))
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e.eq_ignore_ascii_case("gpx")))
        .collect();
    paths.sort();
    paths
}

/// GPX paths from one corpus in a stable order, capped at `limit`.
///
/// Sorted, so a test that takes the first N takes the same N on every machine.
pub fn gpx_files(name: &str, limit: usize) -> Vec<PathBuf> {
    let dir = dir(name);
    let mut paths = gpx_paths(&dir);
    paths.truncate(limit);

    assert!(
        !paths.is_empty(),
        "corpus {name:?} yielded no files after sorting"
    );
    paths
}

/// Asserts a corpus holds at least `minimum` files before a test leans on it.
pub fn require_at_least(name: &str, minimum: usize) -> Vec<PathBuf> {
    let paths = gpx_files(name, usize::MAX);
    assert!(
        paths.len() >= minimum,
        "corpus {name:?} holds {} GPX files, this test needs {minimum}. A short \
         corpus changes what detection finds, so the thresholds below would be \
         measuring a different dataset.",
        paths.len()
    );
    paths
}

// ---------------------------------------------------------------------------
// GPX corpus loading: tracks by activity id plus the start epoch each
// `_meta.json` date gives, chronological as the files sort.
// ---------------------------------------------------------------------------

/// Track points from a GPX, tolerating a `<trkpt` whose attributes wrap onto
/// following lines.
pub fn load_points(path: &Path) -> Vec<GpsPoint> {
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

/// Every track in the named corpus with the start epoch of each dated one.
/// A corpus with fewer than `minimum` tracks is a wrong checkout, not data.
#[allow(clippy::type_complexity)]
pub fn load_tracks(
    name: &str,
    minimum: usize,
) -> (Vec<(String, Vec<GpsPoint>)>, HashMap<String, i64>) {
    let dir = dir(name);
    let dates = load_dates(&dir);
    let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    for path in gpx_files(name, usize::MAX) {
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
        tracks.len() >= minimum,
        "expected at least {minimum} tracks in {name}, got {}",
        tracks.len()
    );
    let starts = tracks
        .iter()
        .filter_map(|(id, _)| {
            dates
                .get(id)
                .and_then(|d| epoch_of(d))
                .map(|e| (id.clone(), e))
        })
        .collect();
    (tracks, starts)
}
