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

/// Points and per-point seconds from a GPX, tolerating a `<trkpt` whose
/// attributes wrap onto following lines.
///
/// Elevation and time are both carried because the lift veto needs both:
/// `lift_spans_tuned` returns empty below two elevated points, and it judges a
/// candidate by speed when the track is timed and by GPS jitter when it is
/// not. A loader that drops either leaves the gates silent about the veto.
///
/// The returned seconds are offsets from the first fix, parallel to the
/// points, and empty unless every point carries a time. A partial stream
/// cannot be indexed by point, and the veto reads `secs[i]` at a point index,
/// so half a stream moves the wrong samples. This is the rule
/// `lift_spans_tuned` applies to a stream whose length disagrees, and the one
/// the engine applies to a stored one.
pub fn load_track(path: &Path) -> (Vec<GpsPoint>, Vec<f64>) {
    let Ok(content) = std::fs::read_to_string(path) else {
        return (Vec::new(), Vec::new());
    };

    let mut points = Vec::new();
    let mut times: Vec<Option<f64>> = Vec::new();
    let mut from = 0usize;
    while let Some(hit) = content[from..].find("<trkpt") {
        let start = from + hit;
        // One point's block runs to the next `<trkpt`, so `<ele>` and `<time>`
        // are found wherever the exporter wrapped them, in either order, and
        // the metadata `<time>` above the first block is never in one.
        let next = content[start + 6..]
            .find("<trkpt")
            .map(|o| start + 6 + o)
            .unwrap_or(content.len());
        let block = &content[start..next];
        from = start + 6;

        let attr = |key: &str| -> Option<f64> {
            let at = block.find(key)? + key.len();
            let close = block[at..].find('"')?;
            block[at..at + close].parse().ok()
        };
        let tag = |key: &str, close: &str| -> Option<&str> {
            let at = block.find(key)? + key.len();
            let end = block[at..].find(close)?;
            Some(&block[at..at + end])
        };

        let (Some(lat), Some(lon)) = (attr("lat=\""), attr("lon=\"")) else {
            continue;
        };
        match tag("<ele>", "</ele>").and_then(|e| e.trim().parse().ok()) {
            Some(ele) => points.push(GpsPoint::with_elevation(lat, lon, ele)),
            None => points.push(GpsPoint::new(lat, lon)),
        }
        times.push(tag("<time>", "</time>").and_then(instant_of));
    }

    let seconds = match times.first().copied().flatten() {
        Some(first) if times.iter().all(Option::is_some) => {
            times.into_iter().flatten().map(|t| t - first).collect()
        }
        _ => Vec::new(),
    };
    (points, seconds)
}

/// Epoch seconds from an ISO instant, `YYYY-MM-DDTHH:MM:SS` with anything
/// after it ignored. The corpus anonymises the date to 1970-01-01 and keeps
/// real offsets, so the date still has to count: an outing past midnight rolls
/// the day rather than restarting the clock.
fn instant_of(stamp: &str) -> Option<f64> {
    let day = epoch_of(stamp)?;
    let field = |at: usize| -> Option<i64> { stamp.get(at..at + 2)?.parse().ok() };
    let (h, m, s) = (field(11)?, field(14)?, field(17)?);
    Some((day + h * 3600 + m * 60 + s) as f64)
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

/// Every track in the named corpus, with the start epoch of each dated one
/// and the per-point seconds of each timed one.
/// A corpus with fewer than `minimum` tracks is a wrong checkout, not data.
#[allow(clippy::type_complexity)]
pub fn load_tracks(
    name: &str,
    minimum: usize,
) -> (
    Vec<(String, Vec<GpsPoint>)>,
    HashMap<String, i64>,
    HashMap<String, Vec<f64>>,
) {
    let dir = dir(name);
    let dates = load_dates(&dir);
    let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    let mut seconds: HashMap<String, Vec<f64>> = HashMap::new();
    for path in gpx_files(name, usize::MAX) {
        let id = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or_default()
            .split('_')
            .next()
            .unwrap_or_default()
            .to_string();
        let (pts, secs) = load_track(&path);
        if pts.len() > 1 {
            if secs.len() == pts.len() {
                seconds.insert(id.clone(), secs);
            }
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
    (tracks, starts, seconds)
}

// ---------------------------------------------------------------------------
// Loader tests. The corpus GPX carries `<ele>` and a per-point `<time>`, and
// the lift veto is inert without both: `lift_spans_tuned` returns empty when
// fewer than two points are elevated, and falls back to the jitter estimate
// when the track is untimed. A loader that drops either makes every gate
// below it vacuous about the veto, the same way an empty corpus does.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const GPX: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="tracematch" xmlns="http://www.topografix.com/GPX/1/1">
  <metadata>
    <name>Morning Run</name>
    <time>2025-12-19T06:23:01</time>
  </metadata>
  <trk>
    <trkseg>
      <trkpt lat="-33.739803" lon="151.005000">
        <ele>120.8</ele>
        <time>1970-01-01T00:00:00+00:00</time>
      </trkpt>
      <trkpt lat="-33.739800" lon="151.004990">
        <ele>121.0</ele>
        <time>1970-01-01T00:00:04+00:00</time>
      </trkpt>
      <trkpt lat="-33.739796" lon="151.004980">
        <ele>121.4</ele>
        <time>1970-01-01T00:01:07+00:00</time>
      </trkpt>
    </trkseg>
  </trk>
</gpx>
"#;

    fn write(name: &str, body: &str) -> PathBuf {
        let path = std::env::temp_dir().join(format!("tracematch-corpus-{name}.gpx"));
        std::fs::write(&path, body).expect("write fixture");
        path
    }

    #[test]
    fn a_loaded_track_carries_the_elevation_and_time_the_gpx_holds() {
        let (points, seconds) = load_track(&write("full", GPX));

        assert_eq!(points.len(), 3);
        assert_eq!(
            points.iter().map(|p| p.elevation).collect::<Vec<_>>(),
            vec![Some(120.8), Some(121.0), Some(121.4)],
            "the veto needs two elevated points before it looks at anything else"
        );
        assert_eq!(
            seconds,
            vec![0.0, 4.0, 67.0],
            "seconds are offsets from the first fix, parallel to the points"
        );
    }

    #[test]
    fn a_track_missing_one_point_time_is_loaded_untimed_rather_than_misaligned() {
        // A partial stream cannot be indexed by point, and the veto's speed
        // windows read `secs[i]` at a point index. Dropping the whole stream
        // costs the velocity veto; keeping part of it moves the wrong points.
        let body = GPX.replace("<time>1970-01-01T00:00:04+00:00</time>", "");
        let (points, seconds) = load_track(&write("partial", &body));

        assert_eq!(points.len(), 3);
        assert!(
            seconds.is_empty(),
            "a stream that does not time every point must not be returned, got {seconds:?}"
        );
    }

    #[test]
    fn a_track_with_no_elevation_still_loads_its_points_and_times() {
        let body: String = GPX
            .lines()
            .filter(|l| !l.trim_start().starts_with("<ele>"))
            .collect::<Vec<_>>()
            .join("\n");
        let (points, seconds) = load_track(&write("flat", &body));

        assert_eq!(points.len(), 3);
        assert!(points.iter().all(|p| p.elevation.is_none()));
        assert_eq!(seconds, vec![0.0, 4.0, 67.0]);
    }

    #[test]
    fn the_metadata_time_is_not_mistaken_for_a_point_time() {
        // `<metadata><time>` dates the activity and is not a fix. Counting it
        // would push every point's offset one place along.
        let (_, seconds) = load_track(&write("meta", GPX));
        assert_eq!(seconds.first().copied(), Some(0.0));
    }
}
