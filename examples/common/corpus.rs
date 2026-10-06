//! Shared helpers for corpus-based example reports.
//!
//! Loads GPX files and formats durations. Used by
//! `corpus_report.rs` and `density_grid_report.rs`.

#![allow(dead_code)]

use std::path::Path;

use tracematch::GpsPoint;

/// Parse a GPX file and return its track points. Returns an empty
/// Vec on read or parse failure (callers gate on `.len() < 50` etc.).
pub fn load_gpx(path: &Path) -> Vec<GpsPoint> {
    let content = match std::fs::read_to_string(path) {
        Ok(c) => c,
        Err(_) => return Vec::new(),
    };

    let mut points = Vec::new();
    for line in content.lines() {
        if !line.contains("<trkpt") {
            continue;
        }
        if let (Some(lat_start), Some(lon_start)) = (line.find("lat=\""), line.find("lon=\""))
            && let (Some(lat_end), Some(lon_end)) = (
                line[lat_start + 5..].find('"'),
                line[lon_start + 5..].find('"'),
            )
            && let (Ok(lat), Ok(lon)) = (
                line[lat_start + 5..lat_start + 5 + lat_end].parse::<f64>(),
                line[lon_start + 5..lon_start + 5 + lon_end].parse::<f64>(),
            )
        {
            points.push(GpsPoint::new(lat, lon));
        }
    }
    points
}

/// Format milliseconds in either "X ms" or "X.YY s" form.
pub fn fmt_ms(ms: u128) -> String {
    if ms >= 1000 {
        format!("{:.2} s", ms as f64 / 1000.0)
    } else {
        format!("{} ms", ms)
    }
}
