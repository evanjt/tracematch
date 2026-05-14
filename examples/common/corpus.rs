//! Shared helpers for corpus-based example reports.
//!
//! Loads GPX files, times detection phases via the
//! `DetectionProgressCallback` trait, formats durations. Used by
//! `corpus_report.rs` and `density_grid_report.rs`.

#![allow(dead_code)]

use std::collections::HashMap;
use std::path::Path;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use tracematch::DetectionProgressCallback;
use tracematch::GpsPoint;
use tracematch::sections::DetectionPhase;

/// Records the wall-clock duration of each detection phase.
///
/// Section detection fires `on_phase` whenever it transitions between
/// stages (BuildingRtrees → FindingOverlaps → Postprocessing). For
/// multiscale detection these fire multiple times (once per scale per
/// sport) and we aggregate the total time spent in each phase.
pub struct PhaseTimer {
    start: Mutex<Option<(DetectionPhase, Instant)>>,
    pub totals: Mutex<HashMap<&'static str, Duration>>,
    pub items_seen: Mutex<HashMap<&'static str, u32>>,
    pub items_total: Mutex<HashMap<&'static str, u32>>,
}

impl PhaseTimer {
    pub fn new() -> Self {
        Self {
            start: Mutex::new(None),
            totals: Mutex::new(HashMap::new()),
            items_seen: Mutex::new(HashMap::new()),
            items_total: Mutex::new(HashMap::new()),
        }
    }

    pub fn finalise(&self) {
        if let Some((phase, started)) = self.start.lock().unwrap().take() {
            let mut totals = self.totals.lock().unwrap();
            *totals.entry(phase.as_str()).or_default() += started.elapsed();
        }
    }
}

impl DetectionProgressCallback for PhaseTimer {
    fn on_phase(&self, phase: DetectionPhase, total: u32) {
        let now = Instant::now();
        let mut start = self.start.lock().unwrap();
        if let Some((prev_phase, prev_started)) = start.take() {
            let mut totals = self.totals.lock().unwrap();
            *totals.entry(prev_phase.as_str()).or_default() += now.duration_since(prev_started);
        }
        *start = Some((phase, now));
        *self
            .items_total
            .lock()
            .unwrap()
            .entry(phase.as_str())
            .or_default() += total;
    }

    fn on_progress(&self) {
        if let Some((phase, _)) = *self.start.lock().unwrap() {
            *self
                .items_seen
                .lock()
                .unwrap()
                .entry(phase.as_str())
                .or_default() += 1;
        }
    }
}

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
