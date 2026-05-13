//! End-to-end report on the sionrunning corpus.
//!
//! Loads every GPX in `sionrunning/`, runs route grouping + section
//! detection through the production paths, and prints concrete numbers:
//! quantities, timings, largest groups, largest sections.
//!
//! Run from the tracematch repo root:
//!     cargo run --release --example sion_corpus_report --features synthetic
//!
//! If the sionrunning/ directory is missing, the example exits with a
//! clear message instead of failing.

use std::collections::HashMap;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use tracematch::sections::DetectionPhase;
use tracematch::{
    DetectionProgressCallback, GpsPoint, MatchConfig, RouteSignature, SectionConfig,
};

/// Records the wall-clock duration of each detection phase.
///
/// Section detection fires `on_phase` whenever it transitions between
/// stages (BuildingRtrees → FindingOverlaps → Postprocessing). For
/// multiscale detection these fire multiple times (once per scale per
/// sport) and we aggregate the total time spent in each phase.
struct PhaseTimer {
    start: Mutex<Option<(DetectionPhase, Instant)>>,
    totals: Mutex<HashMap<&'static str, Duration>>,
    items_seen: Mutex<HashMap<&'static str, u32>>,
    items_total: Mutex<HashMap<&'static str, u32>>,
}

impl PhaseTimer {
    fn new() -> Self {
        Self {
            start: Mutex::new(None),
            totals: Mutex::new(HashMap::new()),
            items_seen: Mutex::new(HashMap::new()),
            items_total: Mutex::new(HashMap::new()),
        }
    }

    fn finalise(&self) {
        // Flush any in-flight phase by accumulating its time up to now.
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
        *self.items_total.lock().unwrap().entry(phase.as_str()).or_default() += total;
    }

    fn on_progress(&self) {
        if let Some((phase, _)) = *self.start.lock().unwrap() {
            *self.items_seen.lock().unwrap().entry(phase.as_str()).or_default() += 1;
        }
    }
}

fn load_gpx(path: &Path) -> Vec<GpsPoint> {
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

fn fmt_ms(ms: u128) -> String {
    if ms >= 1000 {
        format!("{:.2} s", ms as f64 / 1000.0)
    } else {
        format!("{} ms", ms)
    }
}

fn main() {
    let dir = Path::new("sionrunning");
    if !dir.exists() {
        eprintln!(
            "sionrunning/ not found in cwd ({}). \n\
             Run from the tracematch repo root, e.g. `cd ~/projects/personal/intervals/tracematch && cargo run --release --example sion_corpus_report --features synthetic`",
            std::env::current_dir()
                .map(|p| p.display().to_string())
                .unwrap_or_else(|_| "?".into())
        );
        return;
    }

    println!("============================================================");
    println!("  Sion corpus report");
    println!("============================================================\n");

    // --- 1. Load all GPX files ----------------------------------------
    let t_load = Instant::now();
    let entries = std::fs::read_dir(dir).expect("read_dir");
    let mut raw_tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    let mut total_points = 0usize;
    let mut skipped_short = 0usize;
    for entry in entries.flatten() {
        let path = entry.path();
        if !path.extension().is_some_and(|e| e == "gpx") {
            continue;
        }
        let pts = load_gpx(&path);
        if pts.len() < 50 {
            skipped_short += 1;
            continue;
        }
        let name = path
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();
        total_points += pts.len();
        raw_tracks.push((name, pts));
    }
    let dur_load = t_load.elapsed();
    println!(
        "## Load\n  Files loaded:        {}\n  Files skipped (<50 pts): {}\n  Total GPS points:    {} ({:.0} avg/track)\n  Time:                {}\n",
        raw_tracks.len(),
        skipped_short,
        total_points,
        total_points as f64 / raw_tracks.len().max(1) as f64,
        fmt_ms(dur_load.as_millis())
    );

    // --- 2. Build RouteSignatures ------------------------------------
    let match_config = MatchConfig::default();
    let t_sig = Instant::now();
    let mut signatures: Vec<RouteSignature> = Vec::new();
    for (name, pts) in &raw_tracks {
        if let Some(sig) = RouteSignature::from_points(name, pts, &match_config) {
            signatures.push(sig);
        }
    }
    let dur_sig = t_sig.elapsed();
    println!(
        "## Signatures\n  Signatures built:    {} (from {} tracks)\n  Time:                {}\n",
        signatures.len(),
        raw_tracks.len(),
        fmt_ms(dur_sig.as_millis())
    );

    // --- 3. Route grouping (production path) -------------------------
    let t_group = Instant::now();
    let groups = tracematch::group_signatures_parallel(&signatures, &match_config);
    let dur_group = t_group.elapsed();

    let mut group_sizes: Vec<(usize, &str)> = groups
        .iter()
        .map(|g| {
            (
                g.activity_ids.len(),
                g.activity_ids
                    .first()
                    .map(|s| s.as_str())
                    .unwrap_or("?"),
            )
        })
        .collect();
    group_sizes.sort_by(|a, b| b.0.cmp(&a.0));

    let singleton = group_sizes.iter().filter(|(n, _)| *n == 1).count();
    let max_size = group_sizes.first().map(|(n, _)| *n).unwrap_or(0);
    let median_nonsingleton = {
        let mut nonsg: Vec<usize> = group_sizes
            .iter()
            .filter(|(n, _)| *n > 1)
            .map(|(n, _)| *n)
            .collect();
        nonsg.sort();
        if nonsg.is_empty() {
            0
        } else {
            nonsg[nonsg.len() / 2]
        }
    };

    println!(
        "## Route grouping\n  Groups:              {}\n  Singletons:          {} ({:.0}%)\n  Multi-member groups: {}\n  Largest group:       {} activities\n  Median multi-group:  {} activities\n  Time:                {}\n",
        groups.len(),
        singleton,
        100.0 * singleton as f64 / groups.len().max(1) as f64,
        groups.len() - singleton,
        max_size,
        median_nonsingleton,
        fmt_ms(dur_group.as_millis())
    );

    println!("  Top 5 groups by size:");
    for (size, anchor) in group_sizes.iter().take(5) {
        println!("    {:>4}  {}", size, anchor);
    }
    println!();

    // --- 4. Section detection ----------------------------------------
    // Use a per-track sport_types map (all 'Run' for sionrunning).
    let sport_types: HashMap<String, String> = raw_tracks
        .iter()
        .map(|(id, _)| (id.clone(), "Run".to_string()))
        .collect();

    let section_config = SectionConfig::default();

    let timer = Arc::new(PhaseTimer::new());
    let t_sec = Instant::now();
    let result = tracematch::detect_sections_multiscale_with_progress(
        &raw_tracks,
        &sport_types,
        &groups,
        &section_config,
        timer.clone() as Arc<dyn DetectionProgressCallback>,
    );
    let dur_sec = t_sec.elapsed();
    timer.finalise();

    let mut sections = result.sections.clone();
    sections.sort_by(|a, b| b.activity_ids.len().cmp(&a.activity_ids.len()));

    let total_section_visits: usize = sections.iter().map(|s| s.activity_ids.len()).sum();

    // Pair funnel — how does the N*(N-1)/2 exhaustive pair set get pruned
    // by each filter stage?
    let exhaustive_pairs = raw_tracks.len() * raw_tracks.len().saturating_sub(1) / 2;
    let evaluated = timer
        .items_total
        .lock()
        .unwrap()
        .get("finding_overlaps")
        .copied()
        .unwrap_or(0) as usize;
    let median_visits = {
        let mut visits: Vec<usize> = sections.iter().map(|s| s.activity_ids.len()).collect();
        visits.sort();
        if visits.is_empty() { 0 } else { visits[visits.len() / 2] }
    };

    println!(
        "## Section detection\n  Sections:            {}\n  Overlaps found:      {} (before clustering)\n  Total visits:        {}\n  Median visits:       {}\n  Largest section:     {} visits, {:.0} m\n  Time:                {}\n",
        sections.len(),
        result.stats.overlaps_found,
        total_section_visits,
        median_visits,
        sections.first().map(|s| s.activity_ids.len()).unwrap_or(0),
        sections.first().map(|s| s.distance_meters).unwrap_or(0.0),
        fmt_ms(dur_sec.as_millis())
    );

    println!("  Pair funnel:");
    println!(
        "    Exhaustive (N*(N-1)/2):  {:>7}",
        exhaustive_pairs
    );
    println!(
        "    After grid + bbox filter: {:>7} ({:.1}% kept)",
        evaluated,
        100.0 * evaluated as f64 / exhaustive_pairs.max(1) as f64,
    );
    println!(
        "    Produced overlap:         {:>7} ({:.1}% of evaluated)",
        result.stats.overlaps_found,
        100.0 * result.stats.overlaps_found as f64 / evaluated.max(1) as f64,
    );
    println!();

    println!("  Top 5 sections by visits:");
    for s in sections.iter().take(5) {
        println!(
            "    {:>4}  {:>6.0} m  {}",
            s.activity_ids.len(),
            s.distance_meters,
            s.id
        );
    }
    println!();

    println!("  Phase breakdown (multiscale aggregates across scales × sports):");
    let totals = timer.totals.lock().unwrap();
    let items_seen = timer.items_seen.lock().unwrap();
    let items_total = timer.items_total.lock().unwrap();
    let phases = ["building_rtrees", "finding_overlaps", "postprocessing"];
    for phase in &phases {
        let dur = totals.get(phase).copied().unwrap_or_default();
        let seen = items_seen.get(phase).copied().unwrap_or(0);
        let total = items_total.get(phase).copied().unwrap_or(0);
        let pct = 100.0 * dur.as_secs_f64() / dur_sec.as_secs_f64().max(1e-9);
        println!(
            "    {:<18} {:>9}  ({:>4.1}%)  items {}/{}",
            phase,
            fmt_ms(dur.as_millis()),
            pct,
            seen,
            total
        );
    }
    println!();

    // --- 5. Bottom-line totals --------------------------------------
    let dur_total = dur_load + dur_sig + dur_group + dur_sec;
    println!("## Totals");
    println!("  Load + signatures:   {}", fmt_ms((dur_load + dur_sig).as_millis()));
    println!("  Route grouping:      {}", fmt_ms(dur_group.as_millis()));
    println!("  Section detection:   {}", fmt_ms(dur_sec.as_millis()));
    println!("  ─────────────────────────────");
    println!("  End-to-end:          {}", fmt_ms(dur_total.as_millis()));
    println!();
    println!("MatchConfig: endpoint_threshold = {} m, min_match_percentage = {}%", match_config.endpoint_threshold, match_config.min_match_percentage);
    println!("SectionConfig: proximity_threshold = {} m, min_section_length = {} m, min_activities = {}", section_config.proximity_threshold, section_config.min_section_length, section_config.min_activities);
}
