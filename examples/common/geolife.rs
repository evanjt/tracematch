//! GeoLife public-corpus loader for the unified-detector lab.
//!
//! Microsoft GeoLife GPS Trajectories 1.3 ships one folder per user, each with
//! `Trajectory/*.plt` logs and, for 69 of the 182 users, a `labels.txt` of
//! transport-mode spans. This loader keeps only self-powered trajectories
//! (walk, bike, run) so the section detector sees pedestrian and cycling
//! ground, never bus, car, taxi, subway, train, or plane. The point is to
//! validate the detector on trajectories that are not the author's own data.
//!
//! Fetch the data with `scripts/fetch_geolife.sh`. It lands in a gitignored
//! `geolife/` directory and is never committed.
//!
//! Dataset: Microsoft GeoLife GPS Trajectories 1.3.
//! Citation: Yu Zheng, Xing Xie, Wei-Ying Ma, "GeoLife: A Collaborative Social
//! Networking Service among User, Location and Trajectory", IEEE Data
//! Engineering Bulletin, 2010. Microsoft Research Licence (research use only,
//! no redistribution).
//!
//! `.plt` format: six header lines, then CSV rows
//! `lat,lon,0,altitude_ft,days_since_1899,YYYY-MM-DD,HH:MM:SS`. Field 0 is
//! latitude, field 1 is longitude. Altitude is unreliable in GeoLife (sentinel
//! values, feet, frequent zeroes) so it is dropped, not carried as elevation.
//! `labels.txt` format: a header line, then tab-separated rows
//! `YYYY/MM/DD HH:MM:SS \t YYYY/MM/DD HH:MM:SS \t mode` with CRLF endings.

#![allow(dead_code)]

use std::path::Path;

use tracematch::GpsPoint;

/// One self-powered GeoLife trajectory, shaped like the lab's `Activity`.
pub struct GeoTrajectory {
    /// `"<user>_<filestem>"`, unique across the pooled corpus.
    pub id: String,
    /// Veloq sport bucket: `"Walk"`, `"Ride"`, or `"Run"`.
    pub sport: String,
    /// `YYYY-MM-DD` of the first fix, so chronological replay and `day_of`
    /// parsing work exactly as they do for the GPX corpora.
    pub date: String,
    pub points: Vec<GpsPoint>,
    /// Per-point offsets in seconds from the first fix, parallel to `points`.
    /// GeoLife timestamps are reliable, so this always populates and feeds the
    /// unified detector's velocity veto.
    pub seconds: Vec<f64>,
}

/// Accounting for exactly what the subset rule loaded and dropped. Printed by
/// the caller so nothing is silently truncated.
#[derive(Default)]
pub struct GeoStats {
    pub users_with_labels: usize,
    pub users_scanned: usize,
    pub users_loaded: usize,
    pub walk: usize,
    pub bike: usize,
    pub run: usize,
    pub skipped_short: usize,
    pub skipped_huge: usize,
    pub skipped_unlabelled: usize,
    pub skipped_motorised: usize,
}

/// Trajectories longer than this are skipped as pathological (some GeoLife
/// logs run to ~90k points across many stationary hours, which would dominate
/// the O(N^2) pairwise sweep). Counted in `GeoStats::skipped_huge`.
pub const DEFAULT_MAX_POINTS: usize = 50_000;

/// Days from 1970-01-01 for a proleptic-Gregorian date (Howard Hinnant's
/// algorithm, matching the lab's `day_of`). Used to put `.plt` and
/// `labels.txt` timestamps on one comparable second axis.
fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let yy = if m <= 2 { y - 1 } else { y };
    let era = if yy >= 0 { yy } else { yy - 399 } / 400;
    let yoe = yy - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    era * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy
}

fn epoch_seconds(y: i64, mo: i64, d: i64, h: i64, mi: i64, s: i64) -> i64 {
    days_from_civil(y, mo, d) * 86_400 + h * 3600 + mi * 60 + s
}

/// Parse a `YYYY-MM-DD` (dash) or `YYYY/MM/DD` (slash) date plus `HH:MM:SS`
/// time into epoch seconds. Returns None on any malformed field.
fn parse_datetime(date: &str, time: &str) -> Option<i64> {
    let sep = if date.contains('-') { '-' } else { '/' };
    let mut d = date.split(sep);
    let y: i64 = d.next()?.trim().parse().ok()?;
    let mo: i64 = d.next()?.trim().parse().ok()?;
    let da: i64 = d.next()?.trim().parse().ok()?;
    let mut t = time.split(':');
    let h: i64 = t.next()?.trim().parse().ok()?;
    let mi: i64 = t.next()?.trim().parse().ok()?;
    let s: i64 = t.next()?.trim().parse().ok()?;
    Some(epoch_seconds(y, mo, da, h, mi, s))
}

/// Load one `.plt` trajectory: track points, their absolute epoch seconds
/// (parallel to points), and the `YYYY-MM-DD` date of the first fix. Altitude
/// is deliberately ignored (see the module note). Returns None on read
/// failure or if no valid point rows are found.
fn load_plt(path: &Path) -> Option<(Vec<GpsPoint>, Vec<i64>, String)> {
    let content = std::fs::read_to_string(path).ok()?;
    let mut points = Vec::new();
    let mut seconds = Vec::new();
    let mut first_date = String::new();
    // Six header lines: "Geolife trajectory", "WGS 84", "Altitude is in Feet",
    // "Reserved 3", the field descriptor, and a bare "0".
    for line in content.lines().skip(6) {
        let mut f = line.split(',');
        let (Some(lat), Some(lon), Some(_), Some(_), Some(_), Some(date), Some(time)) = (
            f.next(),
            f.next(),
            f.next(),
            f.next(),
            f.next(),
            f.next(),
            f.next(),
        ) else {
            continue;
        };
        let (Ok(lat), Ok(lon)) = (lat.trim().parse::<f64>(), lon.trim().parse::<f64>()) else {
            continue;
        };
        let Some(secs) = parse_datetime(date, time) else {
            continue;
        };
        if first_date.is_empty() {
            first_date = date.trim().to_string();
        }
        points.push(GpsPoint::new(lat, lon));
        seconds.push(secs);
    }
    if points.is_empty() {
        return None;
    }
    Some((points, seconds, first_date))
}

/// Parse `labels.txt` into `(start_epoch_s, end_epoch_s, mode)` spans. The
/// header line and any malformed rows are skipped. CRLF endings are handled by
/// `lines()` plus `trim()`.
fn parse_labels(path: &Path) -> Vec<(i64, i64, String)> {
    let Ok(content) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let mut spans = Vec::new();
    for line in content.lines().skip(1) {
        let mut f = line.split('\t');
        let (Some(start), Some(end), Some(mode)) = (f.next(), f.next(), f.next()) else {
            continue;
        };
        let parse_span = |s: &str| -> Option<i64> {
            let mut parts = s.trim().split_whitespace();
            parse_datetime(parts.next()?, parts.next()?)
        };
        if let (Some(s), Some(e)) = (parse_span(start), parse_span(end)) {
            spans.push((s, e, mode.trim().to_lowercase()));
        }
    }
    spans
}

/// The GeoLife mode that overlaps a trajectory's `[start, end]` span for the
/// most seconds, or None when no label overlaps it at all. Ties break on the
/// smaller mode name so loading is reproducible regardless of map order.
fn dominant_mode<'a>(start: i64, end: i64, spans: &'a [(i64, i64, String)]) -> Option<&'a str> {
    let mut totals: std::collections::HashMap<&str, i64> = std::collections::HashMap::new();
    for (s, e, mode) in spans {
        let overlap = end.min(*e) - start.max(*s);
        if overlap > 0 {
            *totals.entry(mode.as_str()).or_default() += overlap;
        }
    }
    let mut best: Option<(&str, i64)> = None;
    for (mode, secs) in &totals {
        let better = match best {
            None => true,
            Some((bm, b)) => *secs > b || (*secs == b && *mode < bm),
        };
        if better {
            best = Some((mode, *secs));
        }
    }
    best.map(|(m, _)| m)
}

/// Map a GeoLife transport mode to a Veloq sport bucket. Only the self-powered
/// modes map; everything motorised returns None (excluded from the battery).
fn sport_for_mode(mode: &str) -> Option<&'static str> {
    match mode {
        "walk" => Some("Walk"),
        "bike" => Some("Ride"),
        "run" => Some("Run"),
        _ => None,
    }
}

/// Load a manageable, deterministic subset of self-powered GeoLife
/// trajectories, pooled across users.
///
/// Subset rule: users that have a `labels.txt`, sorted by folder id ascending;
/// the first `max_users` of them that yield at least one self-powered
/// trajectory. Within a user, `.plt` files are taken in filename (chronological)
/// order up to `max_per_user` kept trajectories. Each trajectory is assigned
/// the transport mode that overlaps its time span for the most seconds;
/// walk/bike/run are kept (mapped to Walk/Ride/Run), everything motorised is
/// dropped. Trajectories under 50 points or over `DEFAULT_MAX_POINTS` are
/// skipped. The pooled result is sorted chronologically so window and prefix
/// replays behave as they do for the GPX corpora.
pub fn load_geolife(
    root: &Path,
    max_users: usize,
    max_per_user: usize,
) -> (Vec<GeoTrajectory>, GeoStats) {
    let mut stats = GeoStats::default();
    let mut out = Vec::new();

    let Ok(entries) = std::fs::read_dir(root) else {
        return (out, stats);
    };
    let mut user_dirs: Vec<std::path::PathBuf> = entries
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .collect();
    user_dirs.sort();

    for user_dir in &user_dirs {
        if stats.users_loaded >= max_users {
            break;
        }
        let labels_path = user_dir.join("labels.txt");
        if !labels_path.exists() {
            continue;
        }
        stats.users_with_labels += 1;
        let spans = parse_labels(&labels_path);
        if spans.is_empty() {
            continue;
        }
        stats.users_scanned += 1;
        let user = user_dir
            .file_name()
            .map(|n| n.to_string_lossy().to_string())
            .unwrap_or_default();

        let mut plt_files: Vec<std::path::PathBuf> = std::fs::read_dir(user_dir.join("Trajectory"))
            .into_iter()
            .flatten()
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x == "plt"))
            .collect();
        plt_files.sort();

        let mut kept_for_user = 0usize;
        for plt in &plt_files {
            if kept_for_user >= max_per_user {
                break;
            }
            let Some((points, secs, date)) = load_plt(plt) else {
                continue;
            };
            if points.len() < 50 {
                stats.skipped_short += 1;
                continue;
            }
            if points.len() > DEFAULT_MAX_POINTS {
                stats.skipped_huge += 1;
                continue;
            }
            let (start, end) = (secs[0], secs[secs.len() - 1]);
            let Some(mode) = dominant_mode(start, end, &spans) else {
                stats.skipped_unlabelled += 1;
                continue;
            };
            let Some(sport) = sport_for_mode(mode) else {
                stats.skipped_motorised += 1;
                continue;
            };
            match sport {
                "Walk" => stats.walk += 1,
                "Ride" => stats.bike += 1,
                "Run" => stats.run += 1,
                _ => {}
            }
            let stem = plt.file_stem().unwrap_or_default().to_string_lossy();
            let offsets: Vec<f64> = secs.iter().map(|s| (s - start) as f64).collect();
            out.push(GeoTrajectory {
                id: format!("{}_{}", user, stem),
                sport: sport.to_string(),
                date,
                points,
                seconds: offsets,
            });
            kept_for_user += 1;
        }

        if kept_for_user > 0 {
            stats.users_loaded += 1;
        }
    }

    out.sort_by(|a, b| a.date.cmp(&b.date));
    (out, stats)
}
