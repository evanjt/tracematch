//! Geometry codec sizing on real catalogue polylines (D4 storage gate).
//!
//! `section_geometry` stores versioned polylines so history can show and
//! revert re-cuts. Before the schema lands, measure what a version costs
//! on the real batch catalogues of both corpora, under: today's JSON,
//! and a quantised zigzag-varint delta stream at two precisions
//! (1e-6 deg ~= 0.11 m, 1e-5 deg ~= 1.1 m; elevation 0.1 m). Round-trip
//! error is verified against the quantisation bound. The output is the
//! bytes-per-version table and a 10-year retention extrapolation that
//! decides whether versions are stored independently (no delta chains)
//! within the ~2-4 MB budget.
//!
//!     cargo run --release --example geometry_codec -- \
//!         ~/projects/personal/intervals/tracematch/sionrunning
//!
//! GPX parsing is lifted from `examples/unified_lab.rs` (examples cannot
//! import each other's private items, so the loader is copied verbatim).

use std::collections::HashMap;
use std::path::Path;
use std::time::Instant;

use tracematch::{
    FrequentSection, GpsPoint, SectionConfig, detect_sections_unified,
    geo_utils::haversine_distance,
};

// ---------------------------------------------------------------- loading
// Copied from examples/unified_lab.rs so the labs parse identically.

struct Activity {
    id: String,
    sport: String,
    date: String,
    points: Vec<GpsPoint>,
    seconds: Vec<f64>,
}

/// Seconds since epoch zero from a per-trkpt `<time>` line. Corpus
/// exports may anonymise the date and keep real offsets, so day
/// roll-over past midnight still counts.
fn point_seconds(trimmed: &str) -> Option<f64> {
    let inner = &trimmed[trimmed.find("<time>")? + 6..trimmed.find("</time>")?];
    let day: f64 = inner.get(8..10)?.parse().ok()?;
    let h: f64 = inner.get(11..13)?.parse().ok()?;
    let m: f64 = inner.get(14..16)?.parse().ok()?;
    let s: f64 = inner.get(17..19)?.parse().ok()?;
    Some((day - 1.0) * 86_400.0 + h * 3600.0 + m * 60.0 + s)
}

/// Parse a GPX file capturing lat/lon, the `<ele>` that follows each
/// trkpt on the next line, each point's `<time>` offset, and the
/// metadata `<time>` as the activity date.
fn load_gpx_full(path: &Path) -> (Vec<GpsPoint>, Vec<f64>, String) {
    let content = match std::fs::read_to_string(path) {
        Ok(c) => c,
        Err(_) => return (Vec::new(), Vec::new(), String::new()),
    };

    let mut date = String::new();
    let mut points: Vec<GpsPoint> = Vec::new();
    let mut times: Vec<Option<f64>> = Vec::new();
    let mut pending: Option<(f64, f64)> = None;

    for line in content.lines() {
        let trimmed = line.trim();
        if date.is_empty()
            && let Some(start) = trimmed.find("<time>")
            && let Some(end) = trimmed.find("</time>")
            && start + 6 <= end
        {
            date = trimmed[start + 6..end].to_string();
        }

        if trimmed.contains("<trkpt") {
            // Flush a trkpt that never received an <ele>.
            if let Some((lat, lon)) = pending.take() {
                points.push(GpsPoint::new(lat, lon));
                times.push(None);
            }
            if let (Some(lat_start), Some(lon_start)) =
                (trimmed.find("lat=\""), trimmed.find("lon=\""))
                && let (Some(lat_end), Some(lon_end)) = (
                    trimmed[lat_start + 5..].find('"'),
                    trimmed[lon_start + 5..].find('"'),
                )
                && let (Ok(lat), Ok(lon)) = (
                    trimmed[lat_start + 5..lat_start + 5 + lat_end].parse::<f64>(),
                    trimmed[lon_start + 5..lon_start + 5 + lon_end].parse::<f64>(),
                )
            {
                pending = Some((lat, lon));
            }
        } else if let Some((lat, lon)) = pending
            && let Some(start) = trimmed.find("<ele>")
            && let Some(end) = trimmed.find("</ele>")
            && start + 5 <= end
        {
            let ele = trimmed[start + 5..end].parse::<f64>().ok();
            points.push(match ele {
                Some(e) => GpsPoint::with_elevation(lat, lon, e),
                None => GpsPoint::new(lat, lon),
            });
            times.push(None);
            pending = None;
        } else if pending.is_none()
            && let Some(t) = times.last_mut()
            && t.is_none()
            && trimmed.starts_with("<time>")
        {
            *t = point_seconds(trimmed);
        }
    }
    if let Some((lat, lon)) = pending.take() {
        points.push(GpsPoint::new(lat, lon));
        times.push(None);
    }
    let seconds = if !times.is_empty() && times.iter().all(Option::is_some) {
        times.into_iter().flatten().collect()
    } else {
        Vec::new()
    };
    (points, seconds, date)
}

/// Derive a sport type from the activity file name (Sion corpus naming,
/// English + French).
fn sport_from_name(name: &str) -> &'static str {
    let lower = name.to_lowercase();
    if lower.contains("cycl") || lower.contains("ride") || lower.contains("vélo") {
        "Ride"
    } else if lower.contains("run") || lower.contains("course") {
        "Run"
    } else if lower.contains("hik") || lower.contains("walk") || lower.contains("march") {
        "Walk"
    } else {
        "Other"
    }
}

/// Optional `_meta.json` sidecar written by the corpus scraper:
/// authoritative intervals.icu type and local start date per activity
/// id. Filename keyword inference stays as the fallback.
fn load_meta(dir: &Path) -> HashMap<String, (String, String)> {
    let Ok(content) = std::fs::read_to_string(dir.join("_meta.json")) else {
        return HashMap::new();
    };
    let Ok(v) = serde_json::from_str::<serde_json::Value>(&content) else {
        return HashMap::new();
    };
    let Some(map) = v.as_object() else {
        return HashMap::new();
    };
    map.iter()
        .map(|(id, m)| {
            let bucket = match m.get("type").and_then(|t| t.as_str()).unwrap_or("") {
                "Run" | "TrailRun" | "VirtualRun" => "Run",
                "Ride" | "GravelRide" | "MountainBikeRide" | "VirtualRide" => "Ride",
                "Walk" | "Hike" | "Snowshoe" => "Walk",
                "OpenWaterSwim" | "Swim" => "Swim",
                "Snowboard" | "AlpineSki" | "NordicSki" | "BackcountrySki" => "Snow",
                _ => "Other",
            };
            let date = m
                .get("date")
                .and_then(|d| d.as_str())
                .unwrap_or("")
                .to_string();
            (id.clone(), (bucket.to_string(), date))
        })
        .collect()
}

fn load_corpus(dir: &Path) -> Vec<Activity> {
    let meta = load_meta(dir);
    let mut activities = Vec::new();
    let entries = std::fs::read_dir(dir).expect("read_dir");
    for entry in entries.flatten() {
        let path = entry.path();
        if !path.extension().is_some_and(|e| e == "gpx") {
            continue;
        }
        let (points, seconds, date) = load_gpx_full(&path);
        if points.len() < 50 {
            continue;
        }
        let name = path
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();
        let aid = name.split('_').next().unwrap_or("");
        let (sport, date) = match meta.get(aid) {
            Some((bucket, meta_date)) => (
                bucket.clone(),
                if meta_date.is_empty() {
                    date
                } else {
                    meta_date.clone()
                },
            ),
            None => (sport_from_name(&name).to_string(), date),
        };
        activities.push(Activity {
            id: name,
            sport,
            date,
            points,
            seconds,
        });
    }
    activities.sort_by(|a, b| a.date.cmp(&b.date));
    activities
}

// ---------------------------------------------------------------- codec

fn zigzag(v: i64) -> u64 {
    ((v << 1) ^ (v >> 63)) as u64
}

fn unzigzag(v: u64) -> i64 {
    ((v >> 1) as i64) ^ -((v & 1) as i64)
}

fn write_varint(out: &mut Vec<u8>, mut v: u64) {
    loop {
        let b = (v & 0x7f) as u8;
        v >>= 7;
        if v == 0 {
            out.push(b);
            break;
        }
        out.push(b | 0x80);
    }
}

fn read_varint(bytes: &[u8], pos: &mut usize) -> Option<u64> {
    let mut v = 0u64;
    let mut shift = 0u32;
    loop {
        let b = *bytes.get(*pos)?;
        *pos += 1;
        v |= u64::from(b & 0x7f) << shift;
        if b & 0x80 == 0 {
            return Some(v);
        }
        shift += 7;
    }
}

/// Quantised polyline stream: point count, elevation flag, then per point
/// the zigzag varint of the successive quantised deltas (lat, lng at
/// `scale` counts per degree; elevation at 0.1 m when present).
fn encode_quantised(points: &[GpsPoint], scale: f64) -> Vec<u8> {
    let mut out = Vec::new();
    write_varint(&mut out, points.len() as u64);
    let has_ele = points.iter().any(|p| p.elevation.is_some());
    out.push(u8::from(has_ele));
    let (mut plat, mut plng, mut pele) = (0i64, 0i64, 0i64);
    for p in points {
        let lat = (p.latitude * scale).round() as i64;
        let lng = (p.longitude * scale).round() as i64;
        write_varint(&mut out, zigzag(lat - plat));
        write_varint(&mut out, zigzag(lng - plng));
        plat = lat;
        plng = lng;
        if has_ele {
            let e = (p.elevation.unwrap_or(0.0) * 10.0).round() as i64;
            write_varint(&mut out, zigzag(e - pele));
            pele = e;
        }
    }
    out
}

fn decode_quantised(bytes: &[u8], scale: f64) -> Option<Vec<GpsPoint>> {
    let mut pos = 0usize;
    let n = read_varint(bytes, &mut pos)? as usize;
    let has_ele = *bytes.get(pos)? != 0;
    pos += 1;
    let mut points = Vec::with_capacity(n);
    let (mut lat, mut lng, mut ele) = (0i64, 0i64, 0i64);
    for _ in 0..n {
        lat += unzigzag(read_varint(bytes, &mut pos)?);
        lng += unzigzag(read_varint(bytes, &mut pos)?);
        let elevation = if has_ele {
            ele += unzigzag(read_varint(bytes, &mut pos)?);
            Some(ele as f64 / 10.0)
        } else {
            None
        };
        points.push(GpsPoint {
            latitude: lat as f64 / scale,
            longitude: lng as f64 / scale,
            elevation,
        });
    }
    Some(points)
}

// ---------------------------------------------------------------- report

struct Sizes {
    json: usize,
    q6: usize,
    q5: usize,
    points: usize,
}

fn percentile(sorted: &[usize], p: f64) -> usize {
    if sorted.is_empty() {
        return 0;
    }
    let idx = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[idx]
}

fn main() {
    let mut args = std::env::args().skip(1);
    let Some(dir) = args.next() else {
        eprintln!("usage: geometry_codec <corpus-dir>");
        std::process::exit(2);
    };
    let dir = Path::new(&dir);

    let t_load = Instant::now();
    let activities = load_corpus(dir);
    println!(
        "corpus: {} activities from {} ({} ms load)",
        activities.len(),
        dir.display(),
        t_load.elapsed().as_millis()
    );
    if activities.is_empty() {
        eprintln!("no activities loaded; aborting");
        std::process::exit(1);
    }

    let config = SectionConfig::default();
    let sport_types: HashMap<String, String> = activities
        .iter()
        .map(|a| (a.id.clone(), a.sport.clone()))
        .collect();
    let pool: Vec<(String, Vec<GpsPoint>)> = activities
        .iter()
        .map(|a| (a.id.clone(), a.points.clone()))
        .collect();
    let seconds: Vec<&[f64]> = activities.iter().map(|a| a.seconds.as_slice()).collect();

    let t_batch = Instant::now();
    let batch: Vec<FrequentSection> =
        detect_sections_unified(&pool, &seconds, &sport_types, &config);
    println!(
        "batch: {} sections in {:.1} s",
        batch.len(),
        t_batch.elapsed().as_secs_f64()
    );

    // Per-section sizes plus the round-trip error against the bound.
    let mut sizes: Vec<Sizes> = Vec::with_capacity(batch.len());
    let mut max_err_q6 = 0.0f64;
    let mut max_err_q5 = 0.0f64;
    let mut max_ele_err = 0.0f64;
    for s in &batch {
        let json = serde_json::to_string(&s.polyline).expect("json").len();
        let e6 = encode_quantised(&s.polyline, 1e6);
        let e5 = encode_quantised(&s.polyline, 1e5);
        for (scale, bytes, max_err) in [(1e6, &e6, &mut max_err_q6), (1e5, &e5, &mut max_err_q5)] {
            let back = decode_quantised(bytes, scale).expect("decode");
            assert_eq!(back.len(), s.polyline.len());
            for (a, b) in s.polyline.iter().zip(&back) {
                *max_err = max_err.max(haversine_distance(a, b));
                if let (Some(ea), Some(eb)) = (a.elevation, b.elevation) {
                    max_ele_err = max_ele_err.max((ea - eb).abs());
                }
            }
        }
        sizes.push(Sizes {
            json,
            q6: e6.len(),
            q5: e5.len(),
            points: s.polyline.len(),
        });
    }

    let total_points: usize = sizes.iter().map(|s| s.points).sum();
    let tot = |f: fn(&Sizes) -> usize| -> usize { sizes.iter().map(f).sum() };
    let (tj, t6, t5) = (tot(|s| s.json), tot(|s| s.q6), tot(|s| s.q5));
    println!(
        "polylines: {} points total, {:.0} points/section median",
        total_points,
        {
            let mut p: Vec<usize> = sizes.iter().map(|s| s.points).collect();
            p.sort_unstable();
            percentile(&p, 0.5) as f64
        }
    );
    println!(
        "round-trip max error: q6 {:.3} m, q5 {:.3} m, elevation {:.2} m",
        max_err_q6, max_err_q5, max_ele_err
    );

    println!("\nper-version bytes (whole catalogue = one version of every section):");
    for (name, t) in [
        ("json (today)", tj),
        ("q6 (0.11 m)", t6),
        ("q5 (1.1 m)", t5),
    ] {
        let mut per: Vec<usize> = match name {
            "json (today)" => sizes.iter().map(|s| s.json).collect(),
            "q6 (0.11 m)" => sizes.iter().map(|s| s.q6).collect(),
            _ => sizes.iter().map(|s| s.q5).collect(),
        };
        per.sort_unstable();
        println!(
            "  {:14} total {:9} B ({:6.1} KB)  per-section p50 {:6} B  p90 {:6} B  max {:7} B  {:5.2} B/pt",
            name,
            t,
            t as f64 / 1024.0,
            percentile(&per, 0.5),
            percentile(&per, 0.9),
            percentile(&per, 1.0),
            t as f64 / total_points as f64,
        );
    }

    println!("\n10-year retention extrapolation (q6, per stored versions/section):");
    let per_version = t6 as f64 / batch.len().max(1) as f64;
    for kept in [3usize, 6, 10, 20] {
        let total = per_version * batch.len() as f64 * kept as f64;
        println!(
            "  keep {kept:2} versions/section -> {:7.2} MB across {} sections",
            total / (1024.0 * 1024.0),
            batch.len(),
        );
    }
}
