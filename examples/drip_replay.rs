//! Chronological one-at-a-time drip replay over a real GPX corpus.
//!
//! Answers the drip-vs-bulk skepticism empirically: feed activities in
//! date order through [`detect_sections_unified_incremental_cached`],
//! record each fold's added/dissolved/changed counts and the geometry
//! deltas, then run one [`detect_sections_unified`] batch over the same
//! pool and compare the final catalogues on count, endpoints and length.
//!
//!     cargo run --release --example drip_replay -- \
//!         ~/projects/personal/intervals/tracematch/sionrunning
//!
//! GPX parsing is lifted from `examples/unified_lab.rs` (examples cannot
//! import each other's private items, so the loader is copied verbatim).
//! Ordering: `_meta.json` local start date when present, else the GPX
//! metadata `<time>` tag; ties keep directory order.

use std::collections::HashMap;
use std::path::Path;
use std::time::Instant;

use tracematch::{
    FrequentSection, GpsPoint, SectionConfig, SectionEvidenceCache, detect_sections_unified,
    detect_sections_unified_incremental_cached, geo_utils::haversine_distance,
};

// ---------------------------------------------------------------- loading
// Copied from examples/unified_lab.rs so the two labs parse identically.

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
    // Chronological, so the drip replays history in arrival order.
    activities.sort_by(|a, b| a.date.cmp(&b.date));
    activities
}

// ---------------------------------------------------------------- compare

/// Best-orientation worst-endpoint distance between two sections, in
/// metres. Takes the better of forward and reversed pairings.
fn endpoint_distance(a: &FrequentSection, b: &FrequentSection) -> f64 {
    let (Some(af), Some(al), Some(bf), Some(bl)) = (
        a.polyline.first(),
        a.polyline.last(),
        b.polyline.first(),
        b.polyline.last(),
    ) else {
        return f64::INFINITY;
    };
    let fwd = haversine_distance(af, bf).max(haversine_distance(al, bl));
    let rev = haversine_distance(af, bl).max(haversine_distance(al, bf));
    fwd.min(rev)
}

/// Relative length difference against the longer of the two.
fn length_rel_diff(a: &FrequentSection, b: &FrequentSection) -> f64 {
    let m = a.distance_meters.max(b.distance_meters);
    if m <= 0.0 {
        return 0.0;
    }
    (a.distance_meters - b.distance_meters).abs() / m
}

// ---------------------------------------------------------------- main

fn main() {
    let mut args = std::env::args().skip(1);
    let Some(dir) = args.next() else {
        eprintln!("usage: drip_replay <corpus-dir>");
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
    println!(
        "order: chronological by activity date ({} .. {})",
        activities.first().unwrap().date,
        activities.last().unwrap().date
    );

    let config = SectionConfig::default();
    let sport_types: HashMap<String, String> = activities
        .iter()
        .map(|a| (a.id.clone(), a.sport.clone()))
        .collect();

    // ---------------------------------------------------------- drip
    let mut cache = SectionEvidenceCache::new();
    let mut catalogue: Vec<FrequentSection> = Vec::new();
    let mut pool: Vec<(String, Vec<GpsPoint>)> = Vec::with_capacity(activities.len());
    let mut secs_owned: Vec<Vec<f64>> = Vec::with_capacity(activities.len());

    let mut step_ms: Vec<f64> = Vec::with_capacity(activities.len());
    let mut tot_added = 0usize;
    let mut tot_dissolved = 0usize;
    let mut tot_changed = 0usize;
    // Per step: (step, added, dissolved, changed) for the age histogram.
    let mut per_step: Vec<(usize, usize, usize, usize)> = Vec::with_capacity(activities.len());

    let t_drip = Instant::now();
    for (step, act) in activities.iter().enumerate() {
        pool.push((act.id.clone(), act.points.clone()));
        secs_owned.push(act.seconds.clone());
        let seconds: Vec<&[f64]> = secs_owned.iter().map(|s| s.as_slice()).collect();
        let new_ids = [act.id.as_str()];

        let t = Instant::now();
        let res = detect_sections_unified_incremental_cached(
            &mut cache,
            &catalogue,
            &pool,
            &new_ids,
            &seconds,
            &sport_types,
            &config,
        );
        let ms = t.elapsed().as_secs_f64() * 1000.0;
        step_ms.push(ms);

        let (a, d, c) = (res.added.len(), res.dissolved.len(), res.changed.len());
        tot_added += a;
        tot_dissolved += d;
        tot_changed += c;
        per_step.push((step + 1, a, d, c));

        if a + d + c > 0 {
            println!(
                "step {:>3} [{}] {}: +{} -{} ~{} | catalogue {} | {:.0} ms",
                step + 1,
                act.date.get(..10).unwrap_or(&act.date),
                act.id,
                a,
                d,
                c,
                res.catalogue.len(),
                ms
            );
            for ch in &res.changed {
                println!(
                    "    changed {} -> {}: shift {:.1} m, length {:.1} -> {:.1} m (delta {:+.1} m), prev visits {}",
                    ch.previous.id,
                    ch.current.id,
                    ch.endpoint_shift_m,
                    ch.previous.distance_meters,
                    ch.current.distance_meters,
                    ch.length_delta_m,
                    ch.previous.visit_count
                );
            }
        }
        catalogue = res.catalogue;
    }
    let drip_total_s = t_drip.elapsed().as_secs_f64();

    // ---------------------------------------------------------- summary
    let mut sorted = step_ms.clone();
    sorted.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    let median = sorted[sorted.len() / 2];
    let max = sorted.last().copied().unwrap_or(0.0);
    println!("\n== drip summary ==");
    println!(
        "steps {} | added {} | dissolved {} | changed {} | final catalogue {}",
        activities.len(),
        tot_added,
        tot_dissolved,
        tot_changed,
        catalogue.len()
    );
    println!(
        "per-add ms: median {:.0}, max {:.0} | drip total {:.1} s",
        median, max, drip_total_s
    );

    // Churn by library age: decile buckets over the drip.
    println!("\nchurn by drip decile (steps: added/dissolved/changed):");
    let n = per_step.len();
    for dec in 0..10 {
        let lo = dec * n / 10;
        let hi = ((dec + 1) * n / 10).min(n);
        let (mut a, mut d, mut c) = (0, 0, 0);
        for &(_, sa, sd, sc) in &per_step[lo..hi] {
            a += sa;
            d += sd;
            c += sc;
        }
        println!(
            "  steps {:>3}-{:>3}: +{:<4} -{:<4} ~{}",
            lo + 1,
            hi,
            a,
            d,
            c
        );
    }

    // ---------------------------------------------------------- batch
    println!("\n== bulk batch over the same pool ==");
    let seconds: Vec<&[f64]> = secs_owned.iter().map(|s| s.as_slice()).collect();
    let t_batch = Instant::now();
    let batch = detect_sections_unified(&pool, &seconds, &sport_types, &config);
    println!(
        "batch: {} sections in {:.1} s",
        batch.len(),
        t_batch.elapsed().as_secs_f64()
    );

    // Greedy one-to-one pairing: every batch section claims its nearest
    // unclaimed drip section by endpoint distance, same sport only.
    const ENDPOINT_TOL_M: f64 = 25.0;
    const LENGTH_TOL: f64 = 0.05;
    let mut claimed = vec![false; catalogue.len()];
    let mut matched = 0usize;
    let mut shape_ok = 0usize;
    let mut worst_shift = 0.0f64;
    let mut worst_len = 0.0f64;
    let mut misses: Vec<String> = Vec::new();
    for b in &batch {
        let mut best: Option<(usize, f64)> = None;
        for (i, d) in catalogue.iter().enumerate() {
            if claimed[i] || d.sport_type != b.sport_type {
                continue;
            }
            let dist = endpoint_distance(b, d);
            if best.is_none_or(|(_, bd)| dist < bd) {
                best = Some((i, dist));
            }
        }
        match best {
            Some((i, dist)) if dist <= ENDPOINT_TOL_M * 4.0 => {
                claimed[i] = true;
                matched += 1;
                let ldiff = length_rel_diff(b, &catalogue[i]);
                worst_shift = worst_shift.max(dist);
                worst_len = worst_len.max(ldiff);
                if dist <= ENDPOINT_TOL_M && ldiff <= LENGTH_TOL {
                    shape_ok += 1;
                } else {
                    misses.push(format!(
                        "  shape drift {}: endpoints {:.1} m, length {:.1}% ({:.0} vs {:.0} m)",
                        b.id,
                        dist,
                        ldiff * 100.0,
                        b.distance_meters,
                        catalogue[i].distance_meters
                    ));
                }
            }
            _ => misses.push(format!("  batch section {} has no drip counterpart", b.id)),
        }
    }
    let drip_unclaimed = claimed.iter().filter(|c| !**c).count();

    println!("\n== drip vs bulk ==");
    println!(
        "counts: drip {} vs batch {} | paired {} | within tolerance ({} m endpoints, {}% length): {}",
        catalogue.len(),
        batch.len(),
        matched,
        ENDPOINT_TOL_M,
        LENGTH_TOL * 100.0,
        shape_ok
    );
    println!(
        "worst paired endpoint shift {:.1} m | worst paired length diff {:.2}% | drip-only sections {}",
        worst_shift,
        worst_len * 100.0,
        drip_unclaimed
    );
    for m in &misses {
        println!("{}", m);
    }
    if catalogue.len() == batch.len() && shape_ok == batch.len() {
        println!("RESULT: drip == bulk on count and shape");
    } else {
        println!("RESULT: drip != bulk (see divergences above)");
    }
}
