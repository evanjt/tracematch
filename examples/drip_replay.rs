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

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::Path;
use std::time::Instant;

use tracematch::{
    CandidateFate, CandidateSection, FrequentSection, GpsPoint, HysteresisParams, HysteresisState,
    SectionConfig, SectionEvidenceCache, detect_sections_unified_dated,
    detect_sections_unified_incremental_dated,
    geo_utils::haversine_distance,
    matching::{calculate_route_distance, resample_route},
    mutual_overlap,
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
/// Civil date ("YYYY-MM-DD...") to a day count (Hinnant's algorithm),
/// then to epoch seconds; date-only strings land at midday.
fn epoch_of(date: &str) -> Option<i64> {
    let y: i64 = date.get(0..4)?.parse().ok()?;
    if y < 2000 {
        // Anonymised exports pin the date to 1970: not a real start, so
        // the activity counts as its own occasion rather than joining
        // every other anonymised file in one giant fake trip.
        return None;
    }
    let m: i64 = date.get(5..7)?.parse().ok()?;
    let d: i64 = date.get(8..10)?.parse().ok()?;
    let yy = if m <= 2 { y - 1 } else { y };
    let era = if yy >= 0 { yy } else { yy - 399 } / 400;
    let yoe = yy - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let days = era * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let tod = (|| {
        let h: i64 = date.get(11..13)?.parse().ok()?;
        let mi: i64 = date.get(14..16)?.parse().ok()?;
        let s: i64 = date.get(17..19)?.parse().ok()?;
        Some(h * 3600 + mi * 60 + s)
    })()
    .unwrap_or(12 * 3600);
    Some(days * 86400 + tod)
}

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

/// `endpoint_distance` on bare polylines: best-orientation worst-endpoint
/// distance in metres.
fn endpoint_shift_poly(a: &[GpsPoint], b: &[GpsPoint]) -> f64 {
    let (Some(af), Some(al), Some(bf), Some(bl)) = (a.first(), a.last(), b.first(), b.last())
    else {
        return f64::INFINITY;
    };
    let fwd = haversine_distance(af, bf).max(haversine_distance(al, bl));
    let rev = haversine_distance(af, bl).max(haversine_distance(al, bf));
    fwd.min(rev)
}

/// Resample a catalogue into hysteresis candidates at ~50 m spacing,
/// capped at 400 points, mirroring the registry's per-apply feed.
fn to_candidates(sections: &[FrequentSection]) -> Vec<CandidateSection> {
    sections
        .iter()
        .map(|s| {
            let t = ((s.distance_meters / 50.0).ceil() as usize).clamp(2, 400);
            CandidateSection {
                polyline: resample_route(&s.polyline, t),
                visit_count: s.visit_count,
            }
        })
        .collect()
}

/// Median and p90/max over a sorted-on-demand copy.
fn dist_stats(values: &[f64]) -> (f64, f64, f64) {
    if values.is_empty() {
        return (0.0, 0.0, 0.0);
    }
    let mut v = values.to_vec();
    v.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    let med = v[v.len() / 2];
    let p90 = v[((v.len() * 9) / 10).min(v.len() - 1)];
    (med, p90, *v.last().unwrap())
}

/// One drifted adoption in the visible layer. Carries both geometries so
/// the reviewer artefacts can render the move, not just measure it.
struct DriftRow {
    step: usize,
    date: String,
    id: String,
    class: &'static str,
    mutual: f64,
    shift_m: f64,
    len_prev_m: f64,
    len_new_m: f64,
    prev_poly: Vec<GpsPoint>,
    new_poly: Vec<GpsPoint>,
}

/// A GeoJSON LineString feature with the given properties JSON fragment.
fn line_feature(poly: &[GpsPoint], props: &str) -> String {
    let coords: Vec<String> = poly
        .iter()
        .map(|p| format!("[{:.6},{:.6}]", p.longitude, p.latitude))
        .collect();
    format!(
        "{{\"type\":\"Feature\",\"properties\":{{{}}},\"geometry\":{{\"type\":\"LineString\",\"coordinates\":[{}]}}}}",
        props,
        coords.join(",")
    )
}

fn write_geojson(path: &Path, features: &[String]) {
    let body = format!(
        "{{\"type\":\"FeatureCollection\",\"features\":[\n{}\n]}}\n",
        features.join(",\n")
    );
    match std::fs::write(path, body) {
        Ok(()) => println!("wrote {} features -> {}", features.len(), path.display()),
        Err(e) => eprintln!("geojson write failed for {}: {e}", path.display()),
    }
}

/// Shift-ranked review selection per class: the `top` largest shifts plus an
/// even-stride `sample` across the rest, so the reviewer sees the tail AND
/// the typical case, not only the tail.
fn select_review_rows<'a>(rows: &[&'a DriftRow], top: usize, sample: usize) -> Vec<&'a DriftRow> {
    let mut sorted: Vec<&DriftRow> = rows.to_vec();
    sorted.sort_by(|a, b| {
        b.shift_m
            .partial_cmp(&a.shift_m)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut picked: Vec<&DriftRow> = sorted.iter().take(top).copied().collect();
    let rest = &sorted[picked.len().min(sorted.len())..];
    if !rest.is_empty() && sample > 0 {
        let stride = (rest.len() / sample).max(1);
        picked.extend(rest.iter().step_by(stride).take(sample).copied());
    }
    picked
}

// ---------------------------------------------------------------- main

fn main() {
    let (flags, positional): (Vec<String>, Vec<String>) =
        std::env::args().skip(1).partition(|a| a.starts_with("--"));
    let pooled = flags.iter().any(|f| f == "--pooled");
    let mut args = positional.into_iter();
    let Some(dir) = args.next() else {
        eprintln!("usage: drip_replay <corpus-dir> [suite-name] [--pooled]");
        std::process::exit(2);
    };
    let suite_arg = args.next();
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
    // Pooled is the product ruling (a trace is a trace); per-sport remains
    // the diagnostic mode for isolating cross-sport candidate duplication.
    let sport_types: HashMap<String, String> = activities
        .iter()
        .map(|a| {
            let sport = if pooled {
                "All".to_string()
            } else {
                a.sport.clone()
            };
            (a.id.clone(), sport)
        })
        .collect();
    if pooled {
        println!("mode: pooled (all sports detect together)");
    }

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

    // ------------------------------------------------- visible layer
    // The registry mirrors the pure hysteresis layer's fates verbatim, so
    // the user-visible catalogue over this drip IS HysteresisState fed each
    // step's raw catalogue. Every adopted carry is classified against the
    // recut agreement threshold: stable (geometry unchanged), agreement
    // (high-overlap re-pick adopted immediately), fired (a debounced
    // re-cut snapping at k). Frozen carries show no drift by definition.
    let hyst = HysteresisParams::default();
    let mut vis_state = HysteresisState::new(hyst);
    let mut drift_rows: Vec<DriftRow> = Vec::new();
    let (mut n_stable, mut n_agree, mut n_fired, mut n_frozen) = (0usize, 0usize, 0usize, 0usize);
    let (mut n_minted, mut n_restored) = (0usize, 0usize);
    let mut n_recut_applied = 0usize;
    let mut dissolve_pending_steps = 0usize;
    // Pending kind observed per id (1 = re-cut, 2 = dissolve) and the
    // count of 1<->2 flips, the streak-reset pathology the flicker gate
    // is waiting on corpus evidence for.
    let mut pend_kind: BTreeMap<String, u8> = BTreeMap::new();
    let mut kind_flips: BTreeMap<String, usize> = BTreeMap::new();
    let mut vis_ms_total = 0.0f64;

    let t_drip = Instant::now();
    let mut pool_epochs: std::collections::HashMap<String, i64> = std::collections::HashMap::new();
    for (step, act) in activities.iter().enumerate() {
        pool.push((act.id.clone(), act.points.clone()));
        if let Some(e) = epoch_of(&act.date) {
            pool_epochs.insert(act.id.clone(), e);
        }
        secs_owned.push(act.seconds.clone());
        let seconds: Vec<&[f64]> = secs_owned.iter().map(|s| s.as_slice()).collect();
        let new_ids = [act.id.as_str()];

        let t = Instant::now();
        let res = detect_sections_unified_incremental_dated(
            &mut cache,
            &catalogue,
            &pool,
            &new_ids,
            &seconds,
            &sport_types,
            &pool_epochs,
            &config,
            &tracematch::SectionUpdatePolicy::default(),
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
        // Visible-layer fold over this step's raw catalogue.
        let t_vis = Instant::now();
        let prev_vis: BTreeMap<String, Vec<GpsPoint>> =
            vis_state.visible_grounds().into_iter().collect();
        let cands = to_candidates(&res.catalogue);
        let (o, resolutions) = vis_state.step_assign(&cands);
        n_recut_applied += o.recut_applied;
        let resolved: BTreeSet<&str> = resolutions.iter().map(|r| r.id.as_str()).collect();
        let mut step_kind: BTreeMap<String, u8> = BTreeMap::new();
        for (j, r) in resolutions.iter().enumerate() {
            match r.fate {
                CandidateFate::CarriedFrozen => {
                    n_frozen += 1;
                    step_kind.insert(r.id.clone(), 1);
                }
                CandidateFate::Minted => n_minted += 1,
                CandidateFate::Restored => n_restored += 1,
                CandidateFate::CarriedAdopted => {
                    let Some(prev) = prev_vis.get(&r.id) else {
                        continue;
                    };
                    let shift = endpoint_shift_poly(prev, &cands[j].polyline);
                    let len_prev = calculate_route_distance(prev);
                    let len_new = calculate_route_distance(&cands[j].polyline);
                    if shift < 1.0 && (len_prev - len_new).abs() < 1.0 {
                        n_stable += 1;
                        continue;
                    }
                    let mu = mutual_overlap(prev, &cands[j].polyline);
                    let class = if mu >= hyst.recut_agreement {
                        n_agree += 1;
                        "agreement"
                    } else {
                        n_fired += 1;
                        "fired"
                    };
                    println!(
                        "    visible {} {}: shift {:.1} m, length {:.0} -> {:.0} m (mutual {:.2})",
                        class, r.id, shift, len_prev, len_new, mu
                    );
                    drift_rows.push(DriftRow {
                        step: step + 1,
                        date: act.date.get(..10).unwrap_or(&act.date).to_string(),
                        id: r.id.clone(),
                        class,
                        mutual: mu,
                        shift_m: shift,
                        len_prev_m: len_prev,
                        len_new_m: len_new,
                        prev_poly: prev.clone(),
                        new_poly: cands[j].polyline.clone(),
                    });
                }
            }
        }
        for id in vis_state.visible_ids() {
            if !resolved.contains(id.as_str()) && prev_vis.contains_key(&id) {
                dissolve_pending_steps += 1;
                step_kind.insert(id, 2);
            }
        }
        // Kind flips: a pending id whose kind changed between consecutive
        // steps resets its streak in the engine; count those transitions.
        for (id, kind) in &step_kind {
            if let Some(prev_kind) = pend_kind.get(id)
                && *prev_kind != *kind
            {
                *kind_flips.entry(id.clone()).or_insert(0) += 1;
            }
        }
        pend_kind = step_kind;
        vis_ms_total += t_vis.elapsed().as_secs_f64() * 1000.0;

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

    // ------------------------------------------------- visible summary
    println!("\n== visible layer (identity + hysteresis, D2 adoption) ==");
    println!(
        "carries: stable {} | agreement-adopt {} | fired re-cut {} (engine recut_applied {}) | frozen {}",
        n_stable, n_agree, n_fired, n_recut_applied, n_frozen
    );
    println!(
        "minted {} | restored {} | dissolve-pending id-steps {} | visible now {} | overhead median-free total {:.1} s",
        n_minted,
        n_restored,
        dissolve_pending_steps,
        vis_state.visible_len(),
        vis_ms_total / 1000.0
    );
    let agree_shifts: Vec<f64> = drift_rows
        .iter()
        .filter(|r| r.class == "agreement")
        .map(|r| r.shift_m)
        .collect();
    let agree_lens: Vec<f64> = drift_rows
        .iter()
        .filter(|r| r.class == "agreement")
        .map(|r| (r.len_new_m - r.len_prev_m).abs())
        .collect();
    let fired_shifts: Vec<f64> = drift_rows
        .iter()
        .filter(|r| r.class == "fired")
        .map(|r| r.shift_m)
        .collect();
    let fired_lens: Vec<f64> = drift_rows
        .iter()
        .filter(|r| r.class == "fired")
        .map(|r| (r.len_new_m - r.len_prev_m).abs())
        .collect();
    let (am, a9, ax) = dist_stats(&agree_shifts);
    let (alm, al9, alx) = dist_stats(&agree_lens);
    println!(
        "agreement-adopt drift ({}): endpoint shift med {:.0} / p90 {:.0} / max {:.0} m | length delta med {:.0} / p90 {:.0} / max {:.0} m",
        agree_shifts.len(),
        am,
        a9,
        ax,
        alm,
        al9,
        alx
    );
    let (fm, f9, fx) = dist_stats(&fired_shifts);
    let (flm, fl9, flx) = dist_stats(&fired_lens);
    println!(
        "fired re-cut drift ({}): endpoint shift med {:.0} / p90 {:.0} / max {:.0} m | length delta med {:.0} / p90 {:.0} / max {:.0} m",
        fired_shifts.len(),
        fm,
        f9,
        fx,
        flm,
        fl9,
        flx
    );
    let total_flips: usize = kind_flips.values().sum();
    let max_flips = kind_flips.values().copied().max().unwrap_or(0);
    println!(
        "pending kind flips: total {} across {} ids (max per id {})",
        total_flips,
        kind_flips.len(),
        max_flips
    );
    for (id, n) in &kind_flips {
        println!("    kind-flip {}: {} transitions", id, n);
    }

    // ---------------------------------------------------------- batch
    println!("\n== bulk batch over the same pool ==");
    let seconds: Vec<&[f64]> = secs_owned.iter().map(|s| s.as_slice()).collect();
    let t_batch = Instant::now();
    let batch = detect_sections_unified_dated(
        &pool,
        &seconds,
        &sport_types,
        &pool_epochs,
        &config,
        &tracematch::Tunables::DEFAULT,
    )
    .sections;
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

    // ------------------------------------------- visible vs bulk batch
    // A real drip keeps detecting, so drain the pending debounces by
    // feeding the final catalogue until the visible set stops changing
    // (bounded at 20 steps), then pair the settled visible grounds
    // against the batch on geometry alone. The trajectory shows whether
    // stale corridors drain slowly (pairing reshuffles reset streaks as
    // competitors retire) or pin permanently. Lengths are compared in
    // resampled space on both sides so the resampling bias cancels.
    let final_cands = to_candidates(&catalogue);
    let mut settle_dissolved = 0usize;
    let mut settle_recut = 0usize;
    let mut settle_steps = 0usize;
    let mut trajectory: Vec<usize> = vec![vis_state.visible_len()];
    // Per settle step, every id's fate: how a stale corridor evades the
    // debounce is only visible here, where the candidate side is static.
    let mut settle_rows: Vec<String> =
        vec!["step,id,event,cand_idx,cand_sport,cand_len_m,reason".to_string()];
    for _ in 0..20 {
        let before_ids: BTreeSet<String> = vis_state.visible_ids().into_iter().collect();
        let (o, resolutions) = vis_state.step_assign(&final_cands);
        let step_no = settle_steps + 1;
        let mut resolved: BTreeSet<&str> = BTreeSet::new();
        for (j, r) in resolutions.iter().enumerate() {
            resolved.insert(r.id.as_str());
            let event = match r.fate {
                CandidateFate::CarriedAdopted => "carry-adopted",
                CandidateFate::CarriedFrozen => "carry-frozen",
                CandidateFate::Minted => "minted",
                CandidateFate::Restored => "restored",
            };
            settle_rows.push(format!(
                "{},{},{},{},{},{:.0},",
                step_no, r.id, event, j, catalogue[j].sport_type, catalogue[j].distance_meters
            ));
        }
        for r in &o.retired {
            settle_rows.push(format!(
                "{},{},retire-fired,,,,{:?}",
                step_no, r.id, r.reason
            ));
        }
        let fired: BTreeSet<&str> = o.retired.iter().map(|r| r.id.as_str()).collect();
        for id in &before_ids {
            if !resolved.contains(id.as_str()) && !fired.contains(id.as_str()) {
                settle_rows.push(format!("{},{},retire-pending,,,,", step_no, id));
            }
        }
        settle_dissolved += o.dissolved;
        settle_recut += o.recut_applied;
        settle_steps += 1;
        trajectory.push(vis_state.visible_len());
        let n = trajectory.len();
        if settle_steps >= hyst.k as usize + 2
            && n >= hyst.k as usize
            && trajectory[n - hyst.k as usize..]
                .iter()
                .all(|v| *v == trajectory[n - 1])
        {
            break;
        }
    }
    let settled = vis_state.visible_grounds();
    println!("\n== visible vs bulk batch (after {settle_steps} settle steps) ==");
    println!(
        "settle trajectory (visible count): {}",
        trajectory
            .iter()
            .map(|v| v.to_string())
            .collect::<Vec<_>>()
            .join(" -> ")
    );
    println!(
        "settle drained: dissolved {} | re-cuts fired {} | visible {} vs batch {}",
        settle_dissolved,
        settle_recut,
        settled.len(),
        batch.len()
    );
    let batch_resampled = to_candidates(&batch);
    let mut vis_claimed = vec![false; settled.len()];
    let mut vis_matched = 0usize;
    let mut vis_shape_ok = 0usize;
    let mut vis_worst_shift = 0.0f64;
    for (bi, b) in batch_resampled.iter().enumerate() {
        let mut best: Option<(usize, f64)> = None;
        for (i, (_, g)) in settled.iter().enumerate() {
            if vis_claimed[i] {
                continue;
            }
            let dist = endpoint_shift_poly(&b.polyline, g);
            if best.is_none_or(|(_, bd)| dist < bd) {
                best = Some((i, dist));
            }
        }
        if let Some((i, dist)) = best
            && dist <= ENDPOINT_TOL_M * 4.0
        {
            vis_claimed[i] = true;
            vis_matched += 1;
            vis_worst_shift = vis_worst_shift.max(dist);
            let bl = calculate_route_distance(&b.polyline);
            let vl = calculate_route_distance(&settled[i].1);
            let ldiff = if bl.max(vl) > 0.0 {
                (bl - vl).abs() / bl.max(vl)
            } else {
                0.0
            };
            if dist <= ENDPOINT_TOL_M && ldiff <= LENGTH_TOL {
                vis_shape_ok += 1;
            } else {
                println!(
                    "  visible shape drift vs batch {}: endpoints {:.1} m, length {:.1}%",
                    batch[bi].id,
                    dist,
                    ldiff * 100.0
                );
            }
        } else {
            println!(
                "  batch section {} has no visible counterpart",
                batch[bi].id
            );
        }
    }
    let vis_only = vis_claimed.iter().filter(|c| !**c).count();
    println!(
        "visible pairing: matched {} | within tolerance {} | worst endpoint {:.1} m | visible-only {}",
        vis_matched, vis_shape_ok, vis_worst_shift, vis_only
    );

    // ---------------------------------------------------------- csv
    let csv_path = dir.join("identity_drift.csv");
    let mut csv = String::from("step,date,id,class,mutual,shift_m,len_prev_m,len_new_m\n");
    for r in &drift_rows {
        csv.push_str(&format!(
            "{},{},{},{},{:.3},{:.1},{:.1},{:.1}\n",
            r.step, r.date, r.id, r.class, r.mutual, r.shift_m, r.len_prev_m, r.len_new_m
        ));
    }
    match std::fs::write(&csv_path, csv) {
        Ok(()) => println!(
            "\ndrift rows written: {} -> {}",
            drift_rows.len(),
            csv_path.display()
        ),
        Err(e) => eprintln!("csv write failed: {e}"),
    }

    // ------------------------------------------------------- geojson
    // Reviewer artefacts into a suite directory under unified-lab/
    // (gitignored, never committed), following the lab's per-round suite
    // convention (sion-a6i, full-a6i, ...): before/after pairs for a
    // shift-ranked selection of adoptions per class, the settled
    // visible-only (pinned stale) corridors with their distance to the
    // nearest batch section, and the final batch as the reference truth
    // layer. Matched features share a `pair` property.
    let corpus_name = dir
        .file_name()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "corpus".into());
    let suite = suite_arg.unwrap_or_else(|| corpus_name.clone());
    let lab_dir = Path::new("unified-lab").join(&suite);
    if let Err(e) = std::fs::create_dir_all(&lab_dir) {
        eprintln!("cannot create {}: {e}", lab_dir.display());
    }

    let agree_rows: Vec<&DriftRow> = drift_rows
        .iter()
        .filter(|r| r.class == "agreement")
        .collect();
    let fired_rows: Vec<&DriftRow> = drift_rows.iter().filter(|r| r.class == "fired").collect();
    let mut selected: Vec<&DriftRow> = select_review_rows(&agree_rows, 25, 25);
    selected.extend(select_review_rows(&fired_rows, 25, 25));

    let mut before_features: Vec<String> = Vec::with_capacity(selected.len());
    let mut after_features: Vec<String> = Vec::with_capacity(selected.len());
    for r in &selected {
        let props = format!(
            "\"pair\":\"{}@{}\",\"id\":\"{}\",\"step\":{},\"date\":\"{}\",\"class\":\"{}\",\"mutual\":{:.3},\"shift_m\":{:.1},\"len_prev_m\":{:.1},\"len_new_m\":{:.1}",
            r.id,
            r.step,
            r.id,
            r.step,
            r.date,
            r.class,
            r.mutual,
            r.shift_m,
            r.len_prev_m,
            r.len_new_m
        );
        before_features.push(line_feature(
            &r.prev_poly,
            &format!("{props},\"role\":\"before\""),
        ));
        after_features.push(line_feature(
            &r.new_poly,
            &format!("{props},\"role\":\"after\""),
        ));
    }
    write_geojson(&lab_dir.join("adoptions_before.geojson"), &before_features);
    write_geojson(&lab_dir.join("adoptions_after.geojson"), &after_features);

    let stale_features: Vec<String> = settled
        .iter()
        .enumerate()
        .filter(|(i, _)| !vis_claimed[*i])
        .map(|(_, (id, g))| {
            let nearest = batch_resampled
                .iter()
                .map(|b| endpoint_shift_poly(&b.polyline, g))
                .fold(f64::INFINITY, f64::min);
            let len = calculate_route_distance(g);
            line_feature(
                g,
                &format!(
                    "\"id\":\"{}\",\"len_m\":{:.1},\"nearest_batch_m\":{:.1}",
                    id, len, nearest
                ),
            )
        })
        .collect();
    write_geojson(&lab_dir.join("pinned_stale.geojson"), &stale_features);

    let settle_path = lab_dir.join("settle_trace.csv");
    if let Err(e) = std::fs::write(&settle_path, settle_rows.join("\n") + "\n") {
        eprintln!("settle trace write failed: {e}");
    } else {
        println!(
            "settle trace: {} rows -> {}",
            settle_rows.len() - 1,
            settle_path.display()
        );
    }

    let batch_features: Vec<String> = batch
        .iter()
        .map(|s| {
            line_feature(
                &s.polyline,
                &format!(
                    "\"id\":\"{}\",\"len_m\":{:.1},\"visits\":{}",
                    s.id, s.distance_meters, s.visit_count
                ),
            )
        })
        .collect();
    write_geojson(&lab_dir.join("batch_reference.geojson"), &batch_features);
}
