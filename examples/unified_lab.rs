//! Unified-detector validation lab.
//!
//! Runs the detector over a real GPX corpus, per sport and pooled, and
//! reports the metrics that matter for the unified-detector work:
//! section counts, length distribution, per-activity load, low-visit
//! share, overlapping-pair count, runtime, and peak memory. Writes one
//! GeoJSON per sport for visual inspection.
//!
//!     cargo run --release --example unified_lab -- \
//!         ~/projects/personal/intervals/tracematch/citycorpus \
//!         --out ~/projects/personal/intervals/tracematch/unified-lab
//!
//! `--help` prints every flag. An unknown flag is rejected, so a typo
//! cannot run the default configuration and report it as the requested one.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::Instant;

use tracematch::{
    CandidateSection, Decision, FrequentSection, GpsPoint, HysteresisParams, HysteresisState,
    IdentityParams, MatchConfig, PriorSection, RouteSignature, SectionConfig, Tunables,
    geo_utils::haversine_distance, matching::resample_route, plan_identity_tuned,
};

#[path = "common/corpus.rs"]
mod corpus;
use corpus::fmt_ms;

#[path = "common/geolife.rs"]
mod geolife;

// ---------------------------------------------------------------- loading

struct Activity {
    id: String,
    sport: String,
    date: String,
    points: Vec<GpsPoint>,
    /// Per-point time offsets in seconds, parallel to `points`; empty
    /// when any point lacks a timestamp. Feeds the lift velocity veto.
    seconds: Vec<f64>,
}

/// Seconds since epoch zero from a per-trkpt `<time>` line. Corpus
/// exports anonymise the date to 1970-01-01 and keep real offsets, so
/// day roll-over past midnight still counts.
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

/// Derive a sport type from the activity file name (city corpus naming,
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

/// Optional `_meta.json` sidecar written by the corpus scraper: authoritative
/// intervals.icu type and local start date per activity id. Filename keyword
/// inference stays as the fallback for corpora without one.
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
    // Chronological, so incremental experiments can replay history.
    activities.sort_by(|a, b| a.date.cmp(&b.date));
    activities
}

fn env_usize(key: &str, default: usize) -> usize {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

/// Load the GeoLife public-corpus subset and map it into the lab's `Activity`
/// shape, so every instrument below runs on it unchanged. Prints the subset
/// rule and exactly what was kept or dropped, so nothing is silently truncated.
fn load_geolife_activities(root: &Path, max_users: usize, max_per_user: usize) -> Vec<Activity> {
    let (trajectories, s) = geolife::load_geolife(root, max_users, max_per_user);
    println!(
        "GeoLife corpus: {} → {} self-powered trajectories from {} users \
         (walk {}, bike {}, run {})",
        root.display(),
        trajectories.len(),
        s.users_loaded,
        s.walk,
        s.bike,
        s.run,
    );
    println!(
        "  subset rule: labelled users sorted by id, first {} with self-powered data \
         ({} labelled users seen); <= {} earliest trajectories/user; per-trajectory dominant \
         labelled mode by time overlap; walk->Walk bike->Ride run->Run, motorised excluded; \
         points 50..={}",
        max_users,
        s.users_with_labels,
        max_per_user,
        geolife::DEFAULT_MAX_POINTS,
    );
    println!(
        "  dropped: {} short (<50 pts), {} huge (>{} pts), {} unlabelled span, {} motorised",
        s.skipped_short,
        s.skipped_huge,
        geolife::DEFAULT_MAX_POINTS,
        s.skipped_unlabelled,
        s.skipped_motorised,
    );
    trajectories
        .into_iter()
        .map(|t| Activity {
            id: t.id,
            sport: t.sport,
            date: t.date,
            points: t.points,
            seconds: t.seconds,
        })
        .collect()
}

// ---------------------------------------------------------------- metrics

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let idx = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[idx]
}

/// Fraction of `a`'s resampled points within `threshold` metres of any of
/// `b`'s resampled points. cos(lat)-corrected planar distance.
fn containment(a: &[GpsPoint], b: &[GpsPoint], threshold_m: f64) -> f64 {
    if a.is_empty() || b.is_empty() {
        return 0.0;
    }
    let ref_lat = a[0].latitude.to_radians();
    let m_per_deg_lat = 111_000.0;
    let m_per_deg_lng = 111_000.0 * ref_lat.cos();
    let thr2 = threshold_m * threshold_m;
    let mut hits = 0usize;
    for p in a {
        let mut near = false;
        for q in b {
            let dx = (p.longitude - q.longitude) * m_per_deg_lng;
            let dy = (p.latitude - q.latitude) * m_per_deg_lat;
            if dx * dx + dy * dy <= thr2 {
                near = true;
                break;
            }
        }
        if near {
            hits += 1;
        }
    }
    hits as f64 / a.len() as f64
}

/// Point cloud of a whole catalogue's polylines in a hash grid, for
/// corridor-level coverage queries.
///
/// Corridor persistence asks the detector's own backoff question (rule
/// 6): is this way already represented by the other catalogue's
/// polylines, at the backoff's lateral tolerance of one partition
/// cell? The 1:1 identity test undercounts stability wherever the
/// winner among twin candidates flips between runs, or a corridor is
/// re-cut into different pieces; the ground is still on the map either
/// way. Both numbers are reported: identity stability matters for
/// hysteresis, corridor coverage for black spots.
struct CorridorGrid {
    cells: HashMap<(i32, i32), Vec<(f64, f64)>>,
    tol_m: f64,
    m_lng: f64,
}

const M_PER_DEG_LAT: f64 = 111_000.0;

impl CorridorGrid {
    fn build(catalogue: &[Vec<GpsPoint>], tol_m: f64) -> CorridorGrid {
        let ref_lat = catalogue
            .iter()
            .flat_map(|s| s.first())
            .map(|p| p.latitude)
            .next()
            .unwrap_or(0.0);
        let m_lng = M_PER_DEG_LAT * ref_lat.to_radians().cos();
        let mut cells: HashMap<(i32, i32), Vec<(f64, f64)>> = HashMap::new();
        for s in catalogue {
            for p in s {
                let c = (
                    (p.latitude * M_PER_DEG_LAT / tol_m).floor() as i32,
                    (p.longitude * m_lng / tol_m).floor() as i32,
                );
                cells.entry(c).or_default().push((p.latitude, p.longitude));
            }
        }
        CorridorGrid {
            cells,
            tol_m,
            m_lng,
        }
    }

    fn point_covered(&self, p: &GpsPoint) -> bool {
        let tol2 = self.tol_m * self.tol_m;
        let c = (
            (p.latitude * M_PER_DEG_LAT / self.tol_m).floor() as i32,
            (p.longitude * self.m_lng / self.tol_m).floor() as i32,
        );
        for dy in -1..=1i32 {
            for dx in -1..=1i32 {
                if let Some(v) = self.cells.get(&(c.0 + dy, c.1 + dx))
                    && v.iter().any(|&(lat, lng)| {
                        let ddx = (p.longitude - lng) * self.m_lng;
                        let ddy = (p.latitude - lat) * M_PER_DEG_LAT;
                        ddx * ddx + ddy * ddy <= tol2
                    })
                {
                    return true;
                }
            }
        }
        false
    }

    fn covered_share(&self, sec: &[GpsPoint]) -> f64 {
        if sec.is_empty() {
            return 0.0;
        }
        sec.iter().filter(|p| self.point_covered(p)).count() as f64 / sec.len() as f64
    }
}

/// Fraction of a polyline's length that re-covers ground it already
/// covered (laps, doubled-back legs). A clean single-pass section ≈ 0.
fn self_overlap_frac(polyline: &[GpsPoint]) -> f64 {
    // The engine's single-pass render guard, so the geojson reports the
    // same self-revisit share the render decides on (near 20 m, gap
    // 100 m: the default-proximity corridor scale). The old last-visit
    // cell metric under-reported a tight spin (125 read 0.00).
    tracematch::self_pass_penalty(polyline, 20.0, 100.0)
}

struct MethodReport {
    method: String,
    n_sections: usize,
    len_median: f64,
    len_p90: f64,
    visits_median: f64,
    low_visit_share: f64,
    per_activity_avg: f64,
    per_activity_max: usize,
    overlap_pairs: usize,
    selfdup_count: usize,
    selfdup_max: f64,
    runtime_ms: u128,
}

fn analyse(
    method: &str,
    sections: &[FrequentSection],
    n_activities: usize,
    runtime_ms: u128,
) -> MethodReport {
    let mut lens: Vec<f64> = sections.iter().map(|s| s.distance_meters).collect();
    lens.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    let mut visits: Vec<f64> = sections.iter().map(|s| s.visit_count as f64).collect();
    visits.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    let low_visit = sections.iter().filter(|s| s.visit_count <= 2).count();

    // Per-activity section load.
    let mut per_activity: HashMap<&str, usize> = HashMap::new();
    for s in sections {
        for aid in &s.activity_ids {
            *per_activity.entry(aid.as_str()).or_default() += 1;
        }
    }
    let per_activity_max = per_activity.values().copied().max().unwrap_or(0);
    let per_activity_avg = if n_activities > 0 {
        per_activity.values().sum::<usize>() as f64 / n_activities as f64
    } else {
        0.0
    };

    // Overlapping pairs: resample every polyline to ~50m spacing, then
    // count pairs where either direction containment exceeds 30%.
    let resampled: Vec<Vec<GpsPoint>> = sections
        .iter()
        .map(|s| {
            let target = ((s.distance_meters / 50.0).ceil() as usize).clamp(2, 400);
            resample_route(&s.polyline, target)
        })
        .collect();
    let mut overlap_pairs = 0usize;
    for i in 0..resampled.len() {
        for j in (i + 1)..resampled.len() {
            let c_ij = containment(&resampled[i], &resampled[j], 60.0);
            if c_ij > 0.3 || containment(&resampled[j], &resampled[i], 60.0) > 0.3 {
                overlap_pairs += 1;
            }
            let _ = c_ij;
        }
    }

    let selfdups: Vec<f64> = sections
        .iter()
        .map(|s| self_overlap_frac(&s.polyline))
        .collect();
    let selfdup_count = selfdups.iter().filter(|&&f| f > 0.2).count();
    let selfdup_max = selfdups.iter().cloned().fold(0.0, f64::max);
    for (s, f) in sections.iter().zip(&selfdups) {
        if *f > 0.08 {
            eprintln!(
                "    [selfdup] {} {:.0}% len={:.0}m visits={} rep={}",
                s.id,
                f * 100.0,
                s.distance_meters,
                s.visit_count,
                s.representative_activity_id
            );
        }
    }

    MethodReport {
        method: method.to_string(),
        n_sections: sections.len(),
        len_median: percentile(&lens, 0.5),
        len_p90: percentile(&lens, 0.9),
        visits_median: percentile(&visits, 0.5),
        low_visit_share: if sections.is_empty() {
            0.0
        } else {
            low_visit as f64 / sections.len() as f64
        },
        per_activity_avg,
        per_activity_max,
        overlap_pairs,
        selfdup_count,
        selfdup_max,
        runtime_ms,
    }
}

// ---------------------------------------------------------------- geojson

fn geojson_for_sections(
    sections: &[FrequentSection],
    ranks: Option<&HashMap<String, RankFeatures>>,
) -> serde_json::Value {
    let features: Vec<serde_json::Value> = sections
        .iter()
        .map(|s| {
            let coords: Vec<[f64; 2]> = s
                .polyline
                .iter()
                .map(|p| [p.longitude, p.latitude])
                .collect();
            let gap_m = match (s.polyline.first(), s.polyline.last()) {
                (Some(a), Some(b)) => tracematch::geo_utils::haversine_distance(a, b),
                _ => 0.0,
            };
            let mut props = serde_json::json!({
                "id": s.id,
                "anchor": s.polyline.first().map(|p| format!("{:.4},{:.4}", p.latitude, p.longitude)),
                "visits": s.visit_count,
                "distance_m": s.distance_meters.round(),
                "confidence": s.confidence,
                "activities": s.activity_ids.len(),
                "self_overlap": (self_overlap_frac(&s.polyline) * 100.0).round() / 100.0,
                "gap_m": gap_m.round(),
                "rep": s.representative_activity_id,
            });
            if let Some(r) = ranks.and_then(|m| m.get(&s.id))
                && let Some(obj) = props.as_object_mut()
            {
                let round2 = |v: f64| (v * 100.0).round() / 100.0;
                obj.insert("score".into(), serde_json::json!(round2(r.score)));
                obj.insert("apex".into(), serde_json::json!(round2(r.apex)));
                obj.insert("grade_pct".into(), serde_json::json!(round2(r.grade)));
                obj.insert("months".into(), serde_json::json!(r.months));
                obj.insert("sinuosity".into(), serde_json::json!(round2(r.sinuosity)));
                obj.insert("converge".into(), serde_json::json!(round2(r.converge)));
                obj.insert("oneway".into(), serde_json::json!(round2(r.oneway)));
                obj.insert("recency_days".into(), serde_json::json!(r.recency_days.round()));
            }
            serde_json::json!({
                "type": "Feature",
                "geometry": { "type": "LineString", "coordinates": coords },
                "properties": props,
            })
        })
        .collect();
    serde_json::json!({ "type": "FeatureCollection", "features": features })
}

// ---------------------------------------------------------- interestingness

use tracematch::{RankCandidate, RankFeatures, RankMember, RankOuting, RankTraversal};

/// Civil date string ("YYYY-MM-DD...") to a day count (Hinnant's algorithm).
fn day_of(date: &str) -> Option<i64> {
    let y: i64 = date.get(0..4)?.parse().ok()?;
    let m: i64 = date.get(5..7)?.parse().ok()?;
    let d: i64 = date.get(8..10)?.parse().ok()?;
    let yy = if m <= 2 { y - 1 } else { y };
    let era = if yy >= 0 { yy } else { yy - 399 } / 400;
    let yoe = yy - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    Some(era * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy)
}

/// Civil timestamp ("YYYY-MM-DDTHH:MM:SS...") to epoch seconds; a
/// date-only string lands at midday so day gaps stay unambiguous.
fn epoch_of(date: &str) -> Option<i64> {
    if date.get(0..4)?.parse::<i64>().ok()? < 2000 {
        // Anonymised exports pin the date to 1970: not a real start, so
        // the activity counts as its own occasion rather than joining
        // every other anonymised file in one giant fake trip.
        return None;
    }
    let days = day_of(date)?;
    let tod = (|| {
        let h: i64 = date.get(11..13)?.parse().ok()?;
        let m: i64 = date.get(14..16)?.parse().ok()?;
        let s: i64 = date.get(17..19)?.parse().ok()?;
        Some(h * 3600 + m * 60 + s)
    })()
    .unwrap_or(12 * 3600);
    Some(days * 86400 + tod)
}

/// Rank the catalogue with the library ranker, passes counted by the
/// detector's own matcher.
fn rank_sections(
    sections: &[FrequentSection],
    by_id: &HashMap<&str, &Activity>,
    config: &SectionConfig,
) -> Vec<(String, RankFeatures)> {
    let outings: HashMap<String, RankOuting> = by_id
        .iter()
        .map(|(id, a)| {
            (
                id.to_string(),
                RankOuting {
                    date: Some(a.date.as_str()),
                    points: &a.points,
                },
            )
        })
        .collect();
    let candidates: Vec<RankCandidate> = sections
        .iter()
        .map(|s| RankCandidate {
            id: &s.id,
            polyline: &s.polyline,
            distance_meters: s.distance_meters,
            members: s
                .activity_ids
                .iter()
                .filter_map(|aid| {
                    let act = by_id.get(aid.as_str())?;
                    let traversals: Vec<RankTraversal> =
                        tracematch::track_portions(aid, &act.points, &s.polyline, config)
                            .into_iter()
                            .map(|p| RankTraversal {
                                start: p.start_index as usize,
                                end: p.end_index as usize,
                                direction: p.direction,
                                effort: None,
                            })
                            .collect();
                    Some(RankMember {
                        activity_id: aid.as_str(),
                        traversals,
                    })
                })
                .collect(),
        })
        .collect();
    tracematch::rank(&candidates, &outings, config.proximity_threshold, None)
}

fn write_ranking_md(
    path: &Path,
    sport: &str,
    ranked: &[(String, RankFeatures)],
    sections: &[FrequentSection],
) {
    let by_id: HashMap<&str, &FrequentSection> =
        sections.iter().map(|s| (s.id.as_str(), s)).collect();
    let mut out = String::new();
    out.push_str(&format!("# Interestingness ranking: {}\n\n", sport));
    out.push_str("Score = equal-weight mean of seven percentile-normalised features.\n");
    out.push_str(
        "apex: share of outing roam at the section (was it the point of the ride).\n\
         grade: max sustained gradient over 300 m. months: distinct visit months.\n\
         sinu: 1 - chord/arc. conv: effective approach directions, counting only\n\
         outings not based within two matching tolerances of the section.\n\
         1way: direction purity. rec: days from last visit to the corpus head\n\
         (fresher ranks higher).\n\n",
    );
    out.push_str(
        "| # | id | score | len m | visits | months | apex | grade% | sinu | conv | 1way | rec d |\n",
    );
    out.push_str(
        "|--:|----|------:|------:|-------:|-------:|-----:|-------:|-----:|-----:|-----:|------:|\n",
    );
    for (i, (id, r)) in ranked.iter().enumerate() {
        let (len_m, visits) = by_id
            .get(id.as_str())
            .map(|s| (s.distance_meters, s.visit_count))
            .unwrap_or((0.0, 0));
        out.push_str(&format!(
            "| {} | {} | {:.2} | {:.0} | {} | {} | {:.2} | {:.1} | {:.2} | {:.1} | {:.2} | {:.0} |\n",
            i + 1,
            id,
            r.score,
            len_m,
            visits,
            r.months,
            r.apex,
            r.grade,
            r.sinuosity,
            r.converge,
            r.oneway,
            r.recency_days,
        ));
    }
    std::fs::write(path, out).ok();
}

fn resample_sections(secs: &[FrequentSection]) -> Vec<Vec<GpsPoint>> {
    secs.iter()
        .map(|s| {
            let t = ((s.distance_meters / 50.0).ceil() as usize).clamp(2, 400);
            resample_route(&s.polyline, t)
        })
        .collect()
}

/// Of the `before` sections whose GROUND survives in `after` (containment >= 50%
/// at 60 m, the survival rule used everywhere else here), the fraction that
/// survive under the SAME id. Mirrors the E2E harness `identity_retention`,
/// applied to a labelled catalogue so raw positional ids and stable ids are
/// scored identically. 100% when nothing survives (vacuously stable), matching
/// the survival closure's empty convention.
fn identity_retention(
    before: &[(String, Vec<GpsPoint>)],
    after: &[(String, Vec<GpsPoint>)],
) -> f64 {
    if before.is_empty() {
        return 1.0;
    }
    let mut ground_survivors = 0usize;
    let mut id_survivors = 0usize;
    for (id_b, g_b) in before {
        if !after
            .iter()
            .any(|(_, g_a)| containment(g_b, g_a, 60.0) >= 0.5)
        {
            continue;
        }
        ground_survivors += 1;
        if after
            .iter()
            .any(|(id_a, g_a)| id_a == id_b && containment(g_b, g_a, 60.0) >= 0.5)
        {
            id_survivors += 1;
        }
    }
    if ground_survivors == 0 {
        1.0
    } else {
        id_survivors as f64 / ground_survivors as f64
    }
}

fn peak_rss_mb() -> f64 {
    std::fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("VmHWM"))
                .and_then(|l| l.split_whitespace().nth(1))
                .and_then(|kb| kb.parse::<f64>().ok())
        })
        .map(|kb| kb / 1024.0)
        .unwrap_or(0.0)
}

// ---------------------------------------------------------------- main

struct StderrLog;
impl log::Log for StderrLog {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= log::Level::Debug
    }
    fn log(&self, record: &log::Record) {
        if self.enabled(record.metadata()) {
            eprintln!("{}", record.args());
        }
    }
    fn flush(&self) {}
}
static STDERR_LOG: StderrLog = StderrLog;

const USAGE: &str = "\
Usage: unified_lab [CORPUS_DIR] [OPTIONS]

  CORPUS_DIR              GPX corpus root (default: citycorpus)

Options:
  --out DIR               write GeoJSON, sweep tables and reports here
  --sport NAME            only this sport (default: every sport found)
  --method NAME           only this matching method
  --divergence F          route-split divergence override
  --stability             run the jackknife stability pass
  --windows               run the loading-window pass
  --replay-identity       replay identity + hysteresis (implies --windows)
  --sweep                 run the constants plateau sweep
  --hyst-k N              hysteresis k
  --hyst-dissolve F       hysteresis dissolve pressure
  --hyst-recut F          hysteresis recut agreement
  --hyst-floor F          hysteresis size floor
  --hyst-ratio F          hysteresis size ratio
  -h, --help              print this and exit";

#[derive(Debug, Default)]
struct LabArgs {
    dir: PathBuf,
    out_dir: Option<PathBuf>,
    only_sport: Option<String>,
    only_method: Option<String>,
    divergence: Option<f64>,
    stability: bool,
    windows: bool,
    sweep: bool,
    replay_identity: bool,
    hyst_k: Option<u8>,
    hyst_dissolve: Option<f64>,
    hyst_recut: Option<f64>,
    hyst_floor: Option<f64>,
    hyst_ratio: Option<f64>,
}

/// `Ok(None)` is `--help`, which prints the usage and exits zero. An unknown
/// flag is an error rather than a silent skip, because the run that follows a
/// typo is the default configuration reported as the requested one.
fn parse_args(args: &[String]) -> Result<Option<LabArgs>, String> {
    let mut parsed = LabArgs {
        dir: PathBuf::from("citycorpus"),
        ..LabArgs::default()
    };
    let mut positional_taken = false;
    let mut i = 0;
    let value = |args: &[String], i: usize| -> Result<String, String> {
        args.get(i + 1)
            .cloned()
            .ok_or_else(|| format!("{} needs a value", args[i]))
    };
    let number = |args: &[String], i: usize| -> Result<f64, String> {
        let raw = value(args, i)?;
        raw.parse::<f64>()
            .map_err(|_| format!("{} needs a number, got {raw}", args[i]))
    };
    while i < args.len() {
        match args[i].as_str() {
            "-h" | "--help" => return Ok(None),
            "--out" => {
                parsed.out_dir = Some(PathBuf::from(value(args, i)?));
                i += 2;
            }
            "--sport" => {
                parsed.only_sport = Some(value(args, i)?);
                i += 2;
            }
            "--method" => {
                parsed.only_method = Some(value(args, i)?);
                i += 2;
            }
            "--divergence" => {
                parsed.divergence = Some(number(args, i)?);
                i += 2;
            }
            "--stability" => {
                parsed.stability = true;
                i += 1;
            }
            "--windows" => {
                parsed.windows = true;
                i += 1;
            }
            "--replay-identity" => {
                parsed.replay_identity = true;
                i += 1;
            }
            "--sweep" => {
                parsed.sweep = true;
                i += 1;
            }
            "--hyst-k" => {
                let raw = value(args, i)?;
                parsed.hyst_k = Some(
                    raw.parse::<u8>()
                        .map_err(|_| format!("--hyst-k needs a whole number, got {raw}"))?,
                );
                i += 2;
            }
            "--hyst-dissolve" => {
                parsed.hyst_dissolve = Some(number(args, i)?);
                i += 2;
            }
            "--hyst-recut" => {
                parsed.hyst_recut = Some(number(args, i)?);
                i += 2;
            }
            "--hyst-floor" => {
                parsed.hyst_floor = Some(number(args, i)?);
                i += 2;
            }
            "--hyst-ratio" => {
                parsed.hyst_ratio = Some(number(args, i)?);
                i += 2;
            }
            other if other.starts_with('-') => {
                return Err(format!("unknown option {other}"));
            }
            other => {
                if positional_taken {
                    return Err(format!("unexpected argument {other}"));
                }
                parsed.dir = PathBuf::from(other);
                positional_taken = true;
                i += 1;
            }
        }
    }
    Ok(Some(parsed))
}

fn main() {
    log::set_logger(&STDERR_LOG).ok();
    log::set_max_level(if std::env::var("LAB_DEBUG").is_ok() {
        log::LevelFilter::Debug
    } else {
        log::LevelFilter::Info
    });
    let args: Vec<String> = std::env::args().collect();
    let LabArgs {
        dir,
        out_dir,
        only_sport,
        only_method,
        divergence,
        stability,
        mut windows,
        sweep,
        replay_identity,
        hyst_k,
        hyst_dissolve,
        hyst_recut,
        hyst_floor,
        hyst_ratio,
    } = match parse_args(&args[1..]) {
        Ok(Some(parsed)) => parsed,
        Ok(None) => {
            println!("{USAGE}");
            return;
        }
        Err(message) => {
            eprintln!("{message}\n\n{USAGE}");
            std::process::exit(2);
        }
    };

    // Identity replay reuses the loading-window detections, so it implies the
    // window pass. Params default to the identity.rs plateau values; the CLI
    // overrides let a tuning run pass explicit values without touching defaults.
    if replay_identity {
        windows = true;
    }
    let mut hyst_params = HysteresisParams::default();
    if let Some(k) = hyst_k {
        hyst_params.k = k;
    }
    if let Some(d) = hyst_dissolve {
        hyst_params.dissolve_pressure_hi = d;
    }
    if let Some(r) = hyst_recut {
        hyst_params.recut_agreement = r;
    }

    // GeoLife public-data battery. When LAB_GEOLIFE_DIR points at an existing
    // GeoLife 1.3 Data root it becomes the activity source, so every instrument
    // below runs on trajectories that are not the author's own data. Unset or
    // absent, the positional GPX corpus is used exactly as before.
    let geolife_dir = std::env::var("LAB_GEOLIFE_DIR")
        .ok()
        .filter(|s| !s.is_empty())
        .map(PathBuf::from);
    let use_geolife = geolife_dir.as_ref().is_some_and(|g| {
        let ok = g.exists();
        if !ok {
            println!(
                "GeoLife skipped (LAB_GEOLIFE_DIR set but path not found: {})",
                g.display()
            );
        }
        ok
    });
    if !use_geolife {
        if geolife_dir.is_none() {
            println!("GeoLife skipped (LAB_GEOLIFE_DIR unset); using positional corpus.");
        }
        if !dir.exists() {
            eprintln!("Directory not found: {}", dir.display());
            return;
        }
    }

    let t_load = Instant::now();
    let mut activities = if use_geolife {
        let g = geolife_dir.as_ref().unwrap();
        let max_users = env_usize("LAB_GEOLIFE_MAX_USERS", 15);
        let max_per_user = env_usize("LAB_GEOLIFE_MAX_PER_USER", 25);
        load_geolife_activities(g, max_users, max_per_user)
    } else {
        load_corpus(&dir)
    };
    // Mirrors production, which passes an empty seconds slice to the detector.
    let no_seconds = std::env::var("LAB_NO_SECONDS").is_ok();
    let with_seconds = activities.iter().filter(|a| !a.seconds.is_empty()).count();
    println!(
        "Per-point times on {}/{} activities ({:.0}%){}",
        with_seconds,
        activities.len(),
        100.0 * with_seconds as f64 / activities.len().max(1) as f64,
        if no_seconds { ", blanked" } else { "" }
    );
    if no_seconds {
        for a in &mut activities {
            a.seconds.clear();
        }
    }
    // Mirrors production ingest, which builds every GpsPoint without elevation.
    if std::env::var("LAB_NO_ELEVATION").is_ok() {
        for a in &mut activities {
            for p in &mut a.points {
                p.elevation = None;
            }
        }
        println!("Elevation stripped from every point");
    }
    println!(
        "Loaded {} activities in {} ({} points, elevation on {:.0}%)",
        activities.len(),
        fmt_ms(t_load.elapsed().as_millis()),
        activities.iter().map(|a| a.points.len()).sum::<usize>(),
        100.0
            * activities
                .iter()
                .filter(|a| a.points.first().is_some_and(|p| p.elevation.is_some()))
                .count() as f64
            / activities.len().max(1) as f64,
    );

    let mut sport_counts: HashMap<&str, usize> = HashMap::new();
    for a in &activities {
        *sport_counts.entry(a.sport.as_str()).or_default() += 1;
    }
    let mut sport_list: Vec<(&str, usize)> = sport_counts.into_iter().collect();
    sport_list.sort_by_key(|(_, n)| std::cmp::Reverse(*n));
    println!("Sports: {:?}", sport_list);

    // Lift-carried spans (corpus-confirmed), rendered so the exclusion
    // rule can be eyeballed.
    if let Some(ref out) = out_dir {
        std::fs::create_dir_all(out).ok();
        let track_view: Vec<(&str, &[GpsPoint])> = activities
            .iter()
            .map(|a| (a.id.as_str(), a.points.as_slice()))
            .collect();
        let secs_view: Vec<&[f64]> = activities.iter().map(|a| a.seconds.as_slice()).collect();
        let confirmed = tracematch::confirmed_lift_spans_tuned(
            &track_view,
            &secs_view,
            &tracematch::Tunables::DEFAULT,
        );
        let mut feats: Vec<serde_json::Value> = Vec::new();
        for (a, spans) in activities.iter().zip(&confirmed) {
            for &(s, e) in spans {
                let coords: Vec<[f64; 2]> = a.points[s..=e]
                    .iter()
                    .map(|p| [p.longitude, p.latitude])
                    .collect();
                let rise = match (a.points[e].elevation, a.points[s].elevation) {
                    (Some(top), Some(bot)) => top - bot,
                    _ => 0.0,
                };
                feats.push(serde_json::json!({
                    "type": "Feature",
                    "geometry": { "type": "LineString", "coordinates": coords },
                    "properties": {
                        "activity": a.id,
                        "points": e - s + 1,
                        "rise_m": rise.round(),
                        "start_idx": s,
                        "end_idx": e,
                    },
                }));
            }
        }
        if !feats.is_empty() {
            let n_feats = feats.len();
            let fc = serde_json::json!({ "type": "FeatureCollection", "features": feats });
            let path = out.join("lift_spans.geojson");
            std::fs::write(&path, serde_json::to_string(&fc).unwrap()).ok();
            println!("Lift spans: {} flagged → {}", n_feats, path.display());
        }
    }

    // Pooled pass: the ground is sport-agnostic — a trace is a trace.
    // "All" detects over every activity; sport filtering belongs to the
    // comparison layer, not to detection.
    let mut all_list: Vec<(&str, usize)> = sport_list.clone();
    all_list.push(("All", activities.len()));

    let mut section_config = SectionConfig::default();
    if let Some(d) = divergence {
        section_config.divergence_threshold = d;
    }
    let match_config = MatchConfig::default();
    let mut reports: Vec<MethodReport> = Vec::new();
    let start_epochs: HashMap<String, i64> = if std::env::var("LAB_NO_DATES").is_ok() {
        HashMap::new()
    } else {
        activities
            .iter()
            .filter_map(|a| epoch_of(&a.date).map(|e| (a.id.clone(), e)))
            .collect()
    };
    let activities_by_id: HashMap<&str, &Activity> =
        activities.iter().map(|a| (a.id.as_str(), a)).collect();

    for (sport, n) in &all_list {
        if let Some(ref s) = only_sport
            && s != sport
        {
            continue;
        }
        if *sport != "All" && (*sport == "Other" || *n < 5) {
            continue;
        }

        let tracks: Vec<(String, Vec<GpsPoint>)> = activities
            .iter()
            .filter(|a| *sport == "All" || a.sport == *sport)
            .map(|a| (a.id.clone(), a.points.clone()))
            .collect();
        let seconds: Vec<&[f64]> = activities
            .iter()
            .filter(|a| *sport == "All" || a.sport == *sport)
            .map(|a| a.seconds.as_slice())
            .collect();
        let sport_types: HashMap<String, String> = tracks
            .iter()
            .map(|(id, _)| (id.clone(), sport.to_string()))
            .collect();

        println!();
        println!("=== {} ({} activities) ===", sport, tracks.len());

        // Route grouping as a baseline stat. Meaningless pooled: routes stay
        // within a sport.
        if *sport != "All" {
            let t_group = Instant::now();
            let signatures: Vec<RouteSignature> = tracks
                .iter()
                .filter_map(|(id, pts)| RouteSignature::from_points(id, pts, &match_config))
                .collect();
            let groups = tracematch::group_signatures_parallel(&signatures, &match_config);
            let singleton = groups.iter().filter(|g| g.activity_ids.len() == 1).count();
            println!(
                "Route groups: {} ({} singletons), {}",
                groups.len(),
                singleton,
                fmt_ms(t_group.elapsed().as_millis())
            );
        }

        let unified_boundaries: std::cell::RefCell<Vec<tracematch::BoundaryRecord>> =
            std::cell::RefCell::new(Vec::new());
        let run_method = |name: &str| -> Option<(Vec<FrequentSection>, u128)> {
            if let Some(ref m) = only_method
                && m != name
            {
                return None;
            }
            let t = Instant::now();
            let sections = match name {
                "unified" => {
                    let out = tracematch::detect_sections_unified_dated(
                        &tracks,
                        &seconds,
                        &sport_types,
                        &start_epochs,
                        &section_config,
                        &tracematch::Tunables::DEFAULT,
                    );
                    *unified_boundaries.borrow_mut() = out.boundaries;
                    out.sections
                }
                _ => return None,
            };
            Some((sections, t.elapsed().as_millis()))
        };

        for method in ["unified"] {
            if let Some((sections, ms)) = run_method(method) {
                let report = analyse(method, &sections, tracks.len(), ms);
                println!(
                    "  {:<9} {:>4} sections  median {:>6.0}m  p90 {:>7.0}m  visits med {:>3.0}  \
                     ≤2-visit {:>3.0}%  per-act avg {:>4.1} max {:>3}  overlap pairs {:>4}  \
                     selfdup {:>2} (max {:>3.0}%)  {}",
                    report.method,
                    report.n_sections,
                    report.len_median,
                    report.len_p90,
                    report.visits_median,
                    report.low_visit_share * 100.0,
                    report.per_activity_avg,
                    report.per_activity_max,
                    report.overlap_pairs,
                    report.selfdup_count,
                    report.selfdup_max * 100.0,
                    fmt_ms(report.runtime_ms),
                );
                let ranks: Option<HashMap<String, RankFeatures>> = if method == "unified" {
                    let ranked = rank_sections(&sections, &activities_by_id, &section_config);
                    for (i, (id, r)) in ranked.iter().take(10).enumerate() {
                        let len_m = sections
                            .iter()
                            .find(|s| s.id == *id)
                            .map(|s| s.distance_meters)
                            .unwrap_or(0.0);
                        println!(
                            "    rank {:>2}  {}  score {:.2}  {:.0}m  months {}  apex {:.2}  \
                             grade {:.1}%  sinu {:.2}  conv {:.1}  1way {:.2}  rec {:.0}d",
                            i + 1,
                            id,
                            r.score,
                            len_m,
                            r.months,
                            r.apex,
                            r.grade,
                            r.sinuosity,
                            r.converge,
                            r.oneway,
                            r.recency_days,
                        );
                    }
                    if let Some(ref out) = out_dir {
                        std::fs::create_dir_all(out).ok();
                        let path = out.join(format!("{}_ranking.md", sport.to_lowercase()));
                        write_ranking_md(&path, sport, &ranked, &sections);
                        println!("            ranking → {}", path.display());
                        let brs = unified_boundaries.borrow();
                        let feats: Vec<serde_json::Value> = brs
                            .iter()
                            .map(|r| {
                                serde_json::json!({
                                    "type": "Feature",
                                    "geometry": {
                                        "type": "Point",
                                        "coordinates": [r.longitude, r.latitude],
                                    },
                                    "properties": serde_json::to_value(&r.reason).unwrap(),
                                })
                            })
                            .collect();
                        let bpath =
                            out.join(format!("{}_boundaries.geojson", sport.to_lowercase()));
                        std::fs::write(
                            &bpath,
                            serde_json::to_string(&serde_json::json!({
                                "type": "FeatureCollection",
                                "features": feats,
                            }))
                            .unwrap(),
                        )
                        .ok();
                        println!(
                            "            boundaries → {} ({})",
                            bpath.display(),
                            brs.len()
                        );
                    }
                    Some(ranked.into_iter().collect())
                } else {
                    None
                };
                if std::env::var("LAB_PORTION_DUMP").is_ok() && method == "unified" {
                    for s in &sections {
                        let acts = s.activity_ids.len().max(1);
                        if s.visit_count as usize <= 2 * acts {
                            continue;
                        }
                        println!(
                            "[portions] {} len={:.0}m visits={} activities={}",
                            s.id, s.distance_meters, s.visit_count, acts
                        );
                        let mut by_act: HashMap<&str, Vec<&tracematch::SectionPortion>> =
                            HashMap::new();
                        for p in &s.activity_portions {
                            by_act.entry(p.activity_id.as_str()).or_default().push(p);
                        }
                        let mut rows: Vec<_> = by_act.into_iter().collect();
                        rows.sort_by_key(|(_, v)| std::cmp::Reverse(v.len()));
                        for (aid, ps) in rows.iter().take(4) {
                            let dists: Vec<String> = ps
                                .iter()
                                .take(12)
                                .map(|p| {
                                    format!(
                                        "{}..{} {:.0}m",
                                        p.start_index, p.end_index, p.distance_meters
                                    )
                                })
                                .collect();
                            println!("    {} x{}: {}", aid, ps.len(), dists.join(" | "));
                        }
                    }
                }
                if let Some(ref out) = out_dir {
                    std::fs::create_dir_all(out).ok();
                    let path = out.join(format!("{}_{}.geojson", sport.to_lowercase(), method));
                    std::fs::write(
                        &path,
                        serde_json::to_string(&geojson_for_sections(&sections, ranks.as_ref()))
                            .unwrap(),
                    )
                    .ok();
                    println!("            geojson → {}", path.display());
                }
                reports.push(report);
            }
        }
    }

    // --- Loading-window replay (pooled) ---------------------------------
    // Veloq's first load syncs recent history and backfills older data
    // later. Replay that: detect on the last 90 d / 6 mo / 1 y / 2 y /
    // everything, each window ending at the newest activity, and measure
    // how the catalogue reshapes as history arrives. Persistence uses the
    // jackknife definition: 1:1 containment of at least 50% within 60 m.
    if windows {
        let newest = activities
            .iter()
            .filter_map(|a| day_of(&a.date))
            .max()
            .unwrap_or(0);
        let spans: [(&str, i64); 5] = [
            ("90d", 90),
            ("6mo", 183),
            ("1y", 365),
            ("2y", 730),
            ("all", i64::MAX),
        ];
        println!();
        println!("=== Loading-window replay (pooled, anchored at the newest activity) ===");
        let mut runs: Vec<(&str, usize, Vec<FrequentSection>, u128)> = Vec::new();
        for (label, days) in spans {
            let in_window = |a: &&Activity| {
                days == i64::MAX || day_of(&a.date).is_some_and(|d| newest - d < days)
            };
            let tracks: Vec<(String, Vec<GpsPoint>)> = activities
                .iter()
                .filter(in_window)
                .map(|a| (a.id.clone(), a.points.clone()))
                .collect();
            let secs: Vec<&[f64]> = activities
                .iter()
                .filter(in_window)
                .map(|a| a.seconds.as_slice())
                .collect();
            let types: HashMap<String, String> = tracks
                .iter()
                .map(|(id, _)| (id.clone(), "All".to_string()))
                .collect();
            let t = Instant::now();
            let sections = tracematch::detect_sections_unified_dated(
                &tracks,
                &secs,
                &types,
                &start_epochs,
                &section_config,
                &tracematch::Tunables::DEFAULT,
            )
            .sections;
            runs.push((label, tracks.len(), sections, t.elapsed().as_millis()));
        }
        let rs_all: Vec<Vec<Vec<GpsPoint>>> = runs
            .iter()
            .map(|(_, _, s, _)| resample_sections(s))
            .collect();
        let survival = |from: &[Vec<GpsPoint>], into: &[Vec<GpsPoint>]| -> f64 {
            if from.is_empty() {
                return 1.0;
            }
            from.iter()
                .filter(|f| into.iter().any(|o| containment(f, o, 60.0) >= 0.5))
                .count() as f64
                / from.len() as f64
        };
        let last = runs.len() - 1;
        // Corridor coverage against the final catalogue: identity churn
        // between horizons (winner flips, re-cuts) does not count as
        // loss when the ground stays represented.
        let cell_m = (section_config.proximity_threshold * 0.5).clamp(50.0, 150.0);
        let same_traffic = 1.0 - section_config.divergence_threshold.clamp(0.05, 0.5);
        let full_grid = CorridorGrid::build(&rs_all[last], cell_m);
        for (i, (label, n_acts, sections, ms)) in runs.iter().enumerate() {
            let into_next = if i < last {
                survival(&rs_all[i], &rs_all[i + 1])
            } else {
                1.0
            };
            let into_full = survival(&rs_all[i], &rs_all[last]);
            let of_full = survival(&rs_all[last], &rs_all[i]);
            let corr_full = if rs_all[i].is_empty() {
                1.0
            } else {
                rs_all[i]
                    .iter()
                    .filter(|s| full_grid.covered_share(s) >= same_traffic)
                    .count() as f64
                    / rs_all[i].len() as f64
            };
            println!(
                "  {:<4} {:>5} acts  {:>4} sections  {:>7}  survive→next {:>3.0}%  \
                 survive→full {:>3.0}%  corridor→full {:>3.0}%  of-final-already-present {:>3.0}%",
                label,
                n_acts,
                sections.len(),
                fmt_ms(*ms),
                into_next * 100.0,
                into_full * 100.0,
                corr_full * 100.0,
                of_full * 100.0,
            );
        }

        // Reference evolution across horizons: for sections that persist
        // into the next window, does the chosen real trace change, and how
        // far does the extent move? Measures how alive an unpinned section
        // would be under history growth.
        println!();
        println!("  reference evolution between consecutive windows (matched sections):");
        for i in 0..last {
            let (la, _, sa, _) = &runs[i];
            let (lb, _, sb, _) = &runs[i + 1];
            let rb = &rs_all[i + 1];
            let mut matched = 0usize;
            let mut same_ref = 0usize;
            let mut len_deltas: Vec<f64> = Vec::new();
            for (si, s) in sa.iter().enumerate() {
                let mut best: Option<(usize, f64)> = None;
                for (oi, o) in rb.iter().enumerate() {
                    let c = containment(&rs_all[i][si], o, 60.0);
                    if c >= 0.5 && best.is_none_or(|(_, bc)| c > bc) {
                        best = Some((oi, c));
                    }
                }
                if let Some((oi, _)) = best {
                    matched += 1;
                    if sb[oi].representative_activity_id == s.representative_activity_id {
                        same_ref += 1;
                    }
                    len_deltas.push(
                        (sb[oi].distance_meters - s.distance_meters).abs()
                            / s.distance_meters.max(1.0),
                    );
                }
            }
            len_deltas.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let med = len_deltas.get(len_deltas.len() / 2).copied().unwrap_or(0.0);
            println!(
                "    {:>4}→{:<4} matched {:>3}  same reference {:>3.0}%  median extent delta {:>3.0}%",
                la,
                lb,
                matched,
                if matched > 0 {
                    same_ref as f64 / matched as f64 * 100.0
                } else {
                    0.0
                },
                med * 100.0,
            );
        }

        // --- Identity + hysteresis: the visible view over the same windows ---
        // Feed each window's detected catalogue through the identity and hysteresis layer
        // in horizon order, snapshot the visible catalogue after each step, and
        // score the SAME survival metric on it. Raw reproduces the churn
        // baseline (survive->full above); visible is what a user's catalogue
        // actually shows once identity is assigned once and re-cuts/dissolves
        // are debounced.
        if replay_identity {
            println!();
            println!("=== Identity + hysteresis replay (visible view, same windows) ===");
            let d = HysteresisParams::default();
            let overridden = hyst_params.k != d.k
                || hyst_params.dissolve_pressure_hi != d.dissolve_pressure_hi
                || hyst_params.recut_agreement != d.recut_agreement;
            println!(
                "  base params: k={} dissolve_pressure_hi={:.2} recut_agreement={:.2} ({})",
                hyst_params.k,
                hyst_params.dissolve_pressure_hi,
                hyst_params.recut_agreement,
                if overridden { "overridden" } else { "defaults" },
            );

            // Raw positional ids per window (floor-independent): the id-reshuffle
            // contrast against the stable ids the hysteresis layer assigns.
            let raw_ids: Vec<Vec<(String, Vec<GpsPoint>)>> = runs
                .iter()
                .zip(rs_all.iter())
                .map(|((_, _, secs, _), grounds)| {
                    secs.iter()
                        .zip(grounds.iter())
                        .map(|(s, g)| (s.id.clone(), g.clone()))
                        .collect()
                })
                .collect();
            // Churn events between two ground lists (disappeared + appeared) at
            // the same 60 m / 50% ground match the survival metric uses.
            let churn = |a: &[Vec<GpsPoint>], b: &[Vec<GpsPoint>]| -> usize {
                let gone = a
                    .iter()
                    .filter(|f| !b.iter().any(|o| containment(f, o, 60.0) >= 0.5))
                    .count();
                let appeared = b
                    .iter()
                    .filter(|o| !a.iter().any(|f| containment(o, f, 60.0) >= 0.5))
                    .count();
                gone + appeared
            };

            struct ReplaySummary {
                final_visible: usize,
                final_overlaps: usize,
                total_minted: usize,
                total_dissolved: usize,
                total_recut: usize,
                merge_inherit: usize,
                mint: usize,
                raw_churn: usize,
                visible_churn: usize,
                vis_idk_first: f64,
            }

            // One replay at a given merge floor and size ratio. Detection is
            // already done, so this re-runs only the cheap hysteresis fold, and
            // both arms share the identical raw catalogues, so the raw churn is
            // a shared baseline.
            let run_replay = |floor: f64, ratio: f64| -> ReplaySummary {
                let mut params = hyst_params;
                params.merge_mutual_floor = floor;
                params.merge_size_ratio = ratio;
                println!();
                println!("--- merge_mutual_floor = {floor:.2} ---");
                let mut state = HysteresisState::new(params);
                let mut vis: Vec<Vec<(String, Vec<GpsPoint>)>> = Vec::new();
                let (mut n_carry, mut n_split, mut n_merge, mut n_mint) =
                    (0usize, 0usize, 0usize, 0usize);
                let (mut t_mint, mut t_diss, mut t_recut) = (0usize, 0usize, 0usize);
                for (label, n_acts, sections, _) in runs.iter() {
                    let cands: Vec<CandidateSection> = sections
                        .iter()
                        .map(|s| {
                            let t = ((s.distance_meters / 50.0).ceil() as usize).clamp(2, 400);
                            CandidateSection {
                                polyline: resample_route(&s.polyline, t),
                                visit_count: s.visit_count,
                            }
                        })
                        .collect();
                    // Decision mix (observational): re-derive the plan against the
                    // pre-step visible view at this floor. first_seen is exact (the
                    // mint ordinal is the id suffix); visit_count only breaks ties.
                    let prior: Vec<PriorSection> = state
                        .visible_grounds()
                        .into_iter()
                        .map(|(id, g)| PriorSection {
                            first_seen: id.trim_start_matches("s_").parse().unwrap_or(0),
                            id,
                            polyline: g,
                            visit_count: 0,
                        })
                        .collect();
                    let plan = plan_identity_tuned(
                        &prior,
                        &cands,
                        &IdentityParams {
                            merge_mutual_floor: floor,
                            merge_size_ratio: ratio,
                        },
                    );
                    for decision in plan.decisions {
                        match decision {
                            Decision::Carry { .. } => n_carry += 1,
                            Decision::SplitInherit { .. } => n_split += 1,
                            Decision::MergeInherit { .. } => n_merge += 1,
                            Decision::Mint { .. } => n_mint += 1,
                        }
                    }
                    let o = state.step(&cands);
                    t_mint += o.minted;
                    t_diss += o.dissolved;
                    t_recut += o.recut_applied;
                    println!(
                        "  {:<4} {:>5} acts  raw {:>3} -> visible {:>3}   minted {:>2}  restored {:>2}  \
                         dissolved {:>2}  re-cut {:>2}  debouncing {:>2}",
                        label,
                        n_acts,
                        o.raw_count,
                        o.visible_count,
                        o.minted,
                        o.restored,
                        o.dissolved,
                        o.recut_applied,
                        o.debouncing,
                    );
                    vis.push(state.visible_grounds());
                }

                let vis_grounds: Vec<Vec<Vec<GpsPoint>>> = vis
                    .iter()
                    .map(|snap| snap.iter().map(|(_, g)| g.clone()).collect())
                    .collect();
                let vlast = vis_grounds.len() - 1;

                // Settle phase. The loading-window replay ends right after the
                // largest (final) reshuffle, so retirements are left mid-debounce
                // (streak < k) and re-cuts unsnapped — the visible count is frozen
                // in transit, not converged. A real drip keeps detecting; here we
                // feed the FINAL window a few more detects so the plan-retirement
                // debounce drains stale-but-covered duplicates and re-cuts snap to
                // the batch geometry. This is the honest "visible converges toward
                // raw" check.
                let raw_final = rs_all[last].len();
                let final_cands: Vec<CandidateSection> = runs[last]
                    .2
                    .iter()
                    .map(|s| {
                        let t = ((s.distance_meters / 50.0).ceil() as usize).clamp(2, 400);
                        CandidateSection {
                            polyline: resample_route(&s.polyline, t),
                            visit_count: s.visit_count,
                        }
                    })
                    .collect();
                let settle_from = state.visible_len();
                let mut settle_dissolved = 0usize;
                for _ in 0..(hyst_params.k as usize + 2) {
                    settle_dissolved += state.step(&final_cands).dissolved;
                }
                let settled = state.visible_grounds();
                let settled_grounds: Vec<Vec<GpsPoint>> =
                    settled.iter().map(|(_, g)| g.clone()).collect();
                let settled_overlaps = settled_grounds
                    .iter()
                    .enumerate()
                    .filter(|(a, ga)| {
                        settled_grounds
                            .iter()
                            .enumerate()
                            .any(|(b, gb)| *a != b && containment(ga, gb, 60.0) >= 0.5)
                    })
                    .count();
                // Identity retention of the last visible window onto the settled
                // view: ids must survive the settle (the drain retires stale
                // ground, it must not renumber the survivors).
                let settled_retention = identity_retention(&vis[vlast], &settled) * 100.0;

                println!(
                    "  {:<7} {:>9} {:>13} {:>13} {:>13}",
                    "horizon", "raw->full", "visible->full", "raw id-keep", "vis id-keep"
                );
                for i in 0..runs.len() {
                    let raw_full = survival(&rs_all[i], &rs_all[last]) * 100.0;
                    let vis_full = survival(&vis_grounds[i], &vis_grounds[vlast]) * 100.0;
                    let raw_idk = identity_retention(&raw_ids[i], &raw_ids[last]) * 100.0;
                    let vis_idk = identity_retention(&vis[i], &vis[vlast]) * 100.0;
                    println!(
                        "  {:<7} {:>8.0}% {:>12.0}% {:>12.0}% {:>12.0}%",
                        runs[i].0, raw_full, vis_full, raw_idk, vis_idk,
                    );
                }

                // Visible extent drift per window step: for ids carried between
                // consecutive snapshots, how far the VISIBLE geometry moved now
                // that the registry adopts (the raw baseline for this ladder is
                // Round 8c's same-reference 54/30/38/51% at median extent delta
                // 29/16/16/10%).
                let ep_shift = |a: &[GpsPoint], b: &[GpsPoint]| -> f64 {
                    let (Some(af), Some(al), Some(bf), Some(bl)) =
                        (a.first(), a.last(), b.first(), b.last())
                    else {
                        return f64::INFINITY;
                    };
                    let fwd = haversine_distance(af, bf).max(haversine_distance(al, bl));
                    let rev = haversine_distance(af, bl).max(haversine_distance(al, bf));
                    fwd.min(rev)
                };
                println!("  visible drift per window step (carried ids):");
                for i in 0..last {
                    let a: std::collections::BTreeMap<&str, &Vec<GpsPoint>> =
                        vis[i].iter().map(|(id, g)| (id.as_str(), g)).collect();
                    let mut same = 0usize;
                    let mut shifts: Vec<f64> = Vec::new();
                    let mut deltas: Vec<f64> = Vec::new();
                    for (id, gb) in &vis[i + 1] {
                        let Some(ga) = a.get(id.as_str()) else {
                            continue;
                        };
                        let s = ep_shift(ga.as_slice(), gb);
                        let la = tracematch::matching::calculate_route_distance(ga.as_slice());
                        let lb = tracematch::matching::calculate_route_distance(gb);
                        if s < 1.0 && (la - lb).abs() < 1.0 {
                            same += 1;
                        } else {
                            shifts.push(s);
                            deltas.push(if la.max(lb) > 0.0 {
                                (la - lb).abs() / la.max(lb)
                            } else {
                                0.0
                            });
                        }
                    }
                    shifts.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
                    deltas.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
                    let carried = same + shifts.len();
                    let med = shifts.get(shifts.len() / 2).copied().unwrap_or(0.0);
                    let p90 = shifts
                        .get(((shifts.len() * 9) / 10).min(shifts.len().saturating_sub(1)))
                        .copied()
                        .unwrap_or(0.0);
                    let dmed = deltas.get(deltas.len() / 2).copied().unwrap_or(0.0);
                    println!(
                        "    {:>4}→{:<4} carried {:>3}  same geometry {:>3.0}%  moved {:>3}: shift med {:>4.0} / p90 {:>4.0} m  extent delta med {:>3.0}%",
                        runs[i].0,
                        runs[i + 1].0,
                        carried,
                        if carried > 0 {
                            same as f64 / carried as f64 * 100.0
                        } else {
                            0.0
                        },
                        shifts.len(),
                        med,
                        p90,
                        dmed * 100.0,
                    );
                }

                let mut raw_churn = 0usize;
                let mut visible_churn = 0usize;
                for i in 0..last {
                    raw_churn += churn(&rs_all[i], &rs_all[i + 1]);
                    visible_churn += churn(&vis_grounds[i], &vis_grounds[i + 1]);
                }
                // Overlapping visible sections at the final view: a section whose
                // ground is co-covered by another visible one (the duplication
                // signature the merge floor targets).
                let fin = &vis_grounds[vlast];
                let final_overlaps = fin
                    .iter()
                    .enumerate()
                    .filter(|(a, ga)| {
                        fin.iter()
                            .enumerate()
                            .any(|(b, gb)| *a != b && containment(ga, gb, 60.0) >= 0.5)
                    })
                    .count();
                println!(
                    "  churn events: raw {}  visible {}  (damped {:.0}%);  \
                     final visible {}  overlapping {}",
                    raw_churn,
                    visible_churn,
                    if raw_churn > 0 {
                        100.0 * raw_churn.saturating_sub(visible_churn) as f64 / raw_churn as f64
                    } else {
                        0.0
                    },
                    fin.len(),
                    final_overlaps,
                );
                println!(
                    "  decision mix over {} detects: carry {}  split-inherit {}  merge-inherit {}  mint {}",
                    runs.len(),
                    n_carry,
                    n_split,
                    n_merge,
                    n_mint,
                );
                println!(
                    "  SETTLED (+{} detects of the final window): visible {} -> {}  (raw {})  \
                     overlapping {}  dissolved-in-settle {}  id-retention {:.0}%",
                    hyst_params.k as usize + 2,
                    settle_from,
                    settled.len(),
                    raw_final,
                    settled_overlaps,
                    settle_dissolved,
                    settled_retention,
                );
                ReplaySummary {
                    final_visible: fin.len(),
                    final_overlaps,
                    total_minted: t_mint,
                    total_dissolved: t_diss,
                    total_recut: t_recut,
                    merge_inherit: n_merge,
                    mint: n_mint,
                    raw_churn,
                    visible_churn,
                    vis_idk_first: identity_retention(&vis[0], &vis[vlast]) * 100.0,
                }
            };

            // Neither knob has a corpus-confirmed value, so the on-arm takes
            // whichever the caller named. --hyst-ratio alone sweeps the size
            // guard with the floor left at its shipped 0.0.
            let hi = hyst_floor.unwrap_or(if hyst_ratio.is_some() { 0.0 } else { 0.4 });
            let hi_ratio = hyst_ratio.unwrap_or(0.0);
            let off = run_replay(0.0, 0.0);
            let on = run_replay(hi, hi_ratio);
            println!();
            println!("=== OFF (floor 0.00, ratio 0.00) vs floor {hi:.2}, ratio {hi_ratio:.2} ===");
            let row = |name: &str, a: String, b: String| {
                println!("  {name:<24} {a:>12}  {b:>12}");
            };
            row("metric", "floor 0.00".into(), format!("floor {hi:.2}"));
            row(
                "final visible",
                off.final_visible.to_string(),
                on.final_visible.to_string(),
            );
            row(
                "overlapping (dup)",
                off.final_overlaps.to_string(),
                on.final_overlaps.to_string(),
            );
            row(
                "total minted",
                off.total_minted.to_string(),
                on.total_minted.to_string(),
            );
            row(
                "total dissolved",
                off.total_dissolved.to_string(),
                on.total_dissolved.to_string(),
            );
            row(
                "total re-cut",
                off.total_recut.to_string(),
                on.total_recut.to_string(),
            );
            row(
                "merge-inherit",
                off.merge_inherit.to_string(),
                on.merge_inherit.to_string(),
            );
            row("mint decisions", off.mint.to_string(), on.mint.to_string());
            row(
                "visible churn",
                off.visible_churn.to_string(),
                on.visible_churn.to_string(),
            );
            row(
                "raw churn",
                off.raw_churn.to_string(),
                on.raw_churn.to_string(),
            );
            row(
                "90d->full vis id-keep",
                format!("{:.0}%", off.vis_idk_first),
                format!("{:.0}%", on.vis_idk_first),
            );
        }
    }

    // --- Constants plateau sweep (pooled, one factor at a time) ---------
    // Each Tunables field is varied around Tunables::DEFAULT while every
    // other field stays at its default. A defensible default sits on a
    // plateau: catalogue shape and stability barely move across the
    // neighbouring values. A peak (quality degrading on both sides of
    // the default) means the value is fitted and must be re-derived.
    if sweep {
        let tracks: Vec<(String, Vec<GpsPoint>)> = activities
            .iter()
            .map(|a| (a.id.clone(), a.points.clone()))
            .collect();
        let types: HashMap<String, String> = tracks
            .iter()
            .map(|(id, _)| (id.clone(), "All".to_string()))
            .collect();
        let jk_tracks: Vec<(String, Vec<GpsPoint>)> = tracks
            .iter()
            .enumerate()
            .filter(|(i, _)| i % 10 != 0)
            .map(|(_, t)| t.clone())
            .collect();
        let all_secs: Vec<&[f64]> = activities.iter().map(|a| a.seconds.as_slice()).collect();
        let jk_secs: Vec<&[f64]> = all_secs
            .iter()
            .enumerate()
            .filter(|(i, _)| i % 10 != 0)
            .map(|(_, s)| *s)
            .collect();
        let track_view: Vec<(&str, &[GpsPoint])> = activities
            .iter()
            .map(|a| (a.id.as_str(), a.points.as_slice()))
            .collect();
        let cell_m = (section_config.proximity_threshold * 0.5).clamp(50.0, 150.0);
        let same_traffic = 1.0 - section_config.divergence_threshold.clamp(0.05, 0.5);

        // pass_window_needed values are encoded as window * 10 + needed.
        type Setter = fn(&mut Tunables, f64);
        let axes: Vec<(&str, Vec<f64>, Setter)> = vec![
            (
                "pass_away_cells",
                vec![3.0, 4.0, 5.0, 6.0, 7.0, 8.0],
                |t, v| t.pass_away_cells = v as usize,
            ),
            ("ele_level_tol_m", vec![10.0, 15.0, 20.0, 25.0], |t, v| {
                t.ele_level_tol_m = v
            }),
            ("pass_subgrid", vec![2.0, 3.0, 4.0], |t, v| {
                t.pass_subgrid = v
            }),
            ("dwell_events", vec![4.0, 5.0, 6.0, 8.0, 10.0], |t, v| {
                t.dwell_events = v as usize
            }),
            (
                "pass_window_needed",
                vec![42.0, 53.0, 63.0, 64.0],
                |t, v| {
                    t.pass_window = (v / 10.0) as usize;
                    t.pass_needed = v as usize % 10;
                },
            ),
            ("reach", vec![1.0, 2.0], |t, v| t.reach = v as i32),
            ("lift_span_m", vec![200.0, 300.0, 400.0], |t, v| {
                t.lift_span_m = v
            }),
            ("lift_min_grade", vec![0.18, 0.22, 0.26, 0.30], |t, v| {
                t.lift_min_grade = v
            }),
            ("lift_min_straight", vec![0.96, 0.975, 0.985], |t, v| {
                t.lift_min_straight = v
            }),
            (
                "jitter_human_min",
                vec![1.02, 1.035, 1.05, 1.065, 1.08],
                |t, v| t.jitter_human_min = v,
            ),
            ("descent_match_m", vec![40.0, 60.0, 80.0], |t, v| {
                t.descent_match_m = v
            }),
            (
                "cluster_gap_m",
                vec![10_000.0, 25_000.0, 50_000.0, 100_000.0, 200_000.0],
                |t, v| t.cluster_gap_m = v,
            ),
            ("self_pass_clean", vec![0.02, 0.05, 0.08, 0.10], |t, v| {
                t.self_pass_clean = v
            }),
            ("minority_run_m", vec![40.0, 60.0, 80.0, 100.0], |t, v| {
                t.minority_run_m = v
            }),
            (
                "occasion_span_h",
                vec![72.0, 120.0, 168.0, 240.0, 336.0],
                |t, v| t.occasion_span_h = v,
            ),
        ];

        let bbox = |pts: &[GpsPoint]| -> (f64, f64, f64, f64) {
            let mut bb = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for p in pts {
                bb.0 = bb.0.min(p.latitude);
                bb.1 = bb.1.max(p.latitude);
                bb.2 = bb.2.min(p.longitude);
                bb.3 = bb.3.max(p.longitude);
            }
            bb
        };

        println!();
        println!(
            "=== Constants sweep (pooled, {} acts, jackknife 90%) ===",
            tracks.len()
        );
        let mut md = String::from(
            "# Constants plateau sweep (pooled)\n\n\
             One factor at a time around `Tunables::DEFAULT`. Columns: catalogue\n\
             size and shape, overlapping pairs (>30% containment at 60 m), core\n\
             (>=5 visits) persistence under a 90% jackknife as 1:1 identity and\n\
             as corridor coverage (rule 6 backoff tolerance), ground retained\n\
             (60 m), lift spans excluded, sections holding >=40% sustained\n\
             grade, and combined full+jackknife detect time.\n",
        );
        for (name, values, set) in &axes {
            md.push_str(&format!(
                "\n## {}\n\n\
                 | value | sections | med m | total km | ovl pairs | core | 1:1 % | corridor % | ground % | lift spans | steep >=40% | detect |\n\
                 |------:|---------:|------:|---------:|----------:|-----:|------:|-----------:|---------:|-----------:|-------------|-------:|\n",
                name
            ));
            for &v in values {
                let mut tun = Tunables::DEFAULT;
                set(&mut tun, v);
                let t0 = Instant::now();
                let full = tracematch::detect_sections_unified_dated(
                    &tracks,
                    &all_secs,
                    &types,
                    &start_epochs,
                    &section_config,
                    &tun,
                )
                .sections;
                let jk = tracematch::detect_sections_unified_dated(
                    &jk_tracks,
                    &jk_secs,
                    &types,
                    &start_epochs,
                    &section_config,
                    &tun,
                )
                .sections;
                let ms = t0.elapsed().as_millis();

                let full_rs = resample_sections(&full);
                let jk_rs = resample_sections(&jk);
                let mut lens: Vec<f64> = full.iter().map(|s| s.distance_meters).collect();
                lens.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let total_km = lens.iter().sum::<f64>() / 1000.0;

                let bbs: Vec<(f64, f64, f64, f64)> = full_rs.iter().map(|s| bbox(s)).collect();
                let pad = 60.0 / M_PER_DEG_LAT;
                let mut ovl = 0usize;
                for i in 0..full_rs.len() {
                    for j in (i + 1)..full_rs.len() {
                        let (a, b) = (&bbs[i], &bbs[j]);
                        if a.0 > b.1 + pad || a.1 < b.0 - pad || a.2 > b.3 + pad || a.3 < b.2 - pad
                        {
                            continue;
                        }
                        if containment(&full_rs[i], &full_rs[j], 60.0) > 0.3
                            || containment(&full_rs[j], &full_rs[i], 60.0) > 0.3
                        {
                            ovl += 1;
                        }
                    }
                }

                let core_idx: Vec<usize> = full
                    .iter()
                    .enumerate()
                    .filter(|(_, s)| s.visit_count >= 5)
                    .map(|(i, _)| i)
                    .collect();
                let jk_grid = CorridorGrid::build(&jk_rs, cell_m);
                let (mut c11, mut ccorr) = (0usize, 0usize);
                for &i in &core_idx {
                    if jk_rs
                        .iter()
                        .any(|o| containment(&full_rs[i], o, 60.0) >= 0.5)
                    {
                        c11 += 1;
                    }
                    if jk_grid.covered_share(&full_rs[i]) >= same_traffic {
                        ccorr += 1;
                    }
                }
                let pct = |n: usize, d: usize| {
                    if d == 0 {
                        100.0
                    } else {
                        n as f64 / d as f64 * 100.0
                    }
                };

                let g_grid = CorridorGrid::build(&jk_rs, 60.0);
                let (mut gc, mut gt) = (0usize, 0usize);
                for f in &full_rs {
                    for p in f {
                        gt += 1;
                        if g_grid.point_covered(p) {
                            gc += 1;
                        }
                    }
                }

                let lift = tracematch::confirmed_lift_spans_tuned(&track_view, &all_secs, &tun);
                let n_lift: usize = lift.iter().map(|s| s.len()).sum();

                let steep: Vec<String> = full
                    .iter()
                    .filter_map(|s| {
                        let g = tracematch::max_sustained_grade(&s.polyline).unwrap_or(0.0);
                        (g >= 40.0).then(|| format!("{} {:.0}%/{}v", s.id, g, s.visit_count))
                    })
                    .collect();
                let steep_txt = if steep.is_empty() {
                    "-".to_string()
                } else {
                    steep.join(", ")
                };

                let label = if *name == "pass_window_needed" {
                    format!("{}of{}", v as usize % 10, (v / 10.0) as usize)
                } else {
                    format!("{}", v)
                };
                println!(
                    "  {:<17} {:>6}  {:>4} sections  med {:>5.0}m  {:>6.0}km  ovl {:>3}  \
                     core {:>3}  1:1 {:>3.0}%  corr {:>3.0}%  ground {:>3.0}%  lift {:>3}  \
                     steep [{}]  {}",
                    name,
                    label,
                    full.len(),
                    percentile(&lens, 0.5),
                    total_km,
                    ovl,
                    core_idx.len(),
                    pct(c11, core_idx.len()),
                    pct(ccorr, core_idx.len()),
                    pct(gc, gt),
                    n_lift,
                    steep_txt,
                    fmt_ms(ms),
                );
                md.push_str(&format!(
                    "| {} | {} | {:.0} | {:.0} | {} | {} | {:.0} | {:.0} | {:.0} | {} | {} | {} |\n",
                    label,
                    full.len(),
                    percentile(&lens, 0.5),
                    total_km,
                    ovl,
                    core_idx.len(),
                    pct(c11, core_idx.len()),
                    pct(ccorr, core_idx.len()),
                    pct(gc, gt),
                    n_lift,
                    steep_txt,
                    fmt_ms(ms),
                ));
            }
        }
        if let Some(ref out) = out_dir {
            std::fs::create_dir_all(out).ok();
            let path = out.join("sweep.md");
            std::fs::write(&path, &md).ok();
            println!("  sweep table → {}", path.display());
        }
    }

    // --- Stability experiments (unified only) ---------------------------
    if stability {
        for (sport, n) in &all_list {
            if *sport != "All" && (*sport == "Other" || *n < 20) {
                continue;
            }
            if let Some(ref s) = only_sport
                && s != sport
            {
                continue;
            }
            let sport_acts: Vec<&Activity> = activities
                .iter()
                .filter(|a| *sport == "All" || a.sport == *sport)
                .collect();
            let full_tracks: Vec<(String, Vec<GpsPoint>)> = sport_acts
                .iter()
                .map(|a| (a.id.clone(), a.points.clone()))
                .collect();
            let full_secs: Vec<&[f64]> = sport_acts.iter().map(|a| a.seconds.as_slice()).collect();
            let types = |ts: &[(String, Vec<GpsPoint>)]| -> HashMap<String, String> {
                ts.iter()
                    .map(|(id, _)| (id.clone(), sport.to_string()))
                    .collect()
            };
            let detect = |ts: &[(String, Vec<GpsPoint>)], ss: &[&[f64]]| -> Vec<FrequentSection> {
                tracematch::detect_sections_unified_dated(
                    ts,
                    ss,
                    &types(ts),
                    &start_epochs,
                    &section_config,
                    &tracematch::Tunables::DEFAULT,
                )
                .sections
            };

            let full = detect(&full_tracks, &full_secs);
            let resample_all = |secs: &[FrequentSection]| -> Vec<Vec<GpsPoint>> {
                secs.iter()
                    .map(|s| {
                        let t = ((s.distance_meters / 50.0).ceil() as usize).clamp(2, 400);
                        resample_route(&s.polyline, t)
                    })
                    .collect()
            };
            let full_rs = resample_all(&full);

            // A section "persists" if some single section in the other
            // run contains ≥50% of it within 60 m (same partitioning).
            let persistence = |other: &[FrequentSection]| -> f64 {
                if full.is_empty() {
                    return 0.0;
                }
                let other_rs = resample_all(other);
                let survived = full_rs
                    .iter()
                    .filter(|f| other_rs.iter().any(|o| containment(f, o, 60.0) >= 0.5))
                    .count();
                survived as f64 / full_rs.len() as f64
            };

            // Ground retained: fraction of the full catalogue's length
            // still covered by SOME section in the other run. Separates
            // "same ground, cut differently" from "ground lost".
            let ground = |other: &[FrequentSection]| -> f64 {
                if full_rs.is_empty() {
                    return 0.0;
                }
                let other_rs = resample_all(other);
                let mut covered = 0usize;
                let mut total = 0usize;
                for f in &full_rs {
                    for p in f {
                        total += 1;
                        if other_rs.iter().any(|o| {
                            o.iter().any(|q| {
                                let m_lng = 111_000.0 * q.latitude.to_radians().cos();
                                let dx = (p.longitude - q.longitude) * m_lng;
                                let dy = (p.latitude - q.latitude) * 111_000.0;
                                dx * dx + dy * dy <= 60.0 * 60.0
                            })
                        }) {
                            covered += 1;
                        }
                    }
                }
                if total > 0 {
                    covered as f64 / total as f64
                } else {
                    0.0
                }
            };

            // Jackknife: drop every 10th activity (deterministic).
            let jk: Vec<(String, Vec<GpsPoint>)> = full_tracks
                .iter()
                .enumerate()
                .filter(|(i, _)| i % 10 != 0)
                .map(|(_, t)| t.clone())
                .collect();
            let jk_secs: Vec<&[f64]> = full_secs
                .iter()
                .enumerate()
                .filter(|(i, _)| i % 10 != 0)
                .map(|(_, s)| *s)
                .collect();
            let jk_sections = detect(&jk, &jk_secs);
            let jk_persist = persistence(&jk_sections);

            // Chronological prefixes (activities are date-sorted).
            let prefix_k =
                |frac: f64| (((full_tracks.len() as f64) * frac).round() as usize).max(1);
            let prefix = |frac: f64| -> Vec<(String, Vec<GpsPoint>)> {
                full_tracks[..prefix_k(frac)].to_vec()
            };
            let half = detect(&prefix(0.5), &full_secs[..prefix_k(0.5)]);
            let three_q = detect(&prefix(0.75), &full_secs[..prefix_k(0.75)]);
            // Growth stability: what fraction of the half-corpus catalogue
            // still exists (per containment) in the full catalogue?
            let growth_half = {
                if half.is_empty() {
                    1.0
                } else {
                    let half_rs = resample_all(&half);
                    let survived = half_rs
                        .iter()
                        .filter(|h| full_rs.iter().any(|f| containment(h, f, 60.0) >= 0.5))
                        .count();
                    survived as f64 / half_rs.len() as f64
                }
            };

            // Core catalogue: sections with real support. Marginal
            // sections legitimately vanish when 10% of the corpus does;
            // the core must not. Persistence is reported both ways:
            // 1:1 identity (some single section contains ≥50% of me
            // within 60 m) and corridor coverage (rule 6's backoff
            // tolerance, one partition cell, against the whole other
            // catalogue) which is immune to winner flips and re-cuts.
            let jk_rs = resample_all(&jk_sections);
            let cell_m = (section_config.proximity_threshold * 0.5).clamp(50.0, 150.0);
            let same_traffic = 1.0 - section_config.divergence_threshold.clamp(0.05, 0.5);
            let jk_grid = CorridorGrid::build(&jk_rs, cell_m);
            println!();
            for min_visits in [5u32, 10] {
                let core_rs: Vec<Vec<GpsPoint>> = full
                    .iter()
                    .filter(|s| s.visit_count >= min_visits)
                    .map(|s| {
                        let t = ((s.distance_meters / 50.0).ceil() as usize).clamp(2, 400);
                        resample_route(&s.polyline, t)
                    })
                    .collect();
                let (core_persist, core_corridor, core_either) = if core_rs.is_empty() {
                    (1.0, 1.0, 1.0)
                } else {
                    let one_to_one =
                        |c: &Vec<GpsPoint>| jk_rs.iter().any(|o| containment(c, o, 60.0) >= 0.5);
                    let corridor = |c: &Vec<GpsPoint>| jk_grid.covered_share(c) >= same_traffic;
                    let n = core_rs.len() as f64;
                    (
                        core_rs.iter().filter(|c| one_to_one(c)).count() as f64 / n,
                        core_rs.iter().filter(|c| corridor(c)).count() as f64 / n,
                        core_rs
                            .iter()
                            .filter(|c| one_to_one(c) || corridor(c))
                            .count() as f64
                            / n,
                    )
                };
                println!(
                    "[stability {}] core (≥{} visits) {} sections under jackknife: {:.0}% persist \
                     1:1, {:.0}% corridor-covered, {:.0}% represented either way",
                    sport,
                    min_visits,
                    core_rs.len(),
                    core_persist * 100.0,
                    core_corridor * 100.0,
                    core_either * 100.0
                );
            }
            println!(
                "[stability {}] full {} | jackknife-90%: {} sections, {:.0}% persist 1:1, \
                 {:.0}% ground retained | 50% corpus: {} sections ({:.0}% persist into full) | \
                 75% corpus: {}",
                sport,
                full.len(),
                jk_sections.len(),
                jk_persist * 100.0,
                ground(&jk_sections) * 100.0,
                half.len(),
                growth_half * 100.0,
                three_q.len(),
            );
        }
    }

    println!();
    println!("Peak RSS: {:.0} MB", peak_rss_mb());
}

#[cfg(test)]
mod arg_tests {
    use super::*;

    fn args(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn a_misspelt_flag_is_rejected_rather_than_skipped() {
        let err = parse_args(&args(&["--sweeep"])).unwrap_err();
        assert!(err.contains("--sweeep"), "{err}");
    }

    #[test]
    fn help_asks_for_the_usage_rather_than_a_run() {
        assert!(parse_args(&args(&["--help"])).unwrap().is_none());
        assert!(parse_args(&args(&["-h"])).unwrap().is_none());
    }

    #[test]
    fn the_positional_corpus_and_out_are_both_read() {
        let parsed = parse_args(&args(&["/corpus", "--out", "/out", "--sweep"]))
            .unwrap()
            .unwrap();
        assert_eq!(parsed.dir, PathBuf::from("/corpus"));
        assert_eq!(parsed.out_dir, Some(PathBuf::from("/out")));
        assert!(parsed.sweep);
    }

    #[test]
    fn no_arguments_keeps_the_documented_default_corpus() {
        let parsed = parse_args(&[]).unwrap().unwrap();
        assert_eq!(parsed.dir, PathBuf::from("citycorpus"));
        assert_eq!(parsed.out_dir, None);
    }

    #[test]
    fn a_flag_missing_its_value_is_rejected() {
        assert!(parse_args(&args(&["--out"])).is_err());
        assert!(parse_args(&args(&["--divergence", "wide"])).is_err());
    }

    #[test]
    fn a_second_positional_is_rejected() {
        assert!(parse_args(&args(&["/corpus", "/other"])).is_err());
    }
}
