//! GeoLife, one person at a time.
//!
//! The pooled lab folds every logger into one catalogue, and strangers who
//! share a campus then supply each other's repeats. The app detects within one
//! athlete, so this reports each person on their own, alongside enough about
//! how they moved that a figure reads against who produced it.
//!
//!     LAB_GEOLIFE_DIR=".../Geolife Trajectories 1.3/Data" \
//!         cargo run --release --example geolife_users -- [--min 20] [--cap 120] [--out table.md]
//!
//! `--min` is the fewest self-powered trajectories a person needs to be listed,
//! `--cap` the most taken per person in chronological order. Persistence is
//! the lab's jackknife: drop every tenth trajectory, detect again, and count
//! the full catalogue's sections whose ground is still there (half the
//! resampled line within 60 m).

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;

use tracematch::{
    FrequentSection, GpsPoint, MatchConfig, RouteSignature, SectionConfig, Tunables,
    detect_sections_unified_dated, geo_utils::haversine_distance, group_signatures_parallel,
    matching::resample_route,
};

#[path = "common/geolife.rs"]
mod geolife;
use geolife::GeoTrajectory;

struct Person {
    user: String,
    trips: Vec<GeoTrajectory>,
}

struct Detected {
    sections: usize,
    median_len_m: f64,
    median_visits: u32,
    trips_with_section: f64,
    persist: f64,
}

struct Row {
    user: String,
    trips: usize,
    walk: usize,
    bike: usize,
    run: usize,
    span_days: i64,
    median_km: f64,
    repeat_share: f64,
    detected: Detected,
}

fn user_of(id: &str) -> &str {
    id.split_once('_').map(|(u, _)| u).unwrap_or(id)
}

/// Days since 1970-01-01 for `YYYY-MM-DD`, Howard Hinnant's civil-from-days.
fn day_of(date: &str) -> Option<i64> {
    let mut it = date.split('-');
    let y: i64 = it.next()?.parse().ok()?;
    let m: i64 = it.next()?.parse().ok()?;
    let d: i64 = it.next()?.get(..2)?.parse().ok()?;
    let yy = if m <= 2 { y - 1 } else { y };
    let era = if yy >= 0 { yy } else { yy - 399 } / 400;
    let yoe = yy - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    Some(era * 146_097 + doe - 719_468)
}

fn track_km(points: &[GpsPoint]) -> f64 {
    points
        .windows(2)
        .map(|w| haversine_distance(&w[0], &w[1]))
        .sum::<f64>()
        / 1000.0
}

fn median_f64(values: &mut [f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    values[values.len() / 2]
}

fn median_u32(values: &mut [u32]) -> u32 {
    if values.is_empty() {
        return 0;
    }
    values.sort_unstable();
    values[values.len() / 2]
}

/// Share of `a`'s points within `threshold_m` of any point of `b`.
fn containment(a: &[GpsPoint], b: &[GpsPoint], threshold_m: f64) -> f64 {
    if a.is_empty() || b.is_empty() {
        return 0.0;
    }
    let m_per_deg_lat = 111_000.0;
    let m_per_deg_lng = 111_000.0 * a[0].latitude.to_radians().cos();
    let thr2 = threshold_m * threshold_m;
    let hits = a
        .iter()
        .filter(|p| {
            b.iter().any(|q| {
                let dx = (p.longitude - q.longitude) * m_per_deg_lng;
                let dy = (p.latitude - q.latitude) * m_per_deg_lat;
                dx * dx + dy * dy <= thr2
            })
        })
        .count();
    hits as f64 / a.len() as f64
}

fn resample_sections(secs: &[FrequentSection]) -> Vec<Vec<GpsPoint>> {
    secs.iter()
        .map(|s| {
            let t = ((s.distance_meters / 50.0).ceil() as usize).clamp(2, 400);
            resample_route(&s.polyline, t)
        })
        .collect()
}

/// Grouping then detection, the order the engine runs them in. Returns the
/// catalogue and the share of trips that belong to a route group of two or
/// more.
fn detect(trips: &[&GeoTrajectory]) -> (Vec<FrequentSection>, f64) {
    let match_config = MatchConfig::default();
    let tracks: Vec<(String, Vec<GpsPoint>)> = trips
        .iter()
        .map(|t| (t.id.clone(), t.points.clone()))
        .collect();
    let sports: HashMap<String, String> = trips
        .iter()
        .map(|t| (t.id.clone(), t.sport.clone()))
        .collect();
    let signatures: Vec<RouteSignature> = tracks
        .iter()
        .filter_map(|(id, pts)| RouteSignature::from_points(id, pts, &match_config))
        .collect();
    let groups = group_signatures_parallel(&signatures, &match_config);
    let repeated: usize = groups
        .iter()
        .filter(|g| g.activity_ids.len() >= 2)
        .map(|g| g.activity_ids.len())
        .sum();
    let repeat_share = repeated as f64 / tracks.len().max(1) as f64;
    let config = SectionConfig::default();
    let sections = {
        let seconds: Vec<&[f64]> = trips.iter().map(|t| t.seconds.as_slice()).collect();
        let start_epochs: HashMap<String, i64> = trips
            .iter()
            .filter_map(|t| day_of(&t.date).map(|d| (t.id.clone(), d * 86_400)))
            .collect();
        detect_sections_unified_dated(
            &tracks,
            &seconds,
            &sports,
            &start_epochs,
            &config,
            &Tunables::DEFAULT,
        )
        .sections
    };
    (sections, repeat_share)
}

fn measure(all: &[&GeoTrajectory]) -> (Detected, f64) {
    let (sections, repeat_share) = detect(all);
    let kept: Vec<&GeoTrajectory> = all
        .iter()
        .enumerate()
        .filter(|(i, _)| i % 10 != 9)
        .map(|(_, t)| *t)
        .collect();
    let (jack, _) = detect(&kept);
    let full_lines = resample_sections(&sections);
    let jack_lines = resample_sections(&jack);
    let persist = if full_lines.is_empty() {
        0.0
    } else {
        full_lines
            .iter()
            .filter(|f| jack_lines.iter().any(|o| containment(f, o, 60.0) >= 0.5))
            .count() as f64
            / full_lines.len() as f64
    };
    let mut lens: Vec<f64> = sections.iter().map(|s| s.distance_meters).collect();
    let mut visits: Vec<u32> = sections.iter().map(|s| s.visit_count).collect();
    let covered: std::collections::BTreeSet<&str> = sections
        .iter()
        .flat_map(|s| s.activity_ids.iter().map(String::as_str))
        .collect();
    (
        Detected {
            sections: sections.len(),
            median_len_m: median_f64(&mut lens),
            median_visits: median_u32(&mut visits),
            trips_with_section: covered.len() as f64 / all.len().max(1) as f64,
            persist,
        },
        repeat_share,
    )
}

fn profile(person: &Person) -> Row {
    let all: Vec<&GeoTrajectory> = person.trips.iter().collect();
    let (detected, repeat_share) = measure(&all);

    let days: Vec<i64> = person
        .trips
        .iter()
        .filter_map(|t| day_of(&t.date))
        .collect();
    let span_days = match (days.iter().min(), days.iter().max()) {
        (Some(a), Some(b)) => b - a + 1,
        _ => 0,
    };
    let mut kms: Vec<f64> = person.trips.iter().map(|t| track_km(&t.points)).collect();
    let count = |sport: &str| person.trips.iter().filter(|t| t.sport == sport).count();

    Row {
        user: person.user.clone(),
        trips: person.trips.len(),
        walk: count("Walk"),
        bike: count("Ride"),
        run: count("Run"),
        span_days,
        median_km: median_f64(&mut kms),
        repeat_share,
        detected,
    }
}

/// A short reading of who this person is from how they moved, so a figure can
/// be judged against the behaviour that produced it.
fn character(r: &Row) -> String {
    let mode = match (r.walk, r.bike) {
        (w, b) if b == 0 && w > 0 => "walker".to_string(),
        (w, b) if w == 0 && b > 0 => "cyclist".to_string(),
        (w, b) if b as f64 >= 0.7 * (w + b) as f64 => "mostly cyclist".to_string(),
        (w, b) if w as f64 >= 0.7 * (w + b) as f64 => "mostly walker".to_string(),
        _ => "walks and rides".to_string(),
    };
    let rhythm = if r.repeat_share >= 0.5 {
        "keeps to the same few routes"
    } else if r.repeat_share >= 0.2 {
        "repeats some routes"
    } else {
        "rarely takes the same route twice"
    };
    let span = if r.span_days >= 60 {
        format!("over {} months", (r.span_days as f64 / 30.4).round() as i64)
    } else {
        format!("over {} days", r.span_days)
    };
    let length = if r.median_km >= 8.0 {
        "long trips"
    } else if r.median_km >= 2.5 {
        "mid-length trips"
    } else {
        "short trips"
    };
    format!("{mode}, {rhythm}, {length}, {span}")
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut min_trips = 20usize;
    let mut cap = 120usize;
    let mut out: Option<PathBuf> = None;
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--min" => {
                min_trips = args
                    .get(i + 1)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(min_trips);
                i += 2;
            }
            "--cap" => {
                cap = args.get(i + 1).and_then(|s| s.parse().ok()).unwrap_or(cap);
                i += 2;
            }
            "--out" => {
                out = args.get(i + 1).map(PathBuf::from);
                i += 2;
            }
            _ => i += 1,
        }
    }

    let root = std::env::var("LAB_GEOLIFE_DIR")
        .map(PathBuf::from)
        .expect("LAB_GEOLIFE_DIR must point at the GeoLife Data directory");
    let (trajectories, stats) = geolife::load_geolife(&root, usize::MAX, cap);
    println!(
        "loaded {} trajectories from {} people ({} carried labels), skipped {} short, {} huge, {} motorised",
        trajectories.len(),
        stats.users_loaded,
        stats.users_with_labels,
        stats.skipped_short,
        stats.skipped_huge,
        stats.skipped_motorised
    );

    let mut people: BTreeMap<String, Vec<GeoTrajectory>> = BTreeMap::new();
    for t in trajectories {
        people
            .entry(user_of(&t.id).to_string())
            .or_default()
            .push(t);
    }
    let people: Vec<Person> = people
        .into_iter()
        .filter(|(_, trips)| trips.len() >= min_trips)
        .map(|(user, mut trips)| {
            trips.sort_by(|a, b| a.date.cmp(&b.date).then(a.id.cmp(&b.id)));
            Person { user, trips }
        })
        .collect();
    println!(
        "{} people with at least {min_trips} trajectories\n",
        people.len()
    );

    let mut rows: Vec<Row> = people.iter().map(profile).collect();
    rows.sort_by(|a, b| b.trips.cmp(&a.trips).then(a.user.cmp(&b.user)));

    fn cell(d: &Detected) -> String {
        if d.sections == 0 {
            return "0 | | | | ".to_string();
        }
        format!(
            "{} | {:.0} m | {} | {:.0}% | {:.0}%",
            d.sections,
            d.median_len_m,
            d.median_visits,
            d.trips_with_section * 100.0,
            d.persist * 100.0
        )
    }
    let mut md = String::new();
    md.push_str("Per person, `--cap` trips in date order.\n\n");
    md.push_str("| Person | Trips | Walk / bike / run | Span | Median trip | Repeat share | Sections | Median length | Median visits | Trips on a section | Jackknife persist | Character |\n");
    md.push_str("|---|--:|---|--:|--:|--:|--:|--:|--:|--:|--:|---|\n");
    for r in &rows {
        md.push_str(&format!(
            "| {} | {} | {} / {} / {} | {} d | {:.1} km | {:.0}% | {} | {} |\n",
            r.user,
            r.trips,
            r.walk,
            r.bike,
            r.run,
            r.span_days,
            r.median_km,
            r.repeat_share * 100.0,
            cell(&r.detected),
            character(r),
        ));
    }
    print!("{md}");

    let with: Vec<&Detected> = rows
        .iter()
        .map(|r| &r.detected)
        .filter(|d| d.sections > 0)
        .collect();
    let mut summary = String::new();
    let total: usize = with.iter().map(|d| d.sections).sum();
    let mut persist: Vec<f64> = with.iter().map(|d| d.persist).collect();
    let weighted = if total == 0 {
        0.0
    } else {
        with.iter()
            .map(|d| d.persist * d.sections as f64)
            .sum::<f64>()
            / total as f64
    };
    summary.push_str(&format!(
        "\n{} of {} people yield a section, {total} sections in all. Jackknife persistence: median {:.0}% across people, {:.0}% weighted by section count.\n",
        with.len(),
        rows.len(),
        median_f64(&mut persist) * 100.0,
        weighted * 100.0
    ));
    print!("{summary}");

    if let Some(path) = out {
        std::fs::write(&path, md + &summary).expect("write table");
        println!("written to {}", path.display());
    }
}
