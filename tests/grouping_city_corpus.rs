//! Real-corpus smoke test for the endpoint-grid grouping path.
//!
//! Runs `group_signatures_parallel` over the `citycorpus` corpus and asserts
//! the grouping is non-trivial: groups form, and they do not collapse into one.
//! Synthetic corpora replay a canonical polyline, so this is the only place the
//! grouping pipeline meets braided GPS.
//!
//! Run: `cargo test --features real-corpus --test grouping_city_corpus`

mod corpus;

use std::path::Path;
use std::time::Instant;

use tracematch::{GpsPoint, MatchConfig, RouteSignature};

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

#[test]
fn route_grouping_city_corpus_smoke() {
    let paths = corpus::require_at_least("citycorpus", 50);
    let config = MatchConfig::default();

    // Sorted paths, so the signature order is the same on every machine.
    let mut signatures: Vec<RouteSignature> = Vec::new();
    for path in paths {
        let points = load_gpx(&path);
        if points.len() < 50 {
            continue;
        }
        let name = path
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();
        if let Some(sig) = RouteSignature::from_points(&name, &points, &config) {
            signatures.push(sig);
        }
    }

    println!("[sion corpus] {} signatures loaded", signatures.len());
    assert!(
        signatures.len() >= 50,
        "expected ≥50 GPX files in citycorpus/, got {}",
        signatures.len()
    );

    let t = Instant::now();
    let groups = tracematch::group_signatures_parallel(&signatures, &config);
    let dur = t.elapsed();

    println!(
        "[sion corpus] {} groups in {:?} ({} signatures)",
        groups.len(),
        dur,
        signatures.len()
    );

    // Sanity: we found non-trivial groups (the corpus covers a small enough city
    // that most routes share corridors, so we expect groups but not
    // EVERYTHING in one big group).
    assert!(
        groups.len() >= 50 && groups.len() < signatures.len(),
        "expected 50 ≤ groups < signatures, got {} groups for {} sigs",
        groups.len(),
        signatures.len()
    );

    let max_group_size = groups
        .iter()
        .map(|g| g.activity_ids.len())
        .max()
        .unwrap_or(0);
    assert!(
        max_group_size >= 5,
        "expected at least one group with ≥5 members on a corpus of {} sigs, got max {}",
        signatures.len(),
        max_group_size
    );
}
