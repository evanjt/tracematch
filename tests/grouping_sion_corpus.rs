//! Real-corpus smoke test for the endpoint-grid grouping path.
//!
//! Loads the full 428-GPX corpus from `sionrunning/` (gated on
//! directory existence so this test silently skips when the fixture is
//! absent — e.g. in the veloq submodule), runs route grouping via the
//! production `group_signatures_parallel`, and asserts the output is
//! non-trivial (groups found, no panic).
//!
//! This is the regression backstop for any future refactor of the
//! grouping pipeline. The legacy R-tree path was deleted in Phase 3
//! after the refinement-equivalent A/B test passed on this corpus.

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
fn route_grouping_sion_corpus_smoke() {
    let dir = Path::new("sionrunning");
    if !dir.exists() {
        println!("[sion corpus] sionrunning/ not found — skipping (expected in submodule)");
        return;
    }

    let entries = match std::fs::read_dir(dir) {
        Ok(e) => e,
        Err(e) => {
            println!("[sion corpus] read_dir failed: {e}; skipping");
            return;
        }
    };

    let config = MatchConfig::default();

    let mut signatures: Vec<RouteSignature> = Vec::new();
    for entry in entries.flatten() {
        let path = entry.path();
        if path.extension().is_some_and(|e| e == "gpx") {
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
    }

    println!("[sion corpus] {} signatures loaded", signatures.len());
    assert!(
        signatures.len() >= 50,
        "expected ≥50 GPX files in sionrunning/, got {}",
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

    // Sanity: we found non-trivial groups (Sion is a small enough city
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
