//! Integration tests for the new section manipulation functions.
//!
//! Tests:
//! - find_sections_in_route
//! - split_section_at_point
//! - split_section_at_index
//! - recalculate_section_polyline

use std::path::Path;
use tracematch::{
    FrequentSection, GpsPoint, SectionConfig, find_sections_in_route, recalculate_section_polyline,
    split_section_at_index, split_section_at_point,
};

/// Load GPX file and extract GPS points
fn load_gpx(path: &Path) -> Vec<GpsPoint> {
    let content = std::fs::read_to_string(path).expect("Failed to read GPX file");

    let mut points = Vec::new();

    // Simple GPX parsing - look for trkpt elements
    for line in content.lines() {
        if line.contains("<trkpt") {
            // Extract lat and lon attributes
            if let (Some(lat_start), Some(lon_start)) = (line.find("lat=\""), line.find("lon=\"")) {
                let lat_str = &line[lat_start + 5..];
                let lon_str = &line[lon_start + 5..];

                if let (Some(lat_end), Some(lon_end)) = (lat_str.find('"'), lon_str.find('"')) {
                    if let (Ok(lat), Ok(lon)) = (
                        lat_str[..lat_end].parse::<f64>(),
                        lon_str[..lon_end].parse::<f64>(),
                    ) {
                        points.push(GpsPoint::new(lat, lon));
                    }
                }
            }
        }
    }

    points
}

/// Load multiple GPX files from a directory
fn load_gpx_files(dir: &Path, limit: usize) -> Vec<(String, Vec<GpsPoint>)> {
    let mut tracks = Vec::new();

    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().is_some_and(|e| e == "gpx") {
                let points = load_gpx(&path);
                if points.len() >= 50 {
                    let name = path.file_stem().unwrap().to_string_lossy().to_string();
                    tracks.push((name, points));
                    if tracks.len() >= limit {
                        break;
                    }
                }
            }
        }
    }

    tracks
}

#[test]
fn test_find_sections_in_route() {
    let gpx_dir = Path::new("sionrunning");
    if !gpx_dir.exists() {
        println!("Skipping test - sionrunning directory not found");
        return;
    }

    // Load some GPX files
    let tracks = load_gpx_files(gpx_dir, 20);
    if tracks.len() < 5 {
        println!("Skipping test - not enough GPX files");
        return;
    }

    println!("Loaded {} tracks", tracks.len());

    // Create a simple section from the first track's middle portion
    let (ref_name, ref_track) = &tracks[0];
    let section_start = ref_track.len() / 4;
    let section_end = ref_track.len() * 3 / 4;
    let section_polyline: Vec<_> = ref_track[section_start..section_end].to_vec();

    let section = FrequentSection {
        id: "test_section".to_string(),
        name: Some("Test Section".to_string()),
        sport_type: "Run".to_string(),
        polyline: section_polyline.clone(),
        representative_activity_id: ref_name.clone(),
        activity_ids: vec![ref_name.clone()],
        activity_portions: vec![],
        route_ids: vec![],
        visit_count: 1,
        distance_meters: calculate_distance(&section_polyline),
        activity_traces: std::collections::HashMap::new(),
        confidence: 0.8,
        observation_count: 1,
        average_spread: 10.0,
        point_density: vec![1; section_polyline.len()],
        scale: Some("test".to_string()),
        version: 1,
        is_user_defined: false,
        created_at: None,
        updated_at: None,
        stability: 0.5,
    };

    let config = SectionConfig::default();

    // Test: Find the section in the original track (should match)
    let matches = find_sections_in_route(ref_track, &[section.clone()], &config);

    println!(
        "Finding section in original track: {} matches",
        matches.len()
    );
    assert!(
        !matches.is_empty(),
        "Section should be found in original track"
    );

    let m = &matches[0];
    println!(
        "  Match: section_id={}, start={}, end={}, quality={:.2}, same_dir={}",
        m.section_id, m.start_index, m.end_index, m.match_quality, m.same_direction
    );

    assert!(m.match_quality >= 0.6, "Match quality should be >= 0.6");
    assert!(m.same_direction, "Direction should match for same track");

    // Test: Find section in other tracks
    let mut found_in_others = 0;
    for (name, track) in tracks.iter().skip(1) {
        let matches = find_sections_in_route(track, &[section.clone()], &config);
        if !matches.is_empty() {
            found_in_others += 1;
            println!(
                "  Found in {}: quality={:.2}",
                name, matches[0].match_quality
            );
        }
    }
    println!("Section found in {} other tracks", found_in_others);
}

#[test]
fn test_split_section_at_index() {
    // Create a simple linear section
    let polyline: Vec<_> = (0..100)
        .map(|i| GpsPoint::new(46.23 + i as f64 * 0.0001, 7.36 + i as f64 * 0.0001))
        .collect();

    let section = FrequentSection {
        id: "split_test".to_string(),
        name: Some("Split Test Section".to_string()),
        sport_type: "Run".to_string(),
        polyline: polyline.clone(),
        representative_activity_id: "test".to_string(),
        activity_ids: vec!["test".to_string()],
        activity_portions: vec![],
        route_ids: vec![],
        visit_count: 5,
        distance_meters: calculate_distance(&polyline),
        activity_traces: std::collections::HashMap::new(),
        confidence: 0.8,
        observation_count: 5,
        average_spread: 10.0,
        point_density: vec![5; polyline.len()],
        scale: Some("test".to_string()),
        version: 1,
        is_user_defined: false,
        created_at: None,
        updated_at: None,
        stability: 0.5,
    };

    // Split at index 50
    let result = split_section_at_index(&section, 50);
    assert!(result.is_some(), "Split should succeed");

    let split = result.unwrap();
    println!("Split at index 50:");
    println!(
        "  First: {} points, {:.0}m",
        split.first.polyline.len(),
        split.first.distance_meters
    );
    println!(
        "  Second: {} points, {:.0}m",
        split.second.polyline.len(),
        split.second.distance_meters
    );

    assert_eq!(split.first.polyline.len(), 51); // 0..=50
    assert_eq!(split.second.polyline.len(), 50); // 50..100
    assert!(
        split.first.is_user_defined,
        "Split sections should be marked user-defined"
    );
    assert!(
        split.second.is_user_defined,
        "Split sections should be marked user-defined"
    );

    // Test invalid splits
    assert!(
        split_section_at_index(&section, 0).is_none(),
        "Split at 0 should fail"
    );
    assert!(
        split_section_at_index(&section, 99).is_none(),
        "Split at last index should fail"
    );
    assert!(
        split_section_at_index(&section, 100).is_none(),
        "Split beyond length should fail"
    );
}

#[test]
fn test_split_section_at_point() {
    // Create a simple linear section
    let polyline: Vec<_> = (0..100)
        .map(|i| GpsPoint::new(46.23 + i as f64 * 0.0001, 7.36 + i as f64 * 0.0001))
        .collect();

    let section = FrequentSection {
        id: "split_point_test".to_string(),
        name: Some("Split Point Test".to_string()),
        sport_type: "Run".to_string(),
        polyline: polyline.clone(),
        representative_activity_id: "test".to_string(),
        activity_ids: vec!["test".to_string()],
        activity_portions: vec![],
        route_ids: vec![],
        visit_count: 5,
        distance_meters: calculate_distance(&polyline),
        activity_traces: std::collections::HashMap::new(),
        confidence: 0.8,
        observation_count: 5,
        average_spread: 10.0,
        point_density: vec![5; polyline.len()],
        scale: Some("test".to_string()),
        version: 1,
        is_user_defined: false,
        created_at: None,
        updated_at: None,
        stability: 0.5,
    };

    // Split at the midpoint (approximately index 50)
    let mid_point = &polyline[50];
    let split_point = GpsPoint::new(mid_point.latitude, mid_point.longitude);

    let result = split_section_at_point(&section, &split_point, 50.0);
    assert!(result.is_some(), "Split at midpoint should succeed");

    let split = result.unwrap();
    println!("Split at geographic midpoint:");
    println!(
        "  First: {} points, {:.0}m",
        split.first.polyline.len(),
        split.first.distance_meters
    );
    println!(
        "  Second: {} points, {:.0}m",
        split.second.polyline.len(),
        split.second.distance_meters
    );

    // Test split at point too far away
    let far_point = GpsPoint::new(46.5, 7.5); // Far from section
    let result = split_section_at_point(&section, &far_point, 50.0);
    assert!(result.is_none(), "Split at distant point should fail");
}

#[test]
fn test_recalculate_section_polyline() {
    // Create a section with multiple activity traces
    let base_polyline: Vec<_> = (0..50)
        .map(|i| GpsPoint::new(46.23 + i as f64 * 0.0002, 7.36 + i as f64 * 0.0002))
        .collect();

    // Create slightly offset traces
    let mut activity_traces = std::collections::HashMap::new();

    // Trace 1: slightly north
    let trace1: Vec<_> = base_polyline
        .iter()
        .map(|p| GpsPoint::new(p.latitude + 0.00005, p.longitude))
        .collect();
    activity_traces.insert("trace1".to_string(), trace1);

    // Trace 2: slightly south
    let trace2: Vec<_> = base_polyline
        .iter()
        .map(|p| GpsPoint::new(p.latitude - 0.00005, p.longitude))
        .collect();
    activity_traces.insert("trace2".to_string(), trace2);

    // Trace 3: original
    activity_traces.insert("trace3".to_string(), base_polyline.clone());

    let section = FrequentSection {
        id: "recalc_test".to_string(),
        name: Some("Recalculate Test".to_string()),
        sport_type: "Run".to_string(),
        polyline: base_polyline.clone(),
        representative_activity_id: "trace1".to_string(),
        activity_ids: vec![
            "trace1".to_string(),
            "trace2".to_string(),
            "trace3".to_string(),
        ],
        activity_portions: vec![],
        route_ids: vec![],
        visit_count: 3,
        distance_meters: calculate_distance(&base_polyline),
        activity_traces,
        confidence: 0.8,
        observation_count: 3,
        average_spread: 10.0,
        point_density: vec![3; base_polyline.len()],
        scale: Some("test".to_string()),
        version: 1,
        is_user_defined: false,
        created_at: None,
        updated_at: None,
        stability: 0.5,
    };

    let config = SectionConfig::default();

    let recalculated = recalculate_section_polyline(&section, &config);

    println!("Recalculated section:");
    println!("  Original points: {}", section.polyline.len());
    println!("  Recalculated points: {}", recalculated.polyline.len());
    println!("  Original distance: {:.0}m", section.distance_meters);
    println!(
        "  Recalculated distance: {:.0}m",
        recalculated.distance_meters
    );
    println!("  Version: {} -> {}", section.version, recalculated.version);

    assert_eq!(
        recalculated.version,
        section.version + 1,
        "Version should increment"
    );
    assert!(
        !recalculated.polyline.is_empty(),
        "Recalculated polyline should not be empty"
    );

    // Test that user-defined sections are not modified
    let mut user_section = section.clone();
    user_section.is_user_defined = true;

    let not_modified = recalculate_section_polyline(&user_section, &config);
    assert_eq!(
        not_modified.version, user_section.version,
        "User-defined sections should not be modified"
    );
}

#[test]
fn test_find_sections_real_data() {
    let gpx_dir = Path::new("sionrunning");
    if !gpx_dir.exists() {
        println!("Skipping test - sionrunning directory not found");
        return;
    }

    // Load GPX files
    let tracks = load_gpx_files(gpx_dir, 50);
    if tracks.len() < 10 {
        println!(
            "Skipping test - not enough GPX files (need 10, have {})",
            tracks.len()
        );
        return;
    }

    println!("Testing with {} tracks", tracks.len());

    // Detect sections using the library
    let config = SectionConfig::default();

    // Create sport types map (all Run for simplicity)
    let sport_types: std::collections::HashMap<String, String> = tracks
        .iter()
        .map(|(name, _)| (name.clone(), "Run".to_string()))
        .collect();

    // Use detect_sections_optimized directly
    let sections = tracematch::sections::detect_sections_optimized(&tracks, &sport_types, &config);

    println!("Detected {} sections", sections.len());

    if sections.is_empty() {
        println!("No sections detected - cannot test find_sections_in_route");
        return;
    }

    // Show detected sections
    for (i, sec) in sections.iter().take(5).enumerate() {
        println!(
            "  Section {}: {} - {:.0}m, {} visits",
            i, sec.id, sec.distance_meters, sec.visit_count
        );
    }

    // Test finding sections in each track
    let mut total_matches = 0;
    for (name, track) in tracks.iter().take(10) {
        let matches = find_sections_in_route(track, &sections, &config);
        if !matches.is_empty() {
            println!("Track {}: {} sections found", name, matches.len());
            for m in &matches {
                println!(
                    "  - {} at {}..{} (quality: {:.2})",
                    m.section_id, m.start_index, m.end_index, m.match_quality
                );
            }
            total_matches += matches.len();
        }
    }

    println!("\nTotal matches across 10 tracks: {}", total_matches);
}

/// Calculate total distance of a polyline
fn calculate_distance(points: &[GpsPoint]) -> f64 {
    if points.len() < 2 {
        return 0.0;
    }

    points
        .windows(2)
        .map(|w| haversine_distance(&w[0], &w[1]))
        .sum()
}

/// Haversine distance in meters
fn haversine_distance(p1: &GpsPoint, p2: &GpsPoint) -> f64 {
    let r = 6_371_000.0; // Earth radius in meters

    let lat1 = p1.latitude.to_radians();
    let lat2 = p2.latitude.to_radians();
    let dlat = (p2.latitude - p1.latitude).to_radians();
    let dlon = (p2.longitude - p1.longitude).to_radians();

    let a = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();

    r * c
}
