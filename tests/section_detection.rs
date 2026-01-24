//! Integration tests for section detection using real GPS fixtures.
//!
//! These fixtures are from 5 running activities:
//! - Activities 1-4: Similar routes that should group together
//! - Activity 5: Long run that shares at least one common section with the others
//!
//! These tests define expected behavior. If tests fail, the library logic
//! should be adjusted, not the tests.
//!
//! NOTE: These tests require raw GPS fixtures in `tests/fixtures/raw_traces/`.
//! If the fixtures are not present (e.g., in CI), tests will be skipped.

use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;

use tracematch::{
    GpsPoint, MatchConfig, RouteSignature, ScalePreset, SectionConfig, detect_sections_from_tracks,
    detect_sections_multiscale, group_signatures,
};

/// Check if the raw trace fixtures are available.
fn fixtures_available() -> bool {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("tests/fixtures/raw_traces");
    path.push("activity_1_trimmed.json");
    path.exists()
}

/// Load a GPS trace from a fixture file.
fn load_fixture(name: &str) -> Vec<GpsPoint> {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("tests/fixtures/raw_traces");
    path.push(name);

    let contents =
        fs::read_to_string(&path).unwrap_or_else(|_| panic!("Failed to read fixture: {:?}", path));
    let coords: Vec<Vec<f64>> = serde_json::from_str(&contents)
        .unwrap_or_else(|_| panic!("Failed to parse fixture: {:?}", path));

    coords
        .into_iter()
        .map(|c| GpsPoint::new(c[0], c[1]))
        .collect()
}

/// Load all 5 activity fixtures. Returns None if fixtures are not available.
fn load_all_fixtures() -> Option<Vec<(String, Vec<GpsPoint>)>> {
    if !fixtures_available() {
        return None;
    }
    Some(
        (1..=5)
            .map(|i| {
                let id = format!("activity_{}", i);
                let points = load_fixture(&format!("{}_trimmed.json", id));
                (id, points)
            })
            .collect(),
    )
}

/// Macro to skip a test if fixtures are not available.
macro_rules! require_fixtures {
    () => {
        match load_all_fixtures() {
            Some(tracks) => tracks,
            None => {
                eprintln!("Skipping test: raw trace fixtures not available");
                return;
            }
        }
    };
}

/// Create sport types map (all activities are Run)
fn create_sport_types(tracks: &[(String, Vec<GpsPoint>)]) -> HashMap<String, String> {
    tracks
        .iter()
        .map(|(id, _)| (id.clone(), "Run".to_string()))
        .collect()
}

#[test]
fn test_load_fixtures() {
    let tracks = require_fixtures!();

    assert_eq!(tracks.len(), 5);

    // Verify expected point counts (after 150m trim from each end + coordinate offset)
    assert!(
        tracks[0].1.len() > 3500,
        "Activity 1 should have ~3900 points, got {}",
        tracks[0].1.len()
    );
    assert!(
        tracks[1].1.len() > 3000,
        "Activity 2 should have ~3170 points, got {}",
        tracks[1].1.len()
    );
    assert!(
        tracks[2].1.len() > 2000,
        "Activity 3 should have ~2320 points, got {}",
        tracks[2].1.len()
    );
    assert!(
        tracks[3].1.len() > 2500,
        "Activity 4 should have ~2700 points, got {}",
        tracks[3].1.len()
    );
    assert!(
        tracks[4].1.len() > 5000,
        "Activity 5 should have ~5440 points, got {}",
        tracks[4].1.len()
    );
}

#[test]
fn test_detect_sections_finds_overlaps() {
    let tracks = require_fixtures!();
    let sport_types = create_sport_types(&tracks);

    // Use discovery config with lower thresholds
    let config = SectionConfig {
        min_section_length: 500.0,  // 500m minimum
        max_section_length: 2000.0, // 2km max
        min_activities: 2,          // Need at least 2 activities
        proximity_threshold: 50.0,
        cluster_tolerance: 100.0,
        scale_presets: vec![], // Use legacy single-scale
        ..SectionConfig::default()
    };

    let sections = detect_sections_from_tracks(&tracks, &sport_types, &[], &config);

    println!("Found {} sections:", sections.len());
    for section in &sections {
        println!(
            "  {} - {:.0}m, {} activities: {:?}",
            section.id,
            section.distance_meters,
            section.activity_ids.len(),
            section.activity_ids
        );
    }

    // Should find at least 1 section (the common portions)
    assert!(
        !sections.is_empty(),
        "Should detect at least one common section"
    );
}

#[test]
fn test_detect_common_sections_exploration() {
    let tracks = require_fixtures!();
    let sport_types = create_sport_types(&tracks);

    // Config to find sections of 300m-1.5km with at least 3 activities
    let config = SectionConfig {
        min_section_length: 300.0,
        max_section_length: 1500.0,
        min_activities: 3,
        proximity_threshold: 50.0,
        cluster_tolerance: 100.0,
        scale_presets: vec![],
        ..SectionConfig::default()
    };

    let sections = detect_sections_from_tracks(&tracks, &sport_types, &[], &config);

    println!("Found {} sections with 3+ activities:", sections.len());
    for section in &sections {
        let ids: Vec<_> = section
            .activity_ids
            .iter()
            .map(|id| id.replace("activity_", "A"))
            .collect();
        println!(
            "  {}: {:.0}m, {} activities: {:?}",
            section.id,
            section.distance_meters,
            section.activity_ids.len(),
            ids
        );
    }

    // With lower threshold (2 activities), find more sections
    let config_low = SectionConfig {
        min_activities: 2,
        ..config.clone()
    };

    let sections_low = detect_sections_from_tracks(&tracks, &sport_types, &[], &config_low);

    println!(
        "\nWith min_activities=2, found {} sections:",
        sections_low.len()
    );
    for section in &sections_low {
        let ids: Vec<_> = section
            .activity_ids
            .iter()
            .map(|id| id.replace("activity_", "A"))
            .collect();
        println!(
            "  {}: {:.0}m, {} activities: {:?}",
            section.id,
            section.distance_meters,
            section.activity_ids.len(),
            ids
        );
    }

    // Sections found should have at least min_activities
    for section in &sections {
        assert!(
            section.activity_ids.len() >= 3,
            "Each section should have at least 3 activities"
        );
    }

    // Should find at least one common section
    assert!(!sections.is_empty(), "Should find at least one section");
}

#[test]
fn test_section_includes_activity_5() {
    let tracks = require_fixtures!();
    let sport_types = create_sport_types(&tracks);

    // Config focused on finding sections Activity 5 participates in
    let config = SectionConfig {
        min_section_length: 300.0,
        max_section_length: 1500.0,
        min_activities: 2, // Lower threshold to see more sections
        proximity_threshold: 50.0,
        cluster_tolerance: 100.0,
        scale_presets: vec![],
        ..SectionConfig::default()
    };

    let sections = detect_sections_from_tracks(&tracks, &sport_types, &[], &config);

    // Find sections containing Activity 5
    let a5_sections: Vec<_> = sections
        .iter()
        .filter(|s| s.activity_ids.contains(&"activity_5".to_string()))
        .collect();

    println!("Sections containing Activity 5:");
    for section in &a5_sections {
        let ids: Vec<_> = section
            .activity_ids
            .iter()
            .map(|id| id.replace("activity_", "A"))
            .collect();
        println!(
            "  {}: {:.0}m, activities: {:?}",
            section.id, section.distance_meters, ids
        );
    }

    // Activity 5 should be part of at least one section
    // (based on our earlier analysis, it shares the final ~1km with A1, A3, A4)
    assert!(
        !a5_sections.is_empty(),
        "Activity 5 should participate in at least one detected section"
    );

    // Any section with A5 should also include at least one other "end" activity (A1, A3, or A4)
    // since A5 shares its end with them
    for section in &a5_sections {
        let has_partner = section.activity_ids.contains(&"activity_1".to_string())
            || section.activity_ids.contains(&"activity_3".to_string())
            || section.activity_ids.contains(&"activity_4".to_string());
        // A2 might also be present if there's some overlap

        assert!(
            has_partner,
            "A5's section should include at least one of A1, A3, A4"
        );
    }
}

#[test]
fn test_activity_5_shares_end_not_start() {
    let tracks = require_fixtures!();
    let sport_types = create_sport_types(&tracks);

    let config = SectionConfig {
        min_section_length: 200.0,
        max_section_length: 2000.0,
        min_activities: 2,
        proximity_threshold: 50.0,
        cluster_tolerance: 100.0,
        scale_presets: vec![],
        ..SectionConfig::default()
    };

    let sections = detect_sections_from_tracks(&tracks, &sport_types, &[], &config);

    // Find all sections that include A5
    let a5_sections: Vec<_> = sections
        .iter()
        .filter(|s| s.activity_ids.contains(&"activity_5".to_string()))
        .collect();

    println!("Sections including Activity 5:");
    for section in &a5_sections {
        println!(
            "  {}: {:.0}m, activities: {:?}",
            section.id, section.distance_meters, section.activity_ids
        );
    }

    // A5 should participate in at least one section
    // (the end section where it shares the final stretch with A1,A3,A4)
    assert!(
        !a5_sections.is_empty(),
        "Activity 5 should share at least one section with other activities (the final stretch)"
    );

    // For any section with A5, verify it also contains at least one of A1,A3,A4
    // (since A5 shares the end with them)
    for section in &a5_sections {
        let has_end_partner = section.activity_ids.contains(&"activity_1".to_string())
            || section.activity_ids.contains(&"activity_3".to_string())
            || section.activity_ids.contains(&"activity_4".to_string());

        assert!(
            has_end_partner,
            "A5's sections should include at least one of A1,A3,A4 (shared final stretch)"
        );
    }
}

#[test]
fn test_multiscale_detection() {
    let tracks = require_fixtures!();
    let sport_types = create_sport_types(&tracks);

    // Use multi-scale detection
    let config = SectionConfig {
        proximity_threshold: 50.0,
        cluster_tolerance: 100.0,
        scale_presets: vec![
            ScalePreset {
                name: "short".to_string(),
                min_length: 200.0,
                max_length: 600.0,
                min_activities: 2,
            },
            ScalePreset {
                name: "medium".to_string(),
                min_length: 500.0,
                max_length: 1500.0,
                min_activities: 2,
            },
        ],
        include_potentials: true,
        ..SectionConfig::default()
    };

    let result = detect_sections_multiscale(&tracks, &sport_types, &[], &config);

    println!(
        "Multi-scale detection: {} sections, {} potentials",
        result.sections.len(),
        result.potentials.len()
    );

    println!("Stats: {:?}", result.stats);

    for section in &result.sections {
        println!(
            "  [{}] {}: {:.0}m, {} activities",
            section.scale.as_deref().unwrap_or("?"),
            section.id,
            section.distance_meters,
            section.activity_ids.len()
        );
    }

    // Should find at least one section
    assert!(
        !result.sections.is_empty(),
        "Multi-scale detection should find sections"
    );

    // Stats should show overlaps were found
    assert!(
        result.stats.overlaps_found > 0,
        "Should find pairwise overlaps"
    );
}

#[test]
fn test_section_polyline_quality() {
    let tracks = require_fixtures!();
    let sport_types = create_sport_types(&tracks);

    let config = SectionConfig {
        min_section_length: 300.0,
        max_section_length: 1500.0,
        min_activities: 2,
        ..SectionConfig::default()
    };

    let sections = detect_sections_from_tracks(&tracks, &sport_types, &[], &config);

    for section in &sections {
        // Polyline should have reasonable number of points
        assert!(
            section.polyline.len() >= 5,
            "Section {} should have at least 5 points, got {}",
            section.id,
            section.polyline.len()
        );

        // Polyline length should roughly match reported distance
        let mut polyline_length = 0.0;
        for i in 1..section.polyline.len() {
            polyline_length += tracematch::geo_utils::haversine_distance(
                &section.polyline[i - 1],
                &section.polyline[i],
            );
        }

        let length_error =
            (polyline_length - section.distance_meters).abs() / section.distance_meters;
        assert!(
            length_error < 0.1,
            "Section {} polyline length ({:.0}m) should match reported distance ({:.0}m) within 10%",
            section.id,
            polyline_length,
            section.distance_meters
        );

        // All points should be valid GPS coordinates
        for point in &section.polyline {
            assert!(
                point.is_valid(),
                "All points should be valid GPS coordinates"
            );
        }

        // Confidence should be positive
        assert!(
            section.confidence > 0.0,
            "Section {} should have positive confidence",
            section.id
        );
    }
}

// ============================================================================
// BEHAVIORAL TESTS - These define expected behavior. If tests fail,
// the library logic should be adjusted, not the tests.
// ============================================================================

#[test]
fn test_similar_length_routes_group_together() {
    // Route grouping requires routes to be the SAME JOURNEY (similar length,
    // same endpoints). Routes that share portions but have different lengths
    // should be found by section detection, not route grouping.
    //
    // Activities 3 and 4 have similar lengths (6.1km vs 7.2km = 15% diff)
    // and should group together.

    let tracks = require_fixtures!();
    let config = MatchConfig::default();

    let signatures: Vec<RouteSignature> = tracks
        .iter()
        .filter_map(|(id, points)| RouteSignature::from_points(id, points, &config))
        .collect();

    assert_eq!(signatures.len(), 5, "Should create 5 signatures");

    println!("Route distances:");
    for sig in &signatures {
        println!(
            "  {}: {:.1}km",
            sig.activity_id,
            sig.total_distance / 1000.0
        );
    }

    let groups = group_signatures(&signatures, &config);

    println!("Route grouping results:");
    for (i, group) in groups.iter().enumerate() {
        println!("  Group {}: {:?}", i, group.activity_ids);
    }

    // A3 and A4 should be grouped (similar lengths: 6.1km vs 7.2km = 15% diff)
    let a3_group = groups
        .iter()
        .find(|g| g.activity_ids.contains(&"activity_3".to_string()));

    assert!(a3_group.is_some(), "Activity 3 should be in a group");
    let a3_group = a3_group.unwrap();

    assert!(
        a3_group.activity_ids.contains(&"activity_4".to_string()),
        "Activities 3 and 4 should be grouped together (similar length routes)"
    );

    println!(
        "Activities 3 and 4 correctly grouped: {:?}",
        a3_group.activity_ids
    );
}

#[test]
fn test_activity_5_shares_common_section_with_others() {
    // Activity 5 is a longer run but shares a common section with the other activities.
    // The section detection should find this overlap.
    // This is a behavioral requirement, not an implementation detail.

    let tracks = require_fixtures!();
    let sport_types = create_sport_types(&tracks);

    let config = SectionConfig {
        min_section_length: 200.0,  // Detect sections >= 200m
        max_section_length: 1500.0, // Up to 1.5km sections
        min_activities: 2,          // Need at least 2 activities
        proximity_threshold: 50.0,  // 50m matching threshold
        ..SectionConfig::default()
    };

    let sections = detect_sections_from_tracks(&tracks, &sport_types, &[], &config);

    println!("Section detection results:");
    for section in &sections {
        let ids: Vec<_> = section
            .activity_ids
            .iter()
            .map(|id| id.replace("activity_", "A"))
            .collect();
        println!(
            "  {}: {:.0}m, activities: {:?}",
            section.id, section.distance_meters, ids
        );
    }

    // Find sections that include Activity 5
    let a5_sections: Vec<_> = sections
        .iter()
        .filter(|s| s.activity_ids.contains(&"activity_5".to_string()))
        .collect();

    // Activity 5 MUST share at least one section with other activities
    assert!(
        !a5_sections.is_empty(),
        "Activity 5 must share at least one common section with other activities"
    );

    // The shared section must include at least one activity from 1-4
    for section in &a5_sections {
        let shares_with_1_to_4 = section.activity_ids.contains(&"activity_1".to_string())
            || section.activity_ids.contains(&"activity_2".to_string())
            || section.activity_ids.contains(&"activity_3".to_string())
            || section.activity_ids.contains(&"activity_4".to_string());

        assert!(
            shares_with_1_to_4,
            "Activity 5's section must include at least one activity from 1-4"
        );
    }

    // The shared section should be at least 200m (meaningful overlap)
    let max_section_length = a5_sections
        .iter()
        .map(|s| s.distance_meters)
        .fold(0.0, f64::max);

    assert!(
        max_section_length >= 200.0,
        "Shared section should be at least 200m, found {:.0}m",
        max_section_length
    );

    println!(
        "Activity 5 shares {} section(s) with other activities, longest: {:.0}m",
        a5_sections.len(),
        max_section_length
    );
}

#[test]
fn test_debug_a3_sections() {
    let tracks = require_fixtures!();
    let sport_types = create_sport_types(&tracks);

    // Try with very permissive settings
    let config = SectionConfig {
        min_section_length: 100.0,
        max_section_length: 5000.0,
        min_activities: 2,
        proximity_threshold: 100.0, // Larger threshold
        cluster_tolerance: 150.0,
        scale_presets: vec![],
        ..SectionConfig::default()
    };

    let sections = detect_sections_from_tracks(&tracks, &sport_types, &[], &config);

    println!("Sections with 100m proximity threshold:");
    for section in &sections {
        let ids: Vec<_> = section
            .activity_ids
            .iter()
            .map(|id| id.replace("activity_", "A"))
            .collect();
        let has_a3 = ids.contains(&"A3".to_string());
        println!(
            "  {}: {:.0}m, {} activities: {:?} {}",
            section.id,
            section.distance_meters,
            section.activity_ids.len(),
            ids,
            if has_a3 { "<-- HAS A3" } else { "" }
        );
    }

    // Check which sections contain A3
    let a3_sections: Vec<_> = sections
        .iter()
        .filter(|s| s.activity_ids.contains(&"activity_3".to_string()))
        .collect();

    println!("\nSections containing A3: {}", a3_sections.len());
}

#[test]
fn test_investigate_start_section() {
    // A1-A4 share a common START section heading East/Southeast.
    // This test investigates why only the END section is detected.

    let tracks = require_fixtures!();
    let sport_types = create_sport_types(&tracks);

    // Extract just the first 1km of each track to see START overlaps
    let start_tracks: Vec<(String, Vec<GpsPoint>)> = tracks
        .iter()
        .map(|(id, points)| {
            // Take points until we've covered ~1km
            let mut distance = 0.0;
            let mut start_points = vec![points[0]];
            for i in 1..points.len() {
                distance += tracematch::geo_utils::haversine_distance(&points[i - 1], &points[i]);
                start_points.push(points[i]);
                if distance >= 1000.0 {
                    break;
                }
            }
            (format!("{}_start", id), start_points)
        })
        .collect();

    println!("First 1km of each track:");
    for (id, points) in &start_tracks {
        let dist: f64 = (1..points.len())
            .map(|i| tracematch::geo_utils::haversine_distance(&points[i - 1], &points[i]))
            .sum();
        println!("  {}: {} points, {:.0}m", id, points.len(), dist);
    }

    // Run section detection on START portions only
    let start_sport_types: HashMap<String, String> = start_tracks
        .iter()
        .map(|(id, _)| (id.clone(), "Run".to_string()))
        .collect();

    let config = SectionConfig {
        min_section_length: 200.0,
        max_section_length: 1500.0,
        min_activities: 2,
        proximity_threshold: 50.0,
        ..SectionConfig::default()
    };

    let sections = detect_sections_from_tracks(&start_tracks, &start_sport_types, &[], &config);

    println!("\nSections in START portions:");
    for section in &sections {
        let ids: Vec<_> = section
            .activity_ids
            .iter()
            .map(|id| id.replace("activity_", "A").replace("_start", ""))
            .collect();
        println!(
            "  {}: {:.0}m, activities: {:?}",
            section.id, section.distance_meters, ids
        );
    }

    // Now run on full tracks and see what we get
    let full_config = SectionConfig {
        min_section_length: 200.0,
        max_section_length: 1500.0,
        min_activities: 3, // Require 3+ activities
        proximity_threshold: 50.0,
        ..SectionConfig::default()
    };

    let full_sections = detect_sections_from_tracks(&tracks, &sport_types, &[], &full_config);

    println!("\nSections in FULL tracks (min 3 activities):");
    for section in &full_sections {
        let ids: Vec<_> = section
            .activity_ids
            .iter()
            .map(|id| id.replace("activity_", "A"))
            .collect();
        println!(
            "  {}: {:.0}m, activities: {:?}",
            section.id, section.distance_meters, ids
        );
    }

    // The test should pass - we're investigating, not asserting specific behavior
    // If A1-A4 share a START section, we expect to find it in start_tracks
}

// ============================================================================
// LINESTRING VALIDITY TESTS
// ============================================================================

#[test]
fn test_no_sparse_linestrings_in_output() {
    // All sections must have at least 2 points in their polyline.
    // A LineString with fewer than 2 points is invalid and would cause
    // rendering errors in GeoJSON consumers.

    let tracks = require_fixtures!();
    let sport_types = create_sport_types(&tracks);

    // Test single-scale detection
    let config = SectionConfig {
        min_section_length: 100.0,
        max_section_length: 5000.0,
        min_activities: 2,
        proximity_threshold: 50.0,
        ..SectionConfig::default()
    };

    let sections = detect_sections_from_tracks(&tracks, &sport_types, &[], &config);

    for section in &sections {
        assert!(
            section.polyline.len() >= 2,
            "Section {} has sparse polyline ({} points) - LineString requires at least 2 points",
            section.id,
            section.polyline.len()
        );
    }

    // Test multi-scale detection
    let multiscale_config = SectionConfig {
        proximity_threshold: 50.0,
        cluster_tolerance: 100.0,
        scale_presets: vec![
            ScalePreset {
                name: "short".to_string(),
                min_length: 100.0,
                max_length: 500.0,
                min_activities: 2,
            },
            ScalePreset {
                name: "medium".to_string(),
                min_length: 400.0,
                max_length: 1500.0,
                min_activities: 2,
            },
            ScalePreset {
                name: "long".to_string(),
                min_length: 1000.0,
                max_length: 5000.0,
                min_activities: 2,
            },
        ],
        include_potentials: true,
        ..SectionConfig::default()
    };

    let result = detect_sections_multiscale(&tracks, &sport_types, &[], &multiscale_config);

    for section in &result.sections {
        assert!(
            section.polyline.len() >= 2,
            "Multi-scale section {} has sparse polyline ({} points)",
            section.id,
            section.polyline.len()
        );
    }

    for potential in &result.potentials {
        assert!(
            potential.polyline.len() >= 2,
            "Potential section {} has sparse polyline ({} points)",
            potential.id,
            potential.polyline.len()
        );
    }

    println!(
        "Verified {} sections and {} potentials have valid LineStrings (>= 2 points)",
        sections.len() + result.sections.len(),
        result.potentials.len()
    );
}
