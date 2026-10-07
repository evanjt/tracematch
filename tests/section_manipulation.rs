//! Integration tests for the new section manipulation functions.
//!
//! Tests:
//! - recalculate_section_polyline

use tracematch::{
    FrequentSection, GpsPoint, ScaleName, SectionConfig, recalculate_section_polyline,
};

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
        representative_range: None,
        activity_ids: vec![
            "trace1".to_string(),
            "trace2".to_string(),
            "trace3".to_string(),
        ],
        activity_portions: vec![],
        visit_count: 3,
        distance_meters: calculate_distance(&base_polyline),
        activity_traces,
        confidence: 0.8,
        observation_count: 3,
        average_spread: 10.0,
        point_density: vec![3; base_polyline.len()],
        scale: Some(ScaleName::Short),
        version: 1,
        is_user_defined: false,
        created_at: None,
        enrichment: Default::default(),
        rank: None,
        consensus_state: None,
        updated_at: None,
        stability: 0.5,
        elevation_gain_m: None,
        avg_grade_percent: None,
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
