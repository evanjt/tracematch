//! Tests for grouping module

use tracematch::grouping::*;
use tracematch::{GpsPoint, MatchConfig, RouteSignature};

fn create_long_route() -> Vec<GpsPoint> {
    // Create a route long enough to meet min_route_distance (500m)
    // Each point is about 100m apart, 10 points = ~1km
    (0..10)
        .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
        .collect()
}

#[test]
fn test_group_identical_routes() {
    let long_route = create_long_route();

    let sig1 = RouteSignature::from_points("test-1", &long_route, &MatchConfig::default()).unwrap();
    let sig2 = RouteSignature::from_points("test-2", &long_route, &MatchConfig::default()).unwrap();

    let groups = group_signatures(&[sig1, sig2], &MatchConfig::default());

    // Should have 1 group with both routes
    assert_eq!(groups.len(), 1);
    assert_eq!(groups[0].activity_ids.len(), 2);
}

#[test]
fn test_group_different_routes() {
    let route1: Vec<GpsPoint> = (0..10)
        .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
        .collect();

    let route2: Vec<GpsPoint> = (0..10)
        .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
        .collect();

    let sig1 = RouteSignature::from_points("test-1", &route1, &MatchConfig::default()).unwrap();
    let sig2 = RouteSignature::from_points("test-2", &route2, &MatchConfig::default()).unwrap();

    let groups = group_signatures(&[sig1, sig2], &MatchConfig::default());

    // Should have 2 groups (different routes)
    assert_eq!(groups.len(), 2);
}

#[test]
fn test_distance_ratio_ok() {
    assert!(distance_ratio_ok(1000.0, 1200.0)); // 83% - ok
    assert!(distance_ratio_ok(1000.0, 500.0)); // 50% - ok
    assert!(!distance_ratio_ok(1000.0, 400.0)); // 40% - not ok
    assert!(!distance_ratio_ok(0.0, 1000.0)); // Invalid
}

/// Create a route with a deviation at a given position
fn create_route_with_deviation(deviation_meters: f64, deviation_position: f64) -> Vec<GpsPoint> {
    // 10 points, ~100m apart = ~1km total
    (0..10)
        .map(|i| {
            let progress = i as f64 / 9.0;
            let lat = 51.5074 + i as f64 * 0.001;
            let lng = if (progress - deviation_position).abs() < 0.1 {
                // Add lateral deviation (each 0.001 degree longitude ~ 70m at this latitude)
                -0.1278 + deviation_meters / 70000.0
            } else {
                -0.1278
            };
            GpsPoint::new(lat, lng)
        })
        .collect()
}

#[test]
fn test_grouping_deterministic() {
    // Run grouping multiple times - results should be identical
    let long_route = create_long_route();
    let deviated_route = create_route_with_deviation(30.0, 0.5);

    let results: Vec<_> = (0..5)
        .map(|_| {
            let sig1 =
                RouteSignature::from_points("a", &long_route, &MatchConfig::default()).unwrap();
            let sig2 =
                RouteSignature::from_points("b", &deviated_route, &MatchConfig::default()).unwrap();

            group_signatures_with_matches(&[sig1, sig2], &MatchConfig::default())
        })
        .collect();

    // All results should have same groups and representatives
    for i in 1..results.len() {
        assert_eq!(
            results[0].groups.len(),
            results[i].groups.len(),
            "Different group counts on run {i}"
        );

        for j in 0..results[0].groups.len() {
            assert_eq!(
                results[0].groups[j].representative_id, results[i].groups[j].representative_id,
                "Different representative on run {i}"
            );
        }
    }
}

#[test]
fn test_grouping_load_order_independence() {
    // Same routes loaded in different orders should produce same groups
    let route_a = create_long_route();
    let route_b = create_route_with_deviation(20.0, 0.3);
    let route_c = create_route_with_deviation(25.0, 0.6);

    // Different route (different location entirely)
    let route_d: Vec<GpsPoint> = (0..10)
        .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
        .collect();

    let config = MatchConfig::default();

    // Order 1: A, B, C, D
    let sigs_order1 = vec![
        RouteSignature::from_points("a", &route_a, &config).unwrap(),
        RouteSignature::from_points("b", &route_b, &config).unwrap(),
        RouteSignature::from_points("c", &route_c, &config).unwrap(),
        RouteSignature::from_points("d", &route_d, &config).unwrap(),
    ];

    // Order 2: D, C, B, A (reversed)
    let sigs_order2 = vec![
        RouteSignature::from_points("d", &route_d, &config).unwrap(),
        RouteSignature::from_points("c", &route_c, &config).unwrap(),
        RouteSignature::from_points("b", &route_b, &config).unwrap(),
        RouteSignature::from_points("a", &route_a, &config).unwrap(),
    ];

    // Order 3: B, D, A, C (shuffled)
    let sigs_order3 = vec![
        RouteSignature::from_points("b", &route_b, &config).unwrap(),
        RouteSignature::from_points("d", &route_d, &config).unwrap(),
        RouteSignature::from_points("a", &route_a, &config).unwrap(),
        RouteSignature::from_points("c", &route_c, &config).unwrap(),
    ];

    let result1 = group_signatures_with_matches(&sigs_order1, &config);
    let result2 = group_signatures_with_matches(&sigs_order2, &config);
    let result3 = group_signatures_with_matches(&sigs_order3, &config);

    // Helper to get sorted groups for comparison
    let get_sorted_groups = |result: &tracematch::GroupingResult| -> Vec<Vec<String>> {
        let mut groups: Vec<Vec<String>> = result
            .groups
            .iter()
            .map(|g| {
                let mut ids = g.activity_ids.clone();
                ids.sort();
                ids
            })
            .collect();
        groups.sort();
        groups
    };

    // Should have same number of groups
    assert_eq!(
        result1.groups.len(),
        result2.groups.len(),
        "Different group counts for order 1 vs 2"
    );
    assert_eq!(
        result1.groups.len(),
        result3.groups.len(),
        "Different group counts for order 1 vs 3"
    );

    // Each group should contain the same activities
    assert_eq!(
        get_sorted_groups(&result1),
        get_sorted_groups(&result2),
        "Groups differ for order 1 vs 2"
    );
    assert_eq!(
        get_sorted_groups(&result1),
        get_sorted_groups(&result3),
        "Groups differ for order 1 vs 3"
    );

    // Representatives should be the same (after determinism fix)
    let get_reps = |result: &tracematch::GroupingResult| -> Vec<String> {
        let mut reps: Vec<String> = result
            .groups
            .iter()
            .map(|g| g.representative_id.clone())
            .collect();
        reps.sort();
        reps
    };

    assert_eq!(
        get_reps(&result1),
        get_reps(&result2),
        "Representatives differ for order 1 vs 2"
    );
    assert_eq!(
        get_reps(&result1),
        get_reps(&result3),
        "Representatives differ for order 1 vs 3"
    );
}
