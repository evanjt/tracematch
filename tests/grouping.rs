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
