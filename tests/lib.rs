//! Tests for lib.rs core types and functions

use tracematch::{Direction, GpsPoint, MatchConfig, RouteSignature, compare_routes, group_signatures};

fn sample_route() -> Vec<GpsPoint> {
    vec![
        GpsPoint::new(51.5074, -0.1278),
        GpsPoint::new(51.5080, -0.1290),
        GpsPoint::new(51.5090, -0.1300),
        GpsPoint::new(51.5100, -0.1310),
        GpsPoint::new(51.5110, -0.1320),
    ]
}

#[test]
fn test_gps_point_validation() {
    assert!(GpsPoint::new(51.5074, -0.1278).is_valid());
    assert!(!GpsPoint::new(91.0, 0.0).is_valid());
    assert!(!GpsPoint::new(0.0, 181.0).is_valid());
    assert!(!GpsPoint::new(f64::NAN, 0.0).is_valid());
}

#[test]
fn test_create_signature() {
    let points = sample_route();
    let sig = RouteSignature::from_points("test-1", &points, &MatchConfig::default());

    assert!(sig.is_some());
    let sig = sig.unwrap();
    assert_eq!(sig.activity_id, "test-1");
    assert!(sig.total_distance > 0.0);
}

#[test]
fn test_identical_routes_match() {
    let points = sample_route();
    let sig1 = RouteSignature::from_points("test-1", &points, &MatchConfig::default()).unwrap();
    let sig2 = RouteSignature::from_points("test-2", &points, &MatchConfig::default()).unwrap();

    let result = compare_routes(&sig1, &sig2, &MatchConfig::default());
    assert!(result.is_some());
    let result = result.unwrap();
    assert!(result.match_percentage > 95.0);
    // Direction is Same when routes go the same direction
    assert_eq!(result.direction, Direction::Same);
}

#[test]
fn test_reverse_routes_match() {
    let points = sample_route();
    let mut reversed = points.clone();
    reversed.reverse();

    let sig1 = RouteSignature::from_points("test-1", &points, &MatchConfig::default()).unwrap();
    let sig2 = RouteSignature::from_points("test-2", &reversed, &MatchConfig::default()).unwrap();

    let result = compare_routes(&sig1, &sig2, &MatchConfig::default());
    assert!(result.is_some());
    assert_eq!(result.unwrap().direction, Direction::Reverse);
}

#[test]
fn test_group_signatures() {
    // Create a longer route that meets min_route_distance (500m)
    // Each point is about 100m apart, 10 points = ~1km
    let long_route: Vec<GpsPoint> = (0..10)
        .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
        .collect();

    let different_route: Vec<GpsPoint> = (0..10)
        .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
        .collect();

    let sig1 = RouteSignature::from_points("test-1", &long_route, &MatchConfig::default()).unwrap();
    let sig2 = RouteSignature::from_points("test-2", &long_route, &MatchConfig::default()).unwrap();
    let sig3 =
        RouteSignature::from_points("test-3", &different_route, &MatchConfig::default()).unwrap();

    let groups = group_signatures(&[sig1, sig2, sig3], &MatchConfig::default());

    // Should have 2 groups: one with test-1 and test-2, one with test-3
    assert_eq!(groups.len(), 2);

    // Verify the grouping is correct
    let group_with_1 = groups
        .iter()
        .find(|g| g.activity_ids.contains(&"test-1".to_string()))
        .unwrap();
    assert!(group_with_1.activity_ids.contains(&"test-2".to_string()));
    assert!(!group_with_1.activity_ids.contains(&"test-3".to_string()));
}
