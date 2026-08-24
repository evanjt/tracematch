//! Tests for geo_utils module

use tracematch::GpsPoint;
use tracematch::geo_utils::*;

fn approx_eq(a: f64, b: f64, epsilon: f64) -> bool {
    (a - b).abs() < epsilon
}

#[test]
fn test_haversine_distance_same_point() {
    let p = GpsPoint::new(51.5074, -0.1278);
    assert_eq!(haversine_distance(&p, &p), 0.0);
}

#[test]
fn test_haversine_distance_known_value() {
    // London to Paris is approximately 344 km
    let london = GpsPoint::new(51.5074, -0.1278);
    let paris = GpsPoint::new(48.8566, 2.3522);
    let dist = haversine_distance(&london, &paris);
    assert!(approx_eq(dist, 343_560.0, 5000.0)); // Within 5km
}

#[test]
fn test_compute_bounds() {
    let track = vec![
        GpsPoint::new(51.50, -0.13),
        GpsPoint::new(51.51, -0.12),
        GpsPoint::new(51.505, -0.125),
    ];
    let bounds = compute_bounds(&track);
    assert_eq!(bounds.min_lat, 51.50);
    assert_eq!(bounds.max_lat, 51.51);
    assert_eq!(bounds.min_lng, -0.13);
    assert_eq!(bounds.max_lng, -0.12);
}

#[test]
fn test_compute_center() {
    let track = vec![GpsPoint::new(51.50, -0.10), GpsPoint::new(51.52, -0.12)];
    let center = compute_center(&track);
    assert!(approx_eq(center.latitude, 51.51, 0.001));
    assert!(approx_eq(center.longitude, -0.11, 0.001));
}

#[test]
fn test_compute_center_empty() {
    let empty: Vec<GpsPoint> = vec![];
    let center = compute_center(&empty);
    assert_eq!(center.latitude, 0.0);
    assert_eq!(center.longitude, 0.0);
}
