//! Tests for sections module

use tracematch::GpsPoint;
use tracematch::geo_utils::{compute_center, haversine_distance};

fn make_point(lat: f64, lng: f64) -> GpsPoint {
    GpsPoint::new(lat, lng)
}

#[test]
fn test_haversine_distance() {
    let p1 = make_point(51.5074, -0.1278); // London
    let p2 = make_point(48.8566, 2.3522); // Paris
    let dist = haversine_distance(&p1, &p2);
    // London to Paris is about 344 km
    assert!(dist > 340_000.0 && dist < 350_000.0);
}

#[test]
fn test_compute_center() {
    let points = vec![make_point(0.0, 0.0), make_point(2.0, 2.0)];
    let center = compute_center(&points);
    assert!((center.latitude - 1.0).abs() < 0.001);
    assert!((center.longitude - 1.0).abs() < 0.001);
}
