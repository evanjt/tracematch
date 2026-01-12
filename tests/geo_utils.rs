//! Tests for geo_utils module

use tracematch::{Bounds, GpsPoint};
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

#[test]
fn test_bounds_overlap_yes() {
    let a = Bounds {
        min_lat: 51.50,
        max_lat: 51.52,
        min_lng: -0.13,
        max_lng: -0.11,
    };
    let b = Bounds {
        min_lat: 51.51,
        max_lat: 51.53,
        min_lng: -0.12,
        max_lng: -0.10,
    };
    assert!(bounds_overlap(&a, &b, 0.0, 51.5));
}

#[test]
fn test_bounds_overlap_no() {
    let a = Bounds {
        min_lat: 51.50,
        max_lat: 51.51,
        min_lng: -0.13,
        max_lng: -0.12,
    };
    let b = Bounds {
        min_lat: 51.52,
        max_lat: 51.53,
        min_lng: -0.11,
        max_lng: -0.10,
    };
    assert!(!bounds_overlap(&a, &b, 0.0, 51.5));
}

#[test]
fn test_bounds_overlap_with_buffer() {
    let a = Bounds {
        min_lat: 51.50,
        max_lat: 51.51,
        min_lng: -0.13,
        max_lng: -0.12,
    };
    let b = Bounds {
        min_lat: 51.52,
        max_lat: 51.53,
        min_lng: -0.11,
        max_lng: -0.10,
    };
    // With large buffer (5km), these should overlap
    assert!(bounds_overlap(&a, &b, 5000.0, 51.5));
}

#[test]
fn test_meters_to_degrees() {
    // At equator, 111km = 1 degree
    let deg = meters_to_degrees(111_320.0, 0.0);
    assert!(approx_eq(deg, 1.0, 0.01));

    // At higher latitude, same distance = more degrees
    let deg_45 = meters_to_degrees(111_320.0, 45.0);
    assert!(deg_45 > 1.0);
}
