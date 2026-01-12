//! Tests for matching module

use tracematch::matching::*;
use tracematch::{GpsPoint, MatchConfig, RouteSignature};

fn sample_route() -> Vec<GpsPoint> {
    vec![
        GpsPoint::new(51.5074, -0.1278),
        GpsPoint::new(51.5080, -0.1290),
        GpsPoint::new(51.5090, -0.1300),
        GpsPoint::new(51.5100, -0.1310),
        GpsPoint::new(51.5110, -0.1320),
    ]
}

/// Generate a route with approximately the given distance in meters
fn generate_route_with_distance(distance_m: f64) -> Vec<GpsPoint> {
    // ~111km per degree latitude
    let total_degrees = distance_m / 111_000.0;
    let points = 100;
    (0..points)
        .map(|i| {
            let progress = i as f64 / points as f64;
            GpsPoint::new(51.5074 + progress * total_degrees, -0.1278)
        })
        .collect()
}

#[test]
fn test_amd_to_percentage() {
    // Below perfect threshold = 100%
    assert_eq!(amd_to_percentage(10.0, 30.0, 250.0), 100.0);

    // Above zero threshold = 0%
    assert_eq!(amd_to_percentage(300.0, 30.0, 250.0), 0.0);

    // At perfect threshold = 100%
    assert_eq!(amd_to_percentage(30.0, 30.0, 250.0), 100.0);

    // At zero threshold = 0%
    assert_eq!(amd_to_percentage(250.0, 30.0, 250.0), 0.0);

    // Midpoint
    let mid = amd_to_percentage(140.0, 30.0, 250.0);
    assert!(mid > 45.0 && mid < 55.0);
}

#[test]
fn test_resample_route() {
    let points = sample_route();
    let resampled = resample_route(&points, 10);
    assert_eq!(resampled.len(), 10);

    // First and last points should be preserved
    assert_eq!(resampled[0].latitude, points[0].latitude);
    assert_eq!(
        resampled.last().unwrap().latitude,
        points.last().unwrap().latitude
    );
}

#[test]
fn test_calculate_route_distance() {
    let points = sample_route();
    let distance = calculate_route_distance(&points);
    assert!(distance > 0.0);
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
}

// ========================================================================
// Distance-Proportional Resampling Tests
// ========================================================================

#[test]
fn test_resample_for_comparison_short_route() {
    // 1km route with 50m spacing -> 20 points (clamped to min)
    let points = generate_route_with_distance(1000.0);
    let config = MatchConfig::default();

    let resampled = resample_for_comparison(&points, 1000.0, &config);

    // 1000m / 50m = 20 points (at minimum)
    assert_eq!(resampled.len(), 20);
}

#[test]
fn test_resample_for_comparison_medium_route() {
    // 5km route with 50m spacing -> 100 points
    let points = generate_route_with_distance(5000.0);
    let config = MatchConfig::default();

    let resampled = resample_for_comparison(&points, 5000.0, &config);

    // 5000m / 50m = 100 points
    assert_eq!(resampled.len(), 100);
}

#[test]
fn test_resample_for_comparison_long_route() {
    // 50km route with 50m spacing -> 200 points (clamped to max)
    let points = generate_route_with_distance(50000.0);
    let config = MatchConfig::default();

    let resampled = resample_for_comparison(&points, 50000.0, &config);

    // 50000m / 50m = 1000, but clamped to max 200
    assert_eq!(resampled.len(), 200);
}

#[test]
fn test_resample_for_comparison_consistent_spacing() {
    // Two routes of different lengths should have consistent point spacing
    let short_route = generate_route_with_distance(2000.0); // 2km
    let long_route = generate_route_with_distance(8000.0); // 8km
    let config = MatchConfig::default();

    let resampled_short = resample_for_comparison(&short_route, 2000.0, &config);
    let resampled_long = resample_for_comparison(&long_route, 8000.0, &config);

    // Short: 2000/50 = 40 points
    // Long: 8000/50 = 160 points
    assert_eq!(resampled_short.len(), 40);
    assert_eq!(resampled_long.len(), 160);

    // Both should have ~50m spacing between points
    let spacing_short = 2000.0 / (resampled_short.len() - 1) as f64;
    let spacing_long = 8000.0 / (resampled_long.len() - 1) as f64;

    assert!(
        (spacing_short - 50.0).abs() < 5.0_f64,
        "Short route spacing: {}",
        spacing_short
    );
    assert!(
        (spacing_long - 50.0).abs() < 5.0_f64,
        "Long route spacing: {}",
        spacing_long
    );
}

#[test]
fn test_resample_for_comparison_legacy_mode() {
    // When resample_spacing_meters is 0, use legacy fixed count
    let points = generate_route_with_distance(5000.0);
    let mut config = MatchConfig::default();
    config.resample_spacing_meters = 0.0;
    config.resample_count = 75;

    let resampled = resample_for_comparison(&points, 5000.0, &config);

    assert_eq!(resampled.len(), 75);
}

#[test]
fn test_resample_for_comparison_custom_spacing() {
    // Custom 100m spacing
    let points = generate_route_with_distance(5000.0);
    let mut config = MatchConfig::default();
    config.resample_spacing_meters = 100.0;

    let resampled = resample_for_comparison(&points, 5000.0, &config);

    // 5000m / 100m = 50 points
    assert_eq!(resampled.len(), 50);
}

#[test]
fn test_compare_routes_different_lengths() {
    // Test that routes of different lengths can still match
    // when they represent the same path
    let short_points = generate_route_with_distance(5000.0);
    let long_points = generate_route_with_distance(5000.0);

    let sig1 =
        RouteSignature::from_points("short", &short_points, &MatchConfig::default()).unwrap();
    let sig2 = RouteSignature::from_points("long", &long_points, &MatchConfig::default()).unwrap();

    let result = compare_routes(&sig1, &sig2, &MatchConfig::default());
    assert!(result.is_some());
    assert!(result.unwrap().match_percentage > 95.0);
}
