//! A route signature keeps the elevation of each vertex it keeps.

use tracematch::{GpsPoint, MatchConfig, RouteSignature};

/// A winding climb: enough bends that simplification drops some vertices.
fn climb(count: usize) -> Vec<GpsPoint> {
    (0..count)
        .map(|i| {
            let t = i as f64;
            GpsPoint::with_elevation(
                46.5 + t * 0.0004,
                6.6 + (t * 0.5).sin() * 0.003,
                400.0 + t * 1.5,
            )
        })
        .collect()
}

fn bare(points: &[GpsPoint]) -> Vec<GpsPoint> {
    points
        .iter()
        .map(|p| GpsPoint::new(p.latitude, p.longitude))
        .collect()
}

#[test]
fn every_kept_vertex_carries_the_elevation_of_its_source_point() {
    let track = climb(200);
    let sig = RouteSignature::from_points("a", &track, &MatchConfig::default()).unwrap();
    assert!(sig.points.len() >= 2);
    assert!(sig.points.len() < track.len());
    for p in &sig.points {
        let source = track
            .iter()
            .find(|t| t.latitude == p.latitude && t.longitude == p.longitude)
            .expect("kept vertex is a source point");
        assert!(p.elevation.is_some());
        assert_eq!(p.elevation, source.elevation);
    }
    assert!(sig.start_point.elevation.is_some());
    assert!(sig.end_point.elevation.is_some());
}

#[test]
fn the_capped_signature_carries_elevation_too() {
    let track = climb(400);
    let config = MatchConfig {
        simplification_tolerance: 0.0,
        max_simplified_points: 25,
        ..MatchConfig::default()
    };
    let sig = RouteSignature::from_points("a", &track, &config).unwrap();
    assert_eq!(sig.points.len(), 25);
    assert!(sig.points.iter().all(|p| p.elevation.is_some()));
    assert_eq!(sig.points[0].elevation, track[0].elevation);
    assert_eq!(sig.points[24].elevation, track[399].elevation);
}

#[test]
fn elevation_leaves_the_kept_positions_unchanged() {
    let track = climb(400);
    let with = RouteSignature::from_points("a", &track, &MatchConfig::default()).unwrap();
    let without = RouteSignature::from_points("a", &bare(&track), &MatchConfig::default()).unwrap();
    let positions = |s: &RouteSignature| -> Vec<(f64, f64)> {
        s.points.iter().map(|p| (p.latitude, p.longitude)).collect()
    };
    assert_eq!(positions(&with), positions(&without));
    assert_eq!(with.total_distance, without.total_distance);
    assert!(without.points.iter().all(|p| p.elevation.is_none()));
}

#[test]
fn invalid_points_before_a_kept_vertex_do_not_shift_its_elevation() {
    let mut track = climb(60);
    track.insert(5, GpsPoint::with_elevation(f64::NAN, 6.6, 9999.0));
    track.insert(0, GpsPoint::with_elevation(200.0, 6.6, 9999.0));
    let sig = RouteSignature::from_points("a", &track, &MatchConfig::default()).unwrap();
    assert!(sig.points.iter().all(|p| p.elevation != Some(9999.0)));
    assert!(sig.points.iter().all(|p| p.elevation.is_some()));
}
