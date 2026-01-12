//! Tests for heatmap module

use std::collections::HashMap;
use tracematch::heatmap::*;
use tracematch::{Bounds, GpsPoint, RouteSignature};

fn make_signature(id: &str, points: Vec<(f64, f64)>) -> RouteSignature {
    let gps_points: Vec<GpsPoint> = points
        .iter()
        .map(|(lat, lng)| GpsPoint::new(*lat, *lng))
        .collect();

    let min_lat = points
        .iter()
        .map(|(lat, _)| *lat)
        .fold(f64::INFINITY, f64::min);
    let max_lat = points
        .iter()
        .map(|(lat, _)| *lat)
        .fold(f64::NEG_INFINITY, f64::max);
    let min_lng = points
        .iter()
        .map(|(_, lng)| *lng)
        .fold(f64::INFINITY, f64::min);
    let max_lng = points
        .iter()
        .map(|(_, lng)| *lng)
        .fold(f64::NEG_INFINITY, f64::max);

    let center_lat = points.iter().map(|(lat, _)| *lat).sum::<f64>() / points.len() as f64;
    let center_lng = points.iter().map(|(_, lng)| *lng).sum::<f64>() / points.len() as f64;

    RouteSignature {
        activity_id: id.to_string(),
        points: gps_points.clone(),
        total_distance: 1000.0,
        start_point: gps_points
            .first()
            .cloned()
            .unwrap_or(GpsPoint::new(0.0, 0.0)),
        end_point: gps_points
            .last()
            .cloned()
            .unwrap_or(GpsPoint::new(0.0, 0.0)),
        bounds: Bounds {
            min_lat,
            max_lat,
            min_lng,
            max_lng,
        },
        center: GpsPoint::new(center_lat, center_lng),
    }
}

#[test]
fn test_empty_heatmap() {
    let result = generate_heatmap(&[], &HashMap::new(), &HeatmapConfig::default());
    assert!(result.cells.is_empty());
    assert_eq!(result.total_activities, 0);
}

#[test]
fn test_single_activity() {
    let sig = make_signature(
        "act1",
        vec![
            (37.7749, -122.4194),
            (37.7750, -122.4195),
            (37.7751, -122.4196),
        ],
    );

    let mut data = HashMap::new();
    data.insert(
        "act1".to_string(),
        ActivityHeatmapData {
            activity_id: "act1".to_string(),
            route_id: None,
            route_name: None,
            timestamp: Some(1000000),
        },
    );

    let result = generate_heatmap(&[sig], &data, &HeatmapConfig::default());

    assert!(!result.cells.is_empty());
    assert_eq!(result.total_activities, 1);
    assert_eq!(result.total_routes, 0);
}

#[test]
fn test_multiple_activities_same_path() {
    let sig1 = make_signature("act1", vec![(37.7749, -122.4194), (37.7750, -122.4195)]);
    let sig2 = make_signature("act2", vec![(37.7749, -122.4194), (37.7750, -122.4195)]);

    let mut data = HashMap::new();
    data.insert(
        "act1".to_string(),
        ActivityHeatmapData {
            activity_id: "act1".to_string(),
            route_id: Some("route1".to_string()),
            route_name: Some("Morning Run".to_string()),
            timestamp: None,
        },
    );
    data.insert(
        "act2".to_string(),
        ActivityHeatmapData {
            activity_id: "act2".to_string(),
            route_id: Some("route1".to_string()),
            route_name: Some("Morning Run".to_string()),
            timestamp: None,
        },
    );

    let result = generate_heatmap(&[sig1, sig2], &data, &HeatmapConfig::default());

    assert!(!result.cells.is_empty());
    assert_eq!(result.total_activities, 2);
    assert_eq!(result.total_routes, 1);

    // Cells should have higher density due to overlapping paths
    let max_cell = result
        .cells
        .iter()
        .max_by(|a, b| a.visit_count.cmp(&b.visit_count))
        .unwrap();
    assert!(max_cell.visit_count >= 2);
}

#[test]
fn test_common_path_detection() {
    let sig1 = make_signature("act1", vec![(37.7749, -122.4194)]);
    let sig2 = make_signature("act2", vec![(37.7749, -122.4194)]);

    let mut data = HashMap::new();
    data.insert(
        "act1".to_string(),
        ActivityHeatmapData {
            activity_id: "act1".to_string(),
            route_id: Some("route1".to_string()),
            route_name: None,
            timestamp: None,
        },
    );
    data.insert(
        "act2".to_string(),
        ActivityHeatmapData {
            activity_id: "act2".to_string(),
            route_id: Some("route2".to_string()),
            route_name: None,
            timestamp: None,
        },
    );

    let result = generate_heatmap(&[sig1, sig2], &data, &HeatmapConfig::default());

    // Should detect cells where routes overlap
    let common_cells: Vec<_> = result.cells.iter().filter(|c| c.is_common_path).collect();
    assert!(!common_cells.is_empty());
}
