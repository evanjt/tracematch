//! Integration tests for ModularRouteEngine
//!
//! Tests extracted from src/engine/mod.rs

use tracematch::{GpsPoint, ModularRouteEngine};

fn sample_coords() -> Vec<GpsPoint> {
    (0..10)
        .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
        .collect()
}

fn different_coords() -> Vec<GpsPoint> {
    (0..10)
        .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
        .collect()
}

#[test]
fn test_add_and_query() {
    let mut engine = ModularRouteEngine::new();
    engine.add_activity("a".to_string(), sample_coords(), "cycling".to_string());

    assert_eq!(engine.activity_count(), 1);
    assert!(engine.has_activity("a"));
}

#[test]
fn test_grouping() {
    let mut engine = ModularRouteEngine::new();
    engine.add_activity("a".to_string(), sample_coords(), "cycling".to_string());
    engine.add_activity("b".to_string(), sample_coords(), "cycling".to_string());

    let groups = engine.get_groups();
    assert_eq!(groups.len(), 1);
    assert_eq!(groups[0].activity_ids.len(), 2);
}

#[test]
fn test_different_routes() {
    let mut engine = ModularRouteEngine::new();
    engine.add_activity("a".to_string(), sample_coords(), "cycling".to_string());
    engine.add_activity("b".to_string(), different_coords(), "running".to_string());

    let groups = engine.get_groups();
    assert_eq!(groups.len(), 2);
}

#[test]
fn test_viewport_query() {
    let mut engine = ModularRouteEngine::new();
    engine.add_activity("london".to_string(), sample_coords(), "cycling".to_string());
    engine.add_activity("nyc".to_string(), different_coords(), "running".to_string());

    let results = engine.query_viewport_raw(51.5, 51.52, -0.15, -0.10);
    assert_eq!(results.len(), 1);
    assert_eq!(results[0], "london");
}

#[test]
fn test_remove_activity() {
    let mut engine = ModularRouteEngine::new();
    engine.add_activity("a".to_string(), sample_coords(), "cycling".to_string());
    engine.add_activity("b".to_string(), sample_coords(), "cycling".to_string());

    engine.remove_activity("a");

    assert_eq!(engine.activity_count(), 1);
    assert!(!engine.has_activity("a"));
    assert!(engine.has_activity("b"));
}

#[test]
fn test_clear() {
    let mut engine = ModularRouteEngine::new();
    engine.add_activity("a".to_string(), sample_coords(), "cycling".to_string());
    engine.clear();

    assert_eq!(engine.activity_count(), 0);
}

#[test]
fn test_incremental_grouping() {
    let mut engine = ModularRouteEngine::new();

    // Initial batch
    engine.add_activity("a".to_string(), sample_coords(), "cycling".to_string());
    engine.add_activity("b".to_string(), sample_coords(), "cycling".to_string());

    let groups = engine.get_groups();
    assert_eq!(groups.len(), 1);
    assert_eq!(groups[0].activity_ids.len(), 2);

    // Add more
    engine.add_activity("c".to_string(), sample_coords(), "cycling".to_string());
    engine.add_activity("d".to_string(), different_coords(), "cycling".to_string());

    let groups = engine.get_groups();
    assert_eq!(groups.len(), 2);

    let large_group = groups.iter().find(|g| g.activity_ids.len() == 3);
    assert!(large_group.is_some());
}

#[test]
fn test_route_names() {
    let mut engine = ModularRouteEngine::new();
    engine.add_activity("a".to_string(), sample_coords(), "cycling".to_string());

    let groups = engine.get_groups();
    let group_id = groups[0].group_id.clone();

    engine.set_route_name(&group_id, "My Route");
    assert_eq!(
        engine.get_route_name(&group_id),
        Some(&"My Route".to_string())
    );

    let groups = engine.get_groups();
    assert_eq!(groups[0].custom_name, Some("My Route".to_string()));
}
