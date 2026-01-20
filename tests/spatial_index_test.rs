//! Integration tests for SpatialIndex
//!
//! Tests extracted from src/engine/spatial_index.rs

use tracematch::{GpsPoint, engine::SpatialIndex};

fn sample_coords_london() -> Vec<GpsPoint> {
    (0..10)
        .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
        .collect()
}

fn sample_coords_nyc() -> Vec<GpsPoint> {
    (0..10)
        .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
        .collect()
}

fn setup_store() -> tracematch::engine::ActivityStore {
    let mut store = tracematch::engine::ActivityStore::new();
    store.add(
        "london".to_string(),
        sample_coords_london(),
        "cycling".to_string(),
    );
    store.add(
        "nyc".to_string(),
        sample_coords_nyc(),
        "running".to_string(),
    );
    store
}

#[test]
fn test_build_and_query() {
    let store = setup_store();
    let mut index = SpatialIndex::new();

    index.mark_dirty();
    index.ensure_built(&store);

    assert_eq!(index.len(), 2);
    assert!(!index.is_dirty());
}

#[test]
fn test_query_viewport_london() {
    let store = setup_store();
    let mut index = SpatialIndex::new();
    index.rebuild(&store);

    // Query around London
    let results = index.query_viewport_raw(51.5, 51.52, -0.15, -0.10);
    assert_eq!(results.len(), 1);
    assert_eq!(results[0], "london");
}

#[test]
fn test_query_viewport_nyc() {
    let store = setup_store();
    let mut index = SpatialIndex::new();
    index.rebuild(&store);

    // Query around NYC
    let results = index.query_viewport_raw(40.7, 40.75, -74.1, -74.0);
    assert_eq!(results.len(), 1);
    assert_eq!(results[0], "nyc");
}

#[test]
fn test_query_viewport_empty() {
    let store = setup_store();
    let mut index = SpatialIndex::new();
    index.rebuild(&store);

    // Query in empty area (Tokyo)
    let results = index.query_viewport_raw(35.6, 35.7, 139.6, 139.8);
    assert!(results.is_empty());
}

#[test]
fn test_find_nearby() {
    let store = setup_store();
    let mut index = SpatialIndex::new();
    index.rebuild(&store);

    // Find near London
    let results = index.find_nearby(51.5074, -0.1278, 0.1);
    assert_eq!(results.len(), 1);
    assert_eq!(results[0], "london");
}

#[test]
fn test_dirty_tracking() {
    let store = setup_store();
    let mut index = SpatialIndex::new();

    assert!(!index.is_dirty());

    index.mark_dirty();
    assert!(index.is_dirty());

    index.ensure_built(&store);
    assert!(!index.is_dirty());
}

#[test]
fn test_clear() {
    let store = setup_store();
    let mut index = SpatialIndex::new();
    index.rebuild(&store);

    assert_eq!(index.len(), 2);

    index.clear();

    assert!(index.is_empty());
    assert!(!index.is_dirty());
}

#[test]
fn test_query_both_cities() {
    let store = setup_store();
    let mut index = SpatialIndex::new();
    index.rebuild(&store);

    // Query covering both London and NYC (impossible in real coords, but tests the logic)
    let results = index.query_viewport_raw(-90.0, 90.0, -180.0, 180.0);
    assert_eq!(results.len(), 2);
}
