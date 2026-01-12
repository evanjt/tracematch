//! Integration tests for ActivityStore
//!
//! Tests extracted from src/engine/activity_store.rs

use tracematch::{engine::ActivityStore, GpsPoint};

fn sample_coords() -> Vec<GpsPoint> {
    (0..10)
        .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
        .collect()
}

#[test]
fn test_add_and_get() {
    let mut store = ActivityStore::new();
    store.add("test-1".to_string(), sample_coords(), "cycling".to_string());

    assert_eq!(store.len(), 1);
    assert!(store.contains("test-1"));

    let activity = store.get("test-1").unwrap();
    assert_eq!(activity.id, "test-1");
    assert_eq!(activity.sport_type, "cycling");
    assert_eq!(activity.coords.len(), 10);
}

#[test]
fn test_add_flat() {
    let mut store = ActivityStore::new();
    let flat_coords: Vec<f64> = sample_coords()
        .iter()
        .flat_map(|p| vec![p.latitude, p.longitude])
        .collect();

    store.add_flat("test-1".to_string(), &flat_coords, "running".to_string());

    assert_eq!(store.len(), 1);
    let activity = store.get("test-1").unwrap();
    assert_eq!(activity.coords.len(), 10);
}

#[test]
fn test_add_many_flat() {
    let mut store = ActivityStore::new();
    let coords1 = sample_coords();
    let coords2: Vec<GpsPoint> = (0..5)
        .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
        .collect();

    let mut all_coords = Vec::new();
    for p in &coords1 {
        all_coords.push(p.latitude);
        all_coords.push(p.longitude);
    }
    for p in &coords2 {
        all_coords.push(p.latitude);
        all_coords.push(p.longitude);
    }

    let activity_ids = vec!["a".to_string(), "b".to_string()];
    let offsets = vec![0, 10]; // First activity has 10 points
    let sport_types = vec!["cycling".to_string(), "running".to_string()];

    store.add_many_flat(&activity_ids, &all_coords, &offsets, &sport_types);

    assert_eq!(store.len(), 2);
    assert_eq!(store.get("a").unwrap().coords.len(), 10);
    assert_eq!(store.get("b").unwrap().coords.len(), 5);
}

#[test]
fn test_remove() {
    let mut store = ActivityStore::new();
    store.add("test-1".to_string(), sample_coords(), "cycling".to_string());
    store.add("test-2".to_string(), sample_coords(), "running".to_string());

    let removed = store.remove("test-1");
    assert!(removed.is_some());
    assert_eq!(store.len(), 1);
    assert!(!store.contains("test-1"));
    assert!(store.contains("test-2"));
}

#[test]
fn test_remove_many() {
    let mut store = ActivityStore::new();
    store.add("a".to_string(), sample_coords(), "cycling".to_string());
    store.add("b".to_string(), sample_coords(), "running".to_string());
    store.add("c".to_string(), sample_coords(), "swimming".to_string());

    let removed = store.remove_many(&["a".to_string(), "c".to_string(), "nonexistent".to_string()]);

    assert_eq!(removed.len(), 2);
    assert!(removed.contains(&"a".to_string()));
    assert!(removed.contains(&"c".to_string()));
    assert_eq!(store.len(), 1);
    assert!(store.contains("b"));
}

#[test]
fn test_clear() {
    let mut store = ActivityStore::new();
    store.add("a".to_string(), sample_coords(), "cycling".to_string());
    store.add("b".to_string(), sample_coords(), "running".to_string());

    store.clear();

    assert!(store.is_empty());
    assert_eq!(store.len(), 0);
}

#[test]
fn test_sport_type_map() {
    let mut store = ActivityStore::new();
    store.add("a".to_string(), sample_coords(), "cycling".to_string());
    store.add("b".to_string(), sample_coords(), "running".to_string());

    let map = store.sport_type_map();
    assert_eq!(map.get("a"), Some(&"cycling".to_string()));
    assert_eq!(map.get("b"), Some(&"running".to_string()));
}

#[test]
fn test_bounds_computed() {
    let mut store = ActivityStore::new();
    let coords = sample_coords();
    store.add("test".to_string(), coords.clone(), "cycling".to_string());

    let activity = store.get("test").unwrap();
    let bounds = activity.bounds.unwrap();

    // Verify bounds contain all points
    for p in &coords {
        assert!(p.latitude >= bounds.min_lat && p.latitude <= bounds.max_lat);
        assert!(p.longitude >= bounds.min_lng && p.longitude <= bounds.max_lng);
    }
}

#[test]
fn test_empty_coords() {
    let mut store = ActivityStore::new();
    let bounds = store.add("empty".to_string(), vec![], "cycling".to_string());

    assert!(bounds.is_none());
    assert!(store.contains("empty"));
}

#[test]
fn test_compute_track_distance() {
    let coords = sample_coords();
    let distance = ActivityStore::compute_track_distance(&coords);

    // Should be positive for a non-empty track
    assert!(distance > 0.0);

    // Empty track should have zero distance
    assert_eq!(ActivityStore::compute_track_distance(&[]), 0.0);

    // Single point should have zero distance
    assert_eq!(
        ActivityStore::compute_track_distance(&[GpsPoint::new(0.0, 0.0)]),
        0.0
    );
}
