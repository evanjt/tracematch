//! Integration tests for RouteGrouper
//!
//! Tests extracted from src/engine/route_grouper.rs

use tracematch::{engine::RouteGrouper, GpsPoint, MatchConfig};

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

fn setup_store_and_cache() -> (
    tracematch::engine::ActivityStore,
    tracematch::engine::SignatureStore,
) {
    let mut store = tracematch::engine::ActivityStore::new();
    store.add("a".to_string(), sample_coords(), "cycling".to_string());
    store.add("b".to_string(), sample_coords(), "cycling".to_string());

    let mut cache = tracematch::engine::SignatureStore::new();
    cache.mark_dirty("a");
    cache.mark_dirty("b");

    (store, cache)
}

#[test]
fn test_group_identical_routes() {
    let (store, mut cache) = setup_store_and_cache();
    let mut grouper = RouteGrouper::new();
    let config = MatchConfig::default();

    grouper.mark_dirty();
    grouper.ensure_computed(&mut cache, &store, &config);

    assert_eq!(grouper.len(), 1);
    assert_eq!(grouper.groups()[0].activity_ids.len(), 2);
}

#[test]
fn test_group_different_routes() {
    let mut store = tracematch::engine::ActivityStore::new();
    store.add("a".to_string(), sample_coords(), "cycling".to_string());
    store.add("b".to_string(), different_coords(), "running".to_string());

    let mut cache = tracematch::engine::SignatureStore::new();
    cache.mark_dirty("a");
    cache.mark_dirty("b");

    let mut grouper = RouteGrouper::new();
    let config = MatchConfig::default();

    grouper.mark_dirty();
    grouper.ensure_computed(&mut cache, &store, &config);

    assert_eq!(grouper.len(), 2);
}

#[test]
fn test_get_group_for_activity() {
    let (store, mut cache) = setup_store_and_cache();
    let mut grouper = RouteGrouper::new();
    let config = MatchConfig::default();

    grouper.mark_dirty();
    grouper.ensure_computed(&mut cache, &store, &config);

    let group = grouper.get_group_for_activity("a");
    assert!(group.is_some());
    assert!(group.unwrap().activity_ids.contains(&"a".to_string()));
    assert!(group.unwrap().activity_ids.contains(&"b".to_string()));
}

#[test]
fn test_custom_route_names() {
    let (store, mut cache) = setup_store_and_cache();
    let mut grouper = RouteGrouper::new();
    let config = MatchConfig::default();

    grouper.mark_dirty();
    grouper.ensure_computed(&mut cache, &store, &config);

    let group_id = grouper.groups()[0].group_id.clone();

    grouper.set_route_name(&group_id, "My Favorite Route");
    assert_eq!(
        grouper.get_route_name(&group_id),
        Some(&"My Favorite Route".to_string())
    );

    // Check it's applied to the group
    assert_eq!(
        grouper.groups()[0].custom_name,
        Some("My Favorite Route".to_string())
    );

    // Clear the name
    grouper.set_route_name(&group_id, "");
    assert!(grouper.get_route_name(&group_id).is_none());
    assert!(grouper.groups()[0].custom_name.is_none());
}

#[test]
fn test_dirty_tracking() {
    let (store, mut cache) = setup_store_and_cache();
    let mut grouper = RouteGrouper::new();
    let config = MatchConfig::default();

    assert!(!grouper.is_dirty());

    grouper.mark_dirty();
    assert!(grouper.is_dirty());

    grouper.ensure_computed(&mut cache, &store, &config);
    assert!(!grouper.is_dirty());
}

#[test]
fn test_invalidate() {
    let (store, mut cache) = setup_store_and_cache();
    let mut grouper = RouteGrouper::new();
    let config = MatchConfig::default();

    grouper.mark_dirty();
    grouper.ensure_computed(&mut cache, &store, &config);

    assert_eq!(grouper.len(), 1);

    grouper.invalidate();

    assert!(grouper.is_empty());
    assert!(grouper.is_dirty());
}

#[test]
fn test_sport_type_inherited() {
    let mut store = tracematch::engine::ActivityStore::new();
    store.add("a".to_string(), sample_coords(), "cycling".to_string());

    let mut cache = tracematch::engine::SignatureStore::new();
    cache.mark_dirty("a");

    let mut grouper = RouteGrouper::new();
    let config = MatchConfig::default();

    grouper.mark_dirty();
    grouper.ensure_computed(&mut cache, &store, &config);

    assert_eq!(grouper.groups()[0].sport_type, "cycling");
}

#[test]
fn test_incremental_grouping() {
    let mut store = tracematch::engine::ActivityStore::new();
    store.add("a".to_string(), sample_coords(), "cycling".to_string());
    store.add("b".to_string(), sample_coords(), "cycling".to_string());

    let mut cache = tracematch::engine::SignatureStore::new();
    let mut grouper = RouteGrouper::new();
    let config = MatchConfig::default();

    // First batch
    cache.mark_dirty("a");
    cache.mark_dirty("b");
    grouper.mark_dirty();
    grouper.ensure_computed(&mut cache, &store, &config);

    assert_eq!(grouper.len(), 1);
    assert_eq!(grouper.groups()[0].activity_ids.len(), 2);

    // Add more similar activities
    store.add("c".to_string(), sample_coords(), "cycling".to_string());
    cache.mark_dirty("c");
    grouper.mark_dirty();
    grouper.ensure_computed(&mut cache, &store, &config);

    // Should still be one group with 3 activities
    assert_eq!(grouper.len(), 1);
    assert_eq!(grouper.groups()[0].activity_ids.len(), 3);

    // Add a different route
    store.add("d".to_string(), different_coords(), "cycling".to_string());
    cache.mark_dirty("d");
    grouper.mark_dirty();
    grouper.ensure_computed(&mut cache, &store, &config);

    // Should now have 2 groups
    assert_eq!(grouper.len(), 2);
}
