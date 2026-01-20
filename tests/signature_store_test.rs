//! Integration tests for SignatureStore
//!
//! Tests extracted from src/engine/signature_store.rs

use tracematch::{GpsPoint, MatchConfig, engine::SignatureStore};

fn sample_coords() -> Vec<GpsPoint> {
    (0..10)
        .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
        .collect()
}

fn setup_store() -> tracematch::engine::ActivityStore {
    let mut store = tracematch::engine::ActivityStore::new();
    store.add("a".to_string(), sample_coords(), "cycling".to_string());
    store.add("b".to_string(), sample_coords(), "running".to_string());
    store
}

#[test]
fn test_mark_dirty_and_compute() {
    let store = setup_store();
    let mut cache = SignatureStore::new();
    let config = MatchConfig::default();

    cache.mark_dirty("a");
    cache.mark_dirty("b");

    assert!(cache.has_dirty());
    assert_eq!(cache.dirty_count(), 2);

    cache.ensure_computed(&store, &config);

    assert!(!cache.has_dirty());
    assert!(cache.has_newly_computed());
    assert_eq!(cache.len(), 2);
}

#[test]
fn test_get_computes_on_demand() {
    let store = setup_store();
    let mut cache = SignatureStore::new();
    let config = MatchConfig::default();

    cache.mark_dirty("a");

    // Get should compute the signature
    let sig = cache.get("a", &store, &config);
    assert!(sig.is_some());
    assert!(!cache.is_dirty("a"));
    assert!(cache.has_newly_computed());
}

#[test]
fn test_get_cached_does_not_compute() {
    // Store exists but isn't needed - get_cached doesn't trigger computation
    let _store = setup_store();
    let mut cache = SignatureStore::new();

    cache.mark_dirty("a");

    // get_cached should not compute
    let sig = cache.get_cached("a");
    assert!(sig.is_none());
    assert!(cache.is_dirty("a")); // Still dirty
}

#[test]
fn test_remove() {
    let store = setup_store();
    let mut cache = SignatureStore::new();
    let config = MatchConfig::default();

    cache.mark_dirty("a");
    cache.ensure_computed(&store, &config);

    assert_eq!(cache.len(), 1);

    cache.remove("a");

    assert_eq!(cache.len(), 0);
    assert!(!cache.has_newly_computed());
}

#[test]
fn test_clear_newly_computed() {
    let store = setup_store();
    let mut cache = SignatureStore::new();
    let config = MatchConfig::default();

    cache.mark_dirty("a");
    cache.mark_dirty("b");
    cache.ensure_computed(&store, &config);

    assert!(cache.has_newly_computed());
    assert_eq!(cache.newly_computed_count(), 2);

    cache.clear_newly_computed();

    assert!(!cache.has_newly_computed());
    assert_eq!(cache.len(), 2); // Signatures still stored
}

#[test]
fn test_newly_computed_vs_existing() {
    let store = setup_store();
    let mut cache = SignatureStore::new();
    let config = MatchConfig::default();

    // First batch
    cache.mark_dirty("a");
    cache.ensure_computed(&store, &config);
    cache.clear_newly_computed();

    // Second batch
    cache.mark_dirty("b");
    cache.ensure_computed(&store, &config);

    let newly = cache.newly_computed();
    let existing = cache.existing();

    assert_eq!(newly.len(), 1);
    assert_eq!(newly[0].activity_id, "b");
    assert_eq!(existing.len(), 1);
    assert_eq!(existing[0].activity_id, "a");
}

#[test]
fn test_mark_all_dirty() {
    let store = setup_store();
    let mut cache = SignatureStore::new();
    let config = MatchConfig::default();

    // Compute signatures
    cache.mark_dirty("a");
    cache.mark_dirty("b");
    cache.ensure_computed(&store, &config);

    assert_eq!(cache.len(), 2);

    // Mark all dirty (simulates config change)
    cache.mark_all_dirty(vec!["a".to_string(), "b".to_string()]);

    assert!(cache.is_empty()); // Store cleared
    assert_eq!(cache.dirty_count(), 2);
    assert!(!cache.has_newly_computed());
}

#[test]
fn test_nonexistent_activity() {
    // Use an empty store - the "nonexistent" activity won't be found
    let empty_store = tracematch::engine::ActivityStore::new();
    let mut cache = SignatureStore::new();
    let config = MatchConfig::default();

    cache.mark_dirty("nonexistent");
    cache.ensure_computed(&empty_store, &config);

    assert!(!cache.has_newly_computed());
    assert!(cache.is_empty());
}

#[test]
fn test_short_route_not_cached() {
    let mut store = tracematch::engine::ActivityStore::new();
    // Add a route too short to create a signature
    store.add(
        "short".to_string(),
        vec![GpsPoint::new(0.0, 0.0)],
        "cycling".to_string(),
    );

    let mut cache = SignatureStore::new();
    let config = MatchConfig::default();

    cache.mark_dirty("short");
    cache.ensure_computed(&store, &config);

    assert!(cache.is_empty()); // No signature created
}
