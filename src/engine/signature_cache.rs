//! Lazy signature computation with dirty tracking.
//!
//! Manages route signatures with:
//! - Lazy computation (only when needed)
//! - Dirty tracking for invalidation
//! - Tracking of "new" signatures for incremental grouping

use std::collections::{HashMap, HashSet};

use crate::{MatchConfig, RouteSignature};

use super::activity_store::ActivityStore;

/// Cache for route signatures with lazy computation.
///
/// Signatures are computed on-demand and tracked for incremental
/// grouping (new signatures vs existing ones).
#[derive(Debug, Default)]
pub struct SignatureCache {
    /// Computed signatures by activity ID
    signatures: HashMap<String, RouteSignature>,
    /// IDs that need signature recomputation
    dirty: HashSet<String>,
    /// IDs of signatures computed since last grouping
    newly_computed: HashSet<String>,
}

impl SignatureCache {
    /// Create a new empty signature cache.
    pub fn new() -> Self {
        Self {
            signatures: HashMap::new(),
            dirty: HashSet::new(),
            newly_computed: HashSet::new(),
        }
    }

    /// Mark an activity's signature as needing recomputation.
    pub fn mark_dirty(&mut self, id: &str) {
        self.dirty.insert(id.to_string());
    }

    /// Mark multiple activities as needing recomputation.
    pub fn mark_many_dirty(&mut self, ids: impl IntoIterator<Item = String>) {
        self.dirty.extend(ids);
    }

    /// Mark all signatures as dirty (e.g., after config change).
    pub fn mark_all_dirty(&mut self, ids: impl IntoIterator<Item = String>) {
        self.signatures.clear();
        self.newly_computed.clear();
        self.dirty = ids.into_iter().collect();
    }

    /// Remove a signature from the cache.
    pub fn remove(&mut self, id: &str) {
        self.signatures.remove(id);
        self.dirty.remove(id);
        self.newly_computed.remove(id);
    }

    /// Remove multiple signatures from the cache.
    pub fn remove_many(&mut self, ids: &[String]) {
        for id in ids {
            self.remove(id);
        }
    }

    /// Clear all cached signatures.
    pub fn clear(&mut self) {
        self.signatures.clear();
        self.dirty.clear();
        self.newly_computed.clear();
    }

    /// Clear the "newly computed" tracker.
    ///
    /// Call this after processing new signatures (e.g., after grouping).
    pub fn clear_newly_computed(&mut self) {
        self.newly_computed.clear();
    }

    /// Check if there are any dirty signatures.
    pub fn has_dirty(&self) -> bool {
        !self.dirty.is_empty()
    }

    /// Check if there are any newly computed signatures.
    pub fn has_newly_computed(&self) -> bool {
        !self.newly_computed.is_empty()
    }

    /// Get the number of dirty signatures.
    pub fn dirty_count(&self) -> usize {
        self.dirty.len()
    }

    /// Get the number of newly computed signatures.
    pub fn newly_computed_count(&self) -> usize {
        self.newly_computed.len()
    }

    /// Check if a specific signature is dirty.
    pub fn is_dirty(&self, id: &str) -> bool {
        self.dirty.contains(id)
    }

    /// Ensure all dirty signatures are computed.
    ///
    /// Newly computed signatures are tracked for incremental grouping.
    pub fn ensure_computed(&mut self, store: &ActivityStore, config: &MatchConfig) {
        if self.dirty.is_empty() {
            return;
        }

        let dirty_ids: Vec<String> = self.dirty.drain().collect();

        for id in dirty_ids {
            if let Some(activity) = store.get(&id) {
                if let Some(sig) =
                    RouteSignature::from_points(&activity.id, &activity.coords, config)
                {
                    self.signatures.insert(id.clone(), sig);
                    self.newly_computed.insert(id);
                }
            }
        }
    }

    /// Get a signature, computing it if necessary.
    pub fn get(
        &mut self,
        id: &str,
        store: &ActivityStore,
        config: &MatchConfig,
    ) -> Option<&RouteSignature> {
        // Compute if dirty
        if self.dirty.contains(id) {
            if let Some(activity) = store.get(id) {
                if let Some(sig) =
                    RouteSignature::from_points(&activity.id, &activity.coords, config)
                {
                    self.signatures.insert(id.to_string(), sig);
                    self.newly_computed.insert(id.to_string());
                }
            }
            self.dirty.remove(id);
        }
        self.signatures.get(id)
    }

    /// Get a signature without computing (returns None if not computed).
    pub fn get_cached(&self, id: &str) -> Option<&RouteSignature> {
        if self.dirty.contains(id) {
            None
        } else {
            self.signatures.get(id)
        }
    }

    /// Get all computed signatures.
    pub fn all(&self) -> impl Iterator<Item = &RouteSignature> {
        self.signatures.values()
    }

    /// Get all signatures as owned values.
    pub fn all_cloned(&self) -> Vec<RouteSignature> {
        self.signatures.values().cloned().collect()
    }

    /// Get the newly computed signatures.
    pub fn newly_computed(&self) -> Vec<RouteSignature> {
        self.newly_computed
            .iter()
            .filter_map(|id| self.signatures.get(id).cloned())
            .collect()
    }

    /// Get the existing (not newly computed) signatures.
    pub fn existing(&self) -> Vec<RouteSignature> {
        self.signatures
            .iter()
            .filter(|(id, _)| !self.newly_computed.contains(*id))
            .map(|(_, sig)| sig.clone())
            .collect()
    }

    /// Get the number of cached signatures.
    pub fn len(&self) -> usize {
        self.signatures.len()
    }

    /// Check if the cache is empty.
    pub fn is_empty(&self) -> bool {
        self.signatures.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GpsPoint;

    fn sample_coords() -> Vec<GpsPoint> {
        (0..10)
            .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
            .collect()
    }

    fn setup_store() -> ActivityStore {
        let mut store = ActivityStore::new();
        store.add("a".to_string(), sample_coords(), "cycling".to_string());
        store.add("b".to_string(), sample_coords(), "running".to_string());
        store
    }

    #[test]
    fn test_mark_dirty_and_compute() {
        let store = setup_store();
        let mut cache = SignatureCache::new();
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
        let mut cache = SignatureCache::new();
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
        let mut cache = SignatureCache::new();

        cache.mark_dirty("a");

        // get_cached should not compute
        let sig = cache.get_cached("a");
        assert!(sig.is_none());
        assert!(cache.is_dirty("a")); // Still dirty
    }

    #[test]
    fn test_remove() {
        let store = setup_store();
        let mut cache = SignatureCache::new();
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
        let mut cache = SignatureCache::new();
        let config = MatchConfig::default();

        cache.mark_dirty("a");
        cache.mark_dirty("b");
        cache.ensure_computed(&store, &config);

        assert!(cache.has_newly_computed());
        assert_eq!(cache.newly_computed_count(), 2);

        cache.clear_newly_computed();

        assert!(!cache.has_newly_computed());
        assert_eq!(cache.len(), 2); // Signatures still cached
    }

    #[test]
    fn test_newly_computed_vs_existing() {
        let store = setup_store();
        let mut cache = SignatureCache::new();
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
        let mut cache = SignatureCache::new();
        let config = MatchConfig::default();

        // Compute signatures
        cache.mark_dirty("a");
        cache.mark_dirty("b");
        cache.ensure_computed(&store, &config);

        assert_eq!(cache.len(), 2);

        // Mark all dirty (simulates config change)
        cache.mark_all_dirty(vec!["a".to_string(), "b".to_string()]);

        assert!(cache.is_empty()); // Cache cleared
        assert_eq!(cache.dirty_count(), 2);
        assert!(!cache.has_newly_computed());
    }

    #[test]
    fn test_nonexistent_activity() {
        // Use an empty store - the "nonexistent" activity won't be found
        let empty_store = ActivityStore::new();
        let mut cache = SignatureCache::new();
        let config = MatchConfig::default();

        cache.mark_dirty("nonexistent");
        cache.ensure_computed(&empty_store, &config);

        assert!(!cache.has_newly_computed());
        assert!(cache.is_empty());
    }

    #[test]
    fn test_short_route_not_cached() {
        let mut store = ActivityStore::new();
        // Add a route too short to create a signature
        store.add(
            "short".to_string(),
            vec![GpsPoint::new(0.0, 0.0)],
            "cycling".to_string(),
        );

        let mut cache = SignatureCache::new();
        let config = MatchConfig::default();

        cache.mark_dirty("short");
        cache.ensure_computed(&store, &config);

        assert!(cache.is_empty()); // No signature created
    }
}
