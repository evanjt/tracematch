//! Route grouping with incremental updates.
//!
//! Groups similar routes using Union-Find clustering with support for:
//! - Incremental grouping (add new routes without full recomputation)
//! - Custom route names
//! - Sport type inheritance

use std::collections::HashMap;

use crate::{MatchConfig, RouteGroup};

#[cfg(not(feature = "parallel"))]
use crate::group_signatures;

#[cfg(feature = "parallel")]
use crate::{group_incremental, group_signatures_parallel};

use super::activity_store::ActivityStore;
use super::signature_cache::SignatureCache;

/// Route grouper with incremental update support.
///
/// Maintains groups of similar routes and supports efficient
/// incremental updates when new routes are added.
#[derive(Debug, Default)]
pub struct RouteGrouper {
    /// Current route groups
    groups: Vec<RouteGroup>,
    /// Custom names for routes (route_id -> custom_name)
    route_names: HashMap<String, String>,
    /// Whether groups need recomputation
    dirty: bool,
}

impl RouteGrouper {
    /// Create a new empty route grouper.
    pub fn new() -> Self {
        Self {
            groups: Vec::new(),
            route_names: HashMap::new(),
            dirty: false,
        }
    }

    /// Mark groups as needing recomputation.
    pub fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    /// Check if groups need recomputation.
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// Force full recomputation (e.g., after activity removal).
    pub fn invalidate(&mut self) {
        self.groups.clear();
        self.dirty = true;
    }

    /// Clear all groups and names.
    pub fn clear(&mut self) {
        self.groups.clear();
        self.route_names.clear();
        self.dirty = false;
    }

    /// Ensure groups are computed.
    ///
    /// Uses incremental grouping when possible (new signatures only),
    /// falls back to full recomputation when necessary.
    pub fn ensure_computed(
        &mut self,
        cache: &mut SignatureCache,
        store: &ActivityStore,
        config: &MatchConfig,
    ) {
        if !self.dirty {
            return;
        }

        // Ensure all signatures are computed
        cache.ensure_computed(store, config);

        #[cfg(feature = "parallel")]
        {
            // Check if we can use incremental grouping
            let can_use_incremental = !self.groups.is_empty()
                && cache.has_newly_computed()
                && cache.len() > cache.newly_computed_count();

            if can_use_incremental {
                // Incremental: only compare new signatures vs existing + new vs new
                let new_sigs = cache.newly_computed();
                let existing_sigs = cache.existing();

                self.groups = group_incremental(&new_sigs, &self.groups, &existing_sigs, config);
            } else {
                // Full recomputation needed
                let signatures = cache.all_cloned();
                self.groups = group_signatures_parallel(&signatures, config);
            }
        }

        #[cfg(not(feature = "parallel"))]
        {
            // Non-parallel: always use full grouping (incremental requires rayon)
            let signatures = cache.all_cloned();
            self.groups = group_signatures(&signatures, config);
        }

        // Clear newly computed tracker
        cache.clear_newly_computed();

        // Populate sport_type and custom_name for each group
        for group in &mut self.groups {
            if let Some(activity) = store.get(&group.representative_id) {
                group.sport_type = activity.sport_type.clone();
            }
            if let Some(name) = self.route_names.get(&group.group_id) {
                group.custom_name = Some(name.clone());
            }
        }

        self.dirty = false;
    }

    /// Get all route groups.
    pub fn groups(&self) -> &[RouteGroup] {
        &self.groups
    }

    /// Get the group containing a specific activity.
    pub fn get_group_for_activity(&self, activity_id: &str) -> Option<&RouteGroup> {
        self.groups
            .iter()
            .find(|g| g.activity_ids.contains(&activity_id.to_string()))
    }

    /// Get a group by its ID.
    pub fn get_group(&self, group_id: &str) -> Option<&RouteGroup> {
        self.groups.iter().find(|g| g.group_id == group_id)
    }

    /// Get the number of groups.
    pub fn len(&self) -> usize {
        self.groups.len()
    }

    /// Check if there are no groups.
    pub fn is_empty(&self) -> bool {
        self.groups.is_empty()
    }

    /// Set a custom name for a route.
    ///
    /// Pass empty string to clear the custom name.
    pub fn set_route_name(&mut self, route_id: &str, name: &str) {
        if name.is_empty() {
            self.route_names.remove(route_id);
            // Update in-memory group
            if let Some(group) = self.groups.iter_mut().find(|g| g.group_id == route_id) {
                group.custom_name = None;
            }
        } else {
            self.route_names.insert(route_id.to_string(), name.to_string());
            // Update in-memory group
            if let Some(group) = self.groups.iter_mut().find(|g| g.group_id == route_id) {
                group.custom_name = Some(name.to_string());
            }
        }
    }

    /// Get the custom name for a route.
    pub fn get_route_name(&self, route_id: &str) -> Option<&String> {
        self.route_names.get(route_id)
    }

    /// Get all custom route names.
    pub fn route_names(&self) -> &HashMap<String, String> {
        &self.route_names
    }

    /// Set route names from a map (for deserialization).
    pub fn set_route_names(&mut self, names: HashMap<String, String>) {
        self.route_names = names;
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

    fn different_coords() -> Vec<GpsPoint> {
        (0..10)
            .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
            .collect()
    }

    fn setup_store_and_cache() -> (ActivityStore, SignatureCache) {
        let mut store = ActivityStore::new();
        store.add("a".to_string(), sample_coords(), "cycling".to_string());
        store.add("b".to_string(), sample_coords(), "cycling".to_string());

        let mut cache = SignatureCache::new();
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
        let mut store = ActivityStore::new();
        store.add("a".to_string(), sample_coords(), "cycling".to_string());
        store.add("b".to_string(), different_coords(), "running".to_string());

        let mut cache = SignatureCache::new();
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
        assert_eq!(grouper.get_route_name(&group_id), Some(&"My Favorite Route".to_string()));

        // Check it's applied to the group
        assert_eq!(grouper.groups()[0].custom_name, Some("My Favorite Route".to_string()));

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
        let mut store = ActivityStore::new();
        store.add("a".to_string(), sample_coords(), "cycling".to_string());

        let mut cache = SignatureCache::new();
        cache.mark_dirty("a");

        let mut grouper = RouteGrouper::new();
        let config = MatchConfig::default();

        grouper.mark_dirty();
        grouper.ensure_computed(&mut cache, &store, &config);

        assert_eq!(grouper.groups()[0].sport_type, "cycling");
    }

    #[test]
    fn test_incremental_grouping() {
        let mut store = ActivityStore::new();
        store.add("a".to_string(), sample_coords(), "cycling".to_string());
        store.add("b".to_string(), sample_coords(), "cycling".to_string());

        let mut cache = SignatureCache::new();
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
}
