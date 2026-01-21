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
use super::signature_store::SignatureStore;

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
        cache: &mut SignatureStore,
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
                group.sport_type = if activity.sport_type.is_empty() {
                    "Ride".to_string() // Default for empty sport type
                } else {
                    activity.sport_type.clone()
                };
            } else {
                // Representative activity not found - use default
                group.sport_type = "Ride".to_string();
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
            self.route_names
                .insert(route_id.to_string(), name.to_string());
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
