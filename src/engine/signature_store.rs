//! Lazy signature computation with dirty tracking.
//!
//! Manages route signatures with:
//! - Lazy computation (only when needed)
//! - Dirty tracking for invalidation
//! - Tracking of "new" signatures for incremental grouping

use std::collections::{HashMap, HashSet};

use crate::{MatchConfig, RouteSignature};

use super::activity_store::ActivityStore;

/// Store for lazy signature computation with dirty tracking.
///
/// This is NOT a cache - it's a computation tracker that:
/// - Computes signatures on-demand (lazy evaluation)
/// - Tracks which signatures need recomputation (dirty tracking)
/// - Distinguishes new vs existing signatures for incremental grouping
#[derive(Debug, Default)]
pub struct SignatureStore {
    /// Computed signatures by activity ID
    signatures: HashMap<String, RouteSignature>,
    /// IDs that need signature recomputation
    dirty: HashSet<String>,
    /// IDs of signatures computed since last grouping
    newly_computed: HashSet<String>,
}

impl SignatureStore {
    /// Create a new empty signature store.
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

    /// Remove a signature from the store.
    pub fn remove(&mut self, id: &str) {
        self.signatures.remove(id);
        self.dirty.remove(id);
        self.newly_computed.remove(id);
    }

    /// Remove multiple signatures from the store.
    pub fn remove_many(&mut self, ids: &[String]) {
        for id in ids {
            self.remove(id);
        }
    }

    /// Clear all computed signatures.
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
            if let Some(activity) = store.get(&id)
                && let Some(sig) =
                    RouteSignature::from_points(&activity.id, &activity.coords, config)
            {
                self.signatures.insert(id.clone(), sig);
                self.newly_computed.insert(id);
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
            if let Some(activity) = store.get(id)
                && let Some(sig) =
                    RouteSignature::from_points(&activity.id, &activity.coords, config)
            {
                self.signatures.insert(id.to_string(), sig);
                self.newly_computed.insert(id.to_string());
            }
            self.dirty.remove(id);
        }
        self.signatures.get(id)
    }

    /// Get a signature without computing (returns None if dirty or not computed).
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

    /// Get the number of computed signatures.
    pub fn len(&self) -> usize {
        self.signatures.len()
    }

    /// Check if the store is empty.
    pub fn is_empty(&self) -> bool {
        self.signatures.is_empty()
    }
}
