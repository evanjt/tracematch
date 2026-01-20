//! Spatial indexing for viewport queries.
//!
//! Uses an R-tree to efficiently query activities by geographic bounds.

use rstar::{AABB, RTree, RTreeObject};

use crate::Bounds;

use super::activity_store::ActivityStore;

/// Activity bounds wrapper for R-tree spatial indexing.
#[derive(Debug, Clone)]
pub struct ActivityBounds {
    pub activity_id: String,
    pub min_lat: f64,
    pub max_lat: f64,
    pub min_lng: f64,
    pub max_lng: f64,
}

impl RTreeObject for ActivityBounds {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners([self.min_lng, self.min_lat], [self.max_lng, self.max_lat])
    }
}

/// Spatial index for efficient viewport queries.
///
/// Maintains an R-tree of activity bounds with dirty tracking
/// for incremental updates.
#[derive(Debug)]
pub struct SpatialIndex {
    tree: RTree<ActivityBounds>,
    dirty: bool,
}

impl Default for SpatialIndex {
    fn default() -> Self {
        Self::new()
    }
}

impl SpatialIndex {
    /// Create a new empty spatial index.
    pub fn new() -> Self {
        Self {
            tree: RTree::new(),
            dirty: false,
        }
    }

    /// Mark the index as needing rebuild.
    pub fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    /// Check if the index needs rebuild.
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// Rebuild the index from the activity store.
    pub fn rebuild(&mut self, store: &ActivityStore) {
        let bounds: Vec<ActivityBounds> = store
            .values()
            .filter_map(|activity| {
                activity.bounds.map(|b| ActivityBounds {
                    activity_id: activity.id.clone(),
                    min_lat: b.min_lat,
                    max_lat: b.max_lat,
                    min_lng: b.min_lng,
                    max_lng: b.max_lng,
                })
            })
            .collect();

        self.tree = RTree::bulk_load(bounds);
        self.dirty = false;
    }

    /// Ensure the index is up to date.
    pub fn ensure_built(&mut self, store: &ActivityStore) {
        if self.dirty {
            self.rebuild(store);
        }
    }

    /// Clear the index.
    pub fn clear(&mut self) {
        self.tree = RTree::new();
        self.dirty = false;
    }

    /// Query activities within a viewport.
    pub fn query_viewport(&self, bounds: &Bounds) -> Vec<String> {
        let search_bounds = AABB::from_corners(
            [bounds.min_lng, bounds.min_lat],
            [bounds.max_lng, bounds.max_lat],
        );

        self.tree
            .locate_in_envelope_intersecting(&search_bounds)
            .map(|b| b.activity_id.clone())
            .collect()
    }

    /// Query activities within raw coordinate bounds.
    pub fn query_viewport_raw(
        &self,
        min_lat: f64,
        max_lat: f64,
        min_lng: f64,
        max_lng: f64,
    ) -> Vec<String> {
        self.query_viewport(&Bounds {
            min_lat,
            max_lat,
            min_lng,
            max_lng,
        })
    }

    /// Find activities near a point.
    pub fn find_nearby(&self, lat: f64, lng: f64, radius_degrees: f64) -> Vec<String> {
        self.query_viewport_raw(
            lat - radius_degrees,
            lat + radius_degrees,
            lng - radius_degrees,
            lng + radius_degrees,
        )
    }

    /// Get the number of indexed activities.
    pub fn len(&self) -> usize {
        self.tree.size()
    }

    /// Check if the index is empty.
    pub fn is_empty(&self) -> bool {
        self.tree.size() == 0
    }
}
