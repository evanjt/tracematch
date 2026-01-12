//! Activity storage with efficient bulk operations.
//!
//! Manages the raw GPS data for activities, including:
//! - CRUD operations for activities
//! - Flat buffer handling for efficient FFI
//! - Bounds computation for spatial indexing

use std::collections::HashMap;

use crate::{Bounds, GpsPoint};

/// Activity data stored in the engine
#[derive(Debug, Clone)]
pub struct ActivityData {
    pub id: String,
    pub coords: Vec<GpsPoint>,
    pub sport_type: String,
    pub bounds: Option<Bounds>,
}

/// Storage for activity GPS data.
///
/// Provides efficient storage and retrieval of activity coordinates
/// with support for bulk operations via flat buffers.
#[derive(Debug, Default)]
pub struct ActivityStore {
    activities: HashMap<String, ActivityData>,
}

impl ActivityStore {
    /// Create a new empty activity store.
    pub fn new() -> Self {
        Self {
            activities: HashMap::new(),
        }
    }

    /// Add an activity with its GPS coordinates.
    ///
    /// Returns the computed bounds for the activity.
    pub fn add(&mut self, id: String, coords: Vec<GpsPoint>, sport_type: String) -> Option<Bounds> {
        let bounds = Bounds::from_points(&coords);

        let activity = ActivityData {
            id: id.clone(),
            coords,
            sport_type,
            bounds,
        };

        self.activities.insert(id, activity);
        bounds
    }

    /// Add an activity from flat coordinate buffer.
    ///
    /// Coordinates are [lat1, lng1, lat2, lng2, ...].
    pub fn add_flat(
        &mut self,
        id: String,
        flat_coords: &[f64],
        sport_type: String,
    ) -> Option<Bounds> {
        let coords: Vec<GpsPoint> = flat_coords
            .chunks_exact(2)
            .map(|chunk| GpsPoint::new(chunk[0], chunk[1]))
            .collect();
        self.add(id, coords, sport_type)
    }

    /// Add multiple activities from flat coordinate buffers.
    ///
    /// This is the most efficient way to bulk-add activities from FFI.
    pub fn add_many_flat(
        &mut self,
        activity_ids: &[String],
        all_coords: &[f64],
        offsets: &[u32],
        sport_types: &[String],
    ) -> Vec<String> {
        let mut added_ids = Vec::with_capacity(activity_ids.len());

        for (i, id) in activity_ids.iter().enumerate() {
            let start = offsets[i] as usize;
            let end = offsets
                .get(i + 1)
                .map(|&o| o as usize)
                .unwrap_or(all_coords.len() / 2);

            let coords: Vec<GpsPoint> = (start..end)
                .filter_map(|j| {
                    let idx = j * 2;
                    if idx + 1 < all_coords.len() {
                        Some(GpsPoint::new(all_coords[idx], all_coords[idx + 1]))
                    } else {
                        None
                    }
                })
                .collect();

            let sport = sport_types.get(i).cloned().unwrap_or_default();
            self.add(id.clone(), coords, sport);
            added_ids.push(id.clone());
        }

        added_ids
    }

    /// Remove an activity by ID.
    ///
    /// Returns the removed activity data if it existed.
    pub fn remove(&mut self, id: &str) -> Option<ActivityData> {
        self.activities.remove(id)
    }

    /// Remove multiple activities.
    ///
    /// Returns the IDs of activities that were actually removed.
    pub fn remove_many(&mut self, ids: &[String]) -> Vec<String> {
        let mut removed = Vec::new();
        for id in ids {
            if self.activities.remove(id).is_some() {
                removed.push(id.clone());
            }
        }
        removed
    }

    /// Clear all activities.
    pub fn clear(&mut self) {
        self.activities.clear();
    }

    /// Get an activity by ID.
    pub fn get(&self, id: &str) -> Option<&ActivityData> {
        self.activities.get(id)
    }

    /// Get the coordinates for an activity.
    pub fn get_coords(&self, id: &str) -> Option<&[GpsPoint]> {
        self.activities.get(id).map(|a| a.coords.as_slice())
    }

    /// Get the sport type for an activity.
    pub fn get_sport_type(&self, id: &str) -> Option<&str> {
        self.activities.get(id).map(|a| a.sport_type.as_str())
    }

    /// Check if an activity exists.
    pub fn contains(&self, id: &str) -> bool {
        self.activities.contains_key(id)
    }

    /// Get all activity IDs.
    pub fn ids(&self) -> impl Iterator<Item = &String> {
        self.activities.keys()
    }

    /// Get all activities.
    pub fn iter(&self) -> impl Iterator<Item = (&String, &ActivityData)> {
        self.activities.iter()
    }

    /// Get all activities as values.
    pub fn values(&self) -> impl Iterator<Item = &ActivityData> {
        self.activities.values()
    }

    /// Get the number of activities.
    pub fn len(&self) -> usize {
        self.activities.len()
    }

    /// Check if the store is empty.
    pub fn is_empty(&self) -> bool {
        self.activities.is_empty()
    }

    /// Build a map of activity ID to sport type.
    pub fn sport_type_map(&self) -> HashMap<String, String> {
        self.activities
            .iter()
            .map(|(id, a)| (id.clone(), a.sport_type.clone()))
            .collect()
    }

    /// Build a vector of (id, coords) pairs for all activities.
    pub fn as_tracks(&self) -> Vec<(String, Vec<GpsPoint>)> {
        self.activities
            .values()
            .map(|a| (a.id.clone(), a.coords.clone()))
            .collect()
    }

    /// Compute the total distance of a GPS track in meters.
    pub fn compute_track_distance(coords: &[GpsPoint]) -> f64 {
        if coords.len() < 2 {
            return 0.0;
        }
        coords
            .windows(2)
            .map(|pair| crate::geo_utils::haversine_distance(&pair[0], &pair[1]))
            .sum()
    }
}
