//! # Modular Route Engine
//!
//! This module provides a route engine with focused subcomponents for
//! better testability and maintainability.
//!
//! ## Architecture
//!
//! The engine is composed of focused modules:
//! - `ActivityStore` - Activity CRUD operations
//! - `SignatureStore` - Lazy signature computation with dirty tracking
//! - `SpatialIndex` - R-tree for viewport queries
//! - `RouteGrouper` - Union-Find clustering with incremental support

pub mod activity_store;
pub mod route_grouper;
pub mod signature_store;
pub mod spatial_index;

pub use activity_store::{ActivityData, ActivityStore};
pub use route_grouper::RouteGrouper;
pub use signature_store::SignatureStore;
pub use spatial_index::{ActivityBounds, SpatialIndex};

#[cfg(test)]
mod integration_tests {
    // Tests are in tests/engine/ directory
}

use std::collections::HashMap;

use log::warn;

use crate::{
    ActivityMetrics, Bounds, FrequentSection, GpsPoint, MatchConfig, RouteGroup, RoutePerformance,
    RoutePerformanceResult, RouteSignature, SectionConfig, SectionLap, SectionPerformanceRecord,
    SectionPerformanceResult,
};

/// Modular route engine using extracted components.
///
/// This engine provides clear separation of concerns with each component
/// handling a specific responsibility.
pub struct ModularRouteEngine {
    // Core components
    pub activities: ActivityStore,
    pub signatures: SignatureStore,
    pub grouper: RouteGrouper,
    pub spatial: SpatialIndex,

    // Section detection (not yet extracted)
    sections: Vec<FrequentSection>,
    sections_dirty: bool,

    // Consensus route cache
    consensus_cache: HashMap<String, Vec<GpsPoint>>,

    // Configuration
    match_config: MatchConfig,
    section_config: SectionConfig,

    // Performance data
    activity_metrics: HashMap<String, ActivityMetrics>,
    time_streams: HashMap<String, Vec<u32>>,
}

impl Default for ModularRouteEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl ModularRouteEngine {
    /// Create a new route engine with default configuration.
    pub fn new() -> Self {
        Self {
            activities: ActivityStore::new(),
            signatures: SignatureStore::new(),
            grouper: RouteGrouper::new(),
            spatial: SpatialIndex::new(),
            sections: Vec::new(),
            sections_dirty: false,
            consensus_cache: HashMap::new(),
            match_config: MatchConfig::default(),
            section_config: SectionConfig::default(),
            activity_metrics: HashMap::new(),
            time_streams: HashMap::new(),
        }
    }

    /// Create a new route engine with custom configuration.
    pub fn with_config(match_config: MatchConfig, section_config: SectionConfig) -> Self {
        Self {
            match_config,
            section_config,
            ..Self::new()
        }
    }

    // ========================================================================
    // Activity Management (delegates to ActivityStore)
    // ========================================================================

    /// Add an activity with its GPS coordinates.
    pub fn add_activity(&mut self, id: String, coords: Vec<GpsPoint>, sport_type: String) {
        self.activities.add(id.clone(), coords, sport_type);
        self.signatures.mark_dirty(&id);
        self.grouper.mark_dirty();
        self.sections_dirty = true;
        self.spatial.mark_dirty();
    }

    /// Add an activity from flat coordinate buffer.
    pub fn add_activity_flat(&mut self, id: String, flat_coords: &[f64], sport_type: String) {
        self.activities
            .add_flat(id.clone(), flat_coords, sport_type);
        self.signatures.mark_dirty(&id);
        self.grouper.mark_dirty();
        self.sections_dirty = true;
        self.spatial.mark_dirty();
    }

    /// Add multiple activities from flat coordinate buffers.
    pub fn add_activities_flat(
        &mut self,
        activity_ids: &[String],
        all_coords: &[f64],
        offsets: &[u32],
        sport_types: &[String],
    ) {
        let added = self
            .activities
            .add_many_flat(activity_ids, all_coords, offsets, sport_types);
        for id in added {
            self.signatures.mark_dirty(&id);
        }
        self.grouper.mark_dirty();
        self.sections_dirty = true;
        self.spatial.mark_dirty();
    }

    /// Remove an activity.
    pub fn remove_activity(&mut self, id: &str) {
        self.activities.remove(id);
        self.signatures.remove(id);
        self.grouper.invalidate(); // Removal requires full recomputation
        self.consensus_cache.clear();
        self.sections_dirty = true;
        self.spatial.mark_dirty();
    }

    /// Remove multiple activities.
    pub fn remove_activities(&mut self, ids: &[String]) {
        self.activities.remove_many(ids);
        self.signatures.remove_many(ids);
        if !ids.is_empty() {
            self.grouper.invalidate();
            self.consensus_cache.clear();
            self.sections_dirty = true;
            self.spatial.mark_dirty();
        }
    }

    /// Clear all activities and reset state.
    pub fn clear(&mut self) {
        self.activities.clear();
        self.signatures.clear();
        self.grouper.clear();
        self.spatial.clear();
        self.sections.clear();
        self.consensus_cache.clear();
        self.sections_dirty = false;
        self.activity_metrics.clear();
        self.time_streams.clear();
    }

    /// Get all activity IDs.
    pub fn get_activity_ids(&self) -> Vec<String> {
        self.activities.ids().cloned().collect()
    }

    /// Get the number of activities.
    pub fn activity_count(&self) -> usize {
        self.activities.len()
    }

    /// Check if an activity exists.
    pub fn has_activity(&self, id: &str) -> bool {
        self.activities.contains(id)
    }

    // ========================================================================
    // Signature Operations (delegates to SignatureStore)
    // ========================================================================

    /// Get a signature for an activity.
    pub fn get_signature(&mut self, id: &str) -> Option<&RouteSignature> {
        self.signatures
            .get(id, &self.activities, &self.match_config)
    }

    /// Get all signatures.
    pub fn get_all_signatures(&mut self) -> Vec<&RouteSignature> {
        self.signatures
            .ensure_computed(&self.activities, &self.match_config);
        self.signatures.all().collect()
    }

    /// Get signature points for an activity as JSON.
    pub fn get_signature_points_json(&mut self, id: &str) -> String {
        if let Some(sig) = self.get_signature(id) {
            serde_json::to_string(&sig.points).unwrap_or_else(|e| {
                warn!(
                    "Failed to serialize signature points for activity '{}': {}",
                    id, e
                );
                "[]".to_string()
            })
        } else {
            "[]".to_string()
        }
    }

    // ========================================================================
    // Grouping (delegates to RouteGrouper)
    // ========================================================================

    /// Get all route groups.
    pub fn get_groups(&mut self) -> &[RouteGroup] {
        self.grouper
            .ensure_computed(&mut self.signatures, &self.activities, &self.match_config);
        self.grouper.groups()
    }

    /// Get the group containing a specific activity.
    pub fn get_group_for_activity(&mut self, activity_id: &str) -> Option<&RouteGroup> {
        self.grouper
            .ensure_computed(&mut self.signatures, &self.activities, &self.match_config);
        self.grouper.get_group_for_activity(activity_id)
    }

    /// Set a custom name for a route.
    pub fn set_route_name(&mut self, route_id: &str, name: &str) {
        self.grouper.set_route_name(route_id, name);
    }

    /// Get the custom name for a route.
    pub fn get_route_name(&self, route_id: &str) -> Option<&String> {
        self.grouper.get_route_name(route_id)
    }

    /// Get groups as JSON string with performance stats populated.
    pub fn get_groups_json(&mut self) -> String {
        self.grouper
            .ensure_computed(&mut self.signatures, &self.activities, &self.match_config);

        // Clone groups and populate performance stats from activity_metrics
        let groups_with_stats: Vec<RouteGroup> = self
            .grouper
            .groups()
            .iter()
            .map(|group| {
                let mut g = group.clone();
                self.populate_group_performance_stats(&mut g);
                g
            })
            .collect();

        serde_json::to_string(&groups_with_stats).unwrap_or_else(|e| {
            warn!("Failed to serialize route groups: {}", e);
            "[]".to_string()
        })
    }

    /// Populate performance statistics for a route group from activity metrics.
    fn populate_group_performance_stats(&self, group: &mut RouteGroup) {
        if self.activity_metrics.is_empty() {
            return;
        }

        let mut times: Vec<f64> = Vec::new();
        let mut best_speed: f64 = 0.0;
        let mut best_time_val: f64 = f64::MAX;
        let mut best_id: Option<String> = None;

        for activity_id in &group.activity_ids {
            if let Some(metrics) = self.activity_metrics.get(activity_id) {
                let moving_time = metrics.moving_time as f64;
                if moving_time > 0.0 {
                    times.push(moving_time);

                    // Calculate speed (m/s)
                    let speed = metrics.distance / moving_time;

                    // Track best (fastest) by speed
                    if speed > best_speed {
                        best_speed = speed;
                        best_time_val = moving_time;
                        best_id = Some(activity_id.clone());
                    }
                }
            }
        }

        if !times.is_empty() {
            group.best_time = Some(best_time_val);
            group.avg_time = Some(times.iter().sum::<f64>() / times.len() as f64);
            group.best_pace = Some(best_speed);
            group.best_activity_id = best_id;
        }
    }

    // ========================================================================
    // Spatial Queries (delegates to SpatialIndex)
    // ========================================================================

    /// Query activities within a viewport.
    pub fn query_viewport(&mut self, bounds: &Bounds) -> Vec<String> {
        self.spatial.ensure_built(&self.activities);
        self.spatial.query_viewport(bounds)
    }

    /// Query activities within a viewport (raw coordinates).
    pub fn query_viewport_raw(
        &mut self,
        min_lat: f64,
        max_lat: f64,
        min_lng: f64,
        max_lng: f64,
    ) -> Vec<String> {
        self.spatial.ensure_built(&self.activities);
        self.spatial
            .query_viewport_raw(min_lat, max_lat, min_lng, max_lng)
    }

    /// Find activities near a point.
    pub fn find_nearby(&mut self, lat: f64, lng: f64, radius_degrees: f64) -> Vec<String> {
        self.spatial.ensure_built(&self.activities);
        self.spatial.find_nearby(lat, lng, radius_degrees)
    }

    // ========================================================================
    // Sections
    // ========================================================================

    /// Ensure sections are detected.
    fn ensure_sections(&mut self) {
        if !self.sections_dirty {
            return;
        }

        self.grouper
            .ensure_computed(&mut self.signatures, &self.activities, &self.match_config);

        let tracks = self.activities.as_tracks();
        let sport_map = self.activities.sport_type_map();

        self.sections = crate::sections::detect_sections_from_tracks(
            &tracks,
            &sport_map,
            self.grouper.groups(),
            &self.section_config,
        );

        self.sections_dirty = false;
    }

    /// Get all detected sections.
    pub fn get_sections(&mut self) -> &[FrequentSection] {
        self.ensure_sections();
        &self.sections
    }

    /// Get sections filtered by sport type.
    pub fn get_sections_for_sport(&mut self, sport_type: &str) -> Vec<&FrequentSection> {
        self.ensure_sections();
        self.sections
            .iter()
            .filter(|s| s.sport_type == sport_type)
            .collect()
    }

    /// Get sections as JSON string.
    pub fn get_sections_json(&mut self) -> String {
        self.ensure_sections();
        serde_json::to_string(&self.sections).unwrap_or_else(|e| {
            warn!("Failed to serialize sections: {}", e);
            "[]".to_string()
        })
    }

    // ========================================================================
    // Consensus Route
    // ========================================================================

    /// Get or compute the consensus route for a group.
    pub fn get_consensus_route(&mut self, group_id: &str) -> Option<Vec<GpsPoint>> {
        // Check cache first
        if let Some(cached) = self.consensus_cache.get(group_id) {
            return Some(cached.clone());
        }

        // Find the group
        self.grouper
            .ensure_computed(&mut self.signatures, &self.activities, &self.match_config);
        let group = self.grouper.get_group(group_id)?;

        if group.activity_ids.is_empty() {
            return None;
        }

        // Get all tracks for this group
        let tracks: Vec<&Vec<GpsPoint>> = group
            .activity_ids
            .iter()
            .filter_map(|id| self.activities.get(id).map(|a| &a.coords))
            .collect();

        if tracks.is_empty() {
            return None;
        }

        // Simple consensus: use the medoid
        let consensus = compute_medoid_track(&tracks);

        // Cache the result
        self.consensus_cache
            .insert(group_id.to_string(), consensus.clone());

        Some(consensus)
    }

    // ========================================================================
    // Configuration
    // ========================================================================

    /// Update match configuration.
    pub fn set_match_config(&mut self, config: MatchConfig) {
        self.match_config = config;
        self.signatures
            .mark_all_dirty(self.activities.ids().cloned());
        self.grouper.invalidate();
        self.sections_dirty = true;
    }

    /// Update section configuration.
    pub fn set_section_config(&mut self, config: SectionConfig) {
        self.section_config = config;
        self.sections_dirty = true;
    }

    /// Get current match configuration.
    pub fn get_match_config(&self) -> &MatchConfig {
        &self.match_config
    }

    /// Get current section configuration.
    pub fn get_section_config(&self) -> &SectionConfig {
        &self.section_config
    }

    // ========================================================================
    // Performance Calculations
    // ========================================================================

    /// Set activity metrics for performance calculations.
    pub fn set_activity_metrics(&mut self, metrics: Vec<ActivityMetrics>) {
        for m in metrics {
            self.activity_metrics.insert(m.activity_id.clone(), m);
        }
    }

    /// Set a single activity's metrics.
    pub fn set_activity_metric(&mut self, metric: ActivityMetrics) {
        self.activity_metrics
            .insert(metric.activity_id.clone(), metric);
    }

    /// Get activity metrics by ID.
    pub fn get_activity_metrics(&self, activity_id: &str) -> Option<&ActivityMetrics> {
        self.activity_metrics.get(activity_id)
    }

    /// Calculate route performances for all activities in a group.
    pub fn get_route_performances(
        &mut self,
        route_group_id: &str,
        current_activity_id: Option<&str>,
    ) -> RoutePerformanceResult {
        self.grouper
            .ensure_computed(&mut self.signatures, &self.activities, &self.match_config);

        let group = match self.grouper.get_group(route_group_id) {
            Some(g) => g,
            None => {
                return RoutePerformanceResult {
                    performances: vec![],
                    best: None,
                    current_rank: None,
                }
            }
        };

        let mut performances: Vec<RoutePerformance> = group
            .activity_ids
            .iter()
            .filter_map(|id| {
                let metrics = self.activity_metrics.get(id)?;
                let speed = if metrics.moving_time > 0 {
                    metrics.distance / metrics.moving_time as f64
                } else {
                    0.0
                };

                Some(RoutePerformance {
                    activity_id: id.clone(),
                    name: metrics.name.clone(),
                    date: metrics.date,
                    speed,
                    duration: metrics.elapsed_time,
                    moving_time: metrics.moving_time,
                    distance: metrics.distance,
                    elevation_gain: metrics.elevation_gain,
                    avg_hr: metrics.avg_hr,
                    avg_power: metrics.avg_power,
                    is_current: current_activity_id == Some(id.as_str()),
                    direction: "same".to_string(),
                    match_percentage: 100.0,
                })
            })
            .collect();

        performances.sort_by_key(|p| p.date);

        let best = performances
            .iter()
            .max_by(|a, b| {
                a.speed
                    .partial_cmp(&b.speed)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .cloned();

        let current_rank = current_activity_id.and_then(|current_id| {
            let mut by_speed = performances.clone();
            by_speed.sort_by(|a, b| {
                b.speed
                    .partial_cmp(&a.speed)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            by_speed
                .iter()
                .position(|p| p.activity_id == current_id)
                .map(|idx| (idx + 1) as u32)
        });

        RoutePerformanceResult {
            performances,
            best,
            current_rank,
        }
    }

    /// Set time stream for an activity.
    pub fn set_time_stream(&mut self, activity_id: String, times: Vec<u32>) {
        self.time_streams.insert(activity_id, times);
    }

    /// Set multiple time streams from flat buffer.
    pub fn set_time_streams_flat(
        &mut self,
        activity_ids: &[String],
        all_times: &[u32],
        offsets: &[u32],
    ) {
        for (i, activity_id) in activity_ids.iter().enumerate() {
            let start = offsets[i] as usize;
            let end = offsets
                .get(i + 1)
                .map(|&o| o as usize)
                .unwrap_or(all_times.len());
            let times = all_times[start..end].to_vec();
            self.time_streams.insert(activity_id.clone(), times);
        }
    }

    /// Calculate section performances.
    pub fn get_section_performances(&mut self, section_id: &str) -> SectionPerformanceResult {
        self.ensure_sections();

        let section = match self.sections.iter().find(|s| s.id == section_id) {
            Some(s) => s,
            None => {
                return SectionPerformanceResult {
                    records: vec![],
                    best_record: None,
                }
            }
        };

        let mut portions_by_activity: HashMap<&str, Vec<&crate::SectionPortion>> = HashMap::new();
        for portion in &section.activity_portions {
            portions_by_activity
                .entry(&portion.activity_id)
                .or_default()
                .push(portion);
        }

        let mut records: Vec<SectionPerformanceRecord> = portions_by_activity
            .iter()
            .filter_map(|(activity_id, portions)| {
                let metrics = self.activity_metrics.get(*activity_id)?;
                let times = self.time_streams.get(*activity_id)?;

                let laps: Vec<SectionLap> = portions
                    .iter()
                    .enumerate()
                    .filter_map(|(i, portion)| {
                        let start_idx = portion.start_index as usize;
                        let end_idx = portion.end_index as usize;

                        if start_idx >= times.len() || end_idx >= times.len() {
                            return None;
                        }

                        let lap_time = (times[end_idx] as f64 - times[start_idx] as f64).abs();
                        if lap_time <= 0.0 {
                            return None;
                        }

                        let pace = portion.distance_meters / lap_time;

                        Some(SectionLap {
                            id: format!("{}_lap{}", activity_id, i),
                            activity_id: activity_id.to_string(),
                            time: lap_time,
                            pace,
                            distance: portion.distance_meters,
                            direction: portion.direction.clone(),
                            start_index: portion.start_index,
                            end_index: portion.end_index,
                        })
                    })
                    .collect();

                if laps.is_empty() {
                    return None;
                }

                let best_time = laps.iter().map(|l| l.time).fold(f64::MAX, f64::min);
                let best_pace = laps.iter().map(|l| l.pace).fold(0.0f64, f64::max);
                let avg_time = laps.iter().map(|l| l.time).sum::<f64>() / laps.len() as f64;
                let avg_pace = laps.iter().map(|l| l.pace).sum::<f64>() / laps.len() as f64;

                Some(SectionPerformanceRecord {
                    activity_id: activity_id.to_string(),
                    activity_name: metrics.name.clone(),
                    activity_date: metrics.date,
                    lap_count: laps.len() as u32,
                    best_time,
                    best_pace,
                    avg_time,
                    avg_pace,
                    direction: laps[0].direction.clone(),
                    section_distance: laps[0].distance,
                    laps,
                })
            })
            .collect();

        records.sort_by_key(|r| r.activity_date);

        let best_record = records
            .iter()
            .min_by(|a, b| {
                a.best_time
                    .partial_cmp(&b.best_time)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .cloned();

        SectionPerformanceResult {
            records,
            best_record,
        }
    }

    // ========================================================================
    // Statistics
    // ========================================================================

    /// Get engine statistics.
    pub fn stats(&mut self) -> ModularEngineStats {
        self.grouper
            .ensure_computed(&mut self.signatures, &self.activities, &self.match_config);
        self.ensure_sections();

        ModularEngineStats {
            activity_count: self.activities.len() as u32,
            signature_count: self.signatures.len() as u32,
            group_count: self.grouper.len() as u32,
            section_count: self.sections.len() as u32,
            cached_consensus_count: self.consensus_cache.len() as u32,
        }
    }
}

/// Engine statistics for monitoring (modular engine version).
#[derive(Debug, Clone)]
pub struct ModularEngineStats {
    pub activity_count: u32,
    pub signature_count: u32,
    pub group_count: u32,
    pub section_count: u32,
    pub cached_consensus_count: u32,
}

/// Compute the medoid track (the track most representative of the group).
fn compute_medoid_track(tracks: &[&Vec<GpsPoint>]) -> Vec<GpsPoint> {
    if tracks.is_empty() {
        return vec![];
    }
    if tracks.len() == 1 {
        return tracks[0].clone();
    }

    let mut best_idx = 0;
    let mut best_total_dist = f64::MAX;

    for (i, track_i) in tracks.iter().enumerate() {
        let total_dist: f64 = tracks
            .iter()
            .enumerate()
            .filter(|(j, _)| *j != i)
            .map(|(_, track_j)| track_distance(track_i, track_j))
            .sum();

        if total_dist < best_total_dist {
            best_total_dist = total_dist;
            best_idx = i;
        }
    }

    tracks[best_idx].clone()
}

/// Compute distance between two tracks using sampled AMD.
fn track_distance(track1: &[GpsPoint], track2: &[GpsPoint]) -> f64 {
    if track1.is_empty() || track2.is_empty() {
        return f64::MAX;
    }

    let sample_size = 20.min(track1.len().min(track2.len()));
    let step1 = track1.len() / sample_size;
    let step2 = track2.len() / sample_size;

    let sampled1: Vec<&GpsPoint> = (0..sample_size).map(|i| &track1[i * step1]).collect();
    let sampled2: Vec<&GpsPoint> = (0..sample_size).map(|i| &track2[i * step2]).collect();

    sampled1
        .iter()
        .map(|p1| {
            sampled2
                .iter()
                .map(|p2| crate::geo_utils::haversine_distance(p1, p2))
                .fold(f64::MAX, f64::min)
        })
        .sum::<f64>()
        / sample_size as f64
}
