//! # Adaptive Consensus Section Detection
//!
//! Detects frequently-traveled road sections using FULL GPS tracks.
//! Produces smooth, natural polylines that evolve and refine over time
//! as more tracks are observed.
//!
//! ## Algorithm
//! 1. Load full GPS tracks (1000s of points per activity)
//! 2. Find overlapping portions using R-tree spatial indexing
//! 3. Cluster overlaps that represent the same physical section
//! 4. Select initial medoid as the starting reference
//! 5. Compute consensus polyline via weighted averaging of all tracks
//! 6. Track per-point confidence based on observation density
//! 7. Adapt section boundaries based on where tracks consistently overlap
//!
//! ## Consensus Algorithm
//! - Normalize all tracks to common parameterization (by distance)
//! - At each position, collect nearby points from all tracks
//! - Compute weighted average: weight = 1 / (distance_to_reference + epsilon)
//! - Higher observation density → higher confidence → tighter future matching
//!
//! ## Adaptive Boundaries
//! - Track where each activity's overlap starts/ends relative to section
//! - Section can grow if tracks consistently extend beyond current bounds
//! - Section contracts if tracks consistently end before current bounds

mod consensus;
pub mod incremental;
mod medoid;
pub mod optimized;
mod overlap;
mod portions;
mod postprocess;
pub mod progress;
mod rtree;
mod traces;

use crate::geo_utils::{bounds_overlap, compute_bounds};
use crate::matching::calculate_route_distance;
use crate::{Bounds, GpsPoint, RouteGroup};
use log::info;
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
#[cfg(feature = "parallel")]
use std::sync::atomic::{AtomicUsize, Ordering};

pub use progress::{
    AtomicProgressTracker, DetectionPhase, DetectionProgressCallback, NoopProgress,
};

// Re-export internal utilities for use across submodules
pub(crate) use consensus::compute_consensus_polyline;
pub(crate) use medoid::{compute_stability, select_medoid};
pub(crate) use overlap::{
    FullTrackOverlap, OverlapCluster, cluster_overlaps, cluster_overlaps_with_map,
    find_full_track_overlap,
};
pub(crate) use portions::compute_activity_portions;
pub(crate) use postprocess::{
    consolidate_fragments, filter_low_quality_sections, make_sections_exclusive,
    merge_nearby_sections, remove_overlapping_sections, split_at_gradient_changes,
    split_at_heading_changes, split_folding_sections, split_high_variance_sections,
};
pub(crate) use rtree::IndexedPoint;
pub use rtree::build_rtree;
pub use traces::{extract_activity_trace, extract_all_activity_traces};

// Re-export optimized detection functions
pub use optimized::{
    SectionMatch, SplitResult, detect_sections_optimized, find_all_section_spans_in_route,
    find_sections_in_route, recalculate_section_polyline, split_section_at_index,
    split_section_at_point,
};

/// Detection mode for section detection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DetectionMode {
    /// Lower thresholds, more sections detected
    Discovery,
    /// Higher thresholds, fewer but more confident sections
    Conservative,
    /// Single-scale backward-compatible mode
    Legacy,
}

impl DetectionMode {
    pub fn as_str(&self) -> &'static str {
        match self {
            DetectionMode::Discovery => "discovery",
            DetectionMode::Conservative => "conservative",
            DetectionMode::Legacy => "legacy",
        }
    }
}

impl std::fmt::Display for DetectionMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

impl std::str::FromStr for DetectionMode {
    type Err = ();
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "discovery" => Ok(DetectionMode::Discovery),
            "conservative" => Ok(DetectionMode::Conservative),
            "legacy" => Ok(DetectionMode::Legacy),
            _ => Ok(DetectionMode::Discovery),
        }
    }
}

impl Default for DetectionMode {
    fn default() -> Self {
        DetectionMode::Discovery
    }
}

/// Scale name for multi-scale section detection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScaleName {
    Short,
    Medium,
    Long,
    ExtraLong,
    UltraLong,
    Optimized,
}

impl ScaleName {
    pub fn as_str(&self) -> &'static str {
        match self {
            ScaleName::Short => "short",
            ScaleName::Medium => "medium",
            ScaleName::Long => "long",
            ScaleName::ExtraLong => "extra_long",
            ScaleName::UltraLong => "ultra_long",
            ScaleName::Optimized => "optimized",
        }
    }
}

impl std::fmt::Display for ScaleName {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

impl std::str::FromStr for ScaleName {
    type Err = ();
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "short" => Ok(ScaleName::Short),
            "medium" => Ok(ScaleName::Medium),
            "long" => Ok(ScaleName::Long),
            "extra_long" | "extralong" => Ok(ScaleName::ExtraLong),
            "ultra_long" | "ultralong" => Ok(ScaleName::UltraLong),
            "optimized" => Ok(ScaleName::Optimized),
            _ => Ok(ScaleName::Medium),
        }
    }
}

impl Default for ScaleName {
    fn default() -> Self {
        ScaleName::Medium
    }
}

/// Scale preset for multi-scale section detection
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ScalePreset {
    /// Scale name
    pub name: ScaleName,
    /// Minimum section length for this scale (meters)
    pub min_length: f64,
    /// Maximum section length for this scale (meters)
    pub max_length: f64,
    /// Minimum activities required at this scale (can be lower for short sections)
    pub min_activities: u32,
}

impl ScalePreset {
    pub fn short() -> Self {
        Self {
            name: ScaleName::Short,
            min_length: 100.0,
            max_length: 500.0,
            min_activities: 2,
        }
    }

    pub fn medium() -> Self {
        Self {
            name: ScaleName::Medium,
            min_length: 500.0,
            max_length: 2000.0,
            min_activities: 2,
        }
    }

    pub fn long() -> Self {
        Self {
            name: ScaleName::Long,
            min_length: 2000.0,
            max_length: 5000.0,
            min_activities: 3,
        }
    }

    /// Extra long sections: 5km-50km (long cycling climbs, rail trails).
    pub fn extra_long() -> Self {
        Self {
            name: ScaleName::ExtraLong,
            min_length: 5_000.0,
            max_length: 50_000.0,
            min_activities: 3,
        }
    }

    /// Ultra long sections: 50km-200km (century routes, multi-day corridors).
    pub fn ultra_long() -> Self {
        Self {
            name: ScaleName::UltraLong,
            min_length: 50_000.0,
            max_length: 200_000.0,
            min_activities: 3,
        }
    }

    pub fn default_presets() -> Vec<Self> {
        vec![
            Self::short(),
            Self::medium(),
            Self::long(),
            Self::extra_long(),
            Self::ultra_long(),
        ]
    }
}

/// Configuration for section detection
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SectionConfig {
    /// Maximum distance between tracks to consider overlapping (meters)
    pub proximity_threshold: f64,
    /// Minimum overlap length to consider a section (meters)
    pub min_section_length: f64,
    /// Maximum section length (meters) - prevents sections from becoming full routes
    pub max_section_length: f64,
    /// Minimum number of activities that must share an overlap
    pub min_activities: u32,
    /// Tolerance for clustering similar overlaps (meters)
    pub cluster_tolerance: f64,
    /// Number of sample points for AMD comparison (not for output!)
    pub sample_points: u32,
    /// Detection mode
    pub detection_mode: DetectionMode,
    /// Include potential sections with only 1-2 activities as suggestions
    pub include_potentials: bool,
    /// Scale presets for multi-scale detection (empty = single-scale with min/max_section_length)
    pub scale_presets: Vec<ScalePreset>,
    /// Preserve hierarchical sections (don't deduplicate short sections inside longer ones)
    pub preserve_hierarchy: bool,
}

impl Default for SectionConfig {
    fn default() -> Self {
        Self {
            proximity_threshold: 50.0, // 50m - handles GPS error + wide roads + opposite sides
            min_section_length: 200.0, // 200m minimum section (used when scale_presets is empty)
            max_section_length: 200_000.0, // 200km max (used when scale_presets is empty)
            min_activities: 3,         // Need 3+ activities (used when scale_presets is empty)
            cluster_tolerance: 80.0,   // 80m for clustering similar overlaps
            sample_points: 50,         // For AMD comparison only
            detection_mode: DetectionMode::Discovery,
            include_potentials: true,
            scale_presets: ScalePreset::default_presets(),
            preserve_hierarchy: true,
        }
    }
}

impl SectionConfig {
    /// Create a discovery-mode config (lower thresholds, more sections)
    pub fn discovery() -> Self {
        Self {
            detection_mode: DetectionMode::Discovery,
            include_potentials: true,
            scale_presets: ScalePreset::default_presets(),
            preserve_hierarchy: true,
            ..Default::default()
        }
    }

    /// Create a conservative config (higher thresholds, fewer sections)
    pub fn conservative() -> Self {
        Self {
            detection_mode: DetectionMode::Conservative,
            include_potentials: false,
            min_activities: 4,
            scale_presets: vec![ScalePreset::medium(), ScalePreset::long()],
            preserve_hierarchy: false,
            ..Default::default()
        }
    }

    /// Create a legacy single-scale config (for backward compatibility)
    pub fn legacy() -> Self {
        Self {
            detection_mode: DetectionMode::Legacy,
            include_potentials: false,
            scale_presets: vec![], // Empty = use min/max_section_length directly
            preserve_hierarchy: false,
            min_activities: 3,
            ..Default::default()
        }
    }
}

/// Each activity's portion of a section (for pace comparison)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SectionPortion {
    /// Activity ID
    #[serde(alias = "activity_id")]
    pub activity_id: String,
    /// Start index into the activity's FULL GPS track
    #[serde(alias = "start_index")]
    pub start_index: u32,
    /// End index into the activity's FULL GPS track
    #[serde(alias = "end_index")]
    pub end_index: u32,
    /// Distance of this portion in meters
    #[serde(alias = "distance_meters")]
    pub distance_meters: f64,
    /// Direction relative to representative
    pub direction: crate::Direction,
}

/// A frequently-traveled section with adaptive consensus representation
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FrequentSection {
    /// Unique section ID
    pub id: String,
    /// Custom name (user-defined, None if not set)
    pub name: Option<String>,
    /// Sport type ("Run", "Ride", etc.)
    #[serde(alias = "sport_type")]
    pub sport_type: String,
    /// The consensus polyline - refined from all overlapping tracks
    /// Initially the medoid, evolves via weighted averaging as more tracks are added
    pub polyline: Vec<GpsPoint>,
    /// Which activity provided the initial representative polyline (medoid)
    #[serde(alias = "representative_activity_id")]
    pub representative_activity_id: String,
    /// All activity IDs that traverse this section
    #[serde(alias = "activity_ids")]
    pub activity_ids: Vec<String>,
    /// Each activity's portion (start/end indices, distance, direction)
    #[serde(alias = "activity_portions")]
    pub activity_portions: Vec<SectionPortion>,
    /// Route group IDs that include this section
    #[serde(alias = "route_ids")]
    pub route_ids: Vec<String>,
    /// Number of times traversed
    #[serde(alias = "visit_count")]
    pub visit_count: u32,
    /// Section length in meters
    #[serde(alias = "distance_meters")]
    pub distance_meters: f64,
    /// Pre-computed GPS traces for each activity's overlapping portion
    /// Key is activity ID, value is the GPS points within proximity of section
    #[serde(alias = "activity_traces")]
    pub activity_traces: HashMap<String, Vec<GpsPoint>>,
    /// Confidence score (0.0-1.0) based on observation density
    /// Higher confidence = more tracks observed, tighter consensus
    pub confidence: f64,
    /// Number of observations (tracks) used to compute consensus
    #[serde(alias = "observation_count")]
    pub observation_count: u32,
    /// Average spread (meters) of track observations from consensus line
    /// Lower spread = more consistent track alignment
    #[serde(alias = "average_spread")]
    pub average_spread: f64,
    /// Per-point observation density (how many activities pass through each point)
    /// Used for detecting high-traffic portions that should become separate sections
    #[serde(alias = "point_density")]
    pub point_density: Vec<u32>,
    /// Scale at which this section was detected
    pub scale: Option<ScaleName>,

    /// Whether this section was user-defined (prevents automatic updates)
    #[serde(alias = "is_user_defined")]
    pub is_user_defined: bool,

    /// How well the reference trace aligns with the consensus polyline (0.0-1.0).
    /// 1.0 = perfect alignment, 0.0 = maximum deviation.
    /// Computed as 1.0 - (amd_to_consensus / proximity_threshold).clamp(0.0, 1.0)
    #[serde(default)]
    pub stability: f64,

    /// Number of times this section has been recalibrated
    #[serde(default = "default_version")]
    pub version: u32,

    /// ISO timestamp of last recalibration (reference change or consensus update)
    #[serde(alias = "updated_at")]
    pub updated_at: Option<String>,

    /// ISO timestamp when section was created
    #[serde(alias = "created_at")]
    pub created_at: Option<String>,
}

fn default_version() -> u32 {
    1
}

impl FrequentSection {
    /// Split this section at a polyline index.
    ///
    /// Convenience wrapper around [`split_section_at_index`].
    pub fn split_at_index(&self, split_index: usize) -> Option<SplitResult> {
        optimized::split_section_at_index(self, split_index)
    }

    /// Split this section at a geographic point (finds nearest polyline index).
    ///
    /// Convenience wrapper around [`split_section_at_point`].
    pub fn split_at_point(&self, point: &GpsPoint, max_distance: f64) -> Option<SplitResult> {
        optimized::split_section_at_point(self, point, max_distance)
    }

    /// Recalculate this section's polyline from stored activity traces.
    ///
    /// Convenience wrapper around [`recalculate_section_polyline`].
    pub fn recalculate_polyline(&self, config: &SectionConfig) -> FrequentSection {
        optimized::recalculate_section_polyline(self, config)
    }
}

/// A potential section detected from 1-2 activities.
/// These are suggestions that users can promote to full sections.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PotentialSection {
    /// Unique section ID
    pub id: String,
    /// Sport type ("Run", "Ride", etc.)
    #[serde(alias = "sport_type")]
    pub sport_type: String,
    /// The polyline from the representative activity
    pub polyline: Vec<GpsPoint>,
    /// Activity IDs that traverse this potential section (1-2)
    #[serde(alias = "activity_ids")]
    pub activity_ids: Vec<String>,
    /// Number of times traversed (1-2)
    #[serde(alias = "visit_count")]
    pub visit_count: u32,
    /// Section length in meters
    #[serde(alias = "distance_meters")]
    pub distance_meters: f64,
    /// Confidence score (0.0-1.0), lower than FrequentSection
    pub confidence: f64,
    /// Scale at which this was detected
    pub scale: ScaleName,
}

/// Result of multi-scale section detection
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MultiScaleSectionResult {
    /// Confirmed sections (min_activities met)
    pub sections: Vec<FrequentSection>,
    /// Potential sections (1-2 activities, suggestions for user)
    pub potentials: Vec<PotentialSection>,
    /// Statistics about detection
    pub stats: DetectionStats,
}

/// Statistics from section detection
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct DetectionStats {
    /// Total activities processed
    pub activities_processed: u32,
    /// Total overlaps found across all scales
    pub overlaps_found: u32,
    /// Sections per scale
    pub sections_by_scale: HashMap<String, u32>,
    /// Potentials per scale
    pub potentials_by_scale: HashMap<String, u32>,
}

/// Process a single cluster into a FrequentSection.
fn process_cluster(
    idx: usize,
    cluster: OverlapCluster,
    sport_type: &str,
    track_map: &HashMap<&str, &[GpsPoint]>,
    activity_to_route: &HashMap<&str, &str>,
    config: &SectionConfig,
    scale_name: Option<ScaleName>,
) -> Option<FrequentSection> {
    // Select medoid - an ACTUAL GPS trace (resolved from track_map via index ranges)
    let (representative_id, representative_polyline) = select_medoid(&cluster, track_map);

    if representative_polyline.is_empty() {
        return None;
    }

    let distance_meters = calculate_route_distance(&representative_polyline);

    // Filter by max length - sections shouldn't be whole routes
    if distance_meters > config.max_section_length {
        return None;
    }

    // Compute activity portions for pace comparison
    let activity_portions =
        compute_activity_portions(&cluster, &representative_polyline, track_map, config);

    // Collect route IDs
    let route_ids: Vec<String> = cluster
        .activity_ids
        .iter()
        .filter_map(|aid| activity_to_route.get(aid.as_str()).map(|s| s.to_string()))
        .collect::<HashSet<_>>()
        .into_iter()
        .collect();

    // Extract traces for consensus computation only (not stored in section)
    let activity_id_vec: Vec<String> = cluster.activity_ids.iter().cloned().collect();
    let activity_traces_for_consensus =
        extract_all_activity_traces(&activity_id_vec, &representative_polyline, track_map);

    // Collect all traces for consensus computation
    let all_traces: Vec<Vec<GpsPoint>> = activity_traces_for_consensus.values().cloned().collect();

    // Compute consensus polyline from all overlapping tracks
    let consensus = compute_consensus_polyline(
        &representative_polyline,
        &all_traces,
        config.proximity_threshold,
    );

    // Reject sparse sections - a valid LineString requires at least 2 points
    if consensus.polyline.len() < 2 {
        return None;
    }

    // Use consensus polyline and update distance
    let consensus_distance = calculate_route_distance(&consensus.polyline);

    // Filter by min length - consensus might collapse to fewer points
    if consensus_distance < config.min_section_length {
        return None;
    }

    // Count activity_ids before moving
    let activity_count = cluster.activity_ids.len();

    // Compute initial stability of the selected medoid against the consensus
    let medoid_trace = track_map.get(representative_id.as_str()).map(|track| {
        // Find the overlap range from cluster overlaps for this activity
        cluster
            .overlaps
            .iter()
            .find(|o| o.activity_a == representative_id || o.activity_b == representative_id)
            .map(|o| {
                let (range, track_ref) = if o.activity_a == representative_id {
                    (o.range_a, *track)
                } else {
                    (o.range_b, *track)
                };
                let end = range.1.min(track_ref.len());
                track_ref[range.0..end].to_vec()
            })
            .unwrap_or_else(|| track.to_vec())
    });

    let stability = medoid_trace
        .as_ref()
        .map(|trace| compute_stability(trace, &consensus.polyline, config.proximity_threshold))
        .unwrap_or(0.0);

    Some(FrequentSection {
        id: format!("sec_{}_{}", sport_type.to_lowercase(), idx),
        name: None,
        sport_type: sport_type.to_string(),
        polyline: consensus.polyline,
        representative_activity_id: representative_id,
        activity_ids: cluster.activity_ids.into_iter().collect(),
        activity_portions,
        route_ids,
        // visit_count should equal unique activities
        visit_count: activity_count as u32,
        distance_meters: consensus_distance,
        // Lazy activity_traces: empty during detection, populated on-demand
        activity_traces: HashMap::new(),
        confidence: consensus.confidence,
        observation_count: consensus.observation_count,
        average_spread: consensus.average_spread,
        point_density: consensus.point_density,
        scale: scale_name,
        is_user_defined: false,
        stability,
        version: 1,
        updated_at: None,
        created_at: None,
    })
}

/// Detect frequent sections from FULL GPS tracks.
/// This is the main entry point for section detection.
pub fn detect_sections_from_tracks(
    tracks: &[(String, Vec<GpsPoint>)], // (activity_id, full_gps_points)
    sport_types: &HashMap<String, String>,
    groups: &[RouteGroup],
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    info!("[Sections] Detecting from {} full GPS tracks", tracks.len());

    if tracks.len() < config.min_activities as usize {
        return vec![];
    }

    // Filter to only groups with 2+ activities (these are the ones shown in Routes list)
    let significant_groups: Vec<&RouteGroup> = groups
        .iter()
        .filter(|g| g.activity_ids.len() >= 2)
        .collect();

    // Build activity_id -> route_id mapping (only for significant groups)
    let activity_to_route: HashMap<&str, &str> = significant_groups
        .iter()
        .flat_map(|g| {
            g.activity_ids
                .iter()
                .map(|aid| (aid.as_str(), g.group_id.as_str()))
        })
        .collect();

    // Debug: log the groups we received
    info!(
        "[Sections] Received {} groups, {} with 2+ activities, {} total activity mappings",
        groups.len(),
        significant_groups.len(),
        activity_to_route.len()
    );

    // Build track lookup — borrow from input tracks, no cloning
    let track_map: HashMap<&str, &[GpsPoint]> = tracks
        .iter()
        .map(|(id, pts)| (id.as_str(), pts.as_slice()))
        .collect();

    // Group tracks by sport type
    let mut tracks_by_sport: HashMap<String, Vec<(&str, &[GpsPoint])>> = HashMap::new();
    for (activity_id, points) in tracks {
        let sport = sport_types
            .get(activity_id)
            .cloned()
            .unwrap_or_else(|| "Unknown".to_string());
        tracks_by_sport
            .entry(sport)
            .or_default()
            .push((activity_id.as_str(), points.as_slice()));
    }

    let mut all_sections: Vec<FrequentSection> = Vec::new();
    let mut section_counter = 0;

    // Process each sport type
    for (sport_type, sport_tracks) in &tracks_by_sport {
        if sport_tracks.len() < config.min_activities as usize {
            continue;
        }

        info!(
            "[Sections] Processing {} {} tracks",
            sport_tracks.len(),
            sport_type
        );

        // Pre-compute bounding boxes once per track (avoids O(N²×P) recomputation)
        let track_bounds: Vec<Bounds> = sport_tracks
            .iter()
            .map(|(_, pts)| compute_bounds(pts))
            .collect();

        let rtree_start = std::time::Instant::now();
        #[cfg(feature = "parallel")]
        let rtrees: Vec<rstar::RTree<IndexedPoint>> = sport_tracks
            .par_iter()
            .map(|(_, pts)| build_rtree(pts))
            .collect();

        #[cfg(not(feature = "parallel"))]
        let rtrees: Vec<rstar::RTree<IndexedPoint>> = sport_tracks
            .iter()
            .map(|(_, pts)| build_rtree(pts))
            .collect();

        info!(
            "[Sections] Built {} R-trees in {}ms",
            rtrees.len(),
            rtree_start.elapsed().as_millis()
        );

        // Find pairwise overlaps - PARALLELIZED with rayon
        let overlap_start = std::time::Instant::now();

        // Generate all pairs
        let pairs: Vec<(usize, usize)> = (0..sport_tracks.len())
            .flat_map(|i| ((i + 1)..sport_tracks.len()).map(move |j| (i, j)))
            .collect();

        let total_pairs = pairs.len();

        // Process pairs (parallel if feature enabled)
        #[cfg(feature = "parallel")]
        let overlaps: Vec<FullTrackOverlap> = pairs
            .into_par_iter()
            .filter_map(|(i, j)| {
                let (id_a, track_a) = sport_tracks[i];
                let (id_b, track_b) = sport_tracks[j];

                // Quick bounding box check using pre-computed bounds
                let ref_lat = (track_bounds[i].min_lat + track_bounds[i].max_lat) / 2.0;
                if !bounds_overlap(
                    &track_bounds[i],
                    &track_bounds[j],
                    config.proximity_threshold,
                    ref_lat,
                ) {
                    return None;
                }

                // Find overlap using R-tree
                find_full_track_overlap(id_a, track_a, id_b, track_b, &rtrees[j], config)
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let overlaps: Vec<FullTrackOverlap> = pairs
            .into_iter()
            .filter_map(|(i, j)| {
                let (id_a, track_a) = sport_tracks[i];
                let (id_b, track_b) = sport_tracks[j];

                // Quick bounding box check using pre-computed bounds
                let ref_lat = (track_bounds[i].min_lat + track_bounds[i].max_lat) / 2.0;
                if !bounds_overlap(
                    &track_bounds[i],
                    &track_bounds[j],
                    config.proximity_threshold,
                    ref_lat,
                ) {
                    return None;
                }

                // Find overlap using R-tree
                find_full_track_overlap(id_a, track_a, id_b, track_b, &rtrees[j], config)
            })
            .collect();

        info!(
            "[Sections] Found {} pairwise overlaps for {} ({} pairs) in {}ms",
            overlaps.len(),
            sport_type,
            total_pairs,
            overlap_start.elapsed().as_millis()
        );

        // Cluster overlaps (pass sport_tracks for point resolution)
        let cluster_start = std::time::Instant::now();
        let clusters = cluster_overlaps(overlaps, config, sport_tracks);

        // Filter to clusters with enough activities
        let significant_clusters: Vec<_> = clusters
            .into_iter()
            .filter(|c| c.activity_ids.len() >= config.min_activities as usize)
            .collect();

        info!(
            "[Sections] {} significant clusters ({}+ activities) for {} in {}ms",
            significant_clusters.len(),
            config.min_activities,
            sport_type,
            cluster_start.elapsed().as_millis()
        );

        // Convert clusters to sections - PARALLELIZED with rayon
        let section_convert_start = std::time::Instant::now();

        // Prepare data for parallel processing
        let cluster_data: Vec<_> = significant_clusters.into_iter().enumerate().collect();

        // Process clusters (parallel if feature enabled)
        #[cfg(feature = "parallel")]
        let sport_sections: Vec<FrequentSection> = cluster_data
            .into_par_iter()
            .filter_map(|(idx, cluster)| {
                process_cluster(
                    idx,
                    cluster,
                    sport_type,
                    &track_map,
                    &activity_to_route,
                    config,
                    None,
                )
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let sport_sections: Vec<FrequentSection> = cluster_data
            .into_iter()
            .filter_map(|(idx, cluster)| {
                process_cluster(
                    idx,
                    cluster,
                    sport_type,
                    &track_map,
                    &activity_to_route,
                    config,
                    None,
                )
            })
            .collect();

        info!(
            "[Sections] Converted {} sections for {} in {}ms",
            sport_sections.len(),
            sport_type,
            section_convert_start.elapsed().as_millis()
        );

        // Post-process step 1: Split sections that fold back on themselves (out-and-back)
        let fold_start = std::time::Instant::now();
        let fold_sections = split_folding_sections(sport_sections, config);
        info!(
            "[Sections] After fold splitting: {} sections in {}ms",
            fold_sections.len(),
            fold_start.elapsed().as_millis()
        );

        // Post-process step 1b: Split sections at heading inflection points
        let heading_start = std::time::Instant::now();
        let heading_sections = split_at_heading_changes(fold_sections, config);
        info!(
            "[Sections] After heading splitting: {} sections in {}ms",
            heading_sections.len(),
            heading_start.elapsed().as_millis()
        );

        // Post-process step 1c: Split sections at gradient changes (if elevation available)
        let gradient_start = std::time::Instant::now();
        let gradient_sections = split_at_gradient_changes(heading_sections, config);
        info!(
            "[Sections] After gradient splitting: {} sections in {}ms",
            gradient_sections.len(),
            gradient_start.elapsed().as_millis()
        );

        // Post-process step 2: Merge sections that are nearby (reversed, parallel, GPS drift)
        let merge_start = std::time::Instant::now();
        let merged_sections = merge_nearby_sections(gradient_sections, config);
        info!(
            "[Sections] After nearby merge: {} sections in {}ms",
            merged_sections.len(),
            merge_start.elapsed().as_millis()
        );

        // Post-process step 3: Remove sections that contain or are contained by others
        let dedup_start = std::time::Instant::now();
        let deduped_sections = remove_overlapping_sections(merged_sections, config);
        info!(
            "[Sections] After dedup: {} unique sections in {}ms",
            deduped_sections.len(),
            dedup_start.elapsed().as_millis()
        );

        // Post-process step 4: Split sections with high-traffic portions
        // This creates new sections from portions that are used by many activities
        let split_start = std::time::Instant::now();
        let final_sections = split_high_variance_sections(deduped_sections, &track_map, config);
        info!(
            "[Sections] After density splitting: {} sections in {}ms",
            final_sections.len(),
            split_start.elapsed().as_millis()
        );

        // Re-number sections
        for (i, mut section) in final_sections.into_iter().enumerate() {
            section.id = format!("sec_{}_{}", sport_type.to_lowercase(), section_counter + i);
            all_sections.push(section);
        }
        section_counter += all_sections.len();
    }

    // Sort by visit count (most visited first)
    all_sections.sort_by(|a, b| b.visit_count.cmp(&a.visit_count));

    info!("[Sections] Detected {} total sections", all_sections.len());

    all_sections
}

pub fn detect_sections_multiscale(
    tracks: &[(String, Vec<GpsPoint>)],
    sport_types: &HashMap<String, String>,
    groups: &[RouteGroup],
    config: &SectionConfig,
) -> MultiScaleSectionResult {
    detect_sections_multiscale_with_progress(
        tracks,
        sport_types,
        groups,
        config,
        Arc::new(NoopProgress),
    )
}

/// Detect sections with a progress callback for real-time UI updates.
///
/// Architecture: Overlaps are computed ONCE per sport type and shared across all
/// scale presets (filtered by length). This reduces memory and computation by ~5x
/// compared to the previous approach of running independent overlap detection per preset.
///
/// Phases reported:
/// - `BuildingRtrees`: total = tracks across all sport types
/// - `FindingOverlaps`: total = pairs across all sport types
/// - `Postprocessing`: total = 6 (one per post-processing step)
pub fn detect_sections_multiscale_with_progress(
    tracks: &[(String, Vec<GpsPoint>)],
    sport_types: &HashMap<String, String>,
    groups: &[RouteGroup],
    config: &SectionConfig,
    progress: Arc<dyn DetectionProgressCallback>,
) -> MultiScaleSectionResult {
    info!(
        "[MultiScale] Detecting from {} tracks with {} scale presets",
        tracks.len(),
        config.scale_presets.len()
    );

    let mut stats = DetectionStats {
        activities_processed: tracks.len() as u32,
        overlaps_found: 0,
        sections_by_scale: HashMap::new(),
        potentials_by_scale: HashMap::new(),
    };

    // If no scale presets, fall back to legacy single-scale detection
    if config.scale_presets.is_empty() {
        let sections = detect_sections_from_tracks(tracks, sport_types, groups, config);
        stats
            .sections_by_scale
            .insert("legacy".to_string(), sections.len() as u32);
        return MultiScaleSectionResult {
            sections,
            potentials: vec![],
            stats,
        };
    }

    // Build shared data structures once — borrow from input tracks, no cloning
    let track_map: HashMap<&str, &[GpsPoint]> = tracks
        .iter()
        .map(|(id, pts)| (id.as_str(), pts.as_slice()))
        .collect();

    let significant_groups: Vec<&RouteGroup> = groups
        .iter()
        .filter(|g| g.activity_ids.len() >= 2)
        .collect();

    let activity_to_route: HashMap<&str, &str> = significant_groups
        .iter()
        .flat_map(|g| {
            g.activity_ids
                .iter()
                .map(|aid| (aid.as_str(), g.group_id.as_str()))
        })
        .collect();

    // Group tracks by sport type
    let mut tracks_by_sport: HashMap<String, Vec<(&str, &[GpsPoint])>> = HashMap::new();
    for (activity_id, points) in tracks {
        let sport = sport_types
            .get(activity_id)
            .cloned()
            .unwrap_or_else(|| "Unknown".to_string());
        tracks_by_sport
            .entry(sport)
            .or_default()
            .push((activity_id.as_str(), points.as_slice()));
    }

    // Calculate totals for progress reporting (now ONCE, not per-preset)
    let total_tracks_across_sports: u32 = tracks_by_sport.values().map(|v| v.len() as u32).sum();

    // Phase 1: Building R-trees (ONCE per sport type, not per preset)
    progress.on_phase(DetectionPhase::BuildingRtrees, total_tracks_across_sports);

    // Use the smallest min_section_length across all presets for overlap detection
    // This ensures we capture overlaps that any preset might need
    let global_min_length = config
        .scale_presets
        .iter()
        .map(|p| p.min_length)
        .fold(f64::MAX, f64::min);

    let overlap_config = SectionConfig {
        min_section_length: global_min_length,
        ..config.clone()
    };

    let mut all_sections: Vec<FrequentSection> = Vec::new();
    let mut all_potentials: Vec<PotentialSection> = Vec::new();

    // Process each sport type: build R-trees ONCE, find overlaps ONCE, then filter per preset
    for (sport_type, sport_tracks) in &tracks_by_sport {
        let min_tracks_for_processing = if config.include_potentials {
            1
        } else {
            config
                .scale_presets
                .iter()
                .map(|p| p.min_activities as usize)
                .min()
                .unwrap_or(1)
        };
        if sport_tracks.len() < min_tracks_for_processing {
            continue;
        }

        // Downsample tracks for overlap detection (~10x memory reduction)
        let downsample_target = 100;
        #[cfg(feature = "parallel")]
        let downsampled: Vec<Vec<GpsPoint>> = sport_tracks
            .par_iter()
            .map(|(_, pts)| optimized::downsample_track(pts, downsample_target))
            .collect();

        #[cfg(not(feature = "parallel"))]
        let downsampled: Vec<Vec<GpsPoint>> = sport_tracks
            .iter()
            .map(|(_, pts)| optimized::downsample_track(pts, downsample_target))
            .collect();

        // Pre-compute bounding boxes once per track (avoids O(N²×P) recomputation)
        let ds_bounds: Vec<Bounds> = downsampled.iter().map(|ds| compute_bounds(ds)).collect();

        // Grid filter BEFORE R-tree construction — skip R-trees for unpaired tracks
        #[cfg(feature = "parallel")]
        let track_cells: Vec<std::collections::HashSet<optimized::GridCell>> = downsampled
            .par_iter()
            .map(|ds| optimized::compute_grid_cells(ds))
            .collect();

        #[cfg(not(feature = "parallel"))]
        let track_cells: Vec<std::collections::HashSet<optimized::GridCell>> = downsampled
            .iter()
            .map(|ds| optimized::compute_grid_cells(ds))
            .collect();
        let pairs = optimized::grid_filtered_pairs(&track_cells);
        drop(track_cells);

        let exhaustive_pairs = sport_tracks.len() * sport_tracks.len().saturating_sub(1) / 2;
        info!(
            "[MultiScale] Grid filtering: {} candidate pairs (of {} possible) for {}",
            pairs.len(),
            exhaustive_pairs,
            sport_type,
        );

        // Early exit: no candidate pairs means no overlaps possible
        if pairs.is_empty() {
            info!(
                "[MultiScale] No candidate pairs for {}, skipping",
                sport_type
            );
            // Report progress for skipped phases
            for _ in 0..sport_tracks.len() {
                progress.on_progress();
            }
            continue;
        }

        // Build R-trees only for tracks that appear as j-index in pairs (lazy construction)
        let rtree_start = std::time::Instant::now();
        let mut needed_rtrees: HashSet<usize> = HashSet::new();
        for &(_, j) in &pairs {
            needed_rtrees.insert(j);
        }

        let mut rtrees: Vec<Option<rstar::RTree<IndexedPoint>>> =
            Vec::with_capacity(downsampled.len());
        for _ in 0..downsampled.len() {
            rtrees.push(None);
        }

        // Build only needed R-trees
        {
            let needed_indices: Vec<usize> = needed_rtrees.iter().copied().collect();
            #[cfg(feature = "parallel")]
            {
                let built: Vec<(usize, rstar::RTree<IndexedPoint>)> = needed_indices
                    .into_par_iter()
                    .map(|idx| {
                        let tree = build_rtree(&downsampled[idx]);
                        progress.on_progress();
                        (idx, tree)
                    })
                    .collect();
                for (idx, tree) in built {
                    rtrees[idx] = Some(tree);
                }
            }

            #[cfg(not(feature = "parallel"))]
            {
                for idx in needed_indices {
                    let tree = build_rtree(&downsampled[idx]);
                    progress.on_progress();
                    rtrees[idx] = Some(tree);
                }
            }
        }

        // Report progress for skipped R-trees
        let skipped_rtrees = sport_tracks.len() - needed_rtrees.len();
        for _ in 0..skipped_rtrees {
            progress.on_progress();
        }

        info!(
            "[MultiScale] Built {} R-trees (of {}, downsampled to ~{}) for {} in {}ms",
            needed_rtrees.len(),
            downsampled.len(),
            downsample_target,
            sport_type,
            rtree_start.elapsed().as_millis()
        );

        // Find ALL pairwise overlaps ONCE using downsampled tracks
        let grid_pair_count: u32 = pairs.len() as u32;
        progress.on_phase(DetectionPhase::FindingOverlaps, grid_pair_count);

        let overlap_start = std::time::Instant::now();

        // Use downsampled tracks for overlap detection, then map indices back
        let full_lengths: Vec<usize> = sport_tracks.iter().map(|(_, pts)| pts.len()).collect();
        let ds_lengths: Vec<usize> = downsampled.iter().map(|ds| ds.len()).collect();

        // Batch progress: report every 1% or every 100 pairs, whichever is smaller
        let progress_interval = (pairs.len() / 100).max(1).min(100);

        #[cfg(feature = "parallel")]
        let all_overlaps: Vec<FullTrackOverlap> = {
            let counter = AtomicUsize::new(0);
            pairs
                .into_par_iter()
                .filter_map(|(i, j)| {
                    let result = {
                        let (id_a, _) = sport_tracks[i];
                        let (id_b, _) = sport_tracks[j];
                        let ds_a = &downsampled[i];
                        let ds_b = &downsampled[j];
                        // Quick bounding box check using pre-computed bounds
                        let ref_lat = (ds_bounds[i].min_lat + ds_bounds[i].max_lat) / 2.0;
                        if !bounds_overlap(
                            &ds_bounds[i],
                            &ds_bounds[j],
                            overlap_config.proximity_threshold,
                            ref_lat,
                        ) {
                            None
                        } else {
                            let tree_j = rtrees[j].as_ref().expect("R-tree must exist for j-index");
                            find_full_track_overlap(id_a, ds_a, id_b, ds_b, tree_j, &overlap_config)
                                .map(|mut overlap| {
                                    // Map downsampled indices back to full-resolution
                                    let step_a =
                                        full_lengths[i] as f64 / ds_lengths[i].max(1) as f64;
                                    let step_b =
                                        full_lengths[j] as f64 / ds_lengths[j].max(1) as f64;
                                    overlap.range_a = (
                                        (overlap.range_a.0 as f64 * step_a) as usize,
                                        ((overlap.range_a.1 as f64 * step_a) as usize)
                                            .min(full_lengths[i]),
                                    );
                                    overlap.range_b = (
                                        (overlap.range_b.0 as f64 * step_b) as usize,
                                        ((overlap.range_b.1 as f64 * step_b) as usize)
                                            .min(full_lengths[j]),
                                    );
                                    overlap
                                })
                        }
                    };
                    let prev = counter.fetch_add(1, Ordering::Relaxed);
                    if prev % progress_interval == 0 {
                        // Batch report: advance progress by the interval amount
                        for _ in 0..progress_interval.min(grid_pair_count as usize - prev) {
                            progress.on_progress();
                        }
                    }
                    result
                })
                .collect()
        };

        #[cfg(not(feature = "parallel"))]
        let all_overlaps: Vec<FullTrackOverlap> = {
            let mut counter: usize = 0;
            pairs
                .into_iter()
                .filter_map(|(i, j)| {
                    let result = {
                        let (id_a, _) = sport_tracks[i];
                        let (id_b, _) = sport_tracks[j];
                        let ds_a = &downsampled[i];
                        let ds_b = &downsampled[j];
                        // Quick bounding box check using pre-computed bounds
                        let ref_lat = (ds_bounds[i].min_lat + ds_bounds[i].max_lat) / 2.0;
                        if !bounds_overlap(
                            &ds_bounds[i],
                            &ds_bounds[j],
                            overlap_config.proximity_threshold,
                            ref_lat,
                        ) {
                            None
                        } else {
                            let tree_j = rtrees[j].as_ref().expect("R-tree must exist for j-index");
                            find_full_track_overlap(id_a, ds_a, id_b, ds_b, tree_j, &overlap_config)
                                .map(|mut overlap| {
                                    let step_a =
                                        full_lengths[i] as f64 / ds_lengths[i].max(1) as f64;
                                    let step_b =
                                        full_lengths[j] as f64 / ds_lengths[j].max(1) as f64;
                                    overlap.range_a = (
                                        (overlap.range_a.0 as f64 * step_a) as usize,
                                        ((overlap.range_a.1 as f64 * step_a) as usize)
                                            .min(full_lengths[i]),
                                    );
                                    overlap.range_b = (
                                        (overlap.range_b.0 as f64 * step_b) as usize,
                                        ((overlap.range_b.1 as f64 * step_b) as usize)
                                            .min(full_lengths[j]),
                                    );
                                    overlap
                                })
                        }
                    };
                    counter += 1;
                    if counter % progress_interval == 0 {
                        for _ in 0..progress_interval {
                            progress.on_progress();
                        }
                    }
                    result
                })
                .collect()
        };

        stats.overlaps_found += all_overlaps.len() as u32;
        info!(
            "[MultiScale] Found {} total overlaps for {} in {}ms",
            all_overlaps.len(),
            sport_type,
            overlap_start.elapsed().as_millis()
        );

        // Drop R-trees and downsampled tracks to free memory before clustering
        drop(rtrees);
        drop(downsampled);

        // Process each scale preset: filter overlaps, cluster, convert to sections
        // Each preset is independent — parallelize across presets
        let process_preset =
            |preset: &ScalePreset| -> (Vec<FrequentSection>, Vec<PotentialSection>) {
                let scale_config = SectionConfig {
                    min_section_length: preset.min_length,
                    max_section_length: preset.max_length,
                    min_activities: preset.min_activities,
                    ..config.clone()
                };

                // Filter overlaps by this preset's length range
                let filtered_overlaps: Vec<FullTrackOverlap> = all_overlaps
                    .iter()
                    .filter(|o| o.overlap_length >= preset.min_length)
                    .cloned()
                    .collect();

                info!(
                    "[MultiScale] {} scale: {} overlaps (of {}) for {}",
                    preset.name,
                    filtered_overlaps.len(),
                    all_overlaps.len(),
                    sport_type,
                );

                // Cluster overlaps for this scale
                let clusters = cluster_overlaps(filtered_overlaps, &scale_config, sport_tracks);

                // Separate into confirmed sections and potential sections
                let (significant, potential): (Vec<_>, Vec<_>) = clusters
                    .into_iter()
                    .partition(|c| c.activity_ids.len() >= preset.min_activities as usize);

                // Process confirmed sections (already uses par_iter internally on clusters)
                let sport_sections: Vec<FrequentSection> = significant
                    .into_iter()
                    .enumerate()
                    .filter_map(|(idx, cluster)| {
                        process_cluster(
                            idx,
                            cluster,
                            sport_type,
                            &track_map,
                            &activity_to_route,
                            &scale_config,
                            Some(preset.name),
                        )
                    })
                    .collect();

                // Process potential sections if enabled
                let mut preset_potentials = Vec::new();
                if config.include_potentials {
                    for (idx, cluster) in potential.into_iter().enumerate() {
                        let activity_count = cluster.activity_ids.len();
                        if activity_count >= 1
                            && activity_count < preset.min_activities as usize
                            && let Some((_rep_id, rep_polyline)) =
                                Some(select_medoid(&cluster, &track_map))
                            && !rep_polyline.is_empty()
                        {
                            let distance = calculate_route_distance(&rep_polyline);
                            if distance >= preset.min_length && distance <= preset.max_length {
                                preset_potentials.push(PotentialSection {
                                    id: format!(
                                        "pot_{}_{}_{}",
                                        preset.name,
                                        sport_type.to_lowercase(),
                                        idx
                                    ),
                                    sport_type: sport_type.to_string(),
                                    polyline: rep_polyline,
                                    activity_ids: cluster.activity_ids.into_iter().collect(),
                                    visit_count: activity_count as u32,
                                    distance_meters: distance,
                                    confidence: 0.3 + (activity_count as f64 * 0.2),
                                    scale: preset.name,
                                });
                            }
                        }
                    }
                }

                info!(
                    "[MultiScale] {} scale: {} sections for {}",
                    preset.name,
                    sport_sections.len(),
                    sport_type,
                );

                (sport_sections, preset_potentials)
            };

        #[cfg(feature = "parallel")]
        let preset_results: Vec<(Vec<FrequentSection>, Vec<PotentialSection>)> = config
            .scale_presets
            .par_iter()
            .map(process_preset)
            .collect();

        #[cfg(not(feature = "parallel"))]
        let preset_results: Vec<(Vec<FrequentSection>, Vec<PotentialSection>)> =
            config.scale_presets.iter().map(process_preset).collect();

        // Merge results from all presets
        for (idx, (sections, potentials)) in preset_results.into_iter().enumerate() {
            let preset_name = config.scale_presets[idx].name.as_str().to_string();
            stats
                .sections_by_scale
                .entry(preset_name.clone())
                .and_modify(|n| *n += sections.len() as u32)
                .or_insert(sections.len() as u32);
            stats
                .potentials_by_scale
                .entry(preset_name)
                .and_modify(|n| *n += potentials.len() as u32)
                .or_insert(potentials.len() as u32);
            all_sections.extend(sections);
            all_potentials.extend(potentials);
        }
    }

    // Phase 3: Post-processing (6 steps)
    progress.on_phase(DetectionPhase::Postprocessing, 6);

    // Apply post-processing
    let fold_start = std::time::Instant::now();
    let fold_sections = split_folding_sections(all_sections, config);
    progress.on_progress();
    info!(
        "[MultiScale] After fold splitting: {} sections in {}ms",
        fold_sections.len(),
        fold_start.elapsed().as_millis()
    );

    // Split at heading inflection points
    let heading_start = std::time::Instant::now();
    let heading_sections = split_at_heading_changes(fold_sections, config);
    progress.on_progress();
    info!(
        "[MultiScale] After heading splitting: {} sections in {}ms",
        heading_sections.len(),
        heading_start.elapsed().as_millis()
    );

    // Split at gradient changes (if elevation available)
    let gradient_start = std::time::Instant::now();
    let gradient_sections = split_at_gradient_changes(heading_sections, config);
    progress.on_progress();
    info!(
        "[MultiScale] After gradient splitting: {} sections in {}ms",
        gradient_sections.len(),
        gradient_start.elapsed().as_millis()
    );

    let merge_start = std::time::Instant::now();
    let merged_sections = merge_nearby_sections(gradient_sections, config);
    progress.on_progress();
    info!(
        "[MultiScale] After nearby merge: {} sections in {}ms",
        merged_sections.len(),
        merge_start.elapsed().as_millis()
    );

    // Use hierarchical deduplication if preserve_hierarchy is true
    let dedup_start = std::time::Instant::now();
    let deduped_sections = if config.preserve_hierarchy {
        remove_overlapping_sections_hierarchical(merged_sections, config)
    } else {
        remove_overlapping_sections(merged_sections, config)
    };
    progress.on_progress();
    info!(
        "[MultiScale] After dedup: {} sections in {}ms",
        deduped_sections.len(),
        dedup_start.elapsed().as_millis()
    );

    let split_start = std::time::Instant::now();
    let final_sections = split_high_variance_sections(deduped_sections, &track_map, config);
    // Explicitly drop track_map — it's no longer needed and all borrows into `tracks` are released
    drop(track_map);
    progress.on_progress();
    info!(
        "[MultiScale] After density splitting: {} sections in {}ms",
        final_sections.len(),
        split_start.elapsed().as_millis()
    );

    // Sort sections by visit count
    let mut sorted_sections = final_sections;
    sorted_sections.sort_by(|a, b| b.visit_count.cmp(&a.visit_count));

    // Re-assign unique IDs after all post-processing
    // Each scale generates IDs independently (sec_run_0, sec_run_1, ...),
    // so merging scales can produce duplicate IDs. Renumber here.
    let mut id_counters: HashMap<String, usize> = HashMap::new();
    for section in &mut sorted_sections {
        let sport_key = section.sport_type.to_lowercase();
        let counter = id_counters.entry(sport_key.clone()).or_insert(0);
        section.id = format!("sec_{}_{}", sport_key, *counter);
        *counter += 1;
    }

    // Sort potentials by confidence
    let mut sorted_potentials = all_potentials;
    sorted_potentials.sort_by(|a, b| {
        b.confidence
            .partial_cmp(&a.confidence)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    info!(
        "[MultiScale] Final: {} sections, {} potentials",
        sorted_sections.len(),
        sorted_potentials.len()
    );

    MultiScaleSectionResult {
        sections: sorted_sections,
        potentials: sorted_potentials,
        stats,
    }
}

/// Deduplication that preserves hierarchical sections.
/// Short sections inside longer ones are kept if they're at different scales.
fn remove_overlapping_sections_hierarchical(
    mut sections: Vec<FrequentSection>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    if sections.len() <= 1 {
        return sections;
    }

    // Sort by length descending
    sections.sort_by(|a, b| {
        b.distance_meters
            .partial_cmp(&a.distance_meters)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // PRE-COMPUTE all R-trees once upfront (O(k) builds instead of O(k²))
    #[cfg(feature = "parallel")]
    let rtrees: Vec<rstar::RTree<IndexedPoint>> = {
        use rayon::prelude::*;
        sections
            .par_iter()
            .map(|s| build_rtree(&s.polyline))
            .collect()
    };

    #[cfg(not(feature = "parallel"))]
    let rtrees: Vec<rstar::RTree<IndexedPoint>> =
        sections.iter().map(|s| build_rtree(&s.polyline)).collect();

    let mut keep = vec![true; sections.len()];

    for i in 0..sections.len() {
        if !keep[i] {
            continue;
        }

        let tree_i = &rtrees[i]; // Use pre-computed R-tree (O(1) lookup)

        for j in (i + 1)..sections.len() {
            if !keep[j] {
                continue;
            }

            // Check if shorter section (j) is contained in longer section (i)
            let containment = compute_polyline_containment_with_rtree(
                &sections[j].polyline,
                tree_i,
                config.proximity_threshold,
            );

            // Length ratio
            let length_ratio = sections[j].distance_meters / sections[i].distance_meters;

            // Only remove if:
            // 1. >90% contained
            // 2. Same length class (ratio > 0.7) - meaning it's a true duplicate, not hierarchical
            // 3. Same scale OR no scale info
            let same_scale = match (&sections[i].scale, &sections[j].scale) {
                (Some(a), Some(b)) => a == b,
                _ => true, // If either has no scale, treat as same
            };

            if containment > 0.9 && length_ratio > 0.7 && same_scale {
                keep[j] = false;
            }
        }
    }

    sections
        .into_iter()
        .zip(keep)
        .filter_map(|(s, k)| if k { Some(s) } else { None })
        .collect()
}

/// Compute what fraction of polyline A is contained within proximity of polyline B (using R-tree).
/// O(a * log b) instead of O(a * b).
fn compute_polyline_containment_with_rtree(
    polyline_a: &[GpsPoint],
    tree_b: &rstar::RTree<IndexedPoint>,
    proximity_threshold: f64,
) -> f64 {
    use rstar::PointDistance;

    if polyline_a.is_empty() {
        return 0.0;
    }

    let threshold_deg = proximity_threshold / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;

    let mut contained_count = 0;
    for point_a in polyline_a {
        let query = [point_a.latitude, point_a.longitude];
        if let Some(nearest) = tree_b.nearest_neighbor(&query)
            && nearest.distance_2(&query) <= threshold_deg_sq
        {
            contained_count += 1;
        }
    }

    contained_count as f64 / polyline_a.len() as f64
}
