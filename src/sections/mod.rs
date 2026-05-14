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
mod corridor;
mod density_grid;
mod flow_graph;
pub mod incremental;
mod medoid;
pub mod optimized;
mod overlap;
mod portions;
mod postprocess;
pub mod progress;
mod rtree;
mod traces;

use crate::matching::calculate_route_distance;
use crate::{GpsPoint, RouteGroup};
use log::info;
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

pub use progress::{
    AtomicProgressTracker, DetectionPhase, DetectionProgressCallback, NoopProgress,
};

// Re-export internal utilities for use across submodules
pub(crate) use consensus::compute_consensus_polyline;
pub use consensus::{
    ConsensusAccumulator, ConsensusPointAccumulator, ConsensusResult, TraceRTreeCache,
    build_accumulator_from_traces, build_trace_rtree_cache, merge_traces_into_consensus,
    merge_traces_into_consensus_with_cache,
};
pub(crate) use medoid::{compute_stability, select_medoid};
pub use overlap::{FullTrackOverlap, OverlapCluster};
pub(crate) use portions::compute_activity_portions;
pub use portions::{find_all_track_portions, find_all_track_portions_with_gap};
pub use postprocess::{
    filter_low_quality_sections, merge_nearby_sections, remove_overlapping_sections,
};
pub(crate) use postprocess::{
    split_at_gradient_changes, split_at_heading_changes, split_folding_sections,
    split_high_variance_sections,
};
pub use rtree::{IndexedPoint, build_rtree};
pub use traces::{extract_activity_trace, extract_all_activity_traces};

// Re-export single-route section utilities (find/split known sections).
pub use optimized::{
    SectionMatch, SplitResult, find_all_section_spans_in_route, find_sections_in_route,
    recalculate_section_polyline, split_section_at_index, split_section_at_point,
};

/// Detection mode for section detection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
#[derive(Default)]
pub enum DetectionMode {
    /// Lower thresholds, more sections detected
    #[default]
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

/// Which algorithm to use for section detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
#[derive(Default)]
pub enum DetectionMethod {
    /// Density clustering on raw GPS traces. Default. Best coverage.
    #[default]
    Corridor,
    /// Route-overlap detection via rasterization + union-find.
    /// Requires pre-computed route groups.
    DensityGrid,
    /// Junction detection from directed GPS flow.
    /// Sections are edges between divergence points.
    FlowGraph,
}

impl DetectionMethod {
    pub fn as_str(&self) -> &'static str {
        match self {
            DetectionMethod::Corridor => "corridor",
            DetectionMethod::DensityGrid => "density_grid",
            DetectionMethod::FlowGraph => "flow_graph",
        }
    }
}

impl std::fmt::Display for DetectionMethod {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

impl std::str::FromStr for DetectionMethod {
    type Err = ();
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "corridor" => Ok(DetectionMethod::Corridor),
            "density_grid" | "densitygrid" | "density" => Ok(DetectionMethod::DensityGrid),
            "flow_graph" | "flowgraph" | "flow" => Ok(DetectionMethod::FlowGraph),
            _ => Ok(DetectionMethod::Corridor),
        }
    }
}

/// Scale name for multi-scale section detection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[derive(Default)]
pub enum ScaleName {
    Short,
    #[default]
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
    /// Density-grid section continuity. Two adjacent hot cells merge
    /// iff containment-of-minimum ≥ this threshold:
    /// `|intersection| / min(|A|, |B|)`. Lower (≈0.3) → longer
    /// continuous sections; higher (≈0.8) → more granular pieces
    /// that split at any track divergence. Default 0.5.
    #[serde(default = "default_jaccard_threshold")]
    pub jaccard_threshold: f64,
    /// Minimum number of *distinct routes* (not activities) that must
    /// converge in a region for it to qualify as a section. The
    /// "alphabet overlap" criterion: a section is the intersection of
    /// many routes, not the repetition of one. Default 3.
    #[serde(default = "default_min_routes")]
    pub min_routes: u32,
    /// Allow `split_high_variance_sections` postprocess step to fire.
    /// This step extracts dense "core" portions of sections as
    /// separate sections — useful for activity-based density input,
    /// noisy/over-fragmenting for route-intersection input. Off by
    /// default to avoid the 1.5× section-count multiplier.
    #[serde(default)]
    pub enable_density_splits: bool,
    /// Multiplier on `proximity_threshold` for the `merge_nearby_sections`
    /// postprocess step. Higher → more aggressive merging of nearby
    /// fragments into single sections. Default 4.0 (200 m at the
    /// default 150 m proximity).
    #[serde(default = "default_merge_distance_multiplier")]
    pub merge_distance_multiplier: f64,
    /// Minimum cell visits for a cell to be considered part of the road
    /// network in flow-graph mode. Default 5.
    #[serde(default = "default_min_cell_visits")]
    pub min_cell_visits: u32,
    /// Fraction of outgoing traffic an exit must carry to count as
    /// "significant" for divergence-point detection. Default 0.15 (15%).
    #[serde(default = "default_divergence_threshold")]
    pub divergence_threshold: f64,
    /// Minimum unique tracks per cell for corridor detection. Cells with
    /// fewer unique visitors are not "hot" and won't participate in
    /// skeletonization. Higher = fewer, more confident sections. Default 3.
    #[serde(default = "default_min_corridor_tracks")]
    pub min_corridor_tracks: u32,
    /// Which detection algorithm to use. Default: Corridor.
    #[serde(default)]
    pub detection_method: DetectionMethod,
}

fn default_jaccard_threshold() -> f64 {
    0.5
}
fn default_min_routes() -> u32 {
    3
}
fn default_merge_distance_multiplier() -> f64 {
    4.0
}
fn default_min_cell_visits() -> u32 {
    50
}
fn default_divergence_threshold() -> f64 {
    0.15
}
fn default_min_corridor_tracks() -> u32 {
    3
}

impl Default for SectionConfig {
    fn default() -> Self {
        Self {
            proximity_threshold: 150.0,
            min_section_length: 200.0,
            max_section_length: 200_000.0,
            min_activities: 3,
            cluster_tolerance: 80.0,
            sample_points: 50,
            detection_mode: DetectionMode::Discovery,
            include_potentials: true,
            scale_presets: ScalePreset::default_presets(),
            preserve_hierarchy: false,
            jaccard_threshold: default_jaccard_threshold(),
            min_routes: default_min_routes(),
            enable_density_splits: false,
            merge_distance_multiplier: default_merge_distance_multiplier(),
            min_cell_visits: default_min_cell_visits(),
            divergence_threshold: default_divergence_threshold(),
            min_corridor_tracks: default_min_corridor_tracks(),
            detection_method: DetectionMethod::default(),
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

    /// Incremental-consensus running sums. None means "not yet built" —
    /// the next merge will build it from current traces. Populated after
    /// the first merge through `merge_traces_into_consensus`. Skipped
    /// during JSON serialisation when None to keep blob size flat for
    /// sections that haven't been touched by the incremental path yet.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub consensus_state: Option<ConsensusAccumulator>,
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
/// Expand a route-defined cluster's activity_ids to include all
/// activities from the contributing routes.
///
/// `density_grid` runs on route representatives, so the returned
/// cluster's `activity_ids` initially contains only the rep IDs.
/// Downstream code (consensus polyline, activity portions, visit
/// counts) needs the full activity set across all contributing routes.
/// Since route grouping has already established "these activities are
/// the same route", every activity in a contributing route is assumed
/// to traverse the section.
///
/// The cluster's `overlaps` are NOT expanded — they keep the rep IDs
/// for `select_medoid` to pick a representative trace, which is correct
/// (medoid should be a route-shape representative, not an arbitrary
/// member activity).
fn expand_cluster_to_activities(
    mut cluster: OverlapCluster,
    significant_groups: &[&RouteGroup],
) -> OverlapCluster {
    let rep_ids: HashSet<String> = cluster.activity_ids.iter().cloned().collect();
    let mut expanded: HashSet<String> = HashSet::new();
    for group in significant_groups {
        if rep_ids.contains(&group.representative_id) {
            for aid in &group.activity_ids {
                expanded.insert(aid.clone());
            }
        }
    }
    if !expanded.is_empty() {
        cluster.activity_ids = expanded;
    }
    cluster
}

pub fn process_cluster(
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

    // Extract traces for consensus computation only (not stored in section).
    // Sort for stable iteration order — HashSet iteration is randomized
    // and the consensus computation is order-sensitive.
    let activity_id_vec: Vec<String> = {
        let mut v: Vec<String> = cluster.activity_ids.iter().cloned().collect();
        v.sort();
        v
    };
    let activity_traces_for_consensus =
        extract_all_activity_traces(&activity_id_vec, &representative_polyline, track_map);

    // Collect all traces in sorted-activity-id order so consensus is
    // deterministic. (HashMap::values() iteration order is randomized.)
    let all_traces: Vec<Vec<GpsPoint>> = activity_id_vec
        .iter()
        .filter_map(|id| activity_traces_for_consensus.get(id).cloned())
        .collect();

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
        activity_ids: {
            let mut ids: Vec<String> = cluster.activity_ids.into_iter().collect();
            ids.sort();
            ids
        },
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
        consensus_state: None,
    })
}

/// Detect sections using the configured method.
///
/// Dispatches to corridor, density grid, or flow graph based on
/// `config.detection_method`. Corridor is the default.
pub fn detect_sections(
    tracks: &[(String, Vec<GpsPoint>)],
    sport_types: &HashMap<String, String>,
    groups: &[RouteGroup],
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    match config.detection_method {
        DetectionMethod::Corridor => detect_sections_corridor(tracks, sport_types, config),
        DetectionMethod::FlowGraph => detect_sections_flow_graph(tracks, sport_types, config),
        DetectionMethod::DensityGrid => {
            detect_sections_multiscale(tracks, sport_types, groups, config).sections
        }
    }
}

/// Detect sections via density corridor extraction.
///
/// Rasterises all tracks, thresholds to hot cells, applies Zhang-Suen
/// morphological thinning to extract corridor centerlines, and snaps
/// each skeleton segment to an actual GPS track.
pub fn detect_sections_corridor(
    tracks: &[(String, Vec<GpsPoint>)],
    sport_types: &HashMap<String, String>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    info!(
        "[Corridor] Detecting from {} tracks (min_corridor_tracks={}, proximity={})",
        tracks.len(),
        config.min_corridor_tracks,
        config.proximity_threshold,
    );

    let mut tracks_by_sport: HashMap<String, Vec<(&str, &[GpsPoint])>> = HashMap::new();
    for (id, pts) in tracks {
        let sport = sport_types
            .get(id)
            .cloned()
            .unwrap_or_else(|| "Unknown".to_string());
        tracks_by_sport
            .entry(sport)
            .or_default()
            .push((id.as_str(), pts.as_slice()));
    }

    let mut all_sections = Vec::new();
    for (sport, sport_tracks) in &tracks_by_sport {
        if sport_tracks.len() < config.min_activities as usize {
            continue;
        }
        info!(
            "[Corridor] Processing {} {} tracks",
            sport_tracks.len(),
            sport
        );
        let sections = corridor::detect_sections_via_corridor(sport_tracks, sport, config);
        info!("[Corridor] {} sections for {}", sections.len(), sport);
        all_sections.extend(sections);
    }

    let before = all_sections.len();
    all_sections = postprocess::merge_nearby_sections(all_sections, config);
    all_sections = postprocess::remove_overlapping_sections(all_sections, config);
    info!(
        "[Corridor] {} sections after postprocess (was {})",
        all_sections.len(),
        before
    );

    all_sections
}

/// Detect sections via flow-graph analysis.
///
/// Builds a road/trail network from GPS traces by tracking cell-to-cell
/// flow, finds divergence points (junctions), and traces edges between
/// them. Each edge becomes a section.
pub fn detect_sections_flow_graph(
    tracks: &[(String, Vec<GpsPoint>)],
    sport_types: &HashMap<String, String>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    info!(
        "[FlowGraph] Detecting from {} tracks (min_visits={}, divergence={})",
        tracks.len(),
        config.min_cell_visits,
        config.divergence_threshold,
    );

    let mut tracks_by_sport: HashMap<String, Vec<(&str, &[GpsPoint])>> = HashMap::new();
    for (id, pts) in tracks {
        let sport = sport_types
            .get(id)
            .cloned()
            .unwrap_or_else(|| "Unknown".to_string());
        tracks_by_sport
            .entry(sport)
            .or_default()
            .push((id.as_str(), pts.as_slice()));
    }

    let mut all_sections = Vec::new();
    for (sport, sport_tracks) in &tracks_by_sport {
        if sport_tracks.len() < config.min_activities as usize {
            continue;
        }
        info!(
            "[FlowGraph] Processing {} {} tracks",
            sport_tracks.len(),
            sport
        );
        let sections = flow_graph::detect_sections_via_flow_graph(sport_tracks, sport, config);
        info!("[FlowGraph] {} sections for {}", sections.len(), sport);
        all_sections.extend(sections);
    }

    let before = all_sections.len();
    all_sections = postprocess::merge_nearby_sections(all_sections, config);
    all_sections = postprocess::remove_overlapping_sections(all_sections, config);
    info!(
        "[FlowGraph] {} sections after overlap removal (was {})",
        all_sections.len(),
        before
    );

    all_sections
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
            "[Sections] Processing {} {} tracks via density grid",
            sport_tracks.len(),
            sport_type
        );

        // Containment-gated density grid over ROUTE representatives
        // (one rep per significant group). Sections emerge from the
        // overlap of distinct routes — not from one route being
        // repeated. See `density_grid.rs` for the algorithm.
        //
        // Sport is derived from the authoritative `sport_types` map
        // because `g.sport_type` defaults to "Ride" inside the grouping
        // module unless the caller overrides it (see grouping.rs:626).
        let sport_route_reps: Vec<(&str, &[GpsPoint])> = significant_groups
            .iter()
            .filter(|g| {
                sport_types
                    .get(&g.representative_id)
                    .map(|s| s == sport_type)
                    .unwrap_or(false)
            })
            .filter_map(|g| {
                let rep_pts = track_map.get(g.representative_id.as_str())?;
                Some((g.representative_id.as_str(), *rep_pts))
            })
            .collect();
        if sport_route_reps.len() < config.min_routes as usize {
            continue;
        }
        let cluster_start = web_time::Instant::now();
        let raw_clusters = density_grid::detect_clusters_via_density(&sport_route_reps, config);
        let clusters: Vec<OverlapCluster> = raw_clusters
            .into_iter()
            .map(|c| expand_cluster_to_activities(c, &significant_groups))
            .collect();

        let significant_clusters: Vec<_> = clusters
            .into_iter()
            .filter(|c| c.activity_ids.len() >= config.min_activities as usize)
            .collect();

        info!(
            "[Sections] Density grid: {} significant clusters ({}+ activities) for {} in {}ms",
            significant_clusters.len(),
            config.min_activities,
            sport_type,
            cluster_start.elapsed().as_millis()
        );

        // Convert clusters to sections - PARALLELIZED with rayon
        let section_convert_start = web_time::Instant::now();

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
        let fold_start = web_time::Instant::now();
        let fold_sections = split_folding_sections(sport_sections, config);
        info!(
            "[Sections] After fold splitting: {} sections in {}ms",
            fold_sections.len(),
            fold_start.elapsed().as_millis()
        );

        // Post-process step 1b: Split sections at heading inflection points
        let heading_start = web_time::Instant::now();
        let heading_sections = split_at_heading_changes(fold_sections, config);
        info!(
            "[Sections] After heading splitting: {} sections in {}ms",
            heading_sections.len(),
            heading_start.elapsed().as_millis()
        );

        // Post-process step 1c: Split sections at gradient changes (if elevation available)
        let gradient_start = web_time::Instant::now();
        let gradient_sections = split_at_gradient_changes(heading_sections, config);
        info!(
            "[Sections] After gradient splitting: {} sections in {}ms",
            gradient_sections.len(),
            gradient_start.elapsed().as_millis()
        );

        // Post-process step 2: Merge sections that are nearby (reversed, parallel, GPS drift)
        let merge_start = web_time::Instant::now();
        let merged_sections = merge_nearby_sections(gradient_sections, config);
        info!(
            "[Sections] After nearby merge: {} sections in {}ms",
            merged_sections.len(),
            merge_start.elapsed().as_millis()
        );

        // Post-process step 3: Remove sections that contain or are contained by others
        let dedup_start = web_time::Instant::now();
        let deduped_sections = remove_overlapping_sections(merged_sections, config);
        info!(
            "[Sections] After dedup: {} unique sections in {}ms",
            deduped_sections.len(),
            dedup_start.elapsed().as_millis()
        );

        // Post-process step 4: Split sections with high-traffic portions.
        // Disabled by default — it's a 1.5× section count multiplier and
        // with route-intersection input the cluster IS the high-traffic
        // core, so there's nothing to extract. Opt-in via config.
        let split_start = web_time::Instant::now();
        let final_sections = if config.enable_density_splits {
            split_high_variance_sections(deduped_sections, &track_map, config)
        } else {
            deduped_sections
        };
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
    all_sections.sort_by_key(|s| std::cmp::Reverse(s.visit_count));

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

        // Phase 1 + 2: containment-gated density grid with connectivity
        // bridging. Route representatives are rasterised into cells of
        // size proximity_threshold; bridge cells (≥ 2 tracks) maintain
        // corridor connectivity where GPS jitter shifts a route rep.
        // Extraction still requires min_routes contributing tracks.
        //
        // BuildingRtrees fires once for the density-grid pass (which has
        // no internal progress hooks). FindingOverlaps then ticks per
        // cluster as we run process_cluster on each, so the UI bar moves
        // through the actual assembly work (which dominates wall time on
        // datasets with many clusters).
        progress.on_phase(DetectionPhase::BuildingRtrees, sport_tracks.len() as u32);

        // Build route representatives for THIS sport. Each significant
        // route group contributes one rep to the density grid. This is
        // the "alphabet" view: we draw each distinct route once, and
        // sections emerge from the OVERLAP of multiple routes — not
        // from one route being repeated.
        //
        // Sport is derived from the authoritative `sport_types` map via
        // the representative's id, NOT `g.sport_type` — the grouping
        // module defaults that field to "Ride" and relies on the caller
        // to override it, so trusting it here would silently drop every
        // group when the caller didn't set it.
        let sport_route_reps: Vec<(&str, &[GpsPoint])> = significant_groups
            .iter()
            .filter(|g| {
                sport_types
                    .get(&g.representative_id)
                    .map(|s| s == sport_type)
                    .unwrap_or(false)
            })
            .filter_map(|g| {
                let rep_pts = track_map.get(g.representative_id.as_str())?;
                Some((g.representative_id.as_str(), *rep_pts))
            })
            .collect();

        if sport_route_reps.len() < config.min_routes as usize {
            info!(
                "[MultiScale] {} has {} routes < min_routes ({}), skipping",
                sport_type,
                sport_route_reps.len(),
                config.min_routes
            );
            progress.on_phase(DetectionPhase::FindingOverlaps, 0);
            continue;
        }

        let cluster_start = web_time::Instant::now();
        let raw_clusters =
            density_grid::detect_clusters_via_density(&sport_route_reps, &overlap_config);

        // Expand each cluster's activity_ids: cluster currently
        // identifies routes via their representative IDs, but the
        // FrequentSection needs the full activity set across all
        // contributing routes for visit count and per-activity portions.
        let all_clusters: Vec<OverlapCluster> = raw_clusters
            .into_iter()
            .map(|c| expand_cluster_to_activities(c, &significant_groups))
            .collect();

        let total_overlaps: u32 = all_clusters.iter().map(|c| c.overlaps.len() as u32).sum();
        stats.overlaps_found += total_overlaps;
        info!(
            "[MultiScale] Density grid: {} clusters from {} routes ({} synthesised overlaps) for {} in {}ms",
            all_clusters.len(),
            sport_route_reps.len(),
            total_overlaps,
            sport_type,
            cluster_start.elapsed().as_millis()
        );

        if all_clusters.is_empty() {
            // Still emit a phase header so the UI doesn't sit on the
            // previous label while we move to the next sport / phase.
            progress.on_phase(DetectionPhase::FindingOverlaps, 0);
            continue;
        }

        // Bucket each cluster into the preset whose [min_length, max_length)
        // contains its representative length (longest contributing portion).
        // Each cluster maps to at most one preset.
        let bucketed: Vec<(OverlapCluster, &ScalePreset, bool)> = all_clusters
            .into_iter()
            .filter_map(|c| {
                let max_len = c
                    .overlaps
                    .iter()
                    .map(|o| o.overlap_length)
                    .fold(0.0_f64, f64::max);
                let preset = config
                    .scale_presets
                    .iter()
                    .find(|p| max_len >= p.min_length && max_len < p.max_length)?;
                let is_significant = c.activity_ids.len() >= preset.min_activities as usize;
                Some((c, preset, is_significant))
            })
            .collect();

        // Process all clusters in one parallel pass — each cluster runs
        // through process_cluster (significant) or PotentialSection
        // construction (below min_activities) exactly once. This
        // maximises rayon parallelism: 134 clusters in flight at once
        // instead of 4 preset-buckets each running their cluster set
        // sequentially.
        //
        // Emit a per-cluster tick from inside the closure so the progress
        // bar moves smoothly through assembly work (which dominates wall
        // time on datasets with many clusters). The FFI crossing cost per
        // tick is microseconds vs seconds spent in process_cluster, so we
        // don't bother batching.
        progress.on_phase(DetectionPhase::FindingOverlaps, bucketed.len() as u32);
        let progress_ref: &dyn DetectionProgressCallback = progress.as_ref();
        type ClusterResult = (Option<FrequentSection>, Option<PotentialSection>);
        let process_one = |(idx, (cluster, preset, is_significant)): (
            usize,
            (OverlapCluster, &ScalePreset, bool),
        )|
         -> ClusterResult {
            let scale_config = SectionConfig {
                min_section_length: preset.min_length,
                max_section_length: preset.max_length,
                min_activities: preset.min_activities,
                ..config.clone()
            };
            let result = if is_significant {
                let section = process_cluster(
                    idx,
                    cluster,
                    sport_type,
                    &track_map,
                    &activity_to_route,
                    &scale_config,
                    Some(preset.name),
                );
                (section, None)
            } else if config.include_potentials {
                let activity_count = cluster.activity_ids.len();
                if activity_count == 0 {
                    (None, None)
                } else {
                    let (_rep_id, rep_polyline) = select_medoid(&cluster, &track_map);
                    if rep_polyline.is_empty() {
                        (None, None)
                    } else {
                        let distance = calculate_route_distance(&rep_polyline);
                        if distance < preset.min_length || distance > preset.max_length {
                            (None, None)
                        } else {
                            let potential = PotentialSection {
                                id: format!(
                                    "pot_{}_{}_{}",
                                    preset.name,
                                    sport_type.to_lowercase(),
                                    idx
                                ),
                                sport_type: sport_type.to_string(),
                                polyline: rep_polyline,
                                activity_ids: {
                                    let mut ids: Vec<String> =
                                        cluster.activity_ids.into_iter().collect();
                                    ids.sort();
                                    ids
                                },
                                visit_count: activity_count as u32,
                                distance_meters: distance,
                                confidence: 0.3 + (activity_count as f64 * 0.2),
                                scale: preset.name,
                            };
                            (None, Some(potential))
                        }
                    }
                }
            } else {
                (None, None)
            };
            // One tick per cluster so the bar moves continuously through
            // the dominant ~18s assembly phase.
            progress_ref.on_progress();
            result
        };

        #[cfg(feature = "parallel")]
        let cluster_outputs: Vec<ClusterResult> = bucketed
            .into_par_iter()
            .enumerate()
            .map(process_one)
            .collect();
        #[cfg(not(feature = "parallel"))]
        let cluster_outputs: Vec<ClusterResult> =
            bucketed.into_iter().enumerate().map(process_one).collect();

        for (section_opt, potential_opt) in cluster_outputs {
            if let Some(section) = section_opt {
                let scale_key = section
                    .scale
                    .map(|s| s.as_str().to_string())
                    .unwrap_or_else(|| "unknown".to_string());
                stats
                    .sections_by_scale
                    .entry(scale_key)
                    .and_modify(|n| *n += 1)
                    .or_insert(1);
                all_sections.push(section);
            }
            if let Some(potential) = potential_opt {
                let scale_key = potential.scale.as_str().to_string();
                stats
                    .potentials_by_scale
                    .entry(scale_key)
                    .and_modify(|n| *n += 1)
                    .or_insert(1);
                all_potentials.push(potential);
            }
        }
    }

    // Phase 3: Post-processing (6 steps)
    progress.on_phase(DetectionPhase::Postprocessing, 6);

    // Apply post-processing
    let fold_start = web_time::Instant::now();
    let fold_sections = split_folding_sections(all_sections, config);
    progress.on_progress();
    info!(
        "[MultiScale] After fold splitting: {} sections in {}ms",
        fold_sections.len(),
        fold_start.elapsed().as_millis()
    );

    // Split at heading inflection points
    let heading_start = web_time::Instant::now();
    let heading_sections = split_at_heading_changes(fold_sections, config);
    progress.on_progress();
    info!(
        "[MultiScale] After heading splitting: {} sections in {}ms",
        heading_sections.len(),
        heading_start.elapsed().as_millis()
    );

    // Split at gradient changes (if elevation available)
    let gradient_start = web_time::Instant::now();
    let gradient_sections = split_at_gradient_changes(heading_sections, config);
    progress.on_progress();
    info!(
        "[MultiScale] After gradient splitting: {} sections in {}ms",
        gradient_sections.len(),
        gradient_start.elapsed().as_millis()
    );

    let merge_start = web_time::Instant::now();
    let merged_sections = merge_nearby_sections(gradient_sections, config);
    progress.on_progress();
    info!(
        "[MultiScale] After nearby merge: {} sections in {}ms",
        merged_sections.len(),
        merge_start.elapsed().as_millis()
    );

    // Use hierarchical deduplication if preserve_hierarchy is true
    let dedup_start = web_time::Instant::now();
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

    let split_start = web_time::Instant::now();
    let final_sections = if config.enable_density_splits {
        split_high_variance_sections(deduped_sections, &track_map, config)
    } else {
        deduped_sections
    };
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
    sorted_sections.sort_by_key(|s| std::cmp::Reverse(s.visit_count));

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
