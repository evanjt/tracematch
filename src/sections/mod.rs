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
mod grid;
mod identity;
pub mod interestingness;
mod medoid;
mod naming;
pub mod optimized;
mod overlap;
mod portions;
pub mod progress;
mod rtree;
mod traces;
mod unified;

use crate::GpsPoint;
use crate::matching::calculate_route_distance;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

pub use progress::{
    AtomicProgressTracker, DetectionPhase, DetectionProgressCallback, NoopProgress,
};

// Re-export internal utilities for use across submodules
pub(crate) use consensus::compute_consensus_polyline;
pub use consensus::{
    ConsensusAccumulator, ConsensusPointAccumulator, ConsensusResult, TraceKey, TraceRTreeCache,
    build_accumulator_from_traces, build_trace_rtree_cache, merge_traces_into_consensus,
    merge_traces_into_consensus_with_cache,
};
pub use interestingness::{
    Candidate as RankCandidate, Enrichment, Member as RankMember, Outing as RankOuting,
    RankFeatures, SectionClass, Traversal as RankTraversal, enrich, max_sustained_grade, rank,
};
pub(crate) use medoid::{compute_stability, select_medoid};
pub use overlap::{FullTrackOverlap, OverlapCluster};
pub(crate) use portions::compute_activity_portions;
pub use portions::{
    find_all_track_portions, find_all_track_portions_with_gap, line_match_cell_m, track_portions,
};
pub use rtree::{IndexedPoint, build_rtree};
pub use traces::{
    extract_activity_passes, extract_activity_trace, extract_all_activity_traces,
    longest_pass_per_activity,
};
// Assign-once identity + hysteresis: pure decision layer over the churny
// batch catalogue. tracematch decides which candidate inherits which prior id;
// the engine owns the id strings and the persistence.
pub use identity::{
    ANCHOR_CELL_M, CARRY_COVERAGE, CandidateFate, CandidateResolution, CandidateSection, DEFAULT_K,
    DISSOLVE_PRESSURE_HI, Decision, GROUND_TOL_M, HysteresisParams, HysteresisState,
    IdentityParams, IdentityPlan, PriorSection, RECUT_AGREEMENT, RetireReason, Retirement,
    StepOutcome, dissolve_pressure, earth_cell, mutual_overlap, plan_identity, plan_identity_tuned,
    section_heart, shares_ground,
};
// Named-corridor resolution scoring: the pure layer decides
// which visible section carries a name; the engine owns intent storage and
// SQL.
pub use naming::{
    CORE_FLOOR_M, CORE_TRIM_FRAC, COVERAGE_TIE, DIRECTION_TOL_DEG, NamedCandidate, NamedScore,
    OFFSET_CEILING_M, PART_FLOOR, coverage_and_offset, score_named_candidate, select_candidate,
    split_direction, trim_core,
};

pub use unified::{
    BoundaryReason, BoundaryRecord, SectionEvidenceCache, SectionGeometryChange, SectionMergedAway,
    SectionUpdatePolicy, Tunables, UnifiedDetection, UnifiedIncrementalResult,
    confirmed_lift_spans, confirmed_lift_spans_tuned, detect_sections_unified,
    detect_sections_unified_dated, detect_sections_unified_explained,
    detect_sections_unified_incremental, detect_sections_unified_incremental_cached,
    detect_sections_unified_incremental_cached_with_policy,
    detect_sections_unified_incremental_dated, detect_sections_unified_incremental_observed,
    detect_sections_unified_tuned, lift_spans, lift_spans_tuned, self_pass_penalty,
};

// Re-export single-route section utilities (find known sections).
pub use optimized::{
    SectionMatch, find_all_section_spans_in_route, find_sections_in_route,
    recalculate_section_polyline,
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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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
    /// Retained for stored configs. Two adjacent hot cells merge
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
    /// separate sections, useful for activity-based density input,
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
    /// Retained for stored configs. Minimum cell visits for a cell to count
    /// as road network. Default 5.
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
    /// Detect over one pool rather than one per sport, so a road two
    /// sports share carries the traversals of both. Unified only. The
    /// sport label is derived from the traversals after the cut.
    #[serde(default = "default_pool_sports")]
    pub pool_sports: bool,
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
    2
}
fn default_pool_sports() -> bool {
    true
}

impl Default for SectionConfig {
    fn default() -> Self {
        Self {
            // Tuned for consumer fitness data: GPS noise is higher than research-grade tracks
            // and per-user activity counts are sparse, so defaults are deliberately permissive.
            // Override via SectionConfig { ..Default::default() } or conservative()/discovery()
            // for stricter scenarios.
            proximity_threshold: 200.0,
            min_section_length: 150.0,
            max_section_length: 200_000.0,
            min_activities: 2,
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
            pool_sports: default_pool_sports(),
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
    /// Half-open range into the representative's track that `polyline` was
    /// sliced from. `None` for an averaged line, which is a slice of nothing.
    #[serde(default, alias = "representative_range")]
    pub representative_range: Option<(u32, u32)>,
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

    /// Elevation gain (m) over the representative slice, hysteresis
    /// accumulated; None when the slice lacks elevation coverage.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub elevation_gain_m: Option<f64>,

    /// Net grade (%) over the representative slice; None with
    /// `elevation_gain_m`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub avg_grade_percent: Option<f64>,

    /// Profile and shape off the emitted slice: loss, steepest sustained
    /// grade, straightness, class and the lift flag. The gain and net
    /// grade above are the same numbers, kept where older readers look.
    #[serde(default)]
    pub enrichment: interestingness::Enrichment,

    /// Ranking features and score within the catalogue this section was
    /// last ranked in; None until a rank has run.
    #[serde(default)]
    pub rank: Option<interestingness::RankFeatures>,

    /// Number of times this section has been recalibrated
    #[serde(default = "default_version")]
    pub version: u32,

    /// ISO timestamp of last recalibration (reference change or consensus update)
    #[serde(alias = "updated_at")]
    pub updated_at: Option<String>,

    /// ISO timestamp when section was created
    #[serde(alias = "created_at")]
    pub created_at: Option<String>,

    /// Incremental-consensus running sums. None means "not yet built" -
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
    /// Recalculate this section's polyline from stored activity traces.
    ///
    /// Convenience wrapper around [`recalculate_section_polyline`].
    pub fn recalculate_polyline(&self, config: &SectionConfig) -> FrequentSection {
        optimized::recalculate_section_polyline(self, config)
    }
}

/// The label of a section no activity in `sport_types` traverses.
pub(crate) const UNKNOWN_SPORT: &str = "Unknown";

/// Consecutive off-line points a trace tolerates before it breaks: GPS
/// noise, not a departure. Shared by every track-against-line scan so the
/// unit (points, never metres) cannot drift between them.
pub const TRACK_GAP_POINTS: usize = 3;

/// Pick the most-common sport among the given activities.
///
/// Sections can contain activities from multiple sports (e.g., a road
/// traversed by both cyclists and runners). The section's `sport_type`
/// field carries the dominant one for display; the UI reads
/// `activity_ids` to render per-sport filter chips.
pub fn dominant_sport(activity_ids: &[String], sport_types: &HashMap<String, String>) -> String {
    let mut counts: HashMap<&str, u32> = HashMap::new();
    for id in activity_ids {
        if let Some(s) = sport_types.get(id) {
            *counts.entry(s.as_str()).or_insert(0) += 1;
        }
    }
    // Ties resolve on the sport name, so the label never depends on map order.
    counts
        .into_iter()
        .max_by(|a, b| a.1.cmp(&b.1).then_with(|| b.0.cmp(a.0)))
        .map(|(s, _)| s.to_string())
        .unwrap_or_else(|| UNKNOWN_SPORT.to_string())
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
    let activity_portions = compute_activity_portions(&representative_polyline, track_map, config);

    // Collect route IDs. Sorted after the dedupe, like `activity_id_vec` below:
    // the set drains in hash order and this list is stored on the section.
    let route_ids: Vec<String> = {
        let mut ids: Vec<String> = cluster
            .activity_ids
            .iter()
            .filter_map(|aid| activity_to_route.get(aid.as_str()).map(|s| s.to_string()))
            .collect::<HashSet<_>>()
            .into_iter()
            .collect();
        ids.sort();
        ids
    };

    // Extract traces for consensus computation only (not stored in section).
    // Sort for stable iteration order, HashSet iteration is randomized
    // and the consensus computation is order-sensitive.
    let activity_id_vec: Vec<String> = {
        let mut v: Vec<String> = cluster.activity_ids.iter().cloned().collect();
        v.sort();
        v
    };
    let all_traces: Vec<Vec<GpsPoint>> =
        extract_all_activity_traces(&activity_id_vec, &representative_polyline, track_map)
            .into_iter()
            .map(|(_, trace)| trace)
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

    // A visit is a pass over the ground, not an outing. `activity_portions`
    // holds every traversal and is what the junction table stores. Support
    // is judged separately, on outings and calendar days.
    let visit_count = if activity_portions.is_empty() {
        activity_count as u32
    } else {
        activity_portions.len() as u32
    };

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
        representative_range: None,
        // The section's POPULATION, from discovery. Rule 8 leans on it:
        // two lines on the same ground are duplicates only when the same
        // users travel both, so the other bank of a river stays its own
        // corridor. Traversals of the line live in `activity_portions` and
        // are a wider set.
        activity_ids: {
            let mut ids: Vec<String> = cluster.activity_ids.into_iter().collect();
            ids.sort();
            ids
        },
        activity_portions,
        route_ids,
        visit_count,
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
        elevation_gain_m: None,
        avg_grade_percent: None,
        version: 1,
        updated_at: None,
        created_at: None,
        enrichment: Default::default(),
        rank: None,
        consensus_state: None,
    })
}

#[cfg(test)]
mod dominant_sport_tests {
    use super::*;

    fn sport_map(pairs: &[(&str, &str)]) -> HashMap<String, String> {
        pairs
            .iter()
            .map(|(id, sport)| (id.to_string(), sport.to_string()))
            .collect()
    }

    #[test]
    fn two_way_tie_resolves_on_sport_name_regardless_of_input_order() {
        let sport_types =
            sport_map(&[("a1", "Run"), ("a2", "Ride"), ("a3", "Run"), ("a4", "Ride")]);

        let orderings = [
            ["a1", "a2", "a3", "a4"],
            ["a2", "a1", "a4", "a3"],
            ["a4", "a3", "a2", "a1"],
            ["a3", "a1", "a2", "a4"],
        ];

        for order in orderings {
            let ids: Vec<String> = order.iter().map(|s| s.to_string()).collect();
            assert_eq!(dominant_sport(&ids, &sport_types), "Ride");
        }
    }

    /// Every sport ties on count, so a map-order winner would land on the
    /// alphabetically-first name only by chance, in each of these sets.
    #[test]
    fn wide_ties_always_resolve_on_the_first_sport_name() {
        let sport_sets: [&[&str]; 4] = [
            &[
                "AlpineSki",
                "EBikeRide",
                "Hike",
                "Ride",
                "Rowing",
                "Run",
                "Swim",
                "VirtualRide",
                "Walk",
                "Workout",
            ],
            &[
                "Badminton",
                "Canoeing",
                "Elliptical",
                "GravelRide",
                "IceSkate",
                "Kayaking",
                "MountainBikeRide",
                "NordicSki",
                "Snowboard",
                "TrailRun",
            ],
            &[
                "Handcycle",
                "InlineSkate",
                "Kitesurf",
                "RockClimbing",
                "Sail",
                "Skateboard",
                "Snowshoe",
                "StandUpPaddling",
                "Surfing",
                "Velomobile",
            ],
            &[
                "Crossfit",
                "Golf",
                "Pilates",
                "Racquetball",
                "Soccer",
                "Squash",
                "TableTennis",
                "Tennis",
                "WeightTraining",
                "Yoga",
            ],
        ];

        for sports in sport_sets {
            let mut sorted: Vec<&str> = sports.to_vec();
            sorted.sort_unstable();
            let expected = sorted[0].to_string();

            let mut sport_types = HashMap::new();
            let mut ids = Vec::new();
            for (i, sport) in sports.iter().enumerate() {
                for visit in 0..2 {
                    let id = format!("a{i}-{visit}");
                    sport_types.insert(id.clone(), sport.to_string());
                    ids.push(id);
                }
            }

            assert_eq!(dominant_sport(&ids, &sport_types), expected);
            ids.reverse();
            assert_eq!(dominant_sport(&ids, &sport_types), expected);
        }
    }

    #[test]
    fn majority_sport_wins() {
        let sport_types = sport_map(&[("a1", "Ride"), ("a2", "Run"), ("a3", "Run")]);
        let ids: Vec<String> = ["a1", "a2", "a3"].iter().map(|s| s.to_string()).collect();
        assert_eq!(dominant_sport(&ids, &sport_types), "Run");
    }

    #[test]
    fn unknown_when_no_sports_are_known() {
        assert_eq!(dominant_sport(&[], &HashMap::new()), "Unknown");
    }
}
