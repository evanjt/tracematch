//! FFI bindings for mobile platforms (iOS/Android).
//!
//! This module provides the UniFFI bindings that expose Rust functionality
//! to Kotlin and Swift. All FFI functions are prefixed with `ffi_` to avoid
//! naming conflicts with the internal API.

use crate::{
    compare_routes, init_logging, GpsPoint, MatchConfig, MatchResult, RouteGroup, RouteSignature,
};
use log::{debug, info};

#[cfg(feature = "parallel")]
use crate::grouping::{group_incremental, group_signatures_parallel};

#[cfg(not(feature = "parallel"))]
use crate::group_signatures;

// ============================================================================
// Core Route Functions
// ============================================================================

/// Create a route signature from GPS points.
#[uniffi::export]
pub fn create_signature(activity_id: String, points: Vec<GpsPoint>) -> Option<RouteSignature> {
    init_logging();
    info!(
        "[RouteMatcherRust] create_signature called for {} with {} points",
        activity_id,
        points.len()
    );
    let result = RouteSignature::from_points(&activity_id, &points, &MatchConfig::default());
    if let Some(ref sig) = result {
        info!(
            "[RouteMatcherRust] Created signature: {} points, {:.0}m distance",
            sig.points.len(),
            sig.total_distance
        );
    }
    result
}

/// Create a route signature with custom configuration.
#[uniffi::export]
pub fn create_signature_with_config(
    activity_id: String,
    points: Vec<GpsPoint>,
    config: MatchConfig,
) -> Option<RouteSignature> {
    init_logging();
    info!(
        "[RouteMatcherRust] create_signature_with_config for {} ({} points)",
        activity_id,
        points.len()
    );
    RouteSignature::from_points(&activity_id, &points, &config)
}

/// Compare two routes and return match result.
#[uniffi::export]
pub fn ffi_compare_routes(
    sig1: &RouteSignature,
    sig2: &RouteSignature,
    config: MatchConfig,
) -> Option<MatchResult> {
    init_logging();
    debug!(
        "[RouteMatcherRust] Comparing {} vs {}",
        sig1.activity_id, sig2.activity_id
    );
    let result = compare_routes(sig1, sig2, &config);
    if let Some(ref r) = result {
        info!(
            "[RouteMatcherRust] Match found: {:.1}% ({})",
            r.match_percentage, r.direction
        );
    }
    result
}

/// Group signatures into route groups.
#[uniffi::export]
pub fn ffi_group_signatures(
    signatures: Vec<RouteSignature>,
    config: MatchConfig,
) -> Vec<RouteGroup> {
    init_logging();
    info!(
        "[RouteMatcherRust] RUST groupSignatures called with {} signatures",
        signatures.len()
    );

    let start = std::time::Instant::now();

    #[cfg(feature = "parallel")]
    let groups = {
        info!("[RouteMatcherRust] Using PARALLEL processing (rayon)");
        group_signatures_parallel(&signatures, &config)
    };

    #[cfg(not(feature = "parallel"))]
    let groups = {
        info!("[RouteMatcherRust] Using sequential processing");
        group_signatures(&signatures, &config)
    };

    let elapsed = start.elapsed();
    info!(
        "[RouteMatcherRust] Grouped into {} groups in {:?}",
        groups.len(),
        elapsed
    );

    groups
}

/// Incremental grouping: efficiently add new signatures to existing groups.
/// Only compares new vs existing and new vs new - O(n×m) instead of O(n²).
#[uniffi::export]
pub fn ffi_group_incremental(
    new_signatures: Vec<RouteSignature>,
    existing_groups: Vec<RouteGroup>,
    existing_signatures: Vec<RouteSignature>,
    config: MatchConfig,
) -> Vec<RouteGroup> {
    init_logging();
    info!(
        "[RouteMatcherRust] INCREMENTAL grouping: {} new + {} existing signatures",
        new_signatures.len(),
        existing_signatures.len()
    );

    let start = std::time::Instant::now();

    #[cfg(feature = "parallel")]
    let groups = group_incremental(
        &new_signatures,
        &existing_groups,
        &existing_signatures,
        &config,
    );

    #[cfg(not(feature = "parallel"))]
    let groups = {
        // Fallback to full re-grouping if parallel feature not enabled
        let all_sigs: Vec<RouteSignature> = existing_signatures
            .into_iter()
            .chain(new_signatures.into_iter())
            .collect();
        group_signatures(&all_sigs, &config)
    };

    let elapsed = start.elapsed();
    info!(
        "[RouteMatcherRust] Incremental grouped into {} groups in {:?}",
        groups.len(),
        elapsed
    );

    groups
}

/// Get default configuration.
#[uniffi::export]
pub fn default_config() -> MatchConfig {
    init_logging();
    info!("[RouteMatcherRust] default_config called - Rust is active!");
    MatchConfig::default()
}

// ============================================================================
// Flat Buffer Processing (optimized for TypedArray input)
// ============================================================================

/// Input for flat buffer processing (zero-copy from JS TypedArray)
#[derive(Debug, Clone, uniffi::Record)]
pub struct FlatGpsTrack {
    pub activity_id: String,
    /// Flat array of coordinates: [lat1, lng1, lat2, lng2, ...]
    pub coords: Vec<f64>,
}

/// Create signatures from flat coordinate buffers (optimized for TypedArray input).
/// Each track's coords array contains [lat1, lng1, lat2, lng2, ...].
/// This avoids the overhead of deserializing GpsPoint objects.
#[uniffi::export]
pub fn create_signatures_from_flat(
    tracks: Vec<FlatGpsTrack>,
    config: MatchConfig,
) -> Vec<RouteSignature> {
    init_logging();
    info!(
        "[RouteMatcherRust] FLAT BUFFER createSignatures called with {} tracks",
        tracks.len()
    );

    let start = std::time::Instant::now();

    #[cfg(feature = "parallel")]
    let signatures: Vec<RouteSignature> = {
        use rayon::prelude::*;
        info!("[RouteMatcherRust] Using PARALLEL flat buffer processing (rayon)");
        tracks
            .par_iter()
            .filter_map(|track| {
                // Convert flat coords to GpsPoints
                let points: Vec<GpsPoint> = track
                    .coords
                    .chunks_exact(2)
                    .map(|chunk| GpsPoint::new(chunk[0], chunk[1]))
                    .collect();
                RouteSignature::from_points(&track.activity_id, &points, &config)
            })
            .collect()
    };

    #[cfg(not(feature = "parallel"))]
    let signatures: Vec<RouteSignature> = {
        info!("[RouteMatcherRust] Using sequential flat buffer processing");
        tracks
            .iter()
            .filter_map(|track| {
                let points: Vec<GpsPoint> = track
                    .coords
                    .chunks_exact(2)
                    .map(|chunk| GpsPoint::new(chunk[0], chunk[1]))
                    .collect();
                RouteSignature::from_points(&track.activity_id, &points, &config)
            })
            .collect()
    };

    let elapsed = start.elapsed();
    info!(
        "[RouteMatcherRust] FLAT created {} signatures from {} tracks in {:?}",
        signatures.len(),
        tracks.len(),
        elapsed
    );

    signatures
}

/// Process routes end-to-end from flat buffers: create signatures AND group them.
/// Most efficient way to process many activities from TypedArray input.
#[uniffi::export]
pub fn process_routes_from_flat(tracks: Vec<FlatGpsTrack>, config: MatchConfig) -> Vec<RouteGroup> {
    init_logging();
    info!(
        "[RouteMatcherRust] FLAT BATCH process_routes called with {} tracks",
        tracks.len()
    );

    let start = std::time::Instant::now();

    // Step 1: Create all signatures from flat buffers
    let signatures = create_signatures_from_flat(tracks.clone(), config.clone());

    // Step 2: Group signatures
    #[cfg(feature = "parallel")]
    let groups = group_signatures_parallel(&signatures, &config);

    #[cfg(not(feature = "parallel"))]
    let groups = group_signatures(&signatures, &config);

    let elapsed = start.elapsed();
    info!(
        "[RouteMatcherRust] FLAT batch processing: {} signatures -> {} groups in {:?}",
        signatures.len(),
        groups.len(),
        elapsed
    );

    groups
}


// ============================================================================
// Frequent Sections Detection
// ============================================================================

/// Input mapping activity IDs to sport types
#[derive(Debug, Clone, uniffi::Record)]
pub struct ActivitySportType {
    pub activity_id: String,
    pub sport_type: String,
}


/// Get conservative section config (fewer sections, higher confidence)
#[uniffi::export]
pub fn conservative_section_config() -> crate::SectionConfig {
    crate::SectionConfig::conservative()
}

/// Get legacy section config (backward compatible single-scale)
#[uniffi::export]
pub fn legacy_section_config() -> crate::SectionConfig {
    crate::SectionConfig::legacy()
}

/// Get default scale presets for multi-scale detection
#[uniffi::export]
pub fn default_scale_presets() -> Vec<crate::ScalePreset> {
    crate::ScalePreset::default_presets()
}

/// Detect sections at multiple scales with potential section suggestions.
/// This is the flagship entry point for section detection.
///
/// Returns a MultiScaleSectionResult with:
/// - sections: Confirmed sections (meeting min_activities threshold)
/// - potentials: Suggested sections from 1-2 activity overlaps
/// - stats: Detection statistics
#[uniffi::export]
pub fn ffi_detect_sections_multiscale(
    activity_ids: Vec<String>,
    all_coords: Vec<f64>,
    offsets: Vec<u32>,
    sport_types: Vec<ActivitySportType>,
    groups: Vec<RouteGroup>,
    config: crate::SectionConfig,
) -> crate::MultiScaleSectionResult {
    init_logging();
    info!(
        "[RouteMatcherRust] detect_sections_multiscale: {} activities, {} coords, {} scales",
        activity_ids.len(),
        all_coords.len() / 2,
        config.scale_presets.len()
    );

    let start = std::time::Instant::now();

    // Convert flat coordinates to tracks
    let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::with_capacity(activity_ids.len());

    for (i, activity_id) in activity_ids.iter().enumerate() {
        let start_offset = offsets[i] as usize;
        let end_offset = offsets
            .get(i + 1)
            .map(|&o| o as usize)
            .unwrap_or(all_coords.len() / 2);

        let mut points = Vec::with_capacity(end_offset - start_offset);
        for j in start_offset..end_offset {
            let coord_idx = j * 2;
            if coord_idx + 1 < all_coords.len() {
                points.push(GpsPoint::new(
                    all_coords[coord_idx],
                    all_coords[coord_idx + 1],
                ));
            }
        }

        if !points.is_empty() {
            tracks.push((activity_id.clone(), points));
        }
    }

    info!(
        "[RouteMatcherRust] Converted to {} tracks with full GPS data",
        tracks.len()
    );

    // Convert sport types to HashMap
    let sport_map: std::collections::HashMap<String, String> = sport_types
        .into_iter()
        .map(|st| (st.activity_id, st.sport_type))
        .collect();

    let result = crate::sections::detect_sections_multiscale(&tracks, &sport_map, &groups, &config);

    let elapsed = start.elapsed();
    info!(
        "[RouteMatcherRust] Multi-scale detection: {} sections, {} potentials in {:?}",
        result.sections.len(),
        result.potentials.len(),
        elapsed
    );

    result
}

// ============================================================================
// Heatmap Generation FFI
// ============================================================================

/// Generate a heatmap from route signatures.
/// Uses the simplified GPS traces (~100 points each) for efficient generation.
#[uniffi::export]
pub fn ffi_generate_heatmap(
    signatures: Vec<RouteSignature>,
    activity_data: Vec<crate::ActivityHeatmapData>,
    config: crate::HeatmapConfig,
) -> crate::HeatmapResult {
    init_logging();
    info!(
        "[RouteMatcherRust] generate_heatmap: {} signatures, {}m cells",
        signatures.len(),
        config.cell_size_meters
    );

    let start = std::time::Instant::now();

    // Convert Vec to HashMap for efficient lookup
    let data_map: std::collections::HashMap<String, crate::ActivityHeatmapData> = activity_data
        .into_iter()
        .map(|d| (d.activity_id.clone(), d))
        .collect();

    let result = crate::generate_heatmap(&signatures, &data_map, &config);

    let elapsed = start.elapsed();
    info!(
        "[RouteMatcherRust] Heatmap generated: {} cells, {} routes, {} activities in {:?}",
        result.cells.len(),
        result.total_routes,
        result.total_activities,
        elapsed
    );

    result
}

/// Query the heatmap at a specific location.
#[uniffi::export]
pub fn ffi_query_heatmap_cell(
    heatmap: crate::HeatmapResult,
    lat: f64,
    lng: f64,
) -> Option<crate::CellQueryResult> {
    crate::query_heatmap_cell(&heatmap, lat, lng, heatmap.cell_size_meters)
}

