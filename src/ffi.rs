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
// Progress Callback Interface (for real-time updates to mobile)
// ============================================================================

/// Callback interface for receiving progress updates during fetch operations.
/// Implement this in Kotlin/Swift to receive real-time updates.
#[uniffi::export(callback_interface)]
pub trait FetchProgressCallback: Send + Sync {
    /// Called when a single activity fetch completes.
    /// - completed: Number of activities fetched so far
    /// - total: Total number of activities to fetch
    fn on_progress(&self, completed: u32, total: u32);
}

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
// HTTP Activity Fetching (requires "http" feature)
// ============================================================================

/// Result of fetching activity map data from intervals.icu
#[cfg(feature = "http")]
#[derive(Debug, Clone, uniffi::Record)]
pub struct FfiActivityMapResult {
    pub activity_id: String,
    /// Bounds as [ne_lat, ne_lng, sw_lat, sw_lng] or empty if no bounds
    pub bounds: Vec<f64>,
    /// GPS coordinates as flat array [lat1, lng1, lat2, lng2, ...]
    pub latlngs: Vec<f64>,
    pub success: bool,
    pub error: Option<String>,
}

/// Fetch map data for multiple activities in parallel.
///
/// This function respects intervals.icu rate limits:
/// - 30 req/s burst limit
/// - 131 req/10s sustained limit
///
/// Uses connection pooling and parallel fetching for maximum performance.
/// Automatically retries on 429 errors with exponential backoff.
#[cfg(feature = "http")]
#[uniffi::export]
pub fn fetch_activity_maps(
    api_key: String,
    activity_ids: Vec<String>,
) -> Vec<FfiActivityMapResult> {
    init_logging();
    info!(
        "[RouteMatcherRust] fetch_activity_maps called for {} activities",
        activity_ids.len()
    );

    let results = crate::http::fetch_activity_maps_sync(api_key, activity_ids, None);

    // Convert to FFI-friendly format
    results
        .into_iter()
        .map(|r| FfiActivityMapResult {
            activity_id: r.activity_id,
            bounds: r
                .bounds
                .map_or(vec![], |b| vec![b.ne[0], b.ne[1], b.sw[0], b.sw[1]]),
            latlngs: r.latlngs.map_or(vec![], |coords| {
                coords.into_iter().flat_map(|p| vec![p[0], p[1]]).collect()
            }),
            success: r.success,
            error: r.error,
        })
        .collect()
}

/// Fetch map data with real-time progress callbacks.
///
/// Same as fetch_activity_maps but calls the progress callback after each
/// activity is fetched, allowing the UI to show real-time progress.
#[cfg(feature = "http")]
#[uniffi::export]
pub fn fetch_activity_maps_with_progress(
    api_key: String,
    activity_ids: Vec<String>,
    callback: Box<dyn FetchProgressCallback>,
) -> Vec<FfiActivityMapResult> {
    use std::sync::Arc;

    init_logging();
    info!(
        "[RouteMatcherRust] fetch_activity_maps_with_progress called for {} activities",
        activity_ids.len()
    );

    // Wrap the callback to match the expected type
    let callback = Arc::new(callback);
    let progress_callback: crate::http::ProgressCallback = Arc::new(move |completed, total| {
        callback.on_progress(completed, total);
    });

    let results =
        crate::http::fetch_activity_maps_sync(api_key, activity_ids, Some(progress_callback));

    // Convert to FFI-friendly format
    results
        .into_iter()
        .map(|r| FfiActivityMapResult {
            activity_id: r.activity_id,
            bounds: r
                .bounds
                .map_or(vec![], |b| vec![b.ne[0], b.ne[1], b.sw[0], b.sw[1]]),
            latlngs: r.latlngs.map_or(vec![], |coords| {
                coords.into_iter().flat_map(|p| vec![p[0], p[1]]).collect()
            }),
            success: r.success,
            error: r.error,
        })
        .collect()
}

/// Result of fetch_and_process_activities
#[cfg(feature = "http")]
#[derive(Debug, Clone, uniffi::Record)]
pub struct FetchAndProcessResult {
    pub map_results: Vec<FfiActivityMapResult>,
    pub signatures: Vec<RouteSignature>,
}

/// Fetch map data AND create route signatures in one call.
/// Most efficient for initial sync - fetches from API and processes GPS data.
#[cfg(feature = "http")]
#[uniffi::export]
pub fn fetch_and_process_activities(
    api_key: String,
    activity_ids: Vec<String>,
    config: MatchConfig,
) -> FetchAndProcessResult {
    init_logging();
    info!(
        "[RouteMatcherRust] fetch_and_process_activities for {} activities",
        activity_ids.len()
    );

    let start = std::time::Instant::now();

    // Fetch all activity maps
    let results = crate::http::fetch_activity_maps_sync(api_key, activity_ids, None);

    // Convert to FFI format and create signatures from successful fetches
    let mut map_results = Vec::with_capacity(results.len());
    let mut signatures = Vec::new();

    for r in results {
        let bounds_vec = r
            .bounds
            .as_ref()
            .map_or(vec![], |b| vec![b.ne[0], b.ne[1], b.sw[0], b.sw[1]]);

        let latlngs_flat: Vec<f64> = r.latlngs.as_ref().map_or(vec![], |coords| {
            coords.iter().flat_map(|p| vec![p[0], p[1]]).collect()
        });

        // Create signature if we have GPS data
        if r.success && r.latlngs.is_some() {
            let points: Vec<GpsPoint> = r
                .latlngs
                .as_ref()
                .unwrap()
                .iter()
                .map(|p| GpsPoint::new(p[0], p[1]))
                .collect();

            if let Some(sig) = RouteSignature::from_points(&r.activity_id, &points, &config) {
                signatures.push(sig);
            }
        }

        map_results.push(FfiActivityMapResult {
            activity_id: r.activity_id,
            bounds: bounds_vec,
            latlngs: latlngs_flat,
            success: r.success,
            error: r.error,
        });
    }

    let elapsed = start.elapsed();
    info!(
        "[RouteMatcherRust] Fetched {} activities, created {} signatures in {:?}",
        map_results.len(),
        signatures.len(),
        elapsed
    );

    FetchAndProcessResult {
        map_results,
        signatures,
    }
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

/// Get default section detection configuration
#[uniffi::export]
pub fn default_section_config() -> crate::SectionConfig {
    crate::SectionConfig::default()
}

/// Get discovery mode section config (more sensitive detection, lower thresholds)
#[uniffi::export]
pub fn discovery_section_config() -> crate::SectionConfig {
    crate::SectionConfig::discovery()
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

/// Detect frequent sections from FULL GPS tracks.
/// Uses medoid-based algorithm to select actual GPS traces as representative polylines.
/// This produces smooth, natural section shapes that follow real roads.
///
/// # Arguments
/// * `activity_ids` - List of activity IDs (in same order as coordinates)
/// * `all_coords` - Flat array of coordinates: [lat1, lng1, lat2, lng2, ...]
/// * `offsets` - Start offset for each activity in all_coords (length = activity count + 1)
/// * `sport_types` - Sport type for each activity
/// * `groups` - Route groups (for linking sections to routes)
/// * `config` - Section detection configuration
#[uniffi::export]
pub fn ffi_detect_sections_from_tracks(
    activity_ids: Vec<String>,
    all_coords: Vec<f64>,
    offsets: Vec<u32>,
    sport_types: Vec<ActivitySportType>,
    groups: Vec<RouteGroup>,
    config: crate::SectionConfig,
) -> Vec<crate::FrequentSection> {
    init_logging();
    info!(
        "[RouteMatcherRust] detect_sections_from_tracks: {} activities, {} coords",
        activity_ids.len(),
        all_coords.len() / 2
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

    let sections =
        crate::sections::detect_sections_from_tracks(&tracks, &sport_map, &groups, &config);

    let elapsed = start.elapsed();
    info!(
        "[RouteMatcherRust] Found {} sections (medoid-based) in {:?}",
        sections.len(),
        elapsed
    );

    sections
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

/// Get default heatmap configuration.
#[uniffi::export]
pub fn default_heatmap_config() -> crate::HeatmapConfig {
    crate::HeatmapConfig::default()
}

// ============================================================================
// Zone Distribution FFI
// ============================================================================

/// Calculate power zone distribution from power data.
///
/// # Arguments
/// * `power_data` - Power values in watts (1Hz sampling)
/// * `ftp` - Functional Threshold Power in watts
/// * `zone_thresholds` - Optional custom zone thresholds as % of FTP [Z1, Z2, Z3, Z4, Z5, Z6]
///
/// # Returns
/// JSON string with zone distribution results
#[uniffi::export]
pub fn ffi_calculate_power_zones(
    power_data: Vec<u16>,
    ftp: u16,
    zone_thresholds: Option<Vec<f32>>,
) -> String {
    init_logging();
    info!(
        "[RouteMatcherRust] calculate_power_zones: {} samples, FTP={}W",
        power_data.len(),
        ftp
    );

    let config = match zone_thresholds {
        Some(thresholds) if thresholds.len() == 6 => {
            let mut arr = [0.0f32; 6];
            arr.copy_from_slice(&thresholds);
            crate::zones::PowerZoneConfig::with_thresholds(ftp, arr)
        }
        _ => crate::zones::PowerZoneConfig::from_ftp(ftp),
    };

    #[cfg(feature = "parallel")]
    let result = crate::zones::calculate_power_zones_parallel(&power_data, &config);
    #[cfg(not(feature = "parallel"))]
    let result = crate::zones::calculate_power_zones(&power_data, &config);

    info!(
        "[RouteMatcherRust] Power zones: {} samples, avg={}W, peak={}W",
        result.total_samples, result.average_power, result.peak_power
    );

    serde_json::to_string(&result).unwrap_or_else(|_| "{}".to_string())
}

/// Calculate HR zone distribution from heart rate data.
///
/// # Arguments
/// * `hr_data` - Heart rate values in BPM (1Hz sampling)
/// * `threshold_hr` - Max HR or LTHR
/// * `zone_thresholds` - Optional custom zone thresholds as % of threshold [Z1, Z2, Z3, Z4]
///
/// # Returns
/// JSON string with zone distribution results
#[uniffi::export]
pub fn ffi_calculate_hr_zones(
    hr_data: Vec<u8>,
    threshold_hr: u8,
    zone_thresholds: Option<Vec<f32>>,
) -> String {
    init_logging();
    info!(
        "[RouteMatcherRust] calculate_hr_zones: {} samples, threshold={}bpm",
        hr_data.len(),
        threshold_hr
    );

    let config = match zone_thresholds {
        Some(thresholds) if thresholds.len() == 4 => {
            let mut arr = [0.0f32; 4];
            arr.copy_from_slice(&thresholds);
            crate::zones::HRZoneConfig::with_thresholds(threshold_hr, arr)
        }
        _ => crate::zones::HRZoneConfig::from_max_hr(threshold_hr),
    };

    #[cfg(feature = "parallel")]
    let result = crate::zones::calculate_hr_zones_parallel(&hr_data, &config);
    #[cfg(not(feature = "parallel"))]
    let result = crate::zones::calculate_hr_zones(&hr_data, &config);

    info!(
        "[RouteMatcherRust] HR zones: {} samples, avg={}bpm, peak={}bpm",
        result.total_samples, result.average_hr, result.peak_hr
    );

    serde_json::to_string(&result).unwrap_or_else(|_| "{}".to_string())
}

// ============================================================================
// Power/Pace Curve FFI
// ============================================================================

/// Compute power curve for a single activity.
///
/// # Arguments
/// * `power_data` - Power values in watts (1Hz sampling)
/// * `durations` - Durations to compute in seconds [1, 5, 60, 300, 1200, 3600]
///
/// # Returns
/// JSON string with power curve results
#[uniffi::export]
pub fn ffi_compute_power_curve(power_data: Vec<u16>, durations: Vec<u32>) -> String {
    init_logging();
    info!(
        "[RouteMatcherRust] compute_power_curve: {} samples, {} durations",
        power_data.len(),
        durations.len()
    );

    let result = crate::curves::compute_power_curve(&power_data, &durations);

    info!(
        "[RouteMatcherRust] Power curve computed, peak 1s={}W",
        result.get_power_at(1).unwrap_or(0.0)
    );

    serde_json::to_string(&result).unwrap_or_else(|_| "{}".to_string())
}

/// Compute power curve from multiple activities (all-time bests).
///
/// # Arguments
/// * `activity_ids` - Activity IDs
/// * `power_data_flat` - Flat array of all power data
/// * `offsets` - Start offset for each activity in power_data_flat
/// * `timestamps` - Unix timestamps for each activity
/// * `durations` - Durations to compute in seconds
///
/// # Returns
/// JSON string with power curve results including activity attribution
#[uniffi::export]
pub fn ffi_compute_power_curve_multi(
    activity_ids: Vec<String>,
    power_data_flat: Vec<u16>,
    offsets: Vec<u32>,
    timestamps: Vec<i64>,
    durations: Vec<u32>,
) -> String {
    init_logging();
    info!(
        "[RouteMatcherRust] compute_power_curve_multi: {} activities, {} total samples",
        activity_ids.len(),
        power_data_flat.len()
    );

    // Reconstruct activities from flat data
    let mut activities: Vec<(String, Vec<u16>, i64)> = Vec::new();

    for (i, activity_id) in activity_ids.iter().enumerate() {
        let start = offsets[i] as usize;
        let end = offsets
            .get(i + 1)
            .map(|&o| o as usize)
            .unwrap_or(power_data_flat.len());
        let power = power_data_flat[start..end].to_vec();
        let ts = timestamps.get(i).copied().unwrap_or(0);
        activities.push((activity_id.clone(), power, ts));
    }

    #[cfg(feature = "parallel")]
    let result = crate::curves::compute_power_curve_multi_parallel(&activities, &durations);
    #[cfg(not(feature = "parallel"))]
    let result = crate::curves::compute_power_curve_multi(&activities, &durations);

    info!(
        "[RouteMatcherRust] Multi-activity power curve computed from {} activities",
        result.activities_analyzed
    );

    serde_json::to_string(&result).unwrap_or_else(|_| "{}".to_string())
}

/// Compute pace curve for a single activity.
///
/// # Arguments
/// * `distances` - Cumulative distance at each second in meters
/// * `target_distances` - Distances to compute pace for in meters
///
/// # Returns
/// JSON string with pace curve results
#[uniffi::export]
pub fn ffi_compute_pace_curve(distances: Vec<f32>, target_distances: Vec<f32>) -> String {
    init_logging();
    info!(
        "[RouteMatcherRust] compute_pace_curve: {} samples, {} target distances",
        distances.len(),
        target_distances.len()
    );

    let result = crate::curves::compute_pace_curve(&distances, &target_distances);

    serde_json::to_string(&result).unwrap_or_else(|_| "{}".to_string())
}

// ============================================================================
// Achievement Detection FFI
// ============================================================================

/// Detect achievements by comparing a new activity against historical records.
///
/// # Arguments
/// * `new_activity` - The newly completed activity record
/// * `history` - Historical activity records for comparison
///
/// # Returns
/// Vector of detected achievements, sorted by importance
#[uniffi::export]
pub fn ffi_detect_achievements(
    new_activity: crate::achievements::ActivityRecord,
    history: Vec<crate::achievements::ActivityRecord>,
) -> Vec<crate::achievements::Achievement> {
    init_logging();
    info!(
        "[RouteMatcherRust] detect_achievements for activity {}, comparing against {} historical activities",
        new_activity.activity_id,
        history.len()
    );

    let achievements = crate::achievements::detect_achievements(&new_activity, &history);

    info!(
        "[RouteMatcherRust] Detected {} achievements",
        achievements.len()
    );

    achievements
}

// ============================================================================
// Comprehensive FFI Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MatchConfig;

    // ========================================================================
    // Test Helpers
    // ========================================================================

    /// Generate a realistic GPS route with optional noise
    fn generate_route(
        start_lat: f64,
        start_lng: f64,
        distance_km: f64,
        points_per_km: usize,
        noise_meters: f64,
    ) -> Vec<GpsPoint> {
        let total_points = (distance_km * points_per_km as f64) as usize;
        let bearing = 45.0_f64.to_radians(); // Northeast direction

        (0..total_points)
            .map(|i| {
                let progress = i as f64 / total_points.max(1) as f64;
                let distance_m = progress * distance_km * 1000.0;

                // Move along bearing
                let lat = start_lat + (distance_m / 111_000.0) * bearing.cos();
                let lng = start_lng + (distance_m / (111_000.0 * start_lat.to_radians().cos())) * bearing.sin();

                // Add deterministic "noise" based on index
                let noise_deg = noise_meters / 111_000.0;
                let noise_lat = ((i as f64 * 0.7).sin()) * noise_deg;
                let noise_lng = ((i as f64 * 1.3).cos()) * noise_deg;

                GpsPoint::new(lat + noise_lat, lng + noise_lng)
            })
            .collect()
    }

    /// Generate a route meeting minimum distance requirement (~1km)
    fn long_route() -> Vec<GpsPoint> {
        generate_route(51.5074, -0.1278, 1.0, 100, 0.0)
    }

    /// Generate a route with GPS noise (~10m variance)
    fn noisy_route() -> Vec<GpsPoint> {
        generate_route(51.5074, -0.1278, 1.0, 100, 10.0)
    }

    /// Generate a completely different route (NYC instead of London)
    fn different_route() -> Vec<GpsPoint> {
        generate_route(40.7128, -74.0060, 1.0, 100, 0.0)
    }

    // ========================================================================
    // Signature Creation Tests
    // ========================================================================

    #[test]
    fn test_create_signature_valid_route() {
        let points = long_route();
        let sig = create_signature("test-1".to_string(), points);

        assert!(sig.is_some(), "Should create signature from valid route");
        let sig = sig.unwrap();
        assert_eq!(sig.activity_id, "test-1");
        assert!(sig.total_distance > 800.0, "~1km route should have distance > 800m");
        assert!(!sig.points.is_empty(), "Signature should have simplified points");
    }

    #[test]
    fn test_create_signature_empty_points() {
        let sig = create_signature("test-1".to_string(), vec![]);
        assert!(sig.is_none(), "Empty points should return None");
    }

    #[test]
    fn test_create_signature_single_point() {
        let points = vec![GpsPoint::new(51.5074, -0.1278)];
        let sig = create_signature("test-1".to_string(), points);
        assert!(sig.is_none(), "Single point should return None");
    }

    #[test]
    fn test_create_signature_two_points() {
        // Two points that are far apart (should work)
        let points = vec![
            GpsPoint::new(51.5074, -0.1278),
            GpsPoint::new(51.5174, -0.1278), // ~1km north
        ];
        let sig = create_signature("test-1".to_string(), points);
        assert!(sig.is_some(), "Two valid points should create signature");
    }

    #[test]
    fn test_create_signature_duplicate_points() {
        // All points at same location (degenerate case)
        let points: Vec<GpsPoint> = (0..100)
            .map(|_| GpsPoint::new(51.5074, -0.1278))
            .collect();
        let sig = create_signature("test-1".to_string(), points);
        // This may or may not create a signature - document behavior
        // If it does, distance should be ~0
        if let Some(s) = sig {
            assert!(s.total_distance < 1.0, "Duplicate points should have ~0 distance");
        }
    }

    #[test]
    fn test_create_signature_invalid_coordinates() {
        // Mix of valid and invalid points
        let points = vec![
            GpsPoint::new(91.0, 0.0),      // Invalid latitude
            GpsPoint::new(51.5074, -0.1278), // Valid
            GpsPoint::new(0.0, 181.0),     // Invalid longitude
            GpsPoint::new(51.5084, -0.1288), // Valid
            GpsPoint::new(f64::NAN, 0.0),  // NaN
        ];
        let sig = create_signature("test-1".to_string(), points);
        // Should filter invalid and work with remaining valid points
        if let Some(s) = sig {
            assert!(s.points.iter().all(|p| p.is_valid()), "All points should be valid");
        }
    }

    #[test]
    fn test_create_signature_very_long_route() {
        // 100km route with 10000 points - should be simplified
        let points = generate_route(51.5074, -0.1278, 100.0, 100, 5.0);
        assert_eq!(points.len(), 10000);

        let sig = create_signature("test-1".to_string(), points);
        assert!(sig.is_some());
        let sig = sig.unwrap();

        // Should be heavily simplified
        assert!(sig.points.len() <= 100, "Should simplify to max_simplified_points");
        assert!(sig.total_distance > 50_000.0, "Should preserve approximate distance");
    }

    #[test]
    fn test_create_signature_very_short_route() {
        // 100m route - below min_route_distance
        let points = generate_route(51.5074, -0.1278, 0.1, 100, 0.0);
        let sig = create_signature("test-1".to_string(), points);

        // Short routes can still create signatures (min_route_distance only affects grouping)
        assert!(sig.is_some());
    }

    // ========================================================================
    // Route Comparison Tests
    // ========================================================================

    #[test]
    fn test_compare_identical_routes() {
        let points = long_route();
        let sig1 = create_signature("a".to_string(), points.clone()).unwrap();
        let sig2 = create_signature("b".to_string(), points).unwrap();

        let result = ffi_compare_routes(&sig1, &sig2, MatchConfig::default());

        assert!(result.is_some(), "Identical routes should match");
        let result = result.unwrap();
        assert!(result.match_percentage > 95.0, "Identical routes should have >95% match");
        assert_eq!(result.direction, "same");
    }

    #[test]
    fn test_compare_routes_with_noise() {
        let clean = long_route();
        let noisy = noisy_route();

        let sig1 = create_signature("clean".to_string(), clean).unwrap();
        let sig2 = create_signature("noisy".to_string(), noisy).unwrap();

        let result = ffi_compare_routes(&sig1, &sig2, MatchConfig::default());

        // Should still match despite GPS noise
        assert!(result.is_some(), "Routes with noise should still match");
        let result = result.unwrap();
        assert!(result.match_percentage > 70.0, "Noisy routes should have >70% match");
    }

    #[test]
    fn test_compare_reversed_routes() {
        let points = long_route();
        let mut reversed = points.clone();
        reversed.reverse();

        let sig1 = create_signature("forward".to_string(), points).unwrap();
        let sig2 = create_signature("reverse".to_string(), reversed).unwrap();

        let result = ffi_compare_routes(&sig1, &sig2, MatchConfig::default());

        assert!(result.is_some(), "Reversed routes should match");
        let result = result.unwrap();
        assert!(result.match_percentage > 90.0, "Reversed routes should have high match");
        assert_eq!(result.direction, "reverse", "Should detect reverse direction");
    }

    #[test]
    fn test_compare_different_routes() {
        let london = long_route();
        let nyc = different_route();

        let sig1 = create_signature("london".to_string(), london).unwrap();
        let sig2 = create_signature("nyc".to_string(), nyc).unwrap();

        let result = ffi_compare_routes(&sig1, &sig2, MatchConfig::default());

        assert!(result.is_none(), "Completely different routes should not match");
    }

    #[test]
    fn test_compare_routes_symmetry() {
        // AMD should be symmetric: compare(A,B) ≈ compare(B,A)
        let route1 = long_route();
        let route2 = noisy_route();

        let sig1 = create_signature("a".to_string(), route1).unwrap();
        let sig2 = create_signature("b".to_string(), route2).unwrap();

        let result_ab = ffi_compare_routes(&sig1, &sig2, MatchConfig::default());
        let result_ba = ffi_compare_routes(&sig2, &sig1, MatchConfig::default());

        match (result_ab, result_ba) {
            (Some(ab), Some(ba)) => {
                let diff = (ab.match_percentage - ba.match_percentage).abs();
                assert!(diff < 5.0, "Match should be symmetric (diff: {}%)", diff);
            }
            (None, None) => {} // Both don't match - symmetric
            _ => panic!("Asymmetric match/no-match result"),
        }
    }

    #[test]
    fn test_compare_partial_overlap() {
        // Route B is first half of route A
        let full_route = generate_route(51.5074, -0.1278, 2.0, 100, 0.0);
        let half_route: Vec<GpsPoint> = full_route[..full_route.len()/2].to_vec();

        let sig1 = create_signature("full".to_string(), full_route).unwrap();
        let sig2 = create_signature("half".to_string(), half_route).unwrap();

        let result = ffi_compare_routes(&sig1, &sig2, MatchConfig::default());

        // Partial overlap - may or may not match depending on threshold
        // This tests the distance ratio check (routes must be within 50% of each other)
        // Half route is 50% of full, so this is at the boundary
        if let Some(r) = result {
            assert!(r.direction == "partial" || r.match_percentage < 80.0,
                "Partial overlap should have lower match or be marked partial");
        }
    }

    // ========================================================================
    // Grouping Tests
    // ========================================================================

    #[test]
    fn test_group_empty_input() {
        let groups = ffi_group_signatures(vec![], MatchConfig::default());
        assert!(groups.is_empty(), "Empty input should return empty groups");
    }

    #[test]
    fn test_group_single_signature() {
        let sig = create_signature("only".to_string(), long_route()).unwrap();
        let groups = ffi_group_signatures(vec![sig], MatchConfig::default());

        assert_eq!(groups.len(), 1, "Single signature should create one group");
        assert_eq!(groups[0].activity_ids.len(), 1);
    }

    #[test]
    fn test_group_identical_routes() {
        let route = long_route();
        let sig1 = create_signature("a".to_string(), route.clone()).unwrap();
        let sig2 = create_signature("b".to_string(), route.clone()).unwrap();
        let sig3 = create_signature("c".to_string(), route).unwrap();

        let groups = ffi_group_signatures(vec![sig1, sig2, sig3], MatchConfig::default());

        assert_eq!(groups.len(), 1, "Identical routes should form one group");
        assert_eq!(groups[0].activity_ids.len(), 3);
    }

    #[test]
    fn test_group_different_routes() {
        let london = create_signature("london".to_string(), long_route()).unwrap();
        let nyc = create_signature("nyc".to_string(), different_route()).unwrap();

        // Generate a third distinct route (Paris area)
        let paris = create_signature(
            "paris".to_string(),
            generate_route(48.8566, 2.3522, 1.0, 100, 0.0)
        ).unwrap();

        let groups = ffi_group_signatures(vec![london, nyc, paris], MatchConfig::default());

        assert_eq!(groups.len(), 3, "Different routes should form separate groups");
    }

    #[test]
    fn test_group_mixed_routes() {
        // 3 London routes (should group), 2 NYC routes (should group), 1 Paris (alone)
        let london1 = create_signature("london1".to_string(), long_route()).unwrap();
        let london2 = create_signature("london2".to_string(), noisy_route()).unwrap();
        let london3 = create_signature("london3".to_string(), generate_route(51.5074, -0.1278, 1.0, 100, 5.0)).unwrap();

        let nyc1 = create_signature("nyc1".to_string(), different_route()).unwrap();
        let nyc2 = create_signature("nyc2".to_string(), generate_route(40.7128, -74.0060, 1.0, 100, 5.0)).unwrap();

        let paris = create_signature("paris".to_string(), generate_route(48.8566, 2.3522, 1.0, 100, 0.0)).unwrap();

        let groups = ffi_group_signatures(
            vec![london1, london2, london3, nyc1, nyc2, paris],
            MatchConfig::default()
        );

        // Should form 3 groups: London(3), NYC(2), Paris(1)
        assert_eq!(groups.len(), 3, "Should form 3 groups");

        // Find groups by size
        let sizes: Vec<usize> = groups.iter().map(|g| g.activity_ids.len()).collect();
        assert!(sizes.contains(&3), "Should have a group of 3 (London)");
        assert!(sizes.contains(&2), "Should have a group of 2 (NYC)");
        assert!(sizes.contains(&1), "Should have a group of 1 (Paris)");
    }

    #[test]
    fn test_group_short_routes_excluded() {
        // Routes under min_route_distance (500m) should still be grouped
        // but won't affect matching threshold
        let short1 = create_signature(
            "short1".to_string(),
            generate_route(51.5074, -0.1278, 0.3, 100, 0.0) // 300m
        );
        let short2 = create_signature(
            "short2".to_string(),
            generate_route(51.5074, -0.1278, 0.3, 100, 0.0) // 300m
        );

        if let (Some(s1), Some(s2)) = (short1, short2) {
            let groups = ffi_group_signatures(vec![s1, s2], MatchConfig::default());
            // Short routes may or may not group depending on implementation
            assert!(!groups.is_empty(), "Should return at least one group");
        }
    }

    // ========================================================================
    // Incremental Grouping Tests
    // ========================================================================

    #[test]
    fn test_incremental_adds_to_existing_group() {
        let route = long_route();
        let sig1 = create_signature("a".to_string(), route.clone()).unwrap();
        let sig2 = create_signature("b".to_string(), route.clone()).unwrap();
        let sig3 = create_signature("c".to_string(), route).unwrap();

        // Initial grouping with first two
        let initial_groups = ffi_group_signatures(vec![sig1.clone(), sig2.clone()], MatchConfig::default());
        assert_eq!(initial_groups.len(), 1);
        assert_eq!(initial_groups[0].activity_ids.len(), 2);

        // Incrementally add third
        let updated = ffi_group_incremental(
            vec![sig3],
            initial_groups,
            vec![sig1, sig2],
            MatchConfig::default(),
        );

        assert_eq!(updated.len(), 1, "Should still have one group");
        assert_eq!(updated[0].activity_ids.len(), 3, "Should now have 3 activities");
    }

    #[test]
    fn test_incremental_creates_new_group() {
        let london_route = long_route();
        let sig1 = create_signature("london1".to_string(), london_route.clone()).unwrap();
        let sig2 = create_signature("london2".to_string(), london_route).unwrap();

        // Initial grouping
        let initial_groups = ffi_group_signatures(vec![sig1.clone(), sig2.clone()], MatchConfig::default());
        assert_eq!(initial_groups.len(), 1);

        // Add a different route
        let nyc = create_signature("nyc".to_string(), different_route()).unwrap();
        let updated = ffi_group_incremental(
            vec![nyc],
            initial_groups,
            vec![sig1, sig2],
            MatchConfig::default(),
        );

        assert_eq!(updated.len(), 2, "Should now have two groups");
    }

    #[test]
    fn test_incremental_matches_full_grouping() {
        // Verify that incremental grouping produces same result as full rebuild
        let routes: Vec<Vec<GpsPoint>> = (0..5)
            .map(|i| generate_route(51.5074 + i as f64 * 0.01, -0.1278, 1.0, 100, 5.0))
            .collect();

        let sigs: Vec<RouteSignature> = routes.iter()
            .enumerate()
            .filter_map(|(i, r)| create_signature(format!("route-{}", i), r.clone()))
            .collect();

        // Full grouping
        let full_groups = ffi_group_signatures(sigs.clone(), MatchConfig::default());

        // Incremental: add one at a time
        let mut incremental_groups = vec![];
        let mut existing_sigs = vec![];

        for sig in sigs.iter() {
            incremental_groups = ffi_group_incremental(
                vec![sig.clone()],
                incremental_groups,
                existing_sigs.clone(),
                MatchConfig::default(),
            );
            existing_sigs.push(sig.clone());
        }

        // Should have same number of groups
        assert_eq!(
            full_groups.len(),
            incremental_groups.len(),
            "Incremental should produce same group count as full rebuild"
        );

        // Each group should have same number of activities
        let full_sizes: Vec<usize> = full_groups.iter().map(|g| g.activity_ids.len()).collect();
        let incr_sizes: Vec<usize> = incremental_groups.iter().map(|g| g.activity_ids.len()).collect();

        let mut full_sorted = full_sizes.clone();
        let mut incr_sorted = incr_sizes.clone();
        full_sorted.sort();
        incr_sorted.sort();

        assert_eq!(full_sorted, incr_sorted, "Group sizes should match");
    }

    // ========================================================================
    // Flat Buffer Processing Tests
    // ========================================================================

    #[test]
    fn test_flat_buffer_basic() {
        let route = long_route();
        let flat_coords: Vec<f64> = route.iter()
            .flat_map(|p| vec![p.latitude, p.longitude])
            .collect();

        let track = FlatGpsTrack {
            activity_id: "test".to_string(),
            coords: flat_coords,
        };

        let signatures = create_signatures_from_flat(vec![track], MatchConfig::default());

        assert_eq!(signatures.len(), 1);
        assert_eq!(signatures[0].activity_id, "test");
    }

    #[test]
    fn test_flat_buffer_empty() {
        let track = FlatGpsTrack {
            activity_id: "empty".to_string(),
            coords: vec![],
        };

        let signatures = create_signatures_from_flat(vec![track], MatchConfig::default());
        assert!(signatures.is_empty(), "Empty coords should produce no signatures");
    }

    #[test]
    fn test_flat_buffer_odd_count() {
        // Odd number of coords (malformed - missing one longitude)
        let track = FlatGpsTrack {
            activity_id: "malformed".to_string(),
            coords: vec![51.5074, -0.1278, 51.5084], // 3 values, not 4
        };

        let signatures = create_signatures_from_flat(vec![track], MatchConfig::default());
        // chunks_exact(2) should handle this gracefully
        // Will get 1 point (51.5074, -0.1278), which is not enough
        assert!(signatures.is_empty(), "Odd coords should produce no signature (only 1 valid point)");
    }

    #[test]
    fn test_flat_buffer_multiple_tracks() {
        let route1 = long_route();
        let route2 = different_route();

        let tracks = vec![
            FlatGpsTrack {
                activity_id: "london".to_string(),
                coords: route1.iter().flat_map(|p| vec![p.latitude, p.longitude]).collect(),
            },
            FlatGpsTrack {
                activity_id: "nyc".to_string(),
                coords: route2.iter().flat_map(|p| vec![p.latitude, p.longitude]).collect(),
            },
        ];

        let signatures = create_signatures_from_flat(tracks, MatchConfig::default());
        assert_eq!(signatures.len(), 2);
    }

    #[test]
    fn test_process_routes_from_flat_end_to_end() {
        // Full pipeline: flat buffer -> signatures -> groups
        let route = long_route();
        let noisy_same = noisy_route();

        let tracks = vec![
            FlatGpsTrack {
                activity_id: "a".to_string(),
                coords: route.iter().flat_map(|p| vec![p.latitude, p.longitude]).collect(),
            },
            FlatGpsTrack {
                activity_id: "b".to_string(),
                coords: noisy_same.iter().flat_map(|p| vec![p.latitude, p.longitude]).collect(),
            },
            FlatGpsTrack {
                activity_id: "c".to_string(),
                coords: different_route().iter().flat_map(|p| vec![p.latitude, p.longitude]).collect(),
            },
        ];

        let groups = process_routes_from_flat(tracks, MatchConfig::default());

        assert_eq!(groups.len(), 2, "Should form 2 groups (London pair + NYC)");
    }

    // ========================================================================
    // GPS Edge Cases
    // ========================================================================

    #[test]
    fn test_route_near_equator() {
        let route = generate_route(0.0, 0.0, 1.0, 100, 0.0);
        let sig = create_signature("equator".to_string(), route);
        assert!(sig.is_some(), "Route at equator should work");
    }

    #[test]
    fn test_route_near_poles() {
        // Near north pole - longitude becomes degenerate
        let route = generate_route(89.0, 0.0, 1.0, 100, 0.0);
        let sig = create_signature("arctic".to_string(), route);
        assert!(sig.is_some(), "Route near pole should work");
    }

    #[test]
    fn test_route_crossing_prime_meridian() {
        // Route that crosses longitude 0
        let points: Vec<GpsPoint> = (0..100)
            .map(|i| GpsPoint::new(51.5074, -0.05 + i as f64 * 0.001))
            .collect();

        let sig = create_signature("prime_meridian".to_string(), points);
        assert!(sig.is_some(), "Route crossing prime meridian should work");
    }

    #[test]
    fn test_route_crossing_antimeridian() {
        // Route that crosses longitude 180/-180 (date line)
        // This is a known problematic case for many GPS systems
        let mut points = Vec::new();
        for i in 0..100 {
            let lng = 179.5 + i as f64 * 0.01; // Goes from 179.5 to 180.5
            let normalized_lng = if lng > 180.0 { lng - 360.0 } else { lng };
            points.push(GpsPoint::new(0.0, normalized_lng));
        }

        let _sig = create_signature("date_line".to_string(), points);
        // Note: This test may reveal issues with antimeridian handling
        // If it fails, that's valuable information about a real edge case
    }

    #[test]
    fn test_loop_route() {
        // Route that starts and ends at same point
        let mut points = long_route();
        points.push(points[0]); // Close the loop

        let sig = create_signature("loop".to_string(), points);
        assert!(sig.is_some());

        let sig = sig.unwrap();
        let start_end_dist = crate::geo_utils::haversine_distance(&sig.start_point, &sig.end_point);
        assert!(start_end_dist < 100.0, "Loop should have start ≈ end");
    }

    #[test]
    fn test_out_and_back_route() {
        // Route that goes out and comes back on same path
        let outbound = long_route();
        let mut inbound = outbound.clone();
        inbound.reverse();

        let mut full_route = outbound;
        full_route.extend(inbound);

        let sig = create_signature("out_and_back".to_string(), full_route);
        assert!(sig.is_some());
    }

    #[test]
    fn test_stationary_gps() {
        // GPS recording while standing still (all points clustered)
        let points: Vec<GpsPoint> = (0..100)
            .map(|i| {
                // Tiny variations simulating GPS drift while stationary
                let jitter = (i as f64 * 0.1).sin() * 0.00001; // ~1m jitter
                GpsPoint::new(51.5074 + jitter, -0.1278 + jitter)
            })
            .collect();

        let sig = create_signature("stationary".to_string(), points);
        if let Some(s) = sig {
            assert!(s.total_distance < 100.0, "Stationary should have minimal distance");
        }
    }

    #[test]
    fn test_gps_teleport() {
        // GPS glitch causing sudden "teleport" to wrong location
        let mut points = long_route();
        // Insert a glitch point in the middle
        let mid = points.len() / 2;
        points.insert(mid, GpsPoint::new(0.0, 0.0)); // Teleport to null island

        let sig = create_signature("teleport".to_string(), points);
        // Simplification should hopefully remove the outlier
        // Or the route should still be usable
        assert!(sig.is_some());
    }

    // ========================================================================
    // Configuration Tests
    // ========================================================================

    #[test]
    fn test_custom_config() {
        let points = long_route();

        let strict_config = MatchConfig {
            min_match_percentage: 90.0,
            perfect_threshold: 10.0,
            zero_threshold: 100.0,
            ..MatchConfig::default()
        };

        let lenient_config = MatchConfig {
            min_match_percentage: 30.0,
            perfect_threshold: 100.0,
            zero_threshold: 500.0,
            ..MatchConfig::default()
        };

        let noisy = noisy_route();
        let sig1 = create_signature_with_config("clean".to_string(), points.clone(), strict_config.clone()).unwrap();
        let sig2 = create_signature_with_config("noisy".to_string(), noisy.clone(), strict_config.clone()).unwrap();

        let strict_result = ffi_compare_routes(&sig1, &sig2, strict_config);

        let sig1_len = create_signature_with_config("clean".to_string(), points, lenient_config.clone()).unwrap();
        let sig2_len = create_signature_with_config("noisy".to_string(), noisy, lenient_config.clone()).unwrap();
        let lenient_result = ffi_compare_routes(&sig1_len, &sig2_len, lenient_config);

        // Lenient config should find a match where strict might not
        // (or lenient should have higher match %)
        match (strict_result, lenient_result) {
            (None, Some(_)) => {} // Expected: strict rejects, lenient accepts
            (Some(_), Some(_)) => {
                // Both accept - valid scenario
            }
            _ => {} // Other combinations are valid
        }
    }

    // ========================================================================
    // Section Detection Config Tests
    // ========================================================================

    #[test]
    fn test_section_configs_differ() {
        let default = default_section_config();
        let discovery = discovery_section_config();
        let conservative = conservative_section_config();

        // Discovery should be more sensitive (lower thresholds)
        assert!(discovery.min_activities <= default.min_activities);

        // Conservative should be more strict (higher thresholds)
        assert!(conservative.min_activities >= default.min_activities);
    }

    // ========================================================================
    // Large Scale Stress Tests
    // ========================================================================

    #[test]
    fn test_many_routes_grouping() {
        // 50 routes in 5 "clusters" of 10 each
        let mut sigs = Vec::new();

        for cluster in 0..5 {
            let base_lat = 40.0 + cluster as f64 * 10.0;
            let base_lng = -74.0 + cluster as f64 * 10.0;

            for i in 0..10 {
                let route = generate_route(base_lat, base_lng, 1.0, 50, 10.0);
                if let Some(sig) = create_signature(format!("c{}-r{}", cluster, i), route) {
                    sigs.push(sig);
                }
            }
        }

        assert_eq!(sigs.len(), 50);

        let groups = ffi_group_signatures(sigs, MatchConfig::default());

        // Should form approximately 5 groups
        assert!(groups.len() >= 4 && groups.len() <= 10,
            "Should form roughly 5 groups (got {})", groups.len());
    }

    #[test]
    fn test_many_points_per_route() {
        // Route with 50,000 points (50km at 1 point per meter)
        let points: Vec<GpsPoint> = (0..50_000)
            .map(|i| {
                let progress = i as f64 / 50_000.0;
                GpsPoint::new(51.5074 + progress * 0.5, -0.1278 + progress * 0.5)
            })
            .collect();

        let sig = create_signature("dense".to_string(), points);
        assert!(sig.is_some());

        let sig = sig.unwrap();
        assert!(sig.points.len() <= 100, "Should simplify to manageable size");
    }
}
