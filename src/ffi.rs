//! FFI bindings for mobile platforms (iOS/Android).
//!
//! This module provides the UniFFI bindings that expose Rust functionality
//! to Kotlin and Swift. All FFI functions are prefixed with `ffi_` to avoid
//! naming conflicts with the internal API.

use crate::{GpsPoint, RouteGroup, RouteSignature, init_logging};
use log::info;

// ============================================================================
// Callback Interfaces
// ============================================================================

/// Callback interface for receiving progress updates during fetch operations.
/// Implement this in TypeScript/Kotlin/Swift to receive real-time updates.
#[uniffi::export(callback_interface)]
pub trait FetchProgressCallback: Send + Sync {
    /// Called when a single activity fetch completes.
    /// - completed: Number of activities fetched so far
    /// - total: Total number of activities to fetch
    fn on_progress(&self, completed: u32, total: u32);
}

/// Result of polling download progress.
/// Used by TypeScript to show real-time progress without cross-thread callbacks.
#[derive(Debug, Clone, uniffi::Record)]
pub struct DownloadProgressResult {
    /// Number of activities fetched so far
    pub completed: u32,
    /// Total number of activities to fetch
    pub total: u32,
    /// Whether a download is currently active
    pub active: bool,
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

    // Filter out sparse sections - a valid LineString requires at least 2 points
    // This is defense-in-depth (creation-time validation also exists in mod.rs)
    let filtered_sections: Vec<_> = result
        .sections
        .into_iter()
        .filter(|s| s.polyline.len() >= 2)
        .collect();
    let filtered_potentials: Vec<_> = result
        .potentials
        .into_iter()
        .filter(|p| p.polyline.len() >= 2)
        .collect();

    let elapsed = start.elapsed();
    info!(
        "[RouteMatcherRust] Multi-scale detection: {} sections, {} potentials in {:?}",
        filtered_sections.len(),
        filtered_potentials.len(),
        elapsed
    );

    crate::MultiScaleSectionResult {
        sections: filtered_sections,
        potentials: filtered_potentials,
        stats: result.stats,
    }
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

// =============================================================================
// HTTP Activity Fetching (requires "http" feature)
// =============================================================================

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

/// Fetch map data for multiple activities.
///
/// The auth_header should be a pre-formatted Authorization header value:
/// - For API key auth: "Basic {base64(API_KEY:key)}"
/// - For OAuth: "Bearer {access_token}"
#[cfg(feature = "http")]
#[uniffi::export]
pub fn fetch_activity_maps(
    auth_header: String,
    activity_ids: Vec<String>,
) -> Vec<FfiActivityMapResult> {
    init_logging();
    info!(
        "[RouteMatcherRust] fetch_activity_maps called for {} activities",
        activity_ids.len()
    );

    let results = crate::http::fetch_activity_maps_sync(auth_header, activity_ids, None);

    // Convert to FFI-friendly format (flat arrays)
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
///
/// NOTE: The callback is invoked from tokio worker threads. This may cause
/// crashes with some FFI runtimes (like React Native's Hermes) that aren't
/// thread-safe. Use fetch_activity_maps without callback if you experience crashes.
///
/// The auth_header should be a pre-formatted Authorization header value:
/// - For API key auth: "Basic {base64(API_KEY:key)}"
/// - For OAuth: "Bearer {access_token}"
#[cfg(feature = "http")]
#[uniffi::export]
pub fn fetch_activity_maps_with_progress(
    auth_header: String,
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
        crate::http::fetch_activity_maps_sync(auth_header, activity_ids, Some(progress_callback));

    // Convert to FFI-friendly format (flat arrays)
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

/// Get current download progress for FFI polling.
///
/// TypeScript should poll this every 100ms during fetch operations
/// to get smooth progress updates without cross-thread callback issues.
///
/// Returns DownloadProgressResult with completed/total/active fields.
/// When active is false, the download has completed (or never started).
#[cfg(feature = "http")]
#[uniffi::export]
pub fn get_download_progress() -> DownloadProgressResult {
    let (completed, total, active) = crate::http::get_download_progress();
    DownloadProgressResult {
        completed,
        total,
        active,
    }
}
