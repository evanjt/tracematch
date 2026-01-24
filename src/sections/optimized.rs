//! Optimized section detection for mobile devices.
//!
//! Key optimizations:
//! 1. Track downsampling - Use ~50-100 points for initial detection
//! 2. Geographic grid partitioning - Only compare tracks in same region
//! 3. Incremental mode - Match new activities against existing sections
//! 4. Early termination - Stop when good enough overlap found
//! 5. Parallel processing - Uses rayon for overlap detection and cluster conversion
//!
//! Performance comparison (77 tracks):
//! - Full resolution: ~60-120 seconds
//! - Optimized: ~1-3 seconds

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use super::rtree::{IndexedPoint, bounds_overlap_tracks, build_rtree};
use super::{
    FrequentSection, FullTrackOverlap, SectionConfig, cluster_overlaps, compute_consensus_polyline,
    compute_initial_stability, consolidate_fragments, extract_all_activity_traces,
    filter_low_quality_sections, make_sections_exclusive, merge_nearby_sections,
    remove_overlapping_sections, select_medoid, split_at_gradient_changes,
    split_at_heading_changes,
};
use crate::GpsPoint;
use crate::geo_utils::haversine_distance;
use crate::matching::calculate_route_distance;
use log::info;
use rstar::{PointDistance, RTree};
use std::collections::{HashMap, HashSet};

/// Geographic grid cell for spatial partitioning
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct GridCell {
    lat_idx: i32,
    lng_idx: i32,
}

impl GridCell {
    /// Create grid cell from a GPS point
    /// Uses ~5km grid cells (0.05 degrees)
    fn from_point(lat: f64, lng: f64) -> Self {
        const CELL_SIZE: f64 = 0.05; // ~5km at equator
        Self {
            lat_idx: (lat / CELL_SIZE).floor() as i32,
            lng_idx: (lng / CELL_SIZE).floor() as i32,
        }
    }

    /// Get this cell and all 8 adjacent cells
    fn with_neighbors(&self) -> Vec<GridCell> {
        let mut cells = Vec::with_capacity(9);
        for dlat in -1..=1 {
            for dlng in -1..=1 {
                cells.push(GridCell {
                    lat_idx: self.lat_idx + dlat,
                    lng_idx: self.lng_idx + dlng,
                });
            }
        }
        cells
    }
}

/// Downsample a track to approximately target_points
fn downsample_track(track: &[GpsPoint], target_points: usize) -> Vec<GpsPoint> {
    if track.len() <= target_points {
        return track.to_vec();
    }

    let step = track.len() as f64 / target_points as f64;
    let mut result = Vec::with_capacity(target_points + 2);

    // Always include first point
    result.push(track[0]);

    // Sample intermediate points
    for i in 1..target_points {
        let idx = (i as f64 * step) as usize;
        if idx < track.len() && idx != 0 {
            result.push(track[idx]);
        }
    }

    // Always include last point
    if track.len() > 1 {
        result.push(track[track.len() - 1]);
    }

    result
}

/// Track info for optimized detection
struct TrackInfo {
    activity_id: String,
    full_track: Vec<GpsPoint>,
    downsampled: Vec<GpsPoint>,
    cells: HashSet<GridCell>,
}

impl TrackInfo {
    fn new(activity_id: String, track: Vec<GpsPoint>, downsample_to: usize) -> Self {
        let downsampled = downsample_track(&track, downsample_to);

        // Compute all grid cells this track passes through
        let mut cells = HashSet::new();
        for point in &downsampled {
            cells.insert(GridCell::from_point(point.latitude, point.longitude));
        }

        Self {
            activity_id,
            full_track: track,
            downsampled,
            cells,
        }
    }
}

/// Optimized section detection using downsampling and grid partitioning.
///
/// For mobile devices, this is 20-50x faster than full resolution detection.
pub fn detect_sections_optimized(
    tracks: &[(String, Vec<GpsPoint>)],
    sport_types: &HashMap<String, String>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    let start = std::time::Instant::now();

    if tracks.len() < config.min_activities as usize {
        return vec![];
    }

    // Group by sport type first
    let mut tracks_by_sport: HashMap<String, Vec<TrackInfo>> = HashMap::new();
    for (activity_id, points) in tracks {
        let sport = sport_types
            .get(activity_id)
            .cloned()
            .unwrap_or_else(|| "Unknown".to_string());

        // Downsample to 100 points for initial detection
        let track_info = TrackInfo::new(activity_id.clone(), points.clone(), 100);
        tracks_by_sport.entry(sport).or_default().push(track_info);
    }

    let mut all_sections = Vec::new();

    for (sport_type, sport_tracks) in tracks_by_sport {
        if sport_tracks.len() < config.min_activities as usize {
            continue;
        }

        info!(
            "[OptimizedSections] Processing {} {} tracks",
            sport_tracks.len(),
            sport_type
        );

        // Build grid index: cell -> list of track indices
        let mut grid_index: HashMap<GridCell, Vec<usize>> = HashMap::new();
        for (idx, track) in sport_tracks.iter().enumerate() {
            for cell in &track.cells {
                grid_index.entry(*cell).or_default().push(idx);
            }
        }

        // Find candidate pairs using grid (only tracks in same/adjacent cells)
        let grid_start = std::time::Instant::now();
        let mut candidate_pairs: HashSet<(usize, usize)> = HashSet::new();

        for (idx, track) in sport_tracks.iter().enumerate() {
            // Get all tracks in this track's cells and neighbors
            let mut nearby_indices: HashSet<usize> = HashSet::new();
            for cell in &track.cells {
                for neighbor in cell.with_neighbors() {
                    if let Some(indices) = grid_index.get(&neighbor) {
                        nearby_indices.extend(indices.iter().copied());
                    }
                }
            }

            // Add pairs (avoid duplicates by ensuring i < j)
            for other_idx in nearby_indices {
                if other_idx != idx {
                    let pair = if idx < other_idx {
                        (idx, other_idx)
                    } else {
                        (other_idx, idx)
                    };
                    candidate_pairs.insert(pair);
                }
            }
        }

        info!(
            "[OptimizedSections] Grid filtering: {} candidate pairs (of {} possible) in {}ms",
            candidate_pairs.len(),
            sport_tracks.len() * (sport_tracks.len() - 1) / 2,
            grid_start.elapsed().as_millis()
        );

        let rtree_start = std::time::Instant::now();
        #[cfg(feature = "parallel")]
        let rtrees: Vec<RTree<IndexedPoint>> = sport_tracks
            .par_iter()
            .map(|t| build_rtree(&t.downsampled))
            .collect();

        #[cfg(not(feature = "parallel"))]
        let rtrees: Vec<RTree<IndexedPoint>> = sport_tracks
            .iter()
            .map(|t| build_rtree(&t.downsampled))
            .collect();

        info!(
            "[OptimizedSections] Built {} R-trees in {}ms",
            rtrees.len(),
            rtree_start.elapsed().as_millis()
        );

        let overlap_start = std::time::Instant::now();

        let candidate_vec: Vec<(usize, usize)> = candidate_pairs.into_iter().collect();

        #[cfg(feature = "parallel")]
        let overlaps: Vec<FullTrackOverlap> = candidate_vec
            .into_par_iter()
            .filter_map(|(i, j)| {
                if !bounds_overlap_tracks(
                    &sport_tracks[i].downsampled,
                    &sport_tracks[j].downsampled,
                    config.proximity_threshold,
                ) {
                    return None;
                }
                find_overlap_downsampled(&sport_tracks[i], &sport_tracks[j], &rtrees[j], config)
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let overlaps: Vec<FullTrackOverlap> = candidate_vec
            .into_iter()
            .filter_map(|(i, j)| {
                if !bounds_overlap_tracks(
                    &sport_tracks[i].downsampled,
                    &sport_tracks[j].downsampled,
                    config.proximity_threshold,
                ) {
                    return None;
                }
                find_overlap_downsampled(&sport_tracks[i], &sport_tracks[j], &rtrees[j], config)
            })
            .collect();

        info!(
            "[OptimizedSections] Found {} overlaps in {}ms",
            overlaps.len(),
            overlap_start.elapsed().as_millis()
        );

        // Cluster and convert to sections
        let clusters = cluster_overlaps(overlaps, config);
        let significant: Vec<_> = clusters
            .into_iter()
            .filter(|c| c.activity_ids.len() >= config.min_activities as usize)
            .collect();

        info!(
            "[OptimizedSections] {} significant clusters for {}",
            significant.len(),
            sport_type
        );

        let track_map: HashMap<String, Vec<GpsPoint>> = sport_tracks
            .iter()
            .map(|t| (t.activity_id.clone(), t.full_track.clone()))
            .collect();

        let cluster_data: Vec<_> = significant.into_iter().enumerate().collect();

        #[cfg(feature = "parallel")]
        let sport_sections: Vec<FrequentSection> = cluster_data
            .into_par_iter()
            .filter_map(|(idx, cluster)| {
                convert_cluster_to_section(idx, cluster, &sport_type, &track_map, config)
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let sport_sections: Vec<FrequentSection> = cluster_data
            .into_iter()
            .filter_map(|(idx, cluster)| {
                convert_cluster_to_section(idx, cluster, &sport_type, &track_map, config)
            })
            .collect();

        all_sections.extend(sport_sections);
    }

    info!(
        "[OptimizedSections] Detected {} raw sections in {}ms",
        all_sections.len(),
        start.elapsed().as_millis()
    );

    // ==========================================================================
    // Post-processing Pipeline (MDL-aware, prevents over-fragmentation)
    //
    // Order rationale (based on TRACLUS and TS-MF research):
    // 1. Dedup first - remove obvious duplicates before any splitting
    // 2. Split at terrain changes - while sections are still complete
    // 3. Consolidate fragments - merge back over-split pieces (TS-MF "mergence")
    // 4. Make exclusive - cut at overlaps after meaningful sections exist
    // 5. Merge nearby - handle GPS drift and reversed sections last
    //
    // References:
    // - TRACLUS (Lee, Han, Whang 2007): https://hanj.cs.illinois.edu/pdf/sigmod07_jglee.pdf
    // - TS-MF (Xu et al. 2022): https://www.hindawi.com/journals/wcmc/2022/9540944/
    // ==========================================================================

    // Post-process step 1: Remove obvious duplicates
    let deduped = remove_overlapping_sections(all_sections, config);
    info!(
        "[OptimizedSections] After deduplication: {} sections",
        deduped.len()
    );

    // Post-process step 2: Split at heading inflection points
    // Run BEFORE exclusivity - split complete sections, not arbitrary cuts
    // MDL guards prevent over-fragmentation (skip sections < 500m, check ratio)
    let heading_start = std::time::Instant::now();
    let heading_sections = split_at_heading_changes(deduped, config);
    info!(
        "[OptimizedSections] After heading splitting: {} sections in {}ms",
        heading_sections.len(),
        heading_start.elapsed().as_millis()
    );

    // Post-process step 3: Split at gradient changes (if elevation available)
    let gradient_start = std::time::Instant::now();
    let gradient_sections = split_at_gradient_changes(heading_sections, config);
    info!(
        "[OptimizedSections] After gradient splitting: {} sections in {}ms",
        gradient_sections.len(),
        gradient_start.elapsed().as_millis()
    );

    // Post-process step 4: Consolidate short fragments back together
    // Based on TS-MF "mergence" phase - reverses over-aggressive splitting
    let consolidate_start = std::time::Instant::now();
    let consolidated = consolidate_fragments(gradient_sections, config);
    info!(
        "[OptimizedSections] After consolidation: {} sections in {}ms",
        consolidated.len(),
        consolidate_start.elapsed().as_millis()
    );

    // Post-process step 5: Make sections mutually exclusive (cut at overlap boundaries)
    let exclusive = make_sections_exclusive(consolidated, config);
    info!(
        "[OptimizedSections] After exclusivity: {} sections",
        exclusive.len()
    );

    // Post-process step 6: Merge nearby/reversed sections
    // This handles: reversed sections (out-and-back), parallel tracks, GPS drift
    let merge_start = std::time::Instant::now();
    let merged = merge_nearby_sections(exclusive, config);
    info!(
        "[OptimizedSections] After merging nearby: {} sections in {}ms",
        merged.len(),
        merge_start.elapsed().as_millis()
    );

    // Post-process step 7: Quality filter (length-weighted visit threshold)
    // Short sections need more visits to prove they're meaningful patterns.
    // Reference: Graph-based clustering treats low-density regions as noise.
    let quality_start = std::time::Instant::now();
    let final_sections = filter_low_quality_sections(merged);
    info!(
        "[OptimizedSections] After quality filter: {} sections in {}ms",
        final_sections.len(),
        quality_start.elapsed().as_millis()
    );

    info!(
        "[OptimizedSections] Final: {} sections (total {}ms)",
        final_sections.len(),
        start.elapsed().as_millis()
    );

    final_sections
}

/// Find overlap between two tracks using downsampled versions
fn find_overlap_downsampled(
    track_a: &TrackInfo,
    track_b: &TrackInfo,
    tree_b: &RTree<IndexedPoint>,
    config: &SectionConfig,
) -> Option<FullTrackOverlap> {
    let threshold_deg = config.proximity_threshold / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;

    let mut overlap_points_a = Vec::new();
    let mut overlap_points_b = Vec::new();
    let mut overlap_length = 0.0;

    let mut in_overlap = false;
    let mut last_point: Option<&GpsPoint> = None;

    for point_a in &track_a.downsampled {
        let query = [point_a.latitude, point_a.longitude];

        if let Some(nearest) = tree_b.nearest_neighbor(&query) {
            let dist_sq = nearest.distance_2(&query);

            if dist_sq <= threshold_deg_sq {
                overlap_points_a.push(*point_a);

                if nearest.idx < track_b.downsampled.len() {
                    overlap_points_b.push(track_b.downsampled[nearest.idx]);
                }

                if let Some(prev) = last_point {
                    overlap_length += haversine_distance(prev, point_a);
                }
                in_overlap = true;
                last_point = Some(point_a);
            } else if in_overlap {
                // Gap in overlap - check if we have enough
                if overlap_length >= config.min_section_length {
                    break; // Early termination - found good overlap
                }
                // Reset if not enough
                overlap_points_a.clear();
                overlap_points_b.clear();
                overlap_length = 0.0;
                in_overlap = false;
                last_point = None;
            }
        }
    }

    // Check final overlap
    if overlap_length >= config.min_section_length && !overlap_points_a.is_empty() {
        let center = crate::geo_utils::compute_center(&overlap_points_a);
        Some(FullTrackOverlap {
            activity_a: track_a.activity_id.clone(),
            activity_b: track_b.activity_id.clone(),
            points_a: overlap_points_a,
            points_b: overlap_points_b,
            center,
        })
    } else {
        None
    }
}

/// Convert cluster to FrequentSection using full resolution tracks
fn convert_cluster_to_section(
    idx: usize,
    cluster: super::OverlapCluster,
    sport_type: &str,
    track_map: &HashMap<String, Vec<GpsPoint>>,
    config: &SectionConfig,
) -> Option<FrequentSection> {
    let (representative_id, representative_polyline) = select_medoid(&cluster);

    if representative_polyline.is_empty() {
        return None;
    }

    let distance_meters = calculate_route_distance(&representative_polyline);

    if distance_meters > config.max_section_length {
        return None;
    }

    // Extract traces using full resolution
    let activity_ids: Vec<String> = cluster.activity_ids.iter().cloned().collect();
    let activity_traces =
        extract_all_activity_traces(&activity_ids, &representative_polyline, track_map);

    let all_traces: Vec<Vec<GpsPoint>> = activity_traces.values().cloned().collect();

    let consensus = compute_consensus_polyline(
        &representative_polyline,
        &all_traces,
        config.proximity_threshold,
    );

    // Calculate consensus distance and filter by min length
    let consensus_distance = calculate_route_distance(&consensus.polyline);
    if consensus_distance < config.min_section_length {
        return None;
    }

    let stability = super::compute_initial_stability(
        consensus.observation_count,
        consensus.average_spread,
        config.proximity_threshold,
    );

    // Count activity_ids before moving
    let activity_count = cluster.activity_ids.len();

    Some(FrequentSection {
        id: format!("sec_{}_{}", sport_type.to_lowercase(), idx),
        name: None,
        sport_type: sport_type.to_string(),
        polyline: consensus.polyline,
        representative_activity_id: representative_id,
        activity_ids: cluster.activity_ids.into_iter().collect(),
        activity_portions: vec![], // Skip for optimized mode
        route_ids: vec![],
        // visit_count should equal unique activities
        visit_count: activity_count as u32,
        distance_meters: consensus_distance, // Use consensus distance, not representative
        activity_traces,
        confidence: consensus.confidence,
        observation_count: consensus.observation_count,
        average_spread: consensus.average_spread,
        point_density: consensus.point_density,
        scale: Some("optimized".to_string()),
        version: 1,
        is_user_defined: false,
        created_at: None,
        updated_at: None,
        stability,
    })
}

/// Incremental section detection - for when a new activity is added.
///
/// Instead of re-running full detection, this:
/// 1. Matches the new activity against existing sections
/// 2. Updates section consensus if match found
/// 3. Checks for new section candidates with recent activities only
///
/// This is O(n) in number of sections, not O(n²) in activities.
pub fn detect_sections_incremental(
    new_activity_id: &str,
    new_track: &[GpsPoint],
    existing_sections: &[FrequentSection],
    recent_tracks: &[(String, Vec<GpsPoint>)], // Last ~10 activities for new section detection
    config: &SectionConfig,
) -> IncrementalResult {
    let mut updated_sections = Vec::new();
    let mut new_sections = Vec::new();
    let mut matched_section_ids = Vec::new();

    let downsampled = downsample_track(new_track, 100);

    // Check against existing sections
    for section in existing_sections {
        if matches_section(&downsampled, &section.polyline, config) {
            matched_section_ids.push(section.id.clone());

            // Update section with new trace (if not user-defined)
            if !section.is_user_defined {
                let mut new_traces = HashMap::new();
                new_traces.insert(new_activity_id.to_string(), new_track.to_vec());

                let update_result = super::evolution::update_section_with_new_traces(
                    section,
                    &new_traces,
                    config,
                    None,
                );

                if update_result.was_modified {
                    updated_sections.push(update_result.section);
                }
            }
        }
    }

    // Check for new sections with recent activities
    if matched_section_ids.is_empty() && !recent_tracks.is_empty() {
        for (other_id, other_track) in recent_tracks {
            if other_id == new_activity_id {
                continue;
            }

            let other_downsampled = downsample_track(other_track, 100);

            if !bounds_overlap_tracks(&downsampled, &other_downsampled, config.proximity_threshold)
            {
                continue;
            }

            let other_tree = build_rtree(&other_downsampled);
            let track_info_a = TrackInfo::new(new_activity_id.to_string(), new_track.to_vec(), 100);
            let track_info_b = TrackInfo::new(other_id.clone(), other_track.clone(), 100);

            if let Some(overlap) =
                find_overlap_downsampled(&track_info_a, &track_info_b, &other_tree, config)
            {
                // Found potential new section
                let polyline = overlap.points_a.clone();
                let distance = calculate_route_distance(&polyline);

                if distance >= config.min_section_length && distance <= config.max_section_length {
                    new_sections.push(FrequentSection {
                        id: format!("sec_new_{}", new_activity_id),
                        name: None,
                        sport_type: "Unknown".to_string(),
                        polyline,
                        representative_activity_id: new_activity_id.to_string(),
                        activity_ids: vec![new_activity_id.to_string(), other_id.clone()],
                        activity_portions: vec![],
                        route_ids: vec![],
                        visit_count: 2,
                        distance_meters: distance,
                        activity_traces: HashMap::new(),
                        confidence: 0.5,
                        observation_count: 2,
                        average_spread: config.proximity_threshold / 2.0,
                        point_density: vec![2; 10],
                        scale: Some("incremental".to_string()),
                        version: 1,
                        is_user_defined: false,
                        created_at: None,
                        updated_at: None,
                        stability: 0.3,
                    });
                    break; // One new section per activity is enough
                }
            }
        }
    }

    IncrementalResult {
        matched_section_ids,
        updated_sections,
        new_sections,
    }
}

/// Check if a track matches a section polyline
fn matches_section(
    track: &[GpsPoint],
    section_polyline: &[GpsPoint],
    config: &SectionConfig,
) -> bool {
    if track.is_empty() || section_polyline.is_empty() {
        return false;
    }

    let threshold = config.proximity_threshold * 1.5;

    // Sample section points and check if track passes near them
    let sample_step = (section_polyline.len() / 10).max(1);
    let mut near_count = 0;
    let mut samples = 0;

    for (i, section_point) in section_polyline.iter().enumerate() {
        if i % sample_step != 0 {
            continue;
        }
        samples += 1;

        for track_point in track {
            if haversine_distance(section_point, track_point) <= threshold {
                near_count += 1;
                break;
            }
        }
    }

    // Require 60% coverage
    samples > 0 && (near_count as f64 / samples as f64) >= 0.6
}

/// Result of incremental section detection
#[derive(Debug)]
pub struct IncrementalResult {
    /// IDs of existing sections that the new activity matches
    pub matched_section_ids: Vec<String>,
    /// Sections that were updated with the new activity
    pub updated_sections: Vec<FrequentSection>,
    /// New sections discovered (potential, may need promotion)
    pub new_sections: Vec<FrequentSection>,
}

/// A section match found within a route
#[derive(Debug, Clone)]
pub struct SectionMatch {
    /// The section ID that was matched
    pub section_id: String,
    /// Start index in the route's GPS points
    pub start_index: u64,
    /// End index in the route's GPS points (exclusive)
    pub end_index: u64,
    /// Match quality (0.0 to 1.0)
    pub match_quality: f64,
    /// Direction: true if route travels same direction as section, false if reversed
    pub same_direction: bool,
}

/// Find all sections that exist within a given GPS route.
///
/// This scans the route and identifies where each known section appears,
/// returning the start/end indices and match quality for each.
///
/// # Arguments
/// * `route` - The GPS track to search within
/// * `sections` - Known sections to search for
/// * `config` - Section detection configuration
///
/// # Returns
/// Vector of SectionMatch, sorted by start_index
pub fn find_sections_in_route(
    route: &[GpsPoint],
    sections: &[FrequentSection],
    config: &SectionConfig,
) -> Vec<SectionMatch> {
    if route.is_empty() || sections.is_empty() {
        return Vec::new();
    }

    let mut matches = Vec::new();
    let threshold = config.proximity_threshold * 1.5;

    for section in sections {
        if section.polyline.is_empty() {
            continue;
        }

        // Find where this section appears in the route
        if let Some(match_info) = find_section_span_in_route(route, &section.polyline, threshold) {
            matches.push(SectionMatch {
                section_id: section.id.clone(),
                start_index: match_info.0 as u64,
                end_index: match_info.1 as u64,
                match_quality: match_info.2,
                same_direction: match_info.3,
            });
        }
    }

    // Sort by start index
    matches.sort_by_key(|m| m.start_index);
    matches
}

/// Find where a section polyline appears in a route.
/// Returns (start_index, end_index, quality, same_direction)
fn find_section_span_in_route(
    route: &[GpsPoint],
    section: &[GpsPoint],
    threshold: f64,
) -> Option<(usize, usize, f64, bool)> {
    if route.len() < 3 || section.len() < 3 {
        return None;
    }

    // Try both directions
    let forward = find_section_span_directed(route, section, threshold);
    let reversed: Vec<_> = section.iter().rev().cloned().collect();
    let backward = find_section_span_directed(route, &reversed, threshold);

    match (forward, backward) {
        (Some(f), Some(b)) => {
            if f.2 >= b.2 {
                Some((f.0, f.1, f.2, true))
            } else {
                Some((b.0, b.1, b.2, false))
            }
        }
        (Some(f), None) => Some((f.0, f.1, f.2, true)),
        (None, Some(b)) => Some((b.0, b.1, b.2, false)),
        (None, None) => None,
    }
}

/// Find section span in one direction
fn find_section_span_directed(
    route: &[GpsPoint],
    section: &[GpsPoint],
    threshold: f64,
) -> Option<(usize, usize, f64)> {
    // Find the first section point in the route
    let section_start = &section[0];
    let section_end = section.last()?;

    let mut best_start_idx = None;
    let mut best_start_dist = f64::MAX;

    // Find closest point to section start
    for (i, point) in route.iter().enumerate() {
        let dist = haversine_distance(point, section_start);
        if dist < threshold && dist < best_start_dist {
            best_start_dist = dist;
            best_start_idx = Some(i);
        }
    }

    let start_idx = best_start_idx?;

    // Find closest point to section end (after start)
    let mut best_end_idx = None;
    let mut best_end_dist = f64::MAX;

    for (i, point) in route.iter().enumerate().skip(start_idx + 1) {
        let dist = haversine_distance(point, section_end);
        if dist < threshold && dist < best_end_dist {
            best_end_dist = dist;
            best_end_idx = Some(i);
        }
    }

    let end_idx = best_end_idx.unwrap_or(route.len() - 1);

    // Calculate match quality by sampling section points
    let sample_step = (section.len() / 10).max(1);
    let mut matched = 0;
    let mut total = 0;

    for (i, section_point) in section.iter().enumerate() {
        if i % sample_step != 0 {
            continue;
        }
        total += 1;

        // Check if any route point in the span is near this section point
        for route_point in &route[start_idx..=end_idx.min(route.len() - 1)] {
            if haversine_distance(route_point, section_point) <= threshold {
                matched += 1;
                break;
            }
        }
    }

    let quality = if total > 0 {
        matched as f64 / total as f64
    } else {
        0.0
    };

    // Require at least 60% match
    if quality >= 0.6 {
        Some((start_idx, end_idx + 1, quality))
    } else {
        None
    }
}

/// Result of splitting a section
#[derive(Debug, Clone)]
pub struct SplitResult {
    /// The first part of the split section
    pub first: FrequentSection,
    /// The second part of the split section
    pub second: FrequentSection,
}

/// Split a section at a specific point index.
///
/// This creates two new sections from the original, splitting at the given
/// index in the polyline. Both sections inherit the activity associations
/// but need their portions recalculated.
///
/// # Arguments
/// * `section` - The section to split
/// * `split_index` - Index in the polyline where to split (must be > 0 and < len-1)
///
/// # Returns
/// SplitResult with two new sections, or None if split_index is invalid
pub fn split_section_at_index(
    section: &FrequentSection,
    split_index: usize,
) -> Option<SplitResult> {
    if split_index == 0 || split_index >= section.polyline.len() - 1 {
        return None;
    }

    let first_polyline: Vec<_> = section.polyline[..=split_index].to_vec();
    let second_polyline: Vec<_> = section.polyline[split_index..].to_vec();

    let first_distance = crate::matching::calculate_route_distance(&first_polyline);
    let second_distance = crate::matching::calculate_route_distance(&second_polyline);

    let base_id = section.id.trim_end_matches(char::is_numeric);

    Some(SplitResult {
        first: FrequentSection {
            id: format!("{}_a", base_id),
            name: section.name.clone().map(|n| format!("{} (1)", n)),
            sport_type: section.sport_type.clone(),
            polyline: first_polyline,
            representative_activity_id: section.representative_activity_id.clone(),
            activity_ids: section.activity_ids.clone(),
            activity_portions: vec![], // Need recalculation
            route_ids: section.route_ids.clone(),
            visit_count: section.visit_count,
            distance_meters: first_distance,
            activity_traces: section.activity_traces.clone(),
            confidence: section.confidence * 0.9, // Slight reduction
            observation_count: section.observation_count,
            average_spread: section.average_spread,
            point_density: vec![], // Need recalculation
            scale: section.scale.clone(),
            version: section.version + 1,
            is_user_defined: true, // Mark as user-modified
            created_at: section.created_at.clone(),
            updated_at: None,
            stability: section.stability,
        },
        second: FrequentSection {
            id: format!("{}_b", base_id),
            name: section.name.clone().map(|n| format!("{} (2)", n)),
            sport_type: section.sport_type.clone(),
            polyline: second_polyline,
            representative_activity_id: section.representative_activity_id.clone(),
            activity_ids: section.activity_ids.clone(),
            activity_portions: vec![],
            route_ids: section.route_ids.clone(),
            visit_count: section.visit_count,
            distance_meters: second_distance,
            activity_traces: section.activity_traces.clone(),
            confidence: section.confidence * 0.9,
            observation_count: section.observation_count,
            average_spread: section.average_spread,
            point_density: vec![],
            scale: section.scale.clone(),
            version: section.version + 1,
            is_user_defined: true,
            created_at: section.created_at.clone(),
            updated_at: None,
            stability: section.stability,
        },
    })
}

/// Split a section at a geographic point (finds nearest polyline index).
///
/// # Arguments
/// * `section` - The section to split
/// * `split_point` - The geographic point where to split
///
/// # Returns
/// SplitResult with two new sections, or None if point is not near polyline
pub fn split_section_at_point(
    section: &FrequentSection,
    split_point: &GpsPoint,
    max_distance: f64,
) -> Option<SplitResult> {
    // Find nearest polyline index
    let mut best_idx = None;
    let mut best_dist = f64::MAX;

    for (i, point) in section.polyline.iter().enumerate() {
        let dist = haversine_distance(point, split_point);
        if dist < best_dist {
            best_dist = dist;
            best_idx = Some(i);
        }
    }

    let idx = best_idx?;

    // Check if close enough
    if best_dist > max_distance {
        return None;
    }

    // Don't split at endpoints
    if idx == 0 || idx >= section.polyline.len() - 1 {
        return None;
    }

    split_section_at_index(section, idx)
}

/// Recalculate a section's polyline based on its activity traces.
///
/// This is useful when a section's polyline has drifted or is biased
/// toward certain activities. It recomputes the consensus polyline
/// from all stored traces.
///
/// # Arguments
/// * `section` - The section to adjust
/// * `config` - Section configuration
///
/// # Returns
/// Updated section with recalculated polyline, or original if no traces available
pub fn recalculate_section_polyline(
    section: &FrequentSection,
    config: &SectionConfig,
) -> FrequentSection {
    if section.activity_traces.is_empty() || section.is_user_defined {
        return section.clone();
    }

    // Recompute consensus from stored traces
    let traces: Vec<_> = section.activity_traces.values().cloned().collect();

    if traces.is_empty() {
        return section.clone();
    }

    // Use the first trace as reference for consensus
    let reference = &traces[0];

    let consensus =
        super::compute_consensus_polyline(reference, &traces, config.proximity_threshold);

    let new_distance = crate::matching::calculate_route_distance(&consensus.polyline);
    let new_confidence = compute_initial_stability(
        consensus.observation_count,
        consensus.average_spread,
        config.proximity_threshold,
    );

    FrequentSection {
        polyline: consensus.polyline,
        distance_meters: new_distance,
        average_spread: consensus.average_spread,
        point_density: consensus.point_density,
        confidence: new_confidence,
        observation_count: consensus.observation_count,
        version: section.version + 1,
        updated_at: None,
        ..section.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_downsample() {
        let track: Vec<GpsPoint> = (0..1000)
            .map(|i| GpsPoint::new(46.0 + i as f64 * 0.0001, 7.0 + i as f64 * 0.0001))
            .collect();

        let downsampled = downsample_track(&track, 50);
        assert!(downsampled.len() <= 52); // 50 + first + last
        assert!(downsampled.len() >= 50);

        // First and last should be preserved
        assert_eq!(downsampled[0].latitude, track[0].latitude);
        assert_eq!(
            downsampled.last().unwrap().latitude,
            track.last().unwrap().latitude
        );
    }

    #[test]
    fn test_grid_cell() {
        let cell = GridCell::from_point(46.23, 7.36);
        let neighbors = cell.with_neighbors();
        assert_eq!(neighbors.len(), 9);
    }
}
