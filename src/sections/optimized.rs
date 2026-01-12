//! Optimized section detection for mobile devices.
//!
//! Key optimizations:
//! 1. Track downsampling - Use ~50-100 points for initial detection
//! 2. Geographic grid partitioning - Only compare tracks in same region
//! 3. Incremental mode - Match new activities against existing sections
//! 4. Early termination - Stop when good enough overlap found
//!
//! Performance comparison (77 tracks):
//! - Full resolution: ~60-120 seconds
//! - Optimized: ~1-3 seconds

use super::rtree::{build_rtree, bounds_overlap_tracks, IndexedPoint};
use super::{
    cluster_overlaps, compute_consensus_polyline, extract_all_activity_traces,
    make_sections_exclusive, remove_overlapping_sections, select_medoid,
    split_at_gradient_changes, split_at_heading_changes, FrequentSection, FullTrackOverlap,
    SectionConfig,
};
use crate::geo_utils::haversine_distance;
use crate::matching::calculate_route_distance;
use crate::GpsPoint;
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

        // Build R-trees for downsampled tracks (lazy - only when needed)
        let rtrees: Vec<RTree<IndexedPoint>> = sport_tracks
            .iter()
            .map(|t| build_rtree(&t.downsampled))
            .collect();

        // Find overlaps using downsampled tracks
        let overlap_start = std::time::Instant::now();
        let mut overlaps = Vec::new();

        for (i, j) in candidate_pairs {
            // Bounding box check first
            if !bounds_overlap_tracks(
                &sport_tracks[i].downsampled,
                &sport_tracks[j].downsampled,
                config.proximity_threshold,
            ) {
                continue;
            }

            // Find overlap using downsampled tracks
            if let Some(overlap) = find_overlap_downsampled(
                &sport_tracks[i],
                &sport_tracks[j],
                &rtrees[j],
                config,
            ) {
                overlaps.push(overlap);
            }
        }

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

        // Convert to sections using FULL resolution tracks for final polyline
        let track_map: HashMap<String, Vec<GpsPoint>> = sport_tracks
            .iter()
            .map(|t| (t.activity_id.clone(), t.full_track.clone()))
            .collect();

        for (idx, cluster) in significant.into_iter().enumerate() {
            if let Some(section) = convert_cluster_to_section(
                idx,
                cluster,
                &sport_type,
                &track_map,
                config,
            ) {
                all_sections.push(section);
            }
        }
    }

    info!(
        "[OptimizedSections] Detected {} raw sections in {}ms",
        all_sections.len(),
        start.elapsed().as_millis()
    );

    // Post-process step 1: Remove obvious duplicates
    let deduped = remove_overlapping_sections(all_sections, config);
    info!(
        "[OptimizedSections] After deduplication: {} sections",
        deduped.len()
    );

    // Post-process step 2: Make sections mutually exclusive (cut at overlap boundaries)
    let exclusive = make_sections_exclusive(deduped, config);
    info!(
        "[OptimizedSections] After exclusivity: {} sections",
        exclusive.len()
    );

    // Post-process step 3: Split at heading inflection points
    // (Run AFTER exclusivity so newly-created sections get split too)
    let heading_start = std::time::Instant::now();
    let heading_sections = split_at_heading_changes(exclusive, config);
    info!(
        "[OptimizedSections] After heading splitting: {} sections in {}ms",
        heading_sections.len(),
        heading_start.elapsed().as_millis()
    );

    // Post-process step 4: Split at gradient changes (if elevation available)
    let gradient_start = std::time::Instant::now();
    let final_sections = split_at_gradient_changes(heading_sections, config);
    info!(
        "[OptimizedSections] After gradient splitting: {} sections in {}ms",
        final_sections.len(),
        gradient_start.elapsed().as_millis()
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

    let stability = super::compute_initial_stability(
        consensus.observation_count,
        consensus.average_spread,
        config.proximity_threshold,
    );

    Some(FrequentSection {
        id: format!("sec_{}_{}", sport_type.to_lowercase(), idx),
        name: None,
        sport_type: sport_type.to_string(),
        polyline: consensus.polyline,
        representative_activity_id: representative_id,
        activity_ids: cluster.activity_ids.into_iter().collect(),
        activity_portions: vec![], // Skip for optimized mode
        route_ids: vec![],
        visit_count: cluster.overlaps.len() as u32 + 1,
        distance_meters: calculate_route_distance(&representative_polyline),
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

            if !bounds_overlap_tracks(&downsampled, &other_downsampled, config.proximity_threshold) {
                continue;
            }

            let other_tree = build_rtree(&other_downsampled);
            let track_info_a = TrackInfo::new(new_activity_id.to_string(), new_track.to_vec(), 100);
            let track_info_b = TrackInfo::new(other_id.clone(), other_track.clone(), 100);

            if let Some(overlap) = find_overlap_downsampled(&track_info_a, &track_info_b, &other_tree, config) {
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
fn matches_section(track: &[GpsPoint], section_polyline: &[GpsPoint], config: &SectionConfig) -> bool {
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
