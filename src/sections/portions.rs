//! Activity portion computation for pace comparison.

use super::overlap::OverlapCluster;
use super::rtree::{IndexedPoint, build_rtree};
use super::{SectionConfig, SectionPortion};
use crate::{Direction, GpsPoint};
use crate::matching::calculate_route_distance;
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use rstar::{PointDistance, RTree};

/// Compute each activity's portion of a section.
/// Returns ALL traversals of the section by each activity (supports out-and-back, track laps).
pub fn compute_activity_portions(
    cluster: &OverlapCluster,
    representative_polyline: &[GpsPoint],
    all_tracks: &std::collections::HashMap<&str, &[GpsPoint]>,
    config: &SectionConfig,
) -> Vec<SectionPortion> {
    let activity_ids: Vec<&String> = cluster.activity_ids.iter().collect();

    let compute_for_activity = |activity_id: &&String| -> Vec<SectionPortion> {
        let mut portions = Vec::new();
        if let Some(track) = all_tracks.get(activity_id.as_str()) {
            let all_traversals =
                find_all_track_portions(track, representative_polyline, config.proximity_threshold);

            for (start_idx, end_idx, direction) in all_traversals {
                let distance = calculate_route_distance(&track[start_idx..end_idx]);

                portions.push(SectionPortion {
                    activity_id: (*activity_id).clone(),
                    start_index: start_idx as u32,
                    end_index: end_idx as u32,
                    distance_meters: distance,
                    direction,
                });
            }
        }
        portions
    };

    #[cfg(feature = "parallel")]
    {
        activity_ids
            .par_iter()
            .flat_map(compute_for_activity)
            .collect()
    }

    #[cfg(not(feature = "parallel"))]
    {
        activity_ids
            .iter()
            .flat_map(compute_for_activity)
            .collect()
    }
}

/// A contiguous segment of a track that overlaps with the reference
struct OverlapSegment {
    start_idx: usize,
    end_idx: usize,
    distance: f64,
}

/// Find ALL portions of a track that overlap with a reference polyline.
/// Returns all qualifying traversals (for out-and-back, track laps, etc.).
/// Each traversal is returned with (start_index, end_index, direction).
fn find_all_track_portions(
    track: &[GpsPoint],
    reference: &[GpsPoint],
    threshold: f64,
) -> Vec<(usize, usize, Direction)> {
    if track.is_empty() || reference.is_empty() {
        return Vec::new();
    }

    let ref_tree = build_rtree(reference);
    let threshold_deg = threshold / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;
    let ref_length = calculate_route_distance(reference);

    // Find all contiguous overlapping segments
    let mut segments: Vec<OverlapSegment> = Vec::new();
    let mut current_start: Option<usize> = None;
    let mut gap_count = 0;
    const MAX_GAP: usize = 3; // Allow small gaps for GPS noise

    for (i, point) in track.iter().enumerate() {
        let query = [point.latitude, point.longitude];

        let is_near = ref_tree
            .nearest_neighbor(&query)
            .map(|nearest| nearest.distance_2(&query) <= threshold_deg_sq)
            .unwrap_or(false);

        if is_near {
            if current_start.is_none() {
                current_start = Some(i);
            }
            gap_count = 0;
        } else if current_start.is_some() {
            gap_count += 1;
            if gap_count > MAX_GAP {
                // End this segment
                let start = current_start.unwrap();
                let end = i - gap_count;
                if end > start {
                    let distance = calculate_route_distance(&track[start..end]);
                    segments.push(OverlapSegment {
                        start_idx: start,
                        end_idx: end,
                        distance,
                    });
                }
                current_start = None;
                gap_count = 0;
            }
        }
    }

    // Handle final segment
    if let Some(start) = current_start {
        let end = track.len() - gap_count.min(track.len() - start - 1);
        if end > start {
            let distance = calculate_route_distance(&track[start..end]);
            segments.push(OverlapSegment {
                start_idx: start,
                end_idx: end,
                distance,
            });
        }
    }

    if segments.is_empty() {
        return Vec::new();
    }

    // Filter segments: must be at least 50% of section length
    let min_distance = ref_length * 0.5;

    // Collect all qualifying segments with their directions
    let mut results: Vec<(usize, usize, Direction)> = Vec::new();

    for segment in &segments {
        if segment.distance >= min_distance {
            let direction = detect_direction_robust(
                &track[segment.start_idx..segment.end_idx],
                reference,
                &ref_tree,
            );
            results.push((segment.start_idx, segment.end_idx, direction));
        }
    }

    // Sort by start index (time order)
    results.sort_by_key(|r| r.0);

    results
}

/// Detect direction by sampling multiple points along the track and checking
/// their positions on the reference polyline. More robust than just comparing endpoints.
fn detect_direction_robust(
    track_portion: &[GpsPoint],
    reference: &[GpsPoint],
    ref_tree: &RTree<IndexedPoint>,
) -> Direction {
    if track_portion.len() < 3 || reference.len() < 3 {
        return Direction::Same;
    }

    // Sample 5 points along the track portion
    let sample_count = 5.min(track_portion.len());
    let step = track_portion.len() / sample_count;

    let mut ref_indices: Vec<usize> = Vec::with_capacity(sample_count);

    for i in 0..sample_count {
        let track_idx = (i * step).min(track_portion.len() - 1);
        let point = &track_portion[track_idx];
        let query = [point.latitude, point.longitude];

        if let Some(nearest) = ref_tree.nearest_neighbor(&query) {
            ref_indices.push(nearest.idx);
        }
    }

    if ref_indices.len() < 2 {
        return Direction::Same;
    }

    // Count how many times consecutive samples go forward vs backward on the reference
    let mut forward_count = 0;
    let mut backward_count = 0;

    for i in 1..ref_indices.len() {
        let prev_idx = ref_indices[i - 1];
        let curr_idx = ref_indices[i];

        if curr_idx > prev_idx {
            forward_count += 1;
        } else if curr_idx < prev_idx {
            backward_count += 1;
        }
        // Equal indices don't count (could be same point, noise)
    }

    if backward_count > forward_count {
        Direction::Reverse
    } else {
        Direction::Same
    }
}
