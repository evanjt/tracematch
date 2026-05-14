//! Activity portion computation for pace comparison.

use super::overlap::OverlapCluster;
use super::rtree::{IndexedPoint, build_rtree};
use super::{SectionConfig, SectionPortion};
use crate::geo_utils::haversine_distance;
use crate::matching::calculate_route_distance;
use crate::{Direction, GpsPoint};
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
    // Sort so iteration order is stable across runs (HashSet iteration
    // is randomized, which propagated into section detection output
    // non-determinism).
    let mut activity_ids: Vec<&String> = cluster.activity_ids.iter().collect();
    activity_ids.sort();

    portions_for_sorted_ids(
        &activity_ids,
        representative_polyline,
        all_tracks,
        config.proximity_threshold,
    )
}

/// Compute portions for a flat list of activity_ids against a reference polyline.
///
/// Used by corridor and flow_graph detection paths, which already know
/// cluster membership from their own pipelines and don't need an
/// `OverlapCluster`. Populates `activity_portions` so the section detail
/// screen can show per-activity start/end indices and distances.
pub fn compute_portions_for_activities(
    activity_ids: &[String],
    reference_polyline: &[GpsPoint],
    all_tracks: &std::collections::HashMap<&str, &[GpsPoint]>,
    threshold: f64,
) -> Vec<SectionPortion> {
    let mut sorted_ids: Vec<&String> = activity_ids.iter().collect();
    sorted_ids.sort();
    portions_for_sorted_ids(&sorted_ids, reference_polyline, all_tracks, threshold)
}

fn portions_for_sorted_ids(
    sorted_ids: &[&String],
    reference_polyline: &[GpsPoint],
    all_tracks: &std::collections::HashMap<&str, &[GpsPoint]>,
    threshold: f64,
) -> Vec<SectionPortion> {
    let compute_for_activity = |activity_id: &&String| -> Vec<SectionPortion> {
        let mut portions = Vec::new();
        if let Some(track) = all_tracks.get(activity_id.as_str()) {
            let all_traversals = find_all_track_portions(track, reference_polyline, threshold);

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
        sorted_ids
            .par_iter()
            .flat_map(compute_for_activity)
            .collect()
    }

    #[cfg(not(feature = "parallel"))]
    {
        sorted_ids.iter().flat_map(compute_for_activity).collect()
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
/// Uses a default gap tolerance of 3 points for GPS noise.
pub fn find_all_track_portions(
    track: &[GpsPoint],
    reference: &[GpsPoint],
    threshold: f64,
) -> Vec<(usize, usize, Direction)> {
    find_all_track_portions_with_gap(track, reference, threshold, 3)
}

/// Like `find_all_track_portions`, but with a configurable gap tolerance.
/// A lower `max_gap` (e.g. 1) produces stricter matching — useful when changing
/// a section's reference activity to avoid including parallel roads.
pub fn find_all_track_portions_with_gap(
    track: &[GpsPoint],
    reference: &[GpsPoint],
    threshold: f64,
    max_gap: usize,
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
            if gap_count > max_gap {
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

    let min_distance = ref_length * 0.5;
    let mut results: Vec<(usize, usize, Direction)> = Vec::new();

    for segment in &segments {
        // Try to split into individual laps (handles out-and-back, loops)
        let laps = split_segment_into_laps(track, segment, reference, &ref_tree, ref_length);
        if !laps.is_empty() {
            results.extend(laps);
        } else if segment.distance >= min_distance {
            // Fallback: single traversal (original behavior)
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

// ---------------------------------------------------------------------------
// Lap splitting: detect individual passes within a contiguous overlap segment
// ---------------------------------------------------------------------------

/// Apply a 5-point moving median to smooth a sequence of reference indices.
fn moving_median(values: &[usize], window: usize) -> Vec<usize> {
    let half = window / 2;
    values
        .iter()
        .enumerate()
        .map(|(i, _)| {
            let start = i.saturating_sub(half);
            let end = (i + half + 1).min(values.len());
            let mut buf: Vec<usize> = values[start..end].to_vec();
            buf.sort_unstable();
            buf[buf.len() / 2]
        })
        .collect()
}

/// Split a contiguous overlap segment into individual laps.
/// Handles both linear sections (out-and-back) and loop sections (ovals/tracks).
/// Returns empty if no multi-lap pattern is detected (caller should fall back).
fn split_segment_into_laps(
    track: &[GpsPoint],
    segment: &OverlapSegment,
    reference: &[GpsPoint],
    ref_tree: &RTree<IndexedPoint>,
    ref_length: f64,
) -> Vec<(usize, usize, Direction)> {
    let seg_len = segment.end_idx - segment.start_idx;
    if seg_len < 10 || reference.len() < 5 {
        return Vec::new();
    }

    // 1. Map each track point in segment to nearest reference index
    let ref_indices: Vec<usize> = (segment.start_idx..segment.end_idx)
        .map(|i| {
            let point = &track[i];
            let query = [point.latitude, point.longitude];
            ref_tree
                .nearest_neighbor(&query)
                .map(|n| n.idx)
                .unwrap_or(0)
        })
        .collect();

    // 2. Smooth with 5-point moving median
    let smoothed = moving_median(&ref_indices, 5);

    // 3. Determine if reference is a loop (start ≈ end within 100m)
    let is_loop = reference.len() >= 10
        && haversine_distance(&reference[0], &reference[reference.len() - 1]) < 100.0;

    // 4. Find lap boundary positions within the smoothed array
    let boundary_positions = if is_loop {
        find_loop_boundaries(&smoothed, reference, ref_length)
    } else {
        find_linear_turning_points(&smoothed, reference.len())
    };

    // No boundaries = single pass (or couldn't detect laps)
    if boundary_positions.is_empty() {
        return Vec::new();
    }

    // 5. Build sub-segments from boundary positions
    let min_distance = ref_length * 0.5;
    let mut results = Vec::new();

    let mut split_points = Vec::with_capacity(boundary_positions.len() + 2);
    split_points.push(0usize);
    split_points.extend_from_slice(&boundary_positions);
    split_points.push(seg_len);

    for window in split_points.windows(2) {
        let local_start = window[0];
        let local_end = window[1];

        if local_end <= local_start + 3 {
            continue;
        }

        let track_start = segment.start_idx + local_start;
        let track_end = segment.start_idx + local_end;

        let distance = calculate_route_distance(&track[track_start..track_end]);
        if distance >= min_distance {
            let direction =
                detect_direction_robust(&track[track_start..track_end], reference, ref_tree);
            results.push((track_start, track_end, direction));
        }
    }

    results
}

/// Find turning points in a reference index sequence for a linear (non-loop) section.
/// Detects peaks and valleys where the runner reverses direction.
/// Returns positions within the smoothed array where laps change direction.
fn find_linear_turning_points(smoothed: &[usize], ref_len: usize) -> Vec<usize> {
    if smoothed.len() < 10 || ref_len < 5 {
        return Vec::new();
    }

    // Minimum index change to consider a direction reversal significant
    let min_change = ((ref_len as f64) * 0.3) as usize;
    let min_change = min_change.max(3);

    // Determine initial trend from first significant movement
    let mut searching_for_peak = true;
    for val in smoothed.iter().skip(1) {
        let diff = *val as i64 - smoothed[0] as i64;
        if diff > min_change as i64 / 3 {
            searching_for_peak = true;
            break;
        } else if diff < -(min_change as i64 / 3) {
            searching_for_peak = false;
            break;
        }
    }

    let mut turns = Vec::new();
    let mut best_val = smoothed[0];
    let mut best_pos = 0usize;

    for (i, &val) in smoothed.iter().enumerate().skip(1) {
        if searching_for_peak {
            // Rising — track the running maximum
            if val > best_val {
                best_val = val;
                best_pos = i;
            }
            // Fallen significantly from peak → direction reversed
            if best_val > val && best_val - val >= min_change {
                turns.push(best_pos);
                searching_for_peak = false;
                best_val = val;
                best_pos = i;
            }
        } else {
            // Falling — track the running minimum
            if val < best_val {
                best_val = val;
                best_pos = i;
            }
            // Risen significantly from valley → direction reversed
            if val > best_val && val - best_val >= min_change {
                turns.push(best_pos);
                searching_for_peak = true;
                best_val = val;
                best_pos = i;
            }
        }
    }

    turns
}

/// Find lap boundaries for a loop section based on cumulative arc distance.
/// Returns positions within the smoothed array where new laps start.
fn find_loop_boundaries(smoothed: &[usize], reference: &[GpsPoint], ref_length: f64) -> Vec<usize> {
    if smoothed.len() < 5 || reference.len() < 3 {
        return Vec::new();
    }

    // Precompute cumulative distances along reference
    let mut cum_dist = Vec::with_capacity(reference.len());
    cum_dist.push(0.0);
    for i in 1..reference.len() {
        cum_dist.push(cum_dist[i - 1] + haversine_distance(&reference[i - 1], &reference[i]));
    }

    let total_dist = *cum_dist.last().unwrap_or(&0.0);
    if total_dist < 1.0 {
        return Vec::new();
    }

    let lap_threshold = ref_length * 0.8;
    let mut accumulated = 0.0;
    let mut boundaries = Vec::new();
    let max_ref_idx = reference.len() - 1;

    for i in 1..smoothed.len() {
        let idx_prev = smoothed[i - 1].min(max_ref_idx);
        let idx_curr = smoothed[i].min(max_ref_idx);

        let diff = (cum_dist[idx_curr] - cum_dist[idx_prev]).abs();
        // For a loop: shortest arc handles wrap-around
        let arc = diff.min(total_dist - diff);

        accumulated += arc;

        if accumulated >= lap_threshold {
            boundaries.push(i);
            accumulated -= lap_threshold; // carry over excess
        }
    }

    boundaries
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Create a linear section reference (~500m north-south at 45°N)
    fn make_linear_reference(num_points: usize) -> Vec<GpsPoint> {
        // 0.0045 degrees latitude ≈ 500m
        let length_degrees = 0.0045;
        (0..num_points)
            .map(|i| {
                GpsPoint::new(
                    45.0 + (i as f64 / (num_points - 1) as f64) * length_degrees,
                    7.0,
                )
            })
            .collect()
    }

    /// Create a track that goes forward/reverse over the reference for `passes` passes
    fn make_out_and_back_track(
        reference: &[GpsPoint],
        passes: usize,
        points_per_pass: usize,
    ) -> Vec<GpsPoint> {
        let mut track = Vec::new();
        for pass in 0..passes {
            let forward = pass % 2 == 0;
            for i in 0..points_per_pass {
                let frac = i as f64 / (points_per_pass - 1) as f64;
                let ref_frac = if forward { frac } else { 1.0 - frac };
                let ref_idx = (ref_frac * (reference.len() - 1) as f64).round() as usize;
                let ref_point = &reference[ref_idx.min(reference.len() - 1)];
                track.push(GpsPoint::new(
                    ref_point.latitude + 0.000005 * (pass as f64), // tiny offset
                    ref_point.longitude,
                ));
            }
        }
        track
    }

    /// Create a circular loop reference (~400m circumference at 45°N)
    fn make_loop_reference(num_points: usize) -> Vec<GpsPoint> {
        // radius in degrees: circumference = 2πr, want 400m → r ≈ 63.7m ≈ 0.000573°
        let radius_deg = 0.000573;
        let center_lat = 45.0;
        let center_lng = 7.0;
        let mut points: Vec<GpsPoint> = (0..num_points)
            .map(|i| {
                let angle = 2.0 * std::f64::consts::PI * (i as f64) / (num_points as f64);
                GpsPoint::new(
                    center_lat + radius_deg * angle.cos(),
                    center_lng + radius_deg * angle.sin(),
                )
            })
            .collect();
        // Close the loop: last point ≈ first point
        if let Some(first) = points.first() {
            points.push(GpsPoint::new(first.latitude, first.longitude));
        }
        points
    }

    /// Create a track that goes around a loop multiple times
    fn make_loop_track(
        reference: &[GpsPoint],
        laps: usize,
        points_per_lap: usize,
    ) -> Vec<GpsPoint> {
        let ref_len = reference.len() - 1; // exclude the closing duplicate
        let mut track = Vec::new();
        for lap in 0..laps {
            for i in 0..points_per_lap {
                let frac = i as f64 / points_per_lap as f64;
                let ref_idx = (frac * ref_len as f64).round() as usize;
                let ref_point = &reference[ref_idx.min(ref_len)];
                track.push(GpsPoint::new(
                    ref_point.latitude + 0.000002 * (lap as f64),
                    ref_point.longitude,
                ));
            }
        }
        track
    }

    #[test]
    fn test_linear_out_and_back() {
        let reference = make_linear_reference(50);
        let track = make_out_and_back_track(&reference, 2, 80);

        let results = find_all_track_portions(&track, &reference, 50.0);

        assert_eq!(
            results.len(),
            2,
            "Expected 2 traversals (1 forward, 1 reverse), got {}",
            results.len()
        );
        assert_eq!(results[0].2, Direction::Same);
        assert_eq!(results[1].2, Direction::Reverse);
    }

    #[test]
    fn test_linear_multi_pass() {
        let reference = make_linear_reference(50);
        let track = make_out_and_back_track(&reference, 5, 80);

        let results = find_all_track_portions(&track, &reference, 50.0);

        assert_eq!(
            results.len(),
            5,
            "Expected 5 traversals, got {}",
            results.len()
        );
        let forward_count = results.iter().filter(|r| r.2 == Direction::Same).count();
        let reverse_count = results.iter().filter(|r| r.2 == Direction::Reverse).count();
        assert_eq!(forward_count, 3, "Expected 3 forward passes");
        assert_eq!(reverse_count, 2, "Expected 2 reverse passes");
    }

    #[test]
    fn test_loop_three_laps() {
        let reference = make_loop_reference(50);
        let track = make_loop_track(&reference, 3, 80);

        let results = find_all_track_portions(&track, &reference, 50.0);

        assert!(
            results.len() >= 2,
            "Expected at least 2 laps for 3 circuits, got {}",
            results.len()
        );
        assert!(
            results.len() <= 4,
            "Expected at most 4 laps for 3 circuits, got {}",
            results.len()
        );
    }

    #[test]
    fn test_single_pass_unchanged() {
        let reference = make_linear_reference(50);
        let track = make_out_and_back_track(&reference, 1, 80);

        let results = find_all_track_portions(&track, &reference, 50.0);

        assert_eq!(results.len(), 1, "Single pass should produce 1 result");
        assert_eq!(results[0].2, Direction::Same);
    }

    #[test]
    fn test_gps_noise_no_false_splits() {
        let reference = make_linear_reference(50);
        // Single forward pass with GPS jitter
        let mut track = make_out_and_back_track(&reference, 1, 80);
        // Add jitter: every 5th point gets a small lat offset
        for (i, point) in track.iter_mut().enumerate() {
            if i % 5 == 0 {
                point.latitude += 0.00002; // ~2m jitter
            }
        }

        let results = find_all_track_portions(&track, &reference, 50.0);

        assert_eq!(
            results.len(),
            1,
            "GPS noise should not cause false splits, got {} results",
            results.len()
        );
    }

    #[test]
    fn test_moving_median() {
        // Basic smoothing test
        let values = vec![0, 1, 5, 2, 3, 4, 8, 5, 6];
        let result = moving_median(&values, 5);
        // Middle values should be smoothed; the spike at index 2 (5) should be reduced
        assert_eq!(result[0], 1); // median of [0, 1, 5] = 1
        assert!(result[2] <= 3); // spike at 5 smoothed down
    }

    #[test]
    fn test_linear_turning_points() {
        // Simulate forward then reverse: indices go 0→49 then 49→0
        let mut smoothed = Vec::new();
        for i in 0..50 {
            smoothed.push(i);
        }
        for i in (0..50).rev() {
            smoothed.push(i);
        }

        let turns = find_linear_turning_points(&smoothed, 50);

        assert_eq!(
            turns.len(),
            1,
            "Expected 1 turning point, got {}",
            turns.len()
        );
        // Turning point should be near index 49 (the peak)
        assert!(
            turns[0] >= 45 && turns[0] <= 55,
            "Turning point at {} should be near the peak",
            turns[0]
        );
    }
}
