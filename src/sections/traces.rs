//! Activity trace extraction from GPS tracks.

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use super::rtree::{IndexedPoint, build_rtree};
use crate::GpsPoint;
use rstar::{PointDistance, RTree};
use std::collections::HashMap;

/// Distance threshold for considering a point "on" the section (meters)
const TRACE_PROXIMITY_THRESHOLD: f64 = 50.0;

/// Minimum points to consider a valid overlap trace
const MIN_TRACE_POINTS: usize = 3;

/// Extract the portion(s) of a GPS track that overlap with a section.
/// Returns ALL passes over the section (not just the longest) merged together.
/// This handles out-and-back routes where the activity crosses the section twice.
/// Uses R-tree for efficient O(log n) proximity lookups.
/// Tolerates small gaps (up to 3 points) due to GPS noise.
fn extract_activity_trace(
    track: &[GpsPoint],
    section_polyline: &[GpsPoint],
    polyline_tree: &RTree<IndexedPoint>,
) -> Vec<GpsPoint> {
    if track.len() < MIN_TRACE_POINTS || section_polyline.len() < 2 {
        return Vec::new();
    }

    // Convert threshold from meters to approximate degrees for R-tree comparison
    // Use a slightly larger threshold to catch GPS variations
    let threshold_deg = (TRACE_PROXIMITY_THRESHOLD * 1.2) / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;

    // Find ALL contiguous sequences of points near the section
    let mut sequences: Vec<Vec<GpsPoint>> = Vec::new();
    let mut current_sequence: Vec<GpsPoint> = Vec::new();
    let mut gap_count = 0;
    const MAX_GAP: usize = 3; // Allow small gaps due to GPS noise

    for point in track {
        let query = [point.latitude, point.longitude];

        // Use R-tree for O(log n) nearest neighbor lookup
        let is_near = if let Some(nearest) = polyline_tree.nearest_neighbor(&query) {
            nearest.distance_2(&query) <= threshold_deg_sq
        } else {
            false
        };

        if is_near {
            // Point is near section - reset gap counter
            gap_count = 0;
            current_sequence.push(*point);
        } else {
            gap_count += 1;
            // Allow small gaps but still add the point if we're in a sequence
            if gap_count <= MAX_GAP && !current_sequence.is_empty() {
                current_sequence.push(*point);
            } else if gap_count > MAX_GAP {
                // End current sequence if valid
                if current_sequence.len() >= MIN_TRACE_POINTS {
                    sequences.push(std::mem::take(&mut current_sequence));
                } else {
                    current_sequence.clear();
                }
                gap_count = 0;
            }
        }
    }

    // Don't forget the last sequence
    if current_sequence.len() >= MIN_TRACE_POINTS {
        sequences.push(current_sequence);
    }

    // Return only the FIRST (longest) sequence to avoid "straight line" artifacts
    // When multiple passes are merged, the LineString draws a closing line from
    // the end of pass 1 to start of pass 2, creating an unwanted straight line.
    // Each pass should be treated as a separate lap, handled at the UI layer.
    if sequences.is_empty() {
        return Vec::new();
    }

    // Return the longest sequence (most representative of the actual path)
    sequences
        .into_iter()
        .max_by_key(|seq| seq.len())
        .unwrap_or_default()
}

/// Extract activity traces for all activities in a section.
/// Returns a map of activity_id -> overlapping GPS points
pub fn extract_all_activity_traces(
    activity_ids: &[String],
    section_polyline: &[GpsPoint],
    track_map: &HashMap<String, Vec<GpsPoint>>,
) -> HashMap<String, Vec<GpsPoint>> {
    let polyline_tree = build_rtree(section_polyline);

    #[cfg(feature = "parallel")]
    let traces: HashMap<String, Vec<GpsPoint>> = activity_ids
        .par_iter()
        .filter_map(|activity_id| {
            track_map.get(activity_id).and_then(|track| {
                let trace = extract_activity_trace(track, section_polyline, &polyline_tree);
                if trace.is_empty() {
                    None
                } else {
                    Some((activity_id.clone(), trace))
                }
            })
        })
        .collect();

    #[cfg(not(feature = "parallel"))]
    let traces: HashMap<String, Vec<GpsPoint>> = activity_ids
        .iter()
        .filter_map(|activity_id| {
            track_map.get(activity_id).and_then(|track| {
                let trace = extract_activity_trace(track, section_polyline, &polyline_tree);
                if trace.is_empty() {
                    None
                } else {
                    Some((activity_id.clone(), trace))
                }
            })
        })
        .collect();

    traces
}
