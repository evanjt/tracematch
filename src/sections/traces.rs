//! Activity trace extraction from GPS tracks.

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use super::consensus::TraceKey;
use super::rtree::{IndexedPoint, build_rtree};
use crate::GpsPoint;
use rstar::{PointDistance, RTree};

/// Distance threshold for considering a point "on" the section (meters)
const TRACE_PROXIMITY_THRESHOLD: f64 = 50.0;

/// Minimum points to consider a valid overlap trace
const MIN_TRACE_POINTS: usize = 3;

/// Extract every pass a GPS track makes over a section, in track order.
/// An out-and-back route yields two passes, a three-lap interval three.
/// Uses R-tree for efficient O(log n) proximity lookups.
/// Tolerates small gaps (up to 3 points) due to GPS noise.
pub fn extract_activity_passes(
    track: &[GpsPoint],
    section_polyline: &[GpsPoint],
    polyline_tree: &RTree<IndexedPoint>,
) -> Vec<Vec<GpsPoint>> {
    if track.len() < MIN_TRACE_POINTS || section_polyline.len() < 2 {
        return Vec::new();
    }

    // Convert threshold from meters to approximate degrees for R-tree comparison
    // Use a slightly larger threshold to catch GPS variations
    let threshold_deg = (TRACE_PROXIMITY_THRESHOLD * 1.2) / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;

    // Degree-space bounding box of the polyline, padded by the threshold. The
    // near-predicate below is squared-degree distance, so a track point
    // outside this box cannot be near any polyline point: four comparisons
    // answer most points of a long track without touching the R-tree.
    let mut bb = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for p in section_polyline {
        bb.0 = bb.0.min(p.latitude);
        bb.1 = bb.1.max(p.latitude);
        bb.2 = bb.2.min(p.longitude);
        bb.3 = bb.3.max(p.longitude);
    }
    let (lat0, lat1, lng0, lng1) = (
        bb.0 - threshold_deg,
        bb.1 + threshold_deg,
        bb.2 - threshold_deg,
        bb.3 + threshold_deg,
    );

    // Find ALL contiguous sequences of points near the section
    let mut sequences: Vec<Vec<GpsPoint>> = Vec::new();
    let mut current_sequence: Vec<GpsPoint> = Vec::new();
    let mut gap_count = 0;
    const MAX_GAP: usize = super::TRACK_GAP_POINTS;

    for point in track {
        let query = [point.latitude, point.longitude];

        // Use R-tree for O(log n) nearest neighbor lookup
        let is_near = point.latitude >= lat0
            && point.latitude <= lat1
            && point.longitude >= lng0
            && point.longitude <= lng1
            && polyline_tree
                .nearest_neighbor(&query)
                .is_some_and(|nearest| nearest.distance_2(&query) <= threshold_deg_sq);

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

    sequences
}

/// The single pass that best represents a track's run over a section:
/// the longest. Map rendering wants one line, because a LineString over
/// several passes draws a closing straight line between them.
pub fn extract_activity_trace(
    track: &[GpsPoint],
    section_polyline: &[GpsPoint],
    polyline_tree: &RTree<IndexedPoint>,
) -> Vec<GpsPoint> {
    extract_activity_passes(track, section_polyline, polyline_tree)
        .into_iter()
        .max_by_key(|seq| seq.len())
        .unwrap_or_default()
}

/// Extract every pass over a section for all its activities.
///
/// Returns `((activity_id, pass index), overlapping GPS points)` in
/// `activity_ids` order, passes in track order, and activities with no
/// overlap are dropped. The order is part of the contract: the consensus
/// accumulator folds these sequentially, so a caller that received a map
/// would fold in hash order and produce a different line on every run.
/// The pass index is what keeps a lapped activity's traversals distinct.
pub fn extract_all_activity_traces(
    activity_ids: &[String],
    section_polyline: &[GpsPoint],
    track_map: &std::collections::HashMap<&str, &[GpsPoint]>,
) -> Vec<(TraceKey, Vec<GpsPoint>)> {
    let polyline_tree = build_rtree(section_polyline);

    // `filter_map` + `collect` into a `Vec` concatenates rayon's per-thread
    // buffers left to right, so this stays parallel and stays in source order.
    #[cfg(feature = "parallel")]
    let traces: Vec<(TraceKey, Vec<GpsPoint>)> = activity_ids
        .par_iter()
        .flat_map_iter(|activity_id| {
            passes_for(activity_id, section_polyline, &polyline_tree, track_map).into_iter()
        })
        .collect();

    #[cfg(not(feature = "parallel"))]
    let traces: Vec<(TraceKey, Vec<GpsPoint>)> = activity_ids
        .iter()
        .flat_map(|activity_id| {
            passes_for(activity_id, section_polyline, &polyline_tree, track_map)
        })
        .collect();

    traces
}

fn passes_for(
    activity_id: &str,
    section_polyline: &[GpsPoint],
    polyline_tree: &RTree<IndexedPoint>,
    track_map: &std::collections::HashMap<&str, &[GpsPoint]>,
) -> Vec<(TraceKey, Vec<GpsPoint>)> {
    let Some(track) = track_map.get(activity_id) else {
        return Vec::new();
    };
    extract_activity_passes(track, section_polyline, polyline_tree)
        .into_iter()
        .filter(|pass| !pass.is_empty())
        .enumerate()
        .map(|(i, pass)| ((activity_id.to_string(), i as u32), pass))
        .collect()
}

/// Collapse per-pass traces to one line per activity, the longest pass.
/// For callers that draw a single line and cannot show laps separately.
pub fn longest_pass_per_activity(
    traces: Vec<(TraceKey, Vec<GpsPoint>)>,
) -> Vec<(String, Vec<GpsPoint>)> {
    let mut best: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    for ((activity_id, _), pass) in traces {
        match best.iter_mut().find(|(id, _)| *id == activity_id) {
            Some((_, held)) if held.len() < pass.len() => *held = pass,
            Some(_) => {}
            None => best.push((activity_id, pass)),
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sections::rtree::build_rtree;

    fn line(from: f64, to: f64) -> Vec<GpsPoint> {
        let step = if to >= from { 0.0001 } else { -0.0001 };
        let count = ((to - from) / step).round() as i64;
        (0..=count)
            .map(|i| GpsPoint {
                latitude: from + step * i as f64,
                longitude: 7.0,
                elevation: None,
            })
            .collect()
    }

    #[test]
    fn an_out_and_back_track_contributes_two_passes() {
        let section = line(46.0, 46.005);
        let mut track = line(46.0, 46.006);
        track.extend(line(46.006, 45.999));
        let tree = build_rtree(&section);

        let passes = extract_activity_passes(&track, &section, &tree);
        assert_eq!(passes.len(), 2, "an out and back crosses the section twice");

        let track_map = std::collections::HashMap::from([("act", track.as_slice())]);
        let traces = extract_all_activity_traces(&["act".to_string()], &section, &track_map);
        assert_eq!(
            traces.iter().map(|(key, _)| key.1).collect::<Vec<_>>(),
            vec![0, 1],
            "the passes must be keyed by a zero-based index"
        );
        assert_eq!(
            longest_pass_per_activity(traces).len(),
            1,
            "the collapse adapter serves callers that draw a single line"
        );
    }

    #[test]
    fn a_single_pass_track_still_yields_one_trace() {
        let section = line(46.0, 46.005);
        let track = line(45.999, 46.006);
        let tree = build_rtree(&section);
        assert_eq!(extract_activity_passes(&track, &section, &tree).len(), 1);
    }
}
