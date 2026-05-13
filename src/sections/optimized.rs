//! Route-section utilities: find/split sections within a single GPS route.
//!
//! Originally this module also housed `detect_sections_optimized`, a
//! standalone multi-track detector based on a coarse 5 km grid and 100-point
//! downsampling. That path was superseded by the multiscale +
//! `spatial_filter::fine_grid_filtered_pairs` pipeline (in `sections/mod.rs`)
//! which is faster on real data and produces the same (or better) sections.
//!
//! What remains here:
//! - `downsample_track` — shared with the multiscale path.
//! - `find_sections_in_route`, `find_all_section_spans_in_route` and friends —
//!   used to project known sections onto a single new route.
//! - `split_section_at_index`, `split_section_at_point`,
//!   `recalculate_section_polyline` — section manipulation primitives.

use super::{FrequentSection, SectionConfig};
use crate::GpsPoint;
use crate::geo_utils::haversine_distance;
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Downsample a track to approximately target_points
pub(crate) fn downsample_track(track: &[GpsPoint], target_points: usize) -> Vec<GpsPoint> {
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

/// A section match found within a route
#[derive(Debug, Clone, Serialize, Deserialize)]
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
/// Supports multiple traversals of the same section (out-and-back, track laps).
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

    let threshold = config.proximity_threshold * 2.0;

    let find_matches_for_section = |section: &FrequentSection| -> Vec<SectionMatch> {
        if section.polyline.is_empty() {
            return Vec::new();
        }

        let all_spans = find_all_section_spans_in_route(route, &section.polyline, threshold);
        all_spans
            .into_iter()
            .map(|(start, end, quality, same_dir)| SectionMatch {
                section_id: section.id.clone(),
                start_index: start as u64,
                end_index: end as u64,
                match_quality: quality,
                same_direction: same_dir,
            })
            .collect()
    };

    #[cfg(feature = "parallel")]
    let mut matches: Vec<SectionMatch> = sections
        .par_iter()
        .flat_map(find_matches_for_section)
        .collect();

    #[cfg(not(feature = "parallel"))]
    let mut matches: Vec<SectionMatch> =
        sections.iter().flat_map(find_matches_for_section).collect();

    // Sort by start index
    matches.sort_by_key(|m| m.start_index);
    matches
}

/// Find ALL places where a section polyline appears in a route.
/// Returns Vec of (start_index, end_index, quality, same_direction).
/// Supports multiple traversals (out-and-back, track laps).
/// Used by both auto-detected sections and custom sections.
pub fn find_all_section_spans_in_route(
    route: &[GpsPoint],
    section: &[GpsPoint],
    threshold: f64,
) -> Vec<(usize, usize, f64, bool)> {
    if route.len() < 3 || section.len() < 3 {
        return Vec::new();
    }

    // Try both directions
    let forward_spans = find_all_section_spans_directed(route, section, threshold);
    let reversed: Vec<_> = section.iter().rev().cloned().collect();
    let backward_spans = find_all_section_spans_directed(route, &reversed, threshold);

    // Combine results with direction flag
    let mut all_spans: Vec<(usize, usize, f64, bool)> = Vec::new();

    for (start, end, quality) in forward_spans {
        all_spans.push((start, end, quality, true));
    }

    for (start, end, quality) in backward_spans {
        all_spans.push((start, end, quality, false));
    }

    // Sort by start index and dedupe overlapping spans
    all_spans.sort_by_key(|s| s.0);
    dedupe_overlapping_spans(all_spans)
}

/// Remove overlapping spans, keeping the higher quality one when they conflict.
fn dedupe_overlapping_spans(
    spans: Vec<(usize, usize, f64, bool)>,
) -> Vec<(usize, usize, f64, bool)> {
    if spans.is_empty() {
        return spans;
    }

    let mut result: Vec<(usize, usize, f64, bool)> = Vec::new();

    for span in spans {
        let overlaps_existing = result.iter().any(|existing| {
            // Check if ranges overlap
            span.0 < existing.1 && span.1 > existing.0
        });

        if !overlaps_existing {
            result.push(span);
        }
        // If it overlaps, we skip it (first one wins since sorted by start)
    }

    result
}

/// Find ALL section spans in one direction.
/// Returns Vec of (start_index, end_index, quality).
fn find_all_section_spans_directed(
    route: &[GpsPoint],
    section: &[GpsPoint],
    threshold: f64,
) -> Vec<(usize, usize, f64)> {
    let section_start = &section[0];
    let section_end = match section.last() {
        Some(e) => e,
        None => return Vec::new(),
    };

    // Find ALL points near section start
    let mut start_candidates: Vec<(usize, f64)> = Vec::new();
    for (i, point) in route.iter().enumerate() {
        let dist = haversine_distance(point, section_start);
        if dist < threshold {
            start_candidates.push((i, dist));
        }
    }

    if start_candidates.is_empty() {
        return Vec::new();
    }

    let mut results: Vec<(usize, usize, f64)> = Vec::new();
    let mut used_ranges: Vec<(usize, usize)> = Vec::new();

    // For each start candidate, try to find a valid end point
    for (start_idx, _start_dist) in &start_candidates {
        let start_idx = *start_idx;

        // Skip if this start is already covered by a previous match
        if used_ranges
            .iter()
            .any(|(s, e)| start_idx >= *s && start_idx < *e)
        {
            continue;
        }

        // Find the closest point to section end (after start)
        let mut best_end_idx = None;
        let mut best_end_dist = f64::MAX;

        for (i, point) in route.iter().enumerate().skip(start_idx + 1) {
            // Stop searching if we hit another start candidate (next traversal)
            if i > start_idx + 5 {
                // Allow some buffer for GPS noise
                let is_another_start = start_candidates
                    .iter()
                    .any(|(si, _)| *si == i && *si > start_idx);
                if is_another_start && best_end_idx.is_some() {
                    break;
                }
            }

            let dist = haversine_distance(point, section_end);
            if dist < threshold && dist < best_end_dist {
                best_end_dist = dist;
                best_end_idx = Some(i);
            }
        }

        let end_idx = match best_end_idx {
            Some(idx) => idx,
            None => continue, // No valid end found
        };

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

        // Require at least 50% match (lowered from 60% to reduce missed matches from GPS drift)
        if quality >= 0.5 {
            results.push((start_idx, end_idx + 1, quality));
            used_ranges.push((start_idx, end_idx + 1));
        }
    }

    results
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
            // Lazy: traces need re-extraction for the new polyline
            activity_traces: HashMap::new(),
            confidence: section.confidence * 0.9, // Slight reduction
            observation_count: section.observation_count,
            average_spread: section.average_spread,
            point_density: vec![], // Need recalculation
            scale: section.scale,
            is_user_defined: true, // Mark as user-modified
            stability: 0.0,        // Needs recalculation
            version: 1,
            updated_at: None,
            created_at: section.created_at.clone(),
            consensus_state: None,
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
            activity_traces: HashMap::new(),
            confidence: section.confidence * 0.9,
            observation_count: section.observation_count,
            average_spread: section.average_spread,
            point_density: vec![],
            scale: section.scale,
            is_user_defined: true,
            stability: 0.0, // Needs recalculation
            version: 1,
            updated_at: None,
            created_at: section.created_at.clone(),
            consensus_state: None,
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

    // Recompute stability of representative against new consensus
    let stability = section
        .activity_traces
        .get(&section.representative_activity_id)
        .map(|trace| {
            super::medoid::compute_stability(trace, &consensus.polyline, config.proximity_threshold)
        })
        .unwrap_or(section.stability);

    FrequentSection {
        polyline: consensus.polyline,
        distance_meters: new_distance,
        average_spread: consensus.average_spread,
        point_density: consensus.point_density,
        confidence: consensus.confidence,
        observation_count: consensus.observation_count,
        version: section.version + 1,
        stability,
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
}
