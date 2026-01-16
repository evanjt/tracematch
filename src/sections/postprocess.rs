//! Post-processing for sections: folding detection, merging, deduplication, and density-based splitting.
//!
//! Based on concepts from:
//! - TRACLUS: "Trajectory Clustering: A Partition-and-Group Framework" (Lee, Han, Whang 2007)
//!   https://hanj.cs.illinois.edu/pdf/sigmod07_jglee.pdf
//! - GPS Segment Averaging (MDPI 2019)
//!   https://mdpi.com/2076-3417/9/22/4899/htm

use super::rtree::{build_rtree, IndexedPoint};
use super::{FrequentSection, SectionConfig};
use crate::geo_utils::haversine_distance;
use crate::matching::calculate_route_distance;
use crate::GpsPoint;
use log::info;
use rstar::{PointDistance, RTree};
use std::collections::HashMap;

// =============================================================================
// Self-Folding Section Detection
// =============================================================================

/// Detect if a polyline folds back on itself (out-and-back pattern).
/// Returns the index of the fold point if found, or None if no fold.
fn detect_fold_point(polyline: &[GpsPoint], threshold: f64) -> Option<usize> {
    if polyline.len() < 10 {
        return None;
    }

    let threshold_deg = threshold / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;

    // Build R-tree of the first half of the polyline
    let half = polyline.len() / 2;
    let first_half_tree = build_rtree(&polyline[..half]);

    // Check each point in the second half against the first half
    // Looking for where the track returns close to earlier points
    let mut fold_candidates: Vec<(usize, f64)> = Vec::new();

    for (i, point) in polyline[half..].iter().enumerate() {
        let idx = half + i;
        let query = [point.latitude, point.longitude];

        if let Some(nearest) = first_half_tree.nearest_neighbor(&query) {
            let dist_sq = nearest.distance_2(&query);
            if dist_sq <= threshold_deg_sq {
                // This point is close to an earlier point - potential fold
                // Track the earliest point where this happens
                fold_candidates.push((idx, dist_sq));
            }
        }
    }

    // Find the first substantial fold (where a sequence of points return)
    // We want the point where the track genuinely turns back, not random noise
    if fold_candidates.len() >= 3 {
        // The fold point is approximately where the return starts
        // Use the first candidate that has at least 2 more following candidates
        Some(fold_candidates[0].0)
    } else {
        None
    }
}

/// Check if a section is "folding" - meaning it goes out and comes back
/// on essentially the same path. Returns fold ratio (0.0 = no fold, 1.0 = perfect fold)
fn compute_fold_ratio(polyline: &[GpsPoint], threshold: f64) -> f64 {
    if polyline.len() < 6 {
        return 0.0;
    }

    let threshold_deg = threshold / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;

    // Compare first third to last third (reversed)
    let third = polyline.len() / 3;
    let first_third = &polyline[..third];
    let last_third: Vec<GpsPoint> = polyline[(polyline.len() - third)..].to_vec();

    // Build tree from first third
    let first_tree = build_rtree(first_third);

    // Count how many points in last third are close to points in first third
    let mut close_count = 0;
    for point in last_third.iter().rev() {
        // Reversed order for out-and-back
        let query = [point.latitude, point.longitude];
        if let Some(nearest) = first_tree.nearest_neighbor(&query) {
            if nearest.distance_2(&query) <= threshold_deg_sq {
                close_count += 1;
            }
        }
    }

    close_count as f64 / third as f64
}

/// Process a single section for fold splitting.
/// Returns a vec of 1-2 sections (or original if no split).
fn process_fold_section(section: FrequentSection, config: &SectionConfig) -> Vec<FrequentSection> {
    let fold_ratio = compute_fold_ratio(&section.polyline, config.proximity_threshold);

    if fold_ratio > 0.5 {
        // This section folds back on itself - split it
        if let Some(fold_idx) = detect_fold_point(&section.polyline, config.proximity_threshold) {
            let mut result = Vec::new();

            // Create outbound section (start to fold point)
            let outbound_polyline = section.polyline[..fold_idx].to_vec();
            let outbound_length = calculate_route_distance(&outbound_polyline);

            if outbound_length >= config.min_section_length {
                let mut outbound = section.clone();
                outbound.id = format!("{}_out", section.id);
                outbound.polyline = outbound_polyline;
                outbound.distance_meters = outbound_length;
                // Update activity traces to only include outbound portion
                outbound.activity_traces = HashMap::new(); // Will be recomputed
                result.push(outbound);
            }

            // Create return section (fold point to end)
            let return_polyline = section.polyline[fold_idx..].to_vec();
            let return_length = calculate_route_distance(&return_polyline);

            if return_length >= config.min_section_length {
                let mut return_section = section.clone();
                return_section.id = format!("{}_ret", section.id);
                return_section.polyline = return_polyline;
                return_section.distance_meters = return_length;
                return_section.activity_traces = HashMap::new();
                result.push(return_section);
            }

            info!(
                "[Sections] Split folding section {} at index {} (fold_ratio={:.2})",
                section.id, fold_idx, fold_ratio
            );

            if result.is_empty() {
                // Both parts were too short, keep original
                vec![section]
            } else {
                result
            }
        } else {
            // Couldn't find fold point, keep original
            vec![section]
        }
    } else {
        // Not folding, keep as-is
        vec![section]
    }
}

/// Split sections that fold back on themselves into separate one-way sections.
/// For out-and-back routes, this creates two sections: outbound and return.
pub fn split_folding_sections(
    sections: Vec<FrequentSection>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        sections
            .into_par_iter()
            .flat_map(|section| process_fold_section(section, config))
            .collect()
    }

    #[cfg(not(feature = "parallel"))]
    {
        sections
            .into_iter()
            .flat_map(|section| process_fold_section(section, config))
            .collect()
    }
}

// =============================================================================
// Heading-Based Section Splitting
// =============================================================================

/// Minimum heading change (degrees) to trigger a split.
/// Set high (60°) to only split at major direction changes, not gentle curves.
const MIN_HEADING_CHANGE: f64 = 60.0;

/// Minimum distance (meters) the new heading must be sustained.
const MIN_SUSTAIN_DISTANCE: f64 = 100.0;

/// Window size (meters) for computing average bearing.
const BEARING_WINDOW_METERS: f64 = 50.0;

/// Maximum bearing variance (degrees) within a window to consider it "consistent".
const MAX_BEARING_VARIANCE: f64 = 25.0;

/// Minimum segment length (meters) after splitting.
/// Keeps sections meaningful - no tiny fragments.
const MIN_HEADING_SPLIT_LENGTH: f64 = 300.0;

/// Minimum section length (meters) to consider for splitting.
/// Sections shorter than this are kept whole to avoid over-fragmentation.
///
/// Based on MDL principle from TRACLUS (Lee, Han, Whang 2007):
/// "Only split if it improves the description" - splitting short sections
/// rarely improves quality. Reference: https://hanj.cs.illinois.edu/pdf/sigmod07_jglee.pdf
const MIN_SECTION_FOR_SPLITTING: f64 = 500.0;

/// Process a single section for heading-based splitting.
fn process_heading_section(section: FrequentSection) -> Vec<FrequentSection> {
    let polyline = &section.polyline;

    // Guard 1: Skip splitting sections that are already short
    // Based on MDL principle - splitting short sections rarely improves quality
    if section.distance_meters < MIN_SECTION_FOR_SPLITTING {
        return vec![section];
    }

    // Need at least enough points to compute windows
    if polyline.len() < 10 {
        return vec![section];
    }

    // Find split points
    let split_indices = find_heading_inflection_points(polyline);

    if split_indices.is_empty() {
        return vec![section];
    }

    // Split the section at each inflection point
    let mut segments: Vec<(usize, usize)> = Vec::new();
    let mut start_idx = 0;

    for &split_idx in &split_indices {
        if split_idx > start_idx {
            segments.push((start_idx, split_idx));
        }
        start_idx = split_idx;
    }

    // Add final segment
    if start_idx < polyline.len() {
        segments.push((start_idx, polyline.len()));
    }

    // Guard 2: Check ratio - don't split if smallest fragment < 40% of original
    // This prevents 200m → 100m + 100m type splits that create tiny pieces
    let segment_lengths: Vec<f64> = segments
        .iter()
        .map(|&(s, e)| calculate_route_distance(&polyline[s..e]))
        .collect();
    let min_segment_length = segment_lengths.iter().cloned().fold(f64::MAX, f64::min);
    let split_ratio = min_segment_length / section.distance_meters;

    if split_ratio < 0.4 && segments.len() > 1 {
        // Splitting would create unbalanced fragments - keep original
        return vec![section];
    }

    // Create new sections from segments
    let mut result = Vec::new();
    let mut split_count = 0;
    for (idx, &(seg_start, seg_end)) in segments.iter().enumerate() {
        let seg_polyline = polyline[seg_start..seg_end].to_vec();
        let seg_length = segment_lengths[idx];

        if seg_length >= MIN_HEADING_SPLIT_LENGTH && seg_polyline.len() >= 3 {
            let mut new_section = section.clone();
            new_section.id = format!("{}_h{}", section.id, split_count);
            new_section.polyline = seg_polyline;
            new_section.distance_meters = seg_length;
            new_section.activity_traces = HashMap::new();
            result.push(new_section);
            split_count += 1;
        }
    }

    if split_count > 1 {
        info!(
            "[Sections] Split {} at {} heading inflection points -> {} segments",
            section.id,
            split_indices.len(),
            split_count
        );
    }

    if result.is_empty() {
        // No valid splits produced, keep original
        vec![section]
    } else {
        result
    }
}

/// Split sections at sustained heading inflection points.
///
/// A section is split where direction changes by >= 60° AND the new direction
/// persists for at least 100 meters. This prevents splitting at single sharp
/// turns while catching true direction changes.
///
/// Uses MDL-inspired guards to prevent over-segmentation:
/// - Skips sections already < 500m (splitting rarely helps)
/// - Ensures all fragments are >= 300m
///
/// Reference: TS-MF Algorithm (Xu et al. 2022) - "avoid excessive segmentation"
/// https://www.hindawi.com/journals/wcmc/2022/9540944/
pub fn split_at_heading_changes(
    sections: Vec<FrequentSection>,
    _config: &SectionConfig,
) -> Vec<FrequentSection> {
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        sections
            .into_par_iter()
            .flat_map(process_heading_section)
            .collect()
    }

    #[cfg(not(feature = "parallel"))]
    {
        sections
            .into_iter()
            .flat_map(process_heading_section)
            .collect()
    }
}

/// Find indices where there's a sustained heading change.
fn find_heading_inflection_points(polyline: &[GpsPoint]) -> Vec<usize> {
    use crate::geo_utils::{
        bearing_difference, calculate_bearing, circular_mean_bearing, circular_std_bearing,
    };

    let mut inflection_points = Vec::new();

    if polyline.len() < 10 {
        return inflection_points;
    }

    // Compute per-segment bearings
    let bearings: Vec<f64> = (1..polyline.len())
        .map(|i| calculate_bearing(&polyline[i - 1], &polyline[i]))
        .collect();

    // Estimate average point spacing
    let total_dist = calculate_route_distance(polyline);
    let avg_spacing = total_dist / polyline.len() as f64;

    // Window size in points
    let window_points = ((BEARING_WINDOW_METERS / avg_spacing).ceil() as usize).max(3);

    // Skip if we don't have enough points for windows
    if bearings.len() < window_points * 2 + 1 {
        return inflection_points;
    }

    let mut last_split_idx = 0;

    for i in window_points..(bearings.len() - window_points) {
        // Compute trailing window (before point i)
        let trailing_start = i.saturating_sub(window_points);
        let trailing_bearings = &bearings[trailing_start..i];

        // Compute leading window (after point i)
        let leading_end = (i + window_points).min(bearings.len());
        let leading_bearings = &bearings[i..leading_end];

        // Check trailing window consistency
        let trailing_std = circular_std_bearing(trailing_bearings);
        if trailing_std > MAX_BEARING_VARIANCE {
            continue; // Trailing window is too noisy
        }

        // Check leading window consistency
        let leading_std = circular_std_bearing(leading_bearings);
        if leading_std > MAX_BEARING_VARIANCE {
            continue; // Leading window is too noisy
        }

        // Compute average bearings
        let trailing_avg = circular_mean_bearing(trailing_bearings);
        let leading_avg = circular_mean_bearing(leading_bearings);

        // Check for significant direction change
        let heading_change = bearing_difference(trailing_avg, leading_avg);
        if heading_change < MIN_HEADING_CHANGE {
            continue;
        }

        // Calculate distance from last split to verify sustained change
        let dist_from_last: f64 = (last_split_idx..=i)
            .take(i - last_split_idx)
            .map(|j| {
                if j + 1 < polyline.len() {
                    haversine_distance(&polyline[j], &polyline[j + 1])
                } else {
                    0.0
                }
            })
            .sum();

        if dist_from_last < MIN_SUSTAIN_DISTANCE && last_split_idx > 0 {
            continue; // Too close to last split
        }

        // Found a valid inflection point
        // Use i+1 because bearings array is offset by 1 from polyline
        inflection_points.push(i + 1);
        last_split_idx = i + 1;

        // Skip ahead to avoid multiple splits at same corner
        // (the loop continues from next iteration anyway)
    }

    inflection_points
}

// =============================================================================
// Gradient-Based Section Splitting
// =============================================================================

/// Minimum gradient change (%) to trigger a split.
/// E.g., transitioning from flat (0%) to steep uphill (8%) triggers a split.
/// Set high to only split at significant terrain changes.
const MIN_GRADIENT_CHANGE: f64 = 8.0;

/// Window size (meters) for computing average gradient.
const GRADIENT_WINDOW_METERS: f64 = 75.0;

/// Minimum segment length (meters) after splitting.
/// Keeps sections meaningful - no tiny fragments.
const MIN_GRADIENT_SPLIT_LENGTH: f64 = 300.0;

/// Process a single section for gradient-based splitting.
fn process_gradient_section(section: FrequentSection) -> Vec<FrequentSection> {
    let polyline = &section.polyline;

    // Guard 1: Skip splitting sections that are already short
    // Based on MDL principle - splitting short sections rarely improves quality
    if section.distance_meters < MIN_SECTION_FOR_SPLITTING {
        return vec![section];
    }

    // Check if we have elevation data
    let has_elevation = polyline.iter().any(|p| p.elevation.is_some());
    if !has_elevation || polyline.len() < 10 {
        return vec![section];
    }

    // Find gradient inflection points
    let split_indices = find_gradient_inflection_points(polyline);

    if split_indices.is_empty() {
        return vec![section];
    }

    // Split the section at each inflection point
    let mut segments: Vec<(usize, usize)> = Vec::new();
    let mut start_idx = 0;

    for &split_idx in &split_indices {
        if split_idx > start_idx {
            segments.push((start_idx, split_idx));
        }
        start_idx = split_idx;
    }

    // Add final segment
    if start_idx < polyline.len() {
        segments.push((start_idx, polyline.len()));
    }

    // Guard 2: Check ratio - don't split if smallest fragment < 40% of original
    // This prevents 200m → 100m + 100m type splits that create tiny pieces
    let segment_lengths: Vec<f64> = segments
        .iter()
        .map(|&(s, e)| calculate_route_distance(&polyline[s..e]))
        .collect();
    let min_segment_length = segment_lengths.iter().cloned().fold(f64::MAX, f64::min);
    let split_ratio = min_segment_length / section.distance_meters;

    if split_ratio < 0.4 && segments.len() > 1 {
        // Splitting would create unbalanced fragments - keep original
        return vec![section];
    }

    // Create new sections from segments
    let mut result = Vec::new();
    let mut split_count = 0;
    for (idx, &(seg_start, seg_end)) in segments.iter().enumerate() {
        let seg_polyline = polyline[seg_start..seg_end].to_vec();
        let seg_length = segment_lengths[idx];

        if seg_length >= MIN_GRADIENT_SPLIT_LENGTH && seg_polyline.len() >= 3 {
            let mut new_section = section.clone();
            new_section.id = format!("{}_g{}", section.id, split_count);
            new_section.polyline = seg_polyline;
            new_section.distance_meters = seg_length;
            new_section.activity_traces = HashMap::new();
            result.push(new_section);
            split_count += 1;
        }
    }

    if split_count > 1 {
        info!(
            "[Sections] Split {} at {} gradient inflection points -> {} segments",
            section.id,
            split_indices.len(),
            split_count
        );
    }

    if result.is_empty() {
        vec![section]
    } else {
        result
    }
}

/// Split sections at sustained gradient changes.
///
/// A section is split where gradient changes significantly (flat→uphill,
/// uphill→downhill, etc.). This only works if elevation data is available.
///
/// Uses MDL-inspired guards to prevent over-segmentation:
/// - Skips sections already < 500m (splitting rarely helps)
/// - Ensures all fragments are >= 300m
///
/// Reference: TS-MF Algorithm (Xu et al. 2022) - "avoid excessive segmentation"
/// https://www.hindawi.com/journals/wcmc/2022/9540944/
pub fn split_at_gradient_changes(
    sections: Vec<FrequentSection>,
    _config: &SectionConfig,
) -> Vec<FrequentSection> {
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        sections
            .into_par_iter()
            .flat_map(process_gradient_section)
            .collect()
    }

    #[cfg(not(feature = "parallel"))]
    {
        sections
            .into_iter()
            .flat_map(process_gradient_section)
            .collect()
    }
}

/// Find indices where there's a sustained gradient change.
fn find_gradient_inflection_points(polyline: &[GpsPoint]) -> Vec<usize> {
    use crate::geo_utils::segment_gradient;

    let mut inflection_points = Vec::new();

    if polyline.len() < 10 {
        return inflection_points;
    }

    // Estimate average point spacing
    let total_dist = calculate_route_distance(polyline);
    let avg_spacing = total_dist / polyline.len() as f64;

    // Window size in points
    let window_points = ((GRADIENT_WINDOW_METERS / avg_spacing).ceil() as usize).max(5);

    // Skip if we don't have enough points for windows
    if polyline.len() < window_points * 2 + 1 {
        return inflection_points;
    }

    let mut last_split_idx = 0;

    for i in window_points..(polyline.len() - window_points) {
        // Compute trailing window gradient
        let trailing_start = i.saturating_sub(window_points);
        let trailing_gradient = match segment_gradient(&polyline[trailing_start..i]) {
            Some(g) => g,
            None => continue,
        };

        // Compute leading window gradient
        let leading_end = (i + window_points).min(polyline.len());
        let leading_gradient = match segment_gradient(&polyline[i..leading_end]) {
            Some(g) => g,
            None => continue,
        };

        // Check for significant gradient change
        let gradient_change = (leading_gradient - trailing_gradient).abs();
        if gradient_change < MIN_GRADIENT_CHANGE {
            continue;
        }

        // Also check for sign change (uphill to downhill or vice versa)
        let sign_change = (trailing_gradient > 1.0 && leading_gradient < -1.0)
            || (trailing_gradient < -1.0 && leading_gradient > 1.0);

        // Must have significant change OR sign reversal
        if gradient_change < MIN_GRADIENT_CHANGE && !sign_change {
            continue;
        }

        // Check distance from last split
        let dist_from_last: f64 = (last_split_idx..i)
            .map(|j| {
                if j + 1 < polyline.len() {
                    haversine_distance(&polyline[j], &polyline[j + 1])
                } else {
                    0.0
                }
            })
            .sum();

        if dist_from_last < MIN_GRADIENT_SPLIT_LENGTH && last_split_idx > 0 {
            continue;
        }

        inflection_points.push(i);
        last_split_idx = i;
    }

    inflection_points
}

// =============================================================================
// Nearby Section Merging
// =============================================================================

/// Merge sections that are geometrically close to each other.
/// This handles: reversed sections, parallel tracks (opposite sides of road), GPS drift.
pub fn merge_nearby_sections(
    mut sections: Vec<FrequentSection>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    if sections.len() < 2 {
        return sections;
    }

    // Sort by visit count descending - keep the most visited version
    sections.sort_by(|a, b| b.visit_count.cmp(&a.visit_count));

    // PRE-COMPUTE all R-trees once upfront (O(k) builds instead of O(k²))
    #[cfg(feature = "parallel")]
    let rtrees: Vec<RTree<IndexedPoint>> = {
        use rayon::prelude::*;
        sections
            .par_iter()
            .map(|s| build_rtree(&s.polyline))
            .collect()
    };

    #[cfg(not(feature = "parallel"))]
    let rtrees: Vec<RTree<IndexedPoint>> =
        sections.iter().map(|s| build_rtree(&s.polyline)).collect();

    let mut keep: Vec<bool> = vec![true; sections.len()];

    // Use a very generous threshold for merging nearby sections
    // Wide roads can be 30m+, GPS error can add 20m, so use 2x the base threshold
    let merge_threshold = config.proximity_threshold * 2.0;

    for i in 0..sections.len() {
        if !keep[i] {
            continue;
        }

        let section_i = &sections[i];
        let tree_i = &rtrees[i]; // Use pre-computed R-tree (O(1) lookup)

        for j in (i + 1)..sections.len() {
            if !keep[j] {
                continue;
            }

            let section_j = &sections[j];

            // Skip if sections are very different lengths (>3x difference)
            let length_ratio = section_i.distance_meters / section_j.distance_meters.max(1.0);
            if !(0.33..=3.0).contains(&length_ratio) {
                continue;
            }

            // Check forward containment with generous threshold
            let forward_containment =
                compute_containment(&section_j.polyline, tree_i, merge_threshold);

            // Check reverse containment
            let reversed_j: Vec<GpsPoint> = section_j.polyline.iter().rev().cloned().collect();
            let reverse_containment = compute_containment(&reversed_j, tree_i, merge_threshold);

            let max_containment = forward_containment.max(reverse_containment);

            // Merge if either direction shows overlap (lower threshold since we're using generous distance)
            if max_containment > 0.4 {
                keep[j] = false;

                let direction = if reverse_containment > forward_containment {
                    "reverse"
                } else {
                    "same"
                };

                info!(
                    "[Sections] Merged nearby {} section {} into {} ({:.0}% overlap @ {}m threshold)",
                    direction, section_j.id, section_i.id, max_containment * 100.0, merge_threshold as i32
                );
            }
        }
    }

    sections
        .into_iter()
        .zip(keep)
        .filter_map(|(s, k)| if k { Some(s) } else { None })
        .collect()
}

// =============================================================================
// Fragment Consolidation
// =============================================================================

/// Maximum section length (meters) to consider for fragment consolidation.
/// Longer sections are kept as-is unless endpoints are very close.
const MAX_FRAGMENT_LENGTH: f64 = 400.0;

/// Tight threshold (meters) for joining ANY sections at endpoints.
/// If endpoints are this close, sections should be connected regardless of length.
const TIGHT_ENDPOINT_GAP: f64 = 50.0;

/// Loose threshold (meters) for merging short fragments.
const LOOSE_ENDPOINT_GAP: f64 = 100.0;

/// Maximum combined length (meters) after merging two sections.
const MAX_MERGED_LENGTH: f64 = 3000.0;

/// Consolidate adjacent sections back into longer coherent routes.
///
/// Iterative approach (runs until no more merges possible):
/// 1. **Tight join**: Connect ANY sections with endpoints < 50m apart
/// 2. **Fragment merge**: Merge short fragments (< 400m) with endpoints < 100m apart
///
/// Based on the "mergence" phase of TS-MF algorithm (Xu et al. 2022):
/// "The MDL principle merges subtrajectories by optimizing the cost function"
/// Reference: https://www.hindawi.com/journals/wcmc/2022/9540944/
pub fn consolidate_fragments(
    mut sections: Vec<FrequentSection>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    if sections.len() < 2 {
        return sections;
    }

    // Iterate until no more merges are possible
    // This handles chain merges (A→B→C) that single-pass would miss
    const MAX_ITERATIONS: usize = 10;
    for iteration in 0..MAX_ITERATIONS {
        let before_count = sections.len();

        // Pass 1: Tight endpoint joining (any length sections)
        sections = join_at_endpoints(sections, config, TIGHT_ENDPOINT_GAP);

        // Pass 2: Fragment merging (short sections only)
        sections = merge_short_fragments(sections, config);

        let after_count = sections.len();

        if after_count == before_count {
            info!(
                "[Sections] Consolidation converged after {} iteration(s)",
                iteration + 1
            );
            break;
        }

        info!(
            "[Sections] Consolidation iteration {}: {} → {} sections",
            iteration + 1,
            before_count,
            after_count
        );
    }

    sections
}

/// Join sections where endpoints are very close (regardless of section length).
/// This connects route segments that are clearly continuous.
fn join_at_endpoints(
    sections: Vec<FrequentSection>,
    _config: &SectionConfig,
    max_gap: f64,
) -> Vec<FrequentSection> {
    if sections.len() < 2 {
        return sections;
    }

    // Group sections by sport type
    let mut by_sport: HashMap<String, Vec<usize>> = HashMap::new();
    for (idx, section) in sections.iter().enumerate() {
        by_sport
            .entry(section.sport_type.clone())
            .or_default()
            .push(idx);
    }

    let mut merged: Vec<bool> = vec![false; sections.len()];
    let mut result: Vec<FrequentSection> = Vec::new();

    for indices in by_sport.values() {
        for &i in indices {
            if merged[i] {
                continue;
            }

            let section_i = &sections[i];

            // Find best join candidate (closest endpoint)
            let mut best_match: Option<(usize, f64, bool)> = None; // (index, gap, i_end_to_j_start)

            for &j in indices {
                if i == j || merged[j] {
                    continue;
                }

                let section_j = &sections[j];

                // Check if combined length is reasonable
                let combined_length = section_i.distance_meters + section_j.distance_meters;
                if combined_length > MAX_MERGED_LENGTH {
                    continue;
                }

                // Check endpoint distances
                let i_start = &section_i.polyline[0];
                let i_end = &section_i.polyline[section_i.polyline.len() - 1];
                let j_start = &section_j.polyline[0];
                let j_end = &section_j.polyline[section_j.polyline.len() - 1];

                // We want to join end-to-start (i_end -> j_start or j_end -> i_start)
                let gap_i_end_j_start = haversine_distance(i_end, j_start);
                let gap_j_end_i_start = haversine_distance(j_end, i_start);

                let (min_gap, is_i_to_j) = if gap_i_end_j_start <= gap_j_end_i_start {
                    (gap_i_end_j_start, true)
                } else {
                    (gap_j_end_i_start, false)
                };

                if min_gap <= max_gap && (best_match.is_none() || min_gap < best_match.unwrap().1) {
                    best_match = Some((j, min_gap, is_i_to_j));
                }
            }

            if let Some((j, gap, is_i_to_j)) = best_match {
                merged[i] = true;
                merged[j] = true;

                let section_j = &sections[j];

                // Merge polylines in correct order
                let mut merged_polyline = if is_i_to_j {
                    // i -> j order
                    let mut p = section_i.polyline.clone();
                    p.extend(section_j.polyline.clone());
                    p
                } else {
                    // j -> i order
                    let mut p = section_j.polyline.clone();
                    p.extend(section_i.polyline.clone());
                    p
                };

                // Simplify merged polyline if too dense
                if merged_polyline.len() > 200 {
                    merged_polyline = merged_polyline
                        .into_iter()
                        .enumerate()
                        .filter(|(idx, _)| idx % 2 == 0)
                        .map(|(_, p)| p)
                        .collect();
                }

                let merged_distance = calculate_route_distance(&merged_polyline);

                let mut merged_section = section_i.clone();
                merged_section.id = format!("{}_joined", section_i.id);
                merged_section.polyline = merged_polyline;
                merged_section.distance_meters = merged_distance;
                merged_section.visit_count = section_i.visit_count.max(section_j.visit_count);
                merged_section.confidence = (section_i.confidence + section_j.confidence) / 2.0;
                merged_section.activity_traces = HashMap::new();

                info!(
                    "[Sections] Joined {} + {} -> {} ({:.0}m + {:.0}m = {:.0}m, gap {:.0}m)",
                    section_i.id,
                    section_j.id,
                    merged_section.id,
                    section_i.distance_meters,
                    section_j.distance_meters,
                    merged_distance,
                    gap
                );

                result.push(merged_section);
            }
        }

        // Add non-merged sections
        for &i in indices {
            if !merged[i] {
                result.push(sections[i].clone());
            }
        }
    }

    result
}

/// Merge short fragments (< 400m) that are adjacent (< 100m gap).
fn merge_short_fragments(
    sections: Vec<FrequentSection>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    if sections.len() < 2 {
        return sections;
    }

    // Group sections by sport type
    let mut by_sport: HashMap<String, Vec<usize>> = HashMap::new();
    for (idx, section) in sections.iter().enumerate() {
        by_sport
            .entry(section.sport_type.clone())
            .or_default()
            .push(idx);
    }

    let mut merged: Vec<bool> = vec![false; sections.len()];
    let mut result: Vec<FrequentSection> = Vec::new();

    for indices in by_sport.values() {
        for &i in indices {
            if merged[i] {
                continue;
            }

            let section_i = &sections[i];

            // Only consider short fragments
            if section_i.distance_meters > MAX_FRAGMENT_LENGTH {
                continue;
            }

            // Find best merge candidate
            let mut best_match: Option<(usize, f64)> = None;

            for &j in indices {
                if i == j || merged[j] {
                    continue;
                }

                let section_j = &sections[j];

                // Only merge with other short fragments
                if section_j.distance_meters > MAX_FRAGMENT_LENGTH {
                    continue;
                }

                // Check if combined length is reasonable
                let combined_length = section_i.distance_meters + section_j.distance_meters;
                if combined_length > MAX_MERGED_LENGTH {
                    continue;
                }

                // Check if endpoints are close
                let i_start = &section_i.polyline[0];
                let i_end = &section_i.polyline[section_i.polyline.len() - 1];
                let j_start = &section_j.polyline[0];
                let j_end = &section_j.polyline[section_j.polyline.len() - 1];

                let min_gap = haversine_distance(i_start, j_start)
                    .min(haversine_distance(i_start, j_end))
                    .min(haversine_distance(i_end, j_start))
                    .min(haversine_distance(i_end, j_end));

                if min_gap <= LOOSE_ENDPOINT_GAP
                    && (best_match.is_none() || min_gap < best_match.unwrap().1)
                {
                    best_match = Some((j, min_gap));
                }
            }

            if let Some((j, gap)) = best_match {
                merged[i] = true;
                merged[j] = true;

                let section_j = &sections[j];

                // Determine merge order
                let i_end = &section_i.polyline[section_i.polyline.len() - 1];
                let j_start = &section_j.polyline[0];
                let end_to_start_gap = haversine_distance(i_end, j_start);

                let mut merged_polyline = if end_to_start_gap <= config.proximity_threshold * 2.0 {
                    let mut p = section_i.polyline.clone();
                    p.extend(section_j.polyline.clone());
                    p
                } else {
                    let mut p = section_j.polyline.clone();
                    p.extend(section_i.polyline.clone());
                    p
                };

                if merged_polyline.len() > 200 {
                    merged_polyline = merged_polyline
                        .into_iter()
                        .enumerate()
                        .filter(|(idx, _)| idx % 2 == 0)
                        .map(|(_, p)| p)
                        .collect();
                }

                let merged_distance = calculate_route_distance(&merged_polyline);

                let mut merged_section = section_i.clone();
                merged_section.id = format!("{}_merged", section_i.id);
                merged_section.polyline = merged_polyline;
                merged_section.distance_meters = merged_distance;
                merged_section.visit_count = section_i.visit_count.max(section_j.visit_count);
                merged_section.confidence = (section_i.confidence + section_j.confidence) / 2.0;
                merged_section.activity_traces = HashMap::new();

                info!(
                    "[Sections] Merged fragments {} + {} -> {} ({:.0}m + {:.0}m = {:.0}m, gap {:.0}m)",
                    section_i.id, section_j.id, merged_section.id,
                    section_i.distance_meters, section_j.distance_meters,
                    merged_distance, gap
                );

                result.push(merged_section);
            }
        }

        // Add non-merged sections
        for &i in indices {
            if !merged[i] {
                result.push(sections[i].clone());
            }
        }
    }

    result
}

// =============================================================================
// Section Deduplication
// =============================================================================

/// Remove sections that overlap significantly.
/// Strategy: Prefer SHORTER sections over longer ones that contain them.
/// A short section (like an intersection or bridge) is more specific and useful
/// than a long section that happens to include it.
///
/// **Loop sections are protected** - they won't be removed even if contained
/// in another section, because loops represent complete circuits.
pub fn remove_overlapping_sections(
    mut sections: Vec<FrequentSection>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    if sections.len() < 2 {
        return sections;
    }

    // Loop detection threshold: 2x proximity to allow for GPS drift at closure
    let loop_threshold = config.proximity_threshold * 2.0;

    // Sort by LENGTH ascending (shorter sections first), then by visit count descending
    // This ensures shorter, more specific sections are preferred
    sections.sort_by(
        |a, b| match a.distance_meters.partial_cmp(&b.distance_meters) {
            Some(std::cmp::Ordering::Equal) => b.visit_count.cmp(&a.visit_count),
            Some(ord) => ord,
            None => std::cmp::Ordering::Equal,
        },
    );

    // PRE-COMPUTE all R-trees once upfront (O(k) builds instead of O(k²))
    #[cfg(feature = "parallel")]
    let rtrees: Vec<RTree<IndexedPoint>> = {
        use rayon::prelude::*;
        sections
            .par_iter()
            .map(|s| build_rtree(&s.polyline))
            .collect()
    };

    #[cfg(not(feature = "parallel"))]
    let rtrees: Vec<RTree<IndexedPoint>> =
        sections.iter().map(|s| build_rtree(&s.polyline)).collect();

    // Compute which sections are loops (protected from removal)
    let is_loop: Vec<bool> = sections
        .iter()
        .map(|s| is_loop_section(s, loop_threshold))
        .collect();

    let mut keep: Vec<bool> = vec![true; sections.len()];

    // For each section, check if it's mostly contained in a shorter section
    // If so, the longer section should be removed (or trimmed)
    for i in 0..sections.len() {
        if !keep[i] {
            continue;
        }

        let section_i = &sections[i];
        let tree_i = &rtrees[i]; // Use pre-computed R-tree (O(1) lookup)

        for j in (i + 1)..sections.len() {
            if !keep[j] {
                continue;
            }

            let section_j = &sections[j];
            let tree_j = &rtrees[j]; // Use pre-computed R-tree (O(1) lookup)

            // Check mutual containment
            let j_in_i =
                compute_containment(&section_j.polyline, tree_i, config.proximity_threshold);
            let i_in_j =
                compute_containment(&section_i.polyline, tree_j, config.proximity_threshold);

            // If j is largely contained in i (j is the longer one since we sorted by length)
            // j should be removed because i is the more specific section
            // BUT: Don't remove j if it's a loop!
            if j_in_i > 0.6 && !is_loop[j] {
                info!(
                    "[Sections] Removing {} ({}m) - {}% contained in {} ({}m)",
                    section_j.id,
                    section_j.distance_meters as u32,
                    (j_in_i * 100.0) as u32,
                    section_i.id,
                    section_i.distance_meters as u32
                );
                keep[j] = false;
            } else if i_in_j > 0.8 && !is_loop[i] {
                // If i is almost entirely contained in j, remove i (the smaller one)
                // BUT: Don't remove i if it's a loop!
                info!(
                    "[Sections] Removing {} ({}m) - {}% contained in {} ({}m)",
                    section_i.id,
                    section_i.distance_meters as u32,
                    (i_in_j * 100.0) as u32,
                    section_j.id,
                    section_j.distance_meters as u32
                );
                keep[i] = false;
                break; // Stop checking j's against removed i
            } else if j_in_i > 0.4 && i_in_j > 0.4 && !is_loop[i] && !is_loop[j] {
                // Significant mutual overlap - they're essentially the same
                // Keep the shorter one (i, since sorted by length)
                // BUT: Don't remove either if they're loops!
                info!(
                    "[Sections] Removing {} due to mutual overlap with {} ({}% vs {}%)",
                    section_j.id,
                    section_i.id,
                    (j_in_i * 100.0) as u32,
                    (i_in_j * 100.0) as u32
                );
                keep[j] = false;
            }
        }
    }

    let result: Vec<FrequentSection> = sections
        .into_iter()
        .zip(keep)
        .filter_map(|(s, k)| if k { Some(s) } else { None })
        .collect();

    let loop_count = result
        .iter()
        .filter(|s| is_loop_section(s, loop_threshold))
        .count();
    info!(
        "[Sections] After removing overlaps: {} sections ({} loops protected)",
        result.len(),
        loop_count
    );

    result
}

/// Check if a section forms a closed loop (start ≈ end).
/// Used by both remove_overlapping_sections and make_sections_exclusive.
fn is_loop_section(section: &FrequentSection, threshold: f64) -> bool {
    if section.polyline.len() < 10 {
        return false;
    }
    let start = &section.polyline[0];
    let end = &section.polyline[section.polyline.len() - 1];
    haversine_distance(start, end) < threshold
}

/// Compute what fraction of polyline A is contained within polyline B
fn compute_containment(poly_a: &[GpsPoint], tree_b: &RTree<IndexedPoint>, threshold: f64) -> f64 {
    if poly_a.is_empty() {
        return 0.0;
    }

    let threshold_deg = threshold / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;

    let mut contained_points = 0;

    for point in poly_a {
        let query = [point.latitude, point.longitude];
        if let Some(nearest) = tree_b.nearest_neighbor(&query) {
            if nearest.distance_2(&query) <= threshold_deg_sq {
                contained_points += 1;
            }
        }
    }

    contained_points as f64 / poly_a.len() as f64
}

// =============================================================================
// Density-Based Section Splitting
// =============================================================================

/// Minimum density ratio to trigger a split (high-traffic portion / endpoint density)
const SPLIT_DENSITY_RATIO: f64 = 2.0;

/// Minimum length (meters) for a split portion to become its own section
const MIN_SPLIT_LENGTH: f64 = 100.0;

/// Minimum number of points in a high-density region to consider splitting
const MIN_SPLIT_POINTS: usize = 10;

/// Result of analyzing a section for potential splits
#[derive(Debug)]
struct SplitCandidate {
    /// Start index of the high-density portion
    start_idx: usize,
    /// End index of the high-density portion
    end_idx: usize,
    /// Average density in this portion
    avg_density: f64,
    /// Density ratio compared to endpoints
    density_ratio: f64,
}

/// Analyze a section's point density to find high-traffic portions.
/// Returns split candidates if the section should be divided.
fn find_split_candidates(section: &FrequentSection) -> Vec<SplitCandidate> {
    let density = &section.point_density;

    if density.len() < MIN_SPLIT_POINTS * 2 {
        return vec![]; // Too short to split meaningfully
    }

    // Compute endpoint density (average of first/last 10% of points)
    let endpoint_window = (density.len() / 10).max(3);
    let start_density: f64 = density[..endpoint_window]
        .iter()
        .map(|&d| d as f64)
        .sum::<f64>()
        / endpoint_window as f64;
    let end_density: f64 = density[density.len() - endpoint_window..]
        .iter()
        .map(|&d| d as f64)
        .sum::<f64>()
        / endpoint_window as f64;
    let endpoint_density = (start_density + end_density) / 2.0;

    if endpoint_density < 1.0 {
        return vec![]; // No meaningful endpoint density to compare against
    }

    // Sliding window to find high-density regions
    let window_size = (density.len() / 5).max(MIN_SPLIT_POINTS);
    let mut candidates = Vec::new();

    let mut i = window_size;
    while i < density.len() - window_size {
        // Compute density in current window
        let window_density: f64 = density[i - window_size / 2..i + window_size / 2]
            .iter()
            .map(|&d| d as f64)
            .sum::<f64>()
            / window_size as f64;

        let ratio = window_density / endpoint_density;

        if ratio >= SPLIT_DENSITY_RATIO {
            // Found a high-density region - expand to find boundaries
            let mut start_idx = i - window_size / 2;
            let mut end_idx = i + window_size / 2;

            // Expand start backward while density remains high
            while start_idx > 0 {
                let local_density = density[start_idx - 1] as f64;
                if local_density < endpoint_density * 1.5 {
                    break;
                }
                start_idx -= 1;
            }

            // Expand end forward while density remains high
            while end_idx < density.len() - 1 {
                let local_density = density[end_idx + 1] as f64;
                if local_density < endpoint_density * 1.5 {
                    break;
                }
                end_idx += 1;
            }

            // Compute distance of this portion
            let portion_distance = if end_idx > start_idx {
                calculate_route_distance(&section.polyline[start_idx..=end_idx])
            } else {
                0.0
            };

            // Only consider if long enough
            if portion_distance >= MIN_SPLIT_LENGTH && end_idx - start_idx >= MIN_SPLIT_POINTS {
                let portion_density: f64 = density[start_idx..=end_idx]
                    .iter()
                    .map(|&d| d as f64)
                    .sum::<f64>()
                    / (end_idx - start_idx + 1) as f64;

                candidates.push(SplitCandidate {
                    start_idx,
                    end_idx,
                    avg_density: portion_density,
                    density_ratio: portion_density / endpoint_density,
                });

                // Skip past this region
                i = end_idx + window_size;
            } else {
                i += 1;
            }
        } else {
            i += 1;
        }
    }

    candidates
}

/// Split a section into multiple sections based on density analysis.
/// Returns the original section plus any new sections created from high-density portions.
fn split_section_by_density(
    section: FrequentSection,
    track_map: &HashMap<String, Vec<GpsPoint>>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    let candidates = find_split_candidates(&section);

    if candidates.is_empty() {
        return vec![section];
    }

    info!(
        "[Sections] Found {} split candidates for section {} (len={}m)",
        candidates.len(),
        section.id,
        section.distance_meters as i32
    );

    let mut result = Vec::new();

    // Create new sections from high-density portions
    for (split_idx, candidate) in candidates.iter().enumerate() {
        // Extract the high-density portion
        let split_polyline = section.polyline[candidate.start_idx..=candidate.end_idx].to_vec();
        let split_density = section.point_density[candidate.start_idx..=candidate.end_idx].to_vec();
        let split_distance = calculate_route_distance(&split_polyline);

        // Re-compute which activities overlap with this portion
        let mut split_activity_ids = Vec::new();
        let mut split_activity_traces = HashMap::new();

        let split_tree = build_rtree(&split_polyline);
        let threshold_deg = config.proximity_threshold / 111_000.0;
        let threshold_deg_sq = threshold_deg * threshold_deg;

        for activity_id in &section.activity_ids {
            if let Some(track) = track_map.get(activity_id) {
                // Check if this activity overlaps with the split portion
                let mut overlap_points = Vec::new();

                for point in track {
                    let query = [point.latitude, point.longitude];
                    if let Some(nearest) = split_tree.nearest_neighbor(&query) {
                        if nearest.distance_2(&query) <= threshold_deg_sq {
                            overlap_points.push(*point);
                        }
                    }
                }

                // Need substantial overlap to count
                let overlap_distance = calculate_route_distance(&overlap_points);
                if overlap_distance >= split_distance * 0.5 {
                    split_activity_ids.push(activity_id.clone());
                    if !overlap_points.is_empty() {
                        split_activity_traces.insert(activity_id.clone(), overlap_points);
                    }
                }
            }
        }

        // Only create the split section if it has enough activities
        if split_activity_ids.len() >= config.min_activities as usize {
            let split_section = FrequentSection {
                id: format!("{}_split{}", section.id, split_idx),
                name: None,
                sport_type: section.sport_type.clone(),
                polyline: split_polyline,
                representative_activity_id: section.representative_activity_id.clone(),
                activity_ids: split_activity_ids,
                activity_portions: Vec::new(), // Will be recomputed later if needed
                route_ids: section.route_ids.clone(),
                visit_count: candidate.avg_density as u32,
                distance_meters: split_distance,
                activity_traces: split_activity_traces,
                confidence: section.confidence,
                observation_count: candidate.avg_density as u32,
                average_spread: section.average_spread,
                point_density: split_density,
                scale: section.scale.clone(),
                // Inherit evolution fields from parent section
                version: section.version,
                is_user_defined: section.is_user_defined,
                created_at: section.created_at.clone(),
                updated_at: section.updated_at.clone(),
                stability: section.stability,
            };

            info!(
                "[Sections] Created split section {} with {} activities (density ratio {:.1}x)",
                split_section.id,
                split_section.activity_ids.len(),
                candidate.density_ratio
            );

            result.push(split_section);
        }
    }

    // Keep the original section too (it still represents the full route)
    result.push(section);

    result
}

/// Post-processing step: Split sections with high density variance.
/// Called after initial section detection to break up sections that have
/// high-traffic portions used by many other activities.
pub fn split_high_variance_sections(
    sections: Vec<FrequentSection>,
    track_map: &HashMap<String, Vec<GpsPoint>>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        sections
            .into_par_iter()
            .flat_map(|section| split_section_by_density(section, track_map, config))
            .collect()
    }

    #[cfg(not(feature = "parallel"))]
    {
        sections
            .into_iter()
            .flat_map(|section| split_section_by_density(section, track_map, config))
            .collect()
    }
}

// =============================================================================
// Mutually Exclusive Sections - Cut overlapping portions
// =============================================================================

/// Make sections mutually exclusive by cutting overlapping portions.
/// Uses a "claim territory" approach: higher-priority sections claim their
/// territory first, and later sections are trimmed to exclude claimed areas.
///
/// **Loop sections are preserved intact** - they're not trimmed even if they
/// overlap with other sections. This allows complete circuits to be preserved
/// (e.g., a lake loop and an outer ring that share one edge).
///
/// Priority is determined by: confidence * log(visit_count) * loop_boost
pub fn make_sections_exclusive(
    mut sections: Vec<FrequentSection>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    if sections.len() < 2 {
        return sections;
    }

    // Loop detection threshold: 2x proximity to allow for GPS drift at closure
    let loop_threshold = config.proximity_threshold * 2.0;

    // Sort by priority: loops get a boost, then by confidence and visits
    sections.sort_by(|a, b| {
        let loop_a = is_loop_section(a, loop_threshold);
        let loop_b = is_loop_section(b, loop_threshold);

        // Loops get 1.5x priority boost since they're complete circuits
        let boost_a = if loop_a { 1.5 } else { 1.0 };
        let boost_b = if loop_b { 1.5 } else { 1.0 };

        let priority_a = a.confidence * (a.visit_count as f64).ln().max(1.0) * boost_a;
        let priority_b = b.confidence * (b.visit_count as f64).ln().max(1.0) * boost_b;
        priority_b
            .partial_cmp(&priority_a)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut result: Vec<FrequentSection> = Vec::new();
    let mut claimed_trees: Vec<RTree<IndexedPoint>> = Vec::new();
    let mut loop_count = 0;

    for section in sections {
        let is_loop = is_loop_section(&section, loop_threshold);

        if is_loop {
            // LOOPS: Preserve intact without trimming
            // They may overlap with other loops/sections - that's OK
            info!(
                "[Sections] Preserving loop section {} ({:.0}m, {} visits)",
                section.id, section.distance_meters, section.visit_count
            );
            claimed_trees.push(build_rtree(&section.polyline));
            result.push(section);
            loop_count += 1;
        } else {
            // NON-LOOPS: Trim to exclude claimed territory
            let trimmed = trim_to_unclaimed(&section, &claimed_trees, config);

            if let Some(trimmed_section) = trimmed {
                // Only keep if long enough
                if trimmed_section.distance_meters >= config.min_section_length {
                    // Add this section's territory to claimed areas
                    claimed_trees.push(build_rtree(&trimmed_section.polyline));
                    result.push(trimmed_section);
                }
            }
        }
    }

    info!(
        "[Sections] After making exclusive: {} sections ({} loops preserved)",
        result.len(),
        loop_count
    );

    result
}

/// Trim a section to exclude areas already claimed by other sections.
/// Returns None if the entire section is claimed.
fn trim_to_unclaimed(
    section: &FrequentSection,
    claimed_trees: &[RTree<IndexedPoint>],
    config: &SectionConfig,
) -> Option<FrequentSection> {
    if claimed_trees.is_empty() {
        return Some(section.clone());
    }

    let threshold_deg = config.proximity_threshold / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;

    // Find which points are NOT claimed
    let mut unclaimed_mask: Vec<bool> = vec![true; section.polyline.len()];

    for (point_idx, point) in section.polyline.iter().enumerate() {
        let query = [point.latitude, point.longitude];

        for tree in claimed_trees {
            if let Some(nearest) = tree.nearest_neighbor(&query) {
                if nearest.distance_2(&query) <= threshold_deg_sq {
                    unclaimed_mask[point_idx] = false;
                    break;
                }
            }
        }
    }

    // Find the longest contiguous unclaimed segment
    let mut best_start = 0;
    let mut best_len = 0;
    let mut current_start = 0;
    let mut current_len = 0;
    let mut in_unclaimed = false;

    for (i, &is_unclaimed) in unclaimed_mask.iter().enumerate() {
        if is_unclaimed {
            if !in_unclaimed {
                current_start = i;
                current_len = 0;
                in_unclaimed = true;
            }
            current_len += 1;
        } else {
            if in_unclaimed && current_len > best_len {
                best_start = current_start;
                best_len = current_len;
            }
            in_unclaimed = false;
        }
    }
    // Check final segment
    if in_unclaimed && current_len > best_len {
        best_start = current_start;
        best_len = current_len;
    }

    // Need at least a few points
    if best_len < 3 {
        return None;
    }

    // Extract the unclaimed portion
    let trimmed_polyline: Vec<GpsPoint> =
        section.polyline[best_start..(best_start + best_len)].to_vec();
    let trimmed_distance = calculate_route_distance(&trimmed_polyline);

    if trimmed_distance < config.min_section_length {
        return None;
    }

    // Create trimmed section
    let mut trimmed = section.clone();
    trimmed.polyline = trimmed_polyline;
    trimmed.distance_meters = trimmed_distance;

    // Update point density if available
    if !section.point_density.is_empty() && best_start + best_len <= section.point_density.len() {
        trimmed.point_density = section.point_density[best_start..(best_start + best_len)].to_vec();
    }

    Some(trimmed)
}

// =============================================================================
// Quality Filtering: Length-Weighted Visit Threshold
// =============================================================================
//
// Based on graph-based clustering principles where low-density regions are noise.
// Reference: "Graph-Based Approaches to Clustering Network-Constrained Trajectory Data"
// https://arxiv.org/abs/1310.5249
//
// Philosophy: Since users can create their own segments, automatic detection
// should be conservative. Short sections that are only visited twice are
// probably noise—but short sections visited many times are meaningful patterns.

/// Minimum visits required based on section length.
/// Shorter sections need more visits to prove they're not noise.
///
/// Rationale:
/// - Very short (< 200m): Could be GPS drift or random detour → needs 6+ visits
/// - Short (< 400m): Possible shortcut or intersection → needs 4+ visits
/// - Medium (< 800m): Likely a distinct route portion → needs 3+ visits
/// - Long (800m+): Significant route segment → 2+ visits is enough
fn required_visits_for_length(distance_meters: f64) -> u32 {
    match distance_meters {
        d if d < 200.0 => 6,
        d if d < 400.0 => 4,
        d if d < 800.0 => 3,
        _ => 2,
    }
}

/// Quality filter: Remove low-confidence sections based on length and visits.
///
/// This is the final filter in the pipeline, applied after all merging/splitting.
/// It ensures we only keep sections that are genuinely frequent patterns.
pub fn filter_low_quality_sections(sections: Vec<FrequentSection>) -> Vec<FrequentSection> {
    let before = sections.len();
    let mut filtered: Vec<FrequentSection> = Vec::new();

    for section in sections {
        let min_visits = required_visits_for_length(section.distance_meters);
        let min_points = 8;

        if section.visit_count >= min_visits && section.polyline.len() >= min_points {
            filtered.push(section);
        } else {
            info!(
                "[Sections] Filtered out {}: {:.0}m with {} visits (needs {} visits) and {} points",
                section.id,
                section.distance_meters,
                section.visit_count,
                min_visits,
                section.polyline.len()
            );
        }
    }

    info!(
        "[Sections] Quality filter: {} → {} sections",
        before,
        filtered.len()
    );

    filtered
}
