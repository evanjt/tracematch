//! Post-processing for sections: folding detection, merging, deduplication, and density-based splitting.
//!
//! Inspired by:
//! - TRACLUS (Lee, Han, Whang 2007) — avoid over-segmentation principle
//! - TS-MF (Xu et al. 2022) — two-phase split-then-merge pipeline

use super::portions::find_all_track_portions;
use super::rtree::{IndexedPoint, build_rtree};
use super::{FrequentSection, SectionConfig, SectionPortion};
use crate::GpsPoint;
use crate::geo_utils::haversine_distance;
use crate::matching::calculate_route_distance;
use log::info;
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use rstar::{AABB, PointDistance, RTree, RTreeObject};
use std::collections::HashMap;

/// Section centroid for the post-processing R-tree pre-filter (Tier 2.3).
#[derive(Debug, Clone, Copy)]
struct CentroidEntry {
    idx: usize,
    lat: f64,
    lng: f64,
}

impl RTreeObject for CentroidEntry {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_point([self.lat, self.lng])
    }
}

impl PointDistance for CentroidEntry {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let d0 = self.lat - point[0];
        let d1 = self.lng - point[1];
        d0 * d0 + d1 * d1
    }
}

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
        if let Some(nearest) = first_tree.nearest_neighbor(&query)
            && nearest.distance_2(&query) <= threshold_deg_sq
        {
            close_count += 1;
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
/// Inspired by MDL principle: only split if it improves the description.
/// Raised to 1000m so heading/gradient splits only fire on long sections
/// where the split materially reflects different physical regions.
const MIN_SECTION_FOR_SPLITTING: f64 = 1000.0;

/// Process a single section for heading-based splitting.
fn process_heading_section(section: FrequentSection) -> Vec<FrequentSection> {
    let polyline = &section.polyline;

    // Guard 1: Skip splitting sections that are already short
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
/// Guards to prevent over-segmentation:
/// - Skips sections already < 500m (splitting rarely helps)
/// - Ensures all fragments are >= 300m
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
/// Guards to prevent over-segmentation:
/// - Skips sections already < 500m (splitting rarely helps)
/// - Ensures all fragments are >= 300m
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
    sections.sort_by_key(|s| std::cmp::Reverse(s.visit_count));

    // PRE-COMPUTE all R-trees once upfront (O(k) builds instead of O(k²))
    #[cfg(feature = "parallel")]
    let rtrees: Vec<RTree<IndexedPoint>> = sections
        .par_iter()
        .map(|s| build_rtree(&s.polyline))
        .collect();

    #[cfg(not(feature = "parallel"))]
    let rtrees: Vec<RTree<IndexedPoint>> =
        sections.iter().map(|s| build_rtree(&s.polyline)).collect();

    let mut keep: Vec<bool> = vec![true; sections.len()];

    // Use a generous threshold for merging nearby sections. Default
    // 4× proximity (200m at the default 50m proximity) — recombines
    // adjacent fragments along the same physical path.
    let merge_threshold = config.proximity_threshold * config.merge_distance_multiplier;

    // Tier 2.3: prune the O(S²) candidate space with a centroid R-tree.
    // For each section, only consider partners whose centroid is within
    // (own_length + partner_length + merge_threshold) — anything farther
    // can't overlap geometrically. Builds in O(S log S), queries in
    // O(M log S) where M = real geographic neighbours.
    let centroids: Vec<[f64; 2]> = sections
        .iter()
        .map(|s| {
            let n = s.polyline.len().max(1) as f64;
            let (lat_sum, lng_sum) = s.polyline.iter().fold((0.0, 0.0), |(la, ln), p| {
                (la + p.latitude, ln + p.longitude)
            });
            [lat_sum / n, lng_sum / n]
        })
        .collect();
    let centroid_tree: rstar::RTree<CentroidEntry> = rstar::RTree::bulk_load(
        centroids
            .iter()
            .enumerate()
            .map(|(idx, c)| CentroidEntry {
                idx,
                lat: c[0],
                lng: c[1],
            })
            .collect(),
    );

    // Generate candidate pairs (i, j) where lengths are comparable AND
    // centroids are within plausible overlap range.
    let mut candidate_pairs: Vec<(usize, usize)> = Vec::new();
    for i in 0..sections.len() {
        let li = sections[i].distance_meters;
        // Use the section's own length plus the longest partner length we
        // could plausibly merge with (3x by the length-ratio gate below).
        let max_partner_len = li * 3.0;
        let radius_m = li + max_partner_len + merge_threshold;
        let radius_deg = radius_m / 111_000.0;
        let center = [centroids[i][0], centroids[i][1]];

        for entry in centroid_tree.locate_within_distance(center, radius_deg * radius_deg) {
            let j = entry.idx;
            if j <= i {
                continue;
            }
            let length_ratio = li / sections[j].distance_meters.max(1.0);
            if (0.33..=3.0).contains(&length_ratio) {
                candidate_pairs.push((i, j));
            }
        }
    }

    // Pre-compute containment values for all candidate pairs in parallel
    // Each pair produces (i, j, forward_containment, reverse_containment)
    // Optimization: only compute reverse containment if forward is below threshold
    #[cfg(feature = "parallel")]
    let containment_results: Vec<(usize, usize, f64, f64)> = candidate_pairs
        .par_iter()
        .map(|&(i, j)| {
            let forward = compute_containment(&sections[j].polyline, &rtrees[i], merge_threshold);
            if forward > 0.4 {
                return (i, j, forward, 0.0);
            }
            let reversed_j: Vec<GpsPoint> = sections[j].polyline.iter().rev().cloned().collect();
            let reverse = compute_containment(&reversed_j, &rtrees[i], merge_threshold);
            (i, j, forward, reverse)
        })
        .collect();

    #[cfg(not(feature = "parallel"))]
    let containment_results: Vec<(usize, usize, f64, f64)> = candidate_pairs
        .iter()
        .map(|&(i, j)| {
            let forward = compute_containment(&sections[j].polyline, &rtrees[i], merge_threshold);
            if forward > 0.4 {
                return (i, j, forward, 0.0);
            }
            let reversed_j: Vec<GpsPoint> = sections[j].polyline.iter().rev().cloned().collect();
            let reverse = compute_containment(&reversed_j, &rtrees[i], merge_threshold);
            (i, j, forward, reverse)
        })
        .collect();

    // Apply greedy merge decisions sequentially using pre-computed values
    for (i, j, forward_containment, reverse_containment) in containment_results {
        if !keep[i] || !keep[j] {
            continue;
        }

        let max_containment = forward_containment.max(reverse_containment);

        if max_containment > 0.3 {
            keep[j] = false;

            let direction = if reverse_containment > forward_containment {
                "reverse"
            } else {
                "same"
            };

            info!(
                "[Sections] Merged nearby {} section {} into {} ({:.0}% overlap @ {}m threshold)",
                direction,
                sections[j].id,
                sections[i].id,
                max_containment * 100.0,
                merge_threshold as i32
            );
        }
    }

    sections
        .into_iter()
        .zip(keep)
        .filter_map(|(s, k)| if k { Some(s) } else { None })
        .collect()
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
    let rtrees: Vec<RTree<IndexedPoint>> = sections
        .par_iter()
        .map(|s| build_rtree(&s.polyline))
        .collect();

    #[cfg(not(feature = "parallel"))]
    let rtrees: Vec<RTree<IndexedPoint>> =
        sections.iter().map(|s| build_rtree(&s.polyline)).collect();

    // Compute which sections are loops (protected from removal)
    let is_loop: Vec<bool> = sections
        .iter()
        .map(|s| is_loop_section(s, loop_threshold))
        .collect();

    let mut keep: Vec<bool> = vec![true; sections.len()];

    // Generate all (i, j) pairs for containment checking
    let all_pairs: Vec<(usize, usize)> = (0..sections.len())
        .flat_map(|i| ((i + 1)..sections.len()).map(move |j| (i, j)))
        .collect();

    // Pre-compute mutual containment for all pairs in parallel
    // Each produces (i, j, j_in_i, i_in_j)
    #[cfg(feature = "parallel")]
    let containment_results: Vec<(usize, usize, f64, f64)> = all_pairs
        .par_iter()
        .map(|&(i, j)| {
            let j_in_i = compute_containment(
                &sections[j].polyline,
                &rtrees[i],
                config.proximity_threshold,
            );
            let i_in_j = compute_containment(
                &sections[i].polyline,
                &rtrees[j],
                config.proximity_threshold,
            );
            (i, j, j_in_i, i_in_j)
        })
        .collect();

    #[cfg(not(feature = "parallel"))]
    let containment_results: Vec<(usize, usize, f64, f64)> = all_pairs
        .iter()
        .map(|&(i, j)| {
            let j_in_i = compute_containment(
                &sections[j].polyline,
                &rtrees[i],
                config.proximity_threshold,
            );
            let i_in_j = compute_containment(
                &sections[i].polyline,
                &rtrees[j],
                config.proximity_threshold,
            );
            (i, j, j_in_i, i_in_j)
        })
        .collect();

    // Apply greedy removal decisions sequentially using pre-computed values
    for (i, j, j_in_i, i_in_j) in containment_results {
        if !keep[i] || !keep[j] {
            continue;
        }

        // If j is largely contained in i (j is the longer one since we sorted by length)
        // j should be removed because i is the more specific section
        // BUT: Don't remove j if it's a loop!
        if j_in_i > 0.45 && !is_loop[j] {
            info!(
                "[Sections] Removing {} ({}m) - {}% contained in {} ({}m)",
                sections[j].id,
                sections[j].distance_meters as u32,
                (j_in_i * 100.0) as u32,
                sections[i].id,
                sections[i].distance_meters as u32
            );
            keep[j] = false;
        } else if i_in_j > 0.7 && !is_loop[i] {
            info!(
                "[Sections] Removing {} ({}m) - {}% contained in {} ({}m)",
                sections[i].id,
                sections[i].distance_meters as u32,
                (i_in_j * 100.0) as u32,
                sections[j].id,
                sections[j].distance_meters as u32
            );
            keep[i] = false;
        } else if j_in_i > 0.3 && i_in_j > 0.3 && !is_loop[i] && !is_loop[j] {
            info!(
                "[Sections] Removing {} due to mutual overlap with {} ({}% vs {}%)",
                sections[j].id,
                sections[i].id,
                (j_in_i * 100.0) as u32,
                (i_in_j * 100.0) as u32
            );
            keep[j] = false;
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
        if let Some(nearest) = tree_b.nearest_neighbor(&query)
            && nearest.distance_2(&query) <= threshold_deg_sq
        {
            contained_points += 1;
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
    let polyline_len = section.polyline.len();

    if density.len() < MIN_SPLIT_POINTS * 2 {
        return vec![]; // Too short to split meaningfully
    }

    // Defence in depth: post-processing splits/merges can leave
    // `point_density` and `polyline` out of sync. Cap the loop bound to
    // whichever is shorter so we never index past `polyline`.
    let usable_len = density.len().min(polyline_len);
    if usable_len < MIN_SPLIT_POINTS * 2 {
        return vec![];
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
    let window_size = (usable_len / 5).max(MIN_SPLIT_POINTS);
    let mut candidates = Vec::new();

    let mut i = window_size;
    while i < usable_len - window_size {
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

            // Expand end forward while density remains high.
            // Bound by usable_len so we never expand past polyline.len().
            while end_idx < usable_len - 1 {
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
    track_map: &HashMap<&str, &[GpsPoint]>,
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

        let split_tree = build_rtree(&split_polyline);
        let threshold_deg = config.proximity_threshold / 111_000.0;
        let threshold_deg_sq = threshold_deg * threshold_deg;

        for activity_id in &section.activity_ids {
            if let Some(track) = track_map.get(activity_id.as_str()) {
                // Check if this activity overlaps with the split portion
                let mut overlap_count = 0;
                let mut overlap_distance = 0.0;
                let mut last_overlap_point: Option<&GpsPoint> = None;

                for point in *track {
                    let query = [point.latitude, point.longitude];
                    if let Some(nearest) = split_tree.nearest_neighbor(&query)
                        && nearest.distance_2(&query) <= threshold_deg_sq
                    {
                        overlap_count += 1;
                        if let Some(prev) = last_overlap_point {
                            overlap_distance += crate::geo_utils::haversine_distance(prev, point);
                        }
                        last_overlap_point = Some(point);
                    } else {
                        last_overlap_point = None;
                    }
                }

                // Need substantial overlap to count
                if overlap_count >= 3 && overlap_distance >= split_distance * 0.5 {
                    split_activity_ids.push(activity_id.clone());
                }
            }
        }

        // Only create the split section if it has enough activities
        let split_activity_count = split_activity_ids.len();
        if split_activity_count >= config.min_activities as usize {
            // Compute activity_portions for the split section. Without these,
            // save_sections() inserts zero rows into section_activities for
            // this section — the section exists in the DB but appears as
            // "0 sections attached" in the UI. This was the smoking-gun bug.
            let split_activity_portions: Vec<SectionPortion> = split_activity_ids
                .iter()
                .flat_map(|activity_id| {
                    let Some(track) = track_map.get(activity_id.as_str()) else {
                        return Vec::new();
                    };
                    find_all_track_portions(track, &split_polyline, config.proximity_threshold)
                        .into_iter()
                        .map(|(start_idx, end_idx, direction)| SectionPortion {
                            activity_id: activity_id.clone(),
                            start_index: start_idx as u32,
                            end_index: end_idx as u32,
                            distance_meters: calculate_route_distance(&track[start_idx..end_idx]),
                            direction,
                        })
                        .collect::<Vec<_>>()
                })
                .collect();

            let split_section = FrequentSection {
                id: format!("{}_split{}", section.id, split_idx),
                name: None,
                sport_type: section.sport_type.clone(),
                polyline: split_polyline,
                representative_activity_id: section.representative_activity_id.clone(),
                activity_ids: split_activity_ids,
                activity_portions: split_activity_portions,
                route_ids: section.route_ids.clone(),
                // visit_count should equal unique activities
                visit_count: split_activity_count as u32,
                distance_meters: split_distance,
                // Lazy: traces populated on-demand
                activity_traces: HashMap::new(),
                confidence: section.confidence,
                observation_count: candidate.avg_density as u32,
                average_spread: section.average_spread,
                point_density: split_density,
                scale: section.scale,
                is_user_defined: section.is_user_defined,
                stability: 0.0, // Needs recalculation
                version: 1,
                updated_at: None,
                created_at: section.created_at.clone(),
                consensus_state: None,
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
    track_map: &HashMap<&str, &[GpsPoint]>,
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
// Quality Filtering: Length-Weighted Visit Threshold
// =============================================================================
//
// Philosophy: Since users can create their own segments, automatic detection
// should be conservative. Short sections that are only visited twice are
// probably noise—but short sections visited many times are meaningful patterns.

/// Minimum visits required based on section length and dataset size.
/// Shorter sections need more visits. Larger datasets use slightly higher
/// thresholds to avoid drowning in noise — but the bonus is intentionally
/// modest so a user with 500 activities doesn't have all their valid
/// sections filtered away.
fn required_visits_for_length(distance_meters: f64, total_activities: usize) -> u32 {
    // Softened from the 43a39da bonus of (0 / +1 / +2): the +2 tier was
    // filtering out genuine sections for users with many activities. The
    // base thresholds (2-6) already encode noise rejection per length tier.
    let bonus: u32 = match total_activities {
        0..=200 => 0,
        _ => 1,
    };

    let base = match distance_meters {
        d if d < 200.0 => 6,
        d if d < 400.0 => 4,
        d if d < 800.0 => 3,
        _ => 2,
    };

    base + bonus
}

/// Quality filter: Remove low-confidence sections based on length, visits,
/// and dataset size. Larger datasets require more visits for a section to
/// survive, since the signal-to-noise ratio is lower.
pub fn filter_low_quality_sections(
    sections: Vec<FrequentSection>,
    total_activities: usize,
) -> Vec<FrequentSection> {
    let before = sections.len();
    let mut filtered: Vec<FrequentSection> = Vec::new();

    for section in sections {
        let min_visits = required_visits_for_length(section.distance_meters, total_activities);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Direction;

    /// Build a 1km north-south reference polyline at 45°N. 200 points.
    fn make_reference_polyline(num_points: usize) -> Vec<GpsPoint> {
        // 0.009 degrees latitude ≈ 1km
        let length_degrees = 0.009;
        (0..num_points)
            .map(|i| {
                GpsPoint::new(
                    45.0 + (i as f64 / (num_points - 1) as f64) * length_degrees,
                    7.0,
                )
            })
            .collect()
    }

    /// Density profile: low at endpoints, high in the middle. This forces
    /// `find_split_candidates` to identify the middle as a high-traffic
    /// portion. Endpoint density ~2, middle density ~10 → ratio of 5×.
    fn make_high_variance_density(num_points: usize) -> Vec<u32> {
        let mid = num_points / 2;
        let band = num_points / 4;
        (0..num_points)
            .map(|i| if i.abs_diff(mid) < band { 10 } else { 2 })
            .collect()
    }

    /// Walk every point of the reference plus tiny jitter so the synthesized
    /// track lies within proximity_threshold for every point.
    fn make_full_traversal(reference: &[GpsPoint]) -> Vec<GpsPoint> {
        reference
            .iter()
            .enumerate()
            .map(|(i, p)| {
                GpsPoint::new(p.latitude + 0.000005 * (i as f64 % 3.0 - 1.0), p.longitude)
            })
            .collect()
    }

    /// Regression test for the smoking-gun bug: `split_section_by_density`
    /// used to construct split sections with `activity_portions: Vec::new()`.
    /// `save_sections()` iterates over `activity_portions` to insert junction
    /// rows, so empty portions meant zero rows in `section_activities` →
    /// "0 sections attached" in the UI.
    ///
    /// This test forces the split path with a high-variance density profile
    /// and asserts the resulting split sections have non-empty portions
    /// matching the activity IDs.
    #[test]
    fn split_high_variance_sections_populates_activity_portions() {
        const NUM_POINTS: usize = 200;
        const NUM_ACTIVITIES: usize = 5;

        let reference = make_reference_polyline(NUM_POINTS);
        let density = make_high_variance_density(NUM_POINTS);

        // Synthesize N activity tracks that all traverse the reference.
        let activity_id_strings: Vec<String> = (0..NUM_ACTIVITIES)
            .map(|i| format!("activity_{i}"))
            .collect();
        let tracks: Vec<Vec<GpsPoint>> = (0..NUM_ACTIVITIES)
            .map(|_| make_full_traversal(&reference))
            .collect();

        let mut track_map: HashMap<&str, &[GpsPoint]> = HashMap::new();
        for (id, track) in activity_id_strings.iter().zip(tracks.iter()) {
            track_map.insert(id.as_str(), track.as_slice());
        }

        // Build a parent section with high density variance in its middle.
        let parent = FrequentSection {
            id: "test_parent".to_string(),
            name: None,
            sport_type: "Run".to_string(),
            polyline: reference.clone(),
            representative_activity_id: activity_id_strings[0].clone(),
            activity_ids: activity_id_strings.clone(),
            activity_portions: Vec::new(),
            route_ids: vec![],
            visit_count: NUM_ACTIVITIES as u32,
            distance_meters: calculate_route_distance(&reference),
            activity_traces: HashMap::new(),
            confidence: 0.9,
            observation_count: NUM_ACTIVITIES as u32,
            average_spread: 5.0,
            point_density: density,
            scale: None,
            is_user_defined: false,
            stability: 0.8,
            version: 1,
            updated_at: None,
            created_at: None,
            consensus_state: None,
        };

        let config = SectionConfig {
            proximity_threshold: 50.0,
            min_activities: 3,
            ..SectionConfig::default()
        };

        let split_result = split_high_variance_sections(vec![parent], &track_map, &config);

        // Expect at least one split section was created (plus the original).
        let split_sections: Vec<&FrequentSection> = split_result
            .iter()
            .filter(|s| s.id.contains("_split"))
            .collect();

        assert!(
            !split_sections.is_empty(),
            "split_high_variance_sections did not create any splits — \
             check that the synthetic density profile still triggers \
             SPLIT_DENSITY_RATIO ({SPLIT_DENSITY_RATIO})"
        );

        // The bug being regressed: every split section must have non-empty
        // activity_portions matching its activity_ids.
        for split in &split_sections {
            assert!(
                !split.activity_ids.is_empty(),
                "split section {} has empty activity_ids",
                split.id
            );
            assert!(
                !split.activity_portions.is_empty(),
                "split section {} has empty activity_portions \
                 (the smoking-gun bug)",
                split.id
            );

            // Every portion must reference one of the section's activities.
            for portion in &split.activity_portions {
                assert!(
                    split.activity_ids.contains(&portion.activity_id),
                    "split section {} has portion for unknown activity {}",
                    split.id,
                    portion.activity_id
                );
                assert!(
                    portion.end_index > portion.start_index,
                    "portion has invalid range {}..{}",
                    portion.start_index,
                    portion.end_index
                );
                // Direction must be either Same or Reverse — never garbage.
                assert!(matches!(
                    portion.direction,
                    Direction::Same | Direction::Reverse
                ));
            }
        }
    }

    /// Build a minimal FrequentSection for filter-behaviour tests.
    /// `polyline_pts` controls whether the polyline gate is met
    /// (filter requires ≥ 8 points).
    fn stub_section_for_filter(
        id: &str,
        distance_m: f64,
        visits: u32,
        polyline_pts: usize,
    ) -> FrequentSection {
        FrequentSection {
            id: id.to_string(),
            name: None,
            sport_type: "Run".to_string(),
            polyline: (0..polyline_pts)
                .map(|i| GpsPoint::new(45.0 + (i as f64) * 0.0001, 7.0))
                .collect(),
            representative_activity_id: String::new(),
            activity_ids: (0..visits).map(|i| format!("a_{i}")).collect(),
            activity_portions: Vec::new(),
            route_ids: vec![],
            visit_count: visits,
            distance_meters: distance_m,
            activity_traces: HashMap::new(),
            confidence: 0.5,
            observation_count: visits,
            average_spread: 1.0,
            point_density: vec![],
            scale: None,
            is_user_defined: false,
            stability: 0.0,
            version: 1,
            updated_at: None,
            created_at: None,
            consensus_state: None,
        }
    }

    /// Filter must DROP a 150 m section with only 2 visits — base requires 6.
    #[test]
    fn quality_filter_drops_short_low_visit() {
        let s = stub_section_for_filter("low", 150.0, 2, 20);
        let kept = filter_low_quality_sections(vec![s], 50);
        assert!(
            kept.is_empty(),
            "expected 150 m / 2-visit section to be dropped"
        );
    }

    /// Filter must KEEP a 1 km section with 2 visits — base requires 2 for ≥ 800 m.
    #[test]
    fn quality_filter_keeps_long_two_visit() {
        let s = stub_section_for_filter("long2", 1000.0, 2, 20);
        let kept = filter_low_quality_sections(vec![s], 50);
        assert_eq!(kept.len(), 1, "expected 1 km / 2-visit section to survive");
    }

    /// Filter must KEEP a short section with enough visits — 150m needs 6.
    #[test]
    fn quality_filter_keeps_short_high_visit() {
        let s = stub_section_for_filter("short_many", 150.0, 6, 20);
        let kept = filter_low_quality_sections(vec![s], 50);
        assert_eq!(kept.len(), 1);
    }

    /// Filter must DROP a section with too few polyline points.
    #[test]
    fn quality_filter_drops_underpopulated_polyline() {
        let s = stub_section_for_filter("sparse", 1000.0, 10, 5); // only 5 points
        let kept = filter_low_quality_sections(vec![s], 50);
        assert!(
            kept.is_empty(),
            "expected section with < 8 polyline points to be dropped"
        );
    }

    /// Dataset-size bonus: a 1 km / 2-visit section survives at N=50 but
    /// is filtered at N=300 (>200 → +1 bonus → requires 3 visits).
    /// Protects against future regressions of the softened bonus tier.
    #[test]
    fn quality_filter_dataset_size_bonus_kicks_in_above_200() {
        let s_small = stub_section_for_filter("k50", 1000.0, 2, 20);
        let kept_small = filter_low_quality_sections(vec![s_small], 50);
        assert_eq!(kept_small.len(), 1, "should survive at N=50");

        let s_large = stub_section_for_filter("k300", 1000.0, 2, 20);
        let kept_large = filter_low_quality_sections(vec![s_large], 300);
        assert!(
            kept_large.is_empty(),
            "expected dataset-size bonus to drop 2-visit section at N=300"
        );
    }

    /// Bug B regression: the prior bonus tier was (0 / +1 / +2). The +2
    /// tier at N>200 was filtering valid sections. Verify the softened
    /// tier (0 / +1) doesn't reintroduce the over-filtering.
    #[test]
    fn quality_filter_bonus_does_not_overfilter_at_large_n() {
        // 1 km section with 3 visits at N=500: base=2, bonus=1, requires 3 → keeps.
        let s = stub_section_for_filter("survivor", 1000.0, 3, 20);
        let kept = filter_low_quality_sections(vec![s], 500);
        assert_eq!(
            kept.len(),
            1,
            "1 km / 3-visit section at N=500 should survive softened bonus"
        );
    }
}
