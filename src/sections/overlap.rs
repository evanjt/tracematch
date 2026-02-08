//! Full track overlap detection and clustering.

use super::SectionConfig;
use super::rtree::IndexedPoint;
use crate::GpsPoint;
use crate::geo_utils::{compute_center, haversine_distance};
use rstar::{PointDistance, RTree};
use std::collections::HashSet;

/// A detected overlap between two full GPS tracks.
/// Uses index ranges into original tracks instead of copying GPS points,
/// reducing memory from ~48KB to ~100 bytes per overlap (~480x reduction).
#[derive(Debug, Clone)]
pub struct FullTrackOverlap {
    pub activity_a: String,
    pub activity_b: String,
    /// Index range into track A's original points (start..end)
    pub range_a: (usize, usize),
    /// Index range into track B's original points (start..end)
    pub range_b: (usize, usize),
    /// Center point for clustering
    pub center: GpsPoint,
    /// Pre-computed overlap length in meters
    pub overlap_length: f64,
}

/// A cluster of overlaps representing the same physical section
#[derive(Debug)]
pub struct OverlapCluster {
    /// All overlaps in this cluster
    pub overlaps: Vec<FullTrackOverlap>,
    /// Unique activity IDs in this cluster
    pub activity_ids: HashSet<String>,
}

/// Adaptive hierarchical overlap pre-check.
///
/// Uses a two-level segment-based approach to quickly determine if two tracks
/// could possibly overlap, without scanning all points. Mathematically proven
/// to have zero false negatives via the triangle inequality.
///
/// Returns:
/// - `Some(true)` if overlap is proven (at least one point is within threshold)
/// - `Some(false)` if no overlap is proven (all segments provably outside threshold)
/// - `None` if inconclusive (must fall through to full scan)
pub(crate) fn has_any_overlap(
    track_a: &[GpsPoint],
    tree_b: &RTree<IndexedPoint>,
    threshold_deg_sq: f64,
) -> Option<bool> {
    let n = track_a.len();
    if n < 2 {
        return None;
    }

    let threshold_deg = threshold_deg_sq.sqrt();

    // Level 1: divide track_a into K segments
    let k = 10.min(n);
    let seg_size = n / k;
    if seg_size == 0 {
        return None;
    }

    let mut warm_segments: Vec<(usize, usize)> = Vec::new();

    for s in 0..k {
        let seg_start = s * seg_size;
        let seg_end = if s == k - 1 { n } else { (s + 1) * seg_size };

        // Compute segment center
        let center = &track_a[seg_start + (seg_end - seg_start) / 2];

        // Compute segment radius: max distance from center to any point in segment
        // Using degree-space for consistency with R-tree queries
        let mut radius_sq: f64 = 0.0;
        for p in &track_a[seg_start..seg_end] {
            let dlat = p.latitude - center.latitude;
            let dlng = p.longitude - center.longitude;
            let d_sq = dlat * dlat + dlng * dlng;
            if d_sq > radius_sq {
                radius_sq = d_sq;
            }
        }
        let radius = radius_sq.sqrt();

        // Query R-tree for nearest neighbor to segment center
        let query = [center.latitude, center.longitude];
        if let Some(nearest) = tree_b.nearest_neighbor(&query) {
            let dist_sq = nearest.distance_2(&query);
            let dist = dist_sq.sqrt();

            // DIRECT HIT: center itself is within threshold
            if dist_sq <= threshold_deg_sq {
                return Some(true);
            }

            // COLD: proven no overlap in this segment (triangle inequality)
            // If dist >= threshold + radius, then for ANY point p in segment:
            //   dist(p, nearest_in_B) >= dist(center, nearest_in_B) - dist(center, p)
            //                         >= dist - radius >= threshold
            if dist >= threshold_deg + radius {
                continue; // Proven cold
            }

            // WARM: can't rule out overlap
            warm_segments.push((seg_start, seg_end));
        }
    }

    // If no warm segments, all are cold → proven no overlap
    if warm_segments.is_empty() {
        return Some(false);
    }

    // Level 2: subdivide warm segments into 3 sub-segments
    let mut any_still_warm = false;

    for (seg_start, seg_end) in &warm_segments {
        let seg_len = seg_end - seg_start;
        let sub_k = 3.min(seg_len);
        if sub_k == 0 {
            any_still_warm = true;
            continue;
        }
        let sub_size = seg_len / sub_k;
        if sub_size == 0 {
            any_still_warm = true;
            continue;
        }

        for ss in 0..sub_k {
            let sub_start = seg_start + ss * sub_size;
            let sub_end = if ss == sub_k - 1 { *seg_end } else { seg_start + (ss + 1) * sub_size };

            let center = &track_a[sub_start + (sub_end - sub_start) / 2];

            let mut radius_sq: f64 = 0.0;
            for p in &track_a[sub_start..sub_end] {
                let dlat = p.latitude - center.latitude;
                let dlng = p.longitude - center.longitude;
                let d_sq = dlat * dlat + dlng * dlng;
                if d_sq > radius_sq {
                    radius_sq = d_sq;
                }
            }
            let radius = radius_sq.sqrt();

            let query = [center.latitude, center.longitude];
            if let Some(nearest) = tree_b.nearest_neighbor(&query) {
                let dist_sq = nearest.distance_2(&query);
                let dist = dist_sq.sqrt();

                if dist_sq <= threshold_deg_sq {
                    return Some(true);
                }

                if dist >= threshold_deg + radius {
                    continue; // Proven cold at sub-segment level
                }

                any_still_warm = true;
            }
        }
    }

    if !any_still_warm {
        return Some(false);
    }

    None // Inconclusive → fall through to full scan
}

/// Find overlapping portion between two FULL GPS tracks.
/// Returns index ranges into the original tracks instead of copying points.
pub fn find_full_track_overlap(
    activity_a: &str,
    track_a: &[GpsPoint],
    activity_b: &str,
    track_b: &[GpsPoint],
    tree_b: &RTree<IndexedPoint>,
    config: &SectionConfig,
) -> Option<FullTrackOverlap> {
    // Convert proximity threshold from meters to approximate degrees
    // 1 degree ≈ 111km, so 30m ≈ 0.00027 degrees
    let threshold_deg = config.proximity_threshold / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;

    // Adaptive pre-check: skip full scan if provably no overlap
    match has_any_overlap(track_a, tree_b, threshold_deg_sq) {
        Some(false) => return None,
        _ => {} // Some(true) or None → proceed to full scan
    }

    let mut best_start_a: Option<usize> = None;
    let mut best_end_a = 0;
    let mut best_min_b = usize::MAX;
    let mut best_max_b = 0;
    let mut best_length = 0.0;

    let mut current_start_a: Option<usize> = None;
    let mut current_min_b = usize::MAX;
    let mut current_max_b = 0;
    let mut current_length = 0.0;

    for (i, point_a) in track_a.iter().enumerate() {
        // Use R-tree to find nearest point in track B
        let query_point = [point_a.latitude, point_a.longitude];

        if let Some(nearest) = tree_b.nearest_neighbor(&query_point) {
            let dist_sq = nearest.distance_2(&query_point);

            if dist_sq <= threshold_deg_sq {
                // Point is within threshold
                if current_start_a.is_none() {
                    current_start_a = Some(i);
                    current_min_b = nearest.idx;
                    current_max_b = nearest.idx;
                    current_length = 0.0;
                } else {
                    current_min_b = current_min_b.min(nearest.idx);
                    current_max_b = current_max_b.max(nearest.idx);
                }

                // Accumulate distance
                if i > 0 {
                    current_length += haversine_distance(&track_a[i - 1], point_a);
                }
            } else {
                // Gap - check if current sequence is substantial
                if let Some(start_a) = current_start_a
                    && current_length >= config.min_section_length
                    && current_length > best_length
                {
                    best_start_a = Some(start_a);
                    best_end_a = i;
                    best_min_b = current_min_b;
                    best_max_b = current_max_b;
                    best_length = current_length;
                }
                current_start_a = None;
                current_length = 0.0;
                current_min_b = usize::MAX;
                current_max_b = 0;
            }
        }
    }

    // Check final sequence
    if let Some(start_a) = current_start_a
        && current_length >= config.min_section_length
        && current_length > best_length
    {
        best_start_a = Some(start_a);
        best_end_a = track_a.len();
        best_min_b = current_min_b;
        best_max_b = current_max_b;
        best_length = current_length;
    }

    // Build result if we found a substantial overlap
    best_start_a.map(|start_a| {
        let a_end = best_end_a;
        let b_start = best_min_b;
        let b_end = (best_max_b + 1).min(track_b.len());

        let center = compute_center(&track_a[start_a..a_end]);

        FullTrackOverlap {
            activity_a: activity_a.to_string(),
            activity_b: activity_b.to_string(),
            range_a: (start_a, a_end),
            range_b: (b_start, b_end),
            center,
            overlap_length: best_length,
        }
    })
}

/// Resolve overlap points from original tracks using stored index ranges.
pub fn resolve_points_a<'a>(
    overlap: &FullTrackOverlap,
    tracks: &'a [(&str, &[GpsPoint])],
) -> &'a [GpsPoint] {
    for (id, pts) in tracks {
        if *id == overlap.activity_a {
            let end = overlap.range_a.1.min(pts.len());
            return &pts[overlap.range_a.0..end];
        }
    }
    &[]
}

/// Resolve overlap points from track_map using stored index ranges.
pub fn resolve_points_a_from_map<'a>(
    overlap: &FullTrackOverlap,
    track_map: &'a std::collections::HashMap<&str, &[GpsPoint]>,
) -> &'a [GpsPoint] {
    if let Some(pts) = track_map.get(overlap.activity_a.as_str()) {
        let end = overlap.range_a.1.min(pts.len());
        &pts[overlap.range_a.0..end]
    } else {
        &[]
    }
}

/// Cluster overlaps that represent the same physical section.
/// Requires access to original tracks to resolve points for geometric matching.
pub fn cluster_overlaps(
    overlaps: Vec<FullTrackOverlap>,
    config: &SectionConfig,
    tracks: &[(&str, &[GpsPoint])],
) -> Vec<OverlapCluster> {
    if overlaps.is_empty() {
        return vec![];
    }

    let mut clusters: Vec<OverlapCluster> = Vec::new();
    let mut assigned: HashSet<usize> = HashSet::new();

    for (i, overlap) in overlaps.iter().enumerate() {
        if assigned.contains(&i) {
            continue;
        }

        // Start new cluster with this overlap
        let mut cluster_overlaps = vec![overlap.clone()];
        let mut cluster_activities: HashSet<String> = HashSet::new();
        cluster_activities.insert(overlap.activity_a.clone());
        cluster_activities.insert(overlap.activity_b.clone());
        assigned.insert(i);

        // Resolve points for the seed overlap
        let seed_points = resolve_points_a(overlap, tracks);

        // Find other overlaps that belong to this cluster
        for (j, other) in overlaps.iter().enumerate() {
            if assigned.contains(&j) {
                continue;
            }

            // Check if centers are close enough
            let center_dist = haversine_distance(&overlap.center, &other.center);
            if center_dist <= config.cluster_tolerance {
                // Additional check: verify overlaps are geometrically similar
                let other_points = resolve_points_a(other, tracks);
                if overlaps_match(seed_points, other_points, config.proximity_threshold) {
                    cluster_overlaps.push(other.clone());
                    cluster_activities.insert(other.activity_a.clone());
                    cluster_activities.insert(other.activity_b.clone());
                    assigned.insert(j);
                }
            }
        }

        clusters.push(OverlapCluster {
            overlaps: cluster_overlaps,
            activity_ids: cluster_activities,
        });
    }

    clusters
}

/// Cluster overlaps using a HashMap-based track lookup.
/// Used by the multiscale path where tracks are stored in a HashMap.
pub fn cluster_overlaps_with_map(
    overlaps: Vec<FullTrackOverlap>,
    config: &SectionConfig,
    track_map: &std::collections::HashMap<&str, &[GpsPoint]>,
) -> Vec<OverlapCluster> {
    if overlaps.is_empty() {
        return vec![];
    }

    let mut clusters: Vec<OverlapCluster> = Vec::new();
    let mut assigned: HashSet<usize> = HashSet::new();

    for (i, overlap) in overlaps.iter().enumerate() {
        if assigned.contains(&i) {
            continue;
        }

        let mut cluster_overlaps = vec![overlap.clone()];
        let mut cluster_activities: HashSet<String> = HashSet::new();
        cluster_activities.insert(overlap.activity_a.clone());
        cluster_activities.insert(overlap.activity_b.clone());
        assigned.insert(i);

        let seed_points = resolve_points_a_from_map(overlap, track_map);

        for (j, other) in overlaps.iter().enumerate() {
            if assigned.contains(&j) {
                continue;
            }

            let center_dist = haversine_distance(&overlap.center, &other.center);
            if center_dist <= config.cluster_tolerance {
                let other_points = resolve_points_a_from_map(other, track_map);
                if overlaps_match(seed_points, other_points, config.proximity_threshold) {
                    cluster_overlaps.push(other.clone());
                    cluster_activities.insert(other.activity_a.clone());
                    cluster_activities.insert(other.activity_b.clone());
                    assigned.insert(j);
                }
            }
        }

        clusters.push(OverlapCluster {
            overlaps: cluster_overlaps,
            activity_ids: cluster_activities,
        });
    }

    clusters
}

/// Check if two polylines overlap geometrically (using R-tree for O(log n) lookups).
fn overlaps_match(poly_a: &[GpsPoint], poly_b: &[GpsPoint], threshold: f64) -> bool {
    use super::rtree::build_rtree;
    use rstar::PointDistance;

    if poly_a.is_empty() || poly_b.is_empty() {
        return false;
    }

    // Build R-tree for poly_b (O(n log n)) - enables O(log n) nearest neighbor queries
    let tree_b = build_rtree(poly_b);
    let threshold_deg = threshold / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;

    // Sample points from poly_a and check how many are close to poly_b
    let sample_count = 10.min(poly_a.len());
    let step = poly_a.len() / sample_count;
    let mut matches = 0;

    for i in (0..poly_a.len()).step_by(step.max(1)).take(sample_count) {
        let point = &poly_a[i];
        let query = [point.latitude, point.longitude];

        // Use R-tree for O(log n) nearest neighbor lookup instead of O(n) linear scan
        if let Some(nearest) = tree_b.nearest_neighbor(&query)
            && nearest.distance_2(&query) <= threshold_deg_sq
        {
            matches += 1;
        }
    }

    // Need at least 50% of samples to match
    matches >= sample_count / 2
}
