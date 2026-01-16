//! Route grouping algorithms.
//!
//! This module provides functionality to group similar routes together
//! using spatial indexing and Union-Find for efficient grouping.

use rstar::{RTree, AABB};
use std::collections::HashMap;

use crate::geo_utils::haversine_distance;
use crate::matching::{calculate_checkpoint_match, compare_routes};
use crate::union_find::UnionFind;
use crate::{
    ActivityMatchInfo, Bounds, GpsPoint, GroupingResult, MatchConfig, MatchResult, RouteBounds,
    RouteGroup, RouteSignature,
};

/// Spatial search tolerance in degrees (~1km).
const SPATIAL_TOLERANCE: f64 = 0.01;

/// Result type for parallel group processing - groups and their match info.
type GroupProcessingResult = (Vec<RouteGroup>, HashMap<String, Vec<ActivityMatchInfo>>);

/// Check if two routes should be GROUPED into the same route.
///
/// A "route" is a complete, repeated JOURNEY - not just a shared section.
/// Two activities are the same route only if they represent the same end-to-end trip.
///
/// Criteria:
/// 1. Both routes must be at least min_route_distance
/// 2. Match percentage meets threshold
/// 3. Similar total distance (within max_distance_diff_ratio)
/// 4. Same endpoints (within endpoint_threshold)
/// 5. Middle points must also match
pub fn should_group_routes(
    sig1: &RouteSignature,
    sig2: &RouteSignature,
    match_result: &MatchResult,
    config: &MatchConfig,
) -> bool {
    // CHECK 0: Both routes must be meaningful length
    if sig1.total_distance < config.min_route_distance
        || sig2.total_distance < config.min_route_distance
    {
        return false;
    }

    // CHECK 1: Match percentage must be high enough
    if match_result.match_percentage < config.min_match_percentage {
        return false;
    }

    // CHECK 2: Total distance must be similar
    let distance_diff = (sig1.total_distance - sig2.total_distance).abs();
    let max_distance = sig1.total_distance.max(sig2.total_distance);
    if max_distance > 0.0 && distance_diff / max_distance > config.max_distance_diff_ratio {
        return false;
    }

    // CHECK 3: Endpoints must match closely
    let start1 = &sig1.start_point;
    let end1 = &sig1.end_point;
    let start2 = &sig2.start_point;
    let end2 = &sig2.end_point;

    // Check if routes are loops (start and end are close)
    let sig1_is_loop = haversine_distance(start1, end1) < config.endpoint_threshold;
    let sig2_is_loop = haversine_distance(start2, end2) < config.endpoint_threshold;

    // For loops, the start point is arbitrary - you can start anywhere on the loop.
    // So we DON'T require starts to match. Instead, we check:
    // 1. Both must be loops
    // 2. Their start points must be somewhere on the OTHER loop (within tolerance)
    // 3. Middle points must match (using distance-based comparison)
    if sig1_is_loop && sig2_is_loop {
        // Check if sig1's start is anywhere near sig2's path
        let sig1_start_near_sig2 =
            is_point_near_route(start1, &sig2.points, config.endpoint_threshold);
        // Check if sig2's start is anywhere near sig1's path
        let sig2_start_near_sig1 =
            is_point_near_route(start2, &sig1.points, config.endpoint_threshold);

        if !sig1_start_near_sig2 || !sig2_start_near_sig1 {
            return false;
        }

        // For loops, middle point check is less meaningful since "middle" depends on start.
        // Just rely on the AMD match percentage check that already passed.
        return true;
    }

    // Determine direction by checking which endpoint pairing is closer
    let same_start_dist = haversine_distance(start1, start2);
    let same_end_dist = haversine_distance(end1, end2);
    let reverse_start_dist = haversine_distance(start1, end2);
    let reverse_end_dist = haversine_distance(end1, start2);

    let same_direction_ok =
        same_start_dist < config.endpoint_threshold && same_end_dist < config.endpoint_threshold;
    let reverse_direction_ok = reverse_start_dist < config.endpoint_threshold
        && reverse_end_dist < config.endpoint_threshold;

    if !same_direction_ok && !reverse_direction_ok {
        return false;
    }

    // CHECK 4: Middle points must also match
    let points2_for_middle: Vec<GpsPoint> = if reverse_direction_ok && !same_direction_ok {
        sig2.points.iter().rev().cloned().collect()
    } else {
        sig2.points.clone()
    };

    check_middle_points_match(
        &sig1.points,
        &points2_for_middle,
        config.endpoint_threshold * 2.0,
    )
}

/// Check that the middle portions of two routes also match.
///
/// Uses 7 checkpoint positions (was 3) to catch divergences that occur
/// between the sparse 25/50/75% checkpoints. This catches routes that
/// share start/end but diverge in the middle.
pub fn check_middle_points_match(
    points1: &[GpsPoint],
    points2: &[GpsPoint],
    threshold: f64,
) -> bool {
    if points1.len() < 5 || points2.len() < 5 {
        return true; // Not enough points to check middle
    }

    // Check 7 positions along each route (was [0.25, 0.5, 0.75])
    // More checkpoints catch divergences that slip through sparse sampling
    let check_positions = [0.15, 0.25, 0.35, 0.5, 0.65, 0.75, 0.85];

    for pos in check_positions {
        let idx1 = ((points1.len() - 1) as f64 * pos) as usize;
        let idx2 = ((points2.len() - 1) as f64 * pos) as usize;

        let p1 = &points1[idx1];
        let p2 = &points2[idx2];

        let dist = haversine_distance(p1, p2);
        if dist > threshold {
            return false;
        }
    }

    true
}

/// Group similar routes together.
///
/// Uses an R-tree spatial index for pre-filtering and Union-Find
/// for efficient grouping. Routes that match are grouped together
/// only if they pass strict grouping criteria (same journey, not just shared sections).
pub fn group_signatures(signatures: &[RouteSignature], config: &MatchConfig) -> Vec<RouteGroup> {
    if signatures.is_empty() {
        return vec![];
    }

    // Build spatial index
    let bounds: Vec<RouteBounds> = signatures.iter().map(|s| s.route_bounds()).collect();
    let rtree = RTree::bulk_load(bounds);

    // Create signature lookup
    let sig_map: HashMap<&str, &RouteSignature> = signatures
        .iter()
        .map(|s| (s.activity_id.as_str(), s))
        .collect();

    // Union-Find using our new type
    let mut uf = UnionFind::with_capacity(signatures.len());
    for sig in signatures {
        uf.make_set(sig.activity_id.clone());
    }

    // Find matching pairs
    for sig1 in signatures {
        let search_bounds = create_search_bounds(&sig1.points, SPATIAL_TOLERANCE);

        for bounds in rtree.locate_in_envelope_intersecting(&search_bounds) {
            // Skip self and already-processed pairs
            if bounds.activity_id == sig1.activity_id {
                continue;
            }
            if sig1.activity_id >= bounds.activity_id {
                continue;
            }

            // Distance pre-filter
            if !distance_ratio_ok(sig1.total_distance, bounds.distance) {
                continue;
            }

            if let Some(sig2) = sig_map.get(bounds.activity_id.as_str()) {
                // Only group if match exists AND passes strict grouping criteria
                if let Some(match_result) = compare_routes(sig1, sig2, config) {
                    if should_group_routes(sig1, sig2, &match_result, config) {
                        uf.union(&sig1.activity_id, &bounds.activity_id);
                    }
                }
            }
        }
    }

    // Build groups from Union-Find
    let groups_map = uf.groups();
    build_route_groups(groups_map, &sig_map)
}

/// Group similar routes together and capture match info for each activity.
///
/// Returns both the groups and per-activity match percentages.
/// The match info is calculated by comparing each activity to the group's representative.
///
/// Also performs auto-splitting: if a subset of activities in a group has low match %
/// to the representative (< 80%), they are split into a separate route group.
pub fn group_signatures_with_matches(
    signatures: &[RouteSignature],
    config: &MatchConfig,
) -> GroupingResult {
    if signatures.is_empty() {
        return GroupingResult {
            groups: vec![],
            activity_matches: HashMap::new(),
        };
    }

    // First, do the normal grouping
    let initial_groups = group_signatures(signatures, config);

    // Create signature lookup
    let sig_map: HashMap<&str, &RouteSignature> = signatures
        .iter()
        .map(|s| (s.activity_id.as_str(), s))
        .collect();

    // Calculate match info and split divergent groups
    let mut final_groups: Vec<RouteGroup> = Vec::new();
    let mut activity_matches: HashMap<String, Vec<ActivityMatchInfo>> = HashMap::new();

    for group in &initial_groups {
        let representative_sig = match sig_map.get(group.representative_id.as_str()) {
            Some(sig) => *sig,
            None => {
                final_groups.push(group.clone());
                continue;
            }
        };

        // Calculate match info for each activity using checkpoint-based matching
        let mut matches = Vec::new();
        for activity_id in &group.activity_ids {
            if let Some(activity_sig) = sig_map.get(activity_id.as_str()) {
                let (match_percentage, direction) = if activity_id == &group.representative_id {
                    (100.0, "same".to_string())
                } else {
                    let checkpoint_match = calculate_checkpoint_match(
                        &activity_sig.points,
                        &representative_sig.points,
                        config,
                    );
                    let dir = compare_routes(activity_sig, representative_sig, config)
                        .map(|r| r.direction)
                        .unwrap_or_else(|| "same".to_string());
                    (checkpoint_match, dir)
                };

                matches.push(ActivityMatchInfo {
                    activity_id: activity_id.clone(),
                    match_percentage,
                    direction,
                });
            }
        }

        // Check if this group should be split
        let split_groups = split_divergent_routes(group, &matches, &sig_map, config);

        if split_groups.len() == 1 {
            // No split occurred - use original group and matches
            final_groups.push(group.clone());
            activity_matches.insert(group.group_id.clone(), matches);
        } else {
            // Group was split - recalculate matches for each sub-group
            for split_group in split_groups {
                let split_rep_sig = match sig_map.get(split_group.representative_id.as_str()) {
                    Some(sig) => sig,
                    None => {
                        final_groups.push(split_group);
                        continue;
                    }
                };

                let split_matches: Vec<ActivityMatchInfo> = split_group
                    .activity_ids
                    .iter()
                    .filter_map(|activity_id| {
                        let activity_sig = sig_map.get(activity_id.as_str())?;
                        let (match_percentage, direction) =
                            if activity_id == &split_group.representative_id {
                                (100.0, "same".to_string())
                            } else {
                                let checkpoint_match = calculate_checkpoint_match(
                                    &activity_sig.points,
                                    &split_rep_sig.points,
                                    config,
                                );
                                let dir = compare_routes(activity_sig, split_rep_sig, config)
                                    .map(|r| r.direction)
                                    .unwrap_or_else(|| "same".to_string());
                                (checkpoint_match, dir)
                            };

                        Some(ActivityMatchInfo {
                            activity_id: activity_id.clone(),
                            match_percentage,
                            direction,
                        })
                    })
                    .collect();

                activity_matches.insert(split_group.group_id.clone(), split_matches);
                final_groups.push(split_group);
            }
        }
    }

    GroupingResult {
        groups: final_groups,
        activity_matches,
    }
}

/// Group signatures using parallel processing.
///
/// This is the same as `group_signatures` but uses rayon for parallel
/// comparison of route pairs. Recommended for large datasets (100+ routes).
#[cfg(feature = "parallel")]
pub fn group_signatures_parallel(
    signatures: &[RouteSignature],
    config: &MatchConfig,
) -> Vec<RouteGroup> {
    use rayon::prelude::*;

    if signatures.is_empty() {
        return vec![];
    }

    // Build spatial index
    let bounds: Vec<RouteBounds> = signatures.iter().map(|s| s.route_bounds()).collect();
    let rtree = RTree::bulk_load(bounds);

    // Create signature lookup
    let sig_map: HashMap<&str, &RouteSignature> = signatures
        .iter()
        .map(|s| (s.activity_id.as_str(), s))
        .collect();

    // Find matches in parallel (with strict grouping criteria)
    let matches: Vec<(String, String)> = signatures
        .par_iter()
        .flat_map(|sig1| {
            let search_bounds = create_search_bounds(&sig1.points, SPATIAL_TOLERANCE);

            rtree
                .locate_in_envelope_intersecting(&search_bounds)
                .filter(|b| {
                    b.activity_id != sig1.activity_id
                        && sig1.activity_id < b.activity_id
                        && distance_ratio_ok(sig1.total_distance, b.distance)
                })
                .filter_map(|b| {
                    let sig2 = sig_map.get(b.activity_id.as_str())?;
                    let match_result = compare_routes(sig1, sig2, config)?;
                    // Only group if passes strict grouping criteria
                    if should_group_routes(sig1, sig2, &match_result, config) {
                        Some((sig1.activity_id.clone(), sig2.activity_id.clone()))
                    } else {
                        None
                    }
                })
                .collect::<Vec<_>>()
        })
        .collect();

    // Union-Find (sequential - fast enough)
    let mut uf = UnionFind::with_capacity(signatures.len());
    for sig in signatures {
        uf.make_set(sig.activity_id.clone());
    }

    for (id1, id2) in matches {
        uf.union(&id1, &id2);
    }

    // Build groups from Union-Find
    let groups_map = uf.groups();
    build_route_groups(groups_map, &sig_map)
}

/// Group signatures in parallel and capture match info for each activity.
///
/// Returns both the groups and per-activity match percentages.
/// Also performs auto-splitting for divergent activities.
#[cfg(feature = "parallel")]
pub fn group_signatures_parallel_with_matches(
    signatures: &[RouteSignature],
    config: &MatchConfig,
) -> GroupingResult {
    use rayon::prelude::*;

    if signatures.is_empty() {
        return GroupingResult {
            groups: vec![],
            activity_matches: HashMap::new(),
        };
    }

    // First, do the parallel grouping
    let initial_groups = group_signatures_parallel(signatures, config);

    // Create signature lookup
    let sig_map: HashMap<&str, &RouteSignature> = signatures
        .iter()
        .map(|s| (s.activity_id.as_str(), s))
        .collect();

    // Calculate match info for each activity and check for splits
    // Process groups in parallel, collect results
    let group_results: Vec<GroupProcessingResult> = initial_groups
        .par_iter()
        .filter_map(|group| {
            let representative_sig = sig_map.get(group.representative_id.as_str())?;

            // Calculate matches for all activities in group
            let matches: Vec<ActivityMatchInfo> = group
                .activity_ids
                .iter()
                .filter_map(|activity_id| {
                    let activity_sig = sig_map.get(activity_id.as_str())?;

                    let (match_percentage, direction) = if activity_id == &group.representative_id {
                        (100.0, "same".to_string())
                    } else {
                        let checkpoint_match = calculate_checkpoint_match(
                            &activity_sig.points,
                            &representative_sig.points,
                            config,
                        );
                        let dir = compare_routes(activity_sig, representative_sig, config)
                            .map(|r| r.direction)
                            .unwrap_or_else(|| "same".to_string());
                        (checkpoint_match, dir)
                    };

                    Some(ActivityMatchInfo {
                        activity_id: activity_id.clone(),
                        match_percentage,
                        direction,
                    })
                })
                .collect();

            // Check for splits
            let split_groups = split_divergent_routes(group, &matches, &sig_map, config);

            if split_groups.len() == 1 {
                // No split - return original
                let mut result_matches = HashMap::new();
                result_matches.insert(group.group_id.clone(), matches);
                Some((vec![group.clone()], result_matches))
            } else {
                // Split occurred - recalculate matches for each sub-group
                let mut result_groups = Vec::new();
                let mut result_matches = HashMap::new();

                for split_group in split_groups {
                    let split_rep_sig = match sig_map.get(split_group.representative_id.as_str()) {
                        Some(sig) => sig,
                        None => {
                            result_groups.push(split_group);
                            continue;
                        }
                    };

                    let split_match_info: Vec<ActivityMatchInfo> = split_group
                        .activity_ids
                        .iter()
                        .filter_map(|activity_id| {
                            let activity_sig = sig_map.get(activity_id.as_str())?;
                            let (match_percentage, direction) =
                                if activity_id == &split_group.representative_id {
                                    (100.0, "same".to_string())
                                } else {
                                    let checkpoint_match = calculate_checkpoint_match(
                                        &activity_sig.points,
                                        &split_rep_sig.points,
                                        config,
                                    );
                                    let dir = compare_routes(activity_sig, split_rep_sig, config)
                                        .map(|r| r.direction)
                                        .unwrap_or_else(|| "same".to_string());
                                    (checkpoint_match, dir)
                                };

                            Some(ActivityMatchInfo {
                                activity_id: activity_id.clone(),
                                match_percentage,
                                direction,
                            })
                        })
                        .collect();

                    result_matches.insert(split_group.group_id.clone(), split_match_info);
                    result_groups.push(split_group);
                }

                Some((result_groups, result_matches))
            }
        })
        .collect();

    // Flatten results
    let mut final_groups = Vec::new();
    let mut activity_matches = HashMap::new();

    for (groups, matches) in group_results {
        final_groups.extend(groups);
        activity_matches.extend(matches);
    }

    GroupingResult {
        groups: final_groups,
        activity_matches,
    }
}

/// Incremental grouping: efficiently add new signatures to existing groups.
///
/// This is much faster than re-grouping all signatures when adding new activities:
/// - O(n×m) instead of O(n²) where n = existing, m = new
/// - Only compares: new vs existing AND new vs new
/// - Existing signatures are NOT compared against each other (already grouped)
#[cfg(feature = "parallel")]
pub fn group_incremental(
    new_signatures: &[RouteSignature],
    existing_groups: &[RouteGroup],
    existing_signatures: &[RouteSignature],
    config: &MatchConfig,
) -> Vec<RouteGroup> {
    use rayon::prelude::*;
    use std::collections::HashSet;

    if new_signatures.is_empty() {
        return existing_groups.to_vec();
    }

    if existing_groups.is_empty() {
        // No existing groups - just group the new signatures
        return group_signatures_parallel(new_signatures, config);
    }

    // Combine all signatures for R-tree indexing
    let all_signatures: Vec<&RouteSignature> = existing_signatures
        .iter()
        .chain(new_signatures.iter())
        .collect();

    // Build spatial index from all signatures
    let all_bounds: Vec<RouteBounds> = all_signatures.iter().map(|s| s.route_bounds()).collect();
    let rtree = RTree::bulk_load(all_bounds);

    // Create signature lookup
    let sig_map: HashMap<&str, &RouteSignature> = all_signatures
        .iter()
        .map(|s| (s.activity_id.as_str(), *s))
        .collect();

    // Set of new signature IDs for fast lookup
    let new_ids: HashSet<&str> = new_signatures
        .iter()
        .map(|s| s.activity_id.as_str())
        .collect();

    // Initialize Union-Find with existing group structure
    let mut uf = UnionFind::with_capacity(all_signatures.len());

    // For existing groups: union all members together
    for group in existing_groups {
        if group.activity_ids.len() > 1 {
            let first = &group.activity_ids[0];
            uf.make_set(first.clone());
            for id in group.activity_ids.iter().skip(1) {
                uf.make_set(id.clone());
                uf.union(first, id);
            }
        } else if !group.activity_ids.is_empty() {
            uf.make_set(group.activity_ids[0].clone());
        }
    }

    // For new signatures: each is its own set initially
    for sig in new_signatures {
        uf.make_set(sig.activity_id.clone());
    }

    // Find matches in parallel - but ONLY where at least one signature is new
    let matches: Vec<(String, String)> = new_signatures
        .par_iter()
        .flat_map(|new_sig| {
            let search_bounds = AABB::from_corners(
                [
                    new_sig.bounds.min_lng - SPATIAL_TOLERANCE,
                    new_sig.bounds.min_lat - SPATIAL_TOLERANCE,
                ],
                [
                    new_sig.bounds.max_lng + SPATIAL_TOLERANCE,
                    new_sig.bounds.max_lat + SPATIAL_TOLERANCE,
                ],
            );

            rtree
                .locate_in_envelope_intersecting(&search_bounds)
                .filter(|b| {
                    b.activity_id != new_sig.activity_id
                        && distance_ratio_ok(new_sig.total_distance, b.distance)
                })
                .filter_map(|b| {
                    let other_sig = sig_map.get(b.activity_id.as_str())?;

                    // Skip if both are existing (they're already grouped)
                    let other_is_new = new_ids.contains(b.activity_id.as_str());
                    if other_is_new && new_sig.activity_id >= b.activity_id {
                        // new vs new - only check once (lexicographic ordering)
                        return None;
                    }

                    let match_result = compare_routes(new_sig, other_sig, config)?;
                    if should_group_routes(new_sig, other_sig, &match_result, config) {
                        Some((new_sig.activity_id.clone(), b.activity_id.clone()))
                    } else {
                        None
                    }
                })
                .collect::<Vec<_>>()
        })
        .collect();

    // Apply matches to Union-Find
    for (id1, id2) in matches {
        uf.union(&id1, &id2);
    }

    // Build groups from Union-Find
    let groups_map = uf.groups();
    build_route_groups(groups_map, &sig_map)
}

/// Build RouteGroup instances with full metadata from grouped activity IDs.
fn build_route_groups(
    groups_map: HashMap<String, Vec<String>>,
    sig_map: &HashMap<&str, &RouteSignature>,
) -> Vec<RouteGroup> {
    groups_map
        .into_iter()
        .map(|(group_id, activity_ids)| {
            // Find representative signature (first in group)
            let representative_id = activity_ids.first().cloned().unwrap_or_default();

            // Get sport type from first signature (empty for now - caller should set)
            let sport_type = String::new();

            // Compute combined bounds from all signatures in group
            let bounds = compute_group_bounds(&activity_ids, sig_map);

            RouteGroup {
                group_id,
                representative_id,
                activity_ids,
                sport_type,
                bounds,
                custom_name: None,
                // Performance stats populated by engine when metrics are available
                best_time: None,
                avg_time: None,
                best_pace: None,
                best_activity_id: None,
            }
        })
        .collect()
}

/// Compute combined bounds for a group of activity IDs.
fn compute_group_bounds(
    activity_ids: &[String],
    sig_map: &HashMap<&str, &RouteSignature>,
) -> Option<Bounds> {
    let group_sigs: Vec<_> = activity_ids
        .iter()
        .filter_map(|id| sig_map.get(id.as_str()))
        .collect();

    if group_sigs.is_empty() {
        return None;
    }

    let mut min_lat = f64::MAX;
    let mut max_lat = f64::MIN;
    let mut min_lng = f64::MAX;
    let mut max_lng = f64::MIN;

    for sig in group_sigs {
        min_lat = min_lat.min(sig.bounds.min_lat);
        max_lat = max_lat.max(sig.bounds.max_lat);
        min_lng = min_lng.min(sig.bounds.min_lng);
        max_lng = max_lng.max(sig.bounds.max_lng);
    }

    Some(Bounds {
        min_lat,
        max_lat,
        min_lng,
        max_lng,
    })
}

/// Create search bounds for spatial index query.
fn create_search_bounds(points: &[GpsPoint], tolerance: f64) -> AABB<[f64; 2]> {
    let (min_lat, max_lat, min_lng, max_lng) = crate::geo_utils::compute_bounds_tuple(points);
    AABB::from_corners(
        [min_lng - tolerance, min_lat - tolerance],
        [max_lng + tolerance, max_lat + tolerance],
    )
}

/// Check if two distances are within acceptable ratio (50%).
pub fn distance_ratio_ok(d1: f64, d2: f64) -> bool {
    if d1 <= 0.0 || d2 <= 0.0 {
        return false;
    }
    let ratio = if d1 > d2 { d2 / d1 } else { d1 / d2 };
    ratio >= 0.5
}

/// Check if a point is near any point on a route.
/// Used for loop matching - checks if a point lies on/near another loop.
fn is_point_near_route(point: &GpsPoint, route: &[GpsPoint], threshold: f64) -> bool {
    route
        .iter()
        .any(|route_point| haversine_distance(point, route_point) < threshold)
}

/// Split threshold for divergent route detection.
/// Activities with match % below this will be candidates for splitting.
const SPLIT_THRESHOLD: f64 = 80.0;

/// Minimum activities required in a sub-group to warrant splitting.
const MIN_SPLIT_GROUP_SIZE: usize = 2;

/// Split a route group if a subset of activities diverges significantly.
///
/// If some activities have low match % to the representative, they may actually
/// be a different route variant. This function detects and splits such cases.
///
/// # Algorithm
/// 1. Partition activities into "high match" (>= threshold) and "low match" (< threshold)
/// 2. If low_match group has >= 2 activities, create a new group from them
/// 3. Find new representative for the split group (activity with highest avg match to others)
/// 4. Return the original group (possibly reduced) and any new split groups
fn split_divergent_routes(
    group: &RouteGroup,
    matches: &[ActivityMatchInfo],
    sig_map: &HashMap<&str, &RouteSignature>,
    config: &MatchConfig,
) -> Vec<RouteGroup> {
    // Partition activities by match percentage
    let high_match: Vec<&str> = matches
        .iter()
        .filter(|m| m.match_percentage >= SPLIT_THRESHOLD)
        .map(|m| m.activity_id.as_str())
        .collect();

    let low_match: Vec<&str> = matches
        .iter()
        .filter(|m| m.match_percentage < SPLIT_THRESHOLD)
        .map(|m| m.activity_id.as_str())
        .collect();

    // If not enough low-match activities to form a group, keep as-is
    if low_match.len() < MIN_SPLIT_GROUP_SIZE {
        return vec![group.clone()];
    }

    // If almost all activities are low-match, the representative may be wrong
    // In this case, don't split - just keep the group
    if high_match.len() < MIN_SPLIT_GROUP_SIZE {
        return vec![group.clone()];
    }

    // Create original group (high match activities only)
    let original_group = RouteGroup {
        group_id: group.group_id.clone(),
        representative_id: group.representative_id.clone(),
        activity_ids: high_match.iter().map(|s| s.to_string()).collect(),
        sport_type: group.sport_type.clone(),
        bounds: compute_group_bounds(
            &high_match.iter().map(|s| s.to_string()).collect::<Vec<_>>(),
            sig_map,
        ),
        custom_name: group.custom_name.clone(),
        best_time: None,
        avg_time: None,
        best_pace: None,
        best_activity_id: None,
    };

    // Find representative for split group (activity with highest avg match to others in split)
    let split_representative = find_best_representative(&low_match, sig_map, config);

    // Create split group with new ID
    let split_group_id = format!("{}_split", group.group_id);
    let split_group = RouteGroup {
        group_id: split_group_id,
        representative_id: split_representative,
        activity_ids: low_match.iter().map(|s| s.to_string()).collect(),
        sport_type: group.sport_type.clone(),
        bounds: compute_group_bounds(
            &low_match.iter().map(|s| s.to_string()).collect::<Vec<_>>(),
            sig_map,
        ),
        custom_name: None,
        best_time: None,
        avg_time: None,
        best_pace: None,
        best_activity_id: None,
    };

    vec![original_group, split_group]
}

/// Find the best representative for a group of activities.
/// Returns the activity with highest average checkpoint match to all others.
fn find_best_representative(
    activity_ids: &[&str],
    sig_map: &HashMap<&str, &RouteSignature>,
    config: &MatchConfig,
) -> String {
    if activity_ids.is_empty() {
        return String::new();
    }
    if activity_ids.len() == 1 {
        return activity_ids[0].to_string();
    }

    let mut best_id = activity_ids[0];
    let mut best_avg_match = 0.0;

    for &candidate_id in activity_ids {
        let candidate_sig = match sig_map.get(candidate_id) {
            Some(sig) => sig,
            None => continue,
        };

        // Calculate average match to all other activities
        let mut total_match = 0.0;
        let mut count = 0;

        for &other_id in activity_ids {
            if other_id == candidate_id {
                continue;
            }
            if let Some(other_sig) = sig_map.get(other_id) {
                let match_pct =
                    calculate_checkpoint_match(&candidate_sig.points, &other_sig.points, config);
                total_match += match_pct;
                count += 1;
            }
        }

        let avg_match = if count > 0 {
            total_match / count as f64
        } else {
            0.0
        };

        if avg_match > best_avg_match {
            best_avg_match = avg_match;
            best_id = candidate_id;
        }
    }

    best_id.to_string()
}
