//! Route grouping algorithms.
//!
//! This module provides functionality to group similar routes together
//! using spatial indexing and Union-Find for efficient grouping.

use std::collections::HashMap;

use crate::geo_utils::haversine_distance;
use crate::grouping_filter::{
    RouteEndpointCells, cell_size_for_endpoint_threshold, endpoint_grid_filtered_pairs,
};
use crate::matching::{
    PreparedRoute, calculate_checkpoint_match, compare_prepared_routes, compare_routes,
    prepare_route,
};
use crate::union_find::UnionFind;
use crate::{
    ActivityMatchInfo, Bounds, GpsPoint, GroupingResult, MatchConfig, MatchResult, RouteGroup,
    RouteSignature,
};

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

    // Middle-point tolerance is 1.5× endpoint to accommodate slight day-to-day
    // mid-route variation (lane changes, shortcuts around obstacles) that the
    // 43a39da change made too strict at 1× endpoint.
    check_middle_points_match(
        &sig1.points,
        &points2_for_middle,
        config.endpoint_threshold * 1.5,
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

/// Group similar routes together (sequential).
///
/// Uses the endpoint grid pre-filter from [`crate::grouping_filter`] to
/// drop pairs whose start/end points cannot satisfy
/// `should_group_routes`, then runs the full matching pipeline on the
/// pruned pair set. For large datasets prefer
/// [`group_signatures_parallel`].
pub fn group_signatures(signatures: &[RouteSignature], config: &MatchConfig) -> Vec<RouteGroup> {
    group_signatures_with_progress(signatures, config, &mut |_, _, _| {})
}

/// Phase name used by `group_signatures_with_progress`.
pub const GROUPING_PHASE_COMPARING: &str = "comparing_pairs";

/// Group similar routes together while reporting progress.
///
/// The callback receives `(phase_name, current, total)`. Updates are
/// batched (~100 emissions per phase) so the cost of crossing FFI
/// boundaries stays negligible compared to the pair work itself.
pub fn group_signatures_with_progress(
    signatures: &[RouteSignature],
    config: &MatchConfig,
    on_progress: &mut dyn FnMut(&str, u32, u32),
) -> Vec<RouteGroup> {
    if signatures.is_empty() {
        return vec![];
    }

    let cell_size = cell_size_for_endpoint_threshold(config.endpoint_threshold);
    let endpoints: Vec<RouteEndpointCells> = signatures
        .iter()
        .map(|s| RouteEndpointCells::new(&s.start_point, &s.end_point, cell_size))
        .collect();
    let candidate_pairs = endpoint_grid_filtered_pairs(&endpoints);

    let sig_map: HashMap<&str, &RouteSignature> = signatures
        .iter()
        .map(|s| (s.activity_id.as_str(), s))
        .collect();

    // Precompute resample + R-tree per signature once. Without this the
    // hot loop rebuilds the same R-trees on every pair comparison — at
    // 91k pairs that's 182k R-tree builds and Vec allocations, which is
    // catastrophic in WASM.
    let prepared: Vec<PreparedRoute> = signatures
        .iter()
        .map(|s| prepare_route(s, config))
        .collect();

    let mut uf = UnionFind::with_capacity(signatures.len());
    for sig in signatures {
        uf.make_set(sig.activity_id.clone());
    }

    let total_pairs = candidate_pairs.len() as u32;
    // Emit ~100 progress ticks across the full pair loop. Threshold is
    // pair count / 100, clamped to [1, 1000] so very small or very large
    // datasets still get sensible update granularity.
    let tick_interval = total_pairs.div_ceil(100).clamp(1, 1000);
    on_progress(GROUPING_PHASE_COMPARING, 0, total_pairs);

    for (idx, (i, j)) in candidate_pairs.into_iter().enumerate() {
        let sig1 = &signatures[i];
        let sig2 = &signatures[j];
        if !distance_ratio_ok(sig1.total_distance, sig2.total_distance) {
            // Still tick to keep the bar moving even on skipped pairs.
            if (idx as u32 + 1).is_multiple_of(tick_interval) {
                on_progress(GROUPING_PHASE_COMPARING, idx as u32 + 1, total_pairs);
            }
            continue;
        }
        if !endpoints_could_group(sig1, sig2, config) {
            if (idx as u32 + 1).is_multiple_of(tick_interval) {
                on_progress(GROUPING_PHASE_COMPARING, idx as u32 + 1, total_pairs);
            }
            continue;
        }
        if let Some(match_result) =
            compare_prepared_routes(sig1, &prepared[i], sig2, &prepared[j], config)
            && should_group_routes(sig1, sig2, &match_result, config)
        {
            uf.union(&sig1.activity_id, &sig2.activity_id);
        }
        if (idx as u32 + 1).is_multiple_of(tick_interval) {
            on_progress(GROUPING_PHASE_COMPARING, idx as u32 + 1, total_pairs);
        }
    }
    on_progress(GROUPING_PHASE_COMPARING, total_pairs, total_pairs);

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

    let initial_groups = group_signatures(signatures, config);

    let sig_map: HashMap<&str, &RouteSignature> = signatures
        .iter()
        .map(|s| (s.activity_id.as_str(), s))
        .collect();

    compute_matches_and_split(&initial_groups, &sig_map, config)
}

/// Group signatures using parallel processing.
///
/// Uses [`crate::grouping_filter`]'s endpoint grid to prune the
/// candidate pair set down to those whose start/end points could fall
/// within `config.endpoint_threshold` of each other. Pairs that survive
/// the filter still go through the full
/// `distance_ratio_ok` → `compare_routes` → `should_group_routes`
/// pipeline, so the grouping output is provably identical to the
/// (legacy) all-pairs-after-R-tree implementation. The A/B test in
/// `crate::grouping_filter::tests::filter_produces_identical_groups_as_naive`
/// makes this guarantee explicit.
#[cfg(feature = "parallel")]
pub fn group_signatures_parallel(
    signatures: &[RouteSignature],
    config: &MatchConfig,
) -> Vec<RouteGroup> {
    use rayon::prelude::*;

    if signatures.is_empty() {
        return vec![];
    }

    // Pre-filter pair candidates via endpoint grid.
    let cell_size = cell_size_for_endpoint_threshold(config.endpoint_threshold);
    let endpoints: Vec<RouteEndpointCells> = signatures
        .iter()
        .map(|s| RouteEndpointCells::new(&s.start_point, &s.end_point, cell_size))
        .collect();
    let candidate_pairs = endpoint_grid_filtered_pairs(&endpoints);

    // Precompute resample + R-tree per signature (parallel). Avoids
    // rebuilding R-trees inside every pair comparison.
    let prepared: Vec<PreparedRoute> = signatures
        .par_iter()
        .map(|s| prepare_route(s, config))
        .collect();

    // Find matches in parallel over the pruned pair set.
    let matches: Vec<(String, String)> = candidate_pairs
        .par_iter()
        .filter_map(|&(i, j)| {
            let sig1 = &signatures[i];
            let sig2 = &signatures[j];

            if !distance_ratio_ok(sig1.total_distance, sig2.total_distance) {
                return None;
            }
            if !endpoints_could_group(sig1, sig2, config) {
                return None;
            }

            let match_result =
                compare_prepared_routes(sig1, &prepared[i], sig2, &prepared[j], config)?;
            if !should_group_routes(sig1, sig2, &match_result, config) {
                return None;
            }

            // Order pair lexicographically — Union-Find doesn't care
            // but consistent ordering keeps the matches list canonical.
            if sig1.activity_id < sig2.activity_id {
                Some((sig1.activity_id.clone(), sig2.activity_id.clone()))
            } else {
                Some((sig2.activity_id.clone(), sig1.activity_id.clone()))
            }
        })
        .collect();

    let sig_map: HashMap<&str, &RouteSignature> = signatures
        .iter()
        .map(|s| (s.activity_id.as_str(), s))
        .collect();

    // Union-Find (sequential - fast enough)
    let mut uf = UnionFind::with_capacity(signatures.len());
    for sig in signatures {
        uf.make_set(sig.activity_id.clone());
    }
    for (id1, id2) in matches {
        uf.union(&id1, &id2);
    }

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
    if signatures.is_empty() {
        return GroupingResult {
            groups: vec![],
            activity_matches: HashMap::new(),
        };
    }

    let initial_groups = group_signatures_parallel(signatures, config);

    let sig_map: HashMap<&str, &RouteSignature> = signatures
        .iter()
        .map(|s| (s.activity_id.as_str(), s))
        .collect();

    compute_matches_and_split(&initial_groups, &sig_map, config)
}

/// Incremental grouping: efficiently add new signatures to existing groups.
///
/// Layers the endpoint grid pre-filter on top of the "new x existing
/// plus new x new" pair set, so even at thousands of existing routes
/// the per-new-activity work stays bounded by the spatial density of
/// routes in the same area.
///
/// Existing-vs-existing pairs are never re-evaluated (they're already
/// in their final group). Only pairs where at least one side is new
/// get to AMD.
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

    // Combine all signatures (existing first, new second). The endpoint
    // grid uses the combined index space; we'll filter "new ∩ pair" below.
    let all_signatures: Vec<&RouteSignature> = existing_signatures
        .iter()
        .chain(new_signatures.iter())
        .collect();
    let existing_n = existing_signatures.len();

    let cell_size = cell_size_for_endpoint_threshold(config.endpoint_threshold);
    let endpoints: Vec<RouteEndpointCells> = all_signatures
        .iter()
        .map(|s| RouteEndpointCells::new(&s.start_point, &s.end_point, cell_size))
        .collect();
    let candidate_pairs = endpoint_grid_filtered_pairs(&endpoints);

    let sig_map: HashMap<&str, &RouteSignature> = all_signatures
        .iter()
        .map(|s| (s.activity_id.as_str(), *s))
        .collect();

    let new_ids: HashSet<&str> = new_signatures
        .iter()
        .map(|s| s.activity_id.as_str())
        .collect();

    // Initialize Union-Find with existing group structure
    let mut uf = UnionFind::with_capacity(all_signatures.len());
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
    for sig in new_signatures {
        uf.make_set(sig.activity_id.clone());
    }

    // Filter candidate pairs to those involving at least one new signature.
    // Indices < existing_n are existing; indices >= existing_n are new.
    let new_or_mixed_pairs: Vec<(usize, usize)> = candidate_pairs
        .into_iter()
        .filter(|&(i, j)| i >= existing_n || j >= existing_n)
        .collect();

    let matches: Vec<(String, String)> = new_or_mixed_pairs
        .par_iter()
        .filter_map(|&(i, j)| {
            let sig1 = all_signatures[i];
            let sig2 = all_signatures[j];

            // Skip if BOTH are existing (already grouped) — already
            // filtered above, kept as a defence in depth.
            let i_is_new = new_ids.contains(sig1.activity_id.as_str());
            let j_is_new = new_ids.contains(sig2.activity_id.as_str());
            if !i_is_new && !j_is_new {
                return None;
            }

            if !distance_ratio_ok(sig1.total_distance, sig2.total_distance) {
                return None;
            }
            if !endpoints_could_group(sig1, sig2, config) {
                return None;
            }

            let match_result = compare_routes(sig1, sig2, config)?;
            if !should_group_routes(sig1, sig2, &match_result, config) {
                return None;
            }

            Some((sig1.activity_id.clone(), sig2.activity_id.clone()))
        })
        .collect();

    for (id1, id2) in matches {
        uf.union(&id1, &id2);
    }

    let existing_reps: HashMap<String, String> = existing_groups
        .iter()
        .map(|g| (g.group_id.clone(), g.representative_id.clone()))
        .collect();

    let groups_map = uf.groups();
    build_route_groups_with_existing_reps(groups_map, &sig_map, &existing_reps)
}

/// Calculate match info for each activity in each group and split divergent groups.
///
/// Shared logic between `group_signatures_with_matches` and
/// `group_signatures_parallel_with_matches`.
fn compute_matches_and_split(
    initial_groups: &[RouteGroup],
    sig_map: &HashMap<&str, &RouteSignature>,
    config: &MatchConfig,
) -> GroupingResult {
    let mut final_groups: Vec<RouteGroup> = Vec::new();
    let mut activity_matches: HashMap<String, Vec<ActivityMatchInfo>> = HashMap::new();

    for group in initial_groups {
        let representative_sig = match sig_map.get(group.representative_id.as_str()) {
            Some(sig) => *sig,
            None => {
                final_groups.push(group.clone());
                continue;
            }
        };

        // Calculate match info for each activity using AMD-based matching
        let matches: Vec<ActivityMatchInfo> = group
            .activity_ids
            .iter()
            .filter_map(|activity_id| {
                let activity_sig = sig_map.get(activity_id.as_str())?;
                let result = compare_routes(activity_sig, representative_sig, config)?;

                log::debug!(
                    "tracematch: amd_match for {}: {:.1}% ({})",
                    activity_id,
                    result.match_percentage,
                    result.direction
                );

                Some(ActivityMatchInfo {
                    activity_id: activity_id.clone(),
                    match_percentage: result.match_percentage,
                    direction: result.direction,
                })
            })
            .collect();

        // Check if this group should be split
        let split_groups = split_divergent_routes(group, &matches, sig_map, config);

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
                        let result = compare_routes(activity_sig, split_rep_sig, config)?;

                        Some(ActivityMatchInfo {
                            activity_id: activity_id.clone(),
                            match_percentage: result.match_percentage,
                            direction: result.direction,
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

/// Build RouteGroup instances with full metadata from grouped activity IDs.
fn build_route_groups(
    groups_map: HashMap<String, Vec<String>>,
    sig_map: &HashMap<&str, &RouteSignature>,
) -> Vec<RouteGroup> {
    build_route_groups_with_existing_reps(groups_map, sig_map, &HashMap::new())
}

/// Build RouteGroup instances, preserving existing representatives when available.
/// This ensures incremental loading produces stable results.
fn build_route_groups_with_existing_reps(
    groups_map: HashMap<String, Vec<String>>,
    sig_map: &HashMap<&str, &RouteSignature>,
    existing_reps: &HashMap<String, String>,
) -> Vec<RouteGroup> {
    groups_map
        .into_iter()
        .map(|(group_id, activity_ids)| {
            // activity_ids is already sorted from union_find.groups()
            // Preserve existing representative if still in group, otherwise use first (sorted)
            let representative_id = existing_reps
                .get(&group_id)
                .filter(|rep| activity_ids.contains(rep))
                .cloned()
                .unwrap_or_else(|| activity_ids.first().cloned().unwrap_or_default());

            // Default sport type - caller should override with actual value
            let sport_type = "Ride".to_string();

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

/// Check if two distances are within acceptable ratio (50%).
pub fn distance_ratio_ok(d1: f64, d2: f64) -> bool {
    if d1 <= 0.0 || d2 <= 0.0 {
        return false;
    }
    let ratio = if d1 > d2 { d2 / d1 } else { d1 / d2 };
    ratio >= 0.5
}

/// Cheap O(1) endpoint pre-check used by the grouping path before the
/// expensive `compare_routes` resampling+AMD pipeline.
///
/// Returns `true` if the pair's endpoints could plausibly satisfy the
/// strict `should_group_routes` gate. This duplicates a piece of the
/// gate's logic (lines 89–101) at the cheapest possible cost so we
/// can short-circuit pairs that survived the grid pre-filter (which
/// allows a 4× safety margin) but cannot pass the actual 1× threshold.
///
/// On the sionrunning corpus this cuts `compare_routes` calls by
/// another ~60% on top of the grid filter, since the grid is sized
/// generously to never drop a valid match. The check is conservative:
/// loops (start ≈ end) always pass through to the full gate, since
/// loop matching uses `is_point_near_route` instead of direct endpoint
/// comparison and we'd need the AMD result to decide.
pub fn endpoints_could_group(
    sig1: &RouteSignature,
    sig2: &RouteSignature,
    config: &MatchConfig,
) -> bool {
    let start1 = &sig1.start_point;
    let end1 = &sig1.end_point;
    let start2 = &sig2.start_point;
    let end2 = &sig2.end_point;

    let sig1_is_loop = haversine_distance(start1, end1) < config.endpoint_threshold;
    let sig2_is_loop = haversine_distance(start2, end2) < config.endpoint_threshold;

    // Loops use a different rule (is_point_near_route) in the strict
    // gate — let them through unconditionally here.
    if sig1_is_loop || sig2_is_loop {
        return true;
    }

    let same_direction_ok = haversine_distance(start1, start2) < config.endpoint_threshold
        && haversine_distance(end1, end2) < config.endpoint_threshold;
    let reverse_direction_ok = haversine_distance(start1, end2) < config.endpoint_threshold
        && haversine_distance(end1, start2) < config.endpoint_threshold;

    same_direction_ok || reverse_direction_ok
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
