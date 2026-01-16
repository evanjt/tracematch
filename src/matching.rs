//! Route matching algorithms using Average Minimum Distance (AMD).
//!
//! This module provides the core route comparison functionality:
//! - AMD-based route comparison
//! - Route resampling for fair comparison
//! - Direction detection (same vs reverse)

use crate::geo_utils::haversine_distance;
use crate::{GpsPoint, MatchConfig, MatchResult, RouteSignature};

/// Compare two routes and return a match result using Average Minimum Distance (AMD).
///
/// AMD is robust to GPS noise and doesn't require point ordering.
/// For each point in route1, we find the minimum distance to any point in route2,
/// then average all those distances.
///
/// Returns `None` if the routes don't meet the minimum match threshold.
///
/// # Example
/// ```
/// use tracematch::{GpsPoint, RouteSignature, MatchConfig};
/// use tracematch::matching::compare_routes;
///
/// let points1 = vec![
///     GpsPoint::new(51.5074, -0.1278),
///     GpsPoint::new(51.5090, -0.1300),
/// ];
/// let points2 = points1.clone();
///
/// let sig1 = RouteSignature::from_points("a", &points1, &MatchConfig::default()).unwrap();
/// let sig2 = RouteSignature::from_points("b", &points2, &MatchConfig::default()).unwrap();
///
/// let result = compare_routes(&sig1, &sig2, &MatchConfig::default());
/// assert!(result.is_some());
/// ```
pub fn compare_routes(
    sig1: &RouteSignature,
    sig2: &RouteSignature,
    config: &MatchConfig,
) -> Option<MatchResult> {
    // Quick distance filter - routes must be within 50% of each other's length
    let distance_ratio = if sig1.total_distance > sig2.total_distance {
        sig2.total_distance / sig1.total_distance
    } else {
        sig1.total_distance / sig2.total_distance
    };

    if distance_ratio < 0.5 {
        return None;
    }

    // Resample both routes for fair comparison
    // Use distance-proportional resampling if enabled, otherwise fixed count
    let resampled1 = resample_for_comparison(&sig1.points, sig1.total_distance, config);
    let resampled2 = resample_for_comparison(&sig2.points, sig2.total_distance, config);

    // Calculate AMD in both directions (AMD is asymmetric)
    let amd_1_to_2 = average_min_distance(&resampled1, &resampled2);
    let amd_2_to_1 = average_min_distance(&resampled2, &resampled1);

    // Use average of both directions
    let avg_amd = (amd_1_to_2 + amd_2_to_1) / 2.0;

    // Convert AMD to percentage using thresholds
    let match_percentage =
        amd_to_percentage(avg_amd, config.perfect_threshold, config.zero_threshold);

    // Check if meets minimum threshold
    if match_percentage < config.min_match_percentage {
        return None;
    }

    // Determine direction using endpoint comparison (AMD is symmetric)
    let direction = determine_direction_by_endpoints(sig1, sig2, config.endpoint_threshold);

    // Direction type based on match quality
    let direction_str = if match_percentage >= 70.0 {
        direction
    } else {
        "partial".to_string()
    };

    Some(MatchResult {
        activity_id_1: sig1.activity_id.clone(),
        activity_id_2: sig2.activity_id.clone(),
        match_percentage,
        direction: direction_str,
        amd: avg_amd,
    })
}

/// Resample a route for comparison using the configured strategy.
///
/// If `resample_spacing_meters > 0`, uses distance-proportional resampling
/// for consistent granularity regardless of route length.
/// Otherwise, falls back to fixed `resample_count`.
///
/// # Arguments
/// * `points` - The route points to resample
/// * `total_distance` - Total route distance in meters
/// * `config` - Match configuration with resampling parameters
pub fn resample_for_comparison(
    points: &[GpsPoint],
    total_distance: f64,
    config: &MatchConfig,
) -> Vec<GpsPoint> {
    let target_count = if config.resample_spacing_meters > 0.0 {
        // Distance-proportional: aim for one point per spacing_meters
        let count_from_distance = (total_distance / config.resample_spacing_meters).ceil() as u32;

        // Clamp to min/max bounds
        count_from_distance.clamp(config.min_resample_points, config.max_resample_points) as usize
    } else {
        // Legacy: fixed count
        config.resample_count as usize
    };

    resample_route(points, target_count)
}

/// Calculate Average Minimum Distance from route1 to route2.
///
/// For each point in route1, find the minimum distance to any point in route2.
/// Return the average of these minimum distances.
pub fn average_min_distance(route1: &[GpsPoint], route2: &[GpsPoint]) -> f64 {
    if route1.is_empty() || route2.is_empty() {
        return f64::INFINITY;
    }

    let total_min_dist: f64 = route1
        .iter()
        .map(|p1| {
            route2
                .iter()
                .map(|p2| haversine_distance(p1, p2))
                .fold(f64::INFINITY, f64::min)
        })
        .sum();

    total_min_dist / route1.len() as f64
}

/// Convert AMD to a match percentage using thresholds.
///
/// - AMD <= perfect_threshold → 100% match
/// - AMD >= zero_threshold → 0% match
/// - Linear interpolation between
pub fn amd_to_percentage(amd: f64, perfect_threshold: f64, zero_threshold: f64) -> f64 {
    if amd <= perfect_threshold {
        return 100.0;
    }
    if amd >= zero_threshold {
        return 0.0;
    }

    // Linear interpolation
    100.0 * (1.0 - (amd - perfect_threshold) / (zero_threshold - perfect_threshold))
}

/// Resample a route to have exactly n points, evenly spaced by distance.
pub fn resample_route(points: &[GpsPoint], target_count: usize) -> Vec<GpsPoint> {
    if points.len() < 2 {
        return points.to_vec();
    }
    if points.len() == target_count {
        return points.to_vec();
    }

    // Calculate total distance
    let total_dist = calculate_route_distance(points);
    if total_dist == 0.0 {
        return points[..target_count.min(points.len())].to_vec();
    }

    let step_dist = total_dist / (target_count - 1) as f64;
    let mut resampled: Vec<GpsPoint> = vec![points[0]];

    let mut accumulated = 0.0;
    let mut next_threshold = step_dist;
    let mut prev_point = &points[0];

    for curr in points.iter().skip(1) {
        let seg_dist = haversine_distance(prev_point, curr);

        while accumulated + seg_dist >= next_threshold && resampled.len() < target_count - 1 {
            // Interpolate point at the threshold distance
            let ratio = (next_threshold - accumulated) / seg_dist;
            let new_lat = prev_point.latitude + ratio * (curr.latitude - prev_point.latitude);
            let new_lng = prev_point.longitude + ratio * (curr.longitude - prev_point.longitude);
            resampled.push(GpsPoint::new(new_lat, new_lng));
            next_threshold += step_dist;
        }

        accumulated += seg_dist;
        prev_point = curr;
    }

    // Always include the last point
    if resampled.len() < target_count {
        resampled.push(*points.last().unwrap());
    }

    resampled
}

/// Calculate the total distance of a route in meters.
pub fn calculate_route_distance(points: &[GpsPoint]) -> f64 {
    points
        .windows(2)
        .map(|w| haversine_distance(&w[0], &w[1]))
        .sum()
}

/// Determine direction using endpoint comparison.
///
/// Returns "same" if sig2 starts near sig1's start, "reverse" if near sig1's end.
pub fn determine_direction_by_endpoints(
    sig1: &RouteSignature,
    sig2: &RouteSignature,
    loop_threshold: f64,
) -> String {
    let start1 = &sig1.start_point;
    let end1 = &sig1.end_point;
    let start2 = &sig2.start_point;
    let end2 = &sig2.end_point;

    // Check if either route is a loop (start ≈ end)
    let sig1_is_loop = haversine_distance(start1, end1) < loop_threshold;
    let sig2_is_loop = haversine_distance(start2, end2) < loop_threshold;

    // If both are loops, direction is meaningless
    if sig1_is_loop && sig2_is_loop {
        return "same".to_string();
    }

    // Score for same direction: start2→start1 + end2→end1
    let same_score = haversine_distance(start2, start1) + haversine_distance(end2, end1);
    // Score for reverse direction: start2→end1 + end2→start1
    let reverse_score = haversine_distance(start2, end1) + haversine_distance(end2, start1);

    // Require a significant difference (100m) to call it 'reverse'
    let min_direction_diff = 100.0;

    if reverse_score < same_score - min_direction_diff {
        "reverse".to_string()
    } else {
        "same".to_string()
    }
}

/// Calculate fine-grained match percentage using checkpoint sampling.
///
/// This is more accurate than pure AMD for detecting localized divergences,
/// while remaining efficient O(k) for mobile devices.
///
/// The function samples 9 evenly spaced checkpoints along both routes BY DISTANCE
/// (not by array index) and compares the geographic distance at each checkpoint.
/// Middle positions (30-70%) are weighted more heavily since divergences typically
/// occur in the middle of routes that share start/end points.
///
/// # Returns
/// - 100% if all checkpoints are within `perfect_threshold`
/// - Lower percentages when checkpoints diverge significantly
///
/// # Arguments
/// * `route1` - First route points
/// * `route2` - Second route points (typically the representative/reference)
/// * `config` - Match configuration with thresholds
pub fn calculate_checkpoint_match(
    route1: &[GpsPoint],
    route2: &[GpsPoint],
    config: &MatchConfig,
) -> f64 {
    if route1.len() < 2 || route2.len() < 2 {
        return 0.0;
    }

    // Calculate cumulative distances for both routes
    let dist1 = cumulative_distances(route1);
    let dist2 = cumulative_distances(route2);

    let total_dist1 = *dist1.last().unwrap_or(&0.0);
    let total_dist2 = *dist2.last().unwrap_or(&0.0);

    if total_dist1 < 1.0 || total_dist2 < 1.0 {
        return 0.0;
    }

    // 9 evenly spaced checkpoints by distance (efficient: O(9) comparisons)
    let positions: [f64; 9] = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9];

    let mut total_score = 0.0;
    let mut weight_sum = 0.0;

    for pos in positions {
        // Find point at this percentage of DISTANCE along each route
        let target_dist1 = total_dist1 * pos;
        let target_dist2 = total_dist2 * pos;

        let p1 = point_at_distance(route1, &dist1, target_dist1);
        let p2 = point_at_distance(route2, &dist2, target_dist2);

        let dist = haversine_distance(&p1, &p2);

        // Weight middle positions more heavily (they're more likely to diverge)
        // This catches the "same start/end but different middle" pattern
        let weight = if (0.3..=0.7).contains(&pos) { 1.5 } else { 1.0 };

        // Convert distance to score (0-100 per checkpoint)
        let score = if dist <= config.perfect_threshold {
            100.0
        } else if dist >= config.zero_threshold {
            0.0
        } else {
            100.0
                * (1.0
                    - (dist - config.perfect_threshold)
                        / (config.zero_threshold - config.perfect_threshold))
        };

        total_score += score * weight;
        weight_sum += weight;
    }

    total_score / weight_sum
}

/// Calculate cumulative distances along a route.
/// Returns a vector where dist[i] is the distance from start to point i.
fn cumulative_distances(points: &[GpsPoint]) -> Vec<f64> {
    let mut distances = Vec::with_capacity(points.len());
    distances.push(0.0);

    for i in 1..points.len() {
        let prev_dist = distances[i - 1];
        let segment_dist = haversine_distance(&points[i - 1], &points[i]);
        distances.push(prev_dist + segment_dist);
    }

    distances
}

/// Find the point at a specific distance along the route.
/// Interpolates between points if the exact distance falls between two points.
fn point_at_distance(points: &[GpsPoint], cumulative: &[f64], target_dist: f64) -> GpsPoint {
    if points.is_empty() {
        return GpsPoint::new(0.0, 0.0);
    }
    if target_dist <= 0.0 {
        return points[0];
    }

    let total_dist = *cumulative.last().unwrap_or(&0.0);
    if target_dist >= total_dist {
        return *points.last().unwrap();
    }

    // Binary search for the segment containing target_dist
    let idx = match cumulative.binary_search_by(|d| d.partial_cmp(&target_dist).unwrap()) {
        Ok(i) => return points[i], // Exact match
        Err(i) => i.saturating_sub(1),
    };

    if idx >= points.len() - 1 {
        return *points.last().unwrap();
    }

    // Interpolate between points[idx] and points[idx+1]
    let seg_start_dist = cumulative[idx];
    let seg_end_dist = cumulative[idx + 1];
    let seg_length = seg_end_dist - seg_start_dist;

    if seg_length < 0.001 {
        return points[idx];
    }

    let ratio = (target_dist - seg_start_dist) / seg_length;
    let p1 = &points[idx];
    let p2 = &points[idx + 1];

    GpsPoint::new(
        p1.latitude + ratio * (p2.latitude - p1.latitude),
        p1.longitude + ratio * (p2.longitude - p1.longitude),
    )
}
