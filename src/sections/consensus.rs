#![allow(clippy::type_complexity)]
//! How closely a set of passes agrees with a reference line.
//!
//! For each reference point, every trace's nearest point within the
//! proximity threshold is an observation. The measurements are the mean
//! distance of those observations from the reference (`average_spread`),
//! how many traces observed each point (`point_density`), and a confidence
//! built from both.
//!
//! The polyline these functions also return is a centroid weighted by
//! 1 / (distance + 1e-6), so a trace lying on the reference outweighs every
//! other trace by about a million to one and the centroid is the reference
//! to within float drift. A section's line is never taken from it: it is
//! the medoid's own slice, and [`super::process_cluster`] measures the other
//! passes against that slice, leaving the medoid's own pass out.
//!
//! [`compute_consensus_polyline`] walks every trace's R-tree for every
//! reference point, so the cost is O(T × P log P) where T = traces and
//! P = reference points.

use super::rtree::{PointIndex, build_rtree_at};
use crate::GpsPoint;
#[cfg(feature = "parallel")]
use rayon::prelude::*;

/// Result of consensus computation including confidence metrics
pub struct ConsensusResult {
    /// The weighted centroid of the observations, which collapses onto any
    /// trace that lies on the reference; never a section's line
    pub polyline: Vec<GpsPoint>,
    /// Confidence score (0.0-1.0)
    pub confidence: f64,
    /// Number of tracks that contributed
    pub observation_count: u32,
    /// Mean distance of the observations from the reference (metres)
    pub average_spread: f64,
    /// Per-point observation count (how many tracks contributed to each point)
    pub point_density: Vec<u32>,
}

/// Measure how closely `all_traces` agree with `reference`: spread, per-point
/// density and confidence. The returned polyline is the weighted centroid the
/// module doc describes, and no section line is taken from it.
pub fn compute_consensus_polyline(
    reference: &[GpsPoint],
    all_traces: &[Vec<GpsPoint>],
    proximity_threshold: f64,
) -> ConsensusResult {
    if reference.is_empty() || all_traces.is_empty() {
        return ConsensusResult {
            polyline: reference.to_vec(),
            confidence: 0.0,
            observation_count: 0,
            average_spread: 0.0,
            point_density: vec![0; reference.len()],
        };
    }

    // Build R-trees for all traces for efficient spatial queries
    let reference_latitude = reference[0].latitude;
    #[cfg(feature = "parallel")]
    let trace_trees: Vec<PointIndex> = all_traces
        .par_iter()
        .map(|trace| build_rtree_at(trace, reference_latitude))
        .collect();

    #[cfg(not(feature = "parallel"))]
    let trace_trees: Vec<PointIndex> = all_traces
        .iter()
        .map(|trace| build_rtree_at(trace, reference_latitude))
        .collect();

    let epsilon = 0.000001; // Small constant to avoid division by zero

    // Per-point result for map-reduce consensus computation
    struct PointResult {
        point: GpsPoint,
        density: u32,
        spread_sum: f64,
        observations: u32,
    }

    // Compute consensus for each reference point (parallel if feature enabled)
    let compute_point = |ref_point: &GpsPoint| -> PointResult {
        let ref_coords = [ref_point.latitude, ref_point.longitude];

        let mut weighted_lat = 0.0;
        let mut weighted_lng = 0.0;
        let mut weighted_elev = 0.0;
        let mut total_weight = 0.0;
        let mut elev_weight = 0.0;
        let mut distance_sum = 0.0;
        let mut observation_count = 0u32;

        for (trace_idx, tree) in trace_trees.iter().enumerate() {
            if let Some((nearest_index, dist_meters)) = tree.nearest(&ref_coords)
                && dist_meters <= proximity_threshold
            {
                let trace = &all_traces[trace_idx];
                let trace_point = &trace[nearest_index];
                let weight = 1.0 / (dist_meters + epsilon);

                weighted_lat += trace_point.latitude * weight;
                weighted_lng += trace_point.longitude * weight;
                total_weight += weight;
                distance_sum += dist_meters;
                observation_count += 1;

                if let Some(elev) = trace_point.elevation {
                    weighted_elev += elev * weight;
                    elev_weight += weight;
                }
            }
        }

        if total_weight > 0.0 {
            let consensus_lat = weighted_lat / total_weight;
            let consensus_lng = weighted_lng / total_weight;
            let consensus_elev = if elev_weight > 0.0 {
                Some(weighted_elev / elev_weight)
            } else {
                ref_point.elevation
            };

            let spread_sum = if observation_count > 0 {
                distance_sum / observation_count as f64
            } else {
                0.0
            };

            PointResult {
                point: GpsPoint {
                    latitude: consensus_lat,
                    longitude: consensus_lng,
                    elevation: consensus_elev,
                },
                density: observation_count,
                spread_sum,
                observations: observation_count,
            }
        } else {
            PointResult {
                point: *ref_point,
                density: 0,
                spread_sum: 0.0,
                observations: 0,
            }
        }
    };

    #[cfg(feature = "parallel")]
    let point_results: Vec<PointResult> = reference.par_iter().map(compute_point).collect();

    #[cfg(not(feature = "parallel"))]
    let point_results: Vec<PointResult> = reference.iter().map(compute_point).collect();

    // Reduce results
    let mut consensus_points = Vec::with_capacity(reference.len());
    let mut point_density = Vec::with_capacity(reference.len());
    let mut total_spread = 0.0;
    let mut total_point_observations = 0u32;

    for result in point_results {
        consensus_points.push(result.point);
        point_density.push(result.density);
        total_spread += result.spread_sum;
        total_point_observations += result.observations;
    }

    // Compute overall metrics
    let observation_count = trace_trees.len() as u32;
    let average_spread = if total_point_observations > 0 {
        total_spread / (reference.len() as f64)
    } else {
        proximity_threshold // Default to max threshold if no observations
    };

    // Confidence based on observation count and spread
    // More observations + tighter spread = higher confidence
    let obs_factor = (observation_count as f64).min(10.0) / 10.0; // Saturates at 10 observations
    let spread_factor = 1.0 - (average_spread / proximity_threshold).min(1.0); // Lower spread = higher factor
    let confidence = (obs_factor * 0.5 + spread_factor * 0.5).clamp(0.0, 1.0);

    ConsensusResult {
        polyline: consensus_points,
        confidence,
        observation_count,
        average_spread,
        point_density,
    }
}

/// One traversal: the activity and its zero-based pass over the section.
pub type TraceKey = (String, u32);
