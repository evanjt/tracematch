//! Consensus polyline computation.
//!
//! Computes a refined polyline from multiple overlapping tracks using weighted averaging
//! where weight = 1 / (distance_to_reference + epsilon).
//!
//! Algorithm:
//! 1. Normalize each track to distance parameterization
//! 2. For each position along the reference, find nearby points from all tracks
//! 3. Compute weighted centroid of nearby points
//! 4. Track observation density for confidence scoring

use super::rtree::{IndexedPoint, build_rtree};
use crate::GpsPoint;
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use rstar::{PointDistance, RTree};

/// Result of consensus computation including confidence metrics
pub struct ConsensusResult {
    /// The refined consensus polyline
    pub polyline: Vec<GpsPoint>,
    /// Confidence score (0.0-1.0)
    pub confidence: f64,
    /// Number of tracks that contributed
    pub observation_count: u32,
    /// Average spread of observations from consensus (meters)
    pub average_spread: f64,
    /// Per-point observation count (how many tracks contributed to each point)
    pub point_density: Vec<u32>,
}

/// Compute a consensus polyline from multiple overlapping tracks.
/// Uses weighted averaging where weight = 1 / (distance_to_reference + epsilon).
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
    #[cfg(feature = "parallel")]
    let trace_trees: Vec<RTree<IndexedPoint>> =
        all_traces.par_iter().map(|trace| build_rtree(trace)).collect();

    #[cfg(not(feature = "parallel"))]
    let trace_trees: Vec<RTree<IndexedPoint>> =
        all_traces.iter().map(|trace| build_rtree(trace)).collect();

    let threshold_deg = proximity_threshold / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;
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
            if let Some(nearest) = tree.nearest_neighbor(&ref_coords) {
                let dist_sq = nearest.distance_2(&ref_coords);

                if dist_sq <= threshold_deg_sq {
                    let trace = &all_traces[trace_idx];
                    let trace_point = &trace[nearest.idx];

                    let dist_deg = dist_sq.sqrt();
                    let dist_meters = dist_deg * 111_000.0;
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
