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
//!
//! Two paths:
//! - [`compute_consensus_polyline`] — the original full-recompute path,
//!   used at first detection and as a fallback. Walks every trace's R-tree
//!   for every reference point; cost is O(T × P log P) where T = traces
//!   and P = reference points.
//! - [`merge_traces_into_consensus`] — the incremental path. Maintains a
//!   per-reference-point [`ConsensusPointAccumulator`] of weighted sums
//!   and adds new traces' contributions in O(K × P log P) where K = new
//!   traces only. The reference polyline is held fixed across merges so
//!   the accumulator stays valid.

use super::rtree::{IndexedPoint, build_rtree};
use crate::GpsPoint;
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use rstar::{PointDistance, RTree};
use serde::{Deserialize, Serialize};

/// Per-reference-point running sums for incremental consensus computation.
///
/// Holds the numerators and denominators of the weighted-centroid formula so
/// new traces can be added without re-walking old traces. Polyline geometry
/// is recovered as `weighted_lat_sum / total_weight`.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ConsensusPointAccumulator {
    pub weighted_lat_sum: f64,
    pub weighted_lng_sum: f64,
    pub weighted_elev_sum: f64,
    pub total_weight: f64,
    pub elev_weight: f64,
    pub distance_sum: f64,
    pub observation_count: u32,
}

/// State kept across incremental consensus updates.
///
/// `reference` is the polyline used to anchor R-tree queries. We pin it
/// once at first detection so the per-point sums stay coherent; the
/// resulting consensus polyline is the centroid of trace points keyed
/// against this fixed reference. Without pinning, the reference would
/// drift on each merge and old running sums would no longer correspond to
/// the same reference indices.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsensusAccumulator {
    pub reference: Vec<GpsPoint>,
    pub per_point: Vec<ConsensusPointAccumulator>,
    /// Number of traces folded in so far.
    pub trace_count: u32,
    /// Activity IDs that have been folded in. Prevents double-counting
    /// the same activity if incremental detection sees it twice.
    pub absorbed_activity_ids: Vec<String>,
}

impl ConsensusAccumulator {
    pub fn new(reference: Vec<GpsPoint>) -> Self {
        let len = reference.len();
        Self {
            reference,
            per_point: vec![ConsensusPointAccumulator::default(); len],
            trace_count: 0,
            absorbed_activity_ids: Vec::new(),
        }
    }
}

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
    let trace_trees: Vec<RTree<IndexedPoint>> = all_traces
        .par_iter()
        .map(|trace| build_rtree(trace))
        .collect();

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

/// Build a fresh `ConsensusAccumulator` from a reference polyline and a set
/// of (activity_id, trace) pairs. Used at first detection or as a one-shot
/// backfill when an existing section's accumulator is missing.
///
/// Equivalent in result to `compute_consensus_polyline(reference, traces)`
/// followed by replaying `merge_traces_into_consensus` for each trace, but
/// done in a single pass so we avoid the duplicate R-tree builds.
pub fn build_accumulator_from_traces(
    reference: &[GpsPoint],
    traces: &[(String, Vec<GpsPoint>)],
    proximity_threshold: f64,
) -> ConsensusAccumulator {
    let mut accumulator = ConsensusAccumulator::new(reference.to_vec());

    if reference.is_empty() || traces.is_empty() {
        return accumulator;
    }

    let trace_views: Vec<(String, &[GpsPoint])> =
        traces.iter().map(|(id, t)| (id.clone(), t.as_slice())).collect();
    fold_traces_into_accumulator(&mut accumulator, &trace_views, proximity_threshold);
    accumulator
}

/// Pre-built R-tree cache for new traces, shared across multiple
/// `merge_traces_into_consensus_with_cache` calls within one detection
/// pass. Use [`build_trace_rtree_cache`] to populate. Reusing this cache
/// across multiple sections in the same incremental run amortises the
/// R-tree construction cost — important when many sections are touched
/// by the same handful of new activities.
pub type TraceRTreeCache = std::collections::HashMap<String, std::sync::Arc<RTree<IndexedPoint>>>;

/// Build R-trees for every (id, points) pair in `traces`. Skips empty
/// tracks. Suitable for sharing via [`merge_traces_into_consensus_with_cache`].
pub fn build_trace_rtree_cache(traces: &[(String, Vec<GpsPoint>)]) -> TraceRTreeCache {
    #[cfg(feature = "parallel")]
    let pairs: Vec<(String, std::sync::Arc<RTree<IndexedPoint>>)> = traces
        .par_iter()
        .filter_map(|(id, pts)| {
            if pts.is_empty() {
                None
            } else {
                Some((id.clone(), std::sync::Arc::new(build_rtree(pts))))
            }
        })
        .collect();

    #[cfg(not(feature = "parallel"))]
    let pairs: Vec<(String, std::sync::Arc<RTree<IndexedPoint>>)> = traces
        .iter()
        .filter_map(|(id, pts)| {
            if pts.is_empty() {
                None
            } else {
                Some((id.clone(), std::sync::Arc::new(build_rtree(pts))))
            }
        })
        .collect();

    pairs.into_iter().collect()
}

/// Incremental consensus update with a shared R-tree cache. The cache
/// contains R-trees for activity tracks, keyed by activity_id. When the
/// same activity contributes to multiple sections in one detection pass,
/// each section's merge reuses the same R-tree instead of rebuilding.
pub fn merge_traces_into_consensus_with_cache(
    accumulator: &mut ConsensusAccumulator,
    new_traces: &[(String, Vec<GpsPoint>)],
    cache: &TraceRTreeCache,
    proximity_threshold: f64,
) -> ConsensusResult {
    if accumulator.reference.is_empty() {
        return ConsensusResult {
            polyline: vec![],
            confidence: 0.0,
            observation_count: 0,
            average_spread: 0.0,
            point_density: vec![],
        };
    }

    let already: std::collections::HashSet<&str> = accumulator
        .absorbed_activity_ids
        .iter()
        .map(|s| s.as_str())
        .collect();
    let to_fold: Vec<(String, &[GpsPoint], Option<std::sync::Arc<RTree<IndexedPoint>>>)> =
        new_traces
            .iter()
            .filter(|(id, _)| !already.contains(id.as_str()))
            .map(|(id, t)| (id.clone(), t.as_slice(), cache.get(id).cloned()))
            .collect();

    if !to_fold.is_empty() {
        fold_traces_with_optional_cache(accumulator, &to_fold, proximity_threshold);
    }

    accumulator_to_result(accumulator, proximity_threshold)
}

/// Incremental consensus update: fold `new_traces` into an existing
/// accumulator. Returns the refreshed [`ConsensusResult`] (polyline +
/// confidence/spread metrics derived from the running sums) and mutates
/// `accumulator` in place.
///
/// `new_traces` whose `activity_id` is already in
/// `accumulator.absorbed_activity_ids` are skipped — this guards against
/// double-counting in batched incremental runs.
pub fn merge_traces_into_consensus(
    accumulator: &mut ConsensusAccumulator,
    new_traces: &[(String, Vec<GpsPoint>)],
    proximity_threshold: f64,
) -> ConsensusResult {
    // Thin wrapper over the cached path with an empty cache. R-trees get
    // built fresh inside fold_traces_with_optional_cache, exactly as the
    // old standalone implementation did. Keeping the no-cache entry point
    // is convenient for callers (and tests) that don't have a shared
    // cache to pass in.
    merge_traces_into_consensus_with_cache(
        accumulator,
        new_traces,
        &TraceRTreeCache::new(),
        proximity_threshold,
    )
}

/// Inner: variant of [`fold_traces_into_accumulator`] that accepts an
/// optional pre-built R-tree per trace. Tracks with `Some(tree)` skip the
/// build step. Used by [`merge_traces_into_consensus_with_cache`].
fn fold_traces_with_optional_cache(
    accumulator: &mut ConsensusAccumulator,
    traces: &[(String, &[GpsPoint], Option<std::sync::Arc<RTree<IndexedPoint>>>)],
    proximity_threshold: f64,
) {
    let owned: Vec<(String, &[GpsPoint], std::sync::Arc<RTree<IndexedPoint>>)> = {
        #[cfg(feature = "parallel")]
        let built: Vec<(String, &[GpsPoint], std::sync::Arc<RTree<IndexedPoint>>)> = traces
            .par_iter()
            .filter_map(|(id, pts, cached)| {
                if pts.is_empty() {
                    return None;
                }
                let tree = match cached {
                    Some(t) => t.clone(),
                    None => std::sync::Arc::new(build_rtree(pts)),
                };
                Some((id.clone(), *pts, tree))
            })
            .collect();
        #[cfg(not(feature = "parallel"))]
        let built: Vec<(String, &[GpsPoint], std::sync::Arc<RTree<IndexedPoint>>)> = traces
            .iter()
            .filter_map(|(id, pts, cached)| {
                if pts.is_empty() {
                    return None;
                }
                let tree = match cached {
                    Some(t) => t.clone(),
                    None => std::sync::Arc::new(build_rtree(pts)),
                };
                Some((id.clone(), *pts, tree))
            })
            .collect();
        built
    };

    fold_resolved_into_accumulator(accumulator, &owned, proximity_threshold);

    // Mark all input traces as absorbed (including empty ones we skipped).
    for (id, _, _) in traces {
        if !accumulator.absorbed_activity_ids.contains(id) {
            accumulator.absorbed_activity_ids.push(id.clone());
        }
    }
}

/// Inner: walk each trace's R-tree against `accumulator.reference`,
/// accumulating weighted sums into `accumulator.per_point`. Updates
/// `trace_count` and `absorbed_activity_ids`.
fn fold_traces_into_accumulator(
    accumulator: &mut ConsensusAccumulator,
    traces: &[(String, &[GpsPoint])],
    proximity_threshold: f64,
) {
    // Build trees and delegate to the resolved-trees folder.
    let resolved: Vec<(String, &[GpsPoint], std::sync::Arc<RTree<IndexedPoint>>)> = {
        #[cfg(feature = "parallel")]
        let built: Vec<(String, &[GpsPoint], std::sync::Arc<RTree<IndexedPoint>>)> = traces
            .par_iter()
            .filter_map(|(id, pts)| {
                if pts.is_empty() {
                    None
                } else {
                    Some((id.clone(), *pts, std::sync::Arc::new(build_rtree(pts))))
                }
            })
            .collect();
        #[cfg(not(feature = "parallel"))]
        let built: Vec<(String, &[GpsPoint], std::sync::Arc<RTree<IndexedPoint>>)> = traces
            .iter()
            .filter_map(|(id, pts)| {
                if pts.is_empty() {
                    None
                } else {
                    Some((id.clone(), *pts, std::sync::Arc::new(build_rtree(pts))))
                }
            })
            .collect();
        built
    };

    fold_resolved_into_accumulator(accumulator, &resolved, proximity_threshold);

    // Mark all input traces as absorbed even if the build was skipped
    // (empty tracks still count).
    for (id, _) in traces {
        if !accumulator.absorbed_activity_ids.contains(id) {
            accumulator.absorbed_activity_ids.push(id.clone());
        }
    }
}

/// Shared inner loop: given pre-resolved (id, points, R-tree) tuples,
/// fold contributions into the accumulator. Used by both the build-now
/// path ([`fold_traces_into_accumulator`]) and the cached path
/// ([`fold_traces_with_optional_cache`]).
fn fold_resolved_into_accumulator(
    accumulator: &mut ConsensusAccumulator,
    resolved: &[(String, &[GpsPoint], std::sync::Arc<RTree<IndexedPoint>>)],
    proximity_threshold: f64,
) {
    if resolved.is_empty() {
        return;
    }

    let threshold_deg = proximity_threshold / 111_000.0;
    let threshold_deg_sq = threshold_deg * threshold_deg;
    let epsilon = 0.000001;

    // Compute per-reference-point contributions from each new trace.
    // Done as map-then-merge so we don't take a mut borrow across the
    // parallel iterator on the read side.
    let reference_coords: Vec<[f64; 2]> = accumulator
        .reference
        .iter()
        .map(|p| [p.latitude, p.longitude])
        .collect();

    struct TraceContribution {
        // For each reference point, the contribution from this trace
        // (lat_w, lng_w, elev_w, w, elev_w_weight, dist_meters, contributed)
        per_ref: Vec<(f64, f64, f64, f64, f64, f64, bool)>,
    }

    let compute_for_trace = |tree: &RTree<IndexedPoint>, trace: &[GpsPoint]| -> TraceContribution {
        let mut per_ref =
            Vec::with_capacity(reference_coords.len());
        for ref_coords in &reference_coords {
            if let Some(nearest) = tree.nearest_neighbor(ref_coords) {
                let dist_sq = nearest.distance_2(ref_coords);
                if dist_sq <= threshold_deg_sq {
                    let trace_point = &trace[nearest.idx];
                    let dist_m = dist_sq.sqrt() * 111_000.0;
                    let weight = 1.0 / (dist_m + epsilon);
                    let (elev_w, elev_w_weight) = match trace_point.elevation {
                        Some(e) => (e * weight, weight),
                        None => (0.0, 0.0),
                    };
                    per_ref.push((
                        trace_point.latitude * weight,
                        trace_point.longitude * weight,
                        elev_w,
                        weight,
                        elev_w_weight,
                        dist_m,
                        true,
                    ));
                    continue;
                }
            }
            per_ref.push((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false));
        }
        TraceContribution { per_ref }
    };

    #[cfg(feature = "parallel")]
    let contributions: Vec<TraceContribution> = resolved
        .par_iter()
        .map(|(_, trace, tree)| compute_for_trace(tree.as_ref(), trace))
        .collect();

    #[cfg(not(feature = "parallel"))]
    let contributions: Vec<TraceContribution> = resolved
        .iter()
        .map(|(_, trace, tree)| compute_for_trace(tree.as_ref(), trace))
        .collect();

    // Merge contributions into the accumulator (serial — mutating shared
    // state, and the per-trace work above already parallelised the heavy
    // R-tree queries).
    for contribution in &contributions {
        for (i, contrib) in contribution.per_ref.iter().enumerate() {
            if !contrib.6 {
                continue;
            }
            let slot = &mut accumulator.per_point[i];
            slot.weighted_lat_sum += contrib.0;
            slot.weighted_lng_sum += contrib.1;
            slot.weighted_elev_sum += contrib.2;
            slot.total_weight += contrib.3;
            slot.elev_weight += contrib.4;
            slot.distance_sum += contrib.5;
            slot.observation_count += 1;
        }
    }

    accumulator.trace_count += resolved.len() as u32;
}

/// Materialize the current accumulator state into a [`ConsensusResult`].
fn accumulator_to_result(
    accumulator: &ConsensusAccumulator,
    proximity_threshold: f64,
) -> ConsensusResult {
    let n = accumulator.reference.len();
    let mut polyline = Vec::with_capacity(n);
    let mut point_density = Vec::with_capacity(n);
    let mut total_spread = 0.0;
    let mut total_point_observations = 0u32;

    for (i, slot) in accumulator.per_point.iter().enumerate() {
        if slot.total_weight > 0.0 {
            let lat = slot.weighted_lat_sum / slot.total_weight;
            let lng = slot.weighted_lng_sum / slot.total_weight;
            let elev = if slot.elev_weight > 0.0 {
                Some(slot.weighted_elev_sum / slot.elev_weight)
            } else {
                accumulator.reference[i].elevation
            };
            polyline.push(GpsPoint {
                latitude: lat,
                longitude: lng,
                elevation: elev,
            });
            let avg_dist = if slot.observation_count > 0 {
                slot.distance_sum / slot.observation_count as f64
            } else {
                0.0
            };
            total_spread += avg_dist;
        } else {
            polyline.push(accumulator.reference[i]);
        }
        point_density.push(slot.observation_count);
        total_point_observations += slot.observation_count;
    }

    let observation_count = accumulator.trace_count;
    let average_spread = if total_point_observations > 0 {
        total_spread / (n as f64)
    } else {
        proximity_threshold
    };
    let obs_factor = (observation_count as f64).min(10.0) / 10.0;
    let spread_factor = 1.0 - (average_spread / proximity_threshold).min(1.0);
    let confidence = (obs_factor * 0.5 + spread_factor * 0.5).clamp(0.0, 1.0);

    ConsensusResult {
        polyline,
        confidence,
        observation_count,
        average_spread,
        point_density,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_corridor(lat0: f64, lng0: f64, n: usize) -> Vec<GpsPoint> {
        (0..n)
            .map(|i| GpsPoint::with_elevation(lat0, lng0 + (i as f64) * 1e-4, 300.0))
            .collect()
    }

    fn jitter(track: &[GpsPoint], dx_deg: f64, seed: u64) -> Vec<GpsPoint> {
        // Deterministic per-point jitter — small, well within proximity threshold.
        track
            .iter()
            .enumerate()
            .map(|(i, p)| GpsPoint::with_elevation(
                p.latitude + dx_deg * (((i as u64).wrapping_mul(seed) % 7) as f64 - 3.0) * 0.1,
                p.longitude,
                p.elevation.unwrap_or(300.0),
            ))
            .collect()
    }

    #[test]
    fn full_and_incremental_paths_produce_equivalent_polylines() {
        // Build 5 traces; full vs incremental-via-accumulator must match.
        let reference = make_corridor(47.37, 8.55, 80);
        let traces: Vec<(String, Vec<GpsPoint>)> = (0..5)
            .map(|i| (format!("a_{i}"), jitter(&reference, 1e-5, (i + 1) as u64)))
            .collect();

        let traces_only: Vec<Vec<GpsPoint>> =
            traces.iter().map(|(_, t)| t.clone()).collect();

        let full = compute_consensus_polyline(&reference, &traces_only, 50.0);

        let mut acc = ConsensusAccumulator::new(reference.clone());
        let inc = merge_traces_into_consensus(&mut acc, &traces, 50.0);

        assert_eq!(full.polyline.len(), inc.polyline.len());
        for (p, q) in full.polyline.iter().zip(inc.polyline.iter()) {
            assert!(
                (p.latitude - q.latitude).abs() < 1e-9,
                "lat mismatch: {} vs {}",
                p.latitude,
                q.latitude
            );
            assert!(
                (p.longitude - q.longitude).abs() < 1e-9,
                "lng mismatch: {} vs {}",
                p.longitude,
                q.longitude
            );
        }
        assert_eq!(full.observation_count, inc.observation_count);
    }

    #[test]
    fn incremental_in_two_batches_matches_single_batch() {
        let reference = make_corridor(47.37, 8.55, 60);
        let traces: Vec<(String, Vec<GpsPoint>)> = (0..6)
            .map(|i| (format!("a_{i}"), jitter(&reference, 1e-5, (i + 1) as u64)))
            .collect();

        // Path A: all at once
        let mut acc_all = ConsensusAccumulator::new(reference.clone());
        let res_all = merge_traces_into_consensus(&mut acc_all, &traces, 50.0);

        // Path B: 3 + 3
        let mut acc_split = ConsensusAccumulator::new(reference.clone());
        merge_traces_into_consensus(&mut acc_split, &traces[..3], 50.0);
        let res_split = merge_traces_into_consensus(&mut acc_split, &traces[3..], 50.0);

        assert_eq!(res_all.polyline.len(), res_split.polyline.len());
        for (p, q) in res_all.polyline.iter().zip(res_split.polyline.iter()) {
            assert!(
                (p.latitude - q.latitude).abs() < 1e-9,
                "lat mismatch after 2-batch merge: {} vs {}",
                p.latitude,
                q.latitude
            );
        }
        assert_eq!(acc_all.absorbed_activity_ids, acc_split.absorbed_activity_ids);
        assert_eq!(acc_all.trace_count, acc_split.trace_count);
    }

    #[test]
    fn duplicate_activity_id_not_double_counted() {
        let reference = make_corridor(47.37, 8.55, 30);
        let trace = (
            "a_dup".to_string(),
            jitter(&reference, 1e-5, 1),
        );
        let mut acc = ConsensusAccumulator::new(reference.clone());
        merge_traces_into_consensus(&mut acc, &[trace.clone()], 50.0);
        let mid_total_weight = acc.per_point[0].total_weight;

        merge_traces_into_consensus(&mut acc, &[trace], 50.0);
        let after = acc.per_point[0].total_weight;

        assert_eq!(
            mid_total_weight, after,
            "duplicate activity_id was counted twice"
        );
        assert_eq!(acc.trace_count, 1);
    }

    #[test]
    fn accumulator_serializes_round_trip() {
        let reference = make_corridor(47.37, 8.55, 20);
        let trace = ("a".to_string(), jitter(&reference, 1e-5, 1));

        let mut acc = ConsensusAccumulator::new(reference.clone());
        merge_traces_into_consensus(&mut acc, &[trace.clone()], 50.0);

        let json = serde_json::to_string(&acc).expect("serialize");
        let mut acc2: ConsensusAccumulator = serde_json::from_str(&json).expect("deserialize");

        // Add another trace; merging into the deserialized accumulator
        // should produce the same result as merging into the original.
        let extra = ("b".to_string(), jitter(&reference, 1e-5, 2));
        let r1 = merge_traces_into_consensus(&mut acc, &[extra.clone()], 50.0);
        let r2 = merge_traces_into_consensus(&mut acc2, &[extra], 50.0);

        for (p, q) in r1.polyline.iter().zip(r2.polyline.iter()) {
            assert!((p.latitude - q.latitude).abs() < 1e-12);
            assert!((p.longitude - q.longitude).abs() < 1e-12);
        }
        assert_eq!(r1.observation_count, r2.observation_count);
    }

    #[test]
    fn build_accumulator_matches_compute() {
        let reference = make_corridor(47.37, 8.55, 50);
        let traces: Vec<(String, Vec<GpsPoint>)> = (0..4)
            .map(|i| (format!("a_{i}"), jitter(&reference, 1e-5, (i + 1) as u64)))
            .collect();
        let traces_only: Vec<Vec<GpsPoint>> = traces.iter().map(|(_, t)| t.clone()).collect();

        let full = compute_consensus_polyline(&reference, &traces_only, 50.0);
        let acc = build_accumulator_from_traces(&reference, &traces, 50.0);
        let from_acc = accumulator_to_result(&acc, 50.0);

        for (p, q) in full.polyline.iter().zip(from_acc.polyline.iter()) {
            assert!((p.latitude - q.latitude).abs() < 1e-9);
            assert!((p.longitude - q.longitude).abs() < 1e-9);
        }
    }
}
