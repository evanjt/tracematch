//! Medoid selection - The key innovation for section detection.
//!
//! The medoid is the actual GPS trace with minimum total AMD (Average Minimum Distance)
//! to all other traces. This ensures we return REAL GPS points, not artificial interpolations.

use super::overlap::OverlapCluster;
use crate::GpsPoint;
use crate::matching;
#[cfg(feature = "parallel")]
use rayon::prelude::*;

/// Select the medoid trace from a cluster using a track map for point resolution.
/// The medoid is the actual GPS trace with minimum total AMD to all other traces.
/// This ensures we return REAL GPS points, not artificial interpolations.
pub fn select_medoid(
    cluster: &OverlapCluster,
    track_map: &std::collections::HashMap<&str, &[GpsPoint]>,
) -> (String, Vec<GpsPoint>) {
    // Collect all unique activity portions in this cluster
    let mut traces: Vec<(&str, Vec<GpsPoint>)> = Vec::new();
    let mut seen: std::collections::HashSet<&str> = std::collections::HashSet::new();

    for overlap in &cluster.overlaps {
        // Add track A's overlapping portion
        if seen.insert(overlap.activity_a.as_str()) {
            if let Some(track) = track_map.get(overlap.activity_a.as_str()) {
                let end = overlap.range_a.1.min(track.len());
                let points = track[overlap.range_a.0..end].to_vec();
                if !points.is_empty() {
                    traces.push((&overlap.activity_a, points));
                }
            }
        }
        // Add track B's overlapping portion
        if seen.insert(overlap.activity_b.as_str()) {
            if let Some(track) = track_map.get(overlap.activity_b.as_str()) {
                let end = overlap.range_b.1.min(track.len());
                let points = track[overlap.range_b.0..end].to_vec();
                if !points.is_empty() {
                    traces.push((&overlap.activity_b, points));
                }
            }
        }
    }

    if traces.is_empty() {
        return (String::new(), Vec::new());
    }

    if traces.len() == 1 {
        return (traces[0].0.to_string(), traces[0].1.clone());
    }

    // For small clusters, compute full pairwise AMD
    // For larger clusters (>10), use approximate method
    let use_full_pairwise = traces.len() <= 10;

    let mut best_idx = 0;
    let mut best_total_amd = f64::MAX;

    if use_full_pairwise {
        // Compute AMD for each trace to all others
        #[cfg(feature = "parallel")]
        {
            let (idx, _) = traces
                .par_iter()
                .enumerate()
                .map(|(i, (_, trace_i))| {
                    let total: f64 = traces
                        .iter()
                        .enumerate()
                        .filter(|(j, _)| *j != i)
                        .map(|(_, (_, trace_j))| average_min_distance(trace_i, trace_j))
                        .sum();
                    (i, total)
                })
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
                .unwrap_or((0, f64::MAX));
            best_idx = idx;
        }

        #[cfg(not(feature = "parallel"))]
        {
            for (i, (_, trace_i)) in traces.iter().enumerate() {
                let mut total_amd = 0.0;

                for (j, (_, trace_j)) in traces.iter().enumerate() {
                    if i != j {
                        total_amd += average_min_distance(trace_i, trace_j);
                    }
                }

                if total_amd < best_total_amd {
                    best_total_amd = total_amd;
                    best_idx = i;
                }
            }
        }
    } else {
        // Approximate: compare each to a random sample of 5 others
        let sample_size = 5.min(traces.len() - 1);

        for (i, (_, trace_i)) in traces.iter().enumerate() {
            let mut total_amd = 0.0;
            let mut count = 0;

            // Sample evenly distributed traces
            let step = traces.len() / sample_size;
            for j in (0..traces.len()).step_by(step.max(1)).take(sample_size) {
                if i != j {
                    total_amd += average_min_distance(trace_i, &traces[j].1);
                    count += 1;
                }
            }

            if count > 0 {
                let avg_amd = total_amd / count as f64;
                if avg_amd < best_total_amd {
                    best_total_amd = avg_amd;
                    best_idx = i;
                }
            }
        }
    }

    (traces[best_idx].0.to_string(), traces[best_idx].1.clone())
}

/// Compute stability: how well a trace aligns with a reference polyline.
/// Returns 0.0-1.0 where 1.0 = perfect alignment.
pub(crate) fn compute_stability(
    trace: &[GpsPoint],
    consensus: &[GpsPoint],
    proximity_threshold: f64,
) -> f64 {
    if trace.is_empty() || consensus.is_empty() || proximity_threshold <= 0.0 {
        return 0.0;
    }
    let amd = average_min_distance(trace, consensus);
    if amd == f64::MAX {
        return 0.0;
    }
    (1.0 - (amd / proximity_threshold)).clamp(0.0, 1.0)
}

/// Average Minimum Distance between two polylines.
///
/// Resamples both polylines to 50 points for fair comparison,
/// then delegates to `matching::average_min_distance` for the actual computation.
pub(crate) fn average_min_distance(poly_a: &[GpsPoint], poly_b: &[GpsPoint]) -> f64 {
    if poly_a.is_empty() || poly_b.is_empty() {
        return f64::MAX;
    }

    // Resample both to same number of points for fair comparison
    let n = 50;
    let resampled_a = matching::resample_route(poly_a, n);
    let resampled_b = matching::resample_route(poly_b, n);

    // Compute symmetric AMD using the canonical implementation
    let amd_a_to_b = matching::average_min_distance(&resampled_a, &resampled_b);
    let amd_b_to_a = matching::average_min_distance(&resampled_b, &resampled_a);

    (amd_a_to_b + amd_b_to_a) / 2.0
}
