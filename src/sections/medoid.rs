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
    // Every pass is its own candidate, keyed by (activity, range): the
    // medoid is weighted by traversal, so lapped ground is represented
    // by its typical revolution, not by whichever pass came first.
    let mut candidates: Vec<(&str, usize, usize, Vec<GpsPoint>)> = Vec::new();
    let mut seen: std::collections::HashSet<(&str, usize, usize)> =
        std::collections::HashSet::new();

    for overlap in &cluster.overlaps {
        // Add track A's overlapping portion
        if seen.insert((
            overlap.activity_a.as_str(),
            overlap.range_a.0,
            overlap.range_a.1,
        )) && let Some(track) = track_map.get(overlap.activity_a.as_str())
        {
            let end = overlap.range_a.1.min(track.len());
            let points = track[overlap.range_a.0..end].to_vec();
            if !points.is_empty() {
                candidates.push((&overlap.activity_a, overlap.range_a.0, end, points));
            }
        }
        // Add track B's overlapping portion
        if seen.insert((
            overlap.activity_b.as_str(),
            overlap.range_b.0,
            overlap.range_b.1,
        )) && let Some(track) = track_map.get(overlap.activity_b.as_str())
        {
            let end = overlap.range_b.1.min(track.len());
            let points = track[overlap.range_b.0..end].to_vec();
            if !points.is_empty() {
                candidates.push((&overlap.activity_b, overlap.range_b.0, end, points));
            }
        }
    }

    // The overlaps arrive in whatever order the caller assembled them, and both
    // the sampled comparison set and the first-wins tie-breaks below read
    // positions. Canonicalise by (activity, range) so the medoid is a function
    // of the candidate set rather than of its arrangement.
    candidates.sort_by(|a, b| {
        a.0.cmp(b.0)
            .then_with(|| a.1.cmp(&b.1))
            .then_with(|| a.2.cmp(&b.2))
    });
    let traces: Vec<(&str, Vec<GpsPoint>)> = candidates
        .into_iter()
        .map(|(id, _, _, pts)| (id, pts))
        .collect();

    if traces.is_empty() {
        return (String::new(), Vec::new());
    }

    if traces.len() == 1 {
        return (traces[0].0.to_string(), traces[0].1.clone());
    }

    // Resample every pass once. AMD compares 50-point resamples, and
    // resampling inside each pairwise call made every pass pay a full-track
    // walk per pair instead of per trace.
    let resampled: Vec<Vec<GpsPoint>> = {
        #[cfg(feature = "parallel")]
        {
            traces
                .par_iter()
                .map(|(_, t)| matching::resample_route(t, 50))
                .collect()
        }
        #[cfg(not(feature = "parallel"))]
        {
            traces
                .iter()
                .map(|(_, t)| matching::resample_route(t, 50))
                .collect()
        }
    };
    let trig: Vec<Vec<(f64, f64, f64)>> = resampled
        .iter()
        .map(|pts| {
            pts.iter()
                .map(|p| {
                    let lat = p.latitude.to_radians();
                    (lat, p.longitude.to_radians(), lat.cos())
                })
                .collect()
        })
        .collect();
    let amd = |i: usize, j: usize| -> f64 {
        if traces[i].1.is_empty() || traces[j].1.is_empty() {
            return f64::MAX;
        }
        let a_to_b = amd_directed_pruned(&resampled[i], &trig[i], &resampled[j], &trig[j]);
        let b_to_a = amd_directed_pruned(&resampled[j], &trig[j], &resampled[i], &trig[i]);
        (a_to_b + b_to_a) / 2.0
    };

    // For small clusters, compute full pairwise AMD
    // For larger clusters (>10), use approximate method
    let use_full_pairwise = traces.len() <= 10;

    let mut best_idx = 0;
    let mut best_total_amd = f64::MAX;

    if use_full_pairwise {
        // Compute AMD for each trace to all others
        #[cfg(feature = "parallel")]
        {
            let (idx, _) = (0..traces.len())
                .into_par_iter()
                .map(|i| {
                    let total: f64 = (0..traces.len())
                        .filter(|&j| j != i)
                        .map(|j| amd(i, j))
                        .sum();
                    (i, total)
                })
                .min_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.cmp(&b.0)))
                .unwrap_or((0, f64::MAX));
            best_idx = idx;
        }

        #[cfg(not(feature = "parallel"))]
        {
            for i in 0..traces.len() {
                let mut total_amd = 0.0;

                for j in 0..traces.len() {
                    if i != j {
                        total_amd += amd(i, j);
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

        for i in 0..traces.len() {
            let mut total_amd = 0.0;
            let mut count = 0;

            // Sample evenly distributed traces
            let step = traces.len() / sample_size;
            for j in (0..traces.len()).step_by(step.max(1)).take(sample_size) {
                if i != j {
                    total_amd += amd(i, j);
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

/// One direction of AMD with the haversine evaluated only for each point's
/// winner: the distance is strictly monotone in the half-chord term, so the
/// scan compares that cheap term (with per-point cached `cos lat`) and the
/// full formula runs once per source point. Same value as the plain scan.
fn amd_directed_pruned(
    a: &[GpsPoint],
    a_trig: &[(f64, f64, f64)],
    b: &[GpsPoint],
    b_trig: &[(f64, f64, f64)],
) -> f64 {
    if a.is_empty() || b.is_empty() {
        return f64::INFINITY;
    }
    let total: f64 = a
        .iter()
        .zip(a_trig)
        .map(|(p1, &(lat1, lng1, cos1))| {
            let mut best_term = f64::INFINITY;
            let mut best_j = 0usize;
            for (j, &(lat2, lng2, cos2)) in b_trig.iter().enumerate() {
                let sdlat = ((lat2 - lat1) * 0.5).sin();
                let sdlng = ((lng2 - lng1) * 0.5).sin();
                let term = sdlat * sdlat + cos1 * cos2 * sdlng * sdlng;
                if term < best_term {
                    best_term = term;
                    best_j = j;
                }
            }
            crate::geo_utils::haversine_distance(p1, &b[best_j])
        })
        .sum();
    total / a.len() as f64
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
