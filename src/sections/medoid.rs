//! Medoid selection - The key innovation for section detection.
//!
//! The medoid is the actual GPS trace with minimum total AMD (Average Minimum Distance)
//! to all other traces. This ensures we return REAL GPS points, not artificial interpolations.

use super::overlap::OverlapCluster;
use crate::GpsPoint;
use crate::matching;
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

/// One pass of a medoid's candidate set: interned track, start, end.
pub(crate) type PassKey = (u32, u32, u32);

/// An unordered pair of passes, the lower key first.
type PairKey = (PassKey, PassKey);

fn pair_key(a: PassKey, b: PassKey) -> PairKey {
    if a <= b { (a, b) } else { (b, a) }
}

/// What earlier medoid picks computed, pure in the passes' points: the
/// symmetric AMD per unordered pass pair. A pick over a grown candidate
/// reads the pairs the last pick scored and computes only the pairs its
/// comparison rule names for the first time. The 50-point resamples are not
/// held: a missed pair mostly names a pass no earlier pick compared against,
/// so a held resample served about one in eight and saved a few per cent of
/// the work, for 2 MB a cache.
#[derive(Clone, Default, Serialize, Deserialize)]
pub(crate) struct MedoidMemo {
    /// Each pass a held pair names, numbered in the order it was first
    /// filed. A pair entry then keys by two numbers, 16 bytes rather than
    /// the 32 two pass keys take.
    passes: HashMap<PassKey, u32>,
    /// The score of each held pair, by the two pass numbers, lower first.
    amds: HashMap<(u32, u32), f64>,
}

impl MedoidMemo {
    /// File what one pick computed.
    pub(crate) fn absorb(&mut self, fresh: MedoidFresh) {
        for ((a, b), v) in fresh.amds {
            let (a, b) = (self.intern(a), self.intern(b));
            self.amds.insert((a.min(b), a.max(b)), v);
        }
    }

    fn intern(&mut self, pass: PassKey) -> u32 {
        let next = self.passes.len() as u32;
        *self.passes.entry(pass).or_insert(next)
    }

    fn amd(&self, (a, b): PairKey) -> Option<f64> {
        let (a, b) = (*self.passes.get(&a)?, *self.passes.get(&b)?);
        self.amds.get(&(a.min(b), a.max(b))).copied()
    }

    /// Clear the memo once it has grown past `cap` pairs.
    pub(crate) fn sweep(&mut self, cap: usize) {
        if self.amds.len() > cap {
            self.amds.clear();
            self.passes.clear();
        }
    }

    #[cfg(all(test, feature = "synthetic"))]
    pub(crate) fn amd_count(&self) -> usize {
        self.amds.len()
    }
}

/// The pair scores one pick computed, for its caller to file in the memo,
/// one row per pair it evaluated, so also the pick's work.
#[derive(Default)]
pub(crate) struct MedoidFresh {
    pub(crate) amds: Vec<(PairKey, f64)>,
}

/// A memo to read, and the interned numbers its keys name tracks by.
#[derive(Clone, Copy)]
pub(crate) struct MedoidCtx<'a> {
    pub(crate) memo: &'a MedoidMemo,
    pub(crate) nums: &'a HashMap<&'a str, u32>,
}

/// Select the medoid trace from a cluster using a track map for point resolution.
/// The medoid is the actual GPS trace with minimum total AMD to all other traces.
/// This ensures we return REAL GPS points, not artificial interpolations.
#[cfg(all(test, feature = "synthetic"))]
pub(crate) fn select_medoid(
    cluster: &OverlapCluster,
    track_map: &HashMap<&str, &[GpsPoint]>,
) -> (String, Vec<GpsPoint>) {
    select_medoid_with(cluster, track_map, None, &mut MedoidFresh::default())
}

/// [`select_medoid`] reading pair scores from `ctx` where it holds them, and writing what it computed into `fresh`. The pick is the
/// same with or without a memo: every value is pure in its passes' points,
/// and the comparison set and tie rule read only the canonical order.
pub(crate) fn select_medoid_with(
    cluster: &OverlapCluster,
    track_map: &HashMap<&str, &[GpsPoint]>,
    ctx: Option<MedoidCtx<'_>>,
    fresh: &mut MedoidFresh,
) -> (String, Vec<GpsPoint>) {
    // Every pass is its own candidate, keyed by (activity, range): the
    // medoid is weighted by traversal, so lapped ground is represented
    // by its typical revolution, not by whichever pass came first.
    let mut candidates: Vec<(&str, usize, usize, &[GpsPoint])> = Vec::new();
    let mut seen: HashSet<(&str, usize, usize)> = HashSet::new();

    for overlap in &cluster.overlaps {
        for (id, range) in [
            (overlap.activity_a.as_str(), overlap.range_a),
            (overlap.activity_b.as_str(), overlap.range_b),
        ] {
            if seen.insert((id, range.0, range.1))
                && let Some(track) = track_map.get(id)
            {
                let end = range.1.min(track.len());
                if range.0 < end {
                    candidates.push((id, range.0, end, &track[range.0..end]));
                }
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
    let n = candidates.len();

    if n == 0 {
        return (String::new(), Vec::new());
    }

    if n == 1 {
        return (candidates[0].0.to_string(), candidates[0].3.to_vec());
    }

    // Memo keys name a pass by its interned track. Without a memo, or for a
    // track the numbering does not know, the pass's position stands in and
    // nothing is read from or written to a memo.
    let interned: Option<Vec<PassKey>> = ctx.and_then(|c| {
        candidates
            .iter()
            .map(|&(id, s, e, _)| {
                let t = *c.nums.get(id)?;
                Some((t, u32::try_from(s).ok()?, u32::try_from(e).ok()?))
            })
            .collect()
    });
    let memo = interned.as_ref().and(ctx.map(|c| c.memo));
    let keys: Vec<PassKey> = interned.unwrap_or_else(|| {
        candidates
            .iter()
            .enumerate()
            .map(|(i, _)| (i as u32, 0, 0))
            .collect()
    });

    // Up to ten passes, each is scored against all the others. Above that,
    // each is scored against up to five taken at a fixed even stride through
    // the canonical order.
    let use_full_pairwise = n <= 10;
    let compared: Vec<usize> = if use_full_pairwise {
        (0..n).collect()
    } else {
        let sample_size = 5.min(n - 1);
        let step = n / sample_size;
        (0..n).step_by(step.max(1)).take(sample_size).collect()
    };

    // Every pair the rule scores, each once: the score is symmetric, so
    // (i, j) and (j, i) share one evaluation.
    let mut scores: HashMap<PairKey, f64> = HashMap::new();
    let mut missing: Vec<(PairKey, usize, usize)> = Vec::new();
    let mut queued: HashSet<PairKey> = HashSet::new();
    for &j in &compared {
        for i in 0..n {
            if i == j {
                continue;
            }
            let pk = pair_key(keys[i], keys[j]);
            if scores.contains_key(&pk) || !queued.insert(pk) {
                continue;
            }
            match memo.and_then(|m| m.amd(pk)) {
                Some(v) => {
                    scores.insert(pk, v);
                }
                None => missing.push((pk, i, j)),
            }
        }
    }

    #[cfg(feature = "profile")]
    eprintln!("PROFILE medoid_pairs {} {}", scores.len(), missing.len());

    if !missing.is_empty() {
        // Resample only the passes a missed pair names. AMD compares 50-point
        // resamples, and resampling inside each pairwise call made every pass
        // pay a full-track walk per pair instead of per trace.
        let mut needed: Vec<usize> = missing.iter().flat_map(|m| [m.1, m.2]).collect();
        needed.sort_unstable();
        needed.dedup();
        let resample = |k: usize| matching::resample_route(candidates[needed[k]].3, 50);
        #[cfg(feature = "parallel")]
        let resampled: Vec<Vec<GpsPoint>> =
            (0..needed.len()).into_par_iter().map(resample).collect();
        #[cfg(not(feature = "parallel"))]
        let resampled: Vec<Vec<GpsPoint>> = (0..needed.len()).map(resample).collect();
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
        let slot: HashMap<usize, usize> = needed.iter().enumerate().map(|(k, &i)| (i, k)).collect();
        let amd = |&(pk, i, j): &(PairKey, usize, usize)| -> (PairKey, f64) {
            let (a, b) = (slot[&i], slot[&j]);
            let a_to_b = amd_directed_pruned(&resampled[a], &trig[a], &resampled[b], &trig[b]);
            let b_to_a = amd_directed_pruned(&resampled[b], &trig[b], &resampled[a], &trig[a]);
            (pk, (a_to_b + b_to_a) / 2.0)
        };
        #[cfg(feature = "parallel")]
        let computed: Vec<(PairKey, f64)> = missing.par_iter().map(amd).collect();
        #[cfg(not(feature = "parallel"))]
        let computed: Vec<(PairKey, f64)> = missing.iter().map(amd).collect();
        scores.extend(computed.iter().copied());
        if memo.is_some() {
            fresh.amds.extend(computed);
        }
    }

    // Each pass's mean score over the passes it is compared with, summed in
    // canonical order; the lowest wins and a tie goes to the earlier pass.
    let mut best_idx = 0;
    let mut best_score = f64::MAX;
    for i in 0..n {
        let mut total = 0.0;
        let mut count = 0usize;
        for &j in &compared {
            if i != j {
                total += scores[&pair_key(keys[i], keys[j])];
                count += 1;
            }
        }
        if count == 0 {
            continue;
        }
        let score = if use_full_pairwise {
            total
        } else {
            total / count as f64
        };
        if score < best_score {
            best_score = score;
            best_idx = i;
        }
    }

    (
        candidates[best_idx].0.to_string(),
        candidates[best_idx].3.to_vec(),
    )
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

#[cfg(all(test, feature = "synthetic"))]
mod memo_tests {
    use super::*;
    use crate::sections::overlap::FullTrackOverlap;

    const METRE_DEG: f64 = 1.0 / 111_000.0;

    /// A gently wandering eastward pass of 80 points, offset north by a
    /// different amount per seed so no two passes score alike.
    fn pass(seed: usize) -> Vec<GpsPoint> {
        let north = ((seed * 37) % 23) as f64 - 11.0;
        (0..80)
            .map(|i| {
                let wobble = ((i + seed * 7) as f64 * 0.3).sin() * 2.0;
                GpsPoint::new(47.1 + (north + wobble) * METRE_DEG, 8.2 + (i as f64) * 1e-4)
            })
            .collect()
    }

    struct Ground {
        ids: Vec<String>,
        tracks: Vec<Vec<GpsPoint>>,
        /// Each pass's interned track number.
        nums: Vec<u32>,
    }

    impl Ground {
        fn new(ids: &[String]) -> Self {
            Ground {
                ids: ids.to_vec(),
                tracks: (0..ids.len()).map(pass).collect(),
                nums: (0..ids.len() as u32).collect(),
            }
        }

        /// The same passes without the first, numbered as before.
        fn without_first(&self) -> Self {
            Ground {
                ids: self.ids[1..].to_vec(),
                tracks: self.tracks[1..].to_vec(),
                nums: self.nums[1..].to_vec(),
            }
        }

        fn track_map(&self) -> HashMap<&str, &[GpsPoint]> {
            self.ids
                .iter()
                .zip(&self.tracks)
                .map(|(id, t)| (id.as_str(), t.as_slice()))
                .collect()
        }

        fn nums(&self) -> HashMap<&str, u32> {
            self.ids
                .iter()
                .zip(&self.nums)
                .map(|(id, &n)| (id.as_str(), n))
                .collect()
        }

        /// The anchor-paired cluster over the first `count` passes.
        fn cluster(&self, count: usize) -> OverlapCluster {
            let full = |t: usize| (0, self.tracks[t].len());
            OverlapCluster {
                overlaps: (1..count)
                    .map(|t| FullTrackOverlap {
                        activity_a: self.ids[0].clone(),
                        activity_b: self.ids[t].clone(),
                        range_a: full(0),
                        range_b: full(t),
                    })
                    .collect(),
                activity_ids: self.ids[..count].iter().cloned().collect(),
            }
        }
    }

    fn ids(count: usize, prefix: &str) -> Vec<String> {
        (0..count).map(|i| format!("{prefix}{i:03}")).collect()
    }

    /// Pick over `count` passes through `memo`, filing what it computed.
    /// Returns the pick and the number of pairs it evaluated.
    fn pick(ground: &Ground, count: usize, memo: &mut MedoidMemo) -> (String, usize) {
        let track_map = ground.track_map();
        let nums = ground.nums();
        let mut fresh = MedoidFresh::default();
        let (id, _) = select_medoid_with(
            &ground.cluster(count),
            &track_map,
            Some(MedoidCtx { memo, nums: &nums }),
            &mut fresh,
        );
        for &(pk, _) in &fresh.amds {
            assert!(
                memo.amd(pk).is_none(),
                "a pair the memo held was evaluated again"
            );
        }
        let evaluated = fresh.amds.len();
        memo.absorb(fresh);
        (id, evaluated)
    }

    fn cold(ground: &Ground, count: usize) -> String {
        select_medoid(&ground.cluster(count), &ground.track_map()).0
    }

    /// Scenario: a sampled candidate grows by one pass that sorts last, so
    /// the stride positions keep their passes.
    /// Expected behaviour: the grown pick matches the cold pick and scores
    /// only the newcomer against the five sampled passes.
    #[test]
    fn grown_candidate_scores_only_its_new_pairs() {
        let ground = Ground::new(&ids(41, "ride"));
        let mut memo = MedoidMemo::default();

        let (first, first_work) = pick(&ground, 40, &mut memo);
        assert_eq!(first, cold(&ground, 40));
        assert!(first_work > 0);

        let (grown, grown_work) = pick(&ground, 41, &mut memo);
        assert_eq!(grown, cold(&ground, 41));
        assert_eq!(grown_work, 5, "the newcomer against each sampled pass");

        let (again, again_work) = pick(&ground, 41, &mut memo);
        assert_eq!(again, grown);
        assert_eq!(again_work, 0, "a repeated pick evaluates nothing");
    }

    /// Scenario: the newcomer sorts first, so every stride position moves
    /// onto a different pass.
    /// Expected behaviour: the pick still matches the cold pick, and the
    /// pairs it scores are only those the first pick never named.
    #[test]
    fn shifted_stride_matches_the_cold_pick() {
        let mut names = ids(40, "ride");
        names.insert(0, "early".to_string());
        let ground = Ground::new(&names);
        let later = ground.without_first();
        let mut memo = MedoidMemo::default();

        let (first, _) = pick(&later, 40, &mut memo);
        assert_eq!(first, cold(&later, 40));
        let held = memo.amd_count();

        let (grown, grown_work) = pick(&ground, 41, &mut memo);
        assert_eq!(grown, cold(&ground, 41));
        let (_, cold_work) = pick(&ground, 41, &mut MedoidMemo::default());
        assert!(grown_work > 0 && grown_work < cold_work);
        assert_eq!(memo.amd_count(), held + grown_work);
    }

    /// Scenario: a candidate crosses from full pairwise scoring at ten
    /// passes to the sampled rule at eleven.
    /// Expected behaviour: both picks match the cold pick, and the eleventh
    /// pass costs only its pairs with the five sampled passes, since full
    /// pairwise already scored every pair among the first ten.
    #[test]
    fn crossing_the_full_pairwise_boundary() {
        let ground = Ground::new(&ids(11, "lap"));
        let mut memo = MedoidMemo::default();

        let (ten, ten_work) = pick(&ground, 10, &mut memo);
        assert_eq!(ten, cold(&ground, 10));
        assert_eq!(ten_work, 45, "every unordered pair once");

        let (eleven, eleven_work) = pick(&ground, 11, &mut memo);
        assert_eq!(eleven, cold(&ground, 11));
        assert_eq!(eleven_work, 5);
    }

    /// Scenario: the memo holds a different score for a pair than its
    /// points give, as if numbering had named the wrong track.
    /// Expected behaviour: the pick reads the memo, which is what proves the
    /// held value is the one used rather than recomputed.
    #[test]
    fn held_scores_are_read_not_recomputed() {
        let ground = Ground::new(&ids(3, "loop"));
        let mut memo = MedoidMemo::default();
        let (honest, _) = pick(&ground, 3, &mut memo);
        assert_eq!(honest, cold(&ground, 3));
        // Make the last pass look closest to everything.
        let last = memo
            .passes
            .iter()
            .find(|(pass, _)| pass.0 == 2)
            .map(|(_, &n)| n)
            .unwrap();
        for (&(a, b), v) in memo.amds.iter_mut() {
            *v = if a == last || b == last { 0.0 } else { 1e6 };
        }
        let (skewed, work) = pick(&ground, 3, &mut memo);
        assert_eq!(work, 0);
        assert_eq!(skewed, "loop002");
    }
}
