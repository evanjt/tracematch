//! Assign-once identity and hysteresis over a churny section catalogue.
//!
//! The Unified batch is legitimately non-monotone: sections dissolve and reform
//! as evidence accumulates. That churn is correct, but a user watching the
//! map does not want to see it. This module is the pure decision half: it
//! matches a fresh batch catalogue against the ids already assigned, so a piece
//! of ground keeps its id across a recompute, and it low-pass filters the churn
//! so a single add can never flip the visible view while the view still
//! converges to the batch over many adds.
//!
//! Two pieces, both pure and deterministic (no HashMap iteration order, no float
//! ordering ambiguity), so the same inputs always produce byte-identical output:
//!
//! - [`plan_identity`] resolves the bipartite same-corridor graph between the
//!   prior ids and the fresh candidate grounds into carry / mint / split / merge
//!   / dissolve, matching the harness ground metric (bidirectional coverage of
//!   at least 0.6 at 50 m). tracematch emits ground; the caller supplies the
//!   opaque ids. This decides only which candidate inherits which prior id.
//! - [`HysteresisState::step`] applies a plan through a debounce: additions are
//!   immediate, but a dissolve or a re-cut only fires once the batch has
//!   pressed the change for `k` detects uninterrupted by a decisive
//!   continuation (an agreement carry, or the other change firing). The two
//!   streaks accumulate through each other's steps, so a marginal capture
//!   cannot erase absence evidence and absence cannot erase re-cut evidence.
//!   It is a debounce, not a freeze: a genuinely sustained change still
//!   applies (after `k`).
//!
//! Identity lives in the engine (veloqrs) in production; this is the lab-first
//! prototype that proves the mechanism on the synthetic corpora before the
//! stateful registry is built on top. The engine swaps the deterministic
//! `s_<n>` mint here for its opaque `s_<ts>__<rand>` scheme and persists the
//! state; the carry-forward structure is identical. Design:
//! `~/.claude/plans/b2-identity-hysteresis-design.md`.

use std::collections::{BTreeMap, HashMap};

use serde::{Deserialize, Serialize};

use crate::GpsPoint;
use crate::geo_utils::haversine_distance;
use crate::sections::FrequentSection;

// ============================================================================
// The ground metric (mirrors the E2E harness so a carry counts toward its
// identity_retention gate) and the decisive-margin predicates.
// ============================================================================

/// Ground-match tolerance: half the ~100 m evidence cell. Two lines within this
/// of each other describe the same corridor.
pub const GROUND_TOL_M: f64 = 50.0;

/// Same-corridor gate: a majority of either polyline lies within [`GROUND_TOL_M`]
/// of the other. Identical to the harness `ground_matches`, so a carried id is
/// counted as retained by the gate it is scored against.
pub const CARRY_COVERAGE: f64 = 0.6;

/// A carried section is a material re-cut (debounced) rather than a free
/// geometry update when the two extents overlap by LESS than this. Above it the
/// extents agree to within ~15% and the new geometry is adopted immediately.
/// The 0.85 plateau is the A1-calibrated corridor-persistence tolerance, shared
/// with detection so "the engine prefers a different cut" has one definition.
pub const RECUT_AGREEMENT: f64 = 0.85;

/// A held section's ground is decisively gone when this fraction of it is
/// uncovered by every batch section. Below it the ground is still substantially
/// present, so a dissolve is not counted toward the debounce.
pub const DISSOLVE_PRESSURE_HI: f64 = 0.7;

/// Default sustained-count before a dissolve or re-cut applies. `k >= 2` is the
/// minimum that stops a single add flipping anything; `k = 3` also damps a
/// two-step flicker while a genuinely gone section still dissolves within a
/// normal backfill drip.
pub const DEFAULT_K: u8 = 3;

/// Fraction of `samples` within `tol_m` of any point on `line`. The directional
/// half of the coverage metric. Two empty grounds are indistinguishable, so
/// both-empty is vacuously full coverage: a degenerate detector output carries
/// its held copy instead of dissolving and re-minting an id every detect. One
/// empty side against real ground is no evidence of sharing and scores 0.
fn coverage(samples: &[GpsPoint], line: &[GpsPoint], tol_m: f64) -> f64 {
    if samples.is_empty() && line.is_empty() {
        return 1.0;
    }
    if samples.is_empty() || line.is_empty() {
        return 0.0;
    }
    // Exact accelerator: bucket the line's points into a degree grid whose
    // cells span at least `tol_m` in both axes, so any line point within
    // tolerance of a sample must sit in the sample's 3x3 cell ring. The
    // haversine predicate is unchanged, the grid only prunes candidates -
    // so the fraction is identical to the plain double scan (the oracle
    // test pins it), at O(A + B) instead of O(A x B).
    let cell_lat = tol_m / 110_574.0;
    let max_abs_lat = samples
        .iter()
        .chain(line.iter())
        .map(|p| p.latitude.abs())
        .fold(0.0f64, f64::max)
        .min(85.0);
    let cell_lng = tol_m / (111_320.0 * max_abs_lat.to_radians().cos()).max(1.0);
    let key = |lat: f64, lng: f64| {
        (
            (lat / cell_lat).floor() as i64,
            (lng / cell_lng).floor() as i64,
        )
    };
    let mut grid: HashMap<(i64, i64), Vec<&GpsPoint>> = HashMap::new();
    for p in line {
        grid.entry(key(p.latitude, p.longitude))
            .or_default()
            .push(p);
    }
    let covered = samples
        .iter()
        .filter(|s| {
            let (cy, cx) = key(s.latitude, s.longitude);
            (-1..=1).any(|dy| {
                (-1..=1).any(|dx| {
                    grid.get(&(cy + dy, cx + dx))
                        .is_some_and(|pts| pts.iter().any(|p| haversine_distance(s, p) <= tol_m))
                })
            })
        })
        .count();
    covered as f64 / samples.len() as f64
}

/// Whether two grounds describe the same corridor: a majority of either lies
/// within [`GROUND_TOL_M`] of the other. Tolerant to extent growth, so a cold
/// section that later grows a longer supported extent still matches its prior.
pub fn shares_ground(a: &[GpsPoint], b: &[GpsPoint]) -> bool {
    coverage(a, b, GROUND_TOL_M) >= CARRY_COVERAGE || coverage(b, a, GROUND_TOL_M) >= CARRY_COVERAGE
}

/// Side of the global cell a section's heart is quantised to for its id:
/// twice [`GROUND_TOL_M`], independent of every setting, so two libraries
/// that cut the same ground mint the same id.
pub const ANCHOR_CELL_M: f64 = 100.0;

/// The point half way along a line by arc length: where a section is, in
/// one point, for anchoring its id.
pub fn section_heart(polyline: &[GpsPoint]) -> Option<GpsPoint> {
    if polyline.is_empty() {
        return None;
    }
    let total = crate::matching::calculate_route_distance(polyline);
    let half = total / 2.0;
    let mut run = 0.0;
    for w in polyline.windows(2) {
        let d = crate::geo_utils::haversine_distance(&w[0], &w[1]);
        if run + d >= half {
            let t = if d > 0.0 { (half - run) / d } else { 0.0 };
            return Some(GpsPoint::new(
                w[0].latitude + (w[1].latitude - w[0].latitude) * t,
                w[0].longitude + (w[1].longitude - w[0].longitude) * t,
            ));
        }
        run += d;
    }
    polyline.last().copied()
}

/// Banded metric quantisation of a point: the latitude band is
/// [`ANCHOR_CELL_M`] tall, and within a band longitude is measured in
/// metres at the band's own latitude, so cells stay square-ish everywhere
/// and never depend on a corpus or a config.
pub fn earth_cell(p: &GpsPoint) -> (i64, i64) {
    const M_PER_DEG_LAT: f64 = 111_132.0;
    let lat_cell = (p.latitude * M_PER_DEG_LAT / ANCHOR_CELL_M).floor();
    let band_lat = lat_cell * ANCHOR_CELL_M / M_PER_DEG_LAT;
    let m_per_deg_lng = (111_320.0 * band_lat.to_radians().cos()).max(1.0);
    let lng_cell = (p.longitude * m_per_deg_lng / ANCHOR_CELL_M).floor();
    (lat_cell as i64, lng_cell as i64)
}

/// The mutual overlap of two grounds: `min(cov(a->b), cov(b->a))`. High only
/// when the two cover each other, so it separates a clean 1:1 (near 1.0) from a
/// subset piece of a split/merge (well below 1.0). Drives the split winner and
/// the re-cut margin.
pub fn mutual_overlap(a: &[GpsPoint], b: &[GpsPoint]) -> f64 {
    coverage(a, b, GROUND_TOL_M).min(coverage(b, a, GROUND_TOL_M))
}

/// A ground's bounding box padded by [`GROUND_TOL_M`] (longitude scaled by the
/// box's own mid-latitude). Two grounds within tolerance of each other must
/// have overlapping padded boxes, so this gates the pairwise coverage scan.
fn ground_bbox_padded(pts: &[GpsPoint]) -> (f64, f64, f64, f64) {
    let mut bb = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for p in pts {
        bb.0 = bb.0.min(p.latitude);
        bb.1 = bb.1.max(p.latitude);
        bb.2 = bb.2.min(p.longitude);
        bb.3 = bb.3.max(p.longitude);
    }
    let pad_lat = GROUND_TOL_M / 111_000.0;
    let mid = ((bb.0 + bb.1) * 0.5).to_radians();
    let pad_lng = GROUND_TOL_M / (111_320.0 * mid.cos().abs().max(0.01));
    (
        bb.0 - pad_lat,
        bb.1 + pad_lat,
        bb.2 - pad_lng,
        bb.3 + pad_lng,
    )
}

/// How decisively a held ground has left the batch: `1 - max coverage of the
/// held ground by any batch section`. 1.0 when no batch section covers it at
/// all, 0.0 when one covers it entirely. The dissolve debounce only counts a
/// step where this is `>= dissolve_pressure_hi`.
pub fn dissolve_pressure(held: &[GpsPoint], next: &[CandidateSection]) -> f64 {
    let best = next
        .iter()
        .map(|c| coverage(held, &c.polyline, GROUND_TOL_M))
        .fold(0.0_f64, f64::max);
    1.0 - best
}

/// Total metres a ground represents (haversine along its polyline). A split
/// tie-break and a merge tie-break both prefer the ground carrying more support.
fn polyline_metres(pts: &[GpsPoint]) -> f64 {
    pts.windows(2)
        .map(|w| haversine_distance(&w[0], &w[1]))
        .sum()
}

/// South-west corner `(min lat, min lng)` of a ground, the final deterministic
/// tie-break for a split winner (two candidates equal on every prior key still
/// resolve to one, always the same one).
fn sw_corner(pts: &[GpsPoint]) -> (f64, f64) {
    let mut sw = (f64::MAX, f64::MAX);
    for p in pts {
        sw.0 = sw.0.min(p.latitude);
        sw.1 = sw.1.min(p.longitude);
    }
    sw
}

// ============================================================================
// Plan inputs and output
// ============================================================================

/// A prior section carrying its assigned stable id and the tie-break metadata
/// the registry holds for it. `first_seen` is a monotonic ordinal: lower is more
/// senior, and the senior prior wins a merge.
#[derive(Clone, Debug, PartialEq)]
pub struct PriorSection {
    pub id: String,
    pub polyline: Vec<GpsPoint>,
    pub first_seen: u64,
    pub visit_count: u32,
}

/// A fresh batch section with no id yet: the detector emits ground, identity is
/// assigned here. `visit_count` feeds a split tie-break and folds into the
/// carried section immediately.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CandidateSection {
    pub polyline: Vec<GpsPoint>,
    pub visit_count: u32,
}

impl CandidateSection {
    /// Build a candidate from a detector section, dropping the throwaway
    /// positional id. The engine converts its batch the same way.
    pub fn from_section(s: &FrequentSection) -> Self {
        Self {
            polyline: s.polyline.clone(),
            visit_count: s.visit_count,
        }
    }
}

/// What happens to one candidate: it either inherits a prior id or mints a new
/// one. The three inheriting variants record WHY the id was carried, so the
/// caller and the tests can see a plain carry apart from a split winner or a
/// merge target without re-deriving the graph shape.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Decision {
    /// 1:1 carry: exactly one prior and one candidate matched, mutually best.
    Carry { id: String },
    /// This candidate is the winning piece of a prior that split into several.
    SplitInherit { id: String },
    /// This candidate is the ground several priors merged onto; it inherits the
    /// senior prior's id.
    MergeInherit { id: String },
    /// New ground: the caller mints a fresh id. `split_from` names the prior
    /// this piece was carved from when the candidate shares a prior's corridor
    /// but lost the carry to a sibling (a split loser), so the caller can
    /// record lineage; `None` for genuinely new ground. The link is a pure
    /// function of the two ground sets, emitted idempotently on every call.
    Mint { split_from: Option<String> },
}

impl Decision {
    /// The inherited id, if this candidate carried one.
    pub fn carried_id(&self) -> Option<&str> {
        match self {
            Decision::Carry { id }
            | Decision::SplitInherit { id }
            | Decision::MergeInherit { id } => Some(id),
            Decision::Mint { .. } => None,
        }
    }

    /// The prior this piece was carved from, if it is a split loser.
    pub fn split_from(&self) -> Option<&str> {
        match self {
            Decision::Mint {
                split_from: Some(id),
            } => Some(id),
            _ => None,
        }
    }
}

/// Why a prior id was retired: it either matched nothing (a dissolve) or lost a
/// merge to a senior prior (retained so a later re-split can restore it).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RetireReason {
    Dissolved,
    MergedInto { id: String },
}

/// A prior id no candidate carried this plan.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Retirement {
    pub id: String,
    pub reason: RetireReason,
}

/// The resolution of one detect: a decision per candidate (parallel to the
/// `next` slice) plus the priors that dropped out. A pure function of the two
/// ground sets and their metadata, so equal inputs give an equal plan.
#[derive(Clone, Debug, PartialEq)]
pub struct IdentityPlan {
    pub decisions: Vec<Decision>,
    pub retired: Vec<Retirement>,
}

// ============================================================================
// plan_identity, the bipartite carry/mint/split/merge/dissolve resolution
// ============================================================================

/// Tunables for [`plan_identity_tuned`]. The default reproduces the shipped
/// behaviour exactly; a non-zero floor is an opt-in experiment, never the
/// default.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct IdentityParams {
    /// Minimum mutual overlap a prior needs to COMPETE for a candidate's merge
    /// nomination. At the default 0.0 every same-corridor prior competes, so
    /// seniority alone decides a merge, and a marginal one-sided edge (a short
    /// senior mostly inside a long candidate) can out-rank a dominant junior.
    /// Above 0.0 a prior below the floor cannot capture a candidate's identity
    /// by seniority, so the dominant junior wins it instead. See the
    /// marginal-capture stress scenario (`tests/b2_inheritance_stress.rs`).
    pub merge_mutual_floor: f64,
}

impl Default for IdentityParams {
    fn default() -> Self {
        Self {
            merge_mutual_floor: 0.0,
        }
    }
}

/// Match a fresh candidate catalogue against the prior ids with the default
/// params. See [`plan_identity_tuned`] for the resolution rule.
pub fn plan_identity(prior: &[PriorSection], next: &[CandidateSection]) -> IdentityPlan {
    plan_identity_tuned(prior, next, &IdentityParams::default())
}

/// Match a fresh candidate catalogue against the prior ids and decide which
/// candidate inherits which id.
///
/// The rule is a mutual-best pairing on the same-corridor graph. Each candidate
/// nominates a prior, an extent-agreeing 1:1 match first, else the SENIOR
/// contained or same-corridor prior (the merge rule), among the priors that
/// clear `params.merge_mutual_floor`; each prior nominates the candidate it
/// overlaps MOST (the split rule). A carry is confirmed only where the two
/// nominations agree, which yields every case the design names with the
/// correct side-specific tie-break and no HashMap-order leak. Pairing a
/// catalogue against itself is the identity map (all carries):
/// - 1:1 -> both nominate each other -> [`Decision::Carry`].
/// - split (one prior, several candidates) -> the prior nominates its best piece,
///   which alone confirms ([`Decision::SplitInherit`]); the rest mint.
/// - merge (several priors, one candidate) -> the candidate nominates the senior,
///   which alone confirms ([`Decision::MergeInherit`]); the juniors retire
///   `MergedInto` the winner.
/// - new candidate -> [`Decision::Mint`]; gone prior -> [`Retirement`]`::Dissolved`.
///
/// All tie-breaks are total, so `plan_identity_tuned(p, n, q)` is byte-identical
/// across runs and permutation-stable.
pub fn plan_identity_tuned(
    prior: &[PriorSection],
    next: &[CandidateSection],
    params: &IdentityParams,
) -> IdentityPlan {
    plan_identity_memo(prior, next, params, &mut HashMap::new())
}

/// Deterministic digest of a line's exact coordinates: the key a pairwise
/// coverage memo carries between plans.
fn line_digest(line: &[GpsPoint]) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    line.len().hash(&mut h);
    for p in line {
        p.latitude.to_bits().hash(&mut h);
        p.longitude.to_bits().hash(&mut h);
    }
    h.finish()
}

/// [`plan_identity_tuned`] with a memo of pairwise ground coverage keyed
/// by line digests, `(prior, candidate) -> (prior over candidate,
/// candidate over prior)`. Coverage is a pure function of the two lines,
/// so a fold that leaves most lines as they were pays only for the pairs
/// it changed.
pub fn plan_identity_memo(
    prior: &[PriorSection],
    next: &[CandidateSection],
    params: &IdentityParams,
    memo: &mut HashMap<(u64, u64), (f64, f64)>,
) -> IdentityPlan {
    let np = prior.len();
    let nc = next.len();

    // Same-corridor edges and their mutual overlap, computed once. The padded
    // bounding boxes gate the O(polyline²) coverage test: two grounds can only
    // share a corridor within GROUND_TOL_M if their padded boxes overlap, so a
    // geographically distant pair costs O(1) instead of a full scan. Behaviour
    // is unchanged, a distant pair scored 0 coverage before, but the plan's
    // cost now tracks the co-located ground, not the whole catalogue.
    let p_bb: Vec<(f64, f64, f64, f64)> = prior
        .iter()
        .map(|p| ground_bbox_padded(&p.polyline))
        .collect();
    let c_bb: Vec<(f64, f64, f64, f64)> = next
        .iter()
        .map(|c| ground_bbox_padded(&c.polyline))
        .collect();
    let p_digest: Vec<u64> = prior.iter().map(|p| line_digest(&p.polyline)).collect();
    let c_digest: Vec<u64> = next.iter().map(|c| line_digest(&c.polyline)).collect();
    let mut edge = vec![vec![false; nc]; np];
    let mut mo = vec![vec![0.0_f64; nc]; np];
    let mut contained = vec![vec![false; nc]; np];
    for (i, p) in prior.iter().enumerate() {
        for (j, c) in next.iter().enumerate() {
            // An empty ground has an inverted padded box, so it would never
            // pass the gate; pair two empties directly (vacuous coverage,
            // see `coverage`) so the degenerate candidate carries its holder.
            if p.polyline.is_empty() || c.polyline.is_empty() {
                if p.polyline.is_empty() && c.polyline.is_empty() {
                    edge[i][j] = true;
                    mo[i][j] = 1.0;
                    contained[i][j] = true;
                }
                continue;
            }
            let (a, b) = (p_bb[i], c_bb[j]);
            if a.0 > b.1 || b.0 > a.1 || a.2 > b.3 || b.2 > a.3 {
                continue;
            }
            // The same line covers itself completely; most candidates are
            // their own prior, so this is the common pair.
            if p.polyline == c.polyline {
                edge[i][j] = true;
                mo[i][j] = 1.0;
                contained[i][j] = true;
                continue;
            }
            let (cov_pc, cov_cp) = *memo.entry((p_digest[i], c_digest[j])).or_insert_with(|| {
                (
                    coverage(&p.polyline, &c.polyline, GROUND_TOL_M),
                    coverage(&c.polyline, &p.polyline, GROUND_TOL_M),
                )
            });
            edge[i][j] = cov_pc.max(cov_cp) >= CARRY_COVERAGE;
            mo[i][j] = cov_pc.min(cov_cp);
            contained[i][j] = cov_pc >= CARRY_COVERAGE;
        }
    }

    // Each candidate nominates a prior in three tiers. First a prior whose
    // extent AGREES with the candidate (mutual overlap at or above
    // RECUT_AGREEMENT): a 1:1 match, ranked by overlap then seniority. This
    // tier makes the pairing idempotent, a catalogue paired against itself
    // is all carries, where containment-plus-seniority let a short senior
    // prior inside a long candidate's corridor out-nominate the candidate's
    // own exact match, minting a duplicate on every converged detect. Then
    // the senior among priors CONTAINED in the candidate's corridor (the
    // genuine merge shape, a retiring prior lies inside its successor).
    // Then the senior among all same-corridor priors (the split shape, a
    // piece nominates the prior it came from). Containment before the rest
    // stops a mere one-way overlap from a senior neighbour out-nominating
    // the candidate's mutual-best prior (marginal capture). Seniority is
    // earliest first_seen, then more visits, then more metres, then smaller id.
    let cand_pick: Vec<Option<usize>> = (0..nc)
        .map(|j| {
            let tier_of = |i: usize| -> u8 {
                if mo[i][j] >= RECUT_AGREEMENT {
                    2
                } else if contained[i][j] {
                    1
                } else {
                    0
                }
            };
            let mut best: Option<usize> = None;
            for i in 0..np {
                if !(edge[i][j] && mo[i][j] >= params.merge_mutual_floor) {
                    continue;
                }
                let wins = match best {
                    None => true,
                    Some(b) => match tier_of(i).cmp(&tier_of(b)) {
                        std::cmp::Ordering::Greater => true,
                        std::cmp::Ordering::Less => false,
                        std::cmp::Ordering::Equal => {
                            if tier_of(i) == 2 && mo[i][j] != mo[b][j] {
                                mo[i][j] > mo[b][j]
                            } else {
                                more_senior(&prior[i], &prior[b])
                            }
                        }
                    },
                };
                if wins {
                    best = Some(i);
                }
            }
            best
        })
        .collect();

    // Each prior nominates the candidate it overlaps most. The winner takes more
    // mutual overlap, then more metres, then more visits, then a smaller SW
    // corner, then a smaller index.
    let prior_pick: Vec<Option<usize>> = (0..np)
        .map(|i| {
            let mut best: Option<usize> = None;
            for j in 0..nc {
                if edge[i][j]
                    && (best.is_none()
                        || better_candidate(
                            mo[i][j],
                            &next[j],
                            mo[i][best.unwrap()],
                            &next[best.unwrap()],
                            j,
                            best.unwrap(),
                        ))
                {
                    best = Some(j);
                }
            }
            best
        })
        .collect();

    // A carry is confirmed where the two nominations agree. Record, per
    // candidate, the prior that carried it (for retirement reasons below).
    let mut carrier_of: Vec<Option<usize>> = vec![None; nc];
    for (i, &pick) in prior_pick.iter().enumerate() {
        if let Some(j) = pick
            && cand_pick[j] == Some(i)
        {
            carrier_of[j] = Some(i);
        }
    }

    // Candidate decisions. Label a confirmed carry by the graph shape around it:
    // a candidate several priors reach is a merge target; a prior reaching
    // several candidates is a split; otherwise a plain 1:1 carry.
    let decisions: Vec<Decision> = (0..nc)
        .map(|j| match carrier_of[j] {
            Some(i) => {
                let merge_degree = (0..np).filter(|&x| edge[x][j]).count();
                let split_degree = (0..nc).filter(|&y| edge[i][y]).count();
                let id = prior[i].id.clone();
                if merge_degree > 1 {
                    Decision::MergeInherit { id }
                } else if split_degree > 1 {
                    Decision::SplitInherit { id }
                } else {
                    Decision::Carry { id }
                }
            }
            None => Decision::Mint {
                // A minted candidate that still nominated a same-corridor prior
                // is a split loser: it shares ground with the prior but lost the
                // carry to a sibling. Record that prior as the parent so lineage
                // survives ("split into X and Y"). A candidate that nominated no
                // prior is genuinely new ground.
                split_from: cand_pick[j].map(|i| prior[i].id.clone()),
            },
        })
        .collect();

    // Retirements: any prior that carried no candidate. It merged into the
    // winner of its best-overlap candidate when that candidate was carried by
    // someone else, otherwise its ground is simply gone.
    let carried_prior: Vec<bool> = (0..np)
        .map(|i| (0..nc).any(|j| carrier_of[j] == Some(i)))
        .collect();
    let mut retired = Vec::new();
    for i in 0..np {
        if carried_prior[i] {
            continue;
        }
        let reason = match prior_pick[i] {
            Some(j) => match carrier_of[j] {
                Some(w) if w != i => RetireReason::MergedInto {
                    id: prior[w].id.clone(),
                },
                _ => RetireReason::Dissolved,
            },
            None => RetireReason::Dissolved,
        };
        retired.push(Retirement {
            id: prior[i].id.clone(),
            reason,
        });
    }

    IdentityPlan { decisions, retired }
}

/// `a` is more senior than `b`: earlier first_seen, then more visits, then more
/// metres, then a smaller id. Total, so a merge always picks the same winner.
fn more_senior(a: &PriorSection, b: &PriorSection) -> bool {
    match a.first_seen.cmp(&b.first_seen) {
        std::cmp::Ordering::Less => true,
        std::cmp::Ordering::Greater => false,
        std::cmp::Ordering::Equal => match a.visit_count.cmp(&b.visit_count) {
            std::cmp::Ordering::Greater => true,
            std::cmp::Ordering::Less => false,
            std::cmp::Ordering::Equal => {
                match polyline_metres(&a.polyline).total_cmp(&polyline_metres(&b.polyline)) {
                    std::cmp::Ordering::Greater => true,
                    std::cmp::Ordering::Less => false,
                    std::cmp::Ordering::Equal => a.id < b.id,
                }
            }
        },
    }
}

/// Candidate `(mo_a, a)` at index `ja` beats `(mo_b, b)` at index `jb` for a
/// prior's split nomination: more mutual overlap, then more metres, then more
/// visits, then a smaller SW corner, then a smaller index.
fn better_candidate(
    mo_a: f64,
    a: &CandidateSection,
    mo_b: f64,
    b: &CandidateSection,
    ja: usize,
    jb: usize,
) -> bool {
    match mo_a.total_cmp(&mo_b) {
        std::cmp::Ordering::Greater => true,
        std::cmp::Ordering::Less => false,
        std::cmp::Ordering::Equal => {
            match polyline_metres(&a.polyline).total_cmp(&polyline_metres(&b.polyline)) {
                std::cmp::Ordering::Greater => true,
                std::cmp::Ordering::Less => false,
                std::cmp::Ordering::Equal => match a.visit_count.cmp(&b.visit_count) {
                    std::cmp::Ordering::Greater => true,
                    std::cmp::Ordering::Less => false,
                    std::cmp::Ordering::Equal => {
                        let (sa, sb) = (sw_corner(&a.polyline), sw_corner(&b.polyline));
                        match sa.0.total_cmp(&sb.0).then(sa.1.total_cmp(&sb.1)) {
                            std::cmp::Ordering::Less => true,
                            std::cmp::Ordering::Greater => false,
                            std::cmp::Ordering::Equal => ja < jb,
                        }
                    }
                },
            }
        }
    }
}

// ============================================================================
// Hysteresis, the low-pass filter over the plan
// ============================================================================

/// Tunable thresholds for [`HysteresisState`]. Defaults are the design's
/// calibrated values; a `Tunables`-style struct rather than magic literals.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct HysteresisParams {
    /// Consecutive decisive plans before a dissolve or re-cut applies. An armed
    /// step ([`HysteresisState::arm_decisive`]) runs at 1 instead.
    pub k: u8,
    /// A dissolve counts toward the debounce only at or above this pressure.
    pub dissolve_pressure_hi: f64,
    /// Extents overlapping by at least this adopt geometry immediately; below it
    /// a carried section is a material re-cut and debounces.
    pub recut_agreement: f64,
    /// Passed through to [`plan_identity_tuned`]'s [`IdentityParams`]. Default
    /// 0.0 keeps the shipped behaviour; a non-zero floor is the opt-in
    /// marginal-capture fix.
    pub merge_mutual_floor: f64,
}

impl Default for HysteresisParams {
    fn default() -> Self {
        Self {
            k: DEFAULT_K,
            dissolve_pressure_hi: DISSOLVE_PRESSURE_HI,
            recut_agreement: RECUT_AGREEMENT,
            merge_mutual_floor: 0.0,
        }
    }
}

/// One held section in the visible view.
#[derive(Clone, Debug, Serialize, Deserialize)]
struct HeldSection {
    polyline: Vec<GpsPoint>,
    visit_count: u32,
    first_seen: u64,
}

/// The debounce ledger of one visible id: both directions' streaks held
/// together, so neither kind of step erases the other's evidence. A frozen
/// carry advances the re-cut streak and preserves the dissolve streak; a
/// retiring plan advances the dissolve streak and preserves the re-cut
/// streak; only a decisive continuation (an agreement carry, or a fired
/// change) clears the ledger. The single-`kind` predecessor reset the other
/// streak on every flip, so a marginal capture rotating with absence pinned a
/// dead id forever (the D2 corpus replay's stale visible-only sections) and
/// alternating plans never converged.
#[derive(Clone, Debug, Serialize, Deserialize)]
struct Pending {
    /// Retiring plans since the last decisive continuation.
    dissolve_streak: u8,
    /// Frozen carries since the last decisive continuation.
    recut_streak: u8,
    /// The batch geometry a re-cut will snap to at `k`: refreshed on every
    /// frozen carry, kept through retiring steps (the id keeps competing on
    /// the geometry it is re-cutting to). `None` while only absence has
    /// accumulated.
    target: Option<CandidateSection>,
}

/// What a held section should become after one step. Returned by the appliers
/// so `step` performs every mutation (including moving a dissolved ground into
/// the tombstones) in one place, after the read-only pass over the old view.
enum HeldAction {
    /// Stays visible with this geometry, no debounce.
    Keep(HeldSection),
    /// Stays visible with an active debounce toward a dissolve or re-cut.
    KeepPending(HeldSection, Pending),
    /// A sustained dissolve fired: retain the ground as a tombstone.
    Tombstone(HeldSection),
}

/// What one [`HysteresisState::step`] did, for measurement and assertions.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct StepOutcome {
    /// Sections in the batch this step.
    pub raw_count: usize,
    /// Sections in the visible view after the step.
    pub visible_count: usize,
    /// Fresh ids assigned to genuinely new ground.
    pub minted: usize,
    /// Ids restored from a tombstone because their ground re-emerged.
    pub restored: usize,
    /// Sustained dissolves that fired this step (moved to a tombstone).
    pub dissolved: usize,
    /// Sustained re-cuts that fired this step (geometry snapped to the batch).
    pub recut_applied: usize,
    /// Ids carrying an active debounce after the step.
    pub debouncing: usize,
    /// The fired retirements, each with the reason the plan gave at fire time
    /// (gone ground, or merged into a named survivor). The emitter's per-id
    /// record; `dissolved` is its length. Mints and restores need no list -
    /// the parallel [`CandidateResolution`]s already name them.
    pub retired: Vec<Retirement>,
    /// The ids whose sustained re-cut fired this step, in visible-id order.
    /// `recut_applied` is its length; the adopted geometry is the resolution
    /// carrying each id.
    pub recut_ids: Vec<String>,
}

/// How one candidate entered the visible view this step. The mirroring
/// contract a stateful caller (the veloqrs registry) builds on: for every
/// fate except [`CarriedFrozen`](CandidateFate::CarriedFrozen), the visible
/// ground held under the candidate's id IS the candidate's polyline, so the
/// caller's payload must follow it. A frozen carry keeps the prior geometry
/// until the re-cut debounce fires.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CandidateFate {
    /// Carried a prior and adopted the batch geometry: the extents agreed,
    /// or a sustained re-cut fired this step.
    CarriedAdopted,
    /// Carried a prior mid re-cut debounce; the held geometry stays frozen.
    CarriedFrozen,
    /// Fresh ground under a newly minted id.
    Minted,
    /// A tombstoned ground re-emerged under its old id.
    Restored,
}

/// One candidate's visible stable id and how it got it, parallel to the
/// `next` slice passed to [`HysteresisState::step_assign`].
#[derive(Clone, Debug)]
pub struct CandidateResolution {
    pub id: String,
    pub fate: CandidateFate,
    /// The visible id of the prior this candidate was carved from, set only on
    /// a freshly minted split loser (fate [`Minted`](CandidateFate::Minted)).
    /// A restored or carried candidate, and genuinely new ground, leave it
    /// `None`. Lets the caller store lineage without re-deriving the graph.
    pub split_from: Option<String>,
}

/// The assign-once identity registry plus its hysteresis debounce. Holds the
/// visible catalogue (stable id -> held ground), the tombstones a dissolved
/// ground can re-emerge under, and the per-id debounce counters. In-memory for
/// the lab; the engine persists the same shape.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HysteresisState {
    params: HysteresisParams,
    visible: BTreeMap<String, HeldSection>,
    tombstones: BTreeMap<String, HeldSection>,
    pending: BTreeMap<String, Pending>,
    /// Monotonic counter minting first_seen ordinals and `s_<n>` ids. Only ever
    /// grows, so a fresh id never collides with a live or tombstoned one.
    ordinal: u64,
    /// Set by [`arm_decisive`](Self::arm_decisive) and consumed by the next
    /// step, which then runs at `k = 1`. Part of the state, so the fold stays a
    /// pure function of it. Trailing and defaulted, so an older blob decodes.
    #[serde(default)]
    decisive: bool,
}

impl Default for HysteresisState {
    fn default() -> Self {
        Self::new(HysteresisParams::default())
    }
}

impl HysteresisState {
    pub fn new(params: HysteresisParams) -> Self {
        Self {
            params,
            visible: BTreeMap::new(),
            tombstones: BTreeMap::new(),
            pending: BTreeMap::new(),
            ordinal: 0,
            decisive: false,
        }
    }

    /// Arm the next step to apply dissolves and re-cuts on the first decisive
    /// plan instead of accumulating `k`.
    ///
    /// The debounce exists to absorb detector noise: a single add must not flip
    /// the visible view. A caller-side event that changes what the detector
    /// looks for is not that noise, so the batch it produces is trusted at once
    /// and the view tracks it in one step. Additions are immediate either way;
    /// this is only about how fast a departure or a re-cut lands.
    pub fn arm_decisive(&mut self) {
        self.decisive = true;
    }

    /// Whether the next step runs decisively.
    pub fn is_decisive(&self) -> bool {
        self.decisive
    }

    /// Fold a fresh batch catalogue into the visible view, damping churn.
    ///
    /// Additions are immediate: a candidate matching no held section appears at
    /// once, restoring its old id if its ground re-emerged from a tombstone,
    /// otherwise minting a fresh one. A carried section folds the batch's visits
    /// immediately and adopts geometry immediately when the extents agree; a
    /// material re-cut, and any dissolve, debounce over `k` decisive detects
    /// before they apply. A debounce, not a freeze: once the batch sustains the
    /// change for `k` steps it applies.
    pub fn step(&mut self, next: &[CandidateSection]) -> StepOutcome {
        self.step_assign(next).0
    }

    /// [`step`](Self::step) that also returns, parallel to `next`, the visible
    /// stable id each candidate resolved to and its [`CandidateFate`]. Every
    /// candidate maps to exactly one visible id (a carry/split/merge target
    /// inherits its prior's id, a mint or a tombstone-restore takes the
    /// fresh/restored id), so a stateful caller like the veloqrs registry can
    /// join its own per-id payload, real DB id, members, name, onto the plan
    /// without re-deriving the graph. The fate carries the geometry contract:
    /// unless it is `CarriedFrozen`, the visible ground under that id is the
    /// candidate's polyline, and the caller's payload must follow it. The pure
    /// layer's `s_<n>` id is the join key; the caller keeps the opaque
    /// `s_<ts>__<rand>` id it persists on the side.
    pub fn step_assign(
        &mut self,
        next: &[CandidateSection],
    ) -> (StepOutcome, Vec<CandidateResolution>) {
        // An armed step runs at k = 1: the first decisive plan applies.
        let k = if std::mem::take(&mut self.decisive) {
            1
        } else {
            self.params.k
        };
        // A section mid re-cut competes on the batch geometry it is re-cutting TO
        // (its pending target), not its stale frozen footprint. Presenting the
        // stale, larger footprint lets it capture a neighbouring candidate's
        // identity while the geometry is held, minting phantom duplicates of the
        // carved-off piece (the balanced-split cascade). It still matches its own
        // re-cut candidate, which equals this target.
        let prior: Vec<PriorSection> = self
            .visible
            .iter()
            .map(|(id, h)| {
                let polyline = match self.pending.get(id).and_then(|p| p.target.as_ref()) {
                    Some(t) => t.polyline.clone(),
                    None => h.polyline.clone(),
                };
                PriorSection {
                    id: id.clone(),
                    polyline,
                    first_seen: h.first_seen,
                    visit_count: h.visit_count,
                }
            })
            .collect();
        let plan = plan_identity_tuned(
            &prior,
            next,
            &IdentityParams {
                merge_mutual_floor: self.params.merge_mutual_floor,
            },
        );

        // Index the plan by prior id: which candidate carried it, and which
        // priors retired and why. `carried` doubles as the carry half of the
        // per-candidate id assignment.
        let mut carried: BTreeMap<&str, usize> = BTreeMap::new();
        for (j, d) in plan.decisions.iter().enumerate() {
            if let Some(id) = d.carried_id() {
                carried.insert(id, j);
            }
        }
        let retired: BTreeMap<&str, &RetireReason> = plan
            .retired
            .iter()
            .map(|r| (r.id.as_str(), &r.reason))
            .collect();

        // The id each candidate becomes visible under. A carried candidate takes
        // its inherited id; a mint/restore fills its slot in the loop below.
        let mut candidate_ids: Vec<String> = vec![String::new(); next.len()];
        for (&id, &j) in &carried {
            candidate_ids[j] = id.to_string();
        }
        let mut fates: Vec<Option<CandidateFate>> = vec![None; next.len()];
        // Parallel to `next`: the split parent a minted loser carried, surfaced
        // on its resolution. Only a fresh mint takes one; a restore comes back
        // as itself.
        let mut split_froms: Vec<Option<String>> = vec![None; next.len()];

        let mut new_visible: BTreeMap<String, HeldSection> = BTreeMap::new();
        let mut new_pending: BTreeMap<String, Pending> = BTreeMap::new();
        let mut out = StepOutcome {
            raw_count: next.len(),
            ..StepOutcome::default()
        };

        // Carried and retired held sections. Every prior is exactly one of the
        // two (plan_identity accounts for all of them). Take the old view out so
        // a fired dissolve can move its ground into the tombstones without an
        // outstanding borrow of `self`.
        let old_visible = std::mem::take(&mut self.visible);
        for (id, held) in old_visible {
            let carried_j = carried.get(id.as_str()).copied();
            let action = if let Some(j) = carried_j {
                self.apply_carry(&id, &held, &next[j], k, &mut out)
            } else if let Some(reason) = retired.get(id.as_str()) {
                self.apply_retire(&id, &held, reason, k, &mut out)
            } else {
                // Unreachable in a correct plan; hold defensively rather than
                // silently drop a section.
                HeldAction::Keep(held)
            };
            if let Some(j) = carried_j {
                // apply_carry only ever keeps: geometry adopted outright, or
                // frozen behind a re-cut debounce.
                fates[j] = Some(match &action {
                    HeldAction::Keep(_) => CandidateFate::CarriedAdopted,
                    HeldAction::KeepPending(..) => CandidateFate::CarriedFrozen,
                    HeldAction::Tombstone(_) => unreachable!("a carried prior is never tombstoned"),
                });
            }
            match action {
                HeldAction::Keep(h) => {
                    new_visible.insert(id, h);
                }
                HeldAction::KeepPending(h, p) => {
                    new_visible.insert(id.clone(), h);
                    new_pending.insert(id, p);
                }
                HeldAction::Tombstone(h) => {
                    self.tombstones.insert(id, h);
                }
            }
        }

        // Mints, immediate. A re-emerged ground restores its tombstoned id.
        for (j, d) in plan.decisions.iter().enumerate() {
            if !matches!(d, Decision::Mint { .. }) {
                continue;
            }
            let cand = &next[j];
            if let Some(tomb_id) = self.match_tombstone(&cand.polyline) {
                let mut held = self.tombstones.remove(&tomb_id).expect("matched tombstone");
                held.polyline = cand.polyline.clone();
                held.visit_count = cand.visit_count.max(held.visit_count);
                candidate_ids[j] = tomb_id.clone();
                fates[j] = Some(CandidateFate::Restored);
                new_visible.insert(tomb_id, held);
                out.restored += 1;
            } else {
                self.ordinal += 1;
                let id = format!("s_{:06}", self.ordinal);
                candidate_ids[j] = id.clone();
                fates[j] = Some(CandidateFate::Minted);
                split_froms[j] = d.split_from().map(str::to_string);
                new_visible.insert(
                    id,
                    HeldSection {
                        polyline: cand.polyline.clone(),
                        visit_count: cand.visit_count,
                        first_seen: self.ordinal,
                    },
                );
                out.minted += 1;
            }
        }

        out.visible_count = new_visible.len();
        out.debouncing = new_pending.len();
        self.visible = new_visible;
        self.pending = new_pending;
        let resolutions = candidate_ids
            .into_iter()
            .zip(fates)
            .zip(split_froms)
            .map(|((id, fate), split_from)| CandidateResolution {
                id,
                fate: fate.expect("every candidate resolves exactly once"),
                split_from,
            })
            .collect();
        (out, resolutions)
    }

    /// Drop an id from the visible view, its debounce, and any tombstone. The
    /// veloqrs registry calls this when a section's ground passes to a durable
    /// intent row (accept/trim/merge): identity ownership transfers to the DB
    /// row, so the registry must stop carrying, and stop debounce-dissolving -
    /// the ground it no longer owns. Idempotent: forgetting an absent id is a
    /// no-op.
    pub fn forget(&mut self, id: &str) {
        self.visible.remove(id);
        self.pending.remove(id);
        self.tombstones.remove(id);
    }

    /// Decide a carried section: fold visits immediately; adopt geometry now
    /// when the extents agree, otherwise debounce the re-cut and keep the old
    /// geometry until it sustains `k`.
    fn apply_carry(
        &self,
        id: &str,
        held: &HeldSection,
        cand: &CandidateSection,
        k: u8,
        out: &mut StepOutcome,
    ) -> HeldAction {
        let visits = cand.visit_count.max(held.visit_count);
        let adopted = HeldSection {
            polyline: cand.polyline.clone(),
            visit_count: visits,
            first_seen: held.first_seen,
        };
        if mutual_overlap(&held.polyline, &cand.polyline) >= self.params.recut_agreement {
            // Extents agree: adopt the batch geometry, clear any debounce.
            return HeldAction::Keep(adopted);
        }
        // Material re-cut: debounce, folding visits but freezing geometry. The
        // dissolve streak rides along untouched, so a marginal capture cannot
        // erase the absence evidence a rotation accumulated.
        let ledger = self.pending.get(id);
        let streak = ledger.map_or(0, |p| p.recut_streak).saturating_add(1);
        if streak >= k {
            out.recut_applied += 1;
            out.recut_ids.push(id.to_string());
            HeldAction::Keep(adopted)
        } else {
            let frozen = HeldSection {
                polyline: held.polyline.clone(),
                visit_count: visits,
                first_seen: held.first_seen,
            };
            HeldAction::KeepPending(
                frozen,
                Pending {
                    dissolve_streak: ledger.map_or(0, |p| p.dissolve_streak),
                    recut_streak: streak,
                    target: Some(cand.clone()),
                },
            )
        }
    }

    /// Decide a retired section: debounce the plan's retirement by COUNT and
    /// tombstone it once the plan has retired it for `k` detects uninterrupted
    /// by a decisive continuation, whether or not its ground stays covered.
    /// The fired retirement is reported per-id with the reason this step's
    /// plan gave (the fire-time reason: what was true when the change became
    /// visible, not when the streak began).
    ///
    /// Both a merge-away and a plain dissolve debounce identically here. The old
    /// gate held a dissolve until [`dissolve_pressure`] cleared a threshold, but
    /// pressure stays ~0 for a section the batch replaced whose ground a
    /// neighbour still covers, so such sections never retired and the visible
    /// catalogue inflated (held > raw, overlapping duplicates). Trusting
    /// plan_identity's retirement persistence drains them. The pressure metric
    /// stays for ground that genuinely vanished, which the plan also retires and
    /// so retires on the same count. A non-debounced pressure fast-path is
    /// deliberately NOT used: it would let a single detect flip a visible
    /// section, breaking single-add stability.
    fn apply_retire(
        &self,
        id: &str,
        held: &HeldSection,
        reason: &RetireReason,
        k: u8,
        out: &mut StepOutcome,
    ) -> HeldAction {
        let ledger = self.pending.get(id);
        let streak = ledger.map_or(0, |p| p.dissolve_streak).saturating_add(1);
        if streak >= k {
            out.dissolved += 1;
            out.retired.push(Retirement {
                id: id.to_string(),
                reason: reason.clone(),
            });
            HeldAction::Tombstone(held.clone())
        } else {
            HeldAction::KeepPending(
                held.clone(),
                Pending {
                    dissolve_streak: streak,
                    recut_streak: ledger.map_or(0, |p| p.recut_streak),
                    target: ledger.and_then(|p| p.target.clone()),
                },
            )
        }
    }

    /// The tombstoned id whose ground the given candidate re-forms, if any. A
    /// restore needs MUTUAL coverage: the one-way tolerance `shares_ground`
    /// grants extent growth would also resurrect a dead id onto a spur that is
    /// mostly foreign ground (the capture-rotation pinning's second half). The
    /// best mutual overlap wins; BTreeMap iteration keeps it deterministic.
    fn match_tombstone(&self, polyline: &[GpsPoint]) -> Option<String> {
        let mut best: Option<(String, f64)> = None;
        for (id, h) in &self.tombstones {
            let mo = mutual_overlap(polyline, &h.polyline);
            if mo >= CARRY_COVERAGE && best.as_ref().is_none_or(|(_, bmo)| mo > *bmo) {
                best = Some((id.clone(), mo));
            }
        }
        best.map(|(id, _)| id)
    }

    // -- read accessors for tests and the measurement -----------------------

    /// The visible section count.
    pub fn visible_len(&self) -> usize {
        self.visible.len()
    }

    /// Visible ids, sorted.
    pub fn visible_ids(&self) -> Vec<String> {
        self.visible.keys().cloned().collect()
    }

    /// Visible ids carrying an active debounce after the last step, sorted.
    /// A caller accumulating what arrived while a change was pending reads
    /// this after every step; an id that leaves the list has either fired
    /// or continued decisively, and its accumulation is over.
    pub fn pending_ids(&self) -> Vec<String> {
        self.pending.keys().cloned().collect()
    }

    /// The visible catalogue as `(id, ground)` pairs, sorted by id.
    pub fn visible_grounds(&self) -> Vec<(String, Vec<GpsPoint>)> {
        self.visible
            .iter()
            .map(|(id, h)| (id.clone(), h.polyline.clone()))
            .collect()
    }

    /// The ground currently held under `id`, if visible.
    pub fn ground_of(&self, id: &str) -> Option<&[GpsPoint]> {
        self.visible.get(id).map(|h| h.polyline.as_slice())
    }

    /// Whether `id` is currently tombstoned (dissolved, retained for re-emergence).
    pub fn is_tombstoned(&self, id: &str) -> bool {
        self.tombstones.contains_key(id)
    }

    /// The ground retained under a tombstoned `id`, if any. A stateful caller
    /// sweeps this when ownership of dead ground passes elsewhere (a durable
    /// user claim), so the tombstone does not outlive the ground it guards.
    pub fn tombstone_ground_of(&self, id: &str) -> Option<&[GpsPoint]> {
        self.tombstones.get(id).map(|h| h.polyline.as_slice())
    }

    /// Tombstoned ids, sorted.
    pub fn tombstone_ids(&self) -> Vec<String> {
        self.tombstones.keys().cloned().collect()
    }

    /// Ids with an active debounce.
    pub fn pending_len(&self) -> usize {
        self.pending.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // A straight north-heading line of `n` points spaced ~11 m, starting at
    // `(lat0, lng0)`. Distinct, well-separated grounds come from distinct lng.
    fn line(lat0: f64, lng0: f64, n: usize) -> Vec<GpsPoint> {
        (0..n)
            .map(|i| GpsPoint::new(lat0 + i as f64 * 1.0e-4, lng0))
            .collect()
    }

    /// The plain double scan the grid-accelerated [`coverage`] must equal
    /// exactly: same haversine predicate, no pruning.
    fn coverage_naive(samples: &[GpsPoint], line: &[GpsPoint], tol_m: f64) -> f64 {
        if samples.is_empty() && line.is_empty() {
            return 1.0;
        }
        if samples.is_empty() || line.is_empty() {
            return 0.0;
        }
        let covered = samples
            .iter()
            .filter(|s| line.iter().any(|p| haversine_distance(s, p) <= tol_m))
            .count();
        covered as f64 / samples.len() as f64
    }

    #[test]
    fn grid_coverage_equals_the_plain_scan() {
        // Deterministic scatter of shapes around the tolerance boundary:
        // wobbled parallels at offsets straddling GROUND_TOL_M, a crossing,
        // and a far line. Every pair and direction must agree exactly with
        // the unpruned scan.
        let base = line(46.0, 7.0, 400);
        let mut shapes: Vec<Vec<GpsPoint>> = vec![base.clone()];
        for (k, off_m) in [8.0, 45.0, 49.9, 50.1, 62.0, 300.0].iter().enumerate() {
            let wob: Vec<GpsPoint> = base
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    let wobble = ((i as f64 * 0.7) + k as f64).sin() * 6.0;
                    GpsPoint::new(p.latitude, p.longitude + (off_m + wobble) / 77_000.0)
                })
                .collect();
            shapes.push(wob);
        }
        shapes.push(line(46.017, 6.999, 120));
        for a in &shapes {
            for b in &shapes {
                for tol in [25.0, GROUND_TOL_M, 80.0] {
                    assert_eq!(
                        coverage(a, b, tol),
                        coverage_naive(a, b, tol),
                        "grid and plain coverage diverged at tol {tol}"
                    );
                }
            }
        }
        assert_eq!(coverage(&[], &[], 50.0), coverage_naive(&[], &[], 50.0));
        assert_eq!(coverage(&base, &[], 50.0), coverage_naive(&base, &[], 50.0));
    }

    fn cand(polyline: Vec<GpsPoint>, visit_count: u32) -> CandidateSection {
        CandidateSection {
            polyline,
            visit_count,
        }
    }

    fn prior(id: &str, polyline: Vec<GpsPoint>, first_seen: u64, visit_count: u32) -> PriorSection {
        PriorSection {
            id: id.to_string(),
            polyline,
            first_seen,
            visit_count,
        }
    }

    // -- plan_identity ------------------------------------------------------

    #[test]
    fn carry_on_a_stable_catalogue() {
        let p = line(46.0, 7.0, 100);
        let prior = vec![prior("s_1", p.clone(), 1, 5)];
        let next = vec![cand(p, 6)];
        let plan = plan_identity(&prior, &next);
        assert_eq!(plan.decisions, vec![Decision::Carry { id: "s_1".into() }]);
        assert!(plan.retired.is_empty());
    }

    #[test]
    fn mint_on_new_ground() {
        // A prior far to the west; a candidate far to the east. No shared ground.
        let prior = vec![prior("s_1", line(46.0, 7.0, 100), 1, 5)];
        let next = vec![cand(line(46.0, 9.0, 100), 3)];
        let plan = plan_identity(&prior, &next);
        // Disjoint ground: a plain mint with no split parent.
        assert_eq!(plan.decisions, vec![Decision::Mint { split_from: None }]);
        assert_eq!(
            plan.retired,
            vec![Retirement {
                id: "s_1".into(),
                reason: RetireReason::Dissolved,
            }]
        );
    }

    #[test]
    fn split_inheritance_picks_the_larger_overlap() {
        // Prior P covers a 100-point line. C1 is its first 70%, C2 its first 40%.
        // Both are covered by P (they share ground), but C1 overlaps P more, so
        // C1 inherits and C2 mints.
        let p = line(46.0, 7.0, 100);
        let c1 = line(46.0, 7.0, 70);
        let c2 = line(46.0, 7.0, 40);
        let prior = vec![prior("s_1", p, 1, 9)];
        let next = vec![cand(c1, 4), cand(c2, 4)];
        let plan = plan_identity(&prior, &next);
        // C2 shares s_1's corridor but lost the carry to C1, so it mints with
        // s_1 recorded as its split parent.
        assert_eq!(
            plan.decisions,
            vec![
                Decision::SplitInherit { id: "s_1".into() },
                Decision::Mint {
                    split_from: Some("s_1".into())
                }
            ]
        );
        assert!(plan.retired.is_empty());
    }

    #[test]
    fn split_tie_break_is_total_and_order_free() {
        // Left half and right half of P: equal mutual overlap, equal metres,
        // equal visits. The only separator is the SW corner (smaller longitude),
        // so the western piece must inherit regardless of input order.
        let p = line(46.0, 7.0, 100);
        let west: Vec<GpsPoint> = p[..50].to_vec();
        let east: Vec<GpsPoint> = p[50..].to_vec();
        let prior = vec![prior("s_1", p, 1, 9)];

        let forward = plan_identity(&prior, &[cand(west.clone(), 4), cand(east.clone(), 4)]);
        // west is index 0.
        assert_eq!(
            forward.decisions[0],
            Decision::SplitInherit { id: "s_1".into() }
        );
        assert_eq!(
            forward.decisions[1],
            Decision::Mint {
                split_from: Some("s_1".into())
            }
        );

        let reversed = plan_identity(&prior, &[cand(east, 4), cand(west, 4)]);
        // west is now index 1, and must still be the one that inherits.
        assert_eq!(
            reversed.decisions[1],
            Decision::SplitInherit { id: "s_1".into() }
        );
        assert_eq!(
            reversed.decisions[0],
            Decision::Mint {
                split_from: Some("s_1".into())
            }
        );
    }

    #[test]
    fn plan_is_deterministic_byte_for_byte() {
        let p = line(46.0, 7.0, 100);
        let prior = vec![
            prior("s_1", p[..50].to_vec(), 1, 3),
            prior("s_2", p[40..].to_vec(), 2, 3),
        ];
        let next = vec![cand(p.clone(), 6), cand(line(46.0, 9.0, 60), 2)];
        assert_eq!(plan_identity(&prior, &next), plan_identity(&prior, &next));
    }

    #[test]
    fn merge_inheritance_goes_to_the_senior() {
        // A (first_seen 1) and B (first_seen 2) each cover half of Z. Z inherits
        // the senior A; B retires MergedInto A.
        let z = line(46.0, 7.0, 100);
        let a = z[..50].to_vec();
        let b = z[50..].to_vec();
        let prior = vec![prior("s_A", a, 1, 5), prior("s_B", b, 2, 5)];
        let next = vec![cand(z, 10)];
        let plan = plan_identity(&prior, &next);
        assert_eq!(
            plan.decisions,
            vec![Decision::MergeInherit { id: "s_A".into() }]
        );
        assert_eq!(
            plan.retired,
            vec![Retirement {
                id: "s_B".into(),
                reason: RetireReason::MergedInto { id: "s_A".into() },
            }]
        );
    }

    #[test]
    fn self_pairing_is_the_identity_map() {
        // A short senior prior sits inside the long junior's corridor, both
        // ridden and both in the catalogue. Pairing the catalogue against
        // itself must carry every id and retire nothing: containment plus
        // seniority alone let the short prior out-nominate the long
        // candidate's own exact match, minting a duplicate on every
        // converged detect.
        let long = line(46.0, 7.0, 100);
        let short: Vec<GpsPoint> = long[20..40].to_vec();
        let away = line(46.0, 9.0, 60);
        let priors = vec![
            prior("s_short", short.clone(), 1, 5),
            prior("s_long", long.clone(), 2, 9),
            prior("s_away", away.clone(), 3, 2),
        ];
        let next = vec![cand(short, 5), cand(long, 9), cand(away, 2)];
        let plan = plan_identity(&priors, &next);
        for (j, id) in ["s_short", "s_long", "s_away"].iter().enumerate() {
            assert_eq!(
                plan.decisions[j].carried_id(),
                Some(*id),
                "candidate {j} must carry its own id"
            );
        }
        assert!(plan.retired.is_empty(), "a fixed point retires nothing");
    }

    #[test]
    fn merge_seniority_ignores_input_order() {
        // Same as above but the junior B is listed first; the senior A still wins.
        let z = line(46.0, 7.0, 100);
        let a = z[..50].to_vec();
        let b = z[50..].to_vec();
        let prior = vec![prior("s_B", b, 2, 5), prior("s_A", a, 1, 5)];
        let next = vec![cand(z, 10)];
        let plan = plan_identity(&prior, &next);
        assert_eq!(
            plan.decisions,
            vec![Decision::MergeInherit { id: "s_A".into() }]
        );
    }

    #[test]
    fn merge_floor_lets_the_dominant_junior_win_over_a_marginal_senior() {
        // A (senior) is a short section marginally inside the long candidate Z;
        // B (junior) dominantly covers Z without agreeing on its extent, so
        // neither prior reaches the 1:1 agreement tier. At the default 0.0
        // floor seniority wins, so A captures Z. Above A's marginal overlap
        // the dominant junior B wins instead, and A folds into B.
        let long = line(46.0, 7.0, 120);
        let short = long[..20].to_vec();
        let most = long[..90].to_vec();
        let prior = vec![prior("s_A", short, 1, 3), prior("s_B", most, 2, 9)];
        let next = vec![cand(long, 12)];

        let default_plan = plan_identity(&prior, &next);
        assert_eq!(
            default_plan.decisions,
            vec![Decision::MergeInherit { id: "s_A".into() }],
            "default: the marginal senior captures the corridor"
        );

        let floored = plan_identity_tuned(
            &prior,
            &next,
            &IdentityParams {
                merge_mutual_floor: 0.4,
            },
        );
        assert_eq!(
            floored.decisions,
            vec![Decision::MergeInherit { id: "s_B".into() }],
            "with the floor: the dominant junior keeps the corridor"
        );
        assert_eq!(
            floored.retired,
            vec![Retirement {
                id: "s_A".into(),
                reason: RetireReason::MergedInto { id: "s_B".into() },
            }]
        );
    }

    #[test]
    fn merge_floor_preserves_a_genuine_half_and_half_merge() {
        // Both priors cover half of Z (mutual 0.5 each), so a 0.4 floor leaves
        // them both competing and seniority still decides.
        let z = line(46.0, 7.0, 100);
        let a = z[..50].to_vec();
        let b = z[50..].to_vec();
        let prior = vec![prior("s_A", a, 1, 5), prior("s_B", b, 2, 5)];
        let next = vec![cand(z, 10)];
        let floored = plan_identity_tuned(
            &prior,
            &next,
            &IdentityParams {
                merge_mutual_floor: 0.4,
            },
        );
        assert_eq!(
            floored.decisions,
            vec![Decision::MergeInherit { id: "s_A".into() }]
        );
    }

    // -- hysteresis ---------------------------------------------------------

    #[test]
    fn hysteresis_carries_a_stable_catalogue() {
        let mut state = HysteresisState::default();
        let p = line(46.0, 7.0, 100);
        let out0 = state.step(&[cand(p.clone(), 5)]);
        assert_eq!(out0.minted, 1);
        let ids0 = state.visible_ids();
        let out1 = state.step(&[cand(p, 6)]);
        assert_eq!(out1.minted, 0);
        assert_eq!(out1.dissolved, 0);
        assert_eq!(
            state.visible_ids(),
            ids0,
            "a stable catalogue must keep its ids"
        );
    }

    #[test]
    fn hysteresis_mints_new_ground_immediately() {
        let mut state = HysteresisState::default();
        state.step(&[cand(line(46.0, 7.0, 100), 5)]);
        let out = state.step(&[cand(line(46.0, 7.0, 100), 5), cand(line(46.0, 9.0, 100), 3)]);
        assert_eq!(out.minted, 1, "genuinely new ground appears at once");
        assert_eq!(state.visible_len(), 2);
    }

    #[test]
    fn dissolve_requires_k_consecutive_detects() {
        let mut state = HysteresisState::default(); // k = 3
        let p = line(46.0, 7.0, 100);
        state.step(&[cand(p.clone(), 5)]);
        let id = state.visible_ids()[0].clone();

        // Two empty detects: the ground is decisively gone but the debounce
        // holds it visible.
        let s1 = state.step(&[]);
        assert_eq!(s1.dissolved, 0);
        assert!(state.visible_ids().contains(&id));
        let s2 = state.step(&[]);
        assert_eq!(s2.dissolved, 0);
        assert!(state.visible_ids().contains(&id));

        // The third sustained empty detect fires the dissolve.
        let s3 = state.step(&[]);
        assert_eq!(s3.dissolved, 1);
        assert_eq!(state.visible_len(), 0);
        assert!(
            state.is_tombstoned(&id),
            "a dissolved id is tombstoned, not forgotten"
        );
    }

    #[test]
    fn an_armed_step_dissolves_gone_ground_at_once() {
        let mut state = HysteresisState::default(); // k = 3
        let p = line(46.0, 7.0, 100);
        state.step(&[cand(p, 5)]);
        let id = state.visible_ids()[0].clone();

        state.arm_decisive();
        let out = state.step(&[]);
        assert_eq!(out.dissolved, 1, "an armed step needs no streak");
        assert_eq!(state.visible_len(), 0);
        assert!(state.is_tombstoned(&id));
    }

    #[test]
    fn an_armed_step_adopts_a_re_cut_at_once() {
        let mut state = HysteresisState::default();
        let held = line(46.0, 7.0, 100);
        state.step(&[cand(held, 5)]);
        let id = state.visible_ids()[0].clone();

        // Half the extent: a material re-cut, normally debounced.
        let recut = line(46.0, 7.0, 40);
        state.arm_decisive();
        let out = state.step(&[cand(recut.clone(), 6)]);
        assert_eq!(out.recut_applied, 1);
        assert_eq!(state.visible_ids(), vec![id.clone()], "the id carries");
        assert_eq!(state.ground_of(&id), Some(recut.as_slice()));
    }

    #[test]
    fn arming_lasts_one_step() {
        let mut state = HysteresisState::default();
        let a = line(46.0, 7.0, 100);
        let b = line(46.0, 9.0, 100);
        state.step(&[cand(a.clone(), 5), cand(b, 5)]);

        state.arm_decisive();
        let armed = state.step(&[cand(a.clone(), 5)]);
        assert_eq!(armed.dissolved, 1);
        assert!(!state.is_decisive(), "the arm is consumed by one step");

        // The surviving ground now goes: back to the full debounce.
        let after = state.step(&[]);
        assert_eq!(after.dissolved, 0, "k applies again on the next step");
        assert_eq!(state.visible_len(), 1);
    }

    #[test]
    fn flip_flop_damps_to_a_stable_view() {
        // A ground that dissolves and reforms every other detect must never
        // dissolve: each reappearance resets the debounce before it reaches k.
        let mut state = HysteresisState::default();
        let p = line(46.0, 7.0, 100);
        state.step(&[cand(p.clone(), 5)]);
        let id = state.visible_ids()[0].clone();
        for step in 0..12 {
            let batch = if step % 2 == 0 {
                vec![]
            } else {
                vec![cand(p.clone(), 5)]
            };
            let out = state.step(&batch);
            assert_eq!(
                out.dissolved, 0,
                "flip-flop must never dissolve (step {step})"
            );
            assert_eq!(
                state.visible_ids(),
                vec![id.clone()],
                "visible view must stay stable"
            );
        }
    }

    #[test]
    fn sustained_change_converges_and_reforms_under_the_old_id() {
        let mut state = HysteresisState::default();
        let p = line(46.0, 7.0, 100);
        state.step(&[cand(p.clone(), 5)]);
        let id = state.visible_ids()[0].clone();

        // Three sustained empty detects dissolve it.
        for _ in 0..3 {
            state.step(&[]);
        }
        assert_eq!(state.visible_len(), 0);
        assert!(state.is_tombstoned(&id));

        // The ground re-forms: it must come back under the SAME id, not a new one.
        let out = state.step(&[cand(p, 5)]);
        assert_eq!(out.restored, 1);
        assert_eq!(out.minted, 0);
        assert_eq!(state.visible_ids(), vec![id]);
    }

    #[test]
    fn a_single_add_never_removes_a_visible_section() {
        // The headline stability property: one added activity (one new corridor
        // plus a churny dissolve of an existing one) cannot drop a visible id.
        let mut state = HysteresisState::default();
        let a = line(46.0, 7.0, 100);
        let b = line(46.0, 9.0, 100);
        state.step(&[cand(a.clone(), 5), cand(b.clone(), 5)]);
        let before = state.visible_ids();
        // Batch now drops `a` and adds a third corridor `c`: a single churny step.
        let c = line(46.0, 11.0, 100);
        let out = state.step(&[cand(b, 6), cand(c, 3)]);
        assert_eq!(out.dissolved, 0, "no dissolve may fire on a single step");
        for id in &before {
            assert!(
                state.visible_ids().contains(id),
                "id {id} must survive a single add"
            );
        }
    }

    #[test]
    fn fates_mirror_ground_adoption() {
        let p = line(46.0, 7.0, 100);
        let mut state = HysteresisState::default();
        let (_, r) = state.step_assign(&[cand(p.clone(), 5)]);
        assert_eq!(r[0].fate, CandidateFate::Minted);
        let id = r[0].id.clone();

        // Agreement carry: 95 of 100 points overlap, above RECUT_AGREEMENT,
        // so the held ground adopts at once.
        let agree: Vec<GpsPoint> = p[..95].to_vec();
        let (_, r) = state.step_assign(&[cand(agree.clone(), 6)]);
        assert_eq!(r[0].fate, CandidateFate::CarriedAdopted);
        assert_eq!(state.ground_of(&id), Some(agree.as_slice()));

        // Material re-cut: frozen on the prior ground until the k-th
        // consecutive detect, which fires and adopts.
        let recut: Vec<GpsPoint> = p[..50].to_vec();
        for _ in 0..(DEFAULT_K - 1) {
            let (_, r) = state.step_assign(&[cand(recut.clone(), 7)]);
            assert_eq!(r[0].fate, CandidateFate::CarriedFrozen);
            assert_eq!(state.ground_of(&id), Some(agree.as_slice()));
        }
        let (out, r) = state.step_assign(&[cand(recut.clone(), 8)]);
        assert_eq!(r[0].fate, CandidateFate::CarriedAdopted);
        assert_eq!(out.recut_applied, 1);
        assert_eq!(state.ground_of(&id), Some(recut.as_slice()));

        // Dissolve over k detects, then the ground re-emerges as itself.
        for _ in 0..DEFAULT_K {
            let (_, r) = state.step_assign(&[]);
            assert!(r.is_empty());
        }
        assert!(state.is_tombstoned(&id));
        let (_, r) = state.step_assign(&[cand(recut.clone(), 8)]);
        assert_eq!(r[0].fate, CandidateFate::Restored);
        assert_eq!(r[0].id, id);
        assert_eq!(state.ground_of(&id), Some(recut.as_slice()));
    }

    /// Scenario: a section's ground stops being ridden, but a spur sharing
    /// 70% of the dead footprint one-way flickers past every second detect.
    /// Expected behaviour: the dead section still retires. The held footprint
    /// captures the spur (edge on one-way coverage, seniority at merge floor
    /// 0.0), but a frozen carry preserves the dissolve streak, so the absence
    /// evidence keeps accumulating through the flicker and the tombstone
    /// fires; the mostly-foreign spur then fails the mutual restore test and
    /// mints its own id instead of resurrecting the dead one.
    #[test]
    fn flickering_marginal_capture_must_not_pin_a_dead_section() {
        let ground = line(46.0, 7.0, 100);
        let mut state = HysteresisState::default();
        let (_, r) = state.step_assign(&[cand(ground.clone(), 8)]);
        let id = r[0].id.clone();

        let mut spur: Vec<GpsPoint> = ground[..70].to_vec();
        spur.extend(line(46.007, 7.02, 60));
        for _ in 0..4 {
            state.step_assign(&[]);
            state.step_assign(&[cand(spur.clone(), 3)]);
        }
        assert!(
            !state.visible_ids().contains(&id),
            "a ground gone for 8 detects must retire; a flickering marginal \
             capture must not reset the dissolve debounce forever"
        );
    }

    /// Scenario: a senior trunk and a junior neighbour whose ground one-way
    /// overlaps the trunk's footprint by more than CARRY_COVERAGE; both are
    /// ridden stably.
    /// Expected behaviour: each keeps its own id and the catalogue holds
    /// exactly two sections. Today the neighbour's candidate nominates the
    /// SENIOR on the one-sided edge (cand_pick is seniority-only at floor
    /// 0.0), the mutual-best confirmation fails for the junior, and the
    /// junior starves: it retires while its candidate re-mints fresh ids.
    #[test]
    fn a_neighbours_candidate_is_not_captured_by_a_senior_one_way_overlap() {
        let trunk = line(46.0, 7.0, 100);
        let mut neighbour: Vec<GpsPoint> = trunk[..50].to_vec();
        neighbour.extend(line(46.005, 7.012, 30));

        let mut state = HysteresisState::default();
        let (_, r) = state.step_assign(&[cand(trunk.clone(), 9)]);
        let trunk_id = r[0].id.clone();
        let (_, r) = state.step_assign(&[cand(trunk.clone(), 9), cand(neighbour.clone(), 4)]);
        assert_eq!(r[1].fate, CandidateFate::Minted, "neighbour ground mints");
        let neighbour_id = r[1].id.clone();

        for step in 0..5 {
            state.step_assign(&[cand(trunk.clone(), 9), cand(neighbour.clone(), 5)]);
            let visible = state.visible_ids();
            assert!(
                visible.contains(&trunk_id) && visible.contains(&neighbour_id),
                "step {step}: both ids must survive stable riding, visible = {visible:?}"
            );
            assert_eq!(
                state.visible_len(),
                2,
                "step {step}: stable ground must not mint churn ids"
            );
        }
    }

    /// The marginal one-sided senior case, held over repeated detects.
    ///
    /// A short senior sits marginally inside a long candidate while a dominant
    /// junior covers most of it. At the default `merge_mutual_floor` of 0.0
    /// seniority alone decides the capture, which the merge-floor tests cover
    /// for a single plan. The open question was whether that capture then
    /// oscillates once the outcome is fed back as the next prior, churning the
    /// id and resetting the section's PR era. It does not: the fold reaches a
    /// fixed point and holds it.
    #[test]
    fn a_marginal_senior_capture_settles_instead_of_churning() {
        let long = line(46.0, 7.0, 120);
        let most = long[..90].to_vec();
        let short = long[..20].to_vec();

        let mut state = HysteresisState::default();
        // The short senior is established first, so it out-ranks the rest.
        state.step_assign(&[cand(short.clone(), 3)]);

        let batch = [
            cand(short.clone(), 3),
            cand(most.clone(), 9),
            cand(long.clone(), 12),
        ];
        let (_, first) = state.step_assign(&batch);
        let settled: Vec<String> = first.iter().map(|r| r.id.clone()).collect();

        for step in 0..8 {
            let (_, r) = state.step_assign(&batch);
            let ids: Vec<String> = r.iter().map(|x| x.id.clone()).collect();
            assert_eq!(
                ids, settled,
                "step {step}: every ground must keep the id it settled on"
            );
            assert_eq!(
                state.visible_len(),
                3,
                "step {step}: a settled catalogue mints nothing further"
            );
            assert!(
                r.iter().all(|x| x.fate != CandidateFate::Minted),
                "step {step}: no ground re-mints once it has an id"
            );
        }
    }
}
