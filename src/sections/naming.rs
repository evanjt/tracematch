//! Named-corridor resolution scoring: which visible section carries a name.
//!
//! A name is user data keyed to ground (a footprint captured at naming time),
//! never to a section row. Resolution binds the name back onto the catalogue:
//! the visible section covering the largest share of the footprint's trimmed
//! core carries it, ties broken by smaller lateral offset, then older section,
//! then id. A contained sub-piece of the named ground qualifies from a quarter
//! of the core. A resolution is refused when the covered core sits further
//! from the section than half the ground tolerance on average, so a name
//! never migrates onto a parallel twin.
//!
//! This module is the pure scoring layer; the engine owns intent storage,
//! SQL, and caching, and feeds candidates through [`score_named_candidate`]
//! and [`select_candidate`].

use crate::GpsPoint;
use crate::geo_utils::haversine_distance;

use super::identity::{CARRY_COVERAGE, GROUND_TOL_M};

/// Core floor: three ~100 m evidence cells, expressed through the ground
/// tolerance anchor (`GROUND_TOL_M` is half a cell).
pub const CORE_FLOOR_M: f64 = 6.0 * GROUND_TOL_M;
/// Fraction trimmed from each end of the footprint to form the resolution
/// core. Extent drift concentrates at endpoints (29% median early, 10%
/// settled), so resolving on the middle keeps a name attached through the
/// re-cuts that motivated the feature.
pub const CORE_TRIM_FRAC: f64 = 0.15;
/// Coverage scores within this of the best are ties, broken by lateral
/// offset.
pub const COVERAGE_TIE: f64 = 0.05;
/// Resolution refuses a section whose covered core sits further away than
/// this on average. Half the ground tolerance: a name must bind finer than
/// the 50 m same-corridor metric or a 30 m parallel twin would satisfy it,
/// while same-line GPS noise stays well under.
pub const OFFSET_CEILING_M: f64 = GROUND_TOL_M / 2.0;
/// A section covering less core than this cannot carry the name even when it
/// is a contained sub-piece of the named ground. Matches the quarter-share
/// floor a split piece needs before it is associated with the name at all.
pub const PART_FLOOR: f64 = 0.25;

/// The middle of the footprint by arc length: trim [`CORE_TRIM_FRAC`] from
/// each end, never below [`CORE_FLOOR_M`] of retained length. Short
/// footprints are their own core.
pub fn trim_core(footprint: &[GpsPoint]) -> Vec<GpsPoint> {
    if footprint.len() < 3 {
        return footprint.to_vec();
    }
    let mut cum = Vec::with_capacity(footprint.len());
    let mut total = 0.0;
    cum.push(0.0);
    for w in footprint.windows(2) {
        total += haversine_distance(&w[0], &w[1]);
        cum.push(total);
    }
    if total <= CORE_FLOOR_M {
        return footprint.to_vec();
    }
    let trim = (CORE_TRIM_FRAC * total).min((total - CORE_FLOOR_M) / 2.0);
    let (lo, hi) = (trim, total - trim);
    let core: Vec<GpsPoint> = footprint
        .iter()
        .zip(&cum)
        .filter(|(_, d)| **d >= lo && **d <= hi)
        .map(|(p, _)| *p)
        .collect();
    if core.len() < 2 {
        footprint.to_vec()
    } else {
        core
    }
}

/// Coverage of `core` by `line` at the ground tolerance, and the mean
/// distance of the covered points to the line. The mean offset is the
/// twin-lane discriminator: a 30 m parallel can reach coverage at a 50 m
/// tolerance, but its offset gives it away.
pub fn coverage_and_offset(core: &[GpsPoint], line: &[GpsPoint]) -> (f64, f64) {
    if core.is_empty() || line.is_empty() {
        return (0.0, f64::INFINITY);
    }
    let mut covered = 0usize;
    let mut offset_sum = 0.0;
    for s in core {
        let d = line
            .iter()
            .map(|p| haversine_distance(s, p))
            .fold(f64::INFINITY, f64::min);
        if d <= GROUND_TOL_M {
            covered += 1;
            offset_sum += d;
        }
    }
    if covered == 0 {
        return (0.0, f64::INFINITY);
    }
    (
        covered as f64 / core.len() as f64,
        offset_sum / covered as f64,
    )
}

/// A qualifying candidate's score against one named footprint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NamedScore {
    pub coverage: f64,
    pub offset_m: f64,
}

/// Score one visible section against a named footprint's core, applying the
/// offset ceiling and the qualification rule: cover most of the core, or be
/// a contained sub-piece of the named ground carrying at least a quarter of
/// it — a corridor that re-emerges shorter than what was named keeps its
/// name. Returns None for a section that cannot carry the name.
pub fn score_named_candidate(
    core: &[GpsPoint],
    footprint: &[GpsPoint],
    line: &[GpsPoint],
) -> Option<NamedScore> {
    let (coverage, offset_m) = coverage_and_offset(core, line);
    if offset_m > OFFSET_CEILING_M {
        return None;
    }
    let qualifies = coverage >= CARRY_COVERAGE
        || (coverage >= PART_FLOOR && coverage_and_offset(line, footprint).0 >= CARRY_COVERAGE);
    qualifies.then_some(NamedScore { coverage, offset_m })
}

/// A qualifying candidate with the tie-break keys of its section row.
#[derive(Debug, Clone)]
pub struct NamedCandidate<'a> {
    pub score: NamedScore,
    pub created_at: &'a str,
    pub id: &'a str,
}

/// The winning candidate: largest core coverage (the split ruling), with
/// candidates within [`COVERAGE_TIE`] of the maximum treated as a band whose
/// winner is the smallest lateral offset, then the older section, then id.
/// Two deterministic total-order passes — a single comparator with a tie
/// band is not a strict weak ordering and `sort_by` may panic on one.
/// Returns the index into `candidates` and the winner's coverage.
pub fn select_candidate(candidates: &[NamedCandidate]) -> Option<(usize, f64)> {
    let top_cov = candidates
        .iter()
        .map(|c| c.score.coverage)
        .fold(None::<f64>, |m, c| Some(m.map_or(c, |m| m.max(c))))?;
    let floor = top_cov - COVERAGE_TIE;
    candidates
        .iter()
        .enumerate()
        .filter(|(_, c)| c.score.coverage >= floor)
        .min_by(|(_, a), (_, b)| {
            a.score
                .offset_m
                .total_cmp(&b.score.offset_m)
                .then_with(|| a.created_at.cmp(b.created_at))
                .then_with(|| a.id.cmp(b.id))
        })
        .map(|(i, c)| (i, c.score.coverage))
}

/// Angular half-width around a cardinal within which a split sibling earns a
/// directional discriminator. Beyond it the direction word would be a guess,
/// so the caller falls back to an ordinal.
pub const DIRECTION_TOL_DEG: f64 = 30.0;

/// The locale-neutral direction token of a freshly split sibling relative to
/// the piece that kept the id: `Some("north" | "east" | "south" | "west")`
/// when the bearing between the two midpoints lies within
/// [`DIRECTION_TOL_DEG`] of that cardinal, `None` when it is diagonal or the
/// midpoints coincide (the caller renders an ordinal instead). Pure geometry;
/// the app renders the token in-locale at read time.
pub fn split_direction(parent: &[GpsPoint], sibling: &[GpsPoint]) -> Option<&'static str> {
    let mid = |pts: &[GpsPoint]| -> Option<GpsPoint> {
        let n = pts.len();
        (n > 0).then(|| pts[n / 2])
    };
    let (p, s) = (mid(parent)?, mid(sibling)?);
    let dy = s.latitude - p.latitude;
    let dx = (s.longitude - p.longitude) * p.latitude.to_radians().cos();
    if dx == 0.0 && dy == 0.0 {
        return None;
    }
    // Compass bearing: 0 = north, clockwise.
    let bearing = dx.atan2(dy).to_degrees().rem_euclid(360.0);
    for (cardinal, name) in [
        (0.0, "north"),
        (90.0, "east"),
        (180.0, "south"),
        (270.0, "west"),
    ] {
        let sep = (bearing - cardinal)
            .abs()
            .min(360.0 - (bearing - cardinal).abs());
        if sep <= DIRECTION_TOL_DEG {
            return Some(name);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(lat: f64, lng: f64) -> GpsPoint {
        GpsPoint {
            latitude: lat,
            longitude: lng,
            elevation: Some(500.0),
        }
    }

    /// A straight north line of `n` points spaced ~`step_m`, offset east.
    fn line(n: usize, step_m: f64, east_m: f64) -> Vec<GpsPoint> {
        (0..n)
            .map(|i| {
                pt(
                    46.0 + (i as f64 * step_m) / 111_320.0,
                    7.0 + east_m / (111_320.0 * 46.0_f64.to_radians().cos()),
                )
            })
            .collect()
    }

    #[test]
    fn split_direction_names_cardinals_and_refuses_diagonals() {
        let parent = line(50, 10.0, 0.0);
        // Same line, continuing north: sibling midpoint due north.
        let north_piece: Vec<GpsPoint> = (0..50)
            .map(|i| pt(46.005 + (i as f64 * 10.0) / 111_320.0, 7.0))
            .collect();
        assert_eq!(split_direction(&parent, &north_piece), Some("north"));
        // A parallel corridor 500 m east at the same latitudes: due east.
        let east_piece = line(50, 10.0, 500.0);
        assert_eq!(split_direction(&parent, &east_piece), Some("east"));
        // A 45-degree diagonal sits outside every cardinal's 30-degree band.
        let diagonal: Vec<GpsPoint> = (0..50)
            .map(|i| pt(46.005 + (i as f64 * 10.0) / 111_320.0, 7.0065))
            .collect();
        assert_eq!(split_direction(&parent, &diagonal), None);
        // Degenerate inputs never invent a direction.
        assert_eq!(split_direction(&parent, &[]), None);
        assert_eq!(split_direction(&[], &parent), None);
        assert_eq!(split_direction(&parent, &parent), None);
    }

    #[test]
    fn core_trims_ends_but_never_below_floor() {
        let long = line(200, 10.0, 0.0); // ~2 km
        let core = trim_core(&long);
        assert!(core.len() < long.len(), "a long footprint must trim");
        assert!(core.len() > long.len() / 2, "the core keeps the middle 70%");

        let short = line(10, 10.0, 0.0); // ~90 m, under the floor
        assert_eq!(trim_core(&short).len(), short.len());
    }

    #[test]
    fn candidate_selection_is_order_independent() {
        let cand = |cov: f64, off: f64, created: &'static str, id: &'static str| NamedCandidate {
            score: NamedScore {
                coverage: cov,
                offset_m: off,
            },
            created_at: created,
            id,
        };
        // Pathological near-tie chain: pairwise tie bands overlap so a naive
        // banded comparator is intransitive. The winner must not depend on
        // input order.
        let a = vec![
            cand(0.34, 10.0, "2026-01-01", "s0"),
            cand(0.30, 5.0, "2026-01-02", "s1"),
            cand(0.26, 1.0, "2026-01-03", "s2"),
        ];
        let b = vec![a[2].clone(), a[1].clone(), a[0].clone()];
        let win_a = select_candidate(&a).map(|(i, cov)| (a[i].id, cov));
        let win_b = select_candidate(&b).map(|(i, cov)| (b[i].id, cov));
        assert_eq!(win_a, win_b);
        // Band anchors at the maximum: 0.26 falls outside 0.34 - 0.05, so the
        // winner is the lower-offset member of {0.34, 0.30}.
        assert_eq!(win_a, Some(("s1", 0.30)));
    }

    #[test]
    fn offset_discriminates_a_parallel_twin() {
        let named = line(100, 10.0, 0.0);
        let twin = line(100, 10.0, 30.0);
        let (cov_self, off_self) = coverage_and_offset(&named, &named);
        let (cov_twin, off_twin) = coverage_and_offset(&named, &twin);
        assert!(cov_self >= 0.99);
        assert!(off_self < 1.0);
        assert!(
            cov_twin > CARRY_COVERAGE,
            "the twin must clear coverage at the 50 m tolerance for the offset ceiling to matter"
        );
        assert!(
            off_self < OFFSET_CEILING_M && off_twin > OFFSET_CEILING_M,
            "the ceiling must separate the named line ({off_self:.1} m) from the twin ({off_twin:.1} m)"
        );
        assert!(score_named_candidate(&named, &named, &twin).is_none());
    }

    #[test]
    fn contained_sub_piece_keeps_the_name_from_a_quarter_of_the_core() {
        let footprint = line(200, 10.0, 0.0); // ~2 km named ground
        let core = trim_core(&footprint);
        // A re-emerged piece: the middle ~30% of the named line. Under the
        // 60% carry floor on core coverage, but a contained sub-piece above
        // the quarter floor.
        let piece: Vec<GpsPoint> = footprint[70..130].to_vec();
        let score = score_named_candidate(&core, &footprint, &piece)
            .expect("a contained sub-piece above the quarter floor qualifies");
        assert!(score.coverage < CARRY_COVERAGE);
        assert!(score.coverage >= PART_FLOOR);

        // Too small a piece: under the quarter floor, no qualification even
        // though it is perfectly contained.
        let sliver: Vec<GpsPoint> = footprint[95..105].to_vec();
        assert!(score_named_candidate(&core, &footprint, &sliver).is_none());
    }
}
