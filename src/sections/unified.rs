//! Unified section detection.
//!
//! One rule defines a section: **a maximal corridor stretch of
//! near-constant traffic composition.**
//!
//! 1. **Evidence — coverage grid.** Rasterise every track of a sport
//!    into ~100 m cells, recording the unique track set per cell and how
//!    many times each track passes through it (pass class 1 / 2 / 3+,
//!    elevation-aware so switchback legs never inflate the count).
//! 2. **Traffic partition.** Union adjacent hot cells whose track sets
//!    are mutually near-identical (both containments ≥ `1 − divergence`,
//!    with one-track slack for GPS wobble) AND whose shared tracks agree
//!    on pass class. Lateral cell braiding on one road collapses into
//!    one component; a fork splits it; so does a change in usage — a
//!    lollipop mouth (stem covered twice, loop once), an oval entrance
//!    (stem twice, oval many), the point where an out-and-back's shared
//!    ground meets a one-way variant. Composition is *who* passes and
//!    *how often*.
//! 3. **Only visible boundaries survive.** A boundary between two
//!    components survives only where the diverging branch is itself a
//!    section: it must take a significant share of the through traffic
//!    AND carry enough activities to stand alone. A cut whose branch
//!    never surfaces is invisible on the map — the corridor would stop
//!    for no reason the athlete can see. Merging can promote a branch,
//!    so this runs to a fixed point. Pass-class boundaries are exempt:
//!    a turnaround or loop mouth is a visible reason to end a section
//!    even with no third corridor there. Sections are disjoint by
//!    construction — no post-hoc overlap removal or splitting needed.
//! 4. **Support rule.** Adaptive visit floor by length and corpus size
//!    (`required_visits_for_length`), plus min-length.
//! 5. **Geometry — one real trace.** Each contributing activity's
//!    portion is reduced to a single pass (see [`simple_pass_range`]);
//!    `process_cluster` picks the medoid and computes consensus for its
//!    metadata (confidence, spread, point density), but the polyline
//!    itself is replaced by the medoid activity's actual points. A
//!    section is always ground someone really covered, never an average
//!    and never stitched from several activities.
//! 6. **Selection backoff.** Candidates are emitted best-first (most
//!    real usage represented) and ground already represented by an
//!    accepted polyline is never re-emitted: a candidate wholly within
//!    a cell's width of the accepted catalogue backs off, and a partly
//!    represented one is trimmed to its longest unrepresented run,
//!    standing only if that remnant still qualifies on its own
//!    ([`probe_mask`], [`unrepresented_runs`]). Nothing is merged and
//!    nothing synthetic is created — braid-lane twins simply lose to
//!    the better line, and their traversals still match it.
//!    Under-representing beats scattering near-duplicates. "Too close"
//!    needs no new constant: it is one cell (braid width, proximity/2),
//!    and deliberately planimetric — plan cells are atomic in the
//!    partition, so stacked ground never surfaces as a separate
//!    candidate here, and cross-day absolute elevation comparison is
//!    barometric drift, not signal. Geometry against polylines, not
//!    cell blobs: a loop or variant beside a corridor swings wide of
//!    the winner's line and keeps its own distinct shape.
//! 7. **Chain-coherent references.** Sections that tile one physical
//!    line (endpoints within two cells) prefer geometry cut from ONE
//!    covering activity ([`unify_chain_references`]): a fork-cut chain
//!    renders as consecutive ranges of a single real trace, joints
//!    meeting on a shared trace point instead of splices from
//!    different days. Boundaries, visits, and evidence are untouched —
//!    only the reference pick is coordinated, and never across ground
//!    no single pass actually connected.

use super::density_grid::{CellGrid, bresenham_cells, longest_run_in_cells};
use super::overlap::{FullTrackOverlap, OverlapCluster};
use super::postprocess::required_visits_for_length;
use super::{FrequentSection, SectionConfig, process_cluster};
use crate::GpsPoint;
use crate::union_find::UnionFind;
use log::info;
use std::collections::{HashMap, HashSet};

type Cell = (i32, i32);

/// One contributing activity's single-pass portion through a component:
/// (track index, start point, end point, metres).
type Portion = (usize, usize, usize, f64);

/// Pass class: how many times one track passes through one cell.
/// Saturates at 3 — an oval lapped 4 or 12 times reads the same.
const PASS_CLASS_MAX: u8 = 3;

/// One explained decision: why a cut between two adjacent components
/// survived, or why a candidate corridor was not emitted. Every
/// boundary has a visible reason; these records carry that reason as
/// data, in place of log lines.
#[derive(Clone, Debug, serde::Serialize)]
pub struct BoundaryRecord {
    pub latitude: f64,
    pub longitude: f64,
    pub reason: BoundaryReason,
}

/// The mechanism behind a [`BoundaryRecord`], with the numbers that
/// decided it. The decisions themselves live in
/// [`merge_non_fork_boundaries`] and the selection backoff; records
/// only report them.
#[derive(Clone, Debug, serde::Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum BoundaryReason {
    /// Most shared traffic crosses the join at a different pass class:
    /// a turnaround, a lollipop mouth, an oval entrance.
    UsageChange { shared: u32, mismatched: u32 },
    /// The departing traffic is collected by a section-worthy third
    /// corridor that physically meets the join.
    Fork {
        through: u32,
        needed: f64,
        branch_leavers: u32,
    },
    /// The candidate's own geometry already runs within a cell of the
    /// accepted catalogue: represented by the better line, not emitted.
    Backoff {
        represented: u32,
        probed: u32,
        score_metres: f64,
    },
    /// Part of the candidate ran within a cell of the accepted
    /// catalogue and was cut away; the section stands on the remnant.
    Trim {
        kept_metres: f64,
        dropped_metres: f64,
    },
    /// No contributing pass through the ground is a single traversal:
    /// a junction where every visit mills, or a lone spin with no clean
    /// line. The candidate backs off; its evidence stays and can
    /// re-emerge, like any backoff.
    NoSinglePass { best_penalty: f64, portions: u32 },
    /// A stretch of the candidate's ground was traversed by fewer of
    /// its own contributors than the support floor: a one-off spur
    /// welded to a busy corridor through the traffic gradient at their
    /// junction. The stretch is cut; the section stands on the
    /// supported remnant.
    LowSupport { floor: u32, dropped_cells: u32 },
}

/// A detection outcome that carries its explanations.
pub struct UnifiedDetection {
    pub sections: Vec<FrequentSection>,
    pub boundaries: Vec<BoundaryRecord>,
}

/// The detector's free constants, each documented with its meaning and
/// the evidence behind its default. Production always runs
/// [`Tunables::DEFAULT`]; [`detect_sections_unified_tuned`] exists so
/// the validation lab can sweep each value one at a time and verify the
/// defaults sit on plateaus rather than on peaks fitted to one
/// athlete's corpus (plateau tables: unified-lab REPORT.md, A1).
#[derive(Clone, Copy, Debug)]
pub struct Tunables {
    /// A re-arrival only counts as a new pass after this many distinct
    /// fine-cell events away: a wiggly line re-clipping a cell corner
    /// is one pass, a lap or a return leg is not. Must exceed the
    /// couple of events a wobbled line spends around one corner while
    /// staying far below a lap's event count. Plateau: catalogue
    /// byte-stable across 3-8 on both corpora.
    pub pass_away_cells: usize,
    /// Elevation separation that makes ground a different level.
    /// Re-arrivals beyond it are switchback legs stacked vertically
    /// (new ground, never a repeat pass); the single-pass cut uses the
    /// same tolerance to keep hairpin climbs uncut. Sits between GPS
    /// elevation noise (under ~10 m) and the vertical gap of stacked
    /// switchback legs (20-40 m on the validation corpora). Plateau:
    /// 10-25 m within a few points on every metric, both corpora.
    pub ele_level_tol_m: f64,
    /// Passes are counted on a subgrid this many times finer than the
    /// partition grid. Corridor cells must be coarse so lateral GPS
    /// braiding stays one corridor, but at that size a small loop's
    /// laps re-touch each cell within the away-gap and are invisible.
    /// Counting on the fine grid and aggregating each track's
    /// fine-cell classes to the partition cell by mode answers both
    /// questions at their natural scale. A resolution, not a free
    /// knob: fine cells are cell/3 ≈ 33 m at default proximity,
    /// between GPS braid noise (subgrid 4 over-cuts, stability −12
    /// points on Sion) and small-feature scale (subgrid 2
    /// under-resolves laps on Sion, over-fragments the full corpus).
    /// 3 is the value that behaves on both corpora.
    pub pass_subgrid: f64,
    /// Single-pass cut: a re-entry only counts against ground last
    /// touched more than this many cell events ago, so wobbling along
    /// a cell boundary never looks like a return. Derived as
    /// `pass_window + 1`: no event inside the decision window can arm
    /// its own trigger. Plateau: 4-10 moves the catalogue ≤4% on both
    /// corpora.
    pub dwell_events: usize,
    /// Single-pass cut: how many recent cell events are inspected...
    pub pass_window: usize,
    /// ...and how many of them must be re-entries before the portion
    /// is cut where the re-covering began. A majority-of-recent rule;
    /// the swept combinations (2-of-4 through 4-of-6) are flat on both
    /// corpora.
    pub pass_needed: usize,
    /// Single-pass cut: neighbourhood reach in fine cells. Reach, not
    /// cell coarseness, supplies the lateral tolerance for GPS
    /// braiding, so small features still produce enough events to be
    /// seen. One fine cell ≈ braid width (proximity/6); the minimum
    /// that absorbs braid. Reach 2 triples the lateral reach and costs
    /// 16-18% of catalogue ground on both corpora.
    pub reach: i32,
    /// Lift exclusion: window length over which ascent and
    /// straightness must be sustained. The climb-convention sustain
    /// scale (a climb, not a spike). Calibrated, with measured failure
    /// on each side: at 200 m merged spans absorb approach wobble and
    /// a cable car reads human (the 82% section resurfaces) while the
    /// steep-walk anchor is excluded; at 400 m the steep-walk anchor
    /// is lost on both corpora. Residual risk is why the velocity veto
    /// below is the robust signal wherever per-point time exists.
    pub lift_span_m: f64,
    /// Lift exclusion: minimum sustained ascent grade for a window.
    /// Measured chairlifts climb at 17-34%; steep walked ground
    /// overlaps the same range, which is why grade alone never
    /// classifies (straightness and jitter do). Working band measured
    /// at 0.18-0.26; at 0.30 the floor exceeds real lift grades and
    /// cable-car ground resurfaces. 0.18 was probed against 0.22: it
    /// flags 29 more spans (mostly real 18-21% lift rides) but its
    /// only catalogue effect is losing a real winding piste section
    /// to collateral span growth, and a straightness-by-grade scan
    /// shows the 0.22 catalogue holds no lift-like section anyway.
    /// The floor only marks ascending ground; lift lines are only
    /// ever ascended, and their sub-floor evidence forms no section.
    pub lift_min_grade: f64,
    /// Lift exclusion: minimum chord/arc straightness, for qualifying
    /// windows and for the descent-retrace rescue path. Measured lift
    /// windows: median 0.996; self-powered steep ground: median 0.73,
    /// p90 0.894. Calibrated between them: at 0.96 spans merge wobbly
    /// connectors and dilute the glide signature, at 0.985 spans trim
    /// to straight cores that dodge the jitter veto; both lose the
    /// steep-walk anchor on both corpora.
    pub lift_min_straight: f64,
    /// Lift exclusion: raw arc over smoothed arc at and above which a
    /// span moves like a human. Cabins glide (measured median 1.02,
    /// p95 1.065); every walked steep-straight climb measured, across
    /// four countries, sits at 1.053 or higher. Deliberately the
    /// highest safe value: 1.065+ starts eating real climbs on both
    /// corpora, while the cable-car gate holds all the way down to
    /// 1.02. Device-sensitive; when per-point time exists the velocity
    /// veto below is the robust signal and jitter is only the fallback
    /// for time-less tracks.
    pub jitter_human_min: f64,
    /// Lift exclusion, velocity veto: median ground speed below which a
    /// timed span is moving at human climbing pace. Every measured
    /// human span among the geometry candidates walks at 0.40-0.63 m/s;
    /// carried spans ride at 4.7-5.8 m/s and chairlifts are engineered
    /// for 2-5 m/s line speed. 1.5 m/s (5.4 km/h on >=22% ground) is
    /// above any sustainable walking pace on that grade yet under every
    /// carried system; any value in 0.7-4.6 separates the measured
    /// corpora. Paired with `lift_min_climb_mh` because elite uphill
    /// racers can reach carried GROUND speeds on moderate grades while
    /// their vertical rate stays human.
    pub lift_min_speed_ms: f64,
    /// Lift exclusion, velocity veto: vertical rate below which a timed
    /// span climbs like a human. The all-time human ceiling is the
    /// Vertical Kilometre record, ~2,020 m/h sustained for half an hour
    /// (Fully, 2017); recreational steep hiking runs 300-900 m/h.
    /// Measured candidates: human spans 317-566 m/h, carried spans
    /// 4,400-13,900 m/h. 1,500 m/h sits 2.6x above the measured human
    /// envelope and 2.9x under the measured carried envelope. A span is
    /// declassified as human only when BOTH this and the speed floor
    /// say human; either signal alone at carried levels keeps the span.
    pub lift_min_climb_mh: f64,
    /// Lift exclusion: matching tolerance when hunting a straight
    /// descent along a lift candidate's own line (the rescue for
    /// stairs and fall-line paths that people also walk down).
    /// Plateau: 40-80 m byte-flat on both corpora.
    pub descent_match_m: f64,
    /// Geographic clustering: tracks whose bounding boxes come within
    /// this gap share one cluster, and each cluster projects onto its
    /// own reference latitude. Correctness only needs clusters farther
    /// apart than a couple of grid cells (evidence never interacts
    /// beyond ~2 x 100 m); the value sits far above that so a region's
    /// outings share one stable plane, and far below continent spacing
    /// so hemispheres never share one (a single global plane sized
    /// Melbourne's east-west cells ~14% wrong against a Valais-heavy
    /// mean, and let ground on another continent move every cell
    /// boundary at home). Within a cluster the residual scale error is
    /// tan(lat) x half-span: under 1% out to ~60 km half-spans at
    /// mid-latitudes. Swept 10-200 km: single-region corpora are
    /// byte-flat across the whole range; on the two-continent corpus
    /// the partition itself moves with the gap, which re-buckets
    /// marginal ground while core one-to-one stability stays flat at
    /// 76-80%. Derivation-anchored, never plateau-fitted: any partition
    /// change re-cuts marginals by the same mechanism that makes local
    /// projection necessary in the first place.
    pub cluster_gap_m: f64,
    /// Single-pass render guard: the largest share of a
    /// representative's points that may revisit its own earlier ground
    /// (see [`self_pass_penalty`]) before the render rejects it. The
    /// default render (longest portion, or the medoid) stands when it
    /// sits under this floor; above it, the cleanest contributing pass
    /// is rendered instead, and a candidate with no pass under the
    /// floor backs off as a junction blob. Wide margin on the review
    /// corpora: kept single passes and clean laps measure <= 0.14, a
    /// mid-line spin 0.34, a directionless junction 0.60; the plateau
    /// sweep lands with A1.
    pub self_pass_max: f64,
}

impl Tunables {
    pub const DEFAULT: Tunables = Tunables {
        pass_away_cells: 5,
        ele_level_tol_m: 15.0,
        pass_subgrid: 3.0,
        dwell_events: 6,
        pass_window: 5,
        pass_needed: 3,
        reach: 1,
        lift_span_m: 300.0,
        lift_min_grade: 0.22,
        lift_min_straight: 0.975,
        jitter_human_min: 1.05,
        lift_min_speed_ms: 1.5,
        lift_min_climb_mh: 1500.0,
        descent_match_m: 60.0,
        cluster_gap_m: 50_000.0,
        self_pass_max: 0.25,
    };
}

impl Default for Tunables {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Per-sport coverage grid: unique tracks per cell, plus each track's
/// pass class per cell.
struct CoverageGrid {
    grid: CellGrid,
    ref_lat: f64,
    cell_tracks: HashMap<Cell, HashSet<u32>>,
    cell_passes: HashMap<Cell, HashMap<u32, u8>>,
    /// Per track, the index ranges that remain once lift-carried spans
    /// are removed. Geometry cuts walk these ranges and nothing else, so
    /// a portion can never bridge across excluded ground that happens to
    /// touch the component at both ends (base station and summit do).
    keep: Vec<Vec<(usize, usize)>>,
}

/// Per-track scratch while counting passes through one cell.
struct PassScratch {
    last_seq: usize,
    /// (elevation level, passes at that level).
    levels: Vec<(Option<f64>, u8)>,
}

/// Ineligible ground: lift-carried spans (cable cars, chairlifts,
/// funiculars) detected from geometry alone.
///
/// The separator is straightness at window scale, not grade: measured
/// on the validation corpora, lift windows at climbing grades run
/// station-to-station straight (median chord/arc 0.996) while
/// self-powered steep ground winds (median 0.73, p90 0.894). Grade
/// alone cannot separate: typical chairlifts climb at 17-34%, inside
/// the steep-hiking range. A window qualifies when it sustains at
/// least `LIFT_MIN_GRADE` ascent over `LIFT_SPAN_M` while staying
/// straighter than `LIFT_MIN_STRAIGHT`; overlapping qualifying windows
/// merge into spans. A span must then survive three vetoes: real net
/// rise (guards barometric drift), low micro-jitter, and human-paced
/// movement wherever per-point time exists. Jitter is the raw arc over
/// the smoothed arc: a cabin glides (measured lift median 1.02, p95
/// 1.065) while a walker on steep ground wobbles (every walked
/// steep-straight climb measured, across four countries, sits at 1.053
/// or higher, and bootpack ascents inside snowboard days land in the
/// same band). Jitter depends on device sampling, so when `seconds`
/// are supplied the velocity veto decides instead: a span whose median
/// windowed ground speed AND vertical rate both sit at human climbing
/// levels was walked, whatever its geometry says (measured envelopes
/// on the constants in [`Tunables`]). Marked spans contribute no
/// evidence to the coverage grid, so lift ground never becomes a
/// section and never bridges the runs it connects. A descending lift
/// ride is geometrically indistinguishable from a steep descent and is
/// never marked; descent ground keeps its evidence.
pub fn lift_spans(pts: &[GpsPoint], seconds: Option<&[f64]>) -> Vec<(usize, usize)> {
    lift_spans_tuned(pts, seconds, &Tunables::DEFAULT)
}

/// [`lift_spans`] with explicit [`Tunables`], for the lab's sweeps.
/// `seconds` is the per-point time offset parallel to `pts`; `None` (or
/// a length mismatch) leaves the velocity veto out and geometry decides
/// alone.
pub fn lift_spans_tuned(
    pts: &[GpsPoint],
    seconds: Option<&[f64]>,
    tun: &Tunables,
) -> Vec<(usize, usize)> {
    let seconds = seconds.filter(|s| s.len() == pts.len());
    let n = pts.len();
    if n < 3 || pts.iter().filter(|p| p.elevation.is_some()).count() < 2 {
        return Vec::new();
    }
    // All window geometry runs on a lightly smoothed track: slow lifts
    // sample GPS jitter comparable to the point spacing, which inflates
    // the raw arc and hides the line's straightness. Smoothing at ±2
    // points is far below a switchback's wavelength, so winding ground
    // stays winding.
    let sp: Vec<GpsPoint> = (0..n)
        .map(|i| {
            let lo = i.saturating_sub(2);
            let hi = (i + 2).min(n - 1);
            let c = (hi - lo + 1) as f64;
            GpsPoint::new(
                pts[lo..=hi].iter().map(|p| p.latitude).sum::<f64>() / c,
                pts[lo..=hi].iter().map(|p| p.longitude).sum::<f64>() / c,
            )
        })
        .collect();
    let mut cum = vec![0.0f64; n];
    let mut cum_raw = vec![0.0f64; n];
    for i in 1..n {
        cum[i] = cum[i - 1] + crate::geo_utils::haversine_distance(&sp[i - 1], &sp[i]);
        cum_raw[i] = cum_raw[i - 1] + crate::geo_utils::haversine_distance(&pts[i - 1], &pts[i]);
    }
    // Light smoothing so one elevation spike cannot fake a grade.
    let smooth: Vec<Option<f64>> = (0..n)
        .map(|i| {
            let lo = i.saturating_sub(1);
            let hi = (i + 1).min(n - 1);
            let (mut s, mut c) = (0.0, 0u32);
            for p in &pts[lo..=hi] {
                if let Some(e) = p.elevation {
                    s += e;
                    c += 1;
                }
            }
            if c > 0 { Some(s / f64::from(c)) } else { None }
        })
        .collect();

    let mut marked = vec![false; n];
    let mut j = 0usize;
    for i in 0..n {
        if j < i {
            j = i;
        }
        while j + 1 < n && cum[j] - cum[i] < tun.lift_span_m {
            j += 1;
        }
        let run = cum[j] - cum[i];
        if run < tun.lift_span_m {
            break;
        }
        let (Some(a), Some(b)) = (smooth[i], smooth[j]) else {
            continue;
        };
        if (b - a) / run < tun.lift_min_grade {
            continue;
        }
        let chord = crate::geo_utils::haversine_distance(&sp[i], &sp[j]);
        if chord / run < tun.lift_min_straight {
            continue;
        }
        for m in &mut marked[i..=j] {
            *m = true;
        }
    }

    let mut spans = Vec::new();
    let mut push_span = |s: usize, e: usize| {
        let rise = match (smooth[e], smooth[s]) {
            (Some(top), Some(bot)) => top - bot,
            _ => 0.0,
        };
        if rise < tun.lift_span_m * tun.lift_min_grade * 0.5 {
            return;
        }
        // Velocity veto when the track is timed, jitter otherwise.
        // Speed is sampled over fixed-distance windows and judged by
        // median, so a mid-ride halt occupies one window instead of
        // dragging the whole estimate; the vertical rate spans the full
        // duration. Windows are lift_span_m/10: coarse enough to absorb
        // point cadence, fine enough for several samples per span.
        if let Some(secs) = seconds {
            let dur = secs[e] - secs[s];
            let win = tun.lift_span_m / 10.0;
            let mut speeds: Vec<f64> = Vec::new();
            let (mut d0, mut t0) = (cum_raw[s], secs[s]);
            for i in s + 1..=e {
                if cum_raw[i] - d0 >= win {
                    let dt = secs[i] - t0;
                    if dt > 0.0 {
                        speeds.push((cum_raw[i] - d0) / dt);
                    }
                    d0 = cum_raw[i];
                    t0 = secs[i];
                }
            }
            if dur > 0.0 && speeds.len() >= 3 {
                speeds.sort_unstable_by(f64::total_cmp);
                let human_speed = speeds[speeds.len() / 2] < tun.lift_min_speed_ms;
                let human_climb = rise / dur * 3600.0 < tun.lift_min_climb_mh;
                if !(human_speed && human_climb) {
                    spans.push((s, e));
                }
                return;
            }
        }
        let smooth_arc = (cum[e] - cum[s]).max(1.0);
        let jitter = (cum_raw[e] - cum_raw[s]) / smooth_arc;
        if jitter < tun.jitter_human_min {
            spans.push((s, e));
        }
    };
    let mut start: Option<usize> = None;
    for (i, &m) in marked.iter().enumerate() {
        match (m, start) {
            (true, None) => start = Some(i),
            (false, Some(s)) => {
                push_span(s, i - 1);
                start = None;
            }
            _ => {}
        }
    }
    if let Some(s) = start {
        push_span(s, n - 1);
    }
    spans
}

/// Corpus confirmation for lift candidates: carried ground is ground
/// nobody ever comes straight down under their own power.
///
/// A steep straight walked climb (vineyard stairs, a fall-line path) is
/// geometrically identical to a chairlift line, but its climbers retrace
/// it downhill, near-straight, on the same or another outing. A lift
/// line is only ever descended by riders weaving under it, whose own
/// paths are anything but straight. A candidate span is therefore
/// rescued when any track descends most of its rise along its line with
/// a near-straight path of its own.
pub fn confirmed_lift_spans(
    tracks: &[(&str, &[GpsPoint])],
    seconds: &[&[f64]],
) -> Vec<Vec<(usize, usize)>> {
    confirmed_lift_spans_tuned(tracks, seconds, &Tunables::DEFAULT)
}

/// [`confirmed_lift_spans`] with explicit [`Tunables`], for the lab's
/// sweeps. `seconds` holds each track's per-point time offsets,
/// parallel to `tracks`; pass `&[]` (or an empty slice per untimed
/// track) where time is unavailable.
pub fn confirmed_lift_spans_tuned(
    tracks: &[(&str, &[GpsPoint])],
    seconds: &[&[f64]],
    tun: &Tunables,
) -> Vec<Vec<(usize, usize)>> {
    let candidates: Vec<Vec<(usize, usize)>> = tracks
        .iter()
        .enumerate()
        .map(|(i, (_, pts))| lift_spans_tuned(pts, seconds.get(i).copied(), tun))
        .collect();
    if candidates.iter().all(|c| c.is_empty()) {
        return candidates;
    }

    let bboxes: Vec<(f64, f64, f64, f64)> = tracks
        .iter()
        .map(|(_, pts)| {
            let mut bb = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for p in pts.iter() {
                bb.0 = bb.0.min(p.latitude);
                bb.1 = bb.1.max(p.latitude);
                bb.2 = bb.2.min(p.longitude);
                bb.3 = bb.3.max(p.longitude);
            }
            bb
        })
        .collect();
    let pad = tun.descent_match_m / 111_000.0 * 2.0;

    candidates
        .iter()
        .enumerate()
        .map(|(ti, spans)| {
            let pts = tracks[ti].1;
            spans
                .iter()
                .copied()
                .filter(|&(s, e)| {
                    let line = &pts[s..=e];
                    let rise = match (
                        line.last().and_then(|p| p.elevation),
                        line.first().and_then(|p| p.elevation),
                    ) {
                        (Some(top), Some(bot)) => top - bot,
                        _ => return true,
                    };
                    let (mut lat0, mut lat1, mut lng0, mut lng1) =
                        (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
                    for p in line {
                        lat0 = lat0.min(p.latitude);
                        lat1 = lat1.max(p.latitude);
                        lng0 = lng0.min(p.longitude);
                        lng1 = lng1.max(p.longitude);
                    }
                    let descended_straight = tracks.iter().enumerate().any(|(oi, (_, op))| {
                        let bb = &bboxes[oi];
                        if bb.0 > lat1 + pad
                            || bb.1 < lat0 - pad
                            || bb.2 > lng1 + pad
                            || bb.3 < lng0 - pad
                        {
                            return false;
                        }
                        super::find_all_track_portions(op, line, tun.descent_match_m)
                            .into_iter()
                            .any(|(ps, pe, _)| {
                                let pe = pe.min(op.len() - 1);
                                if pe <= ps + 2 {
                                    return false;
                                }
                                let (Some(a), Some(b)) = (op[ps].elevation, op[pe].elevation)
                                else {
                                    return false;
                                };
                                if b - a > -rise * 0.5 {
                                    return false;
                                }
                                let seg = &op[ps..=pe];
                                let arc: f64 = seg
                                    .windows(2)
                                    .map(|w| crate::geo_utils::haversine_distance(&w[0], &w[1]))
                                    .sum();
                                let chord = crate::geo_utils::haversine_distance(
                                    &seg[0],
                                    &seg[seg.len() - 1],
                                );
                                arc > 1.0 && chord / arc >= tun.lift_min_straight
                            })
                    });
                    !descended_straight
                })
                .collect()
        })
        .collect()
}

fn build_coverage_grid(
    tracks: &[(&str, &[GpsPoint])],
    seconds: &[&[f64]],
    cell_size_m: f64,
    tun: &Tunables,
) -> CoverageGrid {
    let ref_lat: f64 = {
        // Summed in sorted order: float addition is not permutation
        // stable, and the catalogue must not depend on arrival order
        // even at the last bit of a cell boundary.
        let mut lats: Vec<f64> = tracks
            .iter()
            .flat_map(|(_, pts)| pts.iter().step_by(50).map(|p| p.latitude))
            .collect();
        lats.sort_unstable_by(f64::total_cmp);
        if lats.is_empty() {
            0.0
        } else {
            lats.iter().sum::<f64>() / lats.len() as f64
        }
    };
    let grid = CellGrid::new(cell_size_m, ref_lat);

    let fine = CellGrid::new(cell_size_m / tun.pass_subgrid, ref_lat);

    let lift = confirmed_lift_spans_tuned(tracks, seconds, tun);
    let keep: Vec<Vec<(usize, usize)>> = tracks
        .iter()
        .enumerate()
        .map(|(t_idx, (_, pts))| {
            let mut k: Vec<(usize, usize)> = Vec::new();
            let mut cursor = 0usize;
            for &(s, e) in &lift[t_idx] {
                if s > cursor {
                    k.push((cursor, s - 1));
                }
                cursor = e + 1;
            }
            if cursor < pts.len() {
                k.push((cursor, pts.len() - 1));
            }
            k
        })
        .collect();

    let mut cell_tracks: HashMap<Cell, HashSet<u32>> = HashMap::new();
    let mut cell_passes: HashMap<Cell, HashMap<u32, u8>> = HashMap::new();
    for (t_idx, (_, pts)) in tracks.iter().enumerate() {
        if pts.is_empty() {
            continue;
        }
        let t = t_idx as u32;

        // Fine-cell pass counting; each fine cell remembers the
        // partition cell it first appeared in.
        let mut scratch: HashMap<Cell, (Cell, PassScratch)> = HashMap::new();
        let mut fine_seq = 0usize;
        let mut visit_fine =
            |fc: Cell, pc: Cell, ele: Option<f64>, seq: usize| match scratch.entry(fc) {
                std::collections::hash_map::Entry::Occupied(mut e) => {
                    let (_, sc) = e.get_mut();
                    if seq - sc.last_seq > tun.pass_away_cells {
                        match sc.levels.iter_mut().find(|(lvl, _)| match (*lvl, ele) {
                            (Some(a), Some(b)) => (a - b).abs() < tun.ele_level_tol_m,
                            _ => true,
                        }) {
                            Some((_, n)) => *n = (*n + 1).min(PASS_CLASS_MAX),
                            None => sc.levels.push((ele, 1)),
                        }
                    }
                    sc.last_seq = seq;
                }
                std::collections::hash_map::Entry::Vacant(e) => {
                    e.insert((
                        pc,
                        PassScratch {
                            last_seq: seq,
                            levels: vec![(ele, 1)],
                        },
                    ));
                }
            };

        // Lift-carried stretches contribute nothing; each remaining
        // stretch walks with a fresh cell seed so no evidence bridges
        // across the skipped ground.
        for (r_idx, &(rs, re)) in keep[t_idx].iter().enumerate() {
            if r_idx > 0 {
                // A skipped lift stretch always counts as "away".
                fine_seq += tun.pass_away_cells + 1;
            }
            let seg = &pts[rs..=re];
            let mut prev = grid.cell_of(seg[0].latitude, seg[0].longitude);
            let mut prev_fine = fine.cell_of(seg[0].latitude, seg[0].longitude);
            cell_tracks.entry(prev).or_default().insert(t);
            visit_fine(prev_fine, prev, seg[0].elevation, fine_seq);
            for w in seg.windows(2) {
                let b = grid.cell_of(w[1].latitude, w[1].longitude);
                if b != prev {
                    // Bresenham covers GPS gaps that skip cells.
                    for c in bresenham_cells(prev, b).into_iter().skip(1) {
                        cell_tracks.entry(c).or_default().insert(t);
                    }
                    prev = b;
                }
                let bf = fine.cell_of(w[1].latitude, w[1].longitude);
                if bf != prev_fine {
                    for fc in bresenham_cells(prev_fine, bf).into_iter().skip(1) {
                        fine_seq += 1;
                        visit_fine(fc, prev, w[1].elevation, fine_seq);
                    }
                    prev_fine = bf;
                }
            }
        }

        // Aggregate: a partition cell's class for this track is the mode
        // of its fine-cell classes (ties to the lower class), so one
        // double-clipped fine cell can't relabel a whole corridor cell.
        let mut per_cell: HashMap<Cell, [u32; PASS_CLASS_MAX as usize]> = HashMap::new();
        for (_, (pc, sc)) in scratch {
            let class = sc.levels.iter().map(|&(_, n)| n).max().unwrap_or(1);
            per_cell.entry(pc).or_default()[(class - 1) as usize] += 1;
        }
        for (pc, counts) in per_cell {
            let mode = counts
                .iter()
                .enumerate()
                .max_by_key(|(i, n)| (**n, std::cmp::Reverse(*i)))
                .map(|(i, _)| i as u8 + 1)
                .unwrap_or(1);
            cell_passes.entry(pc).or_default().insert(t, mode);
        }
    }

    CoverageGrid {
        grid,
        ref_lat,
        cell_tracks,
        cell_passes,
        keep,
    }
}

/// Near-identical traffic test: mutual containment ≥ `same_traffic`,
/// with one-track absolute slack so low-traffic corridors don't
/// fragment on a single wobbled GPS trace.
fn same_traffic_sets(a: &HashSet<u32>, b: &HashSet<u32>, same_traffic: f64) -> bool {
    if a.is_empty() || b.is_empty() {
        return false;
    }
    let inter = a.intersection(b).count();
    let max_missing = (a.len() - inter).max(b.len() - inter);
    if max_missing <= 1 {
        return true;
    }
    let mutual = (inter as f64 / a.len() as f64).min(inter as f64 / b.len() as f64);
    mutual >= same_traffic
}

/// Shared tracks must pass through both cells equally often. Each track
/// is compared against itself across the two cells, so a corpus mixing
/// through-runners (1 pass) with out-and-backers (2) stays one corridor
/// — a boundary appears only where many tracks' own count changes at
/// the same place: a common turnaround, a lollipop mouth, an oval
/// entrance. Tolerates a divergence share of disagreeing tracks with
/// one-track slack, mirroring [`same_traffic_sets`].
fn pass_classes_agree(coverage: &CoverageGrid, a: Cell, b: Cell, divergence: f64) -> bool {
    let (Some(pa), Some(pb)) = (coverage.cell_passes.get(&a), coverage.cell_passes.get(&b)) else {
        return true;
    };
    let mut shared = 0usize;
    let mut mismatch = 0usize;
    for (t, ca) in pa {
        if let Some(cb) = pb.get(t) {
            shared += 1;
            if ca != cb {
                mismatch += 1;
            }
        }
    }
    shared == 0 || mismatch as f64 <= (divergence * shared as f64).max(1.0)
}

/// Reduce a portion to a **simple pass**: cut where the track starts
/// re-covering ground it already covered, so the polyline is a simple
/// path or a single closed loop and never retraces itself.
///
/// Walks the fine cell stream asking, at each cell change: am I within
/// one neighbouring cell of ground I covered more than a few events
/// ago, at my elevation level? When such re-entries dominate the recent
/// events (3 of the last 5), the portion is cut where the re-covering
/// began. The neighbourhood reach — not cell coarseness — supplies the
/// lateral tolerance for GPS braiding, so small features (a village
/// loop spans only a couple of coarse cells) still produce enough
/// events to be seen.
///
/// Guards: each cell keeps one entry per elevation level, so stacked
/// switchback legs (20–40 m apart vertically) never match each other
/// and a hairpin climb is never cut, while a descent matches its own
/// level and is. Levels refresh on every touch, so resting or wobbling
/// on a cell boundary never looks like a return.
///
/// Loops need no special case. Pass-class partitioning already splits
/// an approach stem from the loop it feeds, so a portion through a loop
/// component starts on the loop itself: a lapped oval cuts at the start
/// of lap two (one closed revolution) and a loop route only re-enters
/// covered ground back at its closing point.
fn simple_pass_range(pts: &[GpsPoint], fine: &CellGrid, tun: &Tunables) -> (usize, usize) {
    if pts.len() < 4 {
        return (0, pts.len());
    }

    // Per fine cell, one (last-touch event, elevation) entry per level.
    let mut visited: HashMap<Cell, Vec<(usize, Option<f64>)>> = HashMap::new();
    let mut prev_cell: Option<Cell> = None;
    let mut evt = 0usize;
    // Point index of each re-entry among the last pass_window cell events.
    let mut recent: std::collections::VecDeque<Option<usize>> =
        std::collections::VecDeque::with_capacity(tun.pass_window + 1);

    let same_level = |a: Option<f64>, b: Option<f64>| match (a, b) {
        (Some(a), Some(b)) => (a - b).abs() < tun.ele_level_tol_m,
        _ => true,
    };

    for (i, p) in pts.iter().enumerate() {
        let c = fine.cell_of(p.latitude, p.longitude);
        if prev_cell == Some(c) {
            continue;
        }
        prev_cell = Some(c);
        evt += 1;

        let mut reentry = false;
        for dy in -tun.reach..=tun.reach {
            for dx in -tun.reach..=tun.reach {
                if let Some(levels) = visited.get(&(c.0 + dy, c.1 + dx))
                    && levels.iter().any(|&(last, ele)| {
                        evt - last > tun.dwell_events && same_level(ele, p.elevation)
                    })
                {
                    reentry = true;
                }
            }
        }

        recent.push_back(reentry.then_some(i));
        if recent.len() > tun.pass_window {
            recent.pop_front();
        }
        if recent.iter().flatten().count() >= tun.pass_needed {
            let cut = recent.iter().flatten().next().copied().unwrap_or(i);
            return (0, cut);
        }

        let levels = visited.entry(c).or_default();
        match levels
            .iter_mut()
            .find(|(_, ele)| same_level(*ele, p.elevation))
        {
            Some(level) => level.0 = evt,
            None => levels.push((evt, p.elevation)),
        }
    }

    (0, pts.len())
}

/// A maximal same-traffic stretch of corridor.
struct Supernode {
    cells: Vec<Cell>,
    tracks: HashSet<u32>,
}

/// Partition hot cells into same-traffic components.
fn partition_supernodes(
    hot_cells: &[Cell],
    coverage: &CoverageGrid,
    same_traffic: f64,
) -> Vec<Supernode> {
    let index_of: HashMap<Cell, usize> =
        hot_cells.iter().enumerate().map(|(i, c)| (*c, i)).collect();
    let mut uf: UnionFind<usize> = UnionFind::with_capacity(hot_cells.len());

    for (i, cell) in hot_cells.iter().enumerate() {
        uf.make_set(i);
        let my_tracks = &coverage.cell_tracks[cell];
        for dy in -1..=1i32 {
            for dx in -1..=1i32 {
                if dy == 0 && dx == 0 {
                    continue;
                }
                let n = (cell.0 + dy, cell.1 + dx);
                if let Some(&j) = index_of.get(&n)
                    && j > i
                    && same_traffic_sets(my_tracks, &coverage.cell_tracks[&n], same_traffic)
                    && pass_classes_agree(coverage, *cell, n, 1.0 - same_traffic)
                {
                    uf.union(&i, &j);
                }
            }
        }
    }

    let mut by_root: HashMap<usize, Vec<Cell>> = HashMap::new();
    for (i, cell) in hot_cells.iter().enumerate() {
        by_root.entry(uf.find(&i)).or_default().push(*cell);
    }

    let mut supernodes: Vec<Supernode> = by_root
        .into_values()
        .map(|mut cells| {
            cells.sort_unstable();
            let mut tracks: HashSet<u32> = HashSet::new();
            for c in &cells {
                tracks.extend(coverage.cell_tracks[c].iter().copied());
            }
            Supernode { cells, tracks }
        })
        .collect();
    // Deterministic output order: section ids and geojson must be stable
    // across runs (HashMap iteration is randomised).
    supernodes.sort_by_key(|s| s.cells[0]);
    supernodes
}

/// Rebuild components from a union-find over component indices.
fn regroup(supernodes: Vec<Supernode>, uf: &mut UnionFind<usize>) -> Vec<Supernode> {
    let mut grouped: HashMap<usize, Supernode> = HashMap::new();
    for (i, s) in supernodes.into_iter().enumerate() {
        let entry = grouped.entry(uf.find(&i)).or_insert_with(|| Supernode {
            cells: Vec::new(),
            tracks: HashSet::new(),
        });
        entry.cells.extend(s.cells);
        entry.tracks.extend(s.tracks);
    }
    let mut merged: Vec<Supernode> = grouped.into_values().collect();
    for s in &mut merged {
        s.cells.sort_unstable();
    }
    merged.sort_by_key(|s| s.cells[0]);
    merged
}

fn cell_owners(supernodes: &[Supernode]) -> HashMap<Cell, usize> {
    supernodes
        .iter()
        .enumerate()
        .flat_map(|(i, s)| s.cells.iter().map(move |c| (*c, i)))
        .collect()
}

/// Adjacent cell pairs along each component pair's shared boundary,
/// stored as (cell in lower-index component, cell in higher-index one).
fn pair_boundaries(
    supernodes: &[Supernode],
    owner: &HashMap<Cell, usize>,
) -> HashMap<(usize, usize), Vec<(Cell, Cell)>> {
    let mut boundary: HashMap<(usize, usize), Vec<(Cell, Cell)>> = HashMap::new();
    for (i, s) in supernodes.iter().enumerate() {
        for c in &s.cells {
            for dy in -1..=1i32 {
                for dx in -1..=1i32 {
                    let n = (c.0 + dy, c.1 + dx);
                    if let Some(&j) = owner.get(&n)
                        && j != i
                    {
                        let (key, pair) = if i < j {
                            ((i, j), (*c, n))
                        } else {
                            ((j, i), (n, *c))
                        };
                        boundary.entry(key).or_default().push(pair);
                    }
                }
            }
        }
    }
    boundary
}

/// Per-track pass-class comparison across a join: (shared, mismatched)
/// aggregated over every adjacent cell pair on it.
fn join_usage_mismatch(pairs: &[(Cell, Cell)], coverage: &CoverageGrid) -> (usize, usize) {
    let mut shared = 0usize;
    let mut mismatch = 0usize;
    for &(ca, cb) in pairs {
        let (Some(pa), Some(pb)) = (coverage.cell_passes.get(&ca), coverage.cell_passes.get(&cb))
        else {
            continue;
        };
        for (t, ka) in pa {
            if let Some(kb) = pb.get(t) {
                shared += 1;
                if ka != kb {
                    mismatch += 1;
                }
            }
        }
    }
    (shared, mismatch)
}

/// A pass-class boundary counts only when the change is the experience
/// of MOST of the traffic at the join — a turnaround or loop mouth for
/// its users. A minority's turnaround (walkers turning back on a
/// corridor runners continue along) must not cut the majority's through
/// corridor.
fn is_usage_boundary(shared: usize, mismatch: usize) -> bool {
    shared > 0 && mismatch as f64 > (0.5 * shared as f64).max(1.0)
}

/// The departing traffic collected by the best section-worthy third
/// corridor at the join, when any collects enough to make the fork
/// real.
///
/// The test is on the LEAVERS: testing the through traffic instead
/// would let a collinear continuation of the same corridor justify the
/// cut — it shares the through traffic by definition — and chain
/// fragments would hold each other apart forever. The branch must
/// physically meet the join (direct adjacency only: a corridor two
/// cells away, reached by a stub too short to be a section, must not
/// arbitrate a cut it never visibly touches) and must carry enough
/// activities to be a section in its own right — on a corridor used by
/// three activities, one turning off is noise.
#[allow(clippy::too_many_arguments)]
fn branch_collecting_leavers(
    a: usize,
    b: usize,
    supernodes: &[Supernode],
    owner: &HashMap<Cell, usize>,
    pairs: &[(Cell, Cell)],
    section_worthy: &[bool],
    leavers: &HashSet<u32>,
    needed: f64,
) -> Option<usize> {
    let mut third: HashSet<usize> = HashSet::new();
    for &(ca, cb) in pairs {
        for c in [ca, cb] {
            for dy in -1..=1i32 {
                for dx in -1..=1i32 {
                    if let Some(&o) = owner.get(&(c.0 + dy, c.1 + dx))
                        && o != a
                        && o != b
                        && section_worthy[o]
                    {
                        third.insert(o);
                    }
                }
            }
        }
    }
    third
        .iter()
        .map(|&o| supernodes[o].tracks.intersection(leavers).count())
        .filter(|&n| n as f64 >= needed)
        .max()
}

/// Explain the boundaries that survived the merge fixed point: re-run
/// the keep tests read-only on the final components and record the
/// mechanism and the numbers for each. Decisions are made in
/// [`merge_non_fork_boundaries`]; this pass only reports them.
fn explain_boundaries(
    supernodes: &[Supernode],
    coverage: &CoverageGrid,
    divergence: f64,
    min_activities: u32,
    section_worthy: &[bool],
    records: &mut Vec<BoundaryRecord>,
) {
    let owner = cell_owners(supernodes);
    let boundary = pair_boundaries(supernodes, &owner);
    let mut joins: Vec<(usize, usize)> = boundary.keys().copied().collect();
    joins.sort_unstable();
    for (a, b) in joins {
        let pairs = &boundary[&(a, b)];
        let (lat, lng) = coverage.grid.centre_of(pairs[0].0);
        let (shared, mismatch) = join_usage_mismatch(pairs, coverage);
        if is_usage_boundary(shared, mismatch) {
            records.push(BoundaryRecord {
                latitude: lat,
                longitude: lng,
                reason: BoundaryReason::UsageChange {
                    shared: shared as u32,
                    mismatched: mismatch as u32,
                },
            });
            continue;
        }
        if !section_worthy[a] || !section_worthy[b] {
            continue;
        }
        let through: HashSet<u32> = supernodes[a]
            .tracks
            .intersection(&supernodes[b].tracks)
            .copied()
            .collect();
        if through.is_empty() {
            continue;
        }
        let leavers: HashSet<u32> = supernodes[a]
            .tracks
            .symmetric_difference(&supernodes[b].tracks)
            .copied()
            .collect();
        let needed = (divergence * through.len() as f64).max(min_activities as f64);
        if let Some(collected) = branch_collecting_leavers(
            a,
            b,
            supernodes,
            &owner,
            pairs,
            section_worthy,
            &leavers,
            needed,
        ) {
            records.push(BoundaryRecord {
                latitude: lat,
                longitude: lng,
                reason: BoundaryReason::Fork {
                    through: through.len() as u32,
                    needed,
                    branch_leavers: collected as u32,
                },
            });
        }
    }
}

/// Merge component pairs whose boundary is not a genuine fork.
///
/// A composition change only deserves to end a section when the athlete
/// can see why it ends there. Two reasons qualify: the departing
/// traffic had somewhere else to go (a fork with a section-worthy third
/// corridor), or the way the ground is used changes (a pass-class
/// boundary — a turnaround, a lollipop mouth, an oval entrance). Where
/// two components simply meet end-to-end with neither, the cut is
/// attrition (activities starting, stopping, or drifting in and out)
/// and the two are one continuous stretch.
fn merge_non_fork_boundaries(
    supernodes: Vec<Supernode>,
    coverage: &CoverageGrid,
    divergence: f64,
    min_activities: u32,
    section_worthy: &[bool],
) -> Vec<Supernode> {
    let owner = cell_owners(&supernodes);
    let boundary = pair_boundaries(&supernodes, &owner);
    let mut joins: Vec<(usize, usize)> = boundary.keys().copied().collect();
    joins.sort_unstable();

    let mut uf: UnionFind<usize> = UnionFind::with_capacity(supernodes.len());
    for i in 0..supernodes.len() {
        uf.make_set(i);
    }
    // Non-section fragments and their adjacent components, for
    // directed absorption after the pairwise decisions.
    let mut frag_neighbours: HashMap<usize, Vec<usize>> = HashMap::new();

    for (a, b) in joins {
        let pairs = &boundary[&(a, b)];
        let (shared, mismatch) = join_usage_mismatch(pairs, coverage);
        if is_usage_boundary(shared, mismatch) {
            continue;
        }

        // Traffic that actually continues through the join, and the
        // traffic that leaves at it.
        let through: HashSet<u32> = supernodes[a]
            .tracks
            .intersection(&supernodes[b].tracks)
            .copied()
            .collect();
        if through.is_empty() {
            continue;
        }
        // Fork arbitration is only meaningful between two components
        // that will both be sections. A fragment that cannot be one is
        // handled by directed absorption below — its under-sampled
        // composition must not fake a fork (a one-cell sliver shares
        // few tracks with its own corridor, so every through runner
        // looks like a leaver).
        if !section_worthy[a] || !section_worthy[b] {
            if !section_worthy[a] {
                frag_neighbours.entry(a).or_default().push(b);
            }
            if !section_worthy[b] {
                frag_neighbours.entry(b).or_default().push(a);
            }
            continue;
        }
        let leavers: HashSet<u32> = supernodes[a]
            .tracks
            .symmetric_difference(&supernodes[b].tracks)
            .copied()
            .collect();
        let needed = (divergence * through.len() as f64).max(min_activities as f64);
        let forked = branch_collecting_leavers(
            a,
            b,
            &supernodes,
            &owner,
            pairs,
            section_worthy,
            &leavers,
            needed,
        )
        .is_some();
        if !forked {
            uf.union(&a, &b);
        }
    }

    // Directed absorption: each fragment joins the ONE section-worthy
    // neighbour sharing most of its traffic. One neighbour only — a
    // junction apron touches every arm, and absorbing it into all of
    // them would fuse the junction itself. Pass-class-separated
    // neighbours never appear here (gated above), so a lollipop stem
    // still cannot fall into its loop.
    let mut frags: Vec<usize> = frag_neighbours.keys().copied().collect();
    frags.sort_unstable();
    for f in frags {
        let mut ns = frag_neighbours[&f].clone();
        ns.sort_unstable();
        ns.dedup();
        let best = ns
            .iter()
            .filter(|&&n| section_worthy[n])
            .map(|&n| {
                (
                    supernodes[f]
                        .tracks
                        .intersection(&supernodes[n].tracks)
                        .count(),
                    std::cmp::Reverse(n),
                )
            })
            .max()
            .filter(|&(shared_tracks, _)| shared_tracks > 0);
        if let Some((_, std::cmp::Reverse(n))) = best {
            uf.union(&f, &n);
        }
    }

    regroup(supernodes, &mut uf)
}

/// Fraction of a line's points that revisit ground it has already
/// travelled: a point within `near` metres of an earlier point first
/// reached at least `gap` arc-metres back, with the opening `gap`
/// metres exempt so a closed loop's join is not charged as a revisit.
///
/// A clean line or a single lap scores ~0; a mid-line spin or a
/// directionless junction blob scores high. This enforces the
/// single-pass rule on the rendered representative: the cell-event cut
/// in [`portions_for`]/[`simple_pass_range`] misses a spin too tight to
/// dwell between re-entries, so the render guards against it with this.
/// O(n): earlier points enter a `near`-sized spatial hash on a `gap`
/// arc-lag, and the opening `gap` never enters, so a query only meets
/// eligible earlier ground.
pub fn self_pass_penalty(pts: &[GpsPoint], near: f64, gap: f64) -> f64 {
    if pts.len() < 3 {
        return 0.0;
    }
    let m_lat = 111_132.0;
    let m_lng = 111_320.0 * pts[0].latitude.to_radians().cos();
    let xy: Vec<(f64, f64)> = pts
        .iter()
        .map(|p| (p.latitude * m_lat, p.longitude * m_lng))
        .collect();
    let mut cum = vec![0.0f64; pts.len()];
    for i in 1..pts.len() {
        let (dx, dy) = (xy[i].0 - xy[i - 1].0, xy[i].1 - xy[i - 1].1);
        cum[i] = cum[i - 1] + (dx * dx + dy * dy).sqrt();
    }
    if cum[pts.len() - 1] < gap {
        return 0.0;
    }
    let near2 = near * near;
    let key = |x: f64, y: f64| ((x / near).floor() as i32, (y / near).floor() as i32);
    let mut grid: HashMap<(i32, i32), Vec<(f64, f64)>> = HashMap::new();
    let mut lag = 0usize;
    let mut hits = 0usize;
    for i in 0..pts.len() {
        while lag < i && cum[i] - cum[lag] >= gap {
            if cum[lag] >= gap {
                let (x, y) = xy[lag];
                grid.entry(key(x, y)).or_default().push((x, y));
            }
            lag += 1;
        }
        let (x, y) = xy[i];
        let (cx, cy) = key(x, y);
        let revisit = (-1..=1).any(|dx| {
            (-1..=1).any(|dy| {
                grid.get(&(cx + dx, cy + dy)).is_some_and(|b| {
                    b.iter()
                        .any(|&(ex, ey)| (x - ex).powi(2) + (y - ey).powi(2) < near2)
                })
            })
        });
        if revisit {
            hits += 1;
        }
    }
    hits as f64 / pts.len() as f64
}

/// Share of a CLOSED line (endpoints within `CLOSE_FRAC` of its length)
/// that runs back over its own ground in the opposite direction: a
/// forward-and-reverse. Only closed lines are scored, so a switchback
/// climb (which climbs away, endpoints far apart) is never charged for
/// its antiparallel hairpin legs, and a clean loop scores ~0 (its
/// ground is travelled once, same sense). Complements
/// [`self_pass_penalty`], which needs an arc gap and so misses the short
/// spurs of an out-and-back that returns to its start.
fn out_and_back_penalty(pts: &[GpsPoint], near: f64) -> f64 {
    const CLOSE_FRAC: f64 = 0.2;
    if pts.len() < 5 {
        return 0.0;
    }
    let m_lat = 111_132.0;
    let m_lng = 111_320.0 * pts[0].latitude.to_radians().cos();
    let xy: Vec<(f64, f64)> = pts
        .iter()
        .map(|p| (p.latitude * m_lat, p.longitude * m_lng))
        .collect();
    let total: f64 = xy
        .windows(2)
        .map(|w| ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt())
        .sum();
    let last = xy[xy.len() - 1];
    let end = ((last.0 - xy[0].0).powi(2) + (last.1 - xy[0].1).powi(2)).sqrt();
    if total <= 0.0 || end > CLOSE_FRAC * total {
        return 0.0;
    }
    let hdg = |i: usize| {
        let a = xy[i.saturating_sub(1)];
        let b = xy[(i + 1).min(xy.len() - 1)];
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let n = (dx * dx + dy * dy).sqrt().max(1e-9);
        (dx / n, dy / n)
    };
    let hd: Vec<(f64, f64)> = (0..xy.len()).map(hdg).collect();
    let near2 = near * near;
    let key = |x: f64, y: f64| ((x / near).floor() as i32, (y / near).floor() as i32);
    let mut grid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (i, &(x, y)) in xy.iter().enumerate() {
        grid.entry(key(x, y)).or_default().push(i);
    }
    let mut hits = 0usize;
    for (i, &(x, y)) in xy.iter().enumerate() {
        let (cx, cy) = key(x, y);
        let anti = (-1..=1).any(|dx| {
            (-1..=1).any(|dy| {
                grid.get(&(cx + dx, cy + dy)).is_some_and(|b| {
                    b.iter().any(|&j| {
                        (i as i32 - j as i32).abs() >= 4
                            && (x - xy[j].0).powi(2) + (y - xy[j].1).powi(2) < near2
                            && hd[i].0 * hd[j].0 + hd[i].1 * hd[j].1 < -0.5
                    })
                })
            })
        });
        if anti {
            hits += 1;
        }
    }
    hits as f64 / xy.len() as f64
}

/// Every contributing track's single-pass portion through a component.
///
/// The run is found on the dilated cell set (jitter tolerance mid-run),
/// trimmed back to core cells so neighbouring sections don't annex each
/// other's halo, then reduced to a simple pass.
fn portions_for(
    node: &Supernode,
    coverage: &CoverageGrid,
    sport_tracks: &[(&str, &[GpsPoint])],
    config: &SectionConfig,
    cell_size: f64,
    tun: &Tunables,
) -> Vec<Portion> {
    let core: HashSet<Cell> = node.cells.iter().copied().collect();
    let mut cell_set: HashSet<Cell> = HashSet::with_capacity(node.cells.len() * 9);
    for c in &node.cells {
        for dy in -1..=1i32 {
            for dx in -1..=1i32 {
                cell_set.insert((c.0 + dy, c.1 + dx));
            }
        }
    }

    let pass_grid = CellGrid::new(cell_size / tun.pass_subgrid, coverage.ref_lat);
    // Canonical portion order: by activity id, never by arrival index.
    // The anchor and every tie-break downstream inherit this order, so
    // the catalogue stays a pure function of the activity set.
    let mut t_indices: Vec<u32> = node.tracks.iter().copied().collect();
    t_indices.sort_unstable_by(|&a, &b| sport_tracks[a as usize].0.cmp(sport_tracks[b as usize].0));

    let mut portions: Vec<Portion> = Vec::new();
    for &t_idx in &t_indices {
        let pts = sport_tracks[t_idx as usize].1;
        // Walk each lift-free range on its own: excluded ground is not
        // usable geometry even when the component touches both of its
        // ends (a base station and a summit do exactly that).
        let mut best: Option<(usize, usize, f64)> = None;
        for &(rs, re) in &coverage.keep[t_idx as usize] {
            if let Some((s, e, d)) = longest_run_in_cells(&pts[rs..=re], &cell_set, &coverage.grid)
                && best.as_ref().is_none_or(|b| d > b.2)
            {
                best = Some((rs + s, rs + e, d));
            }
        }
        let Some((mut s, mut e, _)) = best else {
            continue;
        };
        while s < e && !core.contains(&coverage.grid.cell_of(pts[s].latitude, pts[s].longitude)) {
            s += 1;
        }
        while e > s
            && !core.contains(
                &coverage
                    .grid
                    .cell_of(pts[e - 1].latitude, pts[e - 1].longitude),
            )
        {
            e -= 1;
        }
        if e <= s + 1 {
            continue;
        }
        // Single pass only: laps, return legs, and loop stems are
        // traversals, not section geometry.
        let (ps, pe) = simple_pass_range(&pts[s..e], &pass_grid, tun);
        let limit = e;
        e = s + pe;
        s += ps;
        if e <= s + 1 {
            continue;
        }
        let mut dist = crate::matching::calculate_route_distance(&pts[s..e]);
        // Closure snap: the cut lands on a coarse cell boundary, which
        // can stop a loop up to a cell short of actually closing. If
        // the pass already nearly closes, slide the end forward (within
        // ~1.5 cells of travel) to the true closest-return point. An
        // out-and-back's cut sits at the turnaround, nowhere near its
        // start, so this touches loops only.
        let gap = crate::geo_utils::haversine_distance(&pts[e - 1], &pts[s]);
        if gap < 2.0 * cell_size && dist > 4.0 * cell_size {
            let mut best = gap;
            let mut best_e = e;
            let mut travelled = 0.0;
            let mut j = e;
            while j < limit && travelled < 1.5 * cell_size {
                travelled += crate::geo_utils::haversine_distance(&pts[j - 1], &pts[j]);
                let d = crate::geo_utils::haversine_distance(&pts[j], &pts[s]);
                if d < best {
                    best = d;
                    best_e = j + 1;
                }
                j += 1;
            }
            if best_e > e {
                e = best_e;
                dist = crate::matching::calculate_route_distance(&pts[s..e]);
            }
        }
        if dist >= config.min_section_length && dist <= config.max_section_length {
            portions.push((t_idx as usize, s, e, dist));
        }
    }
    portions
}

/// Tracks passing within ~2 cells of the component: the population
/// that had the opportunity to traverse it. Support floors scale with
/// opportunity, not global corpus size — a trail only reachable by the
/// few who go there is not held to the whole corpus's bar, while busy
/// town ground is.
fn opportunity(node: &Supernode, coverage: &CoverageGrid) -> usize {
    let mut tracks: HashSet<u32> = HashSet::new();
    for c in &node.cells {
        for dy in -2..=2i32 {
            for dx in -2..=2i32 {
                if let Some(ts) = coverage.cell_tracks.get(&(c.0 + dy, c.1 + dx)) {
                    tracks.extend(ts.iter().copied());
                }
            }
        }
    }
    tracks.len()
}

/// Support test: enough activities traverse this stretch for it to be a
/// section, with the visit floor scaling by length and opportunity.
fn has_support(
    portions: &[Portion],
    fallback_len: f64,
    config: &SectionConfig,
    corpus: usize,
) -> bool {
    let mut lens: Vec<f64> = portions.iter().map(|p| p.3).collect();
    lens.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median_len = lens.get(lens.len() / 2).copied().unwrap_or(fallback_len);
    let required = required_visits_for_length(median_len, corpus) as usize;
    portions.len() >= required.max(config.min_activities as usize)
}

/// Distinct contributors with a qualifying pass over each cell of a
/// candidate's ground, one ring of tolerance so a co-traveller a braid
/// row over still lends the lane support. A pass is a maximal stretch
/// of a contributor's track inside the candidate's cell set at least
/// `pass_min_m` long in arc: a corner clip or a crossing lends no support,
/// while every lap and fragment of a genuine traversal does — a
/// contributor whose selected single pass is a fragment of its outing
/// must not read as absent from the rest of its loop. Ground a
/// contributor covers elsewhere in its outing, outside this candidate,
/// supports nothing: that is what lets a stranger-hot spur read as the
/// one-rider ground it is. Cost is one linear walk per contributor.
fn candidate_support(
    portions: &[Portion],
    cell_set: &HashSet<Cell>,
    coverage: &CoverageGrid,
    sport_tracks: &[(&str, &[GpsPoint])],
    pass_min_m: f64,
) -> HashMap<Cell, u32> {
    let contributors: HashSet<usize> = portions.iter().map(|p| p.0).collect();
    // Cell bounding box: most of a contributor's outing is nowhere near
    // the candidate, and four compares are far cheaper than a set probe.
    let (mut y0, mut y1, mut x0, mut x1) = (i32::MAX, i32::MIN, i32::MAX, i32::MIN);
    for c in cell_set {
        y0 = y0.min(c.0);
        y1 = y1.max(c.0);
        x0 = x0.min(c.1);
        x1 = x1.max(c.1);
    }
    let mut support: HashMap<Cell, u32> = HashMap::new();
    for &t in &contributors {
        let pts = sport_tracks[t].1;
        let mut touched: HashSet<Cell> = HashSet::new();
        for &(rs, re) in &coverage.keep[t] {
            let seg = &pts[rs..=re];
            let mut run_cells: Vec<Cell> = Vec::new();
            let mut run_m = 0.0_f64;
            let mut in_run = false;
            for (i, p) in seg.iter().enumerate() {
                let c = coverage.grid.cell_of(p.latitude, p.longitude);
                if c.0 >= y0 && c.0 <= y1 && c.1 >= x0 && c.1 <= x1 && cell_set.contains(&c) {
                    if in_run {
                        run_m += crate::geo_utils::haversine_distance(&seg[i - 1], p);
                    }
                    in_run = true;
                    if run_cells.last() != Some(&c) {
                        run_cells.push(c);
                    }
                } else {
                    if in_run && run_m >= pass_min_m {
                        touched.extend(run_cells.iter().copied());
                    }
                    run_cells.clear();
                    run_m = 0.0;
                    in_run = false;
                }
            }
            if in_run && run_m >= pass_min_m {
                touched.extend(run_cells.iter().copied());
            }
        }
        let dilated: HashSet<Cell> = touched
            .iter()
            .flat_map(|c| {
                (-1..=1i32).flat_map(move |dy| (-1..=1i32).map(move |dx| (c.0 + dy, c.1 + dx)))
            })
            .collect();
        for c in dilated {
            *support.entry(c).or_insert(0) += 1;
        }
    }
    support
}

/// Which components would survive as sections on their own evidence.
fn section_worthiness(
    supernodes: &[Supernode],
    coverage: &CoverageGrid,
    sport_tracks: &[(&str, &[GpsPoint])],
    config: &SectionConfig,
    cell_size: f64,
    tun: &Tunables,
) -> Vec<bool> {
    supernodes
        .iter()
        .map(|n| {
            let portions = portions_for(n, coverage, sport_tracks, config, cell_size, tun);
            has_support(
                &portions,
                n.cells.len() as f64 * cell_size,
                config,
                opportunity(n, coverage),
            )
        })
        .collect()
}

/// Geographic clusters of one sport's tracks: connected components of
/// bounding boxes padded by half [`Tunables::cluster_gap_m`], so any
/// pair that could ever share evidence lands together while far-apart
/// regions each get their own projection plane. The partition is a pure
/// function of the activity set (box connectivity ignores input order)
/// and clusters are ordered by their south-west corner, so section
/// numbering stays deterministic. A track spanning two regions bridges
/// them into one cluster by construction.
fn geo_clusters(tracks: &[(&str, &[GpsPoint])], gap_m: f64) -> Vec<Vec<usize>> {
    let boxes: Vec<Option<(f64, f64, f64, f64)>> = tracks
        .iter()
        .map(|(_, pts)| {
            if pts.is_empty() {
                return None;
            }
            let mut bb = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for p in pts.iter() {
                bb.0 = bb.0.min(p.latitude);
                bb.1 = bb.1.max(p.latitude);
                bb.2 = bb.2.min(p.longitude);
                bb.3 = bb.3.max(p.longitude);
            }
            Some(bb)
        })
        .collect();
    let pad_lat = gap_m * 0.5 / 111_132.0;
    let padded: Vec<Option<(f64, f64, f64, f64)>> = boxes
        .iter()
        .map(|b| {
            b.map(|(lat0, lat1, lng0, lng1)| {
                let mid = ((lat0 + lat1) * 0.5).to_radians();
                let pad_lng = gap_m * 0.5 / (111_320.0 * mid.cos().abs().max(0.01));
                (
                    lat0 - pad_lat,
                    lat1 + pad_lat,
                    lng0 - pad_lng,
                    lng1 + pad_lng,
                )
            })
        })
        .collect();

    let mut uf: crate::union_find::UnionFind<usize> = crate::union_find::UnionFind::new();
    for i in 0..tracks.len() {
        uf.make_set(i);
    }
    for (i, a) in padded.iter().enumerate() {
        let Some(a) = a else { continue };
        for (j, b) in padded.iter().enumerate().skip(i + 1) {
            let Some(b) = b else { continue };
            if a.0 <= b.1 && b.0 <= a.1 && a.2 <= b.3 && b.2 <= a.3 {
                uf.union(&i, &j);
            }
        }
    }

    let mut by_root: HashMap<usize, Vec<usize>> = HashMap::new();
    for (i, b) in boxes.iter().enumerate() {
        if b.is_some() {
            by_root.entry(uf.find(&i)).or_default().push(i);
        }
    }
    let mut keyed: Vec<((f64, f64), Vec<usize>)> = by_root
        .into_values()
        .map(|members| {
            let mut sw = (f64::MAX, f64::MAX);
            for &i in &members {
                let bb = boxes[i].expect("cluster members have boxes");
                sw.0 = sw.0.min(bb.0);
                sw.1 = sw.1.min(bb.2);
            }
            (sw, members)
        })
        .collect();
    keyed.sort_by(|a, b| a.0.0.total_cmp(&b.0.0).then(a.0.1.total_cmp(&b.0.1)));
    keyed.into_iter().map(|(_, members)| members).collect()
}

/// Per-point representation mask for the selection backoff: a probe
/// point is represented when accepted geometry runs within a cell's
/// width in plan. Deliberately planimetric: plan cells are atomic in
/// the partition, so truly stacked ground (a balcony path directly
/// over a road) shares its cells and never surfaces as a separate
/// candidate here, while comparing absolute elevations across
/// activities recorded on different days is barometric-drift noise
/// that makes every trim decision flicker.
fn probe_mask(
    probe: &[GpsPoint],
    accepted: &HashMap<Cell, Vec<GpsPoint>>,
    grid: &CellGrid,
    cell_size: f64,
) -> Vec<bool> {
    probe
        .iter()
        .map(|p| {
            let c = grid.cell_of(p.latitude, p.longitude);
            (-1..=1i32).any(|dy| {
                (-1..=1i32).any(|dx| {
                    accepted.get(&(c.0 + dy, c.1 + dx)).is_some_and(|v| {
                        v.iter()
                            .any(|q| crate::geo_utils::haversine_distance(p, q) < cell_size)
                    })
                })
            })
        })
        .collect()
}

/// Maximal unrepresented runs of a probe, as half-open index ranges.
/// Represented islands shorter than `bridge_m` of travel are absorbed
/// into the surrounding run: crossing the accepted line is incidental
/// contact, not duplication.
fn unrepresented_runs(mask: &[bool], cum: &[f64], bridge_m: f64) -> Vec<(usize, usize)> {
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut i = 0usize;
    while i < mask.len() {
        if mask[i] {
            i += 1;
            continue;
        }
        let s = i;
        while i < mask.len() && !mask[i] {
            i += 1;
        }
        match runs.last_mut() {
            Some(last) if cum[s] - cum[last.1 - 1] < bridge_m => last.1 = i,
            _ => runs.push((s, i)),
        }
    }
    runs
}

/// Rule 5 corollary: sections that tile one physical line prefer
/// geometry cut from ONE covering activity, so a chain renders as
/// consecutive ranges of a single real trace and its joints meet on a
/// shared trace point instead of splices from different days.
/// Boundaries, visits, evidence, and — critically — each member's SPAN
/// are untouched: the covering trace is cut where the member's own
/// medoid extent projects onto it, so only the source of the pixels
/// changes and jackknife extent stability is preserved (re-deriving
/// spans from the cover's portions sheared extents and cost ~20 points
/// of core persistence when probed). Assignment is a deterministic
/// greedy cover: the activity with a portion in the most
/// still-unassigned members wins (ties: most metres it already
/// represents, most portion metres, then activity id), so a Y-joint
/// unifies each limb pair that a real pass actually connects and never
/// invents a through-line no one rode.
fn unify_chain_references(
    sections: &mut [FrequentSection],
    portions: &[Vec<Portion>],
    sport_tracks: &[(&str, &[GpsPoint])],
    cell_size: f64,
    divergence: f64,
) {
    let n = sections.len();
    if n < 2 {
        return;
    }
    let link_tol = 2.0 * cell_size;
    let ends: Vec<Option<[GpsPoint; 2]>> = sections
        .iter()
        .map(|s| match (s.polyline.first(), s.polyline.last()) {
            (Some(a), Some(b)) => Some([*a, *b]),
            _ => None,
        })
        .collect();

    let mut uf: UnionFind<usize> = UnionFind::with_capacity(n);
    for i in 0..n {
        uf.make_set(i);
    }
    let mut links: Vec<(usize, usize)> = Vec::new();
    for (i, ea) in ends.iter().enumerate() {
        let Some(ea) = ea else { continue };
        for (j, eb) in ends.iter().enumerate().skip(i + 1) {
            let Some(eb) = eb else { continue };
            let close = ea.iter().any(|a| {
                eb.iter()
                    .any(|b| crate::geo_utils::haversine_distance(a, b) <= link_tol)
            });
            if close {
                links.push((i, j));
                uf.union(&i, &j);
            }
        }
    }
    if links.is_empty() {
        return;
    }
    let mut components: Vec<Vec<usize>> = uf.groups().into_values().collect();
    components.sort();

    // member → (track index, start, end) once a covering activity wins it
    let mut chosen: HashMap<usize, (usize, usize, usize)> = HashMap::new();
    for members in &components {
        if members.len() < 2 {
            continue;
        }
        let mut cand: HashMap<usize, Vec<usize>> = HashMap::new();
        for &m in members {
            for &(t, ..) in &portions[m] {
                cand.entry(t).or_default().push(m);
            }
        }
        let mut ts: Vec<usize> = cand.keys().copied().collect();
        ts.sort_by(|&a, &b| sport_tracks[a].0.cmp(sport_tracks[b].0));
        let mut unassigned: HashSet<usize> = members.iter().copied().collect();
        let mut banned: HashSet<usize> = HashSet::new();
        loop {
            let mut pick: Option<(usize, f64, f64, usize)> = None;
            for &t in &ts {
                if banned.contains(&t) {
                    continue;
                }
                let ms: Vec<usize> = cand[&t]
                    .iter()
                    .copied()
                    .filter(|m| unassigned.contains(m))
                    .collect();
                if ms.len() < 2 {
                    continue;
                }
                let rep_m: f64 = ms
                    .iter()
                    .filter(|&&m| sections[m].representative_activity_id == sport_tracks[t].0)
                    .map(|&m| sections[m].distance_meters)
                    .sum();
                let tot_m: f64 = ms
                    .iter()
                    .map(|&m| {
                        portions[m]
                            .iter()
                            .find(|p| p.0 == t)
                            .map(|p| p.3)
                            .unwrap_or(0.0)
                    })
                    .sum();
                let better = match pick {
                    None => true,
                    Some((c, r, tm, _)) => {
                        ms.len() > c || (ms.len() == c && (rep_m > r || (rep_m == r && tot_m > tm)))
                    }
                };
                if better {
                    pick = Some((ms.len(), rep_m, tot_m, t));
                }
            }
            let Some((_, _, _, t)) = pick else { break };
            // Extent-preserving re-cut: the member keeps the span its
            // medoid machinery chose (that span is what jackknife
            // stability rests on); only the source trace changes.
            // Project the current polyline's ends onto t's portion and
            // cut there. The cover must run the member's own line, not
            // a braid twin or a shortcut through the same corridor:
            // both ends and the whole body must sit within half a cell
            // of the cut, or the member keeps its own reference.
            let half = 0.5 * cell_size;
            let mut assigned_any = false;
            for &m in &cand[&t] {
                if !unassigned.contains(&m) {
                    continue;
                }
                let Some(&(_, s, e, _)) = portions[m].iter().find(|p| p.0 == t) else {
                    continue;
                };
                let pts = sport_tracks[t].1;
                let g = &sections[m].polyline;
                let (Some(gf), Some(gl)) = (g.first(), g.last()) else {
                    continue;
                };
                let nearest = |target: &GpsPoint| -> (usize, f64) {
                    let mut best = (s, f64::INFINITY);
                    for (k, p) in pts.iter().enumerate().take(e).skip(s) {
                        let d = crate::geo_utils::haversine_distance(p, target);
                        if d < best.1 {
                            best = (k, d);
                        }
                    }
                    best
                };
                let (i0, d0) = nearest(gf);
                let (i1, d1) = nearest(gl);
                if d0 > half || d1 > half {
                    continue;
                }
                let (lo, hi) = if i0 <= i1 { (i0, i1) } else { (i1, i0) };
                if hi < lo + 1 {
                    continue;
                }
                let cut = &pts[lo..=hi];
                let off_line = g
                    .iter()
                    .step_by(3)
                    .filter(|p| {
                        !cut.iter()
                            .any(|q| crate::geo_utils::haversine_distance(p, q) < half)
                    })
                    .count();
                if off_line * 20 > g.len().div_ceil(3) {
                    continue;
                }
                // A shallow shortcut can hide inside the lateral
                // envelope (a hairpin bulge adds path without leaving
                // it); the cut must also match the member's length
                // within the divergence share.
                let cut_m = crate::matching::calculate_route_distance(cut);
                if (cut_m - sections[m].distance_meters).abs()
                    > divergence * sections[m].distance_meters
                {
                    continue;
                }
                chosen.insert(m, (t, lo, hi + 1));
                unassigned.remove(&m);
                assigned_any = true;
            }
            if !assigned_any {
                banned.insert(t);
            }
        }
    }
    if chosen.is_empty() {
        return;
    }

    // Joint snap: linked pairs now cut from the same trace meet at a
    // shared trace point. A gap wider than the link tolerance is real
    // unrepresented ground and stays open.
    for &(i, j) in &links {
        let (Some(&(ti, si, ei)), Some(&(tj, sj, ej))) = (chosen.get(&i), chosen.get(&j)) else {
            continue;
        };
        if ti != tj {
            continue;
        }
        let pts = sport_tracks[ti].1;
        let (ma, sa, ea, mb, sb, eb) = if si <= sj {
            (i, si, ei, j, sj, ej)
        } else {
            (j, sj, ej, i, si, ei)
        };
        if ea < sb {
            let gap_m: f64 = (ea..=sb)
                .map(|k| crate::geo_utils::haversine_distance(&pts[k - 1], &pts[k]))
                .sum();
            if gap_m > link_tol {
                continue;
            }
        }
        let mid = (ea + sb) / 2;
        if mid <= sa || mid + 2 > eb {
            continue;
        }
        chosen.insert(ma, (ti, sa, mid + 1));
        chosen.insert(mb, (ti, mid, eb));
    }

    for (&m, &(t, s, e)) in &chosen {
        let pts = sport_tracks[t].1;
        let e = e.min(pts.len());
        if e < s + 2 {
            continue;
        }
        sections[m].polyline = pts[s..e].to_vec();
        sections[m].distance_meters = crate::matching::calculate_route_distance(&pts[s..e]);
        sections[m].representative_activity_id = sport_tracks[t].0.to_string();
    }
}

/// Detect sections for one sport's tracks via the unified pipeline.
///
/// Tracks split into geographic clusters first and the pipeline runs
/// per cluster on the cluster's own reference latitude. One global
/// plane sizes east-west cells by cos(reference)/cos(local) — ~14%
/// wrong for a corpus spanning Valais and Melbourne — and lets ground
/// on another continent shift every cell boundary at home; local
/// projection removes both, and makes each region's catalogue
/// independent of the rest of the corpus.
fn detect_for_sport(
    sport: &str,
    sport_tracks: &[(&str, &[GpsPoint])],
    sport_seconds: &[&[f64]],
    config: &SectionConfig,
    tun: &Tunables,
    section_idx: &mut usize,
    records: &mut Vec<BoundaryRecord>,
) -> Vec<FrequentSection> {
    if sport_tracks.len() < config.min_activities as usize {
        return Vec::new();
    }
    let mut sections = Vec::new();
    for cluster in geo_clusters(sport_tracks, tun.cluster_gap_m) {
        let c_tracks: Vec<(&str, &[GpsPoint])> = cluster.iter().map(|&i| sport_tracks[i]).collect();
        let c_seconds: Vec<&[f64]> = cluster
            .iter()
            .map(|&i| sport_seconds.get(i).copied().unwrap_or(&[]))
            .collect();
        sections.extend(detect_for_cluster(
            sport,
            &c_tracks,
            &c_seconds,
            config,
            tun,
            section_idx,
            records,
        ));
    }
    sections
}

/// One geographic cluster's pipeline: coverage grid → same-traffic
/// supernodes → fixed point → candidates → selection backoff.
fn detect_for_cluster(
    sport: &str,
    sport_tracks: &[(&str, &[GpsPoint])],
    sport_seconds: &[&[f64]],
    config: &SectionConfig,
    tun: &Tunables,
    section_idx: &mut usize,
    records: &mut Vec<BoundaryRecord>,
) -> Vec<FrequentSection> {
    if sport_tracks.len() < config.min_activities as usize {
        return Vec::new();
    }

    let cell_size = (config.proximity_threshold * 0.5).clamp(50.0, 150.0);
    let coverage = build_coverage_grid(sport_tracks, sport_seconds, cell_size, tun);

    // Hot cells: adaptive floor, never an absolute constant.
    let hot_min = (config.min_activities as usize).max(2);
    let mut hot_cells: Vec<Cell> = coverage
        .cell_tracks
        .iter()
        .filter(|(_, ts)| ts.len() >= hot_min)
        .map(|(c, _)| *c)
        .collect();
    hot_cells.sort_unstable();

    if hot_cells.is_empty() {
        return Vec::new();
    }

    let divergence = config.divergence_threshold.clamp(0.05, 0.5);
    let same_traffic = 1.0 - divergence;

    let mut supernodes = partition_supernodes(&hot_cells, &coverage, same_traffic);

    // Two rules applied to a fixed point:
    //  * a component that cannot be a section is not a barrier — absorb it
    //  * a boundary only survives where the diverging branch is itself a
    //    section. A cut whose branch never surfaces is invisible on the
    //    map: the corridor stops for no reason the athlete can see.
    // Absorbing and merging can promote a component, so iterate.
    let mut worthy =
        section_worthiness(&supernodes, &coverage, sport_tracks, config, cell_size, tun);
    for _ in 0..5 {
        let before = supernodes.len();
        supernodes = merge_non_fork_boundaries(
            supernodes,
            &coverage,
            divergence,
            config.min_activities,
            &worthy,
        );
        if supernodes.len() == before {
            // Nothing merged, so `worthy` still describes exactly these
            // components; the explanation pass below reuses it.
            break;
        }
        worthy = section_worthiness(&supernodes, &coverage, sport_tracks, config, cell_size, tun);
    }

    let mut sn_sizes: Vec<usize> = supernodes.iter().map(|s| s.cells.len()).collect();
    sn_sizes.sort_unstable();
    info!(
        "[Unified] {}: {} hot cells → {} supernodes (median {} cells, max {})",
        sport,
        hot_cells.len(),
        supernodes.len(),
        sn_sizes.get(sn_sizes.len() / 2).copied().unwrap_or(0),
        sn_sizes.last().copied().unwrap_or(0),
    );

    // Every boundary that survived explains itself, as data.
    explain_boundaries(
        &supernodes,
        &coverage,
        divergence,
        config.min_activities,
        &worthy,
        records,
    );

    let track_map: HashMap<&str, &[GpsPoint]> =
        sport_tracks.iter().map(|(id, pts)| (*id, *pts)).collect();
    let activity_to_route: HashMap<&str, &str> = HashMap::new();

    let mut sections: Vec<FrequentSection> = Vec::new();

    // Candidates that could stand as sections, scored by the real usage
    // they represent (total portion metres).
    let mut candidates: Vec<(usize, Supernode, Vec<Portion>, f64)> = Vec::new();
    for (n_idx, node) in supernodes.iter().enumerate() {
        // Rough length from core cell count (cells are ~square).
        let approx_len = node.cells.len() as f64 * cell_size;
        if approx_len < config.min_section_length {
            continue;
        }

        let mut portions = portions_for(node, &coverage, sport_tracks, config, cell_size, tun);
        if portions.is_empty()
            || !has_support(&portions, approx_len, config, opportunity(node, &coverage))
        {
            continue;
        }

        // Pre-trim score: ordering must not depend on the trim below,
        // or a trimmed candidate sinks in the backoff queue and every
        // later candidate's represented-ground trim reshuffles. The
        // trim corrects a candidate's extent, never its priority.
        let score: f64 = portions.iter().map(|p| p.3).sum();

        // Support binds along the length, not just in total (rule 5): a
        // stretch of a candidate's ground traversed by fewer of its OWN
        // contributors than the floor is not this section's ground,
        // however hot stranger traffic made its cells. Without this, a
        // near-private spur welds onto a busy corridor through the
        // traffic gradient at their junction (adjacent thin cells always
        // pass the one-missing-track rule), inherits the corridor's
        // visit count, and can be rendered. The floor is the same bar
        // the whole section must meet in [`has_support`], fixed from
        // the pre-trim portions: recomputing it as portions shorten
        // would ratchet the length tier stricter each round and spiral
        // a legitimate short section to death. A qualifying pass must
        // cover two cells of arc, scaled down for candidates shorter
        // than that so a genuine pass over a short section still
        // counts. Cells and portions only ever shrink, so the loop
        // terminates.
        let mut lens: Vec<f64> = portions.iter().map(|p| p.3).collect();
        lens.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median_len = lens[lens.len() / 2];
        let required = required_visits_for_length(median_len, opportunity(node, &coverage)) as usize;
        let floor = required.max(config.min_activities as usize) as u32;
        let pass_min_m = (2.0 * cell_size).min(0.5 * median_len);
        let mut node = Supernode {
            cells: node.cells.clone(),
            tracks: node.tracks.clone(),
        };
        loop {
            let cell_set: HashSet<Cell> = node
                .cells
                .iter()
                .flat_map(|c| {
                    (-1..=1i32).flat_map(move |dy| (-1..=1i32).map(move |dx| (c.0 + dy, c.1 + dx)))
                })
                .collect();
            let support =
                candidate_support(&portions, &cell_set, &coverage, sport_tracks, pass_min_m);
            let kept: Vec<Cell> = node
                .cells
                .iter()
                .copied()
                .filter(|c| support.get(c).copied().unwrap_or(0) >= floor)
                .collect();
            if kept.len() == node.cells.len() {
                break;
            }
            let kept_set: HashSet<Cell> = kept.iter().copied().collect();
            let dropped: Vec<Cell> = node
                .cells
                .iter()
                .copied()
                .filter(|c| !kept_set.contains(c))
                .collect();
            let (lat, lng) = dropped.iter().fold((0.0, 0.0), |(la, ln), c| {
                let (cla, cln) = coverage.grid.centre_of(*c);
                (la + cla, ln + cln)
            });
            records.push(BoundaryRecord {
                latitude: lat / dropped.len() as f64,
                longitude: lng / dropped.len() as f64,
                reason: BoundaryReason::LowSupport {
                    floor,
                    dropped_cells: dropped.len() as u32,
                },
            });
            if kept.is_empty() {
                portions.clear();
                break;
            }
            node.cells = kept;
            portions = portions_for(&node, &coverage, sport_tracks, config, cell_size, tun);
            if portions.is_empty() {
                break;
            }
        }
        let approx_len = node.cells.len() as f64 * cell_size;
        if portions.is_empty()
            || approx_len < config.min_section_length
            || !has_support(&portions, approx_len, config, opportunity(&node, &coverage))
        {
            continue;
        }
        candidates.push((n_idx, node, portions, score));
    }
    candidates.sort_by(|a, b| {
        b.3.partial_cmp(&a.3)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.0.cmp(&b.0))
    });

    let backoff_grid = CellGrid::new(cell_size, coverage.ref_lat);
    let mut accepted_pts: HashMap<Cell, Vec<GpsPoint>> = HashMap::new();
    let mut emitted_portions: Vec<Vec<Portion>> = Vec::new();
    for (_, node, portions, score) in candidates {
        // Selection backoff (rule 6): ground already represented by an
        // accepted polyline is never re-emitted. The probe walks the
        // candidate's best portion; representation means accepted
        // geometry within a cell's width (braid width, proximity/2) in
        // plan at the same elevation level — a balcony path above an
        // accepted road is distinct ground, not a duplicate. A wholly
        // represented candidate backs off; a partly represented one is
        // trimmed to its longest unrepresented run and stands only if
        // the remnant still qualifies on its own. Geometry against
        // polylines, not cell blobs: a braid twin hugs the winner
        // point for point, while a loop or variant beside a corridor
        // swings wide of it and keeps its own distinct shape.
        let Some(&(pt_idx, ps, pe, _)) = portions
            .iter()
            .max_by(|a, b| a.3.partial_cmp(&b.3).unwrap_or(std::cmp::Ordering::Equal))
        else {
            continue;
        };
        let probe = &sport_tracks[pt_idx].1[ps..pe];
        let mask = probe_mask(probe, &accepted_pts, &backoff_grid, cell_size);
        let near = mask.iter().filter(|&&m| m).count();
        let (portions, approx_len, was_trimmed) = if near == 0 {
            (portions, node.cells.len() as f64 * cell_size, false)
        } else {
            let mut cum = Vec::with_capacity(probe.len());
            let mut acc = 0.0;
            cum.push(0.0);
            for w in probe.windows(2) {
                acc += crate::geo_utils::haversine_distance(&w[0], &w[1]);
                cum.push(acc);
            }
            let runs = unrepresented_runs(&mask, &cum, cell_size);
            let kept = runs
                .iter()
                .copied()
                .max_by(|&(s0, e0), &(s1, e1)| {
                    (cum[e0 - 1] - cum[s0])
                        .partial_cmp(&(cum[e1 - 1] - cum[s1]))
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .filter(|&(s, e)| cum[e - 1] - cum[s] >= config.min_section_length);
            let Some((rs, re)) = kept else {
                let mid = probe[probe.len() / 2];
                records.push(BoundaryRecord {
                    latitude: mid.latitude,
                    longitude: mid.longitude,
                    reason: BoundaryReason::Backoff {
                        represented: near as u32,
                        probed: mask.len() as u32,
                        score_metres: score,
                    },
                });
                continue;
            };
            if re - rs == probe.len() {
                (portions, node.cells.len() as f64 * cell_size, false)
            } else {
                // One ring of dilation keeps the corridor's full lane
                // width: the probe is a single track's line, and
                // co-travellers one braid row over must not fall out of
                // the reduced core (that collapses their portions and
                // kills support for ground that was never represented).
                let kept_cells: HashSet<Cell> = probe[rs..re]
                    .iter()
                    .flat_map(|p| {
                        let c = coverage.grid.cell_of(p.latitude, p.longitude);
                        (-1..=1i32)
                            .flat_map(move |dy| (-1..=1i32).map(move |dx| (c.0 + dy, c.1 + dx)))
                    })
                    .collect();
                let reduced = Supernode {
                    cells: node
                        .cells
                        .iter()
                        .copied()
                        .filter(|c| kept_cells.contains(c))
                        .collect(),
                    tracks: node.tracks.clone(),
                };
                let approx = reduced.cells.len() as f64 * cell_size;
                let trimmed =
                    portions_for(&reduced, &coverage, sport_tracks, config, cell_size, tun);
                if reduced.cells.is_empty()
                    || trimmed.is_empty()
                    || !has_support(&trimmed, approx, config, opportunity(&reduced, &coverage))
                {
                    let mid = probe[probe.len() / 2];
                    records.push(BoundaryRecord {
                        latitude: mid.latitude,
                        longitude: mid.longitude,
                        reason: BoundaryReason::Backoff {
                            represented: near as u32,
                            probed: mask.len() as u32,
                            score_metres: score,
                        },
                    });
                    continue;
                }
                let kept_m = cum[re - 1] - cum[rs];
                for cut in [(rs > 0).then_some(rs), (re < probe.len()).then_some(re - 1)]
                    .into_iter()
                    .flatten()
                {
                    records.push(BoundaryRecord {
                        latitude: probe[cut].latitude,
                        longitude: probe[cut].longitude,
                        reason: BoundaryReason::Trim {
                            kept_metres: kept_m,
                            dropped_metres: acc - kept_m,
                        },
                    });
                }
                (trimmed, approx, true)
            }
        };

        // Synthesise an OverlapCluster (anchor-paired) for process_cluster.
        let anchor = portions[0];
        let anchor_id = sport_tracks[anchor.0].0.to_string();
        let anchor_range = (anchor.1, anchor.2);
        let anchor_pts =
            &sport_tracks[anchor.0].1[anchor.1..anchor.2.min(sport_tracks[anchor.0].1.len())];
        let center = if anchor_pts.is_empty() {
            GpsPoint::new(0.0, 0.0)
        } else {
            anchor_pts[anchor_pts.len() / 2]
        };

        let mut overlaps: Vec<FullTrackOverlap> = Vec::with_capacity(portions.len());
        let mut activity_ids: HashSet<String> = HashSet::with_capacity(portions.len());
        activity_ids.insert(anchor_id.clone());
        for &(t_idx, s, e, dist) in portions.iter().skip(1) {
            let other_id = sport_tracks[t_idx].0.to_string();
            activity_ids.insert(other_id.clone());
            overlaps.push(FullTrackOverlap {
                activity_a: anchor_id.clone(),
                activity_b: other_id,
                range_a: anchor_range,
                range_b: (s, e),
                center,
                overlap_length: dist.min(anchor.3),
            });
        }
        if overlaps.is_empty() {
            overlaps.push(FullTrackOverlap {
                activity_a: anchor_id.clone(),
                activity_b: anchor_id,
                range_a: anchor_range,
                range_b: anchor_range,
                center,
                overlap_length: anchor.3,
            });
        }

        let cluster = OverlapCluster {
            overlaps,
            activity_ids,
        };

        if let Some(mut section) = process_cluster(
            *section_idx,
            cluster,
            sport,
            &track_map,
            &activity_to_route,
            config,
            None,
        ) {
            // The polyline must be a real trace, never a synthetic
            // average: replace the consensus geometry with a real
            // activity's actual portion. Consensus results stay as
            // metadata (confidence, spread, density). The app's
            // reference-activity model (set reference, trim/extend)
            // builds on exactly this guarantee.
            //
            // Rule 6 binds on the render: a section's line is ONE real
            // single pass. portions_for cuts laps and returns by cell
            // events, but a spin too tight to dwell between re-entries
            // slips through, a forward-and-reverse returns over its own
            // ground, and a junction where every visit mills has no
            // single traversal at all. The default render (longest
            // portion when trimmed, else the medoid: remnant portions
            // mix full coverers with corner clips, and the medoid's
            // average-minimum-distance has a subset bias — a short
            // central fragment is near every longer trace) stands when
            // it is a single pass; above the floor the cleanest
            // contributing pass is rendered instead, and a candidate
            // with no single pass backs off as a blob. Longest is the
            // stable pick under jackknife (median and closest-to-remnant
            // were probed and both jitter); visits still count everyone.
            let near = cell_size * 0.2;
            let gap = cell_size;
            let pens: Vec<f64> = portions
                .iter()
                .map(|&(t, s, e, _)| {
                    let seg = &sport_tracks[t].1[s..e];
                    self_pass_penalty(seg, near, gap).max(out_and_back_penalty(seg, near))
                })
                .collect();
            let default_i = if was_trimmed {
                (0..portions.len()).max_by(|&a, &b| {
                    portions[a]
                        .3
                        .partial_cmp(&portions[b].3)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
            } else {
                portions
                    .iter()
                    .position(|&(t, ..)| sport_tracks[t].0 == section.representative_activity_id)
            };
            // A section OCCUPIES its represented ground (the default,
            // longest-or-medoid portion) for the trim of later candidates,
            // independent of which cleaner strand it DISPLAYS. Filling the
            // trim grid from a shorter rendered strand instead lets a
            // neighbour re-expand into the freed ground and inherit its
            // milling: that is how a junction's noise migrated into an
            // innocent neighbour when the junction re-rendered short.
            let footprint: Vec<GpsPoint> = default_i
                .map(|di| {
                    let (t, s, e, _) = portions[di];
                    sport_tracks[t].1[s..e].to_vec()
                })
                .unwrap_or_else(|| section.polyline.clone());
            let cleanest_i = (0..portions.len()).min_by(|&a, &b| {
                pens[a]
                    .partial_cmp(&pens[b])
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then(
                        portions[b]
                            .3
                            .partial_cmp(&portions[a].3)
                            .unwrap_or(std::cmp::Ordering::Equal),
                    )
            });
            // The default render (longest/medoid) stands when it is a
            // single pass; above the floor the cleanest contributing pass
            // is displayed instead, and a candidate whose cleanest pass is
            // still over the floor backs off as a blob.
            let chosen = match default_i {
                Some(i) if pens[i] <= tun.self_pass_max => Some(i),
                _ => cleanest_i.filter(|&i| pens[i] <= tun.self_pass_max),
            };
            let Some(i) = chosen else {
                // No pass is a single traversal: the candidate backs off,
                // but its footprint still occupies the ground so a
                // neighbour cannot re-expand into a junction with no line.
                for p in footprint.iter().step_by(3) {
                    accepted_pts
                        .entry(backoff_grid.cell_of(p.latitude, p.longitude))
                        .or_default()
                        .push(*p);
                }
                let best = pens.iter().copied().fold(1.0_f64, f64::min);
                let mid = section.polyline[section.polyline.len() / 2];
                records.push(BoundaryRecord {
                    latitude: mid.latitude,
                    longitude: mid.longitude,
                    reason: BoundaryReason::NoSinglePass {
                        best_penalty: best,
                        portions: portions.len() as u32,
                    },
                });
                continue;
            };
            let (t_idx, s, e, dist) = portions[i];
            section.polyline = sport_tracks[t_idx].1[s..e].to_vec();
            section.distance_meters = dist;
            section.representative_activity_id = sport_tracks[t_idx].0.to_string();
            info!(
                "[Unified]   node cell0={:?} cells={} approx_len={:.0} portions={} → {} len={:.0} visits={}",
                node.cells[0],
                node.cells.len(),
                approx_len,
                portions.len(),
                section.id,
                section.distance_meters,
                section.visit_count,
            );
            *section_idx += 1;
            for p in footprint.iter().step_by(3) {
                accepted_pts
                    .entry(backoff_grid.cell_of(p.latitude, p.longitude))
                    .or_default()
                    .push(*p);
            }
            sections.push(section);
            emitted_portions.push(portions);
        }
    }

    unify_chain_references(
        &mut sections,
        &emitted_portions,
        sport_tracks,
        cell_size,
        divergence,
    );

    sections
}

/// Detect sections using the unified pipeline: coverage grid →
/// same-traffic supernodes → medoid + consensus geometry.
///
/// `seconds` carries each track's per-point time offsets, parallel to
/// `tracks`; it feeds only the lift velocity veto. Pass `&[]` when time
/// streams are unavailable and the lift rule rests on geometry alone.
pub fn detect_sections_unified(
    tracks: &[(String, Vec<GpsPoint>)],
    seconds: &[&[f64]],
    sport_types: &HashMap<String, String>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    detect_sections_unified_tuned(tracks, seconds, sport_types, config, &Tunables::DEFAULT)
}

/// [`detect_sections_unified`] with explicit [`Tunables`]. The
/// validation lab's plateau sweeps run through here; production never
/// passes anything but [`Tunables::DEFAULT`].
pub fn detect_sections_unified_tuned(
    tracks: &[(String, Vec<GpsPoint>)],
    seconds: &[&[f64]],
    sport_types: &HashMap<String, String>,
    config: &SectionConfig,
    tun: &Tunables,
) -> Vec<FrequentSection> {
    detect_sections_unified_explained(tracks, seconds, sport_types, config, tun).sections
}

/// [`detect_sections_unified`] carrying its boundary records beside the
/// sections: why each surviving cut exists and which candidates backed
/// off, with the numbers that decided it.
pub fn detect_sections_unified_explained(
    tracks: &[(String, Vec<GpsPoint>)],
    seconds: &[&[f64]],
    sport_types: &HashMap<String, String>,
    config: &SectionConfig,
    tun: &Tunables,
) -> UnifiedDetection {
    const NO_TIME: &[f64] = &[];
    // Partition tracks per sport; sections never span sports.
    type SportTracks<'a> = (Vec<(&'a str, &'a [GpsPoint])>, Vec<&'a [f64]>);
    let mut by_sport: HashMap<&str, SportTracks> = HashMap::new();
    for (i, (id, pts)) in tracks.iter().enumerate() {
        let sport = sport_types.get(id).map(|s| s.as_str()).unwrap_or("Unknown");
        let entry = by_sport.entry(sport).or_default();
        entry.0.push((id.as_str(), pts.as_slice()));
        entry.1.push(seconds.get(i).copied().unwrap_or(NO_TIME));
    }

    let mut sport_names: Vec<&str> = by_sport.keys().copied().collect();
    sport_names.sort_unstable();

    let mut all_sections: Vec<FrequentSection> = Vec::new();
    let mut boundaries: Vec<BoundaryRecord> = Vec::new();
    for sport in sport_names {
        let (sport_tracks, sport_seconds) = &by_sport[sport];
        let mut idx = 0usize;
        let sections = detect_for_sport(
            sport,
            sport_tracks,
            sport_seconds,
            config,
            tun,
            &mut idx,
            &mut boundaries,
        );
        all_sections.extend(sections);
    }

    info!(
        "[Unified] {} sections, {} boundary records",
        all_sections.len(),
        boundaries.len()
    );
    UnifiedDetection {
        sections: all_sections,
        boundaries,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn climb(step_deg: f64, ele_step: f64, zigzag: bool, n: usize) -> Vec<GpsPoint> {
        (0..n)
            .map(|i| {
                let lng = if zigzag {
                    // Alternate heading so the arc far exceeds the chord.
                    if i % 2 == 0 { 0.0 } else { step_deg * 1.8 }
                } else {
                    0.0
                };
                GpsPoint::with_elevation(46.0 + step_deg * i as f64, 7.0 + lng, ele_step * i as f64)
            })
            .collect()
    }

    #[test]
    fn straight_steep_ascent_is_lift() {
        // ~10m steps north, +5m elevation each: 50% grade, dead straight.
        let pts = climb(9.0e-5, 5.0, false, 80);
        let spans = lift_spans(&pts, None);
        assert_eq!(spans.len(), 1);
        let (s, e) = spans[0];
        assert!(
            s <= 2 && e >= 75,
            "span {}..{} should cover the ascent",
            s,
            e
        );
    }

    #[test]
    fn winding_climb_is_not_lift() {
        // Same climb rate per metre of ground but zigzagging: a trail.
        let pts = climb(9.0e-5, 5.0, true, 80);
        assert!(lift_spans(&pts, None).is_empty());
    }

    #[test]
    fn straight_steep_descent_is_not_lift() {
        let mut pts = climb(9.0e-5, 5.0, false, 80);
        pts.reverse();
        assert!(lift_spans(&pts, None).is_empty());
    }

    #[test]
    fn straight_chairlift_grade_is_lift() {
        // 26% sustained and dead straight: a typical chairlift line.
        let pts = climb(9.0e-5, 2.6, false, 80);
        assert_eq!(lift_spans(&pts, None).len(), 1);
    }

    #[test]
    fn gentle_straight_ascent_is_not_lift() {
        // 10% sustained: an ordinary straight road climb.
        let pts = climb(9.0e-5, 1.0, false, 80);
        assert!(lift_spans(&pts, None).is_empty());
    }

    #[test]
    fn wobbling_stair_climb_is_not_lift() {
        // A walker's micro-wobble inflates the raw arc over the smoothed
        // arc; a cabin's track does not.
        let pts: Vec<GpsPoint> = (0..80)
            .map(|i| {
                let lng = 7.0 + if i % 2 == 0 { 5.0e-5 } else { -5.0e-5 };
                GpsPoint::with_elevation(46.0 + 9.0e-5 * i as f64, lng, 2.6 * i as f64)
            })
            .collect();
        assert!(lift_spans(&pts, None).is_empty());
    }

    #[test]
    fn straight_retrace_downhill_rescues_stairs() {
        // Vineyard stairs: climbed straight, walked straight back down.
        let up = climb(9.0e-5, 2.6, false, 80);
        let mut down = up.clone();
        down.reverse();
        let tracks: Vec<(&str, &[GpsPoint])> =
            vec![("up", up.as_slice()), ("down", down.as_slice())];
        let confirmed = confirmed_lift_spans(&tracks, &[]);
        assert!(confirmed.iter().all(|c| c.is_empty()));
    }

    #[test]
    fn weaving_descent_does_not_rescue_lift() {
        // Riders weave under the lift line; the line stays carried ground.
        let up = climb(9.0e-5, 2.6, false, 80);
        let down: Vec<GpsPoint> = (0..80)
            .map(|i| {
                let t = 79 - i;
                let lng = 7.0 + if i % 2 == 0 { 2.0e-4 } else { -2.0e-4 };
                GpsPoint::with_elevation(46.0 + 9.0e-5 * t as f64, lng, 2.6 * t as f64)
            })
            .collect();
        let tracks: Vec<(&str, &[GpsPoint])> =
            vec![("up", up.as_slice()), ("down", down.as_slice())];
        let confirmed = confirmed_lift_spans(&tracks, &[]);
        assert_eq!(confirmed[0].len(), 1);
    }

    /// Timestamps for `climb` geometry at a constant ground speed;
    /// steps are ~10 m apart.
    fn times_at(pts: &[GpsPoint], speed_ms: f64) -> Vec<f64> {
        let mut t = vec![0.0f64];
        for w in pts.windows(2) {
            let d = crate::geo_utils::haversine_distance(&w[0], &w[1]);
            t.push(t.last().unwrap() + d / speed_ms);
        }
        t
    }

    #[test]
    fn slow_timed_climb_is_not_lift() {
        // Straight and steep enough to be a lift by geometry, but timed
        // at hiking pace: the velocity veto keeps it human ground.
        let pts = climb(9.0e-5, 2.6, false, 80);
        let secs = times_at(&pts, 0.7);
        assert!(lift_spans(&pts, Some(&secs)).is_empty());
    }

    #[test]
    fn carried_speed_confirms_lift() {
        let pts = climb(9.0e-5, 2.6, false, 80);
        let secs = times_at(&pts, 4.0);
        assert_eq!(lift_spans(&pts, Some(&secs)).len(), 1);
    }

    #[test]
    fn far_blobs_cluster_apart_and_a_bridge_joins_them() {
        let near: Vec<GpsPoint> = (0..10)
            .map(|i| GpsPoint::new(46.0 + 1.0e-4 * i as f64, 7.0))
            .collect();
        let far: Vec<GpsPoint> = (0..10)
            .map(|i| GpsPoint::new(37.0 + 1.0e-4 * i as f64, 7.0))
            .collect();
        let tracks: Vec<(&str, &[GpsPoint])> =
            vec![("north", near.as_slice()), ("south", far.as_slice())];
        let clusters = geo_clusters(&tracks, Tunables::DEFAULT.cluster_gap_m);
        assert_eq!(clusters.len(), 2);
        assert_eq!(clusters[0], vec![1], "south-west cluster orders first");
        assert_eq!(clusters[1], vec![0]);

        let bridge: Vec<GpsPoint> = (0..=90)
            .map(|i| GpsPoint::new(37.0 + 0.1 * i as f64, 7.0))
            .collect();
        let tracks: Vec<(&str, &[GpsPoint])> = vec![
            ("north", near.as_slice()),
            ("south", far.as_slice()),
            ("bridge", bridge.as_slice()),
        ];
        assert_eq!(
            geo_clusters(&tracks, Tunables::DEFAULT.cluster_gap_m).len(),
            1,
            "a track spanning both grounds bridges them"
        );
    }

    #[test]
    fn halted_cabin_is_still_lift() {
        // A mid-ride loading halt drags the mean but occupies a single
        // distance window; the median windowed speed stays carried.
        let pts = climb(9.0e-5, 2.6, false, 80);
        let mut secs = times_at(&pts, 4.0);
        for s in secs.iter_mut().skip(40) {
            *s += 240.0;
        }
        assert_eq!(lift_spans(&pts, Some(&secs)).len(), 1);
    }

    /// A line of points heading north at ~10 m spacing, offset east.
    fn row(east_m: f64, n: usize) -> Vec<GpsPoint> {
        (0..n)
            .map(|i| GpsPoint::new(46.0 + 9.0e-5 * i as f64, 7.0 + east_m / 77_000.0))
            .collect()
    }

    fn accept(line: &[GpsPoint], grid: &CellGrid) -> HashMap<Cell, Vec<GpsPoint>> {
        let mut map: HashMap<Cell, Vec<GpsPoint>> = HashMap::new();
        for p in line {
            map.entry(grid.cell_of(p.latitude, p.longitude))
                .or_default()
                .push(*p);
        }
        map
    }

    #[test]
    fn probe_beside_accepted_line_is_represented() {
        let grid = CellGrid::new(100.0, 46.0);
        let accepted = accept(&row(0.0, 100), &grid);
        let mask = probe_mask(&row(30.0, 100), &accepted, &grid, 100.0);
        assert!(mask.iter().all(|&m| m), "30 m offset is braid width");
        let mask = probe_mask(&row(300.0, 100), &accepted, &grid, 100.0);
        assert!(mask.iter().all(|&m| !m), "300 m offset is distinct ground");
    }

    #[test]
    fn crossing_islands_bridge_but_long_overlap_splits() {
        // 10 m point spacing: cum[i] = 10 * i.
        let cum: Vec<f64> = (0..40).map(|i| 10.0 * i as f64).collect();
        let mut mask = vec![false; 40];
        for m in mask.iter_mut().skip(18).take(3) {
            *m = true; // 30 m contact: an incidental crossing
        }
        let runs = unrepresented_runs(&mask, &cum, 100.0);
        assert_eq!(runs, vec![(0, 40)], "short island bridges");
        for m in mask.iter_mut().skip(18).take(15) {
            *m = true; // 150 m of real duplication
        }
        let runs = unrepresented_runs(&mask, &cum, 100.0);
        assert_eq!(runs, vec![(0, 18), (33, 40)], "long overlap splits runs");
    }

    #[test]
    fn straight_line_has_no_self_pass() {
        let pen = self_pass_penalty(&row(0.0, 100), 20.0, 100.0);
        assert!(pen < 0.02, "a single straight pass never revisits: {pen}");
    }

    #[test]
    fn out_and_back_revisits_its_outbound_leg() {
        let out = row(0.0, 60);
        let mut pts = out.clone();
        pts.extend(out.into_iter().rev().skip(1));
        let pen = self_pass_penalty(&pts, 20.0, 100.0);
        // The retracing half scores far above a single pass (~0); the
        // turnaround and the start-adjacent tail are legitimately exempt.
        assert!(
            pen > 0.3,
            "the return leg retraces the outbound ground: {pen}"
        );
    }

    #[test]
    fn closed_loop_join_is_exempt() {
        // A square circuit back to the start: each edge is fresh ground
        // ~300 m from any other, only the final vertex meets the first,
        // and the opening gap exempts that join.
        let s = 30i32;
        let (n, e) = (9.0e-5, 1.3e-4);
        let mut pts = Vec::new();
        for i in 0..s {
            pts.push(GpsPoint::new(46.0 + n * i as f64, 7.0));
        }
        for i in 0..s {
            pts.push(GpsPoint::new(46.0 + n * s as f64, 7.0 + e * i as f64));
        }
        for i in 0..s {
            pts.push(GpsPoint::new(46.0 + n * (s - i) as f64, 7.0 + e * s as f64));
        }
        for i in 0..=s {
            pts.push(GpsPoint::new(46.0, 7.0 + e * (s - i) as f64));
        }
        let pen = self_pass_penalty(&pts, 20.0, 100.0);
        assert!(pen < 0.05, "a clean loop only rejoins at the start: {pen}");
    }

    #[test]
    fn support_counts_contributor_passes_not_strangers_or_clips() {
        // Two corridor traversals (one a braid row over), one that
        // continues onto a private tail, and a perpendicular crossing.
        // Corridor ground is supported by all three traversals, tail
        // ground by its one rider alone; the crossing's corner clip
        // lends nothing, and a non-contributor lends nothing anywhere.
        let corridor = row(0.0, 60);
        let braid = row(30.0, 60);
        let with_tail = row(0.0, 120);
        let cross: Vec<GpsPoint> = (0..60)
            .map(|i| GpsPoint::new(46.0 + 9.0e-5 * 30.0, 7.0 + (i as f64 * 10.0 - 300.0) / 77_000.0))
            .collect();
        let tracks: Vec<(&str, &[GpsPoint])> = vec![
            ("a", corridor.as_slice()),
            ("b", braid.as_slice()),
            ("c", with_tail.as_slice()),
            ("x", cross.as_slice()),
        ];
        let coverage = build_coverage_grid(&tracks, &[], 100.0, &Tunables::DEFAULT);
        let cell_set: HashSet<Cell> = with_tail
            .iter()
            .chain(braid.iter())
            .flat_map(|p| {
                let c = coverage.grid.cell_of(p.latitude, p.longitude);
                (-1..=1i32).flat_map(move |dy| (-1..=1i32).map(move |dx| (c.0 + dy, c.1 + dx)))
            })
            .collect();
        // The crossing is deliberately NOT a contributor; c's fragment
        // status is irrelevant here — every qualifying pass counts.
        let portions: Vec<Portion> =
            vec![(0, 0, 60, 600.0), (1, 0, 60, 600.0), (2, 0, 120, 1200.0)];
        let support = candidate_support(&portions, &cell_set, &coverage, &tracks, 100.0);
        let mid = coverage
            .grid
            .cell_of(corridor[30].latitude, corridor[30].longitude);
        assert_eq!(support[&mid], 3, "braid row lends the lane support");
        let tail = coverage
            .grid
            .cell_of(with_tail[100].latitude, with_tail[100].longitude);
        assert_eq!(support[&tail], 1, "a tail is its one rider's alone");
    }

    #[test]
    fn out_and_back_scores_high_but_a_through_line_does_not() {
        // Out along a street and back over it: closed and antiparallel.
        let out = row(0.0, 50);
        let mut there_back = out.clone();
        there_back.extend(out.into_iter().rev().skip(1));
        assert!(
            out_and_back_penalty(&there_back, 20.0) > 0.6,
            "a forward-and-reverse runs back over its own ground"
        );
        // A single straight pass ends far from its start: never scored.
        assert_eq!(out_and_back_penalty(&row(0.0, 50), 20.0), 0.0);
    }

    #[test]
    fn switchback_climb_is_not_an_out_and_back() {
        // Hairpins with antiparallel legs, but the line climbs away and
        // ends far from its start, so the closed-line gate never fires.
        let mut pts = Vec::new();
        for leg in 0..6 {
            let base = 46.0 + 6.0e-4 * leg as f64;
            let east = leg % 2 == 0;
            for i in 0..20 {
                let lng = if east {
                    7.0 + 1.3e-4 * i as f64
                } else {
                    7.0 + 1.3e-4 * (20 - i) as f64
                };
                pts.push(GpsPoint::new(base + 3.0e-5 * i as f64, lng));
            }
        }
        assert_eq!(
            out_and_back_penalty(&pts, 20.0),
            0.0,
            "a switchback climbs away; its hairpins are not a retrace"
        );
    }
}
