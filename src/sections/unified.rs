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
use super::identity::shares_ground;
use super::overlap::{FullTrackOverlap, OverlapCluster};
use super::postprocess::required_visits_for_length;
use super::{FrequentSection, SectionConfig, SectionPortion, process_cluster};
use crate::GpsPoint;
use crate::union_find::UnionFind;
use log::info;
use serde::{Deserialize, Serialize};
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
    /// The join-local traffic halves: most of the thick side's users
    /// never reach the thin side, and their departure is the visible
    /// reason the section ends — a braid mouth rejoining wide, a
    /// popular stopping point, a corridor shedding most of its traffic
    /// at once. Rule 9's majority usage change measured as volume,
    /// where [`BoundaryReason::UsageChange`] measures it as pass class.
    TrafficCliff { thin: u32, thick: u32 },
    /// The section's one real pass ended here but its corridor's
    /// ground runs on. The ground beyond re-entered the candidate
    /// queue on its own merits; this record marks the seam between a
    /// section and the next link of its chain.
    PassEnd { requeued_cells: u32 },
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
    /// Clean-render bar: the default render (longest portion, or the
    /// medoid) keeps its stability privilege only while essentially
    /// fold-free — under this share of revisiting points. Between here
    /// and [`Tunables::self_pass_max`] a legal but visibly folded
    /// default is displaced by the cleanest contributing pass (the
    /// penalty's blind band: a knot 100-200 m along-trace is charged
    /// by [`self_pass_penalty`] but too tight for the dwell cut).
    /// Sits above the clean-line noise floor (straight passes measure
    /// < 0.02); plateau sweep beside `self_pass_max`.
    pub self_pass_clean: f64,
    /// Majority-render bar: the longest contiguous stretch of a
    /// rendered line allowed on minority ground (supported by fewer
    /// than half the section's own contributing portions) before the
    /// render prefers a majority-faithful portion instead. The
    /// coverage ring's lane-width capture can thread one contributor's
    /// braid variant through a candidate; the drawn line is the
    /// section's public face and must be where the majority went. A
    /// sustained run marks a variant walk; scattered single samples (a
    /// staircase jog, an end taper) never trip it. Three samples at
    /// the 20 m step, mirroring [`minority_end_clip`]'s sustained-run
    /// floor; plateau sweep beside `self_pass_max`.
    pub minority_run_m: f64,
    /// Occasion span floor, in hours. Support counts distinct calendar
    /// DAYS, and — when every visit is dated — additionally requires
    /// the days to stretch beyond one stay: ground visited only within
    /// this window was one trip to a place, not repetition, however
    /// many recordings the trip produced ([`occasion_support`]). A gap
    /// threshold cannot draw this line (a two-day trip and two commute
    /// days are both ~24 h apart; pairwise separation halves a daily
    /// commuter's support); the span can — a stay is compact, routine
    /// stretches over weeks. One week; sweep 72-336 beside the other
    /// floors. Tracks with no known start count as their own day and
    /// waive the span, so dateless corpora keep activity counting.
    pub occasion_span_h: f64,
    /// Projection latitude quantisation, in degrees. A cluster's
    /// `ref_lat` snaps to the nearest multiple, so the metre projection
    /// is a step function of the activity set: a routine add leaves
    /// every cell boundary in place, which is what makes per-track and
    /// per-cluster leaves cacheable at all. Derivation-anchored, like
    /// `cluster_gap_m`: the snap moves the projection latitude at most
    /// half a band, an east-west scale error of tan(lat) x half-band —
    /// about 0.15% at 60 degrees, an order under the 1% budget that
    /// sized `cluster_gap_m`. When a mean does cross a band edge the
    /// cluster re-projects once and the hysteresis layer absorbs the
    /// re-cut like any other evidence shift.
    pub ref_lat_quant_deg: f64,
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
        self_pass_clean: 0.05,
        occasion_span_h: 168.0,
        minority_run_m: 60.0,
        ref_lat_quant_deg: 0.1,
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
    rescue_confirmed(candidates, tracks, tun)
}

/// The cross-track half of [`confirmed_lift_spans_tuned`]: drop any candidate
/// span some track descends straight (lift lines are never descended). Split
/// from the per-track candidate scan so the incremental path can reuse cached
/// candidates while recomputing the rescue over the current membership.
fn rescue_confirmed(
    candidates: Vec<Vec<(usize, usize)>>,
    tracks: &[(&str, &[GpsPoint])],
    tun: &Tunables,
) -> Vec<Vec<(usize, usize)>> {
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

/// The coverage-grid cell size for a cluster, derived from the config's
/// proximity threshold. The batch [`detect_for_cluster`] and the cached
/// fold must agree on this exactly, so it lives in one place.
fn cluster_cell_size(config: &SectionConfig) -> f64 {
    (config.proximity_threshold * 0.5).clamp(50.0, 150.0)
}

fn build_coverage_grid(
    tracks: &[(&str, &[GpsPoint])],
    seconds: &[&[f64]],
    cell_size_m: f64,
    tun: &Tunables,
    lift_memo: &mut HashMap<String, Vec<(usize, usize)>>,
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
            // Snap to the quantisation lattice so the projection is a step
            // function of the activity set (see `Tunables::ref_lat_quant_deg`).
            let mean = lats.iter().sum::<f64>() / lats.len() as f64;
            (mean / tun.ref_lat_quant_deg).round() * tun.ref_lat_quant_deg
        }
    };
    let grid = CellGrid::new(cell_size_m, ref_lat);

    let fine = CellGrid::new(cell_size_m / tun.pass_subgrid, ref_lat);

    // Candidate spans come from the memo (pure per track); the cross-track
    // rescue is recomputed over the present membership, so a late straight
    // descender still clears a span whose candidates were cached earlier.
    let candidates: Vec<Vec<(usize, usize)>> = tracks
        .iter()
        .enumerate()
        .map(|(i, (id, pts))| {
            lift_memo
                .entry((*id).to_string())
                .or_insert_with(|| lift_spans_tuned(pts, seconds.get(i).copied(), tun))
                .clone()
        })
        .collect();
    let lift = rescue_confirmed(candidates, tracks, tun);
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
        fold_track_into_grid(
            &grid,
            &fine,
            &mut cell_tracks,
            &mut cell_passes,
            t_idx as u32,
            pts,
            &keep[t_idx],
            tun,
        );
    }

    CoverageGrid {
        grid,
        ref_lat,
        cell_tracks,
        cell_passes,
        keep,
    }
}

/// Accumulate one track's coverage into the grid maps: its unique-cell
/// visits (`cell_tracks`) and its per-cell pass class (`cell_passes`).
/// The per-track step of [`build_coverage_grid`].
///
/// `t` is the track's index in the cluster's track ordering (the `u32`
/// the grid keys everything by); `keep_t` is its lift-excluded keep
/// ranges. `grid` and `fine` are the partition and sub-cell grids, both
/// derived from the cluster's cell size and reference latitude.
#[allow(clippy::too_many_arguments)]
fn fold_track_into_grid(
    grid: &CellGrid,
    fine: &CellGrid,
    cell_tracks: &mut HashMap<Cell, HashSet<u32>>,
    cell_passes: &mut HashMap<Cell, HashMap<u32, u8>>,
    t: u32,
    pts: &[GpsPoint],
    keep_t: &[(usize, usize)],
    tun: &Tunables,
) {
    if pts.is_empty() {
        return;
    }

    // Fine-cell pass counting; each fine cell remembers the
    // partition cell it first appeared in.
    let mut scratch: HashMap<Cell, (Cell, PassScratch)> = HashMap::new();
    let mut fine_seq = 0usize;
    let mut visit_fine = |fc: Cell, pc: Cell, ele: Option<f64>, seq: usize| match scratch.entry(fc)
    {
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
    for (r_idx, &(rs, re)) in keep_t.iter().enumerate() {
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
    partition_by(hot_cells, coverage, same_traffic, |a, b| {
        same_traffic_sets(a, b, same_traffic)
    })
}

/// Reassemble pooled orphan ground at through-traffic granularity:
/// adjacent cells union when the thinner side's traffic is carried by
/// the thicker side (containment ≥ the same-traffic anchor). The
/// strict MUTUAL containment of [`partition_supernodes`] is right for
/// first-pass supernodes — it keeps a 3-track street out of a 5-track
/// street's node — but it re-shreds salvaged junction ground on the
/// same junction cells that killed it: crossing paths inflate one
/// cell's set and mutuality fails there forever, while the through
/// traffic runs contained through the junction. One-sided containment
/// follows the through traffic; a dishonest glue (a quiet stub riding
/// a busy corridor's containment) cannot RENDER — rule B displaces any
/// pass walking sustained minority ground and the remainder re-queue
/// re-separates what the render disowns — so honesty is policed where
/// it binds, at the line.
fn partition_pooled(
    hot_cells: &[Cell],
    coverage: &CoverageGrid,
    same_traffic: f64,
) -> Vec<Supernode> {
    partition_by(hot_cells, coverage, same_traffic, |a, b| {
        if a.is_empty() || b.is_empty() {
            return false;
        }
        let inter = a.intersection(b).count();
        let thin = a.len().min(b.len());
        inter as f64 / thin as f64 >= same_traffic
    })
}

fn partition_by(
    hot_cells: &[Cell],
    coverage: &CoverageGrid,
    same_traffic: f64,
    join: impl Fn(&HashSet<u32>, &HashSet<u32>) -> bool,
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
                    && join(my_tracks, &coverage.cell_tracks[&n])
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

/// Join-local traffic on each side: the union of tracks over the
/// boundary-touching cells. The join is a cliff when the thin side
/// carries less than half the thick side's traffic — the majority's
/// experience of the ground ends there (rule 9's majority usage
/// change, measured as volume), even when the leavers' alternative is
/// not visible at the join: a braid mouth rejoining wide, unique
/// dispersal exits, activities simply ending. The majority bar mirrors
/// [`is_usage_boundary`]; the one-track absolute slack mirrors
/// [`same_traffic_sets`] so tiny corridors never fragment. Gradual
/// attrition (traffic shed one track at a time along the corridor)
/// never trips it — each join's local sets stay within the slack.
fn traffic_cliff(pairs: &[(Cell, Cell)], coverage: &CoverageGrid) -> Option<(u32, u32)> {
    let mut la: HashSet<u32> = HashSet::new();
    let mut lb: HashSet<u32> = HashSet::new();
    for &(ca, cb) in pairs {
        if let Some(t) = coverage.cell_tracks.get(&ca) {
            la.extend(t.iter().copied());
        }
        if let Some(t) = coverage.cell_tracks.get(&cb) {
            lb.extend(t.iter().copied());
        }
    }
    let (thin, thick) = if la.len() <= lb.len() {
        (la.len(), lb.len())
    } else {
        (lb.len(), la.len())
    };
    (thick - thin > 1 && 2 * thin < thick).then_some((thin as u32, thick as u32))
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
        if let Some((thin, thick)) = traffic_cliff(pairs, coverage) {
            records.push(BoundaryRecord {
                latitude: lat,
                longitude: lng,
                reason: BoundaryReason::TrafficCliff { thin, thick },
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
        // A majority usage change is a boundary in its own right: the
        // join neither merges nor lets a fragment absorb across it, or
        // the thin side's ground would wear the thick side's counts.
        if traffic_cliff(pairs, coverage).is_some() {
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
    if pts.is_empty() {
        return 0.0;
    }
    let marks = self_pass_marks(pts, near, gap);
    marks.iter().filter(|&&m| m).count() as f64 / pts.len() as f64
}

/// Per-point revisit marks behind [`self_pass_penalty`]: `true` on the
/// LATER visitor, so a folded line's clean opening stays unmarked and
/// the clean-stretch clip can keep it.
fn self_pass_marks(pts: &[GpsPoint], near: f64, gap: f64) -> Vec<bool> {
    let mut marks = vec![false; pts.len()];
    if pts.len() < 3 {
        return marks;
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
        return marks;
    }
    let near2 = near * near;
    let key = |x: f64, y: f64| ((x / near).floor() as i32, (y / near).floor() as i32);
    let mut grid: HashMap<(i32, i32), Vec<(f64, f64)>> = HashMap::new();
    let mut lag = 0usize;
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
            marks[i] = true;
        }
    }
    marks
}

/// Share of a CLOSED line (endpoints within `CLOSE_FRAC` of its length)
/// that runs back over its own ground in the opposite direction: a
/// forward-and-reverse. Only closed lines are scored, so a switchback
/// climb (which climbs away, endpoints far apart) is never charged for
/// its antiparallel hairpin legs, and a clean loop scores ~0 (its
/// ground is travelled once, same sense). Antiparallel pairs must also
/// sit at least `gap` apart ALONG the line: a hairpin-tight corner's
/// two legs are antiparallel within `near` but only a corner's arc
/// apart — the corner's own shape, not a retrace — while an
/// out-and-back's pairs sit a full out-leg apart. Complements
/// [`self_pass_penalty`], which needs an arc gap and so misses the
/// short spurs of an out-and-back that returns to its start.
fn out_and_back_penalty(pts: &[GpsPoint], near: f64, gap: f64) -> f64 {
    if pts.is_empty() {
        return 0.0;
    }
    let marks = out_and_back_marks(pts, near, gap);
    marks.iter().filter(|&&m| m).count() as f64 / pts.len() as f64
}

/// Per-point retrace marks behind [`out_and_back_penalty`]. Both
/// directions of a retraced stretch are marked — the ground genuinely
/// has no single clean pass through it.
fn out_and_back_marks(pts: &[GpsPoint], near: f64, gap: f64) -> Vec<bool> {
    // A hill-sprint hairpin drifts at the turn, so its endpoints sit a
    // quarter of its length apart — over the old 0.2 gate — while the
    // self-pass arc exemptions make any sub-2×gap line unchargeable
    // there. 0.3 keeps every genuinely open line out (a through pass
    // ends a full length away) and lets the antiparallel test judge
    // the rest.
    const CLOSE_FRAC: f64 = 0.3;
    let mut marks = vec![false; pts.len()];
    if pts.len() < 5 {
        return marks;
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
    let total = cum[pts.len() - 1];
    let last = xy[xy.len() - 1];
    let end = ((last.0 - xy[0].0).powi(2) + (last.1 - xy[0].1).powi(2)).sqrt();
    if total <= 0.0 || end > CLOSE_FRAC * total {
        return marks;
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
    for (i, &(x, y)) in xy.iter().enumerate() {
        let (cx, cy) = key(x, y);
        let anti = (-1..=1).any(|dx| {
            (-1..=1).any(|dy| {
                grid.get(&(cx + dx, cy + dy)).is_some_and(|b| {
                    b.iter().any(|&j| {
                        (cum[i] - cum[j]).abs() >= gap
                            && (x - xy[j].0).powi(2) + (y - xy[j].1).powi(2) < near2
                            && hd[i].0 * hd[j].0 + hd[i].1 * hd[j].1 < -0.5
                    })
                })
            })
        });
        if anti {
            marks[i] = true;
        }
    }
    marks
}

/// A pass that terminates by closing onto its own interior is a loop
/// plus an arrival (or departure) stem: the LOOP is the honest face of
/// lapped ground, and the stem is through-ground the closure disowns.
///
/// The end may be extended along the SAME trace by up to `max_ext_m`
/// to complete the revolution: the pass cutter cuts where re-covering
/// BEGINS, which at a small loop's mouth is a breath before the
/// closure point, so the cut pass ends a few dozen metres short of
/// closing. The extension keeps the render one contiguous real range.
///
/// Returns the loop as a half-open track range when a closure exists
/// and something meaningful changes: a clean closed loop closes onto
/// its own start (nothing to trim, no extension), and a
/// forward-and-reverse ends beside its opening metres (stem under the
/// floor of twice `near`); both return `None`.
fn closing_loop_range(
    track: &[GpsPoint],
    rs: usize,
    re: usize,
    near: f64,
    gap: f64,
    max_ext_m: f64,
) -> Option<(usize, usize)> {
    let n = re - rs;
    if n < 5 {
        return None;
    }
    let m_lat = 111_132.0;
    let m_lng = 111_320.0 * track[rs].latitude.to_radians().cos();
    // Extend the frame past the cut while the extension stays short.
    let mut ext_re = re;
    let mut ext_m = 0.0;
    while ext_re < track.len() && ext_m < max_ext_m {
        ext_m += crate::geo_utils::haversine_distance(&track[ext_re - 1], &track[ext_re]);
        ext_re += 1;
    }
    let pts = &track[rs..ext_re];
    let xy: Vec<(f64, f64)> = pts
        .iter()
        .map(|p| (p.latitude * m_lat, p.longitude * m_lng))
        .collect();
    let mut cum = vec![0.0f64; pts.len()];
    for i in 1..pts.len() {
        let (dx, dy) = (xy[i].0 - xy[i - 1].0, xy[i].1 - xy[i - 1].1);
        cum[i] = cum[i - 1] + (dx * dx + dy * dy).sqrt();
    }
    let min_stem = 2.0 * near;
    let near2 = near * near;
    let d2 = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2);
    // Earliest end (never before the original cut) that closes onto
    // the latest possible interior point of the ORIGINAL pass.
    for end_k in n - 1..pts.len() {
        let mut start_at: Option<usize> = None;
        for j in 0..n - 1 {
            if cum[end_k] - cum[j] >= gap && d2(xy[end_k], xy[j]) < near2 {
                start_at = Some(j);
            }
        }
        if let Some(j) = start_at {
            let meaningful = cum[j] >= min_stem || end_k > n - 1;
            let kept = cum[end_k] - cum[j];
            if meaningful && kept >= gap {
                return Some((rs + j, rs + end_k + 1));
            }
            return None;
        }
    }
    // No end closure: a loop first, then a departure stem — the start
    // is revisited later, and the loop before the stem is the face.
    let total = cum[n - 1];
    for j in (1..n).rev() {
        if cum[j] >= gap && d2(xy[0], xy[j]) < near2 {
            if total - cum[j] >= min_stem {
                return Some((rs, rs + j + 1));
            }
            return None;
        }
    }
    None
}

/// The longest fold-free stretch of a line: the maximal contiguous run
/// of points carrying neither a self-pass revisit mark nor a retrace
/// mark, as a half-open index range. A clean line returns the whole
/// range.
fn longest_clean_range(pts: &[GpsPoint], near: f64, gap: f64) -> (usize, usize) {
    if pts.is_empty() {
        return (0, 0);
    }
    let sp = self_pass_marks(pts, near, gap);
    let oab = out_and_back_marks(pts, near, gap);
    let mut best = (0usize, 0usize);
    let mut best_m = -1.0f64;
    let mut run = 0usize;
    let mut run_m = 0.0f64;
    for i in 0..pts.len() {
        if sp[i] || oab[i] {
            run = i + 1;
            run_m = 0.0;
            continue;
        }
        if i > run {
            run_m += crate::geo_utils::haversine_distance(&pts[i - 1], &pts[i]);
        }
        if run_m > best_m {
            best_m = run_m;
            best = (run, i + 1);
        }
    }
    best
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
        if let Some((s, e, dist)) = track_portion(
            pts,
            &coverage.keep[t_idx as usize],
            &core,
            &cell_set,
            &coverage.grid,
            &pass_grid,
            cell_size,
            config,
            tun,
        ) {
            portions.push((t_idx as usize, s, e, dist));
        }
    }
    portions
}

/// One track's single-pass portion over one supernode's ground: the
/// per-track half of [`portions_for`], independent of every other track.
#[allow(clippy::too_many_arguments)]
fn track_portion(
    pts: &[GpsPoint],
    keep: &[(usize, usize)],
    core: &HashSet<Cell>,
    cell_set: &HashSet<Cell>,
    grid: &CellGrid,
    pass_grid: &CellGrid,
    cell_size: f64,
    config: &SectionConfig,
    tun: &Tunables,
) -> Option<(usize, usize, f64)> {
    {
        // Walk each lift-free range on its own: excluded ground is not
        // usable geometry even when the component touches both of its
        // ends (a base station and a summit do exactly that).
        let mut best: Option<(usize, usize, f64)> = None;
        for &(rs, re) in keep {
            if let Some((s, e, d)) = longest_run_in_cells(&pts[rs..=re], cell_set, grid)
                && best.as_ref().is_none_or(|b| d > b.2)
            {
                best = Some((rs + s, rs + e, d));
            }
        }
        let (mut s, mut e, _) = best?;
        while s < e && !core.contains(&grid.cell_of(pts[s].latitude, pts[s].longitude)) {
            s += 1;
        }
        while e > s && !core.contains(&grid.cell_of(pts[e - 1].latitude, pts[e - 1].longitude)) {
            e -= 1;
        }
        if e <= s + 1 {
            return None;
        }
        // Single pass only: laps, return legs, and loop stems are
        // traversals, not section geometry.
        let (ps, pe) = simple_pass_range(&pts[s..e], pass_grid, tun);
        let limit = e;
        e = s + pe;
        s += ps;
        if e <= s + 1 {
            return None;
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
            Some((s, e, dist))
        } else {
            None
        }
    }
}

/// [`portions_for`] through the per-track leaf memo: the node's cell set
/// is interned once, then each member track hits on its own complete
/// fingerprint or computes via the same [`track_portion`] the direct path
/// uses. Observationally identical to [`portions_for`] (the warm/cold
/// certificate pins it); on saturated ground a new activity leaves every
/// existing member's entry valid and pays only for its own cut.
#[allow(clippy::too_many_arguments)]
fn portions_for_memo(
    leaves: &mut LeafMemos,
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

    let mut cells_sorted = node.cells.clone();
    cells_sorted.sort_unstable();
    let next_id = leaves.cell_sets.len() as u32;
    let cells_id = *leaves.cell_sets.entry(cells_sorted).or_insert(next_id);

    let mut t_indices: Vec<u32> = node.tracks.iter().copied().collect();
    t_indices.sort_unstable_by(|&a, &b| sport_tracks[a as usize].0.cmp(sport_tracks[b as usize].0));

    let mut portions: Vec<Portion> = Vec::new();
    for &t_idx in &t_indices {
        let (id, pts) = sport_tracks[t_idx as usize];
        let key = TrackPortionKey {
            track: id.to_string(),
            cells: cells_id,
            keep: coverage.keep[t_idx as usize].clone(),
            ref_lat_bits: coverage.ref_lat.to_bits(),
            cell_size_bits: cell_size.to_bits(),
            min_len_bits: config.min_section_length.to_bits(),
            max_len_bits: config.max_section_length.to_bits(),
        };
        let cut = match leaves.track_portions.get(&key) {
            Some(hit) => *hit,
            None => {
                let fresh = track_portion(
                    pts,
                    &coverage.keep[t_idx as usize],
                    &core,
                    &cell_set,
                    &coverage.grid,
                    &pass_grid,
                    cell_size,
                    config,
                    tun,
                );
                leaves.track_portions.insert(key, fresh);
                fresh
            }
        };
        if let Some((ps, pe, dist)) = cut {
            portions.push((t_idx as usize, ps, pe, dist));
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
/// Occasion support for a set of portions: the count of distinct
/// calendar DAYS among known starts (plus one per track with no known
/// start), and the span between the earliest and latest known start.
/// Together they answer the support question: ground is repeated when
/// it is visited on enough separate days AND those days stretch beyond
/// one stay ([`Tunables::occasion_span_h`]). Day counting alone cannot
/// tell a two-day trip from two commute days; the span can — a trip is
/// compact, routine stretches over weeks.
fn occasion_support(
    portions: &[Portion],
    sport_tracks: &[(&str, &[GpsPoint])],
    starts: &HashMap<String, i64>,
) -> (usize, Option<i64>, usize) {
    let mut days: HashSet<i64> = HashSet::with_capacity(portions.len());
    let mut undated = 0usize;
    let (mut lo, mut hi) = (i64::MAX, i64::MIN);
    for &(t, ..) in portions {
        match starts.get(sport_tracks[t].0) {
            Some(&e) => {
                days.insert(e.div_euclid(86_400));
                lo = lo.min(e);
                hi = hi.max(e);
            }
            None => undated += 1,
        }
    }
    let span = (days.len() >= 2).then(|| hi - lo);
    (undated + days.len(), span, undated)
}

/// Distinct support days for one cell's contributing track ids: known
/// starts count by calendar day, unknown starts each count alone. The
/// span requirement binds at the candidate level, not per cell.
fn cell_support_days(ids: &[&str], starts: &HashMap<String, i64>) -> usize {
    let mut days: HashSet<i64> = HashSet::with_capacity(ids.len());
    let mut undated = 0usize;
    for id in ids {
        match starts.get(*id) {
            Some(&e) => {
                days.insert(e.div_euclid(86_400));
            }
            None => undated += 1,
        }
    }
    undated + days.len()
}

fn has_support(
    portions: &[Portion],
    fallback_len: f64,
    config: &SectionConfig,
    corpus: usize,
    sport_tracks: &[(&str, &[GpsPoint])],
    starts: &HashMap<String, i64>,
    span_s: i64,
) -> bool {
    let mut lens: Vec<f64> = portions.iter().map(|p| p.3).collect();
    lens.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median_len = lens.get(lens.len() / 2).copied().unwrap_or(fallback_len);
    let required = required_visits_for_length(median_len, corpus) as usize;
    let (days, span, undated) = occasion_support(portions, sport_tracks, starts);
    if days < required.max(config.min_activities as usize) {
        return false;
    }
    // The span binds only when every contributing visit is dated: a
    // fully-known history that fits inside one stay is one visit to
    // the place, however many days it filled. Any unknown start
    // restores the classic activity-counting benefit of the doubt.
    match span {
        Some(s) if undated == 0 => s >= span_s,
        _ => true,
    }
}

/// A fine-resolution spatial index of every contributing portion's
/// points, keyed on a `cell_m` grid and carrying the track index and
/// coordinates. Built once per candidate so the representative choice
/// can ask, at metre resolution, how many distinct contributors run
/// near any point of a rendered line — a question the coarse coverage
/// grid (cell = proximity/2) cannot answer, because its one-ring
/// tolerance leaks a junction's through-traffic onto a branch that
/// diverges from it.
fn portion_point_index(
    portions: &[Portion],
    sport_tracks: &[(&str, &[GpsPoint])],
    ref_lat: f64,
    cell_m: f64,
) -> HashMap<Cell, Vec<(u32, f64, f64)>> {
    let m_lng = 111_320.0 * ref_lat.to_radians().cos();
    let mut idx: HashMap<Cell, Vec<(u32, f64, f64)>> = HashMap::new();
    for &(t, s, e, _) in portions {
        for p in &sport_tracks[t].1[s..e] {
            let c = (
                (p.latitude * 111_132.0 / cell_m).floor() as i32,
                (p.longitude * m_lng / cell_m).floor() as i32,
            );
            idx.entry(c)
                .or_default()
                .push((t as u32, p.latitude, p.longitude));
        }
    }
    idx
}

/// Longest contiguous run (metres) of a line's ~20 m samples on
/// minority ground: supported by fewer than half of `contributors`
/// distinct portions within `near_m` at metre resolution. A braid
/// variant captured by the coverage ring's lane width, or a private
/// tail, reads as a sustained run; a staircase jog or an end taper
/// reads as scattered single samples. The render choice prefers lines
/// without a sustained minority run — the drawn line is the section's
/// public face and must be where the majority went (rule 9's majority
/// principle applied to the DISPLAY).
fn minority_run_m(
    line: &[GpsPoint],
    index: &HashMap<Cell, Vec<(u32, f64, f64)>>,
    ref_lat: f64,
    near_m: f64,
    cell_m: f64,
    contributors: usize,
) -> f64 {
    const STEP_M: f64 = 20.0;
    if line.len() < 2 || contributors < 3 {
        return 0.0;
    }
    let m_lat = 111_132.0;
    let m_lng = 111_320.0 * ref_lat.to_radians().cos();
    let reach = (near_m / cell_m).ceil() as i32;
    let mut longest = 0.0f64;
    let mut run_start: Option<f64> = None;
    let mut acc = 0.0;
    let mut last = -STEP_M;
    for (i, p) in line.iter().enumerate() {
        if i > 0 {
            let (dx, dy) = (
                (p.latitude - line[i - 1].latitude) * m_lat,
                (p.longitude - line[i - 1].longitude) * m_lng,
            );
            acc += (dx * dx + dy * dy).sqrt();
        }
        if acc - last < STEP_M {
            continue;
        }
        last = acc;
        let (px, py) = (p.latitude * m_lat, p.longitude * m_lng);
        let c = (
            (p.latitude * m_lat / cell_m).floor() as i32,
            (p.longitude * m_lng / cell_m).floor() as i32,
        );
        let mut sup: HashSet<u32> = HashSet::new();
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                if let Some(v) = index.get(&(c.0 + dy, c.1 + dx)) {
                    for &(t, lat, lng) in v {
                        if ((lat * m_lat - px).powi(2) + (lng * m_lng - py).powi(2)).sqrt() < near_m
                        {
                            sup.insert(t);
                        }
                    }
                }
            }
        }
        if 2 * sup.len() < contributors {
            let start = *run_start.get_or_insert(acc);
            longest = longest.max(acc - start + STEP_M);
        } else {
            run_start = None;
        }
    }
    longest
}

/// The point-index range of a rendered line that keeps only its
/// commonly-traversed body, clipping a minority branch behind a cliff
/// at either end. The line is sampled every ~20 m; each sample's
/// support is the count of distinct contributors within `near_m`
/// (metre resolution, so a junction's through-traffic that diverges
/// from a branch does not prop the branch up the way the coarse
/// coverage grid's one-ring tolerance would). An end run is clipped
/// only when it is a genuine BRANCH, not a taper or a low-traffic
/// continuation:
///   * short — under [`BRANCH_MAX_M`] and a third of the line, so a
///     section with a genuinely lower-traffic half or a long tapering
///     end (uneven but legitimate support) is left whole;
///   * under half the line's median support, sustained past a lone
///     staggered-ending sample; and
///   * behind a cliff — the body sample adjoining the run carries at
///     least double it, so a gradual taper (which never doubles across
///     one step) is not read as a branch.
///
/// The drop is a wrong-way turn at a junction that most of the
/// section's traffic skips; it leaves the DISPLAY only. The extent,
/// counts, and occupied footprint keep the full portion.
fn minority_end_clip(
    line: &[GpsPoint],
    index: &HashMap<Cell, Vec<(u32, f64, f64)>>,
    ref_lat: f64,
    near_m: f64,
    cell_m: f64,
) -> (usize, usize) {
    const STEP_M: f64 = 20.0;
    const MIN_RUN: usize = 3;
    const BRANCH_MAX_M: f64 = 300.0;
    const MIN_MEDIAN: u32 = 8;
    if line.len() < 6 {
        return (0, line.len());
    }
    let m_lat = 111_132.0;
    let m_lng = 111_320.0 * ref_lat.to_radians().cos();
    let reach = (near_m / cell_m).ceil() as i32;
    let mut samples: Vec<(usize, u32)> = Vec::new();
    let mut acc = 0.0;
    let mut last = -STEP_M;
    for (i, p) in line.iter().enumerate() {
        if i > 0 {
            acc += crate::geo_utils::haversine_distance(&line[i - 1], p);
        }
        if acc - last < STEP_M && i + 1 < line.len() {
            continue;
        }
        last = acc;
        let c = (
            (p.latitude * m_lat / cell_m).floor() as i32,
            (p.longitude * m_lng / cell_m).floor() as i32,
        );
        let mut near: HashSet<u32> = HashSet::new();
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                if let Some(v) = index.get(&(c.0 + dy, c.1 + dx)) {
                    for &(t, la, ln) in v {
                        if f64::hypot((p.latitude - la) * m_lat, (p.longitude - ln) * m_lng)
                            < near_m
                        {
                            near.insert(t);
                        }
                    }
                }
            }
        }
        samples.push((i, near.len() as u32));
    }
    let n = samples.len();
    if n < 2 * MIN_RUN + 1 {
        return (0, line.len());
    }
    let mut sorted: Vec<u32> = samples.iter().map(|s| s.1).collect();
    sorted.sort_unstable();
    let median = sorted[sorted.len() / 2];
    if median < MIN_MEDIAN {
        return (0, line.len());
    }
    let max_run = (BRANCH_MAX_M / STEP_M) as usize;
    let frac_cap = n / 3;
    let minor = |k: usize| samples[k].1 * 2 < median;
    let mut lead = 0;
    while lead < n && minor(lead) {
        lead += 1;
    }
    let mut trail = 0;
    while trail < n && minor(n - 1 - trail) {
        trail += 1;
    }
    // A run is a clipped branch only if it is short, sustained, and its
    // highest sample is at most half the body sample abutting it (the
    // cliff): a gradual taper never doubles across one step, so it is
    // left whole.
    let lead_run_max = (0..lead).map(|k| samples[k].1).max().unwrap_or(0);
    let trail_run_max = (0..trail).map(|k| samples[n - 1 - k].1).max().unwrap_or(0);
    let lead_ok = lead >= MIN_RUN
        && lead <= max_run
        && lead <= frac_cap
        && lead < n
        && samples[lead].1 >= 2 * lead_run_max.max(1);
    let trail_ok = trail >= MIN_RUN
        && trail <= max_run
        && trail <= frac_cap
        && lead + trail < n
        && samples[n - 1 - trail].1 >= 2 * trail_run_max.max(1);
    let start = if lead_ok { samples[lead].0 } else { 0 };
    let end = if trail_ok {
        samples[n - trail].0
    } else {
        line.len()
    };
    (start.min(end.saturating_sub(2)), end)
}

/// Clip a render's ends back to supported ground: a SUSTAINED end run
/// whose metre-resolution support among the section's own contributors
/// is below `min_tracks` extends over ground that can never be hot — a
/// one-walker overshoot past the usage change, kept only because the
/// node's boundary cell is coarser than the change. End of support is
/// a visible boundary (rule 9). The run must be sustained (`MIN_RUN`
/// samples, as in [`minority_end_clip`]): a sample or two of thin end
/// taper is receiver-noise staggering of where contributors start and
/// stop, and clipping it would make each sport bucket's render end
/// drift by its own stagger — enough to push the identity carry off a
/// row another bucket minted on the same ground. Display-only,
/// ends-only: mid-line support is rule B's business.
fn support_end_clip(
    line: &[GpsPoint],
    index: &HashMap<Cell, Vec<(u32, f64, f64)>>,
    ref_lat: f64,
    near_m: f64,
    cell_m: f64,
    min_tracks: u32,
) -> (usize, usize) {
    const STEP_M: f64 = 20.0;
    if line.len() < 6 {
        return (0, line.len());
    }
    let m_lat = 111_132.0;
    let m_lng = 111_320.0 * ref_lat.to_radians().cos();
    let reach = (near_m / cell_m).ceil() as i32;
    let mut samples: Vec<(usize, u32)> = Vec::new();
    let mut acc = 0.0;
    let mut last = -STEP_M;
    for (i, p) in line.iter().enumerate() {
        if i > 0 {
            acc += crate::geo_utils::haversine_distance(&line[i - 1], p);
        }
        if acc - last < STEP_M && i + 1 < line.len() {
            continue;
        }
        last = acc;
        let c = (
            (p.latitude * m_lat / cell_m).floor() as i32,
            (p.longitude * m_lng / cell_m).floor() as i32,
        );
        let mut near: HashSet<u32> = HashSet::new();
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                if let Some(v) = index.get(&(c.0 + dy, c.1 + dx)) {
                    for &(t, la, ln) in v {
                        if f64::hypot((p.latitude - la) * m_lat, (p.longitude - ln) * m_lng)
                            < near_m
                        {
                            near.insert(t);
                        }
                    }
                }
            }
        }
        samples.push((i, near.len() as u32));
    }
    let n = samples.len();
    if n < 3 {
        return (0, line.len());
    }
    let mut lead = 0;
    while lead < n && samples[lead].1 < min_tracks {
        lead += 1;
    }
    if lead >= n {
        return (0, line.len());
    }
    let mut trail = 0;
    while trail < n - lead && samples[n - 1 - trail].1 < min_tracks {
        trail += 1;
    }
    const MIN_RUN: usize = 3;
    if lead < MIN_RUN {
        lead = 0;
    }
    if trail < MIN_RUN {
        trail = 0;
    }
    let start = if lead > 0 {
        samples[lead.min(n - 1)].0
    } else {
        0
    };
    let end = if trail > 0 {
        samples[n - trail].0
    } else {
        line.len()
    };
    (start.min(end.saturating_sub(2)), end)
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
    starts: &HashMap<String, i64>,
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
    let mut cell_tracks_local: HashMap<Cell, Vec<&str>> = HashMap::new();
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
        let id = sport_tracks[t].0;
        for c in dilated {
            cell_tracks_local.entry(c).or_default().push(id);
        }
    }
    cell_tracks_local
        .into_iter()
        .map(|(c, ids)| (c, cell_support_days(&ids, starts) as u32))
        .collect()
}

/// Which components would survive as sections on their own evidence.
#[allow(clippy::too_many_arguments)]
fn section_worthiness(
    supernodes: &[Supernode],
    coverage: &CoverageGrid,
    sport_tracks: &[(&str, &[GpsPoint])],
    config: &SectionConfig,
    cell_size: f64,
    tun: &Tunables,
    leaves: &mut LeafMemos,
    starts: &HashMap<String, i64>,
) -> Vec<bool> {
    supernodes
        .iter()
        .map(|n| {
            let portions =
                portions_for_memo(leaves, n, coverage, sport_tracks, config, cell_size, tun);
            has_support(
                &portions,
                n.cells.len() as f64 * cell_size,
                config,
                opportunity(n, coverage),
                sport_tracks,
                starts,
                (tun.occasion_span_h * 3600.0) as i64,
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
/// Every bar a would-be candidate must clear before it may compete for
/// a render: whole-candidate support, per-cell support pruning, and the
/// post-prune gates. One implementation shared by the supernode sweep
/// and by remainder re-queue, so ground a render could not draw faces
/// exactly the bars a fresh candidate does.
#[allow(clippy::too_many_arguments)]
fn qualify_candidate(
    node: Supernode,
    mut portions: Vec<Portion>,
    coverage: &CoverageGrid,
    sport_tracks: &[(&str, &[GpsPoint])],
    config: &SectionConfig,
    cell_size: f64,
    tun: &Tunables,
    starts: &HashMap<String, i64>,
    span_s: i64,
    records: &mut Vec<BoundaryRecord>,
) -> Option<(Supernode, Vec<Portion>, f64)> {
    let approx_len = node.cells.len() as f64 * cell_size;
    if portions.is_empty()
        || !has_support(
            &portions,
            approx_len,
            config,
            opportunity(&node, coverage),
            sport_tracks,
            starts,
            span_s,
        )
    {
        return None;
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
    let required = required_visits_for_length(median_len, opportunity(&node, coverage)) as usize;
    let floor = required.max(config.min_activities as usize) as u32;
    let pass_min_m = (2.0 * cell_size).min(0.5 * median_len);
    let mut node = node;
    loop {
        let cell_set: HashSet<Cell> = node
            .cells
            .iter()
            .flat_map(|c| {
                (-1..=1i32).flat_map(move |dy| (-1..=1i32).map(move |dx| (c.0 + dy, c.1 + dx)))
            })
            .collect();
        let support = candidate_support(
            &portions,
            &cell_set,
            coverage,
            sport_tracks,
            pass_min_m,
            starts,
        );
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
        portions = portions_for(&node, coverage, sport_tracks, config, cell_size, tun);
        if portions.is_empty() {
            break;
        }
    }
    let approx_len = node.cells.len() as f64 * cell_size;
    if portions.is_empty()
        || approx_len < config.min_section_length
        || !has_support(
            &portions,
            approx_len,
            config,
            opportunity(&node, coverage),
            sport_tracks,
            starts,
            span_s,
        )
    {
        return None;
    }
    Some((node, portions, score))
}

/// Whether an accepted section's traffic carries the candidate's:
/// containment of the candidate's set at the same-traffic bar. A braid
/// twin or chain neighbour shares the users whose ground it claims; a
/// distinct population's parallel path — the other bank of a river a
/// lane's width away in plan — does not, and a line its users never
/// run cannot represent their corridor.
fn shares_traffic(cand: &HashSet<u32>, acc: &HashSet<u32>, bar: f64) -> bool {
    if cand.is_empty() {
        return true;
    }
    let inter = cand.iter().filter(|t| acc.contains(t)).count();
    inter as f64 >= bar * cand.len() as f64
}

/// Ground the candidate's own tracks LAP (a class-3 pass) is
/// revolutions: an accepted through-line a braid width away may carry
/// the same users' approaches, but it cannot represent their laps.
fn probe_mask(
    probe: &[GpsPoint],
    accepted: &HashMap<Cell, Vec<(GpsPoint, u32)>>,
    acc_tracks: &[HashSet<u32>],
    cand: &HashSet<u32>,
    own_lapped: &dyn Fn(&GpsPoint) -> bool,
    same_traffic: f64,
    grid: &CellGrid,
    cell_size: f64,
) -> Vec<bool> {
    probe
        .iter()
        .map(|p| {
            if own_lapped(p) {
                return false;
            }
            let c = grid.cell_of(p.latitude, p.longitude);
            (-1..=1i32).any(|dy| {
                (-1..=1i32).any(|dx| {
                    accepted.get(&(c.0 + dy, c.1 + dx)).is_some_and(|v| {
                        v.iter().any(|(q, ai)| {
                            crate::geo_utils::haversine_distance(p, q) < cell_size
                                && shares_traffic(cand, &acc_tracks[*ai as usize], same_traffic)
                        })
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
    tun: &Tunables,
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
                // A cover must not be dirtier than the line it
                // replaces: the shared trace can spin inside the
                // member's extent where the member's own render was
                // fold-free, and a tight fold hides inside both the
                // lateral envelope and the length share.
                let pen_near = cell_size * 0.2;
                let cut_pen = self_pass_penalty(cut, pen_near, cell_size)
                    .max(out_and_back_penalty(cut, pen_near, cell_size));
                let own_pen = self_pass_penalty(g, pen_near, cell_size)
                    .max(out_and_back_penalty(g, pen_near, cell_size));
                if cut_pen > own_pen.max(tun.self_pass_clean) {
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
#[allow(clippy::too_many_arguments)]
fn detect_for_sport(
    sport: &str,
    sport_tracks: &[(&str, &[GpsPoint])],
    sport_seconds: &[&[f64]],
    config: &SectionConfig,
    tun: &Tunables,
    section_idx: &mut usize,
    records: &mut Vec<BoundaryRecord>,
    starts: &HashMap<String, i64>,
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
            starts,
        ));
    }
    sections
}

/// One geographic cluster's pipeline: coverage grid → same-traffic
/// supernodes → fixed point → candidates → selection backoff.
///
/// The body past the grid is [`detect_for_cluster_with_grid`], shared
/// verbatim with the cached incremental
/// ([`detect_sections_unified_incremental_cached`]) so a touched cluster's
/// recompute is byte-identical to the batch's.
#[allow(clippy::too_many_arguments)]
fn detect_for_cluster(
    sport: &str,
    sport_tracks: &[(&str, &[GpsPoint])],
    sport_seconds: &[&[f64]],
    config: &SectionConfig,
    tun: &Tunables,
    section_idx: &mut usize,
    records: &mut Vec<BoundaryRecord>,
    starts: &HashMap<String, i64>,
) -> Vec<FrequentSection> {
    if sport_tracks.len() < config.min_activities as usize {
        return Vec::new();
    }
    let cell_size = cluster_cell_size(config);
    let mut leaves = LeafMemos::default();
    let coverage = build_coverage_grid(
        sport_tracks,
        sport_seconds,
        cell_size,
        tun,
        &mut leaves.lift_candidates,
    );
    detect_for_cluster_with_grid(
        sport,
        sport_tracks,
        &coverage,
        cell_size,
        config,
        tun,
        section_idx,
        records,
        &mut leaves,
        starts,
    )
}

/// The discovery half of [`detect_for_cluster`], run against an
/// already-built [`CoverageGrid`]. A pure function of `(coverage,
/// sport_tracks)`: given the same grid and the same track ordering it
/// emits byte-identical sections. Both the batch and the cached
/// incremental build the grid ([`build_coverage_grid`]) then call this, so
/// a touched cluster's recompute matches the batch exactly.
#[allow(clippy::too_many_arguments)]
fn detect_for_cluster_with_grid(
    sport: &str,
    sport_tracks: &[(&str, &[GpsPoint])],
    coverage: &CoverageGrid,
    cell_size: f64,
    config: &SectionConfig,
    tun: &Tunables,
    section_idx: &mut usize,
    records: &mut Vec<BoundaryRecord>,
    leaves: &mut LeafMemos,
    starts: &HashMap<String, i64>,
) -> Vec<FrequentSection> {
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
    let span_s = (tun.occasion_span_h * 3600.0) as i64;
    let same_traffic = 1.0 - divergence;

    let mut supernodes = partition_supernodes(&hot_cells, coverage, same_traffic);

    // Two rules applied to a fixed point:
    //  * a component that cannot be a section is not a barrier — absorb it
    //  * a boundary only survives where the diverging branch is itself a
    //    section. A cut whose branch never surfaces is invisible on the
    //    map: the corridor stops for no reason the athlete can see.
    // Absorbing and merging can promote a component, so iterate.
    let mut worthy = section_worthiness(
        &supernodes,
        coverage,
        sport_tracks,
        config,
        cell_size,
        tun,
        leaves,
        starts,
    );
    for _ in 0..5 {
        let before = supernodes.len();
        supernodes = merge_non_fork_boundaries(
            supernodes,
            coverage,
            divergence,
            config.min_activities,
            &worthy,
        );
        if supernodes.len() == before {
            // Nothing merged, so `worthy` still describes exactly these
            // components; the explanation pass below reuses it.
            break;
        }
        worthy = section_worthiness(
            &supernodes,
            coverage,
            sport_tracks,
            config,
            cell_size,
            tun,
            leaves,
            starts,
        );
    }

    // Every boundary that survived explains itself, as data.
    explain_boundaries(
        &supernodes,
        coverage,
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
    //
    // Ground whose candidate dies — under the length floor, at
    // qualification, or backed off during selection — is not discarded:
    // its cells pool in `orphaned` and re-enter the queue once the
    // first pass drains. The strict same-traffic partition shreds a
    // corridor at every junction cell whose track set a crossing path
    // inflates, and the shreds die one by one while the through
    // traffic they carry is real. The pool re-partitions at
    // through-traffic granularity so junction shreds reassemble, then
    // every pooled candidate faces the same floors, probes, and
    // backoff as any other — one retry per cell, so the salvage
    // terminates.
    let mut orphaned: std::collections::BTreeSet<Cell> = std::collections::BTreeSet::new();
    let mut candidates: Vec<(usize, Supernode, Vec<Portion>, f64)> = Vec::new();
    for (n_idx, node) in supernodes.iter().enumerate() {
        // Rough length from core cell count (cells are ~square).
        let approx_len = node.cells.len() as f64 * cell_size;
        if approx_len < config.min_section_length {
            orphaned.extend(node.cells.iter().copied());
            continue;
        }

        let portions =
            portions_for_memo(leaves, node, coverage, sport_tracks, config, cell_size, tun);
        let owned = Supernode {
            cells: node.cells.clone(),
            tracks: node.tracks.clone(),
        };
        if let Some(qualified) = qualify_candidate(
            owned,
            portions,
            coverage,
            sport_tracks,
            config,
            cell_size,
            tun,
            starts,
            span_s,
            records,
        ) {
            let (node, portions, score) = qualified;
            candidates.push((n_idx, node, portions, score));
        } else {
            orphaned.extend(node.cells.iter().copied());
        }
    }
    candidates.sort_by(|a, b| {
        b.3.partial_cmp(&a.3)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.0.cmp(&b.0))
    });

    let backoff_grid = CellGrid::new(cell_size, coverage.ref_lat);
    let mut accepted_pts: HashMap<Cell, Vec<(GpsPoint, u32)>> = HashMap::new();
    let mut acc_tracks: Vec<HashSet<u32>> = Vec::new();
    let mut emitted_portions: Vec<Vec<Portion>> = Vec::new();
    let mut queue: std::collections::VecDeque<(usize, Supernode, Vec<Portion>, f64)> =
        candidates.into_iter().collect();
    let mut pooled_ever: HashSet<Cell> = HashSet::new();
    loop {
        let Some((mark, node, portions, score)) = queue.pop_front() else {
            // The pass drained: pool the orphaned ground and retry it.
            // Cells already retried once and cells the accepted lines
            // now stand on are dropped, so the salvage strictly
            // shrinks and terminates.
            let pool: Vec<Cell> = orphaned
                .iter()
                .copied()
                .filter(|c| !pooled_ever.contains(c))
                .filter(|c| {
                    let (la, ln) = coverage.grid.centre_of(*c);
                    let empty = HashSet::new();
                    let cell_cand = coverage.cell_tracks.get(c).unwrap_or(&empty);
                    !accepted_pts
                        .get(&backoff_grid.cell_of(la, ln))
                        .is_some_and(|v| {
                            v.iter().any(|(_, ai)| {
                                shares_traffic(cell_cand, &acc_tracks[*ai as usize], same_traffic)
                            })
                        })
                })
                .collect();
            orphaned.clear();
            if pool.is_empty() {
                break;
            }
            pooled_ever.extend(pool.iter().copied());
            for remainder in partition_pooled(&pool, coverage, same_traffic) {
                if (remainder.cells.len() as f64) * cell_size < config.min_section_length {
                    continue;
                }
                let rem_portions =
                    portions_for(&remainder, coverage, sport_tracks, config, cell_size, tun);
                let Some((rem_node, rem_portions, rem_score)) = qualify_candidate(
                    remainder,
                    rem_portions,
                    coverage,
                    sport_tracks,
                    config,
                    cell_size,
                    tun,
                    starts,
                    span_s,
                    records,
                ) else {
                    continue;
                };
                queue.push_back((usize::MAX, rem_node, rem_portions, rem_score));
            }
            if queue.is_empty() {
                break;
            }
            continue;
        };
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
        let own_lapped = |p: &GpsPoint| {
            let c = coverage.grid.cell_of(p.latitude, p.longitude);
            coverage
                .cell_passes
                .get(&c)
                .is_some_and(|m| m.iter().any(|(t, &cl)| cl >= 3 && node.tracks.contains(t)))
        };
        let mut mask = probe_mask(
            probe,
            &accepted_pts,
            &acc_tracks,
            &node.tracks,
            &own_lapped,
            same_traffic,
            &backoff_grid,
            cell_size,
        );
        // The mouth of a lapped feature blends stem and lap ground, so
        // its cells read single-pass while the revolutions start a
        // step away along the SAME pass: a masked point within one
        // cell's arc of the candidate's own lapped ground is the way
        // in and out of its revolutions, not represented corridor.
        // Arc reach, never plan reach — a separate stem candidate a
        // ring away gains nothing.
        if mask.iter().any(|&m| m) {
            let strict: Vec<bool> = probe.iter().map(|p| own_lapped(p)).collect();
            if strict.iter().any(|&s| s) {
                let mut cum2 = Vec::with_capacity(probe.len());
                let mut acc2 = 0.0;
                cum2.push(0.0);
                for w in probe.windows(2) {
                    acc2 += crate::geo_utils::haversine_distance(&w[0], &w[1]);
                    cum2.push(acc2);
                }
                let mut near_strict = vec![f64::INFINITY; probe.len()];
                let mut last = f64::NEG_INFINITY;
                for i in 0..probe.len() {
                    if strict[i] {
                        last = cum2[i];
                    }
                    near_strict[i] = cum2[i] - last;
                }
                let mut nxt = f64::INFINITY;
                for i in (0..probe.len()).rev() {
                    if strict[i] {
                        nxt = cum2[i];
                    }
                    near_strict[i] = near_strict[i].min(nxt - cum2[i]);
                }
                for i in 0..mask.len() {
                    if mask[i] && near_strict[i] <= cell_size {
                        mask[i] = false;
                    }
                }
            }
        }
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
                orphaned.extend(node.cells.iter().copied());
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
                    portions_for(&reduced, coverage, sport_tracks, config, cell_size, tun);
                if reduced.cells.is_empty()
                    || trimmed.is_empty()
                    || !has_support(
                        &trimmed,
                        approx,
                        config,
                        opportunity(&reduced, coverage),
                        sport_tracks,
                        starts,
                        span_s,
                    )
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
                    orphaned.extend(node.cells.iter().copied());
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

        let ckey = ConsensusKey {
            portions: {
                let mut v: Vec<(String, usize, usize)> = portions
                    .iter()
                    .map(|&(t, ps, pe, _)| (sport_tracks[t].0.to_string(), ps, pe))
                    .collect();
                v.sort();
                v
            },
            sport: sport.to_string(),
            proximity_bits: config.proximity_threshold.to_bits(),
            min_activities: config.min_activities,
            max_len_bits: config.max_section_length.to_bits(),
        };
        let processed = match leaves.consensus.get(&ckey).cloned() {
            Some(hit) => hit.map(|mut sec| {
                sec.id = format!("sec_{}_{}", sport.to_lowercase(), *section_idx);
                sec
            }),
            None => {
                let fresh = process_cluster(
                    *section_idx,
                    cluster,
                    sport,
                    &track_map,
                    &activity_to_route,
                    config,
                    None,
                );
                leaves.consensus.insert(
                    ckey,
                    fresh.clone().map(|mut sec| {
                        sec.id = String::new();
                        sec
                    }),
                );
                fresh
            }
        };
        if let Some(mut section) = processed {
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
                    self_pass_penalty(seg, near, gap).max(out_and_back_penalty(seg, near, gap))
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
            // The default render (longest/medoid) keeps its stability
            // privilege only while essentially clean AND on majority
            // ground. Otherwise the render falls to the best legal
            // pass: majority-faithful first, then cleanest, then
            // longest — a visibly folded or minority-walking default
            // must not be the section's public face. A candidate whose
            // every pass is over the fold floor backs off as a blob.
            let fine = portion_point_index(&portions, sport_tracks, coverage.ref_lat, 25.0);
            let runs: Vec<f64> = portions
                .iter()
                .map(|&(t, s, e, _)| {
                    minority_run_m(
                        &sport_tracks[t].1[s..e],
                        &fine,
                        coverage.ref_lat,
                        25.0,
                        25.0,
                        portions.len(),
                    )
                })
                .collect();
            let faithful = |i: usize| runs[i] < tun.minority_run_m;
            let legal = |i: usize| pens[i] <= tun.self_pass_max;
            // A faithful default under the clean bar keeps its
            // stability privilege. Otherwise the render falls to the
            // legal pass with the shortest sustained minority run,
            // cleanest then longest among ties: a variant-walking or
            // silent-ground-bridging line must not be the section's
            // public face when a truer one exists. A candidate with no
            // legal pass at all backs off as a blob.
            let chosen = match default_i {
                Some(d) if legal(d) && faithful(d) && pens[d] <= tun.self_pass_clean => Some(d),
                _ => (0..portions.len()).filter(|&i| legal(i)).min_by(|&a, &b| {
                    runs[a]
                        .partial_cmp(&runs[b])
                        .unwrap_or(std::cmp::Ordering::Equal)
                        .then(
                            pens[a]
                                .partial_cmp(&pens[b])
                                .unwrap_or(std::cmp::Ordering::Equal),
                        )
                        .then(
                            portions[b]
                                .3
                                .partial_cmp(&portions[a].3)
                                .unwrap_or(std::cmp::Ordering::Equal),
                        )
                }),
            };
            let Some(i) = chosen else {
                // No pass is a single traversal: the candidate backs off,
                // but its footprint still occupies the ground so a
                // neighbour cannot re-expand into a junction with no line.
                let ai = acc_tracks.len() as u32;
                acc_tracks.push(portions.iter().map(|p| p.0 as u32).collect());
                for p in footprint.iter().step_by(3) {
                    accepted_pts
                        .entry(backoff_grid.cell_of(p.latitude, p.longitude))
                        .or_default()
                        .push((*p, ai));
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
            // Trim a minority end-branch off the DISPLAY: a stretch at
            // one end that most of the section's own traffic skips (a
            // wrong-way turn at a junction) reads thin at metre
            // resolution while the body stays busy. The drawn line is
            // still one real single pass — a sub-range of the same
            // trace — and the extent, counts, and occupied footprint
            // below keep the full portion, so nothing downstream shifts.
            let (cs, ce) = minority_end_clip(
                &sport_tracks[t_idx].1[s..e],
                &fine,
                coverage.ref_lat,
                25.0,
                25.0,
            );
            let (rs, re) = if cs > 0 || ce < e - s {
                let kept = &sport_tracks[t_idx].1[s + cs..s + ce];
                if crate::matching::calculate_route_distance(kept) >= config.min_section_length {
                    (s + cs, s + ce)
                } else {
                    (s, e)
                }
            } else {
                (s, e)
            };
            // A RE-QUEUED candidate's drawn ends must sit on supported
            // ground: its node was assembled from freed cells and
            // salvage pools, so its portions carry ring bleed past the
            // real usage change — a one-walker overshoot that survives
            // the clips above only because the node's boundary cell is
            // coarser than the change. First-pass candidates keep
            // their renders untouched: their extents came through the
            // boundary machinery, and their long-reviewed lines must
            // not drift with a display rule aimed at salvage bleed.
            let (rs, re) = if mark == usize::MAX {
                let seg = &sport_tracks[t_idx].1[rs..re];
                let (us, ue) = support_end_clip(
                    seg,
                    &fine,
                    coverage.ref_lat,
                    25.0,
                    25.0,
                    config.min_activities,
                );
                if (us, ue) != (0, seg.len())
                    && crate::matching::calculate_route_distance(&seg[us..ue])
                        < config.min_section_length
                {
                    // A salvaged candidate whose supported core falls
                    // under the length floor has no honest line: it
                    // backs off instead of rendering a stub.
                    let mid = seg[seg.len() / 2];
                    records.push(BoundaryRecord {
                        latitude: mid.latitude,
                        longitude: mid.longitude,
                        reason: BoundaryReason::LowSupport {
                            floor: config.min_activities,
                            dropped_cells: node.cells.len() as u32,
                        },
                    });
                    orphaned.extend(node.cells.iter().copied());
                    continue;
                }
                (rs + us, rs + ue.min(seg.len()))
            } else {
                (rs, re)
            };
            // A pass that terminates by closing onto its own interior
            // renders the LOOP: lapped ground's honest face is the
            // revolution, and the stem it was reached by re-enters the
            // queue with the rest of the undrawn ground below. Only
            // LAPPED ground reads this way — the loop's cells must
            // carry a class-3 pass from somebody's revolutions
            // (class 2 is any out-and-back; a corridor travelled
            // there-and-home must not read as a circuit). A winding
            // corridor that merely coils back within a lane's width of
            // itself is single-passed everywhere and keeps its full
            // render.
            let lapped = |t: usize, ls: usize, le: usize| {
                let track = sport_tracks[t].1;
                let mut loop_cells: Vec<Cell> = track[ls..le]
                    .iter()
                    .map(|p| coverage.grid.cell_of(p.latitude, p.longitude))
                    .collect();
                loop_cells.dedup();
                loop_cells.sort_unstable();
                loop_cells.dedup();
                let multi = loop_cells
                    .iter()
                    .filter(|c| {
                        coverage
                            .cell_passes
                            .get(c)
                            .is_some_and(|m| m.values().any(|&cl| cl >= 3))
                    })
                    .count();
                2 * multi >= loop_cells.len()
            };
            // Mean nearest-neighbour distance from a loop's sampled
            // points to OTHER contributors' portion points: how well
            // this revolution agrees with everyone else's laps.
            // Distances beyond the fine index's reach read as far.
            let loop_agreement = |t: usize, ls: usize, le: usize| -> f64 {
                const STEP_M: f64 = 20.0;
                const FAR_M: f64 = 75.0;
                let m_lat = 111_132.0;
                let m_lng = 111_320.0 * coverage.ref_lat.to_radians().cos();
                let pts = &sport_tracks[t].1[ls..le];
                let mut tot = 0.0;
                let mut n = 0usize;
                let mut acc = 0.0;
                let mut last = -STEP_M;
                for (k, p) in pts.iter().enumerate() {
                    if k > 0 {
                        acc += crate::geo_utils::haversine_distance(&pts[k - 1], p);
                    }
                    if acc - last < STEP_M && k + 1 < pts.len() {
                        continue;
                    }
                    last = acc;
                    let c = (
                        (p.latitude * m_lat / 25.0).floor() as i32,
                        (p.longitude * m_lng / 25.0).floor() as i32,
                    );
                    let mut best = FAR_M;
                    for dy in -1..=1i32 {
                        for dx in -1..=1i32 {
                            if let Some(v) = fine.get(&(c.0 + dy, c.1 + dx)) {
                                for &(tt, la, ln) in v {
                                    if tt as usize == t {
                                        continue;
                                    }
                                    let d = f64::hypot(
                                        (p.latitude - la) * m_lat,
                                        (p.longitude - ln) * m_lng,
                                    );
                                    if d < best {
                                        best = d;
                                    }
                                }
                            }
                        }
                    }
                    tot += best;
                    n += 1;
                }
                if n == 0 {
                    f64::INFINITY
                } else {
                    tot / n as f64
                }
            };
            let (t_idx, rs, re) = {
                let track = sport_tracks[t_idx].1;
                // The extension bound only matters once a genuine
                // closure exists and the lapped gate holds, so it can
                // afford a partial arc: a trim at the mouth can leave
                // barely half a small ring, and completing the
                // revolution retraces the candidate's own lapped
                // ground.
                match closing_loop_range(track, rs, re, near, gap, 4.0 * cell_size) {
                    Some((ls, le))
                        if crate::matching::calculate_route_distance(&track[ls..le])
                            >= config.min_section_length
                            && lapped(t_idx, ls, le) =>
                    {
                        // The chosen pass's revolution keeps its
                        // privilege while it agrees with the other
                        // laps at metre resolution. An outlier lap —
                        // one that weaves off the circuit in bursts
                        // too short for the sustained-minority bar —
                        // is displaced by the legal pass whose
                        // revolution agrees best.
                        let mut pick = (t_idx, rs, re, ls, le, loop_agreement(t_idx, ls, le));
                        if pick.5 > 25.0 {
                            for (j, &(tj, sj, ej, _)) in portions.iter().enumerate() {
                                if j == i || !legal(j) {
                                    continue;
                                }
                                let Some((ls2, le2)) = closing_loop_range(
                                    sport_tracks[tj].1,
                                    sj,
                                    ej,
                                    near,
                                    gap,
                                    4.0 * cell_size,
                                ) else {
                                    continue;
                                };
                                if crate::matching::calculate_route_distance(
                                    &sport_tracks[tj].1[ls2..le2],
                                ) < config.min_section_length
                                    || !lapped(tj, ls2, le2)
                                {
                                    continue;
                                }
                                let sc = loop_agreement(tj, ls2, le2);
                                if sc + 1e-9 < pick.5 {
                                    pick = (tj, sj, ej, ls2, le2, sc);
                                }
                            }
                        }
                        let (tw, fs, fe, ls, le, _) = pick;
                        let wtrack = sport_tracks[tw].1;
                        let kept_m = crate::matching::calculate_route_distance(&wtrack[ls..le]);
                        let full_m = crate::matching::calculate_route_distance(&wtrack[fs..fe]);
                        for cut in [(ls > fs).then_some(ls), (le < fe).then_some(le - 1)]
                            .into_iter()
                            .flatten()
                        {
                            records.push(BoundaryRecord {
                                latitude: wtrack[cut].latitude,
                                longitude: wtrack[cut].longitude,
                                reason: BoundaryReason::Trim {
                                    kept_metres: kept_m,
                                    dropped_metres: (full_m - kept_m).max(0.0),
                                },
                            });
                        }
                        (tw, ls, le)
                    }
                    _ => (t_idx, rs, re),
                }
            };
            // When even the chosen pass folds (no clean pass exists in
            // the whole candidate), the DISPLAY keeps its longest
            // fold-free stretch rather than drawing the knot: interval
            // reps over a corridor render as the corridor, not the
            // reps. Judged on the final clipped line's OWN score — the
            // clips above change the denominator, and a long portion
            // under the clean bar can hide a knot that dominates its
            // clipped sub-range. Display-only, like the clips above:
            // the extent, counts, and occupied footprint keep the full
            // portion, and the undrawn ground re-enters the queue on
            // its own merits below.
            let (rs, re) = {
                let seg = &sport_tracks[t_idx].1[rs..re];
                let seg_pen =
                    self_pass_penalty(seg, near, gap).max(out_and_back_penalty(seg, near, gap));
                if seg_pen <= tun.self_pass_clean {
                    (rs, re)
                } else {
                    let (fs, fe) = longest_clean_range(seg, near, gap);
                    let kept_m = crate::matching::calculate_route_distance(&seg[fs..fe]);
                    if (fs, fe) != (0, seg.len()) && kept_m >= config.min_section_length {
                        let full_m = crate::matching::calculate_route_distance(seg);
                        for cut in [(fs > 0).then_some(fs), (fe < seg.len()).then_some(fe - 1)]
                            .into_iter()
                            .flatten()
                        {
                            records.push(BoundaryRecord {
                                latitude: seg[cut].latitude,
                                longitude: seg[cut].longitude,
                                reason: BoundaryReason::Trim {
                                    kept_metres: kept_m,
                                    dropped_metres: full_m - kept_m,
                                },
                            });
                        }
                        (rs + fs, rs + fe)
                    } else {
                        (rs, re)
                    }
                }
            };
            // Backoff binds on the RENDER, not only on the probe: the
            // probe walked one portion, but the trim's reduced node and
            // the faithfulness/fold displacement can hand the render to
            // a DIFFERENT pass, one that sweeps the ring straight back
            // over accepted ground. A line adding less than a section's
            // worth of unrepresented travel is a duplicate of what is
            // already drawn, whatever its probe said.
            let render = &sport_tracks[t_idx].1[rs..re];
            let own_lapped = |p: &GpsPoint| {
                let c = coverage.grid.cell_of(p.latitude, p.longitude);
                coverage
                    .cell_passes
                    .get(&c)
                    .is_some_and(|m| m.iter().any(|(t, &cl)| cl >= 3 && node.tracks.contains(t)))
            };
            let render_mask = probe_mask(
                render,
                &accepted_pts,
                &acc_tracks,
                &node.tracks,
                &own_lapped,
                same_traffic,
                &backoff_grid,
                cell_size,
            );
            let fresh_m: f64 = render
                .windows(2)
                .enumerate()
                .filter(|(k, _)| !render_mask[*k] && !render_mask[k + 1])
                .map(|(_, w)| crate::geo_utils::haversine_distance(&w[0], &w[1]))
                .sum();
            if render_mask.iter().any(|&m| m) && fresh_m < config.min_section_length {
                let mid = render[render.len() / 2];
                records.push(BoundaryRecord {
                    latitude: mid.latitude,
                    longitude: mid.longitude,
                    reason: BoundaryReason::Backoff {
                        represented: render_mask.iter().filter(|&&m| m).count() as u32,
                        probed: render_mask.len() as u32,
                        score_metres: score,
                    },
                });
                orphaned.extend(node.cells.iter().copied());
                continue;
            }
            section.polyline = sport_tracks[t_idx].1[rs..re].to_vec();
            section.distance_meters = if (rs, re) == (s, e) {
                dist
            } else {
                crate::matching::calculate_route_distance(&section.polyline)
            };
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
            // Occupy the ground of the line the section actually SHOWS,
            // not the default portion's. The default can run wide of
            // the render (a trimmed candidate's longest pass walks
            // every ring-captured excursion, and rules B and D displace
            // unfaithful or folded defaults), and ground occupied by a
            // line nobody sees pinches real neighbours into backing off
            // with nothing drawn in their place. The old default-line
            // occupation guarded a junction re-rendering short and
            // bleeding its milling into a re-expanding neighbour; the
            // faithfulness and fold guards on every render now police
            // that directly.
            // A section's population is its CONTRIBUTORS, not its node's
            // cell union: a walking loop whose cells graze the oval must
            // not claim to carry the runners lapping inside it.
            let ai = acc_tracks.len() as u32;
            acc_tracks.push(portions.iter().map(|p| p.0 as u32).collect());
            for p in section.polyline.iter().step_by(3) {
                accepted_pts
                    .entry(backoff_grid.cell_of(p.latitude, p.longitude))
                    .or_default()
                    .push((*p, ai));
            }
            // Rule 6 draws ONE real pass, so a corridor no one crosses
            // end to end can never be drawn by a single section — and
            // this node may own far more ground than its line reaches
            // (a displaced default, or a valley whose longest pass
            // spans a third of it). That ground re-enters the queue on
            // its own merits: same floors, backoff against everything
            // accepted. Cells near any accepted geometry are dropped
            // rather than re-queued (they would only back off), so
            // every re-queue shrinks and the loop terminates.
            let mut represented: HashSet<Cell> = HashSet::new();
            for p in &section.polyline {
                let c = coverage.grid.cell_of(p.latitude, p.longitude);
                for dy in -1..=1i32 {
                    for dx in -1..=1i32 {
                        represented.insert((c.0 + dy, c.1 + dx));
                    }
                }
            }
            let mut free: Vec<Cell> = node
                .cells
                .iter()
                .copied()
                .filter(|c| !represented.contains(c))
                .filter(|c| {
                    let (la, ln) = coverage.grid.centre_of(*c);
                    let bc = backoff_grid.cell_of(la, ln);
                    let empty = HashSet::new();
                    let cell_cand = coverage.cell_tracks.get(c).unwrap_or(&empty);
                    !(-1..=1i32).any(|dy| {
                        (-1..=1i32).any(|dx| {
                            accepted_pts.get(&(bc.0 + dy, bc.1 + dx)).is_some_and(|v| {
                                v.iter().any(|(_, ai)| {
                                    shares_traffic(
                                        cell_cand,
                                        &acc_tracks[*ai as usize],
                                        same_traffic,
                                    )
                                })
                            })
                        })
                    })
                })
                .collect();
            free.sort_unstable();
            if (free.len() as f64) * cell_size >= config.min_section_length {
                // The freed ground is re-partitioned the same way all
                // ground is — same-traffic supernodes — never by bare
                // adjacency: a corner gluing a 3-track street to a
                // 5-track street would otherwise pool their traffic
                // into one node and borrow a longer piece's softer
                // floor.
                for remainder in partition_supernodes(&free, coverage, same_traffic) {
                    if (remainder.cells.len() as f64) * cell_size < config.min_section_length {
                        orphaned.extend(remainder.cells.iter().copied());
                        continue;
                    }
                    let rem_portions =
                        portions_for(&remainder, coverage, sport_tracks, config, cell_size, tun);
                    let rem_cells = remainder.cells.clone();
                    let Some((rem_node, rem_portions, rem_score)) = qualify_candidate(
                        remainder,
                        rem_portions,
                        coverage,
                        sport_tracks,
                        config,
                        cell_size,
                        tun,
                        starts,
                        span_s,
                        records,
                    ) else {
                        orphaned.extend(rem_cells);
                        continue;
                    };
                    let (seam_lat, seam_lng) = rem_node
                        .cells
                        .iter()
                        .map(|&c| coverage.grid.centre_of(c))
                        .min_by(|a, b| {
                            let d = |&(la, ln): &(f64, f64)| {
                                let p = GpsPoint {
                                    latitude: la,
                                    longitude: ln,
                                    elevation: None,
                                };
                                section
                                    .polyline
                                    .iter()
                                    .step_by(5)
                                    .map(|q| crate::geo_utils::haversine_distance(&p, q))
                                    .fold(f64::INFINITY, f64::min)
                            };
                            d(a).partial_cmp(&d(b)).unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .unwrap_or((section.polyline[0].latitude, section.polyline[0].longitude));
                    records.push(BoundaryRecord {
                        latitude: seam_lat,
                        longitude: seam_lng,
                        reason: BoundaryReason::PassEnd {
                            requeued_cells: rem_node.cells.len() as u32,
                        },
                    });
                    queue.push_back((usize::MAX, rem_node, rem_portions, rem_score));
                }
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
        tun,
    );
    reconcile_seam_overruns(&mut sections, coverage.ref_lat, config.min_section_length);

    sections
}

/// Chain neighbours must MEET, not double-draw: when one line's end
/// runs sustained alongside another accepted line — two renderings of
/// the same trail from different real activities, a GPS-drift lane
/// apart — the quieter line's overrunning end clips back to the meet
/// point and the busier line keeps the shared stretch. Ends only,
/// display only, and only within the drift scale: genuinely parallel
/// streets sit wider than [`SEAM_TOL_M`] and are never touched.
const SEAM_TOL_M: f64 = 25.0;
const SEAM_RUN_M: f64 = 60.0;

fn reconcile_seam_overruns(sections: &mut [FrequentSection], ref_lat: f64, min_len: f64) {
    if sections.len() < 2 {
        return;
    }
    let m_lat = 111_132.0;
    let m_lng = 111_320.0 * ref_lat.to_radians().cos();
    let key = |p: &GpsPoint| {
        (
            (p.latitude * m_lat / SEAM_TOL_M).floor() as i32,
            (p.longitude * m_lng / SEAM_TOL_M).floor() as i32,
        )
    };
    // Rank: the busier line owns shared ground; ties to the longer,
    // then the earlier id, so the outcome is deterministic.
    let rank = |s: &FrequentSection| (s.visit_count, s.distance_meters.round() as i64);
    for i in 0..sections.len() {
        // Foreign geometry that outranks section i, hashed at seam scale.
        let mut grid: HashMap<(i32, i32), Vec<(f64, f64)>> = HashMap::new();
        let my_rank = rank(&sections[i]);
        let my_id = sections[i].id.clone();
        for (j, other) in sections.iter().enumerate() {
            if j == i {
                continue;
            }
            let beats = match rank(other).cmp(&my_rank) {
                std::cmp::Ordering::Greater => true,
                std::cmp::Ordering::Less => false,
                std::cmp::Ordering::Equal => other.id < my_id,
            };
            if !beats {
                continue;
            }
            for p in &other.polyline {
                grid.entry(key(p))
                    .or_default()
                    .push((p.latitude * m_lat, p.longitude * m_lng));
            }
        }
        if grid.is_empty() {
            continue;
        }
        let tol2 = SEAM_TOL_M * SEAM_TOL_M;
        let near = |p: &GpsPoint| {
            let (x, y) = (p.latitude * m_lat, p.longitude * m_lng);
            let c = key(p);
            (-1..=1i32).any(|dy| {
                (-1..=1i32).any(|dx| {
                    grid.get(&(c.0 + dy, c.1 + dx)).is_some_and(|v| {
                        v.iter()
                            .any(|&(ex, ey)| (x - ex).powi(2) + (y - ey).powi(2) < tol2)
                    })
                })
            })
        };
        let line = &sections[i].polyline;
        let n = line.len();
        let mut cum = vec![0.0f64; n];
        for k in 1..n {
            cum[k] = cum[k - 1] + crate::geo_utils::haversine_distance(&line[k - 1], &line[k]);
        }
        let total = cum[n - 1];
        // A closed line has no overrunning end to reconcile: a lapped
        // ring hugging the busier path around its own ground is a
        // revolution, not a double-draw.
        let endgap = crate::geo_utils::haversine_distance(&line[0], &line[n - 1]);
        if endgap <= (0.2 * total).max(2.0 * SEAM_TOL_M) {
            continue;
        }
        // Leading overrun: the prefix run of near points, sustained and
        // confined to the first half of the line.
        let mut lead = 0usize;
        while lead < n && near(&line[lead]) {
            lead += 1;
        }
        let lead_ok =
            lead > 0 && lead < n && cum[lead - 1] >= SEAM_RUN_M && cum[lead] < 0.5 * total;
        let mut trail = 0usize;
        while trail < n && near(&line[n - 1 - trail]) {
            trail += 1;
        }
        let trail_ok = trail > 0
            && trail < n
            && total - cum[n - trail] >= SEAM_RUN_M
            && total - cum[n - 1 - trail] < 0.5 * total;
        let s = if lead_ok { lead } else { 0 };
        let e = if trail_ok { n - trail } else { n };
        if s == 0 && e == n {
            continue;
        }
        if e <= s + 1 {
            continue;
        }
        let kept = &sections[i].polyline[s..e];
        let kept_m = crate::matching::calculate_route_distance(kept);
        if kept_m < min_len {
            continue;
        }
        sections[i].polyline = kept.to_vec();
        sections[i].distance_meters = kept_m;
    }
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
    detect_sections_unified_dated(tracks, seconds, sport_types, &HashMap::new(), config, tun)
}

/// [`detect_sections_unified_explained`] with per-activity start times.
/// Starts chaining within [`Tunables::occasion_gap_h`] form one
/// OCCASION, and every support floor counts occasions instead of
/// activities: a multi-day trip's files are one visit to their ground.
/// Ids absent from `start_epochs` each count as their own occasion, so
/// a dateless call is exactly the classic entry.
pub fn detect_sections_unified_dated(
    tracks: &[(String, Vec<GpsPoint>)],
    seconds: &[&[f64]],
    sport_types: &HashMap<String, String>,
    start_epochs: &HashMap<String, i64>,
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
            start_epochs,
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

/// The outcome of folding one activity into an existing catalogue: the
/// full converged catalogue plus the delta a persistence layer applies.
///
/// The batch catalogue is legitimately NON-MONOTONE — sections dissolve
/// and reform as evidence accumulates (measured: section count walks a
/// path like `.. 3, 3, 2, 2, 3 ..` over a growing pool, never a
/// superset of an earlier step). So an incremental that only ever ADDED
/// would drift away from the batch. `dissolved` is how a fold retires
/// ground that no longer qualifies; `added` is genuinely new ground.
/// Damping this churn into a stable, believable view (assign-once
/// identity, hysteresis) is the engine layer's job, not the detector's:
/// this layer converges to the churny truth, the layer above presents it
/// calmly.
pub struct UnifiedIncrementalResult {
    /// The catalogue after the fold. Under the default policy this is the
    /// fresh detection verbatim (order and ids included), so it converges
    /// to [`detect_sections_unified`] over `pool` exactly, order-free by
    /// construction. Under a pinning policy, frozen sections are emitted
    /// after the fresh ones; nothing downstream may read meaning into
    /// catalogue position.
    pub catalogue: Vec<FrequentSection>,
    /// Catalogue sections whose ground no prior section carried: the
    /// sections this fold surfaced. A split loser (new cut on ground a
    /// prior partly covered, where a sibling inherited the prior) counts
    /// as added — from the caller's side it is a new list entry.
    pub added: Vec<FrequentSection>,
    /// Parallel to [`added`](Self::added): the prior id each added section was
    /// carved from when it is a split loser (it shares a prior's corridor but a
    /// sibling inherited that prior), else `None`. The caller records lineage
    /// ("split into X and Y") without re-deriving the graph. On this path the
    /// prior ids are the caller's own section ids, so no id translation is
    /// needed; it mirrors [`CandidateResolution::split_from`] on the visible
    /// path so the lab replay and the engine see the same lineage.
    pub added_split_from: Vec<Option<String>>,
    /// Prior sections whose ground decisively left the catalogue (the
    /// non-monotone case). Ground that survived under another id is in
    /// `merged`, not here.
    pub dissolved: Vec<FrequentSection>,
    /// Prior sections whose ground survives inside another surviving
    /// section: the id left the list, the corridor did not.
    pub merged: Vec<SectionMergedAway>,
    /// Prior sections that survived on the same ground but are now drawn
    /// with a materially different extent. Neither an add nor a dissolve,
    /// and invisible to a caller that only diffs those two. A fold that
    /// re-cuts a trunk because a junction appeared reports the trunk here
    /// and the new branches in `added`.
    pub changed: Vec<SectionGeometryChange>,
    /// Re-cuts the policy held back: the evidence preferred `current`,
    /// but `previous` is pinned or frozen so the catalogue keeps it. The
    /// delta stays observable while the geometry stays still — a frozen
    /// caller can see drift accumulating instead of discovering it as one
    /// burst at unfreeze.
    pub held: Vec<SectionGeometryChange>,
    /// Every surviving prior id and the catalogue id now covering its
    /// ground, `(prior_id, catalogue_id)`. Batch ids are positional and
    /// renumber freely between folds, so a caller holding the previous
    /// result re-keys through this map. Id-stable presentation with
    /// debounce is the identity layer's job ([`HysteresisState`]); this
    /// map is the raw, undamped fact of one fold. Id-stable presentation
    /// with debounce is the identity layer's job
    /// ([`super::identity::HysteresisState`]).
    pub carried: Vec<(String, String)>,
}

/// A prior section whose ground merged into another surviving section.
#[derive(Clone, Debug)]
pub struct SectionMergedAway {
    /// The section as the caller held it.
    pub previous: FrequentSection,
    /// The PRIOR id whose successor absorbed this ground; chase it
    /// through `carried` for the catalogue id.
    pub into_id: String,
}

/// A section that survived a fold on the same ground with different
/// geometry. Carries both extents so a caller can decide whether the
/// move is worth showing, and the magnitudes so it does not have to
/// recompute them.
///
/// "Materially different" is an absolute one-evidence-cell test
/// ([`cluster_cell_size`]) on endpoint shift or length delta: did the
/// drawn line move perceptibly. The identity layer's
/// [`RECUT_AGREEMENT`](super::identity::RECUT_AGREEMENT) asks a
/// different, proportional question (is this re-cut material relative to
/// the extent, worth debouncing); the two thresholds are deliberately
/// not unified.
#[derive(Clone, Debug)]
pub struct SectionGeometryChange {
    /// The section as the caller held it before this fold.
    pub previous: FrequentSection,
    /// The same ground as the fold now cuts it.
    pub current: FrequentSection,
    /// The larger of the two endpoint shifts, in metres, taking the
    /// better of the two orientations.
    pub endpoint_shift_m: f64,
    /// Signed length change in metres: positive means the section grew.
    pub length_delta_m: f64,
}

/// What a fold is allowed to do to geometry the caller already holds.
///
/// Pure input. The DECISION to pin a section is durable user intent and
/// belongs to the caller; the CONSEQUENCE of a pin — which ground a
/// fresh cut may still claim once a pinned corridor is spoken for — is
/// detection maths and belongs here, beside the selection backoff it
/// mirrors. Policy never alters discovery or the evidence cache, only
/// emission: the raw evidence stays a pure function of the activity set.
#[derive(Clone, Debug, Default)]
pub struct SectionUpdatePolicy {
    /// Ids of prior sections whose geometry must survive the fold. A
    /// pinned section keeps its polyline, id, and name, is never
    /// dissolved even if its ground lost support, and claims its
    /// corridor: a fresh cut sharing ground with it is withheld (reported
    /// in `held`) rather than emitted beside it, per the
    /// never-merge-near-duplicates rule. Its traversal evidence keeps
    /// growing — new members are grafted append-only with portions
    /// recomputed against the FROZEN polyline — while the consensus
    /// fields stay frozen with the geometry they describe. Note the
    /// corridor consequence: a pinned trunk owns its full corridor, so
    /// junction re-cuts along it stay withheld until unpinned.
    pub pinned_ids: Vec<String>,
    /// Freeze every surviving section's geometry, not just the pinned
    /// ones: the catalogue's drawn lines hold still. Unlike a pin this
    /// does NOT freeze existence — ground that decisively lost support
    /// still dissolves (a dissolve removes, it does not redraw) — and
    /// genuinely new ground still lands in `added`. Withheld re-cuts are
    /// reported in `held`, so drift stays observable. Default false:
    /// geometry follows the evidence.
    pub freeze_all_geometry: bool,
}

/// Fold one activity into an existing Unified catalogue, order-free and
/// converging to the [`detect_sections_unified`] batch over the same
/// pool.
///
/// `pool` is the FULL accumulated pool INCLUDING the just-added activity;
/// `existing` is the catalogue before it. The result carries the new
/// catalogue plus the add/dissolve delta a persistence layer applies.
///
/// # Convergence, not accumulation
///
/// Detection is already per-(sport, geo-cluster): the batch is a union
/// over geographically disjoint clusters of [`detect_for_cluster`], and
/// each cluster's catalogue is a pure function of that cluster's activity
/// SET (proven order-free — canonical portion order, sorted grid
/// accumulation). So the correct incremental only needs to re-run
/// discovery for the cluster(s) the new activity touches, reuse untouched
/// clusters verbatim, and it converges to the batch by construction. The
/// dissolve set falls out naturally: a touched cluster's fresh discovery
/// replaces its prior sections wholesale.
///
/// # This is the NAIVE-CORRECT baseline (B1 anchor)
///
/// The body re-batches the WHOLE pool on every call: correct by
/// construction (the batch IS the convergence target) but O(N) per add,
/// O(N^2) over a drip. It exists to green the convergence contract
/// (`gate_unified_incremental_converges_to_batch`) and to hand the engine
/// layer a passing baseline to optimise UNDER. The optimisation — a
/// persisted per-cluster evidence grid folded in O(cluster) per add —
/// keeps this delta contract ([`UnifiedIncrementalResult`]) unchanged;
/// only the body and an added evidence-cache handle change. Design:
/// `~/.claude/plans/b1-incremental-design.md`.
pub fn detect_sections_unified_incremental(
    existing: &[FrequentSection],
    pool: &[(String, Vec<GpsPoint>)],
    seconds: &[&[f64]],
    sport_types: &HashMap<String, String>,
    config: &SectionConfig,
) -> UnifiedIncrementalResult {
    let fresh = detect_sections_unified(pool, seconds, sport_types, config);
    let lookup: HashMap<&str, (&[GpsPoint], &[f64])> = pool
        .iter()
        .enumerate()
        .map(|(i, (id, pts))| {
            let secs: &[f64] = seconds.get(i).copied().unwrap_or(&[]);
            (id.as_str(), (pts.as_slice(), secs))
        })
        .collect();
    resolve_fold(
        fresh,
        existing,
        &SectionUpdatePolicy::default(),
        config,
        &lookup,
    )
}

/// Total length of a polyline in metres.
fn polyline_length_m(pts: &[GpsPoint]) -> f64 {
    pts.windows(2)
        .map(|w| crate::geo_utils::haversine_distance(&w[0], &w[1]))
        .sum()
}

/// The larger of two sections' endpoint shifts, taking the better of the
/// two orientations: a section carries a direction, the ground does not.
fn endpoint_shift_m(a: &FrequentSection, b: &FrequentSection) -> f64 {
    let (Some(a0), Some(a1), Some(b0), Some(b1)) = (
        a.polyline.first(),
        a.polyline.last(),
        b.polyline.first(),
        b.polyline.last(),
    ) else {
        return f64::INFINITY;
    };
    let dist = crate::geo_utils::haversine_distance;
    let forward = dist(a0, b0).max(dist(a1, b1));
    let reversed = dist(a0, b1).max(dist(a1, b0));
    forward.min(reversed)
}

/// Turn a freshly detected catalogue plus the caller's prior one into the
/// fold outcome, honouring `policy`.
///
/// Pairing is delegated to the identity layer's [`plan_identity`]: one
/// notion of "same section" across the crate, permutation-stable, split
/// and merge aware. From the plan: a carried candidate is the prior's
/// successor (a `changed` entry when the geometry moved by more than one
/// evidence cell), a minted candidate is `added`, a retired prior is
/// `dissolved` or `merged` by its retirement reason.
///
/// Policy applies AFTER pairing, on emission only. A frozen prior
/// (pinned, or everything under `freeze_all_geometry`) keeps its
/// geometry: its fresh partner is withheld into `held` and its evidence
/// grafted append-only. A withheld corridor is spoken for — any other
/// candidate sharing ground with a frozen emission backs off: a minted
/// one is dropped into `held`; a carried one hands back its prior
/// verbatim (frozen by adjacency, one pass, no cascade). Only an
/// explicit pin also freezes existence; under `freeze_all_geometry` an
/// unpaired prior still dissolves.
///
/// With the default policy nothing is frozen, so the emitted catalogue
/// is exactly the fresh one in its original order and ids. The
/// batch-parity gates depend on that: the policy is an addition to this
/// layer, never a detour around it.
fn resolve_fold(
    fresh: Vec<FrequentSection>,
    existing: &[FrequentSection],
    policy: &SectionUpdatePolicy,
    config: &SectionConfig,
    tracks: &HashMap<&str, (&[GpsPoint], &[f64])>,
) -> UnifiedIncrementalResult {
    use super::identity::{CandidateSection, PriorSection, RetireReason, plan_identity};

    // The identity plan pairs priors to candidates. Seniority (merge
    // inheritance) is the caller's list order: earlier is more senior.
    let priors: Vec<PriorSection> = existing
        .iter()
        .enumerate()
        .map(|(i, s)| PriorSection {
            id: s.id.clone(),
            polyline: s.polyline.clone(),
            first_seen: i as u64,
            visit_count: s.visit_count,
        })
        .collect();
    let candidates: Vec<CandidateSection> =
        fresh.iter().map(CandidateSection::from_section).collect();
    let plan = plan_identity(&priors, &candidates);

    let index_of: HashMap<&str, usize> = existing
        .iter()
        .enumerate()
        .map(|(i, s)| (s.id.as_str(), i))
        .collect();
    let explicit_pin: HashSet<&str> = policy.pinned_ids.iter().map(|s| s.as_str()).collect();
    let frozen_prior = |id: &str| policy.freeze_all_geometry || explicit_pin.contains(id);

    // A visible re-cut has to clear one evidence cell, so sub-cell jitter in
    // the reference trace never reports as a geometry change.
    let move_threshold = cluster_cell_size(config);
    let change = |prev: &FrequentSection, cur: &FrequentSection| SectionGeometryChange {
        previous: prev.clone(),
        current: cur.clone(),
        endpoint_shift_m: endpoint_shift_m(prev, cur),
        length_delta_m: polyline_length_m(&cur.polyline) - polyline_length_m(&prev.polyline),
    };
    let moved = |prev: &FrequentSection, cur: &FrequentSection| {
        endpoint_shift_m(prev, cur) > move_threshold
            || (polyline_length_m(&cur.polyline) - polyline_length_m(&prev.polyline)).abs()
                > move_threshold
    };

    // Per-candidate carrier from the plan, then each prior's fate.
    let carrier: Vec<Option<usize>> = plan
        .decisions
        .iter()
        .map(|d| d.carried_id().and_then(|id| index_of.get(id).copied()))
        .collect();

    let mut changed = Vec::new();
    let mut held = Vec::new();
    let mut dissolved = Vec::new();
    let mut merged = Vec::new();
    let mut carried = Vec::new();
    // Frozen emissions, in existing order; reserved before candidates land.
    let mut frozen_out: Vec<FrequentSection> = Vec::new();
    // Candidate fate: emit as-is, replaced by its prior, or withheld.
    let mut suppressed = vec![false; fresh.len()];

    for (j, cand) in fresh.iter().enumerate() {
        let Some(pi) = carrier[j] else { continue };
        let prior = &existing[pi];
        if frozen_prior(&prior.id) {
            suppressed[j] = true;
            if moved(prior, cand) {
                held.push(change(prior, cand));
            }
            frozen_out.push(graft_frozen(
                prior,
                cand,
                tracks,
                config.proximity_threshold,
            ));
            carried.push((prior.id.clone(), prior.id.clone()));
        }
    }
    for r in &plan.retired {
        let Some(&pi) = index_of.get(r.id.as_str()) else {
            continue;
        };
        let prior = &existing[pi];
        match &r.reason {
            _ if explicit_pin.contains(prior.id.as_str()) => {
                // A pin freezes existence: no partner to graft from, the
                // section simply survives untouched.
                frozen_out.push(prior.clone());
                carried.push((prior.id.clone(), prior.id.clone()));
            }
            RetireReason::Dissolved => dissolved.push(prior.clone()),
            RetireReason::MergedInto { id } => merged.push(SectionMergedAway {
                previous: prior.clone(),
                into_id: id.clone(),
            }),
        }
    }

    // Frozen corridors are spoken for: any candidate sharing ground with a
    // frozen emission backs off. A carried candidate hands back its prior
    // verbatim (frozen by adjacency); a minted one is dropped. One pass —
    // adjacency freezes do not themselves claim further ground.
    let claims: Vec<&FrequentSection> = frozen_out.iter().collect();
    let mut adjacency_out: Vec<FrequentSection> = Vec::new();
    for (j, cand) in fresh.iter().enumerate() {
        if suppressed[j] {
            continue;
        }
        if claims
            .iter()
            .any(|fz| shares_ground(&fz.polyline, &cand.polyline))
        {
            suppressed[j] = true;
            match carrier[j] {
                Some(pi) => {
                    let prior = &existing[pi];
                    held.push(change(prior, cand));
                    adjacency_out.push(prior.clone());
                    carried.push((prior.id.clone(), prior.id.clone()));
                }
                None => {
                    // Report the withheld cut against the claiming section.
                    let fz = claims
                        .iter()
                        .find(|fz| shares_ground(&fz.polyline, &cand.polyline))
                        .expect("a claim matched above");
                    held.push(change(fz, cand));
                }
            }
        }
    }
    frozen_out.extend(adjacency_out);

    // Emit the surviving candidates in fresh order, renumbering any id a
    // frozen emission already holds (deterministic given the inputs).
    let reserved: HashSet<String> = frozen_out.iter().map(|s| s.id.clone()).collect();
    let mut catalogue: Vec<FrequentSection> = Vec::with_capacity(fresh.len());
    let mut added = Vec::new();
    let mut added_split_from = Vec::new();
    for (j, mut cand) in fresh.into_iter().enumerate() {
        if suppressed[j] {
            continue;
        }
        if reserved.contains(&cand.id) {
            cand.id = disambiguate_id(&cand.id, &reserved, &catalogue);
        }
        match carrier[j] {
            Some(pi) => {
                let prior = &existing[pi];
                if moved(prior, &cand) {
                    changed.push(change(prior, &cand));
                }
                carried.push((prior.id.clone(), cand.id.clone()));
            }
            None => {
                added.push(cand.clone());
                added_split_from.push(plan.decisions[j].split_from().map(str::to_string));
            }
        }
        catalogue.push(cand);
    }
    catalogue.extend(frozen_out);

    UnifiedIncrementalResult {
        catalogue,
        added,
        added_split_from,
        dissolved,
        merged,
        changed,
        held,
        carried,
    }
}

/// A frozen section with its fresh partner's evidence grafted on,
/// append-only: members the frozen section has not seen are added with
/// portions recomputed against the FROZEN polyline (the partner's
/// portions index a different extent), `visit_count` rises with each
/// qualifying pass, and nothing is ever removed. Geometry, id, name, and
/// the consensus fields stay exactly as held — they describe the frozen
/// polyline. A partner member whose track shows no qualifying portion
/// against the frozen line is skipped: evidence must be attributable to
/// the extent it is counted against.
fn graft_frozen(
    prior: &FrequentSection,
    partner: &FrequentSection,
    tracks: &HashMap<&str, (&[GpsPoint], &[f64])>,
    proximity_m: f64,
) -> FrequentSection {
    let mut out = prior.clone();
    for aid in &partner.activity_ids {
        if out.activity_ids.iter().any(|x| x == aid) {
            continue;
        }
        let Some(&(pts, _)) = tracks.get(aid.as_str()) else {
            continue;
        };
        let portions = crate::find_all_track_portions(pts, &out.polyline, proximity_m);
        if portions.is_empty() {
            continue;
        }
        out.activity_ids.push(aid.clone());
        for (s, e, direction) in portions {
            // Exclusive end, exactly as compute_activity_portions stores it.
            let e = e.min(pts.len());
            if s >= e {
                continue;
            }
            out.visit_count += 1;
            out.activity_portions.push(SectionPortion {
                activity_id: aid.clone(),
                start_index: s as u32,
                end_index: e as u32,
                distance_meters: crate::matching::calculate_route_distance(&pts[s..e]),
                direction,
            });
        }
    }
    out
}

/// A deterministic replacement id for a candidate whose fresh positional
/// id collides with a frozen emission: the original id with the smallest
/// `_f<n>` suffix not already in use.
fn disambiguate_id(id: &str, reserved: &HashSet<String>, emitted: &[FrequentSection]) -> String {
    let mut n = 1usize;
    loop {
        let candidate = format!("{id}_f{n}");
        if !reserved.contains(&candidate) && !emitted.iter().any(|s| s.id == candidate) {
            return candidate;
        }
        n += 1;
    }
}

// ============================================================================
// Cached cluster-recompute incremental (the O(touched-cluster) fast path)
// ============================================================================
//
// The naive [`detect_sections_unified_incremental`] above re-batches the WHOLE
// pool on every add. This path holds the per-(sport, cluster) catalogue across
// calls and re-runs detection ONLY for the cluster(s) a new activity touches,
// reusing every untouched cluster verbatim. Detection is already partitioned
// per geo-cluster and each cluster is an order-free pure function of its set
// ([`detect_for_cluster`]), so the touched cluster is recomputed from its
// current tracks and the catalogue equals the batch by construction — the same
// per-cluster decomposition, minus the clusters the add did not reach.
//
// The touched cluster is recomputed with a FRESH reference latitude (a plain
// [`build_coverage_grid`] over its members), so it is byte-identical to what the
// batch computes for that cluster. An earlier design froze the reference
// latitude and folded the grid in place; measurement retired it — the grid build
// is ~3% of a cluster's detect cost (discovery dominates), so the fold saved
// almost nothing, while a frozen projection drifts against the batch's and, far
// from the meridian, the longitude scale amplifies that drift enough to flip
// marginal sections. Always recomputing the touched cluster is exact and only
// marginally dearer. The win is not re-touching the OTHER clusters.

/// On-disk layout version of [`SectionEvidenceCache`]. Bump when the stored
/// per-cluster shape changes so a persisted blob from an older build is
/// recognised as stale and the engine cold-rebatches instead of trusting it.
const EVIDENCE_CACHE_VERSION: u32 = 1;

/// Persisted per-(sport, geo-cluster) evidence backing
/// [`detect_sections_unified_incremental_cached`]. The engine holds one across
/// folds and (in a later phase) persists it as a blob; this layer only defines
/// the type and keeps it warm in memory.
///
/// Per cluster it stores the member set with per-member bounding boxes (for
/// routing a new activity and detecting a bridge) and the cluster's last-emitted
/// catalogue, reused verbatim whenever an add does not touch that cluster. No
/// grid is persisted: a touched cluster is rebuilt from the pool on demand.
/// `version` guards the blob: a decode that finds a different version must
/// discard and cold-rebatch.
#[derive(Clone, Serialize, Deserialize)]
pub struct SectionEvidenceCache {
    version: u32,
    /// One bucket per sport; each holds that sport's geographically disjoint
    /// clusters. Sport is a bucketing key here only, never part of a section's
    /// identity (that is the engine's concern).
    sports: HashMap<String, Vec<ClusterEvidence>>,
    /// Memoised pure leaves of the per-cluster pipeline (see [`LeafMemos`]).
    #[serde(default)]
    leaves: LeafMemos,
}

/// Memoised pure leaves of the per-cluster pipeline, carried in the evidence
/// cache so a routine add pays only for the ground it touched. Every entry is
/// a pure function of its key; track data is immutable per id (the engine
/// drops the whole cache when a track's GPS changes) and the tunables are
/// fixed for a cache's lifetime. Entries for supernode shapes that stop
/// occurring linger until the cache is dropped: the population is bounded by
/// the distinct configurations the evidence has actually taken, and the
/// engine-level cache invalidation is the reset.
#[derive(Clone, Default, Serialize, Deserialize)]
struct LeafMemos {
    /// Per-track lift candidate spans ([`lift_spans_tuned`]), keyed by
    /// activity id. The cross-track descent rescue is recomputed per rebuild
    /// (bbox-gated, and free while no member carries a candidate).
    lift_candidates: HashMap<String, Vec<(usize, usize)>>,
    /// Interned supernode cell sets: full-equality mapping from the sorted
    /// cell list to a stable small id, so per-track keys stay exact without
    /// hashing the whole cell list per track. Append-only between sweeps;
    /// cleared ONLY together with `track_portions` (a reused intern id under
    /// a live key would alias two different cell sets).
    cell_sets: HashMap<Vec<Cell>, u32>,
    /// Per-track [`track_portion`] results keyed by (activity id, interned
    /// cell set, lift-free keep ranges, projection, length bounds). Portions
    /// are computed per track independently, so on saturated ground a new
    /// activity leaves every existing track's entry valid and pays only for
    /// its own cut.
    track_portions: HashMap<TrackPortionKey, Option<(usize, usize, f64)>>,
    /// [`process_cluster`] results keyed by the candidate's portion set,
    /// including refusals. The id is rewritten on every hit exactly as a
    /// miss would mint it, so numbering never depends on the cache.
    consensus: HashMap<ConsensusKey, Option<FrequentSection>>,
}

/// Complete input fingerprint of one [`track_portion`] call: the activity,
/// the supernode's interned cell set, the track's lift-free keep ranges,
/// the projection, and the config fields the cut reads.
#[derive(Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
struct TrackPortionKey {
    track: String,
    cells: u32,
    keep: Vec<(usize, usize)>,
    ref_lat_bits: u64,
    cell_size_bits: u64,
    min_len_bits: u64,
    max_len_bits: u64,
}

/// Memory bounds for the leaf memos, in entries. Blunt and exact: tripping
/// a cap clears the affected maps whole (one cold rebuild re-warms them),
/// never evicts selectively. `cell_sets` and `track_portions` clear
/// together — intern ids must not outlive their table.
const MEMO_CELL_SETS_CAP: usize = 20_000;
const MEMO_TRACK_PORTIONS_CAP: usize = 200_000;
const MEMO_CONSENSUS_CAP: usize = 10_000;

/// Complete input fingerprint of one [`process_cluster`] call: the portion
/// set (activity id + range), the sport label baked into the id and the
/// section, and the config fields the consensus path reads.
#[derive(Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
struct ConsensusKey {
    portions: Vec<(String, usize, usize)>,
    sport: String,
    proximity_bits: u64,
    min_activities: u32,
    max_len_bits: u64,
}

impl Default for SectionEvidenceCache {
    fn default() -> Self {
        Self {
            version: EVIDENCE_CACHE_VERSION,
            sports: HashMap::new(),
            leaves: LeafMemos::default(),
        }
    }
}

impl SectionEvidenceCache {
    /// A fresh, empty cache at the current layout version.
    pub fn new() -> Self {
        Self::default()
    }

    /// True when the blob's layout matches this build. A mismatch means the
    /// persisted grid cannot be trusted and the caller should cold-rebatch.
    pub fn is_current(&self) -> bool {
        self.version == EVIDENCE_CACHE_VERSION
    }

    /// Per-cluster `(sport, members, ref_lat, sections)` snapshot, for tests
    /// and diagnostics.
    #[doc(hidden)]
    pub fn debug_summary(&self) -> Vec<(String, usize, f64, usize)> {
        let mut sports: Vec<&String> = self.sports.keys().collect();
        sports.sort_unstable();
        let mut out = Vec::new();
        for s in sports {
            for c in &self.sports[s] {
                out.push((s.clone(), c.member_ids.len(), c.ref_lat, c.sections.len()));
            }
        }
        out
    }
}

/// One geographic cluster's evidence within a sport.
#[derive(Clone, Serialize, Deserialize)]
struct ClusterEvidence {
    /// Member activity ids in arrival order (also the track ordering a rebuild
    /// keys the grid by). Grows by append; a bridge concatenates the merged
    /// clusters' members.
    member_ids: Vec<String>,
    /// Per-member raw bbox `(lat0, lat1, lng0, lng1)`, parallel to `member_ids`.
    /// The routing/bridge relation pads these exactly as [`geo_clusters`] does.
    member_bboxes: Vec<(f64, f64, f64, f64)>,
    /// Union of the member bboxes: a routing pre-filter so a new activity far
    /// from this cluster is rejected in O(1) instead of scanning every member.
    union_bbox: (f64, f64, f64, f64),
    /// The reference latitude of the last rebuild (the batch's set-mean over the
    /// members). Informational; a rebuild recomputes it.
    ref_lat: f64,
    /// The cluster's last-emitted sections, reused verbatim whenever an add does
    /// not touch this cluster.
    sections: Vec<FrequentSection>,
    /// Set whenever the membership changed this call (an add or a bridge merge)
    /// so the recompute pass rebuilds this cluster exactly ONCE over its FINAL
    /// membership, not once per new activity. Transient (never persisted): the
    /// recompute pass clears it before the call returns.
    #[serde(skip)]
    dirty: bool,
}

impl ClusterEvidence {
    fn empty() -> Self {
        Self {
            member_ids: Vec::new(),
            member_bboxes: Vec::new(),
            union_bbox: (f64::MAX, f64::MIN, f64::MAX, f64::MIN),
            ref_lat: 0.0,
            sections: Vec::new(),
            dirty: false,
        }
    }

    /// Add a member, keeping the union bbox current and marking the cluster for
    /// recompute.
    fn push_member(&mut self, id: &str, bbox: (f64, f64, f64, f64)) {
        self.member_ids.push(id.to_string());
        self.member_bboxes.push(bbox);
        let u = &mut self.union_bbox;
        u.0 = u.0.min(bbox.0);
        u.1 = u.1.max(bbox.1);
        u.2 = u.2.min(bbox.2);
        u.3 = u.3.max(bbox.3);
        self.dirty = true;
    }

    /// South-west corner `(min lat, min lng)` over member boxes, the key
    /// [`geo_clusters`] sorts clusters by. Used to order clusters (and thus
    /// section ids) exactly as the batch does.
    fn sw_corner(&self) -> (f64, f64) {
        let mut sw = (f64::MAX, f64::MAX);
        for b in &self.member_bboxes {
            sw.0 = sw.0.min(b.0);
            sw.1 = sw.1.min(b.2);
        }
        sw
    }
}

/// Fold new activities into an existing Unified catalogue using the persisted
/// per-cluster evidence in `cache`. Order-free and equal to the
/// [`detect_sections_unified`] batch over `pool` (same per-cluster
/// decomposition), so it converges at >= 0.95 ground overlap with identical
/// section count.
///
/// `pool` is the full accumulated pool (the new activities included) and is the
/// source of truth for every touched cluster's tracks; `new_activity_ids` names
/// the just-arrived entries, so only the clusters they touch are recomputed.
/// The delta ([`UnifiedIncrementalResult`]) is computed against `existing`
/// exactly as the naive baseline does (ground overlap), so the two agree.
///
/// The cache is mutated in place: a touched cluster is rebuilt fresh from its
/// members and re-detected; untouched clusters keep their stored sections. A new
/// activity that bridges two clusters merges them and rebuilds the union (the
/// one genuinely global event, bounded by the merged size). The add cost is
/// therefore O(touched cluster), not O(whole pool) as the naive re-batch is.
pub fn detect_sections_unified_incremental_cached(
    cache: &mut SectionEvidenceCache,
    existing: &[FrequentSection],
    pool: &[(String, Vec<GpsPoint>)],
    new_activity_ids: &[&str],
    seconds: &[&[f64]],
    sport_types: &HashMap<String, String>,
    config: &SectionConfig,
) -> UnifiedIncrementalResult {
    detect_sections_unified_incremental_cached_with_policy(
        cache,
        existing,
        pool,
        new_activity_ids,
        seconds,
        sport_types,
        config,
        &SectionUpdatePolicy::default(),
    )
}

/// [`detect_sections_unified_incremental_cached`] with control over what
/// the fold may do to geometry the caller already holds.
///
/// Discovery is unchanged — pins never alter which ground the evidence
/// supports, only which cut is emitted for ground already spoken for — so
/// the catalogue under the default policy is identical to the plain
/// entry point, and the batch-parity gates hold for both.
#[allow(clippy::too_many_arguments)]
pub fn detect_sections_unified_incremental_cached_with_policy(
    cache: &mut SectionEvidenceCache,
    existing: &[FrequentSection],
    pool: &[(String, Vec<GpsPoint>)],
    new_activity_ids: &[&str],
    seconds: &[&[f64]],
    sport_types: &HashMap<String, String>,
    config: &SectionConfig,
    policy: &SectionUpdatePolicy,
) -> UnifiedIncrementalResult {
    detect_sections_unified_incremental_dated(
        cache,
        existing,
        pool,
        new_activity_ids,
        seconds,
        sport_types,
        &HashMap::new(),
        config,
        policy,
    )
}

/// [`detect_sections_unified_incremental_cached_with_policy`] with
/// per-activity start times: the incremental twin of
/// [`detect_sections_unified_dated`], so drip and batch count occasions
/// identically and the parity gates keep holding under dated corpora.
#[allow(clippy::too_many_arguments)]
pub fn detect_sections_unified_incremental_dated(
    cache: &mut SectionEvidenceCache,
    existing: &[FrequentSection],
    pool: &[(String, Vec<GpsPoint>)],
    new_activity_ids: &[&str],
    seconds: &[&[f64]],
    sport_types: &HashMap<String, String>,
    start_epochs: &HashMap<String, i64>,
    config: &SectionConfig,
    policy: &SectionUpdatePolicy,
) -> UnifiedIncrementalResult {
    let tun = Tunables::DEFAULT;
    let cell_size = cluster_cell_size(config);

    // Pool lookup: id → (points, seconds). Empty slice when a stream is absent,
    // mirroring how the batch reads a missing `seconds` entry.
    let lookup: HashMap<&str, (&[GpsPoint], &[f64])> = pool
        .iter()
        .enumerate()
        .map(|(i, (id, pts))| {
            let secs: &[f64] = seconds.get(i).copied().unwrap_or(&[]);
            (id.as_str(), (pts.as_slice(), secs))
        })
        .collect();

    // Phase 1 — ROUTE every new activity into its sport's clusters, marking each
    // touched cluster dirty. Routing stays per-id and ordered because routing a
    // later activity can bridge clusters an earlier one just formed. NO detection
    // runs here: only membership + bridges are updated.
    for &new_id in new_activity_ids {
        let Some((new_pts, _)) = lookup.get(new_id).copied() else {
            continue; // a named new id not present in the pool: nothing to route
        };
        let sport = sport_types
            .get(new_id)
            .map(|s| s.as_str())
            .unwrap_or("Unknown");
        route_only(cache, sport, new_id, new_pts, &tun);
    }

    // Phase 2 — RECOMPUTE each touched cluster exactly ONCE, over its FINAL
    // membership. This is what makes a cold cache or a bulk window-expand O(sum
    // of touched clusters) = O(N), not the O(N²) of recomputing per new activity.
    // Recompute-once over the final membership equals the batch's per-cluster
    // detect, so `cached == batch` is preserved (the oracle guards it).
    let leaves = &mut cache.leaves;
    // Blunt cap sweeps: a full clear costs one cold rebuild and keeps every
    // hit exact. The intern table and its dependent keys clear together.
    if leaves.cell_sets.len() > MEMO_CELL_SETS_CAP
        || leaves.track_portions.len() > MEMO_TRACK_PORTIONS_CAP
    {
        leaves.cell_sets.clear();
        leaves.track_portions.clear();
    }
    if leaves.consensus.len() > MEMO_CONSENSUS_CAP {
        leaves.consensus.clear();
    }
    for (sport, clusters) in cache.sports.iter_mut() {
        for c in clusters.iter_mut() {
            if c.dirty {
                recompute_cluster(
                    c,
                    sport,
                    &lookup,
                    config,
                    cell_size,
                    &tun,
                    leaves,
                    start_epochs,
                );
                c.dirty = false;
            }
        }
    }

    // Assemble the catalogue from every cluster, renumbering section ids
    // per-sport in SW-corner cluster order so they match the batch's scheme,
    // then resolve it against what the caller holds. The pairing inside
    // `resolve_fold` is bbox-gated, so the delta cost tracks the CHANGED
    // ground rather than the whole (growing) catalogue.
    resolve_fold(assemble_catalogue(cache), existing, policy, config, &lookup)
}

/// Raw lat/lng bounding box of a track: `(lat0, lat1, lng0, lng1)`.
fn track_bbox(pts: &[GpsPoint]) -> (f64, f64, f64, f64) {
    let mut bb = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for p in pts {
        bb.0 = bb.0.min(p.latitude);
        bb.1 = bb.1.max(p.latitude);
        bb.2 = bb.2.min(p.longitude);
        bb.3 = bb.3.max(p.longitude);
    }
    bb
}

/// Pad a raw bbox by half `gap_m`, exactly as [`geo_clusters`] does (longitude
/// scaled by the box's own mid-latitude cosine).
fn pad_bbox(bb: (f64, f64, f64, f64), gap_m: f64) -> (f64, f64, f64, f64) {
    let pad_lat = gap_m * 0.5 / 111_132.0;
    let mid = ((bb.0 + bb.1) * 0.5).to_radians();
    let pad_lng = gap_m * 0.5 / (111_320.0 * mid.cos().abs().max(0.01));
    (
        bb.0 - pad_lat,
        bb.1 + pad_lat,
        bb.2 - pad_lng,
        bb.3 + pad_lng,
    )
}

/// Two padded boxes overlap: the [`geo_clusters`] union relation.
fn boxes_overlap(a: (f64, f64, f64, f64), b: (f64, f64, f64, f64)) -> bool {
    a.0 <= b.1 && b.0 <= a.1 && a.2 <= b.3 && b.2 <= a.3
}

/// Route a new activity into the cache's clusters (Phase 1): append it to the
/// single cluster it touches, seed a new singleton if it touches none, or bridge
/// (merge) the clusters it connects. Marks every touched cluster dirty and does
/// NOT recompute — the recompute pass does that once, after all routing.
fn route_only(
    cache: &mut SectionEvidenceCache,
    sport: &str,
    new_id: &str,
    new_pts: &[GpsPoint],
    tun: &Tunables,
) {
    let gap = tun.cluster_gap_m;
    let new_bbox = track_bbox(new_pts);
    let new_padded = pad_bbox(new_bbox, gap);

    let clusters = cache.sports.entry(sport.to_string()).or_default();
    let touched: Vec<usize> = clusters
        .iter()
        .enumerate()
        .filter(|(_, c)| {
            // Union-bbox pre-filter rejects a far cluster in O(1); only a cluster
            // the activity might actually join pays the per-member scan.
            boxes_overlap(new_padded, pad_bbox(c.union_bbox, gap))
                && c.member_bboxes
                    .iter()
                    .any(|&m| boxes_overlap(new_padded, pad_bbox(m, gap)))
        })
        .map(|(i, _)| i)
        .collect();

    match touched.as_slice() {
        [] => {
            let mut c = ClusterEvidence::empty();
            c.push_member(new_id, new_bbox);
            clusters.push(c);
        }
        [i] => {
            clusters[*i].push_member(new_id, new_bbox);
        }
        _ => bridge_only(clusters, &touched, new_id, new_bbox),
    }
}

/// Merge every cluster a bridging activity touches, plus the activity itself,
/// into one cluster (marked dirty). The connected-component result matches the
/// batch's [`geo_clusters`] union-find. Bounded by the merged size, and rare (a
/// genuinely new connecting route). The recompute pass rebuilds the union once.
fn bridge_only(
    clusters: &mut Vec<ClusterEvidence>,
    touched: &[usize],
    new_id: &str,
    new_bbox: (f64, f64, f64, f64),
) {
    // Remove touched clusters high-index first so earlier indices stay valid,
    // then restore ascending order so members keep their arrival sequence.
    let mut desc = touched.to_vec();
    desc.sort_unstable_by(|a, b| b.cmp(a));
    let mut removed: Vec<ClusterEvidence> = desc.iter().map(|&i| clusters.remove(i)).collect();
    removed.reverse();

    let mut merged = ClusterEvidence::empty();
    for c in removed {
        for (id, bbox) in c.member_ids.iter().zip(&c.member_bboxes) {
            merged.push_member(id, *bbox);
        }
    }
    merged.push_member(new_id, new_bbox);
    clusters.push(merged);
}

/// Rebuild one cluster's coverage grid over its current members and re-run
/// detection, storing the fresh catalogue. This is exactly the batch's
/// per-cluster computation ([`build_coverage_grid`] +
/// [`detect_for_cluster_with_grid`]), so the cluster's sections equal what the
/// batch would emit for it. Below `min_activities` the cluster emits nothing,
/// matching the batch's per-cluster gate.
#[allow(clippy::too_many_arguments)]
fn recompute_cluster(
    cluster: &mut ClusterEvidence,
    sport: &str,
    lookup: &HashMap<&str, (&[GpsPoint], &[f64])>,
    config: &SectionConfig,
    cell_size: f64,
    tun: &Tunables,
    leaves: &mut LeafMemos,
    starts: &HashMap<String, i64>,
) {
    if cluster.member_ids.len() < config.min_activities as usize {
        cluster.sections.clear();
        return;
    }

    let members = std::mem::take(&mut cluster.member_ids);
    let sport_tracks: Vec<(&str, &[GpsPoint])> = members
        .iter()
        .map(|id| (id.as_str(), lookup[id.as_str()].0))
        .collect();
    let sport_seconds: Vec<&[f64]> = members.iter().map(|id| lookup[id.as_str()].1).collect();

    let coverage = build_coverage_grid(
        &sport_tracks,
        &sport_seconds,
        cell_size,
        tun,
        &mut leaves.lift_candidates,
    );
    let ref_lat = coverage.ref_lat;
    let mut idx = 0usize;
    let mut records = Vec::new();
    let sections = detect_for_cluster_with_grid(
        sport,
        &sport_tracks,
        &coverage,
        cell_size,
        config,
        tun,
        &mut idx,
        &mut records,
        leaves,
        starts,
    );

    cluster.member_ids = members;
    cluster.ref_lat = ref_lat;
    cluster.sections = sections;
}

/// Concatenate every cluster's sections into the full catalogue, renumbering
/// ids per sport in SW-corner cluster order so a cached catalogue carries the
/// same `sec_<sport>_<n>` ids the batch would assign for the same ground.
fn assemble_catalogue(cache: &SectionEvidenceCache) -> Vec<FrequentSection> {
    let mut out = Vec::new();
    let mut sports: Vec<&String> = cache.sports.keys().collect();
    sports.sort_unstable();
    for sport in sports {
        let clusters = &cache.sports[sport];
        let mut order: Vec<usize> = (0..clusters.len()).collect();
        order.sort_by(|&a, &b| {
            let sa = clusters[a].sw_corner();
            let sb = clusters[b].sw_corner();
            sa.0.total_cmp(&sb.0).then(sa.1.total_cmp(&sb.1))
        });
        let mut idx = 0usize;
        let lower = sport.to_lowercase();
        for &ci in &order {
            for s in &clusters[ci].sections {
                let mut s = s.clone();
                s.id = format!("sec_{lower}_{idx}");
                idx += 1;
                out.push(s);
            }
        }
    }
    out
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
    fn ref_lat_is_quantised_and_add_stable() {
        // Two track sets whose latitude means differ but sit inside one
        // quantisation band must project identically: the grid is a step
        // function of the activity set, so a routine add moves no cell
        // boundary. Scenario: home ground near 46.02; a new ride at 46.04.
        let a: Vec<GpsPoint> = (0..200)
            .map(|i| GpsPoint::new(46.02 + 1.0e-5 * i as f64, 7.0))
            .collect();
        let b: Vec<GpsPoint> = (0..200)
            .map(|i| GpsPoint::new(46.04 + 1.0e-5 * i as f64, 7.01))
            .collect();
        let one: Vec<(&str, &[GpsPoint])> = vec![("a", a.as_slice())];
        let two: Vec<(&str, &[GpsPoint])> = vec![("a", a.as_slice()), ("b", b.as_slice())];
        let g1 = build_coverage_grid(&one, &[], 100.0, &Tunables::DEFAULT, &mut HashMap::new());
        let g2 = build_coverage_grid(&two, &[], 100.0, &Tunables::DEFAULT, &mut HashMap::new());
        assert_eq!(
            g1.ref_lat, g2.ref_lat,
            "an add inside the band must not move the projection"
        );
        let bands = g1.ref_lat / Tunables::DEFAULT.ref_lat_quant_deg;
        assert!(
            (bands - bands.round()).abs() < 1e-9,
            "ref_lat {} is not on the quantisation lattice",
            g1.ref_lat
        );
    }

    #[test]
    fn ref_lat_quantisation_error_is_negligible() {
        // Snapping the projection latitude moves it at most half a band;
        // the resulting east-west scale error must stay an order under the
        // 1% budget that sized cluster_gap_m, out to polar-circle latitudes.
        let half_band = Tunables::DEFAULT.ref_lat_quant_deg / 2.0;
        let mut lat: f64 = 0.0;
        while lat <= 66.5 {
            let snapped = (lat + half_band).to_radians().cos();
            let true_scale = lat.to_radians().cos();
            let err = (snapped / true_scale - 1.0).abs();
            // tan(66.5 deg) x half a band = 0.201%, the polar-circle worst
            // case; still five times under the 1% cluster_gap_m budget.
            assert!(
                err < 0.0025,
                "scale error {:.4} at latitude {} exceeds the bound",
                err,
                lat
            );
            lat += 0.5;
        }
    }

    #[test]
    fn leaf_memos_do_not_change_the_catalogue() {
        // Cold, warm, and re-warmed runs over the same grid and leaves must
        // be byte-identical: the memos may only ever change cost.
        let a = row(0.0, 60);
        let b = row(6.0, 60);
        let c = row(-6.0, 60);
        let d = row(3.0, 60);
        let tracks: Vec<(&str, &[GpsPoint])> = vec![("a", &a), ("b", &b), ("c", &c), ("d", &d)];
        let config = SectionConfig::default();
        let cell = cluster_cell_size(&config);
        let mut leaves = LeafMemos::default();
        let coverage = build_coverage_grid(
            &tracks,
            &[],
            cell,
            &Tunables::DEFAULT,
            &mut leaves.lift_candidates,
        );
        let run = |leaves: &mut LeafMemos| {
            let mut idx = 0usize;
            let mut records = Vec::new();
            detect_for_cluster_with_grid(
                "All",
                &tracks,
                &coverage,
                cell,
                &config,
                &Tunables::DEFAULT,
                &mut idx,
                &mut records,
                leaves,
                &HashMap::new(),
            )
        };
        let cold = run(&mut leaves);
        assert!(!cold.is_empty(), "the corridor must emit a section");
        let warm = run(&mut leaves);
        let rewarm = run(&mut leaves);
        let enc = |s: &Vec<FrequentSection>| serde_json::to_string(s).unwrap();
        assert_eq!(enc(&cold), enc(&warm));
        assert_eq!(enc(&cold), enc(&rewarm));
        assert!(
            !leaves.track_portions.is_empty() && !leaves.consensus.is_empty(),
            "the warm runs must actually have consulted the memos"
        );
    }

    #[test]
    fn lift_memo_matches_fresh_computation() {
        // The memoised candidate path must be observationally identical to
        // the fresh scan: same keep ranges cold, warm, and re-warmed.
        let up = climb(9.0e-5, 5.0, false, 80);
        let flat: Vec<GpsPoint> = (0..80)
            .map(|i| GpsPoint::with_elevation(46.0 + 9.0e-5 * i as f64, 7.02, 500.0))
            .collect();
        let tracks: Vec<(&str, &[GpsPoint])> = vec![("up", up.as_slice()), ("flat", &flat)];
        let fresh =
            build_coverage_grid(&tracks, &[], 100.0, &Tunables::DEFAULT, &mut HashMap::new());
        let mut memo = HashMap::new();
        let cold = build_coverage_grid(&tracks, &[], 100.0, &Tunables::DEFAULT, &mut memo);
        let warm = build_coverage_grid(&tracks, &[], 100.0, &Tunables::DEFAULT, &mut memo);
        assert_eq!(fresh.keep, cold.keep);
        assert_eq!(fresh.keep, warm.keep);
        assert!(memo.contains_key("up") && memo.contains_key("flat"));
    }

    #[test]
    fn late_descender_rescue_survives_the_candidate_memo() {
        // A lift span whose candidates were cached before the rescuing
        // descent arrived must still be rescued: the rescue is recomputed
        // over the present membership, never cached with the candidates.
        let up = climb(9.0e-5, 5.0, false, 80);
        let mut down = up.clone();
        down.reverse();
        let one: Vec<(&str, &[GpsPoint])> = vec![("up", up.as_slice())];
        let both: Vec<(&str, &[GpsPoint])> = vec![("up", up.as_slice()), ("down", &down)];
        let mut memo = HashMap::new();
        let alone = build_coverage_grid(&one, &[], 100.0, &Tunables::DEFAULT, &mut memo);
        assert_ne!(
            alone.keep[0],
            vec![(0usize, 79usize)],
            "the straight steep ascent must be excluded while unrescued"
        );
        let rescued = build_coverage_grid(&both, &[], 100.0, &Tunables::DEFAULT, &mut memo);
        assert_eq!(
            rescued.keep[0],
            vec![(0usize, 79usize)],
            "a straight descent over the same line rescues the cached span"
        );
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

    fn accept(line: &[GpsPoint], grid: &CellGrid) -> HashMap<Cell, Vec<(GpsPoint, u32)>> {
        let mut map: HashMap<Cell, Vec<(GpsPoint, u32)>> = HashMap::new();
        for p in line {
            map.entry(grid.cell_of(p.latitude, p.longitude))
                .or_default()
                .push((*p, 0));
        }
        map
    }

    #[test]
    fn probe_beside_accepted_line_is_represented() {
        let grid = CellGrid::new(100.0, 46.0);
        let accepted = accept(&row(0.0, 100), &grid);
        let shared: Vec<HashSet<u32>> = vec![[1u32, 2, 3].into_iter().collect()];
        let cand: HashSet<u32> = [1u32, 2, 3].into_iter().collect();
        let mask = probe_mask(
            &row(30.0, 100),
            &accepted,
            &shared,
            &cand,
            &|_| false,
            0.5,
            &grid,
            100.0,
        );
        assert!(mask.iter().all(|&m| m), "30 m offset is braid width");
        let mask = probe_mask(
            &row(300.0, 100),
            &accepted,
            &shared,
            &cand,
            &|_| false,
            0.5,
            &grid,
            100.0,
        );
        assert!(mask.iter().all(|&m| !m), "300 m offset is distinct ground");
    }

    #[test]
    fn a_foreign_populations_line_represents_nothing() {
        // The other bank of a river sits a braid width away in plan,
        // but its users never run this side: the candidate keeps its
        // ground. Regression: the south Rhone bank (99 exclusive
        // tracks) was suffocated by the north bank's accepted line.
        let grid = CellGrid::new(100.0, 46.0);
        let accepted = accept(&row(0.0, 100), &grid);
        let foreign: Vec<HashSet<u32>> = vec![[7u32, 8, 9].into_iter().collect()];
        let cand: HashSet<u32> = [1u32, 2, 3, 4].into_iter().collect();
        let mask = probe_mask(
            &row(60.0, 100),
            &accepted,
            &foreign,
            &cand,
            &|_| false,
            0.5,
            &grid,
            100.0,
        );
        assert!(
            mask.iter().all(|&m| !m),
            "a line whose users never run this corridor cannot represent it"
        );
        let sharing: Vec<HashSet<u32>> = vec![[1u32, 2, 7].into_iter().collect()];
        let mask = probe_mask(
            &row(60.0, 100),
            &accepted,
            &sharing,
            &cand,
            &|_| false,
            0.5,
            &grid,
            100.0,
        );
        assert!(
            mask.iter().all(|&m| m),
            "a line carrying half the candidate's users still represents it"
        );
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
            .map(|i| {
                GpsPoint::new(
                    46.0 + 9.0e-5 * 30.0,
                    7.0 + (i as f64 * 10.0 - 300.0) / 77_000.0,
                )
            })
            .collect();
        let tracks: Vec<(&str, &[GpsPoint])> = vec![
            ("a", corridor.as_slice()),
            ("b", braid.as_slice()),
            ("c", with_tail.as_slice()),
            ("x", cross.as_slice()),
        ];
        let coverage =
            build_coverage_grid(&tracks, &[], 100.0, &Tunables::DEFAULT, &mut HashMap::new());
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
        let support = candidate_support(
            &portions,
            &cell_set,
            &coverage,
            &tracks,
            100.0,
            &HashMap::new(),
        );
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
    fn a_minority_branch_behind_a_cliff_clips_but_a_taper_does_not() {
        // Twenty tracks run a 600 m body east; only two turn north for a
        // 200 m branch. The branch is short, thin, and behind a cliff, so
        // it leaves the display; a body that instead tapers gradually to
        // the same thinness keeps its full length.
        let east = |m: f64| -> Vec<GpsPoint> {
            let n = (m / 10.0) as usize;
            (0..=n)
                .map(|i| GpsPoint::new(46.0, 7.0 + i as f64 * 10.0 / 77_000.0))
                .collect()
        };
        let end_lng = 7.0 + 600.0 / 77_000.0;
        let mut branch = east(600.0);
        branch.extend((1..=20).map(|i| GpsPoint::new(46.0 + i as f64 * 10.0 / 111_132.0, end_lng)));
        let mut tracks: Vec<Vec<GpsPoint>> = (0..20).map(|_| east(600.0)).collect();
        tracks[0] = branch.clone();
        tracks[1] = branch.clone();
        let views: Vec<(&str, &[GpsPoint])> = tracks.iter().map(|t| ("", t.as_slice())).collect();
        let portions: Vec<Portion> = tracks
            .iter()
            .enumerate()
            .map(|(i, t)| (i, 0, t.len(), 0.0))
            .collect();
        let index = portion_point_index(&portions, &views, 46.0, 25.0);
        let (s, e) = minority_end_clip(&branch, &index, 46.0, 25.0, 25.0);
        let kept = crate::matching::calculate_route_distance(&branch[s..e]);
        assert_eq!(s, 0, "the busy body start is kept");
        assert!(
            (540.0..=680.0).contains(&kept),
            "the north branch is clipped back to the ~600 m body, got {kept:.0}"
        );

        // Gradual taper: twenty bodies, each 20 m shorter than the last,
        // so support falls one track per step. No cliff — kept whole.
        let taper: Vec<Vec<GpsPoint>> = (0..20).map(|i| east(600.0 - i as f64 * 20.0)).collect();
        let tviews: Vec<(&str, &[GpsPoint])> = taper.iter().map(|t| ("", t.as_slice())).collect();
        let tportions: Vec<Portion> = taper
            .iter()
            .enumerate()
            .map(|(i, t)| (i, 0, t.len(), 0.0))
            .collect();
        let tindex = portion_point_index(&tportions, &tviews, 46.0, 25.0);
        let n = taper[0].len();
        assert_eq!(
            minority_end_clip(&taper[0], &tindex, 46.0, 25.0, 25.0),
            (0, n),
            "a gradual taper is not a branch"
        );
    }

    #[test]
    fn out_and_back_scores_high_but_a_through_line_does_not() {
        // Out along a street and back over it: closed and antiparallel.
        let out = row(0.0, 50);
        let mut there_back = out.clone();
        there_back.extend(out.into_iter().rev().skip(1));
        assert!(
            out_and_back_penalty(&there_back, 20.0, 100.0) > 0.6,
            "a forward-and-reverse runs back over its own ground"
        );
        // A single straight pass ends far from its start: never scored.
        assert_eq!(out_and_back_penalty(&row(0.0, 50), 20.0, 100.0), 0.0);
    }

    #[test]
    fn tight_corner_on_a_closed_loop_is_not_an_out_and_back() {
        // A closed lap whose entry corner is hairpin-tight: the legs
        // into and out of the corner run antiparallel within `near`,
        // but only a corner's arc apart along the line — the corner's
        // own shape, not a retrace. An out-and-back's antiparallel
        // pairs sit a full out-leg apart. Regression: the Sion
        // athletics oval charged its own entry corner 0.064, failed
        // the clean bar, and lost the closed-lap render to a
        // lap-plus-stem pass.
        let mut xy: Vec<(f64, f64)> = Vec::new();
        for i in 0..=6 {
            xy.push((5.0 * i as f64, 0.0));
        }
        for i in 1..6 {
            let a = -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * i as f64 / 6.0;
            xy.push((30.0 + 7.5 * a.cos(), 7.5 + 7.5 * a.sin()));
        }
        for i in (0..=6).rev() {
            xy.push((5.0 * i as f64, 15.0));
        }
        // Wide return arc closing the loop from (0, 15) back to (0, 0).
        let r = (55.0f64 * 55.0 + 7.5 * 7.5).sqrt();
        let a0 = (7.5f64).atan2(55.0);
        for i in 1..=72 {
            let a = a0 + (2.0 * std::f64::consts::PI - 2.0 * a0) * i as f64 / 72.0;
            xy.push((-55.0 + r * a.cos(), 7.5 + r * a.sin()));
        }
        let pts: Vec<GpsPoint> = xy
            .iter()
            .map(|&(x, y)| {
                GpsPoint::new(
                    46.0 + y / 111_132.0,
                    7.0 + x / (111_320.0 * 46.0f64.to_radians().cos()),
                )
            })
            .collect();
        assert_eq!(
            out_and_back_penalty(&pts, 20.0, 100.0),
            0.0,
            "a tight corner is the loop's own shape, not a retrace"
        );
        assert_eq!(
            self_pass_penalty(&pts, 20.0, 100.0),
            0.0,
            "a single closed lap never revisits its own ground"
        );
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
            out_and_back_penalty(&pts, 20.0, 100.0),
            0.0,
            "a switchback climbs away; its hairpins are not a retrace"
        );
    }
}

#[cfg(test)]
mod seam_tests {
    use super::*;

    fn sec(id: &str, visits: u32, pts: Vec<GpsPoint>) -> FrequentSection {
        let dist = crate::matching::calculate_route_distance(&pts);
        FrequentSection {
            id: id.to_string(),
            name: None,
            sport_type: "All".to_string(),
            polyline: pts,
            representative_activity_id: "a".to_string(),
            activity_ids: vec!["a".to_string()],
            activity_portions: Vec::new(),
            route_ids: Vec::new(),
            visit_count: visits,
            distance_meters: dist,
            activity_traces: HashMap::new(),
            confidence: 1.0,
            observation_count: visits,
            average_spread: 0.0,
            point_density: Vec::new(),
            scale: None,
            is_user_defined: false,
            stability: 1.0,
            version: 1,
            updated_at: None,
            created_at: None,
            consensus_state: None,
        }
    }

    fn east_line(x0: f64, x1: f64, y: f64) -> Vec<GpsPoint> {
        let m_lng = 111_320.0 * 46.0f64.to_radians().cos();
        let n = ((x1 - x0).abs() / 10.0) as usize;
        (0..=n)
            .map(|k| {
                let x = x0 + (x1 - x0) * k as f64 / n as f64;
                GpsPoint::new(46.0 + y / 111_132.0, 7.0 + x / m_lng)
            })
            .collect()
    }

    #[test]
    fn an_overrunning_tail_clips_back_to_the_meet() {
        // The busier line covers x 0..1000; the quieter neighbour's
        // opening 200 m runs a GPS-drift lane (10 m) alongside it
        // before diverging east. The quieter line clips to the meet;
        // the busier keeps its full render. Regression: full
        // sec_all_146/149 double-drew ~200 m of the same trail from
        // two activities' traces.
        let busy = east_line(0.0, 1000.0, 0.0);
        let mut quiet = east_line(800.0, 1000.0, 10.0);
        quiet.extend(east_line(1010.0, 1780.0, 40.0));
        let mut sections = vec![sec("s_busy", 73, busy.clone()), sec("s_quiet", 65, quiet)];
        reconcile_seam_overruns(&mut sections, 46.0, 150.0);
        assert_eq!(
            sections[0].polyline.len(),
            busy.len(),
            "the busier line must keep its full render"
        );
        let q0 = &sections[1].polyline[0];
        let m_lng = 111_320.0 * 46.0f64.to_radians().cos();
        let qx = (q0.longitude - 7.0) * m_lng;
        assert!(
            qx > 990.0,
            "quieter line still starts at x {qx:.0}: the overrun did not clip to the meet"
        );
    }

    #[test]
    fn a_meeting_end_is_not_an_overrun() {
        // Chain members whose ends merely touch within tolerance for a
        // few metres must keep their renders.
        let a = east_line(0.0, 1000.0, 0.0);
        let b = east_line(990.0, 1990.0, 8.0);
        let mut sections = vec![sec("s_a", 40, a.clone()), sec("s_b", 30, b.clone())];
        reconcile_seam_overruns(&mut sections, 46.0, 150.0);
        assert_eq!(sections[0].polyline.len(), a.len());
        assert_eq!(sections[1].polyline.len(), b.len());
    }

    #[test]
    fn a_closed_ring_beside_a_busier_path_stays_whole() {
        // A lapped ring hugs the busier walking path around its own
        // ground: a revolution is self-justified, never an overrun.
        let m_lng = 111_320.0 * 46.0f64.to_radians().cos();
        let ring: Vec<GpsPoint> = (0..=60)
            .map(|k| {
                let a = std::f64::consts::PI * 2.0 * k as f64 / 60.0;
                GpsPoint::new(
                    46.0 + (70.0 * a.sin()) / 111_132.0,
                    7.0 + (70.0 + 70.0 * a.cos()) / m_lng,
                )
            })
            .collect();
        let path: Vec<GpsPoint> = (0..=80)
            .map(|k| {
                let a = std::f64::consts::PI * 2.0 * k as f64 / 80.0;
                GpsPoint::new(
                    46.0 + (85.0 * a.sin()) / 111_132.0,
                    7.0 + (70.0 + 85.0 * a.cos()) / m_lng,
                )
            })
            .collect();
        let ring_len = ring.len();
        let mut sections = vec![sec("s_path", 68, path), sec("s_ring", 9, ring)];
        reconcile_seam_overruns(&mut sections, 46.0, 150.0);
        assert_eq!(
            sections[1].polyline.len(),
            ring_len,
            "the closed ring must keep its full revolution"
        );
    }

    #[test]
    fn parallel_streets_are_never_touched() {
        // Two genuinely parallel lines 40 m apart sit wider than the
        // drift scale and keep their full renders.
        let a = east_line(0.0, 1000.0, 0.0);
        let b = east_line(0.0, 1000.0, 40.0);
        let mut sections = vec![sec("s_a", 40, a.clone()), sec("s_b", 30, b.clone())];
        reconcile_seam_overruns(&mut sections, 46.0, 150.0);
        assert_eq!(sections[0].polyline.len(), a.len());
        assert_eq!(sections[1].polyline.len(), b.len());
    }
}
