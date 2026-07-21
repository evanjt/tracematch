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
//!    real usage represented) and a candidate whose own geometry mostly
//!    runs within a cell's width of accepted polylines is not emitted
//!    at all. Nothing is merged and nothing synthetic is created —
//!    braid-lane twins simply lose to the better line, and their
//!    traversals still match it. Under-representing beats scattering
//!    near-duplicates. "Too close" needs no new constant: laterally it
//!    is one cell (braid width, proximity/2), and "mostly" is
//!    `1 − divergence`, the same share that defines same-traffic.
//!    Geometry against polylines, not cell blobs: a loop or variant
//!    beside a corridor swings wide of the winner's line and keeps its
//!    own distinct shape.

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
    /// is lost on both corpora. Residual risk is why the app-side
    /// speed check remains the eventual robust signal.
    pub lift_span_m: f64,
    /// Lift exclusion: minimum sustained ascent grade for a window.
    /// Measured chairlifts climb at 17-34%; steep walked ground
    /// overlaps the same range, which is why grade alone never
    /// classifies (straightness and jitter do). Working band measured
    /// at 0.18-0.26; at 0.30 the floor exceeds real lift grades and
    /// cable-car ground resurfaces.
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
    /// 1.02. Device-sensitive; the app-side speed check is the
    /// eventual robust signal.
    pub jitter_human_min: f64,
    /// Lift exclusion: matching tolerance when hunting a straight
    /// descent along a lift candidate's own line (the rescue for
    /// stairs and fall-line paths that people also walk down).
    /// Plateau: 40-80 m byte-flat on both corpora.
    pub descent_match_m: f64,
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
        descent_match_m: 60.0,
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
/// merge into spans. A span must then survive two vetoes: real net rise
/// (guards barometric drift), and low micro-jitter. Jitter is the raw
/// arc over the smoothed arc: a cabin glides (measured lift median
/// 1.02, p95 1.065) while a walker on steep ground wobbles (every
/// walked steep-straight climb measured, across four countries, sits at
/// 1.053 or higher, and bootpack ascents inside snowboard days land in
/// the same band). Jitter depends on device sampling, so the constant
/// is device-sensitive; the app-side speed check is the eventual robust
/// signal. Marked spans contribute no evidence to the coverage grid, so
/// lift ground never becomes a section and never bridges the runs it
/// connects. A descending lift ride is geometrically indistinguishable
/// from a steep descent and is also left to the app-side speed check.
/// Constants live in [`Tunables`] with their measured envelopes.
pub fn lift_spans(pts: &[GpsPoint]) -> Vec<(usize, usize)> {
    lift_spans_tuned(pts, &Tunables::DEFAULT)
}

/// [`lift_spans`] with explicit [`Tunables`], for the lab's sweeps.
pub fn lift_spans_tuned(pts: &[GpsPoint], tun: &Tunables) -> Vec<(usize, usize)> {
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
pub fn confirmed_lift_spans(tracks: &[(&str, &[GpsPoint])]) -> Vec<Vec<(usize, usize)>> {
    confirmed_lift_spans_tuned(tracks, &Tunables::DEFAULT)
}

/// [`confirmed_lift_spans`] with explicit [`Tunables`], for the lab's
/// sweeps.
pub fn confirmed_lift_spans_tuned(
    tracks: &[(&str, &[GpsPoint])],
    tun: &Tunables,
) -> Vec<Vec<(usize, usize)>> {
    let candidates: Vec<Vec<(usize, usize)>> = tracks
        .iter()
        .map(|(_, pts)| lift_spans_tuned(pts, tun))
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
    cell_size_m: f64,
    tun: &Tunables,
) -> CoverageGrid {
    let ref_lat: f64 = {
        let mut sum = 0.0;
        let mut n = 0usize;
        for (_, pts) in tracks {
            for p in pts.iter().step_by(50) {
                sum += p.latitude;
                n += 1;
            }
        }
        if n == 0 { 0.0 } else { sum / n as f64 }
    };
    let grid = CellGrid::new(cell_size_m, ref_lat);

    let fine = CellGrid::new(cell_size_m / tun.pass_subgrid, ref_lat);

    let lift = confirmed_lift_spans_tuned(tracks, tun);

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
        let spans = &lift[t_idx];
        let mut keep: Vec<(usize, usize)> = Vec::new();
        let mut cursor = 0usize;
        for &(s, e) in spans {
            if s > cursor {
                keep.push((cursor, s - 1));
            }
            cursor = e + 1;
        }
        if cursor < pts.len() {
            keep.push((cursor, pts.len() - 1));
        }

        for (r_idx, &(rs, re)) in keep.iter().enumerate() {
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

    let mut uf: UnionFind<usize> = UnionFind::with_capacity(supernodes.len());
    for i in 0..supernodes.len() {
        uf.make_set(i);
    }
    // Non-section fragments and their adjacent components, for
    // directed absorption after the pairwise decisions.
    let mut frag_neighbours: HashMap<usize, Vec<usize>> = HashMap::new();

    for (&(a, b), pairs) in &boundary {
        // A pass-class boundary is exempt from merging only when the
        // change is the experience of MOST of the traffic at the join —
        // a turnaround or loop mouth for its users. A minority's
        // turnaround (walkers turning back on a corridor runners
        // continue along) must not cut the majority's through
        // corridor. Aggregated per-track over every adjacent cell pair
        // on the join.
        let mut shared = 0usize;
        let mut mismatch = 0usize;
        for &(ca, cb) in pairs {
            let (Some(pa), Some(pb)) =
                (coverage.cell_passes.get(&ca), coverage.cell_passes.get(&cb))
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
        if shared > 0 && mismatch as f64 > (0.5 * shared as f64).max(1.0) {
            log::debug!(
                "[Unified] keep cut MULT {:?}|{:?}: shared={} mismatch={} at {:?}",
                supernodes[a].cells[0],
                supernodes[b].cells[0],
                shared,
                mismatch,
                pairs[0].0,
            );
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
        // A fork is real when the DEPARTING traffic is collected by a
        // section-worthy corridor at the join. Testing the through
        // traffic instead would let a collinear continuation of the
        // same corridor justify the cut — it shares the through
        // traffic by definition — and chain fragments would hold each
        // other apart forever. A branch must also carry enough
        // activities to be a section in its own right: on a corridor
        // used by three activities, one turning off is noise.
        let needed = (divergence * through.len() as f64).max(min_activities as f64);

        // A fork needs a substantial third corridor that physically
        // meets the join — direct adjacency only. A corridor two cells
        // away (the far bank, reached by a stub too short to be a
        // section) must not arbitrate a cut it never visibly touches.
        // A minor side path does not end a busy corridor.
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
        let forked = third
            .iter()
            .any(|&o| supernodes[o].tracks.intersection(&leavers).count() as f64 >= needed);

        if forked {
            log::debug!(
                "[Unified] keep cut FORK {:?}({} cells)|{:?}({} cells): through={} needed={:.1} branches={:?}",
                supernodes[a].cells[0],
                supernodes[a].cells.len(),
                supernodes[b].cells[0],
                supernodes[b].cells.len(),
                through.len(),
                needed,
                third
                    .iter()
                    .map(|&o| {
                        (
                            supernodes[o].cells[0],
                            supernodes[o].cells.len(),
                            supernodes[o].tracks.intersection(&leavers).count(),
                        )
                    })
                    .collect::<Vec<_>>()
            );
        } else {
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
    let mut t_indices: Vec<u32> = node.tracks.iter().copied().collect();
    t_indices.sort_unstable();

    let mut portions: Vec<Portion> = Vec::new();
    for &t_idx in &t_indices {
        let pts = sport_tracks[t_idx as usize].1;
        let Some((mut s, mut e, _)) = longest_run_in_cells(pts, &cell_set, &coverage.grid) else {
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

/// Detect sections for one sport's tracks via the unified pipeline.
fn detect_for_sport(
    sport: &str,
    sport_tracks: &[(&str, &[GpsPoint])],
    config: &SectionConfig,
    tun: &Tunables,
    section_idx: &mut usize,
) -> Vec<FrequentSection> {
    if sport_tracks.len() < config.min_activities as usize {
        return Vec::new();
    }

    let cell_size = (config.proximity_threshold * 0.5).clamp(50.0, 150.0);
    let coverage = build_coverage_grid(sport_tracks, cell_size, tun);

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
    for _ in 0..5 {
        let before = supernodes.len();
        let worthy =
            section_worthiness(&supernodes, &coverage, sport_tracks, config, cell_size, tun);
        supernodes = merge_non_fork_boundaries(
            supernodes,
            &coverage,
            divergence,
            config.min_activities,
            &worthy,
        );
        if supernodes.len() == before {
            break;
        }
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

    let track_map: HashMap<&str, &[GpsPoint]> =
        sport_tracks.iter().map(|(id, pts)| (*id, *pts)).collect();
    let activity_to_route: HashMap<&str, &str> = HashMap::new();

    let mut sections: Vec<FrequentSection> = Vec::new();

    // Candidates that could stand as sections, scored by the real usage
    // they represent (total portion metres).
    let mut candidates: Vec<(usize, Vec<Portion>, f64)> = Vec::new();
    for (n_idx, node) in supernodes.iter().enumerate() {
        // Rough length from core cell count (cells are ~square).
        let approx_len = node.cells.len() as f64 * cell_size;
        if approx_len < config.min_section_length {
            continue;
        }

        let portions = portions_for(node, &coverage, sport_tracks, config, cell_size, tun);
        if portions.is_empty()
            || !has_support(&portions, approx_len, config, opportunity(node, &coverage))
        {
            continue;
        }
        let score: f64 = portions.iter().map(|p| p.3).sum();
        candidates.push((n_idx, portions, score));
    }
    candidates.sort_by(|a, b| {
        b.2.partial_cmp(&a.2)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.0.cmp(&b.0))
    });

    let backoff_grid = CellGrid::new(cell_size, coverage.ref_lat);
    let mut accepted_pts: HashMap<Cell, Vec<GpsPoint>> = HashMap::new();
    for (n_idx, portions, score) in candidates {
        let node = &supernodes[n_idx];
        let approx_len = node.cells.len() as f64 * cell_size;
        // Selection backoff (rule 6): a candidate whose own geometry
        // mostly runs within a cell's width (braid width, proximity/2)
        // of accepted polylines is not emitted — the better line
        // already represents this way and the candidate's traversals
        // still match it. Geometry against polylines, not cell blobs:
        // a braid twin hugs the winner point for point, while a loop or
        // variant beside a corridor swings wide of it and keeps its own
        // distinct shape.
        let probe = portions
            .iter()
            .max_by(|a, b| a.3.partial_cmp(&b.3).unwrap_or(std::cmp::Ordering::Equal))
            .map(|&(t, s, e, _)| &sport_tracks[t].1[s..e]);
        let mut near = 0usize;
        let mut total = 0usize;
        if let Some(probe) = probe {
            for p in probe.iter().step_by(5) {
                total += 1;
                let c = backoff_grid.cell_of(p.latitude, p.longitude);
                let hit = (-1..=1i32).any(|dy| {
                    (-1..=1i32).any(|dx| {
                        accepted_pts.get(&(c.0 + dy, c.1 + dx)).is_some_and(|v| {
                            v.iter()
                                .any(|q| crate::geo_utils::haversine_distance(p, q) < cell_size)
                        })
                    })
                });
                if hit {
                    near += 1;
                }
            }
        }
        if total > 0 && near as f64 >= same_traffic * total as f64 {
            log::debug!(
                "[Unified] backoff {:?}({} cells, score {:.0}): {}/{} probe points already represented",
                node.cells[0],
                node.cells.len(),
                score,
                near,
                total,
            );
            continue;
        }

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
            // average: replace the consensus geometry with the medoid
            // activity's actual trimmed portion. Consensus results stay
            // as metadata (confidence, spread, density). The app's
            // reference-activity model (set reference, trim/extend)
            // builds on exactly this guarantee.
            if let Some(&(t_idx, s, e, dist)) = portions
                .iter()
                .find(|&&(t_idx, ..)| sport_tracks[t_idx].0 == section.representative_activity_id)
            {
                section.polyline = sport_tracks[t_idx].1[s..e].to_vec();
                section.distance_meters = dist;
            }
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
            for p in section.polyline.iter().step_by(3) {
                accepted_pts
                    .entry(backoff_grid.cell_of(p.latitude, p.longitude))
                    .or_default()
                    .push(*p);
            }
            sections.push(section);
        }
    }

    sections
}

/// Detect sections using the unified pipeline: coverage grid →
/// same-traffic supernodes → medoid + consensus geometry.
pub fn detect_sections_unified(
    tracks: &[(String, Vec<GpsPoint>)],
    sport_types: &HashMap<String, String>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    detect_sections_unified_tuned(tracks, sport_types, config, &Tunables::DEFAULT)
}

/// [`detect_sections_unified`] with explicit [`Tunables`]. The
/// validation lab's plateau sweeps run through here; production never
/// passes anything but [`Tunables::DEFAULT`].
pub fn detect_sections_unified_tuned(
    tracks: &[(String, Vec<GpsPoint>)],
    sport_types: &HashMap<String, String>,
    config: &SectionConfig,
    tun: &Tunables,
) -> Vec<FrequentSection> {
    // Partition tracks per sport; sections never span sports.
    let mut by_sport: HashMap<&str, Vec<(&str, &[GpsPoint])>> = HashMap::new();
    for (id, pts) in tracks {
        let sport = sport_types.get(id).map(|s| s.as_str()).unwrap_or("Unknown");
        by_sport
            .entry(sport)
            .or_default()
            .push((id.as_str(), pts.as_slice()));
    }

    let mut sport_names: Vec<&str> = by_sport.keys().copied().collect();
    sport_names.sort_unstable();

    let mut all_sections: Vec<FrequentSection> = Vec::new();
    for sport in sport_names {
        let sport_tracks = &by_sport[sport];
        let mut idx = 0usize;
        let sections = detect_for_sport(sport, sport_tracks, config, tun, &mut idx);
        all_sections.extend(sections);
    }

    info!("[Unified] {} sections total", all_sections.len());
    all_sections
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
        let spans = lift_spans(&pts);
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
        assert!(lift_spans(&pts).is_empty());
    }

    #[test]
    fn straight_steep_descent_is_not_lift() {
        let mut pts = climb(9.0e-5, 5.0, false, 80);
        pts.reverse();
        assert!(lift_spans(&pts).is_empty());
    }

    #[test]
    fn straight_chairlift_grade_is_lift() {
        // 26% sustained and dead straight: a typical chairlift line.
        let pts = climb(9.0e-5, 2.6, false, 80);
        assert_eq!(lift_spans(&pts).len(), 1);
    }

    #[test]
    fn gentle_straight_ascent_is_not_lift() {
        // 10% sustained: an ordinary straight road climb.
        let pts = climb(9.0e-5, 1.0, false, 80);
        assert!(lift_spans(&pts).is_empty());
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
        assert!(lift_spans(&pts).is_empty());
    }

    #[test]
    fn straight_retrace_downhill_rescues_stairs() {
        // Vineyard stairs: climbed straight, walked straight back down.
        let up = climb(9.0e-5, 2.6, false, 80);
        let mut down = up.clone();
        down.reverse();
        let tracks: Vec<(&str, &[GpsPoint])> =
            vec![("up", up.as_slice()), ("down", down.as_slice())];
        let confirmed = confirmed_lift_spans(&tracks);
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
        let confirmed = confirmed_lift_spans(&tracks);
        assert_eq!(confirmed[0].len(), 1);
    }
}
