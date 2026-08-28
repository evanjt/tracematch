//! Enrichment and ranking of an emitted catalogue.
//!
//! Everything here reads a finished catalogue. It never feeds boundary
//! detection and never overrides the support floor: the score ranks,
//! labels and breaks ties, and a missing sensor degrades a feature to
//! neutral, never to a penalty.
//!
//! Grounding (full citations in `REFERENCES.md`, "Interestingness"):
//! detours reveal value (Salazar Miranda et al., Computers, Environment
//! and Urban Systems 2021; Quercia et al., ACM Hypertext 2014; leisure
//! detour magnitudes, Land 13(5):589, 2024), so ground near the far point
//! of an outing was the point of going out (`apex`) and winding ground is
//! preferred in its own right (`sinuosity`). Challenge and accomplishment
//! are core motives (Transport Policy 2017; Journal of Outdoor Recreation
//! and Tourism 2024) and flow needs clear goals with feedback (Psychology
//! of Sport and Exercise 2018, 2022), so a sustained gradient scores
//! (`grade`) and effort spent on the ground counts (`effort`). Attachment
//! grows with repetition in a stable context (Frontiers in Psychology
//! 10:1278, 2019), so sustained return scores (`months`) and stale ground
//! reads as history (`recency_days`). Converged-upon ground is sought out
//! (`converge`), and one-way ground reads as a descent or a circuit rather
//! than a connector (`oneway`).

use std::collections::{HashMap, HashSet};

use serde::{Deserialize, Serialize};

use crate::geo_utils::{ELEVATION_GAIN_HYSTERESIS_M, haversine_distance};
use crate::{Direction, GpsPoint};

/// Window a gradient must hold to read as a climb rather than a spike:
/// the climb-detection convention shared with the lift veto.
pub const SUSTAIN_M: f64 = 300.0;

/// Net grade (%) at which a line reads as a climb (or, negated, a
/// descent). Below it the line is flat or rolling.
pub const CLIMB_GRADE_PCT: f64 = 3.0;

/// Gain plus loss per kilometre (m/km) above which a line that is neither
/// climb nor descent reads as rolling rather than flat.
pub const ROLLING_M_PER_KM: f64 = 20.0;

/// Chord over arc below which a line closes on itself and reads as a loop.
pub const LOOP_CLOSURE: f64 = 0.15;

/// Share of a line's points inside a lift span at which the line itself
/// is carried ground.
pub const LIFT_SHARE: f64 = 0.5;

/// Base apron for the home-funnel discount, in matching tolerances: when
/// an outing starts or ends this close to a section, its approach and
/// leave bearings measure where the athlete lives, not which ground they
/// chose, so the outing stays out of `converge`. Plateau in the lab
/// (`LAB_FUNNEL_MULT` 1.5-3 moves no top-ten row).
pub const FUNNEL_MULT: f64 = 2.0;

/// Staleness charged to a section none of whose visits carry a date.
pub const UNDATED_RECENCY_DAYS: f64 = 36500.0;

/// What a line's shape and profile say it is. Terrain first, then shape:
/// a climb that closes on itself is still a climb.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SectionClass {
    Climb,
    Descent,
    Rolling,
    Flat,
    Loop,
}

impl SectionClass {
    pub fn as_str(self) -> &'static str {
        match self {
            SectionClass::Climb => "climb",
            SectionClass::Descent => "descent",
            SectionClass::Rolling => "rolling",
            SectionClass::Flat => "flat",
            SectionClass::Loop => "loop",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "climb" => SectionClass::Climb,
            "descent" => SectionClass::Descent,
            "rolling" => SectionClass::Rolling,
            "flat" => SectionClass::Flat,
            "loop" => SectionClass::Loop,
            _ => return None,
        })
    }
}

/// Profile and shape of one line, computed from its own polyline only.
/// Elevation fields are None when the line lacks elevation coverage.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct Enrichment {
    pub elevation_gain_m: Option<f64>,
    pub elevation_loss_m: Option<f64>,
    /// Net grade (%) start to end, signed.
    pub avg_grade_percent: Option<f64>,
    /// Steepest absolute grade (%) held over [`SUSTAIN_M`].
    pub max_grade_percent: Option<f64>,
    /// Chord over arc, 0..1. A straight line is 1, a closed loop is 0.
    pub straightness: Option<f64>,
    pub klass: Option<SectionClass>,
    /// The line is carried ground (a lift) by its own geometry.
    pub is_lift: bool,
}

/// Elevation gain and loss with the same smoothing and hysteresis as the
/// gain the detector already reports, so the two never disagree.
fn elevation_gain_loss(points: &[GpsPoint]) -> Option<(f64, f64)> {
    if points.len() < 2 {
        return None;
    }
    let carrying = points.iter().filter(|p| p.elevation.is_some()).count();
    if (carrying as f64) < 0.9 * points.len() as f64 {
        return None;
    }
    let elevs: Vec<f64> = points.iter().filter_map(|p| p.elevation).collect();
    let n = elevs.len();
    let smoothed: Vec<f64> = (0..n)
        .map(|i| {
            let lo = i.saturating_sub(1);
            let hi = (i + 1).min(n - 1);
            elevs[lo..=hi].iter().sum::<f64>() / (hi - lo + 1) as f64
        })
        .collect();
    let (mut gain, mut loss) = (0.0, 0.0);
    let mut anchor = smoothed[0];
    for &e in &smoothed[1..] {
        let delta = e - anchor;
        if delta >= ELEVATION_GAIN_HYSTERESIS_M {
            gain += delta;
            anchor = e;
        } else if delta <= -ELEVATION_GAIN_HYSTERESIS_M {
            loss -= delta;
            anchor = e;
        }
    }
    Some((gain, loss))
}

/// Steepest absolute grade held over [`SUSTAIN_M`] (or the whole line
/// when shorter). Neighbour-smoothed so one elevation spike cannot fake
/// a grade. None without elevation.
pub fn max_sustained_grade(polyline: &[GpsPoint]) -> Option<f64> {
    let n = polyline.len();
    if n < 2 {
        return None;
    }
    let ele: Vec<Option<f64>> = polyline.iter().map(|p| p.elevation).collect();
    if ele.iter().flatten().count() < 2 {
        return None;
    }
    let mut cum = vec![0.0f64; n];
    for i in 1..n {
        cum[i] = cum[i - 1] + haversine_distance(&polyline[i - 1], &polyline[i]);
    }
    let smooth: Vec<Option<f64>> = (0..n)
        .map(|i| {
            let (mut s, mut c) = (0.0, 0u32);
            for e in ele[i.saturating_sub(1)..=(i + 1).min(n - 1)]
                .iter()
                .flatten()
            {
                s += e;
                c += 1;
            }
            (c > 0).then(|| s / c as f64)
        })
        .collect();
    let total = cum[n - 1];
    let window = SUSTAIN_M.min(total.max(1.0));
    let mut best = 0.0f64;
    let mut j = 0usize;
    for i in 0..n {
        if cum[i] + window > total + 1e-9 {
            break;
        }
        if j < i {
            j = i;
        }
        while j < n - 1 && cum[j] - cum[i] < window {
            j += 1;
        }
        if let (Some(a), Some(b)) = (smooth[i], smooth[j]) {
            let d = cum[j] - cum[i];
            if d > 1.0 {
                best = best.max((b - a).abs() / d * 100.0);
            }
        }
    }
    Some(best)
}

/// Chord over arc for a line, None when it has no length.
pub fn straightness(polyline: &[GpsPoint], distance_meters: f64) -> Option<f64> {
    let (a, b) = (polyline.first()?, polyline.last()?);
    if distance_meters <= 0.0 {
        return None;
    }
    Some((haversine_distance(a, b) / distance_meters).clamp(0.0, 1.0))
}

/// Class a line from its profile and shape. A closed line with no
/// terrain signal is a loop; terrain wins when present.
pub fn classify(
    avg_grade_percent: Option<f64>,
    gain_m: Option<f64>,
    loss_m: Option<f64>,
    straightness: Option<f64>,
    distance_meters: f64,
) -> Option<SectionClass> {
    if let Some(grade) = avg_grade_percent {
        if grade >= CLIMB_GRADE_PCT {
            return Some(SectionClass::Climb);
        }
        if grade <= -CLIMB_GRADE_PCT {
            return Some(SectionClass::Descent);
        }
    }
    if straightness.is_some_and(|s| s < LOOP_CLOSURE) {
        return Some(SectionClass::Loop);
    }
    match (gain_m, loss_m) {
        (Some(g), Some(l)) if distance_meters > 0.0 => {
            let per_km = (g + l) / (distance_meters / 1000.0);
            Some(if per_km >= ROLLING_M_PER_KM {
                SectionClass::Rolling
            } else {
                SectionClass::Flat
            })
        }
        _ => None,
    }
}

/// Enrich one line from its own polyline. Elevation fields stay None
/// without coverage; class stays None when neither terrain nor shape
/// says anything.
pub fn enrich(polyline: &[GpsPoint], distance_meters: f64) -> Enrichment {
    let (gain, loss) = match elevation_gain_loss(polyline) {
        Some((g, l)) => (Some(g), Some(l)),
        None => (None, None),
    };
    let avg_grade = crate::geo_utils::elevation_stats(polyline).map(|(_, grade)| grade);
    let max_grade = max_sustained_grade(polyline);
    let straight = straightness(polyline, distance_meters);
    let klass = classify(avg_grade, gain, loss, straight, distance_meters);
    let lifted: usize = super::unified::lift_spans(polyline, None)
        .iter()
        .map(|&(s, e)| e.saturating_sub(s) + 1)
        .sum();
    let is_lift = !polyline.is_empty() && lifted as f64 >= LIFT_SHARE * polyline.len() as f64;
    Enrichment {
        elevation_gain_m: gain,
        elevation_loss_m: loss,
        avg_grade_percent: avg_grade,
        max_grade_percent: max_grade,
        straightness: straight,
        klass,
        is_lift,
    }
}

/// One pass of one activity over a line, as the catalogue records it.
#[derive(Debug, Clone)]
pub struct Traversal {
    pub start: usize,
    pub end: usize,
    pub direction: Direction,
    /// Effort on this pass as a percentile (0..1) of the athlete's own
    /// norm for the sport; None when the pass carried no sensor.
    pub effort: Option<f64>,
}

/// An activity that visited a line, with every pass it made.
#[derive(Debug, Clone)]
pub struct Member<'a> {
    pub activity_id: &'a str,
    pub traversals: Vec<Traversal>,
}

/// A line to rank.
#[derive(Debug, Clone)]
pub struct Candidate<'a> {
    pub id: &'a str,
    pub polyline: &'a [GpsPoint],
    pub distance_meters: f64,
    pub members: Vec<Member<'a>>,
}

/// One activity's track and start, looked up by id while ranking.
#[derive(Debug, Clone, Copy)]
pub struct Outing<'a> {
    /// ISO date or timestamp; None counts as undated.
    pub date: Option<&'a str>,
    pub points: &'a [GpsPoint],
}

/// Per-line ranking features and the score they combine into.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RankFeatures {
    /// Mean share of the outing's roam at which the line sits.
    pub apex: f64,
    /// Steepest grade (%) held over [`SUSTAIN_M`], 0 without elevation.
    pub grade: f64,
    /// Distinct calendar months with a visit.
    pub months: u32,
    /// 1 minus chord over arc.
    pub sinuosity: f64,
    /// Effective number of approach and leave directions.
    pub converge: f64,
    /// |same - reverse| over traversals.
    pub oneway: f64,
    /// Days from the newest visit to the newest outing anywhere.
    pub recency_days: f64,
    /// Mean effort percentile over the passes that carried a sensor.
    pub effort: Option<f64>,
    /// Equal-weight mean of the feature percentile ranks within the
    /// ranked set, 0..1. A set of one scores 0.5.
    pub score: f64,
    /// The same score ranked among the section's own sport only; equal
    /// to `score` until a per-sport rank has run.
    pub sport_score: f64,
}

/// Civil date ("YYYY-MM-DD...") to a day count (Hinnant's algorithm).
/// Years before 2000 are anonymised placeholders and read as undated.
pub fn day_of(date: &str) -> Option<i64> {
    let y: i64 = date.get(0..4)?.parse().ok()?;
    if y < 2000 {
        return None;
    }
    let m: i64 = date.get(5..7)?.parse().ok()?;
    let d: i64 = date.get(8..10)?.parse().ok()?;
    let yy = if m <= 2 { y - 1 } else { y };
    let era = if yy >= 0 { yy } else { yy - 399 } / 400;
    let yoe = yy - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    Some(era * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy)
}

fn bearing_deg(a: &GpsPoint, b: &GpsPoint) -> f64 {
    let (la, lb) = (a.latitude.to_radians(), b.latitude.to_radians());
    let dl = (b.longitude - a.longitude).to_radians();
    let y = dl.sin() * lb.cos();
    let x = la.cos() * lb.sin() - la.sin() * lb.cos() * dl.cos();
    (y.atan2(x).to_degrees() + 360.0) % 360.0
}

/// Walk back from `from` until `dist` metres of trace have accumulated.
/// None when the trace ends first.
fn point_at_distance_back(pts: &[GpsPoint], from: usize, dist: f64) -> Option<GpsPoint> {
    let mut acc = 0.0;
    let mut i = from.min(pts.len().saturating_sub(1));
    while i > 0 {
        acc += haversine_distance(&pts[i - 1], &pts[i]);
        i -= 1;
        if acc >= dist {
            return Some(pts[i]);
        }
    }
    None
}

fn point_at_distance_fwd(pts: &[GpsPoint], from: usize, dist: f64) -> Option<GpsPoint> {
    let mut acc = 0.0;
    let mut i = from;
    while i + 1 < pts.len() {
        acc += haversine_distance(&pts[i], &pts[i + 1]);
        i += 1;
        if acc >= dist {
            return Some(pts[i]);
        }
    }
    None
}

fn sector(bearing: f64) -> usize {
    (((bearing + 22.5) % 360.0) / 45.0) as usize % 8
}

/// Rank a set of lines against each other. The score is a percentile
/// within `candidates`, so the same line scores differently in a
/// per-sport set and a pooled one; callers rank each context they show.
/// `newest_day` is the day count of the newest outing anywhere, the
/// "now" recency is measured from (a static corpus must not go stale by
/// being analysed later); None derives it from the outings given.
/// Output is sorted best first, ties by id, so equal input gives equal
/// output.
pub fn rank(
    candidates: &[Candidate],
    outings: &HashMap<String, Outing>,
    proximity: f64,
    newest_day: Option<i64>,
) -> Vec<(String, RankFeatures)> {
    let newest = newest_day.unwrap_or_else(|| {
        outings
            .values()
            .filter_map(|o| o.date.and_then(day_of))
            .max()
            .unwrap_or(0)
    });
    let funnel_r = FUNNEL_MULT * proximity;
    let mut feats: Vec<(String, RankFeatures)> = Vec::with_capacity(candidates.len());
    for c in candidates {
        let mut apex_vals: Vec<f64> = Vec::new();
        let mut sectors = [0usize; 8];
        let (mut same, mut rev) = (0usize, 0usize);
        let mut months: HashSet<String> = HashSet::new();
        let mut seen: HashSet<&str> = HashSet::new();
        let mut last_day = i64::MIN;
        let mut efforts: Vec<f64> = Vec::new();
        for m in &c.members {
            if !seen.insert(m.activity_id) || m.traversals.is_empty() {
                continue;
            }
            for t in &m.traversals {
                match t.direction {
                    Direction::Same => same += 1,
                    Direction::Reverse => rev += 1,
                    _ => {}
                }
                if let Some(e) = t.effort {
                    efforts.push(e);
                }
            }
            let Some(o) = outings.get(m.activity_id) else {
                continue;
            };
            if let Some(d) = o.date {
                if d.len() >= 7 && day_of(d).is_some() {
                    months.insert(d[..7].to_string());
                }
                if let Some(day) = day_of(d) {
                    last_day = last_day.max(day);
                }
            }
            let pts = o.points;
            if pts.len() < 2 {
                continue;
            }
            let terminal_carried = {
                let head = &pts[0];
                let tail = &pts[pts.len() - 1];
                c.polyline.iter().any(|p| {
                    haversine_distance(head, p) < funnel_r || haversine_distance(tail, p) < funnel_r
                })
            };
            let start = &pts[0];
            let roam = pts
                .iter()
                .step_by(10)
                .map(|p| haversine_distance(start, p))
                .fold(0.0, f64::max);
            // One apex sample per activity: laps of the same ground sit in
            // the same place on the outing.
            let first = &m.traversals[0];
            let mid = &pts[((first.start + first.end) / 2).min(pts.len() - 1)];
            if roam > 50.0 {
                apex_vals.push((haversine_distance(start, mid) / roam).min(1.0));
            }
            if terminal_carried {
                continue;
            }
            for t in &m.traversals {
                let st = t.start.min(pts.len() - 1);
                let en = t.end.min(pts.len() - 1);
                if let Some(p) = point_at_distance_back(pts, st, proximity) {
                    sectors[sector(bearing_deg(&p, &pts[st]))] += 1;
                }
                if let Some(p) = point_at_distance_fwd(pts, en, proximity) {
                    sectors[sector(bearing_deg(&pts[en], &p))] += 1;
                }
            }
        }
        let apex = if apex_vals.is_empty() {
            0.0
        } else {
            apex_vals.iter().sum::<f64>() / apex_vals.len() as f64
        };
        let total_b: usize = sectors.iter().sum();
        let converge = if total_b == 0 {
            1.0
        } else {
            sectors
                .iter()
                .filter(|&&n| n > 0)
                .map(|&n| {
                    let p = n as f64 / total_b as f64;
                    -p * p.ln()
                })
                .sum::<f64>()
                .exp()
        };
        let trav_total = same + rev;
        let oneway = if trav_total == 0 {
            0.0
        } else {
            (same as f64 - rev as f64).abs() / trav_total as f64
        };
        let sinuosity = 1.0 - straightness(c.polyline, c.distance_meters).unwrap_or(1.0);
        let effort =
            (!efforts.is_empty()).then(|| efforts.iter().sum::<f64>() / efforts.len() as f64);
        feats.push((
            c.id.to_string(),
            RankFeatures {
                apex,
                grade: max_sustained_grade(c.polyline).unwrap_or(0.0),
                months: months.len() as u32,
                sinuosity,
                converge,
                oneway,
                recency_days: if last_day == i64::MIN {
                    UNDATED_RECENCY_DAYS
                } else {
                    (newest - last_day).max(0) as f64
                },
                effort,
                score: 0.0,
                sport_score: 0.0,
            },
        ));
    }
    score(&mut feats);
    feats.sort_by(|a, b| {
        b.1.score
            .partial_cmp(&a.1.score)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.0.cmp(&b.0))
    });
    feats
}

/// Percentile-normalise each feature within the set, equal weights, ties
/// at their average rank. A missing effort sits at the neutral 0.5.
fn score(feats: &mut [(String, RankFeatures)]) {
    let n = feats.len();
    if n == 0 {
        return;
    }
    if n == 1 {
        feats[0].1.score = 0.5;
        feats[0].1.sport_score = 0.5;
        return;
    }
    let cols: Vec<fn(&RankFeatures) -> Option<f64>> = vec![
        |f| Some(f.apex),
        |f| Some(f.grade),
        |f| Some(f.months as f64),
        |f| Some(f.sinuosity),
        |f| Some(f.converge),
        |f| Some(f.oneway),
        // Fresher ranks higher: percentile of the negated staleness.
        |f| Some(-f.recency_days),
        |f| f.effort,
    ];
    let n_cols = cols.len() as f64;
    let mut pct_sum = vec![0.0f64; n];
    for col in cols {
        let mut order: Vec<usize> = (0..n).filter(|&i| col(&feats[i].1).is_some()).collect();
        for i in 0..n {
            if col(&feats[i].1).is_none() {
                pct_sum[i] += 0.5;
            }
        }
        let m = order.len();
        if m < 2 {
            for &i in &order {
                pct_sum[i] += 0.5;
            }
            continue;
        }
        let v = |i: usize| col(&feats[i].1).unwrap_or(0.0);
        order.sort_by(|&a, &b| v(a).partial_cmp(&v(b)).unwrap_or(std::cmp::Ordering::Equal));
        let mut i = 0;
        while i < m {
            let mut j = i;
            while j + 1 < m && (v(order[j + 1]) - v(order[i])).abs() < 1e-12 {
                j += 1;
            }
            let avg = (i + j) as f64 / 2.0 / (m - 1) as f64;
            for k in i..=j {
                pct_sum[order[k]] += avg;
            }
            i = j + 1;
        }
    }
    for (idx, f) in feats.iter_mut().enumerate() {
        f.1.score = pct_sum[idx] / n_cols;
        f.1.sport_score = f.1.score;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(n: usize, step_deg: f64, ele: impl Fn(usize) -> Option<f64>) -> Vec<GpsPoint> {
        (0..n)
            .map(|i| GpsPoint {
                latitude: 46.0 + i as f64 * step_deg,
                longitude: 7.0,
                elevation: ele(i),
            })
            .collect()
    }

    fn arc(points: &[GpsPoint]) -> f64 {
        points
            .windows(2)
            .map(|w| haversine_distance(&w[0], &w[1]))
            .sum()
    }

    #[test]
    fn a_steady_rise_is_a_climb_with_matching_gain_and_grade() {
        // 40 points, ~11 m apart, rising 1 m per point: ~9 % net grade.
        let pts = line(40, 0.0001, |i| Some(i as f64));
        let e = enrich(&pts, arc(&pts));
        assert_eq!(e.klass, Some(SectionClass::Climb));
        assert!(e.elevation_gain_m.unwrap() > 30.0);
        assert_eq!(e.elevation_loss_m, Some(0.0));
        assert!(e.max_grade_percent.unwrap() > 5.0);
        assert!(e.straightness.unwrap() > 0.99);
        assert!(!e.is_lift);
    }

    #[test]
    fn the_same_line_reversed_is_a_descent_with_the_loss() {
        let mut pts = line(40, 0.0001, |i| Some(i as f64));
        pts.reverse();
        let e = enrich(&pts, arc(&pts));
        assert_eq!(e.klass, Some(SectionClass::Descent));
        assert_eq!(e.elevation_gain_m, Some(0.0));
        assert!(e.elevation_loss_m.unwrap() > 30.0);
    }

    #[test]
    fn no_elevation_leaves_the_terrain_fields_empty_and_the_class_open() {
        let pts = line(40, 0.0001, |_| None);
        let e = enrich(&pts, arc(&pts));
        assert_eq!(e.elevation_gain_m, None);
        assert_eq!(e.max_grade_percent, None);
        assert_eq!(e.klass, None);
    }

    #[test]
    fn a_closed_ring_is_a_loop() {
        let pts: Vec<GpsPoint> = (0..=60)
            .map(|i| {
                let t = i as f64 / 60.0 * std::f64::consts::TAU;
                GpsPoint {
                    latitude: 46.0 + 0.002 * t.sin(),
                    longitude: 7.0 + 0.003 * t.cos(),
                    elevation: Some(500.0),
                }
            })
            .collect();
        let e = enrich(&pts, arc(&pts));
        assert_eq!(e.klass, Some(SectionClass::Loop));
        assert!(e.straightness.unwrap() < LOOP_CLOSURE);
    }

    #[test]
    fn a_spike_does_not_fake_a_sustained_grade() {
        let pts = line(60, 0.0001, |i| Some(if i == 30 { 80.0 } else { 100.0 }));
        let g = max_sustained_grade(&pts).unwrap();
        assert!(g < 3.0, "spike read as {g:.1} % sustained");
    }

    fn candidate<'a>(id: &'a str, poly: &'a [GpsPoint], members: Vec<Member<'a>>) -> Candidate<'a> {
        Candidate {
            id,
            polyline: poly,
            distance_meters: arc(poly),
            members,
        }
    }

    fn pass(start: usize, end: usize, effort: Option<f64>) -> Traversal {
        Traversal {
            start,
            end,
            direction: Direction::Same,
            effort,
        }
    }

    #[test]
    fn scores_are_percentiles_that_favour_the_richer_line() {
        let track = line(200, 0.0001, |_| None);
        let poly_far: Vec<GpsPoint> = track[150..190].to_vec();
        let poly_near: Vec<GpsPoint> = track[5..45].to_vec();
        let outings: HashMap<String, Outing> = [
            (
                "a".to_string(),
                Outing {
                    date: Some("2026-01-05"),
                    points: &track,
                },
            ),
            (
                "b".to_string(),
                Outing {
                    date: Some("2026-03-05"),
                    points: &track,
                },
            ),
        ]
        .into_iter()
        .collect();
        let far = candidate(
            "far",
            &poly_far,
            vec![
                Member {
                    activity_id: "a",
                    traversals: vec![pass(150, 190, Some(0.9))],
                },
                Member {
                    activity_id: "b",
                    traversals: vec![pass(150, 190, Some(0.8))],
                },
            ],
        );
        let near = candidate(
            "near",
            &poly_near,
            vec![Member {
                activity_id: "a",
                traversals: vec![pass(5, 45, None)],
            }],
        );
        let ranked = rank(&[near.clone(), far.clone()], &outings, 100.0, None);
        assert_eq!(ranked[0].0, "far");
        let f = &ranked[0].1;
        assert!(f.apex > ranked[1].1.apex);
        assert_eq!(f.months, 2);
        assert!((f.effort.unwrap() - 0.85).abs() < 1e-9);
        assert_eq!(ranked[1].1.effort, None);
        assert!(ranked[0].1.score > ranked[1].1.score);
        assert!(ranked.iter().all(|(_, f)| (0.0..=1.0).contains(&f.score)));
        let again = rank(&[far, near], &outings, 100.0, None);
        assert_eq!(again, ranked, "input order must not change the ranking");
    }

    #[test]
    fn a_set_of_one_scores_neutral_and_undated_ground_reads_stale() {
        let track = line(100, 0.0001, |_| None);
        let poly: Vec<GpsPoint> = track[10..50].to_vec();
        let outings: HashMap<String, Outing> = [(
            "a".to_string(),
            Outing {
                date: None,
                points: &track,
            },
        )]
        .into_iter()
        .collect();
        let only = candidate(
            "only",
            &poly,
            vec![Member {
                activity_id: "a",
                traversals: vec![pass(10, 50, None)],
            }],
        );
        let ranked = rank(&[only], &outings, 100.0, None);
        assert_eq!(ranked[0].1.score, 0.5);
        assert_eq!(ranked[0].1.recency_days, UNDATED_RECENCY_DAYS);
        assert_eq!(ranked[0].1.months, 0);
    }

    #[test]
    fn day_counts_are_monotonic_and_placeholders_are_undated() {
        assert!(day_of("2026-03-01").unwrap() > day_of("2026-02-28").unwrap());
        assert_eq!(
            day_of("2026-03-01").unwrap() - day_of("2026-02-28").unwrap(),
            1
        );
        assert_eq!(day_of("1970-01-01"), None);
        assert_eq!(day_of("garbage"), None);
    }
}
