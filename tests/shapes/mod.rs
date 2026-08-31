//! Deterministic synthetic ground for the unified-detector contracts.
//!
//! Every generator is a pure function of its arguments: no RNG, no clock.
//! GPS realism comes from a perpendicular sinusoidal wobble whose phase
//! varies per activity, which braids traces like receiver noise while
//! keeping corpora byte-reproducible — the order-free parity contract
//! depends on that.
//!
//! Geometry is authored in a local metre frame (x east, y north) around
//! 46°N 7°E and converted to degrees at the end.

#![allow(dead_code)]

use std::collections::HashMap;
use tracematch::GpsPoint;

pub const BASE_LAT: f64 = 46.0;
pub const BASE_LNG: f64 = 7.0;
const M_PER_DEG_LAT: f64 = 111_320.0;
pub const SPACING_M: f64 = 10.0;
/// Wobble that reads as human to the lift classifier's jitter veto.
pub const HUMAN_WOBBLE_M: f64 = 2.5;

pub fn to_gps(x: f64, y: f64) -> GpsPoint {
    GpsPoint::new(
        BASE_LAT + y / M_PER_DEG_LAT,
        BASE_LNG + x / (M_PER_DEG_LAT * BASE_LAT.to_radians().cos()),
    )
}

pub fn to_gps_ele(x: f64, y: f64, ele: f64) -> GpsPoint {
    let p = to_gps(x, y);
    GpsPoint::with_elevation(p.latitude, p.longitude, ele)
}

/// Straight-segment densification of a waypoint polyline at SPACING_M.
pub fn densify(waypoints: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = Vec::new();
    for w in waypoints.windows(2) {
        let (ax, ay) = w[0];
        let (bx, by) = w[1];
        let len = ((bx - ax).powi(2) + (by - ay).powi(2)).sqrt();
        let steps = (len / SPACING_M).ceil().max(1.0) as usize;
        for s in 0..steps {
            let t = s as f64 / steps as f64;
            out.push((ax + (bx - ax) * t, ay + (by - ay) * t));
        }
    }
    if let Some(&last) = waypoints.last() {
        out.push(last);
    }
    out
}

/// Circular arc from `a0` to `a1` radians (sign of the sweep sets the
/// direction), densified at SPACING_M.
pub fn arc(cx: f64, cy: f64, r: f64, a0: f64, a1: f64) -> Vec<(f64, f64)> {
    let sweep = a1 - a0;
    let steps = ((sweep.abs() * r) / SPACING_M).ceil().max(2.0) as usize;
    (0..=steps)
        .map(|i| {
            let a = a0 + sweep * i as f64 / steps as f64;
            (cx + r * a.cos(), cy + r * a.sin())
        })
        .collect()
}

/// Perpendicular sinusoidal wobble along the path's arc length. `phase`
/// varies per activity so traces braid instead of stacking.
pub fn wobble(pts: &[(f64, f64)], amp_m: f64, phase: f64) -> Vec<(f64, f64)> {
    const WAVELENGTH_M: f64 = 23.0;
    let mut out = Vec::with_capacity(pts.len());
    let mut acc = 0.0f64;
    for i in 0..pts.len() {
        if i > 0 {
            let (dx, dy) = (pts[i].0 - pts[i - 1].0, pts[i].1 - pts[i - 1].1);
            acc += (dx * dx + dy * dy).sqrt();
        }
        let (j, k) = if i + 1 < pts.len() {
            (i, i + 1)
        } else {
            (i.saturating_sub(1), i)
        };
        let (dx, dy) = (pts[k].0 - pts[j].0, pts[k].1 - pts[j].1);
        let len = (dx * dx + dy * dy).sqrt().max(1e-9);
        let (nx, ny) = (-dy / len, dx / len);
        let off = amp_m * (2.0 * std::f64::consts::PI * acc / WAVELENGTH_M + phase).sin();
        out.push((pts[i].0 + nx * off, pts[i].1 + ny * off));
    }
    out
}

pub fn track(pts: &[(f64, f64)]) -> Vec<GpsPoint> {
    pts.iter().map(|&(x, y)| to_gps(x, y)).collect()
}

/// Convert with a linear elevation profile along the arc from `e0` to `e1`.
pub fn track_ele(pts: &[(f64, f64)], e0: f64, e1: f64) -> Vec<GpsPoint> {
    let mut arc_len = vec![0.0f64; pts.len()];
    for i in 1..pts.len() {
        let (dx, dy) = (pts[i].0 - pts[i - 1].0, pts[i].1 - pts[i - 1].1);
        arc_len[i] = arc_len[i - 1] + (dx * dx + dy * dy).sqrt();
    }
    let total = arc_len.last().copied().unwrap_or(0.0).max(1e-9);
    pts.iter()
        .zip(&arc_len)
        .map(|(&(x, y), &d)| to_gps_ele(x, y, e0 + (e1 - e0) * d / total))
        .collect()
}

/// Translate a corpus east so shape families combined into one corpus
/// occupy disjoint ground.
pub fn shift_east(tracks: Vec<(String, Vec<GpsPoint>)>, dx_m: f64) -> Vec<(String, Vec<GpsPoint>)> {
    let dlng = dx_m / (M_PER_DEG_LAT * BASE_LAT.to_radians().cos());
    tracks
        .into_iter()
        .map(|(id, pts)| {
            let moved = pts
                .into_iter()
                .map(|p| GpsPoint {
                    longitude: p.longitude + dlng,
                    ..p
                })
                .collect();
            (id, moved)
        })
        .collect()
}

/// Translate a corpus by whole degrees, for placing a shape family on
/// another continent or in the other hemisphere.
pub fn translate_deg(
    tracks: Vec<(String, Vec<GpsPoint>)>,
    dlat: f64,
    dlng: f64,
) -> Vec<(String, Vec<GpsPoint>)> {
    tracks
        .into_iter()
        .map(|(id, pts)| {
            let moved = pts
                .into_iter()
                .map(|p| GpsPoint {
                    latitude: p.latitude + dlat,
                    longitude: p.longitude + dlng,
                    ..p
                })
                .collect();
            (id, moved)
        })
        .collect()
}

/// Pooled sport map: detection sees one bucket, like production.
pub fn pooled(tracks: &[(String, Vec<GpsPoint>)]) -> HashMap<String, String> {
    tracks
        .iter()
        .map(|(id, _)| (id.clone(), "All".to_string()))
        .collect()
}

fn phase(i: usize) -> f64 {
    i as f64 * 1.7
}

// ------------------------------------------------------------------ shapes

/// Approach stem to an athletics oval, three laps, stem back home.
/// Stem ground is passed twice per outing, oval ground three times.
pub fn oval_stem(outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    use std::f64::consts::PI;
    (0..outings)
        .map(|i| {
            let mut path = densify(&[(0.0, 0.0), (600.0, 0.0)]);
            path.extend(arc(800.0, 0.0, 200.0, PI, PI + 6.0 * PI));
            path.extend(densify(&[(600.0, 0.0), (0.0, 0.0)]));
            (
                format!("oval_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// Out-and-back stick with a single-lap loop head.
pub fn lollipop(outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    use std::f64::consts::PI;
    (0..outings)
        .map(|i| {
            let mut path = densify(&[(0.0, 0.0), (800.0, 0.0)]);
            path.extend(arc(950.0, 0.0, 150.0, PI, PI + 2.0 * PI));
            path.extend(densify(&[(800.0, 0.0), (0.0, 0.0)]));
            (
                format!("lolli_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// Unique approaches and exits sharing one straight: only the straight
/// repeats, every whole activity is distinct. Per-outing lane offsets
/// braid the shared ground.
pub fn deviation_straight(outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    (0..outings)
        .map(|i| {
            let lane = -14.0 + 4.0 * i as f64;
            let start = (-350.0, 260.0 - 70.0 * i as f64);
            let exit = (1550.0, -260.0 + 65.0 * i as f64);
            let path = densify(&[start, (0.0, lane), (1200.0, lane), exit]);
            (
                format!("dev_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// Up one line, back down a parallel variant `offset` metres east,
/// linked at both ends. Same activity set on both lines.
pub fn parallel_variants(outings: usize, offset: f64) -> Vec<(String, Vec<GpsPoint>)> {
    (0..outings)
        .map(|i| {
            let path = densify(&[
                (0.0, 0.0),
                (0.0, 700.0),
                (offset, 700.0),
                (offset, 0.0),
                (0.0, 0.0),
            ]);
            (
                format!("var_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// A busy corridor whose continuation keeps only a minority of its
/// traffic: all `heavy` outings run the west stretch (x 0..1500), the
/// first `light` of them continue east to 3000. Everything is
/// collinear with staggered starts and stops (no turnaround, no third
/// corridor anywhere), so the only visible reason for the busy section
/// to end near 1500 is the majority usage change there.
pub fn cliff_tail(heavy: usize, light: usize) -> Vec<(String, Vec<GpsPoint>)> {
    (0..heavy)
        .map(|i| {
            let start = (-200.0 - 8.0 * i as f64, 0.0);
            let end = if i < light {
                (3000.0 + 8.0 * i as f64, 0.0)
            } else {
                (1500.0, 0.0)
            };
            let path = densify(&[start, end]);
            (
                format!("cliff_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// A straight body with a minority mid-life variant: every outing runs
/// x 0..1200 along y=0, but the first `divert` swap onto a parallel
/// strand 180 m north between x 400 and 800. The strand is real
/// repeated ground for its minority; the body keeps the majority. The
/// braid mouths are the only joins — no turnaround, no third corridor.
pub fn minority_braid(outings: usize, divert: usize) -> Vec<(String, Vec<GpsPoint>)> {
    (0..outings)
        .map(|i| {
            let start = (-200.0 - 8.0 * i as f64, 0.0);
            let end = (2400.0 + 8.0 * i as f64, 0.0);
            let path = if i < divert {
                vec![
                    start,
                    (400.0, 0.0),
                    (460.0, 180.0),
                    (1340.0, 180.0),
                    (1400.0, 0.0),
                    end,
                ]
            } else {
                vec![start, end]
            };
            (
                format!("braid_{}", i),
                track(&wobble(&densify(&path), HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// The short-strand braid: every outing runs the same busy line, but
/// the first `divert` swap onto a ~90 m variant 120 m north — far below
/// section length, so the strand can only ever be a fragment. Its
/// ground is real minority evidence, but the sections around it belong
/// to the majority line.
pub fn short_strand(outings: usize, divert: usize) -> Vec<(String, Vec<GpsPoint>)> {
    (0..outings)
        .map(|i| {
            let start = (-200.0 - 8.0 * i as f64, 0.0);
            let end = (1400.0 + 8.0 * i as f64, 0.0);
            let path = if i < divert {
                vec![
                    start,
                    (500.0, 0.0),
                    (530.0, 120.0),
                    (590.0, 120.0),
                    (620.0, 0.0),
                    end,
                ]
            } else {
                vec![start, end]
            };
            (
                format!("strand_{}", i),
                track(&wobble(&densify(&path), HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// A bare straight corridor, one pass per outing, collinear staggered
/// starts and ends: no boundary mechanism anywhere, so what surfaces is
/// purely a question of support.
pub fn plain_corridor(outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    (0..outings)
        .map(|i| {
            let path = densify(&[
                (-100.0 - 8.0 * i as f64, 0.0),
                (900.0 + 8.0 * i as f64, 0.0),
            ]);
            (
                format!("cor_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// A corridor whose thirds belong to different riders, joined through
/// short overlaps so the traffic chain merges into one node: `through`
/// riders cover all 2400 m, and three locals cover a 900 m third each.
/// Every cell carries at least two riders and no third is minority
/// ground, so the render is a through pass over the whole corridor, and
/// no local covers half of it. The candidate qualifies on four
/// contributors and the drawn line admits only the `through` riders.
pub fn split_population_corridor(through: usize) -> Vec<(String, Vec<GpsPoint>)> {
    const LEGS: [(f64, f64); 3] = [(0.0, 900.0), (800.0, 1700.0), (1600.0, 2400.0)];
    let mut out: Vec<(String, Vec<GpsPoint>)> = (0..through)
        .map(|i| {
            let path = densify(&[(0.0, 0.0), (2400.0, 0.0)]);
            (
                format!("through_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect();
    for (i, &(x0, x1)) in LEGS.iter().enumerate() {
        let path = densify(&[(x0, 0.0), (x1, 0.0)]);
        out.push((
            format!("local_{}", i),
            track(&wobble(&path, HUMAN_WOBBLE_M, phase(through + i))),
        ));
    }
    out
}

/// A long corridor no one traverses end to end: outing i covers a
/// 1500 m window starting 110 m after outing i-1's, the way a valley
/// path collects travellers who each join and leave at their own
/// points. Local traffic is steady through the interior and changes by
/// single tracks along the way, so no join is ever a cliff and the
/// ground is one corridor — but the longest single pass covers barely
/// a third of it.
pub fn sliding_corridor(outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    (0..outings)
        .map(|i| {
            let x0 = 110.0 * i as f64;
            let path = densify(&[(x0, 0.0), (x0 + 1500.0, 0.0)]);
            (
                format!("slide_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// A busy corridor whose traffic scatters wide and incoherently: every
/// outing rides the full length, but each drifts laterally through its
/// own deterministic knot sequence (±55 m, knots every 150 m), the way
/// riders spread across a boulevard's parallel paths. Any one evidence
/// cell records only the subset that crossed it, and no two outings
/// share a lane long enough to fake a parallel corridor. The travelling
/// population is identical everywhere along the corridor; the per-cell
/// counts are not.
pub fn scattered_corridor(outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    let lateral = |i: usize, x: f64| -> f64 {
        let seg = (x / 150.0).floor();
        let t = x / 150.0 - seg;
        let knot = |k: f64| {
            ((i * 73 + (k as i64).rem_euclid(1_000_003) as usize * 37) % 97) as f64 / 96.0 * 110.0
                - 55.0
        };
        let a = knot(seg);
        let b = knot(seg + 1.0);
        a + (b - a) * t
    };
    (0..outings)
        .map(|i| {
            let x0 = -100.0 - 8.0 * i as f64;
            let x1 = 2900.0 + 8.0 * i as f64;
            let n = ((x1 - x0) / 10.0).ceil() as usize;
            let path: Vec<(f64, f64)> = (0..=n)
                .map(|s| {
                    let x = x0 + (x1 - x0) * s as f64 / n as f64;
                    (x, lateral(i, x))
                })
                .collect();
            (
                format!("scat_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// Two parallel streets 180 m apart spanning the same corridor, west
/// of them a much busier trunk ending where the busy street begins.
/// The trunk is accepted first and edge-trims the busy street's
/// candidate, so the busy street's default becomes its LONGEST pass —
/// and the longest passes are the swap outings, which cross onto the
/// quiet street for the middle stretch and run unbroken through the
/// busy street's capture ring. The shape of a displaced render
/// claiming ground it never draws.
pub fn parallel_street(busy: usize, quiet: usize, swap: usize) -> Vec<(String, Vec<GpsPoint>)> {
    let mut out = Vec::new();
    for i in 0..30 {
        let path = densify(&[(-900.0 - 8.0 * i as f64, 0.0), (250.0, 0.0)]);
        out.push((
            format!("trunk_{}", i),
            track(&wobble(&path, HUMAN_WOBBLE_M, phase(50 + i))),
        ));
    }
    for i in 0..busy {
        let path = densify(&[
            (-100.0 - 8.0 * i as f64, 0.0),
            (1700.0 + 8.0 * i as f64, 0.0),
        ]);
        out.push((
            format!("busy_{}", i),
            track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
        ));
    }
    for i in 0..quiet {
        let path = densify(&[
            (-80.0 - 8.0 * i as f64, 180.0),
            (1680.0 + 8.0 * i as f64, 180.0),
        ]);
        out.push((
            format!("quiet_{}", i),
            track(&wobble(&path, HUMAN_WOBBLE_M, phase(busy + i))),
        ));
    }
    for i in 0..swap {
        let path = densify(&[
            (-320.0 - 8.0 * i as f64, 0.0),
            (560.0, 0.0),
            (640.0, 180.0),
            (960.0, 180.0),
            (1040.0, 0.0),
            (1920.0 + 8.0 * i as f64, 0.0),
        ]);
        out.push((
            format!("swap_{}", i),
            track(&wobble(&path, HUMAN_WOBBLE_M, phase(busy + quiet + i))),
        ));
    }
    out
}

/// A shared trunk splitting into two worthy branches: half the outings
/// take each. Point-to-point, one pass everywhere, so the only boundary
/// mechanism available at the join is the fork.
pub fn fork_y(outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    (0..outings)
        .map(|i| {
            let branch_end = if i % 2 == 0 {
                (-600.0, 1800.0)
            } else {
                (600.0, 1800.0)
            };
            let path = densify(&[(0.0, 0.0), (0.0, 1000.0), branch_end]);
            (
                format!("fork_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// A trunk that only becomes a fork later in the library's life: the
/// first `early` outings run straight through the junction, then `late`
/// outings peel east at the midpoint. A batch over the whole set sees a
/// fork; a drip sees one through-line until the branch arrives, so the
/// trunk has to re-cut mid-life. This is the ingestion order that a
/// section-carrying incremental would get wrong.
pub fn late_fork(early: usize, late: usize) -> Vec<(String, Vec<GpsPoint>)> {
    let leg = |end: (f64, f64)| densify(&[(0.0, 0.0), (0.0, 1000.0), end]);
    let straight = leg((0.0, 2000.0));
    let branch = leg((600.0, 1800.0));
    (0..early + late)
        .map(|i| {
            let (name, path) = if i < early {
                (format!("early_{i}"), &straight)
            } else {
                (format!("late_{}", i - early), &branch)
            };
            (name, track(&wobble(path, HUMAN_WOBBLE_M, phase(i))))
        })
        .collect()
}

/// Zigzag climb: 8 hairpin legs 30 m apart in plan, 25 m of gain each,
/// so one coarse cell holds several legs at distinct elevation levels.
/// Return leg is a detached road well east of the climb.
pub fn switchback_climb(outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    let mut up: Vec<(f64, f64)> = vec![(0.0, 0.0)];
    for leg in 0..8 {
        let y = 30.0 * leg as f64;
        let x_far = if leg % 2 == 0 { 150.0 } else { 0.0 };
        up.push((x_far, y));
        up.push((x_far, y + 30.0));
    }
    let top = *up.last().unwrap();
    (0..outings)
        .map(|i| {
            let mut path = densify(&up);
            let n_ascent = path.len();
            path.extend(densify(&[
                top,
                (600.0, top.1),
                (600.0, -150.0),
                (0.0, -150.0),
                (0.0, 0.0),
            ]));
            let wob = wobble(&path, HUMAN_WOBBLE_M, phase(i));
            // 0 -> 200 m over the climb, back to 0 along the return road.
            let ascent = track_ele(&wob[..n_ascent], 0.0, 200.0);
            let descent = track_ele(&wob[n_ascent..], 200.0, 0.0);
            let mut pts = ascent;
            pts.extend(descent);
            (format!("swb_{}", i), pts)
        })
        .collect()
}

/// A 4x4 street grid of 250 m blocks. Eight staircase commutes from SW
/// to NE share the first and last blocks, a designed 3-visit two-block
/// run, and a designed 2-visit block that must stay below the floor.
pub fn grid_city() -> Vec<(String, Vec<GpsPoint>)> {
    let b = 250.0;
    // Waypoints in block units. All start (0,0), end (3,3).
    let routes: [&[(f64, f64)]; 8] = [
        &[(0.0, 0.0), (1.0, 0.0), (1.0, 2.0), (3.0, 2.0), (3.0, 3.0)],
        &[
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (2.0, 1.0),
            (2.0, 2.0),
            (3.0, 2.0),
            (3.0, 3.0),
        ],
        &[
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (2.0, 2.0),
            (3.0, 2.0),
            (3.0, 3.0),
        ],
        &[
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 2.0),
            (2.0, 2.0),
            (2.0, 3.0),
            (3.0, 3.0),
        ],
        &[(0.0, 0.0), (1.0, 0.0), (3.0, 0.0), (3.0, 3.0)],
        &[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (3.0, 1.0), (3.0, 3.0)],
        &[
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (2.0, 1.0),
            (3.0, 1.0),
            (3.0, 3.0),
        ],
        &[(0.0, 0.0), (1.0, 0.0), (1.0, 2.0), (1.0, 3.0), (3.0, 3.0)],
    ];
    routes
        .iter()
        .enumerate()
        .map(|(i, r)| {
            let wps: Vec<(f64, f64)> = r.iter().map(|&(x, y)| (x * b, y * b)).collect();
            (
                format!("grid_{}", i),
                track(&wobble(&densify(&wps), HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// Cable-smooth straight 25% ascent beside a winding human descent,
/// closed by a flat valley link. The lift line is only ever ascended.
pub fn lift_piste(outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    // Winding piste: heads south-east from the top with a lateral swing
    // wide enough that no 300 m extent reads straight.
    let mut piste: Vec<(f64, f64)> = Vec::new();
    let steps = 140;
    for s in 0..=steps {
        let t = s as f64 / steps as f64;
        let y = 900.0 * (1.0 - t);
        let x = 150.0 + 130.0 * (t * 5.5 * std::f64::consts::PI).sin();
        piste.push((x, y));
    }
    (0..outings)
        .map(|i| {
            let lift = densify(&[(0.0, 0.0), (0.0, 900.0)]);
            let lift_pts = track_ele(&wobble(&lift, 0.3, phase(i)), 0.0, 225.0);
            let piste_wob = wobble(&densify(&piste), 3.0, phase(i));
            let piste_pts = track_ele(&piste_wob, 225.0, 0.0);
            let link = densify(&[*piste.last().unwrap(), (0.0, 0.0)]);
            let link_pts = track_ele(&wobble(&link, HUMAN_WOBBLE_M, phase(i)), 0.0, 0.0);
            let mut pts = lift_pts;
            pts.extend(piste_pts);
            pts.extend(link_pts);
            (format!("snow_{}", i), pts)
        })
        .collect()
}

// ---------------------------------------------------------------- personas

/// Casual: a handful of one-off loops, no shared ground at all.
pub fn persona_casual() -> Vec<(String, Vec<GpsPoint>)> {
    use std::f64::consts::PI;
    (0..8)
        .map(|i| {
            let bearing = i as f64 * PI / 4.0;
            let r = 350.0 + 60.0 * i as f64;
            let (cx, cy) = (3000.0 * bearing.cos() + r, 3000.0 * bearing.sin());
            let path = arc(cx, cy, r, PI, PI + 2.0 * PI);
            (
                format!("cas_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// Weekender: one favourite circuit ridden six times plus four one-offs.
pub fn persona_weekender() -> Vec<(String, Vec<GpsPoint>)> {
    use std::f64::consts::PI;
    let mut out: Vec<(String, Vec<GpsPoint>)> = (0..6)
        .map(|i| {
            let mut path = densify(&[(0.0, 0.0), (400.0, 0.0)]);
            path.extend(arc(900.0, 0.0, 500.0, PI, PI + 2.0 * PI));
            path.extend(densify(&[(400.0, 0.0), (0.0, 0.0)]));
            (
                format!("wkd_loop_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect();
    for i in 0..4 {
        let x0 = -2500.0 - 700.0 * i as f64;
        let path = densify(&[(x0, 2000.0), (x0 + 300.0, 2600.0), (x0 - 200.0, 3100.0)]);
        out.push((
            format!("wkd_one_{}", i),
            track(&wobble(&path, HUMAN_WOBBLE_M, phase(i + 6))),
        ));
    }
    out
}

/// Commuter: the same two-corner road, six mornings out and six
/// evenings back, each direction its own activity.
pub fn persona_commuter() -> Vec<(String, Vec<GpsPoint>)> {
    let road = [(0.0, 0.0), (600.0, 0.0), (600.0, 500.0), (1300.0, 500.0)];
    (0..12)
        .map(|i| {
            let mut wps = road.to_vec();
            if i % 2 == 1 {
                wps.reverse();
            }
            (
                format!("com_{}", i),
                track(&wobble(&densify(&wps), HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// Racer: warm-up out, four hill repeats (eight passes), warm-down back.
pub fn persona_racer() -> Vec<(String, Vec<GpsPoint>)> {
    (0..6)
        .map(|i| {
            let mut path = densify(&[(0.0, 0.0), (500.0, 0.0)]);
            let n_flat = path.len();
            let mut hill_reps: Vec<(f64, f64)> = Vec::new();
            for _ in 0..4 {
                hill_reps.extend(densify(&[(500.0, 0.0), (900.0, 0.0)]));
                hill_reps.extend(densify(&[(900.0, 0.0), (500.0, 0.0)]));
            }
            path.extend(hill_reps);
            path.extend(densify(&[(500.0, 0.0), (0.0, 0.0)]));
            let wob = wobble(&path, HUMAN_WOBBLE_M, phase(i));
            // Flat approach; the hill rises 0 -> 40 m per rep leg.
            let pts: Vec<GpsPoint> = wob
                .iter()
                .enumerate()
                .map(|(k, &(x, y))| {
                    if k < n_flat || x < 500.0 {
                        to_gps_ele(x, y, 0.0)
                    } else {
                        to_gps_ele(x, y, (x - 500.0) / 400.0 * 40.0)
                    }
                })
                .collect();
            (format!("rac_{}", i), pts)
        })
        .collect()
}

/// A busy west-east corridor where one outing takes a wrong turn: it
/// alone rides a 900 m northward tail to its end. Other riders peel off
/// over the tail's first 450 m, so cell traffic thins one track at a
/// time (the gradient that welds the tail onto the corridor), and three
/// staggered 120 m strands keep the deep tail's cells hot without
/// lending it a single portion-length traversal.
pub fn welded_tail() -> Vec<(String, Vec<GpsPoint>)> {
    let mut out: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    for (i, up) in [0.0, 150.0, 300.0, 450.0, 900.0].into_iter().enumerate() {
        let mut wps = vec![(0.0, 0.0), (1500.0, 0.0)];
        if up > 0.0 {
            wps.push((1500.0, up));
        }
        out.push((
            format!("cor_{}", i),
            track(&wobble(&densify(&wps), HUMAN_WOBBLE_M, phase(i))),
        ));
    }
    for (i, (y0, y1)) in [(450.0, 570.0), (550.0, 670.0), (650.0, 770.0)]
        .into_iter()
        .enumerate()
    {
        out.push((
            format!("str_{}", i),
            track(&densify(&[(1500.0, y0), (1500.0, y1)])),
        ));
    }
    out
}

/// A corridor where every outing folds mid-line: forward, back over
/// its own ground, forward again (interval reps). Rep windows shift
/// 40 m per outing so no coherent pass-class boundary forms and the
/// fold stays inside one candidate's portions — every pass is legal
/// but none is clean.
pub fn interval_reps(outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    (0..outings)
        .map(|i| {
            let s = 250.0 + 40.0 * i as f64;
            let e = s + 90.0;
            let path = densify(&[
                (-8.0 * i as f64, 0.0),
                (e, 0.0),
                (s, 0.0),
                (800.0 + 8.0 * i as f64, 0.0),
            ]);
            (
                format!("reps_{}", i),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// A through corridor whose middle is shredded into sub-length nodes:
/// heavy bulks turn off at both ends (cliff + fork) and a crossing
/// path inflates one middle cell's track set, so the strict
/// same-traffic partition cuts the through traffic's ground at the
/// junction while the flanks' accepted lines mask the shreds' probes.
pub fn shredded_corridor() -> Vec<(String, Vec<GpsPoint>)> {
    let mut out = Vec::new();
    for i in 0..12 {
        let path = densify(&[(-8.0 * i as f64, 0.0), (1480.0 + 8.0 * i as f64, 0.0)]);
        out.push((
            format!("through_{}", i),
            track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
        ));
    }
    for i in 0..30 {
        let path = densify(&[
            (-8.0 * i as f64, 0.0),
            (480.0, 0.0),
            (480.0, 600.0 + 8.0 * i as f64),
        ]);
        out.push((
            format!("abulk_{}", i),
            track(&wobble(&path, HUMAN_WOBBLE_M, phase(12 + i))),
        ));
    }
    for i in 0..30 {
        let path = densify(&[
            (980.0, 600.0 + 8.0 * i as f64),
            (980.0, 0.0),
            (1480.0 + 8.0 * i as f64, 0.0),
        ]);
        out.push((
            format!("bbulk_{}", i),
            track(&wobble(&path, HUMAN_WOBBLE_M, phase(42 + i))),
        ));
    }
    for i in 0..16 {
        let x = 700.0 + 4.0 * i as f64;
        let path = densify(&[(x, -60.0), (x, 60.0)]);
        out.push((
            format!("cross_{}", i),
            track(&wobble(&path, HUMAN_WOBBLE_M, phase(72 + i))),
        ));
    }
    out
}

/// Interval sessions on a small oval: stem in, ten laps, stem home.
/// The oval spans only a couple of evidence cells, so the mouth cell
/// blends lap and stem ground and the usage boundary cannot separate
/// them; each outing's simple pass is stem plus first lap.
pub fn small_oval_stem(outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    (0..outings)
        .map(|i| {
            let mut path: Vec<(f64, f64)> = Vec::new();
            path.extend(densify(&[(-8.0 * i as f64, 0.0), (150.0, 0.0)]));
            for _ in 0..10 {
                path.extend(arc(
                    220.0,
                    0.0,
                    70.0,
                    std::f64::consts::PI,
                    -std::f64::consts::PI,
                ));
            }
            path.extend(densify(&[(150.0, 0.0), (-8.0 * i as f64, 0.0)]));
            (
                format!("oval_{i}"),
                track(&wobble(&path, HUMAN_WOBBLE_M, phase(i))),
            )
        })
        .collect()
}

/// Five clean interval sessions on the small oval plus one deviant
/// outing whose laps cut a chord across the east side and whose longer
/// stem makes its pass the longest. The deviant's revolution strays up
/// to ~45 m from the circuit the other five lap faithfully.
pub fn small_oval_stem_deviant() -> Vec<(String, Vec<GpsPoint>)> {
    use std::f64::consts::PI;
    let mut out = small_oval_stem(5);
    let mut path: Vec<(f64, f64)> = Vec::new();
    path.extend(densify(&[(-120.0, 0.0), (150.0, 0.0)]));
    // First lap weaves ±45 m around the circuit with short excursions,
    // so every beyond-tolerance run stays under the sustained-minority
    // bar while the lap as a whole strays far from everyone else's.
    let steps = 120;
    for k in 0..=steps {
        let a = PI - 2.0 * PI * k as f64 / steps as f64;
        let r = 70.0 + 45.0 * (6.0 * a).sin();
        path.push((220.0 + r * a.cos(), r * a.sin()));
    }
    for _ in 0..9 {
        path.extend(arc(220.0, 0.0, 70.0, PI, -PI));
    }
    path.extend(densify(&[(150.0, 0.0), (-120.0, 0.0)]));
    out.push((
        "oval_dev".to_string(),
        track(&wobble(&path, HUMAN_WOBBLE_M, phase(9))),
    ));
    // A busy crossing street near the stem: accepted first, it masks
    // the oval candidate's west end, so the node is TRIMMED and the
    // default falls to the longest pass — the deviant — exactly the
    // structure that let the outlier lap win on the real oval.
    for i in 0..20 {
        let road = densify(&[
            (-60.0, -400.0 - 8.0 * i as f64),
            (-60.0, 400.0 + 8.0 * i as f64),
        ]);
        out.push((
            format!("road_{i}"),
            track(&wobble(&road, HUMAN_WOBBLE_M, phase(20 + i))),
        ));
    }
    out
}

/// A plain corridor walked by `outings` riders, plus one who traverses
/// it `laps` times in a single outing, turning at each end. Every pass
/// covers the same ground; only the pass count separates the lapper.
pub fn corridor_with_a_lapper(outings: usize, laps: usize) -> Vec<(String, Vec<GpsPoint>)> {
    let mut out = plain_corridor(outings);
    let mut path: Vec<(f64, f64)> = Vec::new();
    for lap in 0..laps {
        let leg = densify(&[(-100.0, 0.0), (900.0, 0.0)]);
        if lap % 2 == 1 {
            path.extend(leg.into_iter().rev());
        } else {
            path.extend(leg);
        }
    }
    out.push((
        "lapper".to_string(),
        track(&wobble(&path, HUMAN_WOBBLE_M, phase(outings))),
    ));
    out
}

/// One athlete's interval session: a stem in, `reps` there-and-back
/// repeats over a 300 m stretch, a stem home. No one else uses the
/// stretch, so the only thing standing behind it is one outing.
pub fn lone_interval_session(reps: usize) -> Vec<(String, Vec<GpsPoint>)> {
    let mut path: Vec<(f64, f64)> = densify(&[(-600.0, 0.0), (0.0, 0.0)]);
    for _ in 0..reps {
        path.extend(densify(&[(0.0, 0.0), (300.0, 0.0)]));
        path.extend(densify(&[(300.0, 0.0), (0.0, 0.0)]));
    }
    path.extend(densify(&[(0.0, 0.0), (-600.0, 0.0)]));
    let mut out = vec![(
        "session".to_string(),
        track(&wobble(&path, HUMAN_WOBBLE_M, phase(0))),
    )];
    // Unrelated background traffic a long way off, so detection has a
    // pool to work in and the session is not the only thing in it.
    for i in 0..10 {
        let road = densify(&[
            (-2_000.0, 3_000.0 + 8.0 * i as f64),
            (-1_000.0, 3_000.0 + 8.0 * i as f64),
        ]);
        out.push((
            format!("road_{i}"),
            track(&wobble(&road, HUMAN_WOBBLE_M, phase(20 + i))),
        ));
    }
    out
}

/// Six sessions lapping the small oval, one of which breaks off the
/// circuit between laps to fetch water and rejoins where it left. The
/// excursion is one pass over ground nobody else touches, taken at the
/// seam where laps join.
pub fn small_oval_seam_excursion() -> Vec<(String, Vec<GpsPoint>)> {
    use std::f64::consts::PI;
    let mut out = small_oval_stem(5);
    let mut path: Vec<(f64, f64)> = Vec::new();
    path.extend(densify(&[(-40.0, 0.0), (150.0, 0.0)]));
    for lap in 0..10 {
        path.extend(arc(220.0, 0.0, 70.0, PI, -PI));
        if lap == 4 {
            // Off the circuit at the seam and straight back again.
            path.extend(densify(&[(150.0, 0.0), (150.0, 260.0)]));
            path.extend(densify(&[(150.0, 260.0), (150.0, 0.0)]));
        }
    }
    path.extend(densify(&[(150.0, 0.0), (-40.0, 0.0)]));
    out.push((
        "oval_seam".to_string(),
        track(&wobble(&path, HUMAN_WOBBLE_M, phase(11))),
    ));
    out
}
