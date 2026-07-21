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
