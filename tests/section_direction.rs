//! Scenario: a section's rows carry a direction the fold wrote, and the same
//! ground read afresh through `PreparedLine` gives a direction of its own.
//!
//! Expected behaviour: for every row, the two agree.

use std::collections::HashMap;
use tracematch::{Direction, GpsPoint, PreparedLine, SectionConfig, detect_sections};

/// A ride up a straight street, a turn at its end and the ride back down,
/// sampled every `step_m` metres.
fn out_and_back(len_m: f64, step_m: f64, offset_m: f64) -> Vec<GpsPoint> {
    let deg_per_m = 1.0 / 111_132.0;
    let n = (len_m / step_m) as usize;
    let at = |i: usize| GpsPoint {
        latitude: 46.5 + (i as f64 * step_m + offset_m) * deg_per_m,
        longitude: 6.6,
        elevation: None,
    };
    let up: Vec<GpsPoint> = (0..=n).map(at).collect();
    let down: Vec<GpsPoint> = (0..n).rev().map(at).collect();
    up.into_iter().chain(down).collect()
}

/// An interval session: `reps` rides up the street and back, each turn
/// carrying on `overshoot_m` past the end of the street before it returns.
fn intervals(len_m: f64, step_m: f64, reps: usize, overshoot_m: f64) -> Vec<GpsPoint> {
    let deg_per_m = 1.0 / 111_132.0;
    let at = |m: f64| GpsPoint {
        latitude: 46.5 + m * deg_per_m,
        longitude: 6.6,
        elevation: None,
    };
    let top = len_m + overshoot_m;
    let n = (top / step_m) as usize;
    let mut out = Vec::new();
    for _ in 0..reps {
        out.extend((0..=n).map(|i| at(i as f64 * step_m)));
        out.extend((0..n).rev().map(|i| at(i as f64 * step_m)));
    }
    out
}

/// A ride along a street that doubles back on itself: `arm_m` north, across
/// `gap_m` east, then `arm_m` south, sampled every `step_m` metres.
fn hairpin(arm_m: f64, gap_m: f64, step_m: f64, reversed: bool) -> Vec<GpsPoint> {
    let m_lat = 1.0 / 111_132.0;
    let m_lng = 1.0 / (111_320.0 * 46.5_f64.to_radians().cos());
    let mut path: Vec<(f64, f64)> = Vec::new();
    let mut y = 0.0;
    while y < arm_m {
        path.push((0.0, y));
        y += step_m;
    }
    let mut x = 0.0;
    while x < gap_m {
        path.push((x, arm_m));
        x += step_m;
    }
    let mut y = arm_m;
    while y >= 0.0 {
        path.push((gap_m, y));
        y -= step_m;
    }
    if reversed {
        path.reverse();
    }
    path.into_iter()
        .map(|(x, y)| GpsPoint {
            latitude: 46.5 + y * m_lat,
            longitude: 6.6 + x * m_lng,
            elevation: None,
        })
        .collect()
}

#[test]
fn hairpin_rows_agree() {
    check(
        (0..6)
            .map(|k| (format!("ride-{k}"), hairpin(150.0, 35.0, 8.0, k % 2 == 1)))
            .collect(),
    );
}

#[test]
fn out_and_back_rows_agree() {
    check(tracks_out_and_back());
}

fn tracks_out_and_back() -> Vec<(String, Vec<GpsPoint>)> {
    (0..5)
        .map(|k| {
            let t = if k == 0 {
                intervals(400.0, 30.0, 5, 40.0)
            } else {
                out_and_back(400.0, 30.0, k as f64 * 3.0)
            };
            (format!("ride-{k}"), t)
        })
        .collect()
}

fn check(tracks: Vec<(String, Vec<GpsPoint>)>) {
    let seconds: Vec<Vec<f64>> = tracks
        .iter()
        .map(|(_, t)| (0..t.len()).map(|i| i as f64).collect())
        .collect();
    let seconds_ref: Vec<&[f64]> = seconds.iter().map(|s| s.as_slice()).collect();
    let config = SectionConfig::default();
    let sections = detect_sections(&tracks, &seconds_ref, &HashMap::new(), &config);
    assert!(!sections.is_empty(), "the street is a section");

    let by_id: HashMap<&str, &[GpsPoint]> = tracks
        .iter()
        .map(|(id, t)| (id.as_str(), t.as_slice()))
        .collect();
    let mut checked = 0;
    let mut reverse_rows = 0;
    for section in &sections {
        let line = PreparedLine::new(&section.polyline, &config).expect("a line to match");
        for row in &section.activity_portions {
            let track = by_id[row.activity_id.as_str()];
            let fresh = line.portions(&row.activity_id, track);
            let same_leg = fresh
                .iter()
                .find(|p| p.start_index < row.end_index && row.start_index < p.end_index)
                .unwrap_or_else(|| panic!("no fresh pass over row {row:?}"));
            assert_eq!(
                row.direction,
                same_leg.direction,
                "row {}..{} of {} against fresh pass {}..{}",
                row.start_index,
                row.end_index,
                row.activity_id,
                same_leg.start_index,
                same_leg.end_index
            );
            checked += 1;
            if row.direction == Direction::Reverse {
                reverse_rows += 1;
            }
        }
    }
    assert!(
        checked >= 6,
        "every ride is at least one row, got {checked}"
    );
    assert!(reverse_rows < checked);
}
