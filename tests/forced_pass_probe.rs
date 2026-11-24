//! Probe: how the forced pass matcher counts a retraced line and where
//! its rows end when the line sits inside a longer out-back-out ride.

use tracematch::sections::GROUND_TOL_M;
use tracematch::{GpsPoint, PreparedLine, SectionConfig, SectionPortion};

const FORCED_PROXIMITY_FACTOR: f64 = 2.5;

fn northbound(points: usize, spacing_m: f64, east_m: f64, start: f64) -> Vec<GpsPoint> {
    let lat0 = 46.5_f64;
    let lng0 = 6.6_f64;
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lng = m_per_deg_lat * lat0.to_radians().cos();
    (0..points)
        .map(|i| {
            GpsPoint::new(
                lat0 + (start + i as f64) * spacing_m / m_per_deg_lat,
                lng0 + east_m / m_per_deg_lng,
            )
        })
        .collect()
}

fn forced_rows(line: &[GpsPoint], ride: &[GpsPoint]) -> Vec<SectionPortion> {
    let config = SectionConfig::default();
    let lateral_m = config.proximity_threshold * FORCED_PROXIMITY_FACTOR - GROUND_TOL_M;
    PreparedLine::with_lateral_tolerance(line, &config, lateral_m)
        .expect("a line of two points or more")
        .portions("ride", ride)
}

fn strict_rows(line: &[GpsPoint], ride: &[GpsPoint]) -> Vec<SectionPortion> {
    PreparedLine::new(line, &SectionConfig::default())
        .expect("a line of two points or more")
        .portions("ride", ride)
}

fn spans(rows: &[SectionPortion]) -> Vec<(usize, usize)> {
    rows.iter()
        .map(|r| (r.start_index as usize, r.end_index as usize))
        .collect()
}

fn out_back_out(points: usize, spacing: f64, back_east_m: f64) -> Vec<GpsPoint> {
    let out = northbound(points, spacing, 0.0, 0.0);
    let back: Vec<GpsPoint> = northbound(points, spacing, back_east_m, 0.0)
        .into_iter()
        .rev()
        .collect();
    let mut ride = out.clone();
    ride.extend(back);
    ride.extend(out);
    ride
}

#[test]
fn retraced_line_at_the_ride_start() {
    let spacing = 11.1_f64;
    let points = (150.0 / spacing).round() as usize + 1;
    let line = northbound(points, spacing, 0.0, 0.0);
    for east in [0.0, 2.0, 5.0, 10.0] {
        let ride = out_back_out(points, spacing, east);
        let f = forced_rows(&line, &ride);
        let s = strict_rows(&line, &ride);
        println!(
            "retrace east={east} m: forced {} rows {:?}; strict {} rows {:?}",
            f.len(),
            spans(&f),
            s.len(),
            spans(&s)
        );
    }
}

#[test]
fn line_inside_a_longer_out_back_out() {
    let spacing = 11.1_f64;
    let points = 115;
    let line = northbound(16, spacing, 0.0, 50.0);
    for east in [0.0, 10.0] {
        let ride = out_back_out(points, spacing, east);
        assert_eq!(ride.len(), 345);
        let f = forced_rows(&line, &ride);
        let s = strict_rows(&line, &ride);
        println!(
            "mid-line east={east} m: forced {} rows {:?}; strict {} rows {:?}",
            f.len(),
            spans(&f),
            s.len(),
            spans(&s)
        );
    }
}

#[test]
fn line_inside_a_longer_out_and_back_only() {
    let spacing = 11.1_f64;
    let points = 115;
    let line = northbound(16, spacing, 0.0, 50.0);
    let out = northbound(points, spacing, 0.0, 0.0);
    let back: Vec<GpsPoint> = out.iter().rev().copied().collect();
    let mut ride = out;
    ride.extend(back);
    let f = forced_rows(&line, &ride);
    let s = strict_rows(&line, &ride);
    println!(
        "out-back only: forced {} rows {:?}; strict {} rows {:?}",
        f.len(),
        spans(&f),
        s.len(),
        spans(&s)
    );
}

#[test]
fn retrace_with_third_leg_offset() {
    let spacing = 11.1_f64;
    let points = (150.0 / spacing).round() as usize + 1;
    let line = northbound(points, spacing, 0.0, 0.0);
    for (back_east, third_east) in [(0.0, 10.0), (0.0, 30.0), (5.0, 10.0), (10.0, 0.0)] {
        let out = northbound(points, spacing, 0.0, 0.0);
        let back: Vec<GpsPoint> = northbound(points, spacing, back_east, 0.0)
            .into_iter()
            .rev()
            .collect();
        let third = northbound(points, spacing, third_east, 0.0);
        let mut ride = out;
        ride.extend(back);
        ride.extend(third);
        let f = forced_rows(&line, &ride);
        let s = strict_rows(&line, &ride);
        println!(
            "retrace back={back_east} third={third_east}: forced {} rows {:?}; strict {} rows {:?}",
            f.len(),
            spans(&f),
            s.len(),
            spans(&s)
        );
    }
}

#[test]
fn line_inside_out_back_with_nearer_turn() {
    let spacing = 11.1_f64;
    let line = northbound(16, spacing, 0.0, 50.0);
    for points in [80usize, 100, 110, 115, 130] {
        let out = northbound(points, spacing, 0.0, 0.0);
        let back: Vec<GpsPoint> = out.iter().rev().copied().collect();
        let mut ride = out;
        ride.extend(back);
        let f = forced_rows(&line, &ride);
        println!(
            "out-back turn {} m past line end: forced {} rows {:?}",
            (points as f64 - 66.0) * spacing,
            f.len(),
            spans(&f)
        );
    }
}

#[test]
fn trace_third_leg_30m() {
    let spacing = 11.1_f64;
    let points = (150.0 / spacing).round() as usize + 1;
    let line = northbound(points, spacing, 0.0, 0.0);
    let out = northbound(points, spacing, 0.0, 0.0);
    let back: Vec<GpsPoint> = northbound(points, spacing, 0.0, 0.0)
        .into_iter()
        .rev()
        .collect();
    let third = northbound(points, spacing, 30.0, 0.0);
    let mut ride = out;
    ride.extend(back);
    ride.extend(third);
    let s = strict_rows(&line, &ride);
    println!("strict third=30: {:?}", spans(&s));
}

#[test]
fn turn_band() {
    let spacing = 11.1_f64;
    let line = northbound(16, spacing, 0.0, 50.0);
    for points in 108..=125usize {
        let out = northbound(points, spacing, 0.0, 0.0);
        let back: Vec<GpsPoint> = out.iter().rev().copied().collect();
        let mut ride = out;
        ride.extend(back);
        let f = forced_rows(&line, &ride);
        println!(
            "turn {:.0} m past end: {:?}",
            (points as f64 - 66.0) * spacing,
            spans(&f)
        );
    }
}
