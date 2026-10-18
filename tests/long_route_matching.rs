//! Two recordings of one long road must group, however many metres the resample spacing grows to.

use tracematch::grouping::group_signatures;
use tracematch::{GpsPoint, MatchConfig, RouteSignature};

const METRES_PER_DEGREE_LAT: f64 = 111_195.0;

/// A straight north-south road sampled every 25 m, starting `offset_m` along it.
fn straight_road(length_m: f64, offset_m: f64) -> Vec<GpsPoint> {
    let count = (length_m / 25.0) as usize;
    (0..=count)
        .map(|i| {
            let along = offset_m + i as f64 * 25.0;
            GpsPoint::new(46.0 + along / METRES_PER_DEGREE_LAT, 7.0)
        })
        .collect()
}

fn grouped(length_m: f64) -> bool {
    let config = MatchConfig::default();
    let first = straight_road(length_m, 0.0);
    // Second ride starts 5 m further along and stops 200 m short, inside the
    // endpoint tolerance, so the two resampled lines drift through every
    // phase against each other.
    let second = straight_road(length_m - 205.0, 5.0);
    let sigs = vec![
        RouteSignature::from_points("a", &first, &config).unwrap(),
        RouteSignature::from_points("b", &second, &config).unwrap(),
    ];
    let groups = group_signatures(&sigs, &config);
    groups.len() == 1 && groups[0].activity_ids.len() == 2
}

#[test]
fn identical_rides_group_at_30_km() {
    assert!(grouped(30_000.0));
}

#[test]
fn identical_rides_group_at_60_km() {
    assert!(grouped(60_000.0));
}

#[test]
fn identical_rides_group_at_100_km() {
    assert!(grouped(100_000.0));
}

#[test]
fn parallel_roads_a_block_apart_do_not_group_at_60_km() {
    let config = MatchConfig::default();
    let first = straight_road(60_000.0, 0.0);
    let second: Vec<GpsPoint> = first
        .iter()
        .map(|p| {
            GpsPoint::new(
                p.latitude,
                p.longitude + 150.0 / (METRES_PER_DEGREE_LAT * 0.6947),
            )
        })
        .collect();
    let sigs = vec![
        RouteSignature::from_points("a", &first, &config).unwrap(),
        RouteSignature::from_points("b", &second, &config).unwrap(),
    ];
    assert_eq!(group_signatures(&sigs, &config).len(), 2);
}
