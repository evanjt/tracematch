use super::*;

const LATITUDES: [f64; 3] = [0.0, 46.0, 60.0];

fn north_deg(metres: f64) -> f64 {
    metres / 111_000.0
}

fn east_deg(latitude: f64, metres: f64) -> f64 {
    metres / (111_000.0 * latitude.to_radians().cos())
}

/// A kilometre north from `latitude` in 10 m steps.
fn northward(latitude: f64) -> Vec<GpsPoint> {
    (0..=100)
        .map(|i| GpsPoint::new(latitude + north_deg(i as f64 * 10.0), 7.0))
        .collect()
}

/// A kilometre east along `latitude` in 10 m steps.
fn eastward(latitude: f64) -> Vec<GpsPoint> {
    (0..=100)
        .map(|i| GpsPoint::new(latitude, 7.0 + east_deg(latitude, i as f64 * 10.0)))
        .collect()
}

/// Each line paired with a copy of it moved `offset_m` across it: the
/// northward line moved east, the eastward line moved north.
fn offset_pairs(latitude: f64, offset_m: f64) -> [(&'static str, Vec<GpsPoint>, Vec<GpsPoint>); 2] {
    let north = northward(latitude);
    let east_of_north = north
        .iter()
        .map(|p| GpsPoint::new(p.latitude, p.longitude + east_deg(latitude, offset_m)))
        .collect();
    let east = eastward(latitude);
    let north_of_east = east
        .iter()
        .map(|p| GpsPoint::new(p.latitude + north_deg(offset_m), p.longitude))
        .collect();
    [
        ("east", north, east_of_north),
        ("north", east, north_of_east),
    ]
}

#[test]
fn test_build_rtree_equal_metres_at_different_latitudes() {
    for latitude in LATITUDES {
        let index = build_rtree(&[GpsPoint::new(latitude, 0.0)]);
        let north = [latitude + north_deg(50.0), 0.0];
        let east = [latitude, east_deg(latitude, 50.0)];
        for query in [north, east] {
            let distance = index.nearest(&query).unwrap().1;
            assert!(
                (distance - 50.0).abs() < 0.01,
                "latitude {latitude}, point {query:?}: {distance} m"
            );
        }
    }
}

#[test]
fn test_find_all_track_portions_gives_one_verdict_east_and_north() {
    for latitude in LATITUDES {
        for (offset_m, near) in [(55.0, true), (65.0, false)] {
            for (side, reference, track) in offset_pairs(latitude, offset_m) {
                let portions =
                    crate::sections::find_all_track_portions_with_gap(&track, &reference, 60.0, 0);
                assert_eq!(
                    portions.len(),
                    usize::from(near),
                    "latitude {latitude}, {offset_m} m {side}"
                );
            }
        }
    }
}

#[test]
fn test_trace_extraction_gives_one_verdict_east_and_north() {
    for latitude in LATITUDES {
        for (offset_m, near) in [(55.0, true), (65.0, false)] {
            for (side, section, track) in offset_pairs(latitude, offset_m) {
                let tree = build_rtree(&section);
                let passes = crate::sections::extract_activity_passes(&track, &section, &tree);
                assert_eq!(
                    passes.len(),
                    usize::from(near),
                    "latitude {latitude}, {offset_m} m {side}"
                );
            }
        }
    }
}

#[test]
fn test_consensus_observes_and_measures_in_metres_east_and_north() {
    for latitude in LATITUDES {
        for (offset_m, observed) in [(30.0, true), (150.0, true), (199.0, true), (210.0, false)] {
            for (side, reference, trace) in offset_pairs(latitude, offset_m) {
                let result = crate::sections::consensus::compute_consensus_polyline(
                    &reference,
                    &[trace],
                    200.0,
                );
                let context = format!("latitude {latitude}, {offset_m} m {side}");
                let expected_density = u32::from(observed);
                assert!(
                    result.point_density.iter().all(|&d| d == expected_density),
                    "{context}: density {:?}",
                    result.point_density
                );
                if observed {
                    assert!(
                        (result.average_spread - offset_m).abs() < 0.5,
                        "{context}: spread {}",
                        result.average_spread
                    );
                }
            }
        }
    }
}
