use super::*;

#[test]
fn test_build_rtree_equal_metres_at_different_latitudes() {
    for latitude in [0.0_f64, 46.0, 60.0] {
        let index = build_rtree(&[GpsPoint::new(latitude, 0.0)]);
        let north = [latitude + 50.0 / 111_000.0, 0.0];
        let east = [latitude, 50.0 / (111_000.0 * latitude.to_radians().cos())];
        for query in [north, east] {
            let distance = index.nearest(&query).unwrap().1;
            assert!(distance <= 60.0, "latitude {latitude}, point {query:?}");
        }
    }
}

#[test]
fn test_find_all_track_portions_accepts_equal_offsets_at_latitude() {
    for latitude in [0.0_f64, 46.0, 60.0] {
        let reference: Vec<GpsPoint> = (0..8)
            .map(|i| GpsPoint::new(latitude + i as f64 * 50.0 / 111_000.0, 0.0))
            .collect();
        let east_offset = 50.0 / (111_000.0 * latitude.to_radians().cos());
        let east: Vec<GpsPoint> = reference
            .iter()
            .map(|point| GpsPoint::new(point.latitude, east_offset))
            .collect();
        let portions =
            crate::sections::find_all_track_portions_with_gap(&east, &reference, 60.0, 0);
        assert_eq!(portions.len(), 1, "latitude {latitude}");
    }
}
