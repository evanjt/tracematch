//! Property-based invariants for the pure geometry and grouping primitives.
//! These assert the laws the algorithms must obey for any input, complementing
//! the example-based integration tests.

use proptest::prelude::*;
use tracematch::GpsPoint;
use tracematch::geo_utils::haversine_distance;
use tracematch::union_find::UnionFind;

fn pt(lat: f64, lon: f64) -> GpsPoint {
    GpsPoint::new(lat, lon)
}

proptest! {
    #[test]
    fn haversine_is_symmetric_and_non_negative(
        lat1 in -90.0f64..90.0,
        lon1 in -180.0f64..180.0,
        lat2 in -90.0f64..90.0,
        lon2 in -180.0f64..180.0,
    ) {
        let a = pt(lat1, lon1);
        let b = pt(lat2, lon2);
        let dab = haversine_distance(&a, &b);
        let dba = haversine_distance(&b, &a);
        prop_assert!(dab.is_finite());
        prop_assert!(dab >= 0.0);
        prop_assert!((dab - dba).abs() < 1e-6);
    }

    #[test]
    fn haversine_is_zero_for_identical_points(lat in -90.0f64..90.0, lon in -180.0f64..180.0) {
        let a = pt(lat, lon);
        prop_assert!(haversine_distance(&a, &a).abs() < 1e-6);
    }

    #[test]
    fn union_find_grouping_is_deterministic(
        edges in prop::collection::vec((0u32..20, 0u32..20), 0..40)
    ) {
        let build = || {
            let mut uf: UnionFind<u32> = UnionFind::new();
            for i in 0..20u32 {
                uf.make_set(i);
            }
            for (a, b) in &edges {
                uf.union(a, b);
            }
            uf.groups()
        };
        prop_assert_eq!(build(), build());
    }

    #[test]
    fn union_makes_elements_connected(a in 0u32..50, b in 0u32..50) {
        let mut uf: UnionFind<u32> = UnionFind::new();
        uf.union(&a, &b);
        prop_assert!(uf.connected(&a, &b));
    }
}

proptest! {
    /// Resampling must never emit NaN coordinates, even when the track
    /// contains exact duplicate points (zero-length segments).
    #[test]
    fn resample_route_finite_with_duplicate_points(
        n_points in 4usize..40,
        dup_idx in 0usize..38,
        target in 3usize..20,
    ) {
        let mut points: Vec<GpsPoint> = (0..n_points)
            .map(|i| pt(47.0 + i as f64 * 0.001, 7.0 + i as f64 * 0.001))
            .collect();
        let dup_at = dup_idx % (n_points - 1);
        let dup = points[dup_at];
        points.insert(dup_at + 1, dup);

        let resampled = tracematch::matching::resample_route(&points, target);
        for p in &resampled {
            prop_assert!(p.latitude.is_finite());
            prop_assert!(p.longitude.is_finite());
        }
    }
}
