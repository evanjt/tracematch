//! FFI integration tests extracted from src/ffi.rs.
//!
//! These tests exercise the public FFI API exposed to mobile platforms.

use tracematch::{
    ffi::{
        create_signature, create_signature_with_config, create_signatures_from_flat,
        ffi_compare_routes, ffi_group_incremental, ffi_group_signatures, process_routes_from_flat,
        FlatGpsTrack,
    },
    GpsPoint, MatchConfig, RouteSignature, SectionConfig,
};

// ============================================================================
// Test Helpers
// ============================================================================

/// Generate a realistic GPS route with optional noise
fn generate_route(
    start_lat: f64,
    start_lng: f64,
    distance_km: f64,
    points_per_km: usize,
    noise_meters: f64,
) -> Vec<GpsPoint> {
    let total_points = (distance_km * points_per_km as f64) as usize;
    let bearing = 45.0_f64.to_radians(); // Northeast direction

    (0..total_points)
        .map(|i| {
            let progress = i as f64 / total_points.max(1) as f64;
            let distance_m = progress * distance_km * 1000.0;

            // Move along bearing
            let lat = start_lat + (distance_m / 111_000.0) * bearing.cos();
            let lng = start_lng
                + (distance_m / (111_000.0 * start_lat.to_radians().cos())) * bearing.sin();

            // Add deterministic "noise" based on index
            let noise_deg = noise_meters / 111_000.0;
            let noise_lat = ((i as f64 * 0.7).sin()) * noise_deg;
            let noise_lng = ((i as f64 * 1.3).cos()) * noise_deg;

            GpsPoint::new(lat + noise_lat, lng + noise_lng)
        })
        .collect()
}

/// Generate a route meeting minimum distance requirement (~1km)
fn long_route() -> Vec<GpsPoint> {
    generate_route(51.5074, -0.1278, 1.0, 100, 0.0)
}

/// Generate a route with GPS noise (~10m variance)
fn noisy_route() -> Vec<GpsPoint> {
    generate_route(51.5074, -0.1278, 1.0, 100, 10.0)
}

/// Generate a completely different route (NYC instead of London)
fn different_route() -> Vec<GpsPoint> {
    generate_route(40.7128, -74.0060, 1.0, 100, 0.0)
}

// ============================================================================
// Signature Creation Tests
// ============================================================================

#[test]
fn test_create_signature_valid_route() {
    let points = long_route();
    let sig = create_signature("test-1".to_string(), points);

    assert!(sig.is_some(), "Should create signature from valid route");
    let sig = sig.unwrap();
    assert_eq!(sig.activity_id, "test-1");
    assert!(
        sig.total_distance > 800.0,
        "~1km route should have distance > 800m"
    );
    assert!(
        !sig.points.is_empty(),
        "Signature should have simplified points"
    );
}

#[test]
fn test_create_signature_empty_points() {
    let sig = create_signature("test-1".to_string(), vec![]);
    assert!(sig.is_none(), "Empty points should return None");
}

#[test]
fn test_create_signature_single_point() {
    let points = vec![GpsPoint::new(51.5074, -0.1278)];
    let sig = create_signature("test-1".to_string(), points);
    assert!(sig.is_none(), "Single point should return None");
}

#[test]
fn test_create_signature_two_points() {
    // Two points that are far apart (should work)
    let points = vec![
        GpsPoint::new(51.5074, -0.1278),
        GpsPoint::new(51.5174, -0.1278), // ~1km north
    ];
    let sig = create_signature("test-1".to_string(), points);
    assert!(sig.is_some(), "Two valid points should create signature");
}

#[test]
fn test_create_signature_duplicate_points() {
    // All points at same location (degenerate case)
    let points: Vec<GpsPoint> = (0..100).map(|_| GpsPoint::new(51.5074, -0.1278)).collect();
    let sig = create_signature("test-1".to_string(), points);
    // This may or may not create a signature - document behavior
    // If it does, distance should be ~0
    if let Some(s) = sig {
        assert!(
            s.total_distance < 1.0,
            "Duplicate points should have ~0 distance"
        );
    }
}

#[test]
fn test_create_signature_invalid_coordinates() {
    // Mix of valid and invalid points
    let points = vec![
        GpsPoint::new(91.0, 0.0),        // Invalid latitude
        GpsPoint::new(51.5074, -0.1278), // Valid
        GpsPoint::new(0.0, 181.0),       // Invalid longitude
        GpsPoint::new(51.5084, -0.1288), // Valid
        GpsPoint::new(f64::NAN, 0.0),    // NaN
    ];
    let sig = create_signature("test-1".to_string(), points);
    // Should filter invalid and work with remaining valid points
    if let Some(s) = sig {
        assert!(
            s.points.iter().all(|p| p.is_valid()),
            "All points should be valid"
        );
    }
}

#[test]
fn test_create_signature_very_long_route() {
    // 100km route with 10000 points - should be simplified
    let points = generate_route(51.5074, -0.1278, 100.0, 100, 5.0);
    assert_eq!(points.len(), 10000);

    let sig = create_signature("test-1".to_string(), points);
    assert!(sig.is_some());
    let sig = sig.unwrap();

    // Should be heavily simplified
    assert!(
        sig.points.len() <= 100,
        "Should simplify to max_simplified_points"
    );
    assert!(
        sig.total_distance > 50_000.0,
        "Should preserve approximate distance"
    );
}

#[test]
fn test_create_signature_very_short_route() {
    // 100m route - below min_route_distance
    let points = generate_route(51.5074, -0.1278, 0.1, 100, 0.0);
    let sig = create_signature("test-1".to_string(), points);

    // Short routes can still create signatures (min_route_distance only affects grouping)
    assert!(sig.is_some());
}

// ============================================================================
// Route Comparison Tests
// ============================================================================

#[test]
fn test_compare_identical_routes() {
    let points = long_route();
    let sig1 = create_signature("a".to_string(), points.clone()).unwrap();
    let sig2 = create_signature("b".to_string(), points).unwrap();

    let result = ffi_compare_routes(&sig1, &sig2, MatchConfig::default());

    assert!(result.is_some(), "Identical routes should match");
    let result = result.unwrap();
    assert!(
        result.match_percentage > 95.0,
        "Identical routes should have >95% match"
    );
    assert_eq!(result.direction, "same");
}

#[test]
fn test_compare_routes_with_noise() {
    let clean = long_route();
    let noisy = noisy_route();

    let sig1 = create_signature("clean".to_string(), clean).unwrap();
    let sig2 = create_signature("noisy".to_string(), noisy).unwrap();

    let result = ffi_compare_routes(&sig1, &sig2, MatchConfig::default());

    // Should still match despite GPS noise
    assert!(result.is_some(), "Routes with noise should still match");
    let result = result.unwrap();
    assert!(
        result.match_percentage > 70.0,
        "Noisy routes should have >70% match"
    );
}

#[test]
fn test_compare_reversed_routes() {
    let points = long_route();
    let mut reversed = points.clone();
    reversed.reverse();

    let sig1 = create_signature("forward".to_string(), points).unwrap();
    let sig2 = create_signature("reverse".to_string(), reversed).unwrap();

    let result = ffi_compare_routes(&sig1, &sig2, MatchConfig::default());

    assert!(result.is_some(), "Reversed routes should match");
    let result = result.unwrap();
    assert!(
        result.match_percentage > 90.0,
        "Reversed routes should have high match"
    );
    assert_eq!(
        result.direction, "reverse",
        "Should detect reverse direction"
    );
}

#[test]
fn test_compare_different_routes() {
    let london = long_route();
    let nyc = different_route();

    let sig1 = create_signature("london".to_string(), london).unwrap();
    let sig2 = create_signature("nyc".to_string(), nyc).unwrap();

    let result = ffi_compare_routes(&sig1, &sig2, MatchConfig::default());

    assert!(
        result.is_none(),
        "Completely different routes should not match"
    );
}

#[test]
fn test_compare_routes_symmetry() {
    // AMD should be symmetric: compare(A,B) ≈ compare(B,A)
    let route1 = long_route();
    let route2 = noisy_route();

    let sig1 = create_signature("a".to_string(), route1).unwrap();
    let sig2 = create_signature("b".to_string(), route2).unwrap();

    let result_ab = ffi_compare_routes(&sig1, &sig2, MatchConfig::default());
    let result_ba = ffi_compare_routes(&sig2, &sig1, MatchConfig::default());

    match (result_ab, result_ba) {
        (Some(ab), Some(ba)) => {
            let diff = (ab.match_percentage - ba.match_percentage).abs();
            assert!(diff < 5.0, "Match should be symmetric (diff: {}%)", diff);
        }
        (None, None) => {} // Both don't match - symmetric
        _ => panic!("Asymmetric match/no-match result"),
    }
}

#[test]
fn test_compare_partial_overlap() {
    // Route B is first half of route A
    let full_route = generate_route(51.5074, -0.1278, 2.0, 100, 0.0);
    let half_route: Vec<GpsPoint> = full_route[..full_route.len() / 2].to_vec();

    let sig1 = create_signature("full".to_string(), full_route).unwrap();
    let sig2 = create_signature("half".to_string(), half_route).unwrap();

    let result = ffi_compare_routes(&sig1, &sig2, MatchConfig::default());

    // Partial overlap - may or may not match depending on threshold
    // This tests the distance ratio check (routes must be within 50% of each other)
    // Half route is 50% of full, so this is at the boundary
    if let Some(r) = result {
        assert!(
            r.direction == "partial" || r.match_percentage < 80.0,
            "Partial overlap should have lower match or be marked partial"
        );
    }
}

// ============================================================================
// Grouping Tests
// ============================================================================

#[test]
fn test_group_empty_input() {
    let groups = ffi_group_signatures(vec![], MatchConfig::default());
    assert!(groups.is_empty(), "Empty input should return empty groups");
}

#[test]
fn test_group_single_signature() {
    let sig = create_signature("only".to_string(), long_route()).unwrap();
    let groups = ffi_group_signatures(vec![sig], MatchConfig::default());

    assert_eq!(groups.len(), 1, "Single signature should create one group");
    assert_eq!(groups[0].activity_ids.len(), 1);
}

#[test]
fn test_group_identical_routes() {
    let route = long_route();
    let sig1 = create_signature("a".to_string(), route.clone()).unwrap();
    let sig2 = create_signature("b".to_string(), route.clone()).unwrap();
    let sig3 = create_signature("c".to_string(), route).unwrap();

    let groups = ffi_group_signatures(vec![sig1, sig2, sig3], MatchConfig::default());

    assert_eq!(groups.len(), 1, "Identical routes should form one group");
    assert_eq!(groups[0].activity_ids.len(), 3);
}

#[test]
fn test_group_different_routes() {
    let london = create_signature("london".to_string(), long_route()).unwrap();
    let nyc = create_signature("nyc".to_string(), different_route()).unwrap();

    // Generate a third distinct route (Paris area)
    let paris = create_signature(
        "paris".to_string(),
        generate_route(48.8566, 2.3522, 1.0, 100, 0.0),
    )
    .unwrap();

    let groups = ffi_group_signatures(vec![london, nyc, paris], MatchConfig::default());

    assert_eq!(
        groups.len(),
        3,
        "Different routes should form separate groups"
    );
}

#[test]
fn test_group_mixed_routes() {
    // 3 London routes (should group), 2 NYC routes (should group), 1 Paris (alone)
    let london1 = create_signature("london1".to_string(), long_route()).unwrap();
    let london2 = create_signature("london2".to_string(), noisy_route()).unwrap();
    let london3 = create_signature(
        "london3".to_string(),
        generate_route(51.5074, -0.1278, 1.0, 100, 5.0),
    )
    .unwrap();

    let nyc1 = create_signature("nyc1".to_string(), different_route()).unwrap();
    let nyc2 = create_signature(
        "nyc2".to_string(),
        generate_route(40.7128, -74.0060, 1.0, 100, 5.0),
    )
    .unwrap();

    let paris = create_signature(
        "paris".to_string(),
        generate_route(48.8566, 2.3522, 1.0, 100, 0.0),
    )
    .unwrap();

    let groups = ffi_group_signatures(
        vec![london1, london2, london3, nyc1, nyc2, paris],
        MatchConfig::default(),
    );

    // Should form 3 groups: London(3), NYC(2), Paris(1)
    assert_eq!(groups.len(), 3, "Should form 3 groups");

    // Find groups by size
    let sizes: Vec<usize> = groups.iter().map(|g| g.activity_ids.len()).collect();
    assert!(sizes.contains(&3), "Should have a group of 3 (London)");
    assert!(sizes.contains(&2), "Should have a group of 2 (NYC)");
    assert!(sizes.contains(&1), "Should have a group of 1 (Paris)");
}

#[test]
fn test_group_short_routes_excluded() {
    // Routes under min_route_distance (500m) should still be grouped
    // but won't affect matching threshold
    let short1 = create_signature(
        "short1".to_string(),
        generate_route(51.5074, -0.1278, 0.3, 100, 0.0), // 300m
    );
    let short2 = create_signature(
        "short2".to_string(),
        generate_route(51.5074, -0.1278, 0.3, 100, 0.0), // 300m
    );

    if let (Some(s1), Some(s2)) = (short1, short2) {
        let groups = ffi_group_signatures(vec![s1, s2], MatchConfig::default());
        // Short routes may or may not group depending on implementation
        assert!(!groups.is_empty(), "Should return at least one group");
    }
}

// ============================================================================
// Incremental Grouping Tests
// ============================================================================

#[test]
fn test_incremental_adds_to_existing_group() {
    let route = long_route();
    let sig1 = create_signature("a".to_string(), route.clone()).unwrap();
    let sig2 = create_signature("b".to_string(), route.clone()).unwrap();
    let sig3 = create_signature("c".to_string(), route).unwrap();

    // Initial grouping with first two
    let initial_groups =
        ffi_group_signatures(vec![sig1.clone(), sig2.clone()], MatchConfig::default());
    assert_eq!(initial_groups.len(), 1);
    assert_eq!(initial_groups[0].activity_ids.len(), 2);

    // Incrementally add third
    let updated = ffi_group_incremental(
        vec![sig3],
        initial_groups,
        vec![sig1, sig2],
        MatchConfig::default(),
    );

    assert_eq!(updated.len(), 1, "Should still have one group");
    assert_eq!(
        updated[0].activity_ids.len(),
        3,
        "Should now have 3 activities"
    );
}

#[test]
fn test_incremental_creates_new_group() {
    let london_route = long_route();
    let sig1 = create_signature("london1".to_string(), london_route.clone()).unwrap();
    let sig2 = create_signature("london2".to_string(), london_route).unwrap();

    // Initial grouping
    let initial_groups =
        ffi_group_signatures(vec![sig1.clone(), sig2.clone()], MatchConfig::default());
    assert_eq!(initial_groups.len(), 1);

    // Add a different route
    let nyc = create_signature("nyc".to_string(), different_route()).unwrap();
    let updated = ffi_group_incremental(
        vec![nyc],
        initial_groups,
        vec![sig1, sig2],
        MatchConfig::default(),
    );

    assert_eq!(updated.len(), 2, "Should now have two groups");
}

#[test]
fn test_incremental_matches_full_grouping() {
    // Verify that incremental grouping produces same result as full rebuild
    let routes: Vec<Vec<GpsPoint>> = (0..5)
        .map(|i| generate_route(51.5074 + i as f64 * 0.01, -0.1278, 1.0, 100, 5.0))
        .collect();

    let sigs: Vec<RouteSignature> = routes
        .iter()
        .enumerate()
        .filter_map(|(i, r)| create_signature(format!("route-{}", i), r.clone()))
        .collect();

    // Full grouping
    let full_groups = ffi_group_signatures(sigs.clone(), MatchConfig::default());

    // Incremental: add one at a time
    let mut incremental_groups = vec![];
    let mut existing_sigs = vec![];

    for sig in sigs.iter() {
        incremental_groups = ffi_group_incremental(
            vec![sig.clone()],
            incremental_groups,
            existing_sigs.clone(),
            MatchConfig::default(),
        );
        existing_sigs.push(sig.clone());
    }

    // Should have same number of groups
    assert_eq!(
        full_groups.len(),
        incremental_groups.len(),
        "Incremental should produce same group count as full rebuild"
    );

    // Each group should have same number of activities
    let full_sizes: Vec<usize> = full_groups.iter().map(|g| g.activity_ids.len()).collect();
    let incr_sizes: Vec<usize> = incremental_groups
        .iter()
        .map(|g| g.activity_ids.len())
        .collect();

    let mut full_sorted = full_sizes.clone();
    let mut incr_sorted = incr_sizes.clone();
    full_sorted.sort();
    incr_sorted.sort();

    assert_eq!(full_sorted, incr_sorted, "Group sizes should match");
}

// ============================================================================
// Flat Buffer Processing Tests
// ============================================================================

#[test]
fn test_flat_buffer_basic() {
    let route = long_route();
    let flat_coords: Vec<f64> = route
        .iter()
        .flat_map(|p| vec![p.latitude, p.longitude])
        .collect();

    let track = FlatGpsTrack {
        activity_id: "test".to_string(),
        coords: flat_coords,
    };

    let signatures = create_signatures_from_flat(vec![track], MatchConfig::default());

    assert_eq!(signatures.len(), 1);
    assert_eq!(signatures[0].activity_id, "test");
}

#[test]
fn test_flat_buffer_empty() {
    let track = FlatGpsTrack {
        activity_id: "empty".to_string(),
        coords: vec![],
    };

    let signatures = create_signatures_from_flat(vec![track], MatchConfig::default());
    assert!(
        signatures.is_empty(),
        "Empty coords should produce no signatures"
    );
}

#[test]
fn test_flat_buffer_odd_count() {
    // Odd number of coords (malformed - missing one longitude)
    let track = FlatGpsTrack {
        activity_id: "malformed".to_string(),
        coords: vec![51.5074, -0.1278, 51.5084], // 3 values, not 4
    };

    let signatures = create_signatures_from_flat(vec![track], MatchConfig::default());
    // chunks_exact(2) should handle this gracefully
    // Will get 1 point (51.5074, -0.1278), which is not enough
    assert!(
        signatures.is_empty(),
        "Odd coords should produce no signature (only 1 valid point)"
    );
}

#[test]
fn test_flat_buffer_multiple_tracks() {
    let route1 = long_route();
    let route2 = different_route();

    let tracks = vec![
        FlatGpsTrack {
            activity_id: "london".to_string(),
            coords: route1
                .iter()
                .flat_map(|p| vec![p.latitude, p.longitude])
                .collect(),
        },
        FlatGpsTrack {
            activity_id: "nyc".to_string(),
            coords: route2
                .iter()
                .flat_map(|p| vec![p.latitude, p.longitude])
                .collect(),
        },
    ];

    let signatures = create_signatures_from_flat(tracks, MatchConfig::default());
    assert_eq!(signatures.len(), 2);
}

#[test]
fn test_process_routes_from_flat_end_to_end() {
    // Full pipeline: flat buffer -> signatures -> groups
    let route = long_route();
    let noisy_same = noisy_route();

    let tracks = vec![
        FlatGpsTrack {
            activity_id: "a".to_string(),
            coords: route
                .iter()
                .flat_map(|p| vec![p.latitude, p.longitude])
                .collect(),
        },
        FlatGpsTrack {
            activity_id: "b".to_string(),
            coords: noisy_same
                .iter()
                .flat_map(|p| vec![p.latitude, p.longitude])
                .collect(),
        },
        FlatGpsTrack {
            activity_id: "c".to_string(),
            coords: different_route()
                .iter()
                .flat_map(|p| vec![p.latitude, p.longitude])
                .collect(),
        },
    ];

    let groups = process_routes_from_flat(tracks, MatchConfig::default());

    assert_eq!(groups.len(), 2, "Should form 2 groups (London pair + NYC)");
}

// ============================================================================
// GPS Edge Cases
// ============================================================================

#[test]
fn test_route_near_equator() {
    let route = generate_route(0.0, 0.0, 1.0, 100, 0.0);
    let sig = create_signature("equator".to_string(), route);
    assert!(sig.is_some(), "Route at equator should work");
}

#[test]
fn test_route_near_poles() {
    // Near north pole - longitude becomes degenerate
    let route = generate_route(89.0, 0.0, 1.0, 100, 0.0);
    let sig = create_signature("arctic".to_string(), route);
    assert!(sig.is_some(), "Route near pole should work");
}

#[test]
fn test_route_crossing_prime_meridian() {
    // Route that crosses longitude 0
    let points: Vec<GpsPoint> = (0..100)
        .map(|i| GpsPoint::new(51.5074, -0.05 + i as f64 * 0.001))
        .collect();

    let sig = create_signature("prime_meridian".to_string(), points);
    assert!(sig.is_some(), "Route crossing prime meridian should work");
}

#[test]
fn test_route_crossing_antimeridian() {
    // Route that crosses longitude 180/-180 (date line)
    // This is a known problematic case for many GPS systems
    let mut points = Vec::new();
    for i in 0..100 {
        let lng = 179.5 + i as f64 * 0.01; // Goes from 179.5 to 180.5
        let normalized_lng = if lng > 180.0 { lng - 360.0 } else { lng };
        points.push(GpsPoint::new(0.0, normalized_lng));
    }

    let _sig = create_signature("date_line".to_string(), points);
    // Note: This test may reveal issues with antimeridian handling
    // If it fails, that's valuable information about a real edge case
}

#[test]
fn test_loop_route() {
    // Route that starts and ends at same point
    let mut points = long_route();
    points.push(points[0]); // Close the loop

    let sig = create_signature("loop".to_string(), points);
    assert!(sig.is_some());

    let sig = sig.unwrap();
    let start_end_dist = tracematch::geo_utils::haversine_distance(&sig.start_point, &sig.end_point);
    assert!(start_end_dist < 100.0, "Loop should have start ≈ end");
}

#[test]
fn test_out_and_back_route() {
    // Route that goes out and comes back on same path
    let outbound = long_route();
    let mut inbound = outbound.clone();
    inbound.reverse();

    let mut full_route = outbound;
    full_route.extend(inbound);

    let sig = create_signature("out_and_back".to_string(), full_route);
    assert!(sig.is_some());
}

#[test]
fn test_stationary_gps() {
    // GPS recording while standing still (all points clustered)
    let points: Vec<GpsPoint> = (0..100)
        .map(|i| {
            // Tiny variations simulating GPS drift while stationary
            let jitter = (i as f64 * 0.1).sin() * 0.00001; // ~1m jitter
            GpsPoint::new(51.5074 + jitter, -0.1278 + jitter)
        })
        .collect();

    let sig = create_signature("stationary".to_string(), points);
    if let Some(s) = sig {
        assert!(
            s.total_distance < 100.0,
            "Stationary should have minimal distance"
        );
    }
}

#[test]
fn test_gps_teleport() {
    // GPS glitch causing sudden "teleport" to wrong location
    let mut points = long_route();
    // Insert a glitch point in the middle
    let mid = points.len() / 2;
    points.insert(mid, GpsPoint::new(0.0, 0.0)); // Teleport to null island

    let sig = create_signature("teleport".to_string(), points);
    // Simplification should hopefully remove the outlier
    // Or the route should still be usable
    assert!(sig.is_some());
}

// ============================================================================
// Configuration Tests
// ============================================================================

#[test]
fn test_custom_config() {
    let points = long_route();

    let strict_config = MatchConfig {
        min_match_percentage: 90.0,
        perfect_threshold: 10.0,
        zero_threshold: 100.0,
        ..MatchConfig::default()
    };

    let lenient_config = MatchConfig {
        min_match_percentage: 30.0,
        perfect_threshold: 100.0,
        zero_threshold: 500.0,
        ..MatchConfig::default()
    };

    let noisy = noisy_route();
    let sig1 = create_signature_with_config(
        "clean".to_string(),
        points.clone(),
        strict_config.clone(),
    )
    .unwrap();
    let sig2 =
        create_signature_with_config("noisy".to_string(), noisy.clone(), strict_config.clone())
            .unwrap();

    let strict_result = ffi_compare_routes(&sig1, &sig2, strict_config);

    let sig1_len =
        create_signature_with_config("clean".to_string(), points, lenient_config.clone())
            .unwrap();
    let sig2_len =
        create_signature_with_config("noisy".to_string(), noisy, lenient_config.clone())
            .unwrap();
    let lenient_result = ffi_compare_routes(&sig1_len, &sig2_len, lenient_config);

    // Lenient config should find a match where strict might not
    // (or lenient should have higher match %)
    match (strict_result, lenient_result) {
        (None, Some(_)) => {} // Expected: strict rejects, lenient accepts
        (Some(_), Some(_)) => {
            // Both accept - valid scenario
        }
        _ => {} // Other combinations are valid
    }
}

// ============================================================================
// Section Detection Config Tests
// ============================================================================

#[test]
fn test_section_configs_differ() {
    let default = SectionConfig::default();
    let discovery = SectionConfig::discovery();
    let conservative = SectionConfig::conservative();

    // Discovery should be more sensitive (lower thresholds)
    assert!(discovery.min_activities <= default.min_activities);

    // Conservative should be more strict (higher thresholds)
    assert!(conservative.min_activities >= default.min_activities);
}

// ============================================================================
// Large Scale Stress Tests
// ============================================================================

#[test]
fn test_many_routes_grouping() {
    // 50 routes in 5 "clusters" of 10 each
    let mut sigs = Vec::new();

    for cluster in 0..5 {
        let base_lat = 40.0 + cluster as f64 * 10.0;
        let base_lng = -74.0 + cluster as f64 * 10.0;

        for i in 0..10 {
            let route = generate_route(base_lat, base_lng, 1.0, 50, 10.0);
            if let Some(sig) = create_signature(format!("c{}-r{}", cluster, i), route) {
                sigs.push(sig);
            }
        }
    }

    assert_eq!(sigs.len(), 50);

    let groups = ffi_group_signatures(sigs, MatchConfig::default());

    // Should form approximately 5 groups
    assert!(
        groups.len() >= 4 && groups.len() <= 10,
        "Should form roughly 5 groups (got {})",
        groups.len()
    );
}

#[test]
fn test_many_points_per_route() {
    // Route with 50,000 points (50km at 1 point per meter)
    let points: Vec<GpsPoint> = (0..50_000)
        .map(|i| {
            let progress = i as f64 / 50_000.0;
            GpsPoint::new(51.5074 + progress * 0.5, -0.1278 + progress * 0.5)
        })
        .collect();

    let sig = create_signature("dense".to_string(), points);
    assert!(sig.is_some());

    let sig = sig.unwrap();
    assert!(
        sig.points.len() <= 100,
        "Should simplify to manageable size"
    );
}
