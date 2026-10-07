//! Tests for grouping module

use tracematch::grouping::*;
use tracematch::{GpsPoint, GroupingResult, MatchConfig, RouteGroup, RouteSignature};

fn create_long_route() -> Vec<GpsPoint> {
    // Create a route long enough to meet min_route_distance (500m)
    // Each point is about 100m apart, 10 points = ~1km
    (0..10)
        .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
        .collect()
}

#[test]
fn test_group_identical_routes() {
    let long_route = create_long_route();

    let sig1 = RouteSignature::from_points("test-1", &long_route, &MatchConfig::default()).unwrap();
    let sig2 = RouteSignature::from_points("test-2", &long_route, &MatchConfig::default()).unwrap();

    let groups = group_signatures(&[sig1, sig2], &MatchConfig::default());

    // Should have 1 group with both routes
    assert_eq!(groups.len(), 1);
    assert_eq!(groups[0].activity_ids.len(), 2);
}

#[test]
fn test_group_different_routes() {
    let route1: Vec<GpsPoint> = (0..10)
        .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
        .collect();

    let route2: Vec<GpsPoint> = (0..10)
        .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
        .collect();

    let sig1 = RouteSignature::from_points("test-1", &route1, &MatchConfig::default()).unwrap();
    let sig2 = RouteSignature::from_points("test-2", &route2, &MatchConfig::default()).unwrap();

    let groups = group_signatures(&[sig1, sig2], &MatchConfig::default());

    // Should have 2 groups (different routes)
    assert_eq!(groups.len(), 2);
}

#[test]
fn test_distance_ratio_ok() {
    assert!(distance_ratio_ok(1000.0, 1200.0)); // 83% - ok
    assert!(distance_ratio_ok(1000.0, 500.0)); // 50% - ok
    assert!(!distance_ratio_ok(1000.0, 400.0)); // 40% - not ok
    assert!(!distance_ratio_ok(0.0, 1000.0)); // Invalid
}

/// Create a route with a deviation at a given position
fn create_route_with_deviation(deviation_meters: f64, deviation_position: f64) -> Vec<GpsPoint> {
    // 10 points, ~100m apart = ~1km total
    (0..10)
        .map(|i| {
            let progress = i as f64 / 9.0;
            let lat = 51.5074 + i as f64 * 0.001;
            let lng = if (progress - deviation_position).abs() < 0.1 {
                // Add lateral deviation (each 0.001 degree longitude ~ 70m at this latitude)
                -0.1278 + deviation_meters / 70000.0
            } else {
                -0.1278
            };
            GpsPoint::new(lat, lng)
        })
        .collect()
}

#[test]
fn test_grouping_deterministic() {
    // Run grouping multiple times - results should be identical
    let long_route = create_long_route();
    let deviated_route = create_route_with_deviation(30.0, 0.5);

    let results: Vec<_> = (0..5)
        .map(|_| {
            let sig1 =
                RouteSignature::from_points("a", &long_route, &MatchConfig::default()).unwrap();
            let sig2 =
                RouteSignature::from_points("b", &deviated_route, &MatchConfig::default()).unwrap();

            group_signatures_with_matches(&[sig1, sig2], &MatchConfig::default())
        })
        .collect();

    // All results should have same groups and representatives
    for i in 1..results.len() {
        assert_eq!(
            results[0].groups.len(),
            results[i].groups.len(),
            "Different group counts on run {i}"
        );

        for j in 0..results[0].groups.len() {
            assert_eq!(
                results[0].groups[j].representative_id, results[i].groups[j].representative_id,
                "Different representative on run {i}"
            );
        }
    }
}

#[test]
fn test_grouping_load_order_independence() {
    // Same routes loaded in different orders should produce same groups
    let route_a = create_long_route();
    let route_b = create_route_with_deviation(20.0, 0.3);
    let route_c = create_route_with_deviation(25.0, 0.6);

    // Different route (different location entirely)
    let route_d: Vec<GpsPoint> = (0..10)
        .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
        .collect();

    let config = MatchConfig::default();

    // Order 1: A, B, C, D
    let sigs_order1 = vec![
        RouteSignature::from_points("a", &route_a, &config).unwrap(),
        RouteSignature::from_points("b", &route_b, &config).unwrap(),
        RouteSignature::from_points("c", &route_c, &config).unwrap(),
        RouteSignature::from_points("d", &route_d, &config).unwrap(),
    ];

    // Order 2: D, C, B, A (reversed)
    let sigs_order2 = vec![
        RouteSignature::from_points("d", &route_d, &config).unwrap(),
        RouteSignature::from_points("c", &route_c, &config).unwrap(),
        RouteSignature::from_points("b", &route_b, &config).unwrap(),
        RouteSignature::from_points("a", &route_a, &config).unwrap(),
    ];

    // Order 3: B, D, A, C (shuffled)
    let sigs_order3 = vec![
        RouteSignature::from_points("b", &route_b, &config).unwrap(),
        RouteSignature::from_points("d", &route_d, &config).unwrap(),
        RouteSignature::from_points("a", &route_a, &config).unwrap(),
        RouteSignature::from_points("c", &route_c, &config).unwrap(),
    ];

    let result1 = group_signatures_with_matches(&sigs_order1, &config);
    let result2 = group_signatures_with_matches(&sigs_order2, &config);
    let result3 = group_signatures_with_matches(&sigs_order3, &config);

    // Helper to get sorted groups for comparison
    let get_sorted_groups = |result: &tracematch::GroupingResult| -> Vec<Vec<String>> {
        let mut groups: Vec<Vec<String>> = result
            .groups
            .iter()
            .map(|g| {
                let mut ids = g.activity_ids.clone();
                ids.sort();
                ids
            })
            .collect();
        groups.sort();
        groups
    };

    // Should have same number of groups
    assert_eq!(
        result1.groups.len(),
        result2.groups.len(),
        "Different group counts for order 1 vs 2"
    );
    assert_eq!(
        result1.groups.len(),
        result3.groups.len(),
        "Different group counts for order 1 vs 3"
    );

    // Each group should contain the same activities
    assert_eq!(
        get_sorted_groups(&result1),
        get_sorted_groups(&result2),
        "Groups differ for order 1 vs 2"
    );
    assert_eq!(
        get_sorted_groups(&result1),
        get_sorted_groups(&result3),
        "Groups differ for order 1 vs 3"
    );

    // Representatives should be the same (after determinism fix)
    let get_reps = |result: &tracematch::GroupingResult| -> Vec<String> {
        let mut reps: Vec<String> = result
            .groups
            .iter()
            .map(|g| g.representative_id.clone())
            .collect();
        reps.sort();
        reps
    };

    assert_eq!(
        get_reps(&result1),
        get_reps(&result2),
        "Representatives differ for order 1 vs 2"
    );
    assert_eq!(
        get_reps(&result1),
        get_reps(&result3),
        "Representatives differ for order 1 vs 3"
    );
}

const BASE_LAT: f64 = 47.0;
const BASE_LNG: f64 = 8.0;

/// A point `east` and `north` metres from the base, on a flat-earth approximation.
fn offset_point(east: f64, north: f64) -> GpsPoint {
    let metres_per_degree = 111_320.0;
    GpsPoint::new(
        BASE_LAT + north / metres_per_degree,
        BASE_LNG + east / (metres_per_degree * BASE_LAT.to_radians().cos()),
    )
}

/// Points every ~25 m along the polyline through `corners`.
fn trace_through(corners: &[(f64, f64)]) -> Vec<GpsPoint> {
    let mut points = Vec::new();
    for pair in corners.windows(2) {
        let (from, to) = (pair[0], pair[1]);
        let length = ((to.0 - from.0).powi(2) + (to.1 - from.1).powi(2)).sqrt();
        let steps = (length / 25.0).ceil() as usize;
        for step in 0..steps {
            let t = step as f64 / steps as f64;
            points.push(offset_point(
                from.0 + (to.0 - from.0) * t,
                from.1 + (to.1 - from.1) * t,
            ));
        }
    }
    let last = corners[corners.len() - 1];
    points.push(offset_point(last.0, last.1));
    points
}

/// A 2.5 km rectangular loop starting and ending at the origin.
fn plain_loop_corners() -> Vec<(f64, f64)> {
    vec![
        (0.0, 0.0),
        (600.0, 0.0),
        (600.0, 650.0),
        (0.0, 650.0),
        (0.0, 0.0),
    ]
}

/// The same loop with a straight lead-in and lead-out of `lead` metres.
fn loop_with_lead_corners(lead: f64) -> Vec<(f64, f64)> {
    let mut corners = vec![(0.0, -lead)];
    corners.extend(plain_loop_corners());
    corners.push((0.0, -lead));
    corners
}

fn signature(id: &str, corners: &[(f64, f64)], config: &MatchConfig) -> RouteSignature {
    RouteSignature::from_points(id, &trace_through(corners), config).unwrap()
}

fn loop_recordings(config: &MatchConfig) -> Vec<RouteSignature> {
    vec![
        signature("a1", &loop_with_lead_corners(200.0), config),
        signature("a2", &plain_loop_corners(), config),
        signature("a3", &plain_loop_corners(), config),
        signature("a4", &plain_loop_corners(), config),
    ]
}

/// The loop with its north edge pushed `shift` metres north.
fn loop_with_shifted_north_edge(shift: f64) -> Vec<(f64, f64)> {
    vec![
        (0.0, 0.0),
        (600.0, 0.0),
        (600.0, 650.0 + shift),
        (0.0, 650.0 + shift),
        (0.0, 0.0),
    ]
}

/// `a1` and `a2` drift from the plain loop by different amounts, so `a1` joins the group
/// only through `a2` and reads under the grouping threshold against `a3` and `a4`.
fn chained_recordings(config: &MatchConfig) -> Vec<RouteSignature> {
    vec![
        signature("a1", &loop_with_shifted_north_edge(250.0), config),
        signature("a2", &loop_with_shifted_north_edge(120.0), config),
        signature("a3", &plain_loop_corners(), config),
        signature("a4", &plain_loop_corners(), config),
    ]
}

fn single_group(result: &GroupingResult) -> &RouteGroup {
    assert_eq!(
        result.groups.len(),
        1,
        "expected one route: {:?}",
        result.groups
    );
    &result.groups[0]
}

fn assert_loop_represented_by_plain_recording(result: &GroupingResult) {
    let group = single_group(result);
    assert_eq!(group.activity_ids.len(), 4);
    assert_ne!(
        group.representative_id, "a1",
        "the recording with the lead-in must not represent the route"
    );
}

fn assert_chain_represented_by_member_others_match(result: &GroupingResult) {
    let group = single_group(result);
    assert_eq!(group.activity_ids.len(), 4);
    assert_ne!(group.representative_id, "a1");
    let matches = &result.activity_matches[&group.group_id];
    for id in ["a2", "a3", "a4"] {
        let read = matches
            .iter()
            .find(|m| m.activity_id == id)
            .unwrap_or_else(|| panic!("{id} has no match against the representative"));
        assert!(
            read.match_percentage >= 80.0,
            "{id} reads {}",
            read.match_percentage
        );
    }
}

#[test]
fn parallel_representative_is_not_the_sorted_first_recording_with_a_detour() {
    let config = MatchConfig::default();
    let result = group_signatures_parallel_with_matches(&loop_recordings(&config), &config);
    assert_loop_represented_by_plain_recording(&result);
}

#[test]
fn sequential_representative_is_not_the_sorted_first_recording_with_a_detour() {
    let config = MatchConfig::default();
    let result = group_signatures_with_matches(&loop_recordings(&config), &config);
    assert_loop_represented_by_plain_recording(&result);
}

#[test]
fn incremental_representative_is_not_the_sorted_first_recording_with_a_detour() {
    let config = MatchConfig::default();
    let far_away: Vec<GpsPoint> = (0..10)
        .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
        .collect();
    let existing = vec![RouteSignature::from_points("far", &far_away, &config).unwrap()];
    let existing_groups = group_signatures(&existing, &config);

    let result = group_incremental_with_matches(
        &loop_recordings(&config),
        &existing_groups,
        &existing,
        &config,
    );

    let loop_group = result
        .groups
        .iter()
        .find(|g| g.activity_ids.len() == 4)
        .expect("the loop recordings form one route");
    assert_ne!(loop_group.representative_id, "a1");
}

#[test]
fn parallel_chained_group_is_represented_by_a_member_the_others_match() {
    let config = MatchConfig::default();
    let result = group_signatures_parallel_with_matches(&chained_recordings(&config), &config);
    assert_chain_represented_by_member_others_match(&result);
}

#[test]
fn incremental_chained_group_is_represented_by_a_member_the_others_match() {
    let config = MatchConfig::default();
    let far_away: Vec<GpsPoint> = (0..10)
        .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
        .collect();
    let existing = vec![RouteSignature::from_points("far", &far_away, &config).unwrap()];
    let existing_groups = group_signatures(&existing, &config);

    let result = group_incremental_with_matches(
        &chained_recordings(&config),
        &existing_groups,
        &existing,
        &config,
    );

    let chain = result
        .groups
        .iter()
        .find(|g| g.activity_ids.len() == 4)
        .expect("the chained recordings form one route");
    assert_ne!(chain.representative_id, "a1");
}

#[test]
fn incremental_keeps_an_existing_representative_that_matches_poorly() {
    let config = MatchConfig::default();
    let existing = vec![
        signature("a1", &loop_with_lead_corners(150.0), &config),
        signature("a2", &plain_loop_corners(), &config),
        signature("a3", &plain_loop_corners(), &config),
    ];
    let mut existing_groups = group_signatures(&existing, &config);
    assert_eq!(existing_groups.len(), 1);
    existing_groups[0].representative_id = "a1".to_string();
    let new = vec![signature("a4", &plain_loop_corners(), &config)];

    let result = group_incremental_with_matches(&new, &existing_groups, &existing, &config);

    assert_eq!(single_group(&result).representative_id, "a1");
}

#[test]
fn a_large_group_picks_its_representative_within_the_sample() {
    let config = MatchConfig::default();
    let mut recordings = vec![signature("a000", &loop_with_lead_corners(150.0), &config)];
    for i in 1..60 {
        recordings.push(signature(
            &format!("a{i:03}"),
            &plain_loop_corners(),
            &config,
        ));
    }
    let result = group_signatures_parallel_with_matches(&recordings, &config);
    let group = single_group(&result);
    assert_eq!(group.activity_ids.len(), 60);
    assert_ne!(group.representative_id, "a000");
}
