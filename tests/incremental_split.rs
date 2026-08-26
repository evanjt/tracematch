//! Scenario: a new activity shares the endpoints of an existing route group
//! but runs different ground in the middle.
//!
//! Expected behaviour: the incremental add path splits it out, the same way
//! a full regroup would, instead of leaving it in the group unchecked.

use tracematch::grouping::group_incremental_with_matches;
use tracematch::{GpsPoint, MatchConfig, RouteGroup, RouteSignature, group_signatures_parallel};

/// A 3 km run east along latitude 45, bulged north by `peak_deg` across the
/// middle 40% of the route. `jitter` separates otherwise identical rides.
fn route(peak_deg: f64, jitter: f64) -> Vec<GpsPoint> {
    (0..300)
        .map(|i| {
            let t = i as f64 / 299.0;
            let bulge = if (0.3..=0.7).contains(&t) {
                let u = (t - 0.3) / 0.4;
                peak_deg * (u * std::f64::consts::PI).sin()
            } else {
                0.0
            };
            GpsPoint::new(45.0 + bulge + jitter, t * 0.038)
        })
        .collect()
}

fn signature(id: &str, points: &[GpsPoint]) -> RouteSignature {
    RouteSignature::from_points(id, points, &MatchConfig::default()).unwrap()
}

/// The bulge is tuned to land between `min_match_percentage` and the split
/// threshold, so the pair groups on endpoints and then fails verification.
const DETOUR_PEAK_DEG: f64 = 0.0015;

fn direct_group() -> (Vec<RouteSignature>, Vec<RouteGroup>) {
    let config = MatchConfig::default();
    let signatures: Vec<RouteSignature> = (0..3)
        .map(|i| signature(&format!("direct_{i}"), &route(0.0, i as f64 * 0.000_01)))
        .collect();
    let groups = group_signatures_parallel(&signatures, &config);
    assert_eq!(groups.len(), 1, "the three direct rides are one route");
    (signatures, groups)
}

#[test]
fn a_new_ride_that_diverges_in_the_middle_is_split_out() {
    let config = MatchConfig::default();
    let (existing_sigs, existing_groups) = direct_group();

    let new_sigs: Vec<RouteSignature> = (0..2)
        .map(|i| {
            signature(
                &format!("detour_{i}"),
                &route(DETOUR_PEAK_DEG, i as f64 * 0.000_01),
            )
        })
        .collect();

    let result =
        group_incremental_with_matches(&new_sigs, &existing_groups, &existing_sigs, &config);

    let detours: Vec<&RouteGroup> = result
        .groups
        .iter()
        .filter(|g| g.activity_ids.iter().any(|id| id.starts_with("detour_")))
        .collect();

    assert_eq!(detours.len(), 1, "both detour rides land in one group");
    assert!(
        detours[0]
            .activity_ids
            .iter()
            .all(|id| id.starts_with("detour_")),
        "the detour group holds no direct ride: {:?}",
        detours[0].activity_ids
    );
}

#[test]
fn the_pair_groups_before_it_is_split_so_the_split_is_what_separates_them() {
    let config = MatchConfig::default();
    let (existing_sigs, _) = direct_group();

    let mut all = existing_sigs.clone();
    all.push(signature("detour_0", &route(DETOUR_PEAK_DEG, 0.0)));

    let groups = group_signatures_parallel(&all, &config);

    assert_eq!(
        groups.len(),
        1,
        "endpoint matching alone puts the detour in the group, so only \
         verification can take it out again: {groups:?}"
    );
}

#[test]
fn a_settled_group_keeps_its_members_when_nothing_new_touches_it() {
    let config = MatchConfig::default();
    let (existing_sigs, existing_groups) = direct_group();

    let elsewhere: Vec<RouteSignature> = (0..2)
        .map(|i| {
            let points: Vec<GpsPoint> = (0..300)
                .map(|j| {
                    let t = j as f64 / 299.0;
                    GpsPoint::new(48.0 + i as f64 * 0.000_01, t * 0.038)
                })
                .collect();
            signature(&format!("far_{i}"), &points)
        })
        .collect();

    let result =
        group_incremental_with_matches(&elsewhere, &existing_groups, &existing_sigs, &config);

    let settled = result
        .groups
        .iter()
        .find(|g| g.activity_ids.iter().any(|id| id.starts_with("direct_")))
        .expect("the direct group survives");
    assert_eq!(settled.activity_ids.len(), 3);
}
