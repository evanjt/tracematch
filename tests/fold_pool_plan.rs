//! What a fold actually has to read, asked before anything is read.
//!
//! Scenario: a sync stores one activity into a library of hundreds. The caller
//! loads and decodes every track in the library before folding, because it
//! cannot tell which ones the fold will touch.
//!
//! Expected behaviour: `pool_for_fold` answers that from the cache's own
//! footprints. It is the new ids plus every member of every cluster they route
//! into, and it must never be short: phase two indexes a dirty cluster's members
//! by a hard lookup, so a missing member is a panic rather than a worse answer.
//!
//! Run: `cargo test --test fold_pool_plan -p tracematch`

use std::collections::BTreeSet;

use tracematch::sections::{
    CLUSTER_GAP_M, ClusterFootprint, POOLED_SPORT, clusters_touched_by, pool_for_fold,
};

/// `(min_lat, max_lat, min_lng, max_lng)`.
type Bbox = (f64, f64, f64, f64);

/// A box a metre across at the given degrees, far enough from its neighbours
/// that the 50 km cluster gap does not bridge them unless meant to.
fn at(lat: f64, lng: f64) -> Bbox {
    (lat, lat + 0.00001, lng, lng + 0.00001)
}

fn cluster(members: &[(&str, Bbox)]) -> ClusterFootprint {
    let mut union = members[0].1;
    for (_, bb) in members {
        union.0 = union.0.min(bb.0);
        union.1 = union.1.max(bb.1);
        union.2 = union.2.min(bb.2);
        union.3 = union.3.max(bb.3);
    }
    ClusterFootprint {
        member_ids: members.iter().map(|(id, _)| id.to_string()).collect(),
        member_bboxes: members.iter().map(|(_, bb)| *bb).collect(),
        union_bbox: union,
    }
}

fn ids(got: &BTreeSet<String>) -> Vec<String> {
    got.iter().cloned().collect()
}

/// Geneva, Sydney and Tokyo: three clusters no 50 km gap can bridge.
const GENEVA: (f64, f64) = (46.20, 6.14);
const SYDNEY: (f64, f64) = (-33.87, 151.21);
const TOKYO: (f64, f64) = (35.68, 139.69);

fn three_cities() -> Vec<ClusterFootprint> {
    vec![
        cluster(&[
            ("geneva_1", at(GENEVA.0, GENEVA.1)),
            ("geneva_2", at(GENEVA.0 + 0.01, GENEVA.1)),
        ]),
        cluster(&[
            ("sydney_1", at(SYDNEY.0, SYDNEY.1)),
            ("sydney_2", at(SYDNEY.0 + 0.01, SYDNEY.1)),
        ]),
        cluster(&[("tokyo_1", at(TOKYO.0, TOKYO.1))]),
    ]
}

#[test]
fn a_new_ride_at_home_asks_for_its_own_cluster_and_no_other() {
    let pool = pool_for_fold(
        &three_cities(),
        &[("new_geneva".to_string(), at(GENEVA.0, GENEVA.1))],
    );

    assert_eq!(
        ids(&pool),
        vec!["geneva_1", "geneva_2", "new_geneva"],
        "Sydney and Tokyo are untouched, so their tracks are not read"
    );
}

#[test]
fn a_ride_bridging_two_clusters_asks_for_both() {
    let pool = pool_for_fold(
        &three_cities(),
        &[
            ("new_geneva".to_string(), at(GENEVA.0, GENEVA.1)),
            ("new_tokyo".to_string(), at(TOKYO.0, TOKYO.1)),
        ],
    );

    assert_eq!(
        ids(&pool),
        vec!["geneva_1", "geneva_2", "new_geneva", "new_tokyo", "tokyo_1"]
    );
}

#[test]
fn a_ride_in_new_country_asks_only_for_itself() {
    // Reykjavik: no cluster is within reach, so the fold makes a fresh one
    // holding this id alone.
    let pool = pool_for_fold(
        &three_cities(),
        &[("new_iceland".to_string(), at(64.13, -21.90))],
    );

    assert_eq!(ids(&pool), vec!["new_iceland"]);
}

#[test]
fn a_cold_cache_asks_for_every_new_id_and_nothing_else_exists() {
    let pool = pool_for_fold(
        &[],
        &[
            ("a".to_string(), at(GENEVA.0, GENEVA.1)),
            ("b".to_string(), at(SYDNEY.0, SYDNEY.1)),
        ],
    );

    assert_eq!(ids(&pool), vec!["a", "b"]);
}

#[test]
fn nothing_new_asks_for_nothing() {
    assert!(pool_for_fold(&three_cities(), &[]).is_empty());
}

/// The planner's whole reason to exist: it must agree with the routing the fold
/// itself does, and it must never answer short.
#[test]
fn the_plan_covers_every_cluster_the_router_would_pick() {
    let clusters = three_cities();
    let gap = CLUSTER_GAP_M;

    for (lat, lng) in [GENEVA, SYDNEY, TOKYO, (64.13, -21.90)] {
        let bbox = at(lat, lng);
        let routed = clusters_touched_by(&clusters, bbox, gap);
        let plan = pool_for_fold(&clusters, &[("newcomer".to_string(), bbox)]);

        for index in routed {
            for member in &clusters[index].member_ids {
                assert!(
                    plan.contains(member),
                    "{member} is in a cluster the router picked and the plan left it out"
                );
            }
        }
        assert!(plan.contains("newcomer"));
    }
}

#[test]
fn the_pooled_sport_label_is_the_one_the_fold_uses() {
    // The planner is asked per sport bucket, and under the default config every
    // track is relabelled to this one before the cache is touched.
    assert_eq!(POOLED_SPORT, "All");
}
