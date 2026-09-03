//! The serialised shape the WASM boundary hands to the website.
//!
//! `web/src/lib/wasm/types.ts` mirrors these structs by hand, and nothing
//! else can catch it drifting: `wasm-bindgen` types every export as `any`,
//! so `svelte-check` sees a wrong or missing field as perfectly fine. The
//! site then reads `undefined` and renders nothing, silently.
//!
//! Field NAMES are what drift, and `serde_json` derives them from the same
//! attributes `serde-wasm-bindgen` uses, so a golden of the key set gates
//! the browser shape without needing a wasm build.
//!
//! Two known differences from what the browser receives, neither affecting
//! names: `Option::None` is `null` here and `undefined` there, and a
//! `HashMap` is an object here and a `Map` there.

use std::collections::HashMap;

use serde_json::Value;
use tracematch::{BoundaryReason, BoundaryRecord, Tunables};
use tracematch::{FrequentSection, GpsPoint};

fn keys(value: &Value) -> Vec<String> {
    let mut k: Vec<String> = value
        .as_object()
        .expect("expected a JSON object")
        .keys()
        .cloned()
        .collect();
    k.sort();
    k
}

fn reason_keys(reason: BoundaryReason) -> Vec<String> {
    let record = BoundaryRecord {
        latitude: 0.0,
        longitude: 0.0,
        reason,
    };
    let value: Value = serde_json::to_value(&record).expect("BoundaryRecord serialises");
    assert_eq!(
        keys(&value),
        vec!["latitude", "longitude", "reason"],
        "BoundaryRecord's own keys"
    );
    keys(&value["reason"])
}

/// Scenario: the site lists a section and reads its enrichment.
/// Expected behaviour: every key the mirror declares is present, spelled
/// exactly as `types.ts` spells it.
#[test]
fn frequent_section_serialises_the_keys_the_site_reads() {
    let section = FrequentSection {
        id: "s1".into(),
        name: None,
        sport_type: "Run".into(),
        polyline: vec![GpsPoint::new(46.0, 7.0), GpsPoint::new(46.001, 7.001)],
        representative_activity_id: "a1".into(),
        representative_range: Some((0, 1)),
        activity_ids: vec!["a1".into()],
        activity_portions: Vec::new(),
        route_ids: Vec::new(),
        visit_count: 1,
        distance_meters: 100.0,
        activity_traces: HashMap::new(),
        confidence: 1.0,
        observation_count: 1,
        average_spread: 0.0,
        point_density: vec![1, 1],
        scale: None,
        is_user_defined: false,
        stability: 1.0,
        // Both carry skip_serializing_if, so they must be Some here or
        // the golden would silently stop covering them.
        elevation_gain_m: Some(12.0),
        avg_grade_percent: Some(1.5),
        enrichment: Default::default(),
        rank: None,
        version: 1,
        updated_at: None,
        created_at: None,
        consensus_state: None,
    };

    let value = serde_json::to_value(&section).expect("FrequentSection serialises");
    assert_eq!(
        keys(&value),
        vec![
            "activityIds",
            "activityPortions",
            "activityTraces",
            "averageSpread",
            "avgGradePercent",
            "confidence",
            "createdAt",
            "distanceMeters",
            "elevationGainM",
            "enrichment",
            "id",
            "isUserDefined",
            "name",
            "observationCount",
            "pointDensity",
            "polyline",
            "rank",
            "representativeActivityId",
            "representativeRange",
            "routeIds",
            "scale",
            "sportType",
            "stability",
            "updatedAt",
            "version",
            "visitCount",
        ],
        "FrequentSection keys; update web/src/lib/wasm/types.ts to match"
    );

    assert_eq!(
        keys(&value["enrichment"]),
        vec![
            "avgGradePercent",
            "elevationGainM",
            "elevationLossM",
            "isLift",
            "klass",
            "maxGradePercent",
            "straightness",
        ],
        "Enrichment keys"
    );
}

/// Scenario: the map pins a boundary and prints the numbers behind it.
///
/// Expected behaviour: the variant tag is snake_case AND so are the payload
/// keys. `rename_all` on this enum renames variants only, so these are the
/// one place in the FFI surface that is not camelCase. Getting that wrong
/// renders an empty popup rather than an error.
#[test]
fn boundary_reason_payloads_stay_snake_case() {
    assert_eq!(
        reason_keys(BoundaryReason::UsageChange {
            shared: 1,
            mismatched: 1
        }),
        vec!["kind", "mismatched", "shared"]
    );
    assert_eq!(
        reason_keys(BoundaryReason::Fork {
            through: 1,
            needed: 1.0,
            branch_leavers: 1,
            branch_activity_ids: vec!["a".into()],
        }),
        vec![
            "branch_activity_ids",
            "branch_leavers",
            "kind",
            "needed",
            "through"
        ]
    );
    assert_eq!(
        reason_keys(BoundaryReason::Backoff {
            represented: 1,
            probed: 1,
            score_metres: 1.0,
        }),
        vec!["kind", "probed", "represented", "score_metres"]
    );
    assert_eq!(
        reason_keys(BoundaryReason::Trim {
            kept_metres: 1.0,
            dropped_metres: 1.0,
        }),
        vec!["dropped_metres", "kept_metres", "kind"]
    );
    assert_eq!(
        reason_keys(BoundaryReason::NoSinglePass {
            best_penalty: 1.0,
            portions: 1,
        }),
        vec!["best_penalty", "kind", "portions"]
    );
    assert_eq!(
        reason_keys(BoundaryReason::LowSupport {
            floor: 1,
            dropped_cells: 1,
        }),
        vec!["dropped_cells", "floor", "kind"]
    );
    assert_eq!(
        reason_keys(BoundaryReason::TrafficCliff { thin: 1, thick: 1 }),
        vec!["kind", "thick", "thin"]
    );
    assert_eq!(
        reason_keys(BoundaryReason::PassEnd { requeued_cells: 1 }),
        vec!["kind", "requeued_cells"]
    );
    assert_eq!(
        reason_keys(BoundaryReason::DrawnPopulation { kept: 1, floor: 1 }),
        vec!["floor", "kept", "kind"]
    );
    assert_eq!(
        reason_keys(BoundaryReason::SeamClip {
            section_id: "s".into(),
            clipped_metres: 1.0,
            kept_metres: 1.0,
        }),
        vec!["clipped_metres", "kept_metres", "kind", "section_id"]
    );
    assert_eq!(
        reason_keys(BoundaryReason::DrawnEmpty {
            section_id: "s".into(),
            contributors: 1,
        }),
        vec!["contributors", "kind", "section_id"]
    );
}

/// The variant tags themselves, which the site switches on.
#[test]
fn boundary_reason_tags_match_the_sites_switch() {
    let tag = |reason: BoundaryReason| {
        let record = BoundaryRecord {
            latitude: 0.0,
            longitude: 0.0,
            reason,
        };
        serde_json::to_value(&record).unwrap()["reason"]["kind"]
            .as_str()
            .unwrap()
            .to_string()
    };

    assert_eq!(
        tag(BoundaryReason::UsageChange {
            shared: 0,
            mismatched: 0
        }),
        "usage_change"
    );
    assert_eq!(
        tag(BoundaryReason::TrafficCliff { thin: 0, thick: 0 }),
        "traffic_cliff"
    );
    assert_eq!(
        tag(BoundaryReason::NoSinglePass {
            best_penalty: 0.0,
            portions: 0
        }),
        "no_single_pass"
    );
    assert_eq!(
        tag(BoundaryReason::PassEnd { requeued_cells: 0 }),
        "pass_end"
    );
    assert_eq!(
        tag(BoundaryReason::DrawnPopulation { kept: 0, floor: 0 }),
        "drawn_population"
    );
    assert_eq!(
        tag(BoundaryReason::SeamClip {
            section_id: String::new(),
            clipped_metres: 0.0,
            kept_metres: 0.0,
        }),
        "seam_clip"
    );
    assert_eq!(
        tag(BoundaryReason::DrawnEmpty {
            section_id: String::new(),
            contributors: 0,
        }),
        "drawn_empty"
    );
}

/// Scenario: the settings panel sends only the knobs the user moved.
///
/// Expected behaviour: every constant is camelCase and every one of them
/// defaults, so a partial object leaves the rest on `DEFAULT`. The panel's
/// KNOB_GROUPS table keys straight into this, so a rename here silently
/// stops that slider working unless this fails first.
#[test]
fn tunables_round_trip_partially_and_stay_camel_case() {
    let value = serde_json::to_value(Tunables::DEFAULT).expect("Tunables serialises");
    assert_eq!(
        keys(&value),
        vec![
            "clusterGapM",
            "descentMatchM",
            "dwellEvents",
            "eleLevelTolM",
            "jitterHumanMin",
            "liftMinClimbMh",
            "liftMinGrade",
            "liftMinSpeedMs",
            "liftMinStraight",
            "liftSpanM",
            "minorityRunM",
            "occasionSpanH",
            "passAwayCells",
            "passNeeded",
            "passSubgrid",
            "passWindow",
            "reach",
            "refLatQuantDeg",
            "selfPassClean",
            "selfPassMax",
        ],
        "Tunables keys; update KNOB_GROUPS in web/src/routes/+page.svelte to match"
    );

    // A partial object is the whole point: the panel sends only what moved.
    let partial: Tunables = serde_json::from_str(r#"{"passSubgrid": 4}"#).expect("partial parses");
    assert_eq!(partial.pass_subgrid, 4.0, "the moved knob takes effect");
    assert_eq!(
        partial.lift_min_climb_mh,
        Tunables::DEFAULT.lift_min_climb_mh,
        "every other knob stays on its default"
    );

    let empty: Tunables = serde_json::from_str("{}").expect("empty parses");
    assert_eq!(
        empty.pass_subgrid,
        Tunables::DEFAULT.pass_subgrid,
        "an empty object is DEFAULT"
    );
}
