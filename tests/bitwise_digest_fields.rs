//! The fields the catalogue digest leaves out are covered by the enrichment digest.
//!
//! Scenario: a change to the enrichment, the elevation figures or the
//! representative range moves no polyline.
//!
//! Expected behaviour: the catalogue digest stays put and the enrichment digest
//! moves, so the golden gate notices.

#![allow(dead_code)]

mod bitwise;

use bitwise::{catalogue_digest, enrichment_digest};
use tracematch::sections::{FrequentSection, SectionClass};

fn section() -> FrequentSection {
    serde_json::from_value(serde_json::json!({
        "id": "sec_run_0",
        "name": null,
        "sportType": "Run",
        "polyline": [
            {"latitude": 1.0, "longitude": 2.0},
            {"latitude": 1.001, "longitude": 2.001}
        ],
        "representativeActivityId": "a1",
        "activityIds": ["a1"],
        "activityPortions": [],
        "visitCount": 1,
        "distanceMeters": 150.0,
        "activityTraces": {},
        "confidence": 0.5,
        "observationCount": 1,
        "averageSpread": 1.0,
        "pointDensity": [1, 1],
        "scale": null,
        "isUserDefined": false
    }))
    .expect("section")
}

type Edit = Box<dyn Fn(&mut FrequentSection)>;

fn moved(edit: impl Fn(&mut FrequentSection)) -> (bool, bool) {
    let base = vec![section()];
    let mut changed = base.clone();
    edit(&mut changed[0]);
    (
        catalogue_digest(&base) != catalogue_digest(&changed),
        enrichment_digest(&base) != enrichment_digest(&changed),
    )
}

#[test]
fn an_enrichment_change_moves_only_the_enrichment_digest() {
    let edits: Vec<(&str, Edit)> = vec![
        (
            "max grade",
            Box::new(|s| s.enrichment.max_grade_percent = Some(9.5)),
        ),
        (
            "loss",
            Box::new(|s| s.enrichment.elevation_loss_m = Some(3.0)),
        ),
        (
            "gain",
            Box::new(|s| s.enrichment.elevation_gain_m = Some(3.0)),
        ),
        (
            "net grade",
            Box::new(|s| s.enrichment.avg_grade_percent = Some(-2.0)),
        ),
        (
            "straightness",
            Box::new(|s| s.enrichment.straightness = Some(0.5)),
        ),
        (
            "class",
            Box::new(|s| s.enrichment.klass = Some(SectionClass::Climb)),
        ),
        ("lift", Box::new(|s| s.enrichment.is_lift = true)),
        (
            "section gain",
            Box::new(|s| s.elevation_gain_m = Some(12.0)),
        ),
        (
            "section grade",
            Box::new(|s| s.avg_grade_percent = Some(4.0)),
        ),
        ("range", Box::new(|s| s.representative_range = Some((2, 9)))),
    ];
    for (name, edit) in edits {
        assert_eq!(moved(edit), (false, true), "{name}");
    }
}

#[test]
fn a_float_one_bit_apart_moves_the_digest() {
    let (_, moved) = moved(|s| {
        s.enrichment.max_grade_percent = Some(f64::from_bits(5.0f64.to_bits() + 1));
    });
    assert!(moved);
}

#[test]
fn absent_and_zero_differ() {
    let (_, moved) = moved(|s| s.enrichment.elevation_loss_m = Some(0.0));
    assert!(moved);
}
