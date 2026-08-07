//! Plateau-adjacent sanity net for [`Tunables`]. Production always runs
//! `Tunables::DEFAULT`, and each default is documented as sitting on a
//! plateau rather than a peak fitted to one athlete's corpus, yet no
//! CI test ever ran the detector off the defaults. This net holds the
//! one-step neighbourhood of the integer pass tunables on robust
//! synthetic shapes: every neighbour must detect without panicking and
//! keep the catalogue invariants, and where the plateau genuinely holds
//! on these shapes the section count must not move either.

mod shapes;

use std::collections::HashMap;
use tracematch::geo_utils::haversine_distance;
use tracematch::{
    FrequentSection, GpsPoint, SectionConfig, Tunables, detect_sections_unified_tuned,
};

fn detect_with(tracks: &[(String, Vec<GpsPoint>)], tun: &Tunables) -> Vec<FrequentSection> {
    detect_sections_unified_tuned(
        tracks,
        &[],
        &shapes::pooled(tracks),
        &SectionConfig::default(),
        tun,
    )
}

/// One-step neighbours of the integer tunables that shape the pass
/// walk: the away gap, the cut window and its majority, and the lateral
/// reach. The float tunables (lift calibration, cluster gap) have
/// corpus-measured working bands and are the lab's territory, not a
/// unit net's.
fn neighbours() -> Vec<(&'static str, Tunables)> {
    let d = Tunables::DEFAULT;
    vec![
        (
            "pass_away_cells 4",
            Tunables {
                pass_away_cells: 4,
                ..d
            },
        ),
        (
            "pass_away_cells 6",
            Tunables {
                pass_away_cells: 6,
                ..d
            },
        ),
        (
            "pass_window 4",
            Tunables {
                pass_window: 4,
                ..d
            },
        ),
        (
            "pass_window 6",
            Tunables {
                pass_window: 6,
                ..d
            },
        ),
        (
            "pass_needed 2",
            Tunables {
                pass_needed: 2,
                ..d
            },
        ),
        (
            "pass_needed 4",
            Tunables {
                pass_needed: 4,
                ..d
            },
        ),
        ("reach 2", Tunables { reach: 2, ..d }),
    ]
}

// ------------------------------------------------------------- helpers

fn min_dist(p: &GpsPoint, line: &[GpsPoint]) -> f64 {
    line.iter()
        .map(|q| haversine_distance(p, q))
        .fold(f64::INFINITY, f64::min)
}

/// Share of a polyline's points that revisit ground the same polyline
/// already covered (index gap > 15, within 20 m, same elevation level).
fn self_overlap_frac(polyline: &[GpsPoint]) -> f64 {
    if polyline.len() < 20 {
        return 0.0;
    }
    let mut overlapping = 0usize;
    let mut total = 0usize;
    for i in (0..polyline.len()).step_by(3) {
        total += 1;
        let p = &polyline[i];
        let near = polyline.iter().enumerate().any(|(j, q)| {
            if (j as i64 - i as i64).abs() <= 15 || haversine_distance(p, q) >= 20.0 {
                return false;
            }
            match (p.elevation, q.elevation) {
                (Some(a), Some(b)) => (a - b).abs() < 15.0,
                _ => true,
            }
        });
        if near {
            overlapping += 1;
        }
    }
    overlapping as f64 / total.max(1) as f64
}

/// The catalogue invariants every neighbour must hold: real
/// single-passed geometry cut from an input trace, and corridor-disjoint
/// sections.
fn assert_catalogue_invariants(
    name: &str,
    tracks: &[(String, Vec<GpsPoint>)],
    sections: &[FrequentSection],
) {
    let by_id: HashMap<&str, &Vec<GpsPoint>> =
        tracks.iter().map(|(id, pts)| (id.as_str(), pts)).collect();
    for s in sections {
        assert!(!s.polyline.is_empty(), "{name} {}: empty polyline", s.id);
        assert!(s.distance_meters > 0.0, "{name} {}: no length", s.id);
        let rep = by_id
            .get(s.representative_activity_id.as_str())
            .unwrap_or_else(|| panic!("{name} {}: representative not an input track", s.id));
        let worst = s
            .polyline
            .iter()
            .map(|p| min_dist(p, rep))
            .fold(0.0, f64::max);
        assert!(
            worst < 25.0,
            "{name} {}: polyline strays {worst:.0} m from its representative trace",
            s.id
        );
        let so = self_overlap_frac(&s.polyline);
        assert!(so < 0.25, "{name} {}: self-overlap {so:.2}", s.id);
    }
    for a in sections {
        for b in sections {
            if a.id == b.id {
                continue;
            }
            let shared_acts = a
                .activity_ids
                .iter()
                .filter(|x| b.activity_ids.contains(x))
                .count();
            if 2 * shared_acts < a.activity_ids.len().min(b.activity_ids.len()) {
                continue;
            }
            let within = a
                .polyline
                .iter()
                .step_by(3)
                .filter(|p| min_dist(p, &b.polyline) < 100.0)
                .count() as f64
                / a.polyline.iter().step_by(3).count().max(1) as f64;
            assert!(
                within < 0.6,
                "{name} {} vs {}: {:.0}% shared corridor",
                a.id,
                b.id,
                within * 100.0
            );
        }
    }
}

// --------------------------------------------------------------- tests

#[test]
fn commuter_corridor_holds_across_pass_neighbours() {
    // The commuter road has no laps, no forks, and no usage change, so
    // no pass tunable should have anything to decide: one section at
    // the default. A neighbour that reads the corner as a re-entry
    // (reach 2 triples the lateral reach) may split the road's passes
    // there and draw it as a two-link chain — what it must never do is
    // lose the road: before remainder re-queue that same neighbour
    // silently dropped a 560 m stretch of a 12-track road, and a
    // count-only assertion could not see it.
    let tracks = shapes::persona_commuter();
    let base = detect_with(&tracks, &Tunables::DEFAULT);
    assert_catalogue_invariants("default", &tracks, &base);
    assert_eq!(base.len(), 1, "one commuter corridor at the default");
    let spine = metre_road_samples();

    for (name, tun) in neighbours() {
        let sections = detect_with(&tracks, &tun);
        assert_catalogue_invariants(name, &tracks, &sections);
        assert!(
            sections.len() <= 2,
            "{name}: the commuter corridor shattered ({} sections)",
            sections.len()
        );
        let cov = spine
            .iter()
            .filter(|p| sections.iter().any(|s| min_dist(p, &s.polyline) < 90.0))
            .count() as f64
            / spine.len() as f64;
        assert!(
            cov >= 0.95,
            "{name}: commuter road ground lost (coverage {:.0}%)",
            cov * 100.0
        );
    }
}

fn metre_road_samples() -> Vec<GpsPoint> {
    shapes::densify(&[(0.0, 0.0), (600.0, 0.0), (600.0, 500.0), (1300.0, 500.0)])
        .iter()
        .step_by(8)
        .map(|&(x, y)| shapes::to_gps(x, y))
        .collect()
}

#[test]
fn oval_stem_holds_across_pass_neighbours() {
    // The oval's laps are exactly what the pass tunables measure, so
    // this shape exercises them for real: the stem-oval usage boundary
    // and the two-section catalogue must survive every one-step
    // neighbour, and the oval must stay one lap.
    let tracks = shapes::oval_stem(6);
    let base = detect_with(&tracks, &Tunables::DEFAULT);
    assert_catalogue_invariants("default", &tracks, &base);
    assert_eq!(base.len(), 2, "stem + oval at the default");

    for (name, tun) in neighbours() {
        let sections = detect_with(&tracks, &tun);
        assert_catalogue_invariants(name, &tracks, &sections);
        assert_eq!(
            sections.len(),
            base.len(),
            "{name}: the oval-stem catalogue is not on the plateau"
        );
        let oval = sections
            .iter()
            .map(|s| s.distance_meters)
            .fold(0.0, f64::max);
        assert!(
            oval < 1600.0,
            "{name}: longest section {oval:.0} m, laps leaked into the cut"
        );
    }
}
