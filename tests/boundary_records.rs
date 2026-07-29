//! Contracts for the boundary records the detector emits beside its
//! catalogue. `UsageChange`, `Fork`, and `LowSupport` are asserted in
//! `unified_contracts.rs`; this file covers the remaining kinds
//! (`Backoff`, `Trim`, and `NoSinglePass`), each on synthetic ground
//! built to provably produce it, and checks that the record explains
//! what the catalogue shows.

mod shapes;

use std::collections::HashMap;
use tracematch::geo_utils::haversine_distance;
use tracematch::{
    BoundaryReason, FrequentSection, GpsPoint, SectionConfig, Tunables, UnifiedDetection,
    detect_sections_unified_explained,
};

fn detect(tracks: &[(String, Vec<GpsPoint>)]) -> UnifiedDetection {
    detect_sections_unified_explained(
        tracks,
        &[],
        &shapes::pooled(tracks),
        &SectionConfig::default(),
        &Tunables::DEFAULT,
    )
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

/// The catalogue invariants every shape must hold: real single-passed
/// geometry cut from an input trace, and corridor-disjoint sections.
fn assert_catalogue_invariants(tracks: &[(String, Vec<GpsPoint>)], sections: &[FrequentSection]) {
    let by_id: HashMap<&str, &Vec<GpsPoint>> =
        tracks.iter().map(|(id, pts)| (id.as_str(), pts)).collect();
    for s in sections {
        assert!(!s.polyline.is_empty(), "{}: empty polyline", s.id);
        assert!(s.distance_meters > 0.0, "{}: no length", s.id);
        let rep = by_id
            .get(s.representative_activity_id.as_str())
            .unwrap_or_else(|| panic!("{}: representative not an input track", s.id));
        let worst = s
            .polyline
            .iter()
            .map(|p| min_dist(p, rep))
            .fold(0.0, f64::max);
        assert!(
            worst < 25.0,
            "{}: polyline strays {:.0} m from its representative trace",
            s.id,
            worst
        );
        let so = self_overlap_frac(&s.polyline);
        assert!(so < 0.25, "{}: self-overlap {:.2}", s.id, so);
    }
    for a in sections {
        for b in sections {
            if a.id == b.id {
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
                "{} vs {}: {:.0}% shared corridor",
                a.id,
                b.id,
                within * 100.0
            );
        }
    }
}

// -------------------------------------------------------------- ground

/// `outings` one-way west-east passes along the line `y`. Placed so a
/// 90 m-apart pair sits in adjacent coverage rows (cell = 100 m at the
/// default proximity): distinct cells, so the quieter line becomes its
/// own candidate, yet inside the one-cell representation tolerance the
/// selection backoff probes with.
fn lane(tag: &str, y: f64, outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    (0..outings)
        .map(|i| {
            let path = shapes::densify(&[(0.0, y), (1500.0, y)]);
            (
                format!("{tag}_{i}"),
                shapes::track(&shapes::wobble(
                    &path,
                    shapes::HUMAN_WOBBLE_M,
                    i as f64 * 1.7,
                )),
            )
        })
        .collect()
}

/// `outings` passes of an elbow: west along `y = 140` beside the busy
/// lane, then north on ground of its own.
fn elbow(tag: &str, outings: usize) -> Vec<(String, Vec<GpsPoint>)> {
    (0..outings)
        .map(|i| {
            let path = shapes::densify(&[(0.0, 140.0), (800.0, 140.0), (800.0, 800.0)]);
            (
                format!("{tag}_{i}"),
                shapes::track(&shapes::wobble(
                    &path,
                    shapes::HUMAN_WOBBLE_M,
                    i as f64 * 1.7,
                )),
            )
        })
        .collect()
}

/// `outings` spins of `laps` laps around a circle too tight for the
/// cell-event walk to cut: a lap re-touches every fine cell within the
/// dwell window, so each portion stays a multi-lap composite.
fn mill(tag: &str, outings: usize, laps: usize) -> Vec<(String, Vec<GpsPoint>)> {
    use std::f64::consts::PI;
    (0..outings)
        .map(|i| {
            let path = shapes::arc(50.0, 0.0, 22.3, 0.0, laps as f64 * 2.0 * PI);
            (
                format!("{tag}_{i}"),
                shapes::track(&shapes::wobble(
                    &path,
                    shapes::HUMAN_WOBBLE_M,
                    i as f64 * 1.7,
                )),
            )
        })
        .collect()
}

// --------------------------------------------------------------- tests

#[test]
fn near_duplicate_corridor_backs_off_with_a_record() {
    // A busy lane and a quieter twin 90 m away. The twin is its own
    // candidate (own cells, own traffic) but its whole probe runs within
    // a cell of the accepted busy line, so it is represented ground: one
    // section on the busy lane, no section on the twin, and a Backoff
    // record on the twin whose numbers say the entire probe was covered.
    let mut tracks = lane("busy", 50.0, 6);
    tracks.extend(lane("twin", 140.0, 4));
    let out = detect(&tracks);
    assert_catalogue_invariants(&tracks, &out.sections);
    assert_eq!(out.sections.len(), 1, "only the busy lane may stand");

    let busy_line: Vec<GpsPoint> = (0..=150)
        .map(|k| shapes::to_gps(k as f64 * 10.0, 50.0))
        .collect();
    let twin_mid = shapes::to_gps(750.0, 140.0);
    let s = &out.sections[0];
    let on_busy = s
        .polyline
        .iter()
        .filter(|p| min_dist(p, &busy_line) < 10.0)
        .count() as f64
        / s.polyline.len() as f64;
    assert!(
        on_busy >= 0.9,
        "{}: the surviving section must lie on the busy lane ({:.0}%)",
        s.id,
        on_busy * 100.0
    );
    assert!(
        min_dist(&twin_mid, &s.polyline) > 50.0,
        "twin ground must not be re-emitted"
    );

    let backoff = out
        .boundaries
        .iter()
        .find(|r| matches!(r.reason, BoundaryReason::Backoff { .. }))
        .unwrap_or_else(|| panic!("no backoff record: {:?}", out.boundaries));
    let BoundaryReason::Backoff {
        represented,
        probed,
        score_metres,
    } = backoff.reason
    else {
        unreachable!()
    };
    assert!(probed > 0);
    assert_eq!(
        represented, probed,
        "the twin's whole probe runs within a cell of the accepted line"
    );
    assert!(
        (3000.0..9000.0).contains(&score_metres),
        "score {score_metres:.0} m should be the twin's own portion metres (4 passes x ~1.5 km)"
    );
    let at = GpsPoint::new(backoff.latitude, backoff.longitude);
    assert!(
        haversine_distance(&at, &twin_mid) < 150.0,
        "the record must sit on the refused twin, not the winner"
    );
    assert!(
        out.boundaries
            .iter()
            .all(|r| !matches!(r.reason, BoundaryReason::Trim { .. })),
        "a wholly represented duplicate is refused, never trimmed: {:?}",
        out.boundaries
    );
}

#[test]
fn partly_represented_candidate_is_trimmed_to_its_own_run() {
    // The elbow shares its west leg with the busy lane's corridor (90 m
    // away, represented) then heads north on ground of its own. The
    // candidate is cut back to the north remnant and stands there, and
    // the Trim record carries the kept and dropped metres of that cut.
    let mut tracks = lane("busy", 50.0, 6);
    tracks.extend(elbow("elbow", 4));
    let out = detect(&tracks);
    assert_catalogue_invariants(&tracks, &out.sections);
    assert_eq!(out.sections.len(), 2, "busy lane + trimmed remnant");

    for y in [300.0, 500.0, 700.0] {
        let p = shapes::to_gps(800.0, y);
        assert!(
            out.sections
                .iter()
                .any(|s| min_dist(&p, &s.polyline) < 130.0),
            "the elbow's own north run must survive at y={y}"
        );
    }
    for x in [100.0, 300.0, 500.0] {
        let p = shapes::to_gps(x, 140.0);
        assert!(
            out.sections
                .iter()
                .all(|s| min_dist(&p, &s.polyline) > 60.0),
            "the represented west leg must not be re-emitted at x={x}"
        );
    }

    let trim = out
        .boundaries
        .iter()
        .find(|r| matches!(r.reason, BoundaryReason::Trim { .. }))
        .unwrap_or_else(|| panic!("no trim record: {:?}", out.boundaries));
    let BoundaryReason::Trim {
        kept_metres,
        dropped_metres,
    } = trim.reason
    else {
        unreachable!()
    };
    assert!(
        (400.0..800.0).contains(&kept_metres),
        "kept {kept_metres:.0} m should be the ~650 m north remnant"
    );
    assert!(
        (500.0..1100.0).contains(&dropped_metres),
        "dropped {dropped_metres:.0} m should be the ~800 m represented west leg"
    );
    let corner = shapes::to_gps(800.0, 150.0);
    let at = GpsPoint::new(trim.latitude, trim.longitude);
    assert!(
        haversine_distance(&at, &corner) < 200.0,
        "the cut must sit where the elbow leaves the represented corridor"
    );
}

#[test]
fn milling_ground_with_no_single_pass_is_refused_with_a_record() {
    // Six outings each spin four laps of a 140 m circle. A lap
    // re-touches every fine cell inside the dwell window, so the
    // cell-event walk never cuts and every contributing portion is a
    // multi-lap composite. Rule 6 refuses to render any of them: the
    // ground backs off as a blob and the record carries the numbers.
    let tracks = mill("mill", 6, 4);
    let out = detect(&tracks);
    assert!(
        out.sections.is_empty(),
        "ground with no single-passed cutout must not emit: {:?}",
        out.sections
            .iter()
            .map(|s| (&s.id, s.distance_meters))
            .collect::<Vec<_>>()
    );

    let rec = out
        .boundaries
        .iter()
        .find(|r| matches!(r.reason, BoundaryReason::NoSinglePass { .. }))
        .unwrap_or_else(|| panic!("no no-single-pass record: {:?}", out.boundaries));
    let BoundaryReason::NoSinglePass {
        best_penalty,
        portions,
    } = rec.reason
    else {
        unreachable!()
    };
    assert_eq!(portions, 6, "every outing contributed a (refused) portion");
    assert!(
        best_penalty > Tunables::DEFAULT.self_pass_max && best_penalty <= 1.0,
        "best penalty {best_penalty:.2} must sit above the render floor, which is why no pass rendered"
    );
    let centre = shapes::to_gps(50.0, 0.0);
    let at = GpsPoint::new(rec.latitude, rec.longitude);
    assert!(
        haversine_distance(&at, &centre) < 80.0,
        "the record must sit on the refused mill"
    );
}
