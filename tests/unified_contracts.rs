//! Executable contracts for the unified section detector.
//!
//! Each test pins a rule or a locked invariant of the algorithm
//! (`src/sections/unified.rs`) on deterministic synthetic ground, so a
//! behaviour change that breaks a ruling fails CI instead of waiting for
//! a corpus review. The assumption register mapping v1 failure modes to
//! these tests lives in `tests/ASSUMPTIONS.md`.

mod shapes;

use std::collections::HashMap;
use tracematch::geo_utils::haversine_distance;
use tracematch::{Direction, FrequentSection, GpsPoint, SectionConfig, detect_sections_unified};

fn config() -> SectionConfig {
    SectionConfig::default()
}

fn detect(tracks: &[(String, Vec<GpsPoint>)]) -> Vec<FrequentSection> {
    detect_sections_unified(tracks, &[], &shapes::pooled(tracks), &config())
}

// ------------------------------------------------------------- helpers

fn min_dist(p: &GpsPoint, line: &[GpsPoint]) -> f64 {
    line.iter()
        .map(|q| haversine_distance(p, q))
        .fold(f64::INFINITY, f64::min)
}

/// Share of `samples` lying within `tol` of any section polyline.
fn coverage(samples: &[GpsPoint], sections: &[FrequentSection], tol: f64) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let hit = samples
        .iter()
        .filter(|p| sections.iter().any(|s| min_dist(p, &s.polyline) < tol))
        .count();
    hit as f64 / samples.len() as f64
}

/// Share of a polyline's points that revisit ground the same polyline
/// already covered (index gap > 15, within 20 m, same elevation level).
/// Single-passed real traces stay near zero; a laps-included cut does
/// not. The elevation guard mirrors the detector's pass rule so hairpin
/// legs stacked in plan do not read as re-passes.
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

/// One line per section on stderr: visible only when a test fails.
fn dump(sections: &[FrequentSection]) {
    for s in sections {
        let a = s.polyline.first().map(|p| (p.latitude, p.longitude));
        eprintln!(
            "  {}  {:.0} m  visits {}  rep {}  anchor {:?}",
            s.id, s.distance_meters, s.visit_count, s.representative_activity_id, a
        );
    }
}

fn metre_samples(waypoints: &[(f64, f64)], step: f64) -> Vec<GpsPoint> {
    let dense = shapes::densify(waypoints);
    let stride = (step / shapes::SPACING_M).round().max(1.0) as usize;
    dense
        .iter()
        .step_by(stride)
        .map(|&(x, y)| shapes::to_gps(x, y))
        .collect()
}

/// Invariants that hold for every catalogue on every shape:
/// real-trace geometry (rule 5), single-passed geometry, and
/// corridor-disjointness at the backoff tolerance (rule 6).
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
            "{}: polyline strays {:.0} m from its representative trace — geometry must be a real single cutout, never synthesised",
            s.id,
            worst
        );
        let so = self_overlap_frac(&s.polyline);
        assert!(
            so < 0.25,
            "{}: self-overlap {:.2} — geometry must be single-passed",
            s.id,
            so
        );
    }
    for a in sections {
        for b in sections {
            if a.id == b.id {
                continue;
            }
            // Corridor-disjointness binds per population: two lines on
            // the same ground are a duplicate only when they carry the
            // same users. A distinct population's parallel path — the
            // other bank of a river — is its own corridor.
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
                "{} vs {}: {:.0}% shared corridor — represented ground must be trimmed, not re-emitted",
                a.id,
                b.id,
                within * 100.0
            );
        }
    }
}

/// Order-insensitive catalogue identity: everything that defines the
/// catalogue as a pure function of the activity set.
fn normalise(sections: &[FrequentSection]) -> String {
    let mut rows: Vec<String> = sections
        .iter()
        .map(|s| {
            let coords: Vec<(i64, i64)> = s
                .polyline
                .iter()
                .map(|p| {
                    (
                        (p.latitude * 1e6).round() as i64,
                        (p.longitude * 1e6).round() as i64,
                    )
                })
                .collect();
            let mut acts = s.activity_ids.clone();
            acts.sort();
            acts.dedup();
            format!(
                "{}|{}|{}|{}|{:.0}|{:?}",
                s.id,
                s.representative_activity_id,
                s.visit_count,
                acts.join(","),
                s.distance_meters,
                coords
            )
        })
        .collect();
    rows.sort();
    rows.join("\n")
}

// ------------------------------------------------- rule 4: boundaries

#[test]
fn oval_and_stem_split_at_the_usage_change() {
    // Scenario: stem to an oval, three laps, stem home. The stem is
    // passed twice per outing, the oval three times.
    // Expected behaviour: the pass-class change at the entrance is the
    // boundary; the oval's geometry is one lap, never three.
    let tracks = shapes::oval_stem(6);
    let sections = detect(&tracks);
    assert_catalogue_invariants(&tracks, &sections);
    assert_eq!(sections.len(), 2, "expected stem + oval");

    let centre = shapes::to_gps(800.0, 0.0);
    let on_oval = |s: &FrequentSection| {
        let mid: f64 = s
            .polyline
            .iter()
            .map(|p| haversine_distance(p, &centre))
            .sum::<f64>()
            / s.polyline.len() as f64;
        (140.0..260.0).contains(&mid)
    };
    let oval = sections.iter().find(|s| on_oval(s)).expect("oval section");
    assert!(
        (900.0..1600.0).contains(&oval.distance_meters),
        "oval length {:.0} m should be one lap (~1250 m)",
        oval.distance_meters
    );
    let stem = sections.iter().find(|s| !on_oval(s)).expect("stem section");
    assert!(
        (400.0..850.0).contains(&stem.distance_meters),
        "stem length {:.0} m should be one pass (~600 m)",
        stem.distance_meters
    );

    let stem_mid = shapes::to_gps(300.0, 0.0);
    let oval_east = shapes::to_gps(1000.0, 0.0);
    for s in &sections {
        let both =
            min_dist(&stem_mid, &s.polyline) < 100.0 && min_dist(&oval_east, &s.polyline) < 100.0;
        assert!(!both, "{}: spans the stem-oval boundary", s.id);
    }
}

#[test]
fn lollipop_splits_at_the_mouth() {
    // Stick passed twice per outing, head once: the mouth is a visible
    // usage boundary and the stick's geometry is one pass, not two.
    let tracks = shapes::lollipop(6);
    let sections = detect(&tracks);
    assert_catalogue_invariants(&tracks, &sections);
    assert_eq!(sections.len(), 2, "expected stick + head");

    let centre = shapes::to_gps(950.0, 0.0);
    // The joint snap may extend the head a few points down the stick to
    // close the mouth seam, so the head is the section MOSTLY on the
    // circle, not entirely on it.
    let head = sections
        .iter()
        .find(|s| {
            let on_circle = s
                .polyline
                .iter()
                .filter(|p| (90.0..230.0).contains(&haversine_distance(p, &centre)))
                .count();
            on_circle * 10 >= s.polyline.len() * 7
        })
        .expect("head section");
    assert!((650.0..1250.0).contains(&head.distance_meters));
    let stick = sections.iter().find(|s| s.id != head.id).unwrap();
    assert!(
        (500.0..1000.0).contains(&stick.distance_meters),
        "stick length {:.0} m should be one pass (~800 m)",
        stick.distance_meters
    );
}

#[test]
fn chain_members_share_one_reference_and_meet_exactly() {
    // Rule 7: the stick and head tile one physical line and every
    // outing covers both, so both sections must cut their geometry
    // from the same activity and their joint must be a shared trace
    // point — never splices from different days with an offset seam.
    let tracks = shapes::lollipop(6);
    let sections = detect(&tracks);
    assert_catalogue_invariants(&tracks, &sections);
    assert_eq!(sections.len(), 2, "expected stick + head");
    assert_eq!(
        sections[0].representative_activity_id, sections[1].representative_activity_id,
        "chain members must share one covering reference"
    );
    let ends = |s: &FrequentSection| [*s.polyline.first().unwrap(), *s.polyline.last().unwrap()];
    let joint = ends(&sections[0])
        .iter()
        .flat_map(|a| ends(&sections[1]).map(|b| haversine_distance(a, &b)))
        .fold(f64::INFINITY, f64::min);
    assert!(
        joint < 0.5,
        "joint gap {joint:.1} m — chain joints must meet on a shared trace point"
    );
}

// ----------------------------------- v1 assumption: whole-track pooling

#[test]
fn deviation_emerges_from_unique_outings() {
    // Eight globally distinct activities share only one straight. v1
    // pooled whole activities, so shared portions inside unique rides
    // never surfaced. The straight must emerge on its own.
    let tracks = shapes::deviation_straight(8);
    let sections = detect(&tracks);
    assert_catalogue_invariants(&tracks, &sections);
    assert!(!sections.is_empty(), "shared straight must emerge");
    assert!(sections.len() <= 2, "one corridor, got {}", sections.len());
    let straight = metre_samples(&[(50.0, 0.0), (1150.0, 0.0)], 50.0);
    assert!(
        coverage(&straight, &sections, 120.0) >= 0.8,
        "straight under-covered"
    );
    for s in &sections {
        for p in s.polyline.iter().step_by(5) {
            let x_ok = min_dist(p, &metre_samples(&[(-300.0, 0.0), (1500.0, 0.0)], 25.0)) < 350.0;
            assert!(
                x_ok,
                "{}: section ground far from the shared straight",
                s.id
            );
        }
    }
}

// ------------------------------- rules 5+6: real geometry, never merged

#[test]
fn usage_cliff_ends_the_busy_corridor() {
    // Scenario: ten outings share a corridor; three continue past its
    // end for another 1.5 km while seven disperse on unique exits. No
    // turnaround, no shared third corridor — the visible reason the
    // busy section ends is that most of its traffic stops there.
    // Expected behaviour: no section spans the cliff; the busy stretch
    // keeps its own honest visit count and any quiet-tail section
    // carries only its minority's.
    let tracks = shapes::cliff_tail(10, 4);
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);
    assert!(!sections.is_empty());
    assert_majority_rendered(&tracks, &sections);

    let busy_core = metre_samples(&[(200.0, 0.0), (1300.0, 0.0)], 10.0);
    let quiet_deep = metre_samples(&[(1900.0, 0.0), (2900.0, 0.0)], 10.0);
    assert!(
        coverage(&busy_core, &sections, 100.0) >= 0.8,
        "the busy corridor must surface"
    );
    assert!(
        coverage(&quiet_deep, &sections, 100.0) >= 0.8,
        "the minority tail is real repeated ground and must surface on its own"
    );
    for s in &sections {
        let in_busy = busy_core.iter().any(|p| min_dist(p, &s.polyline) < 100.0);
        let in_quiet = quiet_deep.iter().any(|p| min_dist(p, &s.polyline) < 100.0);
        assert!(
            !(in_busy && in_quiet),
            "{}: spans the usage cliff ({} visits)",
            s.id,
            s.visit_count
        );
        if in_busy {
            assert!(
                s.visit_count >= 9,
                "{}: busy section under-counted ({} visits)",
                s.id,
                s.visit_count
            );
        }
        if in_quiet {
            assert!(
                s.visit_count <= 5,
                "{}: quiet tail inherited the corridor's visits ({})",
                s.id,
                s.visit_count
            );
        }
    }
}

#[test]
fn scattered_traffic_is_still_one_corridor() {
    // Scenario: twelve outings ride one corridor, each drifting across
    // a wide lane band, so any single evidence cell records only the
    // subset whose lane crossed it. The travelling population is the
    // same everywhere along the corridor.
    // Expected behaviour: per-cell count noise is not a usage boundary.
    // The corridor surfaces whole with an honest count instead of
    // shattering into fragments that all die below the support floor.
    let tracks = shapes::scattered_corridor(12);
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);

    let spine = metre_samples(&[(100.0, 0.0), (2800.0, 0.0)], 10.0);
    assert!(
        coverage(&spine, &sections, 90.0) >= 0.9,
        "scattered corridor ground must survive whole (coverage {:.0}%)",
        coverage(&spine, &sections, 90.0) * 100.0
    );
    let busiest = sections.iter().map(|s| s.visit_count).max().unwrap_or(0);
    assert!(
        busiest >= 9,
        "the corridor's population must be counted, not one cell's sample (max {} visits)",
        busiest
    );
}

#[test]
fn corridor_no_one_runs_end_to_end_is_still_covered() {
    // Scenario: a 4 km valley corridor where every outing covers only
    // its own 1.5 km window, joining and leaving at its own points.
    // Interior traffic is 8-14 tracks and only ever changes by one
    // track at a join, so no boundary fires and the ground is one
    // corridor — while the longest single pass spans barely a third.
    // Expected behaviour: a section draws one real pass, so the
    // corridor is covered by a chain of honest single-pass sections.
    // Ground a section owns but cannot draw must re-enter the
    // candidate pool, not vanish behind the one rendered line.
    let tracks = shapes::sliding_corridor(24);
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);

    let interior = metre_samples(&[(800.0, 0.0), (3200.0, 0.0)], 40.0);
    let cov = coverage(&interior, &sections, 90.0);
    assert!(
        cov >= 0.9,
        "busy interior must stay covered by the section chain (coverage {:.0}%)",
        cov * 100.0
    );
}

#[test]
fn displaced_render_does_not_pinch_the_parallel_street() {
    // Scenario: a busy street and a quieter parallel street 180 m
    // away, plus three outings that swap across for the middle stretch.
    // The swap portions run unbroken through the busy street's capture
    // ring, so its longest pass walks the quiet street's middle while
    // the drawn line stays on the busy street (rule B).
    // Expected behaviour: ground is occupied by the line a section
    // SHOWS. The quiet street keeps its whole extent — its middle must
    // not back off against a line nobody sees.
    let tracks = shapes::parallel_street(16, 5, 3);
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);
    assert_majority_rendered(&tracks, &sections);

    let busy_mid = metre_samples(&[(200.0, 0.0), (1400.0, 0.0)], 10.0);
    let quiet_mid = metre_samples(&[(700.0, 180.0), (900.0, 180.0)], 10.0);
    assert!(
        coverage(&busy_mid, &sections, 60.0) >= 0.8,
        "the busy street must surface"
    );
    assert!(
        coverage(&quiet_mid, &sections, 60.0) >= 0.8,
        "quiet street's middle backed off against unrendered ground (coverage {:.0}%)",
        coverage(&quiet_mid, &sections, 60.0) * 100.0
    );
}

#[test]
fn minority_braid_strand_stays_off_the_body() {
    // Scenario: fourteen outings run one body; five swap onto a strand
    // 180 m north for the middle stretch. The strand is real minority
    // ground; the body keeps the majority everywhere.
    // Expected behaviour: no emitted line mixes the two — a body
    // section stays on the body with the majority's count, and any
    // strand section carries only its minority's.
    let tracks = shapes::minority_braid(14, 5);
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);
    assert_majority_rendered(&tracks, &sections);

    let body_mid = metre_samples(&[(550.0, 0.0), (1250.0, 0.0)], 10.0);
    let strand_mid = metre_samples(&[(550.0, 180.0), (1250.0, 180.0)], 10.0);
    assert!(
        coverage(&body_mid, &sections, 100.0) >= 0.8,
        "the body must surface"
    );
    for s in &sections {
        let on_body = body_mid.iter().any(|p| min_dist(p, &s.polyline) < 60.0);
        let on_strand = strand_mid.iter().any(|p| min_dist(p, &s.polyline) < 60.0);
        assert!(
            !(on_body && on_strand),
            "{}: one line walks both braid strands",
            s.id
        );
        if on_strand {
            assert!(
                s.visit_count <= 6,
                "{}: minority strand inherited the body's visits ({})",
                s.id,
                s.visit_count
            );
        }
        if on_body {
            assert!(
                s.visit_count >= 9,
                "{}: body section under-counted ({} visits)",
                s.id,
                s.visit_count
            );
        }
    }
}

/// Every rendered point must lie on ground that at least half the
/// section's own contributors traverse: the drawn line is where the
/// majority went, never a minority variant or a private tail.
///
/// Both sides count distinct tracks. `visit_count` counts passes, so an
/// out-and-back circuit would put twice the traversals against the same
/// population and read every faithful line as minority ground.
fn assert_majority_rendered(tracks: &[(String, Vec<GpsPoint>)], sections: &[FrequentSection]) {
    for s in sections {
        let mut low = 0usize;
        let mut total = 0usize;
        let contributors = s.activity_ids.len();
        for p in s.polyline.iter().step_by(5) {
            total += 1;
            let sup = tracks
                .iter()
                .filter(|(_, pts)| pts.iter().any(|q| haversine_distance(p, q) < 50.0))
                .count();
            if (sup as f64) < 0.5 * contributors as f64 {
                low += 1;
            }
        }
        assert!(
            low * 10 <= total,
            "{}: {}/{} rendered points sit on minority ground ({} contributors)",
            s.id,
            low,
            total,
            contributors
        );
    }
}

#[test]
fn short_strand_fragment_does_not_bend_the_line() {
    // Scenario: fourteen outings on one busy line; five swap onto a
    // 200 m variant strand mid-way — real minority evidence, but far
    // too short to ever be a section of its own.
    // Expected behaviour: the fragment is not absorbed into the busy
    // sections around it — no rendered line leaves the majority path,
    // and no extent quietly carries the strand's ground.
    let tracks = shapes::short_strand(14, 5);
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);

    let body_mid = metre_samples(&[(400.0, 0.0), (720.0, 0.0)], 10.0);
    assert!(
        coverage(&body_mid, &sections, 100.0) >= 0.8,
        "the body must surface"
    );
    let strand_mid = metre_samples(&[(540.0, 120.0), (580.0, 120.0)], 10.0);
    for s in &sections {
        let on_strand = strand_mid.iter().any(|p| min_dist(p, &s.polyline) < 60.0);
        assert!(
            !on_strand,
            "{}: rendered line walks the minority strand",
            s.id
        );
    }
    assert_majority_rendered(&tracks, &sections);
}

#[test]
fn braid_variants_read_as_real_lines_not_a_midline() {
    // Two lanes 30 m apart, every outing rides both. Whatever extent is
    // emitted, its points must lie on one real lane at a time — a
    // consensus midline (~15 m from both) is the banned v1 geometry.
    let tracks = shapes::parallel_variants(6, 30.0);
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);
    assert!(!sections.is_empty());
    let lane_a = metre_samples(&[(0.0, -30.0), (0.0, 730.0)], 5.0);
    let lane_b = metre_samples(&[(30.0, -30.0), (30.0, 730.0)], 5.0);
    for s in &sections {
        let on_lane = s
            .polyline
            .iter()
            .filter(|p| min_dist(p, &lane_a).min(min_dist(p, &lane_b)) < 9.0)
            .count() as f64
            / s.polyline.len() as f64;
        assert!(
            on_lane >= 0.85,
            "{}: only {:.0}% of points sit on a real lane",
            s.id,
            on_lane * 100.0
        );
    }
}

#[test]
fn separated_variants_stay_distinct_ground() {
    // The same circuit with the lanes 180 m apart: both lines are real,
    // visible ground. They may emit as one circuit or two lines, but
    // never as an averaged midline and never with a bank missing.
    let tracks = shapes::parallel_variants(6, 180.0);
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);
    assert!((1..=2).contains(&sections.len()), "got {}", sections.len());
    let lane_a = metre_samples(&[(0.0, -30.0), (0.0, 730.0)], 5.0);
    let lane_b = metre_samples(&[(180.0, -30.0), (180.0, 730.0)], 5.0);
    let links = metre_samples(&[(-20.0, 700.0), (200.0, 700.0)], 5.0)
        .into_iter()
        .chain(metre_samples(&[(200.0, 0.0), (-20.0, 0.0)], 5.0))
        .collect::<Vec<_>>();
    assert!(coverage(&lane_a, &sections, 100.0) >= 0.6, "west lane lost");
    assert!(coverage(&lane_b, &sections, 100.0) >= 0.6, "east lane lost");
    for s in &sections {
        let real = s
            .polyline
            .iter()
            .filter(|p| {
                min_dist(p, &lane_a)
                    .min(min_dist(p, &lane_b))
                    .min(min_dist(p, &links))
                    < 12.0
            })
            .count() as f64
            / s.polyline.len() as f64;
        assert!(
            real >= 0.85,
            "{}: {:.0}% of points off every real line — midline geometry",
            s.id,
            real * 100.0
        );
    }
}

// ------------------------------------- rule 1: elevation-aware passes

#[test]
fn switchback_climb_survives_as_one_section() {
    // Hairpin legs 30 m apart in plan revisit coarse cells, but at
    // elevation levels 25 m apart — beyond the 15 m tolerance, so the
    // single-pass walk must not cut the climb into legs.
    let tracks = shapes::switchback_climb(6);
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);
    let climb_samples: Vec<GpsPoint> = (0..8)
        .flat_map(|leg| {
            let y = 30.0 * leg as f64 + 15.0;
            (0..=3).map(move |k| shapes::to_gps(50.0 * k as f64, y))
        })
        .collect();
    let climb = sections
        .iter()
        .find(|s| {
            let c = climb_samples
                .iter()
                .filter(|p| min_dist(p, &s.polyline) < 100.0)
                .count() as f64
                / climb_samples.len() as f64;
            c >= 0.7
        })
        .expect("switchback climb missing from the catalogue");
    assert!(
        climb.distance_meters >= 1000.0,
        "climb cut down to {:.0} m — legs must not fragment",
        climb.distance_meters
    );
}

// --------------------------------- rule 3+5: floors and fork boundaries

#[test]
fn grid_city_respects_floors_and_forks() {
    // Eight staircase commutes across a street grid. Minority branches
    // sit below their floors, so they are not section-worthy and their
    // fork boundaries dissolve: the corridor follows the through-flow
    // and its geometry is one real commute. What must hold: the shared
    // terminal blocks are represented, once-visited streets never
    // appear, and the count stays sane for eight overlapping routes.
    let tracks = shapes::grid_city();
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);
    assert!(
        (1..=6).contains(&sections.len()),
        "got {} sections",
        sections.len()
    );

    let covered = [
        shapes::to_gps(125.0, 0.0),   // start block, all eight routes
        shapes::to_gps(750.0, 690.0), // NE funnel block, all eight routes
    ];
    for p in &covered {
        assert!(
            sections.iter().any(|s| min_dist(p, &s.polyline) < 130.0),
            "shared terminal ground missing from the catalogue"
        );
    }
    // The guard is the matching corridor half-width (one evidence
    // cell): quiet ground is IN a section when a line runs within its
    // corridor. Remainder re-queue surfaces the 3-visit bottom-street
    // continuation, whose honest end may stop one ring short of the
    // quiet stretch — outside the corridor, but nearer than padding
    // would allow.
    let silent = [
        shapes::to_gps(625.0, 0.0),   // 1 visit: never hot ground
        shapes::to_gps(250.0, 625.0), // 1 visit
    ];
    for p in &silent {
        assert!(
            sections.iter().all(|s| min_dist(p, &s.polyline) > 100.0),
            "once-visited ground appeared in a section"
        );
    }
}

// ------------------------------------------- occasions: support floor

#[test]
fn one_trip_ground_is_not_a_section() {
    // Scenario: the same corridor walked twice — once as a single
    // weekend trip (two recordings 25 h apart), once as a fortnightly
    // habit. The evidence is identical; only the calendar differs.
    // Expected behaviour: a trip's files chain into one occasion and
    // one occasion is not repetition; the habit is.
    use tracematch::{Tunables, detect_sections_unified_dated};
    let tracks = shapes::plain_corridor(2);
    let detect_at = |epochs: &[(&str, i64)]| {
        let map: HashMap<String, i64> = epochs.iter().map(|&(id, e)| (id.to_string(), e)).collect();
        detect_sections_unified_dated(
            &tracks,
            &[],
            &shapes::pooled(&tracks),
            &map,
            &config(),
            &Tunables::DEFAULT,
        )
        .sections
    };
    const DAY: i64 = 86_400;

    let trip = detect_at(&[("cor_0", 0), ("cor_1", 25 * 3600)]);
    assert!(
        trip.is_empty(),
        "one trip minted a section: {:?}",
        trip.iter().map(|s| &s.id).collect::<Vec<_>>()
    );

    let habit = detect_at(&[("cor_0", 0), ("cor_1", 14 * DAY)]);
    assert_eq!(habit.len(), 1, "the fortnightly habit must surface");
    assert_eq!(habit[0].visit_count, 2, "visits stay real traversals");

    let undated = detect(&tracks);
    assert_eq!(undated.len(), 1, "dateless corpora keep activity counting");
}

#[test]
fn a_daily_habit_is_repetition_but_one_stay_is_not() {
    // Scenario: the same corridor walked every day. Ten days of it is
    // routine (the days stretch past one stay); six days of it is a
    // holiday's jogging loop — one visit to a place, however many
    // recordings the stay produced. Day gaps cannot draw this line
    // (trip days and commute days are both ~24 h apart); the span can.
    use tracematch::{Tunables, detect_sections_unified_dated};
    const DAY: i64 = 86_400;
    let habit = shapes::plain_corridor(10);
    let map: HashMap<String, i64> = (0..10)
        .map(|i| (format!("cor_{}", i), i as i64 * DAY))
        .collect();
    let out = detect_sections_unified_dated(
        &habit,
        &[],
        &shapes::pooled(&habit),
        &map,
        &config(),
        &Tunables::DEFAULT,
    )
    .sections;
    assert_eq!(out.len(), 1, "a daily habit must surface");
    assert_eq!(out[0].visit_count, 10);

    let stay = shapes::plain_corridor(6);
    let map: HashMap<String, i64> = (0..6)
        .map(|i| (format!("cor_{}", i), i as i64 * DAY))
        .collect();
    let out = detect_sections_unified_dated(
        &stay,
        &[],
        &shapes::pooled(&stay),
        &map,
        &config(),
        &Tunables::DEFAULT,
    )
    .sections;
    assert!(
        out.is_empty(),
        "a single stay's daily loop minted a section: {:?}",
        out.iter().map(|s| &s.id).collect::<Vec<_>>()
    );
}

#[test]
fn a_trip_plus_a_later_return_clears_the_floor() {
    // Three recordings: two on consecutive days (one trip), a third a
    // fortnight later. The trip collapses to one occasion; the return
    // makes two — the ground was genuinely returned to.
    use tracematch::{Tunables, detect_sections_unified_dated};
    let tracks = shapes::plain_corridor(3);
    const DAY: i64 = 86_400;
    let map: HashMap<String, i64> = [
        ("cor_0".to_string(), 0),
        ("cor_1".to_string(), 25 * 3600),
        ("cor_2".to_string(), 14 * DAY),
    ]
    .into();
    let out = detect_sections_unified_dated(
        &tracks,
        &[],
        &shapes::pooled(&tracks),
        &map,
        &config(),
        &Tunables::DEFAULT,
    )
    .sections;
    assert_eq!(out.len(), 1, "two occasions are repetition");
    assert_eq!(out[0].visit_count, 3, "all three traversals count");
}

// ------------------------------------------- rule 2: lift exclusion

#[test]
fn lift_ground_forms_no_section_but_the_piste_does() {
    // A cable-smooth straight 25% ascent, only ever ridden uphill, next
    // to a winding human descent. The lift line is ineligible ground;
    // the piste is a section.
    let tracks = shapes::lift_piste(6);
    let sections = detect(&tracks);
    dump(&sections);
    let view: Vec<(&str, &[GpsPoint])> = tracks
        .iter()
        .map(|(id, pts)| (id.as_str(), pts.as_slice()))
        .collect();
    eprintln!(
        "  confirmed lift spans: {:?}",
        tracematch::confirmed_lift_spans(&view, &[])
    );
    assert_catalogue_invariants(&tracks, &sections);
    let lift_line = metre_samples(&[(0.0, 0.0), (0.0, 900.0)], 25.0);
    for s in &sections {
        let on_lift = s
            .polyline
            .iter()
            .filter(|p| min_dist(p, &lift_line) < 80.0)
            .count() as f64
            / s.polyline.len() as f64;
        assert!(
            on_lift < 0.3,
            "{}: {:.0}% of its ground runs up the lift line",
            s.id,
            on_lift * 100.0
        );
    }
    let piste_samples: Vec<GpsPoint> = (0..=20)
        .map(|s| {
            let t = s as f64 / 20.0;
            shapes::to_gps(
                300.0 + 130.0 * (t * 5.5 * std::f64::consts::PI).sin(),
                900.0 * (1.0 - t),
            )
        })
        .collect();
    assert!(
        coverage(&piste_samples, &sections, 120.0) >= 0.6,
        "the human piste should be a section"
    );
}

// ----------------------------------------------------------- personas

#[test]
fn persona_casual_produces_no_sections() {
    // Eight one-off loops with no shared ground: nothing repeats, so
    // nothing may be invented.
    let tracks = shapes::persona_casual();
    let sections = detect(&tracks);
    assert!(
        sections.is_empty(),
        "one-off ground produced {} sections",
        sections.len()
    );
}

#[test]
fn persona_weekender_finds_only_the_favourite_loop() {
    let tracks = shapes::persona_weekender();
    let sections = detect(&tracks);
    assert_catalogue_invariants(&tracks, &sections);
    assert!((1..=3).contains(&sections.len()), "got {}", sections.len());
    let loop_samples: Vec<GpsPoint> = (0..24)
        .map(|k| {
            let a = std::f64::consts::PI + k as f64 / 24.0 * 2.0 * std::f64::consts::PI;
            shapes::to_gps(900.0 + 500.0 * a.cos(), 500.0 * a.sin())
        })
        .collect();
    assert!(
        coverage(&loop_samples, &sections, 120.0) >= 0.6,
        "favourite loop missing"
    );
    let one_off_samples = metre_samples(&[(-2500.0, 2000.0), (-2200.0, 2600.0)], 60.0);
    assert!(
        coverage(&one_off_samples, &sections, 120.0) < 0.2,
        "one-off ground must stay silent"
    );
}

#[test]
fn persona_commuter_is_one_unfragmented_corridor() {
    // Twelve traversals of the same two-corner road, half in each
    // direction. No fork, no usage change, no support edge: no reason
    // for any internal boundary the athlete could see.
    let tracks = shapes::persona_commuter();
    let sections = detect(&tracks);
    assert_catalogue_invariants(&tracks, &sections);
    assert_eq!(sections.len(), 1, "commute fragmented");
    let s = &sections[0];
    assert!(
        (1500.0..2200.0).contains(&s.distance_meters),
        "corridor length {:.0} m",
        s.distance_meters
    );
    let mut same = 0;
    let mut reverse = 0;
    for (_, pts) in &tracks {
        for (_, _, dir) in tracematch::find_all_track_portions(pts, &s.polyline, 200.0) {
            match dir {
                Direction::Same => same += 1,
                Direction::Reverse => reverse += 1,
                _ => {}
            }
        }
    }
    assert!(
        same >= 4 && reverse >= 4,
        "both directions must match one section (same {}, reverse {})",
        same,
        reverse
    );
}

#[test]
fn persona_racer_hill_is_single_passed() {
    // Four hill repeats per outing: eight passes of the same 400 m.
    // The hill is a section once; laps are traversals, not geometry.
    let tracks = shapes::persona_racer();
    let sections = detect(&tracks);
    assert_catalogue_invariants(&tracks, &sections);
    let hill_mid = shapes::to_gps(700.0, 0.0);
    let hill = sections
        .iter()
        .find(|s| min_dist(&hill_mid, &s.polyline) < 100.0)
        .expect("hill section missing");
    assert!(
        hill.distance_meters <= 700.0,
        "hill geometry {:.0} m — repeats leaked into the cut",
        hill.distance_meters
    );
    let approach_mid = shapes::to_gps(250.0, 0.0);
    for s in &sections {
        let both = min_dist(&approach_mid, &s.polyline) < 100.0
            && min_dist(&hill_mid, &s.polyline) < 100.0;
        assert!(!both, "{}: spans the approach-hill usage boundary", s.id);
    }
}

// ----------------------------------- rule 9: boundaries explain themselves

#[test]
fn boundaries_explain_the_cuts() {
    // Every surviving cut carries its mechanism and numbers as data.
    // The oval entrance is a usage change; the Y junction is a fork
    // with a worthy branch; a boundary-free corridor emits nothing.
    use tracematch::{BoundaryReason, Tunables, detect_sections_unified_explained};

    let tracks = shapes::oval_stem(6);
    let out = detect_sections_unified_explained(
        &tracks,
        &[],
        &shapes::pooled(&tracks),
        &config(),
        &Tunables::DEFAULT,
    );
    assert_eq!(out.sections.len(), 2);
    let entrance = shapes::to_gps(600.0, 0.0);
    let near_entrance = out.boundaries.iter().any(|r| {
        matches!(r.reason, BoundaryReason::UsageChange { .. })
            && haversine_distance(&GpsPoint::new(r.latitude, r.longitude), &entrance) < 250.0
    });
    assert!(
        near_entrance,
        "no usage-change record at the oval entrance: {:?}",
        out.boundaries
    );

    let tracks = shapes::fork_y(8);
    let out = detect_sections_unified_explained(
        &tracks,
        &[],
        &shapes::pooled(&tracks),
        &config(),
        &Tunables::DEFAULT,
    );
    assert_catalogue_invariants(&tracks, &out.sections);
    assert_eq!(out.sections.len(), 3, "trunk + two branches");
    let junction = shapes::to_gps(0.0, 1000.0);
    let fork_here = out.boundaries.iter().any(|r| {
        matches!(r.reason, BoundaryReason::Fork { .. })
            && haversine_distance(&GpsPoint::new(r.latitude, r.longitude), &junction) < 250.0
    });
    assert!(
        fork_here,
        "no fork record at the junction: {:?}",
        out.boundaries
    );

    let tracks = shapes::persona_commuter();
    let out = detect_sections_unified_explained(
        &tracks,
        &[],
        &shapes::pooled(&tracks),
        &config(),
        &Tunables::DEFAULT,
    );
    assert_eq!(out.sections.len(), 1);
    assert!(
        out.boundaries.is_empty(),
        "an unbroken corridor has no cuts to explain: {:?}",
        out.boundaries
    );
}

/// Catalogue rows with the positional section ids stripped: cluster
/// ordering renumbers ids when far ground joins the corpus, and ids
/// were never content-stable; everything else must hold exactly.
fn sans_ids(catalogue: &str) -> String {
    catalogue
        .lines()
        .map(|row| row.split_once('|').map_or(row, |(_, rest)| rest))
        .collect::<Vec<_>>()
        .join("\n")
}

#[test]
fn far_ground_changes_nothing_local() {
    // Each geographic cluster projects and detects on its own plane, so
    // a season on another continent must not re-bucket a single home
    // cell, and the far ground must still detect correctly at its own
    // latitude. The oval is the sensitive probe: under one global plane
    // the hemisphere-shifted mean re-sizes home cells and measurably
    // re-cuts curved ground (oval, lollipop, switchback, hill repeats
    // all re-cut when this was probed; straight grids survive).
    let home = shapes::oval_stem(6);
    let alone = detect(&home);
    assert!(!alone.is_empty());

    let mut both = home.clone();
    both.extend(shapes::translate_deg(
        shapes::persona_commuter(),
        -83.8,
        138.0,
    ));
    let combined = detect(&both);

    let (south, north): (Vec<FrequentSection>, Vec<FrequentSection>) = combined
        .into_iter()
        .partition(|s| s.polyline.first().is_some_and(|p| p.latitude < 0.0));
    assert_eq!(
        south.len(),
        1,
        "the far commuter road is one section on its own plane"
    );
    assert_eq!(
        sans_ids(&normalise(&north)),
        sans_ids(&normalise(&alone)),
        "home catalogue re-cut by ground on another continent"
    );
}

// ------------------------------------------- invariant 4: order-free

#[test]
fn catalogue_is_a_pure_function_of_the_activity_set() {
    // The same activity set in three arrival orders must produce the
    // identical catalogue — ids, geometry, everything. This is the
    // parity spec the incremental path must meet. The three shape
    // families sit on disjoint ground so each keeps its own topology.
    let mut tracks = shapes::grid_city();
    tracks.extend(shapes::shift_east(shapes::persona_commuter(), 20_000.0));
    tracks.extend(shapes::shift_east(shapes::persona_weekender(), 40_000.0));

    let base = normalise(&detect(&tracks));

    let mut reversed = tracks.clone();
    reversed.reverse();
    assert_eq!(
        base,
        normalise(&detect(&reversed)),
        "catalogue depends on arrival order (reversed)"
    );

    let n = tracks.len();
    let shuffled: Vec<_> = (0..n).map(|i| tracks[(i * 7) % n].clone()).collect();
    assert_eq!(shuffled.len(), n, "stride shuffle must be a permutation");
    assert_eq!(
        base,
        normalise(&detect(&shuffled)),
        "catalogue depends on arrival order (shuffled)"
    );
}

// --------------------------------- invariant 6: evidence and deletion

#[test]
fn deleting_activities_is_the_only_way_evidence_leaves() {
    // Remove five of the six favourite-loop outings: the loop falls
    // below its floor and vanishes. Restore them: the catalogue is
    // byte-identical to the original. Deletion removes evidence;
    // nothing else is remembered or invented.
    let tracks = shapes::persona_weekender();
    let full = normalise(&detect(&tracks));

    let culled: Vec<_> = tracks
        .iter()
        .filter(|(id, _)| !id.starts_with("wkd_loop_") || id.ends_with("_0"))
        .cloned()
        .collect();
    let after_cull = detect(&culled);
    let loop_samples: Vec<GpsPoint> = (0..24)
        .map(|k| {
            let a = std::f64::consts::PI + k as f64 / 24.0 * 2.0 * std::f64::consts::PI;
            shapes::to_gps(900.0 + 500.0 * a.cos(), 500.0 * a.sin())
        })
        .collect();
    assert!(
        coverage(&loop_samples, &after_cull, 120.0) < 0.2,
        "single-visit loop must fall below the floor"
    );

    assert_eq!(
        full,
        normalise(&detect(&tracks)),
        "restoring the activity set must restore the catalogue exactly"
    );
}

// ------------------------- rule 5: support binds along the length

#[test]
fn one_off_tail_is_cut_where_its_own_support_ends() {
    // One outing takes a wrong turn off a busy corridor and rides a
    // tail nobody else traverses end to end. Peel-off traffic thins one
    // track per cell, so the tail welds onto the corridor through the
    // one-missing-track rule and short stray strands keep its cells
    // hot. Support must bind everywhere, not just in total: the deep
    // tail is one outing's private ground, cut with a low-support
    // record, while the corridor keeps all five contributors.
    use tracematch::{BoundaryReason, Tunables, detect_sections_unified_explained};

    let tracks = shapes::welded_tail();
    let out = detect_sections_unified_explained(
        &tracks,
        &[],
        &shapes::pooled(&tracks),
        &config(),
        &Tunables::DEFAULT,
    );
    dump(&out.sections);
    assert_catalogue_invariants(&tracks, &out.sections);

    let corridor: Vec<GpsPoint> = (2..=13)
        .map(|i| shapes::to_gps(i as f64 * 100.0, 0.0))
        .collect();
    assert!(
        coverage(&corridor, &out.sections, 130.0) >= 0.9,
        "the corridor itself must survive"
    );
    for y in [700.0, 800.0, 900.0] {
        let p = shapes::to_gps(1500.0, y);
        assert!(
            out.sections
                .iter()
                .all(|s| min_dist(&p, &s.polyline) > 130.0),
            "one outing's tail ground appeared in a section at y={y}"
        );
    }
    let cut_line = shapes::to_gps(0.0, 500.0).latitude;
    assert!(
        out.boundaries.iter().any(|r| {
            matches!(r.reason, BoundaryReason::LowSupport { .. }) && r.latitude > cut_line
        }),
        "no low-support record on the tail: {:?}",
        out.boundaries
    );
}

#[test]
fn folded_reps_render_their_longest_clean_stretch() {
    // Scenario: every outing runs a corridor with interval reps mid
    // way — forward, back over the same ground, forward again — at a
    // window that shifts per outing, so no pass-class boundary forms
    // and every pass carries a legal but visibly folded knot.
    // Expected behaviour: the drawn line is the pass's longest
    // fold-free stretch, and the undrawn remainder re-enters the
    // queue, so the corridor is covered by clean lines instead of one
    // folded face.
    let tracks = shapes::interval_reps(6);
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);
    for s in &sections {
        assert!(
            self_overlap_frac(&s.polyline) <= 0.05,
            "{}: rendered line folds over itself ({:.0}%)",
            s.id,
            self_overlap_frac(&s.polyline) * 100.0
        );
    }
    let body = metre_samples(&[(50.0, 0.0), (750.0, 0.0)], 20.0);
    assert!(
        coverage(&body, &sections, 90.0) >= 0.8,
        "corridor body must stay covered by clean renders (coverage {:.0}%)",
        coverage(&body, &sections, 90.0) * 100.0
    );
}

#[test]
fn junction_shredded_middle_pools_back_into_a_section() {
    // Scenario: heavy bulks leave a through corridor at both ends
    // (cliff + fork boundaries) and a crossing path inflates one
    // middle cell's track set, so the strict same-traffic partition
    // shreds the 12-track middle into sub-length pieces. Each piece
    // alone dies — under the length floor or backed off against the
    // flanks' render bleed — while the middle carries 12 through
    // tracks end to end.
    // Expected behaviour: ground orphaned by dead candidates pools at
    // visible-boundary granularity and re-enters the queue, so the
    // honest middle surfaces instead of leaving a hole.
    let tracks = shapes::shredded_corridor();
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);
    let mid = metre_samples(&[(620.0, 0.0), (820.0, 0.0)], 10.0);
    assert!(
        coverage(&mid, &sections, 90.0) >= 0.8,
        "shredded middle must be salvaged (coverage {:.0}%)",
        coverage(&mid, &sections, 90.0) * 100.0
    );
}

#[test]
fn lapped_small_oval_renders_the_closed_lap() {
    // Scenario: interval sessions on a small athletics oval — stem in,
    // many laps, stem home. The oval spans only a couple of evidence
    // cells, so the mouth cell blends lap and stem ground, the usage
    // boundary cannot separate them, and the default pass is the stem
    // plus the first lap.
    // Expected behaviour: a pass that ends by closing onto its own
    // interior renders the LOOP. The stem is through-ground the
    // closure disowns; it re-enters the queue on its own merits.
    let tracks = shapes::small_oval_stem(6);
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);
    let centre = shapes::to_gps(220.0, 0.0);
    let oval = sections
        .iter()
        .find(|s| {
            let mean = s
                .polyline
                .iter()
                .map(|p| haversine_distance(p, &centre))
                .sum::<f64>()
                / s.polyline.len() as f64;
            mean < 120.0
        })
        .expect("oval section");
    let endgap = haversine_distance(
        oval.polyline.first().unwrap(),
        oval.polyline.last().unwrap(),
    );
    assert!(
        endgap <= 40.0,
        "oval render is not a closed lap (endgap {endgap:.0} m)"
    );
    assert!(
        (350.0..520.0).contains(&oval.distance_meters),
        "oval render {:.0} m is not one revolution (~440 m)",
        oval.distance_meters
    );
    let stem_mid = shapes::to_gps(40.0, 0.0);
    assert!(
        min_dist(&stem_mid, &oval.polyline) > 60.0,
        "oval render still reaches down the stem"
    );
}

#[test]
fn lapped_oval_counts_every_revolution() {
    // Scenario: six interval sessions on the small oval, ten laps each.
    // Sixty real revolutions of one 440 m circuit.
    // Expected behaviour: a visit is a pass over the ground, not an
    // outing. Counting outings tells an athlete they ran the oval six
    // times when they ran it sixty, and it starves the render of
    // candidates — only the first lap of each session can ever be
    // drawn, and a first lap carries the entry from the stem.
    let tracks = shapes::small_oval_stem(6);
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);
    let centre = shapes::to_gps(220.0, 0.0);
    let oval = sections
        .iter()
        .find(|s| {
            let mean = s
                .polyline
                .iter()
                .map(|p| haversine_distance(p, &centre))
                .sum::<f64>()
                / s.polyline.len() as f64;
            mean < 120.0
        })
        .expect("oval section");
    assert!(
        oval.visit_count >= 50,
        "oval counts {} visits for sixty real revolutions — laps inside an \
         activity are being discarded",
        oval.visit_count
    );
}

#[test]
fn a_deviating_lap_does_not_render_the_circuit() {
    // Scenario: five clean interval sessions lap the small oval; a
    // sixth outing cuts a chord across it every lap, and its longer
    // stem makes its pass the longest — the default. The deviant's
    // revolution is one real single pass, but it is minority ground
    // against five faithful circuits.
    // Expected behaviour: the loop render is judged like any other
    // render — faithfulness on the final line — so the circuit
    // renders from a faithful lap, not the deviant default.
    let tracks = shapes::small_oval_stem_deviant();
    let sections = detect(&tracks);
    dump(&sections);
    assert_catalogue_invariants(&tracks, &sections);
    let centre = shapes::to_gps(220.0, 0.0);
    let oval = sections
        .iter()
        .find(|s| {
            let mean = s
                .polyline
                .iter()
                .map(|p| haversine_distance(p, &centre))
                .sum::<f64>()
                / s.polyline.len() as f64;
            mean < 120.0
        })
        .expect("oval section");
    let endgap = haversine_distance(
        oval.polyline.first().unwrap(),
        oval.polyline.last().unwrap(),
    );
    assert!(
        endgap <= 40.0,
        "oval render is not a closed lap (endgap {endgap:.0} m)"
    );
    let worst_radial = oval
        .polyline
        .iter()
        .map(|p| (haversine_distance(p, &centre) - 70.0).abs())
        .fold(0.0f64, f64::max);
    assert!(
        worst_radial <= 25.0,
        "oval render strays {worst_radial:.0} m from the circuit: a deviating lap won the render"
    );
}
