//! Detection over a public corpus of real GPS.
//!
//! The synthetic corpora replay one canonical polyline with added noise, so
//! repeated traces are the same line by construction and the "are these the
//! same section" decision is never asked hard. GeoLife carries braided real
//! trajectories from people who are not the author, which is the input that
//! asks it.
//!
//! Unlike the personal GPX corpora this data is public and fetchable on any
//! machine, so this target is the durable real-GPS gate: it survives losing
//! every local file. It sits behind `public-corpus` rather than `real-corpus`
//! for that reason, and CI fetches the archive and runs it on every push.
//!
//! Dataset: Microsoft GeoLife GPS Trajectories 1.3. Research use only, no
//! redistribution, so `geolife/` is gitignored and fetched per machine by
//! `scripts/fetch_geolife.sh`.
//!
//! The bar here is deliberately low: sections form at all, and each one rests
//! on real supplied ground. It is not a quality bar. The busiest GeoLife user
//! yields 2 sections from 120 trajectories, against 14 from a 150-activity
//! synthetic corpus, and which of those two numbers is right for this data is
//! an open question rather than a settled expectation.
//!
//! Run: `scripts/fetch_geolife.sh && cargo test --features public-corpus \
//!       --test geolife_public_corpus`

#[path = "../examples/common/geolife.rs"]
mod geolife;

use std::collections::{BTreeSet, HashMap};
use std::path::PathBuf;

use geolife::load_geolife;
use tracematch::{
    FrequentSection, GpsPoint, MatchConfig, RouteSignature, SectionConfig, detect_sections_unified,
    group_signatures_parallel,
};

const ENV: &str = "LAB_GEOLIFE_DIR";

/// The whole set is scanned so the busiest logger can be picked out of it.
/// Detection then runs on that one person, because the app detects within one
/// athlete's own history and never pools strangers together.
const MAX_USERS: usize = 200;
const MAX_PER_USER: usize = 120;

/// A GeoLife id is `"<user>_<filestem>"`.
fn user_of(id: &str) -> &str {
    id.split_once('_').map(|(u, _)| u).unwrap_or(id)
}

fn data_dir() -> PathBuf {
    if let Ok(p) = std::env::var(ENV)
        && !p.trim().is_empty()
    {
        let path = PathBuf::from(p);
        assert!(
            path.is_dir(),
            "{ENV} points at {}, which is not a directory",
            path.display()
        );
        return path;
    }

    let default =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("geolife/Geolife Trajectories 1.3/Data");
    assert!(
        default.is_dir(),
        "GeoLife not found at {}\n\
         Fetch it with scripts/fetch_geolife.sh, or set {ENV} to an existing copy. \
         Skipping instead would report the same green as a run that detected \
         nothing.",
        default.display()
    );
    default
}

struct Corpus {
    tracks: Vec<(String, Vec<GpsPoint>)>,
    sports: HashMap<String, String>,
    /// Per-point seconds, parallel to each track's points. GeoLife times every
    /// fix and carries no elevation it can trust, so these reach the lift
    /// veto's velocity test and no candidate ever reaches them. They are
    /// loaded anyway so this suite folds the inputs the engine folds.
    seconds: Vec<Vec<f64>>,
}

/// The single busiest logger in the set, which is the closest GeoLife gets to
/// one athlete's own history.
fn load() -> Corpus {
    let (trajectories, stats) = load_geolife(&data_dir(), MAX_USERS, MAX_PER_USER);

    assert!(
        stats.users_loaded > 0,
        "GeoLife directory exists but yielded no users. Only 69 of the 182 users \
         carry the labels.txt this loader needs, so a partial extraction looks \
         exactly like this."
    );

    let mut counts: std::collections::BTreeMap<&str, usize> = std::collections::BTreeMap::new();
    for t in &trajectories {
        *counts.entry(user_of(&t.id)).or_default() += 1;
    }
    let busiest = counts
        .iter()
        .max_by_key(|(user, n)| (**n, std::cmp::Reverse(*user)))
        .map(|(user, _)| (*user).to_string())
        .expect("at least one user loaded");

    let mine: Vec<_> = trajectories
        .into_iter()
        .filter(|t| user_of(&t.id) == busiest)
        .collect();

    assert!(
        mine.len() >= 50,
        "busiest GeoLife user {busiest} logged {} self-powered trajectories, this \
         suite needs 50 to have repeated ground to find",
        mine.len()
    );

    let sports = mine
        .iter()
        .map(|t| (t.id.clone(), t.sport.clone()))
        .collect();
    let mut tracks = Vec::with_capacity(mine.len());
    let mut seconds = Vec::with_capacity(mine.len());
    for t in mine {
        // A stream that does not cover every point cannot be indexed by point.
        seconds.push(if t.seconds.len() == t.points.len() {
            t.seconds
        } else {
            Vec::new()
        });
        tracks.push((t.id, t.points));
    }

    Corpus {
        tracks,
        sports,
        seconds,
    }
}

/// Grouping then detection, the order the engine runs them in. Detection
/// filters on route groups, so feeding it an empty group list measures a path
/// the app never takes.
fn detect(corpus: &Corpus) -> Vec<FrequentSection> {
    let match_config = MatchConfig::default();
    let signatures: Vec<RouteSignature> = corpus
        .tracks
        .iter()
        .filter_map(|(id, pts)| RouteSignature::from_points(id, pts, &match_config))
        .collect();
    let groups = group_signatures_parallel(&signatures, &match_config);

    let _ = groups;
    let seconds: Vec<&[f64]> = corpus.seconds.iter().map(Vec::as_slice).collect();
    detect_sections_unified(
        &corpus.tracks,
        &seconds,
        &corpus.sports,
        &SectionConfig::default(),
    )
}

#[test]
fn detection_finds_repeated_ground_in_public_traces() {
    let corpus = load();
    let sections = detect(&corpus);

    assert!(
        !sections.is_empty(),
        "no sections across {} real trajectories. These are commuters logging the \
         same routes for months, so an empty catalogue means the detector is not \
         seeing repetition that is plainly there.",
        corpus.tracks.len()
    );

    for s in &sections {
        assert!(
            s.polyline.len() >= 2,
            "section {} has {} point(s), which cannot describe ground",
            s.id,
            s.polyline.len()
        );
        assert!(
            s.activity_ids.len() >= 2,
            "section {} rests on {} activity, so it is not repeated ground",
            s.id,
            s.activity_ids.len()
        );
        assert!(
            s.distance_meters > 0.0,
            "section {} measures {}m",
            s.id,
            s.distance_meters
        );
    }
}

/// Every id a section claims must be an activity that was fed in. A section
/// citing ground from a track the caller never supplied is a rendering of
/// something nobody travelled.
#[test]
fn sections_only_cite_activities_that_were_supplied() {
    let corpus = load();
    let supplied: BTreeSet<&str> = corpus.tracks.iter().map(|(id, _)| id.as_str()).collect();
    let sections = detect(&corpus);

    assert!(!sections.is_empty(), "no sections to check");

    for s in &sections {
        for id in &s.activity_ids {
            assert!(
                supplied.contains(id.as_str()),
                "section {} cites activity {id:?}, which was never supplied",
                s.id
            );
        }
    }
}

/// How far along `line` a point sits, as a fraction of the line's length.
fn progress_on(line: &[GpsPoint], cumulative: &[f64], point: &GpsPoint) -> f64 {
    let mut best = (f64::INFINITY, 0.0);
    for (i, p) in line.iter().enumerate() {
        let d = tracematch::geo_utils::haversine_distance(p, point);
        if d < best.0 {
            best = (d, cumulative[i]);
        }
    }
    let total = *cumulative.last().unwrap_or(&0.0);
    if total <= 0.0 { 0.0 } else { best.1 / total }
}

/// The flag has to agree with the ground, measured independently of how the
/// detector measured it. Each portion's points are projected onto the line the
/// section emits and read for which way their progress runs; a portion flagged
/// `Same` whose progress falls is a lap the app draws backwards, names as a
/// best against the wrong population, and offers the wrong direction toggle for.
#[test]
fn every_portion_flag_agrees_with_travel_along_the_emitted_line() {
    let corpus = load();
    let tracks: HashMap<&str, &[GpsPoint]> = corpus
        .tracks
        .iter()
        .map(|(id, pts)| (id.as_str(), pts.as_slice()))
        .collect();
    let sections = detect(&corpus);

    assert!(!sections.is_empty(), "no sections to check");

    let mut measured = 0usize;
    let mut disagreed = Vec::new();
    for s in &sections {
        if s.polyline.len() < 3 {
            continue;
        }
        let mut cumulative = Vec::with_capacity(s.polyline.len());
        let mut run = 0.0;
        cumulative.push(0.0);
        for w in s.polyline.windows(2) {
            run += tracematch::geo_utils::haversine_distance(&w[0], &w[1]);
            cumulative.push(run);
        }

        for portion in &s.activity_portions {
            let Some(track) = tracks.get(portion.activity_id.as_str()) else {
                continue;
            };
            let start = portion.start_index as usize;
            let end = (portion.end_index as usize).min(track.len());
            if end < start + 3 {
                continue;
            }
            let first = progress_on(&s.polyline, &cumulative, &track[start]);
            let last = progress_on(&s.polyline, &cumulative, &track[end - 1]);
            // Only a lap that plainly crosses the section says anything about
            // direction; a pass that barely moves along the line does not.
            if (last - first).abs() < 0.5 {
                continue;
            }
            measured += 1;
            let travels_forward = last > first;
            let flagged_forward = portion.direction == tracematch::Direction::Same;
            if travels_forward != flagged_forward {
                disagreed.push(format!(
                    "{} / {} flagged {:?} but ran {:.2} to {:.2}",
                    s.id, portion.activity_id, portion.direction, first, last
                ));
            }
        }
    }

    assert!(
        measured > 0,
        "no portion crossed enough of its section to measure a direction"
    );
    assert!(
        disagreed.is_empty(),
        "{} of {measured} portions run against their own flag:\n{}",
        disagreed.len(),
        disagreed.join("\n")
    );
}
