//! How often the relaxed cut offers a pass over a section the strict cut
//! left unattached, over the two corpora, and what those offers look like.
//!
//! The strict cut is the detector's: `PreparedLine::new`. The relaxed cut is
//! the attach-by-hand bar, the strict proximity threshold times 2.5 less the
//! ground tolerance as a lateral allowance, which the engine's scan and its
//! force-attach both cut with. A ride is "offered" a section when the relaxed
//! cut finds a pass over a section the fold did not make it a member of.
//!
//! Prints a per-corpus and per-sport summary and writes every offer's metrics,
//! with geometry for a deterministic sample, under `target/scan-miss/`.
//!
//! Run: `cargo test --release --features real-corpus,public-corpus \
//!       --test scan_miss_rate -- --nocapture --test-threads 1`

mod bitwise;
#[cfg(feature = "real-corpus")]
mod corpus;
#[path = "../examples/common/geolife.rs"]
mod geolife;

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::PathBuf;

use rayon::prelude::*;
use serde::Serialize;
use tracematch::geo_utils::{distance_to_segment, haversine_distance};
use tracematch::sections::{FrequentSection, GROUND_TOL_M};
use tracematch::{Direction, GpsPoint, PreparedLine, SectionConfig, SectionEvidenceCache};

use bitwise::Corpus;

/// The engine's relaxed bar over the strict proximity threshold.
const FORCED_PROXIMITY_FACTOR: f64 = 2.5;
/// Offers carrying geometry for the by-eye sample, per corpus.
const GEOMETRY_SAMPLE: usize = 40;
/// A pass covering at least this share of the line is counted full.
const FULL_COVERAGE: f64 = 0.9;

#[derive(Serialize, Clone)]
struct Offer {
    corpus: String,
    ride: String,
    ride_sport: String,
    section: String,
    section_sport: String,
    section_m: f64,
    section_members: usize,
    start_index: u32,
    end_index: u32,
    pass_m: f64,
    coverage: f64,
    direction: String,
    /// Whether the strict cut also finds a pass here, so the miss is the
    /// fold's membership rather than the bar.
    strict_finds: bool,
    /// Whether the ride holds no section at all, the empty-state case.
    ride_has_no_section: bool,
    /// Share of the offered pass's points that lie inside a pass the ride
    /// already holds on some section: ground the library already covers.
    member_pass_share: f64,
    lateral_median_m: f64,
    lateral_p90_m: f64,
    lateral_max_m: f64,
    within_ground_share: f64,
    within_strict_share: f64,
    /// Share of the line's 10 m stations within the ground tolerance of
    /// some point of the pass, and within twice it.
    station_share_50: f64,
    station_share_100: f64,
    /// Whether both ends of the line lie within the ground tolerance of
    /// the pass.
    ends_within_50: bool,
    ends_within_100: bool,
    /// Metres of the line's stations beyond the ground tolerance of the
    /// pass, split into the run at the line's head, the run at its tail,
    /// and everything between.
    uncovered_head_m: f64,
    uncovered_tail_m: f64,
    uncovered_interior_m: f64,
    /// Probes for a pass the strict cut rejects: whether it finds the pass
    /// on the pass's own points alone, and with 10 m or 25 m of lateral
    /// tolerance over the whole track.
    strict_on_pass_only: bool,
    strict_lateral_10: bool,
    strict_lateral_25: bool,
    /// Share of the pass's steps that move along the line in the pass's
    /// own direction, by nearest station: a meander over a short line
    /// scores low, a traversal scores near one.
    monotone: f64,
    /// The longest stretch of the pass, in metres along the ride, that
    /// runs beyond the ground tolerance of the line.
    excursion_max_m: f64,
    /// Whether the offer is a candidate real pass: full coverage, median
    /// lateral within the ground tolerance, over ground the ride holds no
    /// pass on.
    candidate: bool,
    /// Ride points around the pass and the section line, sampled rides only.
    #[serde(skip_serializing_if = "Option::is_none")]
    geometry: Option<Geometry>,
}

#[derive(Serialize, Clone)]
struct Geometry {
    ride: Vec<[f64; 2]>,
    pass: Vec<[f64; 2]>,
    line: Vec<[f64; 2]>,
}

fn relaxed_lateral_m(config: &SectionConfig) -> f64 {
    config.proximity_threshold * FORCED_PROXIMITY_FACTOR - GROUND_TOL_M
}

/// Distance from a point to the nearest segment of a line.
fn lateral_m(p: &GpsPoint, line: &[GpsPoint]) -> f64 {
    line.windows(2)
        .map(|ab| distance_to_segment(p, &ab[0], &ab[1]))
        .fold(f64::INFINITY, f64::min)
}

fn percentile(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    let i = ((sorted.len() - 1) as f64 * q).round() as usize;
    sorted[i.min(sorted.len() - 1)]
}

fn thin(points: &[GpsPoint], cap: usize) -> Vec<[f64; 2]> {
    let step = (points.len() / cap).max(1);
    points
        .iter()
        .step_by(step)
        .map(|p| [p.latitude, p.longitude])
        .collect()
}

/// The line resampled every `step_m` metres, its own vertices kept.
fn stations(line: &[GpsPoint], step_m: f64) -> Vec<GpsPoint> {
    let mut out = Vec::new();
    for ab in line.windows(2) {
        let d = haversine_distance(&ab[0], &ab[1]);
        let n = (d / step_m).floor() as usize;
        out.push(ab[0]);
        for k in 1..=n {
            let t = k as f64 * step_m / d;
            if t < 1.0 {
                out.push(GpsPoint::new(
                    ab[0].latitude + (ab[1].latitude - ab[0].latitude) * t,
                    ab[0].longitude + (ab[1].longitude - ab[0].longitude) * t,
                ));
            }
        }
    }
    if let Some(last) = line.last() {
        out.push(*last);
    }
    out
}

/// Distance from a point to the nearest point of a track.
fn nearest_m(p: &GpsPoint, track: &[GpsPoint]) -> f64 {
    track
        .iter()
        .map(|q| haversine_distance(p, q))
        .fold(f64::INFINITY, f64::min)
}

struct Prepared<'a> {
    section: &'a FrequentSection,
    stations: Vec<GpsPoint>,
    lateral_10: PreparedLine<'a>,
    lateral_25: PreparedLine<'a>,
    members: HashSet<&'a str>,
    strict: PreparedLine<'a>,
    relaxed: PreparedLine<'a>,
}

fn measure(name: &str, c: &Corpus, out_dir: &PathBuf) -> Vec<Offer> {
    let config = SectionConfig::default();
    let all_ids: Vec<&str> = c.tracks.iter().map(|(id, _)| id.as_str()).collect();
    let t = std::time::Instant::now();
    let result = tracematch::sections::detect_sections_incremental_dated(
        &mut SectionEvidenceCache::new(),
        &[],
        &c.tracks,
        &all_ids,
        &bitwise::seconds_view(&c.tracks, c),
        &c.sports,
        &c.starts,
        &config,
        &tracematch::SectionUpdatePolicy::default(),
    );
    let catalogue = result.catalogue;
    // Every clip or trim the fold recorded against a section, by id.
    let mut clips: HashMap<String, Vec<String>> = HashMap::new();
    for b in &result.boundaries {
        let reason = format!("{:?}", b.reason);
        for id in &b.section_ids {
            clips.entry(id.clone()).or_default().push(reason.clone());
        }
    }
    println!(
        "[{name}] fold: {} rides, {} sections, {} ms",
        c.tracks.len(),
        catalogue.len(),
        t.elapsed().as_millis()
    );

    let lateral = relaxed_lateral_m(&config);
    let prepared: Vec<Prepared> = catalogue
        .iter()
        .filter(|s| s.polyline.len() >= 2)
        .filter_map(|s| {
            Some(Prepared {
                section: s,
                stations: stations(&s.polyline, 10.0),
                lateral_10: PreparedLine::with_lateral_tolerance(&s.polyline, &config, 10.0)?,
                lateral_25: PreparedLine::with_lateral_tolerance(&s.polyline, &config, 25.0)?,
                members: s.activity_ids.iter().map(String::as_str).collect(),
                strict: PreparedLine::new(&s.polyline, &config)?,
                relaxed: PreparedLine::with_lateral_tolerance(&s.polyline, &config, lateral)?,
            })
        })
        .collect();

    let attached: HashSet<&str> = catalogue
        .iter()
        .flat_map(|s| s.activity_ids.iter().map(String::as_str))
        .collect();
    // Every pass each ride already holds, as index ranges, over any section.
    let mut held: HashMap<&str, Vec<(usize, usize)>> = HashMap::new();
    for s in &catalogue {
        for ap in &s.activity_portions {
            held.entry(ap.activity_id.as_str())
                .or_default()
                .push((ap.start_index as usize, ap.end_index as usize));
        }
    }

    let t = std::time::Instant::now();
    let mut offers: Vec<Offer> = c
        .tracks
        .par_iter()
        .flat_map_iter(|(ride, track)| {
            let ride_sport = c.sports.get(ride).cloned().unwrap_or_default();
            let no_section = !attached.contains(ride.as_str());
            let mut found = Vec::new();
            for p in &prepared {
                if p.members.contains(ride.as_str()) {
                    continue;
                }
                let relaxed = p.relaxed.portions(ride, track);
                if relaxed.is_empty() {
                    continue;
                }
                let strict = p.strict.portions(ride, track);
                for portion in relaxed {
                    let (s, e) = (portion.start_index as usize, portion.end_index as usize);
                    let strict_finds = strict
                        .iter()
                        .any(|q| (q.start_index as usize) < e && s < q.end_index as usize);
                    let member_pass_share = held.get(ride.as_str()).map_or(0.0, |ranges| {
                        let inside = (s..e)
                            .filter(|i| ranges.iter().any(|(a, b)| a <= i && i < b))
                            .count();
                        inside as f64 / (e - s).max(1) as f64
                    });
                    let pass_pts = &track[s..e.min(track.len())];
                    let raw: Vec<f64> = pass_pts
                        .iter()
                        .map(|pt| lateral_m(pt, &p.section.polyline))
                        .collect();
                    let mut excursion_max_m: f64 = 0.0;
                    let mut run = 0.0;
                    for (i, d) in raw.iter().enumerate() {
                        if *d > GROUND_TOL_M {
                            if i > 0 {
                                run += haversine_distance(&pass_pts[i - 1], &pass_pts[i]);
                            }
                            excursion_max_m = excursion_max_m.max(run);
                        } else {
                            run = 0.0;
                        }
                    }
                    let st: Vec<f64> = p.stations.iter().map(|q| nearest_m(q, pass_pts)).collect();
                    let station_share = |bar: f64| {
                        st.iter().filter(|&&d| d <= bar).count() as f64 / st.len().max(1) as f64
                    };
                    let ends_within_50 = st.first().is_some_and(|d| *d <= GROUND_TOL_M)
                        && st.last().is_some_and(|d| *d <= GROUND_TOL_M);
                    let ends_within_100 = st.first().is_some_and(|d| *d <= 2.0 * GROUND_TOL_M)
                        && st.last().is_some_and(|d| *d <= 2.0 * GROUND_TOL_M);
                    let (strict_on_pass_only, strict_lateral_10, strict_lateral_25) =
                        if strict_finds {
                            (true, true, true)
                        } else {
                            (
                                !p.strict.portions(ride, pass_pts).is_empty(),
                                !p.lateral_10.portions(ride, track).is_empty(),
                                !p.lateral_25.portions(ride, track).is_empty(),
                            )
                        };
                    let far: Vec<bool> = st.iter().map(|d| *d > GROUND_TOL_M).collect();
                    let head = far.iter().take_while(|f| **f).count();
                    let tail = if head == far.len() {
                        0
                    } else {
                        far.iter().rev().take_while(|f| **f).count()
                    };
                    let interior = far.iter().filter(|f| **f).count() - head - tail;
                    let (uncovered_head_m, uncovered_tail_m, uncovered_interior_m) = (
                        head as f64 * 10.0,
                        tail as f64 * 10.0,
                        interior as f64 * 10.0,
                    );
                    let nearest_station: Vec<usize> = pass_pts
                        .iter()
                        .map(|q| {
                            p.stations
                                .iter()
                                .enumerate()
                                .min_by(|a, b| {
                                    haversine_distance(q, a.1)
                                        .total_cmp(&haversine_distance(q, b.1))
                                })
                                .map_or(0, |(i, _)| i)
                        })
                        .collect();
                    let steps: Vec<i64> = nearest_station
                        .windows(2)
                        .map(|w| w[1] as i64 - w[0] as i64)
                        .filter(|d| *d != 0)
                        .collect();
                    let forward = steps.iter().filter(|d| **d > 0).count() as f64;
                    let monotone = if steps.is_empty() {
                        0.0
                    } else {
                        (forward / steps.len() as f64).max(1.0 - forward / steps.len() as f64)
                    };
                    let mut lat = raw;
                    lat.sort_by(f64::total_cmp);
                    let n = lat.len().max(1) as f64;
                    let share = |bar: f64| lat.iter().filter(|&&d| d <= bar).count() as f64 / n;
                    found.push(Offer {
                        corpus: name.to_string(),
                        ride: ride.clone(),
                        ride_sport: ride_sport.clone(),
                        section: p.section.id.clone(),
                        section_sport: p.section.sport_type.clone(),
                        section_m: p.section.distance_meters,
                        section_members: p.section.activity_ids.len(),
                        start_index: portion.start_index,
                        end_index: portion.end_index,
                        pass_m: portion.distance_meters,
                        coverage: if p.section.distance_meters > 0.0 {
                            (portion.distance_meters / p.section.distance_meters).min(1.0)
                        } else {
                            1.0
                        },
                        direction: format!("{:?}", portion.direction),
                        strict_finds,
                        ride_has_no_section: no_section,
                        member_pass_share,
                        lateral_median_m: percentile(&lat, 0.5),
                        lateral_p90_m: percentile(&lat, 0.9),
                        lateral_max_m: lat.last().copied().unwrap_or(f64::NAN),
                        within_ground_share: share(GROUND_TOL_M),
                        within_strict_share: share(config.proximity_threshold),
                        station_share_50: station_share(GROUND_TOL_M),
                        station_share_100: station_share(2.0 * GROUND_TOL_M),
                        ends_within_50,
                        ends_within_100,
                        uncovered_head_m,
                        uncovered_tail_m,
                        uncovered_interior_m,
                        strict_on_pass_only,
                        strict_lateral_10,
                        strict_lateral_25,
                        monotone,
                        excursion_max_m,
                        candidate: false,
                        geometry: None,
                    });
                }
            }
            found
        })
        .collect();
    offers.sort_by(|a, b| {
        (&a.ride, &a.section, a.start_index).cmp(&(&b.ride, &b.section, b.start_index))
    });
    for o in &mut offers {
        o.candidate = o.coverage >= FULL_COVERAGE
            && o.lateral_median_m <= GROUND_TOL_M
            && o.member_pass_share < 0.5;
    }
    println!(
        "[{name}] scan: {} offers over {} rides x {} sections, {} ms",
        offers.len(),
        c.tracks.len(),
        prepared.len(),
        t.elapsed().as_millis()
    );

    // A deterministic sample carries geometry: every k-th offer in sorted order.
    if !offers.is_empty() {
        let stride = (offers.len() / GEOMETRY_SAMPLE).max(1);
        let tracks: HashMap<&str, &[GpsPoint]> = c
            .tracks
            .iter()
            .map(|(id, pts)| (id.as_str(), pts.as_slice()))
            .collect();
        let lines: HashMap<&str, &[GpsPoint]> = catalogue
            .iter()
            .map(|s| (s.id.as_str(), s.polyline.as_slice()))
            .collect();
        // Two samples: every k-th offer, which is what the scan would show,
        // and every k-th candidate real pass, the class the question is about.
        let candidates: Vec<usize> = offers
            .iter()
            .enumerate()
            .filter(|(_, o)| {
                o.coverage >= FULL_COVERAGE
                    && o.lateral_median_m <= GROUND_TOL_M
                    && o.member_pass_share < 0.5
            })
            .map(|(i, _)| i)
            .collect();
        let cand_stride = (candidates.len() / GEOMETRY_SAMPLE).max(1);
        let mut chosen: Vec<usize> = (0..offers.len())
            .step_by(stride)
            .take(GEOMETRY_SAMPLE)
            .collect();
        chosen.extend(candidates.iter().step_by(cand_stride).take(GEOMETRY_SAMPLE));
        chosen.sort_unstable();
        chosen.dedup();
        for i in chosen {
            let offer = &mut offers[i];
            let track = tracks[offer.ride.as_str()];
            let (s, e) = (
                offer.start_index as usize,
                (offer.end_index as usize).min(track.len()),
            );
            let pad = ((e - s) / 2).max(20);
            let from = s.saturating_sub(pad);
            let to = (e + pad).min(track.len());
            offer.geometry = Some(Geometry {
                ride: thin(&track[from..to], 600),
                pass: thin(&track[s..e], 400),
                line: thin(lines[offer.section.as_str()], 400),
            });
        }
    }

    summarise(name, c, &catalogue, &offers, &attached);
    #[derive(Serialize)]
    struct SectionRow {
        id: String,
        distance_m: f64,
        polyline_m: f64,
        representative_m: Option<f64>,
        members: usize,
        rows: usize,
        /// Members whose fresh strict cut over the stored line finds no pass.
        members_without_fresh_pass: usize,
        /// Members whose fresh strict cut starts at a different index from
        /// every stored row.
        members_with_moved_rows: usize,
        /// Non-members whose fresh strict cut over the stored line finds a pass.
        non_members_with_fresh_pass: usize,
        representative_is_member: bool,
        boundary_reasons: Vec<String>,
        /// Whether the stored polyline is, point for point, the
        /// representative track's stored range.
        polyline_is_representative_range: bool,
        /// Metres between the polyline's ends and the representative
        /// range's ends.
        start_shift_m: f64,
        end_shift_m: f64,
    }
    let track_by_id: HashMap<&str, &[GpsPoint]> = c
        .tracks
        .iter()
        .map(|(id, pts)| (id.as_str(), pts.as_slice()))
        .collect();
    let rows: Vec<SectionRow> = catalogue
        .iter()
        .map(|s| {
            let line = PreparedLine::new(&s.polyline, &config);
            let mut without = 0;
            let mut moved = 0;
            if let Some(line) = &line {
                for id in &s.activity_ids {
                    let Some(t) = track_by_id.get(id.as_str()) else {
                        continue;
                    };
                    let fresh = line.portions(id, t);
                    if fresh.is_empty() {
                        without += 1;
                    } else if !fresh.iter().any(|f| {
                        s.activity_portions.iter().any(|r| {
                            r.activity_id == *id && r.start_index.abs_diff(f.start_index) <= 2
                        })
                    }) {
                        moved += 1;
                    }
                }
            }
            let non_members = offers
                .iter()
                .filter(|o| o.section == s.id && o.strict_finds)
                .map(|o| o.ride.as_str())
                .collect::<HashSet<_>>()
                .len();
            SectionRow {
                members_without_fresh_pass: without,
                members_with_moved_rows: moved,
                non_members_with_fresh_pass: non_members,
                representative_is_member: s.activity_ids.contains(&s.representative_activity_id),
                boundary_reasons: clips.get(&s.id).cloned().unwrap_or_default(),
                polyline_is_representative_range: s.representative_range.is_some_and(|(a, b)| {
                    track_by_id
                        .get(s.representative_activity_id.as_str())
                        .is_some_and(|t| {
                            let (a, b) = (a as usize, (b as usize).min(t.len()));
                            b > a
                                && t[a..b].len() == s.polyline.len()
                                && t[a..b].iter().zip(&s.polyline).all(|(x, y)| {
                                    x.latitude == y.latitude && x.longitude == y.longitude
                                })
                        })
                }),
                start_shift_m: s
                    .representative_range
                    .and_then(|(a, _)| {
                        track_by_id
                            .get(s.representative_activity_id.as_str())
                            .and_then(|t| t.get(a as usize))
                            .zip(s.polyline.first())
                            .map(|(x, y)| haversine_distance(x, y))
                    })
                    .unwrap_or(f64::NAN),
                end_shift_m: s
                    .representative_range
                    .and_then(|(_, b)| {
                        track_by_id
                            .get(s.representative_activity_id.as_str())
                            .and_then(|t| t.get((b as usize).saturating_sub(1)))
                            .zip(s.polyline.last())
                            .map(|(x, y)| haversine_distance(x, y))
                    })
                    .unwrap_or(f64::NAN),
                id: s.id.clone(),
                distance_m: s.distance_meters,
                polyline_m: tracematch::matching::calculate_route_distance(&s.polyline),
                representative_m: s.representative_range.and_then(|(a, b)| {
                    track_by_id
                        .get(s.representative_activity_id.as_str())
                        .map(|t| {
                            tracematch::matching::calculate_route_distance(
                                &t[a as usize..(b as usize).min(t.len())],
                            )
                        })
                }),
                members: s.activity_ids.len(),
                rows: s.activity_portions.len(),
            }
        })
        .collect();
    // Members whose stored rows disagree with a fresh strict cut, with
    // geometry, for a by-eye check of what the stored row covers.
    #[derive(Serialize)]
    struct Disagreement {
        corpus: String,
        section: String,
        ride: String,
        kind: String,
        section_m: f64,
        stored: Vec<(u32, u32)>,
        fresh: Vec<(u32, u32)>,
        stored_station_share_50: f64,
        stored_lateral_median_m: f64,
        line: Vec<[f64; 2]>,
        stored_pts: Vec<[f64; 2]>,
        fresh_pts: Vec<[f64; 2]>,
        ride_pts: Vec<[f64; 2]>,
    }
    let mut disagreements: Vec<Disagreement> = Vec::new();
    for s in &catalogue {
        let Some(line) = PreparedLine::new(&s.polyline, &config) else {
            continue;
        };
        let st_pts = stations(&s.polyline, 10.0);
        for id in &s.activity_ids {
            let Some(t) = track_by_id.get(id.as_str()) else {
                continue;
            };
            let fresh = line.portions(id, t);
            let stored: Vec<(u32, u32)> = s
                .activity_portions
                .iter()
                .filter(|r| r.activity_id == *id)
                .map(|r| (r.start_index, r.end_index))
                .collect();
            let kind = if fresh.is_empty() {
                "stale"
            } else if !fresh
                .iter()
                .any(|f| stored.iter().any(|r| r.0.abs_diff(f.start_index) <= 2))
            {
                "moved"
            } else {
                continue;
            };
            let per_kind = disagreements.iter().filter(|d| d.kind == kind).count();
            if per_kind >= 12 {
                continue;
            }
            let Some(&(rs, re)) = stored.first() else {
                continue;
            };
            let (rs, re) = (rs as usize, (re as usize).min(t.len()));
            let row = &t[rs..re];
            let st: Vec<f64> = st_pts.iter().map(|q| nearest_m(q, row)).collect();
            let mut lat: Vec<f64> = row.iter().map(|q| lateral_m(q, &s.polyline)).collect();
            lat.sort_by(f64::total_cmp);
            let pad = ((re - rs) / 2).max(20);
            let (fs, fe) = fresh.first().map_or((rs, re), |f| {
                (f.start_index as usize, (f.end_index as usize).min(t.len()))
            });
            disagreements.push(Disagreement {
                corpus: name.to_string(),
                section: s.id.clone(),
                ride: id.clone(),
                kind: kind.to_string(),
                section_m: s.distance_meters,
                stored: stored.clone(),
                fresh: fresh.iter().map(|f| (f.start_index, f.end_index)).collect(),
                stored_station_share_50: st.iter().filter(|d| **d <= GROUND_TOL_M).count() as f64
                    / st.len().max(1) as f64,
                stored_lateral_median_m: percentile(&lat, 0.5),
                line: thin(&s.polyline, 400),
                stored_pts: thin(row, 400),
                fresh_pts: thin(&t[fs..fe], 400),
                ride_pts: thin(&t[rs.saturating_sub(pad)..(re + pad).min(t.len())], 600),
            });
        }
    }
    std::fs::create_dir_all(out_dir).unwrap();
    std::fs::write(
        out_dir.join(format!("{name}-disagreements.json")),
        serde_json::to_string(&disagreements).unwrap(),
    )
    .unwrap();
    std::fs::write(
        out_dir.join(format!("{name}-sections.json")),
        serde_json::to_string(&rows).unwrap(),
    )
    .unwrap();

    std::fs::create_dir_all(out_dir).unwrap();
    let path = out_dir.join(format!("{name}-offers.json"));
    std::fs::write(&path, serde_json::to_string(&offers).unwrap()).unwrap();
    println!("[{name}] wrote {}", path.display());
    offers
}

fn summarise(
    name: &str,
    c: &Corpus,
    catalogue: &[FrequentSection],
    offers: &[Offer],
    attached: &HashSet<&str>,
) {
    let rides_offered: HashSet<&str> = offers.iter().map(|o| o.ride.as_str()).collect();
    let n = c.tracks.len();
    let no_section = n - attached.len();
    let no_section_offered = offers
        .iter()
        .filter(|o| o.ride_has_no_section)
        .map(|o| o.ride.as_str())
        .collect::<HashSet<_>>()
        .len();
    println!(
        "[{name}] rides {n}, attached to a section {}, with no section {no_section}; \
         rides with at least one offer {} ({:.1}%), of which with no section {no_section_offered} \
         ({:.1}% of the {no_section} unattached)",
        attached.len(),
        rides_offered.len(),
        100.0 * rides_offered.len() as f64 / n.max(1) as f64,
        100.0 * no_section_offered as f64 / no_section.max(1) as f64,
    );
    let full = offers
        .iter()
        .filter(|o| o.coverage >= FULL_COVERAGE)
        .count();
    let full_on_line = offers
        .iter()
        .filter(|o| o.coverage >= FULL_COVERAGE && o.lateral_median_m <= GROUND_TOL_M)
        .count();
    let strict_finds = offers.iter().filter(|o| o.strict_finds).count();
    let strict_full = offers
        .iter()
        .filter(|o| o.strict_finds && o.coverage >= FULL_COVERAGE)
        .count();
    let rides_full_on_line: HashSet<&str> = offers
        .iter()
        .filter(|o| o.coverage >= FULL_COVERAGE && o.lateral_median_m <= GROUND_TOL_M)
        .map(|o| o.ride.as_str())
        .collect();
    println!(
        "[{name}] offers {}: coverage >= {FULL_COVERAGE} {full}, of those median lateral <= {GROUND_TOL_M} m \
         {full_on_line} (over {} rides, {:.1}%); strict cut also finds a pass {strict_finds} \
         (full {strict_full})",
        offers.len(),
        rides_full_on_line.len(),
        100.0 * rides_full_on_line.len() as f64 / n.max(1) as f64,
    );
    let full_on_line_new_ground: HashSet<&str> = offers
        .iter()
        .filter(|o| {
            o.coverage >= FULL_COVERAGE
                && o.lateral_median_m <= GROUND_TOL_M
                && o.member_pass_share < 0.5
        })
        .map(|o| o.ride.as_str())
        .collect();
    println!(
        "[{name}] full-on-line offers over ground the ride holds no pass on: {} offers, {} rides ({:.1}%)",
        offers
            .iter()
            .filter(|o| o.coverage >= FULL_COVERAGE
                && o.lateral_median_m <= GROUND_TOL_M
                && o.member_pass_share < 0.5)
            .count(),
        full_on_line_new_ground.len(),
        100.0 * full_on_line_new_ground.len() as f64 / n.max(1) as f64,
    );
    let cross_sport = offers
        .iter()
        .filter(|o| !o.ride_sport.is_empty() && o.ride_sport != o.section_sport)
        .count();
    println!("[{name}] offers whose ride sport differs from the section's: {cross_sport}");

    let mut by_sport: BTreeMap<&str, (usize, usize, usize)> = BTreeMap::new();
    for (id, _) in &c.tracks {
        let sport = c.sports.get(id).map(String::as_str).unwrap_or("?");
        let e = by_sport.entry(sport).or_default();
        e.0 += 1;
        if rides_offered.contains(id.as_str()) {
            e.1 += 1;
        }
        if rides_full_on_line.contains(id.as_str()) {
            e.2 += 1;
        }
    }
    for (sport, (rides, offered, full)) in &by_sport {
        println!(
            "[{name}]   {sport:<14} rides {rides:>5}  offered {offered:>4} ({:>5.1}%)  full on line {full:>4} ({:>5.1}%)",
            100.0 * *offered as f64 / (*rides).max(1) as f64,
            100.0 * *full as f64 / (*rides).max(1) as f64,
        );
    }
    let mut lat: Vec<f64> = offers.iter().map(|o| o.lateral_median_m).collect();
    lat.sort_by(f64::total_cmp);
    let mut cov: Vec<f64> = offers.iter().map(|o| o.coverage).collect();
    cov.sort_by(f64::total_cmp);
    println!(
        "[{name}] offer median-lateral distribution p10 {:.0} p50 {:.0} p90 {:.0} m; coverage p10 {:.2} p50 {:.2} p90 {:.2}",
        percentile(&lat, 0.1),
        percentile(&lat, 0.5),
        percentile(&lat, 0.9),
        percentile(&cov, 0.1),
        percentile(&cov, 0.5),
        percentile(&cov, 0.9),
    );
    let reverse = offers
        .iter()
        .filter(|o| o.direction == format!("{:?}", Direction::Reverse))
        .count();
    println!(
        "[{name}] sections {} with {} member rows; offers reverse {reverse}",
        catalogue.len(),
        catalogue
            .iter()
            .map(|s| s.activity_ids.len())
            .sum::<usize>()
    );
}

fn out_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("target/scan-miss")
}

#[cfg(feature = "real-corpus")]
#[test]
fn full_corpus_scan_offers() {
    let (tracks, starts, seconds) = corpus::load_tracks("fullcorpus", 1000);
    let meta: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(corpus::dir("fullcorpus").join("_meta.json")).unwrap(),
    )
    .unwrap();
    let sports: HashMap<String, String> = tracks
        .iter()
        .map(|(id, _)| {
            let sport = meta
                .get(id)
                .and_then(|m| m.get("type"))
                .and_then(|t| t.as_str())
                .unwrap_or("Unknown")
                .to_string();
            (id.clone(), sport)
        })
        .collect();
    let c = Corpus {
        tracks,
        sports,
        starts,
        seconds,
    };
    measure("fullcorpus", &c, &out_dir());
}

#[cfg(feature = "public-corpus")]
#[test]
fn geolife_scan_offers() {
    let root = std::env::var("LAB_GEOLIFE_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("geolife/Geolife Trajectories 1.3/Data")
        });
    let (trajectories, stats) = geolife::load_geolife(&root, 200, 120);
    println!(
        "[geolife] users loaded {}, walk {} bike {} run {}",
        stats.users_loaded, stats.walk, stats.bike, stats.run
    );
    let mut by_user: BTreeMap<String, Vec<geolife::GeoTrajectory>> = BTreeMap::new();
    for t in trajectories {
        let user = t.id.split_once('_').map(|(u, _)| u.to_string()).unwrap();
        by_user.entry(user).or_default().push(t);
    }
    let mut users: Vec<(String, Vec<geolife::GeoTrajectory>)> = by_user.into_iter().collect();
    users.sort_by_key(|(u, ts)| (std::cmp::Reverse(ts.len()), u.clone()));
    // The five busiest loggers, each folded alone: one athlete's library is
    // the shape the app detects in.
    let mut all = Vec::new();
    for (user, mut mine) in users.into_iter().take(5) {
        mine.sort_by(|a, b| a.date.cmp(&b.date).then(a.id.cmp(&b.id)));
        let sports = mine
            .iter()
            .map(|t| (t.id.clone(), t.sport.clone()))
            .collect();
        let starts = mine
            .iter()
            .filter_map(|t| day_of(&t.date).map(|d| (t.id.clone(), d * 86_400)))
            .collect();
        let seconds = mine
            .iter()
            .filter(|t| t.seconds.len() == t.points.len())
            .map(|t| (t.id.clone(), t.seconds.clone()))
            .collect();
        let tracks = mine.into_iter().map(|t| (t.id, t.points)).collect();
        let c = Corpus {
            tracks,
            sports,
            starts,
            seconds,
        };
        all.extend(measure(&format!("geolife-{user}"), &c, &out_dir()));
    }
    println!("[geolife] offers over the five users: {}", all.len());
}

/// Days since 1970-01-01 for `YYYY-MM-DD`.
fn day_of(date: &str) -> Option<i64> {
    let mut it = date.split('-');
    let y: i64 = it.next()?.parse().ok()?;
    let m: i64 = it.next()?.parse().ok()?;
    let d: i64 = it.next()?.get(..2)?.parse().ok()?;
    let yy = if m <= 2 { y - 1 } else { y };
    let era = if yy >= 0 { yy } else { yy - 399 } / 400;
    let yoe = yy - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    Some(era * 146_097 + doe - 719_468)
}
