//! Density corridor detection via morphological skeletonization.
//!
//! Finds the centerline of every genuinely dense corridor in a GPS
//! trace cloud. Only areas where many tracks converge produce sections
//! — sparse paths stay invisible.
//!
//! ## Algorithm
//!
//! 1. Rasterise all tracks into cells, counting unique tracks per cell.
//! 2. Threshold to binary mask: cells with ≥ `min_corridor_tracks` = hot.
//! 3. Zhang-Suen morphological thinning → 1-pixel-wide skeleton.
//! 4. Decompose skeleton into polyline segments at branch points.
//! 5. Prune short branches below `min_section_length`.
//! 6. Snap each skeleton segment to an actual GPS track (median-distance).
//! 7. Post-process: merge nearby, remove overlapping.
//!
//! ## References
//!
//! Zhang, T. Y. & Suen, C. Y. (1984). "A fast parallel algorithm for
//! thinning digital patterns." Communications of the ACM, 27(3), 236–239.
//!
//! Zygouras, N., et al. "Discovering Corridors from GPS Trajectories."

use super::density_grid::{CellGrid, bresenham_cells};
use super::{FrequentSection, SectionConfig};
use crate::GpsPoint;
use crate::geo_utils::haversine_distance;
use std::collections::{HashMap, HashSet};

type CellCounts = HashMap<(i32, i32), u32>;
type CellTrackSets = HashMap<(i32, i32), HashSet<u32>>;

/// Rasterise all tracks into cells and build a unique-track-count map.
fn rasterise_tracks(
    tracks: &[(&str, &[GpsPoint])],
    cell_size_m: f64,
) -> (CellCounts, CellTrackSets, CellGrid) {
    let ref_lat: f64 = {
        let mut sum = 0.0;
        let mut n = 0usize;
        for (_, pts) in tracks {
            for p in pts.iter().step_by(50) {
                sum += p.latitude;
                n += 1;
            }
        }
        if n == 0 { 0.0 } else { sum / n as f64 }
    };

    let grid = CellGrid::new(cell_size_m, ref_lat);
    let mut cell_tracks: HashMap<(i32, i32), HashSet<u32>> = HashMap::new();

    for (t_idx, (_, pts)) in tracks.iter().enumerate() {
        let mut visited: HashSet<(i32, i32)> = HashSet::new();
        if pts.is_empty() {
            continue;
        }
        let first = grid.cell_of(pts[0].latitude, pts[0].longitude);
        visited.insert(first);
        for w in pts.windows(2) {
            let a = grid.cell_of(w[0].latitude, w[0].longitude);
            let b = grid.cell_of(w[1].latitude, w[1].longitude);
            if a == b {
                visited.insert(a);
            } else {
                for c in bresenham_cells(a, b) {
                    visited.insert(c);
                }
            }
        }
        for c in visited {
            cell_tracks.entry(c).or_default().insert(t_idx as u32);
        }
    }

    let cell_counts: HashMap<(i32, i32), u32> = cell_tracks
        .iter()
        .map(|(c, ts)| (*c, ts.len() as u32))
        .collect();

    (cell_counts, cell_tracks, grid)
}

fn sorted_intersection_count(a: &[u32], b: &[u32]) -> usize {
    let mut i = 0;
    let mut j = 0;
    let mut count = 0;
    while i < a.len() && j < b.len() {
        match a[i].cmp(&b[j]) {
            std::cmp::Ordering::Less => i += 1,
            std::cmp::Ordering::Greater => j += 1,
            std::cmp::Ordering::Equal => {
                count += 1;
                i += 1;
                j += 1;
            }
        }
    }
    count
}

/// Find the longest contiguous run of track points through a cell set.
fn find_best_run(
    pts: &[GpsPoint],
    cell_set: &HashSet<(i32, i32)>,
    grid: &CellGrid,
) -> Option<(usize, usize, f64)> {
    let mut best_start = 0usize;
    let mut best_end = 0usize;
    let mut best_dist = 0.0f64;
    let mut run_start: Option<usize> = None;
    let mut run_dist = 0.0f64;

    for (i, p) in pts.iter().enumerate() {
        let c = grid.cell_of(p.latitude, p.longitude);
        if cell_set.contains(&c) {
            if run_start.is_none() {
                run_start = Some(i);
                run_dist = 0.0;
            } else if i > 0 {
                run_dist += haversine_distance(&pts[i - 1], p);
            }
        } else if let Some(s) = run_start {
            if run_dist > best_dist {
                best_start = s;
                best_end = i;
                best_dist = run_dist;
            }
            run_start = None;
        }
    }
    if let Some(s) = run_start
        && run_dist > best_dist
    {
        best_start = s;
        best_end = pts.len();
        best_dist = run_dist;
    }

    if best_dist > 0.0 {
        Some((best_start, best_end, best_dist))
    } else {
        None
    }
}


/// Detect sections via density corridor extraction.
///
/// Uses hot cells as a density filter, then extracts sections directly
/// from actual GPS track runs through dense regions. Connected components
/// of hot cells define corridor regions; tracks traversing each region
/// are grouped and the median-distance track becomes the section polyline.
pub(super) fn detect_sections_via_corridor(
    tracks: &[(&str, &[GpsPoint])],
    sport_types: &HashMap<String, String>,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    if tracks.len() < 2 {
        return vec![];
    }

    let cell_size_m = config.proximity_threshold;
    let min_tracks = config.min_corridor_tracks;

    let (_cell_counts, cell_track_sets, grid) = rasterise_tracks(tracks, cell_size_m);

    // Threshold to hot cells: only cells visited by ≥ min_tracks unique activities
    let hot_cells: HashSet<(i32, i32)> = cell_track_sets
        .iter()
        .filter(|(_, ts)| ts.len() >= min_tracks as usize)
        .map(|(c, _)| *c)
        .collect();

    if hot_cells.is_empty() {
        return vec![];
    }

    // Jaccard-gated union-find: hot cells only merge if their track sets
    // overlap sufficiently. This naturally splits corridors at intersections
    // where runners diverge into different directions — the track sets
    // diverge, Jaccard drops, cells don't merge.
    use crate::union_find::UnionFind;

    // Pre-sort track sets for efficient Jaccard computation
    let hot_tracks_sorted: HashMap<(i32, i32), Vec<u32>> = hot_cells
        .iter()
        .map(|c| {
            let mut v: Vec<u32> = cell_track_sets
                .get(c)
                .map(|ts| ts.iter().copied().collect())
                .unwrap_or_default();
            v.sort_unstable();
            v.dedup();
            (*c, v)
        })
        .collect();

    let jaccard_threshold = config.jaccard_threshold;
    let connectivity_min = 2usize;

    let mut uf: UnionFind<(i32, i32)> = UnionFind::with_capacity(hot_cells.len());
    for &c in &hot_cells {
        uf.make_set(c);
    }
    for &c in &hot_cells {
        let a_tracks = &hot_tracks_sorted[&c];
        for (dy, dx) in [(-1i32, 0), (1, 0), (0, -1), (0, 1)] {
            let nbr = (c.0 + dy, c.1 + dx);
            if nbr <= c {
                continue;
            }
            if !hot_cells.contains(&nbr) {
                continue;
            }
            let b_tracks = &hot_tracks_sorted[&nbr];
            let n_int = sorted_intersection_count(a_tracks, b_tracks);
            if n_int < connectivity_min {
                continue;
            }
            let n_union = a_tracks.len() + b_tracks.len() - n_int;
            if n_union > 0 && (n_int as f64) / (n_union as f64) >= jaccard_threshold {
                uf.union(&c, &nbr);
            }
        }
    }

    let components = uf.groups();
    let mut component_list: Vec<Vec<(i32, i32)>> = components.into_values().collect();
    component_list.sort_by_key(|cells| std::cmp::Reverse(cells.len()));

    // Track lookup map for activity_portions computation. Built once per call
    // and reused across all components — section detail screens read
    // `activity_portions` to render per-activity start/end indices, so leaving
    // it empty here produces "no data" sections in the UI.
    let track_lookup: HashMap<&str, &[GpsPoint]> =
        tracks.iter().map(|(id, pts)| (*id, *pts)).collect();

    // For each component, find all track runs through it and build a section
    // from the median-distance track
    let mut sections: Vec<FrequentSection> = Vec::new();

    for (comp_idx, component) in component_list.iter().enumerate() {
        // Dilate component cells by ±1 to absorb GPS jitter at boundaries
        let cell_set: HashSet<(i32, i32)> = {
            let mut expanded = HashSet::new();
            for &(r, c) in component {
                for dr in -1..=1i32 {
                    for dc in -1..=1i32 {
                        expanded.insert((r + dr, c + dc));
                    }
                }
            }
            expanded
        };

        // Find all tracks with contributing cells in this component
        let mut contributing_tracks: HashSet<u32> = HashSet::new();
        for c in component {
            if let Some(ts) = cell_track_sets.get(c) {
                for &t in ts {
                    contributing_tracks.insert(t);
                }
            }
        }

        struct TrackRun {
            track_idx: usize,
            start: usize,
            end: usize,
            distance: f64,
        }

        let mut runs: Vec<TrackRun> = Vec::new();
        for &t_idx in &contributing_tracks {
            let idx = t_idx as usize;
            if idx >= tracks.len() {
                continue;
            }
            if let Some((s, e, d)) = find_best_run(tracks[idx].1, &cell_set, &grid)
                && d >= config.min_section_length
            {
                runs.push(TrackRun {
                    track_idx: idx,
                    start: s,
                    end: e,
                    distance: d,
                });
            }
        }

        if runs.len() < config.min_activities as usize {
            continue;
        }

        // Select median-distance track as representative
        let mut distances: Vec<f64> = runs.iter().map(|r| r.distance).collect();
        distances.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = distances[distances.len() / 2];

        let best = runs
            .iter()
            .min_by(|a, b| {
                let da = (a.distance - median).abs();
                let db = (b.distance - median).abs();
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap();

        let polyline = tracks[best.track_idx].1[best.start..best.end].to_vec();
        if polyline.len() < 2 {
            continue;
        }

        let activity_ids: Vec<String> = runs
            .iter()
            .filter_map(|r| tracks.get(r.track_idx).map(|(id, _)| id.to_string()))
            .collect();

        let rep_id = tracks
            .get(best.track_idx)
            .map(|(id, _)| id.to_string())
            .unwrap_or_default();

        let activity_portions = super::compute_portions_for_activities(
            &activity_ids,
            &polyline,
            &track_lookup,
            config.proximity_threshold,
        );

        let section_sport = super::dominant_sport(&activity_ids, sport_types);

        sections.push(FrequentSection {
            id: format!("sec_{section_sport}_{comp_idx}").to_lowercase(),
            name: None,
            sport_type: section_sport,
            polyline,
            representative_activity_id: rep_id,
            representative_range: Some((best.start as u32, best.end as u32)),
            visit_count: activity_ids.len() as u32,
            activity_ids,
            activity_portions,
            route_ids: vec![],
            distance_meters: best.distance,
            activity_traces: HashMap::new(),
            confidence: 0.5,
            observation_count: runs.len() as u32,
            average_spread: 0.0,
            point_density: vec![],
            scale: None,
            is_user_defined: false,
            stability: 0.0,
            elevation_gain_m: None,
            avg_grade_percent: None,
            version: 1,
            updated_at: None,
            created_at: None,
            consensus_state: None,
        });
    }

    sections.sort_by_key(|s| std::cmp::Reverse(s.visit_count));
    sections
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corridor_detects_parallel_tracks() {
        let config = SectionConfig {
            proximity_threshold: 50.0,
            min_section_length: 50.0,
            min_activities: 2,
            min_corridor_tracks: 2,
            ..SectionConfig::default()
        };

        // 5 parallel tracks running north-south, spaced 5m apart
        let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
        for t in 0..5 {
            let pts: Vec<GpsPoint> = (0..200)
                .map(|i| GpsPoint::new(46.20 + (i as f64) * 0.00005, 7.36 + (t as f64) * 0.00005))
                .collect();
            tracks.push((format!("track_{}", t), pts));
        }

        let sport_tracks: Vec<(&str, &[GpsPoint])> = tracks
            .iter()
            .map(|(id, pts)| (id.as_str(), pts.as_slice()))
            .collect();

        let sport_types: HashMap<String, String> = sport_tracks
            .iter()
            .map(|(id, _)| (id.to_string(), "Run".to_string()))
            .collect();
        let sections = detect_sections_via_corridor(&sport_tracks, &sport_types, &config);
        assert!(
            !sections.is_empty(),
            "should detect corridor from 5 parallel tracks"
        );
        assert!(
            sections[0].visit_count >= 3,
            "corridor should have ≥3 traversals, got {}",
            sections[0].visit_count
        );
    }

    #[test]
    fn corridor_detects_cross_sport_section() {
        // A road traversed by cyclists and runners should produce a single
        // section whose `activity_ids` includes activities from both sports.
        // The section's `sport_type` is the dominant sport; the UI uses
        // `activity_ids` to render per-sport filter chips.
        let config = SectionConfig {
            proximity_threshold: 50.0,
            min_section_length: 50.0,
            min_activities: 2,
            min_corridor_tracks: 2,
            ..SectionConfig::default()
        };

        let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
        let mut sport_types: HashMap<String, String> = HashMap::new();
        // 3 Ride tracks
        for t in 0..3 {
            let id = format!("ride_{}", t);
            let pts: Vec<GpsPoint> = (0..200)
                .map(|i| GpsPoint::new(46.20 + (i as f64) * 0.00005, 7.36 + (t as f64) * 0.00005))
                .collect();
            sport_types.insert(id.clone(), "Ride".to_string());
            tracks.push((id, pts));
        }
        // 3 Run tracks along the same corridor, slightly offset
        for t in 0..3 {
            let id = format!("run_{}", t);
            let pts: Vec<GpsPoint> = (0..200)
                .map(|i| {
                    GpsPoint::new(
                        46.20 + (i as f64) * 0.00005,
                        7.36 + ((t + 3) as f64) * 0.00005,
                    )
                })
                .collect();
            sport_types.insert(id.clone(), "Run".to_string());
            tracks.push((id, pts));
        }

        let sport_tracks: Vec<(&str, &[GpsPoint])> = tracks
            .iter()
            .map(|(id, pts)| (id.as_str(), pts.as_slice()))
            .collect();

        let sections = detect_sections_via_corridor(&sport_tracks, &sport_types, &config);
        assert!(
            !sections.is_empty(),
            "should detect a corridor section across both sports"
        );

        let section = &sections[0];
        let sports_in_section: HashSet<&str> = section
            .activity_ids
            .iter()
            .filter_map(|id| sport_types.get(id).map(|s| s.as_str()))
            .collect();
        assert!(
            sports_in_section.len() >= 2,
            "section should contain activities from at least 2 sports, got {:?}",
            sports_in_section
        );
        assert!(
            section.activity_ids.len() >= 4,
            "section should aggregate activities across sports, got {}",
            section.activity_ids.len()
        );
        assert!(
            section.sport_type == "Ride" || section.sport_type == "Run",
            "section sport_type should be one of the contributing sports, got {}",
            section.sport_type
        );
    }

    #[test]
    fn sparse_tracks_produce_no_sections() {
        let config = SectionConfig {
            proximity_threshold: 50.0,
            min_section_length: 50.0,
            min_activities: 3,
            min_corridor_tracks: 5,
            ..SectionConfig::default()
        };

        // Only 2 tracks — below min_corridor_tracks threshold
        let track_a: Vec<GpsPoint> = (0..100)
            .map(|i| GpsPoint::new(46.20 + (i as f64) * 0.00005, 7.36))
            .collect();
        let track_b: Vec<GpsPoint> = (0..100)
            .map(|i| GpsPoint::new(46.20 + (i as f64) * 0.00005, 7.36005))
            .collect();

        let sport_tracks: Vec<(&str, &[GpsPoint])> =
            vec![("a", track_a.as_slice()), ("b", track_b.as_slice())];

        let sport_types: HashMap<String, String> = sport_tracks
            .iter()
            .map(|(id, _)| (id.to_string(), "Run".to_string()))
            .collect();
        let sections = detect_sections_via_corridor(&sport_tracks, &sport_types, &config);
        assert!(
            sections.is_empty(),
            "sparse tracks should produce no sections, got {}",
            sections.len()
        );
    }
}
