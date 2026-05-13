//! Density-grid section detection.
//!
//! Replaces the pairwise `find_full_track_overlap` + `cluster_overlaps`
//! pipeline. Treats section detection as a density problem: rasterise
//! each track's polyline into a fine cell grid, build an inverted index
//! `cell → [track_ids]`, find cells visited by ≥ `min_activities` tracks,
//! and connect them via Jaccard-gated union-find into candidate sections.
//!
//! Output is `Vec<OverlapCluster>` shaped exactly like the legacy
//! `cluster_overlaps()` output, so downstream `process_cluster`
//! (medoid + consensus polyline + activity portions) runs unchanged.
//!
//! ## Algorithm
//!
//! 1. **Rasterise** each track to the set of cells its polyline touches
//!    (Bresenham line through cells of size `proximity_threshold / 2`).
//! 2. **Invert** to `cell → [track_ids]`.
//! 3. **Hot cells**: keep cells visited by ≥ `min_activities` tracks.
//! 4. **Connect**: 4-adjacency union-find, gated by Jaccard similarity
//!    of adjacent cells' track sets. Two cells merge iff their track
//!    sets overlap by ≥ `JACCARD_THRESHOLD` (and share ≥ `min_activities`
//!    tracks). At an intersection the track sets diverge, so the edge
//!    isn't added and the section splits naturally.
//! 5. **Extract** per-track portions: for each contributing track, find
//!    the longest continuous run of points whose cells fall in the
//!    component's cell-set. Discard portions shorter than
//!    `min_section_length`.
//! 6. **Synthesise** `OverlapCluster` per candidate with anchor-paired
//!    `FullTrackOverlap` entries — preserves the wire format that
//!    `select_medoid` / `process_cluster` consume.

use super::SectionConfig;
use super::overlap::{FullTrackOverlap, OverlapCluster};
use crate::GpsPoint;
use crate::geo_utils::haversine_distance;
use crate::union_find::UnionFind;
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};

/// Two adjacent hot cells merge into the same section iff their track
/// sets overlap by ≥ this Jaccard fraction. Tuned empirically against
/// the Sion 426-GPX corpus: 0.8 produces sections of reasonable length
/// (300-500 m typically) without over-merging entire shared routes.
const JACCARD_THRESHOLD: f64 = 0.8;

/// Cell grid converting lat/lng ↔ integer cell indices.
///
/// Latitude → meters is roughly constant at 111 km/°. Longitude →
/// meters depends on `cos(lat)`, so we cache that factor for a
/// reference latitude (the corpus centroid). The resulting cells are
/// approximately `cell_size_m` square in physical space.
#[derive(Clone, Copy)]
pub(super) struct CellGrid {
    cell_size_m: f64,
    lat_to_m: f64,
    lng_to_m: f64,
}

impl CellGrid {
    pub fn new(cell_size_m: f64, ref_lat_deg: f64) -> Self {
        Self {
            cell_size_m,
            lat_to_m: 111_000.0,
            lng_to_m: 111_000.0 * ref_lat_deg.to_radians().cos(),
        }
    }

    pub fn cell_of(&self, lat: f64, lng: f64) -> (i32, i32) {
        let lat_idx = (lat * self.lat_to_m / self.cell_size_m).floor() as i32;
        let lng_idx = (lng * self.lng_to_m / self.cell_size_m).floor() as i32;
        (lat_idx, lng_idx)
    }
}

/// Standard 2D Bresenham — 4-connected, every cell from start to end inclusive.
fn bresenham_cells(start: (i32, i32), end: (i32, i32)) -> Vec<(i32, i32)> {
    let (x0, y0) = start;
    let (x1, y1) = end;
    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx - dy;
    let mut x = x0;
    let mut y = y0;
    let mut cells = Vec::with_capacity((dx.max(dy) + 1) as usize);
    cells.push((x, y));
    while x != x1 || y != y1 {
        let e2 = 2 * err;
        if e2 > -dy {
            err -= dy;
            x += sx;
        }
        if e2 < dx {
            err += dx;
            y += sy;
        }
        cells.push((x, y));
    }
    cells
}

/// Rasterise a track into the set of cells its polyline touches.
fn rasterise_track(track: &[GpsPoint], grid: &CellGrid) -> HashSet<(i32, i32)> {
    let mut cells = HashSet::new();
    if track.is_empty() {
        return cells;
    }
    cells.insert(grid.cell_of(track[0].latitude, track[0].longitude));
    for w in track.windows(2) {
        let a = grid.cell_of(w[0].latitude, w[0].longitude);
        let b = grid.cell_of(w[1].latitude, w[1].longitude);
        if a == b {
            continue;
        }
        for c in bresenham_cells(a, b) {
            cells.insert(c);
        }
    }
    cells
}

/// Sorted-list intersection count. Used by the Jaccard edge gate.
fn intersection_size(a: &[u32], b: &[u32]) -> usize {
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

/// Walk a track's points, find the longest contiguous run whose cells
/// fall in `cell_set`. Returns `(start_idx, end_idx_exclusive, distance_m)`.
fn longest_run_in_cells(
    pts: &[GpsPoint],
    cell_set: &HashSet<(i32, i32)>,
    grid: &CellGrid,
) -> Option<(usize, usize, f64)> {
    let mut best_start = 0usize;
    let mut best_end = 0usize;
    let mut best_dist = 0.0f64;
    let mut current_start: Option<usize> = None;
    let mut current_dist = 0.0f64;

    for (i, p) in pts.iter().enumerate() {
        let c = grid.cell_of(p.latitude, p.longitude);
        if cell_set.contains(&c) {
            if current_start.is_none() {
                current_start = Some(i);
                current_dist = 0.0;
            } else if i > 0 {
                current_dist += haversine_distance(&pts[i - 1], p);
            }
        } else if let Some(s) = current_start {
            if current_dist > best_dist {
                best_start = s;
                best_end = i;
                best_dist = current_dist;
            }
            current_start = None;
            current_dist = 0.0;
        }
    }
    if let Some(s) = current_start
        && current_dist > best_dist
    {
        best_start = s;
        best_end = pts.len();
        best_dist = current_dist;
    }
    if best_dist == 0.0 {
        return None;
    }
    Some((best_start, best_end, best_dist))
}

/// Extract a single candidate from a connected component of hot cells.
///
/// Returns `None` if fewer than `min_activities` tracks have a portion
/// of length in `[min_section_length, max_section_length]`.
fn extract_cluster(
    component: &[(i32, i32)],
    cell_to_tracks: &HashMap<(i32, i32), Vec<u32>>,
    sport_tracks: &[(&str, &[GpsPoint])],
    grid: &CellGrid,
    config: &SectionConfig,
) -> Option<OverlapCluster> {
    let min_acts = config.min_activities as usize;

    let mut contributing: HashSet<u32> = HashSet::new();
    for c in component {
        if let Some(ts) = cell_to_tracks.get(c) {
            for t in ts {
                contributing.insert(*t);
            }
        }
    }
    if contributing.len() < min_acts {
        return None;
    }

    // Dilate component cells by 1 to absorb GPS jitter at boundaries.
    let cell_set: HashSet<(i32, i32)> = {
        let mut s = HashSet::with_capacity(component.len() * 9);
        for c in component {
            for dy in -1..=1i32 {
                for dx in -1..=1i32 {
                    s.insert((c.0 + dy, c.1 + dx));
                }
            }
        }
        s
    };

    let mut t_indices: Vec<u32> = contributing.into_iter().collect();
    t_indices.sort_unstable();

    // Per-track portion: (track_idx, start_idx, end_idx, distance_m).
    let mut portions: Vec<(usize, usize, usize, f64)> = Vec::new();
    for &t_idx in &t_indices {
        let pts = sport_tracks[t_idx as usize].1;
        if let Some((s, e, dist)) = longest_run_in_cells(pts, &cell_set, grid)
            && dist >= config.min_section_length
            && dist <= config.max_section_length
        {
            portions.push((t_idx as usize, s, e, dist));
        }
    }

    if portions.len() < min_acts {
        return None;
    }

    // Synthesise OverlapCluster with anchor-paired entries: each
    // non-anchor track gets one `FullTrackOverlap` with the anchor.
    // `select_medoid` walks the overlaps and dedup-pulls each
    // activity's portion from `range_a` / `range_b`, so every track
    // surfaces with its correct range.
    let anchor = portions[0];
    let anchor_id = sport_tracks[anchor.0].0.to_string();
    let anchor_range = (anchor.1, anchor.2);
    let anchor_pts =
        &sport_tracks[anchor.0].1[anchor.1..anchor.2.min(sport_tracks[anchor.0].1.len())];
    let center = if anchor_pts.is_empty() {
        GpsPoint::new(0.0, 0.0)
    } else {
        anchor_pts[anchor_pts.len() / 2]
    };

    let mut overlaps: Vec<FullTrackOverlap> = Vec::with_capacity(portions.len());
    let mut activity_ids: HashSet<String> = HashSet::with_capacity(portions.len());
    activity_ids.insert(anchor_id.clone());

    for &(t_idx, s, e, dist) in portions.iter().skip(1) {
        let other_id = sport_tracks[t_idx].0.to_string();
        activity_ids.insert(other_id.clone());
        overlaps.push(FullTrackOverlap {
            activity_a: anchor_id.clone(),
            activity_b: other_id,
            range_a: anchor_range,
            range_b: (s, e),
            center,
            overlap_length: dist.min(anchor.3),
        });
    }

    // Guard: single-track cluster (shouldn't happen with min_acts ≥ 2,
    // but emit a self-paired entry rather than producing a malformed
    // cluster with zero overlaps).
    if overlaps.is_empty() {
        overlaps.push(FullTrackOverlap {
            activity_a: anchor_id.clone(),
            activity_b: anchor_id,
            range_a: anchor_range,
            range_b: anchor_range,
            center,
            overlap_length: anchor.3,
        });
    }

    Some(OverlapCluster {
        overlaps,
        activity_ids,
    })
}

/// Detect overlap clusters via density grid for one sport's tracks.
///
/// Replaces the legacy `find_full_track_overlap` + `cluster_overlaps`
/// pipeline. Output is consumed by `process_cluster` exactly as before.
pub(super) fn detect_clusters_via_density(
    sport_tracks: &[(&str, &[GpsPoint])],
    config: &SectionConfig,
) -> Vec<OverlapCluster> {
    if sport_tracks.is_empty() {
        return Vec::new();
    }

    // Reference latitude for the lng cos-factor — corpus centroid.
    let ref_lat: f64 = {
        let mut sum = 0.0;
        let mut n = 0usize;
        for (_, pts) in sport_tracks {
            for p in pts.iter().step_by(50) {
                sum += p.latitude;
                n += 1;
            }
        }
        if n == 0 { 0.0 } else { sum / n as f64 }
    };

    let cell_size_m = config.proximity_threshold / 2.0;
    let grid = CellGrid::new(cell_size_m, ref_lat);
    let min_acts = config.min_activities as usize;

    // Phase A: rasterise.
    #[cfg(feature = "parallel")]
    let track_cells: Vec<HashSet<(i32, i32)>> = sport_tracks
        .par_iter()
        .map(|(_, pts)| rasterise_track(pts, &grid))
        .collect();
    #[cfg(not(feature = "parallel"))]
    let track_cells: Vec<HashSet<(i32, i32)>> = sport_tracks
        .iter()
        .map(|(_, pts)| rasterise_track(pts, &grid))
        .collect();

    // Phase B: inverted index cell → tracks.
    let mut cell_to_tracks: HashMap<(i32, i32), Vec<u32>> = HashMap::new();
    for (t_idx, cells) in track_cells.iter().enumerate() {
        for c in cells {
            cell_to_tracks.entry(*c).or_default().push(t_idx as u32);
        }
    }

    // Phase C: hot cells + Jaccard-gated 4-connected union-find.
    let mut hot_cells: Vec<(i32, i32)> = cell_to_tracks
        .iter()
        .filter(|(_, v)| v.len() >= min_acts)
        .map(|(c, _)| *c)
        .collect();
    hot_cells.sort_unstable();
    let hot_set: HashSet<(i32, i32)> = hot_cells.iter().copied().collect();

    let cell_tracks_sorted: HashMap<(i32, i32), Vec<u32>> = hot_cells
        .iter()
        .map(|c| {
            let mut v = cell_to_tracks.get(c).cloned().unwrap_or_default();
            v.sort_unstable();
            v.dedup();
            (*c, v)
        })
        .collect();

    let edge_ok = |a: &[u32], b: &[u32]| -> bool {
        let n_int = intersection_size(a, b);
        if n_int < min_acts {
            return false;
        }
        let n_union = a.len() + b.len() - n_int;
        if n_union == 0 {
            return false;
        }
        (n_int as f64) / (n_union as f64) >= JACCARD_THRESHOLD
    };

    let mut uf: UnionFind<(i32, i32)> = UnionFind::with_capacity(hot_cells.len());
    for c in &hot_cells {
        uf.make_set(*c);
    }
    for &c in &hot_cells {
        let a_tracks = &cell_tracks_sorted[&c];
        for (dy, dx) in [(-1i32, 0), (1, 0), (0, -1), (0, 1)] {
            let nbr = (c.0 + dy, c.1 + dx);
            if nbr <= c {
                continue;
            }
            if !hot_set.contains(&nbr) {
                continue;
            }
            let b_tracks = &cell_tracks_sorted[&nbr];
            if edge_ok(a_tracks, b_tracks) {
                uf.union(&c, &nbr);
            }
        }
    }

    // Components, sorted for determinism.
    let components_map = uf.groups();
    let mut components: Vec<Vec<(i32, i32)>> = components_map.into_values().collect();
    components.sort_by_key(|cells| cells.iter().min().copied());

    // Phase D: extract clusters per component (parallel).
    #[cfg(feature = "parallel")]
    let clusters: Vec<OverlapCluster> = components
        .par_iter()
        .filter_map(|component| {
            extract_cluster(component, &cell_to_tracks, sport_tracks, &grid, config)
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let clusters: Vec<OverlapCluster> = components
        .iter()
        .filter_map(|component| {
            extract_cluster(component, &cell_to_tracks, sport_tracks, &grid, config)
        })
        .collect();

    clusters
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bresenham_horizontal() {
        let cells = bresenham_cells((0, 0), (3, 0));
        assert_eq!(cells, vec![(0, 0), (1, 0), (2, 0), (3, 0)]);
    }

    #[test]
    fn bresenham_vertical() {
        let cells = bresenham_cells((0, 0), (0, 3));
        assert_eq!(cells, vec![(0, 0), (0, 1), (0, 2), (0, 3)]);
    }

    #[test]
    fn cellgrid_deterministic() {
        let g = CellGrid::new(25.0, 46.2);
        let a = g.cell_of(46.2, 7.36);
        let b = g.cell_of(46.2, 7.36);
        assert_eq!(a, b);
    }

    #[test]
    fn rasterise_short_segment() {
        let g = CellGrid::new(25.0, 46.2);
        let pts = vec![
            GpsPoint::new(46.20000, 7.36000),
            GpsPoint::new(46.20005, 7.36000),
        ];
        let cells = rasterise_track(&pts, &g);
        assert!(!cells.is_empty());
    }

    #[test]
    fn intersection_size_basic() {
        assert_eq!(intersection_size(&[1, 2, 3], &[2, 3, 4]), 2);
        assert_eq!(intersection_size(&[1, 2, 3], &[4, 5, 6]), 0);
        assert_eq!(intersection_size(&[1, 2, 3], &[1, 2, 3]), 3);
    }

    #[test]
    fn longest_run_finds_contiguous() {
        let g = CellGrid::new(25.0, 46.2);
        let mut pts = Vec::new();
        for i in 0..50 {
            pts.push(GpsPoint::new(46.2 + (i as f64) * 0.0001, 7.36));
        }
        let mut cells = HashSet::new();
        for p in pts.iter().take(20) {
            cells.insert(g.cell_of(p.latitude, p.longitude));
        }
        let result = longest_run_in_cells(&pts, &cells, &g);
        assert!(result.is_some());
        let (start, end, _dist) = result.unwrap();
        assert_eq!(start, 0);
        // The first 20 points span ~9 distinct cells; later points may
        // share trailing cells, so the run can extend a few points past
        // index 20. Just require it doesn't extend through the whole
        // track (i.e., did terminate before the end).
        assert!(
            end < pts.len(),
            "run shouldn't reach end of track; got end={}",
            end
        );
    }

    /// Two parallel tracks 5 m apart should land in the same component
    /// (track sets share fully → Jaccard 1.0).
    #[test]
    fn parallel_tracks_form_one_cluster() {
        let mut config = SectionConfig::default();
        config.min_activities = 2;
        config.min_section_length = 50.0;
        config.proximity_threshold = 50.0;

        let track_a: Vec<GpsPoint> = (0..100)
            .map(|i| GpsPoint::new(46.20 + (i as f64) * 0.00005, 7.36))
            .collect();
        let track_b: Vec<GpsPoint> = (0..100)
            .map(|i| GpsPoint::new(46.20 + (i as f64) * 0.00005, 7.36005))
            .collect();
        let sport_tracks: Vec<(&str, &[GpsPoint])> =
            vec![("a", track_a.as_slice()), ("b", track_b.as_slice())];

        let clusters = detect_clusters_via_density(&sport_tracks, &config);
        assert!(!clusters.is_empty(), "expected at least one cluster");
        let total: usize = clusters.iter().map(|c| c.activity_ids.len()).sum();
        assert!(total >= 2, "expected both tracks present: total={}", total);
    }

    /// Disjoint tracks (far apart) produce zero clusters.
    #[test]
    fn far_apart_tracks_produce_no_cluster() {
        let mut config = SectionConfig::default();
        config.min_activities = 2;
        config.min_section_length = 50.0;

        let track_a: Vec<GpsPoint> = (0..100)
            .map(|i| GpsPoint::new(46.20 + (i as f64) * 0.00005, 7.36))
            .collect();
        let track_b: Vec<GpsPoint> = (0..100)
            .map(|i| GpsPoint::new(47.20 + (i as f64) * 0.00005, 8.36))
            .collect();
        let sport_tracks: Vec<(&str, &[GpsPoint])> =
            vec![("a", track_a.as_slice()), ("b", track_b.as_slice())];

        let clusters = detect_clusters_via_density(&sport_tracks, &config);
        assert!(
            clusters.is_empty(),
            "expected no clusters, got {}",
            clusters.len()
        );
    }
}
