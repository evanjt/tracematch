//! Phase-1 prototype: density-grid section detection.
//!
//! Builds an inverted index from rasterised tracks to find dense regions
//! visited by ≥ min_activities tracks, then 8-connects those regions
//! into candidate sections. Compares output against the baseline pairwise
//! detection from `detect_sections_multiscale_with_progress` on the
//! Sion 426-GPX corpus.
//!
//! Acid test (medium gate):
//!   - Section count within ±10% of baseline (83–101)
//!   - ≥75% of new sections Hausdorff-match an existing baseline section
//!     within 30 m
//!   - Top 5 by visits have polyline-shape equivalents on both sides
//!   - Detection wall clock < 1.0 s
//!   - End-to-end < 2.0 s
//!
//! Run from the tracematch repo root:
//!   cargo run --release --example density_grid_report

use std::collections::{HashMap, HashSet};
use std::path::Path;
use std::time::Instant;

use rstar::{PointDistance, RTree};
use tracematch::geo_utils::haversine_distance;
use tracematch::sections::{
    FullTrackOverlap, IndexedPoint, OverlapCluster, build_rtree, filter_low_quality_sections,
    merge_nearby_sections, process_cluster, remove_overlapping_sections,
};
use tracematch::{
    FrequentSection, GpsPoint, MatchConfig, RouteSignature, SectionConfig, UnionFind,
};

#[path = "common/corpus.rs"]
mod corpus;
use corpus::{fmt_ms, load_gpx};

#[cfg(feature = "parallel")]
use rayon::prelude::*;

// ============================================================================
// Rasteriser
// ============================================================================

/// Conversion factors used by the rasteriser. Lat→meters is roughly
/// constant; lng→meters depends on cos(lat) so we cache it for a
/// reference latitude (the centroid of the corpus).
#[derive(Clone, Copy)]
struct CellGrid {
    cell_size_m: f64,
    lat_to_m: f64,    // 111_000
    lng_to_m: f64,    // 111_000 * cos(ref_lat)
    ref_lat_rad: f64, // for reproducibility
}

impl CellGrid {
    fn new(cell_size_m: f64, ref_lat_deg: f64) -> Self {
        let lat_to_m = 111_000.0;
        let lng_to_m = 111_000.0 * ref_lat_deg.to_radians().cos();
        Self {
            cell_size_m,
            lat_to_m,
            lng_to_m,
            ref_lat_rad: ref_lat_deg.to_radians(),
        }
    }

    fn cell_of(&self, lat: f64, lng: f64) -> (i32, i32) {
        let lat_idx = (lat * self.lat_to_m / self.cell_size_m).floor() as i32;
        let lng_idx = (lng * self.lng_to_m / self.cell_size_m).floor() as i32;
        (lat_idx, lng_idx)
    }

    /// Inverse: cell index → lower-left corner lat/lng (used for bbox).
    fn cell_corner(&self, cell: (i32, i32)) -> (f64, f64) {
        let lat = (cell.0 as f64 * self.cell_size_m) / self.lat_to_m;
        let lng = (cell.1 as f64 * self.cell_size_m) / self.lng_to_m;
        (lat, lng)
    }
}

/// Standard 2D Bresenham line — 4-connected. Returns every cell from
/// `start` to `end` inclusive.
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

/// Rasterise a polyline into the set of cells it occupies.
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

// ============================================================================
// Density-grid detection
// ============================================================================

#[derive(Debug, Clone)]
struct DensityCandidate {
    /// Per-track portions: (track_idx, start_idx, end_idx, distance_m).
    /// `track_idx` is the index into the `tracks` slice passed to
    /// `detect_via_density_grid`. start_idx/end_idx are indices into
    /// that track's full GPS points (start inclusive, end exclusive).
    portions: Vec<(usize, usize, usize, f64)>,
}

impl DensityCandidate {
    fn visit_count(&self) -> usize {
        self.portions.len()
    }
    fn max_distance_m(&self) -> f64 {
        self.portions.iter().map(|p| p.3).fold(0.0, f64::max)
    }
    /// Representative polyline = the longest contributing portion's points.
    fn representative_polyline<'a>(
        &self,
        tracks: &'a [(String, Vec<GpsPoint>)],
    ) -> Option<&'a [GpsPoint]> {
        let (t_idx, s, e, _) = self
            .portions
            .iter()
            .max_by(|a, b| a.3.partial_cmp(&b.3).unwrap_or(std::cmp::Ordering::Equal))?;
        let pts = &tracks[*t_idx].1;
        let end = (*e).min(pts.len());
        if *s >= end {
            return None;
        }
        Some(&pts[*s..end])
    }
}

#[derive(Default)]
struct PhaseStats {
    rasterise_ms: u128,
    invert_ms: u128,
    connect_ms: u128,
    extract_ms: u128,
    total_cells: usize,
    hot_cells: usize,
    components: usize,
    final_sections: usize,
}

/// Output of the density-grid detection. Includes the raw cell→track
/// map so callers can write a heatmap GeoJSON for visualisation.
struct DensityResult {
    candidates: Vec<DensityCandidate>,
    cell_to_tracks: HashMap<(i32, i32), Vec<u32>>,
    grid: CellGrid,
    stats: PhaseStats,
}

fn detect_via_density_grid(
    tracks: &[(String, Vec<GpsPoint>)],
    config: &SectionConfig,
) -> DensityResult {
    let mut stats = PhaseStats::default();

    if tracks.is_empty() {
        return DensityResult {
            candidates: Vec::new(),
            cell_to_tracks: HashMap::new(),
            grid: CellGrid::new(config.proximity_threshold / 2.0, 0.0),
            stats,
        };
    }

    // Reference latitude: midpoint of the corpus bbox (good enough for
    // lng cos-factor; mis-scaling by 1 km of latitude doesn't matter).
    let ref_lat: f64 = {
        let mut sum = 0.0;
        let mut n = 0;
        for (_, pts) in tracks {
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

    // ---- Phase A: rasterise each track ----
    let t0 = Instant::now();
    #[cfg(feature = "parallel")]
    let track_cells: Vec<HashSet<(i32, i32)>> = tracks
        .par_iter()
        .map(|(_, pts)| rasterise_track(pts, &grid))
        .collect();
    #[cfg(not(feature = "parallel"))]
    let track_cells: Vec<HashSet<(i32, i32)>> = tracks
        .iter()
        .map(|(_, pts)| rasterise_track(pts, &grid))
        .collect();
    stats.rasterise_ms = t0.elapsed().as_millis();

    // ---- Phase B: build inverted index ----
    let t0 = Instant::now();
    let mut cell_to_tracks: HashMap<(i32, i32), Vec<u32>> = HashMap::new();
    for (t_idx, cells) in track_cells.iter().enumerate() {
        for c in cells {
            cell_to_tracks.entry(*c).or_default().push(t_idx as u32);
        }
    }
    stats.total_cells = cell_to_tracks.len();
    stats.invert_ms = t0.elapsed().as_millis();

    // ---- Phase C: 8-connected union-find on hot cells ----
    let t0 = Instant::now();
    let mut hot_cells: Vec<(i32, i32)> = cell_to_tracks
        .iter()
        .filter(|(_, v)| v.len() >= min_acts)
        .map(|(c, _)| *c)
        .collect();
    hot_cells.sort_unstable();
    stats.hot_cells = hot_cells.len();
    let hot_set: HashSet<(i32, i32)> = hot_cells.iter().copied().collect();

    // Track-aware connectivity: two adjacent hot cells merge only if they
    // share ≥ min_activities tracks. This breaks union-find at junctions
    // where the track set diverges (a side road's cells share fewer
    // tracks with the main road's cells), so a city's road network
    // doesn't collapse into one giant component.
    //
    // For each hot cell we pre-compute a sorted Vec<u32> of track ids
    // (already deduped during Phase B insertion). Sorted intersection is
    // O(|A|+|B|) and stays cheap because per-cell track lists are small.
    let cell_tracks_sorted: HashMap<(i32, i32), Vec<u32>> = hot_cells
        .iter()
        .map(|c| {
            let mut v = cell_to_tracks.get(c).cloned().unwrap_or_default();
            v.sort_unstable();
            v.dedup();
            (*c, v)
        })
        .collect();

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

    /// Connectivity criterion: two adjacent hot cells belong to the
    /// same section iff a large fraction of their tracks coincide
    /// (Jaccard ≥ JACCARD_THRESHOLD) AND the absolute intersection size
    /// meets min_activities (so single-shared-track noise doesn't pass).
    const JACCARD_THRESHOLD: f64 = 0.8;
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

    // Build the union-find using Jaccard-based edge gating. No
    // explicit junction-cut is needed: at an intersection, the cells'
    // track sets diverge, so the Jaccard drops below threshold and
    // the edge isn't added. The "section" is naturally a maximal chain
    // of cells where the same tracks travel together.
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

    // Group cells by root representative
    let components: HashMap<(i32, i32), Vec<(i32, i32)>> = uf.groups();
    stats.components = components.len();
    stats.connect_ms = t0.elapsed().as_millis();

    // ---- Phase E: extract per-track portions per candidate ----
    // (Phase D — junction cut — deferred; revisit if gate fails on
    // T-merge artefacts.)
    let t0 = Instant::now();
    let mut sorted_components: Vec<Vec<(i32, i32)>> = components.into_values().collect();
    sorted_components.sort_by_key(|cells| cells.iter().min().copied());

    #[cfg(feature = "parallel")]
    let candidates: Vec<DensityCandidate> = sorted_components
        .par_iter()
        .filter_map(|component| {
            extract_candidate(component, &cell_to_tracks, tracks, &grid, config, min_acts)
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let candidates: Vec<DensityCandidate> = sorted_components
        .iter()
        .filter_map(|component| {
            extract_candidate(component, &cell_to_tracks, tracks, &grid, config, min_acts)
        })
        .collect();
    stats.extract_ms = t0.elapsed().as_millis();
    stats.final_sections = candidates.len();

    DensityResult {
        candidates,
        cell_to_tracks,
        grid,
        stats,
    }
}

/// For one connected component, gather contributing tracks and produce
/// a `DensityCandidate` with the longest single-track portion as the
/// representative polyline.
fn extract_candidate(
    component: &[(i32, i32)],
    cell_to_tracks: &HashMap<(i32, i32), Vec<u32>>,
    tracks: &[(String, Vec<GpsPoint>)],
    grid: &CellGrid,
    config: &SectionConfig,
    min_acts: usize,
) -> Option<DensityCandidate> {
    // Tracks that touch this component
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

    // Component cell-set for fast point-in-component lookup. We use
    // the component cells PLUS their 8-neighbours so that GPS points
    // adjacent to but just outside the component still count as inside
    // (prevents jagged truncation at cell boundaries).
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

    // For each contributing track, find the longest continuous run of
    // points whose cell is in cell_set. Skip if shorter than
    // min_section_length OR longer than max_section_length.
    let mut portions: Vec<(usize, usize, usize, f64)> = Vec::new();

    let mut t_indices: Vec<u32> = contributing.into_iter().collect();
    t_indices.sort_unstable();

    for &t_idx in &t_indices {
        let pts = &tracks[t_idx as usize].1;
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

    Some(DensityCandidate { portions })
}

/// Walk a track's GPS points, find the longest contiguous run whose
/// cells are in `cell_set`. Returns `(start_idx, end_idx, distance_m)`
/// for the run (end_idx is exclusive).
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
        let inside = cell_set.contains(&c);
        if inside {
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

// ============================================================================
// Hybrid: density candidate → OverlapCluster → process_cluster → FrequentSection
// ============================================================================

/// Synthesise an OverlapCluster from a DensityCandidate.
///
/// `process_cluster` (and `select_medoid` underneath it) consumes
/// `FullTrackOverlap` entries to learn each activity's portion range.
/// We construct N-1 anchor-paired entries: each contributing track
/// gets paired with `portions[0]` so every track surfaces in
/// `select_medoid`'s seen-set.
fn synthesise_cluster(
    candidate: &DensityCandidate,
    tracks: &[(String, Vec<GpsPoint>)],
) -> OverlapCluster {
    let anchor = candidate.portions[0];
    let anchor_id = tracks[anchor.0].0.clone();
    let anchor_range = (anchor.1, anchor.2);
    let anchor_pts = &tracks[anchor.0].1[anchor.1..anchor.2.min(tracks[anchor.0].1.len())];
    let center = if anchor_pts.is_empty() {
        GpsPoint::new(0.0, 0.0)
    } else {
        anchor_pts[anchor_pts.len() / 2]
    };

    let mut overlaps: Vec<FullTrackOverlap> = Vec::with_capacity(candidate.portions.len());
    let mut activity_ids: HashSet<String> = HashSet::with_capacity(candidate.portions.len());
    activity_ids.insert(anchor_id.clone());

    for &(t_idx, s, e, dist) in candidate.portions.iter().skip(1) {
        let other_id = tracks[t_idx].0.clone();
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

    // Single-track candidates can't satisfy min_activities anyway, but
    // guard explicitly: if we somehow have a one-portion candidate,
    // emit a self-paired entry so select_medoid still sees the activity.
    if overlaps.is_empty() {
        overlaps.push(FullTrackOverlap {
            activity_a: anchor_id.clone(),
            activity_b: anchor_id.clone(),
            range_a: anchor_range,
            range_b: anchor_range,
            center,
            overlap_length: anchor.3,
        });
    }

    OverlapCluster {
        overlaps,
        activity_ids,
    }
}

/// Full hybrid pipeline: density candidates → process_cluster → postprocess.
fn run_hybrid_pipeline(
    candidates: &[DensityCandidate],
    tracks: &[(String, Vec<GpsPoint>)],
    sport_types: &HashMap<String, String>,
    activity_to_route: &HashMap<&str, &str>,
    config: &SectionConfig,
) -> (Vec<FrequentSection>, u128, u128) {
    let track_map: HashMap<&str, &[GpsPoint]> = tracks
        .iter()
        .map(|(id, pts)| (id.as_str(), pts.as_slice()))
        .collect();

    // Resolve sport — assume single-sport corpus for prototype; pick the
    // first activity's sport, fall back to "Run".
    let sport = tracks
        .iter()
        .find_map(|(id, _)| sport_types.get(id).cloned())
        .unwrap_or_else(|| "Run".to_string());

    // 1. Synthesise OverlapClusters
    let t_synth = Instant::now();
    let clusters: Vec<OverlapCluster> = candidates
        .iter()
        .map(|c| synthesise_cluster(c, tracks))
        .collect();
    let dur_synth = t_synth.elapsed().as_millis();

    // 2. Run process_cluster on each
    let t_proc = Instant::now();
    #[cfg(feature = "parallel")]
    let mut sections: Vec<FrequentSection> = clusters
        .into_par_iter()
        .enumerate()
        .filter_map(|(idx, c)| {
            process_cluster(idx, c, &sport, &track_map, activity_to_route, config, None)
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let mut sections: Vec<FrequentSection> = clusters
        .into_iter()
        .enumerate()
        .filter_map(|(idx, c)| {
            process_cluster(idx, c, &sport, &track_map, activity_to_route, config, None)
        })
        .collect();

    // 3. Postprocess: same three public steps as detect_sections_multiscale
    sections = filter_low_quality_sections(sections, tracks.len());
    sections = merge_nearby_sections(sections, config);
    sections = remove_overlapping_sections(sections, config);

    // Renumber IDs deterministically
    for (i, s) in sections.iter_mut().enumerate() {
        s.id = format!("sec_{}_{}", sport.to_lowercase(), i);
    }
    let dur_proc = t_proc.elapsed().as_millis();

    (sections, dur_synth, dur_proc)
}

// ============================================================================
// Comparison: directed Hausdorff with R-tree acceleration
// ============================================================================

/// Sample-based Hausdorff distance in meters. Builds an R-tree of B,
/// queries each sampled point of A for the nearest in B. Returns
/// max distance. Approximate but tight enough for our 30 m gate.
fn one_sided_hausdorff_m(a: &[GpsPoint], tree_b: &RTree<IndexedPoint>) -> f64 {
    let mut max_d_m = 0.0f64;
    let step = (a.len() / 64).max(1);
    for p in a.iter().step_by(step) {
        let q = [p.latitude, p.longitude];
        if let Some(nearest) = tree_b.nearest_neighbor(&q) {
            // distance_2 returns squared degree-space distance; convert
            // to meters via 111_000 (approx).
            let d_deg = nearest.distance_2(&q).sqrt();
            let d_m = d_deg * 111_000.0;
            if d_m > max_d_m {
                max_d_m = d_m;
            }
        }
    }
    max_d_m
}

fn symmetric_hausdorff_m(
    a_points: &[GpsPoint],
    b_points: &[GpsPoint],
    a_tree: &RTree<IndexedPoint>,
    b_tree: &RTree<IndexedPoint>,
) -> f64 {
    let ab = one_sided_hausdorff_m(a_points, b_tree);
    let ba = one_sided_hausdorff_m(b_points, a_tree);
    ab.max(ba)
}

/// For each section set, count how many sections have a match in the
/// other set within `cutoff_m` Hausdorff. Returns (a→b, b→a).
fn bidirectional_match(
    a: &[FrequentSection],
    b: &[FrequentSection],
    cutoff_m: f64,
) -> (usize, usize) {
    if a.is_empty() || b.is_empty() {
        return (0, 0);
    }
    let a_trees: Vec<RTree<IndexedPoint>> = a.iter().map(|s| build_rtree(&s.polyline)).collect();
    let b_trees: Vec<RTree<IndexedPoint>> = b.iter().map(|s| build_rtree(&s.polyline)).collect();

    let mut ab = 0;
    for (ai, sa) in a.iter().enumerate() {
        for (bi, sb) in b.iter().enumerate() {
            let h = symmetric_hausdorff_m(&sa.polyline, &sb.polyline, &a_trees[ai], &b_trees[bi]);
            if h < cutoff_m {
                ab += 1;
                break;
            }
        }
    }
    let mut ba = 0;
    for (bi, sb) in b.iter().enumerate() {
        for (ai, sa) in a.iter().enumerate() {
            let h = symmetric_hausdorff_m(&sa.polyline, &sb.polyline, &a_trees[ai], &b_trees[bi]);
            if h < cutoff_m {
                ba += 1;
                break;
            }
        }
    }
    (ab, ba)
}

// ============================================================================
// GeoJSON export
// ============================================================================

/// Polygon corners (lng, lat) for a single cell, closed ring.
fn cell_polygon_coords(grid: &CellGrid, cell: (i32, i32)) -> [(f64, f64); 5] {
    let (lat_sw, lng_sw) = grid.cell_corner(cell);
    let (lat_ne, lng_ne) = grid.cell_corner((cell.0 + 1, cell.1 + 1));
    [
        (lng_sw, lat_sw),
        (lng_ne, lat_sw),
        (lng_ne, lat_ne),
        (lng_sw, lat_ne),
        (lng_sw, lat_sw),
    ]
}

fn write_cells_geojson(
    path: &str,
    cell_to_tracks: &HashMap<(i32, i32), Vec<u32>>,
    grid: &CellGrid,
    min_acts: usize,
    hot_only: bool,
) -> std::io::Result<usize> {
    use std::io::Write;
    let mut file = std::fs::File::create(path)?;
    write!(file, "{{\"type\":\"FeatureCollection\",\"features\":[")?;
    let mut first = true;
    let mut written = 0usize;
    for (cell, tracks) in cell_to_tracks {
        let count = tracks.len();
        if hot_only && count < min_acts {
            continue;
        }
        let coords = cell_polygon_coords(grid, *cell);
        let coords_str: String = coords
            .iter()
            .map(|(lng, lat)| format!("[{:.6},{:.6}]", lng, lat))
            .collect::<Vec<_>>()
            .join(",");
        if !first {
            write!(file, ",")?;
        }
        first = false;
        write!(
            file,
            "{{\"type\":\"Feature\",\"geometry\":{{\"type\":\"Polygon\",\"coordinates\":[[{}]]}},\"properties\":{{\"count\":{},\"is_hot\":{}}}}}",
            coords_str,
            count,
            count >= min_acts
        )?;
        written += 1;
    }
    write!(file, "]}}")?;
    Ok(written)
}

fn write_sections_geojson(
    path: &str,
    sections: &[FrequentSection],
    layer_name: &str,
) -> std::io::Result<usize> {
    use std::io::Write;
    let mut file = std::fs::File::create(path)?;
    write!(file, "{{\"type\":\"FeatureCollection\",\"features\":[")?;
    let mut first = true;
    for s in sections {
        if s.polyline.is_empty() {
            continue;
        }
        let coords: String = s
            .polyline
            .iter()
            .map(|p| format!("[{:.6},{:.6}]", p.longitude, p.latitude))
            .collect::<Vec<_>>()
            .join(",");
        if !first {
            write!(file, ",")?;
        }
        first = false;
        write!(
            file,
            "{{\"type\":\"Feature\",\"geometry\":{{\"type\":\"LineString\",\"coordinates\":[{}]}},\"properties\":{{\"id\":\"{}\",\"layer\":\"{}\",\"visits\":{},\"distance_m\":{:.0}}}}}",
            coords,
            s.id,
            layer_name,
            s.activity_ids.len(),
            s.distance_meters
        )?;
    }
    write!(file, "]}}")?;
    Ok(sections.len())
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    let dir = Path::new("sionrunning");
    if !dir.exists() {
        eprintln!(
            "sionrunning/ not found in cwd ({}). Run from the tracematch repo root.",
            std::env::current_dir()
                .map(|p| p.display().to_string())
                .unwrap_or_else(|_| "?".into())
        );
        return;
    }

    println!("============================================================");
    println!("  Density-grid prototype vs pairwise baseline (Sion 426 GPX)");
    println!("============================================================\n");

    // ---- Load corpus ----
    let t_load = Instant::now();
    let entries = std::fs::read_dir(dir).expect("read_dir");
    let mut raw_tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    let mut total_points = 0usize;
    for entry in entries.flatten() {
        let path = entry.path();
        if !path.extension().is_some_and(|e| e == "gpx") {
            continue;
        }
        let pts = load_gpx(&path);
        if pts.len() < 50 {
            continue;
        }
        let name = path
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();
        total_points += pts.len();
        raw_tracks.push((name, pts));
    }
    let dur_load = t_load.elapsed();
    println!(
        "## Load\n  Files loaded:        {}\n  Total GPS points:    {} ({:.0} avg/track)\n  Time:                {}\n",
        raw_tracks.len(),
        total_points,
        total_points as f64 / raw_tracks.len().max(1) as f64,
        fmt_ms(dur_load.as_millis())
    );

    let match_config = MatchConfig::default();
    let section_config = SectionConfig::default();

    // ---- Build signatures + route groups (used by baseline only) ----
    let t_sig = Instant::now();
    let signatures: Vec<RouteSignature> = raw_tracks
        .iter()
        .filter_map(|(n, p)| RouteSignature::from_points(n, p, &match_config))
        .collect();
    let groups = tracematch::group_signatures_parallel(&signatures, &match_config);
    let dur_sig = t_sig.elapsed();

    let sport_types: HashMap<String, String> = raw_tracks
        .iter()
        .map(|(id, _)| (id.clone(), "Run".to_string()))
        .collect();

    // ---- Baseline: pairwise multiscale ----
    let t_base = Instant::now();
    let baseline_result =
        tracematch::detect_sections_multiscale(&raw_tracks, &sport_types, &groups, &section_config);
    let dur_base = t_base.elapsed();
    let mut baseline = baseline_result.sections.clone();
    baseline.sort_by(|a, b| b.activity_ids.len().cmp(&a.activity_ids.len()));

    println!(
        "## Baseline (pairwise multiscale)\n  Sections:            {}\n  Total visits:        {}\n  Largest section:     {} visits, {:.0} m\n  Time:                {}\n",
        baseline.len(),
        baseline.iter().map(|s| s.activity_ids.len()).sum::<usize>(),
        baseline.first().map(|s| s.activity_ids.len()).unwrap_or(0),
        baseline.first().map(|s| s.distance_meters).unwrap_or(0.0),
        fmt_ms(dur_base.as_millis())
    );
    println!("  Top 5 baseline sections by visits:");
    for s in baseline.iter().take(5) {
        println!(
            "    {:>4}  {:>6.0} m  {}",
            s.activity_ids.len(),
            s.distance_meters,
            s.id
        );
    }
    println!();

    // ---- Density-grid stage 1: find candidate regions ----
    let t_dg = Instant::now();
    let density = detect_via_density_grid(&raw_tracks, &section_config);
    let dur_dg = t_dg.elapsed();
    let mut candidates = density.candidates;
    let stats = density.stats;
    let cell_to_tracks = density.cell_to_tracks;
    let grid = density.grid;
    candidates.sort_by(|a, b| b.visit_count().cmp(&a.visit_count()));

    println!(
        "## Density-grid stage 1 (candidate detection)\n  Candidates:          {}\n  Largest candidate:   {} contributors, {:.0} m\n  Time:                {}\n",
        candidates.len(),
        candidates.first().map(|s| s.visit_count()).unwrap_or(0),
        candidates
            .first()
            .map(|s| s.max_distance_m())
            .unwrap_or(0.0),
        fmt_ms(dur_dg.as_millis())
    );

    println!("  Phase breakdown:");
    println!("    rasterise           {:>9}", fmt_ms(stats.rasterise_ms));
    println!(
        "    invert (build idx)  {:>9}  cells: {}",
        fmt_ms(stats.invert_ms),
        stats.total_cells
    );
    println!(
        "    connect (8-conn UF) {:>9}  hot cells: {}, components: {}",
        fmt_ms(stats.connect_ms),
        stats.hot_cells,
        stats.components
    );
    println!(
        "    extract portions    {:>9}  final candidates: {}",
        fmt_ms(stats.extract_ms),
        stats.final_sections
    );
    println!();

    // ---- Hybrid: candidates → process_cluster + postprocess → FrequentSection ----
    let activity_to_route: HashMap<&str, &str> = groups
        .iter()
        .filter(|g| g.activity_ids.len() >= 2)
        .flat_map(|g| {
            g.activity_ids
                .iter()
                .map(|aid| (aid.as_str(), g.group_id.as_str()))
        })
        .collect();

    let t_hyb = Instant::now();
    let (mut hybrid, dur_synth, dur_proc) = run_hybrid_pipeline(
        &candidates,
        &raw_tracks,
        &sport_types,
        &activity_to_route,
        &section_config,
    );
    let dur_hyb = t_hyb.elapsed();
    hybrid.sort_by(|a, b| b.activity_ids.len().cmp(&a.activity_ids.len()));

    println!(
        "## Hybrid (density grid → process_cluster → postprocess)\n  Sections:            {}\n  Total visits:        {}\n  Largest section:     {} visits, {:.0} m\n  Time (total hybrid): {}\n    synthesise         {:>9}\n    process+post       {:>9}\n",
        hybrid.len(),
        hybrid.iter().map(|s| s.activity_ids.len()).sum::<usize>(),
        hybrid.first().map(|s| s.activity_ids.len()).unwrap_or(0),
        hybrid.first().map(|s| s.distance_meters).unwrap_or(0.0),
        fmt_ms(dur_hyb.as_millis()),
        fmt_ms(dur_synth),
        fmt_ms(dur_proc),
    );
    println!("  Top 5 hybrid sections by visits:");
    for s in hybrid.iter().take(5) {
        let start = s.polyline.first();
        let coord = start
            .map(|p| format!("({:.5}, {:.5})", p.latitude, p.longitude))
            .unwrap_or_else(|| "(?)".into());
        println!(
            "    {:>4}  {:>6.0} m  start {}  {}",
            s.activity_ids.len(),
            s.distance_meters,
            coord,
            s.id
        );
    }
    println!();

    // ---- Comparison: hybrid vs baseline ----
    println!("## Hybrid vs baseline comparison\n");

    let dg_full = dur_dg + dur_hyb;
    let speedup = dur_base.as_secs_f64() / dg_full.as_secs_f64().max(1e-9);
    println!(
        "  Wall clock: baseline {}, hybrid {} (detection {}, pipeline {}) → speedup {:.1}×",
        fmt_ms(dur_base.as_millis()),
        fmt_ms(dg_full.as_millis()),
        fmt_ms(dur_dg.as_millis()),
        fmt_ms(dur_hyb.as_millis()),
        speedup,
    );

    let count_lo = (baseline.len() as f64 * 0.9).floor() as usize;
    let count_hi = (baseline.len() as f64 * 1.1).ceil() as usize;
    println!(
        "  Section count: hybrid {} vs baseline {} (±10% allowed: {}..={})",
        hybrid.len(),
        baseline.len(),
        count_lo,
        count_hi,
    );

    println!("\n  Hausdorff overlap by distance threshold:");
    for cutoff in [30.0_f64, 60.0, 100.0, 150.0] {
        let (h_to_b, b_to_h) = bidirectional_match(&hybrid, &baseline, cutoff);
        println!(
            "    <{:>4.0} m: hybrid→baseline {:>3}/{} ({:>5.1}%), baseline→hybrid {:>3}/{} ({:>5.1}%)",
            cutoff,
            h_to_b,
            hybrid.len(),
            100.0 * h_to_b as f64 / hybrid.len().max(1) as f64,
            b_to_h,
            baseline.len(),
            100.0 * b_to_h as f64 / baseline.len().max(1) as f64,
        );
    }

    // ---- GeoJSON export for visualisation ----
    println!("\n## GeoJSON output (drag into geojson.io or QGIS)\n");
    let min_acts = section_config.min_activities as usize;
    match write_cells_geojson(
        "density_cells.geojson",
        &cell_to_tracks,
        &grid,
        min_acts,
        true,
    ) {
        Ok(n) => println!(
            "  density_cells.geojson         {} hot cells (≥{} tracks each)",
            n, min_acts
        ),
        Err(e) => eprintln!("  density_cells.geojson FAILED: {}", e),
    }
    match write_cells_geojson(
        "density_all_cells.geojson",
        &cell_to_tracks,
        &grid,
        min_acts,
        false,
    ) {
        Ok(n) => println!(
            "  density_all_cells.geojson     {} total cells (full heatmap)",
            n
        ),
        Err(e) => eprintln!("  density_all_cells.geojson FAILED: {}", e),
    }
    match write_sections_geojson("density_hybrid_sections.geojson", &hybrid, "hybrid") {
        Ok(n) => println!(
            "  density_hybrid_sections.geojson  {} polylines (hybrid pipeline)",
            n
        ),
        Err(e) => eprintln!("  density_hybrid_sections.geojson FAILED: {}", e),
    }
    match write_sections_geojson("density_baseline_sections.geojson", &baseline, "baseline") {
        Ok(n) => println!(
            "  density_baseline_sections.geojson {} polylines (pairwise baseline)",
            n
        ),
        Err(e) => eprintln!("  density_baseline_sections.geojson FAILED: {}", e),
    }
    let _ = dur_load + dur_sig;
}

// ============================================================================
// Unit tests (compiled with the example via #[cfg(test)])
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bresenham_horizontal_line() {
        let cells = bresenham_cells((0, 0), (5, 0));
        assert_eq!(cells, vec![(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)]);
    }

    #[test]
    fn bresenham_vertical_line() {
        let cells = bresenham_cells((0, 0), (0, 3));
        assert_eq!(cells, vec![(0, 0), (0, 1), (0, 2), (0, 3)]);
    }

    #[test]
    fn bresenham_diagonal() {
        let cells = bresenham_cells((0, 0), (3, 3));
        // 4-connected: visits both (0,0)→(1,0)→(1,1)→... or similar
        assert!(cells.contains(&(0, 0)));
        assert!(cells.contains(&(3, 3)));
        assert!(cells.len() >= 4);
    }

    #[test]
    fn cellgrid_roundtrip() {
        let g = CellGrid::new(25.0, 46.2);
        let c = g.cell_of(46.2, 7.36);
        // Same point hashes to the same cell deterministically.
        assert_eq!(c, g.cell_of(46.2, 7.36));
    }

    #[test]
    fn rasterise_single_point_track() {
        let g = CellGrid::new(25.0, 46.2);
        let pts = vec![GpsPoint::new(46.2, 7.36)];
        let cells = rasterise_track(&pts, &g);
        assert_eq!(cells.len(), 1);
    }

    #[test]
    fn rasterise_short_segment() {
        let g = CellGrid::new(25.0, 46.2);
        // Two points ~5m apart should produce 1-2 cells.
        let pts = vec![
            GpsPoint::new(46.20000, 7.36000),
            GpsPoint::new(46.20005, 7.36000),
        ];
        let cells = rasterise_track(&pts, &g);
        assert!(cells.len() >= 1 && cells.len() <= 2);
    }
}
