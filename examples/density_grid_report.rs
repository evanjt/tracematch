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
use tracematch::sections::IndexedPoint;
use tracematch::sections::build_rtree;
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
    activity_ids: Vec<String>,
    polyline: Vec<GpsPoint>,
    distance_m: f64,
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

fn detect_via_density_grid(
    tracks: &[(String, Vec<GpsPoint>)],
    config: &SectionConfig,
) -> (Vec<DensityCandidate>, PhaseStats) {
    let mut stats = PhaseStats::default();

    if tracks.is_empty() {
        return (Vec::new(), stats);
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

    (candidates, stats)
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
    let mut valid_ids: Vec<String> = Vec::new();
    let mut best_portion: Option<(Vec<GpsPoint>, f64)> = None;

    let mut t_indices: Vec<u32> = contributing.into_iter().collect();
    t_indices.sort_unstable();

    for &t_idx in &t_indices {
        let (id, pts) = &tracks[t_idx as usize];
        let portion = longest_run_in_cells(pts, &cell_set, grid);
        if let Some((portion_pts, dist)) = portion
            && dist >= config.min_section_length
            && dist <= config.max_section_length
        {
            valid_ids.push(id.clone());
            if best_portion
                .as_ref()
                .map(|(_, d)| dist > *d)
                .unwrap_or(true)
            {
                best_portion = Some((portion_pts, dist));
            }
        }
    }

    if valid_ids.len() < min_acts {
        return None;
    }
    let (polyline, distance_m) = best_portion?;

    Some(DensityCandidate {
        activity_ids: valid_ids,
        polyline,
        distance_m,
    })
}

/// Walk a track's GPS points, find the longest contiguous run whose
/// cells are in `cell_set`. Returns the points and the haversine
/// path length of that run.
fn longest_run_in_cells(
    pts: &[GpsPoint],
    cell_set: &HashSet<(i32, i32)>,
    grid: &CellGrid,
) -> Option<(Vec<GpsPoint>, f64)> {
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
    Some((pts[best_start..best_end].to_vec(), best_dist))
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

/// For each density-grid section, find the closest baseline section by
/// Hausdorff distance. Returns (match_rate_at_30m, top5_match_count).
fn compare_section_sets(
    density: &[DensityCandidate],
    baseline: &[FrequentSection],
) -> (f64, usize) {
    if density.is_empty() {
        return (0.0, 0);
    }
    let baseline_trees: Vec<RTree<IndexedPoint>> =
        baseline.iter().map(|b| build_rtree(&b.polyline)).collect();
    let density_trees: Vec<RTree<IndexedPoint>> =
        density.iter().map(|d| build_rtree(&d.polyline)).collect();

    // For each density section, find any baseline section with Hausdorff < 30m.
    let mut matched = 0;
    for (di, d) in density.iter().enumerate() {
        for (bi, b) in baseline.iter().enumerate() {
            let h = symmetric_hausdorff_m(
                &d.polyline,
                &b.polyline,
                &density_trees[di],
                &baseline_trees[bi],
            );
            if h < 30.0 {
                matched += 1;
                break;
            }
        }
    }
    let match_rate = matched as f64 / density.len() as f64;

    // Top 5 by visits — how many of the new top-5 have a baseline equivalent?
    let mut top_new: Vec<usize> = (0..density.len()).collect();
    top_new.sort_by(|&a, &b| {
        density[b]
            .activity_ids
            .len()
            .cmp(&density[a].activity_ids.len())
    });
    top_new.truncate(5);

    let mut top5_match = 0;
    for &di in &top_new {
        for (bi, b) in baseline.iter().enumerate() {
            let h = symmetric_hausdorff_m(
                &density[di].polyline,
                &b.polyline,
                &density_trees[di],
                &baseline_trees[bi],
            );
            if h < 30.0 {
                top5_match += 1;
                break;
            }
        }
    }

    (match_rate, top5_match)
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

    // ---- Density-grid prototype ----
    let t_dg = Instant::now();
    let (mut candidates, stats) = detect_via_density_grid(&raw_tracks, &section_config);
    let dur_dg = t_dg.elapsed();
    candidates.sort_by(|a, b| b.activity_ids.len().cmp(&a.activity_ids.len()));

    println!(
        "## Density-grid prototype\n  Sections:            {}\n  Total visits:        {}\n  Largest section:     {} visits, {:.0} m\n  Time:                {}\n",
        candidates.len(),
        candidates
            .iter()
            .map(|s| s.activity_ids.len())
            .sum::<usize>(),
        candidates
            .first()
            .map(|s| s.activity_ids.len())
            .unwrap_or(0),
        candidates.first().map(|s| s.distance_m).unwrap_or(0.0),
        fmt_ms(dur_dg.as_millis())
    );
    println!("  Top 5 density-grid sections by visits:");
    for (i, c) in candidates.iter().take(5).enumerate() {
        println!(
            "    #{:>1}  {:>4} visits  {:>6.0} m  ({} polyline pts)",
            i + 1,
            c.activity_ids.len(),
            c.distance_m,
            c.polyline.len()
        );
    }
    println!();

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
        "    extract portions    {:>9}  final sections: {}",
        fmt_ms(stats.extract_ms),
        stats.final_sections
    );
    println!();

    // ---- Gate analysis ----
    println!("## Medium gate check\n");
    let baseline_count = baseline.len();
    let dg_count = candidates.len();
    let count_lo = (baseline_count as f64 * 0.9).floor() as usize;
    let count_hi = (baseline_count as f64 * 1.1).ceil() as usize;
    let count_pass = dg_count >= count_lo && dg_count <= count_hi;
    println!(
        "  Section count:  {} (baseline {}, allowed range {}..={})  {}",
        dg_count,
        baseline_count,
        count_lo,
        count_hi,
        if count_pass { "PASS" } else { "FAIL" }
    );

    // Diagnostic: print spatial relation of top-3 density to top-3 baseline.
    if !candidates.is_empty() && !baseline.is_empty() {
        println!("\n  Diagnostic — top density section anchor points:");
        for c in candidates.iter().take(3) {
            if let Some(p) = c.polyline.first() {
                println!(
                    "    density: start ({:.5}, {:.5}), dist {:.0}m, visits {}",
                    p.latitude,
                    p.longitude,
                    c.distance_m,
                    c.activity_ids.len()
                );
            }
        }
        println!("  Diagnostic — top baseline section anchor points:");
        for b in baseline.iter().take(3) {
            if let Some(p) = b.polyline.first() {
                println!(
                    "    baseline: start ({:.5}, {:.5}), dist {:.0}m, visits {}",
                    p.latitude,
                    p.longitude,
                    b.distance_meters,
                    b.activity_ids.len()
                );
            }
        }
        // What's the min Hausdorff distance for each density top-5 against ALL baseline?
        println!("  Diagnostic — density top-5 closest baseline distances:");
        for (ci, c) in candidates.iter().take(5).enumerate() {
            let d_tree = build_rtree(&c.polyline);
            let mut min_h = f64::INFINITY;
            let mut min_idx = 0;
            for (bi, b) in baseline.iter().enumerate() {
                let b_tree = build_rtree(&b.polyline);
                let h = symmetric_hausdorff_m(&c.polyline, &b.polyline, &d_tree, &b_tree);
                if h < min_h {
                    min_h = h;
                    min_idx = bi;
                }
            }
            println!(
                "    density #{}: closest baseline idx {} (visits {}, dist {:.0}m) at Hausdorff {:.1} m",
                ci + 1,
                min_idx,
                baseline[min_idx].activity_ids.len(),
                baseline[min_idx].distance_meters,
                min_h
            );
        }
        // Reverse direction: for each baseline top-5, what's the closest density?
        println!("  Diagnostic — baseline top-5 closest density distances:");
        for (bi, b) in baseline.iter().take(5).enumerate() {
            let b_tree = build_rtree(&b.polyline);
            let mut min_h = f64::INFINITY;
            let mut min_idx = 0;
            for (ci, c) in candidates.iter().enumerate() {
                let d_tree = build_rtree(&c.polyline);
                let h = symmetric_hausdorff_m(&c.polyline, &b.polyline, &d_tree, &b_tree);
                if h < min_h {
                    min_h = h;
                    min_idx = ci;
                }
            }
            println!(
                "    baseline #{}: closest density idx {} (visits {}, dist {:.0}m) at Hausdorff {:.1} m",
                bi + 1,
                min_idx,
                candidates[min_idx].activity_ids.len(),
                candidates[min_idx].distance_m,
                min_h
            );
        }
    }

    let t_cmp = Instant::now();
    let (match_rate, top5_match) = compare_section_sets(&candidates, &baseline);
    let dur_cmp = t_cmp.elapsed();
    let match_pass = match_rate >= 0.75;
    println!(
        "  Hausdorff <30m: {:.1}% of density sections match baseline ({:.0}/{}) [compare took {}]  {}",
        match_rate * 100.0,
        match_rate * candidates.len() as f64,
        candidates.len(),
        fmt_ms(dur_cmp.as_millis()),
        if match_pass { "PASS" } else { "FAIL" }
    );
    let top5_pass = top5_match >= 5;
    println!(
        "  Top-5 shape:    {}/5 of density top-5 have a baseline match  {}",
        top5_match,
        if top5_pass { "PASS" } else { "FAIL" }
    );
    let speed_pass = dur_dg.as_millis() < 1000;
    println!(
        "  Detection time: {} (gate <1000 ms)  {}",
        fmt_ms(dur_dg.as_millis()),
        if speed_pass { "PASS" } else { "FAIL" }
    );

    let speedup = dur_base.as_secs_f64() / dur_dg.as_secs_f64().max(1e-9);
    println!(
        "\n  Speedup vs baseline: {:.1}×  ({} → {})",
        speedup,
        fmt_ms(dur_base.as_millis()),
        fmt_ms(dur_dg.as_millis())
    );

    let all_pass = count_pass && match_pass && top5_pass && speed_pass;
    println!(
        "\n  Overall: {}",
        if all_pass {
            "ALL PASS — proceed to Phase 2 integration"
        } else {
            "FAIL — diagnose gap before Phase 2"
        }
    );
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
