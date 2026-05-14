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

#[allow(dead_code)]
/// Build a bounded 2D grid from hot cells for Zhang-Suen thinning.
///
/// Returns (grid, min_row, min_col) where grid[r][c] = true means hot,
/// and (min_row, min_col) is the offset to convert back to cell coords.
fn build_bounded_grid(hot_cells: &HashSet<(i32, i32)>) -> (Vec<Vec<bool>>, i32, i32) {
    if hot_cells.is_empty() {
        return (vec![], 0, 0);
    }

    let min_row = hot_cells.iter().map(|c| c.0).min().unwrap();
    let max_row = hot_cells.iter().map(|c| c.0).max().unwrap();
    let min_col = hot_cells.iter().map(|c| c.1).min().unwrap();
    let max_col = hot_cells.iter().map(|c| c.1).max().unwrap();

    // +2 padding on each side for boundary checks
    let rows = (max_row - min_row + 5) as usize;
    let cols = (max_col - min_col + 5) as usize;
    let mut grid = vec![vec![false; cols]; rows];

    for &(r, c) in hot_cells {
        let gr = (r - min_row + 2) as usize;
        let gc = (c - min_col + 2) as usize;
        if gr < rows && gc < cols {
            grid[gr][gc] = true;
        }
    }

    (grid, min_row - 2, min_col - 2)
}

#[allow(dead_code)]
/// Zhang-Suen morphological thinning.
///
/// Iteratively erodes boundary pixels while preserving connectivity
/// and endpoints. Produces a 1-pixel-wide skeleton.
fn zhang_suen_thin(grid: &mut [Vec<bool>]) {
    if grid.is_empty() || grid[0].is_empty() {
        return;
    }
    let rows = grid.len();
    let cols = grid[0].len();

    loop {
        let mut to_remove: Vec<(usize, usize)> = Vec::new();

        // Sub-iteration 1
        for r in 1..rows.saturating_sub(1) {
            for c in 1..cols.saturating_sub(1) {
                if !grid[r][c] {
                    continue;
                }
                // P2..P9 in clockwise order starting from north
                let p2 = grid[r - 1][c] as u8;
                let p3 = grid[r - 1][c + 1] as u8;
                let p4 = grid[r][c + 1] as u8;
                let p5 = grid[r + 1][c + 1] as u8;
                let p6 = grid[r + 1][c] as u8;
                let p7 = grid[r + 1][c - 1] as u8;
                let p8 = grid[r][c - 1] as u8;
                let p9 = grid[r - 1][c - 1] as u8;

                let neighbors = [p2, p3, p4, p5, p6, p7, p8, p9];
                let b = neighbors.iter().sum::<u8>();
                if !(2..=6).contains(&b) {
                    continue;
                }

                // Count 0→1 transitions in the ordered sequence
                let a = transitions_count(&neighbors);
                if a != 1 {
                    continue;
                }

                // Conditions for sub-iteration 1: p2*p4*p6 == 0 AND p4*p6*p8 == 0
                if p2 * p4 * p6 != 0 || p4 * p6 * p8 != 0 {
                    continue;
                }

                to_remove.push((r, c));
            }
        }

        for &(r, c) in &to_remove {
            grid[r][c] = false;
        }
        let removed1 = to_remove.len();
        to_remove.clear();

        // Sub-iteration 2
        for r in 1..rows.saturating_sub(1) {
            for c in 1..cols.saturating_sub(1) {
                if !grid[r][c] {
                    continue;
                }
                let p2 = grid[r - 1][c] as u8;
                let p3 = grid[r - 1][c + 1] as u8;
                let p4 = grid[r][c + 1] as u8;
                let p5 = grid[r + 1][c + 1] as u8;
                let p6 = grid[r + 1][c] as u8;
                let p7 = grid[r + 1][c - 1] as u8;
                let p8 = grid[r][c - 1] as u8;
                let p9 = grid[r - 1][c - 1] as u8;

                let neighbors = [p2, p3, p4, p5, p6, p7, p8, p9];
                let b = neighbors.iter().sum::<u8>();
                if !(2..=6).contains(&b) {
                    continue;
                }

                let a = transitions_count(&neighbors);
                if a != 1 {
                    continue;
                }

                // Conditions for sub-iteration 2: p2*p4*p8 == 0 AND p2*p6*p8 == 0
                if p2 * p4 * p8 != 0 || p2 * p6 * p8 != 0 {
                    continue;
                }

                to_remove.push((r, c));
            }
        }

        for &(r, c) in &to_remove {
            grid[r][c] = false;
        }
        let removed2 = to_remove.len();

        if removed1 + removed2 == 0 {
            break;
        }
    }
}

#[allow(dead_code)]
/// Count 0→1 transitions in the circular neighbor sequence.
fn transitions_count(neighbors: &[u8; 8]) -> u8 {
    let mut count = 0u8;
    for i in 0..8 {
        if neighbors[i] == 0 && neighbors[(i + 1) % 8] == 1 {
            count += 1;
        }
    }
    count
}

#[allow(dead_code)]
/// Count 8-connected neighbors of a skeleton cell.
fn neighbor_count(r: usize, c: usize, grid: &[Vec<bool>]) -> u8 {
    let mut count = 0u8;
    for dr in [-1i32, 0, 1] {
        for dc in [-1i32, 0, 1] {
            if dr == 0 && dc == 0 {
                continue;
            }
            let nr = r as i32 + dr;
            let nc = c as i32 + dc;
            if nr >= 0
                && nc >= 0
                && (nr as usize) < grid.len()
                && (nc as usize) < grid[0].len()
                && grid[nr as usize][nc as usize]
            {
                count += 1;
            }
        }
    }
    count
}

#[allow(dead_code)]
/// Decompose skeleton into polyline segments, splitting at branch points.
///
/// Returns segments as sequences of (row, col) in grid coordinates.
fn decompose_skeleton(grid: &[Vec<bool>]) -> Vec<Vec<(usize, usize)>> {
    if grid.is_empty() {
        return vec![];
    }
    let rows = grid.len();
    let cols = grid[0].len();

    type Cell = (usize, usize);
    type Edge = (Cell, Cell);

    // Classify cells
    let mut endpoints: Vec<Cell> = Vec::new();
    let mut branch_points: HashSet<Cell> = HashSet::new();

    for r in 0..rows {
        for c in 0..cols {
            if !grid[r][c] {
                continue;
            }
            let n = neighbor_count(r, c, grid);
            match n {
                0 | 1 => endpoints.push((r, c)),
                n if n >= 3 => {
                    branch_points.insert((r, c));
                }
                _ => {}
            }
        }
    }

    // Track visited edges (directed pairs) to avoid retracing
    let mut visited_edges: HashSet<Edge> = HashSet::new();
    let mut segments: Vec<Vec<Cell>> = Vec::new();

    // Start points: endpoints first, then branch points
    let mut start_points: Vec<Cell> = endpoints.clone();
    for &bp in &branch_points {
        start_points.push(bp);
    }

    for start in &start_points {
        if !grid[start.0][start.1] {
            continue;
        }

        let neighbors = get_neighbors(start.0, start.1, grid);
        let dirs: Vec<Cell> = neighbors
            .into_iter()
            .filter(|n| !visited_edges.contains(&(*start, *n)))
            .collect();

        for first_step in dirs {
            let mut segment: Vec<Cell> = vec![*start];
            let mut current = first_step;
            visited_edges.insert((*start, first_step));
            visited_edges.insert((first_step, *start));
            segment.push(current);

            loop {
                if branch_points.contains(&current)
                    || neighbor_count(current.0, current.1, grid) <= 1
                {
                    break;
                }

                let next_neighbors = get_neighbors(current.0, current.1, grid);
                let prev = segment[segment.len() - 2];
                let next = next_neighbors
                    .into_iter()
                    .find(|n| *n != prev && !branch_points.contains(n));

                match next {
                    Some(n) => {
                        visited_edges.insert((current, n));
                        visited_edges.insert((n, current));
                        segment.push(n);
                        current = n;
                    }
                    None => break,
                }
            }

            if segment.len() >= 2 {
                segments.push(segment);
            }
        }
    }

    // Handle isolated loops (no endpoints or branch points)
    for r in 0..rows {
        for c in 0..cols {
            if !grid[r][c] || neighbor_count(r, c, grid) != 2 {
                continue;
            }
            let neighbors = get_neighbors(r, c, grid);
            let unvisited: Vec<Cell> = neighbors
                .into_iter()
                .filter(|nb| !visited_edges.contains(&((r, c), *nb)))
                .collect();
            if unvisited.is_empty() {
                continue;
            }

            let mut segment: Vec<Cell> = vec![(r, c)];
            let mut current = unvisited[0];
            visited_edges.insert(((r, c), current));
            visited_edges.insert((current, (r, c)));
            segment.push(current);

            loop {
                if current == (r, c) {
                    break;
                }
                let next_neighbors = get_neighbors(current.0, current.1, grid);
                let prev = segment[segment.len() - 2];
                let next = next_neighbors.into_iter().find(|nb| *nb != prev);
                match next {
                    Some(n) => {
                        if visited_edges.contains(&(current, n)) {
                            break;
                        }
                        visited_edges.insert((current, n));
                        visited_edges.insert((n, current));
                        segment.push(n);
                        current = n;
                    }
                    None => break,
                }
            }

            if segment.len() >= 3 {
                segments.push(segment);
            }
        }
    }

    segments
}

#[allow(dead_code)]
/// Merge skeleton segments through branch points to form longer corridors.
///
/// After decomposition, a straight corridor with side branches is split
/// into many short pieces at every branch point. This step chains
/// adjacent segments back together when they pass roughly straight
/// through a branch point (angle deviation < 90°).
fn merge_collinear_segments(segments: Vec<Vec<(usize, usize)>>) -> Vec<Vec<(usize, usize)>> {
    if segments.len() < 2 {
        return segments;
    }

    type Cell = (usize, usize);

    // Build adjacency: branch point cell → list of (segment_idx, which_end)
    // where which_end = 0 means the segment starts there, 1 means it ends there.
    let mut junction_adj: HashMap<Cell, Vec<(usize, bool)>> = HashMap::new();
    for (i, seg) in segments.iter().enumerate() {
        if let Some(&first) = seg.first() {
            junction_adj.entry(first).or_default().push((i, false));
        }
        if let Some(&last) = seg.last()
            && seg.len() > 1
        {
            junction_adj.entry(last).or_default().push((i, true));
        }
    }

    // Direction vector at a segment endpoint (last 3 cells averaged)
    fn endpoint_direction(seg: &[(usize, usize)], at_end: bool) -> (f64, f64) {
        let n = seg.len().min(3);
        if n < 2 {
            return (0.0, 0.0);
        }
        let (a, b) = if at_end {
            (seg[seg.len() - n], seg[seg.len() - 1])
        } else {
            (seg[n - 1], seg[0])
        };
        let dr = b.0 as f64 - a.0 as f64;
        let dc = b.1 as f64 - a.1 as f64;
        let len = (dr * dr + dc * dc).sqrt().max(1e-9);
        (dr / len, dc / len)
    }

    let mut merged_into: Vec<Option<usize>> = vec![None; segments.len()];
    let mut result_segs = segments;

    // Greedy merge: at each junction with exactly 2 segment endpoints,
    // merge if the segments are roughly collinear (dot product > 0, i.e.
    // angle < 90°). Also merge at junctions with more endpoints by
    // picking the best-aligned pair.
    loop {
        let mut did_merge = false;

        for (junction, adj) in &junction_adj {
            // Collect live (non-merged) segments at this junction
            let live: Vec<(usize, bool)> = adj
                .iter()
                .filter(|(idx, _)| merged_into[*idx].is_none() && !result_segs[*idx].is_empty())
                .copied()
                .collect();

            if live.len() < 2 {
                continue;
            }

            // Find best-aligned pair
            let mut best_pair: Option<(usize, usize)> = None;
            let mut best_dot = -2.0f64;

            for i in 0..live.len() {
                for j in (i + 1)..live.len() {
                    let (idx_a, end_a) = live[i];
                    let (idx_b, end_b) = live[j];
                    let dir_a = endpoint_direction(&result_segs[idx_a], end_a);
                    let dir_b = endpoint_direction(&result_segs[idx_b], end_b);
                    // Both directions point TOWARD the junction, so collinear
                    // means they point in opposite directions (dot product < 0
                    // before negation). We want the pair where the through-angle
                    // is straightest, i.e. dot of outward directions is most negative.
                    let dot = dir_a.0 * dir_b.0 + dir_a.1 * dir_b.1;
                    // dot < 0 means roughly straight through; more negative = straighter
                    if dot < -0.0 && (-dot) > best_dot {
                        best_dot = -dot;
                        best_pair = Some((i, j));
                    }
                }
            }

            let Some((pi, pj)) = best_pair else {
                continue;
            };

            let (idx_a, end_a) = live[pi];
            let (idx_b, end_b) = live[pj];

            // Merge B into A at the junction
            let mut seg_a = std::mem::take(&mut result_segs[idx_a]);
            let seg_b = std::mem::take(&mut result_segs[idx_b]);

            // Orient: seg_a should end at junction, seg_b should start at junction
            if !end_a {
                seg_a.reverse();
            }
            let b_cells: Vec<Cell> = if !end_b {
                // seg_b starts at junction — skip the junction cell (already in seg_a)
                seg_b[1..].to_vec()
            } else {
                // seg_b ends at junction — reverse, skip junction
                let mut rev = seg_b;
                rev.reverse();
                rev[1..].to_vec()
            };

            seg_a.extend(b_cells);
            result_segs[idx_a] = seg_a;
            merged_into[idx_b] = Some(idx_a);
            // Don't update junction_adj — we'll re-scan next iteration
            let _ = junction;
            did_merge = true;
            break; // restart scan after each merge
        }

        if !did_merge {
            break;
        }

        // Rebuild adjacency for next iteration
        junction_adj.clear();
        for (i, seg) in result_segs.iter().enumerate() {
            if merged_into[i].is_some() || seg.is_empty() {
                continue;
            }
            if let Some(&first) = seg.first() {
                junction_adj.entry(first).or_default().push((i, false));
            }
            if let Some(&last) = seg.last()
                && seg.len() > 1
            {
                junction_adj.entry(last).or_default().push((i, true));
            }
        }
    }

    result_segs
        .into_iter()
        .enumerate()
        .filter(|(i, seg)| merged_into[*i].is_none() && seg.len() >= 2)
        .map(|(_, seg)| seg)
        .collect()
}

#[allow(dead_code)]
/// Get 8-connected neighbors that are skeleton cells.
fn get_neighbors(r: usize, c: usize, grid: &[Vec<bool>]) -> Vec<(usize, usize)> {
    let mut result = Vec::new();
    for dr in [-1i32, 0, 1] {
        for dc in [-1i32, 0, 1] {
            if dr == 0 && dc == 0 {
                continue;
            }
            let nr = r as i32 + dr;
            let nc = c as i32 + dc;
            if nr >= 0
                && nc >= 0
                && (nr as usize) < grid.len()
                && (nc as usize) < grid[0].len()
                && grid[nr as usize][nc as usize]
            {
                result.push((nr as usize, nc as usize));
            }
        }
    }
    result
}

/// Sorted-list intersection count for Jaccard computation.
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

#[allow(dead_code)]
/// Convert a skeleton segment to a FrequentSection by snapping to GPS tracks.
fn skeleton_segment_to_section(
    segment_cells: &[(i32, i32)],
    segment_idx: usize,
    cell_track_sets: &HashMap<(i32, i32), HashSet<u32>>,
    tracks: &[(&str, &[GpsPoint])],
    grid: &CellGrid,
    sport_type: &str,
    config: &SectionConfig,
) -> Option<FrequentSection> {
    // Dilate skeleton cells by ±1 to recapture tracks from the original
    // hot region that was thinned away. The skeleton defines the centerline
    // shape, but track counting and GPS snapping need the full corridor width.
    let cell_set: HashSet<(i32, i32)> = {
        let mut expanded = HashSet::new();
        for &(r, c) in segment_cells {
            for dr in -1..=1i32 {
                for dc in -1..=1i32 {
                    expanded.insert((r + dr, c + dc));
                }
            }
        }
        expanded
    };

    // Collect all tracks that traverse any cell in the dilated segment
    let mut contributing_tracks: HashSet<u32> = HashSet::new();
    for c in &cell_set {
        if let Some(ts) = cell_track_sets.get(c) {
            for &t in ts {
                contributing_tracks.insert(t);
            }
        }
    }

    // Find each track's longest run through the segment
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
        if let Some((s, e, d)) = find_best_run(tracks[idx].1, &cell_set, grid)
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
        return None;
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
        return None;
    }

    let activity_ids: Vec<String> = runs
        .iter()
        .filter_map(|r| tracks.get(r.track_idx).map(|(id, _)| id.to_string()))
        .collect();

    let rep_id = tracks
        .get(best.track_idx)
        .map(|(id, _)| id.to_string())
        .unwrap_or_default();

    Some(FrequentSection {
        id: format!("sec_{sport_type}_{segment_idx}").to_lowercase(),
        name: None,
        sport_type: sport_type.to_string(),
        polyline,
        representative_activity_id: rep_id,
        visit_count: activity_ids.len() as u32,
        activity_ids,
        activity_portions: vec![],
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
        version: 1,
        updated_at: None,
        created_at: None,
        consensus_state: None,
    })
}

/// Detect sections via density corridor extraction.
///
/// Uses hot cells as a density filter, then extracts sections directly
/// from actual GPS track runs through dense regions. Connected components
/// of hot cells define corridor regions; tracks traversing each region
/// are grouped and the median-distance track becomes the section polyline.
pub(super) fn detect_sections_via_corridor(
    tracks: &[(&str, &[GpsPoint])],
    sport_type: &str,
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

        sections.push(FrequentSection {
            id: format!("sec_{sport_type}_{comp_idx}").to_lowercase(),
            name: None,
            sport_type: sport_type.to_string(),
            polyline,
            representative_activity_id: rep_id,
            visit_count: activity_ids.len() as u32,
            activity_ids,
            activity_portions: vec![],
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
    fn zhang_suen_thins_rectangle_to_line() {
        // 3-wide horizontal bar should thin to 1-wide
        let mut grid = vec![
            vec![false; 12],
            vec![false; 12],
            vec![
                false, false, true, true, true, true, true, true, true, true, false, false,
            ],
            vec![
                false, false, true, true, true, true, true, true, true, true, false, false,
            ],
            vec![
                false, false, true, true, true, true, true, true, true, true, false, false,
            ],
            vec![false; 12],
            vec![false; 12],
        ];
        zhang_suen_thin(&mut grid);

        let skeleton_count: usize = grid.iter().flat_map(|r| r.iter()).filter(|&&v| v).count();
        // Should reduce to a thin line (roughly 8 cells, the length of the bar)
        assert!(
            skeleton_count <= 10,
            "skeleton should be thin, got {} cells",
            skeleton_count
        );
        assert!(
            skeleton_count >= 4,
            "skeleton should preserve length, got {} cells",
            skeleton_count
        );
    }

    #[test]
    fn zhang_suen_preserves_endpoints() {
        // Thin line should not be eroded further
        let mut grid = vec![
            vec![false; 8],
            vec![false, false, true, true, true, true, false, false],
            vec![false; 8],
        ];
        let before: usize = grid.iter().flat_map(|r| r.iter()).filter(|&&v| v).count();
        zhang_suen_thin(&mut grid);
        let after: usize = grid.iter().flat_map(|r| r.iter()).filter(|&&v| v).count();
        assert_eq!(before, after, "thin line should not be eroded further");
    }

    #[test]
    fn decompose_simple_line() {
        let grid = vec![
            vec![false; 8],
            vec![false, false, true, true, true, true, false, false],
            vec![false; 8],
        ];
        let segments = decompose_skeleton(&grid);
        assert_eq!(segments.len(), 1, "straight line = 1 segment");
        assert_eq!(segments[0].len(), 4, "should have 4 cells");
    }

    #[test]
    fn decompose_t_junction() {
        // T-shape: horizontal bar with vertical stem
        let grid = vec![
            vec![false; 9],
            vec![false, false, true, true, true, true, true, false, false],
            vec![false, false, false, false, true, false, false, false, false],
            vec![false, false, false, false, true, false, false, false, false],
            vec![false, false, false, false, true, false, false, false, false],
            vec![false; 9],
        ];
        let segments = decompose_skeleton(&grid);
        assert!(
            segments.len() >= 2,
            "T-junction should produce ≥2 segments, got {}",
            segments.len()
        );
    }

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

        let sections = detect_sections_via_corridor(&sport_tracks, "Run", &config);
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

        let sections = detect_sections_via_corridor(&sport_tracks, "Run", &config);
        assert!(
            sections.is_empty(),
            "sparse tracks should produce no sections, got {}",
            sections.len()
        );
    }
}
