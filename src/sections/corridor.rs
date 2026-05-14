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
    let min_dist = distances[0];
    let max_dist = distances[distances.len() - 1];

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

    let debug_name = format!(
        "{}cells | {}trk | dist {}-{}-{}m | sel {}m",
        segment_cells.len(),
        runs.len(),
        min_dist as i32,
        median as i32,
        max_dist as i32,
        best.distance as i32,
    );

    Some(FrequentSection {
        id: format!("sec_{sport_type}_{segment_idx}").to_lowercase(),
        name: Some(debug_name),
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
/// Rasterises tracks, thresholds hot cells, skeletonises the binary
/// mask via Zhang-Suen thinning, decomposes into segments, and snaps
/// each segment to an actual GPS track.
pub(super) fn detect_sections_via_corridor(
    tracks: &[(&str, &[GpsPoint])],
    sport_type: &str,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    if tracks.len() < 2 {
        return vec![];
    }

    let cell_size_m = config.proximity_threshold / 2.0;
    let min_tracks = config.min_corridor_tracks;

    let (cell_counts, cell_track_sets, grid) = rasterise_tracks(tracks, cell_size_m);

    // Threshold to binary mask
    let hot_cells: HashSet<(i32, i32)> = cell_counts
        .into_iter()
        .filter(|(_, count)| *count >= min_tracks)
        .map(|(cell, _)| cell)
        .collect();

    if hot_cells.is_empty() {
        return vec![];
    }

    // Build bounded grid and skeletonise
    let (mut bounded_grid, row_offset, col_offset) = build_bounded_grid(&hot_cells);
    zhang_suen_thin(&mut bounded_grid);

    // Decompose skeleton into segments
    let grid_segments = decompose_skeleton(&bounded_grid);

    // Convert grid segments back to cell coordinates and prune short ones
    let min_cells = (config.min_section_length / cell_size_m).ceil() as usize;
    let cell_segments: Vec<Vec<(i32, i32)>> = grid_segments
        .into_iter()
        .map(|seg| {
            seg.into_iter()
                .map(|(r, c)| (r as i32 + row_offset, c as i32 + col_offset))
                .collect::<Vec<_>>()
        })
        .filter(|seg| seg.len() >= min_cells.max(2))
        .collect();

    // Convert each segment to a section
    let mut sections: Vec<FrequentSection> = cell_segments
        .iter()
        .enumerate()
        .filter_map(|(i, seg)| {
            skeleton_segment_to_section(seg, i, &cell_track_sets, tracks, &grid, sport_type, config)
        })
        .filter(|s| s.visit_count >= config.min_activities)
        .collect();

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
            sections[0].visit_count >= 4,
            "corridor should have ≥4 traversals, got {}",
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
