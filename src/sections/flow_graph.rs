//! Flow-graph section detection.
//!
//! Builds a road/trail network graph from GPS traces by tracking
//! cell-to-cell transitions (directional flow). Sections are edges
//! between divergence points — junctions where traffic naturally
//! splits into multiple directions.
//!
//! ## Algorithm
//!
//! 1. **Rasterise** all tracks into cells, recording both visits and
//!    cell-to-cell transitions.
//! 2. **Find nodes**: cells where ≥ 2 exits each carry ≥ 15% of
//!    outgoing traffic are divergence points (junctions). Cells with
//!    0–1 exits on the road network are endpoints.
//! 3. **Trace edges**: BFS along corridor cells between nodes,
//!    following highest-traffic neighbors.
//! 4. **Convert** each edge to a `FrequentSection` with a polyline
//!    derived from cell centers and route/activity counts from the
//!    inverted index.

use super::density_grid::{CellGrid, bresenham_cells};
use super::{FrequentSection, SectionConfig};
use crate::GpsPoint;
use crate::geo_utils::haversine_distance;
use std::collections::{HashMap, HashSet};

type CellPair = ((i32, i32), (i32, i32));

struct FlowGraph {
    cell_visits: HashMap<(i32, i32), u32>,
    cell_tracks: HashMap<(i32, i32), HashSet<u32>>,
    transitions: HashMap<CellPair, u32>,
    grid: CellGrid,
}

struct GraphNode {
    cell: (i32, i32),
}

struct GraphEdge {
    cells: Vec<(i32, i32)>,
    track_ids: HashSet<u32>,
}

fn build_flow_graph(tracks: &[(&str, &[GpsPoint])], config: &SectionConfig) -> FlowGraph {
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

    let cell_size_m = config.proximity_threshold;
    let grid = CellGrid::new(cell_size_m, ref_lat);

    let mut cell_visits: HashMap<(i32, i32), u32> = HashMap::new();
    let mut cell_tracks: HashMap<(i32, i32), HashSet<u32>> = HashMap::new();
    let mut transition_tracks: HashMap<CellPair, HashSet<u32>> = HashMap::new();

    for (t_idx, (_, pts)) in tracks.iter().enumerate() {
        if pts.len() < 2 {
            continue;
        }
        let mut prev_cell: Option<(i32, i32)> = None;
        for w in pts.windows(2) {
            let a = grid.cell_of(w[0].latitude, w[0].longitude);
            let b = grid.cell_of(w[1].latitude, w[1].longitude);

            let cells = if a == b {
                vec![a]
            } else {
                bresenham_cells(a, b)
            };

            for (i, &c) in cells.iter().enumerate() {
                *cell_visits.entry(c).or_insert(0) += 1;
                cell_tracks.entry(c).or_default().insert(t_idx as u32);

                let prev = if i > 0 { Some(cells[i - 1]) } else { prev_cell };
                if let Some(p) = prev
                    && p != c
                {
                    transition_tracks
                        .entry((p, c))
                        .or_default()
                        .insert(t_idx as u32);
                }
            }
            prev_cell = Some(*cells.last().unwrap_or(&a));
        }
    }

    let transitions: HashMap<CellPair, u32> = transition_tracks
        .iter()
        .map(|(k, v)| (*k, v.len() as u32))
        .collect();

    FlowGraph {
        cell_visits,
        cell_tracks,
        transitions,
        grid,
    }
}

fn find_nodes(flow: &FlowGraph, min_visits: u32, divergence_threshold: f64) -> Vec<GraphNode> {
    let road_cells: HashSet<(i32, i32)> = flow
        .cell_visits
        .iter()
        .filter(|(_, v)| **v >= min_visits)
        .map(|(c, _)| *c)
        .collect();

    let mut candidates: Vec<(i32, i32)> = Vec::new();

    for &cell in &road_cells {
        // Group 8-connected exits into 4 cardinal quadrants (N/S/E/W).
        // Diagonals go to the axis with larger delta.
        let mut dir_traffic: [u32; 4] = [0; 4]; // N, S, W, E
        for (dy, dx) in [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ] {
            let nbr = (cell.0 + dy, cell.1 + dx);
            if !road_cells.contains(&nbr) {
                continue;
            }
            let count = flow.transitions.get(&(cell, nbr)).copied().unwrap_or(0);
            let q = if dy.abs() >= dx.abs() {
                if dy < 0 { 0 } else { 1 }
            } else if dx < 0 {
                2
            } else {
                3
            };
            dir_traffic[q] += count;
        }

        let total: u32 = dir_traffic.iter().sum();
        if total == 0 {
            continue;
        }

        let min_exit_tracks = 3u32;
        let significant_dirs = dir_traffic
            .iter()
            .filter(|&&c| {
                c >= min_exit_tracks && (c as f64) >= (total as f64) * divergence_threshold
            })
            .count();

        if significant_dirs >= 3 {
            candidates.push(cell);
        }
    }

    // Merge nearby junctions: if two candidates are within 2 cells,
    // keep the one with more traffic.
    let mut merged: Vec<bool> = vec![false; candidates.len()];
    for i in 0..candidates.len() {
        if merged[i] {
            continue;
        }
        for j in (i + 1)..candidates.len() {
            if merged[j] {
                continue;
            }
            let (a, b) = (candidates[i], candidates[j]);
            let dist = ((a.0 - b.0).abs()).max((a.1 - b.1).abs());
            if dist <= 2 {
                let va = flow.cell_visits.get(&a).copied().unwrap_or(0);
                let vb = flow.cell_visits.get(&b).copied().unwrap_or(0);
                if va >= vb {
                    merged[j] = true;
                } else {
                    merged[i] = true;
                    break;
                }
            }
        }
    }

    candidates
        .into_iter()
        .zip(merged)
        .filter(|(_, m)| !m)
        .map(|(cell, _)| GraphNode { cell })
        .collect()
}

fn trace_edges(
    flow: &FlowGraph,
    nodes: &[GraphNode],
    min_visits: u32,
    min_edge_length: f64,
) -> Vec<GraphEdge> {
    let road_cells: HashSet<(i32, i32)> = flow
        .cell_visits
        .iter()
        .filter(|(_, v)| **v >= min_visits)
        .map(|(c, _)| *c)
        .collect();

    let node_set: HashSet<(i32, i32)> = nodes.iter().map(|n| n.cell).collect();
    let node_index: HashMap<(i32, i32), usize> =
        nodes.iter().enumerate().map(|(i, n)| (n.cell, i)).collect();

    let mut edges: Vec<GraphEdge> = Vec::new();
    let mut visited_edges: HashSet<((i32, i32), (i32, i32))> = HashSet::new();

    for (start_idx, node) in nodes.iter().enumerate() {
        for (dy, dx) in [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ] {
            let first_step = (node.cell.0 + dy, node.cell.1 + dx);
            if !road_cells.contains(&first_step) || node_set.contains(&first_step) {
                if node_set.contains(&first_step) {
                    let end_idx = node_index[&first_step];
                    if end_idx > start_idx {
                        let key = (node.cell, first_step);
                        if !visited_edges.contains(&key) {
                            visited_edges.insert(key);
                            let mut track_ids = HashSet::new();
                            if let Some(t) = flow.cell_tracks.get(&node.cell) {
                                track_ids.extend(t);
                            }
                            if let Some(t) = flow.cell_tracks.get(&first_step) {
                                for &id in t {
                                    track_ids.insert(id);
                                }
                            }
                            let length = flow.grid.cell_of_to_distance(node.cell, first_step);
                            if length >= min_edge_length {
                                edges.push(GraphEdge {
                                    cells: vec![node.cell, first_step],
                                    track_ids,
                                });
                            }
                        }
                    }
                }
                continue;
            }

            let mut path = vec![node.cell, first_step];
            let mut current = first_step;
            let mut visited_in_trace: HashSet<(i32, i32)> = HashSet::new();
            visited_in_trace.insert(node.cell);
            visited_in_trace.insert(first_step);

            loop {
                let mut best_nbr: Option<(i32, i32)> = None;
                let mut best_traffic: u32 = 0;

                for (dy2, dx2) in [
                    (-1, 0),
                    (1, 0),
                    (0, -1),
                    (0, 1),
                    (-1, -1),
                    (-1, 1),
                    (1, -1),
                    (1, 1),
                ] {
                    let nbr = (current.0 + dy2, current.1 + dx2);
                    if visited_in_trace.contains(&nbr) {
                        continue;
                    }
                    if !road_cells.contains(&nbr) {
                        continue;
                    }
                    let traffic = flow.cell_visits.get(&nbr).copied().unwrap_or(0);
                    if traffic > best_traffic {
                        best_traffic = traffic;
                        best_nbr = Some(nbr);
                    }
                }

                match best_nbr {
                    Some(nbr) if node_set.contains(&nbr) => {
                        path.push(nbr);
                        let end_idx = node_index[&nbr];
                        let key = if start_idx < end_idx {
                            (node.cell, nbr)
                        } else {
                            (nbr, node.cell)
                        };
                        if !visited_edges.contains(&key) {
                            visited_edges.insert(key);
                            let mut track_ids = HashSet::new();
                            for c in &path {
                                if let Some(t) = flow.cell_tracks.get(c) {
                                    for &id in t {
                                        track_ids.insert(id);
                                    }
                                }
                            }
                            let length = compute_path_length(&path, &flow.grid);
                            if length >= min_edge_length {
                                edges.push(GraphEdge {
                                    cells: path,
                                    track_ids,
                                });
                            }
                        }
                        break;
                    }
                    Some(nbr) => {
                        path.push(nbr);
                        visited_in_trace.insert(nbr);
                        current = nbr;
                    }
                    None => break,
                }
            }
        }
    }

    edges
}

/// Merge edges at pass-through junctions. If a junction connects exactly
/// 2 surviving edges, the junction is irrelevant for sections (the third
/// exit was too short) — merge the two edges into one longer section.
fn merge_pass_through_edges(mut edges: Vec<GraphEdge>, nodes: &[GraphNode]) -> Vec<GraphEdge> {
    if edges.len() < 2 {
        return edges;
    }

    let node_cells: HashSet<(i32, i32)> = nodes.iter().map(|n| n.cell).collect();

    loop {
        // Build adjacency: junction cell → list of edge indices
        let mut junction_edges: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for (i, e) in edges.iter().enumerate() {
            if let Some(&first) = e.cells.first()
                && node_cells.contains(&first)
            {
                junction_edges.entry(first).or_default().push(i);
            }
            if let Some(&last) = e.cells.last()
                && node_cells.contains(&last)
            {
                junction_edges.entry(last).or_default().push(i);
            }
        }

        // Find a pass-through junction (exactly 2 edges)
        let merge = junction_edges.iter().find_map(|(junction, edge_idxs)| {
            if edge_idxs.len() == 2 {
                Some((*junction, edge_idxs[0], edge_idxs[1]))
            } else {
                None
            }
        });

        let Some((junction, idx_a, idx_b)) = merge else {
            break;
        };

        // Merge edge B into edge A at the junction point
        let (a, b) = if idx_a < idx_b {
            let (left, right) = edges.split_at_mut(idx_b);
            (&mut left[idx_a], &right[0])
        } else {
            let (left, right) = edges.split_at_mut(idx_a);
            (&mut left[idx_b], &right[0])
        };

        // Orient: ensure a ends at junction and b starts at junction
        if a.cells.last() != Some(&junction) {
            a.cells.reverse();
        }
        let b_cells: Vec<(i32, i32)> = if b.cells.first() == Some(&junction) {
            b.cells[1..].to_vec()
        } else {
            let mut rev = b.cells.clone();
            rev.reverse();
            if rev.first() == Some(&junction) {
                rev[1..].to_vec()
            } else {
                rev
            }
        };

        a.cells.extend(b_cells);
        for id in &b.track_ids {
            a.track_ids.insert(*id);
        }

        // Remove edge B
        let remove_idx = idx_a.max(idx_b);
        edges.remove(remove_idx);
    }

    edges
}

fn compute_path_length(cells: &[(i32, i32)], grid: &CellGrid) -> f64 {
    let mut total = 0.0;
    for w in cells.windows(2) {
        total += grid.cell_of_to_distance(w[0], w[1]);
    }
    total
}

struct TrackRun {
    track_idx: usize,
    start: usize,
    end: usize,
    distance: f64,
}

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

fn edge_to_section(
    edge: &GraphEdge,
    edge_idx: usize,
    flow: &FlowGraph,
    tracks: &[(&str, &[GpsPoint])],
    sport_type: &str,
) -> Option<FrequentSection> {
    let cell_set: HashSet<(i32, i32)> = edge.cells.iter().copied().collect();

    let mut runs: Vec<TrackRun> = Vec::new();
    for &t_idx in &edge.track_ids {
        let idx = t_idx as usize;
        if idx >= tracks.len() {
            continue;
        }
        if let Some((s, e, d)) = find_best_run(tracks[idx].1, &cell_set, &flow.grid) {
            runs.push(TrackRun {
                track_idx: idx,
                start: s,
                end: e,
                distance: d,
            });
        }
    }

    if runs.is_empty() {
        return None;
    }

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

    let activity_ids: Vec<String> = edge
        .track_ids
        .iter()
        .filter_map(|&t_idx| tracks.get(t_idx as usize).map(|(id, _)| id.to_string()))
        .collect();

    let rep_id = tracks
        .get(best.track_idx)
        .map(|(id, _)| id.to_string())
        .unwrap_or_default();

    let debug_name = format!(
        "{}cells | {}trk | dist {}-{}-{}m | sel {}m",
        edge.cells.len(),
        runs.len(),
        min_dist as i32,
        median as i32,
        max_dist as i32,
        best.distance as i32,
    );

    Some(FrequentSection {
        id: format!("sec_{sport_type}_{edge_idx}").to_lowercase(),
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
        observation_count: edge.track_ids.len() as u32,
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

/// Detect sections via flow-graph analysis.
///
/// Builds a road/trail network from GPS traces by tracking cell-to-cell
/// flow, finds divergence points (junctions), and traces edges between
/// them. Each edge becomes a section.
pub(super) fn detect_sections_via_flow_graph(
    tracks: &[(&str, &[GpsPoint])],
    sport_type: &str,
    config: &SectionConfig,
) -> Vec<FrequentSection> {
    if tracks.len() < 2 {
        return vec![];
    }

    let flow = build_flow_graph(tracks, config);

    let min_visits = config.min_cell_visits;
    let divergence_threshold = config.divergence_threshold;

    let nodes = find_nodes(&flow, min_visits, divergence_threshold);
    if nodes.is_empty() {
        return vec![];
    }

    let raw_edges = trace_edges(&flow, &nodes, min_visits, config.min_section_length);
    let edges = merge_pass_through_edges(raw_edges, &nodes);

    let mut sections: Vec<FrequentSection> = edges
        .iter()
        .enumerate()
        .filter_map(|(i, e)| edge_to_section(e, i, &flow, tracks, sport_type))
        .filter(|s| s.visit_count >= config.min_activities)
        .collect();

    sections.sort_by_key(|s| std::cmp::Reverse(s.visit_count));
    sections
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flow_graph_detects_junction() {
        let config = SectionConfig {
            proximity_threshold: 25.0,
            min_section_length: 10.0,
            min_activities: 1,
            min_cell_visits: 2,
            divergence_threshold: 0.15,
            ..SectionConfig::default()
        };

        let n = 100;
        let mid = n / 2;
        let mid_lat = 46.20 + (mid as f64) * 0.00005;
        let mid_lng = 7.36001;

        // Track A: south to north
        let track_a: Vec<GpsPoint> = (0..n)
            .map(|i| GpsPoint::new(46.20 + (i as f64) * 0.00005, 7.36))
            .collect();
        // Track B: shares first half with A, then goes east
        let track_b: Vec<GpsPoint> = (0..n)
            .map(|i| {
                if i < mid {
                    GpsPoint::new(46.20 + (i as f64) * 0.00005, mid_lng)
                } else {
                    GpsPoint::new(mid_lat, mid_lng + (i - mid) as f64 * 0.00005)
                }
            })
            .collect();
        // Track C: shares first half, then goes west
        let track_c: Vec<GpsPoint> = (0..n)
            .map(|i| {
                if i < mid {
                    GpsPoint::new(46.20 + (i as f64) * 0.00005, mid_lng + 0.00001)
                } else {
                    GpsPoint::new(mid_lat, mid_lng - (i - mid) as f64 * 0.00005)
                }
            })
            .collect();

        // Need ≥3 tracks per exit direction for junction detection.
        // Duplicate each direction to simulate realistic traffic.
        let track_a2 = track_a
            .iter()
            .map(|p| GpsPoint::new(p.latitude + 0.00001, p.longitude))
            .collect::<Vec<_>>();
        let track_a3 = track_a
            .iter()
            .map(|p| GpsPoint::new(p.latitude - 0.00001, p.longitude))
            .collect::<Vec<_>>();
        let track_b2 = track_b
            .iter()
            .map(|p| GpsPoint::new(p.latitude, p.longitude + 0.00001))
            .collect::<Vec<_>>();
        let track_b3 = track_b
            .iter()
            .map(|p| GpsPoint::new(p.latitude, p.longitude - 0.00001))
            .collect::<Vec<_>>();
        let track_c2 = track_c
            .iter()
            .map(|p| GpsPoint::new(p.latitude, p.longitude + 0.00001))
            .collect::<Vec<_>>();
        let track_c3 = track_c
            .iter()
            .map(|p| GpsPoint::new(p.latitude, p.longitude - 0.00001))
            .collect::<Vec<_>>();

        let sport_tracks: Vec<(&str, &[GpsPoint])> = vec![
            ("a1", track_a.as_slice()),
            ("a2", track_a2.as_slice()),
            ("a3", track_a3.as_slice()),
            ("b1", track_b.as_slice()),
            ("b2", track_b2.as_slice()),
            ("b3", track_b3.as_slice()),
            ("c1", track_c.as_slice()),
            ("c2", track_c2.as_slice()),
            ("c3", track_c3.as_slice()),
        ];

        let flow = build_flow_graph(&sport_tracks, &config);
        let nodes = find_nodes(&flow, 2, 0.15);
        assert!(
            !nodes.is_empty(),
            "expected junction where 3 tracks diverge"
        );
    }
}
