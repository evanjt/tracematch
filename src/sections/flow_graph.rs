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
    let mut transitions: HashMap<CellPair, u32> = HashMap::new();

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
                    *transitions.entry((p, c)).or_insert(0) += 1;
                }
            }
            prev_cell = Some(*cells.last().unwrap_or(&a));
        }
    }

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

    let mut nodes = Vec::new();

    for &cell in &road_cells {
        let mut exit_counts: HashMap<(i32, i32), u32> = HashMap::new();
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
            if count > 0 {
                exit_counts.insert(nbr, count);
            }
        }

        let total_exits: u32 = exit_counts.values().sum();
        if total_exits == 0 {
            nodes.push(GraphNode { cell });
            continue;
        }

        let significant_exits: usize = exit_counts
            .values()
            .filter(|&&c| (c as f64) >= (total_exits as f64) * divergence_threshold)
            .count();

        if significant_exits >= 3 {
            nodes.push(GraphNode { cell });
        }
    }

    nodes
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

fn compute_path_length(cells: &[(i32, i32)], grid: &CellGrid) -> f64 {
    let mut total = 0.0;
    for w in cells.windows(2) {
        total += grid.cell_of_to_distance(w[0], w[1]);
    }
    total
}

fn snap_edge_to_track(
    edge: &GraphEdge,
    flow: &FlowGraph,
    tracks: &[(&str, &[GpsPoint])],
) -> (Vec<GpsPoint>, usize, f64) {
    let cell_set: HashSet<(i32, i32)> = {
        let mut s = HashSet::with_capacity(edge.cells.len() * 9);
        for c in &edge.cells {
            for dy in -1..=1i32 {
                for dx in -1..=1i32 {
                    s.insert((c.0 + dy, c.1 + dx));
                }
            }
        }
        s
    };

    let mut best_track: usize = 0;
    let mut best_pts: Vec<GpsPoint> = Vec::new();
    let mut best_dist: f64 = 0.0;

    for &t_idx in &edge.track_ids {
        let idx = t_idx as usize;
        if idx >= tracks.len() {
            continue;
        }
        let pts = tracks[idx].1;

        let mut run_start: Option<usize> = None;
        let mut run_dist = 0.0f64;
        let mut cur_best_start = 0usize;
        let mut cur_best_end = 0usize;
        let mut cur_best_dist = 0.0f64;

        for (i, p) in pts.iter().enumerate() {
            let c = flow.grid.cell_of(p.latitude, p.longitude);
            if cell_set.contains(&c) {
                if run_start.is_none() {
                    run_start = Some(i);
                    run_dist = 0.0;
                } else if i > 0 {
                    run_dist += haversine_distance(&pts[i - 1], p);
                }
            } else if let Some(s) = run_start {
                if run_dist > cur_best_dist {
                    cur_best_start = s;
                    cur_best_end = i;
                    cur_best_dist = run_dist;
                }
                run_start = None;
            }
        }
        if let Some(s) = run_start
            && run_dist > cur_best_dist
        {
            cur_best_start = s;
            cur_best_end = pts.len();
            cur_best_dist = run_dist;
        }

        if cur_best_dist > best_dist {
            best_dist = cur_best_dist;
            best_track = idx;
            best_pts = pts[cur_best_start..cur_best_end].to_vec();
        }
    }

    (best_pts, best_track, best_dist)
}

fn edge_to_section(
    edge: &GraphEdge,
    edge_idx: usize,
    flow: &FlowGraph,
    tracks: &[(&str, &[GpsPoint])],
    sport_type: &str,
) -> Option<FrequentSection> {
    let (polyline, rep_idx, distance) = snap_edge_to_track(edge, flow, tracks);
    if polyline.len() < 2 {
        return None;
    }

    let activity_ids: Vec<String> = edge
        .track_ids
        .iter()
        .filter_map(|&t_idx| tracks.get(t_idx as usize).map(|(id, _)| id.to_string()))
        .collect();

    let rep_id = tracks
        .get(rep_idx)
        .map(|(id, _)| id.to_string())
        .unwrap_or_default();

    Some(FrequentSection {
        id: format!("sec_{sport_type}_{edge_idx}").to_lowercase(),
        name: None,
        sport_type: sport_type.to_string(),
        polyline,
        representative_activity_id: rep_id,
        visit_count: activity_ids.len() as u32,
        activity_ids,
        activity_portions: vec![],
        route_ids: vec![],
        distance_meters: distance,
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

    let edges = trace_edges(&flow, &nodes, min_visits, config.min_section_length);

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
        let track_a: Vec<GpsPoint> = (0..n)
            .map(|i| GpsPoint::new(46.20 + (i as f64) * 0.00005, 7.36))
            .collect();
        let track_b: Vec<GpsPoint> = (0..n)
            .map(|i| {
                if i < n / 2 {
                    GpsPoint::new(46.20 + (i as f64) * 0.00005, 7.36001)
                } else {
                    let j = (i - n / 2) as f64;
                    GpsPoint::new(46.20 + (n as f64 / 2.0) * 0.00005, 7.36001 + j * 0.00005)
                }
            })
            .collect();

        let sport_tracks: Vec<(&str, &[GpsPoint])> =
            vec![("a", track_a.as_slice()), ("b", track_b.as_slice())];

        let flow = build_flow_graph(&sport_tracks, &config);
        let nodes = find_nodes(&flow, 2, 0.15);
        assert!(!nodes.is_empty(), "expected junction where tracks diverge");
    }
}
