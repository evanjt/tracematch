//! Cell grid the unified detector rasterises tracks onto.

use crate::GpsPoint;
use crate::geo_utils::haversine_distance;
use std::collections::HashSet;

/// Cell grid converting lat/lng ↔ integer cell indices.
///
/// Latitude → meters is roughly constant at 111 km/°. Longitude →
/// meters depends on `cos(lat)`, so we cache that factor for a
/// reference latitude (the corpus centroid). The resulting cells are
/// approximately `cell_size_m` square in physical space.
#[derive(Clone, Copy)]
pub(super) struct CellGrid {
    pub(super) cell_size_m: f64,
    pub(super) lat_to_m: f64,
    pub(super) lng_to_m: f64,
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

    /// Centre of a cell, as (lat, lng). Inverse of [`Self::cell_of`].
    pub fn centre_of(&self, cell: (i32, i32)) -> (f64, f64) {
        (
            (cell.0 as f64 + 0.5) * self.cell_size_m / self.lat_to_m,
            (cell.1 as f64 + 0.5) * self.cell_size_m / self.lng_to_m,
        )
    }
}

/// Standard 2D Bresenham — 4-connected, every cell from start to end inclusive.
pub(super) fn bresenham_cells(start: (i32, i32), end: (i32, i32)) -> Vec<(i32, i32)> {
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

/// Every contiguous run of `pts` whose cells fall in `cell_set`, as
/// `(start_idx, end_idx_exclusive, distance_m)`. Each return to the cell
/// set is its own run.
pub(super) fn runs_in_cells(
    pts: &[GpsPoint],
    cell_set: &HashSet<(i32, i32)>,
    grid: &CellGrid,
) -> Vec<(usize, usize, f64)> {
    let mut runs = Vec::new();
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
            if current_dist > 0.0 {
                runs.push((s, i, current_dist));
            }
            current_start = None;
            current_dist = 0.0;
        }
    }
    if let Some(s) = current_start
        && current_dist > 0.0
    {
        runs.push((s, pts.len(), current_dist));
    }
    runs
}
