//! Hierarchical spatial pre-filter for O(N²) pair pruning.
//!
//! Section detection compares every pair of tracks within a sport to find
//! overlaps. For N tracks that's N(N-1)/2 pairs — at N=1000 that's nearly
//! 500k comparisons. The bottleneck (~70% of detection time per
//! `progress::PHASE_*` accounting) is the pairwise R-tree query.
//!
//! Most of those pairs provably can't overlap: two tracks 50km apart
//! never touch even after factoring in proximity_threshold (typically
//! 50m). The existing 5km grid in `optimized.rs` would prune them, but
//! its cells are too coarse for the common "user runs the same routes
//! repeatedly" case where every track lands in the same 1-2 cells.
//!
//! This module provides a configurable fine-grained spatial filter:
//! pick a cell size related to `proximity_threshold` (typically a few
//! hundred metres), and only emit pairs whose tracks share a cell or
//! neighbouring cell. Effective filter radius is `~1.5 × cell_size`, so
//! `cell_size = 4 × proximity_threshold` gives a 6× safety margin —
//! plenty even with GPS drift and route variation.
//!
//! The filter is conservative by design: it may emit pairs that don't
//! actually overlap, but it never DROPS a pair that could overlap. The
//! existing `find_full_track_overlap` does exact R-tree work on the
//! pruned set, so quality is unchanged vs. naive all-pairs.

use crate::GpsPoint;
use std::collections::{HashMap, HashSet};

/// A geographic grid cell at configurable size.
///
/// Coordinates are stored as integer indices for hash-friendly equality.
/// The actual cell extent in degrees is fixed at construction time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FineGridCell {
    lat_idx: i32,
    lng_idx: i32,
}

impl FineGridCell {
    /// Compute the cell containing a GPS point at the given cell size.
    ///
    /// `cell_size_deg` is the cell extent in degrees; ~0.0009° is roughly
    /// 100m at the equator (and at midlatitudes for the lat direction).
    fn from_point(lat: f64, lng: f64, cell_size_deg: f64) -> Self {
        Self {
            lat_idx: (lat / cell_size_deg).floor() as i32,
            lng_idx: (lng / cell_size_deg).floor() as i32,
        }
    }

    /// This cell plus the 8 adjacent cells (3×3 neighborhood).
    fn with_neighbors(&self) -> [FineGridCell; 9] {
        let mut out = [Self {
            lat_idx: 0,
            lng_idx: 0,
        }; 9];
        let mut i = 0;
        for dlat in -1..=1 {
            for dlng in -1..=1 {
                out[i] = FineGridCell {
                    lat_idx: self.lat_idx + dlat,
                    lng_idx: self.lng_idx + dlng,
                };
                i += 1;
            }
        }
        out
    }
}

/// Compute the set of fine grid cells that a track passes through.
///
/// `cell_size_deg` is the cell edge length in degrees. Use
/// [`cell_size_for_proximity`] to derive a sensible value from the
/// section detector's proximity threshold.
pub fn compute_fine_cells(track: &[GpsPoint], cell_size_deg: f64) -> HashSet<FineGridCell> {
    let mut cells = HashSet::with_capacity(track.len() / 8);
    for point in track {
        cells.insert(FineGridCell::from_point(
            point.latitude,
            point.longitude,
            cell_size_deg,
        ));
    }
    cells
}

/// Derive a sensible cell size from the proximity threshold.
///
/// We want the effective filter radius (~1.5 × cell_size) to be
/// comfortably larger than `proximity_threshold` to avoid false negatives
/// at cell boundaries. `4 × proximity_threshold` gives 6× safety margin,
/// which handles GPS drift and route variation while still pruning
/// aggressively in real-world data.
pub fn cell_size_for_proximity(proximity_threshold_meters: f64) -> f64 {
    // 111_000 m/degree latitude (Equator-ish; close enough at any latitude
    // for the modest precision needed here).
    let cell_size_meters = (proximity_threshold_meters * 4.0).max(50.0);
    cell_size_meters / 111_000.0
}

/// Generate the candidate pair list using the fine-grained grid filter.
///
/// Returns pairs as `(i, j)` with `i < j`. Pairs are unique. The set
/// returned is a strict subset of all N(N-1)/2 pairs — specifically, the
/// subset whose tracks share at least one cell-or-neighbour at the given
/// cell size.
///
/// This is the only pruning function in the multiscale detection path:
/// downstream code (`find_full_track_overlap`) still applies its own
/// bounding-box and R-tree checks on the result, so the filter doesn't
/// need to be exact — it only needs to never DROP a valid pair.
pub fn fine_grid_filtered_pairs(track_cells: &[HashSet<FineGridCell>]) -> Vec<(usize, usize)> {
    // Inverted index: cell -> list of track indices.
    let mut grid_index: HashMap<FineGridCell, Vec<usize>> = HashMap::new();
    for (idx, cells) in track_cells.iter().enumerate() {
        for cell in cells {
            grid_index.entry(*cell).or_default().push(idx);
        }
    }

    // Walk each track and emit pairs with neighbours via the inverted index.
    // Using a HashSet to dedupe — the same pair shows up once per shared cell.
    let mut candidate_pairs: HashSet<(usize, usize)> = HashSet::new();
    for (idx, cells) in track_cells.iter().enumerate() {
        for cell in cells {
            for neighbour in cell.with_neighbors() {
                if let Some(indices) = grid_index.get(&neighbour) {
                    for &other_idx in indices {
                        if other_idx != idx {
                            let pair = if idx < other_idx {
                                (idx, other_idx)
                            } else {
                                (other_idx, idx)
                            };
                            candidate_pairs.insert(pair);
                        }
                    }
                }
            }
        }
    }

    candidate_pairs.into_iter().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(lat: f64, lng: f64) -> GpsPoint {
        GpsPoint::new(lat, lng)
    }

    #[test]
    fn cell_size_scales_with_proximity() {
        // 50m proximity should give a cell of ~200m (50 * 4 = 200m).
        let cell_deg = cell_size_for_proximity(50.0);
        let cell_meters = cell_deg * 111_000.0;
        assert!((cell_meters - 200.0).abs() < 1.0, "got {cell_meters}");

        // Floor at 50m so tiny proximity values don't produce useless cells.
        let cell_deg_tiny = cell_size_for_proximity(5.0);
        let cell_meters_tiny = cell_deg_tiny * 111_000.0;
        assert!(cell_meters_tiny >= 50.0);
    }

    #[test]
    fn nearby_tracks_share_cells() {
        // Two tracks 100m apart should pass the filter.
        let cell_size = cell_size_for_proximity(50.0);
        let track_a = vec![pt(45.0, 7.0), pt(45.0005, 7.0)];
        let track_b = vec![pt(45.0001, 7.0), pt(45.0006, 7.0)];

        let cells_a = compute_fine_cells(&track_a, cell_size);
        let cells_b = compute_fine_cells(&track_b, cell_size);
        let pairs = fine_grid_filtered_pairs(&[cells_a, cells_b]);

        assert_eq!(pairs, vec![(0, 1)]);
    }

    #[test]
    fn far_tracks_are_pruned() {
        // Two tracks 10km apart should NOT be emitted as a candidate pair.
        // 10km ≈ 0.09° latitude, well beyond any reasonable cell radius.
        let cell_size = cell_size_for_proximity(50.0);
        let track_a = vec![pt(45.0, 7.0), pt(45.0005, 7.0)];
        let track_b = vec![pt(45.1, 7.0), pt(45.1005, 7.0)];

        let cells_a = compute_fine_cells(&track_a, cell_size);
        let cells_b = compute_fine_cells(&track_b, cell_size);
        let pairs = fine_grid_filtered_pairs(&[cells_a, cells_b]);

        assert!(pairs.is_empty(), "expected far tracks to be pruned");
    }

    #[test]
    fn cluster_of_overlapping_tracks_emits_all_pairs() {
        // 5 tracks all on roughly the same route should produce
        // C(5, 2) = 10 candidate pairs.
        let cell_size = cell_size_for_proximity(50.0);
        let tracks: Vec<Vec<GpsPoint>> = (0..5)
            .map(|i| {
                (0..20)
                    .map(|j| {
                        // Small lat offset per track to simulate GPS drift
                        pt(45.0 + (i as f64) * 0.00001 + (j as f64) * 0.0001, 7.0)
                    })
                    .collect()
            })
            .collect();

        let cell_sets: Vec<_> = tracks
            .iter()
            .map(|t| compute_fine_cells(t, cell_size))
            .collect();
        let pairs = fine_grid_filtered_pairs(&cell_sets);

        assert_eq!(pairs.len(), 10);
    }

    #[test]
    fn no_self_pairs() {
        let cell_size = cell_size_for_proximity(50.0);
        let track = vec![pt(45.0, 7.0), pt(45.0005, 7.0)];
        let cells = compute_fine_cells(&track, cell_size);
        let pairs = fine_grid_filtered_pairs(&[cells]);
        assert!(pairs.is_empty());
    }

    #[test]
    fn pairs_are_ordered_i_less_than_j() {
        let cell_size = cell_size_for_proximity(50.0);
        let tracks: Vec<Vec<GpsPoint>> = (0..3).map(|_| vec![pt(45.0, 7.0)]).collect();
        let cell_sets: Vec<_> = tracks
            .iter()
            .map(|t| compute_fine_cells(t, cell_size))
            .collect();
        let pairs = fine_grid_filtered_pairs(&cell_sets);

        for (i, j) in &pairs {
            assert!(i < j, "pair ({i}, {j}) is not ordered");
        }
    }
}
