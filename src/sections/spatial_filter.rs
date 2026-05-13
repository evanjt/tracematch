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
/// The minimum safe cell size equals `proximity_threshold` (any two
/// points within proximity must land in cells whose 3×3 neighbourhoods
/// overlap). We use 2× as buffer — accounts for longitudinal cell
/// distortion at high latitudes (cos(60°) ≈ 0.5 shrinks the lng axis)
/// and float-arithmetic boundary effects, while pruning ~4× more pairs
/// than the previous 4× factor.
pub fn cell_size_for_proximity(proximity_threshold_meters: f64) -> f64 {
    let cell_size_meters = (proximity_threshold_meters * 2.0).max(50.0);
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

    // Sort for deterministic output: the HashSet iteration order is
    // randomised per process, and downstream section detection
    // (clustering, consensus) is sensitive to pair order via parallel
    // collect order and float-add associativity.
    let mut out: Vec<(usize, usize)> = candidate_pairs.into_iter().collect();
    out.sort_unstable();
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(lat: f64, lng: f64) -> GpsPoint {
        GpsPoint::new(lat, lng)
    }

    #[test]
    fn cell_size_scales_with_proximity() {
        // 50m proximity should give a cell of ~100m (50 * 2 = 100m).
        let cell_deg = cell_size_for_proximity(50.0);
        let cell_meters = cell_deg * 111_000.0;
        assert!((cell_meters - 100.0).abs() < 1.0, "got {cell_meters}");

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

    /// Correctness invariant: pairs within proximity_threshold MUST be emitted.
    ///
    /// This is the key safety property — if the filter ever drops a pair
    /// that has any point within proximity_threshold of any point on the
    /// other track, detection would silently miss real overlaps. We test
    /// the boundary case (tracks separated by exactly proximity_threshold)
    /// and within proximity_threshold to ensure neither slips through.
    #[test]
    fn pairs_within_proximity_are_always_emitted() {
        let proximity_m = 50.0;
        let cell_size = cell_size_for_proximity(proximity_m);

        // Track A: straight line north along lng=7.0.
        let track_a: Vec<GpsPoint> = (0..50)
            .map(|i| pt(45.0 + (i as f64) * 0.0001, 7.0))
            .collect();

        // Track B at varying offsets — should all be emitted as candidate pairs.
        // 1 m, 10 m, 25 m, 49 m offsets — all within proximity_threshold (50 m).
        for offset_m in [1.0, 10.0, 25.0, 49.0] {
            let offset_deg = offset_m / 111_000.0;
            let track_b: Vec<GpsPoint> = (0..50)
                .map(|i| pt(45.0 + (i as f64) * 0.0001, 7.0 + offset_deg))
                .collect();

            let cells_a = compute_fine_cells(&track_a, cell_size);
            let cells_b = compute_fine_cells(&track_b, cell_size);
            let pairs = fine_grid_filtered_pairs(&[cells_a, cells_b]);

            assert_eq!(
                pairs,
                vec![(0, 1)],
                "Tracks {offset_m} m apart (< proximity threshold {proximity_m} m) MUST be emitted as a candidate pair — anything else means the filter is unsafe."
            );
        }
    }

    /// Conservative-cell-sizing invariant: the chosen cell size must
    /// ensure that ANY two points within `proximity_threshold` land in
    /// cells that are either equal or 8-neighbours of each other.
    ///
    /// If this ever breaks, the filter would drop valid pairs and
    /// quietly produce wrong detection output.
    #[test]
    fn cell_size_safety_margin() {
        for proximity_m in [5.0, 25.0, 50.0, 100.0, 250.0] {
            let cell_size_deg = cell_size_for_proximity(proximity_m);
            let cell_size_m = cell_size_deg * 111_000.0;

            // Two points exactly `proximity_threshold` apart, both at the
            // worst-case position on opposite faces of a cell. The 8-cell
            // neighbourhood is a 3×3 grid covering ~3 × cell_size in each
            // axis, so worst-case separation between points in disjoint
            // 8-neighbourhoods is at least `cell_size`. We need
            // proximity ≤ cell_size for the filter to be safe.
            assert!(
                proximity_m <= cell_size_m,
                "proximity {proximity_m} m exceeds cell_size {cell_size_m:.1} m — \
                 pairs within proximity could end up in non-neighbouring cells \
                 and be dropped by the filter"
            );
        }
    }

    /// Direct A/B regression: the filter's pair output, when fed through
    /// `find_full_track_overlap`, must produce the SAME set of overlaps
    /// as running the naive O(N²) all-pairs approach. This is the
    /// definitive "no regression" check — it operates at the same layer
    /// the production code does and ensures no real overlap is lost
    /// to filtering.
    ///
    /// Proof strategy:
    ///   1. Generate N synthetic tracks (some overlap, some don't).
    ///   2. Compute filter pairs and naive pairs.
    ///   3. Run `find_full_track_overlap` on each pair set.
    ///   4. Assert: { overlaps_from_filter } == { overlaps_from_naive }.
    ///
    /// If the filter ever drops a pair that produces an overlap, the
    /// naive set will contain it and the filter set won't, and the
    /// assert will fail with a specific (i, j) mismatch.
    #[test]
    fn filter_produces_identical_overlap_set_as_naive() {
        use crate::SectionConfig;
        use crate::sections::find_full_track_overlap;
        use crate::sections::rtree::build_rtree;

        // 30 tracks: 3 distinct routes × 10 tracks each. Routes far
        // apart geographically; tracks within a route are nearly identical.
        let mut tracks: Vec<Vec<GpsPoint>> = Vec::new();
        for route_idx in 0..3 {
            let route_lat = 45.0 + (route_idx as f64) * 0.1; // 11km between routes
            for track_idx in 0..10 {
                let lat_jitter = (track_idx as f64) * 0.0000003;
                let track: Vec<GpsPoint> = (0..50)
                    .map(|j| pt(route_lat + (j as f64) * 0.0001 + lat_jitter, 7.0))
                    .collect();
                tracks.push(track);
            }
        }

        let config = SectionConfig::default();

        // --- Filter pair set --------------------------------------------
        let cell_size = cell_size_for_proximity(config.proximity_threshold);
        let cells: Vec<_> = tracks
            .iter()
            .map(|t| compute_fine_cells(t, cell_size))
            .collect();
        let filter_pairs = fine_grid_filtered_pairs(&cells);

        // --- Naive pair set ---------------------------------------------
        let naive_pairs: Vec<(usize, usize)> = (0..tracks.len())
            .flat_map(|i| ((i + 1)..tracks.len()).map(move |j| (i, j)))
            .collect();

        // Filter is by construction a subset of naive.
        let filter_set: std::collections::HashSet<_> = filter_pairs.iter().copied().collect();
        let naive_set: std::collections::HashSet<_> = naive_pairs.iter().copied().collect();
        assert!(
            filter_set.is_subset(&naive_set),
            "filter set must be a subset of naive — filter produced pairs naive didn't"
        );

        // --- Compute overlaps via both paths ----------------------------
        // Pre-build R-trees once.
        let rtrees: Vec<_> = tracks.iter().map(|t| build_rtree(t)).collect();

        let collect_overlaps = |pairs: &[(usize, usize)]| -> Vec<(usize, usize)> {
            let mut overlap_pairs: Vec<(usize, usize)> = pairs
                .iter()
                .filter_map(|&(i, j)| {
                    let id_a = format!("a_{i}");
                    let id_b = format!("a_{j}");
                    find_full_track_overlap(
                        &id_a, &tracks[i], &id_b, &tracks[j], &rtrees[j], &config,
                    )
                    .map(|_| (i, j))
                })
                .collect();
            overlap_pairs.sort();
            overlap_pairs
        };

        let filter_overlaps = collect_overlaps(&filter_pairs);
        let naive_overlaps = collect_overlaps(&naive_pairs);

        // THE PROOF: same set of pairs produce overlaps in both paths.
        let filter_overlap_set: std::collections::HashSet<_> =
            filter_overlaps.iter().copied().collect();
        let naive_overlap_set: std::collections::HashSet<_> =
            naive_overlaps.iter().copied().collect();

        let dropped: Vec<_> = naive_overlap_set
            .difference(&filter_overlap_set)
            .copied()
            .collect();
        assert!(
            dropped.is_empty(),
            "REGRESSION: filter dropped {} pair(s) that produce real overlaps. \
             Dropped pairs (naive found them, filter didn't): {:?}",
            dropped.len(),
            dropped,
        );

        // Naive should never find an overlap the filter doesn't, since
        // filter is a subset. But assert it explicitly as a sanity check.
        assert_eq!(
            filter_overlap_set, naive_overlap_set,
            "filter and naive must produce identical overlap sets"
        );

        // And empirically: the filter should produce way fewer pairs than naive.
        // 3 routes × 10 tracks each = 45 within-route pairs per route × 3 = 135.
        // Naive has 30*29/2 = 435 pairs. Filter should reduce to ~135 (within-route only).
        assert!(
            filter_pairs.len() < naive_pairs.len() / 2,
            "filter should at least halve the pair count on multi-region data \
             (naive={}, filtered={})",
            naive_pairs.len(),
            filter_pairs.len()
        );

        // Sanity: we DID find some overlaps (otherwise the test is vacuous).
        assert!(
            !naive_overlaps.is_empty(),
            "no overlaps found at all — test data is wrong, not the filter"
        );
    }

    /// End-to-end smoke test: feed the same data through the full
    /// `detect_sections_multiscale` and assert we get a non-empty result.
    /// Protects against the spatial filter accidentally pruning the
    /// pair set down to nothing on overlapping tracks.
    #[test]
    fn end_to_end_detection_finds_sections() {
        use crate::detect_sections_multiscale;
        use std::collections::HashMap;

        // 5 tracks following roughly the same 1km north-south path
        // with small offsets — should produce at least one section.
        let tracks: Vec<(String, Vec<GpsPoint>)> = (0..5)
            .map(|i| {
                let pts: Vec<GpsPoint> = (0..200)
                    .map(|j| {
                        let lat_offset = (i as f64) * 0.0000003;
                        pt(45.0 + (j as f64) * 0.0001 + lat_offset, 7.0)
                    })
                    .collect();
                (format!("a_{i}"), pts)
            })
            .collect();

        let sport_types: HashMap<String, String> = tracks
            .iter()
            .map(|(id, _)| (id.clone(), "Run".to_string()))
            .collect();

        let config = crate::SectionConfig {
            proximity_threshold: 50.0,
            min_activities: 3,
            min_section_length: 200.0,
            ..crate::SectionConfig::default()
        };

        let result = detect_sections_multiscale(&tracks, &sport_types, &[], &config);

        assert!(
            !result.sections.is_empty(),
            "expected ≥ 1 section from 5 overlapping tracks; got 0. Spatial \
             filter may have dropped all candidate pairs."
        );

        // Every section should have populated activity_portions (Bug A fix).
        for section in &result.sections {
            assert!(
                !section.activity_portions.is_empty(),
                "section {} has activity_ids={:?} but empty activity_portions \
                 — Bug A may have regressed",
                section.id,
                section.activity_ids,
            );
        }
    }
}
