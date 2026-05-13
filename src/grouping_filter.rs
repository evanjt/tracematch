//! Endpoint-keyed spatial pre-filter for route grouping pair pruning.
//!
//! Route grouping (Union-Find of "same-route" activities) compares every
//! pair of route signatures within a dataset. For N routes that's
//! N(N-1)/2 pairs — at N=1000 that's nearly 500k comparisons, each
//! costing ~25ms of AMD work (`compare_routes`). The existing
//! `RTree<RouteBounds>` pre-filter in `grouping.rs` only prunes pairs
//! whose bounding boxes are further apart than `SPATIAL_TOLERANCE`
//! (~1km), but the strict grouping gate in `should_group_routes`
//! requires start/end points within `endpoint_threshold` (250m
//! default). Two routes whose bounds overlap (everyone running in the
//! same city) pay the full AMD cost only to be rejected by the
//! endpoint check.
//!
//! This filter operates at the same boundary as
//! [`crate::sections::spatial_filter`], but keyed on each route's
//! start AND end points rather than every-track-point. Pairs are emitted
//! only when at least one endpoint of route i lands within a
//! neighbour-cell of an endpoint of route j. That correctly captures
//! both the same-direction case (start_a ≈ start_b, end_a ≈ end_b)
//! and the reverse case (start_a ≈ end_b, end_a ≈ start_b), because
//! all four endpoints register in the same grid.
//!
//! The filter is conservative by construction: any pair the existing
//! `should_group_routes` gate would accept must have its endpoints
//! within `endpoint_threshold`, which guarantees the filter emits it
//! (cell size is sized at 4× threshold with 9-cell neighbour lookup).

use crate::GpsPoint;
use std::collections::{HashMap, HashSet};

/// A geographic grid cell at endpoint-tolerance scale.
///
/// Coordinates stored as integer indices for hash-friendly equality.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EndpointCell {
    lat_idx: i32,
    lng_idx: i32,
}

impl EndpointCell {
    /// Compute the cell containing a GPS point at the given cell size.
    pub fn from_point(lat: f64, lng: f64, cell_size_deg: f64) -> Self {
        Self {
            lat_idx: (lat / cell_size_deg).floor() as i32,
            lng_idx: (lng / cell_size_deg).floor() as i32,
        }
    }

    /// This cell plus the 8 adjacent cells (3×3 neighbourhood).
    pub fn with_neighbors(&self) -> [EndpointCell; 9] {
        let mut out = [Self {
            lat_idx: 0,
            lng_idx: 0,
        }; 9];
        let mut i = 0;
        for dlat in -1..=1 {
            for dlng in -1..=1 {
                out[i] = EndpointCell {
                    lat_idx: self.lat_idx + dlat,
                    lng_idx: self.lng_idx + dlng,
                };
                i += 1;
            }
        }
        out
    }
}

/// The start and end cells of a single route, used to register the
/// route in the candidate-pair grid.
///
/// Loop routes (start ≈ end) collapse to a single cell — the
/// filter handles them naturally because both `start` and `end` end up
/// in the same cell.
#[derive(Debug, Clone, Copy)]
pub struct RouteEndpointCells {
    pub start: EndpointCell,
    pub end: EndpointCell,
}

impl RouteEndpointCells {
    /// Compute endpoint cells for a route given its start/end points.
    pub fn new(start: &GpsPoint, end: &GpsPoint, cell_size_deg: f64) -> Self {
        Self {
            start: EndpointCell::from_point(start.latitude, start.longitude, cell_size_deg),
            end: EndpointCell::from_point(end.latitude, end.longitude, cell_size_deg),
        }
    }
}

/// Derive a sensible cell size from the endpoint threshold.
///
/// `4 × endpoint_threshold` gives a 6× safety margin (effective filter
/// radius is ~1.5 × cell_size with the 9-cell neighbour lookup),
/// matching the safety margin used in
/// [`crate::sections::spatial_filter::cell_size_for_proximity`].
/// Floor at 50m so very tight thresholds don't produce useless cells.
pub fn cell_size_for_endpoint_threshold(endpoint_threshold_meters: f64) -> f64 {
    let cell_size_meters = (endpoint_threshold_meters * 4.0).max(50.0);
    cell_size_meters / 111_000.0 // ~m per degree of latitude
}

/// Generate the candidate pair list using the endpoint grid filter.
///
/// Returns pairs as `(i, j)` with `i < j`. Pairs are unique. The set
/// is a strict subset of all N(N-1)/2 pairs — specifically, the subset
/// where at least one endpoint (start or end) of route `i` shares a
/// cell-or-neighbour with at least one endpoint of route `j`.
///
/// Downstream code (`compare_routes`, `should_group_routes`) still
/// applies its own AMD + endpoint + middle-point checks on the result,
/// so the filter doesn't need to be exact — it only needs to never DROP
/// a pair that the strict grouping gate would accept.
pub fn endpoint_grid_filtered_pairs(endpoints: &[RouteEndpointCells]) -> Vec<(usize, usize)> {
    // Inverted index: cell -> list of route indices that have an endpoint there.
    // A route registers in 1-2 distinct cells (1 if it's a loop with start ≈ end).
    let mut grid_index: HashMap<EndpointCell, Vec<usize>> = HashMap::new();
    for (idx, ep) in endpoints.iter().enumerate() {
        grid_index.entry(ep.start).or_default().push(idx);
        if ep.end != ep.start {
            grid_index.entry(ep.end).or_default().push(idx);
        }
    }

    // For each route, look up neighbour cells of both its endpoints and
    // collect candidate partner indices. Dedupe pairs across the 18
    // possible cell lookups via HashSet.
    let mut candidate_pairs: HashSet<(usize, usize)> = HashSet::new();
    for (idx, ep) in endpoints.iter().enumerate() {
        for anchor in [ep.start, ep.end] {
            for neighbour in anchor.with_neighbors() {
                if let Some(indices) = grid_index.get(&neighbour) {
                    for &other_idx in indices {
                        if other_idx == idx {
                            continue;
                        }
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
        // For loops (start == end), the second iteration of the
        // for-loop is a no-op duplicate. The HashSet handles it.
    }

    // Sort for deterministic output: the HashSet iteration order is
    // randomised per process. Stable pair order makes downstream
    // grouping output reproducible across runs.
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
    fn cell_size_scales_with_threshold() {
        // 250m threshold should give a cell of ~1000m (250 * 4).
        let cell_deg = cell_size_for_endpoint_threshold(250.0);
        let cell_m = cell_deg * 111_000.0;
        assert!((cell_m - 1000.0).abs() < 1.0, "got {cell_m}");

        // Tiny thresholds floor at 50m.
        let cell_deg_tiny = cell_size_for_endpoint_threshold(5.0);
        let cell_m_tiny = cell_deg_tiny * 111_000.0;
        assert!(cell_m_tiny >= 50.0);
    }

    #[test]
    fn nearby_routes_share_cells() {
        let cell_size = cell_size_for_endpoint_threshold(250.0);
        // Two routes starting and ending within 100m of each other.
        let r0 = RouteEndpointCells::new(&pt(46.0, 7.0), &pt(46.01, 7.01), cell_size);
        let r1 = RouteEndpointCells::new(&pt(46.0001, 7.0001), &pt(46.0101, 7.0101), cell_size);
        let pairs = endpoint_grid_filtered_pairs(&[r0, r1]);
        assert_eq!(pairs, vec![(0, 1)]);
    }

    #[test]
    fn far_routes_are_pruned() {
        let cell_size = cell_size_for_endpoint_threshold(250.0);
        // Two routes 10km apart at both endpoints.
        let r0 = RouteEndpointCells::new(&pt(46.0, 7.0), &pt(46.01, 7.01), cell_size);
        let r1 = RouteEndpointCells::new(&pt(46.1, 7.0), &pt(46.11, 7.01), cell_size);
        let pairs = endpoint_grid_filtered_pairs(&[r0, r1]);
        assert!(pairs.is_empty(), "expected far routes to be pruned");
    }

    #[test]
    fn reverse_direction_routes_match() {
        // Route A: starts at P1, ends at P2.
        // Route B: starts at P2, ends at P1 (same physical route, opposite direction).
        // Filter must emit (0,1) so the strict grouping gate can detect the reverse match.
        let cell_size = cell_size_for_endpoint_threshold(250.0);
        let p1 = pt(46.0, 7.0);
        let p2 = pt(46.01, 7.01);
        let r0 = RouteEndpointCells::new(&p1, &p2, cell_size);
        let r1 = RouteEndpointCells::new(&p2, &p1, cell_size);
        let pairs = endpoint_grid_filtered_pairs(&[r0, r1]);
        assert_eq!(pairs, vec![(0, 1)], "reverse-direction pair must be emitted");
    }

    #[test]
    fn loops_handled() {
        // Loop route: start ≈ end. RouteEndpointCells should collapse to a single cell;
        // two loops starting at the same place should pair.
        let cell_size = cell_size_for_endpoint_threshold(250.0);
        let p = pt(46.0, 7.0);
        let r0 = RouteEndpointCells::new(&p, &p, cell_size);
        let r1 = RouteEndpointCells::new(&pt(46.0001, 7.0001), &pt(46.0001, 7.0001), cell_size);
        assert_eq!(r0.start, r0.end);
        let pairs = endpoint_grid_filtered_pairs(&[r0, r1]);
        assert_eq!(pairs, vec![(0, 1)]);
    }

    #[test]
    fn no_self_pairs() {
        let cell_size = cell_size_for_endpoint_threshold(250.0);
        let r0 = RouteEndpointCells::new(&pt(46.0, 7.0), &pt(46.01, 7.01), cell_size);
        let pairs = endpoint_grid_filtered_pairs(&[r0]);
        assert!(pairs.is_empty());
    }

    #[test]
    fn pairs_are_ordered_i_less_than_j() {
        let cell_size = cell_size_for_endpoint_threshold(250.0);
        let p = pt(46.0, 7.0);
        let routes: Vec<_> = (0..4)
            .map(|_| RouteEndpointCells::new(&p, &p, cell_size))
            .collect();
        let pairs = endpoint_grid_filtered_pairs(&routes);
        for (i, j) in &pairs {
            assert!(i < j, "pair ({i}, {j}) is not ordered");
        }
    }

    /// Safety invariant: any pair whose endpoints are within
    /// `endpoint_threshold` of each other MUST be emitted. If the
    /// filter ever drops a pair the strict grouping gate would accept,
    /// route grouping would silently lose real groups.
    #[test]
    fn pairs_within_threshold_are_always_emitted() {
        let threshold_m = 250.0;
        let cell_size = cell_size_for_endpoint_threshold(threshold_m);

        // Anchor route at (46.0, 7.0) → (46.01, 7.01).
        let anchor = RouteEndpointCells::new(&pt(46.0, 7.0), &pt(46.01, 7.01), cell_size);

        // Partner routes with starts offset by various amounts under threshold.
        // All should be emitted.
        for offset_m in [1.0, 50.0, 100.0, 200.0, 249.0] {
            let offset_deg = offset_m / 111_000.0;
            let partner = RouteEndpointCells::new(
                &pt(46.0 + offset_deg, 7.0),
                &pt(46.01 + offset_deg, 7.01),
                cell_size,
            );
            let pairs = endpoint_grid_filtered_pairs(&[anchor, partner]);
            assert_eq!(
                pairs,
                vec![(0, 1)],
                "routes with starts {offset_m} m apart (< threshold {threshold_m} m) MUST be emitted"
            );
        }
    }

    /// Cell size safety: the chosen cell size with 9-neighbour lookup
    /// must cover any separation up to `endpoint_threshold`. If this
    /// breaks, the filter would silently drop valid pairs.
    #[test]
    fn cell_size_safety_margin() {
        for threshold_m in [50.0, 100.0, 250.0, 500.0, 1000.0] {
            let cell_size_deg = cell_size_for_endpoint_threshold(threshold_m);
            let cell_size_m = cell_size_deg * 111_000.0;
            assert!(
                threshold_m <= cell_size_m,
                "threshold {threshold_m} m exceeds cell_size {cell_size_m:.1} m — \
                 pairs within threshold could fall in non-neighbouring cells \
                 and be dropped"
            );
        }
    }

    /// Definitive A/B regression: feed N routes through both the
    /// filter and the naive O(N²) pair list. For each pair set, run
    /// the actual `should_group_routes` gate. The set of pairs that
    /// PASS the gate via the filter must equal the set via naive.
    ///
    /// If the filter ever drops a pair that the strict gate would
    /// accept, this test fails with the specific (i, j) that was lost.
    #[test]
    fn filter_produces_identical_groups_as_naive() {
        use crate::matching::compare_routes;
        use crate::{MatchConfig, RouteSignature};
        use crate::grouping::should_group_routes;

        // Build 30 synthetic signatures: 3 distinct "real" routes × 10
        // similar variants (slight start/end jitter within threshold)
        // + the routes are 5km apart so cross-route pairs should not group.
        let mut sigs: Vec<RouteSignature> = Vec::new();
        let mc = MatchConfig::default();
        for route_idx in 0..3 {
            let base_lat = 46.0 + (route_idx as f64) * 0.05; // ~5.5km apart
            for variant in 0..10 {
                let jitter = (variant as f64) * 0.0000005; // sub-metre jitter
                let pts: Vec<GpsPoint> = (0..200)
                    .map(|j| pt(base_lat + (j as f64) * 0.00005 + jitter, 7.0))
                    .collect();
                let id = format!("r{route_idx}_v{variant}");
                let sig = RouteSignature::from_points(&id, &pts, &mc)
                    .expect("synthetic track should produce a signature");
                sigs.push(sig);
            }
        }

        let config = MatchConfig::default();
        let cell_size = cell_size_for_endpoint_threshold(config.endpoint_threshold);

        // Filter pair set
        let endpoints: Vec<RouteEndpointCells> = sigs
            .iter()
            .map(|s| RouteEndpointCells::new(&s.start_point, &s.end_point, cell_size))
            .collect();
        let filter_pairs = endpoint_grid_filtered_pairs(&endpoints);

        // Naive pair set
        let naive_pairs: Vec<(usize, usize)> = (0..sigs.len())
            .flat_map(|i| ((i + 1)..sigs.len()).map(move |j| (i, j)))
            .collect();

        // Filter must be a strict subset.
        let filter_set: HashSet<_> = filter_pairs.iter().copied().collect();
        let naive_set: HashSet<_> = naive_pairs.iter().copied().collect();
        assert!(
            filter_set.is_subset(&naive_set),
            "filter is not a subset of naive — broken invariant"
        );

        // Run the strict grouping gate on each pair set, collect the
        // pairs that pass.
        let gate_pass = |pairs: &[(usize, usize)]| -> Vec<(usize, usize)> {
            let mut out: Vec<(usize, usize)> = pairs
                .iter()
                .filter_map(|&(i, j)| {
                    let mr = compare_routes(&sigs[i], &sigs[j], &config)?;
                    if should_group_routes(&sigs[i], &sigs[j], &mr, &config) {
                        Some((i, j))
                    } else {
                        None
                    }
                })
                .collect();
            out.sort();
            out
        };

        let filter_grouped = gate_pass(&filter_pairs);
        let naive_grouped = gate_pass(&naive_pairs);

        let filter_grouped_set: HashSet<_> = filter_grouped.iter().copied().collect();
        let naive_grouped_set: HashSet<_> = naive_grouped.iter().copied().collect();

        let dropped: Vec<_> = naive_grouped_set
            .difference(&filter_grouped_set)
            .copied()
            .collect();
        assert!(
            dropped.is_empty(),
            "REGRESSION: filter dropped {} pair(s) that the grouping gate would accept. \
             Dropped (naive grouped them, filter didn't): {:?}",
            dropped.len(),
            dropped,
        );

        assert_eq!(
            filter_grouped_set, naive_grouped_set,
            "filter and naive must produce identical grouped-pair sets"
        );

        // Empirically the filter should prune most cross-route pairs.
        // 3 routes × 10 variants = 30 routes. Naive = 30*29/2 = 435 pairs.
        // Within-route pairs = 3 × C(10,2) = 135. Filter should keep
        // those 135 and drop the ~300 cross-route pairs.
        assert!(
            filter_pairs.len() < naive_pairs.len() / 2,
            "filter should at least halve the pair count on multi-region data \
             (naive={}, filtered={})",
            naive_pairs.len(),
            filter_pairs.len(),
        );

        // Sanity: actual groupings must be non-empty.
        assert!(
            !naive_grouped.is_empty(),
            "test produced zero grouped pairs — test data is wrong, not the filter"
        );
    }

}
