//! Wire format for section detection.
//!
//! Defines the `FullTrackOverlap` and `OverlapCluster` types that flow
//! between the cluster-detection layer (`density_grid`) and the
//! consensus layer (`process_cluster` → `select_medoid`,
//! `compute_consensus_polyline`). The legacy pairwise overlap
//! detection (`find_full_track_overlap`, `cluster_overlaps`,
//! `has_any_overlap`) lived here previously; it was replaced by the
//! density-grid pass and removed.

use crate::GpsPoint;
use std::collections::HashSet;

/// A detected overlap between two full GPS tracks.
/// Uses index ranges into original tracks instead of copying GPS points,
/// reducing memory from ~48KB to ~100 bytes per overlap (~480x reduction).
#[derive(Debug, Clone)]
pub struct FullTrackOverlap {
    pub activity_a: String,
    pub activity_b: String,
    /// Index range into track A's original points (start..end)
    pub range_a: (usize, usize),
    /// Index range into track B's original points (start..end)
    pub range_b: (usize, usize),
    /// Center point for clustering
    pub center: GpsPoint,
    /// Pre-computed overlap length in meters
    pub overlap_length: f64,
}

/// A cluster of overlaps representing the same physical section
#[derive(Debug, Clone)]
pub struct OverlapCluster {
    /// All overlaps in this cluster
    pub overlaps: Vec<FullTrackOverlap>,
    /// Unique activity IDs in this cluster
    pub activity_ids: HashSet<String>,
}
