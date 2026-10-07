//! Wire format for section detection.
//!
//! Defines the `FullTrackOverlap` and `OverlapCluster` types that flow
//! between the unified cut and the consensus layer (`process_cluster` to
//! `select_medoid` and `compute_consensus_polyline`).

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
}

/// A cluster of overlaps representing the same physical section
#[derive(Debug, Clone)]
pub struct OverlapCluster {
    /// All overlaps in this cluster
    pub overlaps: Vec<FullTrackOverlap>,
    /// Unique activity IDs in this cluster
    pub activity_ids: HashSet<String>,
}
