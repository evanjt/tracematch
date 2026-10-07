//! Recalculate a stored section's polyline from its traces.

use super::{FrequentSection, SectionConfig};
use crate::GpsPoint;

/// Recalculate a section's polyline based on its activity traces.
///
/// This is useful when a section's polyline has drifted or is biased
/// toward certain activities. It recomputes the consensus polyline
/// from all stored traces.
///
/// # Arguments
/// * `section` - The section to adjust
/// * `config` - Section configuration
///
/// # Returns
/// Updated section with recalculated polyline, or original if no traces available
pub fn recalculate_section_polyline(
    section: &FrequentSection,
    config: &SectionConfig,
) -> FrequentSection {
    if section.activity_traces.is_empty() || section.is_user_defined {
        return section.clone();
    }

    // Ordered by activity id: `HashMap` iteration order varies per process, and
    // both the reference and the accumulation order steer the consensus.
    let mut ordered: Vec<(&String, &Vec<GpsPoint>)> = section.activity_traces.iter().collect();
    ordered.sort_by(|a, b| a.0.cmp(b.0));

    // The representative is the medoid, so it anchors the consensus closest to
    // the ground the section actually covers.
    let reference = section
        .activity_traces
        .get(&section.representative_activity_id)
        .unwrap_or_else(|| ordered[0].1)
        .clone();

    let traces: Vec<Vec<GpsPoint>> = ordered.into_iter().map(|(_, t)| t.clone()).collect();

    let consensus =
        super::compute_consensus_polyline(&reference, &traces, config.proximity_threshold);

    let new_distance = crate::matching::calculate_route_distance(&consensus.polyline);

    // Recompute stability of representative against new consensus
    let stability = section
        .activity_traces
        .get(&section.representative_activity_id)
        .map(|trace| {
            super::medoid::compute_stability(trace, &consensus.polyline, config.proximity_threshold)
        })
        .unwrap_or(section.stability);

    FrequentSection {
        polyline: consensus.polyline,
        distance_meters: new_distance,
        average_spread: consensus.average_spread,
        point_density: consensus.point_density,
        confidence: consensus.confidence,
        observation_count: consensus.observation_count,
        version: section.version + 1,
        stability,
        // Averaging leaves the line a slice of nothing, so the range dies here.
        representative_range: None,
        ..section.clone()
    }
}
