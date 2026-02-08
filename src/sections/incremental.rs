//! Incremental section detection.
//!
//! Instead of reprocessing ALL activities from scratch (O(N²)),
//! matches new activities against existing sections (O(K × S)) where:
//! - K = number of new activities
//! - S = number of existing sections
//!
//! Unmatched activities are then processed with full pairwise detection
//! among themselves only (O(K² × P log P)), which is dramatically
//! faster when K << N.
//!
//! Fallback to full detection when >30% of activities are new.

use super::optimized::find_sections_in_route;
use super::{
    FrequentSection, SectionConfig, SectionPortion,
    compute_consensus_polyline, compute_initial_stability, extract_all_activity_traces,
};
use super::progress::{DetectionPhase, DetectionProgressCallback};
use crate::matching::calculate_route_distance;
use crate::{GpsPoint, RouteGroup};
use log::info;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

/// Result of incremental section detection.
pub struct IncrementalResult {
    /// Updated existing sections (with new activities matched in)
    pub updated_sections: Vec<FrequentSection>,
    /// New sections discovered from unmatched activities
    pub new_sections: Vec<FrequentSection>,
    /// Activity IDs that were successfully matched to existing sections
    pub matched_activity_ids: Vec<String>,
    /// Activity IDs that didn't match any existing section
    pub unmatched_activity_ids: Vec<String>,
}

/// Incrementally detect sections by matching new activities against existing sections.
///
/// # Algorithm
/// 1. For each new activity, use `find_sections_in_route` to check overlap with existing sections
/// 2. Activities that match are added to those sections (portions updated, consensus re-averaged)
/// 3. Unmatched activities are collected for pairwise detection among themselves
/// 4. Full detection runs only on the unmatched pool, producing new sections
///
/// # When to use
/// - Adding a small batch of new activities (K) to a large existing dataset (N)
/// - K/N < 0.3 (otherwise full detection is more appropriate)
pub fn detect_sections_incremental(
    new_tracks: &[(String, Vec<GpsPoint>)],
    existing_sections: &[FrequentSection],
    all_tracks: &[(String, Vec<GpsPoint>)], // All tracks (existing + new) for consensus recalc
    sport_types: &HashMap<String, String>,
    groups: &[RouteGroup],
    config: &SectionConfig,
    progress: Arc<dyn DetectionProgressCallback>,
) -> IncrementalResult {
    info!(
        "[Incremental] Processing {} new activities against {} existing sections",
        new_tracks.len(),
        existing_sections.len()
    );

    if existing_sections.is_empty() || new_tracks.is_empty() {
        return IncrementalResult {
            updated_sections: existing_sections.to_vec(),
            new_sections: vec![],
            matched_activity_ids: vec![],
            unmatched_activity_ids: new_tracks.iter().map(|(id, _)| id.clone()).collect(),
        };
    }

    // Phase 1: Match new activities against existing sections
    progress.on_phase(DetectionPhase::FindingOverlaps, new_tracks.len() as u32);

    let mut matched_activity_ids: Vec<String> = Vec::new();
    let mut unmatched_activity_ids: Vec<String> = Vec::new();
    // section_id -> list of (activity_id, SectionMatch) for each new match
    let mut section_matches: HashMap<String, Vec<(String, super::optimized::SectionMatch)>> =
        HashMap::new();

    for (activity_id, track) in new_tracks {
        let matches = find_sections_in_route(track, existing_sections, config);
        progress.on_progress();

        if matches.is_empty() {
            unmatched_activity_ids.push(activity_id.clone());
        } else {
            matched_activity_ids.push(activity_id.clone());
            for m in matches {
                section_matches
                    .entry(m.section_id.clone())
                    .or_default()
                    .push((activity_id.clone(), m));
            }
        }
    }

    info!(
        "[Incremental] Matched {} activities, {} unmatched",
        matched_activity_ids.len(),
        unmatched_activity_ids.len()
    );

    // Phase 2: Update existing sections with newly matched activities
    progress.on_phase(
        DetectionPhase::Postprocessing,
        existing_sections.len() as u32,
    );

    // Build track_map for all tracks (borrowed)
    let track_map: HashMap<&str, &[GpsPoint]> = all_tracks
        .iter()
        .map(|(id, pts)| (id.as_str(), pts.as_slice()))
        .collect();

    let mut updated_sections: Vec<FrequentSection> = Vec::with_capacity(existing_sections.len());

    for section in existing_sections {
        progress.on_progress();

        if let Some(new_matches) = section_matches.get(&section.id) {
            // This section has new activities — update it
            let mut updated = section.clone();

            for (activity_id, section_match) in new_matches {
                // Add activity to section if not already there
                if !updated.activity_ids.contains(activity_id) {
                    updated.activity_ids.push(activity_id.clone());
                    updated.visit_count += 1;

                    // Compute portion for this new activity
                    if let Some(track) = track_map.get(activity_id.as_str()) {
                        let direction = if section_match.same_direction {
                            "same"
                        } else {
                            "reverse"
                        };
                        let start = section_match.start_index as u32;
                        let end = section_match.end_index as u32;
                        let portion_slice =
                            &track[start as usize..(end as usize).min(track.len())];
                        let distance = calculate_route_distance(portion_slice);

                        updated.activity_portions.push(SectionPortion {
                            activity_id: activity_id.clone(),
                            start_index: start,
                            end_index: end,
                            distance_meters: distance,
                            direction: direction.to_string(),
                        });
                    }
                }
            }

            // Re-compute consensus polyline with all activities (including new ones)
            let all_activity_ids: Vec<String> = updated.activity_ids.clone();
            let traces =
                extract_all_activity_traces(&all_activity_ids, &updated.polyline, &track_map);
            let all_traces: Vec<Vec<GpsPoint>> = traces.values().cloned().collect();

            if !all_traces.is_empty() {
                // Use existing polyline as reference for consensus
                let consensus = compute_consensus_polyline(
                    &updated.polyline,
                    &all_traces,
                    config.proximity_threshold,
                );
                if consensus.polyline.len() >= 2 {
                    updated.polyline = consensus.polyline;
                    updated.distance_meters = calculate_route_distance(&updated.polyline);
                    updated.confidence = consensus.confidence;
                    updated.observation_count = consensus.observation_count;
                    updated.average_spread = consensus.average_spread;
                    updated.point_density = consensus.point_density;
                    updated.stability = compute_initial_stability(
                        consensus.observation_count,
                        consensus.average_spread,
                        config.proximity_threshold,
                    );
                }
            }

            updated.version += 1;
            updated_sections.push(updated);
        } else {
            // No new matches — keep section as-is
            updated_sections.push(section.clone());
        }
    }

    // Phase 3: Run full detection on unmatched activities only
    let new_sections = if unmatched_activity_ids.len() >= config.min_activities as usize {
        info!(
            "[Incremental] Running pairwise detection on {} unmatched activities",
            unmatched_activity_ids.len()
        );

        let unmatched_set: HashSet<&str> = unmatched_activity_ids
            .iter()
            .map(|s| s.as_str())
            .collect();
        let unmatched_tracks: Vec<(String, Vec<GpsPoint>)> = new_tracks
            .iter()
            .filter(|(id, _)| unmatched_set.contains(id.as_str()))
            .cloned()
            .collect();

        // Run full detection on just the unmatched pool
        let result = super::detect_sections_multiscale_with_progress(
            &unmatched_tracks,
            sport_types,
            groups,
            config,
            progress.clone(),
        );

        result.sections
    } else {
        vec![]
    };

    info!(
        "[Incremental] Result: {} updated sections, {} new sections",
        updated_sections.len(),
        new_sections.len()
    );

    IncrementalResult {
        updated_sections,
        new_sections,
        matched_activity_ids,
        unmatched_activity_ids,
    }
}
