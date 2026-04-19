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
use super::progress::{DetectionPhase, DetectionProgressCallback};
use super::{
    FrequentSection, SectionConfig, SectionPortion, compute_consensus_polyline,
    extract_all_activity_traces,
};
use crate::matching::calculate_route_distance;
use crate::{Direction, GpsPoint, RouteGroup};
use log::info;
#[cfg(feature = "parallel")]
use rayon::prelude::*;
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

    // section_id -> list of (activity_id, SectionMatch) for each new match
    let mut section_matches: HashMap<String, Vec<(String, super::optimized::SectionMatch)>> =
        HashMap::new();

    // Each activity's matching is independent — parallelize
    #[cfg(feature = "parallel")]
    let per_activity_results: Vec<(String, Vec<super::optimized::SectionMatch>)> = new_tracks
        .par_iter()
        .map(|(activity_id, track)| {
            let matches = find_sections_in_route(track, existing_sections, config);
            (activity_id.clone(), matches)
        })
        .collect();

    #[cfg(not(feature = "parallel"))]
    let per_activity_results: Vec<(String, Vec<super::optimized::SectionMatch>)> = new_tracks
        .iter()
        .map(|(activity_id, track)| {
            let matches = find_sections_in_route(track, existing_sections, config);
            (activity_id.clone(), matches)
        })
        .collect();

    // Reduce results sequentially (progress reporting + HashMap building)
    let mut matched_activity_ids: Vec<String> = Vec::new();
    let mut unmatched_activity_ids: Vec<String> = Vec::new();

    for (activity_id, matches) in per_activity_results {
        progress.on_progress();

        if matches.is_empty() {
            unmatched_activity_ids.push(activity_id);
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
                            Direction::Same
                        } else {
                            Direction::Reverse
                        };
                        let start = section_match.start_index as u32;
                        let end = section_match.end_index as u32;
                        let portion_slice = &track[start as usize..(end as usize).min(track.len())];
                        let distance = calculate_route_distance(portion_slice);

                        updated.activity_portions.push(SectionPortion {
                            activity_id: activity_id.clone(),
                            start_index: start,
                            end_index: end,
                            distance_meters: distance,
                            direction,
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
                }
            }

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

        let unmatched_set: HashSet<&str> =
            unmatched_activity_ids.iter().map(|s| s.as_str()).collect();
        let unmatched_tracks: Vec<(String, Vec<GpsPoint>)> = new_tracks
            .iter()
            .filter(|(id, _)| unmatched_set.contains(id.as_str()))
            .cloned()
            .collect();

        // Run full detection on just the unmatched pool. Its renumbering
        // pass starts per-sport counters at 0, so the IDs it returns will
        // collide with the existing sections we kept above (which also
        // follow `sec_<sport>_<n>`). Renumber the new ones to sit above
        // the highest existing per-sport counter.
        let mut result = super::detect_sections_multiscale_with_progress(
            &unmatched_tracks,
            sport_types,
            groups,
            config,
            progress.clone(),
        );
        renumber_to_avoid_collisions(&mut result.sections, &updated_sections);

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

/// Renumber `new_sections` so their `sec_<sport>_<n>` IDs don't collide with
/// the IDs already present in `existing_sections`. Per-sport counters start
/// at one past the highest existing counter for that sport.
///
/// Sections whose ids don't match the canonical `sec_<sport>_<n>` pattern
/// (e.g. user-defined sections, post-processing-suffixed ids that survived
/// renumbering, ids from a future scheme) are left untouched and their ids
/// added to a "taken" set so the new numbering avoids them too.
fn renumber_to_avoid_collisions(
    new_sections: &mut [FrequentSection],
    existing_sections: &[FrequentSection],
) {
    use std::collections::HashMap;

    fn parse_canonical_id(id: &str) -> Option<(String, u32)> {
        let rest = id.strip_prefix("sec_")?;
        let underscore = rest.rfind('_')?;
        let (sport, num_part) = rest.split_at(underscore);
        let n: u32 = num_part[1..].parse().ok()?;
        Some((sport.to_string(), n))
    }

    // Per-sport "next free counter" derived from existing IDs.
    let mut next_counter: HashMap<String, u32> = HashMap::new();
    let mut taken_ids: HashSet<String> = HashSet::new();
    for s in existing_sections {
        taken_ids.insert(s.id.clone());
        if let Some((sport, n)) = parse_canonical_id(&s.id) {
            let slot = next_counter.entry(sport).or_insert(0);
            if n + 1 > *slot {
                *slot = n + 1;
            }
        }
    }

    for section in new_sections.iter_mut() {
        // Only renumber if the id collides; leave clean ids alone so the
        // shape remains predictable.
        if !taken_ids.contains(&section.id) {
            taken_ids.insert(section.id.clone());
            // Still bump the per-sport counter past this id so future
            // unmatched-pool runs don't re-collide.
            if let Some((sport, n)) = parse_canonical_id(&section.id) {
                let slot = next_counter.entry(sport).or_insert(0);
                if n + 1 > *slot {
                    *slot = n + 1;
                }
            }
            continue;
        }

        let sport_key = section.sport_type.to_lowercase();
        let counter = next_counter.entry(sport_key.clone()).or_insert(0);
        loop {
            let candidate = format!("sec_{}_{}", sport_key, *counter);
            *counter += 1;
            if !taken_ids.contains(&candidate) {
                section.id = candidate.clone();
                taken_ids.insert(candidate);
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Direction, GpsPoint};

    fn stub_section(id: &str, sport: &str) -> FrequentSection {
        FrequentSection {
            id: id.to_string(),
            name: None,
            sport_type: sport.to_string(),
            polyline: vec![GpsPoint::new(0.0, 0.0); 2],
            representative_activity_id: String::new(),
            activity_ids: vec![],
            activity_portions: vec![],
            route_ids: vec![],
            visit_count: 0,
            distance_meters: 0.0,
            activity_traces: Default::default(),
            confidence: 0.0,
            observation_count: 0,
            average_spread: 0.0,
            point_density: vec![],
            scale: None,
            is_user_defined: false,
            stability: 0.0,
            version: 1,
            updated_at: None,
            created_at: None,
        }
    }

    #[allow(dead_code)]
    fn _exercise_imports() {
        // Compile-only touch of types we re-import to keep the deny(unused)
        // stable across future changes.
        let _ = GpsPoint::new(0.0, 0.0);
    }

    #[test]
    fn renumber_avoids_collision_with_existing_ids() {
        let existing = vec![
            stub_section("sec_ride_0", "Ride"),
            stub_section("sec_ride_1", "Ride"),
            stub_section("sec_run_0", "Run"),
        ];
        let mut new_sections = vec![
            stub_section("sec_ride_0", "Ride"), // collides
            stub_section("sec_ride_1", "Ride"), // collides
            stub_section("sec_run_0", "Run"),   // collides
        ];

        renumber_to_avoid_collisions(&mut new_sections, &existing);

        let new_ids: Vec<_> = new_sections.iter().map(|s| s.id.as_str()).collect();
        assert_eq!(new_ids, vec!["sec_ride_2", "sec_ride_3", "sec_run_1"]);
    }

    #[test]
    fn renumber_leaves_non_colliding_ids_alone_and_advances_counter() {
        let existing = vec![stub_section("sec_ride_0", "Ride")];
        let mut new_sections = vec![
            stub_section("sec_ride_5", "Ride"), // no collision; keep
            stub_section("sec_ride_0", "Ride"), // collides; first free is 6
        ];
        renumber_to_avoid_collisions(&mut new_sections, &existing);
        assert_eq!(new_sections[0].id, "sec_ride_5");
        assert_eq!(new_sections[1].id, "sec_ride_6");
    }

    #[test]
    fn renumber_handles_empty_existing() {
        let existing: Vec<FrequentSection> = vec![];
        let mut new_sections = vec![
            stub_section("sec_ride_0", "Ride"),
            stub_section("sec_run_0", "Run"),
        ];
        renumber_to_avoid_collisions(&mut new_sections, &existing);
        assert_eq!(new_sections[0].id, "sec_ride_0");
        assert_eq!(new_sections[1].id, "sec_run_0");
    }

    #[test]
    fn renumber_handles_postprocess_suffixed_existing_ids() {
        // Existing ids may carry post-processing suffixes (_split0, _ret).
        // parse_canonical_id returns None for them, so they don't seed the
        // per-sport counter. The first colliding new id therefore lands at
        // sec_ride_0; subsequent ids that would have used that slot must
        // jump past it.
        let existing = vec![stub_section("sec_ride_0_split0", "Ride")];
        let mut new_sections = vec![
            stub_section("sec_ride_0_split0", "Ride"), // exact collision
            stub_section("sec_ride_0", "Ride"),         // becomes taken after first renumber
        ];
        renumber_to_avoid_collisions(&mut new_sections, &existing);
        assert_eq!(new_sections[0].id, "sec_ride_0");
        assert_eq!(new_sections[1].id, "sec_ride_1");
    }
}
