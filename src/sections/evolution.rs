//! Section Evolution - Incremental Updates and Merging
//!
//! Provides functionality for evolving sections as new GPS traces are added:
//! - Incremental consensus updates (refine polyline with new observations)
//! - Section merging (combine overlapping detected sections)
//! - Stability tracking (how established a section has become)
//!
//! Key principles:
//! - User-defined sections are immutable (is_user_defined = true)
//! - Highly stable sections receive smaller updates
//! - New observations contribute less as section matures

use super::{
    compute_consensus_polyline, compute_initial_stability, FrequentSection, SectionConfig,
    SectionPortion,
};
use crate::geo_utils::haversine_distance;
use crate::matching::calculate_route_distance;
use crate::GpsPoint;
use log::info;
use std::collections::HashMap;

/// Result of updating a section with new traces
#[derive(Debug, Clone)]
pub struct SectionUpdateResult {
    /// The updated section
    pub section: FrequentSection,
    /// Whether the section was actually modified
    pub was_modified: bool,
    /// Number of new activities incorporated
    pub new_activities_added: u32,
    /// Change in confidence score
    pub confidence_delta: f64,
    /// Change in stability score
    pub stability_delta: f64,
}

/// Update an existing section with new GPS traces.
///
/// This implements incremental consensus refinement:
/// 1. Only updates if section is not user-defined
/// 2. Weight new observations based on section stability (stable sections change less)
/// 3. Incrementally update consensus polyline
/// 4. Track version and update timestamps
///
/// # Arguments
/// * `section` - The existing section to update
/// * `new_traces` - Map of activity_id -> GPS trace for new activities
/// * `config` - Section detection configuration
/// * `timestamp` - Optional ISO timestamp for updated_at field
///
/// # Returns
/// SectionUpdateResult with the updated section and change metrics
pub fn update_section_with_new_traces(
    section: &FrequentSection,
    new_traces: &HashMap<String, Vec<GpsPoint>>,
    config: &SectionConfig,
    timestamp: Option<String>,
) -> SectionUpdateResult {
    // Don't modify user-defined sections
    if section.is_user_defined {
        return SectionUpdateResult {
            section: section.clone(),
            was_modified: false,
            new_activities_added: 0,
            confidence_delta: 0.0,
            stability_delta: 0.0,
        };
    }

    // Filter to traces that are near our section but not already included
    let existing_activity_ids: std::collections::HashSet<_> =
        section.activity_ids.iter().cloned().collect();

    let relevant_traces: HashMap<String, Vec<GpsPoint>> = new_traces
        .iter()
        .filter(|(id, _)| !existing_activity_ids.contains(*id))
        .filter(|(_, trace)| is_trace_near_section(trace, &section.polyline, config))
        .map(|(id, trace)| (id.clone(), trace.clone()))
        .collect();

    if relevant_traces.is_empty() {
        return SectionUpdateResult {
            section: section.clone(),
            was_modified: false,
            new_activities_added: 0,
            confidence_delta: 0.0,
            stability_delta: 0.0,
        };
    }

    info!(
        "[Evolution] Updating section {} with {} new traces",
        section.id,
        relevant_traces.len()
    );

    // Build the full track map (existing + new)
    let mut full_track_map = section.activity_traces.clone();
    for (id, trace) in &relevant_traces {
        // Extract the portion near the section
        let extracted =
            extract_trace_near_section(trace, &section.polyline, config.proximity_threshold);
        if !extracted.is_empty() {
            full_track_map.insert(id.clone(), extracted);
        }
    }

    // Compute new consensus from all traces (existing + new)
    let all_traces: Vec<Vec<GpsPoint>> = full_track_map.values().cloned().collect();

    // Use stability-weighted blending: highly stable sections get smaller updates
    let blend_factor = 1.0 - (section.stability * 0.7); // Range: 0.3 to 1.0

    let new_consensus = compute_weighted_consensus(
        &section.polyline,
        &all_traces,
        config.proximity_threshold,
        blend_factor,
    );

    // Update section fields
    let mut updated = section.clone();
    let old_confidence = section.confidence;
    let old_stability = section.stability;

    // Blend old and new polyline based on stability
    updated.polyline = blend_polylines(&section.polyline, &new_consensus.polyline, blend_factor);
    updated.activity_ids.extend(relevant_traces.keys().cloned());
    updated.activity_traces = full_track_map;
    updated.visit_count += relevant_traces.len() as u32;
    updated.observation_count = all_traces.len() as u32;
    updated.distance_meters = calculate_route_distance(&updated.polyline);
    updated.confidence = new_consensus.confidence;
    updated.average_spread = new_consensus.average_spread;
    updated.point_density = new_consensus.point_density;
    updated.version += 1;
    updated.updated_at = timestamp;

    // Update stability (increases with more observations)
    updated.stability = compute_initial_stability(
        updated.observation_count,
        updated.average_spread,
        config.proximity_threshold,
    );

    // Update activity portions for new activities
    for (activity_id, trace) in &relevant_traces {
        if let Some(portion) = compute_portion_for_trace(activity_id, trace, &updated.polyline) {
            updated.activity_portions.push(portion);
        }
    }

    SectionUpdateResult {
        section: updated.clone(),
        was_modified: true,
        new_activities_added: relevant_traces.len() as u32,
        confidence_delta: updated.confidence - old_confidence,
        stability_delta: updated.stability - old_stability,
    }
}

/// Merge multiple overlapping sections into a single section.
///
/// Use this when:
/// - Re-running detection finds sections that should be combined
/// - User explicitly requests a merge
///
/// The merged section inherits:
/// - All activity IDs from both sections
/// - Combined activity traces
/// - Consensus polyline from merged observations
///
/// # Arguments
/// * `sections` - The sections to merge (must be same sport type)
/// * `config` - Section detection configuration
/// * `new_id` - ID for the merged section
/// * `timestamp` - Optional ISO timestamp for created_at field
///
/// # Returns
/// The merged FrequentSection, or None if merge is not valid
pub fn merge_overlapping_sections(
    sections: &[FrequentSection],
    config: &SectionConfig,
    new_id: String,
    timestamp: Option<String>,
) -> Option<FrequentSection> {
    if sections.is_empty() {
        return None;
    }

    if sections.len() == 1 {
        return Some(sections[0].clone());
    }

    // Don't merge user-defined sections
    if sections.iter().any(|s| s.is_user_defined) {
        info!("[Evolution] Cannot merge: contains user-defined sections");
        return None;
    }

    // All sections must be same sport type
    let sport_type = &sections[0].sport_type;
    if !sections.iter().all(|s| &s.sport_type == sport_type) {
        info!("[Evolution] Cannot merge: different sport types");
        return None;
    }

    info!(
        "[Evolution] Merging {} sections into {}",
        sections.len(),
        new_id
    );

    // Collect all activity IDs
    let mut all_activity_ids: Vec<String> = sections
        .iter()
        .flat_map(|s| s.activity_ids.iter().cloned())
        .collect();
    all_activity_ids.sort();
    all_activity_ids.dedup();

    // Collect all activity traces
    let mut all_traces: HashMap<String, Vec<GpsPoint>> = HashMap::new();
    for section in sections {
        for (id, trace) in &section.activity_traces {
            // If activity appears in multiple sections, use the longer trace
            if let Some(existing) = all_traces.get(id) {
                if trace.len() > existing.len() {
                    all_traces.insert(id.clone(), trace.clone());
                }
            } else {
                all_traces.insert(id.clone(), trace.clone());
            }
        }
    }

    // Collect all route IDs
    let mut all_route_ids: Vec<String> = sections
        .iter()
        .flat_map(|s| s.route_ids.iter().cloned())
        .collect();
    all_route_ids.sort();
    all_route_ids.dedup();

    // Use the longest section's polyline as the reference
    let reference_section = sections
        .iter()
        .max_by(|a, b| {
            a.distance_meters
                .partial_cmp(&b.distance_meters)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap();

    // Compute new consensus from all traces
    let trace_list: Vec<Vec<GpsPoint>> = all_traces.values().cloned().collect();
    let consensus = compute_consensus_polyline(
        &reference_section.polyline,
        &trace_list,
        config.proximity_threshold,
    );

    // Combine activity portions
    let mut all_portions: Vec<SectionPortion> = Vec::new();
    for section in sections {
        all_portions.extend(section.activity_portions.iter().cloned());
    }
    // Remove duplicates by activity_id (keep first occurrence)
    let mut seen_activities = std::collections::HashSet::new();
    all_portions.retain(|p| seen_activities.insert(p.activity_id.clone()));

    let stability = compute_initial_stability(
        consensus.observation_count,
        consensus.average_spread,
        config.proximity_threshold,
    );

    Some(FrequentSection {
        id: new_id,
        name: None, // User can set later
        sport_type: sport_type.clone(),
        polyline: consensus.polyline,
        representative_activity_id: reference_section.representative_activity_id.clone(),
        activity_ids: all_activity_ids,
        activity_portions: all_portions,
        route_ids: all_route_ids,
        visit_count: all_traces.len() as u32,
        distance_meters: calculate_route_distance(&reference_section.polyline),
        activity_traces: all_traces,
        confidence: consensus.confidence,
        observation_count: consensus.observation_count,
        average_spread: consensus.average_spread,
        point_density: consensus.point_density,
        scale: reference_section.scale.clone(),
        version: 1,
        is_user_defined: false,
        created_at: timestamp,
        updated_at: None,
        stability,
    })
}

/// Check if a trace passes near a section polyline.
fn is_trace_near_section(
    trace: &[GpsPoint],
    section_polyline: &[GpsPoint],
    config: &SectionConfig,
) -> bool {
    if trace.is_empty() || section_polyline.is_empty() {
        return false;
    }

    let threshold = config.proximity_threshold * 1.5; // Slightly larger threshold for matching

    // Sample points along the section and check if trace passes near them
    let sample_step = (section_polyline.len() / 10).max(1);
    let mut near_count = 0;
    let mut samples_checked = 0;

    for (i, section_point) in section_polyline.iter().enumerate() {
        if i % sample_step != 0 {
            continue;
        }
        samples_checked += 1;

        for trace_point in trace {
            let dist = haversine_distance(section_point, trace_point);
            if dist <= threshold {
                near_count += 1;
                break;
            }
        }
    }

    // At least 60% of samples should have nearby trace points
    samples_checked > 0 && (near_count as f64 / samples_checked as f64) >= 0.6
}

/// Extract the portion of a trace that is near the section polyline.
fn extract_trace_near_section(
    trace: &[GpsPoint],
    section_polyline: &[GpsPoint],
    proximity_threshold: f64,
) -> Vec<GpsPoint> {
    let threshold = proximity_threshold * 1.2; // 20% buffer

    trace
        .iter()
        .filter(|point| {
            section_polyline
                .iter()
                .any(|sp| haversine_distance(point, sp) <= threshold)
        })
        .cloned()
        .collect()
}

/// Compute a weighted consensus from all traces.
fn compute_weighted_consensus(
    reference: &[GpsPoint],
    traces: &[Vec<GpsPoint>],
    proximity_threshold: f64,
    blend_factor: f64,
) -> super::consensus::ConsensusResult {
    // For now, just use the standard consensus with the blend factor applied
    // In a more sophisticated implementation, we'd weight traces differently
    let _ = blend_factor; // TODO: Use blend_factor to weight contributions
    compute_consensus_polyline(reference, traces, proximity_threshold)
}

/// Blend two polylines together based on a factor.
/// factor = 1.0 means use entirely new_polyline
/// factor = 0.0 means use entirely old_polyline
fn blend_polylines(
    old_polyline: &[GpsPoint],
    new_polyline: &[GpsPoint],
    factor: f64,
) -> Vec<GpsPoint> {
    if factor >= 1.0 {
        return new_polyline.to_vec();
    }
    if factor <= 0.0 {
        return old_polyline.to_vec();
    }

    // Use the new polyline but blend point positions
    // This is a simple linear interpolation approach
    let factor = factor.clamp(0.0, 1.0);

    // Match points by normalized position along polyline
    let mut blended = Vec::with_capacity(new_polyline.len());

    for (i, new_point) in new_polyline.iter().enumerate() {
        // Find corresponding point in old polyline (by normalized position)
        let normalized_pos = i as f64 / (new_polyline.len().max(1) - 1).max(1) as f64;
        let old_idx = (normalized_pos * (old_polyline.len().max(1) - 1) as f64).round() as usize;
        let old_idx = old_idx.min(old_polyline.len().saturating_sub(1));

        if old_idx < old_polyline.len() {
            let old_point = &old_polyline[old_idx];
            let blended_lat = old_point.latitude * (1.0 - factor) + new_point.latitude * factor;
            let blended_lng = old_point.longitude * (1.0 - factor) + new_point.longitude * factor;
            blended.push(GpsPoint::new(blended_lat, blended_lng));
        } else {
            blended.push(*new_point);
        }
    }

    blended
}

/// Compute a SectionPortion for a new trace relative to a section polyline.
fn compute_portion_for_trace(
    activity_id: &str,
    trace: &[GpsPoint],
    _section_polyline: &[GpsPoint],
) -> Option<SectionPortion> {
    if trace.is_empty() {
        return None;
    }

    let distance = calculate_route_distance(trace);

    Some(SectionPortion {
        activity_id: activity_id.to_string(),
        start_index: 0, // We don't have index info for new traces
        end_index: trace.len() as u32,
        distance_meters: distance,
        direction: "same".to_string(), // Would need direction detection
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_section() -> FrequentSection {
        FrequentSection {
            id: "test_sec".to_string(),
            name: None,
            sport_type: "Run".to_string(),
            polyline: vec![GpsPoint::new(46.23, 7.36), GpsPoint::new(46.24, 7.37)],
            representative_activity_id: "act1".to_string(),
            activity_ids: vec!["act1".to_string()],
            activity_portions: vec![],
            route_ids: vec![],
            visit_count: 1,
            distance_meters: 1000.0,
            activity_traces: HashMap::new(),
            confidence: 0.5,
            observation_count: 1,
            average_spread: 25.0,
            point_density: vec![1, 1],
            scale: Some("medium".to_string()),
            version: 1,
            is_user_defined: false,
            created_at: None,
            updated_at: None,
            stability: 0.3,
        }
    }

    #[test]
    fn test_user_defined_not_modified() {
        let mut section = make_test_section();
        section.is_user_defined = true;

        let new_traces = HashMap::new();
        let config = SectionConfig::default();

        let result = update_section_with_new_traces(&section, &new_traces, &config, None);

        assert!(!result.was_modified);
        assert_eq!(result.new_activities_added, 0);
    }

    #[test]
    fn test_empty_traces_not_modified() {
        let section = make_test_section();
        let new_traces = HashMap::new();
        let config = SectionConfig::default();

        let result = update_section_with_new_traces(&section, &new_traces, &config, None);

        assert!(!result.was_modified);
    }

    #[test]
    fn test_blend_polylines_extremes() {
        let old = vec![GpsPoint::new(0.0, 0.0), GpsPoint::new(1.0, 1.0)];
        let new = vec![GpsPoint::new(2.0, 2.0), GpsPoint::new(3.0, 3.0)];

        // Factor 0 = all old
        let result = blend_polylines(&old, &new, 0.0);
        assert!((result[0].latitude - 0.0).abs() < 0.001);

        // Factor 1 = all new
        let result = blend_polylines(&old, &new, 1.0);
        assert!((result[0].latitude - 2.0).abs() < 0.001);

        // Factor 0.5 = blend
        let result = blend_polylines(&old, &new, 0.5);
        assert!((result[0].latitude - 1.0).abs() < 0.001);
    }
}
