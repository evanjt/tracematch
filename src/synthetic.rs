//! Synthetic GPS data generator for stress testing and benchmarking.
//!
//! Generates realistic GPS activity sets with known shared corridors,
//! providing ground truth for validation of section detection algorithms.
//!
//! Feature-gated behind `synthetic` — not included in production builds.
//!
//! # Example
//!
//! ```rust
//! use tracematch::synthetic::{SyntheticScenario, CorridorConfig, CorridorPattern};
//! use tracematch::GpsPoint;
//!
//! let scenario = SyntheticScenario {
//!     origin: GpsPoint::new(47.37, 8.55),
//!     activity_count: 100,
//!     corridors: vec![CorridorConfig {
//!         length_meters: 10_000.0,
//!         overlap_fraction: 0.8,
//!         pattern: CorridorPattern::Winding,
//!         approach_length: 500.0,
//!     }],
//!     gps_noise_sigma_meters: 3.0,
//!     seed: 42,
//! };
//!
//! let dataset = scenario.generate();
//! assert_eq!(dataset.tracks.len(), 100);
//! ```

use crate::GpsPoint;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::collections::HashMap;
use std::f64::consts::PI;

// ============================================================================
// Types
// ============================================================================

/// Pattern for generating corridor polylines.
#[derive(Debug, Clone, Copy)]
pub enum CorridorPattern {
    /// Straight line with minor perturbation.
    Straight,
    /// Winding road with realistic turns (max 30deg heading change per step).
    Winding,
    /// Loop that returns near its start point.
    Loop,
}

/// Configuration for a shared corridor.
#[derive(Debug, Clone)]
pub struct CorridorConfig {
    /// Length of the shared corridor in meters.
    pub length_meters: f64,
    /// Fraction of activities that traverse this corridor (0.0-1.0).
    pub overlap_fraction: f64,
    /// Shape pattern of the corridor.
    pub pattern: CorridorPattern,
    /// Length of random approach/departure routes in meters.
    pub approach_length: f64,
}

/// An expected section for ground truth validation.
#[derive(Debug, Clone)]
pub struct ExpectedSection {
    /// The corridor polyline (ground truth).
    pub polyline: Vec<GpsPoint>,
    /// Length in meters.
    pub length_meters: f64,
    /// Activity IDs that traverse this corridor.
    pub activity_ids: Vec<String>,
    /// Name for identification.
    pub name: String,
}

/// Metadata about a generated dataset.
#[derive(Debug, Clone)]
pub struct DatasetMetadata {
    /// Total GPS points across all activities.
    pub total_points: usize,
    /// Number of pairwise comparisons (N*(N-1)/2).
    pub total_pairs: usize,
    /// Estimated memory usage in bytes (rough).
    pub estimated_memory_bytes: usize,
}

/// A complete synthetic dataset with ground truth.
pub struct SyntheticDataset {
    /// Generated tracks: (activity_id, gps_points).
    pub tracks: Vec<(String, Vec<GpsPoint>)>,
    /// Sport type for each activity.
    pub sport_types: HashMap<String, String>,
    /// Ground truth expected sections.
    pub expected_sections: Vec<ExpectedSection>,
    /// Dataset statistics.
    pub metadata: DatasetMetadata,
}

/// Scenario configuration for generating synthetic data.
#[derive(Debug, Clone)]
pub struct SyntheticScenario {
    /// Origin point for all generated data.
    pub origin: GpsPoint,
    /// Number of activities to generate.
    pub activity_count: usize,
    /// Shared corridors (ground truth sections).
    pub corridors: Vec<CorridorConfig>,
    /// GPS noise standard deviation in meters.
    pub gps_noise_sigma_meters: f64,
    /// RNG seed for deterministic reproduction.
    pub seed: u64,
}

// ============================================================================
// Coordinate Helpers
// ============================================================================

/// Meters per degree of latitude (approximately constant).
const METERS_PER_DEG_LAT: f64 = 111_320.0;

/// Convert meters to degrees of latitude.
fn meters_to_deg_lat(meters: f64) -> f64 {
    meters / METERS_PER_DEG_LAT
}

/// Convert meters to degrees of longitude at a given latitude.
fn meters_to_deg_lng(meters: f64, latitude: f64) -> f64 {
    let meters_per_deg_lng = METERS_PER_DEG_LAT * latitude.to_radians().cos();
    if meters_per_deg_lng.abs() < 1e-10 {
        return 0.0;
    }
    meters / meters_per_deg_lng
}

/// Approximate distance in meters between two GpsPoints (fast, no trig).
fn approx_distance(a: &GpsPoint, b: &GpsPoint) -> f64 {
    let dlat = (b.latitude - a.latitude) * METERS_PER_DEG_LAT;
    let avg_lat = (a.latitude + b.latitude) / 2.0;
    let dlng = (b.longitude - a.longitude) * METERS_PER_DEG_LAT * avg_lat.to_radians().cos();
    (dlat * dlat + dlng * dlng).sqrt()
}

// ============================================================================
// Corridor Generation
// ============================================================================

/// Point spacing within corridors (meters).
const POINT_SPACING: f64 = 10.0;

/// Generate a corridor polyline of the given length from origin.
fn generate_corridor(
    origin: &GpsPoint,
    config: &CorridorConfig,
    corridor_index: usize,
    rng: &mut StdRng,
) -> Vec<GpsPoint> {
    let num_points = (config.length_meters / POINT_SPACING).ceil() as usize;
    let mut points = Vec::with_capacity(num_points + 1);

    // Each corridor starts in a different direction from origin to avoid overlap
    let base_heading = (corridor_index as f64) * (2.0 * PI / 8.0) + PI / 6.0;
    let mut heading = base_heading;
    let mut current = *origin;

    // Offset corridor start from origin slightly to separate corridors
    let offset_meters = 200.0 * (corridor_index as f64 + 1.0);
    current.latitude += meters_to_deg_lat(offset_meters * heading.sin());
    current.longitude += meters_to_deg_lng(offset_meters * heading.cos(), current.latitude);

    points.push(current);

    for i in 0..num_points {
        let heading_change = match config.pattern {
            CorridorPattern::Straight => {
                // Very slight random drift
                rng.gen_range(-0.02..0.02)
            }
            CorridorPattern::Winding => {
                // Realistic turns: max ~30 degrees per step, with gentle sinusoidal base
                let base_turn = (i as f64 * 0.01).sin() * 0.3;
                let random_turn: f64 = rng.gen_range(-0.15..0.15);
                base_turn + random_turn
            }
            CorridorPattern::Loop => {
                // Gradually curve to form a loop
                let total_turn_needed = 2.0 * PI;
                let turn_per_step = total_turn_needed / num_points as f64;
                turn_per_step + rng.gen_range(-0.05..0.05)
            }
        };

        heading += heading_change;

        // Advance position by POINT_SPACING meters
        let dlat = meters_to_deg_lat(POINT_SPACING * heading.sin());
        let dlng = meters_to_deg_lng(POINT_SPACING * heading.cos(), current.latitude);

        current = GpsPoint::with_elevation(
            current.latitude + dlat,
            current.longitude + dlng,
            // Simple sinusoidal elevation
            300.0 + 50.0 * (i as f64 * 0.005).sin(),
        );

        points.push(current);
    }

    points
}

/// Generate a random approach/departure route.
fn generate_random_segment(
    start: &GpsPoint,
    length_meters: f64,
    initial_heading: f64,
    rng: &mut StdRng,
) -> Vec<GpsPoint> {
    let num_points = (length_meters / POINT_SPACING).ceil() as usize;
    if num_points == 0 {
        return vec![*start];
    }

    let mut points = Vec::with_capacity(num_points + 1);
    let mut heading = initial_heading;
    let mut current = *start;
    points.push(current);

    for i in 0..num_points {
        // Random winding with no constraint
        heading += rng.gen_range(-0.3..0.3);

        let dlat = meters_to_deg_lat(POINT_SPACING * heading.sin());
        let dlng = meters_to_deg_lng(POINT_SPACING * heading.cos(), current.latitude);

        current = GpsPoint::with_elevation(
            current.latitude + dlat,
            current.longitude + dlng,
            300.0 + 30.0 * (i as f64 * 0.01).sin(),
        );

        points.push(current);
    }

    points
}

/// Add Gaussian GPS noise to a polyline.
fn add_gps_noise(points: &[GpsPoint], sigma_meters: f64, rng: &mut StdRng) -> Vec<GpsPoint> {
    if sigma_meters <= 0.0 {
        return points.to_vec();
    }

    points
        .iter()
        .map(|p| {
            // Box-Muller transform for Gaussian noise
            let u1: f64 = rng.gen_range(0.0001..1.0);
            let u2: f64 = rng.r#gen();
            let z0 = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
            let z1 = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).sin();

            let noise_lat = meters_to_deg_lat(z0 * sigma_meters);
            let noise_lng = meters_to_deg_lng(z1 * sigma_meters, p.latitude);

            GpsPoint::with_elevation(
                p.latitude + noise_lat,
                p.longitude + noise_lng,
                p.elevation.unwrap_or(300.0) + rng.gen_range(-2.0..2.0),
            )
        })
        .collect()
}

// ============================================================================
// Scenario Implementation
// ============================================================================

impl SyntheticScenario {
    /// Generate a complete synthetic dataset from this scenario.
    pub fn generate(&self) -> SyntheticDataset {
        let mut rng = StdRng::seed_from_u64(self.seed);

        // Generate corridor polylines
        let corridor_polylines: Vec<Vec<GpsPoint>> = self
            .corridors
            .iter()
            .enumerate()
            .map(|(i, c)| generate_corridor(&self.origin, c, i, &mut rng))
            .collect();

        let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::with_capacity(self.activity_count);
        let mut sport_types = HashMap::new();
        let mut expected_sections: Vec<ExpectedSection> = Vec::new();

        // Track which activities use each corridor
        let mut corridor_activity_ids: Vec<Vec<String>> = vec![Vec::new(); self.corridors.len()];

        for activity_idx in 0..self.activity_count {
            let activity_id = format!("synth_{:04}", activity_idx);
            sport_types.insert(activity_id.clone(), "Ride".to_string());

            let mut full_track: Vec<GpsPoint> = Vec::new();

            // Determine which corridors this activity traverses
            let mut used_any_corridor = false;
            for (ci, corridor_config) in self.corridors.iter().enumerate() {
                let uses_corridor: f64 = rng.r#gen();
                if uses_corridor < corridor_config.overlap_fraction {
                    used_any_corridor = true;
                    corridor_activity_ids[ci].push(activity_id.clone());

                    // Generate approach to corridor start
                    let approach_heading: f64 = rng.gen_range(0.0..(2.0 * PI));
                    let approach_start = GpsPoint::new(
                        corridor_polylines[ci][0].latitude
                            + meters_to_deg_lat(
                                corridor_config.approach_length * approach_heading.sin(),
                            ),
                        corridor_polylines[ci][0].longitude
                            + meters_to_deg_lng(
                                corridor_config.approach_length * approach_heading.cos(),
                                corridor_polylines[ci][0].latitude,
                            ),
                    );

                    let approach = generate_random_segment(
                        &approach_start,
                        corridor_config.approach_length * 0.8,
                        approach_heading + PI, // head toward corridor
                        &mut rng,
                    );

                    // Add approach (connect to corridor)
                    if !full_track.is_empty() {
                        // Connect previous portion to this approach
                        let connector = generate_random_segment(
                            full_track.last().unwrap(),
                            200.0,
                            rng.gen_range(0.0..(2.0 * PI)),
                            &mut rng,
                        );
                        full_track.extend(connector);
                    }
                    full_track.extend(approach);

                    // Add corridor with noise
                    let noisy_corridor = add_gps_noise(
                        &corridor_polylines[ci],
                        self.gps_noise_sigma_meters,
                        &mut rng,
                    );
                    full_track.extend(noisy_corridor);

                    // Generate departure from corridor end
                    let departure_heading: f64 = rng.gen_range(0.0..(2.0 * PI));
                    let departure = generate_random_segment(
                        corridor_polylines[ci].last().unwrap(),
                        corridor_config.approach_length * 0.8,
                        departure_heading,
                        &mut rng,
                    );
                    full_track.extend(departure);
                }
            }

            if !used_any_corridor {
                // Activity with no corridor: purely random route
                let random_length: f64 = rng.gen_range(3000.0..15000.0);
                let random_heading: f64 = rng.gen_range(0.0..(2.0 * PI));
                full_track =
                    generate_random_segment(&self.origin, random_length, random_heading, &mut rng);
            }

            tracks.push((activity_id, full_track));
        }

        // Build expected sections from corridors
        for (ci, corridor_polyline) in corridor_polylines.iter().enumerate() {
            let length = compute_polyline_length(corridor_polyline);
            expected_sections.push(ExpectedSection {
                polyline: corridor_polyline.clone(),
                length_meters: length,
                activity_ids: corridor_activity_ids[ci].clone(),
                name: format!("corridor_{}", ci),
            });
        }

        // Compute metadata
        let total_points: usize = tracks.iter().map(|(_, pts)| pts.len()).sum();
        let n = tracks.len();
        let total_pairs = n * (n.saturating_sub(1)) / 2;
        // ~24 bytes per GpsPoint (2 f64 + Option<f64>)
        let estimated_memory_bytes = total_points * 24;

        SyntheticDataset {
            tracks,
            sport_types,
            expected_sections,
            metadata: DatasetMetadata {
                total_points,
                total_pairs,
                estimated_memory_bytes,
            },
        }
    }
}

/// Compute the length of a polyline in meters.
fn compute_polyline_length(points: &[GpsPoint]) -> f64 {
    if points.len() < 2 {
        return 0.0;
    }
    points
        .windows(2)
        .map(|w| approx_distance(&w[0], &w[1]))
        .sum()
}

// ============================================================================
// Predefined Scenarios
// ============================================================================

/// Zurich origin — central Europe, representative latitude for GPS calculations.
const ZURICH: GpsPoint = GpsPoint {
    latitude: 47.37,
    longitude: 8.55,
    elevation: None,
};

impl SyntheticScenario {
    /// 100 activities, 1x 10km corridor with 80% overlap. Baseline benchmark.
    pub fn standard_cycling() -> Self {
        Self {
            origin: ZURICH,
            activity_count: 100,
            corridors: vec![CorridorConfig {
                length_meters: 10_000.0,
                overlap_fraction: 0.8,
                pattern: CorridorPattern::Winding,
                approach_length: 500.0,
            }],
            gps_noise_sigma_meters: 3.0,
            seed: 42,
        }
    }

    /// 200 activities, 1x 70km corridor with 60% overlap. Tests long section detection.
    pub fn long_sections() -> Self {
        Self {
            origin: ZURICH,
            activity_count: 200,
            corridors: vec![CorridorConfig {
                length_meters: 70_000.0,
                overlap_fraction: 0.6,
                pattern: CorridorPattern::Winding,
                approach_length: 1000.0,
            }],
            gps_noise_sigma_meters: 4.0,
            seed: 43,
        }
    }

    /// 1000 activities, 1x 10km corridor with 50% overlap. Finds the N^2 cliff.
    pub fn extreme_scale() -> Self {
        Self {
            origin: ZURICH,
            activity_count: 1000,
            corridors: vec![CorridorConfig {
                length_meters: 10_000.0,
                overlap_fraction: 0.5,
                pattern: CorridorPattern::Winding,
                approach_length: 500.0,
            }],
            gps_noise_sigma_meters: 3.0,
            seed: 44,
        }
    }

    /// 200 activities, no corridors. Worst case: O(N^2) work, zero sections found.
    pub fn no_overlap() -> Self {
        Self {
            origin: ZURICH,
            activity_count: 200,
            corridors: vec![],
            gps_noise_sigma_meters: 3.0,
            seed: 45,
        }
    }

    /// 300 activities, 5 corridors ranging 0.5km to 70km. Realistic mix.
    pub fn multi_corridor() -> Self {
        Self {
            origin: ZURICH,
            activity_count: 300,
            corridors: vec![
                CorridorConfig {
                    length_meters: 500.0,
                    overlap_fraction: 0.7,
                    pattern: CorridorPattern::Straight,
                    approach_length: 200.0,
                },
                CorridorConfig {
                    length_meters: 2_000.0,
                    overlap_fraction: 0.6,
                    pattern: CorridorPattern::Winding,
                    approach_length: 400.0,
                },
                CorridorConfig {
                    length_meters: 10_000.0,
                    overlap_fraction: 0.5,
                    pattern: CorridorPattern::Winding,
                    approach_length: 600.0,
                },
                CorridorConfig {
                    length_meters: 30_000.0,
                    overlap_fraction: 0.3,
                    pattern: CorridorPattern::Winding,
                    approach_length: 800.0,
                },
                CorridorConfig {
                    length_meters: 70_000.0,
                    overlap_fraction: 0.15,
                    pattern: CorridorPattern::Winding,
                    approach_length: 1000.0,
                },
            ],
            gps_noise_sigma_meters: 3.5,
            seed: 46,
        }
    }

    /// 500 activities, 1x 5km corridor with 90% overlap. High overlap density.
    pub fn urban_commute() -> Self {
        Self {
            origin: ZURICH,
            activity_count: 500,
            corridors: vec![CorridorConfig {
                length_meters: 5_000.0,
                overlap_fraction: 0.9,
                pattern: CorridorPattern::Winding,
                approach_length: 300.0,
            }],
            gps_noise_sigma_meters: 5.0,
            seed: 47,
        }
    }

    /// Configurable scenario for benchmarks: N activities, single corridor.
    pub fn with_activity_count(count: usize, corridor_length: f64, overlap: f64) -> Self {
        Self {
            origin: ZURICH,
            activity_count: count,
            corridors: vec![CorridorConfig {
                length_meters: corridor_length,
                overlap_fraction: overlap,
                pattern: CorridorPattern::Winding,
                approach_length: (corridor_length * 0.05).max(200.0),
            }],
            gps_noise_sigma_meters: 3.0,
            seed: count as u64 * 1000 + (corridor_length as u64),
        }
    }

    /// Configurable scenario with no corridors (pure overhead measurement).
    pub fn with_no_overlap(count: usize) -> Self {
        Self {
            origin: ZURICH,
            activity_count: count,
            corridors: vec![],
            gps_noise_sigma_meters: 3.0,
            seed: count as u64 * 7919,
        }
    }
}

// ============================================================================
// RouteGroup helper for benchmarks
// ============================================================================

impl SyntheticDataset {
    /// Create simple route groups (each activity in its own group) for section detection.
    pub fn route_groups(&self) -> Vec<crate::RouteGroup> {
        self.tracks
            .iter()
            .enumerate()
            .map(|(i, (id, _))| crate::RouteGroup {
                group_id: format!("group_{}", i),
                representative_id: id.clone(),
                activity_ids: vec![id.clone()],
                sport_type: "Ride".to_string(),
                bounds: None,
                custom_name: None,
                best_time: None,
                avg_time: None,
                best_pace: None,
                best_activity_id: None,
            })
            .collect()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_standard_cycling_generation() {
        let scenario = SyntheticScenario::standard_cycling();
        let dataset = scenario.generate();

        assert_eq!(dataset.tracks.len(), 100);
        assert_eq!(dataset.sport_types.len(), 100);
        assert_eq!(dataset.expected_sections.len(), 1);

        // ~80% of 100 activities should use the corridor
        let corridor_users = dataset.expected_sections[0].activity_ids.len();
        assert!(
            corridor_users >= 60 && corridor_users <= 95,
            "Expected ~80 corridor users, got {}",
            corridor_users
        );

        // Corridor should be approximately 10km
        let length = dataset.expected_sections[0].length_meters;
        assert!(
            length > 9_000.0 && length < 11_000.0,
            "Expected ~10km corridor, got {}m",
            length
        );
    }

    #[test]
    fn test_no_overlap_generation() {
        let scenario = SyntheticScenario::no_overlap();
        let dataset = scenario.generate();

        assert_eq!(dataset.tracks.len(), 200);
        assert!(dataset.expected_sections.is_empty());
    }

    #[test]
    fn test_deterministic_generation() {
        let scenario = SyntheticScenario::standard_cycling();
        let dataset1 = scenario.generate();
        let dataset2 = scenario.generate();

        // Same seed should produce identical tracks
        assert_eq!(dataset1.tracks.len(), dataset2.tracks.len());
        for (t1, t2) in dataset1.tracks.iter().zip(dataset2.tracks.iter()) {
            assert_eq!(t1.0, t2.0); // Same activity IDs
            assert_eq!(t1.1.len(), t2.1.len()); // Same point counts
            if let (Some(first1), Some(first2)) = (t1.1.first(), t2.1.first()) {
                assert_eq!(first1.latitude, first2.latitude);
                assert_eq!(first1.longitude, first2.longitude);
            }
        }
    }

    #[test]
    fn test_gps_noise_applied() {
        let scenario = SyntheticScenario {
            origin: ZURICH,
            activity_count: 2,
            corridors: vec![CorridorConfig {
                length_meters: 1000.0,
                overlap_fraction: 1.0,
                pattern: CorridorPattern::Straight,
                approach_length: 100.0,
            }],
            gps_noise_sigma_meters: 5.0,
            seed: 100,
        };

        let dataset = scenario.generate();
        let track1 = &dataset.tracks[0].1;
        let track2 = &dataset.tracks[1].1;

        // Both tracks should exist and have points
        assert!(track1.len() > 10);
        assert!(track2.len() > 10);

        // Tracks should not be identical (noise is applied)
        let mut any_different = false;
        let min_len = track1.len().min(track2.len());
        for i in 0..min_len {
            if track1[i].latitude != track2[i].latitude {
                any_different = true;
                break;
            }
        }
        assert!(any_different, "Tracks should differ due to GPS noise");
    }

    #[test]
    fn test_multi_corridor_generation() {
        let scenario = SyntheticScenario::multi_corridor();
        let dataset = scenario.generate();

        assert_eq!(dataset.tracks.len(), 300);
        assert_eq!(dataset.expected_sections.len(), 5);

        // Corridors should have different lengths
        let lengths: Vec<f64> = dataset
            .expected_sections
            .iter()
            .map(|s| s.length_meters)
            .collect();

        // Check order: should be roughly 0.5km, 2km, 10km, 30km, 70km
        assert!(lengths[0] < 1_000.0, "First corridor should be ~0.5km");
        assert!(lengths[4] > 50_000.0, "Last corridor should be ~70km");
    }

    #[test]
    fn test_long_section_generation() {
        let scenario = SyntheticScenario::long_sections();
        let dataset = scenario.generate();

        assert_eq!(dataset.tracks.len(), 200);
        assert_eq!(dataset.expected_sections.len(), 1);

        // Corridor should be approximately 70km
        let length = dataset.expected_sections[0].length_meters;
        assert!(
            length > 60_000.0 && length < 80_000.0,
            "Expected ~70km corridor, got {}m",
            length
        );

        // Corridor polyline should have many points (~7000 at 10m spacing)
        let polyline_points = dataset.expected_sections[0].polyline.len();
        assert!(
            polyline_points > 5000,
            "Expected 5000+ points for 70km corridor, got {}",
            polyline_points
        );
    }

    #[test]
    fn test_metadata_computation() {
        let scenario = SyntheticScenario::with_activity_count(10, 1000.0, 0.5);
        let dataset = scenario.generate();

        assert_eq!(dataset.metadata.total_pairs, 45); // 10*9/2
        assert!(dataset.metadata.total_points > 0);
        assert!(dataset.metadata.estimated_memory_bytes > 0);
    }

    #[test]
    fn test_with_no_overlap_helper() {
        let scenario = SyntheticScenario::with_no_overlap(50);
        let dataset = scenario.generate();

        assert_eq!(dataset.tracks.len(), 50);
        assert!(dataset.expected_sections.is_empty());
        assert_eq!(dataset.metadata.total_pairs, 1225); // 50*49/2
    }

    #[test]
    fn test_route_groups_helper() {
        let scenario = SyntheticScenario::with_activity_count(5, 1000.0, 0.5);
        let dataset = scenario.generate();
        let groups = dataset.route_groups();

        assert_eq!(groups.len(), 5);
        for (i, group) in groups.iter().enumerate() {
            assert_eq!(group.group_id, format!("group_{}", i));
            assert_eq!(group.sport_type, "Ride");
            assert_eq!(group.activity_ids.len(), 1);
        }
    }

    #[test]
    fn test_valid_gps_coordinates() {
        let scenario = SyntheticScenario::standard_cycling();
        let dataset = scenario.generate();

        for (id, track) in &dataset.tracks {
            for (i, point) in track.iter().enumerate() {
                assert!(
                    point.is_valid(),
                    "Invalid GPS point in {}: index {} = ({}, {})",
                    id,
                    i,
                    point.latitude,
                    point.longitude
                );
            }
        }
    }
}
