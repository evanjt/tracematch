//! Lifecycle scenarios for end-to-end testing of the section-detection
//! pipeline.
//!
//! These scenarios simulate the *real usage lifecycle* of the app: cold
//! start, expanding the sync time-range, adding one or a few new activities,
//! and large bulk imports. Each bucket is generated deterministically from a
//! single seed so tests can stack them through the SQLite engine without any
//! external data dependency.
//!
//! Buckets (chronological, disjoint):
//! - `bucket_a` — first 90 days, cold-start corpus
//! - `bucket_b_delta` — days 90..365, the "expand to 1 year" delta
//! - `bucket_c_single` — one new activity dropped on top of B
//! - `bucket_d_delta` — three more activities (mixed cases)
//! - `bucket_e_delta` — large delta extending the corpus to ~2 years
//!
//! Compared to [`crate::synthetic::SyntheticScenario`] (single-corridor,
//! single-sport, IID Gaussian noise), this module adds:
//! - per-activity sport assignment (Ride / Run / cross-sport corridor)
//! - forward / reverse traversal flagged per activity
//! - parallel-street near-miss activities (offset corridor, similar
//!   elevation profile — should NOT match the primary corridor)
//! - first-order Gauss–Markov GPS noise (see [`super::noise`])
//! - per-activity start-date metadata, suitable for the SQLite ingest path

use super::noise::{GaussMarkovConfig, GaussMarkovNoise};
use crate::GpsPoint;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::collections::HashMap;
use std::f64::consts::PI;

const METERS_PER_DEG_LAT: f64 = 111_320.0;
const POINT_SPACING_M: f64 = 10.0;

fn meters_to_deg_lat(meters: f64) -> f64 {
    meters / METERS_PER_DEG_LAT
}

fn meters_to_deg_lng(meters: f64, latitude: f64) -> f64 {
    let cos = latitude.to_radians().cos();
    if cos.abs() < 1e-10 {
        0.0
    } else {
        meters / (METERS_PER_DEG_LAT * cos)
    }
}

/// One synthetic activity with the metadata the DB layer expects.
#[derive(Debug, Clone)]
pub struct LifecycleActivity {
    pub id: String,
    pub sport_type: String,
    /// Unix timestamp (seconds) of the activity start.
    pub start_date_unix: i64,
    pub gps_points: Vec<GpsPoint>,
}

/// Metadata describing a corridor that activities may share.
#[derive(Debug, Clone)]
pub struct CorridorTruth {
    pub id: String,
    /// Canonical (noise-free) polyline.
    pub polyline: Vec<GpsPoint>,
    /// Sport types that traverse this corridor. Single-sport corridors
    /// produce one section; cross-sport corridors test the cross-sport-merge
    /// path in `apply_sections`.
    pub sport_types: Vec<String>,
    /// Activity IDs that traverse this corridor (across all buckets).
    pub activity_ids: Vec<String>,
    pub length_meters: f64,
}

/// Configuration for generating a lifecycle corpus.
#[derive(Debug, Clone)]
pub struct LifecycleConfig {
    pub origin: GpsPoint,
    pub seed: u64,
    pub gps_noise: GaussMarkovConfig,

    /// Activity counts per bucket. Defaults are CI-fast (~35 s total).
    pub bucket_a_count: usize, // cold start, e.g. 60
    pub bucket_b_delta_count: usize, // expand to 1y, e.g. 90
    pub bucket_d_delta_count: usize, // small batch, e.g. 3
    pub bucket_e_delta_count: usize, // year expansion, e.g. 396 (→ 550 total at E)

    /// Fraction of activities that include the primary cycling corridor.
    pub ride_corridor_overlap: f64,
    /// Fraction of activities that include the running corridor (independent
    /// of the cycling corridor — an activity may use both via the cross-sport
    /// shared corridor).
    pub run_corridor_overlap: f64,
    /// Fraction of overlapping traversals that go in the reverse direction.
    pub reverse_fraction: f64,
    /// Number of "parallel street" activities to seed across all buckets —
    /// these run a corridor offset by `parallel_street_offset_meters` from a
    /// primary corridor with similar elevation. They MUST NOT match.
    pub parallel_street_count: usize,
    pub parallel_street_offset_meters: f64,
    /// Fraction of activities that are pure one-offs (no corridor overlap).
    pub one_off_fraction: f64,
}

impl Default for LifecycleConfig {
    fn default() -> Self {
        Self {
            origin: GpsPoint::with_elevation(47.37, 8.55, 410.0),
            seed: 0xC0FFEE,
            gps_noise: GaussMarkovConfig::default(),
            bucket_a_count: 60,
            bucket_b_delta_count: 90,
            bucket_d_delta_count: 3,
            bucket_e_delta_count: 396,
            ride_corridor_overlap: 0.7,
            run_corridor_overlap: 0.5,
            reverse_fraction: 0.15,
            parallel_street_count: 8,
            parallel_street_offset_meters: 60.0,
            one_off_fraction: 0.15,
        }
    }
}

/// One generated corpus, partitioned into the lifecycle buckets.
#[derive(Debug, Clone)]
pub struct LifecycleCorpus {
    pub bucket_a: Vec<LifecycleActivity>,
    pub bucket_b_delta: Vec<LifecycleActivity>,
    pub bucket_c_single: LifecycleActivity,
    pub bucket_d_delta: Vec<LifecycleActivity>,
    pub bucket_e_delta: Vec<LifecycleActivity>,
    /// Ground-truth corridors. Useful for assertions in scenario harnesses
    /// (e.g. "the section detected here should correspond to corridor X").
    pub corridors: Vec<CorridorTruth>,
}

impl LifecycleCorpus {
    pub fn generate(config: &LifecycleConfig) -> Self {
        let generator = CorpusGenerator::new(config);
        generator.run()
    }

    /// All activities up to and including bucket A (cold start state).
    pub fn through_a(&self) -> Vec<&LifecycleActivity> {
        self.bucket_a.iter().collect()
    }

    /// All activities up to and including bucket B (1-year state).
    pub fn through_b(&self) -> Vec<&LifecycleActivity> {
        self.bucket_a
            .iter()
            .chain(self.bucket_b_delta.iter())
            .collect()
    }

    /// All activities up to and including bucket C (1y + 1 activity).
    pub fn through_c(&self) -> Vec<&LifecycleActivity> {
        let mut v = self.through_b();
        v.push(&self.bucket_c_single);
        v
    }

    /// All activities up to and including bucket D.
    pub fn through_d(&self) -> Vec<&LifecycleActivity> {
        let mut v = self.through_c();
        v.extend(self.bucket_d_delta.iter());
        v
    }

    /// All activities up to and including bucket E (~2 years).
    pub fn through_e(&self) -> Vec<&LifecycleActivity> {
        let mut v = self.through_d();
        v.extend(self.bucket_e_delta.iter());
        v
    }

    /// Convenience: the sport-type map the section detector wants.
    pub fn sport_map_through_e(&self) -> HashMap<String, String> {
        self.through_e()
            .iter()
            .map(|a| (a.id.clone(), a.sport_type.clone()))
            .collect()
    }

    /// Convenience: the (id, points) tuples the section detector wants.
    pub fn tracks_through_e(&self) -> Vec<(String, Vec<GpsPoint>)> {
        self.through_e()
            .iter()
            .map(|a| (a.id.clone(), a.gps_points.clone()))
            .collect()
    }
}

// ============================================================================
// Internal generator
// ============================================================================

const SECONDS_PER_DAY: i64 = 86_400;

/// Reference epoch for synthetic dates. Picking a fixed Unix timestamp keeps
/// the generated dates stable across CI runs.
const EPOCH_BASE_UNIX: i64 = 1_700_000_000; // 2023-11-14T22:13:20Z

struct CorpusGenerator<'a> {
    config: &'a LifecycleConfig,
    rng: StdRng,
    // Canonical corridor polylines, generated once and shared across buckets.
    ride_corridor: Vec<GpsPoint>,
    run_corridor: Vec<GpsPoint>,
    cross_sport_corridor: Vec<GpsPoint>,
    /// Per-corridor ground truth, populated as activities are emitted.
    corridor_truth: Vec<CorridorTruth>,
    next_activity_index: usize,
}

impl<'a> CorpusGenerator<'a> {
    fn new(config: &'a LifecycleConfig) -> Self {
        let mut rng = StdRng::seed_from_u64(config.seed);
        let ride_corridor = generate_winding_corridor(
            &config.origin,
            10_000.0,
            heading_radians(0),
            offset_meters(0),
            &mut rng,
        );
        let run_corridor = generate_winding_corridor(
            &config.origin,
            5_000.0,
            heading_radians(2),
            offset_meters(2),
            &mut rng,
        );
        let cross_sport_corridor = generate_winding_corridor(
            &config.origin,
            3_000.0,
            heading_radians(4),
            offset_meters(4),
            &mut rng,
        );

        let corridor_truth = vec![
            CorridorTruth {
                id: "ride_main".to_string(),
                polyline: ride_corridor.clone(),
                sport_types: vec!["Ride".to_string()],
                activity_ids: Vec::new(),
                length_meters: polyline_length(&ride_corridor),
            },
            CorridorTruth {
                id: "run_main".to_string(),
                polyline: run_corridor.clone(),
                sport_types: vec!["Run".to_string()],
                activity_ids: Vec::new(),
                length_meters: polyline_length(&run_corridor),
            },
            CorridorTruth {
                id: "cross_sport".to_string(),
                polyline: cross_sport_corridor.clone(),
                sport_types: vec!["Ride".to_string(), "Run".to_string()],
                activity_ids: Vec::new(),
                length_meters: polyline_length(&cross_sport_corridor),
            },
        ];

        Self {
            config,
            rng,
            ride_corridor,
            run_corridor,
            cross_sport_corridor,
            corridor_truth,
            next_activity_index: 0,
        }
    }

    fn run(mut self) -> LifecycleCorpus {
        // Day-of-corpus starting points; bucket A is the most recent 90
        // days, so its activities have larger day offsets (closer to "now").
        // We place "now" at day 730 (2 years).
        let now_day: i64 = 730;

        let bucket_a = self.emit_bucket("a", self.config.bucket_a_count, now_day - 90, now_day);
        let bucket_b_delta = self.emit_bucket(
            "b",
            self.config.bucket_b_delta_count,
            now_day - 365,
            now_day - 90,
        );

        // Bucket C: a single activity that overlaps the primary ride
        // corridor — used to assert "single add doesn't perturb other
        // sections" in the harness.
        let bucket_c_single = self.emit_overlap_activity(
            "c",
            now_day + 1,
            "Ride",
            CorridorChoice::RideMain,
            DirectionPick::Forward,
        );

        // Bucket D: 3 activities with mixed character.
        let bucket_d_delta = vec![
            self.emit_overlap_activity(
                "d",
                now_day + 2,
                "Run",
                CorridorChoice::RunMain,
                DirectionPick::Forward,
            ),
            self.emit_overlap_activity(
                "d",
                now_day + 3,
                "Ride",
                CorridorChoice::RideMain,
                DirectionPick::Reverse,
            ),
            self.emit_one_off_activity("d", now_day + 4, "Ride"),
        ];

        let bucket_e_delta = self.emit_bucket(
            "e",
            self.config.bucket_e_delta_count,
            now_day - 730,
            now_day - 365,
        );

        LifecycleCorpus {
            bucket_a,
            bucket_b_delta,
            bucket_c_single,
            bucket_d_delta,
            bucket_e_delta,
            corridors: self.corridor_truth,
        }
    }

    fn emit_bucket(
        &mut self,
        bucket_label: &str,
        count: usize,
        day_start: i64,
        day_end: i64,
    ) -> Vec<LifecycleActivity> {
        let mut activities = Vec::with_capacity(count);
        let span = (day_end - day_start).max(1);

        let parallel_street_quota = if bucket_label == "a" || bucket_label == "e" {
            // Spread parallel-street activities across the largest buckets.
            self.config.parallel_street_count / 2
        } else {
            0
        };

        for i in 0..count {
            let day = day_start + (i as i64 * span / count.max(1) as i64);

            // Deterministic decisions per activity using the RNG.
            let sport_roll: f64 = self.rng.r#gen();
            let sport = if sport_roll < 0.55 { "Ride" } else { "Run" };

            let parallel_remaining = parallel_street_quota.saturating_sub(
                activities
                    .iter()
                    .filter(|a: &&LifecycleActivity| a.id.contains("parallel_street"))
                    .count(),
            );
            let force_parallel = parallel_remaining > 0
                && self
                    .rng
                    .gen_bool((parallel_remaining as f64) / (count - i) as f64);

            let one_off_roll: f64 = self.rng.r#gen();
            let force_one_off = one_off_roll < self.config.one_off_fraction;

            let activity = if force_parallel {
                self.emit_parallel_street_activity(bucket_label, day, sport)
            } else if force_one_off {
                self.emit_one_off_activity(bucket_label, day, sport)
            } else {
                let (corridor, sport_use) = self.pick_corridor(sport);
                let dir = if self.rng.gen_bool(self.config.reverse_fraction) {
                    DirectionPick::Reverse
                } else {
                    DirectionPick::Forward
                };
                self.emit_overlap_activity(bucket_label, day, sport_use, corridor, dir)
            };

            activities.push(activity);
        }

        activities
    }

    fn pick_corridor(&mut self, preferred_sport: &str) -> (CorridorChoice, &'static str) {
        let cross_sport_roll: f64 = self.rng.r#gen();
        if cross_sport_roll < 0.18 {
            // Both sports use the same cross-sport corridor.
            return (
                CorridorChoice::CrossSport,
                if preferred_sport == "Ride" {
                    "Ride"
                } else {
                    "Run"
                },
            );
        }
        if preferred_sport == "Ride" {
            if self.rng.gen_bool(self.config.ride_corridor_overlap) {
                (CorridorChoice::RideMain, "Ride")
            } else {
                (CorridorChoice::None, "Ride")
            }
        } else {
            if self.rng.gen_bool(self.config.run_corridor_overlap) {
                (CorridorChoice::RunMain, "Run")
            } else {
                (CorridorChoice::None, "Run")
            }
        }
    }

    fn emit_overlap_activity(
        &mut self,
        bucket_label: &str,
        day: i64,
        sport: &str,
        corridor: CorridorChoice,
        direction: DirectionPick,
    ) -> LifecycleActivity {
        let id = self.next_id(bucket_label, "overlap");
        let approach_heading: f64 = self.rng.gen_range(0.0..(2.0 * PI));

        let canonical = match corridor {
            CorridorChoice::RideMain => Some(self.ride_corridor.clone()),
            CorridorChoice::RunMain => Some(self.run_corridor.clone()),
            CorridorChoice::CrossSport => Some(self.cross_sport_corridor.clone()),
            CorridorChoice::None => None,
        };

        let mut full = Vec::new();

        if let Some(mut canonical) = canonical {
            if matches!(direction, DirectionPick::Reverse) {
                canonical.reverse();
            }
            // Approach segment heading toward the corridor start.
            let start = canonical[0];
            let approach_start = GpsPoint::with_elevation(
                start.latitude + meters_to_deg_lat(400.0 * approach_heading.sin()),
                start.longitude + meters_to_deg_lng(400.0 * approach_heading.cos(), start.latitude),
                start.elevation.unwrap_or(300.0),
            );
            full.extend(generate_random_segment(
                &approach_start,
                350.0,
                approach_heading + PI,
                &mut self.rng,
            ));
            full.extend(canonical.iter().copied());
            // Departure
            let end = *canonical.last().unwrap();
            let depart_heading: f64 = self.rng.gen_range(0.0..(2.0 * PI));
            full.extend(generate_random_segment(
                &end,
                350.0,
                depart_heading,
                &mut self.rng,
            ));

            self.record_truth(corridor, &id);
        } else {
            // No corridor — random meandering.
            full = generate_random_segment(
                &self.config.origin,
                self.rng.gen_range(2_500.0..6_000.0),
                self.rng.gen_range(0.0..(2.0 * PI)),
                &mut self.rng,
            );
        }

        LifecycleActivity {
            id,
            sport_type: sport.to_string(),
            start_date_unix: EPOCH_BASE_UNIX + day * SECONDS_PER_DAY,
            gps_points: self.apply_noise(&full),
        }
    }

    fn emit_parallel_street_activity(
        &mut self,
        bucket_label: &str,
        day: i64,
        sport: &str,
    ) -> LifecycleActivity {
        let id = self.next_id(bucket_label, "parallel_street");
        // Offset perpendicular to the ride corridor's average heading.
        let primary = &self.ride_corridor;
        let perp_offset = self.config.parallel_street_offset_meters
            * if self.rng.r#gen::<bool>() { 1.0 } else { -1.0 };

        let shifted: Vec<GpsPoint> = primary
            .iter()
            .map(|p| {
                // Heading is approximated from local segment in caller; for a
                // simple offset we shift in latitude only (north/south),
                // which is good enough to produce a near-miss without
                // matching the canonical corridor.
                GpsPoint::with_elevation(
                    p.latitude + meters_to_deg_lat(perp_offset),
                    p.longitude,
                    p.elevation.unwrap_or(300.0), // SAME elevation profile
                )
            })
            .collect();

        LifecycleActivity {
            id,
            sport_type: sport.to_string(),
            start_date_unix: EPOCH_BASE_UNIX + day * SECONDS_PER_DAY,
            gps_points: self.apply_noise(&shifted),
        }
    }

    fn emit_one_off_activity(
        &mut self,
        bucket_label: &str,
        day: i64,
        sport: &str,
    ) -> LifecycleActivity {
        let id = self.next_id(bucket_label, "one_off");
        let length: f64 = self.rng.gen_range(2_500.0..7_500.0);
        let heading: f64 = self.rng.gen_range(0.0..(2.0 * PI));
        // Move the origin a long way away so the random meandering can't
        // accidentally hit any of the canonical corridors.
        let origin_offset_m = 25_000.0;
        let origin = GpsPoint::with_elevation(
            self.config.origin.latitude + meters_to_deg_lat(origin_offset_m * heading.sin()),
            self.config.origin.longitude
                + meters_to_deg_lng(origin_offset_m * heading.cos(), self.config.origin.latitude),
            self.config.origin.elevation.unwrap_or(300.0),
        );
        let raw = generate_random_segment(&origin, length, heading, &mut self.rng);

        LifecycleActivity {
            id,
            sport_type: sport.to_string(),
            start_date_unix: EPOCH_BASE_UNIX + day * SECONDS_PER_DAY,
            gps_points: self.apply_noise(&raw),
        }
    }

    fn next_id(&mut self, bucket_label: &str, kind: &str) -> String {
        let i = self.next_activity_index;
        self.next_activity_index += 1;
        format!("life_{bucket_label}_{kind}_{i:05}")
    }

    fn record_truth(&mut self, corridor: CorridorChoice, activity_id: &str) {
        let idx = match corridor {
            CorridorChoice::RideMain => 0,
            CorridorChoice::RunMain => 1,
            CorridorChoice::CrossSport => 2,
            CorridorChoice::None => return,
        };
        self.corridor_truth[idx]
            .activity_ids
            .push(activity_id.to_string());
    }

    fn apply_noise(&mut self, points: &[GpsPoint]) -> Vec<GpsPoint> {
        // Each activity gets its own noise instance, deterministic from the
        // configured seed plus the activity index. This matters: if we
        // shared one Noise instance, AR(1) state would leak across
        // activities.
        let seed = self
            .config
            .seed
            .wrapping_add((self.next_activity_index as u64).wrapping_mul(0x9E3779B97F4A7C15));
        let mut noise = GaussMarkovNoise::new(self.config.gps_noise, seed);
        noise.reset_for_new_activity();
        noise.perturb_track(points)
    }
}

#[derive(Clone, Copy, Debug)]
enum CorridorChoice {
    RideMain,
    RunMain,
    CrossSport,
    None,
}

#[derive(Clone, Copy, Debug)]
enum DirectionPick {
    Forward,
    Reverse,
}

// ============================================================================
// Polyline generators (kept separate from synthetic.rs so we can evolve them
// independently — e.g. add elevation profiles or street grids — without
// disturbing the existing benches).
// ============================================================================

fn heading_radians(corridor_index: usize) -> f64 {
    (corridor_index as f64) * (2.0 * PI / 8.0) + PI / 6.0
}

fn offset_meters(corridor_index: usize) -> f64 {
    250.0 * (corridor_index as f64 + 1.0)
}

fn generate_winding_corridor(
    origin: &GpsPoint,
    length_meters: f64,
    base_heading: f64,
    offset_from_origin_m: f64,
    rng: &mut StdRng,
) -> Vec<GpsPoint> {
    let num_points = (length_meters / POINT_SPACING_M).ceil() as usize;
    let mut points = Vec::with_capacity(num_points + 1);

    let mut heading = base_heading;
    let mut current = GpsPoint::with_elevation(
        origin.latitude + meters_to_deg_lat(offset_from_origin_m * base_heading.sin()),
        origin.longitude
            + meters_to_deg_lng(offset_from_origin_m * base_heading.cos(), origin.latitude),
        origin.elevation.unwrap_or(300.0),
    );
    points.push(current);

    for i in 0..num_points {
        let base_turn = (i as f64 * 0.01).sin() * 0.3;
        let random_turn: f64 = rng.gen_range(-0.15..0.15);
        heading += base_turn + random_turn;

        let dlat = meters_to_deg_lat(POINT_SPACING_M * heading.sin());
        let dlng = meters_to_deg_lng(POINT_SPACING_M * heading.cos(), current.latitude);
        let elev = origin.elevation.unwrap_or(300.0)
            + 50.0 * (i as f64 * 0.005).sin()
            + 12.0 * (i as f64 * 0.04).sin();

        current = GpsPoint::with_elevation(current.latitude + dlat, current.longitude + dlng, elev);
        points.push(current);
    }

    points
}

fn generate_random_segment(
    start: &GpsPoint,
    length_meters: f64,
    initial_heading: f64,
    rng: &mut StdRng,
) -> Vec<GpsPoint> {
    let num_points = (length_meters / POINT_SPACING_M).ceil() as usize;
    if num_points == 0 {
        return vec![*start];
    }

    let mut points = Vec::with_capacity(num_points + 1);
    let mut heading = initial_heading;
    let mut current = *start;
    points.push(current);

    for i in 0..num_points {
        heading += rng.gen_range(-0.3..0.3);
        let dlat = meters_to_deg_lat(POINT_SPACING_M * heading.sin());
        let dlng = meters_to_deg_lng(POINT_SPACING_M * heading.cos(), current.latitude);
        let elev = current.elevation.unwrap_or(300.0) + 30.0 * (i as f64 * 0.01).sin();
        current = GpsPoint::with_elevation(current.latitude + dlat, current.longitude + dlng, elev);
        points.push(current);
    }

    points
}

fn polyline_length(points: &[GpsPoint]) -> f64 {
    if points.len() < 2 {
        return 0.0;
    }
    points
        .windows(2)
        .map(|w| {
            let dlat = (w[1].latitude - w[0].latitude) * METERS_PER_DEG_LAT;
            let avg_lat = (w[0].latitude + w[1].latitude) / 2.0;
            let dlng =
                (w[1].longitude - w[0].longitude) * METERS_PER_DEG_LAT * avg_lat.to_radians().cos();
            (dlat * dlat + dlng * dlng).sqrt()
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn small_corpus() -> LifecycleCorpus {
        let config = LifecycleConfig {
            bucket_a_count: 10,
            bucket_b_delta_count: 8,
            bucket_d_delta_count: 3,
            bucket_e_delta_count: 5,
            parallel_street_count: 2,
            ..LifecycleConfig::default()
        };
        LifecycleCorpus::generate(&config)
    }

    #[test]
    fn corpus_partitions_have_expected_counts() {
        let corpus = small_corpus();
        assert_eq!(corpus.bucket_a.len(), 10);
        assert_eq!(corpus.bucket_b_delta.len(), 8);
        assert_eq!(corpus.bucket_d_delta.len(), 3);
        assert_eq!(corpus.bucket_e_delta.len(), 5);
        // through_e should sum all of them plus the single C activity.
        assert_eq!(corpus.through_e().len(), 10 + 8 + 1 + 3 + 5);
    }

    #[test]
    fn deterministic_for_same_seed() {
        let cfg = LifecycleConfig {
            bucket_a_count: 5,
            bucket_b_delta_count: 5,
            bucket_d_delta_count: 3,
            bucket_e_delta_count: 5,
            parallel_street_count: 0,
            ..LifecycleConfig::default()
        };
        let a = LifecycleCorpus::generate(&cfg);
        let b = LifecycleCorpus::generate(&cfg);
        assert_eq!(a.bucket_a.len(), b.bucket_a.len());
        for (pa, pb) in a.through_e().iter().zip(b.through_e().iter()) {
            assert_eq!(pa.id, pb.id);
            assert_eq!(pa.start_date_unix, pb.start_date_unix);
            assert_eq!(pa.gps_points.len(), pb.gps_points.len());
            if !pa.gps_points.is_empty() {
                assert_eq!(pa.gps_points[0].latitude, pb.gps_points[0].latitude);
            }
        }
    }

    #[test]
    fn dates_are_monotone_within_buckets() {
        let corpus = small_corpus();
        for window in corpus.bucket_a.windows(2) {
            assert!(window[0].start_date_unix <= window[1].start_date_unix);
        }
        for window in corpus.bucket_e_delta.windows(2) {
            assert!(window[0].start_date_unix <= window[1].start_date_unix);
        }
    }

    #[test]
    fn parallel_street_activities_have_recognisable_ids() {
        let cfg = LifecycleConfig {
            bucket_a_count: 30,
            bucket_b_delta_count: 0,
            bucket_d_delta_count: 0,
            bucket_e_delta_count: 0,
            parallel_street_count: 6,
            ..LifecycleConfig::default()
        };
        let corpus = LifecycleCorpus::generate(&cfg);
        let parallel: Vec<_> = corpus
            .bucket_a
            .iter()
            .filter(|a| a.id.contains("parallel_street"))
            .collect();
        // Half the parallel quota goes into bucket A (the rest goes into E).
        assert!(
            !parallel.is_empty(),
            "expected some parallel-street activities in A"
        );
    }

    #[test]
    fn cross_sport_corridor_visited_by_both_sports() {
        // Force enough activities that the 18% cross-sport roll fires for at
        // least one Ride and one Run.
        let cfg = LifecycleConfig {
            bucket_a_count: 200,
            bucket_b_delta_count: 0,
            bucket_d_delta_count: 0,
            bucket_e_delta_count: 0,
            parallel_street_count: 0,
            ..LifecycleConfig::default()
        };
        let corpus = LifecycleCorpus::generate(&cfg);
        let cross = &corpus.corridors[2];
        assert_eq!(cross.id, "cross_sport");

        // Build sport map for lookup.
        let sport_map: HashMap<&str, &str> = corpus
            .bucket_a
            .iter()
            .map(|a| (a.id.as_str(), a.sport_type.as_str()))
            .collect();
        let sports: std::collections::HashSet<&str> = cross
            .activity_ids
            .iter()
            .filter_map(|id| sport_map.get(id.as_str()).copied())
            .collect();
        assert!(
            sports.contains("Ride") && sports.contains("Run"),
            "expected cross-sport corridor to attract both sports, got {:?}",
            sports
        );
    }

    #[test]
    fn through_b_is_strict_prefix_of_through_e() {
        let corpus = small_corpus();
        let b_ids: Vec<_> = corpus.through_b().iter().map(|a| a.id.clone()).collect();
        let e_ids: Vec<_> = corpus.through_e().iter().map(|a| a.id.clone()).collect();
        for (i, id) in b_ids.iter().enumerate() {
            assert_eq!(&e_ids[i], id, "ordering diverged at index {i}");
        }
        assert!(e_ids.len() > b_ids.len());
    }
}
