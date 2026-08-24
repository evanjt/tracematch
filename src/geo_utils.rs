//! # Geographic Utilities
//!
//! Core geographic computation utilities for GPS track analysis.
//!
//! This module provides fundamental geographic operations used throughout the route matching
//! library. All functions are designed to be efficient and accurate for GPS trajectory data.
//!
//! ## Overview
//!
//! | Function | Description |
//! |----------|-------------|
//! | [`haversine_distance`] | Great-circle distance between two GPS points |
//! | [`compute_bounds`] | Bounding box of a GPS track |
//! | [`compute_center`] | Centroid of a GPS track |
//!
//! ## Example
//!
//! ```rust
//! use tracematch::{GpsPoint, geo_utils};
//!
//! let track = vec![
//!     GpsPoint::new(51.5074, -0.1278),  // London
//!     GpsPoint::new(51.5080, -0.1290),
//!     GpsPoint::new(51.5090, -0.1300),
//! ];
//!
//! // Get bounding box
//! let bounds = geo_utils::compute_bounds(&track);
//! println!("Bounds: {:.4}N to {:.4}N", bounds.min_lat, bounds.max_lat);
//!
//! // Distance between two points
//! let dist = geo_utils::haversine_distance(&track[0], &track[2]);
//! println!("Start to end: {:.0}m", dist);
//! ```
//!
//! ## Algorithm Notes
//!
//! ### Haversine Formula
//!
//! The haversine formula calculates the great-circle distance between two points on a sphere.
//! It's the standard method for GPS distance calculation, accurate to within 0.3% for most
//! practical applications.
//!
//! Reference: [Haversine formula (Wikipedia)](https://en.wikipedia.org/wiki/Haversine_formula)
//!
//! ### Coordinate System
//!
//! All functions expect WGS84 coordinates (latitude/longitude in degrees), which is the
//! standard used by GPS receivers and mapping services.

use crate::{Bounds, GpsPoint};
use geo::{Bearing, Distance, Haversine, Point};

// =============================================================================
// Distance Functions
// =============================================================================

/// Calculate the great-circle distance between two GPS points using the Haversine formula.
///
/// Returns the distance in meters along the Earth's surface (assuming a spherical Earth
/// with radius 6,371 km).
///
/// # Arguments
///
/// * `p1` - First GPS point
/// * `p2` - Second GPS point
///
/// # Returns
///
/// Distance in meters between the two points.
///
/// # Example
///
/// ```rust
/// use tracematch::{GpsPoint, geo_utils};
///
/// let london = GpsPoint::new(51.5074, -0.1278);
/// let paris = GpsPoint::new(48.8566, 2.3522);
///
/// let distance = geo_utils::haversine_distance(&london, &paris);
/// assert!((distance - 343_560.0).abs() < 1000.0); // ~344 km
/// ```
///
/// # Performance
///
/// This function is O(1) and involves trigonometric operations. For comparing distances
/// where exact values aren't needed, consider using squared Euclidean distance on
/// projected coordinates for better performance.
#[inline]
pub fn haversine_distance(p1: &GpsPoint, p2: &GpsPoint) -> f64 {
    let point1 = Point::new(p1.longitude, p1.latitude);
    let point2 = Point::new(p2.longitude, p2.latitude);
    Haversine::distance(point1, point2)
}

// =============================================================================
// Bounding Box Functions
// =============================================================================

/// Compute the bounding box of a GPS track.
///
/// Returns a [`Bounds`] struct containing the minimum and maximum latitude/longitude
/// values that enclose all points in the track.
///
/// # Arguments
///
/// * `points` - Slice of GPS points
///
/// # Returns
///
/// A [`Bounds`] struct with the bounding box coordinates. For empty input,
/// returns a bounds with MIN/MAX values that will fail any overlap check.
///
/// # Example
///
/// ```rust
/// use tracematch::{GpsPoint, geo_utils};
///
/// let track = vec![
///     GpsPoint::new(51.5000, -0.1300),
///     GpsPoint::new(51.5100, -0.1200),
///     GpsPoint::new(51.5050, -0.1250),
/// ];
///
/// let bounds = geo_utils::compute_bounds(&track);
/// assert_eq!(bounds.min_lat, 51.5000);
/// assert_eq!(bounds.max_lat, 51.5100);
/// assert_eq!(bounds.min_lng, -0.1300);
/// assert_eq!(bounds.max_lng, -0.1200);
/// ```
pub fn compute_bounds(points: &[GpsPoint]) -> Bounds {
    Bounds::from_points(points).unwrap_or(Bounds {
        min_lat: f64::MAX,
        max_lat: f64::MIN,
        min_lng: f64::MAX,
        max_lng: f64::MIN,
    })
}

// =============================================================================
// Center/Centroid Functions
// =============================================================================

/// Compute the geographic center (centroid) of a GPS track.
///
/// Returns the arithmetic mean of all latitude and longitude values.
/// This is a simple centroid calculation suitable for small geographic areas.
///
/// # Arguments
///
/// * `points` - Slice of GPS points
///
/// # Returns
///
/// A [`GpsPoint`] at the center of the track. Returns (0, 0) for empty input.
///
/// # Notes
///
/// For tracks spanning large areas or crossing the antimeridian (180°/-180° longitude),
/// this simple averaging may produce unexpected results. For such cases, consider
/// using a proper spherical centroid calculation.
///
/// # Example
///
/// ```rust
/// use tracematch::{GpsPoint, geo_utils};
///
/// let track = vec![
///     GpsPoint::new(51.50, -0.10),
///     GpsPoint::new(51.52, -0.12),
/// ];
///
/// let center = geo_utils::compute_center(&track);
/// assert!((center.latitude - 51.51).abs() < 0.001);
/// assert!((center.longitude - (-0.11)).abs() < 0.001);
/// ```
pub fn compute_center(points: &[GpsPoint]) -> GpsPoint {
    if points.is_empty() {
        return GpsPoint::new(0.0, 0.0);
    }

    let sum_lat: f64 = points.iter().map(|p| p.latitude).sum();
    let sum_lng: f64 = points.iter().map(|p| p.longitude).sum();
    let n = points.len() as f64;

    GpsPoint::new(sum_lat / n, sum_lng / n)
}

// =============================================================================
// Bearing and Gradient Utilities
// =============================================================================

/// Calculate the initial bearing (forward azimuth) from p1 to p2.
///
/// Returns bearing in degrees (0-360), where:
/// - 0° = North
/// - 90° = East
/// - 180° = South
/// - 270° = West
///
/// Uses the spherical law of cosines for azimuth calculation.
#[inline]
pub fn calculate_bearing(p1: &GpsPoint, p2: &GpsPoint) -> f64 {
    let point1 = Point::new(p1.longitude, p1.latitude);
    let point2 = Point::new(p2.longitude, p2.latitude);
    let bearing = Haversine::bearing(point1, point2);
    // geo returns [-180, 180], normalize to [0, 360)
    (bearing + 360.0) % 360.0
}

/// Calculate the absolute angular difference between two bearings.
///
/// Returns a value in the range [0, 180] degrees, handling the wraparound at 360°.
#[inline]
pub fn bearing_difference(b1: f64, b2: f64) -> f64 {
    let diff = (b1 - b2).abs();
    if diff > 180.0 { 360.0 - diff } else { diff }
}

/// Calculate the circular mean of a set of bearings.
///
/// Handles the circular nature of bearings (e.g., average of 350° and 10° should be 0°).
/// Returns a bearing in the range [0, 360).
pub fn circular_mean_bearing(bearings: &[f64]) -> f64 {
    if bearings.is_empty() {
        return 0.0;
    }

    let sum_sin: f64 = bearings.iter().map(|b| b.to_radians().sin()).sum();
    let sum_cos: f64 = bearings.iter().map(|b| b.to_radians().cos()).sum();

    (sum_sin.atan2(sum_cos).to_degrees() + 360.0) % 360.0
}

/// Calculate the circular standard deviation of bearings.
///
/// Measures how spread out the bearings are. A value near 0 indicates
/// consistent direction; higher values indicate more variation.
pub fn circular_std_bearing(bearings: &[f64]) -> f64 {
    if bearings.len() < 2 {
        return 0.0;
    }

    let mean = circular_mean_bearing(bearings);
    let sum_sq: f64 = bearings
        .iter()
        .map(|b| bearing_difference(*b, mean).powi(2))
        .sum();

    (sum_sq / bearings.len() as f64).sqrt()
}

/// Calculate gradient (grade %) between two points.
///
/// Returns `None` if either point lacks elevation data.
/// Positive values indicate uphill (ascending), negative indicates downhill.
///
/// Formula: gradient = (elevation_change / horizontal_distance) * 100
pub fn calculate_gradient(p1: &GpsPoint, p2: &GpsPoint) -> Option<f64> {
    let elev1 = p1.elevation?;
    let elev2 = p2.elevation?;

    let horizontal_dist = haversine_distance(p1, p2);
    if horizontal_dist < 1.0 {
        // Too close - avoid division by very small numbers
        return Some(0.0);
    }

    let elevation_change = elev2 - elev1;
    Some((elevation_change / horizontal_dist) * 100.0)
}

/// Compute average gradient over a segment of points.
///
/// Returns `None` if insufficient elevation data is available.
pub fn segment_gradient(points: &[GpsPoint]) -> Option<f64> {
    if points.len() < 2 {
        return None;
    }

    let first = points.first()?;
    let last = points.last()?;

    let elev_start = first.elevation?;
    let elev_end = last.elevation?;

    // Calculate total horizontal distance
    let mut total_dist = 0.0;
    for i in 1..points.len() {
        total_dist += haversine_distance(&points[i - 1], &points[i]);
    }

    if total_dist < 1.0 {
        return Some(0.0);
    }

    Some(((elev_end - elev_start) / total_dist) * 100.0)
}

/// A climb must clear this before it counts towards elevation gain,
/// filtering barometric and GPS noise between real rises.
pub const ELEVATION_GAIN_HYSTERESIS_M: f64 = 3.0;

/// Elevation gain (m) and net grade (%) of one real track slice.
///
/// Returns `None` unless at least 90% of the points carry elevation.
/// Elevations are conditioned by a 3-point mean, gain accumulates only
/// when a rise clears [`ELEVATION_GAIN_HYSTERESIS_M`] above the last
/// anchor, and grade is the net elevation change over the track's
/// horizontal distance (the [`segment_gradient`] formula).
pub fn elevation_stats(points: &[GpsPoint]) -> Option<(f64, f64)> {
    if points.len() < 2 {
        return None;
    }
    let carrying = points.iter().filter(|p| p.elevation.is_some()).count();
    if (carrying as f64) < 0.9 * points.len() as f64 {
        return None;
    }

    let elevs: Vec<f64> = points.iter().filter_map(|p| p.elevation).collect();
    let n = elevs.len();
    let smoothed: Vec<f64> = (0..n)
        .map(|i| {
            let lo = i.saturating_sub(1);
            let hi = (i + 1).min(n - 1);
            elevs[lo..=hi].iter().sum::<f64>() / (hi - lo + 1) as f64
        })
        .collect();

    let mut gain = 0.0;
    let mut anchor = smoothed[0];
    for &e in &smoothed[1..] {
        let delta = e - anchor;
        if delta >= ELEVATION_GAIN_HYSTERESIS_M {
            gain += delta;
            anchor = e;
        } else if delta <= -ELEVATION_GAIN_HYSTERESIS_M {
            anchor = e;
        }
    }

    let mut total_dist = 0.0;
    for w in points.windows(2) {
        total_dist += haversine_distance(&w[0], &w[1]);
    }
    if total_dist < 1.0 {
        return Some((gain, 0.0));
    }
    let first = points.iter().find_map(|p| p.elevation)?;
    let last = points.iter().rev().find_map(|p| p.elevation)?;
    let grade = ((last - first) / total_dist) * 100.0;

    Some((gain, grade))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn track(elevations: &[Option<f64>]) -> Vec<GpsPoint> {
        // Points ~11 m apart along a meridian at synthetic coordinates.
        elevations
            .iter()
            .enumerate()
            .map(|(i, e)| GpsPoint {
                latitude: 47.0 + i as f64 * 0.0001,
                longitude: 8.0,
                elevation: *e,
            })
            .collect()
    }

    #[test]
    fn flat_track_reports_zero_gain_and_grade() {
        let pts = track(&[Some(500.0); 20]);
        let (gain, grade) = elevation_stats(&pts).unwrap();
        assert_eq!(gain, 0.0);
        assert_eq!(grade, 0.0);
    }

    #[test]
    fn single_climb_reports_rise_and_positive_grade() {
        let elevs: Vec<Option<f64>> = (0..21).map(|i| Some(400.0 + i as f64 * 5.0)).collect();
        let pts = track(&elevs);
        let (gain, grade) = elevation_stats(&pts).unwrap();
        // Smoothing halves the first and last steps; the middle rise survives.
        assert!((gain - 95.0).abs() < 5.0, "gain {gain}");
        assert!(grade > 4.0, "grade {grade}");
    }

    #[test]
    fn hysteresis_ignores_noise_on_a_climb() {
        // A steady 50 m rise carrying +-1 m sawtooth noise: hysteresis keeps
        // the oscillations from inflating the gain past the true rise.
        let elevs: Vec<Option<f64>> = (0..51)
            .map(|i| {
                let noise = if i % 2 == 0 { 1.0 } else { -1.0 };
                Some(300.0 + i as f64 + noise)
            })
            .collect();
        let pts = track(&elevs);
        let (gain, _) = elevation_stats(&pts).unwrap();
        assert!((40.0..=55.0).contains(&gain), "gain {gain}");
    }

    #[test]
    fn descent_reports_zero_gain_and_negative_grade() {
        let elevs: Vec<Option<f64>> = (0..21).map(|i| Some(600.0 - i as f64 * 5.0)).collect();
        let pts = track(&elevs);
        let (gain, grade) = elevation_stats(&pts).unwrap();
        assert_eq!(gain, 0.0);
        assert!(grade < -4.0, "grade {grade}");
    }

    #[test]
    fn sparse_elevation_below_floor_is_none() {
        let elevs: Vec<Option<f64>> = (0..20)
            .map(|i| if i % 2 == 0 { Some(500.0) } else { None })
            .collect();
        assert!(elevation_stats(&track(&elevs)).is_none());
    }

    #[test]
    fn ninety_percent_coverage_passes_the_floor() {
        let mut elevs: Vec<Option<f64>> = (0..20).map(|i| Some(500.0 + i as f64)).collect();
        elevs[3] = None;
        elevs[11] = None;
        assert!(elevation_stats(&track(&elevs)).is_some());
    }

    #[test]
    fn short_track_is_none() {
        assert!(elevation_stats(&track(&[Some(500.0)])).is_none());
        assert!(elevation_stats(&[]).is_none());
    }
}
