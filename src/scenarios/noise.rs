//! GPS noise models for synthetic scenario generation.
//!
//! Real GPS error is **not** independent-and-identically-distributed Gaussian
//! noise. Empirical studies of consumer GPS receivers show significant
//! temporal autocorrelation, dominated by multipath, ionospheric delay, and
//! receiver-clock drift. Standard models in the literature:
//!
//! - **White Gaussian** — analytically tractable, used in basic Kalman-filter
//!   derivations. Significantly understates the visual "wandering" seen in
//!   real tracks because successive samples are independent.
//! - **First-order Gauss–Markov / AR(1)** — single time constant `τ`,
//!   exponentially decaying autocorrelation. Standard model for multipath in
//!   GNSS literature; captures the "drift" character of real GPS error in
//!   urban / forested environments.
//! - **Coloured noise (flicker, random-walk)** — more accurate at long
//!   horizons but overkill for our purposes (we care about meter-scale noise
//!   over minutes, not the geodetic-network-scale noise studied in the
//!   geodesy literature).
//!
//! For lifecycle scenarios we use a first-order Gauss–Markov process applied
//! independently to the latitude and longitude axes. The recurrence is:
//!
//! ```text
//! x_t = φ · x_{t-1} + √(1 - φ²) · σ · w_t,   w_t ~ N(0, 1)
//! φ   = exp(-Δt / τ)
//! ```
//!
//! `σ` is the steady-state position-error standard deviation in metres. `τ`
//! is the autocorrelation time constant in seconds. Typical values for a
//! consumer fitness-tracker GPS in mixed urban/open conditions:
//! - σ ≈ 3–5 m (open sky), 8–15 m (urban canyon)
//! - τ ≈ 20–60 s (multipath-dominated)
//!
//! References (research informing this model — not cited in code, kept here
//! for traceability):
//! - Amiri-Simkooei (2007), *J. Geophys. Res.* — empirical GPS noise spectra,
//!   shows white + flicker dominates real coordinate time series.
//! - Khider et al. (2013), *IET Radar, Sonar & Navigation* — multipath as a
//!   first-order Gauss–Markov process for pseudorange-based positioning.
//! - "Overbounding the effect of uncertain Gauss–Markov noise in Kalman
//!   filtering" (NAVIGATION, 2021) — bounds and time-constant estimation.

use crate::GpsPoint;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::f64::consts::PI;

const METERS_PER_DEG_LAT: f64 = 111_320.0;

fn meters_to_deg_lat(meters: f64) -> f64 {
    meters / METERS_PER_DEG_LAT
}

fn meters_to_deg_lng(meters: f64, latitude: f64) -> f64 {
    let meters_per_deg_lng = METERS_PER_DEG_LAT * latitude.to_radians().cos();
    if meters_per_deg_lng.abs() < 1e-10 {
        return 0.0;
    }
    meters / meters_per_deg_lng
}

/// Box–Muller transform: two independent N(0, 1) samples from two U(0, 1).
fn standard_normal_pair(rng: &mut StdRng) -> (f64, f64) {
    let u1: f64 = rng.gen_range(1e-12..1.0);
    let u2: f64 = rng.r#gen();
    let r = (-2.0 * u1.ln()).sqrt();
    let theta = 2.0 * PI * u2;
    (r * theta.cos(), r * theta.sin())
}

/// Configuration for a first-order Gauss–Markov GPS noise process.
#[derive(Debug, Clone, Copy)]
pub struct GaussMarkovConfig {
    /// Steady-state position-error standard deviation, metres.
    pub sigma_meters: f64,
    /// Autocorrelation time constant, seconds. Larger τ → smoother drift.
    pub time_constant_seconds: f64,
    /// Effective sample interval between successive GPS points, seconds.
    /// At the synthetic generator's POINT_SPACING of 10 m and a typical
    /// cycling pace of 5 m/s, Δt ≈ 2 s.
    pub sample_interval_seconds: f64,
}

impl Default for GaussMarkovConfig {
    fn default() -> Self {
        // Conservative open-sky GPS profile.
        Self {
            sigma_meters: 4.0,
            time_constant_seconds: 30.0,
            sample_interval_seconds: 2.0,
        }
    }
}

impl GaussMarkovConfig {
    /// φ = exp(-Δt / τ). Higher φ → more autocorrelation. φ → 1 means random
    /// walk; φ → 0 means white noise.
    fn phi(&self) -> f64 {
        (-self.sample_interval_seconds / self.time_constant_seconds.max(1e-6)).exp()
    }
}

/// Stateful first-order Gauss–Markov noise generator.
///
/// Maintains the AR(1) state across successive `perturb_track` calls only when
/// you explicitly call `reset_for_new_activity`. Two activities sampled
/// without reset will share noise correlation across the boundary, which is
/// not what we want — always reset between activities.
pub struct GaussMarkovNoise {
    config: GaussMarkovConfig,
    rng: StdRng,
    state_north_m: f64,
    state_east_m: f64,
    state_elev_m: f64,
}

impl GaussMarkovNoise {
    pub fn new(config: GaussMarkovConfig, seed: u64) -> Self {
        Self {
            config,
            rng: StdRng::seed_from_u64(seed),
            state_north_m: 0.0,
            state_east_m: 0.0,
            state_elev_m: 0.0,
        }
    }

    /// Reset the AR(1) state so the next activity's noise is uncorrelated
    /// with the previous one. Call between successive `perturb_track` runs.
    pub fn reset_for_new_activity(&mut self) {
        // Initialise the state from the steady-state distribution so the
        // first sample is already realistic, not biased toward zero.
        let (z_n, z_e) = standard_normal_pair(&mut self.rng);
        let (z_v, _) = standard_normal_pair(&mut self.rng);
        self.state_north_m = self.config.sigma_meters * z_n;
        self.state_east_m = self.config.sigma_meters * z_e;
        self.state_elev_m = (self.config.sigma_meters * 0.5) * z_v;
    }

    /// Apply correlated noise to one track. The track's points are perturbed
    /// in-place coordinate space — the geometry follows the original
    /// polyline plus the AR(1) wander.
    pub fn perturb_track(&mut self, points: &[GpsPoint]) -> Vec<GpsPoint> {
        if self.config.sigma_meters <= 0.0 {
            return points.to_vec();
        }

        let phi = self.config.phi();
        let innovation_scale = self.config.sigma_meters * (1.0 - phi * phi).sqrt();
        // Vertical noise is typically ~1.5–2× horizontal. We use 0.5× because
        // most consumer GPS reports barometric-corrected elevation in this
        // app's input, which is more stable than raw GPS altitude.
        let elev_innovation_scale = innovation_scale * 0.5;

        let mut out = Vec::with_capacity(points.len());

        for p in points {
            let (w_n, w_e) = standard_normal_pair(&mut self.rng);
            let (w_v, _) = standard_normal_pair(&mut self.rng);

            self.state_north_m = phi * self.state_north_m + innovation_scale * w_n;
            self.state_east_m = phi * self.state_east_m + innovation_scale * w_e;
            self.state_elev_m = phi * self.state_elev_m + elev_innovation_scale * w_v;

            let lat = p.latitude + meters_to_deg_lat(self.state_north_m);
            let lng = p.longitude + meters_to_deg_lng(self.state_east_m, p.latitude);
            let elev = p.elevation.unwrap_or(0.0) + self.state_elev_m;

            out.push(GpsPoint::with_elevation(lat, lng, elev));
        }

        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight_track(n: usize) -> Vec<GpsPoint> {
        (0..n)
            .map(|i| GpsPoint::with_elevation(47.37, 8.55 + (i as f64) * 1e-4, 300.0))
            .collect()
    }

    #[test]
    fn perturbed_track_matches_input_length() {
        let mut noise = GaussMarkovNoise::new(GaussMarkovConfig::default(), 1);
        noise.reset_for_new_activity();
        let track = straight_track(100);
        let out = noise.perturb_track(&track);
        assert_eq!(out.len(), 100);
    }

    #[test]
    fn deterministic_with_seed() {
        let cfg = GaussMarkovConfig::default();
        let track = straight_track(50);

        let mut a = GaussMarkovNoise::new(cfg, 42);
        a.reset_for_new_activity();
        let out_a = a.perturb_track(&track);

        let mut b = GaussMarkovNoise::new(cfg, 42);
        b.reset_for_new_activity();
        let out_b = b.perturb_track(&track);

        for (pa, pb) in out_a.iter().zip(out_b.iter()) {
            assert_eq!(pa.latitude, pb.latitude);
            assert_eq!(pa.longitude, pb.longitude);
        }
    }

    #[test]
    fn zero_sigma_returns_input_unchanged() {
        let cfg = GaussMarkovConfig {
            sigma_meters: 0.0,
            ..GaussMarkovConfig::default()
        };
        let mut noise = GaussMarkovNoise::new(cfg, 1);
        noise.reset_for_new_activity();
        let track = straight_track(20);
        let out = noise.perturb_track(&track);
        for (p, q) in track.iter().zip(out.iter()) {
            assert_eq!(p.latitude, q.latitude);
            assert_eq!(p.longitude, q.longitude);
        }
    }

    #[test]
    fn standard_deviation_close_to_sigma() {
        // Many independent activity runs; per-run mean position error stdev
        // should approach configured sigma.
        let cfg = GaussMarkovConfig {
            sigma_meters: 5.0,
            time_constant_seconds: 30.0,
            sample_interval_seconds: 2.0,
        };
        let track = straight_track(2000);

        let mut noise = GaussMarkovNoise::new(cfg, 7);
        let mut north_offsets: Vec<f64> = Vec::with_capacity(2000);

        // Burn in to reach steady state, then collect.
        for run in 0..40 {
            noise.reset_for_new_activity();
            let out = noise.perturb_track(&track);
            for (p, q) in track.iter().zip(out.iter()) {
                let dlat = q.latitude - p.latitude;
                let north_m = dlat * METERS_PER_DEG_LAT;
                if run >= 5 {
                    north_offsets.push(north_m);
                }
            }
        }

        let mean = north_offsets.iter().sum::<f64>() / north_offsets.len() as f64;
        let var = north_offsets
            .iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f64>()
            / north_offsets.len() as f64;
        let stdev = var.sqrt();

        // Allow ±15% — AR(1) steady-state stdev should equal sigma exactly,
        // sample-size noise is the only source of error.
        assert!(
            (stdev - 5.0).abs() / 5.0 < 0.15,
            "expected stdev ~5m, got {stdev}m"
        );
    }

    #[test]
    fn ar1_is_more_correlated_than_iid() {
        // Successive points in an AR(1) walk are correlated; in IID Gaussian
        // they are not. Compare lag-1 autocorrelation.
        let cfg = GaussMarkovConfig {
            sigma_meters: 5.0,
            time_constant_seconds: 30.0,
            sample_interval_seconds: 1.0,
        };
        let track = straight_track(5000);

        let mut noise = GaussMarkovNoise::new(cfg, 11);
        noise.reset_for_new_activity();
        let out = noise.perturb_track(&track);

        // Per-point northward offset.
        let offsets: Vec<f64> = track
            .iter()
            .zip(out.iter())
            .map(|(p, q)| (q.latitude - p.latitude) * METERS_PER_DEG_LAT)
            .collect();

        let mean = offsets.iter().sum::<f64>() / offsets.len() as f64;
        let var: f64 =
            offsets.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / offsets.len() as f64;
        let cov_lag1: f64 = offsets
            .windows(2)
            .map(|w| (w[0] - mean) * (w[1] - mean))
            .sum::<f64>()
            / (offsets.len() - 1) as f64;
        let acf_lag1 = cov_lag1 / var;

        // φ = exp(-1/30) ≈ 0.967, so lag-1 autocorrelation should be high.
        // IID Gaussian would give acf ≈ 0.
        assert!(
            acf_lag1 > 0.7,
            "expected lag-1 autocorrelation > 0.7 for AR(1), got {acf_lag1}"
        );
    }

    #[test]
    fn phi_zero_when_time_constant_tiny() {
        let cfg = GaussMarkovConfig {
            sigma_meters: 5.0,
            time_constant_seconds: 1e-9,
            sample_interval_seconds: 1.0,
        };
        // exp(-1/1e-9) → 0, recovers white noise behaviour.
        assert!(cfg.phi() < 1e-6);
    }
}
