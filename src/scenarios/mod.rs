//! Lifecycle-style scenarios for end-to-end testing.
//!
//! Builds on top of [`crate::synthetic`] with extra structure needed by the
//! veloqrs persistence harness:
//! - per-activity sport assignment
//! - per-activity start dates
//! - forward / reverse traversal
//! - parallel-street near-misses (false-match avoidance)
//! - first-order Gauss–Markov GPS noise
//!
//! Feature-gated behind `synthetic` (same gate as the underlying generator).

pub mod lifecycle;
pub mod noise;

pub use lifecycle::{
    CorridorTruth, LifecycleActivity, LifecycleConfig, LifecycleCorpus,
};
pub use noise::{GaussMarkovConfig, GaussMarkovNoise};
