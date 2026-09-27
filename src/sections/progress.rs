//! Progress callback for section detection phases.
//!
//! No detector in tracematch calls `on_phase` or `on_progress` today, so these
//! types are a contract with no emitter: the Unified detector reports no
//! phases through them. Implementations must be `Send + Sync`.

use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};

/// The phases a detector would report through the callback.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectionPhase {
    /// Building spatial indices
    BuildingRtrees,
    /// Finding overlaps between tracks
    FindingOverlaps,
    /// Everything after the overlaps are found
    Postprocessing,
}

impl DetectionPhase {
    /// The phase name in snake case.
    pub fn as_str(&self) -> &'static str {
        match self {
            DetectionPhase::BuildingRtrees => "building_rtrees",
            DetectionPhase::FindingOverlaps => "finding_overlaps",
            DetectionPhase::Postprocessing => "postprocessing",
        }
    }
}

/// Trait for receiving progress updates during section detection.
///
/// Implementations must be thread-safe, because a detector that reported through
/// it would do so from its worker threads.
pub trait DetectionProgressCallback: Send + Sync {
    /// Called when entering a new phase. `total` is the number of items in this phase.
    fn on_phase(&self, phase: DetectionPhase, total: u32);
    /// Called after completing one item in the current phase.
    fn on_progress(&self);
}

/// No-op implementation for backwards compatibility.
pub struct NoopProgress;

impl DetectionProgressCallback for NoopProgress {
    fn on_phase(&self, _phase: DetectionPhase, _total: u32) {}
    fn on_progress(&self) {}
}

/// Simple atomic progress tracker that can be polled from another thread.
/// Useful for testing and as a reference implementation.
pub struct AtomicProgressTracker {
    pub phase: Mutex<String>,
    pub completed: AtomicU32,
    pub total: AtomicU32,
}

impl Default for AtomicProgressTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl AtomicProgressTracker {
    pub fn new() -> Self {
        Self {
            phase: Mutex::new(String::new()),
            completed: AtomicU32::new(0),
            total: AtomicU32::new(0),
        }
    }
}

impl DetectionProgressCallback for AtomicProgressTracker {
    fn on_phase(&self, phase: DetectionPhase, total: u32) {
        *self.phase.lock().unwrap() = phase.as_str().to_string();
        self.completed.store(0, Ordering::SeqCst);
        self.total.store(total, Ordering::SeqCst);
    }

    fn on_progress(&self) {
        self.completed.fetch_add(1, Ordering::SeqCst);
    }
}
