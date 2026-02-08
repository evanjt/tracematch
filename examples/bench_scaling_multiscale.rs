//! Scaling curve — Multiscale detection at increasing activity counts.
//!
//! This is the slowest benchmark (minutes at 1000 activities).
//!
//! Run with: `cargo run --example bench_scaling_multiscale --features synthetic --release`

mod common;

#[global_allocator]
static ALLOCATOR: common::TrackingAllocator = common::TrackingAllocator;

fn main() {
    common::bench_scaling_multiscale();
}
