//! Scaling curve — Optimized (downsampled) detection at increasing activity counts.
//!
//! Run with: `cargo run --example bench_scaling_optimized --features synthetic --release`

mod common;

#[global_allocator]
static ALLOCATOR: common::TrackingAllocator = common::TrackingAllocator;

fn main() {
    common::bench_scaling_optimized();
}
