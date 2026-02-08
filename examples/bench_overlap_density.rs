//! Overlap density — how overlap percentage affects detection time.
//!
//! Run with: `cargo run --example bench_overlap_density --features synthetic --release`

mod common;

#[global_allocator]
static ALLOCATOR: common::TrackingAllocator = common::TrackingAllocator;

fn main() {
    common::bench_overlap_density();
}
