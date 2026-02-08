//! Incremental detection — adding new activities to existing sections.
//!
//! Run with: `cargo run --example bench_incremental --features synthetic --release`

mod common;

#[global_allocator]
static ALLOCATOR: common::TrackingAllocator = common::TrackingAllocator;

fn main() {
    common::bench_incremental();
}
