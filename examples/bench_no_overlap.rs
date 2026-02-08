//! No overlap worst case — pure overhead when activities don't overlap.
//!
//! Run with: `cargo run --example bench_no_overlap --features synthetic --release`

mod common;

#[global_allocator]
static ALLOCATOR: common::TrackingAllocator = common::TrackingAllocator;

fn main() {
    common::bench_no_overlap();
}
