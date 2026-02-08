//! Route length impact — how corridor length affects detection time.
//!
//! Run with: `cargo run --example bench_route_length --features synthetic --release`

mod common;

#[global_allocator]
static ALLOCATOR: common::TrackingAllocator = common::TrackingAllocator;

fn main() {
    common::bench_route_length();
}
