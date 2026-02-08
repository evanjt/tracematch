//! Realistic chaos — low overlap (12%) simulating diverse real-world routes.
//!
//! Run with: `cargo run --example bench_chaos --features synthetic --release`

mod common;

#[global_allocator]
static ALLOCATOR: common::TrackingAllocator = common::TrackingAllocator;

fn main() {
    common::bench_chaos();
}
