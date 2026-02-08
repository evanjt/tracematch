//! Predefined scenarios — standard_cycling, long_sections, multi_corridor, etc.
//!
//! Run with: `cargo run --example bench_scenarios --features synthetic --release`

mod common;

#[global_allocator]
static ALLOCATOR: common::TrackingAllocator = common::TrackingAllocator;

fn main() {
    common::bench_predefined_scenarios();
}
