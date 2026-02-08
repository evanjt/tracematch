//! Grid filtering impact — pair reduction from spatial grid pre-filtering.
//!
//! Run with: `cargo run --example bench_grid_filtering --features synthetic --release`

mod common;

#[global_allocator]
static ALLOCATOR: common::TrackingAllocator = common::TrackingAllocator;

fn main() {
    common::bench_grid_filtering();
}
