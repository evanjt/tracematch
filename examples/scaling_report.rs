//! Full scaling report: runs all benchmarks and outputs a complete markdown report.
//!
//! Run with: `cargo run --example scaling_report --features synthetic --release`

mod common;

#[global_allocator]
static ALLOCATOR: common::TrackingAllocator = common::TrackingAllocator;

fn main() {
    println!("# Section Detection Scaling Report\n");
    println!(
        "Config: `SectionConfig::default()` (presets: short, medium, long, extra_long, ultra_long)\n"
    );

    common::bench_scaling_multiscale();
    println!();
    common::bench_scaling_optimized();
    println!();
    common::bench_route_length();
    println!();
    common::bench_overlap_density();
    println!();
    common::bench_no_overlap();
    println!();
    common::bench_predefined_scenarios();
    println!();
    common::bench_predefined_scenarios_large();
    println!();
    common::bench_grid_filtering();
    println!();
    common::bench_incremental();
    println!();
    common::bench_chaos();
    println!();
    common::bench_mobile_estimates();

    println!(
        "\n_Generated with `cargo run --example scaling_report --features synthetic --release`_"
    );
}
