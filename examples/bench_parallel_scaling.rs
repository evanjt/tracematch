//! Parallel scaling benchmark: measures speedup at various thread counts
//! and estimates mobile device performance.
//!
//! Run with: `cargo run --example bench_parallel_scaling --features synthetic --release`

mod common;

use std::time::{Duration, Instant};

use rayon::ThreadPoolBuilder;
use tracematch::synthetic::SyntheticScenario;
use tracematch::{SectionConfig, detect_sections_optimized};

#[global_allocator]
static ALLOCATOR: common::TrackingAllocator = common::TrackingAllocator;

/// Run detection with a specific thread count, returns (duration, sections_found).
fn run_with_threads(n_threads: usize, scenario: &SyntheticScenario) -> (Duration, usize) {
    let pool = ThreadPoolBuilder::new()
        .num_threads(n_threads)
        .build()
        .unwrap();

    let dataset = scenario.generate();
    let config = SectionConfig::default();

    pool.install(|| {
        let start = Instant::now();
        let result = detect_sections_optimized(&dataset.tracks, &dataset.sport_types, &config);
        (start.elapsed(), result.len())
    })
}

/// Run detection N times and return median duration.
fn median_of(runs: usize, n_threads: usize, scenario: &SyntheticScenario) -> (Duration, usize) {
    let mut durations = Vec::with_capacity(runs);
    let mut sections = 0;

    for _ in 0..runs {
        let (d, s) = run_with_threads(n_threads, scenario);
        durations.push(d);
        sections = s;
    }

    durations.sort();
    (durations[runs / 2], sections)
}

// Mobile device profiles
struct DeviceProfile {
    name: &'static str,
    // Effective thread count for compute-heavy work
    effective_threads: usize,
    // Single-core sustained perf relative to desktop (accounts for clock, IPC, thermals)
    sustained_factor: f64,
}

fn main() {
    let host_cores = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(8);

    println!("# Parallel Scaling Report (Optimized Detection)\n");
    println!("Host: {} logical cores\n", host_cores);

    // Test scenarios at different scales
    let test_configs: Vec<(&str, usize)> = vec![
        ("200 activities, 10km", 200),
        ("500 activities, 10km", 500),
        ("2000 activities, 10km", 2000),
    ];

    for (label, count) in &test_configs {
        let scenario = SyntheticScenario::with_activity_count(*count, 10_000.0, 0.8);
        let runs = if *count >= 2000 { 2 } else { 4 };

        println!("## {} ({} median runs)\n", label, runs);
        println!("| Threads | Time       | Speedup | Sections |");
        println!("|---------|------------|---------|----------|");

        // Thread counts to test
        let thread_counts: Vec<usize> = vec![1, 2, 4, 6, 8, 12, 16]
            .into_iter()
            .filter(|&t| t <= host_cores)
            .collect();

        let mut baseline_ms = 0u128;
        let mut results: Vec<(usize, u128, usize)> = Vec::new();

        for &threads in &thread_counts {
            let (duration, sections) = median_of(runs, threads, &scenario);
            let ms = duration.as_millis();

            if threads == 1 {
                baseline_ms = ms;
            }

            let speedup = baseline_ms as f64 / ms.max(1) as f64;
            println!(
                "| {:>7} | {:>8}ms | {:>5.2}x | {:>8} |",
                threads, ms, speedup, sections,
            );

            results.push((threads, ms, sections));
        }

        // Mobile estimates
        let devices = vec![
            DeviceProfile {
                name: "iPhone 15 Pro (A17)",
                // 2 performance + 4 efficiency (~0.5x perf each) ≈ 4 effective
                effective_threads: 4,
                // A17 sustained single-core is ~65% of desktop
                sustained_factor: 0.65,
            },
            DeviceProfile {
                name: "Flagship Android (SD 8 Elite)",
                // 2 prime (4.32GHz) + 6 perf (3.53GHz, ~0.82x prime) ≈ 7 effective
                effective_threads: 7,
                // SD 8 Elite sustained single-core is ~55% of desktop (thermal throttle)
                sustained_factor: 0.55,
            },
            DeviceProfile {
                name: "Mid-range Android (SD 7+ Gen 2)",
                // 1 prime + 3 perf (~0.7x) + 4 eff (~0.3x) ≈ 4 effective
                effective_threads: 4,
                // SD 7+ Gen 2 sustained is ~35% of desktop
                sustained_factor: 0.35,
            },
        ];

        println!("\n### Estimated Mobile Performance\n");
        println!("| Device                          | Est. threads | Est. time  | vs 1-thread |");
        println!("|---------------------------------|--------------|------------|-------------|");

        for device in &devices {
            // Find closest measured thread count
            let closest = results
                .iter()
                .min_by_key(|(t, _, _)| {
                    (*t as i32 - device.effective_threads as i32).unsigned_abs()
                })
                .unwrap();

            // Scale by sustained performance factor
            // Mobile time = desktop time at similar thread count / sustained_factor
            let estimated_ms = (closest.1 as f64 / device.sustained_factor) as u128;
            let vs_single =
                baseline_ms as f64 / device.sustained_factor / estimated_ms.max(1) as f64;

            println!(
                "| {:>31} | {:>12} | {:>8}ms | {:>9.2}x |",
                device.name, device.effective_threads, estimated_ms, vs_single,
            );
        }

        println!();
    }

    println!("_Note: Mobile estimates use sustained performance factors (not peak).");
    println!(
        "Actual mobile performance depends on thermal state, memory bandwidth, and OS scheduling."
    );
    println!(
        "Thread counts for mobile reflect effective compute threads (efficiency cores count as ~0.5x)._"
    );
}
