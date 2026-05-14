//! End-to-end report on the sionrunning corpus.
//!
//! Loads every GPX in `sionrunning/`, runs route grouping + section
//! detection through all three detection modes, and prints a comparison.
//!
//! Run from the tracematch repo root:
//!     cargo run --release --example sion_corpus_report --features synthetic
//!
//! If the sionrunning/ directory is missing, the example exits with a
//! clear message instead of failing.

use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use tracematch::{
    DetectionProgressCallback, FrequentSection, GpsPoint, MatchConfig, RouteSignature,
    SectionConfig,
};

#[path = "common/corpus.rs"]
mod corpus;
use corpus::{PhaseTimer, fmt_ms, load_gpx};

fn section_stats(sections: &[FrequentSection]) -> (u32, u32, f64, f64) {
    if sections.is_empty() {
        return (0, 0, 0.0, 0.0);
    }
    let max_visits = sections.iter().map(|s| s.visit_count).max().unwrap_or(0);
    let mut visits: Vec<u32> = sections.iter().map(|s| s.visit_count).collect();
    visits.sort();
    let median_visits = visits[visits.len() / 2];
    let mut dists: Vec<f64> = sections.iter().map(|s| s.distance_meters).collect();
    dists.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median_dist = dists[dists.len() / 2];
    let total_visits: u32 = visits.iter().sum();
    let _ = max_visits;
    let _ = total_visits;
    (
        median_visits,
        total_visits,
        median_dist,
        dists[dists.len() - 1],
    )
}

fn main() {
    let dir = Path::new("sionrunning");
    if !dir.exists() {
        eprintln!(
            "sionrunning/ not found in cwd ({}). \n\
             Run from the tracematch repo root.",
            std::env::current_dir()
                .map(|p| p.display().to_string())
                .unwrap_or_else(|_| "?".into())
        );
        return;
    }

    // --- 1. Load all GPX files ----------------------------------------
    let t_load = Instant::now();
    let entries = std::fs::read_dir(dir).expect("read_dir");
    let mut raw_tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    let mut total_points = 0usize;
    let mut skipped_short = 0usize;
    for entry in entries.flatten() {
        let path = entry.path();
        if !path.extension().is_some_and(|e| e == "gpx") {
            continue;
        }
        let pts = load_gpx(&path);
        if pts.len() < 50 {
            skipped_short += 1;
            continue;
        }
        let name = path
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();
        total_points += pts.len();
        raw_tracks.push((name, pts));
    }
    let dur_load = t_load.elapsed();

    // --- 2. Build RouteSignatures ------------------------------------
    let match_config = MatchConfig::default();
    let t_sig = Instant::now();
    let mut signatures: Vec<RouteSignature> = Vec::new();
    for (name, pts) in &raw_tracks {
        if let Some(sig) = RouteSignature::from_points(name, pts, &match_config) {
            signatures.push(sig);
        }
    }
    let dur_sig = t_sig.elapsed();

    // --- 3. Route grouping -------------------------------------------
    let t_group = Instant::now();
    let groups = tracematch::group_signatures_parallel(&signatures, &match_config);
    let dur_group = t_group.elapsed();

    let mut group_sizes: Vec<usize> = groups.iter().map(|g| g.activity_ids.len()).collect();
    group_sizes.sort_by(|a, b| b.cmp(a));
    let singleton = group_sizes.iter().filter(|&&n| n == 1).count();

    // --- 4. Section detection (all three modes) ----------------------
    let sport_types: HashMap<String, String> = raw_tracks
        .iter()
        .map(|(id, _)| (id.clone(), "Run".to_string()))
        .collect();

    let section_config = SectionConfig::default();

    // Density grid (multiscale)
    let timer = Arc::new(PhaseTimer::new());
    let t_density = Instant::now();
    let density_result = tracematch::detect_sections_multiscale_with_progress(
        &raw_tracks,
        &sport_types,
        &groups,
        &section_config,
        timer.clone() as Arc<dyn DetectionProgressCallback>,
    );
    let dur_density = t_density.elapsed();
    let mut density_sections = density_result.sections;
    density_sections.sort_by_key(|s| std::cmp::Reverse(s.visit_count));

    // Flow graph
    let t_flow = Instant::now();
    let mut flow_sections =
        tracematch::detect_sections_flow_graph(&raw_tracks, &sport_types, &section_config);
    let dur_flow = t_flow.elapsed();
    flow_sections.sort_by_key(|s| std::cmp::Reverse(s.visit_count));

    // Corridor
    let t_corr = Instant::now();
    let mut corridor_sections =
        tracematch::detect_sections_corridor(&raw_tracks, &sport_types, &section_config);
    let dur_corr = t_corr.elapsed();
    corridor_sections.sort_by_key(|s| std::cmp::Reverse(s.visit_count));

    // --- Print report ------------------------------------------------
    let dur_total = dur_load + dur_sig + dur_group + dur_density + dur_flow + dur_corr;

    println!();
    println!("┌─────────────────────────────────────────────────────────┐");
    println!("│                  Sion Corpus Report                     │");
    println!("└─────────────────────────────────────────────────────────┘");
    println!();
    println!(
        "  Corpus       {} tracks, {} GPS points ({:.0} avg/track)",
        raw_tracks.len(),
        total_points,
        total_points as f64 / raw_tracks.len().max(1) as f64,
    );
    println!("  Skipped      {} files (<50 points)", skipped_short);
    println!(
        "  Routes       {} groups, {} singletons ({:.0}%), largest {}",
        groups.len(),
        singleton,
        100.0 * singleton as f64 / groups.len().max(1) as f64,
        group_sizes.first().unwrap_or(&0),
    );
    println!(
        "  Config       proximity {}m, min length {}m, min activities {}",
        section_config.proximity_threshold,
        section_config.min_section_length,
        section_config.min_activities,
    );
    println!();

    // Comparison table
    let (d_med_v, d_tot_v, d_med_d, d_max_d) = section_stats(&density_sections);
    let (f_med_v, f_tot_v, f_med_d, f_max_d) = section_stats(&flow_sections);
    let (c_med_v, c_tot_v, c_med_d, c_max_d) = section_stats(&corridor_sections);

    println!("  ┌──────────────────────┬────────────┬────────────┬────────────┐");
    println!("  │                      │  Density   │   Flow     │  Corridor  │");
    println!("  ├──────────────────────┼────────────┼────────────┼────────────┤");
    println!(
        "  │ Sections             │ {:>10} │ {:>10} │ {:>10} │",
        density_sections.len(),
        flow_sections.len(),
        corridor_sections.len()
    );
    println!(
        "  │ Time                 │ {:>10} │ {:>10} │ {:>10} │",
        fmt_ms(dur_density.as_millis()),
        fmt_ms(dur_flow.as_millis()),
        fmt_ms(dur_corr.as_millis())
    );
    println!(
        "  │ Total visits         │ {:>10} │ {:>10} │ {:>10} │",
        d_tot_v, f_tot_v, c_tot_v
    );
    println!(
        "  │ Median visits        │ {:>10} │ {:>10} │ {:>10} │",
        d_med_v, f_med_v, c_med_v
    );
    println!(
        "  │ Median distance (m)  │ {:>10.0} │ {:>10.0} │ {:>10.0} │",
        d_med_d, f_med_d, c_med_d
    );
    println!(
        "  │ Max distance (m)     │ {:>10.0} │ {:>10.0} │ {:>10.0} │",
        d_max_d, f_max_d, c_max_d
    );
    println!("  └──────────────────────┴────────────┴────────────┴────────────┘");
    println!();

    // Top sections per mode
    let modes: [(&str, &[FrequentSection]); 3] = [
        ("Density", &density_sections),
        ("Flow", &flow_sections),
        ("Corridor", &corridor_sections),
    ];

    println!("  Top 5 sections per mode:");
    println!("  ┌────────────┬────────┬──────────┬──────────────────────────────────────┐");
    println!("  │ Mode       │ Visits │ Dist (m) │ Detail                               │");
    println!("  ├────────────┼────────┼──────────┼──────────────────────────────────────┤");

    for (mode_name, secs) in &modes {
        for (i, s) in secs.iter().take(5).enumerate() {
            let label = if i == 0 {
                format!("{:<10}", mode_name)
            } else {
                "          ".to_string()
            };
            let detail = s
                .name
                .as_deref()
                .unwrap_or(&s.id)
                .chars()
                .take(36)
                .collect::<String>();
            println!(
                "  │ {} │ {:>6} │ {:>8.0} │ {:<36} │",
                label, s.visit_count, s.distance_meters, detail
            );
        }
        if *mode_name != "Corridor" {
            println!("  ├────────────┼────────┼──────────┼──────────────────────────────────────┤");
        }
    }
    println!("  └────────────┴────────┴──────────┴──────────────────────────────────────┘");
    println!();

    // Timing breakdown
    println!("  Timing:");
    println!(
        "    Load + signatures    {}",
        fmt_ms((dur_load + dur_sig).as_millis())
    );
    println!("    Route grouping       {}", fmt_ms(dur_group.as_millis()));
    println!(
        "    Density detection    {}",
        fmt_ms(dur_density.as_millis())
    );
    println!("    Flow detection       {}", fmt_ms(dur_flow.as_millis()));
    println!("    Corridor detection   {}", fmt_ms(dur_corr.as_millis()));
    println!("    ─────────────────────────");
    println!("    Total                {}", fmt_ms(dur_total.as_millis()));
    println!();
}
