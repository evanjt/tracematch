//! tracematch CLI - Debug tool for route/section detection
//!
//! Usage:
//!   tracematch-cli routes <folder> [--output <dir>]
//!   tracematch-cli sections <folder> [--output <dir>] [--sport <type>]
//!
//! This tool processes GPX files and shows verbose debug output of the
//! detection algorithms, helping to understand how routes and sections
//! are being matched and grouped.

use clap::{Parser, Subcommand};
use gpx::{Gpx, read};
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{BufReader, BufWriter, Write};
use std::path::PathBuf;
use tracematch::{
    GpsPoint, MatchConfig, RouteGroup, RouteSignature,
    grouping::group_signatures,
    sections::{
        FrequentSection, SectionConfig, detect_sections_multiscale, detect_sections_optimized,
    },
};

#[derive(Parser)]
#[command(name = "tracematch-cli")]
#[command(about = "Debug tool for route and section detection", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,

    /// Enable verbose debug output
    #[arg(short, long, global = true)]
    verbose: bool,
}

#[derive(Subcommand)]
enum Commands {
    /// Detect and group similar routes
    Routes {
        /// Folder containing GPX files
        folder: PathBuf,

        /// Output directory for results (GPX files)
        #[arg(short, long)]
        output: Option<PathBuf>,

        /// Filter by sport type (e.g., "Run", "Ride")
        #[arg(short, long)]
        sport: Option<String>,
    },

    /// Detect frequent sections across routes
    Sections {
        /// Folder containing GPX files
        folder: PathBuf,

        /// Output directory for results (GPX files)
        #[arg(short, long)]
        output: Option<PathBuf>,

        /// Filter by sport type (e.g., "Run", "Ride")
        #[arg(short, long)]
        sport: Option<String>,

        /// Minimum number of activities to form a section
        #[arg(long, default_value = "2")]
        min_activities: u32,

        /// Minimum section length in meters
        #[arg(long, default_value = "100")]
        min_length: f64,

        /// Use legacy full-resolution detection (slower, for debugging)
        #[arg(long)]
        legacy: bool,
    },
}

fn main() {
    // Initialize logging
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format(|buf, record| writeln!(buf, "[{:5}] {}", record.level(), record.args()))
        .init();

    let cli = Cli::parse();

    match cli.command {
        Commands::Routes {
            folder,
            output,
            sport,
        } => {
            run_routes(&folder, output.as_ref(), sport.as_deref(), cli.verbose);
        }
        Commands::Sections {
            folder,
            output,
            sport,
            min_activities,
            min_length,
            legacy,
        } => {
            run_sections(
                &folder,
                output.as_ref(),
                sport.as_deref(),
                min_activities,
                min_length,
                legacy,
                cli.verbose,
            );
        }
    }
}

/// Load GPX files from a folder and extract GPS points
fn load_gpx_files(folder: &PathBuf, sport_filter: Option<&str>, verbose: bool) -> Vec<GpxActivity> {
    println!("\n{}", "=".repeat(60));
    println!("Loading GPX files from: {}", folder.display());
    println!("{}", "=".repeat(60));

    let mut activities = Vec::new();

    let entries = match fs::read_dir(folder) {
        Ok(e) => e,
        Err(e) => {
            eprintln!("Error reading folder: {}", e);
            return activities;
        }
    };

    for entry in entries.flatten() {
        let path = entry.path();
        if path.extension().map_or(false, |ext| ext == "gpx") {
            if verbose {
                println!("\n  Processing: {}", path.display());
            }

            match parse_gpx_file(&path) {
                Ok(activity) => {
                    // Apply sport filter if specified
                    if let Some(filter) = sport_filter {
                        if !activity.sport_type.eq_ignore_ascii_case(filter) {
                            if verbose {
                                println!(
                                    "    Skipped (sport: {} != {})",
                                    activity.sport_type, filter
                                );
                            }
                            continue;
                        }
                    }

                    println!(
                        "  [OK] {} - {} points, {:.1}km, sport: {}",
                        activity.name,
                        activity.points.len(),
                        activity.distance_km(),
                        activity.sport_type
                    );
                    activities.push(activity);
                }
                Err(e) => {
                    eprintln!("  [ERR] Failed to parse {}: {}", path.display(), e);
                }
            }
        }
    }

    println!("\nLoaded {} activities", activities.len());
    activities
}

/// Parse a single GPX file
fn parse_gpx_file(path: &PathBuf) -> Result<GpxActivity, String> {
    let file = File::open(path).map_err(|e| e.to_string())?;
    let reader = BufReader::new(file);
    let gpx: Gpx = read(reader).map_err(|e| e.to_string())?;

    // Extract activity name from filename or GPX metadata
    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("unknown")
        .to_string();

    // Determine sport type from GPX or filename
    let sport_type = detect_sport_type(&gpx, &name);

    // Extract all track points
    let mut points = Vec::new();
    for track in &gpx.tracks {
        for segment in &track.segments {
            for pt in &segment.points {
                if let (Some(lat), Some(lon)) = (pt.point().y().into(), pt.point().x().into()) {
                    points.push(GpsPoint {
                        latitude: lat,
                        longitude: lon,
                        elevation: pt.elevation,
                    });
                }
            }
        }
    }

    if points.is_empty() {
        return Err("No track points found".to_string());
    }

    // Generate unique ID from filename
    let id = path
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("unknown")
        .to_string();

    Ok(GpxActivity {
        id,
        name,
        sport_type,
        points,
    })
}

/// Detect sport type from GPX metadata or filename
fn detect_sport_type(gpx: &Gpx, filename: &str) -> String {
    // Try to get from GPX metadata
    if let Some(metadata) = &gpx.metadata {
        if let Some(name) = &metadata.name {
            let lower = name.to_lowercase();
            if lower.contains("run") || lower.contains("running") {
                return "Run".to_string();
            }
            if lower.contains("ride") || lower.contains("cycling") || lower.contains("bike") {
                return "Ride".to_string();
            }
            if lower.contains("hike") || lower.contains("hiking") || lower.contains("walk") {
                return "Hike".to_string();
            }
            if lower.contains("swim") {
                return "Swim".to_string();
            }
        }
    }

    // Try from filename
    let lower = filename.to_lowercase();
    if lower.contains("run") || lower.contains("running") {
        return "Run".to_string();
    }
    if lower.contains("ride") || lower.contains("cycling") || lower.contains("bike") {
        return "Ride".to_string();
    }
    if lower.contains("hike") || lower.contains("hiking") || lower.contains("walk") {
        return "Hike".to_string();
    }
    if lower.contains("swim") {
        return "Swim".to_string();
    }

    "Unknown".to_string()
}

/// Activity loaded from GPX file
struct GpxActivity {
    id: String,
    name: String,
    sport_type: String,
    points: Vec<GpsPoint>,
}

impl GpxActivity {
    fn distance_km(&self) -> f64 {
        tracematch::matching::calculate_route_distance(&self.points) / 1000.0
    }
}

/// Run route grouping
fn run_routes(
    folder: &PathBuf,
    output: Option<&PathBuf>,
    sport_filter: Option<&str>,
    verbose: bool,
) {
    let activities = load_gpx_files(folder, sport_filter, verbose);
    if activities.is_empty() {
        println!("No activities to process");
        return;
    }

    println!("\n{}", "=".repeat(60));
    println!("ROUTE GROUPING");
    println!("{}", "=".repeat(60));

    // Generate signatures for each activity
    println!("\n[Step 1] Generating route signatures...");
    let config = MatchConfig::default();
    let mut signatures = Vec::new();
    let mut sport_types = HashMap::new();

    for activity in &activities {
        if verbose {
            println!("  Generating signature for: {}", activity.name);
            println!("    Points: {}", activity.points.len());
        }

        if let Some(sig) = RouteSignature::from_points(&activity.id, &activity.points, &config) {
            if verbose {
                println!("    Signature points: {}", sig.points.len());
                println!("    Distance: {:.2}km", sig.total_distance / 1000.0);
                println!(
                    "    Bounds: [{:.4}, {:.4}] to [{:.4}, {:.4}]",
                    sig.bounds.min_lat, sig.bounds.min_lng, sig.bounds.max_lat, sig.bounds.max_lng
                );
            }
            sport_types.insert(activity.id.clone(), activity.sport_type.clone());
            signatures.push(sig);
        } else {
            println!(
                "  [WARN] Could not generate signature for: {}",
                activity.name
            );
        }
    }

    println!("  Generated {} signatures", signatures.len());

    // Group routes
    println!("\n[Step 2] Grouping similar routes...");
    let groups = group_signatures(&signatures, &config);

    println!("\n{}", "-".repeat(60));
    println!("RESULTS: Found {} route groups", groups.len());
    println!("{}", "-".repeat(60));

    for (i, group) in groups.iter().enumerate() {
        println!(
            "\n  Group {} ({} activities):",
            i + 1,
            group.activity_ids.len()
        );
        println!("    Representative: {}", group.representative_id);
        println!("    Sport type: {}", group.sport_type);
        if let Some(bounds) = &group.bounds {
            println!(
                "    Bounds: [{:.4}, {:.4}] to [{:.4}, {:.4}]",
                bounds.min_lat, bounds.min_lng, bounds.max_lat, bounds.max_lng
            );
        }
        println!("    Activities:");
        for aid in &group.activity_ids {
            let activity_name = activities.iter().find(|a| a.id == *aid).map(|a| &a.name);
            println!(
                "      - {} ({})",
                aid,
                activity_name.unwrap_or(&"unknown".to_string())
            );
        }
    }

    // Export results if output directory specified
    if let Some(output_dir) = output {
        export_route_groups(&groups, &signatures, output_dir, verbose);
    }
}

/// Run section detection
fn run_sections(
    folder: &PathBuf,
    output: Option<&PathBuf>,
    sport_filter: Option<&str>,
    min_activities: u32,
    min_length: f64,
    legacy: bool,
    verbose: bool,
) {
    let activities = load_gpx_files(folder, sport_filter, verbose);
    if activities.is_empty() {
        println!("No activities to process");
        return;
    }

    println!("\n{}", "=".repeat(60));
    println!("SECTION DETECTION");
    println!("{}", "=".repeat(60));

    // Prepare data for section detection
    println!("\n[Step 1] Preparing full GPS tracks...");
    let tracks: Vec<(String, Vec<GpsPoint>)> = activities
        .iter()
        .map(|a| (a.id.clone(), a.points.clone()))
        .collect();

    let sport_types: HashMap<String, String> = activities
        .iter()
        .map(|a| (a.id.clone(), a.sport_type.clone()))
        .collect();

    println!("  Prepared {} tracks for analysis", tracks.len());

    // Configure section detection
    let mut config = SectionConfig::discovery();
    config.min_activities = min_activities;
    config.min_section_length = min_length;

    if verbose {
        println!("\n[Config]");
        println!("  proximity_threshold: {}m", config.proximity_threshold);
        println!("  min_section_length: {}m", config.min_section_length);
        println!("  max_section_length: {}m", config.max_section_length);
        println!("  min_activities: {}", config.min_activities);
        println!("  cluster_tolerance: {}m", config.cluster_tolerance);
        println!(
            "  scale_presets: {:?}",
            config
                .scale_presets
                .iter()
                .map(|s| &s.name)
                .collect::<Vec<_>>()
        );
    }

    // Run section detection
    let sections: Vec<FrequentSection> = if legacy {
        // Legacy: full-resolution multi-scale (slower but more detailed)
        println!("\n[Step 2] Running LEGACY multi-scale section detection...");
        println!(
            "  This analyzes pairwise overlaps between all {} tracks",
            tracks.len()
        );
        println!(
            "  Total pairs to check: {}",
            tracks.len() * (tracks.len() - 1) / 2
        );
        println!("  ⚠️  Using legacy mode - this is slower than default!");

        let groups: Vec<RouteGroup> = Vec::new();
        let result = detect_sections_multiscale(&tracks, &sport_types, &groups, &config);

        // Show legacy statistics
        println!("\nLegacy Statistics:");
        println!(
            "  Activities processed: {}",
            result.stats.activities_processed
        );
        println!("  Overlaps found: {}", result.stats.overlaps_found);
        for (scale, count) in &result.stats.sections_by_scale {
            println!("    {}: {}", scale, count);
        }

        result.sections
    } else {
        // Default: optimized detection with downsampling and grid partitioning
        println!("\n[Step 2] Running optimized section detection...");
        println!("  Using downsampling (100 pts) + grid partitioning for speed");
        println!("  Tracks: {}", tracks.len());

        detect_sections_optimized(&tracks, &sport_types, &config)
    };

    println!("\n{}", "-".repeat(60));
    println!("RESULTS");
    println!("{}", "-".repeat(60));

    // Show sections
    println!("\nSections found: {}", sections.len());
    for (i, section) in sections.iter().enumerate() {
        println!("\n  Section {} [{}]:", i + 1, section.id);
        println!(
            "    Name: {}",
            section.name.as_deref().unwrap_or("(unnamed)")
        );
        println!("    Sport: {}", section.sport_type);
        println!("    Distance: {:.0}m", section.distance_meters);
        println!("    Visits: {}", section.visit_count);
        println!("    Confidence: {:.2}", section.confidence);
        println!(
            "    Scale: {}",
            section.scale.as_deref().unwrap_or("legacy")
        );
        println!("    Polyline points: {}", section.polyline.len());
        println!("    Activities ({}):", section.activity_ids.len());
        for aid in &section.activity_ids {
            let activity_name = activities
                .iter()
                .find(|a| a.id == *aid)
                .map(|a| a.name.as_str())
                .unwrap_or("unknown");
            let direction = section
                .activity_portions
                .iter()
                .find(|p| p.activity_id == *aid)
                .map(|p| p.direction.as_str())
                .unwrap_or("unknown");
            println!("      - {} ({}) [{}]", aid, activity_name, direction);
        }

        if verbose {
            println!("    Observation count: {}", section.observation_count);
            println!("    Average spread: {:.2}m", section.average_spread);
            if !section.point_density.is_empty() {
                let avg_density: f64 = section.point_density.iter().map(|&x| x as f64).sum::<f64>()
                    / section.point_density.len() as f64;
                println!("    Avg point density: {:.1}", avg_density);
            }
        }
    }

    // Export results if output directory specified
    if let Some(output_dir) = output {
        export_sections(&sections, output_dir, verbose);
    }
}

/// Export route groups as GPX files
fn export_route_groups(
    groups: &[RouteGroup],
    signatures: &[RouteSignature],
    output_dir: &PathBuf,
    verbose: bool,
) {
    println!(
        "\n[Export] Writing route groups to: {}",
        output_dir.display()
    );
    fs::create_dir_all(output_dir).expect("Failed to create output directory");

    for (i, group) in groups.iter().enumerate() {
        // Find the representative signature
        let rep_sig = signatures
            .iter()
            .find(|s| s.activity_id == group.representative_id);

        if let Some(sig) = rep_sig {
            let filename = format!("route_group_{:03}.gpx", i + 1);
            let path = output_dir.join(&filename);

            if verbose {
                println!("  Writing: {}", filename);
            }

            write_gpx_file(
                &path,
                &sig.points,
                &format!(
                    "Route Group {} ({} activities)",
                    i + 1,
                    group.activity_ids.len()
                ),
            );
        }
    }

    println!("  Exported {} route groups", groups.len());
}

/// Export sections as GeoJSON (combined) and individual GPX files
fn export_sections(sections: &[FrequentSection], output_dir: &PathBuf, verbose: bool) {
    println!("\n[Export] Writing sections to: {}", output_dir.display());
    fs::create_dir_all(output_dir).expect("Failed to create output directory");

    // Write combined GeoJSON with all sections
    let geojson_path = output_dir.join("sections.geojson");
    write_sections_geojson(sections, &geojson_path);
    println!("  Written: sections.geojson ({} features)", sections.len());

    // Also write individual GPX files
    if verbose {
        for (i, section) in sections.iter().enumerate() {
            let filename = format!(
                "section_{:03}_{}.gpx",
                i + 1,
                section.sport_type.to_lowercase()
            );
            let path = output_dir.join(&filename);
            println!(
                "  Writing: {} ({} points)",
                filename,
                section.polyline.len()
            );

            let name = section.name.clone().unwrap_or_else(|| {
                format!(
                    "Section {} ({} visits, {:.0}m)",
                    i + 1,
                    section.visit_count,
                    section.distance_meters
                )
            });
            write_gpx_file(&path, &section.polyline, &name);
        }
        println!("  Exported {} GPX files", sections.len());
    }
}

/// Write all sections to a single GeoJSON FeatureCollection
fn write_sections_geojson(sections: &[FrequentSection], path: &PathBuf) {
    let file = File::create(path).expect("Failed to create GeoJSON file");
    let mut writer = BufWriter::new(file);

    writeln!(writer, r#"{{"type": "FeatureCollection", "features": ["#).unwrap();

    for (i, section) in sections.iter().enumerate() {
        // Build coordinates array [lng, lat] (GeoJSON order)
        let coords: Vec<String> = section
            .polyline
            .iter()
            .map(|p| format!("[{:.6}, {:.6}]", p.longitude, p.latitude))
            .collect();

        let name = section
            .name
            .clone()
            .unwrap_or_else(|| format!("Section {} ({:.0}m)", i + 1, section.distance_meters));

        write!(
            writer,
            r#"  {{"type": "Feature", "properties": {{"id": "{}", "name": "{}", "sport": "{}", "distance_m": {:.0}, "visits": {}, "confidence": {:.2}, "activities": {}}}, "geometry": {{"type": "LineString", "coordinates": [{}]}}}}"#,
            section.id,
            name.replace('"', "'"),
            section.sport_type,
            section.distance_meters,
            section.visit_count,
            section.confidence,
            section.activity_ids.len(),
            coords.join(", ")
        ).unwrap();

        if i < sections.len() - 1 {
            writeln!(writer, ",").unwrap();
        } else {
            writeln!(writer).unwrap();
        }
    }

    writeln!(writer, "]}}").unwrap();
}

/// Write GPS points to a GPX file
fn write_gpx_file(path: &PathBuf, points: &[GpsPoint], name: &str) {
    let file = File::create(path).expect("Failed to create GPX file");
    let mut writer = BufWriter::new(file);

    // Write GPX header
    writeln!(writer, r#"<?xml version="1.0" encoding="UTF-8"?>"#).unwrap();
    writeln!(
        writer,
        r#"<gpx version="1.1" creator="tracematch-cli" xmlns="http://www.topografix.com/GPX/1/1">"#
    )
    .unwrap();
    writeln!(writer, "  <metadata>").unwrap();
    writeln!(writer, "    <name>{}</name>", escape_xml(name)).unwrap();
    writeln!(writer, "  </metadata>").unwrap();
    writeln!(writer, "  <trk>").unwrap();
    writeln!(writer, "    <name>{}</name>", escape_xml(name)).unwrap();
    writeln!(writer, "    <trkseg>").unwrap();

    for point in points {
        writeln!(
            writer,
            r#"      <trkpt lat="{:.6}" lon="{:.6}"></trkpt>"#,
            point.latitude, point.longitude
        )
        .unwrap();
    }

    writeln!(writer, "    </trkseg>").unwrap();
    writeln!(writer, "  </trk>").unwrap();
    writeln!(writer, "</gpx>").unwrap();
}

/// Escape XML special characters
fn escape_xml(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}
