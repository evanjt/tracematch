//! Example: Comparing two routes
//!
//! Run with: cargo run --example route_matching

use tracematch::{GpsPoint, MatchConfig, RouteSignature, compare_routes};

fn main() {
    let london = vec![
        GpsPoint::new(51.5074, -0.1278),
        GpsPoint::new(51.5080, -0.1290),
        GpsPoint::new(51.5090, -0.1300),
    ];

    let nyc = vec![
        GpsPoint::new(40.7128, -74.0060),
        GpsPoint::new(40.7138, -74.0070),
        GpsPoint::new(40.7148, -74.0080),
    ];

    let config = MatchConfig::default();
    let sig1 = RouteSignature::from_points("london-1", &london, &config).unwrap();
    let sig2 = RouteSignature::from_points("london-2", &london, &config).unwrap();
    let sig3 = RouteSignature::from_points("nyc", &nyc, &config).unwrap();

    // Same routes
    println!("london-1 vs london-2:");
    match compare_routes(&sig1, &sig2, &config) {
        Some(result) => println!(
            "  {}% match ({})",
            result.match_percentage, result.direction
        ),
        None => println!("  no match"),
    }

    // Different routes
    println!("london-1 vs nyc:");
    match compare_routes(&sig1, &sig3, &config) {
        Some(result) => println!(
            "  {}% match ({})",
            result.match_percentage, result.direction
        ),
        None => println!("  no match"),
    }
}
