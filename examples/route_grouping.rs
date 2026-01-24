//! Example: Grouping activities by route
//!
//! Run with: cargo run --example route_grouping

use tracematch::{GpsPoint, MatchConfig, RouteSignature, group_signatures};

fn main() {
    // Two activities on the same route (London, ~1km)
    let london: Vec<GpsPoint> = (0..10)
        .map(|i| GpsPoint::new(51.5074 + i as f64 * 0.001, -0.1278))
        .collect();

    // One activity on a different route (NYC, ~1km)
    let nyc: Vec<GpsPoint> = (0..10)
        .map(|i| GpsPoint::new(40.7128 + i as f64 * 0.001, -74.0060))
        .collect();

    let config = MatchConfig::default();
    let sigs = vec![
        RouteSignature::from_points("monday", &london, &config).unwrap(),
        RouteSignature::from_points("wednesday", &london, &config).unwrap(),
        RouteSignature::from_points("friday", &nyc, &config).unwrap(),
    ];

    for group in group_signatures(&sigs, &config) {
        println!("{}: {:?}", group.group_id, group.activity_ids);
    }
}
