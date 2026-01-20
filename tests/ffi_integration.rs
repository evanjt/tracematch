//! FFI integration tests extracted from src/ffi.rs.
//!
//! These tests exercise the public FFI API exposed to mobile platforms.
//! Note: Most route matching functionality is accessed via the persistent engine,
//! not via direct FFI calls. The FFI layer is optimized for section detection,
//! heatmap generation, and bulk GPS data fetching.

use tracematch::{RouteGroup, SectionConfig, ffi::default_scale_presets};

// ============================================================================
// Scale Preset Tests
// ============================================================================

#[test]
fn test_default_scale_presets() {
    let presets = default_scale_presets();

    assert_eq!(presets.len(), 3, "Should have 3 default presets");

    // Verify preset names
    let names: Vec<&str> = presets.iter().map(|p| p.name.as_str()).collect();
    assert!(names.contains(&"short"));
    assert!(names.contains(&"medium"));
    assert!(names.contains(&"long"));

    // Verify presets are ordered by length
    for i in 1..presets.len() {
        assert!(
            presets[i].min_length >= presets[i - 1].max_length,
            "Presets should be ordered by increasing length"
        );
    }
}

// ============================================================================
// Section Detection Tests (using library types)
// ============================================================================

#[test]
fn test_section_configs_differ() {
    let default = SectionConfig::default();
    let discovery = SectionConfig::discovery();
    let conservative = SectionConfig::conservative();

    // Discovery should be more sensitive (lower thresholds)
    assert!(discovery.min_activities <= default.min_activities);

    // Conservative should be more strict (higher thresholds)
    assert!(conservative.min_activities >= default.min_activities);
}

#[test]
fn test_section_config_has_scale_presets() {
    let config = SectionConfig::default();
    assert!(
        !config.scale_presets.is_empty(),
        "Default config should have scale presets"
    );
}

// ============================================================================
// Custom Section JSON Serialization Tests
// ============================================================================

/// Verify that CustomSection serializes to camelCase JSON (matches TypeScript expectations)
#[test]
fn test_custom_section_json_uses_camel_case() {
    use tracematch::{CustomSection, GpsPoint};

    let section = CustomSection {
        id: "custom_123".to_string(),
        name: "Test Section".to_string(),
        polyline: vec![GpsPoint::new(51.5, -0.1)],
        source_activity_id: "act_456".to_string(),
        start_index: 10,
        end_index: 50,
        sport_type: "Ride".to_string(),
        distance_meters: 1234.5,
        created_at: "2024-01-01T00:00:00Z".to_string(),
    };

    let json = serde_json::to_string(&section).unwrap();

    // Verify camelCase field names (matches TypeScript expectations)
    assert!(
        json.contains("sourceActivityId"),
        "should use camelCase for sourceActivityId"
    );
    assert!(
        json.contains("startIndex"),
        "should use camelCase for startIndex"
    );
    assert!(
        json.contains("endIndex"),
        "should use camelCase for endIndex"
    );
    assert!(
        json.contains("sportType"),
        "should use camelCase for sportType"
    );
    assert!(
        json.contains("distanceMeters"),
        "should use camelCase for distanceMeters"
    );
    assert!(
        json.contains("createdAt"),
        "should use camelCase for createdAt"
    );

    // Verify it does NOT contain snake_case
    assert!(
        !json.contains("source_activity_id"),
        "should not use snake_case"
    );
    assert!(!json.contains("start_index"), "should not use snake_case");
    assert!(!json.contains("end_index"), "should not use snake_case");
    assert!(!json.contains("sport_type"), "should not use snake_case");
    assert!(
        !json.contains("distance_meters"),
        "should not use snake_case"
    );
    assert!(!json.contains("created_at"), "should not use snake_case");
}

/// Verify that CustomSection can deserialize from camelCase JSON (as TypeScript sends it)
#[test]
fn test_custom_section_deserialize_from_camel_case() {
    use tracematch::CustomSection;

    // JSON as TypeScript would send it
    let json = r#"{
        "id": "custom_123",
        "name": "Test Section",
        "polyline": [{"latitude": 51.5, "longitude": -0.1}],
        "sourceActivityId": "act_456",
        "startIndex": 10,
        "endIndex": 50,
        "sportType": "Ride",
        "distanceMeters": 1234.5,
        "createdAt": "2024-01-01T00:00:00Z"
    }"#;

    let section: CustomSection =
        serde_json::from_str(json).expect("should deserialize camelCase JSON");
    assert_eq!(section.source_activity_id, "act_456");
    assert_eq!(section.start_index, 10);
    assert_eq!(section.end_index, 50);
    assert_eq!(section.sport_type, "Ride");
    assert_eq!(section.distance_meters, 1234.5);
    assert_eq!(section.created_at, "2024-01-01T00:00:00Z");
}

/// Verify that CustomSectionMatch serializes to camelCase JSON
#[test]
fn test_custom_section_match_json_uses_camel_case() {
    use tracematch::CustomSectionMatch;

    let match_result = CustomSectionMatch {
        activity_id: "act_789".to_string(),
        start_index: 100,
        end_index: 200,
        direction: "same".to_string(),
        distance_meters: 500.0,
    };

    let json = serde_json::to_string(&match_result).unwrap();

    // Verify camelCase field names
    assert!(
        json.contains("activityId"),
        "should use camelCase for activityId"
    );
    assert!(
        json.contains("startIndex"),
        "should use camelCase for startIndex"
    );
    assert!(
        json.contains("endIndex"),
        "should use camelCase for endIndex"
    );
    assert!(
        json.contains("distanceMeters"),
        "should use camelCase for distanceMeters"
    );

    // Verify it does NOT contain snake_case
    assert!(!json.contains("activity_id"), "should not use snake_case");
    assert!(!json.contains("start_index"), "should not use snake_case");
    assert!(
        !json.contains("distance_meters"),
        "should not use snake_case"
    );
}

/// Verify that CustomSectionMatchConfig serializes to camelCase JSON
#[test]
fn test_custom_section_match_config_json_uses_camel_case() {
    use tracematch::CustomSectionMatchConfig;

    let config = CustomSectionMatchConfig {
        proximity_threshold: 50.0,
        min_coverage: 0.8,
    };

    let json = serde_json::to_string(&config).unwrap();

    // Verify camelCase field names
    assert!(
        json.contains("proximityThreshold"),
        "should use camelCase for proximityThreshold"
    );
    assert!(
        json.contains("minCoverage"),
        "should use camelCase for minCoverage"
    );

    // Verify it does NOT contain snake_case
    assert!(
        !json.contains("proximity_threshold"),
        "should not use snake_case"
    );
    assert!(!json.contains("min_coverage"), "should not use snake_case");
}

// ============================================================================
// New camelCase Serialization Tests for Newly Updated Structs
// ============================================================================

/// Verify RouteGroup now serializes to camelCase
#[test]
fn test_route_group_json_uses_camel_case() {
    let group = RouteGroup {
        group_id: "grp_123".to_string(),
        representative_id: "rep_456".to_string(),
        activity_ids: vec!["act_1".to_string(), "act_2".to_string()],
        sport_type: "Run".to_string(),
        bounds: None,
        custom_name: Some("Morning Loop".to_string()),
        best_time: Some(1800.0),
        avg_time: Some(2000.0),
        best_pace: Some(4.5),
        best_activity_id: Some("act_1".to_string()),
    };

    let json = serde_json::to_string(&group).unwrap();

    // Verify camelCase field names
    assert!(json.contains("groupId"), "should use camelCase for groupId");
    assert!(
        json.contains("representativeId"),
        "should use camelCase for representativeId"
    );
    assert!(
        json.contains("activityIds"),
        "should use camelCase for activityIds"
    );
    assert!(
        json.contains("sportType"),
        "should use camelCase for sportType"
    );
    assert!(
        json.contains("customName"),
        "should use camelCase for customName"
    );
    assert!(
        json.contains("bestTime"),
        "should use camelCase for bestTime"
    );
    assert!(json.contains("avgTime"), "should use camelCase for avgTime");
    assert!(
        json.contains("bestPace"),
        "should use camelCase for bestPace"
    );
    assert!(
        json.contains("bestActivityId"),
        "should use camelCase for bestActivityId"
    );

    // Verify no snake_case
    assert!(!json.contains("group_id"), "should not use snake_case");
    assert!(
        !json.contains("representative_id"),
        "should not use snake_case"
    );
    assert!(!json.contains("activity_ids"), "should not use snake_case");
    assert!(!json.contains("sport_type"), "should not use snake_case");
    assert!(!json.contains("custom_name"), "should not use snake_case");
    assert!(!json.contains("best_time"), "should not use snake_case");
    assert!(!json.contains("avg_time"), "should not use snake_case");
    assert!(!json.contains("best_pace"), "should not use snake_case");
    assert!(
        !json.contains("best_activity_id"),
        "should not use snake_case"
    );
}

/// Verify SectionConfig now serializes to camelCase
#[test]
fn test_section_config_json_uses_camel_case() {
    let config = SectionConfig::default();
    let json = serde_json::to_string(&config).unwrap();

    // Verify camelCase field names
    assert!(json.contains("proximityThreshold"), "should use camelCase");
    assert!(json.contains("minSectionLength"), "should use camelCase");
    assert!(json.contains("maxSectionLength"), "should use camelCase");
    assert!(json.contains("minActivities"), "should use camelCase");
    assert!(json.contains("clusterTolerance"), "should use camelCase");
    assert!(json.contains("samplePoints"), "should use camelCase");
    assert!(json.contains("detectionMode"), "should use camelCase");
    assert!(json.contains("includePotentials"), "should use camelCase");
    assert!(json.contains("scalePresets"), "should use camelCase");
    assert!(json.contains("preserveHierarchy"), "should use camelCase");

    // Verify no snake_case
    assert!(
        !json.contains("proximity_threshold"),
        "should not use snake_case"
    );
    assert!(
        !json.contains("min_section_length"),
        "should not use snake_case"
    );
    assert!(
        !json.contains("max_section_length"),
        "should not use snake_case"
    );
    assert!(
        !json.contains("min_activities"),
        "should not use snake_case"
    );
}

/// Verify ScalePreset now serializes to camelCase
#[test]
fn test_scale_preset_json_uses_camel_case() {
    use tracematch::ScalePreset;

    let preset = ScalePreset::short();
    let json = serde_json::to_string(&preset).unwrap();

    // Verify camelCase field names
    assert!(
        json.contains("minLength"),
        "should use camelCase for minLength"
    );
    assert!(
        json.contains("maxLength"),
        "should use camelCase for maxLength"
    );
    assert!(
        json.contains("minActivities"),
        "should use camelCase for minActivities"
    );

    // Verify no snake_case
    assert!(!json.contains("min_length"), "should not use snake_case");
    assert!(!json.contains("max_length"), "should not use snake_case");
    assert!(
        !json.contains("min_activities"),
        "should not use snake_case"
    );
}
