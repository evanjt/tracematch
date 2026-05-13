use wasm_bindgen::prelude::*;

#[wasm_bindgen(start)]
pub fn init() {
    console_error_panic_hook::set_once();
}

/// Create a RouteSignature from raw GPS points.
///
/// points_json: `[{"latitude": f64, "longitude": f64, "elevation": f64?}, ...]`
/// config_json: optional MatchConfig JSON (uses defaults if empty or "{}")
///
/// Returns RouteSignature as JSON, or null if insufficient points.
#[wasm_bindgen(js_name = "createSignature")]
pub fn create_signature(
    activity_id: &str,
    points_json: &str,
    config_json: &str,
) -> Result<JsValue, JsError> {
    let points: Vec<tracematch::GpsPoint> =
        serde_json::from_str(points_json).map_err(|e| JsError::new(&e.to_string()))?;

    let config: tracematch::MatchConfig = if config_json.is_empty() || config_json == "{}" {
        tracematch::MatchConfig::default()
    } else {
        serde_json::from_str(config_json).map_err(|e| JsError::new(&e.to_string()))?
    };

    match tracematch::RouteSignature::from_points(activity_id, &points, &config) {
        Some(sig) => {
            let val =
                serde_wasm_bindgen::to_value(&sig).map_err(|e| JsError::new(&e.to_string()))?;
            Ok(val)
        }
        None => Ok(JsValue::NULL),
    }
}

/// Compare two route signatures.
///
/// Returns MatchResult as JS object, or null if no match.
#[wasm_bindgen(js_name = "compareRoutes")]
pub fn compare_routes(
    sig1_json: &str,
    sig2_json: &str,
    config_json: &str,
) -> Result<JsValue, JsError> {
    let sig1: tracematch::RouteSignature =
        serde_json::from_str(sig1_json).map_err(|e| JsError::new(&e.to_string()))?;
    let sig2: tracematch::RouteSignature =
        serde_json::from_str(sig2_json).map_err(|e| JsError::new(&e.to_string()))?;

    let config: tracematch::MatchConfig = if config_json.is_empty() || config_json == "{}" {
        tracematch::MatchConfig::default()
    } else {
        serde_json::from_str(config_json).map_err(|e| JsError::new(&e.to_string()))?
    };

    match tracematch::compare_routes(&sig1, &sig2, &config) {
        Some(result) => {
            let val =
                serde_wasm_bindgen::to_value(&result).map_err(|e| JsError::new(&e.to_string()))?;
            Ok(val)
        }
        None => Ok(JsValue::NULL),
    }
}

/// Group route signatures into clusters of similar routes.
///
/// Returns array of RouteGroup objects.
#[wasm_bindgen(js_name = "groupRoutes")]
pub fn group_routes(signatures_json: &str, config_json: &str) -> Result<JsValue, JsError> {
    let sigs: Vec<tracematch::RouteSignature> =
        serde_json::from_str(signatures_json).map_err(|e| JsError::new(&e.to_string()))?;

    let config: tracematch::MatchConfig = if config_json.is_empty() || config_json == "{}" {
        tracematch::MatchConfig::default()
    } else {
        serde_json::from_str(config_json).map_err(|e| JsError::new(&e.to_string()))?
    };

    let groups = tracematch::group_signatures(&sigs, &config);

    let val = serde_wasm_bindgen::to_value(&groups).map_err(|e| JsError::new(&e.to_string()))?;
    Ok(val)
}

/// Detect frequent sections across multiple GPS tracks.
///
/// tracks_json: `[["activity_id", [GpsPoint, ...]], ...]`
/// sport_types_json: `{"activity_id": "Ride", ...}`
/// groups_json: result from groupRoutes
/// config_json: optional SectionConfig JSON
///
/// Returns array of FrequentSection objects.
#[wasm_bindgen(js_name = "detectSections")]
pub fn detect_sections(
    tracks_json: &str,
    sport_types_json: &str,
    groups_json: &str,
    config_json: &str,
) -> Result<JsValue, JsError> {
    let tracks: Vec<(String, Vec<tracematch::GpsPoint>)> =
        serde_json::from_str(tracks_json).map_err(|e| JsError::new(&e.to_string()))?;

    let sport_types: std::collections::HashMap<String, String> =
        serde_json::from_str(sport_types_json).map_err(|e| JsError::new(&e.to_string()))?;

    let groups: Vec<tracematch::RouteGroup> =
        serde_json::from_str(groups_json).map_err(|e| JsError::new(&e.to_string()))?;

    let config: tracematch::SectionConfig = if config_json.is_empty() || config_json == "{}" {
        tracematch::SectionConfig::default()
    } else {
        serde_json::from_str(config_json).map_err(|e| JsError::new(&e.to_string()))?
    };

    let ms = tracematch::detect_sections_multiscale(&tracks, &sport_types, &groups, &config);

    let val =
        serde_wasm_bindgen::to_value(&ms.sections).map_err(|e| JsError::new(&e.to_string()))?;
    Ok(val)
}

/// Find known sections within a GPS route.
///
/// Returns array of SectionMatch objects sorted by start_index.
#[wasm_bindgen(js_name = "findSectionsInRoute")]
pub fn find_sections_in_route(
    route_json: &str,
    sections_json: &str,
    config_json: &str,
) -> Result<JsValue, JsError> {
    let route: Vec<tracematch::GpsPoint> =
        serde_json::from_str(route_json).map_err(|e| JsError::new(&e.to_string()))?;

    let sections: Vec<tracematch::FrequentSection> =
        serde_json::from_str(sections_json).map_err(|e| JsError::new(&e.to_string()))?;

    let config: tracematch::SectionConfig = if config_json.is_empty() || config_json == "{}" {
        tracematch::SectionConfig::default()
    } else {
        serde_json::from_str(config_json).map_err(|e| JsError::new(&e.to_string()))?
    };

    let matches = tracematch::find_sections_in_route(&route, &sections, &config);

    let val = serde_wasm_bindgen::to_value(&matches).map_err(|e| JsError::new(&e.to_string()))?;
    Ok(val)
}
