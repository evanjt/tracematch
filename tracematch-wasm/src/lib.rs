use wasm_bindgen::prelude::*;

#[wasm_bindgen(start)]
pub fn init() {
    console_error_panic_hook::set_once();
}

/// Adapter that turns a JS `Function(phase: string, current: number, total: number)`
/// into the `&mut dyn FnMut` the grouping path takes.
///
/// The library already batches its progress calls (~100 per phase), so
/// every emission from Rust calls into JS exactly once. Only grouping
/// reports progress: the unified detector's batch entry points emit
/// nothing, so the site's bar is indeterminate while it runs.
struct JsProgressCallback {
    js_fn: js_sys::Function,
}

impl JsProgressCallback {
    fn new(js_fn: js_sys::Function) -> Self {
        Self { js_fn }
    }

    fn emit(&self, phase: &str, current: u32, total: u32) {
        let this = JsValue::NULL;
        let _ = self.js_fn.call3(
            &this,
            &JsValue::from_str(phase),
            &JsValue::from_f64(current as f64),
            &JsValue::from_f64(total as f64),
        );
    }
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

/// Group route signatures with a progress callback.
///
/// `progress_cb` is called as `progress_cb(phase, current, total)` ~100
/// times across the pair-comparison phase. Phase string is
/// `comparing_pairs` (matches `tracematch::GROUPING_PHASE_COMPARING`).
#[wasm_bindgen(js_name = "groupRoutesWithProgress")]
pub fn group_routes_with_progress(
    signatures_json: &str,
    config_json: &str,
    progress_cb: &js_sys::Function,
) -> Result<JsValue, JsError> {
    let sigs: Vec<tracematch::RouteSignature> =
        serde_json::from_str(signatures_json).map_err(|e| JsError::new(&e.to_string()))?;

    let config: tracematch::MatchConfig = if config_json.is_empty() || config_json == "{}" {
        tracematch::MatchConfig::default()
    } else {
        serde_json::from_str(config_json).map_err(|e| JsError::new(&e.to_string()))?
    };

    let cb = JsProgressCallback::new(progress_cb.clone());
    let groups = tracematch::group_signatures_with_progress(&sigs, &config, &mut |p, c, t| {
        cb.emit(p, c, t);
    });

    let val = serde_wasm_bindgen::to_value(&groups).map_err(|e| JsError::new(&e.to_string()))?;
    Ok(val)
}

/// Detect frequent sections across multiple GPS tracks.
///
/// tracks_json:      `[["activity_id", [GpsPoint, ...]], ...]`
/// seconds_json:     `[[f64, ...], ...]`, one row per track, elapsed
///                   seconds per point. Empty (`""` or `"[]"`) is
///                   supported: the lift veto then rests on geometry.
/// sport_types_json: `{"activity_id": "Ride", ...}`
/// config_json:      optional SectionConfig JSON
///
/// Returns `{ sections: FrequentSection[], boundaries: BoundaryRecord[] }`.
/// The boundaries carry the reason behind every cut and every candidate
/// that backed off, which is what the site renders to explain itself.
#[wasm_bindgen(js_name = "detectSectionsUnified")]
pub fn detect_sections_unified(
    tracks_json: &str,
    seconds_json: &str,
    sport_types_json: &str,
    config_json: &str,
) -> Result<JsValue, JsError> {
    let tracks: Vec<(String, Vec<tracematch::GpsPoint>)> =
        serde_json::from_str(tracks_json).map_err(|e| JsError::new(&e.to_string()))?;

    let seconds_owned: Vec<Vec<f64>> = if seconds_json.is_empty() || seconds_json == "[]" {
        Vec::new()
    } else {
        serde_json::from_str(seconds_json).map_err(|e| JsError::new(&e.to_string()))?
    };
    let seconds: Vec<&[f64]> = seconds_owned.iter().map(|s| s.as_slice()).collect();

    let sport_types: std::collections::HashMap<String, String> =
        serde_json::from_str(sport_types_json).map_err(|e| JsError::new(&e.to_string()))?;

    let config: tracematch::SectionConfig = if config_json.is_empty() || config_json == "{}" {
        tracematch::SectionConfig::default()
    } else {
        serde_json::from_str(config_json).map_err(|e| JsError::new(&e.to_string()))?
    };

    let detection = tracematch::detect_sections_unified_explained(
        &tracks,
        &seconds,
        &sport_types,
        &config,
        &tracematch::Tunables::DEFAULT,
    );

    // `UnifiedDetection` carries no serde derive, so mirror it here.
    #[derive(serde::Serialize)]
    #[serde(rename_all = "camelCase")]
    struct Detection {
        sections: Vec<tracematch::FrequentSection>,
        boundaries: Vec<tracematch::BoundaryRecord>,
    }

    let out = Detection {
        sections: detection.sections,
        boundaries: detection.boundaries,
    };

    let val = serde_wasm_bindgen::to_value(&out).map_err(|e| JsError::new(&e.to_string()))?;
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
