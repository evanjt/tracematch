use std::sync::Arc;
use tracematch::{DetectionPhase, DetectionProgressCallback};
use wasm_bindgen::prelude::*;

#[wasm_bindgen(start)]
pub fn init() {
    console_error_panic_hook::set_once();
}

/// Adapter that turns a JS `Function(phase: string, current: number, total: number)`
/// into a `DetectionProgressCallback`.
///
/// The library already batches `on_progress` calls (~100 per phase), so
/// every emission from Rust calls into JS exactly once — no extra
/// throttling needed here. WASM is single-threaded so the unsafe
/// Send+Sync bounds are sound; the lib's trait requires them because
/// the native parallel path uses rayon.
struct JsProgressCallback {
    js_fn: js_sys::Function,
    current: std::sync::atomic::AtomicU32,
    total: std::sync::atomic::AtomicU32,
    phase: std::sync::Mutex<String>,
}

// SAFETY: JsValue / js_sys::Function are !Send + !Sync because they wrap a
// JS-side reference. We're in WASM (single-threaded), and the
// `DetectionProgressCallback` trait's Send+Sync bound only exists for
// the native rayon path which we don't compile here.
unsafe impl Send for JsProgressCallback {}
unsafe impl Sync for JsProgressCallback {}

impl JsProgressCallback {
    fn new(js_fn: js_sys::Function) -> Self {
        Self {
            js_fn,
            current: std::sync::atomic::AtomicU32::new(0),
            total: std::sync::atomic::AtomicU32::new(0),
            phase: std::sync::Mutex::new(String::new()),
        }
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

impl DetectionProgressCallback for JsProgressCallback {
    fn on_phase(&self, phase: DetectionPhase, total: u32) {
        let phase_str = phase.as_str();
        *self.phase.lock().unwrap() = phase_str.to_string();
        self.total
            .store(total, std::sync::atomic::Ordering::Relaxed);
        self.current
            .store(0, std::sync::atomic::Ordering::Relaxed);
        self.emit(phase_str, 0, total);
    }

    fn on_progress(&self) {
        let current = self
            .current
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed)
            + 1;
        let total = self.total.load(std::sync::atomic::Ordering::Relaxed);
        let phase = self.phase.lock().unwrap().clone();
        self.emit(&phase, current, total);
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

/// Detect sections with a progress callback.
///
/// `progress_cb` is called as `progress_cb(phase, current, total)`. Phase
/// strings come from `DetectionPhase::as_str()`: `building_rtrees`,
/// `finding_overlaps`, `postprocessing`. Update batching (~100 per
/// phase) is done inside the library so this callback path is cheap.
#[wasm_bindgen(js_name = "detectSectionsWithProgress")]
pub fn detect_sections_with_progress(
    tracks_json: &str,
    sport_types_json: &str,
    groups_json: &str,
    config_json: &str,
    progress_cb: &js_sys::Function,
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

    let cb: Arc<dyn DetectionProgressCallback> =
        Arc::new(JsProgressCallback::new(progress_cb.clone()));
    let ms = tracematch::detect_sections_multiscale_with_progress(
        &tracks,
        &sport_types,
        &groups,
        &config,
        cb,
    );

    let val =
        serde_wasm_bindgen::to_value(&ms.sections).map_err(|e| JsError::new(&e.to_string()))?;
    Ok(val)
}

/// Detect sections via flow-graph analysis.
///
/// Builds a road/trail network from GPS traces by tracking cell-to-cell
/// flow, finds divergence points (junctions), and returns edges between
/// them as sections.
#[wasm_bindgen(js_name = "detectSectionsFlowGraph")]
pub fn detect_sections_flow_graph(
    tracks_json: &str,
    sport_types_json: &str,
    config_json: &str,
) -> Result<JsValue, JsError> {
    let tracks: Vec<(String, Vec<tracematch::GpsPoint>)> =
        serde_json::from_str(tracks_json).map_err(|e| JsError::new(&e.to_string()))?;

    let sport_types: std::collections::HashMap<String, String> =
        serde_json::from_str(sport_types_json).map_err(|e| JsError::new(&e.to_string()))?;

    let config: tracematch::SectionConfig = if config_json.is_empty() || config_json == "{}" {
        tracematch::SectionConfig::default()
    } else {
        serde_json::from_str(config_json).map_err(|e| JsError::new(&e.to_string()))?
    };

    let sections = tracematch::detect_sections_flow_graph(&tracks, &sport_types, &config);

    let val =
        serde_wasm_bindgen::to_value(&sections).map_err(|e| JsError::new(&e.to_string()))?;
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
