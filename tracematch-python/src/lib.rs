use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

/// A GPS coordinate with latitude, longitude, and optional elevation.
#[pyclass]
#[derive(Clone)]
pub struct GpsPoint {
    #[pyo3(get)]
    pub lat: f64,
    #[pyo3(get)]
    pub lon: f64,
    #[pyo3(get)]
    pub elevation: Option<f64>,
}

#[pymethods]
impl GpsPoint {
    #[new]
    #[pyo3(signature = (lat, lon, elevation=None))]
    fn new(lat: f64, lon: f64, elevation: Option<f64>) -> Self {
        Self {
            lat,
            lon,
            elevation,
        }
    }

    fn __repr__(&self) -> String {
        match self.elevation {
            Some(e) => format!(
                "GpsPoint(lat={}, lon={}, elevation={})",
                self.lat, self.lon, e
            ),
            None => format!("GpsPoint(lat={}, lon={})", self.lat, self.lon),
        }
    }
}

impl From<&GpsPoint> for tracematch::GpsPoint {
    fn from(p: &GpsPoint) -> Self {
        match p.elevation {
            Some(e) => tracematch::GpsPoint::with_elevation(p.lat, p.lon, e),
            None => tracematch::GpsPoint::new(p.lat, p.lon),
        }
    }
}

impl From<&tracematch::GpsPoint> for GpsPoint {
    fn from(p: &tracematch::GpsPoint) -> Self {
        Self {
            lat: p.latitude,
            lon: p.longitude,
            elevation: p.elevation,
        }
    }
}

/// Configuration for route matching algorithms.
#[pyclass]
#[derive(Clone)]
pub struct MatchConfig {
    inner: tracematch::MatchConfig,
}

#[pymethods]
impl MatchConfig {
    #[new]
    #[pyo3(signature = (
        perfect_threshold=15.0,
        zero_threshold=100.0,
        min_match_percentage=50.0,
        min_route_distance=500.0,
        max_distance_diff_ratio=0.30,
        endpoint_threshold=300.0,
        resample_count=50,
        resample_spacing_meters=50.0,
        min_resample_points=20,
        max_resample_points=200,
        simplification_tolerance=0.0001,
        max_simplified_points=100,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        perfect_threshold: f64,
        zero_threshold: f64,
        min_match_percentage: f64,
        min_route_distance: f64,
        max_distance_diff_ratio: f64,
        endpoint_threshold: f64,
        resample_count: u32,
        resample_spacing_meters: f64,
        min_resample_points: u32,
        max_resample_points: u32,
        simplification_tolerance: f64,
        max_simplified_points: u32,
    ) -> Self {
        Self {
            inner: tracematch::MatchConfig {
                perfect_threshold,
                zero_threshold,
                min_match_percentage,
                min_route_distance,
                max_distance_diff_ratio,
                endpoint_threshold,
                resample_count,
                resample_spacing_meters,
                min_resample_points,
                max_resample_points,
                simplification_tolerance,
                max_simplified_points,
            },
        }
    }
}

/// A simplified route signature for efficient matching.
#[pyclass]
#[derive(Clone)]
pub struct RouteSignature {
    inner: tracematch::RouteSignature,
}

#[pymethods]
impl RouteSignature {
    /// Create a route signature from raw GPS points.
    #[staticmethod]
    #[pyo3(signature = (activity_id, points, config=None))]
    fn from_points(
        activity_id: &str,
        points: Vec<GpsPoint>,
        config: Option<&MatchConfig>,
    ) -> PyResult<Self> {
        let rust_points: Vec<tracematch::GpsPoint> = points.iter().map(|p| p.into()).collect();
        let default_config = tracematch::MatchConfig::default();
        let cfg = config.map(|c| &c.inner).unwrap_or(&default_config);

        tracematch::RouteSignature::from_points(activity_id, &rust_points, cfg)
            .map(|inner| Self { inner })
            .ok_or_else(|| {
                PyValueError::new_err("Could not create signature (need >= 2 valid points)")
            })
    }

    #[getter]
    fn activity_id(&self) -> &str {
        &self.inner.activity_id
    }

    #[getter]
    fn total_distance(&self) -> f64 {
        self.inner.total_distance
    }

    #[getter]
    fn num_points(&self) -> usize {
        self.inner.points.len()
    }

    fn __repr__(&self) -> String {
        format!(
            "RouteSignature(id='{}', points={}, distance={:.0}m)",
            self.inner.activity_id,
            self.inner.points.len(),
            self.inner.total_distance
        )
    }
}

/// Result of comparing two routes.
#[pyclass]
#[derive(Clone)]
pub struct MatchResult {
    #[pyo3(get)]
    pub activity_id_1: String,
    #[pyo3(get)]
    pub activity_id_2: String,
    #[pyo3(get)]
    pub match_percentage: f64,
    #[pyo3(get)]
    pub direction: String,
    #[pyo3(get)]
    pub amd: f64,
}

#[pymethods]
impl MatchResult {
    fn __repr__(&self) -> String {
        format!(
            "MatchResult({}% {}, amd={:.1}m)",
            self.match_percentage as u32, self.direction, self.amd
        )
    }
}

impl From<tracematch::MatchResult> for MatchResult {
    fn from(r: tracematch::MatchResult) -> Self {
        Self {
            activity_id_1: r.activity_id_1,
            activity_id_2: r.activity_id_2,
            match_percentage: r.match_percentage,
            direction: r.direction,
            amd: r.amd,
        }
    }
}

/// A group of similar routes.
#[pyclass]
#[derive(Clone)]
pub struct RouteGroup {
    #[pyo3(get)]
    pub group_id: String,
    #[pyo3(get)]
    pub representative_id: String,
    #[pyo3(get)]
    pub activity_ids: Vec<String>,
    #[pyo3(get)]
    pub sport_type: String,
}

impl From<&tracematch::RouteGroup> for RouteGroup {
    fn from(g: &tracematch::RouteGroup) -> Self {
        Self {
            group_id: g.group_id.clone(),
            representative_id: g.representative_id.clone(),
            activity_ids: g.activity_ids.clone(),
            sport_type: g.sport_type.clone(),
        }
    }
}

#[pymethods]
impl RouteGroup {
    fn __repr__(&self) -> String {
        format!(
            "RouteGroup(id='{}', {} activities)",
            self.group_id,
            self.activity_ids.len()
        )
    }
}

/// Compare two route signatures and return a match result.
#[pyfunction]
#[pyo3(signature = (sig1, sig2, config=None))]
fn compare_routes(
    sig1: &RouteSignature,
    sig2: &RouteSignature,
    config: Option<&MatchConfig>,
) -> Option<MatchResult> {
    let default_config = tracematch::MatchConfig::default();
    let cfg = config.map(|c| &c.inner).unwrap_or(&default_config);

    tracematch::compare_routes(&sig1.inner, &sig2.inner, cfg).map(|r| r.into())
}

/// Group route signatures into clusters of similar routes.
#[pyfunction]
#[pyo3(signature = (signatures, config=None))]
fn group_routes(signatures: Vec<RouteSignature>, config: Option<&MatchConfig>) -> Vec<RouteGroup> {
    let sigs: Vec<tracematch::RouteSignature> =
        signatures.iter().map(|s| s.inner.clone()).collect();
    let default_config = tracematch::MatchConfig::default();
    let cfg = config.map(|c| &c.inner).unwrap_or(&default_config);

    let groups = tracematch::group_signatures(&sigs, cfg);
    groups.iter().map(|g| g.into()).collect()
}

#[pymodule]
#[pyo3(name = "tracematch")]
fn tracematch_python(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<GpsPoint>()?;
    m.add_class::<MatchConfig>()?;
    m.add_class::<RouteSignature>()?;
    m.add_class::<MatchResult>()?;
    m.add_class::<RouteGroup>()?;
    m.add_function(wrap_pyfunction!(compare_routes, m)?)?;
    m.add_function(wrap_pyfunction!(group_routes, m)?)?;
    Ok(())
}
