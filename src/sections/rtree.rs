//! R-tree indexed point types and spatial query utilities.

use crate::GpsPoint;
use rstar::{AABB, PointDistance, RTree, RTreeObject};

/// A GPS point with its index for R-tree queries
#[derive(Debug, Clone, Copy)]
pub struct IndexedPoint {
    pub idx: usize,
    latitude_m: f64,
    longitude_m: f64,
}

impl RTreeObject for IndexedPoint {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_point([self.latitude_m, self.longitude_m])
    }
}

impl PointDistance for IndexedPoint {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dlat = self.latitude_m - point[0];
        let dlng = self.longitude_m - point[1];
        dlat * dlat + dlng * dlng
    }
}

/// A metre-projected index of points on one local line.
pub struct PointIndex {
    tree: RTree<IndexedPoint>,
    longitude_scale: f64,
}

impl PointIndex {
    /// Return the nearest point index and its distance in metres.
    pub fn nearest(&self, coordinates: &[f64; 2]) -> Option<(usize, f64)> {
        let projected = [
            coordinates[0] * 111_000.0,
            coordinates[1] * self.longitude_scale,
        ];
        let point = self.tree.nearest_neighbor(&projected)?;
        Some((point.idx, point.distance_2(&projected).sqrt()))
    }
}

/// Build an index using the latitude of the line being matched.
pub fn build_rtree_at(points: &[GpsPoint], reference_latitude: f64) -> PointIndex {
    let longitude_scale = 111_000.0 * reference_latitude.to_radians().cos();
    let indexed: Vec<IndexedPoint> = points
        .iter()
        .enumerate()
        .map(|(i, p)| IndexedPoint {
            idx: i,
            latitude_m: p.latitude * 111_000.0,
            longitude_m: p.longitude * longitude_scale,
        })
        .collect();
    PointIndex {
        tree: RTree::bulk_load(indexed),
        longitude_scale,
    }
}

/// Build an index in metres for a line's own points.
pub fn build_rtree(points: &[GpsPoint]) -> PointIndex {
    let reference_latitude = points.first().map_or(0.0, |point| point.latitude);
    build_rtree_at(points, reference_latitude)
}

#[cfg(test)]
#[path = "tests/rtree.rs"]
mod tests;
