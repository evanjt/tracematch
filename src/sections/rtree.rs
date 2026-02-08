//! R-tree indexed point types and spatial query utilities.

use crate::GpsPoint;
use rstar::{AABB, PointDistance, RTree, RTreeObject};

/// A GPS point with its index for R-tree queries
#[derive(Debug, Clone, Copy)]
pub struct IndexedPoint {
    pub idx: usize,
    pub lat: f64,
    pub lng: f64,
}

impl RTreeObject for IndexedPoint {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_point([self.lat, self.lng])
    }
}

impl PointDistance for IndexedPoint {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dlat = self.lat - point[0];
        let dlng = self.lng - point[1];
        dlat * dlat + dlng * dlng
    }
}

/// Build R-tree from GPS points for efficient spatial queries
pub fn build_rtree(points: &[GpsPoint]) -> RTree<IndexedPoint> {
    let indexed: Vec<IndexedPoint> = points
        .iter()
        .enumerate()
        .map(|(i, p)| IndexedPoint {
            idx: i,
            lat: p.latitude,
            lng: p.longitude,
        })
        .collect();
    RTree::bulk_load(indexed)
}

