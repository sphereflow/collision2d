use super::bundle::Bundle;
use crate::geo::*;
use std::sync::Arc;

pub struct SimpleCollisionSystem {
    _bounding_rect: Rect,
    bundles: Vec<Bundle>,
}

impl SimpleCollisionSystem {
    pub fn new(_bounding_rect: Rect) -> SimpleCollisionSystem {
        SimpleCollisionSystem {
            _bounding_rect,
            bundles: Vec::new(),
        }
    }

    pub fn insert(&mut self, has_geo: Arc<Box<dyn HasGeometry>>) {
        for bundle in self.bundles.iter_mut() {
            if has_geo.get_geometry().does_overlap(&bundle.bounding_volume) {
                bundle.insert(has_geo.clone());
            }
        }
    }

    pub fn update(&mut self, _frame_time: Float) {}
}
