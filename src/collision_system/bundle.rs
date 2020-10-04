use crate::geo::*;
use std::sync::Arc;

pub struct Bundle {
    data: Vec<Arc<Box<dyn HasGeometry>>>,
    pub bounding_volume: Geo,
}

impl Bundle {
    pub fn new(bounding_volume: Geo) -> Bundle {
        Bundle {
            data: Vec::new(),
            bounding_volume,
        }
    }

    pub fn insert(&mut self, has_geo: Arc<Box<dyn HasGeometry>>) {
        self.data.push(has_geo);
    }

    pub fn update(&mut self, _frame_time: Float) -> Vec<Arc<Box<dyn HasGeometry>>> {
        unimplemented!();
        // update everything, that is movable
        // put everything, that has changed position into updated
        // delete everything that has updated and left the bounding volume
    }
}
