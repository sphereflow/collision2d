use crate::geo::traits::HasGeometry;
use std::cmp::{Ord, Ordering, Reverse};
use std::collections::BinaryHeap;
use std::sync::{Arc, Mutex};

type GeoMut = Arc<Mutex<Box<dyn HasGeometry>>>;
pub struct Collision(GeoMut, GeoMut);

pub struct CollisionQueue {
    queue: BinaryHeap<Reverse<Collision>>,
}

impl PartialEq for Collision {
    fn eq(&self, other: &Collision) -> bool {
        let Collision(la, lb) = self;
        let Collision(lc, ld) = other;
        let a = la.lock().unwrap().get_geometry();
        let b = lb.lock().unwrap().get_geometry();
        let c = lc.lock().unwrap().get_geometry();
        let d = ld.lock().unwrap().get_geometry();
        match (a.time_of_collision(&b), c.time_of_collision(&d)) {
            (Some(toca), Some(tocb)) => toca == tocb,
            _ => false,
        }
    }
}

impl PartialOrd for Collision {
    fn partial_cmp(&self, other: &Collision) -> Option<Ordering> {
        let Collision(la, lb) = self;
        let Collision(lc, ld) = other;
        let a = la.lock().unwrap().get_geometry();
        let b = lb.lock().unwrap().get_geometry();
        let c = lc.lock().unwrap().get_geometry();
        let d = ld.lock().unwrap().get_geometry();
        match (a.time_of_collision(&b), c.time_of_collision(&d)) {
            (Some(toca), Some(tocb)) => toca.partial_cmp(&tocb),
            _ => None,
        }
    }
}

impl Eq for Collision {}

impl Ord for Collision {
    fn cmp(&self, other: &Collision) -> Ordering {
        let Collision(la, lb) = self;
        let Collision(lc, ld) = other;
        let a = la.lock().unwrap().get_geometry();
        let b = lb.lock().unwrap().get_geometry();
        let c = lc.lock().unwrap().get_geometry();
        let d = ld.lock().unwrap().get_geometry();
        match (a.time_of_collision(&b), c.time_of_collision(&d)) {
            (Some(toca), Some(tocb)) if toca < tocb => Ordering::Less,
            (Some(toca), Some(tocb)) if toca > tocb => Ordering::Greater,
            (Some(_), Some(_)) => Ordering::Equal,
            (Some(_), None) => Ordering::Less,
            (None, Some(_)) => Ordering::Greater,
            (None, None) => Ordering::Equal,
        }
    }
}

impl CollisionQueue {
    pub fn new() -> CollisionQueue {
        CollisionQueue {
            queue: BinaryHeap::new(),
        }
    }

    pub fn insert(&mut self, col: Collision) {
        self.queue.push(Reverse(col));
    }
}

impl Default for CollisionQueue {
    fn default() -> Self {
        Self::new()
    }
}

impl Iterator for CollisionQueue {
    type Item = Collision;
    fn next(&mut self) -> Option<Self::Item> {
        self.queue.pop().map(|rev| {
            let Reverse(result) = rev;
            result
        })
    }
}
