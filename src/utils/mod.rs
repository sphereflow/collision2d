extern crate nalgebra as na;
use na::{distance, Matrix2, Point2, Unit, Vector2};

pub type Float = f64;
pub type P2 = Point2<Float>;
pub type V2 = Vector2<Float>;
pub type U2 = Unit<V2>;
pub type Normal = Unit<V2>;
pub const EPSILON: Float = 0.000_001;

pub struct OneOrTwo<T: Copy + Clone> {
    items: (T, Option<T>),
    iter_ix: usize,
}

impl<T: Copy + Clone> OneOrTwo<T> {
    pub fn new(item: T) -> OneOrTwo<T> {
        OneOrTwo {
            items: (item, None),
            iter_ix: 0,
        }
    }

    pub fn to_vec(&self) -> Vec<T> {
        if let Some(item_b) = self.items.1 {
            vec![self.items.0, item_b]
        } else {
            vec![self.items.0]
        }
    }

    pub fn into_vec(self) -> Vec<T> {
        if let Some(item_b) = self.items.1 {
            vec![self.items.0, item_b]
        } else {
            vec![self.items.0]
        }
    }

    pub fn add(&mut self, item: T) {
        if self.items.1.is_none() {
            self.items.1 = Some(item);
        }
    }

    pub fn mappend(&mut self, other: OneOrTwo<T>) {
        self.add(other.items.0);
    }

    pub fn is_full(&self) -> bool {
        self.items.1.is_some()
    }

    pub fn get_items(&self) -> Option<(T, T)> {
        if let (item_a, Some(item_b)) = self.items {
            Some((item_a, item_b))
        } else {
            None
        }
    }

    pub fn get_first(&self) -> T {
        self.items.0
    }

    pub fn get_second(&self) -> Option<T> {
        self.items.1
    }

    pub fn swap(&mut self) {
        if let Some(two) = self.items.1 {
            let new_two = self.items.0;
            self.items.0 = two;
            self.items.1 = Some(new_two);
        }
    }
}

impl<T: Copy + Clone> Iterator for OneOrTwo<T> {
    type Item = T;
    fn next(&mut self) -> Option<Self::Item> {
        let res = match self.iter_ix {
            0 => Some(self.items.0),
            1 => self.items.1,
            _ => None,
        };
        self.iter_ix += 1;
        res
    }
}

pub fn between(num: Float, a: Float, b: Float) -> bool {
    (num >= a) && (num <= b)
}

pub fn smallest_positive_value(a: Float, b: Float) -> Option<Float> {
    if a >= 0. {
        if b >= 0. {
            Some(a.min(b))
        } else {
            Some(a)
        }
    } else if b >= 0. {
        Some(b)
    } else {
        None
    }
}

pub fn first<A, B>((a, _): (A, B)) -> A {
    a
}

pub enum Which {
    A,
    B,
}

pub fn nearest_option(
    p: &P2,
    ova: Option<Vec<P2>>,
    ovb: Option<Vec<P2>>,
) -> Option<(P2, Vec<P2>, Which)> {
    let oa: Option<(P2, Vec<P2>)> = ova.and_then(|v| {
        let mut min_distance = Float::MAX;
        let mut res = None;
        for a in v.iter() {
            let dist = distance(p, a);
            if dist < min_distance {
                min_distance = dist;
                res = Some(*a);
            }
        }
        res.map(|p| (p, v))
    });

    let ob: Option<(P2, Vec<P2>)> = ovb.and_then(|v| {
        let mut min_distance = Float::MAX;
        let mut res = None;
        for b in v.iter() {
            let dist = distance(p, &b);
            if dist < min_distance {
                min_distance = dist;
                res = Some(*b);
            }
        }
        res.map(|p| (p, v))
    });

    if let Some((a, va)) = oa {
        if let Some((b, vb)) = ob {
            if distance(p, &a) < distance(p, &b) {
                Some((a, va, Which::A))
            } else {
                Some((b, vb, Which::B))
            }
        } else {
            Some((a, va, Which::A))
        }
    } else {
        None
    }
}

pub fn farthest_option(
    p: &P2,
    ova: Option<Vec<P2>>,
    ovb: Option<Vec<P2>>,
) -> Option<(P2, Vec<P2>, Which)> {
    let oa: Option<(P2, Vec<P2>)> = ova.and_then(|v| {
        let mut max_distance = 0.;
        let mut res = None;
        for a in v.iter() {
            let dist = distance(p, &a);
            if dist > max_distance {
                max_distance = dist;
                res = Some(*a);
            }
        }
        res.map(|p| (p, v))
    });

    let ob: Option<(P2, Vec<P2>)> = ovb.and_then(|v| {
        let mut max_distance = 0.;
        let mut res = None;
        for b in v.iter() {
            let dist = distance(p, &b);
            if dist > max_distance {
                max_distance = dist;
                res = Some(*b);
            }
        }
        res.map(|p| (p, v))
    });

    if let Some((a, va)) = oa {
        if let Some((b, vb)) = ob {
            if distance(p, &a) > distance(p, &b) {
                Some((a, va, Which::A))
            } else {
                Some((b, vb, Which::B))
            }
        } else {
            Some((a, va, Which::A))
        }
    } else {
        None
    }
}

pub fn inverse(mat: &Matrix2<Float>) -> Option<Matrix2<Float>> {
    let a = mat.index(0);
    // b <==> c because storage in Matrix2 is column major
    let c = mat.index(1);
    let b = mat.index(2);
    let d = mat.index(3);
    let det = a * d - b * c;
    if det == 0.0 {
        return None;
    }
    let idet = 1.0 / det;
    Some(Matrix2::new(d * idet, -b * idet, -c * idet, a * idet))
}
