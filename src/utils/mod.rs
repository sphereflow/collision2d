extern crate nalgebra as na;
use na::{distance, Matrix2, Point2, Rotation2, Unit, Vector2};

pub type Float = f64;
pub type P2 = Point2<Float>;
pub type V2 = Vector2<Float>;
pub type U2 = Unit<V2>;
pub type Normal = Unit<V2>;
pub type Rot2 = Rotation2<Float>;
pub type Reflection = (P2, Normal);
pub type ClosestReflection = (Reflection, Vec<Reflection>);
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

    pub fn map<U, F>(self, func: F) -> OneOrTwo<U>
    where
        F: Fn(T) -> U,
        U: Copy + Clone,
    {
        OneOrTwo {
            items: (func(self.items.0), self.items.1.map(func)),
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

/// the smallest positive value is always in the first position
pub fn smallest_positive_sort(a: Float, b: Float) -> Option<(Float, Float)> {
    if a >= 0. {
        if b > a || b < 0. {
            Some((a, b))
        } else {
            Some((b, a))
        }
    } else if b >= 0. {
        Some((b, a))
    } else {
        None
    }
}

pub fn get_smallest_positive_by<F, T>(v: &mut Vec<T>, f: F) -> Option<T>
where
    F: Fn(&T) -> f64,
    T: Copy,
{
    v.sort_unstable_by(|a, b| {
        f(a).partial_cmp(&f(b))
            .expect("get_smallest_positive: sort_unstable_by failed")
    });
    for num in v {
        if f(&num) >= 0. {
            return Some(*num);
        }
    }
    None
}

pub fn first<A, B>((a, _): (A, B)) -> A {
    a
}

pub fn nearest_option<T: Iterator<Item = Reflection>, U: Iterator<Item = Reflection>>(
    p: &P2,
    ova: Option<T>,
    ovb: Option<U>,
) -> Option<ClosestReflection> {
    let oa: Option<ClosestReflection> = ova.and_then(|iter| {
        let mut min_distance = Float::MAX;
        let mut res = None;
        let v: Vec<Reflection> = iter.collect();
        for a in v.iter() {
            let dist = distance(p, &a.0);
            if dist < min_distance {
                min_distance = dist;
                res = Some(*a);
            }
        }
        res.map(|p| (p, v))
    });

    let ob: Option<ClosestReflection> = ovb.and_then(|iter| {
        let mut min_distance = Float::MAX;
        let mut res = None;
        let v: Vec<Reflection> = iter.collect();
        for b in v.iter() {
            let dist = distance(p, &b.0);
            if dist < min_distance {
                min_distance = dist;
                res = Some(*b);
            }
        }
        res.map(|p| (p, v))
    });

    match (oa, ob) {
        (Some((a, va)), Some((b, vb))) => {
            if distance(p, &a.0) < distance(p, &b.0) {
                Some((a, va))
            } else {
                Some((b, vb))
            }
        }
        (Some((a, va)), None) => Some((a, va)),
        (None, Some((b, vb))) => Some((b, vb)),
        (None, None) => None,
    }
}

pub fn farthest_option<T: Iterator<Item = Reflection>, U: Iterator<Item = Reflection>>(
    p: &P2,
    ova: Option<T>,
    ovb: Option<U>,
) -> Option<ClosestReflection> {
    let oa: Option<ClosestReflection> = ova.and_then(|iter| {
        let mut max_distance = 0.;
        let mut res = None;
        let v: Vec<Reflection> = iter.collect();
        for a in v.iter() {
            let dist = distance(p, &a.0);
            if dist > max_distance {
                max_distance = dist;
                res = Some(*a);
            }
        }
        res.map(|a| (a, v))
    });

    let ob: Option<ClosestReflection> = ovb.and_then(|iter| {
        let mut max_distance = 0.;
        let mut res = None;
        let v: Vec<Reflection> = iter.collect();
        for b in v.iter() {
            let dist = distance(p, &b.0);
            if dist > max_distance {
                max_distance = dist;
                res = Some(*b);
            }
        }
        res.map(|b| (b, v))
    });

    match (oa, ob) {
        (Some((a, va)), Some((b, vb))) => {
            if distance(p, &a.0) > distance(p, &b.0) {
                Some((a, va))
            } else {
                Some((b, vb))
            }
        }
        (Some((a, va)), None) => Some((a, va)),
        (None, Some((b, vb))) => Some((b, vb)),
        (None, None) => None,
    }
}

pub fn extend_opt_vec<T>(isa: Option<Vec<T>>, isb: Option<Vec<T>>) -> Option<Vec<T>> {
    let mut res;
    match (isa, isb) {
        (Some(va), Some(vb)) => {
            res = va;
            if res.is_empty() {
                if vb.is_empty() {
                    None
                } else {
                    Some(vb)
                }
            } else if vb.is_empty() {
                Some(res)
            } else {
                res.extend(vb.into_iter());
                Some(res)
            }
        }
        (Some(va), _) => {
            if !va.is_empty() {
                Some(va)
            } else {
                None
            }
        }
        (_, Some(vb)) => {
            if !vb.is_empty() {
                Some(vb)
            } else {
                None
            }
        }
        _ => None,
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

pub fn is_clockwise_points(p1: &V2, p2: &V2, p3: &V2) -> bool {
    let v21: V2 = p1 - p2;
    // counterclowkwise rotation of diff
    let perpendicular = V2::new(-v21.y, v21.x);
    (p3 - p2).dot(&perpendicular).is_sign_positive()
}

pub fn is_clockwise_directions(d1: &V2, d2: &V2) -> bool {
    let perpendicular = V2::new(-d1.y, d1.x);
    d2.dot(&perpendicular) < 0.
}

/// This function returns the projected points x-value
pub fn separation_axis_projection(origin: &P2, direction: &U2, p: &P2) -> Float {
    let rot = Rotation2::rotation_between(direction, &U2::new_unchecked(V2::new(0., 1.)));
    (rot * (p - origin)).x
}

/// This function returns the global coordinates of a local point with its rotation and origin
pub fn local_to_global(origin: &P2, rotation: &Rotation2<Float>, p: &V2) -> P2 {
    origin + rotation * p
}

pub fn quadratic_roots(a: Float, b: Float) -> Vec<Float> {
    let mah = -1. * a * 0.5;
    let discriminant = mah * mah - b;
    match discriminant {
        _ if discriminant.abs() == 0. => vec![mah],
        _ if discriminant > 0. => {
            let root = discriminant.sqrt();
            vec![mah - root, mah + root]
        }
        _ => vec![],
    }
}

pub fn cubic_roots(steps: u8, a: Float, b: Float, c: Float) -> Vec<Float> {
    let big_q = (a * a - 3. * b) / 9.;
    let big_r = (2. * a.powi(3) - 9. * a * b + 27. * c) / 54.;
    let theta;
    let x3;
    if big_r * big_r < big_q.powi(3) {
        theta = (big_r / big_q.powf(1.5)).acos();
        x3 = -2. * big_q.sqrt() * (theta / 3.).cos() - a / 3.;
    } else {
        let big_a = -big_r.signum() * (big_r.abs() + (big_r.powi(2) - big_q.powi(3)).sqrt()).cbrt();
        let big_b = if big_a != 0. { big_q / big_a } else { 0. };
        x3 = big_a + big_b - a / 3.;
    }
    let (mut u1, mut u2);
    let mut gamma = -x3;
    let mut alpha = a - gamma;
    let mut beta = b - gamma * alpha;
    let (mut delta1, mut delta2, mut delta3);
    let (mut q1, mut q2, mut q3);
    let (mut e1, mut e2, mut e3) = (0., 0., c - gamma * beta);
    for _ in 0..steps {
        u1 = alpha - gamma;
        u2 = beta - gamma * u1;
        q1 = e1;
        q2 = e2 - gamma * q1;
        q3 = e3 - gamma * q2;
        if u2 == 0. {
            delta3 = 0.;
        } else {
            delta3 = q3 / u2;
        }
        delta2 = q2 - u1 * delta3;
        delta1 = q1 - delta3;
        alpha += delta1;
        beta += delta2;
        gamma += delta3;
        e1 = a - gamma - alpha;
        e2 = b - alpha * gamma - beta;
        e3 = c - gamma * beta;
    }
    // solve the quadratic equation
    let mut res = quadratic_roots(alpha, beta);
    res.push(x3);
    res
}
