extern crate nalgebra as na;

use super::*;
use na::geometry::{Point, Point2};
use na::{distance, distance_squared, Unit, Vector2};
use rand::distributions::{Distribution, Standard};
use rand::Rng;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct LineSegment {
    a: P2,
    b: P2,
    normal: Normal,
    direction: U2,
}

impl LineSegment {
    pub fn from_ab(a: P2, b: P2) -> LineSegment {
        let direction = Unit::new_normalize(b - a);
        let normal = Unit::new_normalize(Vector2::new(-direction.y, direction.x));
        LineSegment {
            a,
            b,
            direction,
            normal,
        }
    }

    pub fn get_a(&self) -> P2 {
        self.a
    }

    pub fn set_a(&mut self, a: P2) {
        self.a = a;
        self.direction = Unit::new_normalize(self.b - self.a);
        self.normal = Unit::new_normalize(Vector2::new(-self.direction.y, self.direction.x));
    }

    pub fn shift(&mut self, v: V2) {
        self.a += v;
        self.b += v;
    }

    pub fn set_b(&mut self, b: P2) {
        self.b = b;
        self.direction = Unit::new_normalize(self.b - self.a);
        self.normal = Unit::new_normalize(Vector2::new(-self.direction.y, self.direction.x));
    }

    pub fn get_b(&self) -> P2 {
        self.b
    }

    pub fn get_normal(&self) -> Normal {
        self.normal
    }

    pub fn get_direction(&self) -> U2 {
        self.direction
    }

    pub fn set_direction(&mut self, direction: V2) {
        self.b = self.a + direction;
        self.direction = Unit::new_normalize(direction);
        // normal is rotated counter clockwise
        self.normal = Unit::new_normalize(Vector2::new(-direction.y, direction.x))
    }

    pub fn shorten_towards_b(&mut self, factor: Float) {
        let ab = self.b - self.a;
        self.a += ab * (1.0 - factor);
    }

    pub fn length(&self) -> Float {
        distance(&self.a, &self.b)
    }

    pub fn length_sq(&self) -> Float {
        distance_squared(&self.a, &self.b)
    }

    pub fn eval_at_r(&self, r: Float) -> P2 {
        let ab = self.a - self.b;
        self.a + r * ab
    }
}

impl HasOrigin for LineSegment {
    fn get_origin(&self) -> P2 {
        ((self.a.coords + self.b.coords) * 0.5).into()
    }
    fn set_origin(&mut self, origin: P2) {
        let dir = self.b - self.a;
        let old_pos = self.get_origin();
        self.a = Point::from(self.a + origin.coords - old_pos);
        self.b = self.a + dir;
    }
}

impl Rotate for LineSegment {
    // self.normal is the x axis ; self.direction is the y_axis
    fn set_rotation(&mut self, x_axis: &V2) {
        self.normal = Unit::new_normalize(*x_axis);
        self.direction = Unit::new_unchecked(V2::new(-self.normal.y, self.normal.x));
        // inverse of the old rotation
        let disth = distance(&self.a, &self.b) * 0.5;
        let origin = self.get_origin();
        self.a = origin - self.direction.into_inner() * disth;
        self.b = origin + self.direction.into_inner() * disth;
    }
    fn get_rotation(&self) -> V2 {
        self.normal.into_inner()
    }
}

impl Scale for LineSegment {
    /// scales while a stays fixed
    fn scale(&mut self, scale_x: Float, scale_y: Float) {
        let mut ab = self.b - self.a;
        ab.x *= scale_x;
        ab.y *= scale_y;
        self.b = self.a + ab;
    }
    fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
        self.a.scale_position(scale_x, scale_y);
        self.b.scale_position(scale_x, scale_y);
    }
}

impl ReflectOn<Line> for LineSegment {
    fn reflect_on_normal(&self, line: &Line) -> Option<(LineSegment, V2)> {
        match self.intersect(line) {
            Some(i) => {
                let n = self.normal.into_inner();
                let dist = line.distance(&Point2::from(self.b - i));
                let new_b = self.b - 2.0 * dist * n;
                Some((LineSegment::from_ab(i, new_b), n))
            }
            None => None,
        }
    }
}

impl ReflectOn<Ray> for LineSegment {
    fn reflect_on_normal_intersect(&self, ray: &Ray) -> Option<(LineSegment, V2, P2)> {
        match self.intersect(ray) {
            Some(intersection) => {
                let mut n = ray.get_normal().into_inner();
                if n.dot(&self.get_direction()) < 0.0 {
                    n = -n;
                }
                let line: Line = ray.into();
                let dist = line.distance(&self.b);
                let new_b = self.b - 2.0 * dist * n;
                Some((LineSegment::from_ab(intersection, new_b), n, intersection))
            }
            None => None,
        }
    }
}

impl ReflectOn<LineSegment> for LineSegment {
    fn reflect_on_normal_intersect(&self, ls: &LineSegment) -> Option<(LineSegment, V2, P2)> {
        match self.intersect(ls) {
            Some(intersection) => {
                let mut n = ls.normal.into_inner();
                if n.dot(&self.get_direction()) < 0.0 {
                    n = -n;
                }
                let line: Line = ls.into();
                let dist = line.distance(&self.b);
                let new_b = self.b - 2.0 * dist * n;
                Some((LineSegment::from_ab(intersection, new_b), n, intersection))
            }
            None => None,
        }
    }
}

impl Intersect<Line> for LineSegment {
    type Intersection = P2;

    fn intersect(&self, l: &Line) -> Option<P2> {
        let sline: Line = self.into();
        if let Some((v, r, _)) = sline.intersect(l) {
            if r < 0.0 || r > 1.0 {
                None
            } else {
                Some(v)
            }
        } else {
            None
        }
    }
}

impl Intersect<Ray> for LineSegment {
    type Intersection = P2;

    fn intersect(&self, ray: &Ray) -> Option<P2> {
        ray.intersect(self).map(|(p, _)| p)
    }
}

impl Intersect<LineSegment> for LineSegment {
    type Intersection = P2;

    fn intersect(&self, other: &LineSegment) -> Option<P2> {
        let sline: Line = self.into();
        let oline: Line = other.into();
        if let Some((v, r, s)) = sline.intersect(&oline) {
            if (r < 0.0 || r > 1.0) || (s < 0.0 || s > 1.0) {
                None
            } else {
                Some(v)
            }
        } else {
            None
        }
    }
}

impl ClosestPoint for LineSegment {
    fn closest_point_to(&self, p: &P2) -> P2 {
        let pa = p - self.a;
        let d = self.direction.normalize();
        let ra = d.dot(&pa);
        let rb = d.dot(&(p - self.b));
        if ra < 0. {
            self.a
        } else if rb > 0. {
            self.b
        } else {
            self.a + ra * d
        }
    }
}

impl Distance for LineSegment {
    fn distance(&self, p: &P2) -> Float {
        let normal = self.get_normal();
        let amp = self.a - p;
        let dist_ap = amp.norm();
        let dist_bp = distance(&self.b, p);
        if amp.dot(&(self.b - self.a)) > 0.0 {
            return dist_ap;
        }
        if (self.b - p).dot(&(self.a - self.b)) > 0.0 {
            return dist_bp;
        }
        let dist_line = amp.dot(&normal).abs();
        // println!("dist_line: {}, normal: {}, amp: {}, b - a: {}", dist_line, normal, amp, self.b - self.a);
        if dist_line < dist_ap {
            dist_line.min(dist_bp)
        } else {
            dist_ap.min(dist_bp)
        }
    }
}

impl Distribution<LineSegment> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> LineSegment {
        let a = rng.gen();
        let b = rng.gen();
        LineSegment::from_ab(a, b)
    }
}

use super::circle::Circle;
impl ReflectOn<Circle> for LineSegment {
    fn reflect_on_normal(&self, circle: &Circle) -> Option<(LineSegment, V2)> {
        match circle.intersect(self) {
            Some(ooti) => {
                let i = ooti.get_first();
                let n = (i - circle.origin).normalize();
                let dist = n.dot(&(self.b - i));
                let new_b = self.get_b() - 2.0 * dist * n;
                Some((LineSegment::from_ab(i, new_b), n))
            }
            None => None,
        }
    }
}

impl ReflectOn<Rect> for LineSegment {}
impl ReflectOn<AABB> for LineSegment {}
impl ReflectOn<MCircle> for LineSegment {}

impl CanCollideWith<Line> for LineSegment {}
impl CanCollideWith<Ray> for LineSegment {}
impl CanCollideWith<LineSegment> for LineSegment {}
impl CanCollideWith<Circle> for LineSegment {}
impl CanCollideWith<Rect> for LineSegment {}
impl CanCollideWith<AABB> for LineSegment {}
impl CanCollideWith<MCircle> for LineSegment {}

impl GeoT for LineSegment {}
