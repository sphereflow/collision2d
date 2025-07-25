extern crate nalgebra as na;

use super::*;

#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct Circle {
    pub origin: P2,
    pub radius: Float,
}

impl Intersect<Line> for Circle {
    // r and s
    type Intersection = (Float, Float);

    fn intersect(&self, l: &Line) -> Option<(Float, Float)> {
        let dist = l.distance(&self.origin);
        if dist <= self.radius {
            let co = self.origin - l.origin;
            let ldn = l.get_direction();
            let mut n = l.get_normal();
            if n.dot(&co) < 0.0 {
                n = -n;
            }
            let mid_point = co - n.into_inner() * dist;
            let offset = Float::sqrt(self.radius.powi(2) - dist.powi(2));
            let r;
            let s;
            if ldn.x > ldn.y {
                // project u, v on the x-axis
                let mid_point_projection = mid_point.x / l.get_direction().x;
                r = mid_point_projection + offset;
                s = mid_point_projection - offset;
            } else {
                // project u, v on the y-axis
                let mid_point_projection = mid_point.y / l.get_direction().y;
                r = mid_point_projection + offset;
                s = mid_point_projection - offset;
            }
            Some((r, s))
        } else {
            None
        }
    }

    fn get_extra_data_intersect(&self, l: &Line) -> Vec<Geo> {
        let mut ret = Vec::new();
        let dist = l.distance(&self.origin);
        let co = self.origin - l.origin;
        let mut n = l.get_normal().into_inner();
        if n.dot(&co) < 0.0 {
            n = -n;
        }
        let mid_point = l.origin + co - n * dist;
        ret.push(Geo::GeoPoint(P2::new(mid_point.x, mid_point.y)));
        ret
    }
}

impl Intersect<LineSegment> for Circle {
    // r and s
    type Intersection = OneOrTwo<P2>;

    /// returns the intersection points in order e. g. closest first
    fn intersect(&self, ls: &LineSegment) -> Option<OneOrTwo<P2>> {
        let line: Line = ls.into();
        if let Some((r, s)) = self.intersect(&line) {
            if between(r, 0.0, 1.0) {
                let mut res = OneOrTwo::new(line.eval_at_r(r));
                if between(s, 0.0, 1.0) {
                    res.add(line.eval_at_r(s));
                    if s < r {
                        res.swap();
                    }
                }
                return Some(res);
            } else if between(s, 0.0, 1.0) {
                return Some(OneOrTwo::new(line.eval_at_r(s)));
            }
        }
        None
    }

    fn get_extra_data_intersect(&self, ls: &LineSegment) -> Vec<Geo> {
        let line: Line = ls.into();
        self.get_extra_data_intersect(&line)
    }
}

impl Intersect<Circle> for Circle {
    type Intersection = OneOrTwo<P2>;
    fn intersect(&self, other: &Circle) -> Option<OneOrTwo<P2>> {
        let c: Float = distance(&self.origin, &other.origin);
        if c > (self.radius + other.radius) {
            return None;
        }
        let a = (self.radius.powi(2) + other.radius.powi(2)) / (2. * c) + 0.5 * c;
        let dist = (self.radius.powi(2) - a.powi(2)).sqrt();
        let oon = (self.origin - other.origin).normalize();
        let n: V2 = V2::new(-oon.y, oon.x);
        let mut ret = OneOrTwo::new(self.origin + a * oon + n * dist);
        if a > 0. {
            ret.add(self.origin - a * oon + n * dist)
        }
        Some(ret)
    }
}

impl Intersect<Rect> for Circle {
    type Intersection = Vec<P2>;

    fn intersect(&self, rect: &Rect) -> Option<Vec<P2>> {
        rect.intersect(self)
    }
}

impl HasOrigin for Circle {
    fn get_origin(&self) -> P2 {
        self.origin
    }
    fn set_origin(&mut self, origin: P2) {
        self.origin = origin;
    }
}

impl Scale for Circle {
    fn scale(&mut self, scale_x: Float, scale_y: Float) {
        self.radius *= (scale_x + scale_y) * 0.5;
    }
    fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
        self.origin.x *= scale_x;
        self.origin.y *= scale_y;
    }
}

impl Rotate for Circle {
    fn get_rotation(&self) -> Rot2 {
        Rot2::identity()
    }
    fn set_rotation(&mut self, _rotation: &Rot2) {}
}

impl Contains for Circle {
    fn contains(&self, p: &P2) -> bool {
        distance(&self.origin, p) <= self.radius
    }
}

impl ClosestPoint for Circle {
    fn closest_point_to(&self, p: &P2) -> P2 {
        if p == &self.origin {
            // any point on the outline suffices
            p + V2::new(1., 0.) * self.radius
        } else {
            self.origin + (p - self.origin).normalize() * self.radius
        }
    }
}

impl Mirror for Circle {
    fn mirror_x(&self) -> Self {
        *self
    }
    fn mirror_y(&self) -> Self {
        *self
    }
}

impl Distance for Circle {
    fn distance(&self, p: &P2) -> Float {
        distance(&self.origin, p) - self.radius
    }
}

impl Distribution<Circle> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Circle {
        Circle {
            origin: rng.gen(),
            radius: rng.gen(),
        }
    }
}

impl HasGeometry for Circle {
    fn get_geometry(&self) -> Geo {
        Geo::GeoCircle(*self)
    }
}

impl HasAabb for Circle {
    fn get_aabb(&self) -> Aabb {
        Aabb {
            origin: self.origin,
            width: self.radius * 2.,
            height: self.radius * 2.,
        }
    }
}

// a circle can not be reflected on anything
impl ReflectOn<Line> for Circle {}
impl ReflectOn<Ray> for Circle {}
impl ReflectOn<LineSegment> for Circle {}
impl ReflectOn<Circle> for Circle {}
impl ReflectOn<Rect> for Circle {}
impl ReflectOn<Aabb> for Circle {}
impl ReflectOn<MCircle> for Circle {}

impl CanCollideWith<Line> for Circle {}
impl CanCollideWith<Ray> for Circle {}
impl CanCollideWith<LineSegment> for Circle {}
impl CanCollideWith<Circle> for Circle {}
impl CanCollideWith<Rect> for Circle {}
impl CanCollideWith<Aabb> for Circle {}
impl CanCollideWith<MCircle> for Circle {}

impl GeoT for Circle {}
