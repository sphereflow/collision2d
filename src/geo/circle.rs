extern crate nalgebra as na;
use super::aabb::AABB;
use super::line::*;
use super::line_segment::LineSegment;
use super::mcircle::MCircle;
use super::ray::Ray;
use super::rect::Rect;
use super::traits::*;
use super::Geo;
use crate::utils::*;
use na::{distance, Point2};
use rand::distributions::{Distribution, Standard};
use rand::Rng;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Circle {
    pub origin: P2,
    pub radius: Float,
}

impl Intersect<Line> for Circle {
    // r and s
    type Intersection = (Float, Float);

    fn intersect(&self, l: &Line) -> Option<(Float, Float)> {
        let dist = l.distance(self.origin);
        let dist_sq = dist.powi(2);
        let radius_sq = self.radius.powi(2);
        if dist_sq < radius_sq {
            let co = self.origin - l.origin;
            let ldn = l.get_direction().normalize();
            let mut n = l.get_normal().into_inner();
            if n.dot(&co) < 0.0 {
                n = -n;
            }
            let mid_point = co - n * dist;
            let offset = Float::sqrt(radius_sq - dist_sq);
            let r;
            let s;
            if ldn.x > ldn.y {
                // project u, v on the x-axis
                r = (mid_point.x + offset * ldn.x) / l.get_direction().x;
                s = (mid_point.x - offset * ldn.x) / l.get_direction().x;
            } else {
                // project u, v on the y-axis
                r = (mid_point.y + offset * ldn.y) / l.get_direction().y;
                s = (mid_point.y - offset * ldn.y) / l.get_direction().y;
            }
            Some((r, s))
        } else {
            None
        }
    }

    fn get_extra_data_intersect(&self, l: &Line) -> Vec<Geo> {
        let mut ret = Vec::new();
        let dist = l.distance(self.origin);
        let co = self.origin - l.origin;
        let mut n = l.get_normal().into_inner();
        if n.dot(&co) < 0.0 {
            n = -n;
        }
        let mid_point = l.origin + co - n * dist;
        ret.push(Geo::GeoPoint(Point2::new(mid_point.x, mid_point.y)));
        ret
    }
}

impl Intersect<LineSegment> for Circle {
    // r and s
    type Intersection = OneOrTwo<P2>;

    fn intersect(&self, ls: &LineSegment) -> Option<OneOrTwo<P2>> {
        let line: Line = ls.into();
        if let Some((r, s)) = self.intersect(&line) {
            if between(r, 0.0, 1.0) {
                let mut res = OneOrTwo::new(line.eval_at(r));
                if between(s, 0.0, 1.0) {
                    res.add(line.eval_at(s));
                    if s < r {
                        res.swap();
                    }
                }
                Some(res)
            } else {
                if between(s, 0.0, 1.0) {
                    return Some(OneOrTwo::new(line.eval_at(s)));
                }
                None
            }
        } else {
            None
        }
    }

    fn get_extra_data_intersect(&self, ls: &LineSegment) -> Vec<Geo> {
        let line: Line = ls.into();
        self.get_extra_data_intersect(&line)
    }
}

impl Intersect<Circle> for Circle {
    type Intersection = Float;
    fn intersect(&self, other: &Circle) -> Option<Float> {
        let dist: Float = distance(&self.origin, &other.origin);
        if dist < (self.radius + other.radius) {
            Some(dist)
        } else {
            None
        }
    }
}

impl Intersect<Rect> for Circle {
    type Intersection = (Normal, Float);

    fn intersect(&self, rect: &Rect) -> Option<(Normal, Float)> {
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

impl Contains for Circle {
    fn contains(&self, p: P2) -> bool {
        distance(&self.origin, &p) <= self.radius
    }
}

impl Distance for Circle {
    fn distance(&self, p: P2) -> Float {
        distance(&self.origin, &p) - self.radius
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

// a circle can not be reflected on anything
impl ReflectOn<Line> for Circle {}
impl ReflectOn<Ray> for Circle {}
impl ReflectOn<LineSegment> for Circle {}
impl ReflectOn<Circle> for Circle {}
impl ReflectOn<Rect> for Circle {}
impl ReflectOn<AABB> for Circle {}
impl ReflectOn<MCircle> for Circle {}

impl CanCollideWith<Line> for Circle {}
impl CanCollideWith<Ray> for Circle {}
impl CanCollideWith<LineSegment> for Circle {}
impl CanCollideWith<Circle> for Circle {}
impl CanCollideWith<Rect> for Circle {}
impl CanCollideWith<AABB> for Circle {}
impl CanCollideWith<MCircle> for Circle {}

impl GeoT for Circle {}
