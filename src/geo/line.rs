extern crate nalgebra as na;

use super::aabb::AABB;
use super::circle::Circle;
use super::line_segment::LineSegment;
use super::mcircle::MCircle;
use super::ray::Ray;
use super::rect::Rect;
use super::traits::*;
use crate::utils::*;
use na::base::{Matrix2, Unit};
use na::Vector2;
use rand::distributions::{Distribution, Standard};
use rand::Rng;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Line {
    pub origin: P2,
    direction: V2,
    normal: Normal,
}

impl Line {
    pub fn new(origin: P2, direction: V2) -> Line {
        let normal = Unit::new_normalize(Vector2::new(-direction.y, direction.x));
        Line {
            origin,
            direction,
            normal,
        }
    }

    pub fn get_normal(&self) -> Normal {
        self.normal
    }

    pub fn get_direction(&self) -> V2 {
        self.direction
    }

    pub fn set_direction(&mut self, direction: V2) {
        self.normal = Unit::new_normalize(Vector2::new(-direction.y, direction.x));
        self.direction = direction;
    }

    pub fn eval_at(&self, r: Float) -> P2 {
        self.origin + r * self.direction
    }
}

impl HasOrigin for Line {
    fn get_origin(&self) -> P2 {
        self.origin
    }
    fn set_origin(&mut self, origin: P2) {
        self.origin = origin;
    }
}

impl From<&Ray> for Line {
    fn from(ray: &Ray) -> Line {
        Line {
            origin: ray.get_origin(),
            direction: ray.get_direction(),
            normal: ray.get_normal(),
        }
    }
}

impl From<&LineSegment> for Line {
    fn from(ls: &LineSegment) -> Line {
        Line {
            origin: ls.get_a(),
            direction: ls.get_direction(),
            normal: ls.get_normal(),
        }
    }
}

impl Intersect<Line> for Line {
    type Intersection = (P2, Float, Float);

    // returns r and s
    fn intersect(&self, other: &Line) -> Option<(P2, Float, Float)> {
        let mut inv = Matrix2::from_columns(&[self.direction, -other.direction]);
        if inv.try_inverse_mut() {
            let rs = inv * (other.origin - self.origin);
            Some((self.origin + (self.direction * rs.x), rs.x, rs.y))
        } else {
            None
        }
    }
}

impl Intersect<LineSegment> for Line {
    type Intersection = P2;

    fn intersect(&self, ls: &LineSegment) -> Option<P2> {
        ls.intersect(self)
    }
}

impl ReflectOn<Line> for Line {
    fn reflect_on_normal_intersect(&self, other: &Line) -> Option<(Self, V2, P2)> {
        let dmat = Matrix2::from_columns(&[self.direction, -other.direction]);
        match inverse(&dmat) {
            Some(inv) => {
                let n = other.get_normal().into_inner();
                let rs = inv * (other.origin - self.origin);
                let intersection = self.origin + self.direction * rs.x;
                let c1 = n.x * n.x - n.y * n.y;
                let c2 = 2. * n.x * n.y;
                let reflected = V2::new(
                    self.direction.y * c2 - self.direction.x * c1,
                    self.direction.x * c2 + self.direction.y * c1,
                );
                let normal_line = V2::new(-self.direction.y, self.direction.x);
                let new_line = Line {
                    origin: intersection,
                    direction: reflected,
                    normal: Unit::new_normalize(normal_line),
                };
                Some((new_line, n, intersection))
            }
            None => None,
        }
    }
}

impl ReflectOn<Ray> for Line {
    fn reflect_on_normal_intersect(&self, other: &Ray) -> Option<(Self, V2, P2)> {
        let dmat = Matrix2::from_columns(&[self.direction, -other.get_direction()]);
        match inverse(&dmat) {
            Some(inv) => {
                let n = other.get_normal().into_inner();
                let rs = inv * (other.get_origin() - self.origin);
                if rs[1] < 0. {
                    return None;
                }
                let intersection = self.origin + self.direction * rs.x;
                let c1 = n.x * n.x - n.y * n.y;
                let c2 = 2. * n.x * n.y;
                let reflected = V2::new(
                    self.direction.y * c2 - self.direction.x * c1,
                    self.direction.x * c2 + self.direction.y * c1,
                );
                let normal_line = V2::new(-self.direction.y, self.direction.x);
                let new_line = Line {
                    origin: intersection,
                    direction: reflected,
                    normal: Unit::new_normalize(normal_line),
                };
                Some((new_line, n, intersection))
            }
            None => None,
        }
    }
}

impl ReflectOn<LineSegment> for Line {
    fn reflect_on_normal_intersect(&self, other: &LineSegment) -> Option<(Self, V2, P2)> {
        let dmat = Matrix2::from_columns(&[self.direction, -other.get_direction()]);
        match inverse(&dmat) {
            Some(inv) => {
                let n = other.get_normal().into_inner();
                let rs = inv * (other.get_a() - self.origin);
                if rs[1] < 0. || rs[1] > 1. {
                    return None;
                }
                let intersection = self.origin + self.direction * rs.x;
                let c1 = n.x * n.x - n.y * n.y;
                let c2 = 2. * n.x * n.y;
                let reflected = V2::new(
                    self.direction.y * c2 - self.direction.x * c1,
                    self.direction.x * c2 + self.direction.y * c1,
                );
                let normal_line = V2::new(-self.direction.y, self.direction.x);
                let new_line = Line {
                    origin: intersection,
                    direction: reflected,
                    normal: Unit::new_normalize(normal_line),
                };
                Some((new_line, n, intersection))
            }
            None => None,
        }
    }
}

impl Distance for Line {
    fn distance(&self, p: P2) -> Float {
        Vector2::dot(&(p - self.origin), &self.get_normal()).abs()
    }
}

impl Distribution<Line> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Line {
        let direction: V2 = rng.gen();
        let normal = Unit::new_normalize(Vector2::new(-direction.y, direction.x));
        Line {
            origin: rng.gen(),
            direction,
            normal,
        }
    }
}

impl ReflectOn<Circle> for Line {}
impl ReflectOn<Rect> for Line {}
impl ReflectOn<AABB> for Line {}
impl ReflectOn<MCircle> for Line {}

impl CanCollideWith<Line> for Line {}
impl CanCollideWith<Ray> for Line {}
impl CanCollideWith<LineSegment> for Line {}
impl CanCollideWith<Circle> for Line {}
impl CanCollideWith<Rect> for Line {}
impl CanCollideWith<AABB> for Line {}
impl CanCollideWith<MCircle> for Line {}

impl GeoT for Line {}
