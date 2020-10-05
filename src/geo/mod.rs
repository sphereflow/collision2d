pub mod aabb;
pub mod circle;
pub mod line;
pub mod line_segment;
pub mod mcircle;
pub mod ray;
pub mod rect;
pub mod traits;

extern crate nalgebra as na;

pub use crate::utils::*;
pub use aabb::*;
pub use circle::*;
pub use line::*;
pub use line_segment::*;
pub use mcircle::*;
use na::{distance, Matrix2};
use rand::distributions::{Distribution, Standard};
use rand::Rng;
pub use ray::*;
pub use rect::*;
pub use traits::*;

#[derive(Clone, PartialEq, Debug)]
pub struct Logic {
    op: LogicOp,
    a: Box<Geo>,
    b: Box<Geo>,
    pub origin: P2,
    x_axis: V2,
}

#[derive(Clone, PartialEq, Debug)]
pub enum LogicOp {
    And,
    Or,
    AndNot,
}

impl Logic {
    pub fn get_a(&self) -> Geo {
        let mut ret = (*self.a).clone();
        let transform = Matrix2::new(self.x_axis.x, -self.x_axis.y, self.x_axis.y, self.x_axis.x);
        ret.set_origin(self.origin + transform * self.a.get_origin().coords);
        ret
    }
    pub fn get_b(&self) -> Geo {
        let mut ret = self.b.clone().as_ref().clone();
        let transform = Matrix2::new(self.x_axis.x, -self.x_axis.y, self.x_axis.y, self.x_axis.x);
        ret.set_origin(self.origin + transform * self.b.get_origin().coords);
        ret
    }
}

impl Contains for Logic {
    fn contains(&self, p: P2) -> bool {
        match self.op {
            LogicOp::And => self.get_a().contains(p) & self.get_b().contains(p),
            LogicOp::Or => self.get_a().contains(p) | self.get_b().contains(p),
            LogicOp::AndNot => self.get_a().contains(p) & !self.get_b().contains(p),
        }
    }
}

impl Scale for Logic {
  fn scale(&mut self, scale_x: Float, scale_y: Float) {
      self.a.scale_position(scale_x, scale_y);
      self.b.scale_position(scale_x, scale_y);
      self.a.scale(scale_x, scale_y);
      self.b.scale(scale_x, scale_y);
  }
  fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
      self.origin.scale_position(scale_x, scale_y);
  }
}

#[derive(Clone, PartialEq, Debug)]
pub enum Geo {
    GeoRect(Rect),
    GeoCircle(Circle),
    GeoRay(Ray),
    GeoLineSegment(LineSegment),
    GeoPoint(P2),
    GeoMCircle(MCircle),
    GeoLogic(Logic),
}

impl Geo {
    pub fn does_overlap(&self, other: &Geo) -> bool {
        self.does_collide(other) | self.contains(self.get_origin())
    }

    pub fn close_to(&self, pos: P2, max_distance: Float) -> bool {
        match self {
            Geo::GeoRect(rect) => rect.contains(pos),
            Geo::GeoCircle(circle) => circle.contains(pos),
            Geo::GeoRay(ray) => ray.distance(pos) < max_distance,
            Geo::GeoLineSegment(ls) => ls.distance(pos) < max_distance,
            Geo::GeoPoint(p) => distance(p, &pos) < max_distance,
            Geo::GeoMCircle(mc) => mc.path.distance(pos) - mc.radius < max_distance,
            Geo::GeoLogic(l) => l.contains(pos),
        }
    }

    pub fn does_collide(&self, other: &Geo) -> bool {
        match (self, other) {
            (Geo::GeoRect(r1), Geo::GeoRect(r2)) => r1.does_collide(r2),
            (Geo::GeoRect(r1), Geo::GeoCircle(c2)) => r1.does_collide(c2),
            (Geo::GeoRect(r1), Geo::GeoRay(ray1)) => ray1.does_collide(r1),
            (Geo::GeoRect(r1), Geo::GeoLineSegment(ls2)) => r1.does_collide(ls2),
            (Geo::GeoRect(r1), Geo::GeoPoint(p2)) => r1.contains(*p2),
            (Geo::GeoRect(r1), Geo::GeoMCircle(mc)) => mc.reflect_on(r1).is_some(),
            (Geo::GeoCircle(c1), Geo::GeoCircle(c2)) => c1.intersect(c2).is_some(),
            (Geo::GeoCircle(c1), Geo::GeoRay(ray)) => ray.does_collide(c1),
            (Geo::GeoCircle(c1), Geo::GeoLineSegment(ls2)) => c1.intersect(ls2).is_some(),
            (Geo::GeoCircle(c1), Geo::GeoPoint(p2)) => c1.contains(*p2),
            (Geo::GeoCircle(c1), Geo::GeoMCircle(mc)) => {
                mc.path.distance(c1.origin) < mc.radius + c1.radius
            }
            (Geo::GeoLineSegment(ls), Geo::GeoRay(ray)) => ray.does_collide(ls),
            (Geo::GeoLineSegment(ls1), Geo::GeoLineSegment(ls2)) => ls1.intersect(ls2).is_some(),
            (Geo::GeoLineSegment(ls), Geo::GeoPoint(p2)) => ls.distance(*p2) < 0.000_001,
            (Geo::GeoLineSegment(ls), Geo::GeoMCircle(mc)) => mc.reflect_on(ls).is_some(),
            (Geo::GeoPoint(p), Geo::GeoRay(ray)) => ray.distance(*p) < 0.000_001,
            (Geo::GeoPoint(p1), Geo::GeoPoint(p2)) => p1 == p2,
            (Geo::GeoPoint(p), Geo::GeoMCircle(mc)) => mc.contains(*p),
            (Geo::GeoMCircle(mc), Geo::GeoRay(ray)) => ray.does_collide(mc),
            (Geo::GeoMCircle(mc1), Geo::GeoMCircle(mc2)) => mc1.reflect_on(mc2).is_some(),
            (Geo::GeoRay(ray1), Geo::GeoRay(ray2)) => ray1.does_collide(ray2),
            (_, _) => other.does_collide(self),
        }
    }

    /// returns the time of the collision of two objects in terms of percentage
    pub fn time_of_collision(&self, other: &Geo) -> Option<Float> {
        match (self, other) {
            (Geo::GeoRect(_), Geo::GeoRect(_)) => None,
            (Geo::GeoRect(_), Geo::GeoCircle(_)) => None,
            (Geo::GeoRect(_), Geo::GeoLineSegment(_)) => None,
            (Geo::GeoRect(_), Geo::GeoPoint(_)) => None,
            (Geo::GeoRect(r1), Geo::GeoMCircle(mc)) => mc.reflect_on(r1).map(|res_mc| res_mc.min_r),
            (Geo::GeoCircle(_), Geo::GeoCircle(_)) => None,
            (Geo::GeoCircle(_), Geo::GeoLineSegment(_)) => None,
            (Geo::GeoCircle(_), Geo::GeoPoint(_)) => None,
            (Geo::GeoCircle(c1), Geo::GeoMCircle(mc)) => {
                mc.reflect_on(c1).map(|res_mc| res_mc.min_r)
            }
            (Geo::GeoLineSegment(_), Geo::GeoLineSegment(_)) => None,
            (Geo::GeoLineSegment(_), Geo::GeoPoint(_)) => None,
            (Geo::GeoLineSegment(ls), Geo::GeoMCircle(mc)) => {
                mc.reflect_on(ls).map(|res_mc| res_mc.min_r)
            }
            (Geo::GeoPoint(_), Geo::GeoPoint(_)) => None,
            (Geo::GeoPoint(p), Geo::GeoMCircle(mc)) => mc
                .reflect_on(&Circle {
                    origin: *p,
                    radius: 0.0,
                })
                .map(|res_mc| res_mc.min_r),
            (Geo::GeoMCircle(mc1), Geo::GeoMCircle(mc2)) => {
                mc1.reflect_on(mc2).map(|res_mc| res_mc.min_r)
            }
            (_, _) => other.time_of_collision(self),
        }
    }
}

impl Scale for Geo {
    fn scale(&mut self, scale_x: Float, scale_y: Float) {
        match self {
            Geo::GeoRect(g) => g.scale(scale_x, scale_y),
            Geo::GeoMCircle(g) => g.scale(scale_x, scale_y),
            Geo::GeoPoint(g) => g.scale(scale_x, scale_y),
            Geo::GeoCircle(g) => g.scale(scale_x, scale_y),
            Geo::GeoRay(g) => g.scale(scale_x, scale_y),
            Geo::GeoLineSegment(g) => g.scale(scale_x, scale_y),
            Geo::GeoLogic(g) => g.scale(scale_x, scale_y),
        }
    }
  fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
      match self {
            Geo::GeoRect(g) => g.scale_position(scale_x, scale_y),
            Geo::GeoMCircle(g) => g.scale_position(scale_x, scale_y),
            Geo::GeoPoint(g) => g.scale_position(scale_x, scale_y),
            Geo::GeoCircle(g) => g.scale_position(scale_x, scale_y),
            Geo::GeoRay(g) => g.scale_position(scale_x, scale_y),
            Geo::GeoLineSegment(g) => g.scale_position(scale_x, scale_y),
            Geo::GeoLogic(g) => g.scale_position(scale_x, scale_y),
      }
  }
}

impl HasOrigin for Geo {
    fn get_origin(&self) -> P2 {
        match self {
            Geo::GeoRect(rect) => rect.origin,
            Geo::GeoCircle(circle) => circle.origin,
            Geo::GeoRay(ray) => ray.get_origin(),
            Geo::GeoLineSegment(ls) => ls.eval_at_r(0.5),
            Geo::GeoPoint(p) => *p,
            Geo::GeoMCircle(mc) => mc.path.eval_at_r(0.5),
            Geo::GeoLogic(l) => l.origin,
        }
    }

    fn set_origin(&mut self, origin: P2) {
        match self {
            Geo::GeoRect(rect) => rect.origin = origin,
            Geo::GeoCircle(circle) => circle.origin = origin,
            Geo::GeoRay(ray) => ray.set_origin(origin),
            Geo::GeoLineSegment(ls) => ls.set_origin(origin),
            Geo::GeoPoint(p) => *p = origin,
            Geo::GeoMCircle(mc) => mc.path.set_origin(origin),
            Geo::GeoLogic(l) => l.origin = origin,
        }
    }
}

impl Contains for Geo {
    fn contains(&self, p: P2) -> bool {
        match self {
            Geo::GeoRect(geo) => geo.contains(p),
            Geo::GeoCircle(geo) => geo.contains(p),
            Geo::GeoRay(geo) => geo.distance(p) < 0.00001,
            Geo::GeoLineSegment(geo) => geo.distance(p) < 0.00001,
            Geo::GeoPoint(geo) => *geo == p,
            Geo::GeoMCircle(geo) => geo.contains(p),
            Geo::GeoLogic(geo) => geo.contains(p),
        }
    }
}

impl Distribution<Geo> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Geo {
        match rng.next_u32() % 7 {
            0 => Geo::GeoRect(rng.gen()),
            1 => Geo::GeoCircle(rng.gen()),
            2 => Geo::GeoPoint(rng.gen()),
            3 => Geo::GeoLineSegment(rng.gen()),
            4 => Geo::GeoRay(rng.gen()),
            _ => Geo::GeoMCircle(rng.gen()),
        }
    }
}

impl From<P2> for Geo {
    fn from(p: P2) -> Self {
        Geo::GeoPoint(p)
    }
}
impl From<Ray> for Geo {
    fn from(ray: Ray) -> Self {
        Geo::GeoRay(ray)
    }
}
impl From<Rect> for Geo {
    fn from(rect: Rect) -> Self {
        Geo::GeoRect(rect)
    }
}
impl From<Circle> for Geo {
    fn from(circle: Circle) -> Self {
        Geo::GeoCircle(circle)
    }
}
impl From<MCircle> for Geo {
    fn from(mcircle: MCircle) -> Self {
        Geo::GeoMCircle(mcircle)
    }
}
impl From<LineSegment> for Geo {
    fn from(ls: LineSegment) -> Self {
        Geo::GeoLineSegment(ls)
    }
}
