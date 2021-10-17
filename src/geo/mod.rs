pub mod aabb;
pub mod circle;
pub mod convex_polygon;
pub mod cubic_bezier;
pub mod line;
pub mod line_segment;
pub mod logic;
pub mod mcircle;
pub mod ray;
pub mod rect;
pub mod traits;

extern crate nalgebra as na;

pub use crate::utils::*;
pub use aabb::*;
pub use circle::*;
pub use convex_polygon::*;
pub use cubic_bezier::*;
pub use line::*;
pub use line_segment::*;
pub use logic::*;
pub use mcircle::*;
pub use na::{distance, distance_squared, Matrix2, Rotation2, Unit};
pub use rand::distributions::{Distribution, Standard};
pub use rand::Rng;
pub use ray::*;
pub use rect::*;
use std::ops::{BitAnd, BitOr};
pub use traits::*;
use serde::*;

#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum Geo {
    GeoRect(Rect),
    GeoCircle(Circle),
    GeoRay(Ray),
    GeoLineSegment(LineSegment),
    GeoPoint(P2),
    GeoMCircle(MCircle),
    GeoConvexPolygon(ConvexPolygon),
    GeoCubicBezier(CubicBezier),
    GeoLogic(Logic),
}

impl Geo {
    pub fn does_overlap(&self, other: &Geo) -> bool {
        self.does_collide(other) | self.contains(&self.get_origin())
    }

    pub fn close_to(&self, pos: &P2, max_distance: Float) -> bool {
        match self {
            Geo::GeoRect(rect) => rect.contains(pos),
            Geo::GeoCircle(circle) => circle.contains(pos),
            Geo::GeoRay(ray) => ray.distance(&pos) < max_distance,
            Geo::GeoLineSegment(ls) => ls.distance(&pos) < max_distance,
            Geo::GeoPoint(p) => distance(p, &pos) < max_distance,
            Geo::GeoMCircle(mc) => mc.path.distance(&pos) - mc.radius < max_distance,
            Geo::GeoConvexPolygon(cp) => cp.distance(&pos) < max_distance,
            Geo::GeoCubicBezier(_cb) => false,
            Geo::GeoLogic(l) => l.contains(pos),
        }
    }

    pub fn does_collide(&self, other: &Geo) -> bool {
        match (self, other) {
            (Geo::GeoRect(r1), Geo::GeoRect(r2)) => r1.does_collide(r2),
            (Geo::GeoRect(r1), Geo::GeoCircle(c2)) => r1.does_collide(c2),
            (Geo::GeoRect(r1), Geo::GeoRay(ray1)) => ray1.does_collide(r1),
            (Geo::GeoRect(r1), Geo::GeoLineSegment(ls2)) => r1.does_collide(ls2),
            (Geo::GeoRect(r1), Geo::GeoPoint(p2)) => r1.contains(p2),
            (Geo::GeoRect(r1), Geo::GeoMCircle(mc)) => mc.reflect_on(r1).is_some(),
            (Geo::GeoCircle(c1), Geo::GeoCircle(c2)) => c1.intersect(c2).is_some(),
            (Geo::GeoCircle(c1), Geo::GeoRay(ray)) => ray.does_collide(c1),
            (Geo::GeoCircle(c1), Geo::GeoLineSegment(ls2)) => c1.intersect(ls2).is_some(),
            (Geo::GeoCircle(c1), Geo::GeoPoint(p2)) => c1.contains(p2),
            (Geo::GeoCircle(c1), Geo::GeoMCircle(mc)) => {
                mc.path.distance(&c1.origin) < mc.radius + c1.radius
            }
            (Geo::GeoLineSegment(ls), Geo::GeoRay(ray)) => ray.does_collide(ls),
            (Geo::GeoLineSegment(ls1), Geo::GeoLineSegment(ls2)) => ls1.intersect(ls2).is_some(),
            (Geo::GeoLineSegment(ls), Geo::GeoPoint(p2)) => ls.distance(p2) < EPSILON,
            (Geo::GeoLineSegment(ls), Geo::GeoMCircle(mc)) => mc.reflect_on(ls).is_some(),
            (Geo::GeoPoint(p), Geo::GeoRay(ray)) => ray.distance(&p) < EPSILON,
            (Geo::GeoPoint(p1), Geo::GeoPoint(p2)) => p1 == p2,
            (Geo::GeoPoint(p), Geo::GeoMCircle(mc)) => mc.contains(p),
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

    pub fn and_not(self, not: Self) -> Geo {
        let vdir = (not.get_origin() - self.get_origin()) * 0.5;
        let origin = self.get_origin() + vdir;
        let mut a = Box::new(self);
        let mut b = Box::new(not);
        a.set_origin(P2::from(-vdir));
        b.set_origin(P2::from(vdir));
        Geo::GeoLogic(Logic {
            a,
            b,
            origin,
            op: LogicOp::AndNot,
            rotation: Rot2::identity(),
        })
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
            Geo::GeoConvexPolygon(cp) => cp.scale(scale_x, scale_y),
            Geo::GeoCubicBezier(g) => g.scale(scale_x, scale_y),
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
            Geo::GeoConvexPolygon(cp) => cp.scale_position(scale_x, scale_y),
            Geo::GeoCubicBezier(g) => g.scale_position(scale_x, scale_y),
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
            Geo::GeoConvexPolygon(cp) => cp.get_origin(),
            Geo::GeoCubicBezier(cb) => cb.get_origin(),
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
            Geo::GeoConvexPolygon(cp) => cp.set_origin(origin),
            Geo::GeoCubicBezier(cb) => cb.set_origin(origin),
            Geo::GeoLogic(l) => l.origin = origin,
        }
    }
}

impl HasAabb for Geo {
    fn get_aabb(&self) -> Aabb {
        match self {
            Geo::GeoRect(rect) => rect.get_aabb(),
            Geo::GeoCircle(circle) => circle.get_aabb(),
            Geo::GeoRay(ray) => ray.get_aabb(),
            Geo::GeoLineSegment(ls) => ls.get_aabb(),
            Geo::GeoPoint(p) => p.get_aabb(),
            Geo::GeoMCircle(mc) => mc.get_aabb(),
            Geo::GeoConvexPolygon(cp) => cp.get_aabb(),
            Geo::GeoCubicBezier(cb) => cb.get_aabb(),
            Geo::GeoLogic(l) => l.get_aabb(),
        }
    }
}

impl Contains for Geo {
    fn contains(&self, p: &P2) -> bool {
        match self {
            Geo::GeoRect(geo) => geo.contains(p),
            Geo::GeoCircle(geo) => geo.contains(p),
            Geo::GeoRay(geo) => geo.distance(&p) < EPSILON,
            Geo::GeoLineSegment(geo) => geo.distance(&p) < EPSILON,
            Geo::GeoPoint(geo) => geo == p,
            Geo::GeoMCircle(geo) => geo.contains(p),
            Geo::GeoConvexPolygon(geo) => geo.contains(p),
            Geo::GeoCubicBezier(_) => false,
            Geo::GeoLogic(geo) => geo.contains(p),
        }
    }
}

impl ClosestPoint for Geo {
    fn closest_point_to(&self, p: &P2) -> P2 {
        match self {
            Geo::GeoRect(geo) => geo.closest_point_to(p),
            Geo::GeoCircle(geo) => geo.closest_point_to(p),
            Geo::GeoRay(geo) => geo.closest_point_to(p),
            Geo::GeoLineSegment(geo) => geo.closest_point_to(p),
            Geo::GeoPoint(geo) => *geo,
            Geo::GeoMCircle(geo) => geo.closest_point_to(p),
            Geo::GeoConvexPolygon(geo) => geo.closest_point_to(p),
            Geo::GeoCubicBezier(_) => P2::new(0., 0.),
            Geo::GeoLogic(geo) => geo.closest_point_to(p),
        }
    }
}

impl Distance for Geo {
    fn distance(&self, p: &P2) -> Float {
        match self {
            Geo::GeoRect(g) => g.distance(p),
            Geo::GeoCircle(g) => g.distance(p),
            Geo::GeoRay(g) => g.distance(p),
            Geo::GeoLineSegment(g) => g.distance(p),
            Geo::GeoPoint(g) => distance(g, &p),
            Geo::GeoMCircle(g) => Circle {
                radius: g.radius,
                origin: g.get_origin(),
            }
            .distance(p),
            Geo::GeoConvexPolygon(g) => g.distance(p),
            Geo::GeoCubicBezier(_) => 0.,
            Geo::GeoLogic(g) => g.distance(p),
        }
    }
}

impl Rotate for Geo {
    fn get_rotation(&self) -> Rot2 {
        match self {
            Geo::GeoRect(geo) => geo.get_rotation(),
            Geo::GeoCircle(geo) => geo.get_rotation(),
            Geo::GeoRay(geo) => geo.get_rotation(),
            Geo::GeoLineSegment(geo) => geo.get_rotation(),
            Geo::GeoPoint(geo) => geo.get_rotation(),
            Geo::GeoMCircle(geo) => geo.get_rotation(),
            Geo::GeoConvexPolygon(geo) => geo.get_rotation(),
            Geo::GeoCubicBezier(geo) => geo.get_rotation(),
            Geo::GeoLogic(geo) => geo.get_rotation(),
        }
    }

    fn set_rotation(&mut self, rotation: &Rot2) {
        match self {
            Geo::GeoRect(geo) => geo.set_rotation(rotation),
            Geo::GeoCircle(geo) => geo.set_rotation(rotation),
            Geo::GeoRay(geo) => geo.set_rotation(rotation),
            Geo::GeoLineSegment(geo) => geo.set_rotation(rotation),
            Geo::GeoPoint(geo) => geo.set_rotation(rotation),
            Geo::GeoMCircle(geo) => geo.set_rotation(rotation),
            Geo::GeoConvexPolygon(geo) => geo.set_rotation(rotation),
            Geo::GeoCubicBezier(geo) => geo.set_rotation(rotation),
            Geo::GeoLogic(geo) => geo.set_rotation(rotation),
        }
    }
}

impl Mirror for Geo {
    fn mirror_x(&self) -> Self {
        match self {
            Geo::GeoRect(geo) => geo.mirror_x().into(),
            Geo::GeoCircle(geo) => geo.mirror_x().into(),
            Geo::GeoRay(geo) => geo.mirror_x().into(),
            Geo::GeoLineSegment(geo) => geo.mirror_x().into(),
            Geo::GeoPoint(geo) => geo.mirror_x().into(),
            Geo::GeoMCircle(geo) => geo.mirror_x().into(),
            Geo::GeoConvexPolygon(geo) => geo.mirror_x().into(),
            Geo::GeoCubicBezier(geo) => geo.mirror_x().into(),
            Geo::GeoLogic(geo) => geo.mirror_x().into(),
        }
    }

    fn mirror_y(&self) -> Self {
        match self {
            Geo::GeoRect(geo) => geo.mirror_y().into(),
            Geo::GeoCircle(geo) => geo.mirror_y().into(),
            Geo::GeoRay(geo) => geo.mirror_y().into(),
            Geo::GeoLineSegment(geo) => geo.mirror_y().into(),
            Geo::GeoPoint(geo) => geo.mirror_y().into(),
            Geo::GeoMCircle(geo) => geo.mirror_y().into(),
            Geo::GeoConvexPolygon(geo) => geo.mirror_y().into(),
            Geo::GeoCubicBezier(geo) => geo.mirror_y().into(),
            Geo::GeoLogic(geo) => geo.mirror_y().into(),
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

impl BitAnd for Geo {
    type Output = Self;
    fn bitand(self, rhs: Self) -> Self::Output {
        let vdir = (rhs.get_origin() - self.get_origin()) * 0.5;
        let origin = self.get_origin() + vdir;
        let mut a = Box::new(self);
        let mut b = Box::new(rhs);
        a.set_origin(P2::from(-vdir));
        b.set_origin(P2::from(vdir));
        Geo::GeoLogic(Logic {
            a,
            b,
            origin,
            op: LogicOp::And,
            rotation: Rot2::identity(),
        })
    }
}

impl BitOr for Geo {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self::Output {
        let vdir = (rhs.get_origin() - self.get_origin()) * 0.5;
        let origin = self.get_origin() + vdir;
        let mut a = Box::new(self);
        let mut b = Box::new(rhs);
        a.set_origin(P2::from(-vdir));
        b.set_origin(P2::from(vdir));
        Geo::GeoLogic(Logic {
            a,
            b,
            origin,
            op: LogicOp::Or,
            rotation: Rot2::identity(),
        })
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
impl From<ConvexPolygon> for Geo {
    fn from(cp: ConvexPolygon) -> Self {
        Geo::GeoConvexPolygon(cp)
    }
}
impl From<CubicBezier> for Geo {
    fn from(cb: CubicBezier) -> Self {
        Geo::GeoCubicBezier(cb)
    }
}
impl From<Logic> for Geo {
    fn from(l: Logic) -> Self {
        Geo::GeoLogic(l)
    }
}

// impl Intersect<Ray> for Geo {
// type Intersection = Vec<P2>;
// fn intersect(&self, ray: &Ray) -> Option<Self::Intersection> {
// match self {
// Geo::GeoPoint(p) => {
// if ray.distance(p) < EPSILON {
// Some(vec![*p])
// } else {
// None
// }
// }
// Geo::GeoRay(r) => r.intersect(ray).map(|(p, _)| vec![p]),
// Geo::GeoLineSegment(ls) => ls.intersect(ray).map(|p| vec![p]),
// Geo::GeoRect(r) => r.intersect(ray).map(|oot| oot.into_vec()),
// Geo::GeoCircle(c) => ray.intersect(c).map(|oot| oot.into_vec()),
// Geo::GeoMCircle(mc) => ray.intersect(mc).map(|p| vec![p]),
// Geo::GeoLogic(l) => ray.intersect(l).map(|(_nearest, v, _geo)| v),
// }
// }
// }

impl Intersect<LineSegment> for Geo {
    type Intersection = Vec<P2>;
    fn intersect(&self, line_segment: &LineSegment) -> Option<Self::Intersection> {
        match self {
            Geo::GeoPoint(p) => {
                if line_segment.distance(p) < EPSILON {
                    Some(vec![*p])
                } else {
                    None
                }
            }
            Geo::GeoRay(r) => r.intersect(line_segment).map(|(p, _)| vec![p]),
            Geo::GeoLineSegment(ls) => ls.intersect(line_segment).map(|p| vec![p]),
            Geo::GeoRect(r) => r.intersect(line_segment).map(|oot| oot.into_vec()),
            Geo::GeoCircle(c) => c.intersect(line_segment).map(|oot| oot.into_vec()),
            Geo::GeoMCircle(mc) => mc.intersect(line_segment).map(|oot| oot.into_vec()),
            Geo::GeoConvexPolygon(cp) => cp
                .intersect(line_segment)
                .map(|oot| oot.map(|(p, _)| p).into_vec()),
            Geo::GeoCubicBezier(_) => None,
            Geo::GeoLogic(l) => l.intersect(line_segment),
        }
    }
}

impl Intersect<Rect> for Geo {
    type Intersection = Vec<P2>;
    fn intersect(&self, rect: &Rect) -> Option<Self::Intersection> {
        match self {
            Geo::GeoPoint(p) => {
                if rect.distance(p) < EPSILON {
                    Some(vec![*p])
                } else {
                    None
                }
            }
            Geo::GeoRay(r) => r.intersect(rect).map(|oot| oot.map(|(p, _)| p).into_vec()),
            Geo::GeoLineSegment(ls) => rect.intersect(ls).map(|oot| oot.into_vec()),
            Geo::GeoRect(r) => r.intersect(rect),
            Geo::GeoCircle(c) => rect.intersect(c),
            Geo::GeoMCircle(mc) => mc.intersect(rect),
            Geo::GeoConvexPolygon(cp) => cp.intersect(rect),
            Geo::GeoCubicBezier(_) => None,
            Geo::GeoLogic(l) => l.intersect(rect),
        }
    }
}

impl Intersect<Aabb> for Geo {
    type Intersection = Vec<P2>;
    fn intersect(&self, aabb: &Aabb) -> Option<Self::Intersection> {
        match self {
            Geo::GeoPoint(p) => {
                if aabb.distance(p) < EPSILON {
                    Some(vec![*p])
                } else {
                    None
                }
            }
            Geo::GeoRay(r) => r.intersect(aabb).map(|oot| oot.map(|(p, _)| p).into_vec()),
            Geo::GeoLineSegment(ls) => aabb.intersect(ls).map(|oot| oot.into_vec()),
            Geo::GeoRect(r) => aabb.intersect(r),
            Geo::GeoCircle(c) => aabb.intersect(c).map(|oot| oot.into_vec()),
            Geo::GeoMCircle(mc) => mc.intersect(&aabb.to_rect()),
            Geo::GeoConvexPolygon(cp) => cp.intersect(&aabb.to_rect()),
            Geo::GeoCubicBezier(_) => None,
            Geo::GeoLogic(l) => l.intersect(&aabb.to_rect()),
        }
    }
}

impl Intersect<Circle> for Geo {
    type Intersection = Vec<P2>;
    fn intersect(&self, circle: &Circle) -> Option<Self::Intersection> {
        match self {
            Geo::GeoPoint(p) => {
                if circle.distance(p) < EPSILON {
                    Some(vec![*p])
                } else {
                    None
                }
            }
            Geo::GeoRay(r) => r
                .intersect(circle)
                .map(|oot| oot.map(|(p, _)| p).into_vec()),
            Geo::GeoLineSegment(ls) => circle.intersect(ls).map(|oot| oot.into_vec()),
            Geo::GeoRect(r) => r.intersect(circle),
            Geo::GeoCircle(c) => circle.intersect(c).map(|oot| oot.into_vec()),
            Geo::GeoMCircle(mc) => mc.intersect(circle).map(|oot| oot.into_vec()),
            Geo::GeoConvexPolygon(cp) => cp.intersect(circle),
            Geo::GeoCubicBezier(_) => None,
            Geo::GeoLogic(l) => l.intersect(circle),
        }
    }
}

impl Intersect<MCircle> for Geo {
    type Intersection = Vec<P2>;
    fn intersect(&self, mcircle: &MCircle) -> Option<Self::Intersection> {
        match self {
            Geo::GeoPoint(p) => {
                if mcircle.distance(p) < EPSILON {
                    Some(vec![*p])
                } else {
                    None
                }
            }
            Geo::GeoRay(r) => r.intersect(mcircle).map(|(p, _)| vec![p]),
            Geo::GeoLineSegment(ls) => mcircle.intersect(ls).map(|oot| oot.into_vec()),
            Geo::GeoRect(r) => mcircle.intersect(r),
            Geo::GeoCircle(c) => mcircle.intersect(c).map(|oot| oot.into_vec()),
            Geo::GeoMCircle(mc) => mc.intersect(mcircle).map(|oot| oot.into_vec()),
            Geo::GeoConvexPolygon(cp) => cp.intersect(mcircle),
            Geo::GeoCubicBezier(_) => None,
            Geo::GeoLogic(l) => l.intersect(mcircle),
        }
    }
}

impl Intersect<Logic> for Geo {
    type Intersection = Vec<P2>;
    fn intersect(&self, logic: &Logic) -> Option<Self::Intersection> {
        match self {
            Geo::GeoPoint(p) => {
                if logic.distance(p) < EPSILON {
                    Some(vec![*p])
                } else {
                    None
                }
            }
            Geo::GeoRay(r) => r
                .intersect(logic)
                .map(|v| v.into_iter().map(|(p, _)| p).collect()),
            Geo::GeoLineSegment(ls) => logic.intersect(ls),
            Geo::GeoRect(r) => logic.intersect(r),
            Geo::GeoCircle(c) => logic.intersect(c),
            Geo::GeoMCircle(mc) => logic.intersect(mc),
            Geo::GeoConvexPolygon(cp) => cp.intersect(logic),
            Geo::GeoCubicBezier(_) => None,
            Geo::GeoLogic(l) => l.intersect(logic),
        }
    }
}

impl Intersect<Geo> for Geo {
    type Intersection = Vec<P2>;
    fn intersect(&self, geo: &Geo) -> Option<Self::Intersection> {
        match self {
            Geo::GeoPoint(p) => {
                if geo.distance(p) < EPSILON {
                    Some(vec![*p])
                } else {
                    None
                }
            }
            Geo::GeoRay(r) => r
                .intersect(geo)
                .map(|v| v.into_iter().map(|(p, _)| p).collect()),
            Geo::GeoLineSegment(ls) => geo.intersect(ls),
            Geo::GeoRect(r) => geo.intersect(r),
            Geo::GeoCircle(c) => geo.intersect(c),
            Geo::GeoMCircle(mc) => geo.intersect(mc),
            Geo::GeoConvexPolygon(cp) => cp.intersect(geo),
            Geo::GeoCubicBezier(_) => None,
            Geo::GeoLogic(l) => geo.intersect(l),
        }
    }
}
