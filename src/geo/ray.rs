extern crate nalgebra as na;

use super::*;
use na::geometry::Point2;
use na::{distance, Unit, Vector2};
use rand::distributions::{Distribution, Standard};
use rand::Rng;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Ray {
    origin: P2,
    normal: Normal,
    direction: U2,
}

impl Ray {
    pub fn from_origin(origin: P2, direction: V2) -> Ray {
        let normal = Unit::new_normalize(Vector2::new(-direction.y, direction.x));
        let direction = Unit::new_normalize(direction);
        Ray {
            origin,
            direction,
            normal,
        }
    }

    pub fn to_line(&self) -> Line {
        Line::new(self.origin, self.direction.into_inner())
    }

    pub fn get_origin(&self) -> P2 {
        self.origin
    }

    pub fn set_origin(&mut self, origin: P2) {
        self.origin = origin;
    }

    pub fn shift(&mut self, v: V2) {
        self.origin += v;
    }

    pub fn offset(self) -> Self {
        Ray {
            origin: self.origin + EPSILON * self.direction.into_inner(),
            direction: self.direction,
            normal: self.normal,
        }
    }

    pub fn get_normal(&self) -> Normal {
        self.normal
    }

    pub fn get_direction(&self) -> V2 {
        self.direction.into_inner()
    }

    pub fn set_direction(&mut self, direction: V2) {
        self.direction = Unit::new_normalize(direction);
        // normal is rotated counter clockwise
        self.normal = Unit::new_normalize(Vector2::new(-direction.y, direction.x))
    }

    pub fn eval_at_r(&self, r: Float) -> P2 {
        self.origin + r * self.direction.into_inner()
    }

    pub fn refract_on<T>(
        &self,
        surface: &T,
        incoming_refractive_index: Float,
        outgoing_refractive_index: Float,
        // reflected, transmitted, reflectance
    ) -> Option<(Self, Option<Self>, Float)>
    where
        Ray: ReflectOn<T>,
    {
        // get the intersection point
        if let Some((reflected, normal, intersection)) = self.reflect_on_normal_intersect(surface) {
            let n1 = incoming_refractive_index;
            let n2 = outgoing_refractive_index;
            let n = incoming_refractive_index / outgoing_refractive_index;
            let cos_i = normal.dot(&self.direction).abs();
            let sin_t2 = n * n * (1.0 - cos_i * cos_i);
            let mut transmitted: Option<Ray> = None;
            let mut reflectance = 1.;
            if sin_t2 <= 1. {
                let cos_t = Float::sqrt(1.0 - sin_t2);
                transmitted = Some(
                    Ray::from_origin(
                        intersection,
                        n * self.direction.into_inner() + (n * cos_i - cos_t) * normal,
                    )
                    .offset(),
                );
                let r_orth = (n1 * cos_i - n2 * cos_t) / (n1 * cos_i + n2 * cos_t);
                let r_par = (n2 * cos_i - n1 * cos_t) / (n2 * cos_i + n1 * cos_t);
                reflectance = (r_orth * r_orth + r_par * r_par) * 0.5;
            }
            Some((reflected, transmitted, reflectance))
        } else {
            None
        }
    }

    pub fn refract_on_geo(
        &self,
        geo: &Geo,
        incoming_refractive_index: Float,
        outgoing_refractive_index: Float,
        // reflected, transmitted, reflectance
    ) -> Option<(Self, Option<Self>, Float)> {
        match geo {
            Geo::GeoRect(rect) => {
                self.refract_on(rect, incoming_refractive_index, outgoing_refractive_index)
            }
            Geo::GeoCircle(circle) => {
                self.refract_on(circle, incoming_refractive_index, outgoing_refractive_index)
            }
            Geo::GeoLineSegment(ls) => {
                self.refract_on(ls, incoming_refractive_index, outgoing_refractive_index)
            }
            Geo::GeoMCircle(mcircle) => self.refract_on(
                mcircle,
                incoming_refractive_index,
                outgoing_refractive_index,
            ),
            _ => None,
        }
    }
}

impl ReflectOn<Line> for Ray {
    fn reflect_on_normal_intersect(&self, line: &Line) -> Option<(Ray, V2, P2)> {
        match self.intersect(line) {
            Some((intersection, n)) => {
                let b = 2.0 * intersection - self.origin.coords;
                let dist = line.distance(&Point2::from(b - intersection));
                let new_b = b - 2.0 * dist * n.into_inner();
                Some((
                    Ray::from_origin(intersection, new_b.coords),
                    n.into_inner(),
                    intersection,
                ))
            }
            None => None,
        }
    }
}

impl ReflectOn<Ray> for Ray {
    fn reflect_on_normal_intersect(&self, ray: &Ray) -> Option<(Ray, V2, P2)> {
        match self.intersect(ray) {
            Some((intersection, _)) => {
                let mut n = ray.normal.into_inner();
                if n.dot(&self.get_direction()) < 0.0 {
                    n = -n;
                }
                let b = Point2::from(2.0 * intersection - self.origin);
                let dist = ray.to_line().distance(&b);
                let new_b = b - 2.0 * dist * n;
                Some((
                    Ray::from_origin(intersection, new_b - intersection),
                    n,
                    intersection,
                ))
            }
            None => None,
        }
    }
}

// impl ReflectOn<LineSegment> for Ray {
// fn reflect_on_normal(&self, ls: &LineSegment) -> Option<(Self, V2)> {
// if let Some(i) = self.intersect(ls) {
// let n = ls.get_normal().into_inner();
// let c1 = n[0] * n[0] - n[1] * n[1];
// let c2 = 2. * n[0] * n[1];
// let r = Vector2::new(
// self.direction[0] * c2 + self.direction[1] * c1,
// self.direction[1] * c2 - self.direction[0] * c1,
// );
// Some((Ray::from_origin(Point2::from(i), r), n))
// } else {
// None
// }
// }
// }

impl ReflectOn<LineSegment> for Ray {
    fn reflect_on_normal_intersect(&self, ls: &LineSegment) -> Option<(Self, V2, P2)> {
        if let Some((i, _)) = self.intersect(ls) {
            let n = ls.get_normal().into_inner();
            let d = self.direction.into_inner();
            let r = d - 2. * d.dot(&n) * n;
            let ray = Ray::from_origin(i, r).offset();
            Some((ray, n, i))
        } else {
            None
        }
    }
}

impl ReflectOn<Circle> for Ray {
    fn reflect_on_normal_intersect(&self, circle: &Circle) -> Option<(Ray, V2, P2)> {
        match circle.intersect(&self.to_line()) {
            Some((r, s)) => {
                if let Some(i) = smallest_positive_value(r, s).map(|r| self.eval_at_r(r)) {
                    let n = (i - circle.origin).normalize();
                    let d = self.direction.into_inner();
                    let dotp = d.dot(&n);
                    let r = d - 2. * dotp * n;
                    let ray = Ray::from_origin(i, r).offset();
                    Some((ray, n, i))
                } else {
                    None
                }
            }
            None => None,
        }
    }
}

impl ReflectOn<MCircle> for Ray {
    fn reflect_on_normal_intersect(&self, mcircle: &MCircle) -> Option<(Ray, V2, P2)> {
        let circle = Circle {
            origin: mcircle.path.get_b(),
            radius: mcircle.radius,
        };
        match circle.intersect(&self.to_line()) {
            Some((r, s)) => {
                if let Some(i) = smallest_positive_value(r, s).map(|r| self.eval_at_r(r)) {
                    let n = (i - circle.origin).normalize();
                    let d = self.direction.into_inner();
                    let r = d - 2. * (d.dot(&n) * n);
                    let ray = Ray::from_origin(i, r).offset();
                    Some((ray, n, i))
                } else {
                    None
                }
            }
            None => None,
        }
    }
}

impl ReflectOn<Rect> for Ray {
    fn reflect_on_normal_intersect(&self, rect: &Rect) -> Option<(Ray, V2, P2)> {
        let mut closest: Option<(Ray, V2, P2)> = None;
        for ls in rect.line_segments().iter() {
            if let Some((ray, n, i)) = self.reflect_on_normal_intersect(ls) {
                if let Some((_clray, _, cli)) = closest {
                    if distance(&i, &self.origin) < distance(&cli, &self.origin) {
                        closest = Some((ray, n, i));
                    }
                } else {
                    closest = Some((ray, n, i));
                }
            }
        }
        closest
    }
}

impl ReflectOn<AABB> for Ray {
    fn reflect_on_normal_intersect(&self, rect: &AABB) -> Option<(Ray, V2, P2)> {
        let mut closest: Option<(Ray, V2, P2)> = None;
        for ls in rect.line_segments().iter() {
            if let Some((ray, n, i)) = self.reflect_on_normal_intersect(ls) {
                if let Some((clray, _, _)) = closest {
                    if distance(&ray.origin, &self.origin) < distance(&clray.origin, &self.origin) {
                        closest = Some((ray, n, i));
                    }
                }
            }
        }
        closest
    }
}

impl ReflectOn<Logic> for Ray {
    fn reflect_on_normal_intersect(&self, l: &Logic) -> Option<(Self, V2, P2)> {
        self.intersect(l)
            .and_then(|(_, _, which)| self.reflect_on_normal_intersect(&which))
    }
}

impl ReflectOn<Geo> for Ray {
    fn reflect_on_normal_intersect(&self, other: &Geo) -> Option<(Self, V2, P2)> {
        match other {
            Geo::GeoRay(ray) => self.reflect_on_normal_intersect(ray),
            Geo::GeoRect(rect) => self.reflect_on_normal_intersect(rect),
            Geo::GeoLineSegment(ls) => self.reflect_on_normal_intersect(ls),
            Geo::GeoCircle(c) => self.reflect_on_normal_intersect(c),
            Geo::GeoMCircle(mc) => self.reflect_on_normal_intersect(mc),
            Geo::GeoPoint(_) => None,
            Geo::GeoLogic(l) => self.reflect_on_normal_intersect(l),
        }
    }
}

impl Intersect<Line> for Ray {
    type Intersection = (P2, Normal);
    fn intersect(&self, l: &Line) -> Option<(P2, Normal)> {
        if let Some((v, r, _)) = self.to_line().intersect(l) {
            if r < 0.0 {
                None
            } else {
                Some((v, l.get_normal()))
            }
        } else {
            None
        }
    }
}

impl Intersect<Ray> for Ray {
    type Intersection = (P2, Normal);
    fn intersect(&self, other: &Ray) -> Option<(P2, Normal)> {
        if let Some((v, r, s)) = self.to_line().intersect(&other.to_line()) {
            if r < 0.0 || s < 0.0 {
                None
            } else {
                Some((v, other.get_normal()))
            }
        } else {
            None
        }
    }
}

impl Intersect<LineSegment> for Ray {
    type Intersection = (P2, Normal);
    fn intersect(&self, other: &LineSegment) -> Option<(P2, Normal)> {
        let oline: Line = other.into();
        let length = (other.get_b() - other.get_a()).norm();
        if let Some((v, r, s)) = self.to_line().intersect(&oline) {
            if r < 0.0 || s < 0.0 || s > length {
                None
            } else {
                Some((v, other.get_normal()))
            }
        } else {
            None
        }
    }
}

impl Intersect<Circle> for Ray {
    type Intersection = OneOrTwo<(P2, Normal)>;
    fn intersect(&self, circle: &Circle) -> Option<OneOrTwo<(P2, Normal)>> {
        if let Some((r, s)) = circle.intersect(&self.to_line()) {
            if r > 0. && s > 0. {
                let rp = self.eval_at_r(r);
                let sp = self.eval_at_r(s);
                let normal_r = Unit::new_normalize(rp - circle.get_origin());
                let normal_s = Unit::new_normalize(sp - circle.get_origin());
                let mut res = OneOrTwo::new((rp, normal_r));
                res.add((sp, normal_s));
                Some(res)
            } else if r > 0. {
                let rp = self.eval_at_r(r);
                let normal_r = Unit::new_normalize(rp - circle.get_origin());
                Some(OneOrTwo::new((rp, normal_r)))
            } else if s > 0. {
                let sp = self.eval_at_r(s);
                let normal_s = Unit::new_normalize(sp - circle.get_origin());
                Some(OneOrTwo::new((sp, normal_s)))
            } else {
                None
            }
        } else {
            None
        }
    }
}

impl Intersect<MCircle> for Ray {
    type Intersection = (P2, Normal);
    fn intersect(&self, mcircle: &MCircle) -> Option<(P2, Normal)> {
        let circle = Circle {
            origin: mcircle.path.get_b(),
            radius: mcircle.radius,
        };
        circle
            .intersect(&self.to_line())
            .map(|(r, s)| smallest_positive_value(r, s).map(|r| { let rp = self.eval_at_r(r); let normal_r = Unit::new_normalize(rp - circle.get_origin());
        (rp, normal_r)
            }))
            .flatten()
    }
}

impl Intersect<Rect> for Ray {
    type Intersection = OneOrTwo<(P2, Normal)>;
    fn intersect(&self, rect: &Rect) -> Option<OneOrTwo<(P2, Normal)>> {
        let mut closest: Option<OneOrTwo<(P2, Normal)>> = None;
        for ls in rect.line_segments().iter() {
            if let Some((v, n)) = self.intersect(ls) {
                if let Some(oot) = closest.as_mut() {
                    if distance(&self.origin, &v) < distance(&self.origin, &oot.get_first().0) {
                        oot.add((v, n));
                    }
                } else {
                    closest = Some(OneOrTwo::new((v, n)));
                }
            }
        }
        closest
    }
}

impl Intersect<AABB> for Ray {
    type Intersection = (P2, Normal);
    fn intersect(&self, rect: &AABB) -> Option<(P2, Normal)> {
        let mut closest: Option<(P2, Normal)> = None;
        for ls in rect.line_segments().iter() {
            if let Some((v, n)) = self.intersect(ls) {
                if let Some((cv, _)) = closest {
                    if distance(&self.origin, &v) < distance(&self.origin, &cv) {
                        closest = Some((v, n));
                    } 
                }else {
                        closest = Some((v, n));
                    }
            }
        }
        closest
    }
}

impl Intersect<Logic> for Ray {
    type Intersection = ((P2, Normal), Vec<(P2, Normal)>, Geo);
    fn intersect(&self, l: &Logic) -> Option<Self::Intersection> {
        let a = l.get_a();
        let b = l.get_b();
        let is_inside = l.contains(&self.origin);
        let isa: Option<Vec<(P2, Normal)>> = self.intersect(&a);
        let isb = self.intersect(&b);
        let match_ab = |(p, v, w): ((P2, Normal), Vec<(P2, Normal)>, Which)| match w {
            Which::A => (p, v, a.clone()),
            Which::B => (p, v, b.clone()),
        };
        match l.op {
            LogicOp::And => {
                let isa: Option<Vec<(P2, Normal)>> = isa.map(|va| va.into_iter().filter(|(pa, _)| b.contains(pa)).collect());
                let isb: Option<Vec<(P2, Normal)>> = isb.map(|vb| vb.into_iter().filter(|(pb, _)| a.contains(pb)).collect());
                nearest_option(&self.origin, isa, isb).map(match_ab)
            }
            LogicOp::Or => {
                if is_inside {
                    farthest_option(&self.origin, isa, isb).map(match_ab)
                } else {
                    nearest_option(&self.origin, isa, isb).map(match_ab)
                }
            }
            LogicOp::AndNot => {
                let isb = isb.map(|v| v.into_iter().filter(|(p, _)| a.contains(p)).collect());
                let isa = isa.map(|v| v.into_iter().filter(|(p, _)| !b.contains(p)).collect());
                nearest_option(&self.origin, isa, isb).map(match_ab)
            }
        }
    }
}

impl Intersect<Geo> for Ray {
    type Intersection = Vec<(P2, Normal)>;
    fn intersect(&self, other: &Geo) -> Option<Self::Intersection> {
        match other {
            Geo::GeoRay(ray) => self.intersect(ray).map(|p| vec![p]),
            Geo::GeoRect(rect) => self.intersect(rect).map(|oot| oot.into_vec()),
            Geo::GeoLineSegment(ls) => self.intersect(ls).map(|p| vec![p]),
            Geo::GeoCircle(c) => self.intersect(c).map(|oot| oot.into_vec()),
            Geo::GeoMCircle(mc) => self.intersect(mc).map(|p| vec![p]),
            Geo::GeoPoint(_) => None,
            Geo::GeoLogic(l) => self.intersect(l).map(|(_nearest, v, _geo)| v),
        }
    }
}

impl HasOrigin for Ray {
    fn get_origin(&self) -> P2 {
        self.origin
    }
    fn set_origin(&mut self, origin: P2) {
        self.origin = origin;
    }
}

impl Rotate for Ray {
    fn get_rotation(&self) -> V2 {
        self.get_direction()
    }
    fn set_rotation(&mut self, x_axis: &V2) {
        self.set_direction(*x_axis);
    }
}

impl Scale for Ray {
    fn scale(&mut self, _scale_x: Float, _scale_y: Float) {}
    fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
        self.origin.scale(scale_x, scale_y);
    }
}

impl ClosestPoint for Ray {
    fn closest_point_to(&self, p: &P2) -> P2 {
        let po = p - self.origin;
        let r = self.direction.dot(&po);
        if r < 0. {
            self.origin
        } else {
            self.origin + r * self.get_direction()
        }
    }
}

impl Distance for Ray {
    fn distance(&self, p: &P2) -> Float {
        let normal = self.get_normal();
        let amp = self.origin - p;
        let dist_ap = amp.norm();
        if amp.dot(&self.direction) > 0.0 {
            return dist_ap;
        }
        let dist_line = amp.dot(&normal).abs();
        dist_line.min(dist_ap)
    }
}

impl Distribution<Ray> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Ray {
        let a = rng.gen();
        let direction = rng.gen();
        Ray::from_origin(a, direction)
    }
}

impl HasGeometry for Ray {
    fn get_geometry(&self) -> Geo {
        Geo::GeoRay(*self)
    }
}

impl CanCollideWith<Line> for Ray {}
impl CanCollideWith<Ray> for Ray {}
impl CanCollideWith<LineSegment> for Ray {}
impl CanCollideWith<Circle> for Ray {}
impl CanCollideWith<Rect> for Ray {}
impl CanCollideWith<AABB> for Ray {}
impl CanCollideWith<MCircle> for Ray {}

impl GeoT for Ray {}
