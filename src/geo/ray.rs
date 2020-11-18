extern crate nalgebra as na;

use super::*;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Ray {
    origin: P2,
    normal: Normal,
    direction: U2,
}

impl Ray {
    pub fn from_origin(origin: P2, direction: V2) -> Ray {
        let normal = Unit::new_normalize(V2::new(-direction.y, direction.x));
        let direction = Unit::new_normalize(direction);
        Ray {
            origin,
            direction,
            normal,
        }
    }

    pub fn to_line(&self) -> Line {
        Line::new_unchecked(self.origin, self.direction, self.normal)
    }

    pub fn into_line(self) -> Line {
        Line::new_unchecked(self.origin, self.direction, self.normal)
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

    pub fn eval_at_r(&self, r: Float) -> P2 {
        self.origin + r * self.direction.into_inner()
    }

    pub fn reflect(&self, i: &P2, normal: &Normal) -> Self {
        let n = normal.into_inner();
        let d = self.direction.into_inner();
        let r = d - 2. * d.dot(&n) * n;
        Ray::from_origin(*i, r).offset()
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

impl<T> ReflectOn<T> for Ray
where
    Ray: Intersect<T, Intersection = (P2, Normal)>,
{
    fn reflect_on_normal_intersect(&self, other: &T) -> Option<(Self, V2, P2)> {
        self.intersect(other)
            .map(|(i, normal)| (self.reflect(&i, &normal), normal.into_inner(), i))
    }
}

impl ReflectOn<Circle> for Ray {
    fn reflect_on_normal_intersect(&self, circle: &Circle) -> Option<(Ray, V2, P2)> {
        self.intersect(circle).map(|oot| {
            let (i, normal) = oot.get_first();
            (self.reflect(&i, &normal), normal.into_inner(), i)
        })
    }
}

impl ReflectOn<Rect> for Ray {
    fn reflect_on_normal_intersect(&self, rect: &Rect) -> Option<(Ray, V2, P2)> {
        let mut closest: Option<(Ray, V2, P2)> = None;
        for ls in rect.line_segments().iter() {
            if let Some((ray, n, i)) = self.reflect_on_normal_intersect(ls) {
                if let Some((_clray, _, cli)) = closest {
                    if distance_squared(&i, &self.origin) < distance_squared(&cli, &self.origin) {
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
                    if distance_squared(&ray.origin, &self.origin)
                        < distance_squared(&clray.origin, &self.origin)
                    {
                        closest = Some((ray, n, i));
                    }
                } else {
                    closest = Some((ray, n, i))
                }
            }
        }
        closest
    }
}

impl ReflectOn<ConvexPolygon> for Ray {
    fn reflect_on_normal_intersect(&self, cpoly: &ConvexPolygon) -> Option<(Self, V2, P2)> {
        self.intersect(cpoly).map(|oot| {
            let (i, normal) = oot.get_first();
            (self.reflect(&i, &normal), normal.into_inner(), i)
        })
    }
}

impl ReflectOn<Logic> for Ray {
    fn reflect_on_normal_intersect(&self, l: &Logic) -> Option<(Self, V2, P2)> {
        self.intersect(l).map(|v| {
            let (mut p, mut normal) = v[0];
            let mut min_dist_sq = distance_squared(&self.origin, &p);
            for (vp, vnormal) in v {
                let new_dist_sq = distance_squared(&self.origin, &vp);
                if min_dist_sq > new_dist_sq {
                    min_dist_sq = new_dist_sq;
                    p = vp;
                    normal = vnormal;
                }
            }
            (self.reflect(&p, &normal), normal.into_inner(), p)
        })
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
        if let Some((v, r, s)) = self.to_line().intersect(&oline) {
            if r < 0.0 || s < 0.0 || s * s > other.length_sq() {
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
        if let Some((r, s)) = circle
            .intersect(&self.to_line())
            .and_then(|(r, s)| smallest_positive_sort(r, s))
        {
            // at this point r is guaranteed to be positive
            let rp = self.eval_at_r(r);
            let normal_r = Unit::new_normalize(rp - circle.get_origin());
            let mut res = OneOrTwo::new((rp, normal_r));
            if s > 0. {
                let sp = self.eval_at_r(s);
                let normal_s = Unit::new_normalize(sp - circle.get_origin());
                res.add((sp, normal_s));
            }
            Some(res)
        } else {
            None
        }
    }
}

impl Intersect<MCircle> for Ray {
    type Intersection = (P2, Normal);
    fn intersect(&self, mcircle: &MCircle) -> Option<(P2, Normal)> {
        let circle = mcircle.circle_b();
        circle
            .intersect(&self.to_line())
            .map(|(r, s)| {
                smallest_positive_sort(r, s).map(|(r, _)| {
                    let rp = self.eval_at_r(r);
                    let normal_r = Unit::new_normalize(rp - circle.origin);
                    (rp, normal_r)
                })
            })
            .flatten()
    }
}

impl Intersect<Rect> for Ray {
    type Intersection = OneOrTwo<(P2, Normal)>;
    fn intersect(&self, rect: &Rect) -> Option<Self::Intersection> {
        let mut intersect: Option<Self::Intersection> = None;
        for ls in rect.line_segments().iter() {
            if let Some((v, n)) = self.intersect(ls) {
                if let Some(oot) = intersect.as_mut() {
                    oot.add((v, n));
                    break;
                } else {
                    intersect = Some(OneOrTwo::new((v, n)));
                }
            }
        }
        intersect
    }
}

impl Intersect<AABB> for Ray {
    type Intersection = OneOrTwo<(P2, Normal)>;
    fn intersect(&self, aabb: &AABB) -> Option<Self::Intersection> {
        let mut closest: Option<Self::Intersection> = None;
        for ls in aabb.line_segments().iter() {
            if let Some((v, n)) = self.intersect(ls) {
                if let Some(oot) = closest.as_mut() {
                    if distance_squared(&self.origin, &v)
                        < distance_squared(&self.origin, &oot.get_first().0)
                    {
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

impl Intersect<ConvexPolygon> for Ray {
    type Intersection = OneOrTwo<(P2, Normal)>;
    fn intersect(&self, cpoly: &ConvexPolygon) -> Option<Self::Intersection> {
        cpoly.intersect(self)
    }
}

impl Intersect<Logic> for Ray {
    type Intersection = Vec<(P2, Normal)>;
    fn intersect(&self, l: &Logic) -> Option<Self::Intersection> {
        let a = l.get_a();
        let b = l.get_b();
        let isa = self.intersect(&a);
        let isb = self.intersect(&b);
        match l.op {
            LogicOp::And => {
                let isa = isa.map(|va| va.into_iter().filter(|(pa, _)| b.contains(pa)).collect());
                let isb = isb.map(|vb| vb.into_iter().filter(|(pb, _)| a.contains(pb)).collect());
                extend_opt_vec(isa, isb)
            }
            LogicOp::Or => {
                let isa = isa.map(|va| va.into_iter().filter(|(pa, _)| !b.contains(pa)).collect());
                let isb = isb.map(|vb| vb.into_iter().filter(|(pb, _)| !a.contains(pb)).collect());
                extend_opt_vec(isa, isb)
            }
            LogicOp::AndNot => {
                let isb = isb.map(|v| {
                    v.into_iter()
                        .filter(|(pb, _)| a.contains(pb))
                        .map(|(pb, n)| (pb, -n))
                        .collect()
                });
                let isa = isa.map(|v| v.into_iter().filter(|(pa, _)| !b.contains(pa)).collect());
                extend_opt_vec(isa, isb)
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
            Geo::GeoLogic(l) => self.intersect(l),
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

impl HasDirection for Ray {
    fn get_direction(&self) -> U2 {
        self.direction
    }

    fn set_direction(&mut self, direction: U2) {
        self.normal = Unit::new_unchecked(V2::new(-direction.y, direction.x));
        self.direction = direction;
    }
}

impl HasNormal for Ray {
    fn get_normal(&self) -> Normal {
        self.normal
    }
    fn set_normal(&mut self, normal: Normal) {
        self.normal = normal;
        self.direction = Unit::new_unchecked(V2::new(normal.y, -normal.x));
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
            self.origin + r * self.get_direction().into_inner()
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
