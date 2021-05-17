extern crate nalgebra as na;

use super::*;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Line {
    pub origin: P2,
    direction: U2,
    normal: Normal,
}

impl Line {
    pub fn new(origin: P2, direction: V2) -> Line {
        let normal = Unit::new_normalize(V2::new(-direction.y, direction.x));
        let direction = Unit::new_normalize(direction);
        Line {
            origin,
            direction,
            normal,
        }
    }

    pub fn new_unchecked(origin: P2, direction: U2, normal: Normal) -> Line {
        Line {
            origin,
            direction,
            normal,
        }
    }

    pub fn eval_at_r(&self, r: Float) -> P2 {
        self.origin + r * self.direction.into_inner()
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

impl HasDirection for Line {
    fn get_direction(&self) -> U2 {
        self.direction
    }

    fn set_direction(&mut self, direction: U2) {
        self.normal = Unit::new_unchecked(V2::new(-direction.y, direction.x));
        self.direction = direction;
    }
}

impl HasNormal for Line {
    fn get_normal(&self) -> Normal {
        self.normal
    }
    fn set_normal(&mut self, normal: Normal) {
        self.normal = normal;
        self.direction = Unit::new_unchecked(V2::new(normal.y, -normal.x));
    }
}

impl Scale for Line {
    fn scale(&mut self, _scale_x: Float, _scale_y: Float) {}
    fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
        self.origin.scale_position(scale_x, scale_y);
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
        let mut inv =
            Matrix2::from_columns(&[self.direction.into_inner(), -other.direction.into_inner()]);
        if inv.try_inverse_mut() {
            let rs = inv * (other.origin - self.origin);
            Some((
                self.origin + (self.direction.into_inner() * rs.x),
                rs.x,
                rs.y,
            ))
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
        let dmat =
            Matrix2::from_columns(&[self.direction.into_inner(), -other.direction.into_inner()]);
        match inverse(&dmat) {
            Some(inv) => {
                let n = other.get_normal().into_inner();
                let rs = inv * (other.origin - self.origin);
                let intersection = self.origin + self.direction.into_inner() * rs.x;
                let c1 = n.x * n.x - n.y * n.y;
                let c2 = 2. * n.x * n.y;
                let reflected = V2::new(
                    self.direction.y * c2 - self.direction.x * c1,
                    self.direction.x * c2 + self.direction.y * c1,
                );
                let normal_line = V2::new(-self.direction.y, self.direction.x);
                let new_line = Line {
                    origin: intersection,
                    direction: Unit::new_normalize(reflected),
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
        let dmat = Matrix2::from_columns(&[
            self.direction.into_inner(),
            -other.get_direction().into_inner(),
        ]);
        match inverse(&dmat) {
            Some(inv) => {
                let n = other.get_normal().into_inner();
                let rs = inv * (other.get_origin() - self.origin);
                if rs[1] < 0. {
                    return None;
                }
                let intersection = self.origin + self.direction.into_inner() * rs.x;
                let c1 = n.x * n.x - n.y * n.y;
                let c2 = 2. * n.x * n.y;
                let reflected = V2::new(
                    self.direction.y * c2 - self.direction.x * c1,
                    self.direction.x * c2 + self.direction.y * c1,
                );
                let normal_line = V2::new(-self.direction.y, self.direction.x);
                let new_line = Line {
                    origin: intersection,
                    direction: Unit::new_normalize(reflected),
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
        let dmat = Matrix2::from_columns(&[
            self.direction.into_inner(),
            -other.get_direction().into_inner(),
        ]);
        match inverse(&dmat) {
            Some(inv) => {
                let n = other.get_normal().into_inner();
                let rs = inv * (other.get_a() - self.origin);
                if rs[1] < 0. || rs[1] > 1. {
                    return None;
                }
                let intersection = self.origin + self.get_direction().into_inner() * rs.x;
                let c1 = n.x * n.x - n.y * n.y;
                let c2 = 2. * n.x * n.y;
                let reflected = V2::new(
                    self.direction.y * c2 - self.direction.x * c1,
                    self.direction.x * c2 + self.direction.y * c1,
                );
                let normal_line = V2::new(-self.direction.y, self.direction.x);
                let new_line = Line {
                    origin: intersection,
                    direction: Unit::new_normalize(reflected),
                    normal: Unit::new_normalize(normal_line),
                };
                Some((new_line, n, intersection))
            }
            None => None,
        }
    }
}

impl ClosestPoint for Line {
    fn closest_point_to(&self, p: &P2) -> P2 {
        let po = p - self.origin;
        self.origin + self.direction.dot(&po) * self.get_direction().into_inner()
    }
}

impl Distance for Line {
    fn distance(&self, p: &P2) -> Float {
        V2::dot(&(p - self.origin), &self.get_normal()).abs()
    }
}

impl Distribution<Line> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Line {
        let direction: U2 = rng.gen();
        let normal = Unit::new_normalize(V2::new(-direction.y, direction.x));
        Line {
            origin: rng.gen(),
            direction,
            normal,
        }
    }
}

impl ReflectOn<Circle> for Line {}
impl ReflectOn<Rect> for Line {}
impl ReflectOn<Aabb> for Line {}
impl ReflectOn<MCircle> for Line {}

impl CanCollideWith<Line> for Line {}
impl CanCollideWith<Ray> for Line {}
impl CanCollideWith<LineSegment> for Line {}
impl CanCollideWith<Circle> for Line {}
impl CanCollideWith<Rect> for Line {}
impl CanCollideWith<Aabb> for Line {}
impl CanCollideWith<MCircle> for Line {}

impl GeoT for Line {}
