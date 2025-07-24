extern crate nalgebra as na;

use super::*;

#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct Ellipse {
    pub origin: P2,
    pub a: Float,
    pub b: Float,
    pub rot: Rot2,
}

impl Ellipse {
    pub fn eval_at_t(&self, t: Float) -> P2 {
        let (s, c) = t.sin_cos();
        P2::new(self.a * c, self.b * s)
    }

    /// returns a tangent line with a local rotation angle of -alpha
    /// helper function for get_aabb
    pub fn tangent_at_angle(&self, alpha: Float) -> Line {
        let (dy, dx) = alpha.sin_cos();
        let is_shallow = dy.abs() < dx.abs();
        let a_sq = self.a * self.a;
        let b_sq = self.b * self.b;
        let div = dx * dx * b_sq + dy * dy * a_sq;
        // steep angle => set pl.y to zero and calculate pl.x
        let pl = if !is_shallow {
            let plx_sq = -a_sq * div / (dx.powi(2) * b_sq - div);
            P2::new(plx_sq.sqrt(), 0.)
        }
        // shallow angle => set pl.x to zero and calculate pl.y
        else {
            let ply_sq = -b_sq * div / (dy.powi(2) * a_sq - div);
            P2::new(0., ply_sq.sqrt())
        };
        let d = V2::new(dx, dy);
        Line::new(pl, d)
    }

    fn normal_from_local_point(&self, p1: P2) -> U2 {
        U2::new_normalize(V2::new(p1.x * self.b.powi(2), p1.y * self.a.powi(2)))
    }
}

impl HasOrigin for Ellipse {
    fn get_origin(&self) -> P2 {
        self.origin
    }
    fn set_origin(&mut self, origin: P2) {
        self.origin = origin;
    }
}

impl Scale for Ellipse {
    fn scale(&mut self, scale_x: Float, scale_y: Float) {
        self.a *= scale_x;
        self.b *= scale_y;
    }

    fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
        self.origin.x *= scale_x;
        self.origin.y *= scale_y;
    }
}

impl Rotate for Ellipse {
    fn get_rotation(&self) -> Rot2 {
        self.rot
    }

    fn set_rotation(&mut self, rotation: &Rot2) {
        self.rot = *rotation;
    }
}

impl Contains for Ellipse {
    fn contains(&self, p: &P2) -> bool {
        let pl = self.rot.inverse() * (p - self.origin);
        pl.x.powi(2) / self.a.powi(2) + pl.y.powi(2) / self.b.powi(2) <= 1.0
    }
}

impl ClosestPoint for Ellipse {
    fn closest_point_to(&self, p: &P2) -> P2 {
        let local = self.rot.inverse() * (p - self.origin);
        let x = local.x;
        let y = local.y;
        if y > 0. {
            let r = self.b * ((x / y) / (x * x + y * y).sqrt());
            local_to_global(&self.origin, &self.rot, &(r * local))
        } else if x > 0. {
            let r = self.a * ((y / x) / (x * x + y * y).sqrt());
            local_to_global(&self.origin, &self.rot, &(r * local))
        } else if self.a < self.b {
            local_to_global(&self.origin, &self.rot, &V2::new(self.a, 0.))
        } else {
            local_to_global(&self.origin, &self.rot, &V2::new(0., self.b))
        }
    }
}

impl Mirror for Ellipse {
    fn mirror_x(&self) -> Self {
        Ellipse {
            origin: self.origin,
            a: self.a,
            b: self.b,
            rot: self.rot.inverse(),
        }
    }

    fn mirror_y(&self) -> Self {
        self.mirror_x()
    }
}

impl Distance for Ellipse {
    fn distance(&self, p: &P2) -> Float {
        let local = self.rot.inverse() * (p - self.origin);
        let x = local.x;
        let y = local.y;
        if y > 0. {
            let r = self.b * ((x / y) / (x * x + y * y).sqrt());
            let point_on_ellipse = local_to_global(&self.origin, &self.rot, &(r * local));
            if r > 1. {
                -distance(&point_on_ellipse, p)
            } else {
                distance(&point_on_ellipse, p)
            }
        } else if x > 0. {
            let r = self.a * ((y / x) / (x * x + y * y).sqrt());
            let point_on_ellipse = local_to_global(&self.origin, &self.rot, &(r * local));
            if r > 1. {
                -distance(&point_on_ellipse, p)
            } else {
                distance(&point_on_ellipse, p)
            }
        } else {
            -self.a.min(self.b)
        }
    }
}

impl Distribution<Ellipse> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Ellipse {
        Ellipse {
            origin: rng.gen(),
            a: rng.gen(),
            b: rng.gen(),
            rot: rng.gen(),
        }
    }
}

impl HasGeometry for Ellipse {
    fn get_geometry(&self) -> Geo {
        Geo::GeoEllipse(*self)
    }
}

impl HasAabb for Ellipse {
    fn get_aabb(&self) -> Aabb {
        // global to local coordinate system of the ellipse => inverse rotation
        let alpha = -self.rot.angle();
        // the circle on which all AABB corners are located
        // local ellipse coordinate system => origin = (0.0, 0.0)
        // the tangent is the top line segment of the AABB
        // in coordinates local to the ellipse
        let tangent = self.tangent_at_angle(alpha);
        let circle = Circle {
            origin: P2::new(0., 0.),
            radius: (self.a * self.a + self.b * self.b).sqrt(),
        };
        let (r, _s) = circle
            .intersect(&tangent)
            .expect("Ellipse::get_aabb circle does not intersect line");
        // rotate tangent back to its position in the AABB (no translation)
        let p = self.rot * tangent.eval_at_r(r);
        Aabb {
            origin: self.origin,
            width: 2. * p.x.abs(),
            height: 2. * p.y.abs(),
        }
    }
}

/// helper function for Ellipse <-> Line intersections
fn quadratic_coefficients(
    plx: Float,
    ply: Float,
    dx: Float,
    dy: Float,
    a: Float,
    b: Float,
) -> (Float, Float) {
    let a_sq = a.powi(2);
    let b_sq = b.powi(2);
    let div = dx.powi(2) * b_sq + dy.powi(2) * a_sq;
    let p = 2. * (dx * plx * b_sq + dy * ply * a_sq) / div;
    let q = (plx.powi(2) * b_sq + ply.powi(2) * a_sq - a_sq * b_sq) / div;
    (p, q)
}

impl Intersect<Line> for Ellipse {
    type Intersection = OneOrTwo<P2>;

    fn intersect(&self, line: &Line) -> Option<Self::Intersection> {
        let rinv = self.rot.inverse();
        // origin of line in local coordinates
        let pl = rinv * (line.origin - self.origin);
        // direction of line in local coordinates
        let d = rinv * line.get_direction();
        let (p, q) = if d.y >= d.x * 10. {
            quadratic_coefficients(pl.x, pl.y, d.x, d.y, self.a, self.b)
        } else {
            // d.y may be too small => flip the direction of the x and y axis
            quadratic_coefficients(pl.y, pl.x, d.y, d.x, self.b, self.a)
        };
        let fs = quadratic_roots(p, q);
        match fs.len() {
            0 => None,
            1 => Some(OneOrTwo::new(line.eval_at_r(fs[0]))),
            _ => {
                let mut oot = OneOrTwo::new(line.eval_at_r(fs[0]));
                oot.add(line.eval_at_r(fs[1]));
                Some(oot)
            }
        }
    }
}

impl Intersect<LineSegment> for Ellipse {
    type Intersection = OneOrTwo<P2>;

    fn intersect(&self, ls: &LineSegment) -> Option<Self::Intersection> {
        let rinv = self.rot.inverse();
        // origin of line in local coordinates
        let pl = rinv * (ls.get_a() - self.origin);
        // direction of line in local coordinates
        let d = rinv * ls.get_direction();
        let (p, q) = if d.y >= d.x {
            quadratic_coefficients(pl.x, pl.y, d.x, d.y, self.a, self.b)
        } else {
            // d.y may be too small => flip the direction of the x and y axis
            quadratic_coefficients(pl.y, pl.x, d.y, d.x, self.b, self.a)
        };
        let mut fs = quadratic_roots(p, q);
        fs.retain(|r| (0.0..1.0).contains(r));
        match fs.len() {
            0 => None,
            1 => Some(OneOrTwo::new(ls.eval_at_r(fs[0]))),
            _ => {
                let mut oot = OneOrTwo::new(ls.eval_at_r(fs[0]));
                oot.add(ls.eval_at_r(fs[1]));
                Some(oot)
            }
        }
    }
}

impl Intersect<Ray> for Ellipse {
    type Intersection = OneOrTwo<Reflection>;

    fn intersect(&self, ray: &Ray) -> Option<Self::Intersection> {
        let rinv = self.rot.inverse();
        // origin of line in local coordinates
        let pl = rinv * (ray.get_origin() - self.origin);
        // direction of line in local coordinates
        let d = rinv * ray.get_direction();
        let (p, q) = if d.y >= d.x {
            quadratic_coefficients(pl.x, pl.y, d.x, d.y, self.a, self.b)
        } else {
            // d.y may be too small => flip the direction of the x and y axis
            quadratic_coefficients(pl.y, pl.x, d.y, d.x, self.b, self.a)
        };
        let mut fs = quadratic_roots(p, q);
        fs.retain(|f| *f >= 0.);
        let local_ray = Ray::from_origin(P2::new(pl.x, pl.y), d.into_inner());
        match fs.len() {
            0 => None,
            1 => {
                let p1: P2 = ray.eval_at_r(fs[0]);
                let n1: U2 = self.rot * self.normal_from_local_point(local_ray.eval_at_r(fs[0]));
                Some(OneOrTwo::new((p1, n1)))
            }
            _ => {
                let p1: P2 = ray.eval_at_r(fs[0]);
                let n1: U2 = self.rot * self.normal_from_local_point(local_ray.eval_at_r(fs[0]));
                let mut oot = OneOrTwo::new((p1, n1));
                let p2: P2 = ray.eval_at_r(fs[1]);
                let n2: U2 = self.rot * self.normal_from_local_point(local_ray.eval_at_r(fs[1]));
                oot.add((p2, n2));
                Some(oot)
            }
        }
    }
}
