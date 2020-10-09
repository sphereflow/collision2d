use super::aabb::AABB;
use super::circle::Circle;
use super::line::Line;
use super::line_segment::LineSegment;
use super::traits::*;
use super::*;
use crate::utils::*;
use rand::distributions::{Distribution, Standard};
use rand::Rng;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn min_r_transform_gl() {
        for i in 1..1000 {
            let min_r = i as Float / 1000.0;
            for j in 1..=1000 {
                let val = j as Float / 1000.0;
                // println!("val: {}, min_r: {}, gl: {}", val, min_r, to_global(min_r, to_local(min_r, val)));
                assert!((to_global(min_r, to_local(min_r, val)) - val).abs() < 0.0001);
            }
        }
    }

    #[test]
    fn min_r_transform_lg() {
        for i in 1..1000 {
            let min_r = i as Float / 1000.0;
            for j in 1..=1000 {
                let val = j as Float / 1000.0;
                assert!((to_local(min_r, to_global(min_r, val)) - val).abs() < 0.001);
            }
        }
    }
}

fn to_global(min_r: Float, val: Float) -> Float {
    val + min_r * (1.0 - val)
}

fn to_local(min_r: Float, val: Float) -> Float {
    // 1.0 - ((1.0 - val) / 1.0 - min_r)
    (val - min_r) / (1.0 - min_r)
}

/// moving circle
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct MCircle {
    pub radius: Float,
    pub path: LineSegment,
    pub speed_vector: V2,
    pub min_r: Float, // if a collision already took place path.a's r-value is no longer 0
    pub weight: Float,
}

impl MCircle {
    pub fn update(&mut self, frame_time: Float) {
        let new_a = self.path.get_b();
        self.min_r = 0.0;
        self.path = LineSegment::from_ab(new_a, new_a + self.speed_vector * frame_time);
    }
}

impl ReflectOn<Circle> for MCircle {
    fn reflect_on_normal(&self, c: &Circle) -> Option<(MCircle, V2)> {
        let mut reflection_circle = *c;
        let mut inside: bool = false;
        if self.radius + (self.path.get_a() - c.origin).norm() < c.radius {
            inside = c.radius > self.radius;
        }
        if inside {
            reflection_circle.radius -= self.radius;
        } else {
            reflection_circle.radius += self.radius;
        }
        self.path
            .reflect_on_normal(&reflection_circle)
            .map(|(mut path, normal)| {
                let min_r = to_global(self.min_r, 1.0 - path.length() / self.path.length());
                path.shorten_towards_b(0.999);
                (
                    MCircle {
                        path,
                        radius: self.radius,
                        speed_vector: self.speed_vector
                            - 2.0 * (normal.dot(&self.speed_vector)) * normal,
                        min_r,
                        weight: self.weight,
                    },
                    normal,
                )
            })
    }
}

impl ReflectOn<Line> for MCircle {
    fn reflect_on_normal(&self, line: &Line) -> Option<(MCircle, V2)> {
        // shift the line by the distance of the radius of the moving circle towards the moving circle
        let mut shiftv: V2 = line.get_normal().into_inner();
        if shiftv.dot(&(self.path.get_a() - line.origin)) < 0. {
            shiftv *= -1.;
        }
        let mut shifted = *line;
        shifted.origin += shiftv * self.radius;

        // try to reflect the moving circles path on the shifted linesegment
        self.path
            .reflect_on_normal(&shifted)
            .map(|(path_b, normal)| {
                (
                    MCircle {
                        radius: self.radius,
                        speed_vector: self.speed_vector
                            - 2.0 * normal.dot(&self.speed_vector) * normal,
                        path: path_b,
                        min_r: to_global(self.min_r, 1.0 - path_b.length() / self.path.length()),
                        weight: self.weight,
                    },
                    normal,
                )
            })
    }
}

impl ReflectOn<Ray> for MCircle {
    fn reflect_on_normal(&self, ray: &Ray) -> Option<(MCircle, V2)> {
        // shift the linesegment by the distance of the radius of the moving circle towards the moving circle
        let mut shiftv: V2 = ray.get_normal().into_inner();
        if shiftv.dot(&(self.path.get_a() - ray.get_origin())) < 0. {
            shiftv *= -1.;
        }
        let mut shifted = *ray;
        shifted.shift(shiftv * self.radius);

        // longest path after the collision
        let mut longest_path: Option<(LineSegment, V2)> = None;

        // try to reflect the moving circles path on the shifted ray
        if let Some((path_b, normal)) = self.path.reflect_on_normal(&shifted) {
            if path_b.length() > longest_path.map_or(0., |(path, _)| path.length()) {
                longest_path = Some((path_b, normal));
            }
        }

        // create a circle representing the origin of the ray
        let ca = Circle {
            origin: ray.get_origin(),
            radius: self.radius,
        };

        // try to reflect the moving circles path on the origin circle
        if let Some((path_b, normal)) = self.path.reflect_on_normal(&ca) {
            if path_b.length() > longest_path.map_or(0., |(lpath, _)| lpath.length()) {
                longest_path = Some((path_b, normal));
            }
        }

        longest_path.map(|(mut lpath, normal)| {
            lpath.shorten_towards_b(0.999);
            (
                MCircle {
                    radius: self.radius,
                    speed_vector: self.speed_vector - 2.0 * normal.dot(&self.speed_vector) * normal,
                    path: lpath,
                    min_r: to_global(self.min_r, 1.0 - lpath.length() / self.path.length()),
                    weight: self.weight,
                },
                normal,
            )
        })
    }
}

impl ReflectOn<LineSegment> for MCircle {
    fn reflect_on_normal(&self, ls: &LineSegment) -> Option<(MCircle, V2)> {
        // shift the linesegment by the distance of the radius of the moving circle towards the moving circle
        let mut shiftv: V2 = ls.get_normal().into_inner();
        if shiftv.dot(&(self.path.get_a() - ls.get_a())) < 0. {
            shiftv *= -1.;
        }
        let mut shifted = *ls;
        shifted.shift(shiftv * self.radius);

        // longest path after the collision
        let mut longest_path: Option<(LineSegment, V2)> = None;

        // try to reflect the moving circles path on the shifted linesegment
        if let Some((path_b, normal)) = self.path.reflect_on_normal(&shifted) {
            if path_b.length() > longest_path.map_or(0., |(path, _)| path.length()) {
                longest_path = Some((path_b, normal));
            }
        }

        // create two circles representing the corners of the linesegment
        let ca = Circle {
            origin: ls.get_a(),
            radius: self.radius,
        };
        let cb = Circle {
            origin: ls.get_b(),
            radius: self.radius,
        };

        // try to reflect the moving circles path on the corner circles
        if let Some((path_b, normal)) = self.path.reflect_on_normal(&ca) {
            if path_b.length() > longest_path.map_or(0., |(lpath, _)| lpath.length()) {
                longest_path = Some((path_b, normal));
            }
        }
        if let Some((path_b, normal)) = self.path.reflect_on_normal(&cb) {
            if path_b.length() > longest_path.map_or(0., |(lpath, _)| lpath.length()) {
                longest_path = Some((path_b, normal));
            }
        }

        longest_path.map(|(mut lpath, normal)| {
            lpath.shorten_towards_b(0.999);
            (
                MCircle {
                    radius: self.radius,
                    speed_vector: self.speed_vector - 2.0 * normal.dot(&self.speed_vector) * normal,
                    path: lpath,
                    min_r: to_global(self.min_r, 1.0 - lpath.length() / self.path.length()),
                    weight: self.weight,
                },
                normal,
            )
        })
    }

    fn get_extra_data_reflect_on(&self, ls: &LineSegment) -> Vec<Geo> {
        // shift the linesegment by the distance of the radius of the moving circle towards the moving circle
        let mut shiftv: V2 = ls.get_normal().into_inner();
        if shiftv.dot(&(self.path.get_a() - ls.get_a())) < 0. {
            shiftv *= -1.;
        }
        let mut shifted = *ls;
        shifted.shift(shiftv * self.radius);
        let mut ret = Vec::new();
        ret.push(Geo::GeoLineSegment(shifted));
        ret.push(Geo::GeoLineSegment(self.path));
        ret
    }
}

impl ReflectOn<Rect> for MCircle {
    fn reflect_on_normal(&self, rect: &Rect) -> Option<(MCircle, V2)> {
        let [mut r, mut b, mut l, mut t] = rect.line_segments();
        // find out if the MCircle starts inside the rect
        // if the MCircle is inside the Rect this variable will shift the linesegments inwards
        // the rect has to be at least big enough to contain the MCircle
        let x_shift = rect.x_axis.into_inner() * self.radius;
        let y_shift = rect.get_y_axis() * self.radius;
        let mut inside = false;
        // if the Rect is not too small
        if !(rect.height < 2. * self.radius || rect.width < 2. * self.radius) {
            let mut small_rect: Rect = *rect;
            small_rect.width -= 2. * self.radius;
            small_rect.height -= 2. * self.radius;
            inside = small_rect.contains(self.path.get_a());
        }

        if inside {
            // pull in the linesegments of rect by radius
            r.shift(-x_shift);
            b.shift(y_shift);
            l.shift(x_shift);
            t.shift(-y_shift);
        } else {
            // pull out the linesegments of rect by radius
            r.shift(x_shift);
            b.shift(-y_shift);
            l.shift(-x_shift);
            t.shift(y_shift);
        }

        // longest path after the collision
        let mut longest_path: Option<(LineSegment, V2)> = None;
        for segment in [r, b, t, l].iter() {
            if let Some((path_b, normal)) = self.path.reflect_on_normal(segment) {
                if path_b.length() > longest_path.map_or(0., |(path, _)| path.length()) {
                    longest_path = Some((path_b, normal));
                }
            }
        }

        // if the MCircle is inside the Rect the corner circles are not needed
        if !inside {
            // transform rects points to circles
            let circles: Vec<_> = rect
                .points()
                .iter()
                .map(|&p| Circle {
                    origin: p,
                    radius: self.radius,
                })
                .collect();

            // try to reflect on corner circles
            for corner in circles.into_iter() {
                if let Some((path_b, normal)) = self.path.reflect_on_normal(&corner) {
                    if path_b.length() > longest_path.map_or(0., |(path, _)| path.length()) {
                        longest_path = Some((path_b, normal));
                    }
                }
            }
        }

        longest_path.map(|(mut path, normal)| {
            path.shorten_towards_b(0.999);
            (
                MCircle {
                    radius: self.radius,
                    speed_vector: self.speed_vector - 2.0 * normal.dot(&self.speed_vector) * normal,
                    path,
                    min_r: to_global(self.min_r, 1.0 - path.length() / self.path.length()),
                    weight: self.weight,
                },
                normal,
            )
        })
    }
}

impl ReflectOn<AABB> for MCircle {
    fn reflect_on_normal(&self, other: &AABB) -> Option<(Self, V2)> {
        self.reflect_on_normal(&other.to_rect())
    }
}

impl ReflectOn<MCircle> for MCircle {
    // this does a 2 dimensional elastic collision (not considering rotation)
    fn reflect_on_normal(&self, other: &MCircle) -> Option<(MCircle, V2)> {
        let mut sa = self.path.get_a();
        let mut oa = other.path.get_a();

        let min_r = Float::max(self.min_r, other.min_r);

        if self.min_r < other.min_r {
            // adjust self.a and related
            let r = 1.0 - ((1.0 - other.min_r) / (1.0 - self.min_r));
            sa = self.path.eval_at_r(r);
        } else if other.min_r < self.min_r {
            // adjust other.a and related
            let r = 1.0 - ((1.0 - self.min_r) / (1.0 - other.min_r));
            oa = other.path.eval_at_r(r);
        }

        let sd = self.path.get_b() - sa;
        let od = other.path.get_b() - oa;

        let da = sa - oa;
        let dd = sd - od;

        let dist_sq = (self.radius + other.radius).powi(2);
        let div = dd.x.powi(2) + dd.y.powi(2);
        let q = (da.x.powi(2) + da.y.powi(2) - dist_sq) / div;
        let ph = (da.x * dd.x + da.y * dd.y) / div;
        let discriminant = ph.powi(2) - q;
        let mut opt_gr = None;
        if discriminant >= 0.0 {
            let root = discriminant.sqrt();
            let r1 = -ph - root;
            let r2 = -ph + root;
            let gr1 = to_global(min_r, r1);
            let gr2 = to_global(min_r, r2);
            // println!("r1: {}, r2: {}, min_r: {}, div: {}", r1, r2, min_r, div);
            if between(gr1, min_r, 1.0) {
                opt_gr = Some(gr1);
            }
            if between(gr2, min_r, 1.0) && gr2 < gr1 {
                opt_gr = Some(gr2);
            }
        }
        match opt_gr {
            Some(gr) => {
                let other_path = LineSegment::from_ab(oa, other.path.get_b());
                // get intersection point
                let intersection_point = self.path.eval_at_r(to_local(self.min_r, gr));
                let other_intersection_point = other_path.eval_at_r(to_local(self.min_r, gr));
                let diff = intersection_point - other_intersection_point;
                let mass_ratio = 2.0 * other.weight / (self.weight + other.weight);
                let deflection_v = self.path.get_direction()
                    - mass_ratio * diff * (diff.dot(&dd) / diff.norm_squared());
                let diff_v = self.speed_vector - other.speed_vector;
                let new_speed_v = self.speed_vector
                    - mass_ratio * diff * (diff_v.dot(&diff) / diff.norm_squared());
                let mut path =
                    LineSegment::from_ab(intersection_point, intersection_point + deflection_v);
                path.shorten_towards_b(0.999);

                Some((
                    MCircle {
                        radius: self.radius,
                        path,
                        speed_vector: new_speed_v,
                        min_r: gr,
                        weight: self.weight,
                    },
                    diff,
                ))
            }
            None => None,
        }
    }
}

impl CanCollideWith<Line> for MCircle {}
impl CanCollideWith<Ray> for MCircle {}
impl CanCollideWith<LineSegment> for MCircle {}
impl CanCollideWith<Circle> for MCircle {}
impl CanCollideWith<Rect> for MCircle {}
impl CanCollideWith<AABB> for MCircle {}
impl CanCollideWith<MCircle> for MCircle {}

impl GeoT for MCircle {}

impl HasOrigin for MCircle {
    fn get_origin(&self) -> P2 {
        self.path.get_a()
    }
    fn set_origin(&mut self, origin: P2) {
        let v = self.path.get_a() - origin;
        self.path.shift(v);
    }
}

impl Contains for MCircle {
    fn contains(&self, p: P2) -> bool {
        self.path.distance(p) < self.radius
    }
}

impl Distance for MCircle {
    fn distance(&self, p: P2) -> Float {
        self.path.distance(p) - self.radius
    }
}

impl Scale for MCircle {
    fn scale(&mut self, scale_x: Float, scale_y: Float) {
        self.radius *= (scale_x + scale_y) * 0.5;
    }
    fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
        let old_a = self.path.get_a();
        let mut new_a = self.path.get_a();
        new_a.scale_position(scale_x, scale_y);
        let shift = new_a - old_a;
        self.path.shift(shift);
    }
}

impl Distribution<MCircle> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> MCircle {
        let mut speed_vector: V2 = rng.gen();
        speed_vector *= 10.0;
        let radius = rng.gen();
        MCircle {
            radius,
            path: rng.gen(),
            speed_vector,
            min_r: rng.gen(),
            weight: radius * radius,
        }
    }
}
