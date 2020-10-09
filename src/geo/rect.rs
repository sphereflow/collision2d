extern crate nalgebra as na;

use super::aabb::AABB;
use super::circle::Circle;
use super::line::Line;
use super::line_segment::LineSegment;
use super::mcircle::MCircle;
use super::ray::Ray;
use super::traits::*;
use crate::utils::*;
use na::{distance, Matrix2, Point2, Rotation2, Unit, Vector2};
use rand::distributions::{Distribution, Standard};
use rand::Rng;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Rect {
    pub origin: P2,
    pub x_axis: Normal,
    pub width: Float,
    pub height: Float,
}

pub type RectPoints = [P2; 4];

pub type RectLineSegments = [LineSegment; 4];

impl Rect {
    pub fn new(origin: P2, x_axis: V2, width: Float, height: Float) -> Rect {
        Rect {
            origin,
            x_axis: Unit::new_normalize(x_axis),
            width: width.abs(),
            height: height.abs(),
        }
    }

    pub fn from_tlbr(top: Float, left: Float, bottom: Float, right: Float) -> Self {
        Rect {
            origin: Point2::new((left + right) * 0.5, (top + bottom) * 0.5),
            x_axis: Unit::new_unchecked(Vector2::new(1.0, 0.0)),
            width: (right - left).abs(),
            height: (top - bottom).abs(),
        }
    }

    pub fn set_x_axis(&mut self, v: V2) {
        if !(v[0] == 0.0 && v[1] == 0.0) {
            self.x_axis = Unit::new_normalize(v);
        }
    }

    pub fn get_y_axis(&self) -> V2 {
        let xax = self.x_axis.into_inner();
        Vector2::new(-xax.y, xax.x)
    }

    pub fn points(&self) -> RectPoints {
        let y_offset = Vector2::new(-self.x_axis.y, self.x_axis.x) * self.height * 0.5;
        let x_offset = self.x_axis.into_inner() * self.width * 0.5;
        let top_right = x_offset + y_offset;
        let bottom_right = x_offset - y_offset;
        [
            self.origin + top_right,
            self.origin + bottom_right,
            self.origin - top_right,
            self.origin - bottom_right,
        ]
    }

    pub fn line_segments(&self) -> RectLineSegments {
        let [a, b, c, d] = self.points();
        [
            LineSegment::from_ab(a, b),
            LineSegment::from_ab(b, c),
            LineSegment::from_ab(c, d),
            LineSegment::from_ab(d, a),
        ]
    }

    pub fn seperated_by_line(&self, line: &Line) -> bool {
        let normal = line.get_normal();
        let products: Vec<Float> = self
            .points()
            .iter()
            .map(|p| (p - line.origin).dot(&normal))
            .collect();
        products.iter().all(|x| x.is_sign_positive())
            || products.iter().all(|x| x.is_sign_negative())
    }

    pub fn get_closest_line_segments(&self, other: &Rect) -> [LineSegment; 2] {
        // get the index of the farthest point
        let points = self.points();
        let mut farthest_index = 0;
        let mut farthest = 0.0;
        for (i, p) in points.iter().enumerate() {
            let dist = distance(&other.origin, p);
            if dist > farthest {
                farthest_index = i;
                farthest = dist;
            }
        }
        [
            LineSegment::from_ab(
                points[(farthest_index + 1) % 4],
                points[(farthest_index + 2) % 4],
            ),
            LineSegment::from_ab(
                points[(farthest_index + 2) % 4],
                points[(farthest_index + 3) % 4],
            ),
        ]
    }

    pub fn split_off_closest_point(&self, p: P2) -> (P2, Vec<P2>) {
        let mut min_distance_index = 0;
        let mut min_distance = distance(&self.origin, &p);
        let mut points = self.points();
        for (ix, point) in points.iter().enumerate() {
            let dist = distance(point, &p);
            if dist < min_distance {
                min_distance = dist;
                min_distance_index = ix;
            }
        }
        points.rotate_left(min_distance_index);
        let (closest, rest) = points.split_first().unwrap();
        (*closest, rest.to_vec())
    }

    pub fn separates_rect(&self, other: &Rect) -> bool {
        // get projection matrix + inverse
        // projection => inverse projection => inverse projection in terms of self.x_axis
        // xx yx =>      xx xy =>              xx xy
        // xy yy         yx yy                -xy xx
        let inv_projection =
            Matrix2::new(self.x_axis.x, self.x_axis.y, -self.x_axis.y, self.x_axis.x);
        // transform points of other
        let (xs, ys): (Vec<Float>, Vec<Float>) = other
            .points()
            .iter()
            .map(|p| inv_projection * (p - self.origin))
            .map(|p| (p.x, p.y))
            .unzip();
        // check x <==> width , y <==> height
        let sepx =
            xs.iter().all(|&x| x > self.width * 0.5) || xs.iter().all(|&x| x < -self.width * 0.5);
        let sepy =
            ys.iter().all(|&y| y > self.height * 0.5) || ys.iter().all(|&y| y < -self.height * 0.5);
        sepx || sepy
    }
}

impl Intersect<LineSegment> for Rect {
    type Intersection = P2;
    fn intersect(&self, ls: &LineSegment) -> Option<P2> {
        let mut res;
        for other_segment in self.line_segments().iter() {
            res = ls.intersect(other_segment);
            if res.is_some() {
                return res;
            }
        }
        None
    }
}

impl Intersect<Circle> for Rect {
    type Intersection = (Normal, Float);

    fn intersect(&self, s: &Circle) -> Option<(Normal, Float)> {
        let mut intersection_points: Option<OneOrTwo<P2>> = None;
        for ls in self.line_segments().iter() {
            if let Some(one_or_two) = s.intersect(ls) {
                match intersection_points.as_mut() {
                    Some(oot) => {
                        oot.mappend(one_or_two);
                    }
                    None => intersection_points = Some(one_or_two),
                }
            }
            if let Some((a, b)) = intersection_points.as_ref().and_then(|oot| oot.get_items()) {
                if let Some((first, rest)) = self.points().split_first() {
                    let mut min_dist = distance(&s.origin, first);
                    for p in rest {
                        let dist = distance(&s.origin, p);
                        if dist < min_dist {
                            min_dist = dist;
                        }
                    }
                    return Some((LineSegment::from_ab(a, b).get_normal(), min_dist));
                }
            }
        }
        None
    }
}

impl Intersect<Rect> for Rect {
    type Intersection = ();

    fn intersect(&self, other: &Rect) -> Option<()> {
        if !(self.separates_rect(other) || other.separates_rect(self)) {
            Some(())
        } else {
            None
        }
    }
}

impl HasOrigin for Rect {
    fn get_origin(&self) -> P2 {
        self.origin
    }
    fn set_origin(&mut self, origin: P2) {
        self.origin = origin;
    }
}

impl Scale for Rect {
    fn scale(&mut self, scale_x: Float, scale_y: Float) {
        self.width *= scale_x;
        self.height *= scale_y;
    }
    fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
        self.origin.x *= scale_x;
        self.origin.y *= scale_y;
    }
}

impl Contains for Rect {
    fn contains(&self, p: P2) -> bool {
        let rot = Rotation2::rotation_between(&self.x_axis, &Vector2::new(1.0, 0.0));
        AABB {
            origin: Point2::new(0.0, 0.0),
            width: self.width,
            height: self.height,
        }
        .contains(rot * Point2::from(p - self.origin))
    }
}

impl Distance for Rect {
    fn distance(&self, p: &P2) -> Float {
        let mut min_dist = Float::MAX;
        for ls in self.line_segments().iter() {
            let dist = ls.distance(p);
            if dist < min_dist {
                min_dist = dist;
            }
        }
        if self.contains(*p) {
            -min_dist
        } else {
            min_dist
        }
    }
}

impl Distribution<Rect> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Rect {
        let x_axis: V2 = rng.gen();
        let x_axis = Unit::new_normalize(x_axis);
        Rect {
            origin: rng.gen(),
            x_axis,
            width: rng.gen(),
            height: rng.gen(),
        }
    }
}

impl ReflectOn<Line> for Rect {}
impl ReflectOn<Ray> for Rect {}
impl ReflectOn<LineSegment> for Rect {}
impl ReflectOn<Circle> for Rect {}
impl ReflectOn<Rect> for Rect {}
impl ReflectOn<AABB> for Rect {}
impl ReflectOn<MCircle> for Rect {}

impl CanCollideWith<Line> for Rect {}
impl CanCollideWith<Ray> for Rect {}
impl CanCollideWith<LineSegment> for Rect {}
impl CanCollideWith<Circle> for Rect {}
impl CanCollideWith<Rect> for Rect {}
impl CanCollideWith<AABB> for Rect {}
impl CanCollideWith<MCircle> for Rect {}

impl GeoT for Rect {}
