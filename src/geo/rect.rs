extern crate nalgebra as na;

use super::*;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Rect {
    pub origin: P2,
    pub rotation: Rot2,
    pub width: Float,
    pub height: Float,
}

pub type RectPoints = [P2; 4];

pub type RectLineSegments = [LineSegment; 4];

impl Rect {
    pub fn new(origin: P2, rotation: Rot2, width: Float, height: Float) -> Rect {
        Rect {
            origin,
            rotation,
            width: width.abs(),
            height: height.abs(),
        }
    }

    pub fn from_tlbr(top: Float, left: Float, bottom: Float, right: Float) -> Self {
        Rect {
            origin: P2::new((left + right) * 0.5, (top + bottom) * 0.5),
            rotation: Rot2::identity(),
            width: (right - left).abs(),
            height: (top - bottom).abs(),
        }
    }

    pub fn points(&self) -> RectPoints {
        let x_offset = self.rotation * V2::new(self.width * 0.5, 0.);
        let y_offset = self.rotation * V2::new(0., self.height * 0.5);
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
        // transform points of other
        let (xs, ys): (Vec<Float>, Vec<Float>) = other
            .points()
            .iter()
            .map(|p| self.get_rotation().inverse() * (p - self.origin))
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

impl Intersect<Ray> for Rect {
    type Intersection = OneOrTwo<(P2, Normal)>;
    fn intersect(&self, ray: &Ray) -> Option<OneOrTwo<(P2, Normal)>> {
        ray.intersect(self)
    }
}

impl Intersect<LineSegment> for Rect {
    type Intersection = OneOrTwo<P2>;
    fn intersect(&self, ls: &LineSegment) -> Option<OneOrTwo<P2>> {
        let mut res: Option<OneOrTwo<P2>> = None;
        for rect_ls in self.line_segments().iter() {
            if let Some(intersection) = ls.intersect(rect_ls) {
                if let Some(oot) = res.as_mut() {
                    oot.add(intersection);
                } else {
                    res = Some(OneOrTwo::new(intersection));
                }
            }
        }
        res
    }
}

impl Intersect<Circle> for Rect {
    type Intersection = Vec<P2>;

    fn intersect(&self, circle: &Circle) -> Option<Vec<P2>> {
        let mut intersection_points = Vec::new();
        for ls in self.line_segments().iter() {
            if let Some(one_or_two) = circle.intersect(ls) {
                intersection_points.push(one_or_two.get_first());
                if let Some(p) = one_or_two.get_second() {
                    intersection_points.push(p);
                }
            }
        }
        if !intersection_points.is_empty() {
            Some(intersection_points)
        } else {
            None
        }
    }
}

impl Intersect<Rect> for Rect {
    type Intersection = Vec<P2>;

    fn intersect(&self, other: &Rect) -> Option<Vec<P2>> {
        let mut intersection_points = Vec::new();
        for lsa in self.line_segments().iter() {
            for lsb in other.line_segments().iter() {
                if let Some(p) = lsa.intersect(lsb) {
                    intersection_points.push(p);
                }
            }
        }
        if !intersection_points.is_empty() {
            Some(intersection_points)
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

impl Rotate for Rect {
    fn get_rotation(&self) -> Rot2 {
        self.rotation
    }
    fn set_rotation(&mut self, rotation: &Rot2) {
        self.rotation = *rotation
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
    fn contains(&self, p: &P2) -> bool {
        AABB {
            origin: P2::new(0.0, 0.0),
            width: self.width,
            height: self.height,
        }
        .contains(&(self.rotation.inverse() * P2::from(p - self.origin)))
    }
}

impl ClosestPoint for Rect {
    fn closest_point_to(&self, p: &P2) -> P2 {
        let segments = self.line_segments();
        let mut ret = segments[0].closest_point_to(p);
        for candidate in segments[1..].iter().map(|ls| ls.closest_point_to(p)) {
            if distance(p, &candidate) < distance(p, &ret) {
                ret = candidate;
            }
        }
        ret
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
        if self.contains(p) {
            -min_dist
        } else {
            min_dist
        }
    }
}

impl Distribution<Rect> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Rect {
        let x_axis: V2 = rng.gen();
        let rotation = Rot2::rotation_between(&V2::new(1., 0.), &x_axis);
        Rect {
            origin: rng.gen(),
            rotation,
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
