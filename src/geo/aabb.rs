extern crate nalgebra as na;

use super::*;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct AABB {
    // origin is in the middle
    pub origin: P2,
    pub width: Float,
    pub height: Float,
}

impl AABB {
    pub fn points(&self) -> RectPoints {
        let top_right = V2::new(self.width * 0.5, self.height * 0.5);
        let bottom_right = V2::new(self.width * 0.5, -self.height * 0.5);
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

    pub fn to_rect(&self) -> Rect {
        Rect {
            origin: self.origin,
            x_axis: Unit::new_unchecked(V2::new(1.0, 0.0)),
            width: self.width,
            height: self.height,
        }
    }

    pub fn separated_by(&self, l: &Line) -> bool {
        self.to_rect().seperated_by_line(l)
    }
}

impl Intersect<LineSegment> for AABB {
    type Intersection = OneOrTwo<P2>;

    fn intersect(&self, ls: &LineSegment) -> Option<OneOrTwo<P2>> {
        self.to_rect().intersect(ls)
    }
}

impl Intersect<Circle> for AABB {
    type Intersection = OneOrTwo<P2>;

    fn intersect(&self, s: &Circle) -> Option<OneOrTwo<P2>> {
        let mut intersection_points: Option<OneOrTwo<P2>> = None;
        for ls in self.line_segments().iter() {
            if let Some(oot) = s.intersect(ls) {
                // intersection_points.map_or(Some(oot), |mut one_or_two| {one_or_two.mappend(oot);
                // Some(one_or_two) });
                match intersection_points.as_mut() {
                    Some(one_or_two) => {
                        one_or_two.mappend(oot);
                    }
                    None => intersection_points = Some(oot),
                }
            }
        }
        intersection_points
    }
}

impl Intersect<Rect> for AABB {
    type Intersection = Vec<P2>;

    fn intersect(&self, r: &Rect) -> Option<Vec<P2>> {
        self.to_rect().intersect(r)
    }
}

impl Intersect<AABB> for AABB {
    type Intersection = ();

    fn intersect(&self, other: &AABB) -> Option<()> {
        let p = self.origin - other.origin;
        let w2 = self.width * 0.5 + other.width * 0.5;
        let h2 = self.height * 0.5 + other.height * 0.5;
        if Float::abs(p.x) < w2 && Float::abs(p.y) < h2 {
            Some(())
        } else {
            None
        }
    }
}

impl HasOrigin for AABB {
    fn get_origin(&self) -> P2 {
        self.origin
    }
    fn set_origin(&mut self, origin: P2) {
        self.origin = origin;
    }
}

impl Contains for AABB {
    fn contains(&self, p: &P2) -> bool {
        let trans = p - self.origin;
        Float::abs(trans.x) < (self.width * 0.5) && Float::abs(trans.y) < (self.height * 0.5)
    }
}

impl ClosestPoint for AABB {
    fn closest_point_to(&self, p: &P2) -> P2 {
        self.to_rect().closest_point_to(p)
    }
}

impl Distance for AABB {
    fn distance(&self, p: &P2) -> Float {
        let abs_x = (p.x - self.origin.x).abs();
        let abs_y = (p.y - self.origin.y).abs();
        let wh = self.width * 0.5;
        let hh = self.height * 0.5;
        if abs_x < wh {
            if abs_y < hh {
                (abs_x - wh).min(abs_y - hh)
            } else {
                abs_x - wh
            }
        } else if abs_y < self.height * 0.5 {
            abs_y - hh
        } else {
            Float::sqrt((abs_x - wh).powi(2) + (abs_y - hh).powi(2))
        }
    }
}

impl Distribution<AABB> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> AABB {
        AABB {
            origin: rng.gen(),
            width: rng.gen(),
            height: rng.gen(),
        }
    }
}

impl ReflectOn<Line> for AABB {}
impl ReflectOn<Ray> for AABB {}
impl ReflectOn<LineSegment> for AABB {}
impl ReflectOn<Circle> for AABB {}
impl ReflectOn<Rect> for AABB {}
impl ReflectOn<AABB> for AABB {}
impl ReflectOn<MCircle> for AABB {}

impl CanCollideWith<Line> for AABB {}
impl CanCollideWith<Ray> for AABB {}
impl CanCollideWith<LineSegment> for AABB {}
impl CanCollideWith<Circle> for AABB {}
impl CanCollideWith<Rect> for AABB {}
impl CanCollideWith<AABB> for AABB {}
impl CanCollideWith<MCircle> for AABB {}

impl GeoT for AABB {}
