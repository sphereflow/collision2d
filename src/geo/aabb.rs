extern crate nalgebra as na;

use super::*;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Aabb {
    // origin is in the middle
    pub origin: P2,
    pub width: Float,
    pub height: Float,
}

impl Aabb {
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

    pub fn get_left(&self) -> Float {
        self.origin.x - 0.5 * self.width
    }

    pub fn get_right(&self) -> Float {
        self.origin.x + 0.5 * self.width
    }

    pub fn get_bottom(&self) -> Float {
        self.origin.y - 0.5 * self.height
    }

    pub fn get_top(&self) -> Float {
        self.origin.y + 0.5 * self.height
    }

    pub fn add_point(&mut self, p: &P2) {
        let left = self.get_left();
        let right = self.get_right();
        let bottom = self.get_bottom();
        let top = self.get_top();
        match (p.x < left, p.x > right) {
            (true, _) => {
                self.origin.x = (p.x + right) * 0.5;
                self.width = right - p.x;
            }
            (_, true) => {
                self.origin.x = (left + p.x) * 0.5;
                self.width = p.x - left;
            }
            _ => {}
        }
        match (p.y < bottom, p.y > top) {
            (true, _) => {
                self.origin.y = (p.y + top) * 0.5;
                self.height = top - p.y;
            }
            (_, true) => {
                self.origin.y = (bottom + p.y) * 0.5;
                self.height = p.y - bottom;
            }
            _ => {}
        }
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
            rotation: Rot2::identity(),
            width: self.width,
            height: self.height,
        }
    }

    pub fn separated_by(&self, l: &Line) -> bool {
        self.to_rect().seperated_by_line(l)
    }

    pub fn from_tlbr(top: Float, left: Float, bottom: Float, right: Float) -> Aabb {
        let origin = P2::new((left + right) * 0.5, (bottom + top) * 0.5);
        Aabb {
            origin,
            width: (right - left).abs(),
            height: (top - bottom).abs(),
        }
    }
}

impl Intersect<LineSegment> for Aabb {
    type Intersection = OneOrTwo<P2>;

    fn intersect(&self, ls: &LineSegment) -> Option<OneOrTwo<P2>> {
        self.to_rect().intersect(ls)
    }
}

impl Intersect<Circle> for Aabb {
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

impl Intersect<Rect> for Aabb {
    type Intersection = Vec<P2>;

    fn intersect(&self, r: &Rect) -> Option<Vec<P2>> {
        self.to_rect().intersect(r)
    }
}

impl Intersect<Aabb> for Aabb {
    type Intersection = ();

    fn intersect(&self, other: &Aabb) -> Option<()> {
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

impl HasOrigin for Aabb {
    fn get_origin(&self) -> P2 {
        self.origin
    }
    fn set_origin(&mut self, origin: P2) {
        self.origin = origin;
    }
}

impl Contains for Aabb {
    fn contains(&self, p: &P2) -> bool {
        let trans = p - self.origin;
        Float::abs(trans.x) < (self.width * 0.5) && Float::abs(trans.y) < (self.height * 0.5)
    }
}

impl ClosestPoint for Aabb {
    fn closest_point_to(&self, p: &P2) -> P2 {
        self.to_rect().closest_point_to(p)
    }
}

impl Distance for Aabb {
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

impl Mirror for Aabb {
    fn mirror_x(&self) -> Self {
        *self
    }
    fn mirror_y(&self) -> Self {
        *self
    }
}

impl Distribution<Aabb> for Standard {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> Aabb {
        Aabb {
            origin: rng.gen(),
            width: rng.gen(),
            height: rng.gen(),
        }
    }
}

impl ReflectOn<Line> for Aabb {}
impl ReflectOn<Ray> for Aabb {}
impl ReflectOn<LineSegment> for Aabb {}
impl ReflectOn<Circle> for Aabb {}
impl ReflectOn<Rect> for Aabb {}
impl ReflectOn<Aabb> for Aabb {}
impl ReflectOn<MCircle> for Aabb {}

impl CanCollideWith<Line> for Aabb {}
impl CanCollideWith<Ray> for Aabb {}
impl CanCollideWith<LineSegment> for Aabb {}
impl CanCollideWith<Circle> for Aabb {}
impl CanCollideWith<Rect> for Aabb {}
impl CanCollideWith<Aabb> for Aabb {}
impl CanCollideWith<MCircle> for Aabb {}

impl GeoT for Aabb {}
