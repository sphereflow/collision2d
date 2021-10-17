extern crate nalgebra as na;

use super::*;

#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct Aabb {
    // origin is in the middle
    pub origin: P2,
    pub width: Float,
    pub height: Float,
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum Configuration {
    Lesser,
    Greater,
    Outer,
    Inner,
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

    pub fn from_points(points: &[P2]) -> Self {
        let mut t = points[0].y;
        let mut b = points[0].y;
        let mut r = points[0].x;
        let mut l = points[0].x;
        for point in points {
            t = t.max(point.y);
            b = b.min(point.y);
            r = r.max(point.x);
            l = l.min(point.x);
        }
        Aabb::from_tlbr(t, l, b, r)
    }

    pub fn get_tlbr(&self) -> (Float, Float, Float, Float) {
        let wh = 0.5 * self.width;
        let hh = 0.5 * self.height;
        (
            self.origin.y + hh,
            self.origin.x - wh,
            self.origin.y - hh,
            self.origin.x + wh,
        )
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

    pub fn merge(&self, other: &Aabb) -> Aabb {
        let t = self.get_top().max(other.get_top());
        let b = self.get_bottom().min(other.get_bottom());
        let r = self.get_right().max(other.get_right());
        let l = self.get_left().min(other.get_left());
        Self::from_tlbr(t, l, b, r)
    }

    pub fn intersection(&self, other: &Aabb) -> Aabb {
        let (t1, l1, b1, r1) = self.get_tlbr();
        let (t2, l2, b2, r2) = other.get_tlbr();
        if b1 > t2 || b2 > t1 || l1 > r2 || l2 > r1 {
            Aabb {
                origin: 0.5 * (self.origin + other.get_origin().coords),
                width: 0.,
                height: 0.,
            }
        } else {
            let t = self.get_top().min(other.get_top());
            let b = self.get_bottom().max(other.get_bottom());
            let r = self.get_right().min(other.get_right());
            let l = self.get_left().max(other.get_left());
            Self::from_tlbr(t, l, b, r)
        }
    }

    pub fn from_tlbr(top: Float, left: Float, bottom: Float, right: Float) -> Aabb {
        let origin = P2::new((left + right) * 0.5, (bottom + top) * 0.5);
        Aabb {
            origin,
            width: (right - left).abs(),
            height: (top - bottom).abs(),
        }
    }

    fn get_configuration(a1: Float, b1: Float, a2: Float, b2: Float) -> Configuration {
        match (a1 < a2, b1 < b2) {
            (false, false) => Configuration::Lesser,
            (false, true) => Configuration::Inner,
            (true, false) => Configuration::Outer,
            _ => Configuration::Greater,
        }
    }

    fn get_configuration_crossover(a1: Float, b1: Float, a2: Float, b2: Float) -> Configuration {
        let range1 = a1..=b1;
        match (
            b1 < a2,
            b2 < a1,
            range1.contains(&a2) && range1.contains(&b2),
        ) {
            (true, _, _) => Configuration::Lesser,
            (_, true, _) => Configuration::Greater,
            (_, _, true) => Configuration::Outer,
            _ => Configuration::Inner,
        }
    }

    /// returns the "left" and "right" outlines of 2 Aabb's
    ///           _+----------+                
    ///         _/ |          |                
    /// left  _/   |          |                
    ///     _/     |          |                
    ///   _/       |          |                
    ///  /         +----------+                
    /// +----------+         _/
    /// |          |       _/  
    /// |          |     _/ right
    /// |          |   _/      
    /// |          | _/        
    /// +----------+/
    /// returns None if the Aabb's have more than 2 intersection points
    /// -> (left linesegment, right linesegment)
    pub fn get_outlines(&self, other: &Aabb) -> Option<(LineSegment, LineSegment)> {
        // get the 2 points farthest appart
        let (t1, l1, b1, r1) = self.get_tlbr();
        let (t2, l2, b2, r2) = other.get_tlbr();
        let cfg1 = Aabb::get_configuration(l1, r1, l2, r2);
        let cfg2 = Aabb::get_configuration(b1, t1, b2, t2);
        // LineSegments in clockwise order starting at top left
        let ls1 = LineSegment::from_ab(P2::new(l1, t1), P2::new(l2, t2));
        let ls2 = LineSegment::from_ab(P2::new(r1, t1), P2::new(r2, t2));
        let ls3 = LineSegment::from_ab(P2::new(r1, b1), P2::new(r2, b2));
        let ls4 = LineSegment::from_ab(P2::new(l1, b1), P2::new(l2, b2));
        match (cfg1, cfg2) {
            (Configuration::Lesser, Configuration::Lesser) => Some((ls1, ls3)),
            (Configuration::Lesser, Configuration::Inner) => Some((ls1, ls4)),
            (Configuration::Lesser, Configuration::Outer) => Some((ls2, ls3)),
            (Configuration::Lesser, Configuration::Greater) => Some((ls2, ls4)),
            (Configuration::Inner, Configuration::Lesser) => Some((ls4, ls3)),
            (Configuration::Inner, Configuration::Greater) => Some((ls2, ls1)),
            (Configuration::Outer, Configuration::Greater) => Some((ls3, ls4)),
            (Configuration::Outer, Configuration::Lesser) => Some((ls1, ls2)),
            (Configuration::Greater, Configuration::Lesser) => Some((ls4, ls2)),
            (Configuration::Greater, Configuration::Inner) => Some((ls3, ls2)),
            (Configuration::Greater, Configuration::Outer) => Some((ls4, ls1)),
            (Configuration::Greater, Configuration::Greater) => Some((ls3, ls1)),
            _ => None,
        }
    }

    pub fn get_outlines2(&self, other: &Aabb) -> Option<(LineSegment, LineSegment)> {
        // get the 2 points farthest appart
        let (t1, l1, b1, r1) = self.get_tlbr();
        let (t2, l2, b2, r2) = other.get_tlbr();
        // LineSegments in clockwise order starting at top left
        let ls1 = LineSegment::from_ab(P2::new(l1, t1), P2::new(l2, t2));
        let ls2 = LineSegment::from_ab(P2::new(r1, t1), P2::new(r2, t2));
        let ls3 = LineSegment::from_ab(P2::new(r1, b1), P2::new(r2, b2));
        let ls4 = LineSegment::from_ab(P2::new(l1, b1), P2::new(l2, b2));
        let d1 = ls1.get_direction();
        let d2 = ls2.get_direction();
        let d3 = ls3.get_direction();
        let d4 = ls4.get_direction();
        let mut lefto = None;
        let mut righto = None;
        if d1.x > 0.0 && d1.y >= 0.0 {
            lefto = Some(ls1);
        } else if d1.x <= 0.0 && d1.y < 0.0 {
            righto = Some(ls1);
        }
        if d2.x >= 0.0 && d2.y < 0.0 {
            lefto = Some(ls2);
        } else if d2.x < 0.0 && d2.y >= 0.0 {
            righto = Some(ls2);
        }
        if d3.x < 0.0 && d3.y <= 0.0 {
            if lefto.is_some() {
                return None;
            }
            lefto = Some(ls3);
        } else if d3.x >= 0.0 && d3.y > 0.0 {
            if righto.is_some() {
                return None;
            }
            righto = Some(ls3);
        }
        if d4.x <= 0.0 && d4.y > 0.0 {
            if lefto.is_some() {
                return None;
            }
            lefto = Some(ls4);
        } else if d4.x > 0.0 && d4.y <= 0.0 {
            if righto.is_some() {
                return None;
            }
            righto = Some(ls4);
        }
        if let (Some(left), Some(right)) = (lefto, righto) {
            Some((left, right))
        } else {
            None
        }
    }

    pub fn get_crossover(&self, other: &Aabb) -> Option<(LineSegment, LineSegment)> {
        // get the 2 points farthest appart
        let (t1, l1, b1, r1) = self.get_tlbr();
        let (t2, l2, b2, r2) = other.get_tlbr();
        let cfg1 = Aabb::get_configuration_crossover(l1, r1, l2, r2);
        let cfg2 = Aabb::get_configuration_crossover(b1, t1, b2, t2);
        // LineSegments in clockwise order starting at top left
        let ls1 = LineSegment::from_ab(P2::new(l1, t1), P2::new(r2, b2));
        let ls2 = LineSegment::from_ab(P2::new(r1, t1), P2::new(l2, b2));
        let ls3 = LineSegment::from_ab(P2::new(r1, b1), P2::new(l2, t2));
        let ls4 = LineSegment::from_ab(P2::new(l1, b1), P2::new(r2, t2));
        match (cfg1, cfg2) {
            (Configuration::Lesser, Configuration::Lesser) => Some((ls3, ls1)),
            (Configuration::Lesser, Configuration::Inner | Configuration::Outer) => Some((ls3, ls2)),
            (Configuration::Lesser, Configuration::Greater) => Some((ls4, ls2)),
            (Configuration::Inner | Configuration::Outer, Configuration::Greater) => Some((ls4, ls3)),
            (Configuration::Inner | Configuration::Outer, Configuration::Lesser) => Some((ls2, ls1)),
            (Configuration::Greater, Configuration::Lesser) => Some((ls2, ls4)),
            (Configuration::Greater, Configuration::Inner | Configuration::Outer) => Some((ls1, ls4)),
            (Configuration::Greater, Configuration::Greater) => Some((ls1, ls3)),
            _ => None,
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
    /// there are up to 4 intersection points
    type Intersection = Vec<P2>;

    fn intersect(&self, other: &Aabb) -> Option<Self::Intersection> {
        let (t1, l1, b1, r1) = self.get_tlbr();
        let (t2, l2, b2, r2) = other.get_tlbr();
        if b1 > t2 || b2 > t1 || l1 > r2 || l2 > r1 {
            None
        } else {
            let cfg1 = Aabb::get_configuration(l1, r1, l2, r2);
            let cfg2 = Aabb::get_configuration(b1, t1, b2, t2);
            match (cfg1, cfg2) {
                (Configuration::Lesser, Configuration::Lesser) => {
                    Some(vec![P2::new(l2, t1), P2::new(r1, b2)])
                }
                (Configuration::Lesser, Configuration::Inner) => {
                    Some(vec![P2::new(l2, t1), P2::new(l2, b1)])
                }
                (Configuration::Lesser, Configuration::Outer) => {
                    Some(vec![P2::new(r1, t2), P2::new(r1, b2)])
                }
                (Configuration::Lesser, Configuration::Greater) => {
                    Some(vec![P2::new(r1, t2), P2::new(l2, b1)])
                }
                (Configuration::Inner, Configuration::Lesser) => {
                    Some(vec![P2::new(l1, b2), P2::new(r1, b2)])
                }
                (Configuration::Inner, Configuration::Inner)
                | (Configuration::Outer, Configuration::Outer) => None,
                (Configuration::Inner, Configuration::Outer) => Some(vec![
                    P2::new(l1, t2),
                    P2::new(r1, t2),
                    P2::new(l1, b2),
                    P2::new(r1, b2),
                ]),
                (Configuration::Inner, Configuration::Greater) => {
                    Some(vec![P2::new(l1, t2), P2::new(r1, t2)])
                }
                (Configuration::Outer, Configuration::Lesser) => {
                    Some(vec![P2::new(l2, t1), P2::new(r2, t1)])
                }
                (Configuration::Outer, Configuration::Inner) => Some(vec![
                    P2::new(l2, t1),
                    P2::new(r2, t1),
                    P2::new(l2, b1),
                    P2::new(r2, b1),
                ]),
                (Configuration::Outer, Configuration::Greater) => {
                    Some(vec![P2::new(l2, b1), P2::new(r2, b1)])
                }
                (Configuration::Greater, Configuration::Lesser) => {
                    Some(vec![P2::new(l1, b2), P2::new(r2, t1)])
                }
                (Configuration::Greater, Configuration::Inner) => {
                    Some(vec![P2::new(r2, t1), P2::new(r2, b1)])
                }
                (Configuration::Greater, Configuration::Outer) => {
                    Some(vec![P2::new(l1, t2), P2::new(l1, b2)])
                }
                (Configuration::Greater, Configuration::Greater) => {
                    Some(vec![P2::new(l1, t2), P2::new(r2, b1)])
                }
            }
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

impl HasDirection for Aabb {
    fn get_direction(&self) -> U2 {
        Unit::new_unchecked(V2::new(1.0, 0.0))
    }
    fn set_direction(&mut self, _direction: U2) {}
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

impl HasAabb for Aabb {
    fn get_aabb(&self) -> Aabb {
        *self
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
