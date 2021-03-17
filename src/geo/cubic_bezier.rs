use super::*;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct CubicBezier {
    pub points: [P2; 4],
}

impl CubicBezier {
    pub fn new_sample() -> Self {
        CubicBezier {
            points: [
                P2::new(0., -1.),
                P2::new(0., 0.),
                P2::new(1., 0.),
                P2::new(1., 1.),
            ],
        }
    }

    pub fn new_sample2() -> Self {
        CubicBezier {
            points: [
                P2::new(0., 0.5),
                P2::new(0.7, 0.),
                P2::new(0.3, 1.),
                P2::new(1., 0.3),
            ],
        }
    }

    pub fn normal_at_t(&self, t: Float) -> Normal {
        let t2 = t.powi(2);
        let tt2 = t - t2;
        let ot2 = (1. - t).powi(2);
        let p012: P2 =
            t2 * self.points[2] + 2. * tt2 * self.points[1].coords + ot2 * self.points[0].coords;
        let p123: P2 =
            t2 * self.points[3] + 2. * tt2 * self.points[2].coords + ot2 * self.points[1].coords;
        let tangent = p123 - p012;
        U2::new_normalize(V2::new(-tangent.y, tangent.x))
    }

    pub fn eval_at_t(&self, t: Float) -> P2 {
        (1. - t).powi(3) * self.points[0]
            + (1. - t).powi(2) * t * self.points[1].coords
            + (1. - t) * t.powi(2) * self.points[2].coords
            + t.powi(3) * self.points[3].coords
    }
}

impl HasOrigin for CubicBezier {
    fn get_origin(&self) -> P2 {
        (self.points[1] + self.points[2].coords) * 0.5
    }
    fn set_origin(&mut self, origin: P2) {
        let diff: V2 = (origin - self.get_origin().coords).coords;
        for p in self.points.iter_mut() {
            *p += diff;
        }
    }
}

impl HasDirection for CubicBezier {
    fn get_direction(&self) -> U2 {
        Unit::new_normalize(self.points[2] - self.points[1])
    }
    fn set_direction(&mut self, direction: U2) {
        let o = self.get_origin();
        let rotation = Rot2::rotation_between(&self.get_direction(), &direction);
        for i in 0..4 {
            self.points[i] = o + rotation * (self.points[i] - o);
        }
    }
}

impl Scale for CubicBezier {
    fn scale(&mut self, scale_x: Float, scale_y: Float) {
        let o = self.get_origin();
        for p in self.points.iter_mut() {
            p.scale(scale_x, scale_y);
        }
        self.set_origin(o);
    }

    fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
        let mut o = self.get_origin();
        o.scale(scale_x, scale_y);
        self.set_origin(o);
    }
}

impl Mirror for CubicBezier {
    fn mirror_x(&self) -> Self {
        let mut points = self.points;
        // As in LineSegment we just exchange the x coordinate for points[1] and points[2]
        points[1].x = self.points[2].x;
        points[2].x = self.points[1].x;
        let origin = self.get_origin();
        points[0].x = 2. * origin.x - points[0].x;
        points[1].x = 2. * origin.x - points[1].x;
        CubicBezier { points }
    }

    fn mirror_y(&self) -> Self {
        let mut points = self.points;
        // As in LineSegment we just exchange the y coordinate for points[1] and points[2]
        points[1].y = self.points[2].y;
        points[2].y = self.points[1].y;
        let origin = self.get_origin();
        points[0].y = 2. * origin.y - points[0].y;
        points[1].y = 2. * origin.y - points[1].y;
        CubicBezier { points }
    }
}

impl Intersect<Ray> for CubicBezier {
    type Intersection = (P2, Normal);
    fn intersect(&self, ray: &Ray) -> Option<Self::Intersection> {
        let rot = Rot2::rotation_between(&V2::new(1., 0.), &ray.get_direction());
        // inv translate => inv rotate
        let p: Vec<P2> = self
            .points
            .iter()
            .map(|p| P2::from(rot.inverse() * (p - ray.get_origin().coords).coords))
            .collect();
        // intersect: get closest positive t
        let a = 3. * (p[1] - p[2]) - p[0].coords + p[3].coords;
        let b = 3. * (p[0] - 2. * p[1] + p[2].coords);
        let c = 3. * (p[1] - p[0]);
        let d = p[0];
        let mut rs: Vec<(Float, Float)> = cubic_roots(0, b.y / a.y, c.y / a.y, d.y / a.y)
            .iter()
            .map(|t| (t.powi(3) * a.x + t.powi(2) * b.x + t * c.x + d.x, *t))
            .filter(|(r, t)| *t >= 0. && *t <= 1. && *r >= 0.)
            .collect();
        get_smallest_positive_by(&mut rs, |(r, _t)| *r)
            .map(|(r, t)| (ray.eval_at_r(r), self.normal_at_t(t)))
    }
}

impl Intersect<LineSegment> for CubicBezier {
    type Intersection = Vec<(P2, Normal)>;
    fn intersect(&self, ls: &LineSegment) -> Option<Self::Intersection> {
        let rot = Rot2::rotation_between(&V2::new(1., 0.), &ls.get_direction());
        // inv translate => inv rotate
        let p: Vec<P2> = self
            .points
            .iter()
            .map(|p| P2::from(rot.inverse() * (p - ls.get_origin().coords).coords))
            .collect();
        // intersect: get closest positive t
        let a = 3. * (p[1] - p[2]) - p[0].coords + p[3].coords;
        let b = 3. * (p[0] - 2. * p[1] + p[2].coords);
        let c = 3. * (p[1] - p[0]);
        let d = p[0];
        let intersections: Vec<_> = cubic_roots(0, b.y / a.y, c.y / a.y, d.y / a.y)
            .iter()
            .map(|t| (t.powi(3) * a.x + t.powi(2) * b.x + t * c.x + d.x, *t))
            .filter(|(r, t)| *t >= 0. && *t <= 1. && *r >= 0. && *r <= 1.)
            .map(|(r, t)| (ls.eval_at_r(r), self.normal_at_t(t)))
            .collect();
        if intersections.is_empty() {
            return None;
        }
        Some(intersections)
    }
}

impl Intersect<Line> for CubicBezier {
    type Intersection = (P2, Normal);
    fn intersect(&self, line: &Line) -> Option<Self::Intersection> {
        let rot = Rot2::rotation_between(&V2::new(1., 0.), &line.get_direction());
        // inv translate => inv rotate
        let p: Vec<P2> = self
            .points
            .iter()
            .map(|p| P2::from(rot.inverse() * (p - line.get_origin().coords).coords))
            .collect();
        // intersect: get closest positive t
        let a = 3. * (p[1] - p[2]) - p[0].coords + p[3].coords;
        let b = 3. * (p[0] - 2. * p[1] + p[2].coords);
        let c = 3. * (p[1] - p[0]);
        let d = p[0];
        let mut rs: Vec<(Float, Float)> = cubic_roots(0, b.y / a.y, c.y / a.y, d.y / a.y)
            .iter()
            .map(|t| (t.powi(3) * a.x + t.powi(2) * b.x + t * c.x + d.x, *t))
            .filter(|(_r, t)| *t >= 0. && *t <= 1.)
            .collect();
        get_smallest_positive_by(&mut rs, |(r, _t)| *r)
            .map(|(r, t)| (line.eval_at_r(r), self.normal_at_t(t)))
    }
}
