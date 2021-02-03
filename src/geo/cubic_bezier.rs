use super::*;

pub struct CubicBezier {
    points: [P2; 4],
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
}

impl HasOrigin for CubicBezier {
    fn get_origin(&self) -> P2 {
        (self.points[1] + self.points[2].coords) * 0.5
    }
    fn set_origin(&mut self, origin: P2) {
        let diff: V2 = (self.get_origin() - origin.coords).coords;
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
        for i in 1..4 {
            self.points[i] = o + rotation * (self.points[i] - o);
        }
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
        let b = p[0] - 2. * p[1] + p[2].coords;
        let c = p[1] - p[0];
        let d = p[0];
        let mut rs: Vec<(Float, Float)> = cubic_roots(0, b.y / a.y, c.y / a.y, d.y / a.y)
            .iter()
            .filter(|t| **t >= 0. && **t <= 1.)
            .map(|t| (t.powi(3) * a.x + t.powi(2) * b.x + t * c.x + d.x, *t))
            .collect();
        get_smallest_positive_by(&mut rs, |(r, _t)| *r).map(|(r, t)| {
            let p = ray.eval_at_r(r);
            let slope = 3. * a.x * t.powi(2) + 2. * b.x * t + c.x;
            let normal = Unit::new_normalize(V2::new(-slope, 1.));
            (p, rot * normal)
        })
    }
}
