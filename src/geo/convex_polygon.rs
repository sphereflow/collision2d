use super::*;

#[derive(Clone, PartialEq, Debug)]
pub struct ConvexPolygon {
    origin: P2,
    /// Rotation matrix
    rotation: Rot2,
    /// Bounding box
    aabb: Aabb,
    /// Points relative to origin sorted in clockwise direction
    /// The first and last point should be the same to make functions witch call
    /// points.iter().windows(2) easier to implement
    points: Vec<V2>,
}

impl ConvexPolygon {
    pub fn new_convex_hull(points: &[P2]) -> Self {
        let mut all_points: Vec<V2> = points.iter().map(|p| p.coords).collect();

        // calculate the bounding box and the origin
        let mut min_x = Float::MAX;
        let mut min_y = Float::MAX;
        let mut max_x = Float::MIN;
        let mut max_y = Float::MIN;
        let mut origin = P2::new(0., 0.);
        for p in all_points.iter() {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
            origin += p;
        }
        let aabb = Aabb::from_tlbr(max_y, min_x, min_y, max_x);
        origin.x /= all_points.len() as Float;
        origin.y /= all_points.len() as Float;

        for point in all_points.iter_mut() {
            *point -= origin.coords;
        }

        // split all_points into left (x-component < 0) and right (x-component >= 0)
        //
        let mut left: Vec<V2> = Vec::new();
        let mut right = all_points;
        for p in &right {
            if p.x < 0. {
                left.push(*p);
            }
        }

        // sorting semi clockwise
        //
        right.retain(|p| p.x >= 0.);
        // sort the left side bottom to top (semi clockwise)
        left.sort_unstable_by(|p1, p2| {
            if (p1.y - p2.y).abs() < Float::EPSILON {
                // high to low
                p2.x.partial_cmp(&p1.x)
                    .expect("convex hull: could not sort left side")
            } else {
                // low to high
                p1.y.partial_cmp(&p2.y)
                    .expect("convex hull: could not sort left side")
            }
        });
        // sort the right side top to bottom
        right.sort_unstable_by(|p1, p2| {
            if (p1.y - p2.y).abs() < Float::EPSILON {
                // low to high
                p1.x.partial_cmp(&p2.x)
                    .expect("convex hull: could not sort left side")
            } else {
                // high to low
                p2.y.partial_cmp(&p1.y)
                    .expect("convex hull: could not sort right side")
            }
        });
        right.extend(left.into_iter());

        // first point == last point
        //
        right.push(right[0]);

        let all_points = right;

        // Graham scan
        //
        let mut hull_points: Vec<V2> = Vec::new();
        for p in all_points {
            while hull_points.len() > 1
                && !is_clockwise_points(
                    &hull_points[hull_points.len() - 2],
                    &hull_points[hull_points.len() - 1],
                    &p,
                )
            {
                hull_points.pop();
            }
            hull_points.push(p);
        }

        ConvexPolygon {
            origin,
            rotation: Rot2::identity(),
            aabb,
            points: hull_points,
        }
    }

    pub fn add_point(&mut self, p: &P2) {
        // adapt aabb
        //
        self.aabb.add_point(p);

        let p = p - self.get_origin();
        // sort in p
        //
        for ix in 0..self.points.len() {
            if let Some(snd) = self.points.get(ix + 1) {
                if between(p.y, self.points[ix].y, snd.y) {
                    self.points.insert(ix, p);
                    break;
                }
            } else {
                self.points.push(p);
                break;
            }
        }
        // redo Graham's scan
        let mut hull_points: Vec<V2> = Vec::new();
        for p in self.points.iter() {
            let mut is_cw = is_clockwise_points(
                &hull_points[hull_points.len() - 2],
                &hull_points[hull_points.len() - 1],
                &p,
            );
            // the following variable will be set if the algorithm encounters a non-clockwise turn
            let mut not_cw_once = !is_cw;
            while hull_points.len() > 1 && !is_cw {
                hull_points.pop();
                is_cw = is_clockwise_points(
                    &hull_points[hull_points.len() - 2],
                    &hull_points[hull_points.len() - 1],
                    &p,
                );
                if !is_cw {
                    not_cw_once = true;
                }
            }
            hull_points.push(*p);
            // if we ever have hit a non-clockwise turn our point has been handled
            if not_cw_once {
                break;
            }
        }
        self.points = hull_points;
    }

    pub fn calc_aabb(points: &[P2]) -> Aabb {
        // calculate the bounding box
        let mut min_x = Float::MAX;
        let mut min_y = Float::MAX;
        let mut max_x = Float::MIN;
        let mut max_y = Float::MIN;
        for p in points {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }
        Aabb::from_tlbr(max_y, min_x, min_y, max_x)
    }

    pub fn get_line_segments(&self) -> Vec<LineSegment> {
        let mut res = Vec::with_capacity(self.points.len());
        for w in self.points.windows(2) {
            res.push(LineSegment::from_ab(
                self.get_origin() + w[0],
                self.get_origin() + w[1],
            ));
        }
        res
    }

    pub fn get_global_points(&self) -> Vec<P2> {
        let rot = self.get_rotation();
        self.points
            .iter()
            .map(|p| self.get_origin() + (rot * p))
            .collect()
    }
}

impl HasOrigin for ConvexPolygon {
    fn get_origin(&self) -> P2 {
        self.origin
    }
    fn set_origin(&mut self, origin: P2) {
        self.origin = origin;
    }
}

impl Contains for ConvexPolygon {
    fn contains(&self, p: &P2) -> bool {
        for w in self.points.windows(2) {
            if !is_clockwise_points(&w[0], &w[1], &p.coords) {
                return false;
            }
        }
        true
    }
}

impl Distance for ConvexPolygon {
    fn distance(&self, p: &P2) -> Float {
        let mut min_dist = None;
        for ls in self.get_line_segments() {
            let dist = ls.distance(p);
            if let Some((_, md)) = min_dist {
                if dist < md {
                    min_dist = Some((ls, dist));
                }
            } else {
                min_dist = Some((ls, dist));
            }
        }

        // if self contains p negate the result
        if let Some((ls, md)) = min_dist {
            if is_clockwise_points(&ls.get_a().coords, &ls.get_b().coords, &p.coords) {
                md * -1.
            } else {
                md
            }
        } else {
            0.
        }
    }
}

impl ClosestPoint for ConvexPolygon {
    fn closest_point_to(&self, p: &P2) -> P2 {
        let mut res = (*p, Float::MAX);
        for ls in self.get_line_segments() {
            let cp = ls.closest_point_to(p);
            let dist_sq = distance_squared(p, &cp);
            if dist_sq < res.1 {
                res = (cp, dist_sq)
            }
        }
        res.0
    }
}

impl Rotate for ConvexPolygon {
    fn get_rotation(&self) -> Rot2 {
        self.rotation
    }
    fn set_rotation(&mut self, rotation: &Rot2) {
        self.rotation = *rotation;
    }
}

impl Scale for ConvexPolygon {
    fn scale(&mut self, scale_x: Float, scale_y: Float) {
        for p in self.points.iter_mut() {
            p.x *= scale_x;
            p.y *= scale_y;
        }
    }
    fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
        self.aabb.origin.x *= scale_x;
        self.aabb.origin.y *= scale_y;
    }
}

impl Mirror for ConvexPolygon {
    fn mirror_x(&self) -> Self {
        let mut points = Vec::new();
        // first rotate then mirror
        let rot = self.get_rotation();
        for p in self.points.iter() {
            let rotated = rot * p;
            points.push(V2::new(-rotated.x, rotated.y));
        }
        let global_points: Vec<P2> = points.iter().map(|v| self.origin + *v).collect();
        ConvexPolygon {
            origin: self.origin,
            rotation: Rot2::identity(),
            points,
            aabb: ConvexPolygon::calc_aabb(&global_points),
        }
    }

    fn mirror_y(&self) -> Self {
        let mut points = Vec::new();
        // first rotate then mirror
        let rot = self.get_rotation();
        for p in self.points.iter() {
            let rotated = rot * p;
            points.push(V2::new(-rotated.x, rotated.y));
        }
        let global_points: Vec<P2> = points.iter().map(|v| self.origin + *v).collect();
        ConvexPolygon {
            origin: self.origin,
            rotation: Rot2::identity(),
            points,
            aabb: ConvexPolygon::calc_aabb(&global_points),
        }
    }
}

impl<T: HasOrigin + HasDirection + Intersect<LineSegment, Intersection = P2>> Intersect<T>
    for ConvexPolygon
{
    type Intersection = OneOrTwo<Reflection>;
    fn intersect(&self, l: &T) -> Option<Self::Intersection> {
        let global_points = self.get_global_points();
        let mut res: Option<OneOrTwo<Reflection>> = None;
        for w in global_points
            .iter()
            .map(|p| separation_axis_projection(&l.get_origin(), &l.get_direction(), p))
            .enumerate()
            .collect::<Vec<_>>()
            .windows(2)
        {
            let (ixa, pra) = w[0];
            let (ixb, prb) = w[1];
            if pra * prb < 0. {
                let ls = LineSegment::from_ab(global_points[ixa], global_points[ixb]);
                if let Some(p) = l.intersect(&ls) {
                    let n = ls.get_normal();
                    if let Some(oot) = res.as_mut() {
                        oot.add((p, n));
                        // we got our second point -> break
                        break;
                    } else {
                        res = Some(OneOrTwo::new((p, n)));
                    }
                }
            }
        }
        res
    }
}

impl Intersect<Ray> for ConvexPolygon {
    type Intersection = OneOrTwo<Reflection>;
    fn intersect(&self, ray: &Ray) -> Option<Self::Intersection> {
        let global_points = self.get_global_points();
        let mut res: Option<OneOrTwo<Reflection>> = None;
        for w in global_points
            .iter()
            .map(|p| separation_axis_projection(&ray.get_origin(), &ray.get_direction(), p))
            .enumerate()
            .collect::<Vec<_>>()
            .windows(2)
        {
            let (ixa, pra) = w[0];
            let (ixb, prb) = w[1];
            if pra * prb < 0. {
                let ls = LineSegment::from_ab(global_points[ixa], global_points[ixb]);
                if let Some(pn) = ray.intersect(&ls) {
                    if let Some(oot) = res.as_mut() {
                        oot.add(pn);
                        // we got our second point -> break
                        break;
                    } else {
                        res = Some(OneOrTwo::new(pn));
                    }
                }
            }
        }
        res
    }
}

impl Intersect<Circle> for ConvexPolygon {
    type Intersection = Vec<P2>;
    fn intersect(&self, circle: &Circle) -> Option<Self::Intersection> {
        let mut res: Option<Self::Intersection> = None;
        for ls in self.get_line_segments() {
            if let Some(i) = circle.intersect(&ls) {
                if let Some(v) = res.as_mut() {
                    v.extend(i.into_iter());
                } else {
                    res = Some(i.into_iter().collect());
                }
            }
        }
        res
    }
}

impl Intersect<Rect> for ConvexPolygon {
    type Intersection = Vec<P2>;
    fn intersect(&self, rect: &Rect) -> Option<Self::Intersection> {
        let mut res: Option<Self::Intersection> = None;
        for ls in self.get_line_segments() {
            if let Some(i) = rect.intersect(&ls) {
                if let Some(v) = res.as_mut() {
                    v.extend(i.into_iter());
                } else {
                    res = Some(i.into_iter().collect());
                }
            }
        }
        res
    }
}

impl Intersect<MCircle> for ConvexPolygon {
    type Intersection = Vec<P2>;
    fn intersect(&self, mcircle: &MCircle) -> Option<Self::Intersection> {
        let circle = mcircle.circle_b();
        self.intersect(&circle)
    }
}

impl Intersect<ConvexPolygon> for ConvexPolygon {
    type Intersection = Vec<P2>;
    fn intersect(&self, other: &ConvexPolygon) -> Option<Self::Intersection> {
        let mut res: Option<Self::Intersection> = None;
        let other_segments: Vec<LineSegment> = other.get_line_segments();
        for lsa in self.get_line_segments() {
            for lsb in &other_segments {
                if let Some(i) = lsa.intersect(lsb) {
                    if let Some(v) = res.as_mut() {
                        v.push(i);
                    } else {
                        res = Some(vec![i]);
                    }
                }
            }
        }
        res
    }
}

impl Intersect<CubicBezier> for ConvexPolygon {
    type Intersection = Vec<P2>;
    fn intersect(&self, cb: &CubicBezier) -> Option<Self::Intersection> {
        let mut res: Option<Self::Intersection> = None;
        for ls in self.get_line_segments() {
            if let Some(i) = cb.intersect(&ls) {
                if let Some(v) = res.as_mut() {
                    v.extend(i.into_iter().map(|(p, _n)| p));
                } else {
                    res = Some(i.into_iter().map(|(p, _n)| p).collect());
                }
            }
        }
        res
    }
}

impl Intersect<Logic> for ConvexPolygon {
    type Intersection = Vec<P2>;
    fn intersect(&self, logic: &Logic) -> Option<Self::Intersection> {
        let a = logic.get_a();
        let b = logic.get_b();
        let isa = self.intersect(&a);
        let isb = self.intersect(&b);
        match logic.op {
            LogicOp::And => {
                let isa: Option<Vec<P2>> =
                    isa.map(|v| v.into_iter().filter(|p| b.contains(p)).collect());
                let isb: Option<Vec<P2>> =
                    isb.map(|v| v.into_iter().filter(|p| a.contains(p)).collect());
                extend_opt_vec(isa, isb)
            }
            LogicOp::Or => extend_opt_vec(isa, isb),
            LogicOp::AndNot => {
                let isa = isa.map(|va| va.into_iter().filter(|p| !b.contains(p)).collect());
                let isb = isb.map(|vb| vb.into_iter().filter(|p| a.contains(p)).collect());
                extend_opt_vec(isa, isb)
            }
        }
    }
}

impl Intersect<Geo> for ConvexPolygon {
    type Intersection = Vec<P2>;
    fn intersect(&self, geo: &Geo) -> Option<Self::Intersection> {
        match geo {
            Geo::GeoPoint(_p) => None,
            Geo::GeoRay(ray) => self
                .intersect(ray)
                .map(|oot| oot.map(|(p, _)| p).into_vec()),
            Geo::GeoLineSegment(ls) => self.intersect(ls).map(|oot| oot.map(|(p, _)| p).into_vec()),
            Geo::GeoRect(rect) => self.intersect(rect),
            Geo::GeoCircle(circle) => self.intersect(circle),
            Geo::GeoMCircle(mcircle) => self.intersect(mcircle),
            Geo::GeoCubicBezier(cb) => self.intersect(cb),
            Geo::GeoLogic(logic) => self.intersect(logic),
        }
    }
}
