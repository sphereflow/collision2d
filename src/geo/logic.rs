use super::*;

#[derive(Clone, PartialEq, Debug)]
pub struct Logic {
    pub op: LogicOp,
    pub a: Box<Geo>,
    pub b: Box<Geo>,
    pub origin: P2,
    pub rotation: Rot2,
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum LogicOp {
    And,
    Or,
    AndNot,
}

impl Logic {
    pub fn new(op: LogicOp, a: Geo, b: Geo, origin: P2, rotation: Rot2) -> Self {
        Logic {
            op,
            a: Box::new(a),
            b: Box::new(b),
            origin,
            rotation,
        }
    }
    pub fn get_a(&self) -> Geo {
        let mut ret = (*self.a).clone();
        ret.set_origin(self.origin + self.rotation * self.a.get_origin().coords);
        ret
    }
    pub fn get_b(&self) -> Geo {
        let mut ret = (*self.b).clone();
        ret.set_origin(self.origin + self.rotation * self.b.get_origin().coords);
        ret
    }
}

impl HasOrigin for Logic {
    fn get_origin(&self) -> P2 {
        self.origin
    }
    fn set_origin(&mut self, origin: P2) {
        self.origin = origin;
    }
}

impl Rotate for Logic {
    fn get_rotation(&self) -> Rot2 {
        self.rotation
    }
    fn set_rotation(&mut self, rotation: &Rot2) {
        let rot_diff = Rot2::rotation_between(&self.get_x_axis(), &rotation.matrix().column(0));
        self.rotation = *rotation;

        self.a.rotate(&rot_diff);
        self.b.rotate(&rot_diff);
    }
}

impl Scale for Logic {
    fn scale(&mut self, scale_x: Float, scale_y: Float) {
        self.a.scale_position(scale_x, scale_y);
        self.b.scale_position(scale_x, scale_y);
        self.a.scale(scale_x, scale_y);
        self.b.scale(scale_x, scale_y);
    }
    fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
        self.origin.scale_position(scale_x, scale_y);
    }
}

impl Contains for Logic {
    fn contains(&self, p: &P2) -> bool {
        match self.op {
            LogicOp::And => self.get_a().contains(p) && self.get_b().contains(p),
            LogicOp::Or => self.get_a().contains(p) || self.get_b().contains(p),
            LogicOp::AndNot => self.get_a().contains(p) && !self.get_b().contains(p),
        }
    }
}

impl ClosestPoint for Logic {
    fn closest_point_to(&self, p: &P2) -> P2 {
        let a = self.get_a();
        let b = self.get_b();
        let cla = a.closest_point_to(p);
        let clb = b.closest_point_to(p);
        let mut candidates;
        if let Some(intersect) = a.intersect(&b) {
            candidates = intersect;
        } else {
            candidates = Vec::new();
        }
        match self.op {
            LogicOp::And => {
                if a.contains(&clb) {
                    candidates.push(clb);
                }
                if b.contains(&cla) {
                    candidates.push(cla);
                }
            }
            LogicOp::Or => {
                candidates.push(cla);
                candidates.push(clb);
            }
            LogicOp::AndNot => {
                if a.contains(&clb) {
                    candidates.push(clb);
                }
                if !b.contains(&cla) {
                    candidates.push(cla);
                }
            }
        }
        let mut res = candidates[0];
        for candidate in candidates[1..].into_iter() {
            if distance(&res, p) > distance(candidate, p) {
                res = *candidate;
            }
        }
        res
    }
}

impl Distance for Logic {
    fn distance(&self, p: &P2) -> Float {
        let a = self.get_a();
        let b = self.get_b();
        let da = a.distance(p);
        let db = b.distance(p);
        let mut res = da;
        match self.op {
            LogicOp::And => da.max(db),
            LogicOp::Or => da.min(db),
            LogicOp::AndNot => {
                if a.contains(p) {
                    res = da.min(-db);
                }
                if let Some(intersection_points) = a.intersect(&b) {
                    for i in intersection_points {
                        let dip = distance(&i, p);
                        if dip < da {
                            res = dip;
                        }
                    }
                    res
                } else {
                    res
                }
            }
        }
    }
}

impl Mirror for Logic {
    fn mirror_x(&self) -> Self {
        let mut res = self.clone();
        let ao = res.a.get_origin();
        let bo = res.b.get_origin();
        res.a.set_origin(P2::new(-ao.x, ao.y));
        res.b.set_origin(P2::new(-bo.x, bo.y));
        res.a.mirror_x();
        res.b.mirror_x();
        res
    }

    fn mirror_y(&self) -> Self {
        let mut res = self.clone();
        let ao = res.a.get_origin();
        let bo = res.b.get_origin();
        res.a.set_origin(P2::new(ao.x, -ao.y));
        res.b.set_origin(P2::new(bo.x, -bo.y));
        res.a.mirror_y();
        res.b.mirror_y();
        res
    }
}

impl Intersect<LineSegment> for Logic {
    type Intersection = Vec<P2>;
    fn intersect(&self, line_segment: &LineSegment) -> Option<Self::Intersection> {
        let a = self.get_a();
        let b = self.get_b();
        let isa = a
            .intersect(line_segment)
            .into_iter()
            .flat_map(|i| i.into_iter())
            .filter(|p| match self.op {
                LogicOp::And => b.contains(p),
                LogicOp::Or => true,
                LogicOp::AndNot => !b.contains(p),
            });
        let isb = b
            .intersect(line_segment)
            .into_iter()
            .flat_map(|i| i.into_iter())
            .filter(|p| match self.op {
                LogicOp::And => a.contains(p),
                LogicOp::Or => true,
                LogicOp::AndNot => a.contains(p),
            });
        let res: Vec<P2> = isa.chain(isb).collect();
        if !res.is_empty() {
            Some(res)
        } else {
            None
        }
    }
}

impl Intersect<Rect> for Logic {
    type Intersection = Vec<P2>;
    fn intersect(&self, rect: &Rect) -> Option<Self::Intersection> {
        let a = self.get_a();
        let b = self.get_b();
        let isa = a
            .intersect(rect)
            .into_iter()
            .flat_map(|i| i.into_iter())
            .filter(|p| match self.op {
                LogicOp::And => b.contains(p),
                LogicOp::Or => true,
                LogicOp::AndNot => !b.contains(p),
            });
        let isb = b
            .intersect(rect)
            .into_iter()
            .flat_map(|i| i.into_iter())
            .filter(|p| match self.op {
                LogicOp::And => a.contains(p),
                LogicOp::Or => true,
                LogicOp::AndNot => a.contains(p),
            });
        let res: Vec<P2> = isa.chain(isb).collect();
        if !res.is_empty() {
            Some(res)
        } else {
            None
        }
    }
}

impl Intersect<Circle> for Logic {
    type Intersection = Vec<P2>;
    fn intersect(&self, circle: &Circle) -> Option<Self::Intersection> {
        let a = self.get_a();
        let b = self.get_b();
        let isa = a
            .intersect(circle)
            .into_iter()
            .flat_map(|i| i.into_iter())
            .filter(|p| match self.op {
                LogicOp::And => b.contains(p),
                LogicOp::Or => true,
                LogicOp::AndNot => !b.contains(p),
            });
        let isb = b
            .intersect(circle)
            .into_iter()
            .flat_map(|i| i.into_iter())
            .filter(|p| match self.op {
                LogicOp::And => a.contains(p),
                LogicOp::Or => true,
                LogicOp::AndNot => a.contains(p),
            });
        let res: Vec<P2> = isa.chain(isb).collect();
        if !res.is_empty() {
            Some(res)
        } else {
            None
        }
    }
}

impl Intersect<MCircle> for Logic {
    type Intersection = Vec<P2>;
    fn intersect(&self, mcircle: &MCircle) -> Option<Self::Intersection> {
        let a = self.get_a();
        let b = self.get_b();
        let isa = a
            .intersect(mcircle)
            .into_iter()
            .flat_map(|i| i.into_iter())
            .filter(|p| match self.op {
                LogicOp::And => b.contains(p),
                LogicOp::Or => true,
                LogicOp::AndNot => !b.contains(p),
            });
        let isb = b
            .intersect(mcircle)
            .into_iter()
            .flat_map(|i| i.into_iter())
            .filter(|p| match self.op {
                LogicOp::And => a.contains(p),
                LogicOp::Or => true,
                LogicOp::AndNot => a.contains(p),
            });
        let res: Vec<P2> = isa.chain(isb).collect();
        if !res.is_empty() {
            Some(res)
        } else {
            None
        }
    }
}

impl Intersect<Logic> for Logic {
    type Intersection = Vec<P2>;
    fn intersect(&self, logic: &Logic) -> Option<Self::Intersection> {
        let a = self.get_a();
        let b = self.get_b();
        let isa = a
            .intersect(logic)
            .into_iter()
            .flat_map(|i| i.into_iter())
            .filter(|p| match self.op {
                LogicOp::And => b.contains(p),
                LogicOp::Or => true,
                LogicOp::AndNot => !b.contains(p),
            });
        let isb = b
            .intersect(logic)
            .into_iter()
            .flat_map(|i| i.into_iter())
            .filter(|p| match self.op {
                LogicOp::And => a.contains(p),
                LogicOp::Or => true,
                LogicOp::AndNot => a.contains(p),
            });
        let res: Vec<P2> = isa.chain(isb).collect();
        if !res.is_empty() {
            Some(res)
        } else {
            None
        }
    }
}
