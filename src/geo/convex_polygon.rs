use super::*;

pub struct ConvexPolygon {
    /// the origin of the convex polygon
    origin: P2,
    /// x_axis for rotation
    x_axis: V2,
    /// points relative to origin sorted in clockwise direction
    points: Vec<V2>,
}

use std::ops::Add;
impl ConvexPolygon {
    pub fn new_convex_hull(points: &Vec<P2>) -> Self {
        let mut all_points: Vec<V2> = points.iter().map(|p| p.coords).collect();
        let mut origin: V2 = all_points.iter().fold(V2::new(0., 0.), V2::add);
        origin /= points.len() as Float;
        for point in all_points.iter_mut() {
            point.x -= origin.x;
            point.y -= origin.y;
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
            p1.y.partial_cmp(&p2.y)
                .expect("convex hull: could not sort left side")
        });
        // sort the right side top to bottom
        right.sort_unstable_by(|p1, p2| {
            p2.y.partial_cmp(&p1.y)
                .expect("convex hull: could not sort right side")
        });
        right.extend(left.into_iter());
        let all_points = right;

        // Graham scan
        //
        let mut hull_points: Vec<V2> = Vec::new();
        for p in all_points {
            while hull_points.len() > 1
                && !is_clockwise(
                    &hull_points[hull_points.len() - 2],
                    &hull_points[hull_points.len() - 1],
                    &p,
                )
            {
                hull_points.pop();
            }
            hull_points.push(p);
        }

        let mut res = ConvexPolygon {
            origin: P2::from(origin),
            x_axis: V2::new(1., 0.),
            points: hull_points,
        };
        res.recompute_origin();
        res
    }

    pub fn add_point(&mut self, p: &P2) {
        let p = p - self.origin;
        // sort in p
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
            let mut is_cw = is_clockwise(
                &hull_points[hull_points.len() - 2],
                &hull_points[hull_points.len() - 1],
                &p,
            );
            // the following variable will be set if the algorithm encounters a non-clockwise turn
            let mut not_cw_once = !is_cw;
            while hull_points.len() > 1 && !is_cw {
                hull_points.pop();
                is_cw = is_clockwise(
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
        self.recompute_origin();
    }

    fn recompute_origin(&mut self) {
        let mut offset: V2 = self.points.iter().fold(V2::new(0., 0.), V2::add);
        offset /= self.points.len() as Float;
        for p in self.points.iter_mut() {
            p.x -= offset.x;
            p.y -= offset.y;
        }
        self.origin += offset;
    }

    pub fn get_line_segments(&self) -> Vec<LineSegment> {
        let mut res = Vec::with_capacity(self.points.len());
        for w in self.points.windows(2) {
            res.push(LineSegment::from_ab(self.origin + w[0], self.origin + w[1]));
        }
        if let (Some(p1), Some(p2)) = (self.points.last(), self.points.first()) {
            res.push(LineSegment::from_ab(self.origin + p1, self.origin + p2));
        }
        res
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
            if !is_clockwise(&w[0], &w[1], &p.coords) {
                return false;
            }
        }
        if let (Some(first), Some(last)) = (self.points.first(), self.points.last()) {
            if !is_clockwise(last, first, &p.coords) {
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
            if is_clockwise(&ls.get_a().coords, &ls.get_b().coords, &p.coords) {
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
    fn get_rotation(&self) -> V2 {
        self.x_axis
    }
    fn set_rotation(&mut self, x_axis: &V2) {
        self.x_axis = *x_axis;
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
        self.origin.x *= scale_x;
        self.origin.y *= scale_y;
    }
}
