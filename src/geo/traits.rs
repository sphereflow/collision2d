extern crate nalgebra as na;

use super::aabb::AABB;
use super::circle::Circle;
use super::line::Line;
use super::line_segment::LineSegment;
use super::mcircle::MCircle;
use super::ray::Ray;
use super::rect::Rect;
use super::Geo;
use crate::utils::*;
use na::Matrix2;

pub trait HasOrigin {
    fn get_origin(&self) -> P2;
    fn set_origin(&mut self, origin: P2);
}

/// wether a shape contains a point
pub trait Contains {
    fn contains(&self, p: P2) -> bool;
}

/// trait for distances between Points and other primitives
pub trait Distance {
    fn distance(&self, p: &P2) -> Float;
}

pub trait Rotate
where
    Self: HasOrigin,
{
    fn get_rotation(&self) -> V2;
    fn set_rotation(&mut self, x_axis: V2);
    fn look_at(&mut self, p: P2) {
        self.set_rotation(p - self.get_origin());
    }
    /// rotation from local to global
    fn rotate_point(&self, p: P2) -> P2 {
        let x_axis = self.get_rotation();
        let rotation_matrix = Matrix2::new(x_axis.x, -x_axis.y, x_axis.y, x_axis.x);
        rotation_matrix * p
    }
    /// tranformation from local to global
    fn transform(&self, p: P2) -> P2 {
        (self.get_origin().coords + self.rotate_point(p).coords).into()
    }
}

pub trait Scale {
    fn scale_position(&mut self, scale_x: Float, scale_y: Float);
    fn scale(&mut self, scale_x: Float, scale_y: Float);
}

impl Scale for P2 {
    fn scale(&mut self, _scale_x: Float, _scale_y: Float) {}
    fn scale_position(&mut self, scale_x: Float, scale_y: Float) {
        self.x *= scale_x;
        self.y *= scale_y;
    }
}

pub trait Intersect<T> {
    type Intersection;

    fn intersect(&self, other: &T) -> Option<Self::Intersection>;

    fn does_collide(&self, other: &T) -> bool {
        self.intersect(other).is_some()
    }

    fn get_extra_data_intersect(&self, _other: &T) -> Vec<Geo> {
        Vec::new()
    }
}

/// trait for reflection of moving objects
pub trait ReflectOn<T>
where
    Self: Sized,
{
    /// returns: (reflected object, normal, intersection point)
    fn reflect_on_normal_intersect(&self, _other: &T) -> Option<(Self, V2, P2)> {
        None
    }
    /// reflects the moving object on another object and returns the moving
    /// object after the reflection and the normal vector of the reflection
    fn reflect_on_normal(&self, other: &T) -> Option<(Self, V2)> {
        self.reflect_on_normal_intersect(other)
            .map(|(obj, n, _)| (obj, n))
    }
    /// reflects the moving object on another object and returns the moving
    /// object after the reflection the default implementation uses reflect_on_normal
    fn reflect_on(&self, other: &T) -> Option<Self> {
        self.reflect_on_normal(other).map(|(res, _)| res)
    }
    fn get_extra_data_reflect_on(&self, _other: &T) -> Vec<Geo> {
        Vec::new()
    }
}

pub trait CanCollideWith<T>
where
    Self: Sized + ReflectOn<T>,
{
}

pub trait GeoT
where
    Self: Sized
        + CanCollideWith<Line>
        + CanCollideWith<Ray>
        + CanCollideWith<LineSegment>
        + CanCollideWith<Circle>
        + CanCollideWith<Rect>
        + CanCollideWith<AABB>
        + CanCollideWith<MCircle>,
{
}

pub trait HasGeometry {
    fn get_geometry(&self) -> Geo;
}
