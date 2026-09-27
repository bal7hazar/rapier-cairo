//! Rejected `Shape::compute_aabb` representation, kept for `gas_compute_aabb_*_outlined`
//! (see the module doc of `crate::shape`).

use rapier_math::pose2::Pose2;
use crate::aabb::Aabb;
use super::{
    BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait, Shape,
};

/// Rejected direct polygon payload; increases every Shape value to 34 felts.
#[derive(Copy, Drop)]
pub enum UnboxedShape {
    Ball: super::Ball,
    Cuboid: super::Cuboid,
    Capsule: super::Capsule,
    Segment: super::Segment,
    HalfSpace: super::HalfSpace,
    ConvexPolygon: super::ConvexPolygon,
}

/// Rejected 34-felt enum representation: existing scene probes measured the copy overhead
/// before boxing the polygon payload. This retained AABB path exercises that representation.
#[inline(never)]
pub fn compute_aabb_outlined(shape: Shape, pose: Pose2) -> Aabb {
    let shape = match shape {
        Shape::Ball(s) => UnboxedShape::Ball(s),
        Shape::Cuboid(s) => UnboxedShape::Cuboid(s),
        Shape::Capsule(s) => UnboxedShape::Capsule(s),
        Shape::Segment(s) => UnboxedShape::Segment(s),
        Shape::HalfSpace(s) => UnboxedShape::HalfSpace(s),
        Shape::ConvexPolygon(s) => UnboxedShape::ConvexPolygon(s.unbox()),
        // The SH1 shapes postdate this candidate.
        _ => core::panic_with_felt252('Shape: not in candidate'),
    };
    compute_unboxed_aabb(shape, pose)
}

#[inline(never)]
fn compute_unboxed_aabb(shape: UnboxedShape, pose: Pose2) -> Aabb {
    match shape {
        UnboxedShape::Ball(s) => s.compute_aabb(pose),
        UnboxedShape::Capsule(s) => s.compute_aabb(pose),
        UnboxedShape::Cuboid(s) => s.compute_aabb(pose),
        UnboxedShape::HalfSpace(s) => s.compute_aabb(pose),
        UnboxedShape::Segment(s) => s.compute_aabb(pose),
        UnboxedShape::ConvexPolygon(s) => s.compute_aabb(pose),
    }
}
