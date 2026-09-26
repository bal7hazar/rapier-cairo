//! Parry's `query::intersection_test_*` entry points (`parry/src/query/intersection_test/`) with
//! upstream's names and signatures, as thin wrappers over the kernels of
//! [`crate::dispatch::intersection`] (LO1), and Parry's `ShapeIntersection` result type.
//!
//! `pos12` is the pose of shape 2 in the frame of shape 1, as everywhere in [`crate::query`].
//! Touching counts as intersecting.
//!
//! # Deviations
//!
//! * Parry's generic `SupportMap` / `PointQuery` arguments are the closed [`Shape`] enum here. A
//!   support-map argument that is a half-space panics with `'Query: not a support map'`, the
//!   convention of the other `*_support_map` kernels of this module.
//! * `intersection_test_support_map_support_map` is the exact dispatch of
//!   [`crate::dispatch::intersection_test`] (analytic and SAT kernels, not GJK), so upstream's
//!   `_with_params` variant (a GJK simplex and initial direction in, the last direction out)
//!   has no counterpart.
//! * `intersection_test_ball_point_query` and `intersection_test_point_query_ball` answer the
//!   test only: no shape of the closed set has sub-shapes, so both `subshape` ids are `0` as
//!   upstream's for a non-composite shape.
//! * The cuboid–triangle tests (SH1) go through the exact support-map witness of
//!   [`crate::dispatch::intersection_test`]; composite shapes are deferred (SH2).

use core::panic_with_felt252;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::aabb::{Aabb, AabbTrait};
use crate::dispatch::intersection::{cuboid_capsule, halfspace_convex, point_query_ball};
use crate::feature_id::SubShapeId;
use crate::shape::{Ball, Capsule, Cuboid, CuboidTrait, HalfSpace, Segment, Shape, Triangle};
use super::errors::NOT_SUPPORT_MAP;

/// The result of an intersection test (Parry `ShapeIntersection`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ShapeIntersection {
    /// Do the two shapes intersect (touching included)?
    pub intersecting: bool,
    /// The sub-shape of the first shape involved in the answer (`0` for a simple shape).
    pub subshape1: SubShapeId,
    /// The sub-shape of the second shape involved in the answer (`0` for a simple shape).
    pub subshape2: SubShapeId,
}

/// Constructors and sub-shape helpers of [`ShapeIntersection`].
#[generate_trait]
pub impl ShapeIntersectionImpl of ShapeIntersectionTrait {
    /// An answer without sub-shape ids (both `0`).
    #[inline(always)]
    fn new(intersecting: bool) -> ShapeIntersection {
        ShapeIntersection { intersecting, subshape1: 0, subshape2: 0 }
    }

    /// Sets the sub-shape ids of the answer.
    #[inline(always)]
    fn with_subshapes(
        self: ShapeIntersection, subshape1: SubShapeId, subshape2: SubShapeId,
    ) -> ShapeIntersection {
        ShapeIntersection { subshape1, subshape2, ..self }
    }

    /// The same answer with the two shapes exchanged: the sub-shape ids are swapped.
    #[inline(always)]
    fn swapped(self: ShapeIntersection) -> ShapeIntersection {
        ShapeIntersection { subshape1: self.subshape2, subshape2: self.subshape1, ..self }
    }
}

/// Upstream `From<bool>`: [`ShapeIntersectionTrait::new`].
pub impl BoolIntoShapeIntersection of Into<bool, ShapeIntersection> {
    #[inline(always)]
    fn into(self: bool) -> ShapeIntersection {
        ShapeIntersectionTrait::new(self)
    }
}

/// A segment is a capsule of radius zero for the cuboid kernel.
#[inline(always)]
fn core_of(segment: Segment) -> Capsule {
    Capsule { segment, radius: Default::default() }
}

/// Whether a cuboid and a segment intersect (Parry `intersection_test_cuboid_segment`; SAT on the
/// three axes of the pair, exact).
/// #### Panics
/// * Integer overflow when a transformed coordinate leaves its range (coordinates bounded by
///   2^20 are safe).
pub fn intersection_test_cuboid_segment(pos12: Pose2, cube1: Cuboid, segment2: Segment) -> bool {
    cuboid_capsule(pos12, cube1, core_of(segment2))
}

/// [`intersection_test_cuboid_segment`] with the shapes in the other order (Parry
/// `intersection_test_segment_cuboid`).
/// #### Panics
/// * See [`intersection_test_cuboid_segment`] and `Pose2::inverse`.
pub fn intersection_test_segment_cuboid(pos12: Pose2, segment1: Segment, cuboid2: Cuboid) -> bool {
    cuboid_capsule(pos12.inverse(), cuboid2, core_of(segment1))
}

/// Whether an AABB and a segment given in the frame of the AABB's centre intersect (Parry
/// `intersection_test_aabb_segment`, the AABB seen as a cuboid at the origin: `segment2` is
/// translated by `-center` first, as upstream does).
/// #### Panics
/// * See [`intersection_test_cuboid_segment`].
pub fn intersection_test_aabb_segment(aabb1: Aabb, segment2: Segment) -> bool {
    let cuboid1 = CuboidTrait::new(aabb1.half_extents());
    let pos12 = Pose2Trait::new(-aabb1.center(), Rot2Trait::IDENTITY);
    intersection_test_cuboid_segment(pos12, cuboid1, segment2)
}

/// Whether a cuboid and a triangle placed at `pos12` intersect (Parry
/// `intersection_test_cuboid_triangle`; SAT there, the exact witness here).
/// #### Panics
/// * The overflow panics of the pose transform and of the wide products.
pub fn intersection_test_cuboid_triangle(
    pos12: Pose2, cuboid1: Cuboid, triangle2: Triangle,
) -> bool {
    crate::dispatch::intersection_test(pos12, Shape::Cuboid(cuboid1), triangle2.into()).unwrap()
}

/// [`intersection_test_cuboid_triangle`] with the shapes in the other order (Parry
/// `intersection_test_triangle_cuboid`).
/// #### Panics
/// * See [`intersection_test_cuboid_triangle`].
pub fn intersection_test_triangle_cuboid(
    pos12: Pose2, triangle1: Triangle, cuboid2: Cuboid,
) -> bool {
    crate::dispatch::intersection_test(pos12, triangle1.into(), Shape::Cuboid(cuboid2)).unwrap()
}

/// Whether an AABB and a triangle, both in the same frame, intersect (Parry
/// `intersection_test_aabb_triangle`).
/// #### Panics
/// * See [`intersection_test_cuboid_triangle`].
pub fn intersection_test_aabb_triangle(aabb1: Aabb, triangle2: Triangle) -> bool {
    let cuboid1 = CuboidTrait::new(aabb1.half_extents());
    let pos12 = Pose2Trait::new(-aabb1.center(), Rot2Trait::IDENTITY);
    intersection_test_cuboid_triangle(pos12, cuboid1, triangle2)
}

/// Whether a half-space and a support-map shape intersect: the deepest point of `other` along
/// `-normal` is on the inner side (Parry `intersection_test_halfspace_support_map`).
/// #### Panics
/// * `'Query: not a support map'` when `other` is a half-space; the transform's overflow panics.
pub fn intersection_test_halfspace_support_map(
    pos12: Pose2, halfspace: HalfSpace, other: Shape,
) -> bool {
    halfspace_convex(pos12, halfspace, other).expect(NOT_SUPPORT_MAP)
}

/// [`intersection_test_halfspace_support_map`] with the shapes in the other order (Parry
/// `intersection_test_support_map_halfspace`).
/// #### Panics
/// * See [`intersection_test_halfspace_support_map`] and `Pose2::inverse`.
pub fn intersection_test_support_map_halfspace(
    pos12: Pose2, other: Shape, halfspace: HalfSpace,
) -> bool {
    intersection_test_halfspace_support_map(pos12.inverse(), halfspace, other)
}

/// Whether two support-map shapes intersect (Parry `intersection_test_support_map_support_map`;
/// GJK there, the exact kernel of [`crate::dispatch::intersection_test`] here).
/// #### Panics
/// * `'Query: not a support map'` when a shape is a half-space.
pub fn intersection_test_support_map_support_map(pos12: Pose2, g1: Shape, g2: Shape) -> bool {
    match (g1, g2) {
        (Shape::HalfSpace(_), _) | (_, Shape::HalfSpace(_)) => panic_with_felt252(NOT_SUPPORT_MAP),
        _ => crate::dispatch::intersection_test(pos12, g1, g2).unwrap(),
    }
}

/// Whether a ball centred at `pos12.translation` (a point-query shape 2) touches `point_query1`
/// (Parry `intersection_test_point_query_ball`): the solid projection of the centre is inside
/// the shape or within the radius. Both sub-shape ids are `0`.
/// #### Panics
/// * See [`crate::dispatch::intersection_test`].
pub fn intersection_test_point_query_ball(
    pos12: Pose2, point_query1: Shape, ball2: Ball,
) -> ShapeIntersection {
    ShapeIntersectionTrait::new(
        point_query_ball(point_query1, pos12.translation, ball2.radius).unwrap(),
    )
}

/// [`intersection_test_point_query_ball`] with the shapes in the other order (Parry
/// `intersection_test_ball_point_query`).
/// #### Panics
/// * See [`intersection_test_point_query_ball`] and `Pose2::inverse`.
pub fn intersection_test_ball_point_query(
    pos12: Pose2, ball1: Ball, point_query2: Shape,
) -> ShapeIntersection {
    intersection_test_point_query_ball(pos12.inverse(), point_query2, ball1).swapped()
}

#[cfg(test)]
mod tests;
