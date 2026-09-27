//! Point queries on the [`Compound`] (work package SH2b): Parry `impl PointQuery for Compound`
//! (`query/point/point_composite_shape.rs`, through `CompositeShapeRef`).
//!
//! A compound answers the projection on its closest part: every part in ascending index, the
//! point moved into the part's frame, projected there and moved back (upstream
//! `Shape::project_point(part_pose, pt, solid)`), the first strictly closer part winning (the
//! closest point compared on the exact wide squared distance). Upstream walks its BVH best
//! first, which finds the same minimum; only exact ties may pick another part. The part's index
//! is returned by the `*_part` functions (upstream's `CompositeShapeRef` answers).
//!
//! The parts are convex (`CompoundTrait::new` rejects composites), so their tables here
//! ([`project_part`], …) are the convex arms of `ShapePointQuery`, out of line: the inlined
//! dispatcher cannot sit on the recursion compound → part.

use fixed::wide::distance2;
use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_math::math_ext::norm2::norm2_sq_wide;
use rapier_math::pose2::Pose2Trait;
use crate::aabb::contains_local_point_aabb;
use crate::feature_id::{FEATURE_UNKNOWN, FeatureId};
use crate::shape::{Compound, CompoundTrait, Shape};
use super::convex_polygon::{
    contains_local_point_convex_polygon, project_local_point_and_get_feature_convex_polygon,
    project_local_point_convex_polygon,
};
use super::query::PointQuery;
use super::round_shape::{
    contains_local_point_round, project_local_point_and_get_feature_round,
    project_local_point_round,
};
use super::triangle::{
    contains_local_point_triangle, project_local_point_and_get_feature_triangle,
    project_local_point_triangle,
};
use super::{
    PointProjection, PointProjectionTrait, contains_local_point_ball, contains_local_point_capsule,
    contains_local_point_cuboid, contains_local_point_halfspace, contains_local_point_segment,
    project_local_point_and_get_feature_ball, project_local_point_and_get_feature_capsule,
    project_local_point_and_get_feature_cuboid, project_local_point_and_get_feature_halfspace,
    project_local_point_and_get_feature_segment, project_local_point_ball,
    project_local_point_capsule, project_local_point_cuboid, project_local_point_halfspace,
    project_local_point_segment,
};

/// `ShapePointQuery::project_local_point` on a convex part; `pt` outside for a composite.
#[inline(never)]
pub fn project_part(shape: Shape, pt: Vec2, solid: bool) -> PointProjection {
    match shape {
        Shape::Ball(s) => project_local_point_ball(s, pt, solid),
        Shape::Cuboid(s) => project_local_point_cuboid(s, pt, solid),
        Shape::Capsule(s) => project_local_point_capsule(s, pt, solid),
        Shape::Segment(s) => project_local_point_segment(s, pt, solid),
        Shape::HalfSpace(s) => project_local_point_halfspace(s, pt, solid),
        Shape::ConvexPolygon(s) => project_local_point_convex_polygon(s.unbox(), pt, solid),
        Shape::Triangle(s) => project_local_point_triangle(s.unbox(), pt, solid),
        Shape::RoundCuboid(s) => project_local_point_round(
            s.inner_shape, s.border_radius, pt, solid,
        ),
        Shape::RoundTriangle(s) => {
            let s = s.unbox();
            project_local_point_round(s.inner_shape, s.border_radius, pt, solid)
        },
        Shape::RoundConvexPolygon(s) => {
            let s = s.unbox();
            project_local_point_round(s.inner_shape, s.border_radius, pt, solid)
        },
        _ => PointProjectionTrait::new(false, pt),
    }
}

/// `ShapePointQuery::project_local_point_and_get_feature` on a convex part.
#[inline(never)]
pub fn project_and_get_feature_part(shape: Shape, pt: Vec2) -> (PointProjection, FeatureId) {
    match shape {
        Shape::Ball(s) => project_local_point_and_get_feature_ball(s, pt),
        Shape::Cuboid(s) => project_local_point_and_get_feature_cuboid(s, pt),
        Shape::Capsule(s) => project_local_point_and_get_feature_capsule(s, pt),
        Shape::Segment(s) => project_local_point_and_get_feature_segment(s, pt),
        Shape::HalfSpace(s) => project_local_point_and_get_feature_halfspace(s, pt),
        Shape::ConvexPolygon(s) => project_local_point_and_get_feature_convex_polygon(
            s.unbox(), pt,
        ),
        Shape::Triangle(s) => project_local_point_and_get_feature_triangle(s.unbox(), pt),
        Shape::RoundCuboid(s) => project_local_point_and_get_feature_round(
            s.inner_shape, s.border_radius, pt,
        ),
        Shape::RoundTriangle(s) => {
            let s = s.unbox();
            project_local_point_and_get_feature_round(s.inner_shape, s.border_radius, pt)
        },
        Shape::RoundConvexPolygon(s) => {
            let s = s.unbox();
            project_local_point_and_get_feature_round(s.inner_shape, s.border_radius, pt)
        },
        _ => (PointProjectionTrait::new(false, pt), FEATURE_UNKNOWN),
    }
}

/// `ShapePointQuery::contains_local_point` on a convex part.
#[inline(never)]
pub fn contains_part(shape: Shape, pt: Vec2) -> bool {
    match shape {
        Shape::Ball(s) => contains_local_point_ball(s, pt),
        Shape::Cuboid(s) => contains_local_point_cuboid(s, pt),
        Shape::Capsule(s) => contains_local_point_capsule(s, pt),
        Shape::Segment(s) => contains_local_point_segment(s, pt),
        Shape::HalfSpace(s) => contains_local_point_halfspace(s, pt),
        Shape::ConvexPolygon(s) => contains_local_point_convex_polygon(s.unbox(), pt),
        Shape::Triangle(s) => contains_local_point_triangle(s.unbox(), pt),
        Shape::RoundCuboid(s) => contains_local_point_round(s.inner_shape, s.border_radius, pt),
        Shape::RoundTriangle(s) => {
            let s = s.unbox();
            contains_local_point_round(s.inner_shape, s.border_radius, pt)
        },
        Shape::RoundConvexPolygon(s) => {
            let s = s.unbox();
            contains_local_point_round(s.inner_shape, s.border_radius, pt)
        },
        _ => false,
    }
}

/// `|pt - q|^2`, exact.
#[inline(always)]
fn sq_dist(pt: Vec2, q: Vec2) -> i128 {
    norm2_sq_wide(q.x - pt.x, q.y - pt.y)
}

/// Upstream `CompositeShapeRef::project_local_point` on a compound: `(part, projection)` of the
/// closest part, the projection in the compound's frame.
pub fn project_local_point_compound_part(
    compound: @Compound, pt: Vec2, solid: bool,
) -> Option<(u32, PointProjection)> {
    let mut best: Option<(u32, PointProjection)> = None;
    let mut best_sq: i128 = 0;
    let mut i: u32 = 0;
    for part in compound.shapes() {
        let (pose, shape) = *part;
        let mut proj = project_part(shape, pose.inverse_transform_point(pt), solid);
        proj.point = pose.transform_point(proj.point);
        let d = sq_dist(pt, proj.point);
        if best.is_none() || d < best_sq {
            best = Some((i, proj));
            best_sq = d;
        }
        i += 1;
    }
    best
}

/// Upstream `CompositeShapeRef::project_local_point_and_get_feature` on a compound: `(part,
/// (projection, the part's feature))` of the closest part.
pub fn project_local_point_and_get_feature_compound_part(
    compound: @Compound, pt: Vec2,
) -> Option<(u32, (PointProjection, FeatureId))> {
    let mut best: Option<(u32, (PointProjection, FeatureId))> = None;
    let mut best_sq: i128 = 0;
    let mut i: u32 = 0;
    for part in compound.shapes() {
        let (pose, shape) = *part;
        let (mut proj, feature) = project_and_get_feature_part(
            shape, pose.inverse_transform_point(pt),
        );
        proj.point = pose.transform_point(proj.point);
        let d = sq_dist(pt, proj.point);
        if best.is_none() || d < best_sq {
            best = Some((i, (proj, feature)));
            best_sq = d;
        }
        i += 1;
    }
    best
}

/// Upstream `CompositeShapeRef::contains_local_point` on a compound: the first part (ascending)
/// whose box and shape contain `pt`.
pub fn contains_local_point_compound_part(compound: @Compound, pt: Vec2) -> Option<u32> {
    let shapes = compound.shapes();
    let mut i: u32 = 0;
    for bv in compound.aabbs() {
        if contains_local_point_aabb(*bv, pt) {
            let (pose, shape) = *shapes.at(i);
            if contains_part(shape, pose.inverse_transform_point(pt)) {
                return Some(i);
            }
        }
        i += 1;
    }
    None
}

/// Upstream `PointQuery::project_local_point` for `Compound`: the closest part's projection.
pub fn project_local_point_compound(compound: @Compound, pt: Vec2, solid: bool) -> PointProjection {
    match project_local_point_compound_part(compound, pt, solid) {
        Some((_, proj)) => proj,
        None => PointProjectionTrait::new(false, pt),
    }
}

/// Upstream `PointQuery::project_local_point_and_get_feature` for `Compound`: the closest part's
/// projection and `Unknown`.
pub fn project_local_point_and_get_feature_compound(
    compound: @Compound, pt: Vec2,
) -> (PointProjection, FeatureId) {
    match project_local_point_and_get_feature_compound_part(compound, pt) {
        Some((_, (proj, _))) => (proj, FEATURE_UNKNOWN),
        None => (PointProjectionTrait::new(false, pt), FEATURE_UNKNOWN),
    }
}

/// Upstream's default `distance_to_local_point` on the compound projection: the distance,
/// negated for a non-solid query from inside a part.
pub fn distance_to_local_point_compound(compound: @Compound, pt: Vec2, solid: bool) -> Fixed {
    let proj = project_local_point_compound(compound, pt, solid);
    let dist = distance2(pt.x, pt.y, proj.point.x, proj.point.y);
    if solid || !proj.is_inside {
        dist
    } else {
        ZERO - dist
    }
}

/// Upstream `PointQuery::contains_local_point` for `Compound`: a part contains `pt`.
pub fn contains_local_point_compound(compound: @Compound, pt: Vec2) -> bool {
    contains_local_point_compound_part(compound, pt).is_some()
}

pub impl CompoundPointQuery of PointQuery<Compound> {
    fn project_local_point(self: Compound, pt: Vec2, solid: bool) -> PointProjection {
        project_local_point_compound(@self, pt, solid)
    }
    fn project_local_point_and_get_feature(
        self: Compound, pt: Vec2,
    ) -> (PointProjection, FeatureId) {
        project_local_point_and_get_feature_compound(@self, pt)
    }
    fn distance_to_local_point(self: Compound, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_compound(@self, pt, solid)
    }
    fn contains_local_point(self: Compound, pt: Vec2) -> bool {
        contains_local_point_compound(@self, pt)
    }
}

/// The compound arm of `ShapePointQuery::project_local_point`, out of line.
#[inline(never)]
pub fn project_local_point_compound_shape(shape: Shape, pt: Vec2, solid: bool) -> PointProjection {
    match shape {
        Shape::Compound(s) => project_local_point_compound(@s.unbox(), pt, solid),
        _ => PointProjectionTrait::new(false, pt),
    }
}

/// The compound arm of `ShapePointQuery::project_local_point_and_get_feature`, out of line.
#[inline(never)]
pub fn project_local_point_and_get_feature_compound_shape(
    shape: Shape, pt: Vec2,
) -> (PointProjection, FeatureId) {
    match shape {
        Shape::Compound(s) => project_local_point_and_get_feature_compound(@s.unbox(), pt),
        _ => (PointProjectionTrait::new(false, pt), FEATURE_UNKNOWN),
    }
}

/// The compound arm of `ShapePointQuery::distance_to_local_point`, out of line.
#[inline(never)]
pub fn distance_to_local_point_compound_shape(shape: Shape, pt: Vec2, solid: bool) -> Fixed {
    match shape {
        Shape::Compound(s) => distance_to_local_point_compound(@s.unbox(), pt, solid),
        _ => ZERO,
    }
}
