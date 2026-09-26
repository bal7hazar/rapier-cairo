//! Point projection on a [`RoundShape`] (Parry `query/point/point_round_shape.rs`).
//!
//! Upstream projects on the support map of the dilated shape with GJK. The dilated shape is the
//! set of points within `border_radius` of the inner shape, so the projection is exact in closed
//! form from the inner shape's own projection:
//!
//! * outside the inner shape, at distance `d` from its projection `q`: outside the round shape
//!   when `d > r` (compared wide), the projection being `q + (pt - q) r / d`; otherwise inside,
//!   and the boundary point is the same `q + (pt - q) r / d`;
//! * inside the inner shape (boundary included): the boundary projection `q` of the inner shape
//!   pushed outward by `r` along `(q - pt) / |q - pt|`, or along the inner feature normal when
//!   `pt` is on the inner boundary.
//!
//! The unit directions are `d * recip(|d|)` (the wide reciprocal of `convex_ball`), the pushes one
//! floor per coordinate. As upstream, the feature is always `Unknown`.

use fixed::wide::{NormTrait, RecipTrait, distance2, norm2_wide};
use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::math_ext::norm2::is_norm2_gt;
use crate::feature_id::{FEATURE_UNKNOWN, FeatureId};
use crate::shape::{
    ConvexPolygon, ConvexPolygonTrait, Cuboid, CuboidTrait, RoundShape, Triangle, TriangleTrait,
};
use super::{PointProjection, PointQuery};

/// The outward normal of an inner shape at the feature its boundary projection landed on.
pub trait RoundInner<T> {
    fn boundary_normal(self: T, feature: FeatureId) -> Option<Vec2>;
}

pub impl CuboidRoundInner of RoundInner<Cuboid> {
    #[inline(always)]
    fn boundary_normal(self: Cuboid, feature: FeatureId) -> Option<Vec2> {
        self.feature_normal(feature)
    }
}

pub impl TriangleRoundInner of RoundInner<Triangle> {
    #[inline(always)]
    fn boundary_normal(self: Triangle, feature: FeatureId) -> Option<Vec2> {
        self.feature_normal(feature)
    }
}

pub impl ConvexPolygonRoundInner of RoundInner<ConvexPolygon> {
    #[inline(always)]
    fn boundary_normal(self: ConvexPolygon, feature: FeatureId) -> Option<Vec2> {
        self.feature_normal(feature)
    }
}

/// `d / |d|` from the wide norm, `None` for a zero `d`.
#[inline(always)]
fn unit(d: Vec2) -> Option<Vec2> {
    match norm2_wide(d.x, d.y).try_recip() {
        Some(r) => Some(Vec2 { x: r.mul(d.x), y: r.mul(d.y) }),
        None => None,
    }
}

/// Projects `pt` on the round shape of `inner` and `radius`: onto the filled shape if `solid`,
/// onto its boundary otherwise (upstream `project_local_point`). See the module documentation.
/// #### Panics
/// * The panics of the inner projection; `'Fixed: overflow'` when a pushed point leaves the
///   scalar range.
pub fn project_local_point_round<T, +Drop<T>, +Copy<T>, +PointQuery<T>, +RoundInner<T>>(
    inner: T, radius: Fixed, pt: Vec2, solid: bool,
) -> PointProjection {
    let filled = inner.project_local_point(pt, true);
    if !filled.is_inside {
        let d = pt - filled.point;
        let inside = !is_norm2_gt(d.x, d.y, radius);
        if inside && solid {
            return PointProjection { is_inside: true, point: pt };
        }
        let dir = unit(d).unwrap_or(Vec2Trait::ZERO);
        return PointProjection { is_inside: inside, point: filled.point + dir.mul_scalar(radius) };
    }
    if solid {
        return PointProjection { is_inside: true, point: pt };
    }
    let (boundary, feature) = inner.project_local_point_and_get_feature(pt);
    let dir = unit(boundary.point - pt)
        .unwrap_or_else(|| inner.boundary_normal(feature).unwrap_or(Vec2Trait::ZERO));
    PointProjection { is_inside: true, point: boundary.point + dir.mul_scalar(radius) }
}

/// Upstream 2D `project_local_point_and_get_feature` of a round shape: the boundary projection
/// and `FeatureId::Unknown`.
pub fn project_local_point_and_get_feature_round<
    T, +Drop<T>, +Copy<T>, +PointQuery<T>, +RoundInner<T>,
>(
    inner: T, radius: Fixed, pt: Vec2,
) -> (PointProjection, FeatureId) {
    (project_local_point_round(inner, radius, pt, false), FEATURE_UNKNOWN)
}

/// Signed distance from `pt` to the round shape (negative inside when not `solid`, zero inside a
/// solid one), the floored length of `pt - proj`.
pub fn distance_to_local_point_round<T, +Drop<T>, +Copy<T>, +PointQuery<T>, +RoundInner<T>>(
    inner: T, radius: Fixed, pt: Vec2, solid: bool,
) -> Fixed {
    let proj = project_local_point_round(inner, radius, pt, solid);
    let dist = distance2(pt.x, pt.y, proj.point.x, proj.point.y);
    if proj.is_inside && !solid {
        ZERO - dist
    } else {
        dist
    }
}

/// Whether `pt` is within `radius` of the filled inner shape, boundary included (exact wide
/// comparison).
pub fn contains_local_point_round<T, +Drop<T>, +Copy<T>, +PointQuery<T>>(
    inner: T, radius: Fixed, pt: Vec2,
) -> bool {
    let filled = inner.project_local_point(pt, true);
    let d = pt - filled.point;
    filled.is_inside || !is_norm2_gt(d.x, d.y, radius)
}

/// `PointQuery` of every round shape whose inner shape has one (upstream `impl<S: SupportMap>
/// PointQuery for RoundShape<S>`).
pub impl RoundShapePointQuery<
    T, +Drop<T>, +Copy<T>, +PointQuery<T>, +RoundInner<T>,
> of PointQuery<RoundShape<T>> {
    fn project_local_point(self: RoundShape<T>, pt: Vec2, solid: bool) -> PointProjection {
        project_local_point_round(self.inner_shape, self.border_radius, pt, solid)
    }
    fn project_local_point_and_get_feature(
        self: RoundShape<T>, pt: Vec2,
    ) -> (PointProjection, FeatureId) {
        project_local_point_and_get_feature_round(self.inner_shape, self.border_radius, pt)
    }
    fn distance_to_local_point(self: RoundShape<T>, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_round(self.inner_shape, self.border_radius, pt, solid)
    }
    fn contains_local_point(self: RoundShape<T>, pt: Vec2) -> bool {
        contains_local_point_round(self.inner_shape, self.border_radius, pt)
    }
}
