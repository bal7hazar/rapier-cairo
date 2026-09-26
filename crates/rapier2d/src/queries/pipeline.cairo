//! The rest of upstream's `QueryPipeline` surface (work package QY2): [`intersect_shape`],
//! [`project_point_and_get_feature`] and the [`QueryPipeline`] view.
//!
//! Same conventions as `super`: a brute-force scan of the enabled colliders in ascending handle
//! order, closest-hit queries strictly below their bound with ties to the lowest handle, array
//! queries in ascending handle order.
//!
//! Upstream's `QueryPipeline` borrows the bodies, colliders, broad phase and dispatcher and owns
//! a filter. The port's world holds dicts (it is passed by `ref`, and a struct cannot hold a
//! reference), so [`QueryPipeline`] is the **filter bundle** only: build it with
//! `WorldTrait::query_pipeline` / `query_pipeline_with_filter` or
//! [`QueryPipelineTrait::with_filter`]
//! and hand the world to each call (`pipeline.cast_ray(ref world, ray, max_toi, solid)`).
//!
//! # Candidates for `intersect_shape`
//!
//! Ranked by `queries::benches_qy2` on 20 mixed colliders (upstream prunes with the BVH first):
//!
//! 1. **AABB pre-test** (this module): the query shape's AABB against the collider's world AABB,
//!    then `rapier_geometry2d::query::intersection_test`. **Winner**: 3.04M gas / 21.4k steps
//!    (net of the scene setup: 2.01M / 12.0k).
//! 2. `alternatives::intersect_shape_direct`: the exact test on every candidate. +12 % gas and
//!    +13 % steps in total (net of the setup: +19 % / +24 %): unlike the ray's slab test, the
//!    box-box test costs less than the kernels it skips.
//!
//! # Deviations
//!
//! * A pair without an intersection kernel (half-space against half-space) is reported as not
//!   intersecting, as upstream's `is_ok_and(|hit| hit.intersecting)` on `Err(Unsupported)`.
//! * `project_point_and_get_feature` projects on the boundary (no `solid` flag, as upstream): a
//!   point inside a collider is at the distance to its boundary.

use fixed::Fixed;
use glam::vec2::Vec2;
use rapier_core::Handle;
use rapier_dynamics2d::collider::ColliderTrait;
use rapier_geometry2d::aabb::{Aabb, AabbTrait};
use rapier_geometry2d::feature_id::FeatureId;
use rapier_geometry2d::point::{PointProjection, PointQuery};
use rapier_geometry2d::query::intersection_test;
use rapier_geometry2d::ray::{Ray, RayIntersection};
use rapier_geometry2d::shape::{Shape, ShapeTrait};
use rapier_math::math_ext::norm2::{norm2_sq_wide, sq_wide};
use rapier_math::pose2::Pose2;
use crate::world::World;
use super::{QueryFilter, candidates};

/// Every collider whose shape intersects `shape` placed at `shape_pos` (touching included),
/// ascending handle order (upstream `QueryPipeline::intersect_shape`).
///
/// # Panics
/// * `shape_pos.rotation` must be a unit rotation (`Pose2::inv_mul`), and the exact kernels
///   panic as `rapier_geometry2d::dispatch::intersection_test`.
pub fn intersect_shape(
    ref world: World, shape_pos: Pose2, shape: Shape, filter: QueryFilter,
) -> Array<Handle> {
    let shape_aabb = shape.compute_aabb(shape_pos);
    let mut out = array![];
    for (handle, collider) in candidates(ref world, filter) {
        if collider.compute_aabb().intersects(shape_aabb)
            && intersection_test(
                shape_pos, shape, collider.position(), collider.shape,
            ) == Some(true) {
            out.append(handle);
        }
    }
    out
}

/// The collider whose boundary is closest to `point`, strictly closer than `max_dist`, with the
/// projection in world space and the feature it lands on (upstream
/// `QueryPipeline::project_point_and_get_feature`). Ties go to the lowest handle; distances are
/// compared squared and wide, as `project_point`.
pub fn project_point_and_get_feature(
    ref world: World, point: Vec2, max_dist: Fixed, filter: QueryFilter,
) -> Option<(Handle, PointProjection, FeatureId)> {
    let mut best: Option<(Handle, PointProjection, FeatureId)> = None;
    let mut bound = sq_wide(max_dist);
    for (handle, collider) in candidates(ref world, filter) {
        let (proj, feature) = collider
            .shape
            .project_point_and_get_feature(collider.position(), point);
        let dist = norm2_sq_wide(proj.point.x - point.x, proj.point.y - point.y);
        if dist < bound {
            bound = dist;
            best = Some((handle, proj, feature));
        }
    }
    best
}

/// A scene-query view: the filter every query called through it applies (upstream
/// `QueryPipeline`, whose `with_filter` replaces it). See the module docs for what it does not
/// bundle.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct QueryPipeline {
    pub filter: QueryFilter,
}

#[generate_trait]
pub impl QueryPipelineImpl of QueryPipelineTrait {
    /// A view that filters nothing.
    #[inline(always)]
    fn new() -> QueryPipeline {
        Default::default()
    }

    /// The same view with `filter` (upstream `with_filter`).
    #[inline(always)]
    fn with_filter(self: QueryPipeline, filter: QueryFilter) -> QueryPipeline {
        QueryPipeline { filter }
    }

    /// `queries::cast_ray` with the view's filter.
    #[inline(always)]
    fn cast_ray(
        self: QueryPipeline, ref world: World, ray: Ray, max_toi: Fixed, solid: bool,
    ) -> Option<(Handle, Fixed)> {
        super::cast_ray(ref world, ray, max_toi, solid, self.filter)
    }

    /// `queries::cast_ray_and_get_normal` with the view's filter.
    #[inline(always)]
    fn cast_ray_and_get_normal(
        self: QueryPipeline, ref world: World, ray: Ray, max_toi: Fixed, solid: bool,
    ) -> Option<(Handle, RayIntersection)> {
        super::cast_ray_and_get_normal(ref world, ray, max_toi, solid, self.filter)
    }

    /// `queries::intersect_ray` with the view's filter.
    #[inline(always)]
    fn intersect_ray(
        self: QueryPipeline, ref world: World, ray: Ray, max_toi: Fixed, solid: bool,
    ) -> Array<(Handle, RayIntersection)> {
        super::intersect_ray(ref world, ray, max_toi, solid, self.filter)
    }

    /// `queries::project_point` with the view's filter.
    #[inline(always)]
    fn project_point(
        self: QueryPipeline, ref world: World, point: Vec2, max_dist: Fixed, solid: bool,
    ) -> Option<(Handle, PointProjection)> {
        super::project_point(ref world, point, max_dist, solid, self.filter)
    }

    /// [`project_point_and_get_feature`] with the view's filter.
    #[inline(always)]
    fn project_point_and_get_feature(
        self: QueryPipeline, ref world: World, point: Vec2, max_dist: Fixed,
    ) -> Option<(Handle, PointProjection, FeatureId)> {
        project_point_and_get_feature(ref world, point, max_dist, self.filter)
    }

    /// `queries::intersect_point` with the view's filter.
    #[inline(always)]
    fn intersect_point(self: QueryPipeline, ref world: World, point: Vec2) -> Array<Handle> {
        super::intersect_point(ref world, point, self.filter)
    }

    /// `queries::intersect_aabb_conservative` with the view's filter.
    #[inline(always)]
    fn intersect_aabb_conservative(
        self: QueryPipeline, ref world: World, aabb: Aabb,
    ) -> Array<Handle> {
        super::intersect_aabb_conservative(ref world, aabb, self.filter)
    }

    /// [`intersect_shape`] with the view's filter.
    #[inline(always)]
    fn intersect_shape(
        self: QueryPipeline, ref world: World, shape_pos: Pose2, shape: Shape,
    ) -> Array<Handle> {
        intersect_shape(ref world, shape_pos, shape, self.filter)
    }
}
