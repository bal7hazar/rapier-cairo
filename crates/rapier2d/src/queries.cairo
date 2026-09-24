//! Scene queries over a `World` (upstream `pipeline/query_pipeline.rs`, `QueryPipeline`), work
//! package QP.
//!
//! Upstream answers scene queries by traversing the broad phase's BVH. The port keeps no
//! broad-phase state between steps (`docs/PLAN.md` D7), so every query here is a **brute-force
//! scan** of the collider set in ascending slot order, which is also what makes the answers
//! deterministic: on equal times of impact (or distances) the collider met first — the lowest
//! handle — wins, and the array queries return their hits in ascending handle order.
//!
//! # Candidates
//!
//! Ranked by `queries::benches` (`cast_ray` on rows of balls, net of the scene setup); the
//! rejected ones live in `#[cfg(test)] mod alternatives`.
//!
//! 1. **brute force** (this module): one exact cast per candidate collider, bounded by the best
//!    time so far. **Winner** at 8, 32 and 128 colliders, in Sierra gas and in Cairo steps
//!    (128 colliders: 21.0M gas, 97.8k steps).
//! 2. `alternatives::cast_ray_aabb_pretest`: a ray-AABB slab test before each exact cast.
//!    +42 % gas / +1.5 % steps at 8, +39 % / +21 % at 128: the slab test of the box costs more
//!    than the exact cast of the simple shapes it would skip.
//! 3. `alternatives::cast_ray_grid`: BG's `find_pairs` with every collider static and the
//!    ray's AABB as the one dynamic proxy. +42 % gas and 2.5× the steps at 128.
//!
//! Semantics kept from upstream:
//!
//! * the closest-hit queries ([`cast_ray`], [`cast_ray_and_get_normal`], [`project_point`]) only
//!   accept a hit **strictly** below the bound (`max_toi`, `max_dist`), as upstream's
//!   `Bvh::find_best`; each shape is asked with the best time so far as its own bound;
//! * [`intersect_ray`] reports every collider hit within `max_toi` (inclusive, the per-shape
//!   bound);
//! * disabled colliders are never reported (upstream removes them from the BVH);
//! * [`QueryFilter`] is upstream's, minus `EXCLUDE_SOLIDS` and the predicate closure.
//!
//! Deviations: [`intersect_aabb`] tests the collider's tight world AABB, where upstream's
//! `intersect_aabb_conservative` tests the (possibly enlarged) AABB stored in its BVH.

use fixed::Fixed;
use glam::vec2::Vec2;
use rapier_core::Handle;
use rapier_core::interaction_groups::{InteractionGroups, InteractionGroupsTrait};
use rapier_dynamics2d::collider::{Collider, ColliderTrait};
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use rapier_geometry2d::aabb::{Aabb, AabbTrait};
use rapier_geometry2d::point::{
    PointProjection, contains_local_point_ball, contains_local_point_capsule,
    contains_local_point_cuboid, contains_local_point_halfspace, contains_local_point_segment,
    project_local_point_ball, project_local_point_capsule, project_local_point_cuboid,
    project_local_point_halfspace, project_local_point_segment,
};
use rapier_geometry2d::ray::{
    Ray, RayIntersection, cast_ray as shape_cast_ray, cast_ray_and_get_normal as shape_cast_hit,
};
use rapier_geometry2d::shape::Shape;
use rapier_math::math_ext::norm2::{norm2_sq_wide, sq_wide};
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::world::World;

#[cfg(test)]
mod alternatives;
#[cfg(test)]
mod benches;

/// Excludes colliders without a parent or attached to a fixed body.
pub const EXCLUDE_FIXED: u32 = 1;
/// Excludes colliders attached to a kinematic body.
pub const EXCLUDE_KINEMATIC: u32 = 2;
/// Excludes colliders attached to a dynamic body.
pub const EXCLUDE_DYNAMIC: u32 = 4;
/// Excludes sensors.
pub const EXCLUDE_SENSORS: u32 = 8;
/// `EXCLUDE_FIXED | EXCLUDE_KINEMATIC`.
pub const ONLY_DYNAMIC: u32 = 3;
/// `EXCLUDE_DYNAMIC | EXCLUDE_FIXED`.
pub const ONLY_KINEMATIC: u32 = 5;
/// `EXCLUDE_DYNAMIC | EXCLUDE_KINEMATIC`.
pub const ONLY_FIXED: u32 = 6;

/// Which colliders a scene query considers (upstream `QueryFilter`, without the predicate
/// closure and `EXCLUDE_SOLIDS`).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct QueryFilter {
    /// A union of the `EXCLUDE_*` / `ONLY_*` constants of this module.
    pub flags: u32,
    /// When set, only colliders whose collision groups pass `InteractionGroups::test` with it.
    pub groups: Option<InteractionGroups>,
    pub exclude_collider: Option<Handle>,
    pub exclude_rigid_body: Option<Handle>,
}

#[generate_trait]
pub impl QueryFilterImpl of QueryFilterTrait {
    /// No filtering.
    #[inline(always)]
    fn new() -> QueryFilter {
        Default::default()
    }

    /// Filter with the given flags only.
    #[inline(always)]
    fn from_flags(flags: u32) -> QueryFilter {
        QueryFilter { flags, groups: None, exclude_collider: None, exclude_rigid_body: None }
    }

    #[inline(always)]
    fn exclude_fixed() -> QueryFilter {
        Self::from_flags(EXCLUDE_FIXED)
    }

    #[inline(always)]
    fn exclude_kinematic() -> QueryFilter {
        Self::from_flags(EXCLUDE_KINEMATIC)
    }

    #[inline(always)]
    fn exclude_dynamic() -> QueryFilter {
        Self::from_flags(EXCLUDE_DYNAMIC)
    }

    #[inline(always)]
    fn only_dynamic() -> QueryFilter {
        Self::from_flags(ONLY_DYNAMIC)
    }

    #[inline(always)]
    fn only_kinematic() -> QueryFilter {
        Self::from_flags(ONLY_KINEMATIC)
    }

    #[inline(always)]
    fn only_fixed() -> QueryFilter {
        Self::from_flags(ONLY_FIXED)
    }

    /// Adds `EXCLUDE_SENSORS`.
    #[inline(always)]
    fn exclude_sensors(self: QueryFilter) -> QueryFilter {
        let mut f = self;
        if !has(f.flags, EXCLUDE_SENSORS) {
            f.flags += EXCLUDE_SENSORS;
        }
        f
    }

    /// Only colliders whose collision groups pass `test` against `groups`.
    #[inline(always)]
    fn groups(self: QueryFilter, groups: InteractionGroups) -> QueryFilter {
        let mut f = self;
        f.groups = Some(groups);
        f
    }

    #[inline(always)]
    fn exclude_collider(self: QueryFilter, collider: Handle) -> QueryFilter {
        let mut f = self;
        f.exclude_collider = Some(collider);
        f
    }

    #[inline(always)]
    fn exclude_rigid_body(self: QueryFilter, body: Handle) -> QueryFilter {
        let mut f = self;
        f.exclude_rigid_body = Some(body);
        f
    }

    /// Returns `true` when the collider `handle` passes the filter (upstream `QueryFilter::test`
    /// and `QueryFilterFlags::test`). Reads the parent body only when a body-type flag is set.
    fn test(
        self: QueryFilter, ref bodies: RigidBodySet, handle: Handle, collider: Collider,
    ) -> bool {
        if self.exclude_collider == Some(handle) {
            return false;
        }
        let parent = collider.parent();
        if let Some(body) = self.exclude_rigid_body {
            if parent == Some(body) {
                return false;
            }
        }
        if let Some(groups) = self.groups {
            if !collider.collision_groups().test(groups) {
                return false;
            }
        }
        if self.flags == 0 {
            return true;
        }
        if has(self.flags, EXCLUDE_SENSORS) && collider.is_sensor() {
            return false;
        }
        match parent {
            None => !has(self.flags, EXCLUDE_FIXED),
            Some(parent) => match bodies.get(parent) {
                Some(body) => !((has(self.flags, EXCLUDE_FIXED) && body.is_fixed())
                    || (has(self.flags, EXCLUDE_KINEMATIC) && body.is_kinematic())
                    || (has(self.flags, EXCLUDE_DYNAMIC) && body.is_dynamic())),
                None => true,
            },
        }
    }
}

/// `flags` contains `bit` (a single power of two): integer arithmetic, no bitwise builtin.
#[inline(always)]
fn has(flags: u32, bit: u32) -> bool {
    (flags / bit) % 2 == 1
}

/// Every enabled collider that passes `filter`, in ascending handle order.
pub(crate) fn candidates(ref world: World, filter: QueryFilter) -> Array<(Handle, Collider)> {
    let mut out = array![];
    for (handle, collider) in world.colliders.iter() {
        if collider.is_enabled() && filter.test(ref world.bodies, handle, collider) {
            out.append((handle, collider));
        }
    }
    out
}

/// The collider hit first by `ray` and its time of impact, strictly below `max_toi` (upstream
/// `QueryPipeline::cast_ray`). Ties go to the lowest handle.
pub fn cast_ray(
    ref world: World, ray: Ray, max_toi: Fixed, solid: bool, filter: QueryFilter,
) -> Option<(Handle, Fixed)> {
    let mut best: Option<(Handle, Fixed)> = None;
    let mut bound = max_toi;
    for (handle, collider) in candidates(ref world, filter) {
        if let Some(t) = shape_cast_ray(collider.shape, collider.position(), ray, bound, solid) {
            if t < bound {
                bound = t;
                best = Some((handle, t));
            }
        }
    }
    best
}

/// The collider hit first by `ray`, with the world-space normal and feature (upstream
/// `QueryPipeline::cast_ray_and_get_normal`). Same bound and tie rules as [`cast_ray`].
pub fn cast_ray_and_get_normal(
    ref world: World, ray: Ray, max_toi: Fixed, solid: bool, filter: QueryFilter,
) -> Option<(Handle, RayIntersection)> {
    let mut best: Option<(Handle, RayIntersection)> = None;
    let mut bound = max_toi;
    for (handle, collider) in candidates(ref world, filter) {
        if let Some(hit) = shape_cast_hit(collider.shape, collider.position(), ray, bound, solid) {
            if hit.time_of_impact < bound {
                bound = hit.time_of_impact;
                best = Some((handle, hit));
            }
        }
    }
    best
}

/// Every collider hit by `ray` within `max_toi` (inclusive), ascending handle order (upstream
/// `QueryPipeline::intersect_ray`, whose order is the BVH's).
pub fn intersect_ray(
    ref world: World, ray: Ray, max_toi: Fixed, solid: bool, filter: QueryFilter,
) -> Array<(Handle, RayIntersection)> {
    let mut hits = array![];
    for (handle, collider) in candidates(ref world, filter) {
        if let Some(hit) =
            shape_cast_hit(collider.shape, collider.position(), ray, max_toi, solid) {
            hits.append((handle, hit));
        }
    }
    hits
}

/// Local projection on any shape of the closed set.
#[inline(always)]
fn project_local_point(shape: Shape, pt: Vec2, solid: bool) -> PointProjection {
    match shape {
        Shape::Ball(s) => project_local_point_ball(s, pt, solid),
        Shape::Cuboid(s) => project_local_point_cuboid(s, pt, solid),
        Shape::Capsule(s) => project_local_point_capsule(s, pt, solid),
        Shape::Segment(s) => project_local_point_segment(s, pt, solid),
        Shape::HalfSpace(s) => project_local_point_halfspace(s, pt, solid),
        Shape::ConvexPolygon(s) => rapier_geometry2d::point::convex_polygon::project_local_point_convex_polygon(
            s, pt, solid,
        ),
    }
}

/// Local containment on any shape of the closed set.
#[inline(always)]
fn contains_local_point(shape: Shape, pt: Vec2) -> bool {
    match shape {
        Shape::Ball(s) => contains_local_point_ball(s, pt),
        Shape::Cuboid(s) => contains_local_point_cuboid(s, pt),
        Shape::Capsule(s) => contains_local_point_capsule(s, pt),
        Shape::Segment(s) => contains_local_point_segment(s, pt),
        Shape::HalfSpace(s) => contains_local_point_halfspace(s, pt),
        Shape::ConvexPolygon(s) => rapier_geometry2d::point::convex_polygon::contains_local_point_convex_polygon(
            s, pt,
        ),
    }
}

/// Projection of the world point `point` on `shape` placed at `pose` (upstream
/// `PointQuery::project_point`).
#[inline(always)]
fn project_point_on(shape: Shape, pose: Pose2, point: Vec2, solid: bool) -> PointProjection {
    let local = project_local_point(shape, pose.inverse_transform_point(point), solid);
    PointProjection { is_inside: local.is_inside, point: pose.transform_point(local.point) }
}

/// The collider closest to `point`, strictly closer than `max_dist`, with the projection in
/// world space (upstream `QueryPipeline::project_point`). A `solid` query from inside a
/// collider projects to the point itself (distance 0). Ties go to the lowest handle.
///
/// Distances are compared squared and wide (exact), not as rounded lengths.
pub fn project_point(
    ref world: World, point: Vec2, max_dist: Fixed, solid: bool, filter: QueryFilter,
) -> Option<(Handle, PointProjection)> {
    let mut best: Option<(Handle, PointProjection)> = None;
    let mut bound = sq_wide(max_dist);
    for (handle, collider) in candidates(ref world, filter) {
        let proj = project_point_on(collider.shape, collider.position(), point, solid);
        let dist = norm2_sq_wide(proj.point.x - point.x, proj.point.y - point.y);
        if dist < bound {
            bound = dist;
            best = Some((handle, proj));
        }
    }
    best
}

/// Every collider containing `point` (boundary included), ascending handle order (upstream
/// `QueryPipeline::intersect_point`).
pub fn intersect_point(ref world: World, point: Vec2, filter: QueryFilter) -> Array<Handle> {
    let mut out = array![];
    for (handle, collider) in candidates(ref world, filter) {
        let local = collider.position().inverse_transform_point(point);
        if contains_local_point(collider.shape, local) {
            out.append(handle);
        }
    }
    out
}

/// Every collider whose world AABB intersects `aabb` (closed), ascending handle order
/// (upstream `QueryPipeline::intersect_aabb_conservative`; see the module deviations).
pub fn intersect_aabb(ref world: World, aabb: Aabb, filter: QueryFilter) -> Array<Handle> {
    let mut out = array![];
    for (handle, collider) in candidates(ref world, filter) {
        if collider.compute_aabb().intersects(aabb) {
            out.append(handle);
        }
    }
    out
}
