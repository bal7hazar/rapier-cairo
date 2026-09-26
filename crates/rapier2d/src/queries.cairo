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
//! * [`QueryFilter`] is upstream's, minus the predicate closure (`dyn hooks`);
//! * `EXCLUDE_SOLIDS` (QY2) costs about 2.4k gas per candidate collider in the shared filter test
//!   (`cast_ray` on 32 balls: 6.13M before, 6.21M after). `QueryFilterFlagsTrait::test` is
//!   `#[inline(always)]` into `QueryFilterTrait::test`: as an outlined call it cost 6.41M
//!   (about 8.8k per candidate);
//! * every array query returns handles (or `(handle, hit)` pairs) in ascending handle order.
//!
//! QY2 adds the rest of the world-level surface (`pipeline`): [`intersect_shape`],
//! [`project_point_and_get_feature`], [`QueryPipeline`] (a filter bundle, upstream's `with_filter`
//! chain) and the `QueryFilterFlags` type.
//!
//! Deviations: [`intersect_aabb`] (and its upstream name [`intersect_aabb_conservative`]) tests
//! the collider's tight world AABB, where upstream tests the (possibly enlarged) AABB stored in
//! its BVH: the answer is a subset of upstream's, and never misses a real overlap.

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

/// `QueryPipeline` view, `intersect_shape` and `project_point_and_get_feature`.
pub mod pipeline;
pub use pipeline::{
    QueryPipeline, QueryPipelineTrait, intersect_shape, project_point_and_get_feature,
};

#[cfg(test)]
mod alternatives;
#[cfg(test)]
mod benches;
#[cfg(test)]
mod benches_qy2;

/// Excludes colliders without a parent or attached to a fixed body.
pub const EXCLUDE_FIXED: QueryFilterFlags = QueryFilterFlags { bits: 1 };
/// Excludes colliders attached to a kinematic body.
pub const EXCLUDE_KINEMATIC: QueryFilterFlags = QueryFilterFlags { bits: 2 };
/// Excludes colliders attached to a dynamic body.
pub const EXCLUDE_DYNAMIC: QueryFilterFlags = QueryFilterFlags { bits: 4 };
/// Excludes sensors.
pub const EXCLUDE_SENSORS: QueryFilterFlags = QueryFilterFlags { bits: 8 };
/// Excludes solid colliders (only sensors are hit).
pub const EXCLUDE_SOLIDS: QueryFilterFlags = QueryFilterFlags { bits: 16 };
/// `EXCLUDE_FIXED | EXCLUDE_KINEMATIC`.
pub const ONLY_DYNAMIC: QueryFilterFlags = QueryFilterFlags { bits: 3 };
/// `EXCLUDE_DYNAMIC | EXCLUDE_FIXED`.
pub const ONLY_KINEMATIC: QueryFilterFlags = QueryFilterFlags { bits: 5 };
/// `EXCLUDE_DYNAMIC | EXCLUDE_KINEMATIC`.
pub const ONLY_FIXED: QueryFilterFlags = QueryFilterFlags { bits: 6 };

/// The bit set of the `EXCLUDE_*` / `ONLY_*` constants (upstream `QueryFilterFlags`, a `bitflags`
/// struct over `u32` with the same bit values; `bits` is upstream's `bits()`).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct QueryFilterFlags {
    pub bits: u32,
}

#[generate_trait]
pub impl QueryFilterFlagsImpl of QueryFilterFlagsTrait {
    /// The flags with the given raw bits (upstream `from_bits_retain`).
    #[inline(always)]
    fn from_bits(bits: u32) -> QueryFilterFlags {
        QueryFilterFlags { bits }
    }

    /// No flag set.
    #[inline(always)]
    fn is_empty(self: QueryFilterFlags) -> bool {
        self.bits == 0
    }

    /// Every bit of `other` is set in `self`.
    fn contains(self: QueryFilterFlags, other: QueryFilterFlags) -> bool {
        self.bits & other.bits == other.bits
    }

    /// The union of both sets.
    #[inline(always)]
    fn union(self: QueryFilterFlags, other: QueryFilterFlags) -> QueryFilterFlags {
        QueryFilterFlags { bits: self.bits | other.bits }
    }

    /// Whether `collider` passes these flags (upstream `QueryFilterFlags::test`). Reads the
    /// parent body only when a body-type flag is set; a collider whose parent no longer exists
    /// passes.
    #[inline(always)]
    fn test(self: QueryFilterFlags, ref bodies: RigidBodySet, collider: Collider) -> bool {
        let flags = self.bits;
        if flags == 0 {
            return true;
        }
        if (has(flags, EXCLUDE_SENSORS.bits) && collider.is_sensor())
            || (has(flags, EXCLUDE_SOLIDS.bits) && !collider.is_sensor()) {
            return false;
        }
        match collider.parent() {
            None => !has(flags, EXCLUDE_FIXED.bits),
            Some(parent) => match bodies.get(parent) {
                Some(body) => !((has(flags, EXCLUDE_FIXED.bits) && body.is_fixed())
                    || (has(flags, EXCLUDE_KINEMATIC.bits) && body.is_kinematic())
                    || (has(flags, EXCLUDE_DYNAMIC.bits) && body.is_dynamic())),
                None => true,
            },
        }
    }
}

/// Which colliders a scene query considers (upstream `QueryFilter`, without the predicate
/// closure: excluded as `dyn hooks`).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct QueryFilter {
    /// A union of the `EXCLUDE_*` / `ONLY_*` constants of this module.
    pub flags: QueryFilterFlags,
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
    fn from_flags(flags: QueryFilterFlags) -> QueryFilter {
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
        if !has(f.flags.bits, EXCLUDE_SENSORS.bits) {
            f.flags.bits += EXCLUDE_SENSORS.bits;
        }
        f
    }

    /// Adds `EXCLUDE_SOLIDS`: only sensors are considered.
    #[inline(always)]
    fn exclude_solids(self: QueryFilter) -> QueryFilter {
        let mut f = self;
        if !has(f.flags.bits, EXCLUDE_SOLIDS.bits) {
            f.flags.bits += EXCLUDE_SOLIDS.bits;
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

    /// Returns `true` when the collider `handle` passes the filter (upstream `QueryFilter::test`,
    /// without the predicate): not the excluded collider, not attached to the excluded body,
    /// groups compatible, then `QueryFilterFlagsTrait::test`.
    fn test(
        self: QueryFilter, ref bodies: RigidBodySet, handle: Handle, collider: Collider,
    ) -> bool {
        if self.exclude_collider == Some(handle) {
            return false;
        }
        if let Some(body) = self.exclude_rigid_body {
            if collider.parent() == Some(body) {
                return false;
            }
        }
        if let Some(groups) = self.groups {
            if !collider.collision_groups().test(groups) {
                return false;
            }
        }
        self.flags.test(ref bodies, collider)
    }
}

/// `QueryFilter::from(QueryFilterFlags)`: the flags alone.
pub impl QueryFilterFlagsIntoQueryFilter of Into<QueryFilterFlags, QueryFilter> {
    #[inline(always)]
    fn into(self: QueryFilterFlags) -> QueryFilter {
        QueryFilterTrait::from_flags(self)
    }
}

/// `QueryFilter::from(InteractionGroups)`: the groups alone.
pub impl InteractionGroupsIntoQueryFilter of Into<InteractionGroups, QueryFilter> {
    #[inline(always)]
    fn into(self: InteractionGroups) -> QueryFilter {
        QueryFilterTrait::new().groups(self)
    }
}

/// `flags` contains `bit` (a single power of two): integer arithmetic, no bitwise builtin.
#[inline(always)]
pub(crate) fn has(flags: u32, bit: u32) -> bool {
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
            s.unbox(), pt, solid,
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
            s.unbox(), pt,
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
pub fn intersect_aabb_conservative(
    ref world: World, aabb: Aabb, filter: QueryFilter,
) -> Array<Handle> {
    intersect_aabb(ref world, aabb, filter)
}

/// Every collider whose world AABB intersects `aabb` (closed), ascending handle order.
pub fn intersect_aabb(ref world: World, aabb: Aabb, filter: QueryFilter) -> Array<Handle> {
    let mut out = array![];
    for (handle, collider) in candidates(ref world, filter) {
        if collider.compute_aabb().intersects(aabb) {
            out.append(handle);
        }
    }
    out
}
