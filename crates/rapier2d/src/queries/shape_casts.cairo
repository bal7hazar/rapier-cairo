//! Shape casts against the scene (upstream `QueryPipeline::cast_shape` /
//! `cast_shape_nonlinear`, work package CC1), on the brute-force scan of `super`.
//!
//! Every enabled collider passing the filter is a candidate, in ascending handle order; the
//! collider is shape 1 of `rapier_geometry2d::query::cast_shapes` and the cast shape shape 2, as
//! upstream (`CompositeShapeRef(pipeline).cast_shape`): the hit's `witness1` / `normal1` are in
//! world space (on the collider), `witness2` / `normal2` in the local frame of the cast shape.
//! A hit is kept when its time is **strictly** below the best so far (initially
//! `max_time_of_impact`, resp. `end_time`), as upstream's `Bvh::find_best`: ties go to the lowest
//! handle. Each nonlinear cast runs on the whole `[start_time, end_time]`, as upstream (its
//! bisection depends on the interval); the linear one takes the best time as its bound (same
//! answers, earlier misses). A pair without a cast kernel (half-space against half-space) is
//! skipped, as upstream's `.ok()?`.
//!
//! # Candidates
//!
//! Ranked by the `gas_*` probes of `tests` on the 20-collider scene of `super::benches_qy2`
//! (upstream prunes with its BVH first), net of `gas_setup_20` (1.03M gas / 9,409 steps); the
//! rejected ones live in `alternatives`.
//!
//! | query | pre-test (shipped) | exact on every candidate |
//! |---|---:|---:|
//! | `cast_shape` (small box along a row) | 4.64M gas / 23,079 steps | 10.75M / 77,916 |
//! | `cast_shape_nonlinear` (turning bar) | 93.3M gas / 747,965 steps | 159.9M / 1,284,817 |
//!
//! * `cast_shape`: an AABB pre-test (upstream's Minkowski-sum test on the BVH nodes: the ray from
//!   the cast shape's box centre against the collider's world box grown by the cast shape's half
//!   extents and the target distance; skipped when either shape is a half-space, whose box spans
//!   the whole range), then the exact cast; against the exact cast on every candidate
//!   (`alternatives::cast_shape_direct`).
//! * `cast_shape_nonlinear`: a swept-capsule pre-test (the cast shape stays within its bounding
//!   radius about its rotation centre, which moves on a segment over `[start_time, end_time]`;
//!   the capsule's box against the collider's), then the exact cast; against the exact cast on
//!   every candidate (`alternatives::cast_shape_nonlinear_direct`).

use fixed::{Fixed, ZERO};
use glam_core::vec2::{Vec2, Vec2Trait};
use rapier_core::Handle;
use rapier_dynamics2d::collider::ColliderTrait;
use rapier_geometry2d::aabb::{Aabb, AabbTrait};
use rapier_geometry2d::query::{
    NonlinearRigidMotion, NonlinearRigidMotionTrait, ShapeCastHit, ShapeCastHitTrait,
    ShapeCastOptions, cast_shapes, cast_shapes_nonlinear,
};
use rapier_geometry2d::ray::{Ray, cast_local_ray_cuboid};
use rapier_geometry2d::shape::{Cuboid, Shape, ShapeTrait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::world::World;
use super::{QueryFilter, candidates};

#[cfg(test)]
mod alternatives;
#[cfg(test)]
mod tests;

/// `true` when `ray` meets `aabb` at a time `<= bound` (solid slab test on the box).
#[inline(always)]
pub(crate) fn ray_meets_aabb(aabb: Aabb, ray: Ray, bound: Fixed) -> bool {
    let local = Ray { origin: ray.origin - aabb.center(), dir: ray.dir };
    cast_local_ray_cuboid(Cuboid { half_extents: aabb.half_extents() }, local, bound, true)
        .is_some()
}

/// Whether `shape` is a half-space, whose box spans the whole range: no box pre-test then.
#[inline(always)]
fn unbounded(shape: Shape) -> bool {
    match shape {
        Shape::HalfSpace(_) => true,
        _ => false,
    }
}

/// The first collider hit by `shape`, placed at `shape_pos` and moving at `shape_vel` (world
/// space), with the hit (upstream `QueryPipeline::cast_shape`); see the module documentation for
/// the frames, the bound and the tie rule.
///
/// # Panics
/// * `shape_pos.rotation` and every collider's rotation must be unit (`Pose2::inv_mul`); the
///   panics of `rapier_geometry2d::query::cast_shapes`.
pub fn cast_shape(
    ref world: World,
    shape_pos: Pose2,
    shape_vel: Vec2,
    shape: Shape,
    options: ShapeCastOptions,
    filter: QueryFilter,
) -> Option<(Handle, ShapeCastHit)> {
    let shape_aabb = shape.compute_aabb(shape_pos);
    let margin = Vec2 { x: options.target_distance, y: options.target_distance };
    let grow = shape_aabb.half_extents() + margin;
    let ray = Ray { origin: shape_aabb.center(), dir: shape_vel };
    let pretest = !unbounded(shape);
    let mut best: Option<(Handle, ShapeCastHit)> = None;
    let mut bound = options.max_time_of_impact;
    let zero = Vec2 { x: ZERO, y: ZERO };
    for (handle, collider) in candidates(ref world, filter) {
        if pretest && !unbounded(collider.shape) {
            let aabb = collider.compute_aabb();
            let msum = Aabb { mins: aabb.mins - grow, maxs: aabb.maxs + grow };
            if !ray_meets_aabb(msum, ray, bound) {
                continue;
            }
        }
        let pos = collider.position();
        let opts = ShapeCastOptions { max_time_of_impact: bound, ..options };
        if let Some(Some(hit)) =
            cast_shapes(pos, zero, collider.shape, shape_pos, shape_vel, shape, opts) {
            if hit.time_of_impact < bound {
                bound = hit.time_of_impact;
                best = Some((handle, hit.transform1_by(pos)));
            }
        }
    }
    best
}

/// The box of the region swept over `[start_time, end_time]` by a shape of bounding sphere
/// `(center, radius)` (its local frame) following `motion`: the capsule of radius
/// `|center - local_center| + radius` around the segment the rotation centre travels.
/// #### Panics
/// * See `NonlinearRigidMotionTrait::position_at_time`.
pub(crate) fn swept_box(
    motion: NonlinearRigidMotion, center: Vec2, radius: Fixed, start_time: Fixed, end_time: Fixed,
) -> Aabb {
    let reach = (center - motion.local_center).length() + radius;
    let c0 = motion.start.transform_point(motion.local_center);
    let (lin, pad) = (motion.linvel, Vec2 { x: reach, y: reach });
    let a = c0 + Vec2 { x: lin.x * start_time, y: lin.y * start_time };
    let b = c0 + Vec2 { x: lin.x * end_time, y: lin.y * end_time };
    Aabb { mins: a.min(b) - pad, maxs: a.max(b) + pad }
}

/// The first collider hit by `shape` following `shape_motion` within `[start_time, end_time]`,
/// with the hit (upstream `QueryPipeline::cast_shape_nonlinear`): each collider stands still at
/// its position; `witness1` / `normal1` in world space, `witness2` / `normal2` in the cast
/// shape's frame at the time of impact. See `rapier_geometry2d::query::cast_shapes_nonlinear`
/// for `stop_at_penetration`.
///
/// # Panics
/// * See [`cast_shape`] and `rapier_geometry2d::query::cast_shapes_nonlinear`.
pub fn cast_shape_nonlinear(
    ref world: World,
    shape_motion: NonlinearRigidMotion,
    shape: Shape,
    start_time: Fixed,
    end_time: Fixed,
    stop_at_penetration: bool,
    filter: QueryFilter,
) -> Option<(Handle, ShapeCastHit)> {
    if unbounded(shape) {
        // No nonlinear kernel takes a half-space (upstream: every pair unsupported).
        return None;
    }
    let sphere = shape.compute_local_bounding_sphere();
    let swept = swept_box(shape_motion, sphere.center, sphere.radius, start_time, end_time);
    let mut best: Option<(Handle, ShapeCastHit)> = None;
    let mut bound = end_time;
    for (handle, collider) in candidates(ref world, filter) {
        if !collider.compute_aabb().intersects(swept) {
            continue;
        }
        let pos = collider.position();
        let motion1 = NonlinearRigidMotionTrait::constant_position(pos);
        if let Some(Some(hit)) =
            cast_shapes_nonlinear(
                motion1,
                collider.shape,
                shape_motion,
                shape,
                start_time,
                end_time,
                stop_at_penetration,
            ) {
            if hit.time_of_impact < bound {
                bound = hit.time_of_impact;
                best = Some((handle, hit.transform1_by(pos)));
            }
        }
    }
    best
}
