//! Rejected candidates of `super`, kept for the `gas_*` ranking of `super::tests`: the exact
//! casts on every candidate, without the box pre-tests.

use fixed::{Fixed, ZERO};
use glam::vec2::Vec2;
use rapier_core::Handle;
use rapier_dynamics2d::collider::ColliderTrait;
use rapier_geometry2d::query::{
    NonlinearRigidMotion, NonlinearRigidMotionTrait, ShapeCastHit, ShapeCastHitTrait,
    ShapeCastOptions, cast_shapes, cast_shapes_nonlinear,
};
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::Pose2;
use crate::world::World;
use super::super::{QueryFilter, candidates};

pub fn cast_shape_direct(
    ref world: World,
    shape_pos: Pose2,
    shape_vel: Vec2,
    shape: Shape,
    options: ShapeCastOptions,
    filter: QueryFilter,
) -> Option<(Handle, ShapeCastHit)> {
    let mut best: Option<(Handle, ShapeCastHit)> = None;
    let mut bound = options.max_time_of_impact;
    let zero = Vec2 { x: ZERO, y: ZERO };
    for (handle, collider) in candidates(ref world, filter) {
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

pub fn cast_shape_nonlinear_direct(
    ref world: World,
    shape_motion: NonlinearRigidMotion,
    shape: Shape,
    start_time: Fixed,
    end_time: Fixed,
    stop_at_penetration: bool,
    filter: QueryFilter,
) -> Option<(Handle, ShapeCastHit)> {
    let mut best: Option<(Handle, ShapeCastHit)> = None;
    let mut bound = end_time;
    for (handle, collider) in candidates(ref world, filter) {
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
