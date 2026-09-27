//! The cast-and-slide loop of `move_shape` and its geometric steps: hit classification, slopes,
//! stairs, the snap to the ground (upstream's private methods of the same names).

use fixed::{Fixed, FixedTrait, ZERO};
use glam::vec2::{Vec2, Vec2Trait};
use rapier_dynamics2d::collider::ColliderTrait;
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBodySetTrait, RigidBodyTrait};
use rapier_geometry2d::query::{ShapeCastHit, ShapeCastOptions};
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::Pose2;
use crate::queries::pipeline::{QueryPipeline, QueryPipelineTrait};
use crate::queries::{EXCLUDE_DYNAMIC, QueryFilterFlagsTrait};
use crate::world::World;
use super::contacts::{check_and_fix_penetrations, detect_grounded, detect_grounded_with_friction};
use super::{
    CharacterCollision, CharacterLengthTrait, EPS_LENGTH, EffectiveCharacterMovement,
    HitDecomposition, HitDecompositionTrait, HitInfo, KinematicCharacterController, MAX_ITERATIONS,
    SURFACE_CORRECTION, compute_dims, shifted, try_normalize_and_get_length,
};

/// The options of every cast of the controller.
#[inline(always)]
fn cast_options(offset: Fixed, max_toi: Fixed) -> ShapeCastOptions {
    ShapeCastOptions {
        max_time_of_impact: max_toi,
        target_distance: offset,
        stop_at_penetration: false,
        compute_impact_geometry_on_penetration: true,
    }
}

/// Upstream `move_shape` (see `KinematicCharacterControllerTrait::move_shape`).
pub(crate) fn move_shape(
    controller: KinematicCharacterController,
    dt: Fixed,
    ref world: World,
    queries: QueryPipeline,
    shape: Shape,
    character_pos: Pose2,
    desired_translation: Vec2,
    ref events: Array<CharacterCollision>,
) -> EffectiveCharacterMovement {
    let zero = Vec2 { x: ZERO, y: ZERO };
    let mut result = EffectiveCharacterMovement {
        translation: zero, grounded: false, is_sliding_down_slope: false,
    };
    let dims = compute_dims(controller.up, shape);

    // 1. Depenetrate only when there is no desired movement (the casts handle it otherwise).
    if try_normalize_and_get_length(desired_translation, EPS_LENGTH).is_none() {
        check_and_fix_penetrations(
            controller, ref world, queries, shape, character_pos, dims, ref result,
        );
    }

    let mut translation_remaining = desired_translation;
    let grounded_at_starting_pos = detect_grounded(
        controller, ref world, queries, shape, shifted(character_pos, result.translation), dims,
    );

    let mut max_iters = MAX_ITERATIONS;
    let mut kinematic_friction_translation = zero;
    let offset = controller.offset.eval(dims.y);
    let mut is_moving = false;

    while let Some((dir, dist)) = try_normalize_and_get_length(translation_remaining, EPS_LENGTH) {
        if max_iters == 0 {
            break;
        }
        max_iters -= 1;
        is_moving = true;

        // 2. Cast towards the movement direction.
        let pos = shifted(character_pos, result.translation);
        match queries.cast_shape(ref world, pos, dir, shape, cast_options(offset, dist)) {
            Option::Some((
                handle, hit,
            )) => {
                let allowed = dir.mul_scalar(hit.time_of_impact);
                result.translation += allowed;
                translation_remaining -= allowed;
                let pos = shifted(character_pos, result.translation);
                events
                    .append(
                        CharacterCollision {
                            handle,
                            character_pos: pos,
                            translation_applied: result.translation,
                            translation_remaining,
                            hit,
                        },
                    );
                let hit_info = compute_hit_info(controller, hit);
                if !handle_stairs(
                    controller,
                    ref world,
                    queries,
                    shape,
                    pos,
                    dims,
                    handle,
                    hit_info,
                    ref translation_remaining,
                    ref result,
                ) {
                    translation_remaining =
                        handle_slopes(
                            controller,
                            hit_info,
                            desired_translation,
                            translation_remaining,
                            controller.normal_nudge_factor,
                            ref result,
                        );
                }
            },
            Option::None => {
                // No interference along the path.
                result.translation += translation_remaining;
                result
                    .grounded =
                        detect_grounded(
                            controller,
                            ref world,
                            queries,
                            shape,
                            shifted(character_pos, result.translation),
                            dims,
                        );
                break;
            },
        }
        result
            .grounded =
                detect_grounded_with_friction(
                    controller,
                    dt,
                    ref world,
                    queries,
                    shape,
                    shifted(character_pos, result.translation),
                    dims,
                    ref kinematic_friction_translation,
                    ref translation_remaining,
                );
        if !controller.slide {
            break;
        }
    }
    if !is_moving {
        result
            .grounded =
                detect_grounded(
                    controller,
                    ref world,
                    queries,
                    shape,
                    shifted(character_pos, result.translation),
                    dims,
                );
    }
    if grounded_at_starting_pos {
        let _ = snap_to_ground(
            controller,
            ref world,
            queries,
            shape,
            shifted(character_pos, result.translation),
            dims,
            ref result,
        );
    }
    result
}

/// Upstream `snap_to_ground`: when the movement does not go up, cast down by the snap distance and
/// apply the hit. Returns the hit.
fn snap_to_ground(
    controller: KinematicCharacterController,
    ref world: World,
    queries: QueryPipeline,
    shape: Shape,
    character_pos: Pose2,
    dims: Vec2,
    ref result: EffectiveCharacterMovement,
) -> Option<(rapier_core::Handle, ShapeCastHit)> {
    let snap = controller.snap_to_ground?;
    if result.translation.dot(controller.up) > ZERO {
        return None;
    }
    let snap_distance = snap.eval(dims.y);
    let offset = controller.offset.eval(dims.y);
    let (handle, hit) = queries
        .cast_shape(
            ref world, character_pos, -controller.up, shape, cast_options(offset, snap_distance),
        )?;
    result.translation -= controller.up.mul_scalar(hit.time_of_impact);
    result.grounded = true;
    Some((handle, hit))
}

/// `(vertical, horizontal)` parts of `translation` along `up` (upstream `split_into_components`).
#[inline(always)]
pub(crate) fn split_into_components(up: Vec2, translation: Vec2) -> (Vec2, Vec2) {
    let vertical = up.mul_scalar(up.dot(translation));
    (vertical, translation - vertical)
}

/// Upstream `compute_hit_info`: the floor angle is glam's signed `up.angle_to(normal1)`.
pub(crate) fn compute_hit_info(
    controller: KinematicCharacterController, toi: ShapeCastHit,
) -> HitInfo {
    let angle_with_floor = controller.up.angle_to(toi.normal1);
    let is_ceiling = controller.up.dot(toi.normal1) < ZERO;
    let is_wall = angle_with_floor >= controller.max_slope_climb_angle && !is_ceiling;
    let is_nonslip_slope = angle_with_floor <= controller.min_slope_slide_angle;
    HitInfo { toi, is_wall, is_nonslip_slope }
}

/// Upstream `decompose_hit`, 2D (no horizontal tangent).
pub(crate) fn decompose_hit(translation: Vec2, hit: ShapeCastHit) -> HitDecomposition {
    let zero = Vec2 { x: ZERO, y: ZERO };
    let dist_to_surface = translation.dot(hit.normal1);
    let part = hit.normal1.mul_scalar(dist_to_surface);
    // The penetration part (moving into the surface) or the normal part (moving away).
    let (normal_part, penetration_part) = if dist_to_surface < ZERO {
        (zero, part)
    } else {
        (part, zero)
    };
    HitDecomposition {
        normal_part,
        horizontal_tangent: zero,
        vertical_tangent: translation - normal_part - penetration_part,
    }
}

/// Upstream `handle_slopes`: the part of `translation_remaining` allowed after `hit`, nudged
/// along the normal.
pub(crate) fn handle_slopes(
    controller: KinematicCharacterController,
    hit: HitInfo,
    movement_input: Vec2,
    translation_remaining: Vec2,
    normal_nudge_factor: Fixed,
    ref result: EffectiveCharacterMovement,
) -> Vec2 {
    let up = controller.up;
    let (vertical_input, horizontal_input) = split_into_components(up, movement_input);
    let horiz_input_decomp = decompose_hit(horizontal_input, hit.toi);
    let decomp = decompose_hit(translation_remaining, hit.toi);

    let slipping_intent = up.dot(horiz_input_decomp.vertical_tangent) < ZERO;
    let slipping = up.dot(decomp.vertical_tangent) < ZERO;
    let climbing_intent = up.dot(vertical_input) > ZERO;
    let climbing = up.dot(decomp.vertical_tangent) > ZERO;

    let allowed = if hit.is_wall && climbing && !climbing_intent {
        // Cannot climb: drop the vertical tangent motion induced by the forward motion.
        decomp.horizontal_tangent + decomp.normal_part
    } else if hit.is_nonslip_slope && slipping && !slipping_intent {
        // Do not slide down.
        decomp.horizontal_tangent + decomp.normal_part
    } else {
        result.is_sliding_down_slope = true;
        decomp.unconstrained_slide_part()
    };
    allowed + hit.toi.normal1.mul_scalar(normal_nudge_factor)
}

/// Upstream `subtract_hit`: `translation` with its part into the surface removed (slightly
/// over-corrected).
pub(crate) fn subtract_hit(translation: Vec2, hit: ShapeCastHit) -> Vec2 {
    let correction = (-translation).dot(hit.normal1).max(ZERO) * SURFACE_CORRECTION;
    translation + hit.normal1.mul_scalar(correction)
}

/// Upstream `handle_stairs`: climbs the step `hit` when it is a wall, there is room above and a
/// landing, and the landing is not too steep. Returns whether it stepped (then `result` and
/// `translation_remaining` carry the step). The landing cast is run once (upstream runs the same
/// cast twice, for the slope check and for the step height: same answer).
fn handle_stairs(
    controller: KinematicCharacterController,
    ref world: World,
    queries: QueryPipeline,
    shape: Shape,
    character_pos: Pose2,
    dims: Vec2,
    stair_handle: rapier_core::Handle,
    hit: HitInfo,
    ref translation_remaining: Vec2,
    ref result: EffectiveCharacterMovement,
) -> bool {
    let Some(autostep) = controller.autostep else {
        return false;
    };
    // Only try to autostep on walls.
    if !hit.is_wall {
        return false;
    }
    let up = controller.up;
    let offset = controller.offset.eval(dims.y);
    let min_width = autostep.min_width.eval(dims.x) + offset;
    let max_height = autostep.max_height.eval(dims.y) + offset;

    let mut queries = queries;
    if !autostep.include_dynamic_bodies {
        if let Some(co) = world.colliders.get(stair_handle) {
            if let Some(parent) = co.parent() {
                if let Some(body) = world.bodies.get(parent) {
                    if body.is_dynamic() {
                        // The "stair" is a dynamic body, which the user wants to ignore.
                        return false;
                    }
                }
            }
        }
        queries.filter.flags = queries.filter.flags.union(EXCLUDE_DYNAMIC);
    }

    let shifted_pos = shifted(character_pos, up.mul_scalar(max_height));
    let remaining = translation_remaining;
    let Some(horizontal_dir) = (remaining - up.mul_scalar(remaining.dot(up))).try_normalize() else {
        return false;
    };

    // We cannot go up.
    if queries
        .cast_shape(ref world, character_pos, up, shape, cast_options(offset, max_height))
        .is_some() {
        return false;
    }
    // Not enough room on the stair to stay on it.
    if queries
        .cast_shape(ref world, shifted_pos, horizontal_dir, shape, cast_options(offset, min_width))
        .is_some() {
        return false;
    }
    // The landing: not a ramp too steep after stepping.
    let landing_pos = shifted(shifted_pos, horizontal_dir.mul_scalar(min_width));
    let landing = queries
        .cast_shape(ref world, landing_pos, -up, shape, cast_options(offset, max_height));
    let mut landing_toi = max_height;
    if let Some((_, landing_hit)) = landing {
        let (vertical, horizontal) = split_into_components(up, remaining);
        let slope_translation = subtract_hit(horizontal, landing_hit)
            + subtract_hit(vertical, landing_hit);
        let angle_with_floor = up.angle_to(landing_hit.normal1);
        let climbing = up.dot(slope_translation) >= ZERO;
        if climbing && angle_with_floor > controller.max_slope_climb_angle {
            return false; // The target ramp is too steep.
        }
        landing_toi = landing_hit.time_of_impact;
    }

    // The actual step height.
    let step = up.mul_scalar(max_height - landing_toi);
    translation_remaining -= step;
    // Advance on the step horizontally so that the next move does not stick on its edge.
    let nudge = horizontal_dir.mul_scalar(horizontal_dir.dot(translation_remaining).min(min_width));
    translation_remaining -= nudge;
    result.translation += step + nudge;
    true
}
