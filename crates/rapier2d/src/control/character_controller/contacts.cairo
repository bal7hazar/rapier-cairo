//! The contact-based passes of the controller (upstream's private methods of the same names):
//! depenetration, grounded status with the kinematic platforms' friction, and the impulses on the
//! dynamic bodies pushed by the character.

use fixed::{Fixed, FixedTrait, ONE, ZERO};
use glam::vec2::{Vec2, Vec2Trait};
use rapier_dynamics2d::collider::{Collider, ColliderTrait};
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySetTrait, RigidBodyTrait};
use rapier_geometry2d::aabb::{Aabb, AabbTrait};
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait};
use rapier_geometry2d::dispatch::composite::contact_manifolds_composite;
use rapier_geometry2d::dispatch::contact_manifold_step;
use rapier_geometry2d::query::dispatcher::contact;
use rapier_geometry2d::shape::{Shape, ShapeTrait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::queries::candidates;
use crate::queries::pipeline::QueryPipeline;
use crate::world::World;
use super::{
    CharacterCollision, CharacterLengthTrait, EffectiveCharacterMovement, GROUNDED_COS,
    KinematicCharacterController, MAX_CORRECTION, PENETRATION_EPS, predict_ground, shifted,
};

/// The depenetration passes of `check_and_fix_penetrations`.
const PENETRATION_PASSES: u32 = 4;

/// The colliders passing `queries`' filter whose world box meets `aabb`, ascending handle order
/// (upstream `intersect_aabb_conservative` on its BVH).
fn colliders_in(
    ref world: World, queries: QueryPipeline, aabb: Aabb,
) -> Array<(rapier_core::Handle, Collider)> {
    let mut out = array![];
    for (handle, collider) in candidates(ref world, queries.filter) {
        if collider.compute_aabb().intersects(aabb) {
            out.append((handle, collider));
        }
    }
    out
}

/// Parry `contact_manifolds` from scratch for the pair (`shape1` the character, `pos12` the pose
/// of `shape2` in its frame): one manifold for a convex pair, one per part for a composite one,
/// none for an unsupported pair. The convex pairs go through the step's plain table
/// (`dispatch::contact_manifold_step`): cheaper inside the moves than the metered one
/// (`alternatives::manifolds_metered`), see the module candidates.
pub(crate) fn manifolds(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed,
) -> Array<ContactManifold> {
    if shape1.is_composite() || shape2.is_composite() {
        return contact_manifolds_composite(pos12, shape1, shape2, prediction, [].span())
            .unwrap_or_default();
    }
    let mut manifold = ContactManifoldTrait::new();
    if contact_manifold_step(pos12, shape1, shape2, prediction, ref manifold) {
        array![manifold]
    } else {
        array![]
    }
}

/// Upstream `is_grounded_at_contact_manifold`.
fn is_grounded_at_contact_manifold(
    controller: KinematicCharacterController,
    manifold: @ContactManifold,
    character_pos: Pose2,
    prediction: Fixed,
) -> bool {
    let normal = -character_pos.rotation.rotate(*manifold.local_n1);
    if normal.dot(controller.up) >= GROUNDED_COS {
        for contact in manifold.contacts() {
            if *contact.dist <= prediction {
                return true;
            }
        }
    }
    false
}

/// Upstream `detect_grounded_status_and_apply_friction` without the friction outputs: stops at
/// the first grounding contact.
pub(crate) fn detect_grounded(
    controller: KinematicCharacterController,
    ref world: World,
    queries: QueryPipeline,
    shape: Shape,
    character_pos: Pose2,
    dims: Vec2,
) -> bool {
    let prediction = predict_ground(controller, dims.y);
    let aabb = shape.compute_aabb(character_pos).loosened(prediction);
    for (_, collider) in colliders_in(ref world, queries, aabb) {
        let pos12 = character_pos.inv_mul(collider.position());
        for m in manifolds(pos12, shape, collider.shape, prediction) {
            if is_grounded_at_contact_manifold(controller, @m, character_pos, prediction) {
                return true;
            }
        }
    }
    false
}

/// The parent of `collider` when it is a kinematic body.
fn kinematic_parent(ref world: World, collider: Collider) -> Option<RigidBody> {
    let body = world.bodies.get(collider.parent()?)?;
    if body.is_kinematic() {
        Some(body)
    } else {
        None
    }
}

/// Upstream `detect_grounded_status_and_apply_friction` with the friction outputs: every
/// collider is visited; the contacts with a kinematic body move `translation_remaining` with the
/// platform along the normal, and the largest tangent platform motion (per component) goes into
/// `kinematic_friction_translation` and, as its change, into `translation_remaining`.
pub(crate) fn detect_grounded_with_friction(
    controller: KinematicCharacterController,
    dt: Fixed,
    ref world: World,
    queries: QueryPipeline,
    shape: Shape,
    character_pos: Pose2,
    dims: Vec2,
    ref kinematic_friction_translation: Vec2,
    ref translation_remaining: Vec2,
) -> bool {
    let prediction = predict_ground(controller, dims.y);
    let aabb = shape.compute_aabb(character_pos).loosened(prediction);
    let mut grounded = false;
    for (_, collider) in colliders_in(ref world, queries, aabb) {
        let collider_pos = collider.position();
        let pos12 = character_pos.inv_mul(collider_pos);
        let all = manifolds(pos12, shape, collider.shape, prediction);
        let init = kinematic_friction_translation;
        let parent = kinematic_parent(ref world, collider);
        for m in all {
            if is_grounded_at_contact_manifold(controller, @m, character_pos, prediction) {
                grounded = true;
            }
            if let Some(platform) = @parent {
                let mut num_active: u32 = 0;
                let mut center = Vec2 { x: ZERO, y: ZERO };
                let normal = -character_pos.rotation.rotate(m.local_n1);
                for contact in m.contacts() {
                    if *contact.dist <= prediction {
                        num_active += 1;
                        let point = collider_pos.transform_point(*contact.local_p2);
                        let target_vel = platform.velocity_at_point(point);
                        let normal_target_mvt = target_vel.dot(normal) * dt;
                        let normal_current_mvt = translation_remaining.dot(normal);
                        center += point;
                        translation_remaining += normal
                            .mul_scalar(normal_target_mvt - normal_current_mvt);
                    }
                }
                if num_active > 0 {
                    let n: i32 = num_active.try_into().unwrap();
                    let inv = FixedTrait::from_int(n);
                    let target_vel = platform
                        .velocity_at_point(Vec2 { x: center.x / inv, y: center.y / inv });
                    let tangent = (target_vel - normal.mul_scalar(target_vel.dot(normal)))
                        .mul_scalar(dt);
                    // Larger absolute value wins, per component.
                    if tangent.x.abs() > kinematic_friction_translation.x.abs() {
                        kinematic_friction_translation.x = tangent.x;
                    }
                    if tangent.y.abs() > kinematic_friction_translation.y.abs() {
                        kinematic_friction_translation.y = tangent.y;
                    }
                }
            }
        }
        translation_remaining += kinematic_friction_translation - init;
    }
    grounded
}

/// Upstream `check_and_fix_penetrations`: up to four passes pushing the character out of every
/// solid collider it penetrates by more than `1e-5`, back to the offset gap, within a budget of a
/// quarter of its height per call.
pub(crate) fn check_and_fix_penetrations(
    controller: KinematicCharacterController,
    ref world: World,
    queries: QueryPipeline,
    shape: Shape,
    character_pos: Pose2,
    dims: Vec2,
    ref result: EffectiveCharacterMovement,
) {
    let offset = controller.offset.eval(dims.y);
    let max_correction = dims.y * MAX_CORRECTION;
    let mut applied = ZERO;
    let mut pass: u32 = 0;
    while pass != PENETRATION_PASSES {
        pass += 1;
        let aabb = shape.compute_aabb(shifted(character_pos, result.translation)).loosened(offset);
        let mut corrected = false;
        for (_, collider) in colliders_in(ref world, queries, aabb) {
            if collider.is_sensor() {
                continue;
            }
            let pos = shifted(character_pos, result.translation);
            let pos12 = pos.inv_mul(collider.position());
            if let Some(Some(c)) = contact(pos12, shape, collider.shape, ZERO) {
                if c.dist < -PENETRATION_EPS {
                    // Push out until the offset gap is restored.
                    let push = (offset - c.dist).min(max_correction - applied);
                    if push <= ZERO {
                        return; // The per-call budget is exhausted.
                    }
                    // `normal1` (character frame) points towards the obstacle: move back along it.
                    result.translation -= pos.rotation.rotate(c.normal1).mul_scalar(push);
                    applied += push;
                    corrected = true;
                }
            }
        }
        if !corrected {
            break;
        }
    }
}

/// Upstream `solve_single_character_collision_impulse` (see
/// `KinematicCharacterControllerTrait::solve_character_collision_impulses`).
pub(crate) fn solve_single_character_collision_impulse(
    controller: KinematicCharacterController,
    dt: Fixed,
    ref world: World,
    queries: QueryPipeline,
    shape: Shape,
    character_mass: Fixed,
    collision: CharacterCollision,
) {
    let extents = shape.compute_local_aabb().extents();
    let up_extent = extents.dot(controller.up.abs());
    let hit_normal = collision.hit.normal1;
    let movement_to_transfer = hit_normal
        .mul_scalar(collision.translation_remaining.dot(hit_normal));
    let prediction = predict_ground(controller, up_extent);
    let character_pos = collision.character_pos;
    let aabb = shape.compute_aabb(character_pos).loosened(prediction);

    // Gather the manifolds against dynamic bodies first, as upstream.
    let mut gathered: Array<(ContactManifold, rapier_core::Handle, Pose2)> = array![];
    for (_, collider) in colliders_in(ref world, queries, aabb) {
        if let Some(parent) = collider.parent() {
            if let Some(body) = world.bodies.get(parent) {
                if body.is_dynamic() {
                    let collider_pos = collider.position();
                    let pos12 = character_pos.inv_mul(collider_pos);
                    for m in manifolds(pos12, shape, collider.shape, prediction) {
                        gathered.append((m, parent, collider_pos));
                    }
                }
            }
        }
    }

    let inv_dt = if dt == ZERO {
        ZERO
    } else {
        ONE / dt
    };
    let velocity_to_transfer = movement_to_transfer.mul_scalar(inv_dt);
    for (manifold, body_handle, collider_pos) in gathered {
        let Some(mut body) = world.bodies.get(body_handle) else {
            continue;
        };
        let normal = character_pos.rotation.rotate(manifold.local_n1);
        for pt in manifold.contacts() {
            if *pt.dist <= prediction {
                let body_mass = body.mass();
                let point = collider_pos.transform_point(*pt.local_p2);
                let delta_vel = (velocity_to_transfer - body.velocity_at_point(point)).dot(normal);
                let mass_ratio = body_mass * character_mass / (body_mass + character_mass);
                body
                    .apply_impulse_at_point(
                        normal.mul_scalar(delta_vel.max(ZERO) * mass_ratio), point, true,
                    );
            }
        }
        let _ = world.bodies.set(body_handle, body);
    }
}
