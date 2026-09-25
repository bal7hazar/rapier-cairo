//! Rejected candidates of the fused step (work package OP), kept for re-ranking (AGENTS.md §5).
//! Ranking in the pipeline module documentation; equivalence in `tests`.

use fixed::{Fixed, HALF};
use rapier_core::Handle;
use rapier_core::collider::{ColliderEnabled, ColliderTypeTrait};
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_core::rigid_body::{RigidBodyDominanceTrait, RigidBodyType};
use rapier_dynamics2d::collider::{Collider, ColliderTrait};
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::joint::ImpulseJointSetTrait;
use rapier_dynamics2d::narrow_phase::PairCollider;
use rapier_dynamics2d::rigid_body::RigidBodyMassPropsTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait};
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::broad_phase::BroadPhaseProxy;
use rapier_geometry2d::shape::ShapeTrait;
use crate::world::World;
use super::{
    BodyInfo, SleepCensusTrait, advance_to_final_positions, body_info, detect_collisions,
    handle_user_changes, no_body_info, snapshot_collider, solve, update_islands,
};

/// The pre-OP `World::step`: the public stage functions one after the other, each walking the
/// sets on its own (user changes, proxies, `pair_colliders`, position update).
pub fn step_staged(ref world: World) -> Array<CollisionEvent> {
    handle_user_changes(ref world.bodies, ref world.colliders, world.narrow_phase.pairs.span());
    super::kinematic::interpolate_kinematic_velocities(
        ref world.bodies, world.integration_parameters,
    );
    let events = detect_collisions(
        world.integration_parameters, ref world.bodies, ref world.colliders, ref world.narrow_phase,
    );
    let entries = world.bodies.iter();
    let joints = world.impulse_joints.to_array();
    let _ = update_islands(
        ref world.bodies,
        world.narrow_phase.pairs.span(),
        array![].span(),
        joints.span(),
        entries.span(),
        SleepCensusTrait::taken(entries.span()),
    );
    solve(
        world.gravity,
        world.integration_parameters,
        ref world.bodies,
        ref world.narrow_phase,
        ref world.impulse_joints,
    );
    advance_to_final_positions(ref world.bodies, ref world.colliders, world.integration_parameters);
    events
}

/// `collision_inputs` with the parent read of a sparse set out of line (`#[inline(never)]`):
/// the loop body then pays a call on the dense hit too.
pub fn collision_inputs_outlined_fallback(
    snapshot: Span<(Handle, Collider)>,
    infos: Span<BodyInfo>,
    ref bodies: RigidBodySet,
    prediction: Fixed,
) -> (Array<BroadPhaseProxy>, Span<PairCollider>) {
    let margin = prediction * HALF;
    let mut proxies = array![];
    let mut scratch = array![];
    for (handle, collider) in snapshot {
        let collider = *collider;
        let body = collider.parent();
        let (body_type, world_com, dominance, sleeping) = match body {
            Some(parent) => {
                let mut dense = None;
                if let Some(info) = infos.get(parent.index) {
                    let info = *info.unbox();
                    if info.handle == parent {
                        dense =
                            Some((info.body_type, info.world_com, info.dominance, info.sleeping));
                    }
                }
                match dense {
                    Some(found) => found,
                    None => read_body_info_outlined(parent, ref bodies),
                }
            },
            None => no_body_info(),
        };
        let pose = collider.pos.pose;
        proxies
            .append(
                BroadPhaseProxy {
                    collider: *handle,
                    aabb: collider.shape.compute_aabb(pose).loosened(margin),
                    is_static: body_type == RigidBodyType::Fixed || sleeping,
                },
            );
        scratch.append(scratch_entry(*handle, collider, body_type, world_com, dominance));
    }
    (proxies, scratch.span())
}

#[inline(never)]
fn read_body_info_outlined(
    handle: Handle, ref bodies: RigidBodySet,
) -> (RigidBodyType, glam::Vec2, i16, bool) {
    match bodies.get(handle) {
        Some(body) => (
            body.body_type,
            body.mprops.world_com,
            body.dominance.effective_group(body.body_type),
            body.activation.sleeping,
        ),
        None => no_body_info(),
    }
}

#[inline(always)]
fn scratch_entry(
    handle: Handle,
    collider: Collider,
    body_type: RigidBodyType,
    world_com: glam::Vec2,
    dominance: i16,
) -> PairCollider {
    PairCollider {
        handle,
        solid: collider.is_enabled() && !collider.is_sensor(),
        shape: collider.shape,
        pose: collider.pos.pose,
        friction: collider.material.friction,
        restitution: collider.material.restitution,
        friction_combine_rule: collider.material.friction_combine_rule,
        restitution_combine_rule: collider.material.restitution_combine_rule,
        active_collision_types: collider.flags.active_collision_types,
        collision_groups: collider.flags.collision_groups,
        solver_groups: collider.flags.solver_groups,
        active_events: collider.flags.active_events,
        one_way: collider.one_way,
        body: collider.parent(),
        body_type,
        world_com,
        dominance,
    }
}

/// `collision_inputs` reading the fields it needs through the snapshot instead of copying the
/// whole collider out of it.
pub fn collision_inputs_field_reads(
    snapshot: Span<(Handle, Collider)>,
    infos: Span<BodyInfo>,
    ref bodies: RigidBodySet,
    prediction: Fixed,
) -> (Array<BroadPhaseProxy>, Span<PairCollider>) {
    let margin = prediction * HALF;
    let mut proxies = array![];
    let mut scratch = array![];
    for (handle, collider) in snapshot {
        let (body, parent_info) = match collider.parent {
            Some(parent) => {
                let parent = *parent.handle;
                (Some(parent), body_info(infos, parent, ref bodies))
            },
            None => (None, no_body_info()),
        };
        let (body_type, world_com, dominance, sleeping) = parent_info;
        let shape = *collider.shape;
        let pose = *collider.pos.pose;
        let material = *collider.material;
        let flags = *collider.flags;
        proxies
            .append(
                BroadPhaseProxy {
                    collider: *handle,
                    aabb: shape.compute_aabb(pose).loosened(margin),
                    is_static: body_type == RigidBodyType::Fixed || sleeping,
                },
            );
        scratch
            .append(
                PairCollider {
                    handle: *handle,
                    solid: flags.enabled == ColliderEnabled::Enabled
                        && !(*collider.co_type).is_sensor(),
                    shape,
                    pose,
                    friction: material.friction,
                    restitution: material.restitution,
                    friction_combine_rule: material.friction_combine_rule,
                    restitution_combine_rule: material.restitution_combine_rule,
                    active_collision_types: flags.active_collision_types,
                    collision_groups: flags.collision_groups,
                    solver_groups: flags.solver_groups,
                    active_events: flags.active_events,
                    one_way: *collider.one_way,
                    body,
                    body_type,
                    world_com,
                    dominance,
                },
            );
    }
    (proxies, scratch.span())
}

/// `advance_with_snapshot` with the per-body update out of line (`#[inline(never)]`, as the
/// staged `advance_to_final_positions`).
pub fn advance_with_snapshot_outlined(
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    snapshot: Span<(Handle, Collider)>,
    params: IntegrationParameters,
) {
    for (handle, body) in bodies.iter().span() {
        if *body.enabled && *body.body_type != RigidBodyType::Fixed && !*body.activation.sleeping {
            advance_body_outlined(*handle, *body, ref bodies, ref colliders, snapshot, params);
        }
    }
}

#[inline(never)]
fn advance_body_outlined(
    handle: Handle,
    body: RigidBody,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    snapshot: Span<(Handle, Collider)>,
    params: IntegrationParameters,
) {
    let mut body = body;
    let previous = body.pos.position;
    body.pos.position = body.pos.next_position;
    super::islands::update_sleep_timer(ref body, previous, params);
    body.mprops = body.mprops.update_world_mass_properties(body.body_type, body.pos.position);
    let _ = bodies.set(handle, body);
    for co_handle in body.colliders {
        if let Some(mut collider) = snapshot_collider(snapshot, *co_handle, ref colliders) {
            if let Some(parent) = collider.parent {
                collider.pos.pose = body.pos.position * parent.pos_wrt_parent;
                let _ = colliders.set(*co_handle, collider);
            }
        }
    }
}
