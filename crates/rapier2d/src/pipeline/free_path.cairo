use fixed::{Fixed, HALF};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::ActiveEventsTrait;
use rapier_core::collider::events::CONTACT_FORCE_EVENTS;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_core::rigid_body::{RigidBodyDominance, RigidBodyDominanceTrait, RigidBodyType};
use rapier_dynamics2d::collider::{Collider, ColliderTrait};
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::narrow_phase::PairCollider;
use rapier_dynamics2d::rigid_body::RigidBodyMassPropsTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait};
use rapier_dynamics2d::solver::island::FreeBodySolverTrait;
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::broad_phase::BroadPhaseProxy;
use rapier_geometry2d::shape::ShapeTrait;
use super::{BodyInfo, advance_body_with_snapshot, islands, moving};

/// The [`BodyInfo`] of a missing or absent parent.
#[inline(always)]
pub(crate) fn no_body_info() -> (RigidBodyType, Vec2, i16, bool) {
    let dominance: RigidBodyDominance = Default::default();
    (
        RigidBodyType::Fixed,
        Default::default(),
        dominance.effective_group(RigidBodyType::Fixed),
        false,
    )
}

/// The `(body_type, world_com, dominance, sleeping)` of the body `handle`: `infos[handle.index]`
/// when that entry has the handle, a set read otherwise.
#[inline(always)]
pub(crate) fn body_info(
    infos: Span<BodyInfo>, handle: Handle, ref bodies: RigidBodySet,
) -> (RigidBodyType, Vec2, i16, bool) {
    if let Some(info) = infos.get(handle.index) {
        let info = *info.unbox();
        if info.handle == handle {
            return (info.body_type, info.world_com, info.dominance, info.sleeping);
        }
    }
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
fn body_info_from_entries(
    entries: Span<(Handle, RigidBody)>, handle: Handle, ref bodies: RigidBodySet,
) -> (RigidBodyType, bool) {
    if let Some(entry) = entries.get(handle.index) {
        let (h, body) = *entry.unbox();
        if h == handle {
            return (body.body_type, body.activation.sleeping);
        }
    }
    match bodies.get(handle) {
        Some(body) => (body.body_type, body.activation.sleeping),
        None => {
            let (body_type, _, _, sleeping) = no_body_info();
            (body_type, sleeping)
        },
    }
}

pub(crate) fn collision_proxies_from_entries_with_events(
    snapshot: Span<(Handle, Collider)>,
    entries: Span<(Handle, RigidBody)>,
    ref bodies: RigidBodySet,
    prediction: Fixed,
) -> (Array<BroadPhaseProxy>, bool, bool) {
    let margin = prediction * HALF;
    let mut proxies = array![];
    let mut any_sleeping = false;
    let mut force_events = false;
    for (handle, collider) in snapshot {
        let collider = *collider;
        if collider.flags.active_events.bits != 0
            && collider.flags.active_events.contains(CONTACT_FORCE_EVENTS) {
            force_events = true;
        }
        let (body_type, sleeping) = match collider.parent() {
            Some(parent) => body_info_from_entries(entries, parent, ref bodies),
            None => {
                let (body_type, _, _, sleeping) = no_body_info();
                (body_type, sleeping)
            },
        };
        if sleeping {
            any_sleeping = true;
        }
        proxies
            .append(
                BroadPhaseProxy {
                    collider: *handle,
                    aabb: collider.shape.compute_aabb(collider.pos.pose).loosened(margin),
                    is_static: body_type == RigidBodyType::Fixed || sleeping,
                },
            );
    }
    (proxies, any_sleeping, force_events)
}

pub(crate) fn collision_scratch(
    snapshot: Span<(Handle, Collider)>, infos: Span<BodyInfo>, ref bodies: RigidBodySet,
) -> Span<PairCollider> {
    let mut scratch = array![];
    for (handle, collider) in snapshot {
        let collider = *collider;
        let body = collider.parent();
        let (body_type, world_com, dominance, _) = match body {
            Some(parent) => body_info(infos, parent, ref bodies),
            None => no_body_info(),
        };
        scratch
            .append(
                PairCollider {
                    handle: *handle,
                    solid: collider.is_enabled() && !collider.is_sensor(),
                    sensor: collider.is_enabled() && collider.is_sensor(),
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
                    body,
                    body_type,
                    world_com,
                    dominance,
                },
            );
    }
    scratch.span()
}

#[inline(always)]
fn snapshot_collider_free(
    snapshot: Span<(Handle, Collider)>, handle: Handle, ref colliders: ColliderSet,
) -> Option<Collider> {
    if let Some(entry) = snapshot.get(handle.index) {
        let (h, collider) = *entry.unbox();
        if h == handle {
            return Some(collider);
        }
    }
    colliders.get(handle)
}

#[inline(always)]
fn advance_free_unchecked(
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
    islands::update_sleep_timer(ref body, previous, params);
    body.mprops = body.mprops.update_world_mass_properties(body.body_type, body.pos.position);
    let _ = bodies.set(handle, body);
    if body.colliders.len() == 1 {
        let co_handle = *body.colliders.at(0);
        if let Some(mut collider) = snapshot_collider_free(snapshot, co_handle, ref colliders) {
            if let Some(parent) = collider.parent {
                collider.pos.pose = body.pos.position * parent.pos_wrt_parent;
                let _ = colliders.set(co_handle, collider);
            }
        }
        return;
    }
    for co_handle in body.colliders {
        if let Some(mut collider) = snapshot_collider_free(snapshot, *co_handle, ref colliders) {
            if let Some(parent) = collider.parent {
                collider.pos.pose = body.pos.position * parent.pos_wrt_parent;
                let _ = colliders.set(*co_handle, collider);
            }
        }
    }
}

#[inline(never)]
pub(crate) fn solve_and_advance_free(
    gravity: Vec2,
    params: IntegrationParameters,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    entries: Span<(Handle, RigidBody)>,
    snapshot: Span<(Handle, Collider)>,
    all_moving: bool,
) {
    let free = FreeBodySolverTrait::new(params, gravity);
    if all_moving {
        for (handle, body) in entries {
            let body = free.solve(*handle, *body);
            advance_free_unchecked(*handle, body, ref bodies, ref colliders, snapshot, params);
        }
        return;
    }
    for (handle, body) in entries {
        if moving(body) {
            let body = free.solve(*handle, *body);
            advance_body_with_snapshot(*handle, body, ref bodies, ref colliders, snapshot, params);
        }
    }
}
