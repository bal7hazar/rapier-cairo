//! Stage 1 of the step (upstream `pipeline/user_changes.rs`: `handle_user_changes_to_colliders`
//! then `handle_user_changes_to_rigid_bodies`, and the clearing of the modified sets), moved out
//! of `crate::pipeline` by work package SL (file budget). The fused `user_changes_bodies` walk
//! of the pipeline calls the per-object functions below; [`handle_user_changes`] is the staged
//! stage.
//!
//! Colliders first: a new parent moves the collider; a new shape, mass, parent or enabled state
//! marks the parent's mass for recomputation. Then the bodies: a moved body or a new collider
//! list moves the colliders and refreshes the world mass properties; a changed collider list or
//! local mass recomputes the mass from the colliders (`max_extent` with it, SL). Every change
//! flag is cleared.
//!
//! Sleeping (SL): the colliders a change touches — every flagged collider, and the colliders of
//! a body whose pose, collider list, type, dominance or enabled state changed (upstream flags
//! them `POSITION` or `PARENT_EFFECTIVE_DOMINANCE` / `ENABLED_OR_DISABLED`) — wake up their
//! parent (strongly, in place: the body is in hand) and their contact partners of the previous
//! step (`sleeping::wake_touched_partners`, upstream's modified-colliders pass in the narrow
//! phase).
//!
//! Colliders inserted since the last step (BT2, upstream-exact): upstream's modified-colliders
//! pass only wakes through a collider the narrow phase already indexes, which a collider inserted
//! since the last step is not. Such a collider carries every change flag (the builder's
//! `ColliderChanges::all()`, also raised by `ColliderSetTrait::insert*`): it wakes neither its
//! parent nor anybody else, and a body whose colliders are all new is not woken by its own
//! changes either (a body built `sleeping(true)` stays asleep). The stage returns these *fresh*
//! colliders: during the step their proxy is not static even on a sleeping parent (upstream's
//! broad phase pairs every modified collider), and a contact that starts on one strong-wakes the
//! sleeping dynamic side (`sleeping::wake_started_contacts`, upstream `strong_wake_sleeping_side`).

use fixed::{ONE, ZERO};
use rapier_core::Handle;
use rapier_core::collider::changes::{
    ENABLED_OR_DISABLED, LOCAL_MASS_PROPERTIES as CO_LOCAL_MASS_PROPERTIES, PARENT, SHAPE,
};
use rapier_core::collider::{ColliderChanges, ColliderChangesTrait};
use rapier_core::rigid_body::changes::{COLLIDERS, LOCAL_MASS_PROPERTIES, POSITION};
use rapier_core::rigid_body::{RigidBodyChanges, RigidBodyChangesTrait};
use rapier_dynamics2d::collider::{Collider, ColliderTrait};
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::narrow_phase::ContactPair;
use rapier_dynamics2d::rigid_body::RigidBodyMassPropsTrait;
use rapier_dynamics2d::rigid_body_set::{
    RigidBody, RigidBodySet, RigidBodySetTrait, RigidBodyTrait, cold_or_default,
    extra_additional_is_mass,
};
use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
use rapier_geometry2d::shape::ShapeTrait;
use super::islands::max_extent;
use super::sleeping::wake_touched_partners;

/// The body changes whose colliders upstream flags for the narrow phase (`POSITION` from
/// `update_positions`, `PARENT_EFFECTIVE_DOMINANCE`, `ENABLED_OR_DISABLED`): `POSITION |
/// COLLIDERS | TYPE | DOMINANCE | ENABLED_OR_DISABLED`.
const TOUCHING_CHANGES: RigidBodyChanges = RigidBodyChanges { bits: 0xba };

/// Stage 1: see the module documentation. Scans every collider and every body in ascending
/// slot; only the flagged ones are rewritten; then the wake-up pass over `pairs` (the previous
/// step's contact pairs).
pub fn handle_user_changes(
    ref bodies: RigidBodySet, ref colliders: ColliderSet, pairs: Span<ContactPair>,
) {
    let _ = handle_user_changes_fresh(ref bodies, ref colliders, pairs);
}

/// [`handle_user_changes`] that returns the colliders inserted since the last step (see the
/// module documentation), ascending slot.
pub(crate) fn handle_user_changes_fresh(
    ref bodies: RigidBodySet, ref colliders: ColliderSet, pairs: Span<ContactPair>,
) -> Array<Handle> {
    let mut touched = array![];
    let mut fresh = array![];
    for (handle, collider) in colliders.iter() {
        if !collider.changes.is_empty() {
            collider_changes(handle, collider, ref bodies, ref colliders, ref touched, ref fresh);
        }
    }
    for (handle, body) in bodies.iter() {
        if !body.changes.is_empty() {
            let _ = body_changes(
                handle, body, ref bodies, ref colliders, ref touched, fresh.span(),
            );
        }
    }
    if !touched.is_empty() && !pairs.is_empty() {
        let _ = wake_touched_partners(touched.span(), pairs, ref bodies, ref colliders);
    }
    fresh
}

/// Whether `changes` are those of a collider inserted since the last step (every flag raised).
#[inline(always)]
pub(crate) fn is_new(changes: ColliderChanges) -> bool {
    changes == ColliderChangesTrait::all()
}

/// Whether `handle` is one of `fresh` (ascending slot, as the collider walk appends them):
/// binary search on the slot.
pub(crate) fn is_fresh(fresh: Span<Handle>, handle: Handle) -> bool {
    let mut lo: u32 = 0;
    let mut hi: u32 = fresh.len();
    while lo != hi {
        let mid = (lo + hi) / 2;
        let h = *fresh.at(mid);
        if h.index < handle.index {
            lo = mid + 1;
        } else if h.index > handle.index {
            hi = mid;
        } else {
            return h == handle;
        }
    }
    false
}

/// The user changes of one flagged collider, flags cleared; the collider is appended to
/// `touched` (wake-up pass), or to `fresh` when it was inserted since the last step (it wakes
/// nobody, see the module documentation).
#[inline(never)]
pub(crate) fn collider_changes(
    handle: Handle,
    collider: Collider,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    ref touched: Array<Handle>,
    ref fresh: Array<Handle>,
) {
    let mut collider = collider;
    let changes = collider.changes;
    let new = is_new(changes);
    if let Some(parent) = collider.parent {
        if let Some(mut body) = bodies.get(parent.handle) {
            if changes.contains(PARENT) {
                collider.pos.pose = body.pos.position * parent.pos_wrt_parent;
            }
            // Upstream's modified-colliders pass wakes the parent up strongly, through a
            // collider the narrow phase already knows.
            let mut write = false;
            if !new {
                write = body.activation.sleeping
                    || body.activation.time_since_can_sleep != fixed::ZERO;
                body.wake_up(true);
            }
            if changes.intersects(SHAPE | CO_LOCAL_MASS_PROPERTIES | ENABLED_OR_DISABLED | PARENT) {
                body.changes.insert(LOCAL_MASS_PROPERTIES);
                write = true;
            }
            if write {
                let _ = bodies.set(parent.handle, body);
            }
        }
    }
    collider.changes = ColliderChangesTrait::empty();
    let _ = colliders.set(handle, collider);
    if new {
        fresh.append(handle);
    } else {
        touched.append(handle);
    }
}

/// The user changes of one flagged body, flags cleared; when the change is one of
/// [`TOUCHING_CHANGES`] and a collider of the body is not in `fresh` (inserted since the last
/// step), the body is woken up and its colliders are appended to `touched`.
#[inline(never)]
pub(crate) fn body_changes(
    handle: Handle,
    body: RigidBody,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    ref touched: Array<Handle>,
    fresh: Span<Handle>,
) -> RigidBody {
    let mut body = body;
    let changes = body.changes;
    if changes.contains(POSITION) || changes.contains(COLLIDERS) {
        move_colliders(body, ref colliders);
        body.mprops = body.mprops.update_world_mass_properties(body.body_type, body.pos.position);
    }
    if changes.intersects(LOCAL_MASS_PROPERTIES | COLLIDERS) {
        recompute_mass_properties_from_colliders(ref body, ref colliders);
    }
    if changes.intersects(TOUCHING_CHANGES) && has_known_collider(body.colliders, fresh) {
        // Upstream flags the colliders, whose modified-colliders pass wakes the body strongly
        // through the colliders the narrow phase already knows.
        body.wake_up(true);
        touched.append_span(body.colliders);
    }
    body.changes = RigidBodyChangesTrait::empty();
    let _ = bodies.set(handle, body);
    body
}

/// Upstream `RigidBodyMassProps::recompute_mass_properties_from_colliders`: the local mass
/// properties become the sum, in attachment order, of the enabled colliders' mass properties
/// expressed in the body frame; the world ones are refreshed; `max_extent` is recomputed from
/// the same colliders (upstream `recompute_max_extent`, see `islands::max_extent`).
#[inline(never)]
pub fn recompute_mass_properties_from_colliders(ref body: RigidBody, ref colliders: ColliderSet) {
    let mut shapes = array![];
    let mut local: MassProperties = Default::default();
    let mut unit: MassProperties = Default::default();
    for co_handle in body.colliders {
        if let Some(collider) = colliders.get(*co_handle) {
            if collider.is_enabled() {
                if let Some(parent) = collider.parent {
                    local = local + collider.mass_properties().transform_by(parent.pos_wrt_parent);
                    unit = unit
                        + collider.shape.mass_properties(ONE).transform_by(parent.pos_wrt_parent);
                    shapes.append((collider.shape, parent.pos_wrt_parent));
                }
            }
        }
    }
    let cold = cold_or_default(body.cold);
    if extra_additional_is_mass(cold.solver_flags) {
        let mass = cold.additional_local_mprops.mass();
        let prev_mass = local.mass();
        if prev_mass > ZERO {
            local.set_mass(prev_mass + mass, true);
        } else if unit.mass() > ZERO {
            unit.set_mass(mass, true);
            local = local + unit;
        } else {
            local.set_mass(mass, true);
        }
    } else {
        local = local + cold.additional_local_mprops;
    }
    body.mprops.local_mprops = local;
    body.mprops.max_extent = max_extent(local.local_com, shapes.span());
    body.mprops = body.mprops.update_world_mass_properties(body.body_type, body.pos.position);
}

/// Whether one of `colliders` is not in `fresh`.
#[inline(always)]
pub(crate) fn has_known_collider(colliders: Span<Handle>, fresh: Span<Handle>) -> bool {
    if fresh.is_empty() {
        return !colliders.is_empty();
    }
    for handle in colliders {
        if !is_fresh(fresh, *handle) {
            return true;
        }
    }
    false
}

/// Sets the world pose of the colliders of `body` to `body.position * pos_wrt_parent`, without
/// raising change flags (upstream `RigidBodyColliders::update_positions`).
pub(crate) fn move_colliders(body: RigidBody, ref colliders: ColliderSet) {
    for co_handle in body.colliders {
        if let Some(mut collider) = colliders.get(*co_handle) {
            if let Some(parent) = collider.parent {
                collider.pos.pose = body.pos.position * parent.pos_wrt_parent;
                let _ = colliders.set(*co_handle, collider);
            }
        }
    }
}
