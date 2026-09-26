//! The rigid body (upstream `dynamics/rigid_body.rs`, `RigidBody`) and the set that owns them
//! (upstream `dynamics/rigid_body_set.rs`, `RigidBodySet`).
//!
//! A [`RigidBody`] assembles the vector components of `crate::rigid_body` (pose, mass
//! properties, velocities, forces) with the scalar components of `rapier_core::rigid_body`
//! (type, damping, activation, dominance, change flags) and the handles of its colliders.
//!
//! [`RigidBodySet`] is a generational [`Arena`] (`docs/PLAN.md`, wave-1 outcome: arena for the
//! persistent sets). Handles are those upstream would hand out for the same call sequence.
//!
//! Deviations from upstream:
//! * no modified-body list: `propagate_modified_body_positions_to_colliders` scans every body in
//!   ascending slot index and acts on those whose `changes` contain `POSITION`; clearing the
//!   change flags is the pipeline's job (upstream `clear_modified`), deferred with it;
//! * `get_mut` is `set` (read, modify the copy, write it back), bodies being plain values;
//! * `remove` takes no island manager nor joint sets (islands and joint sets are out of this
//!   package): it removes or detaches the attached colliders only;
//! * sleeping (work package SL): the setters that take a `wake_up` flag upstream wake the body
//!   up with that argument fixed to true (`set_position`, `set_linvel`, `set_angvel`; callers
//!   pass `true`); the force and impulse helpers keep upstream's flag. A wake-up only marks the
//!   body: the step wakes its island (`rapier2d::pipeline::islands`). Write the fields directly
//!   to change a body without waking it.

use core::num::traits::DivRem;
use fixed::{Fixed, ONE, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::changes::{PARENT, POSITION as COLLIDER_POSITION};
use rapier_core::data::arena::{
    Arena, ArenaField, ArenaFieldTrait, ArenaState, ArenaStateTrait, ArenaTrait,
};
use rapier_core::rigid_body::changes::{COLLIDERS, POSITION};
use rapier_core::rigid_body::{
    RigidBodyActivation, RigidBodyChanges, RigidBodyChangesTrait, RigidBodyDamping,
    RigidBodyDominance, RigidBodyType,
};
use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
use rapier_geometry2d::shape::ShapeTrait;
use rapier_math::pose2::Pose2;
use crate::collider::{Collider, ColliderTrait};
use crate::collider_set::{ColliderSet, ColliderSetTrait};
use crate::rigid_body::{
    RigidBodyForces, RigidBodyMassProps, RigidBodyMassPropsTrait, RigidBodyPosition,
    RigidBodyVelocity,
};

/// Panic messages of the body set.
pub mod errors {
    /// A collider was attached to a body handle that does not resolve.
    pub const BODY_NOT_FOUND: felt252 = 'RigidBodySet: body not found';
}

/// A rigid body (upstream `RigidBody`, minus CCD, solver ids and soft-body links).
///
/// `colliders` is a `Span<Handle>` rather than an `Array` because arena values must be `Copy`;
/// rather than a fixed-size array because upstream puts no bound on the number of colliders of
/// a body. Attaching or detaching a collider rebuilds the span, O(colliders of the body), on the
/// cold path; the step only reads it.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RigidBody {
    pub pos: RigidBodyPosition,
    pub mprops: RigidBodyMassProps,
    pub vels: RigidBodyVelocity,
    pub damping: RigidBodyDamping,
    pub forces: RigidBodyForces,
    /// Colliders attached to the body, in attachment order (upstream `RigidBodyColliders`).
    pub colliders: Span<Handle>,
    pub activation: RigidBodyActivation,
    pub changes: RigidBodyChanges,
    pub body_type: RigidBodyType,
    pub dominance: RigidBodyDominance,
    pub enabled: bool,
    /// Cold API-only data, boxed so the per-frame step copies one word for default bodies.
    pub cold: Box<Option<RigidBodyCold>>,
}

/// Rarely used rigid-body API data kept out of the hot body value.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RigidBodyCold {
    pub additional_local_mprops: MassProperties,
    /// Packed as solver iterations | pgs iterations | flags.
    pub solver_flags: u128,
    pub user_data: u128,
}

pub impl RigidBodyColdDefault of Default<RigidBodyCold> {
    #[inline(always)]
    fn default() -> RigidBodyCold {
        RigidBodyCold { additional_local_mprops: Default::default(), solver_flags: 0, user_data: 0 }
    }
}

/// Box serialization stores the cold value, independent of allocation identity.
pub impl BoxedRigidBodyColdSerde of Serde<Box<Option<RigidBodyCold>>> {
    fn serialize(self: @Box<Option<RigidBodyCold>>, ref output: Array<felt252>) {
        let value = (*self).unbox();
        value.serialize(ref output);
    }

    fn deserialize(ref serialized: Span<felt252>) -> Option<Box<Option<RigidBodyCold>>> {
        Some(BoxTrait::new(Serde::<Option<RigidBodyCold>>::deserialize(ref serialized)?))
    }
}

/// Structural equality of boxed cold data.
pub impl BoxedRigidBodyColdPartialEq of PartialEq<Box<Option<RigidBodyCold>>> {
    fn eq(lhs: @Box<Option<RigidBodyCold>>, rhs: @Box<Option<RigidBodyCold>>) -> bool {
        (*lhs).unbox() == (*rhs).unbox()
    }

    fn ne(lhs: @Box<Option<RigidBodyCold>>, rhs: @Box<Option<RigidBodyCold>>) -> bool {
        !Self::eq(lhs, rhs)
    }
}

#[inline(always)]
pub fn no_cold() -> Box<Option<RigidBodyCold>> {
    BoxTrait::new(None)
}

#[inline(always)]
pub fn cold_or_default(cold: Box<Option<RigidBodyCold>>) -> RigidBodyCold {
    cold.unbox().unwrap_or_default()
}

const RB_EXTRA_WORD: NonZero<u128> = 0x100000000;
const RB_EXTRA_FAST_ROTATION: u128 = 0x10000000000000000;
const RB_EXTRA_ADDITIONAL_MASS: u128 = 0x20000000000000000;
const RB_EXTRA_ADDITIONAL_MASS_WORD: NonZero<u128> = 0x20000000000000000;

#[inline(always)]
pub fn extra_solver_iterations(bits: u128) -> u32 {
    let (_, solver) = DivRem::div_rem(bits, RB_EXTRA_WORD);
    solver.try_into().unwrap()
}

#[inline(always)]
pub fn extra_pgs_iterations(bits: u128) -> u32 {
    let (shifted, _) = DivRem::div_rem(bits, RB_EXTRA_WORD);
    let (_, pgs) = DivRem::div_rem(shifted, RB_EXTRA_WORD);
    pgs.try_into().unwrap()
}

#[inline(always)]
pub fn extra_allow_fast_rotation(bits: u128) -> bool {
    let (_, flags) = DivRem::div_rem(bits, RB_EXTRA_ADDITIONAL_MASS_WORD);
    flags >= RB_EXTRA_FAST_ROTATION
}

#[inline(always)]
pub fn extra_additional_is_mass(bits: u128) -> bool {
    bits >= RB_EXTRA_ADDITIONAL_MASS
}

#[inline(always)]
fn extra_flags(bits: u128) -> u128 {
    let mut flags = 0;
    if extra_allow_fast_rotation(bits) {
        flags += RB_EXTRA_FAST_ROTATION;
    }
    if extra_additional_is_mass(bits) {
        flags += RB_EXTRA_ADDITIONAL_MASS;
    }
    flags
}

#[inline(always)]
pub fn set_extra_solver_iterations(bits: u128, solver: u32) -> u128 {
    let (shifted, _) = DivRem::div_rem(bits, RB_EXTRA_WORD);
    shifted * 0x100000000 + solver.into()
}

#[inline(always)]
pub fn set_extra_pgs_iterations(bits: u128, pgs: u32) -> u128 {
    let solver: u128 = extra_solver_iterations(bits).into();
    extra_flags(bits) + pgs.into() * 0x100000000 + solver
}

#[inline(always)]
pub fn set_extra_allow_fast_rotation(bits: u128, allow: bool) -> u128 {
    let cleared = if extra_allow_fast_rotation(bits) {
        bits - RB_EXTRA_FAST_ROTATION
    } else {
        bits
    };
    if allow {
        cleared + RB_EXTRA_FAST_ROTATION
    } else {
        cleared
    }
}

#[inline(always)]
pub fn set_extra_additional_is_mass(bits: u128, is_mass: bool) -> u128 {
    let cleared = if extra_additional_is_mass(bits) {
        bits - RB_EXTRA_ADDITIONAL_MASS
    } else {
        bits
    };
    if is_mass {
        cleared + RB_EXTRA_ADDITIONAL_MASS
    } else {
        cleared
    }
}

/// A pair of rigid-body handles.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct BodyPair {
    pub body1: Handle,
    pub body2: Handle,
}

/// Constructor for [`BodyPair`].
#[generate_trait]
pub impl BodyPairImpl of BodyPairTrait {
    /// Builds a pair of rigid-body handles.
    #[inline(always)]
    fn new(body1: Handle, body2: Handle) -> BodyPair {
        BodyPair { body1, body2 }
    }
}

/// Upstream default: a dynamic body at identity with all defaults.
pub impl RigidBodyDefault of Default<RigidBody> {
    #[inline(always)]
    fn default() -> RigidBody {
        RigidBodyTrait::new(RigidBodyType::Dynamic, Default::default())
    }
}

/// Constructors, getters and setters of [`RigidBody`] (upstream names).
pub mod body_api;
pub mod builder_api;
pub use body_api::{RigidBodyImpl, RigidBodyTrait};
pub use builder_api::{
    RigidBodyBuilder, RigidBodyBuilderDefault, RigidBodyBuilderImpl, RigidBodyBuilderTrait,
    RigidBodyFromBuilder,
};

/// Component reads of a stored body (BT4, `RigidBodySetTrait::get_field`): the activation's
/// `sleeping` flag.
pub impl BodySleeping of ArenaField<RigidBody, bool> {
    #[inline(always)]
    fn read(value: RigidBody) -> bool {
        value.activation.sleeping
    }
}

/// The linear velocity of a stored body.
pub impl BodyLinvel of ArenaField<RigidBody, Vec2> {
    #[inline(always)]
    fn read(value: RigidBody) -> Vec2 {
        value.vels.linvel
    }
}

/// The angular velocity of a stored body.
pub impl BodyAngvel of ArenaField<RigidBody, Fixed> {
    #[inline(always)]
    fn read(value: RigidBody) -> Fixed {
        value.vels.angvel
    }
}

/// The world pose of a stored body.
pub impl BodyPose of ArenaField<RigidBody, Pose2> {
    #[inline(always)]
    fn read(value: RigidBody) -> Pose2 {
        value.pos.position
    }
}

/// The set of rigid bodies. Holds a dict: pass it by `ref`.
#[derive(Destruct, Default)]
pub struct RigidBodySet {
    bodies: Arena<RigidBody>,
}

/// Operations of [`RigidBodySet`]. Reads take `ref self` because arena reads mutate the dict log.
#[generate_trait]
pub impl RigidBodySetImpl of RigidBodySetTrait {
    /// An empty set.
    #[inline(always)]
    fn new() -> RigidBodySet {
        RigidBodySet { bodies: ArenaTrait::new() }
    }

    /// Empty set. Cairo arenas reserve no memory, so `capacity` is intentionally ignored.
    #[inline(always)]
    fn with_capacity(_capacity: u32) -> RigidBodySet {
        Self::new()
    }

    /// Stores `body` and returns its handle. As upstream, the internal links are reset (the
    /// collider list is emptied: colliders attach through `ColliderSetTrait::insert_with_parent`)
    /// and every change flag is raised.
    fn insert(ref self: RigidBodySet, body: RigidBody) -> Handle {
        let mut body = body;
        body.colliders = array![].span();
        body.changes = RigidBodyChangesTrait::all();
        self.bodies.insert(body)
    }

    /// The body behind `handle`, `None` when the handle is stale or unknown.
    #[inline(always)]
    fn get(ref self: RigidBodySet, handle: Handle) -> Option<RigidBody> {
        self.bodies.get(handle)
    }

    /// One component of the body behind `handle` (`P`: [`BodySleeping`], [`BodyLinvel`],
    /// [`BodyAngvel`], [`BodyPose`]), `None` when the handle is stale or unknown. Copies only
    /// the component out of the set, where [`get`](Self::get) copies the whole body.
    fn get_field<F, impl P: ArenaField<RigidBody, F>, +Drop<F>>(
        ref self: RigidBodySet, handle: Handle,
    ) -> Option<F> {
        self.bodies.get_field::<F, P>(handle)
    }

    /// Upstream `get_mut` by name. Values are copied out; write back with [`set`].
    #[inline(always)]
    fn get_mut(ref self: RigidBodySet, handle: Handle) -> Option<RigidBody> {
        self.get(handle)
    }

    /// Gets a body by slot index and returns its live handle. O(len), deterministic slot order.
    fn get_unknown_gen(ref self: RigidBodySet, index: u32) -> Option<(RigidBody, Handle)> {
        let entries = self.bodies.to_array();
        for (handle, body) in entries {
            if handle.index == index {
                return Some((body, handle));
            }
        }
        None
    }

    /// Upstream mutable variant by name. Values are copied out; write back with [`set`].
    #[inline(always)]
    fn get_unknown_gen_mut(ref self: RigidBodySet, index: u32) -> Option<(RigidBody, Handle)> {
        self.get_unknown_gen(index)
    }

    /// Upstream `get_pair_mut` by name. Equal handles return `(first, None)`.
    fn get_pair_mut(
        ref self: RigidBodySet, handle1: Handle, handle2: Handle,
    ) -> (Option<RigidBody>, Option<RigidBody>) {
        if handle1 == handle2 {
            (self.get(handle1), None)
        } else {
            (self.get(handle1), self.get(handle2))
        }
    }

    /// Overwrites the body behind `handle` (upstream `get_mut`: read, modify, write back).
    /// Returns `false` and changes nothing when the handle does not resolve.
    #[inline(always)]
    fn set(ref self: RigidBodySet, handle: Handle, body: RigidBody) -> bool {
        self.bodies.set(handle, body)
    }

    /// `true` when `handle` resolves to a body.
    #[inline(always)]
    fn contains(ref self: RigidBodySet, handle: Handle) -> bool {
        self.bodies.contains(handle)
    }

    /// Removes the body behind `handle`, `None` when it does not resolve. Its colliders are
    /// removed when `remove_attached_colliders`, detached (no parent, `PARENT` raised, world
    /// pose kept) otherwise.
    fn remove(
        ref self: RigidBodySet,
        handle: Handle,
        ref colliders: ColliderSet,
        remove_attached_colliders: bool,
    ) -> Option<RigidBody> {
        let body = self.bodies.remove(handle)?;
        let mut attached = body.colliders;
        while let Some(co_handle) = attached.pop_front() {
            let co_handle = *co_handle;
            if remove_attached_colliders {
                let _ = colliders.remove(co_handle, ref self);
            } else if let Some(mut collider) = colliders.get(co_handle) {
                collider.parent = None;
                collider.changes = collider.changes | PARENT;
                colliders.set(co_handle, collider);
            }
        }
        Some(body)
    }

    /// A body was inserted, written, removed or (de)parented since `clear_modified` / creation
    /// (BT4: the bit lives in the arena, `ArenaStateTrait::is_modified`; a write whose handle
    /// does not resolve and [`set_internal`](Self::set_internal) leave it unchanged).
    #[inline(always)]
    fn is_modified(self: @RigidBodySet) -> bool {
        self.bodies.is_modified()
    }

    /// Raises the [`is_modified`](Self::is_modified) flag (a caller that wrote through
    /// [`set_internal`](Self::set_internal)).
    #[inline(always)]
    fn mark_modified(ref self: RigidBodySet) {
        self.bodies.mark_modified();
    }

    /// [`set`](Self::set) without raising the [`is_modified`](Self::is_modified) flag (upstream
    /// `get_mut_internal`): the step's own write-backs, which it tracks itself (BT4).
    #[inline(always)]
    fn set_internal(ref self: RigidBodySet, handle: Handle, body: RigidBody) -> bool {
        self.bodies.set_untracked(handle, body)
    }

    /// Clears the [`is_modified`](Self::is_modified) flag (the step does).
    #[inline(always)]
    fn clear_modified(ref self: RigidBodySet) {
        self.bodies.clear_modified();
    }

    /// Number of bodies.
    #[inline(always)]
    fn len(self: @RigidBodySet) -> u32 {
        self.bodies.len()
    }

    /// `true` when the set holds no body.
    #[inline(always)]
    fn is_empty(self: @RigidBodySet) -> bool {
        self.bodies.is_empty()
    }

    /// Every `(handle, body)` in ascending slot index.
    #[inline(always)]
    fn iter(ref self: RigidBodySet) -> Array<(Handle, RigidBody)> {
        self.bodies.to_array()
    }

    /// Upstream `iter_mut` by name. Values are copied out; write changed bodies back with `set`.
    #[inline(always)]
    fn iter_mut(ref self: RigidBodySet) -> Array<(Handle, RigidBody)> {
        self.iter()
    }

    /// Sets the world pose of the colliders of every body whose `changes` contain `POSITION`
    /// to `body.position * collider.position_wrt_parent`, raising the colliders' `POSITION`
    /// flag. Bodies are visited in ascending slot index, colliders in attachment order.
    /// Cost: one dict read per body, one read and one write per collider of a moved body.
    fn propagate_modified_body_positions_to_colliders(
        ref self: RigidBodySet, ref colliders: ColliderSet,
    ) {
        let bodies = self.bodies.to_array();
        for (_, body) in bodies {
            if body.changes.contains(POSITION) {
                propagate_positions(body, ref colliders);
            }
        }
    }

    /// Flat image of the set (generation counter, capacity, free list, every `(handle, body)` in
    /// ascending slot index), for save / restore. [`from_state`](Self::from_state) rebuilds a set
    /// that issues the same handles as this one for the same future calls, removals included.
    /// Cost: one dict read per allocated slot and per free slot.
    fn to_state(ref self: RigidBodySet) -> ArenaState<RigidBody> {
        self.bodies.to_state()
    }

    /// Rebuilds a set from its [`to_state`](Self::to_state) image. Cost: one dict write per
    /// allocated slot.
    ///
    /// # Panics
    /// `Arena: state ...` (`rapier_core::data::arena::errors`) when `state` is not a valid image.
    fn from_state(state: ArenaState<RigidBody>) -> RigidBodySet {
        RigidBodySet { bodies: ArenaStateTrait::from_state(state) }
    }
}

/// Upstream `RigidBodyColliders::attach_collider` (the list is `RigidBody::colliders`, a
/// `Span<Handle>`): appends `co_handle` to the collider list of the body behind `handle`, raises
/// `COLLIDERS`, adds the collider's mass (expressed in the body frame through `pos_wrt_parent`)
/// to the body's and refreshes its world mass properties.
/// Returns the body's world pose, which the caller composes with `pos_wrt_parent`.
///
/// # Panics
/// `RigidBodySet: body not found` when `handle` does not resolve.
pub fn attach_collider(
    ref bodies: RigidBodySet,
    handle: Handle,
    co_handle: Handle,
    collider: Collider,
    pos_wrt_parent: Pose2,
) -> Pose2 {
    let mut body = bodies.bodies.get(handle).expect(errors::BODY_NOT_FOUND);
    let mut list: Array<Handle> = array![];
    list.append_span(body.colliders);
    list.append(co_handle);
    body.colliders = list.span();
    body.changes.insert(COLLIDERS);
    let mprops = collider.mass_properties().transform_by(pos_wrt_parent);
    body.mprops.local_mprops = body.mprops.local_mprops + mprops;
    body.mprops = body.mprops.update_world_mass_properties(body.body_type, body.pos.position);
    let _ = bodies.bodies.set(handle, body);
    body.pos.position
}

/// Upstream `RigidBodyColliders::detach_collider` / `remove_collider_internal`: removes
/// `co_handle` from the collider list of the body behind `handle` with upstream's `swap_remove`
/// (the last handle takes its place) and raises `COLLIDERS`. As upstream, the mass properties are
/// left for the pipeline to recompute. Does nothing when the body or the collider is not found.
pub fn detach_collider(ref bodies: RigidBodySet, handle: Handle, co_handle: Handle) {
    if let Some(mut body) = bodies.bodies.get(handle) {
        let list = body.colliders;
        let n = list.len();
        let mut removed_at = 0;
        while removed_at != n && *list.at(removed_at) != co_handle {
            removed_at += 1;
        }
        if removed_at != n {
            let last = n - 1;
            let mut swapped: Array<Handle> = array![];
            let mut k = 0;
            while k != last {
                swapped.append(if k == removed_at {
                    *list.at(last)
                } else {
                    *list.at(k)
                });
                k += 1;
            }
            body.colliders = swapped.span();
            body.changes.insert(COLLIDERS);
            let _ = bodies.bodies.set(handle, body);
        }
    }
}

/// Upstream mass recompute including rigid-body additional mass properties.
pub fn recompute_body_mass_properties(ref body: RigidBody, ref colliders: ColliderSet) {
    let mut local: MassProperties = Default::default();
    let mut unit: MassProperties = Default::default();
    for co_handle in body.colliders {
        if let Some(collider) = colliders.get(*co_handle) {
            if collider.is_enabled() {
                if let Some(parent) = collider.parent {
                    local = local + collider.mass_properties().transform_by(parent.pos_wrt_parent);
                    unit = unit
                        + collider.shape.mass_properties(ONE).transform_by(parent.pos_wrt_parent);
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
    body.mprops = body.mprops.update_world_mass_properties(body.body_type, body.pos.position);
}

/// Moves the colliders of `body` to follow its pose. Out of line: it holds the per-collider
/// loop, which the caller only enters for moved bodies.
#[inline(never)]
fn propagate_positions(body: RigidBody, ref colliders: ColliderSet) {
    let mut attached = body.colliders;
    while let Some(co_handle) = attached.pop_front() {
        if let Some(mut collider) = colliders.get(*co_handle) {
            if let Some(pos_wrt_parent) = collider.position_wrt_parent() {
                collider.pos.pose = body.pos.position * pos_wrt_parent;
                collider.changes = collider.changes | COLLIDER_POSITION;
                colliders.set(*co_handle, collider);
            }
        }
    }
}

#[cfg(test)]
mod tests;
