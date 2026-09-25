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

use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_core::Handle;
use rapier_core::collider::changes::{PARENT, POSITION as COLLIDER_POSITION};
use rapier_core::data::arena::{Arena, ArenaTrait};
use rapier_core::rigid_body::changes::{COLLIDERS, DOMINANCE, POSITION, SLEEP};
use rapier_core::rigid_body::{
    RigidBodyActivation, RigidBodyActivationTrait, RigidBodyChanges, RigidBodyChangesTrait,
    RigidBodyDamping, RigidBodyDominance, RigidBodyType, RigidBodyTypeTrait,
};
use rapier_geometry2d::mass::MassPropertiesTrait;
use rapier_math::math_ext::vec2::gcross_vv;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use crate::collider::{Collider, ColliderTrait};
use crate::collider_set::{ColliderSet, ColliderSetTrait};
use crate::rigid_body::{
    LockedAxesTrait, RigidBodyForces, RigidBodyForcesTrait, RigidBodyMassProps,
    RigidBodyMassPropsTrait, RigidBodyPosition, RigidBodyPositionTrait, RigidBodyVelocity,
    RigidBodyVelocityTrait,
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
    pub user_data: u128,
}

/// Constructors, getters and setters of [`RigidBody`] (upstream names).
#[generate_trait]
pub impl RigidBodyImpl of RigidBodyTrait {
    /// A body of type `body_type` at `position`, at rest, with no collider, no mass (the
    /// colliders bring it), zero damping, unit gravity scale, awake, dominance group 0, enabled
    /// and no change flag (the set raises them all on `insert`). Upstream `RigidBodyBuilder`
    /// defaults.
    fn new(body_type: RigidBodyType, position: Pose2) -> RigidBody {
        RigidBody {
            pos: RigidBodyPositionTrait::from_position(position),
            mprops: RigidBodyMassPropsTrait::from_local(
                Default::default(), LockedAxesTrait::empty(),
            )
                .update_world_mass_properties(body_type, position),
            vels: Default::default(),
            damping: Default::default(),
            forces: Default::default(),
            colliders: array![].span(),
            activation: RigidBodyActivationTrait::active(),
            changes: RigidBodyChangesTrait::empty(),
            body_type,
            dominance: Default::default(),
            enabled: true,
            user_data: 0,
        }
    }

    /// `new(Dynamic, position)`.
    #[inline(always)]
    fn dynamic(position: Pose2) -> RigidBody {
        Self::new(RigidBodyType::Dynamic, position)
    }

    /// `new(Fixed, position)`.
    #[inline(always)]
    fn fixed(position: Pose2) -> RigidBody {
        Self::new(RigidBodyType::Fixed, position)
    }

    /// `new(KinematicPositionBased, position)`.
    #[inline(always)]
    fn kinematic_position_based(position: Pose2) -> RigidBody {
        Self::new(RigidBodyType::KinematicPositionBased, position)
    }

    /// `new(KinematicVelocityBased, position)`: velocities drive motion, forces and
    /// contact impulses cannot change it. Exact initialization; no rounding or panic.
    fn kinematic_velocity_based(position: Pose2) -> RigidBody {
        Self::new(RigidBodyType::KinematicVelocityBased, position)
    }

    /// Target pose (upstream `next_position`), exact copy.
    fn next_position(self: @RigidBody) -> Pose2 {
        *self.pos.next_position
    }

    /// Sets a kinematic body's target, waking it strongly iff different from the current
    /// pose. Other body types are untouched. Exact copy; rotation must be unit.
    /// As upstream, accepts both kinematic types; velocity-based integration replaces it.
    fn set_next_kinematic_position(ref self: RigidBody, position: Pose2) {
        if self.is_kinematic() {
            self.pos.next_position = position;
            if self.pos.position != position {
                self.wake_up(true);
            }
        }
    }

    /// Replaces only target translation; same type and wake rules as the pose setter.
    /// Values must fit Q32.32; exact copy, no arithmetic or panic.
    fn set_next_kinematic_translation(ref self: RigidBody, translation: Vec2) {
        if self.is_kinematic() {
            self.pos.next_position.translation = translation;
            if self.pos.position.translation != translation {
                self.wake_up(true);
            }
        }
    }

    /// Replaces only target rotation (unit complex); same type and wake rules as upstream.
    /// Exact copy, no normalization, arithmetic or panic.
    fn set_next_kinematic_rotation(ref self: RigidBody, rotation: Rot2) {
        if self.is_kinematic() {
            self.pos.next_position.rotation = rotation;
            if self.pos.position.rotation != rotation {
                self.wake_up(true);
            }
        }
    }

    /// Configured dominance group, exactly in [-128, 127]. Fixed effective dominance is 128.
    fn dominance_group(self: @RigidBody) -> i8 {
        *self.dominance.group
    }

    /// Replaces the signed group, raising DOMINANCE iff changed. No direct wake-up, as
    /// upstream; the pipeline handles affected contacts. Exact, no rounding or panic.
    fn set_dominance_group(ref self: RigidBody, group: i8) {
        if self.dominance.group != group {
            self.dominance.group = group;
            self.changes.insert(DOMINANCE);
        }
    }

    /// World pose of the body frame.
    #[inline(always)]
    fn position(self: @RigidBody) -> Pose2 {
        *self.pos.position
    }

    /// World centre of mass (refreshed by `update_world_mass_properties`).
    #[inline(always)]
    fn world_com(self: @RigidBody) -> Vec2 {
        *self.mprops.world_com
    }

    #[inline(always)]
    fn is_dynamic(self: @RigidBody) -> bool {
        (*self.body_type).is_dynamic()
    }

    #[inline(always)]
    fn is_fixed(self: @RigidBody) -> bool {
        (*self.body_type).is_fixed()
    }

    #[inline(always)]
    fn is_kinematic(self: @RigidBody) -> bool {
        (*self.body_type).is_kinematic()
    }

    /// Moves the body (and, after `propagate_modified_body_positions_to_colliders`, its
    /// colliders). Raises `POSITION` when the pose changes; sets both the current and the next
    /// pose, refreshes the world centre of mass and wakes the body up (upstream
    /// `set_position(pos, wake_up = true)`).
    fn set_position(ref self: RigidBody, position: Pose2) {
        if self.pos.position != position {
            self.changes.insert(POSITION);
        }
        self.pos = RigidBodyPositionTrait::from_position(position);
        self.mprops = self.mprops.update_world_mass_properties(self.body_type, position);
        self.wake_up(true);
    }

    /// Linear velocity.
    #[inline(always)]
    fn linvel(self: @RigidBody) -> Vec2 {
        *self.vels.linvel
    }

    /// Replaces the linear velocity and wakes the body up (upstream `set_linvel(linvel, wake_up
    /// = true)`, dynamic and velocity-based kinematic bodies only).
    #[inline(always)]
    fn set_linvel(ref self: RigidBody, linvel: Vec2) {
        if (self.body_type == RigidBodyType::Dynamic
            || self.body_type == RigidBodyType::KinematicVelocityBased)
            && self.vels.linvel != linvel {
            self.vels.linvel = linvel;
            self.wake_up(true);
        }
    }

    /// Replaces the angular velocity and wakes the body up (upstream `set_angvel(angvel, wake_up
    /// = true)`, dynamic and velocity-based kinematic bodies only).
    #[inline(always)]
    fn set_angvel(ref self: RigidBody, angvel: Fixed) {
        if (self.body_type == RigidBodyType::Dynamic
            || self.body_type == RigidBodyType::KinematicVelocityBased)
            && self.vels.angvel != angvel {
            self.vels.angvel = angvel;
            self.wake_up(true);
        }
    }

    /// Is the body asleep (upstream `is_sleeping`)? A sleeping body keeps its pose, has zero
    /// velocities and is skipped by the step until something wakes it up.
    #[inline(always)]
    fn is_sleeping(self: @RigidBody) -> bool {
        *self.activation.sleeping
    }

    /// Wakes the body up (upstream `RigidBody::wake_up`): raises `SLEEP` when it was asleep and
    /// clears the sleeping flag; a `strong` wake-up also resets the still-time counter, so that
    /// the body cannot fall asleep again for `time_until_sleep`. The step then wakes the whole
    /// island of the body (touching bodies, jointed bodies), as upstream's island manager does.
    #[inline(always)]
    fn wake_up(ref self: RigidBody, strong: bool) {
        if self.activation.sleeping {
            self.changes.insert(SLEEP);
        }
        self.activation.wake_up(strong);
    }

    /// Puts the body to sleep (upstream `RigidBody::sleep`): sleeping flag set, still-time
    /// counter filled, both velocities zeroed. The step wakes it again at once when its island
    /// has an awake body (upstream keeps such a body in the awake island).
    #[inline(always)]
    fn sleep(ref self: RigidBody) {
        self.activation.sleep();
        self.vels = RigidBodyVelocityTrait::zero();
    }

    /// Adds `force` to the user force of a dynamic body (upstream `add_force`): kept across
    /// steps until `reset_forces`. Nothing happens for a zero force or a non-dynamic body; the
    /// body is woken up (strongly) when `wake_up` and the force was added.
    fn add_force(ref self: RigidBody, force: Vec2, wake_up: bool) {
        if force != Vec2Trait::ZERO && self.body_type.is_dynamic() {
            self.forces = self.forces.add_force(force);
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Adds `torque` to the user torque of a dynamic body (upstream `add_torque`); as
    /// [`add_force`](RigidBodyTrait::add_force) for the zero and body-type conditions.
    fn add_torque(ref self: RigidBody, torque: Fixed, wake_up: bool) {
        if torque != ZERO && self.body_type.is_dynamic() {
            self.forces = self.forces.add_torque(torque);
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Adds `force` applied at the world `point` (upstream `add_force_at_point`): the force and
    /// its torque about the world centre of mass; as [`add_force`](RigidBodyTrait::add_force)
    /// otherwise.
    fn add_force_at_point(ref self: RigidBody, force: Vec2, point: Vec2, wake_up: bool) {
        if force != Vec2Trait::ZERO && self.body_type.is_dynamic() {
            self.forces = self.forces.add_force_at_point(self.mprops, force, point);
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Clears the user force (upstream `reset_forces`); wakes the body up when `wake_up` and
    /// the force was not zero already.
    fn reset_forces(ref self: RigidBody, wake_up: bool) {
        if self.forces.user_force != Vec2Trait::ZERO {
            self.forces = self.forces.reset_forces();
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Clears the user torque (upstream `reset_torques`); wakes the body up when `wake_up` and
    /// the torque was not zero already.
    fn reset_torques(ref self: RigidBody, wake_up: bool) {
        if self.forces.user_torque != ZERO {
            self.forces = self.forces.reset_torques();
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies a linear impulse at the centre of mass of a dynamic body (upstream
    /// `apply_impulse`): `linvel += impulse * effective_inv_mass`, each component floored once.
    /// Nothing happens for a zero impulse or a non-dynamic body; the body is woken up (strongly)
    /// when `wake_up` and the impulse was applied.
    fn apply_impulse(ref self: RigidBody, impulse: Vec2, wake_up: bool) {
        if impulse != Vec2Trait::ZERO && self.body_type.is_dynamic() {
            self.vels.linvel = self.vels.linvel + impulse * self.mprops.effective_inv_mass;
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies an angular impulse to a dynamic body (upstream `apply_torque_impulse`): `angvel
    /// += effective_world_inv_inertia * torque_impulse`, floored once; as
    /// [`apply_impulse`](RigidBodyTrait::apply_impulse) for the conditions.
    fn apply_torque_impulse(ref self: RigidBody, torque_impulse: Fixed, wake_up: bool) {
        if torque_impulse != ZERO && self.body_type.is_dynamic() {
            self.vels.angvel += self.mprops.effective_world_inv_inertia * torque_impulse;
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies `impulse` at the world `point` (upstream `apply_impulse_at_point`): the linear
    /// impulse, then the angular impulse `(point - world_com) x impulse`, each through the
    /// helpers above (and their conditions).
    fn apply_impulse_at_point(ref self: RigidBody, impulse: Vec2, point: Vec2, wake_up: bool) {
        let dpt = point - self.mprops.world_com;
        let torque_impulse = gcross_vv(dpt.x, dpt.y, impulse.x, impulse.y);
        self.apply_impulse(impulse, wake_up);
        self.apply_torque_impulse(torque_impulse, wake_up);
    }
}

/// Upstream-named body builder. Unexposed builder options remain configurable on `build()`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RigidBodyBuilder {
    body: RigidBody,
}
#[generate_trait]
pub impl RigidBodyBuilderImpl of RigidBodyBuilderTrait {
    /// Default body at identity with the supplied type; exact initialization.
    fn new(body_type: RigidBodyType) -> RigidBodyBuilder {
        RigidBodyBuilder { body: RigidBodyTrait::new(body_type, Default::default()) }
    }
    /// Dynamic body at identity, upstream defaults.
    fn dynamic() -> RigidBodyBuilder {
        Self::new(RigidBodyType::Dynamic)
    }
    /// Fixed body at identity, upstream defaults.
    fn fixed() -> RigidBodyBuilder {
        Self::new(RigidBodyType::Fixed)
    }
    /// Position-controlled body at identity, upstream defaults.
    fn kinematic_position_based() -> RigidBodyBuilder {
        Self::new(RigidBodyType::KinematicPositionBased)
    }
    /// Velocity-controlled body at identity, upstream defaults.
    fn kinematic_velocity_based() -> RigidBodyBuilder {
        Self::new(RigidBodyType::KinematicVelocityBased)
    }
    /// Initial pose, unit rotation required. Refreshes COM; fixed arithmetic overflow panics.
    fn position(mut self: RigidBodyBuilder, position: Pose2) -> RigidBodyBuilder {
        self.body.pos = RigidBodyPositionTrait::from_position(position);
        self
            .body
            .mprops = self
            .body
            .mprops
            .update_world_mass_properties(self.body.body_type, position);
        self
    }
    /// Initial configured dominance, exactly in [-128,127]; no rounding or panic.
    fn dominance_group(mut self: RigidBodyBuilder, group: i8) -> RigidBodyBuilder {
        self.body.dominance.group = group;
        self
    }
    /// Builds the configured body, an exact copy.
    fn build(self: RigidBodyBuilder) -> RigidBody {
        self.body
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
}

/// Upstream `RigidBodyColliders::attach_collider`: appends `co_handle` to the collider list of
/// the body behind `handle`, raises `COLLIDERS`, adds the collider's mass (expressed in the body
/// frame through `pos_wrt_parent`) to the body's and refreshes its world mass properties.
/// Returns the body's world pose, which the caller composes with `pos_wrt_parent`.
///
/// # Panics
/// `RigidBodySet: body not found` when `handle` does not resolve.
pub(crate) fn attach_collider(
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

/// Upstream `remove_collider_internal`: removes `co_handle` from the collider list of the body
/// behind `handle` with upstream's `swap_remove` (the last handle takes its place) and raises
/// `COLLIDERS`. As upstream, the mass properties are left for the pipeline to recompute. Does
/// nothing when the body or the collider is not found.
pub(crate) fn detach_collider(ref bodies: RigidBodySet, handle: Handle, co_handle: Handle) {
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
mod tests {
    use fixed::{Fixed, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_core::Handle;
    use rapier_core::rigid_body::RigidBodyChangesTrait;
    use rapier_math::pose2::Pose2;
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::collider::ColliderBuilderTrait;
    use crate::collider_set::{ColliderSet, ColliderSetTrait};
    use super::{RigidBody, RigidBodySet, RigidBodySetTrait, RigidBodyTrait};

    fn at(x: Fixed, y: Fixed) -> Pose2 {
        Pose2 { translation: Vec2 { x, y }, rotation: Rot2 { re: ONE, im: ZERO } }
    }

    #[test]
    fn test_set_lifecycle() {
        let mut bodies = RigidBodySetTrait::new();
        let mut colliders: ColliderSet = ColliderSetTrait::new();
        let mut body = RigidBodyTrait::dynamic(at(ZERO, ZERO));
        body.colliders = array![Handle { index: 9, generation: 9 }].span();
        let h0 = bodies.insert(body);
        let h1 = bodies.insert(RigidBodyTrait::fixed(at(ONE, ZERO)));
        assert_eq!(h0, Handle { index: 0, generation: 0 });
        assert_eq!(h1, Handle { index: 1, generation: 0 });
        // `insert` resets the collider list and raises every change flag.
        let stored = bodies.get(h0).unwrap();
        assert_eq!(stored.colliders.len(), 0);
        assert_eq!(stored.changes, RigidBodyChangesTrait::all());
        assert_eq!(bodies.len(), 2);
        let mut moved = stored;
        moved.set_linvel(Vec2 { x: ONE, y: ONE });
        assert!(bodies.set(h0, moved));
        assert_eq!(bodies.get(h0).unwrap().linvel(), Vec2 { x: ONE, y: ONE });
        assert!(bodies.remove(h0, ref colliders, true).is_some());
        assert!(!bodies.contains(h0));
        assert!(!bodies.set(h0, moved));
        assert_eq!(bodies.get(h0), None);
        // Slot 0 is reused with the bumped generation; iteration is by slot index.
        let h2 = bodies.insert(RigidBodyTrait::dynamic(at(TWO, ZERO)));
        assert_eq!(h2, Handle { index: 0, generation: 1 });
        let all = bodies.iter();
        assert_eq!(all.len(), 2);
        let (first, _) = *all.at(0);
        let (second, _) = *all.at(1);
        assert_eq!(first, h2);
        assert_eq!(second, h1);
        assert!(!bodies.is_empty());
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }

    #[test]
    fn gas_body_new() {
        let _ = RigidBodyTrait::dynamic(opaque(at(ONE, ZERO)));
    }

    #[test]
    fn gas_insert() {
        let mut bodies = RigidBodySetTrait::new();
        let _ = bodies.insert(opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO))));
    }

    #[test]
    fn gas_get_set() {
        let mut bodies = RigidBodySetTrait::new();
        let h = bodies.insert(opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO))));
        let body = bodies.get(opaque(h)).unwrap();
        let _ = bodies.set(h, body);
    }

    #[test]
    fn gas_remove() {
        let mut bodies = RigidBodySetTrait::new();
        let mut colliders = ColliderSetTrait::new();
        let h = bodies.insert(opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO))));
        let _ = bodies.remove(opaque(h), ref colliders, true);
    }

    #[test]
    fn gas_iter_8() {
        let mut bodies = RigidBodySetTrait::new();
        let body = opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
        let mut i: u32 = 0;
        while i != 8 {
            let _ = bodies.insert(body);
            i += 1;
        }
        let _ = bodies.iter();
    }

    /// Eight moved bodies with one collider each.
    #[test]
    fn gas_propagate_modified_body_positions_8() {
        let mut bodies: RigidBodySet = RigidBodySetTrait::new();
        let mut colliders = ColliderSetTrait::new();
        let body = opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
        let collider = opaque(ColliderBuilderTrait::ball(ONE).build());
        let mut i: u32 = 0;
        while i != 8 {
            let h = bodies.insert(body);
            let _ = colliders.insert_with_parent(collider, h, ref bodies);
            i += 1;
        }
        bodies.propagate_modified_body_positions_to_colliders(ref colliders);
    }

    /// Sleep helpers on a sleeping dynamic body (file budget: one probe per family;
    /// `gas_<family>` − `gas_sleeping_body`).
    #[inline(never)]
    fn sleeping_body() -> RigidBody {
        let mut body = RigidBodyTrait::dynamic(opaque(at(ZERO, ZERO)));
        body.sleep();
        body
    }

    #[test]
    fn gas_sleeping_body() {
        let _ = sleeping_body();
    }

    /// `is_sleeping` then a strong `wake_up`.
    #[test]
    fn gas_wake_up() {
        let mut body = sleeping_body();
        let _ = opaque(body.is_sleeping());
        body.wake_up(opaque(true));
    }

    /// `add_force`, `add_torque`, `add_force_at_point`, `reset_forces`, `reset_torques`.
    #[test]
    fn gas_forces() {
        let mut body = sleeping_body();
        let f = opaque(Vec2 { x: ONE, y: ZERO });
        body.add_force(f, true);
        body.add_torque(ONE, true);
        body.add_force_at_point(f, Vec2 { x: ZERO, y: ONE }, true);
        body.reset_forces(true);
        body.reset_torques(true);
    }

    /// `apply_impulse`, `apply_torque_impulse`, `apply_impulse_at_point`.
    #[test]
    fn gas_impulses() {
        let mut body = sleeping_body();
        let i = opaque(Vec2 { x: ONE, y: ZERO });
        body.apply_impulse(i, true);
        body.apply_torque_impulse(ONE, true);
        body.apply_impulse_at_point(i, Vec2 { x: ZERO, y: ONE }, true);
    }
}
