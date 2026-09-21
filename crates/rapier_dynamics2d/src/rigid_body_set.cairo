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
//!   package): it removes or detaches the attached colliders only.

use fixed::Fixed;
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::changes::{PARENT, POSITION as COLLIDER_POSITION};
use rapier_core::data::arena::{Arena, ArenaTrait};
use rapier_core::rigid_body::changes::{COLLIDERS, POSITION};
use rapier_core::rigid_body::{
    RigidBodyActivation, RigidBodyActivationTrait, RigidBodyChanges, RigidBodyChangesTrait,
    RigidBodyDamping, RigidBodyDominance, RigidBodyType, RigidBodyTypeTrait,
};
use rapier_geometry2d::mass::MassPropertiesTrait;
use rapier_math::pose2::Pose2;
use crate::collider::{Collider, ColliderTrait};
use crate::collider_set::{ColliderSet, ColliderSetTrait};
use crate::rigid_body::{
    LockedAxesTrait, RigidBodyForces, RigidBodyMassProps, RigidBodyMassPropsTrait,
    RigidBodyPosition, RigidBodyPositionTrait, RigidBodyVelocity, RigidBodyVelocityTrait,
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
    /// pose, and refreshes the world centre of mass. Upstream `set_position(pos, wake_up)` minus
    /// the wake-up (islands are deferred).
    fn set_position(ref self: RigidBody, position: Pose2) {
        if self.pos.position != position {
            self.changes.insert(POSITION);
        }
        self.pos = RigidBodyPositionTrait::from_position(position);
        self.mprops = self.mprops.update_world_mass_properties(self.body_type, position);
    }

    /// Linear velocity.
    #[inline(always)]
    fn linvel(self: @RigidBody) -> Vec2 {
        *self.vels.linvel
    }

    /// Replaces the linear velocity (upstream `set_linvel` minus the wake-up).
    #[inline(always)]
    fn set_linvel(ref self: RigidBody, linvel: Vec2) {
        self.vels = RigidBodyVelocityTrait::new(linvel, self.vels.angvel);
    }

    /// Replaces the angular velocity (upstream `set_angvel` minus the wake-up).
    #[inline(always)]
    fn set_angvel(ref self: RigidBody, angvel: Fixed) {
        self.vels = RigidBodyVelocityTrait::new(self.vels.linvel, angvel);
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
    use rapier_core::rigid_body::changes::POSITION;
    use rapier_core::rigid_body::{RigidBodyChangesTrait, RigidBodyType};
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
    fn test_constructors() {
        // (body, type, dynamic, fixed, kinematic)
        let cases: Array<(RigidBody, RigidBodyType, bool, bool, bool)> = array![
            (RigidBodyTrait::dynamic(at(ONE, TWO)), RigidBodyType::Dynamic, true, false, false),
            (RigidBodyTrait::fixed(at(ONE, TWO)), RigidBodyType::Fixed, false, true, false),
            (
                RigidBodyTrait::kinematic_position_based(at(ONE, TWO)),
                RigidBodyType::KinematicPositionBased,
                false,
                false,
                true,
            ),
        ];
        for (body, body_type, dynamic, fixed, kinematic) in cases {
            assert_eq!(body.body_type, body_type);
            assert_eq!(body.is_dynamic(), dynamic);
            assert_eq!(body.is_fixed(), fixed);
            assert_eq!(body.is_kinematic(), kinematic);
            assert_eq!(body.position(), at(ONE, TWO));
            assert_eq!(body.world_com(), Vec2 { x: ONE, y: TWO });
            assert!(body.changes.is_empty());
            assert_eq!(body.colliders.len(), 0);
            assert!(body.enabled);
        }
    }

    #[test]
    fn test_setters() {
        let mut body = RigidBodyTrait::dynamic(at(ZERO, ZERO));
        body.set_position(at(ZERO, ZERO));
        assert!(!body.changes.contains(POSITION));
        body.set_position(at(ONE, ZERO));
        assert!(body.changes.contains(POSITION));
        assert_eq!(body.pos.next_position, at(ONE, ZERO));
        assert_eq!(body.world_com(), Vec2 { x: ONE, y: ZERO });
        body.set_linvel(Vec2 { x: TWO, y: ONE });
        body.set_angvel(ONE);
        assert_eq!(body.linvel(), Vec2 { x: TWO, y: ONE });
        assert_eq!(body.vels.angvel, ONE);
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
}
