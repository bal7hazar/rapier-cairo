//! `World`: everything one simulation owns, bundled (upstream `pipeline/physics_world.rs`,
//! `PhysicsWorld`).
//!
//! The persistent state is exactly `docs/PLAN.md` D9: the body and collider sets (poses,
//! velocities, mass properties, change flags), the impulse joints (with their accumulated
//! impulses) and the narrow-phase pairs (manifolds carrying the warm-start impulses and the event
//! status; sensor pairs with their `intersecting` state), plus (BT2, D7 / D9 amended) the
//! step's active set (`crate::pipeline::active_set`: the awake bodies, the static broad-phase
//! proxies and the positions of the live pairs, so that a step walks the awake bodies only).
//! Everything else [`WorldTrait::step`] needs (the other broad-phase proxies and pairs, solver
//! bodies, constraints) is rebuilt every step and dropped with it. [`WorldTrait::to_state`] /
//! [`WorldTrait::from_state`] save and restore it (`state`, versioned).
//!
//! Mutations go through the sets' own setters, which raise the change flags the next step reads
//! (`RigidBodyTrait::set_position`, `ColliderTrait::set_shape`, …): read a copy with
//! [`WorldTrait::body`] / [`WorldTrait::collider`], modify it, write it back with
//! [`WorldTrait::set_body`] / [`WorldTrait::set_collider`].
//!
//! Sleeping (work package SL): bodies fall asleep and wake up island by island inside `step`
//! (`crate::pipeline::islands`); as upstream's `PhysicsWorld`, inserting or removing a joint
//! wakes both of its bodies up, removing a body or a collider wakes up every body it was in
//! contact with (not its sensor partners: its intersection pairs end at the next step, dormant
//! or not), and a body woken up by hand (`RigidBodyTrait::wake_up`, the setters, forces
//! and impulses) written back with [`WorldTrait::set_body`] wakes its island at the next step.
//! The island manager's view is derived from the bodies' sleep state:
//! [`WorldTrait::active_bodies`] / [`WorldTrait::num_active_bodies`] (no per-step bookkeeping).
//!
//! Deviations from upstream: no persistent island manager (islands are rebuilt when an awake
//! body can fall asleep or touches a sleeping one), no broad-phase tree (the active set keeps the
//! static proxies), no CCD solver, multibody or soft body sets, query pipeline (the scene queries
//! scan the collider set, `crate::queries`), hooks or event handler (events are returned by
//! `step` and `step_with_force_events`, the counterpart of `step_with_events`), thread pool or
//! quarantine (Q32.32 state cannot become non-finite: an overflow panics). Sets are read by
//! copy: [`WorldTrait::rigid_bodies`] / [`WorldTrait::all_colliders`] also stand for upstream's
//! `_mut` iterators (write changes back with `set_body` / `set_collider`).

use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::{CollisionEvent, ContactForceEvent};
use rapier_dynamics2d::joint::{GenericJoint, ImpulseJoint, ImpulseJointSet, ImpulseJointSetTrait};
use rapier_dynamics2d::narrow_phase::{ContactPair, NarrowPhase, NarrowPhaseTrait};
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use rapier_geometry2d::aabb::Aabb;
use rapier_geometry2d::point::PointProjection;
use rapier_geometry2d::ray::{Ray, RayIntersection};
use crate::pipeline::active_set::ActiveSet;
use crate::queries::QueryFilter;

/// Versioned save / restore ([`WorldTrait::to_state`], [`WorldTrait::from_state`]).
pub mod state;
use state::WorldState;

/// A 2D physics world (upstream `PhysicsWorld`). Holds dicts: pass it by `ref`.
#[derive(Destruct)]
pub struct World {
    /// Gravity applied to every dynamic body (scaled by its `gravity_scale`).
    pub gravity: Vec2,
    /// Timestep, solver iterations, softness and length scales of every step.
    pub integration_parameters: IntegrationParameters,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    /// Last step's contact pairs (ascending collider slot), with their warm-start impulses, and
    /// its sensor intersection pairs.
    pub narrow_phase: NarrowPhase,
    /// What the next step needs to skip the sleeping bodies (BT2, `pipeline::active_set`),
    /// maintained by the step: only trusted while neither set was written since. Boxed: the
    /// world is passed by reference, one cell instead of the whole set.
    pub active_set: Box<ActiveSet>,
}

/// Upstream's name of the world.
pub type PhysicsWorld = World;

/// Upstream `PhysicsWorld::default`: gravity `(0, -9.81)` (nearest Q32.32) and the default
/// integration parameters.
pub impl WorldDefault of Default<World> {
    fn default() -> World {
        WorldTrait::new(Vec2 { x: ZERO, y: DEFAULT_GRAVITY_Y }, Default::default())
    }
}

/// `-9.81` in Q32.32, rounded to nearest (upstream's default gravity).
pub const DEFAULT_GRAVITY_Y: Fixed = Fixed { raw: -42133629174 };

/// Operations of [`World`] (upstream `PhysicsWorld` names).
#[generate_trait]
pub impl WorldImpl of WorldTrait {
    /// An empty world with the given gravity and parameters (upstream `PhysicsWorld::new` plus
    /// the two public fields).
    fn new(gravity: Vec2, integration_parameters: IntegrationParameters) -> World {
        World {
            gravity,
            integration_parameters,
            bodies: RigidBodySetTrait::new(),
            colliders: ColliderSetTrait::new(),
            impulse_joints: ImpulseJointSetTrait::new(),
            narrow_phase: NarrowPhaseTrait::new(),
            active_set: BoxTrait::new(Default::default()),
        }
    }

    /// Stores a body without collider and returns its handle. Every change flag is raised: the
    /// next step propagates its pose and recomputes its mass from its colliders.
    fn insert_body(ref self: World, body: RigidBody) -> Handle {
        self.bodies.insert(body)
    }

    /// Stores a collider attached to `parent` (its pose is then relative to the body), or a
    /// standalone collider for `None` (its pose is a world pose), and returns its handle.
    ///
    /// # Panics
    /// `RigidBodySet: body not found` when `parent` does not resolve.
    fn insert_collider(ref self: World, collider: Collider, parent: Option<Handle>) -> Handle {
        match parent {
            Some(parent) => self.colliders.insert_with_parent(collider, parent, ref self.bodies),
            None => self.colliders.insert(collider),
        }
    }

    /// Stores `body` and one collider attached to it (upstream `PhysicsWorld::insert`).
    fn insert(ref self: World, body: RigidBody, collider: Collider) -> (Handle, Handle) {
        let body = self.bodies.insert(body);
        let collider = self.colliders.insert_with_parent(collider, body, ref self.bodies);
        (body, collider)
    }

    /// Stores an impulse joint between two bodies, with zero accumulated impulses, and wakes
    /// both bodies up (upstream `ImpulseJointSet::insert(.., wake_up = true)`). Frames are in
    /// the local frames of the bodies.
    fn insert_impulse_joint(
        ref self: World, body1: Handle, body2: Handle, joint: GenericJoint,
    ) -> Handle {
        self.wake_up(body1);
        self.wake_up(body2);
        self.impulse_joints.insert(body1, body2, joint)
    }

    /// Removes a body, its colliders and its joints (upstream `PhysicsWorld::remove_body`).
    /// `None` when the handle does not resolve. The bodies in contact with the removed colliders
    /// and those its joints linked are woken up; the contact pairs of the removed colliders end
    /// at the next step, with a `Stopped` event flagged `REMOVED` if their `Started` was emitted.
    fn remove_body(ref self: World, handle: Handle) -> Option<RigidBody> {
        let body = self.bodies.get(handle)?;
        for co_handle in body.colliders {
            self.wake_contact_partners(*co_handle);
        }
        let body = self.bodies.remove(handle, ref self.colliders, true)?;
        for (joint_handle, joint) in self.impulse_joints.to_array() {
            if joint.body1 == handle || joint.body2 == handle {
                let _ = self.impulse_joints.remove(joint_handle);
                self.wake_up(if joint.body1 == handle {
                    joint.body2
                } else {
                    joint.body1
                });
            }
        }
        Some(body)
    }

    /// Removes a collider (upstream `PhysicsWorld::remove_collider`); `None` when the handle
    /// does not resolve. Every body in contact with it is woken up (its parent included, which
    /// the next step's user changes also wake as its collider list changed); its sensor partners
    /// are not, as upstream. The next step recomputes its parent's mass and ends its contact and
    /// sensor pairs, dormant or not (see [`WorldTrait::remove_body`]; a sensor pair ends with
    /// `Stopped` flagged `SENSOR | REMOVED` if its `Started` was emitted).
    fn remove_collider(ref self: World, handle: Handle) -> Option<Collider> {
        self.wake_contact_partners(handle);
        self.colliders.remove(handle, ref self.bodies)
    }

    /// Removes an impulse joint and returns its data, waking both of its bodies up (upstream
    /// `remove(.., wake_up = true)`); `None` when the handle does not resolve.
    fn remove_impulse_joint(ref self: World, handle: Handle) -> Option<GenericJoint> {
        let joint = self.impulse_joints.remove(handle)?;
        self.wake_up(joint.body1);
        self.wake_up(joint.body2);
        Some(joint.data)
    }

    /// Wakes up the non-fixed body behind `handle`, strongly (upstream `IslandManager::wake_up
    /// (.., strong = true)`); nothing for a fixed or missing body. The step wakes its island.
    fn wake_up(ref self: World, handle: Handle) {
        if let Some(mut body) = self.bodies.get(handle) {
            if !body.is_fixed() {
                body.wake_up(true);
                let _ = self.bodies.set(handle, body);
            }
        }
    }

    /// Before `collider` goes: wakes up the parents of both colliders of every contact pair of
    /// `collider` (upstream `NarrowPhase::remove_collider`: the contact graph only; its
    /// intersection pairs wake nobody), and clears the body links of all its pairs
    /// (`pipeline::sleeping::release_removed_pairs`) so that the next step ends them, dormant or
    /// not, with their `REMOVED` event.
    fn wake_contact_partners(ref self: World, collider: Handle) {
        let mut touched = array![collider];
        let _ = crate::pipeline::sleeping::wake_touched_partners(
            touched.span(), self.narrow_phase.pairs.span(), ref self.bodies, ref self.colliders,
        );
        self
            .narrow_phase
            .pairs =
                crate::pipeline::sleeping::release_removed_pairs(
                    self.narrow_phase.pairs.span(), collider,
                );
    }

    /// Wakes up every non-fixed body (upstream `wake_up_all`, `IslandManager::wake_up` on each):
    /// a sleeping body wakes up strongly (upstream wakes its whole sleeping island with a strong
    /// timer reset), an awake one resets its sleep timer only when `strong`. Fixed bodies are
    /// skipped. Cost: one read per body, one write per woken body.
    fn wake_up_all(ref self: World, strong: bool) {
        for (handle, body) in self.bodies.iter() {
            if !body.is_fixed()
                && (body.activation.sleeping
                    || (strong && body.activation.time_since_can_sleep != ZERO)) {
                let mut body = body;
                body.wake_up(true);
                let _ = self.bodies.set(handle, body);
            }
        }
    }

    /// Every awake island member as `(handle, body)`, ascending handle order (upstream
    /// `active_bodies`, `IslandManager::active_bodies`): the enabled dynamic and kinematic bodies
    /// that do not sleep. Derived from the sleep flags, as of the last step and the wake-ups
    /// since. Cost: one read per body.
    fn active_bodies(ref self: World) -> Array<(Handle, RigidBody)> {
        let mut out = array![];
        for (handle, body) in self.bodies.iter() {
            if is_active(@body) {
                out.append((handle, body));
            }
        }
        out
    }

    /// The number of [`WorldTrait::active_bodies`] (upstream `IslandManager::
    /// num_active_bodies`), without building the array.
    fn num_active_bodies(ref self: World) -> u32 {
        let mut count = 0;
        for (_, body) in self.bodies.iter() {
            if is_active(@body) {
                count += 1;
            }
        }
        count
    }

    /// Every `(handle, body)`, ascending handle order (upstream `rigid_bodies` and, by copy,
    /// `rigid_bodies_mut`: write changes back with [`WorldTrait::set_body`]).
    #[inline(always)]
    fn rigid_bodies(ref self: World) -> Array<(Handle, RigidBody)> {
        self.bodies.iter()
    }

    /// Every `(handle, collider)`, ascending handle order (upstream `all_colliders` and, by
    /// copy, `all_colliders_mut`: write changes back with [`WorldTrait::set_collider`]).
    #[inline(always)]
    fn all_colliders(ref self: World) -> Array<(Handle, Collider)> {
        self.colliders.iter()
    }

    /// A copy of the body behind `handle`.
    #[inline(always)]
    fn body(ref self: World, handle: Handle) -> Option<RigidBody> {
        self.bodies.get(handle)
    }

    /// Whether the body behind `handle` sleeps (upstream `RigidBody::is_sleeping` through
    /// `bodies.get`); `None` when the handle does not resolve. Reads the body out of the set
    /// without returning it (BT2 addendum B).
    fn is_sleeping(ref self: World, handle: Handle) -> Option<bool> {
        match self.bodies.get(handle) {
            Some(body) => Some(body.activation.sleeping),
            None => None,
        }
    }

    /// The linear velocity of the body behind `handle` (upstream `RigidBody::linvel`); `None`
    /// when the handle does not resolve.
    fn linvel(ref self: World, handle: Handle) -> Option<Vec2> {
        match self.bodies.get(handle) {
            Some(body) => Some(body.vels.linvel),
            None => None,
        }
    }

    /// The angular velocity of the body behind `handle` (upstream `RigidBody::angvel`); `None`
    /// when the handle does not resolve.
    fn angvel(ref self: World, handle: Handle) -> Option<Fixed> {
        match self.bodies.get(handle) {
            Some(body) => Some(body.vels.angvel),
            None => None,
        }
    }

    /// Overwrites the body behind `handle` (upstream `bodies.get_mut`); `false` when the
    /// handle does not resolve. Modify the copy with `RigidBodyTrait` setters so that the change
    /// flags the step reads are raised.
    #[inline(always)]
    fn set_body(ref self: World, handle: Handle, body: RigidBody) -> bool {
        self.bodies.set(handle, body)
    }

    /// A copy of the collider behind `handle`.
    #[inline(always)]
    fn collider(ref self: World, handle: Handle) -> Option<Collider> {
        self.colliders.get(handle)
    }

    /// Overwrites the collider behind `handle` (upstream `colliders.get_mut`); `false` when the
    /// handle does not resolve. Change its parent through `insert_collider` / `remove_collider`
    /// only.
    #[inline(always)]
    fn set_collider(ref self: World, handle: Handle, collider: Collider) -> bool {
        self.colliders.set(handle, collider)
    }

    /// A copy of the impulse joint behind `handle` (with its accumulated impulses).
    #[inline(always)]
    fn impulse_joint(ref self: World, handle: Handle) -> Option<ImpulseJoint> {
        self.impulse_joints.get(handle)
    }

    /// The contact pair of `collider1` and `collider2` (in ascending slot order) found by the
    /// last step, if any (upstream `PhysicsWorld::contact_pair`). Linear scan.
    #[inline(always)]
    fn contact_pair(self: @World, collider1: Handle, collider2: Handle) -> Option<ContactPair> {
        self.narrow_phase.contact_pair(collider1, collider2)
    }

    /// Whether the sensor pair of `collider1` and `collider2` (either order) intersects, as of
    /// the last step (upstream `PhysicsWorld::intersection_pair`): `None` when the two colliders
    /// form no intersection pair (their AABBs did not overlap, neither is a sensor, ...).
    /// Linear scan.
    #[inline(always)]
    fn intersection_pair(self: @World, collider1: Handle, collider2: Handle) -> Option<bool> {
        self.narrow_phase.intersection_pair(collider1, collider2)
    }

    /// `(collider1, collider2, intersecting)` of every sensor pair involving `collider` found
    /// by the last step, in ascending pair order, both colliders still existing (upstream
    /// `PhysicsWorld::intersection_pairs_with`, without the collider references).
    fn intersection_pairs_with(ref self: World, collider: Handle) -> Array<(Handle, Handle, bool)> {
        let pairs = self.narrow_phase.intersection_pairs_with(collider);
        existing(ref self.colliders, pairs.span())
    }

    /// `(collider1, collider2, intersecting)` of every sensor pair found by the last step, in
    /// ascending pair order, both colliders still existing (upstream
    /// `PhysicsWorld::intersection_pairs`, without the collider references).
    fn intersection_pairs(ref self: World) -> Array<(Handle, Handle, bool)> {
        let pairs = self.narrow_phase.intersection_pairs();
        existing(ref self.colliders, pairs.span())
    }

    /// Saves every field of the world into a versioned, serialisable [`WorldState`] (layout and
    /// version policy: [`state`]); the world is unchanged. `from_state(to_state(w))` steps
    /// exactly as `w` and issues the same handles, removals included.
    #[inline(always)]
    fn to_state(ref self: World) -> WorldState {
        state::to_state(ref self)
    }

    /// [`WorldTrait::to_state`] consuming the world: moves the pair list instead of copying it
    /// (cheaper when the world is dropped after the save, e.g. at the end of a chunk).
    #[inline(always)]
    fn into_state(self: World) -> WorldState {
        state::into_state(self)
    }

    /// Rebuilds the world saved by [`WorldTrait::to_state`] or [`WorldTrait::into_state`].
    ///
    /// # Panics
    /// `world state: version` when `state.version` is not `state::WORLD_STATE_VERSION`;
    /// `Arena: state ...` when a set image is invalid.
    #[inline(always)]
    fn from_state(state: WorldState) -> World {
        state::from_state(state)
    }

    /// Advances the simulation by `integration_parameters.dt` (upstream `PhysicsWorld::step`)
    /// and returns the collision events of the step, in the order documented on
    /// `NarrowPhaseTrait::compute_contacts`. See `crate::pipeline::step` for the stages.
    ///
    /// # Panics
    /// As the stages: fixed-point overflow, zero solver iterations, negative parameters.
    fn step(ref self: World) -> Array<CollisionEvent> {
        crate::pipeline::step(ref self)
    }

    /// Advances one step and returns collision and post-solver contact-force events.
    /// Force events use normal impulses only, divided by dt (nearest-even inverse).
    /// Threshold comparison is strict; the minimum enabled collider threshold wins.
    /// Panics as `step`; pair order is ascending and threshold crossing state persists.
    fn step_with_force_events(
        ref self: World,
    ) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
        crate::pipeline::step_with_force_events(ref self)
    }

    /// The collider hit first by `ray` and its time of impact, strictly below `max_toi`
    /// (upstream `QueryPipeline::cast_ray`); ties go to the lowest handle. See
    /// `crate::queries` for the semantics of every query.
    #[inline(always)]
    fn cast_ray(
        ref self: World, ray: Ray, max_toi: Fixed, solid: bool, filter: QueryFilter,
    ) -> Option<(Handle, Fixed)> {
        crate::queries::cast_ray(ref self, ray, max_toi, solid, filter)
    }

    /// [`WorldTrait::cast_ray`] with the world-space normal and feature of the hit.
    #[inline(always)]
    fn cast_ray_and_get_normal(
        ref self: World, ray: Ray, max_toi: Fixed, solid: bool, filter: QueryFilter,
    ) -> Option<(Handle, RayIntersection)> {
        crate::queries::cast_ray_and_get_normal(ref self, ray, max_toi, solid, filter)
    }

    /// Every collider hit by `ray` within `max_toi`, ascending handle order.
    #[inline(always)]
    fn intersect_ray(
        ref self: World, ray: Ray, max_toi: Fixed, solid: bool, filter: QueryFilter,
    ) -> Array<(Handle, RayIntersection)> {
        crate::queries::intersect_ray(ref self, ray, max_toi, solid, filter)
    }

    /// The collider closest to `point`, strictly within `max_dist`, with the world projection.
    #[inline(always)]
    fn project_point(
        ref self: World, point: Vec2, max_dist: Fixed, solid: bool, filter: QueryFilter,
    ) -> Option<(Handle, PointProjection)> {
        crate::queries::project_point(ref self, point, max_dist, solid, filter)
    }

    /// Every collider containing `point`, ascending handle order.
    #[inline(always)]
    fn intersect_point(ref self: World, point: Vec2, filter: QueryFilter) -> Array<Handle> {
        crate::queries::intersect_point(ref self, point, filter)
    }

    /// Every collider whose world AABB intersects `aabb`, ascending handle order.
    #[inline(always)]
    fn intersect_aabb(ref self: World, aabb: Aabb, filter: QueryFilter) -> Array<Handle> {
        crate::queries::intersect_aabb(ref self, aabb, filter)
    }
}

/// An island member that does not sleep: enabled, not fixed, `sleeping` clear.
#[inline(always)]
fn is_active(body: @RigidBody) -> bool {
    *body.enabled && *body.body_type != RigidBodyType::Fixed && !*body.activation.sleeping
}

/// The entries of `pairs` whose two colliders exist.
fn existing(
    ref colliders: ColliderSet, pairs: Span<(Handle, Handle, bool)>,
) -> Array<(Handle, Handle, bool)> {
    let mut out = array![];
    for (h1, h2, intersecting) in pairs {
        if colliders.contains(*h1) && colliders.contains(*h2) {
            out.append((*h1, *h2, *intersecting));
        }
    }
    out
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod alternatives {
    use super::{CollisionEvent, World, WorldTrait};

    /// `num_active_bodies` as the length of `active_bodies` (builds the array).
    pub fn num_active_bodies_via_array(ref world: World) -> u32 {
        world.active_bodies().len()
    }

    /// Previous step entry: the compatibility wrapper adds a full World argument/return copy.
    pub fn step_wrapped(ref world: World) -> Array<CollisionEvent> {
        crate::pipeline::step(ref world)
    }
}
