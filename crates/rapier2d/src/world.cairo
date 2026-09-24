//! `World`: everything one simulation owns, bundled (upstream `pipeline/physics_world.rs`,
//! `PhysicsWorld`).
//!
//! The persistent state is exactly `docs/PLAN.md` D9: the body and collider sets (poses,
//! velocities, mass properties, change flags), the impulse joints (with their accumulated
//! impulses) and the narrow-phase pairs (manifolds carrying the warm-start impulses and the event
//! status). Everything else [`WorldTrait::step`] needs (broad-phase proxies and pairs, solver
//! bodies, constraints) is rebuilt every step and dropped with it.
//!
//! Mutations go through the sets' own setters, which raise the change flags the next step reads
//! (`RigidBodyTrait::set_position`, `ColliderTrait::set_shape`, …): read a copy with
//! [`WorldTrait::body`] / [`WorldTrait::collider`], modify it, write it back with
//! [`WorldTrait::set_body`] / [`WorldTrait::set_collider`].
//!
//! Sleeping (work package SL): bodies fall asleep and wake up island by island inside `step`
//! (`crate::pipeline::islands`); as upstream's `PhysicsWorld`, inserting or removing a joint
//! wakes both of its bodies up, removing a body or a collider wakes up every body it was in
//! contact with, and a body woken up by hand (`RigidBodyTrait::wake_up`, the setters, forces
//! and impulses) written back with [`WorldTrait::set_body`] wakes its island at the next step.
//!
//! Deviations from upstream: no persistent island manager (islands are rebuilt every step),
//! broad-phase state, CCD solver, multibody or soft body sets, query pipeline (the scene queries
//! scan the collider set, `crate::queries`), hooks or event handler (events are returned by
//! `step`).

use fixed::Fixed;
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::joint::{GenericJoint, ImpulseJoint, ImpulseJointSet, ImpulseJointSetTrait};
use rapier_dynamics2d::narrow_phase::{ContactPair, NarrowPhase, NarrowPhaseTrait};
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait, RigidBodyTrait};
use rapier_geometry2d::aabb::Aabb;
use rapier_geometry2d::point::PointProjection;
use rapier_geometry2d::ray::{Ray, RayIntersection};
use crate::queries::QueryFilter;

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
    /// Last step's contact pairs (ascending collider slot), with their warm-start impulses.
    pub narrow_phase: NarrowPhase,
}

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
    /// does not resolve. Every body in contact with it is woken up (its parent included); its
    /// parent's mass is recomputed at the next step, which also ends its contact pairs (see
    /// [`WorldTrait::remove_body`]).
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

    /// Wakes up the parents of both colliders of every contact pair of `collider` (upstream
    /// `NarrowPhase::remove_collider`), before the collider goes.
    fn wake_contact_partners(ref self: World, collider: Handle) {
        let mut touched = array![collider];
        let _ = crate::pipeline::sleeping::wake_touched_partners(
            touched.span(), self.narrow_phase.pairs.span(), ref self.bodies, ref self.colliders,
        );
    }

    /// A copy of the body behind `handle`.
    #[inline(always)]
    fn body(ref self: World, handle: Handle) -> Option<RigidBody> {
        self.bodies.get(handle)
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

    /// Advances the simulation by `integration_parameters.dt` (upstream `PhysicsWorld::step`)
    /// and returns the collision events of the step, in the order documented on
    /// `NarrowPhaseTrait::compute_contacts`. See `crate::pipeline::step` for the stages.
    ///
    /// # Panics
    /// As the stages: fixed-point overflow, zero solver iterations, negative parameters.
    fn step(ref self: World) -> Array<CollisionEvent> {
        crate::pipeline::step(ref self)
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

#[cfg(test)]
mod tests {
    use fixed::{HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_core::Handle;
    use rapier_dynamics2d::collider::{ColliderBuilderTrait, ColliderTrait};
    use rapier_dynamics2d::collider_set::ColliderSetTrait;
    use rapier_dynamics2d::joint::RevoluteJointBuilderTrait;
    use rapier_dynamics2d::rigid_body_set::{RigidBodySetTrait, RigidBodyTrait};
    use rapier_math::pose2::Pose2;
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use super::{World, WorldTrait};

    fn at(x: fixed::Fixed, y: fixed::Fixed) -> Pose2 {
        Pose2 { translation: Vec2 { x, y }, rotation: Rot2 { re: ONE, im: ZERO } }
    }

    /// Two dynamic balls joined by a revolute joint, and a standalone ground collider.
    fn pair_world() -> (World, Handle, Handle, Handle) {
        let mut world = WorldTrait::new(Vec2 { x: ZERO, y: -ONE }, Default::default());
        let (a, _) = world
            .insert(
                RigidBodyTrait::dynamic(at(ZERO, ONE)), ColliderBuilderTrait::ball(HALF).build(),
            );
        let (b, _) = world
            .insert(
                RigidBodyTrait::dynamic(at(ONE, ONE)), ColliderBuilderTrait::ball(HALF).build(),
            );
        let joint = world.insert_impulse_joint(a, b, RevoluteJointBuilderTrait::new().build());
        let _ = world
            .insert_collider(
                ColliderBuilderTrait::halfspace(Vec2 { x: ZERO, y: ONE }).build(), None,
            );
        (world, a, b, joint)
    }

    #[test]
    fn test_insert_links_and_poses() {
        let (mut world, a, _, joint) = pair_world();
        assert_eq!(world.bodies.len(), 2);
        let body = world.body(a).unwrap();
        assert_eq!(body.colliders.len(), 1);
        let collider = world.collider(*body.colliders.at(0)).unwrap();
        assert_eq!(collider.parent(), Some(a));
        assert_eq!(collider.position(), at(ZERO, ONE));
        let standalone = world.collider(rapier_core::Handle { index: 2, generation: 0 }).unwrap();
        assert_eq!(standalone.parent(), None);
        assert_eq!(world.impulse_joint(joint).unwrap().body1, a);
    }

    /// `remove_body` removes the body's colliders and every joint attached to it; the other
    /// body and the standalone collider stay.
    #[test]
    fn test_remove_body_detaches_colliders_and_joints() {
        let (mut world, a, b, joint) = pair_world();
        let co = *world.body(a).unwrap().colliders.at(0);
        assert!(world.remove_body(a).is_some());
        assert!(world.remove_body(a).is_none());
        assert!(world.collider(co).is_none());
        assert!(world.impulse_joint(joint).is_none());
        assert!(world.body(b).is_some());
        assert_eq!(world.colliders.len(), 2);
        let _ = world.step();
    }

    #[test]
    fn test_remove_collider_and_joint() {
        let (mut world, a, _, joint) = pair_world();
        let co = *world.body(a).unwrap().colliders.at(0);
        assert!(world.remove_collider(co).is_some());
        assert!(world.remove_collider(co).is_none());
        assert_eq!(world.body(a).unwrap().colliders.len(), 0);
        assert_eq!(
            world.remove_impulse_joint(joint), Some(RevoluteJointBuilderTrait::new().build()),
        );
        assert!(world.remove_impulse_joint(joint).is_none());
    }

    #[test]
    fn test_setters_write_back() {
        let (mut world, a, _, _) = pair_world();
        let mut body = world.body(a).unwrap();
        body.set_linvel(Vec2 { x: ONE, y: ZERO });
        assert!(world.set_body(a, body));
        assert_eq!(world.body(a).unwrap().linvel(), Vec2 { x: ONE, y: ZERO });
        let co = *body.colliders.at(0);
        let mut collider = world.collider(co).unwrap();
        collider.set_friction(ONE);
        assert!(world.set_collider(co, collider));
        assert_eq!(world.collider(co).unwrap().friction(), ONE);
        let stale = Handle { index: a.index, generation: a.generation + 1 };
        assert!(!world.set_body(stale, body));
        assert!(world.contact_pair(co, co).is_none());
    }

    // Gas probes: `gas_<op>` − `gas_setup` is one call on the `pair_world` state.

    #[test]
    fn gas_baseline() {
        let _ = opaque(1_u32);
    }

    #[test]
    fn gas_new() {
        let _ = WorldTrait::new(opaque(Vec2 { x: ZERO, y: -ONE }), Default::default());
    }

    #[test]
    fn gas_setup() {
        let _ = pair_world();
    }

    #[test]
    fn gas_insert_body() {
        let (mut world, _, _, _) = pair_world();
        let _ = world.insert_body(RigidBodyTrait::dynamic(opaque(at(ZERO, ZERO))));
    }

    #[test]
    fn gas_insert_collider_attached() {
        let (mut world, a, _, _) = pair_world();
        let _ = world.insert_collider(ColliderBuilderTrait::ball(opaque(HALF)).build(), Some(a));
    }

    #[test]
    fn gas_insert_collider_standalone() {
        let (mut world, _, _, _) = pair_world();
        let _ = world.insert_collider(ColliderBuilderTrait::ball(opaque(HALF)).build(), None);
    }

    #[test]
    fn gas_insert() {
        let (mut world, _, _, _) = pair_world();
        let _ = world
            .insert(
                RigidBodyTrait::dynamic(opaque(at(ZERO, ZERO))),
                ColliderBuilderTrait::ball(HALF).build(),
            );
    }

    #[test]
    fn gas_insert_impulse_joint() {
        let (mut world, a, b, _) = pair_world();
        let _ = world.insert_impulse_joint(opaque(a), b, RevoluteJointBuilderTrait::new().build());
    }

    #[test]
    fn gas_remove_body() {
        let (mut world, a, _, _) = pair_world();
        assert!(world.remove_body(opaque(a)).is_some());
    }

    #[test]
    fn gas_remove_collider() {
        let (mut world, _, _, _) = pair_world();
        assert!(world.remove_collider(opaque(Handle { index: 2, generation: 0 })).is_some());
    }

    #[test]
    fn gas_remove_impulse_joint() {
        let (mut world, _, _, joint) = pair_world();
        assert!(world.remove_impulse_joint(opaque(joint)).is_some());
    }

    #[test]
    fn gas_body() {
        let (mut world, a, _, _) = pair_world();
        assert!(world.body(opaque(a)).is_some());
    }

    #[test]
    fn gas_set_body() {
        let (mut world, a, _, _) = pair_world();
        let body = world.body(a).unwrap();
        assert!(world.set_body(opaque(a), body));
    }

    #[test]
    fn gas_collider() {
        let (mut world, _, _, _) = pair_world();
        assert!(world.collider(opaque(Handle { index: 2, generation: 0 })).is_some());
    }

    #[test]
    fn gas_set_collider() {
        let (mut world, _, _, _) = pair_world();
        let h = Handle { index: 2, generation: 0 };
        let collider = world.collider(h).unwrap();
        assert!(world.set_collider(opaque(h), collider));
    }

    #[test]
    fn gas_impulse_joint() {
        let (mut world, _, _, joint) = pair_world();
        assert!(world.impulse_joint(opaque(joint)).is_some());
    }

    #[test]
    fn gas_contact_pair() {
        let (mut world, _, _, _) = pair_world();
        let _ = world.step();
        let h = Handle { index: 2, generation: 0 };
        let _ = world.contact_pair(opaque(h), h);
    }

    #[test]
    fn gas_step() {
        let (mut world, _, _, _) = pair_world();
        let _ = world.step();
    }
}
