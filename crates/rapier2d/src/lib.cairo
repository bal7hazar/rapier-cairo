//! Public 2D facade of rapier-cairo (wave 5, `docs/PLAN.md`): the `World` bundle, the step
//! pipeline and the dispatcher glue between `rapier_geometry2d` and `rapier_dynamics2d`.
//! Modules are pre-declared per work package (see `docs/briefs/`).

pub mod dispatcher;
pub mod pipeline;
pub mod queries;
pub mod world;

/// Everything a game needs to build and step a world (requested by P1).
pub mod prelude {
    pub use fixed::Fixed;
    pub use glam::Vec2;
    pub use rapier_core::collider::events::{
        ActiveEvents, COLLISION_EVENTS, CONTACT_FORCE_EVENTS, CollisionEventFlags, REMOVED, SENSOR,
    };
    pub use rapier_core::data::handle::Handle;
    pub use rapier_core::integration_parameters::IntegrationParameters;
    pub use rapier_dynamics2d::collider::builder::{ColliderBuilder, ColliderBuilderTrait};
    pub use rapier_dynamics2d::collider::{Collider, ColliderTrait};
    pub use rapier_dynamics2d::joint::{
        FixedJointBuilderTrait, GenericJointTrait, ImpulseJoint, ImpulseJointSet,
        ImpulseJointSetTrait, LIN_AXES, PrismaticJointBuilderTrait, RevoluteJointBuilderTrait,
        RopeJointBuilder, RopeJointBuilderTrait, SpringJointBuilder, SpringJointBuilderTrait,
    };
    pub use rapier_dynamics2d::{
        ColliderSet, ColliderSetTrait, CollisionEvent, CollisionEventTrait, ContactForceEvent,
        ContactForceEventTrait, ContactPair, ContactPairTrait, OneWayPlatform, RigidBody,
        RigidBodyBuilder, RigidBodyBuilderTrait, RigidBodySet, RigidBodySetTrait, RigidBodyTrait,
    };
    pub use rapier_geometry2d::ray::{Ray, RayIntersection, RayTrait};
    pub use rapier_geometry2d::shape::Shape;
    pub use rapier_math::pose2::{Pose2, Pose2Trait};
    pub use rapier_math::rot2::{Rot2, Rot2Trait};
    pub use crate::dispatcher::DefaultDispatcher;
    pub use crate::queries::{QueryFilter, QueryFilterTrait};
    pub use crate::world::{World, WorldTrait};
}
