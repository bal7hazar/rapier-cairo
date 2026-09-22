//! Public 2D facade of rapier.cairo (wave 5, `docs/PLAN.md`): the `World` bundle, the step
//! pipeline and the dispatcher glue between `rapier_geometry2d` and `rapier_dynamics2d`.
//! Modules are pre-declared per work package (see `docs/briefs/`).

pub mod dispatcher;
pub mod pipeline;
pub mod world;

/// Everything a game needs to build and step a world (requested by P1).
pub mod prelude {
    pub use fixed::Fixed;
    pub use glam::Vec2;
    pub use rapier_core::data::handle::Handle;
    pub use rapier_core::integration_parameters::IntegrationParameters;
    pub use rapier_dynamics2d::collider::ColliderTrait;
    pub use rapier_dynamics2d::collider::builder::{ColliderBuilder, ColliderBuilderTrait};
    pub use rapier_dynamics2d::joint::{
        FixedJointBuilderTrait, PrismaticJointBuilderTrait, RevoluteJointBuilderTrait,
    };
    pub use rapier_dynamics2d::{CollisionEvent, CollisionEventTrait, RigidBody, RigidBodyTrait};
    pub use rapier_geometry2d::shape::Shape;
    pub use crate::dispatcher::DefaultDispatcher;
    pub use crate::world::{World, WorldTrait};
}
