//! 2D rigid-body dynamics for rapier-cairo (port of the Rapier subset described in
//! `docs/PLAN.md`): body and collider sets, narrow-phase bookkeeping, joints, the soft-contact
//! substep solver. Modules are pre-declared per work package (see `docs/briefs/`).

pub mod collider;
pub mod collider_set;
pub mod events;
pub mod joint;
pub mod narrow_phase;
pub mod rigid_body;
pub mod rigid_body_set;
pub mod solver;
pub use collider::components::OneWayPlatform;

pub use collider_set::{ColliderSet, ColliderSetTrait};
pub use events::{CollisionEvent, CollisionEventTrait, ContactForceEvent, ContactForceEventTrait};
pub use narrow_phase::{
    ContactDispatcher, ContactPair, ContactPairTrait, NarrowPhase, NarrowPhaseTrait,
};
pub use rigid_body_set::{
    RigidBody, RigidBodyBuilder, RigidBodyBuilderTrait, RigidBodySet, RigidBodySetTrait,
    RigidBodyTrait,
};
pub use solver::body_store::{
    SolverBodyIndexMap, SolverBodyIndexMapTrait, SolverBodyStore, SolverBodyStoreTrait,
};
pub use solver::island::solve_island;
