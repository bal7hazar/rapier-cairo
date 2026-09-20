//! Vector-valued components of a 2D rigid-body: the port of the part of upstream's
//! `dynamics/rigid_body_components.rs` that holds a `Vec2` or a `Pose2`.
//!
//! The scalar components (body type, damping coefficients, sleeping state, dominance, change
//! flags) are dimension-agnostic and live in [`rapier_core::rigid_body`]; this module adds the
//! pose ([`position`]), the velocities ([`velocity`]), the external forces ([`forces`]), the
//! world-space mass properties ([`mass_props`]) and the axis-locking mask ([`locked_axes`]).
//!
//! One step of the engine drives them in this order:
//!
//! 1. once per step, `RigidBodyMassProps::update_world_mass_properties` refreshes the world
//!    centre of mass and the effective inverses, then
//!    `RigidBodyForces::compute_effective_force_and_torque` turns gravity into a force;
//! 2. once per substep, `RigidBodyForces::integrate` produces the new velocities and
//!    `RigidBodyVelocity::integrate` the new pose (symplectic Euler);
//! 3. the constraint solver applies impulses through `RigidBodyVelocity::apply_impulse_at_point`
//!    between those two.
//!
//! Every inverse mass and inertia goes through `rapier_math::inv` (`inv(0) = 0`), so a fixed
//! body, an infinite mass and a locked axis are all the same code path. Deferred to later work
//! packages: the body set and its change tracking (DD), CCD, the 3D-only gyroscopic terms and
//! everything that needs the angle of a rotation (`pose_errors`, `interpolate_velocity`).

pub mod forces;
pub mod locked_axes;
pub mod mass_props;
pub mod position;
pub mod velocity;

pub use forces::{
    RigidBodyForces, RigidBodyForcesDefault, RigidBodyForcesImpl, RigidBodyForcesTrait,
};
pub use locked_axes::{
    LockedAxes, LockedAxesBitAnd, LockedAxesBitOr, LockedAxesImpl, LockedAxesTrait, ROTATION_LOCKED,
    TRANSLATION_LOCKED, TRANSLATION_LOCKED_X, TRANSLATION_LOCKED_Y,
};
pub use mass_props::{RigidBodyMassProps, RigidBodyMassPropsImpl, RigidBodyMassPropsTrait};
pub use position::{
    RigidBodyPosition, RigidBodyPositionDefault, RigidBodyPositionImpl, RigidBodyPositionTrait,
};
pub use velocity::{
    RigidBodyVelocity, RigidBodyVelocityAdd, RigidBodyVelocityImpl, RigidBodyVelocitySub,
    RigidBodyVelocityTrait,
};
