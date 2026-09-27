//! Controllers (upstream `rapier::control`, work package KC1): the PD / PID controllers
//! ([`pid_controller`]) and the kinematic character controller ([`character_controller`]).
//! Nothing here is reached by the step: the `BasicStepConfig` program and the step's Cairo steps
//! do not move.
//!
//! Upstream's `DynamicRayCastVehicleController` (and its `Wheel`, `WheelTuning`, `RayCastInfo`,
//! `WheelContactPoint`) is compiled for `dim3` only (`control/mod.rs`): there is no 2D item to
//! port.

/// `PdController`, `PidController`, `PdErrors`.
pub mod pid_controller;
pub use pid_controller::{
    PdController, PdControllerDefault, PdControllerImpl, PdControllerTrait, PdErrors, PidController,
    PidControllerDefault, PidControllerImpl, PidControllerTrait, RigidBodyVelocityIntoPdErrors,
};

/// `KinematicCharacterController`, `CharacterLength`, `CharacterAutostep`, `CharacterCollision`,
/// `EffectiveCharacterMovement`.
pub mod character_controller;
pub use character_controller::{
    CharacterAutostep, CharacterAutostepDefault, CharacterCollision, CharacterLength,
    CharacterLengthImpl, CharacterLengthTrait, EffectiveCharacterMovement, HitDecomposition,
    HitDecompositionImpl, HitDecompositionTrait, KinematicCharacterController,
    KinematicCharacterControllerDefault, KinematicCharacterControllerImpl,
    KinematicCharacterControllerTrait,
};
