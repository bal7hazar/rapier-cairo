//! What a step compiles (work package CS2): the contact dispatcher and the strategies for
//! sensors, composite shapes and impulse joints, as one [`StepConfig`] handed to
//! `WorldTrait::step_with` / `step_with_force_events_with` (upstream hands the same choices to
//! `PhysicsPipeline::step` at run time: a `&dyn QueryDispatcher` in the narrow phase, the joint
//! sets, the optional CCD solver).
//!
//! A Cairo program contains every function its entry points reach, and a proof pays for the whole
//! program (the bootloader hashes it). `World::step` reaches every shape pair, the sensor tests,
//! the composite manifolds and the joint solver; a step monomorphised with a [`StepConfig`] of
//! no-op strategies reaches none of them:
//!
//! * [`DefaultStepConfig`]: everything, what `World::step` and `step_with_force_events` use
//!   (they are `step_with::<DefaultStepConfig>`, same code, same Cairo steps);
//! * [`BasicStepConfig`]: balls, cuboids, convex polygons and half-spaces
//!   (`crate::dispatcher::BasicShapesDispatcher`), no sensor, no composite shape, no joint.
//!
//! A game picks its own mix: `impl MyStep of StepConfig { impl Dispatcher = ..; impl Sensors =
//! ..; impl Composites = ..; impl Joints = ..; }`. `impl Composites =
//! ConstrainedCompositeManifolds;` honours the compounds built with `FIX_INTERNAL_EDGES` (lot CE),
//! which `DefaultStepConfig`'s `CompositeManifolds` ignores.
//!
//! # Stages (CS5, CS6)
//!
//! Where the stages of the step run is a second, separate choice, a [`StageConfig`] handed to
//! `WorldTrait::step_with_stages` / `step_with_force_events_with_stages` (`super::stages`): the
//! broad phase, the narrow phase's pair loop, the island stage, the fused solve and advance and
//! the mass properties, so that a contract can run them in other declared classes
//! (`rapier2d_classes`), and the code a slim caller class leaves out (CS6). `step_with::<C>` is
//! `step_with_stages::<C, InProcessStages<C>>`: every stage in process, the program it compiled
//! before the slots existed.
//!
//! # Rejection instead of silent skipping
//!
//! A disabled feature panics at the step that meets it, never silently: a joint in the world
//! (`errors::JOINTS`, at the next step), a sensor pair (`NoSensors`), a composite pair
//! (`NoComposites`) or a shape outside the dispatcher's (`'Dispatch: not a basic shape'`) when its
//! AABB first meets another collider's. The checks sit on branches the supported worlds never
//! take, so they cost no Cairo step there.
//!
//! # Cost
//!
//! With a world the configuration supports, results are bit-identical to
//! `World::step_with_force_events`, and so are the Cairo steps but for the empty joint stages
//! that `NoJoints` skips (see `docs/research/class-size.md` and the `bytecode_size.py`
//! executable fixtures for the program sizes).
//!
//! CCD is chosen by the entry point, as upstream passes its `CCDSolver`: `step_with` runs none
//! (as `World::step`), `crate::pipeline::ccd::step_with_ccd_with` runs the configured step under
//! the CCD pass.

use core::dict::{Felt252Dict, Felt252DictTrait};
use rapier_core::Handle;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_dynamics2d::joint::{ImpulseJoint, ImpulseJointSet, ImpulseJointSetTrait, JointEnabled};
/// The pieces a [`StepConfig`] is made of, in one place.
pub use rapier_dynamics2d::narrow_phase::ContactDispatcher;
pub use rapier_dynamics2d::narrow_phase::strategies::{
    CompositeManifolds, CompositeStrategy, ConstrainedCompositeManifolds, IntersectionStrategy,
    NoComposites, NoSensors, SensorIntersections,
};
use rapier_dynamics2d::rigid_body_set::RigidBody;
use rapier_dynamics2d::solver::island::{
    SolvedIsland, SolverInput, solve_island_input, solve_island_input_contacts,
};
use rapier_geometry2d::contact::ContactManifold;
pub use crate::dispatcher::{BasicShapesDispatcher, DefaultDispatcher};
/// The stage slots (CS5) and their in-process implementations.
pub use super::stages::{
    BroadPhaseStage, InProcessBroadPhase, InProcessIslands, InProcessMass, InProcessSolveAdvance,
    InProcessStages, IslandStage, MassStage, NarrowPhaseStage, PairLoopNarrowPhase,
    SolveAdvanceStage, StageConfig,
};
use super::{active_joints, joint_values, write_joints};

/// Panics of the disabled strategies.
pub mod errors {
    /// The world has an impulse joint and the step was compiled with [`super::NoJoints`].
    pub const JOINTS: felt252 = 'Step: joints disabled';
}

/// What a step compiles: its contact dispatcher and its strategies (see the module
/// documentation). Selected statically, as the dispatcher: `step_with::<C>`.
pub trait StepConfig {
    /// Contact manifolds of the convex pairs.
    impl Dispatcher: ContactDispatcher;
    /// Pairs with a sensor.
    impl Sensors: IntersectionStrategy;
    /// Pairs the dispatcher does not support (composite shapes).
    impl Composites: CompositeStrategy;
    /// Impulse joints.
    impl Joints: JointStrategy;
}

/// Everything: what `World::step` and `World::step_with_force_events` compile.
pub impl DefaultStepConfig of StepConfig {
    impl Dispatcher = DefaultDispatcher;
    impl Sensors = SensorIntersections;
    impl Composites = CompositeManifolds;
    impl Joints = ImpulseJointSolver;
}

/// Balls, cuboids, convex polygons and half-spaces; no sensor, no composite shape, no joint.
///
/// # Panics
/// As `World::step`, and when the world uses anything else (see the module documentation).
pub impl BasicStepConfig of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
}

/// How the step treats the world's impulse joints.
pub trait JointStrategy {
    /// `true` when `joints` is empty.
    fn joint_free(joints: @ImpulseJointSet) -> bool;
    /// Every joint of `joints` in ascending slot (`ImpulseJointSetTrait::to_array`).
    fn entries(ref joints: ImpulseJointSet) -> Array<(Handle, ImpulseJoint)>;
    /// The joints of `joint_entries` the solver takes (the non-dormant ones when a body of
    /// `entries` is `sleeping`) and their values; the bodies of the enabled ones are marked in
    /// `constrained`.
    fn constrain(
        joint_entries: Span<(Handle, ImpulseJoint)>,
        entries: Span<(Handle, RigidBody)>,
        sleeping: bool,
        ref constrained: Felt252Dict<bool>,
    ) -> (Span<(Handle, ImpulseJoint)>, Array<ImpulseJoint>);
    /// The island solve (`solve_island_input`).
    fn solve(
        params: IntegrationParameters,
        input: SolverInput,
        manifolds: Span<ContactManifold>,
        ref joints: Array<ImpulseJoint>,
    ) -> SolvedIsland;
    /// Writes the solved `joints` back into `impulse_joints`.
    fn write(
        joint_entries: Span<(Handle, ImpulseJoint)>,
        joints: Span<ImpulseJoint>,
        ref impulse_joints: ImpulseJointSet,
    );
}

/// The impulse-joint solver (`rapier_dynamics2d::solver::joint`).
pub impl ImpulseJointSolver of JointStrategy {
    #[inline(always)]
    fn joint_free(joints: @ImpulseJointSet) -> bool {
        joints.len() == 0
    }

    #[inline(always)]
    fn entries(ref joints: ImpulseJointSet) -> Array<(Handle, ImpulseJoint)> {
        joints.to_array()
    }

    #[inline(always)]
    fn constrain(
        joint_entries: Span<(Handle, ImpulseJoint)>,
        entries: Span<(Handle, RigidBody)>,
        sleeping: bool,
        ref constrained: Felt252Dict<bool>,
    ) -> (Span<(Handle, ImpulseJoint)>, Array<ImpulseJoint>) {
        let joint_entries = if sleeping {
            active_joints(joint_entries, entries).span()
        } else {
            joint_entries
        };
        let joints = joint_values(joint_entries);
        for joint in joints.span() {
            if *joint.data.enabled == JointEnabled::Enabled {
                constrained.insert((*joint.body1).into(), true);
                constrained.insert((*joint.body2).into(), true);
            }
        }
        (joint_entries, joints)
    }

    #[inline(always)]
    fn solve(
        params: IntegrationParameters,
        input: SolverInput,
        manifolds: Span<ContactManifold>,
        ref joints: Array<ImpulseJoint>,
    ) -> SolvedIsland {
        solve_island_input(params, input, manifolds, ref joints)
    }

    #[inline(always)]
    fn write(
        joint_entries: Span<(Handle, ImpulseJoint)>,
        joints: Span<ImpulseJoint>,
        ref impulse_joints: ImpulseJointSet,
    ) {
        write_joints(joint_entries, joints, ref impulse_joints);
    }
}

/// No joint: the joint solver is not compiled.
///
/// # Panics
/// `errors::JOINTS` at the first step of a world with an impulse joint.
pub impl NoJoints of JointStrategy {
    /// As [`ImpulseJointSolver`]: a world with a joint takes neither the pair-free path nor the
    /// sparse step, so [`JointStrategy::entries`] rejects it, off the pair-free path.
    #[inline(always)]
    fn joint_free(joints: @ImpulseJointSet) -> bool {
        joints.len() == 0
    }

    #[inline(always)]
    fn entries(ref joints: ImpulseJointSet) -> Array<(Handle, ImpulseJoint)> {
        assert(joints.len() == 0, errors::JOINTS);
        array![]
    }

    #[inline(always)]
    fn constrain(
        joint_entries: Span<(Handle, ImpulseJoint)>,
        entries: Span<(Handle, RigidBody)>,
        sleeping: bool,
        ref constrained: Felt252Dict<bool>,
    ) -> (Span<(Handle, ImpulseJoint)>, Array<ImpulseJoint>) {
        (joint_entries, array![])
    }

    #[inline(always)]
    fn solve(
        params: IntegrationParameters,
        input: SolverInput,
        manifolds: Span<ContactManifold>,
        ref joints: Array<ImpulseJoint>,
    ) -> SolvedIsland {
        solve_island_input_contacts(params, input, manifolds)
    }

    #[inline(always)]
    fn write(
        joint_entries: Span<(Handle, ImpulseJoint)>,
        joints: Span<ImpulseJoint>,
        ref impulse_joints: ImpulseJointSet,
    ) {}
}

#[cfg(test)]
mod alternatives;
#[cfg(test)]
pub(crate) mod tests;
