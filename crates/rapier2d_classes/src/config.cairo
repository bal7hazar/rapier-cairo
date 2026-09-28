//! The stage configurations (`StageConfig`) of a contract whose step library-calls the declared
//! classes, and CS4's layout (a `StepConfig`: its solve is the joint strategy's).

use rapier2d::pipeline::config::{
    NoComposites, NoJoints, NoSensors, PairLoopNarrowPhase, StageConfig, StepConfig,
};
use rapier2d::pipeline::stages::narrow::BatchedNarrowPhase;
use rapier2d::pipeline::stages::{
    BasicShapeKernels, InProcessActiveSet, InProcessForceEvents, InProcessFreePath, InProcessShapes,
    NoFreePath,
};
use crate::active_set::LibraryCallActiveSet;
use crate::advance::{HybridSolveAdvance, LibraryCallSolveAdvance};
use crate::broad_phase::LibraryCallBroadPhase;
use crate::contact::{FamilyBatch, FamilyDispatcher};
use crate::hashes::ClassHashes;
use crate::islands::LibraryCallIslands;
use crate::mass::LibraryCallMass;
use crate::narrow::LibraryCallNarrowPhase;
use crate::solver::LibraryCallSolver;

/// The stages of `BasicStepConfig` (balls, cuboids, convex polygons and half-spaces; no sensor,
/// no composite shape, no joint: `step_with_force_events_with_stages::<BasicStepConfig,
/// SplitStages<H>>`) with every stage but the step's orchestration in a declared class, at `H`'s
/// hashes (CS5): the contact generation of each pair (`ContactBallClass`, `ContactPolygonClass`,
/// one call per pair, as CS4), the broad phase (`BroadPhaseClass`), the island stage
/// (`IslandsClass`), the fused solve and position update (`SolveAdvanceClass`) and the mass
/// properties (`MassClass`). Results are bit-identical to `BasicStepConfig`'s.
///
/// Measured against its two alternatives below on slingfall's pile10 shot
/// (`tests/steps.cairo`): the batched contact generation costs as many Cairo steps across the call
/// and more in the caller class; the hybrid solve saves 1 % of the steps but keeps the free-body
/// solver in the caller.
///
/// # Panics
/// As `BasicStepConfig`; when a library call fails (a class hash that is not declared).
pub impl SplitStages<impl H: ClassHashes> of StageConfig {
    impl Narrow = PairLoopNarrowPhase<FamilyDispatcher<H>, NoSensors, NoComposites>;
    impl Broad = LibraryCallBroadPhase<H>;
    impl Islands = LibraryCallIslands<H>;
    impl Advance = LibraryCallSolveAdvance<H>;
    impl Mass = LibraryCallMass<H>;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

/// [`SplitStages`] with the contact generation batched per shape-pair family
/// (`BatchedNarrowPhase<FamilyBatch>`: one call of each family class per step).
pub impl SplitBatchedStages<impl H: ClassHashes> of StageConfig {
    impl Narrow = BatchedNarrowPhase<FamilyBatch<H>, NoSensors>;
    impl Broad = LibraryCallBroadPhase<H>;
    impl Islands = LibraryCallIslands<H>;
    impl Advance = LibraryCallSolveAdvance<H>;
    impl Mass = LibraryCallMass<H>;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

/// [`SplitStages`] with `HybridSolveAdvance`: no solve-and-advance call on a step without a
/// touching pair (its free bodies move in the caller).
pub impl SplitHybridStages<impl H: ClassHashes> of StageConfig {
    impl Narrow = PairLoopNarrowPhase<FamilyDispatcher<H>, NoSensors, NoComposites>;
    impl Broad = LibraryCallBroadPhase<H>;
    impl Islands = LibraryCallIslands<H>;
    impl Advance = HybridSolveAdvance<H>;
    impl Mass = LibraryCallMass<H>;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

/// [`SplitStages`] for a caller class under the declared-class limit (CS6): the narrow phase's
/// pair loop in `NarrowPhaseClass` (`LibraryCallNarrowPhase`, the contact generation batched per
/// family from there), the rebuild of the active set in `ActiveSetClass`
/// (`LibraryCallActiveSet`), no pair-free fast path (the ordinary path, same results), the
/// bounding boxes of the basic shapes only (`BasicShapeKernels`), no position-based kinematic body.
/// With the basic `WorldState` codec (`rapier2d::world::basic_state`), the caller compiles no
/// joint, no other shape, no one-way filter and no `atan2`.
///
/// # Panics
/// As [`SplitStages`]; `'Step: not a basic shape'` on another shape's proxy,
/// `'Step: kinematic disabled'` on a position-based kinematic body.
pub impl SlimSplitStages<impl H: ClassHashes> of StageConfig {
    impl Narrow = LibraryCallNarrowPhase<H>;
    impl Broad = LibraryCallBroadPhase<H>;
    impl Islands = LibraryCallIslands<H>;
    impl Advance = LibraryCallSolveAdvance<H>;
    impl Mass = LibraryCallMass<H>;
    impl Shapes = BasicShapeKernels;
    impl Forces = InProcessForceEvents;
    impl Active = LibraryCallActiveSet<H>;
    impl Free = NoFreePath;
    const KINEMATIC: bool = false;
}

/// CS4's layout: `BasicStepConfig` with the contact generation of each pair in the family classes
/// (`FamilyDispatcher`) and the island solve in `SolverClass` (`LibraryCallSolver`), every other
/// stage in the caller (`step_with_force_events_with::<ContactSolveStepConfig<H>>`).
pub impl ContactSolveStepConfig<impl H: ClassHashes> of StepConfig {
    impl Dispatcher = FamilyDispatcher<H>;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = LibraryCallSolver<H>;
}
