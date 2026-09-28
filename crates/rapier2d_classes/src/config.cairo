//! The step configurations of a contract whose step library-calls the declared classes.

use rapier2d::pipeline::config::{
    InProcessBroadPhase, InProcessIslands, InProcessMass, InProcessSolveAdvance, NoComposites,
    NoJoints, NoSensors, PairLoopNarrowPhase, StepConfig,
};
use rapier2d::pipeline::stages::narrow::BatchedNarrowPhase;
use crate::advance::{HybridSolveAdvance, LibraryCallSolveAdvance};
use crate::broad_phase::LibraryCallBroadPhase;
use crate::contact::{FamilyBatch, FamilyDispatcher};
use crate::hashes::ClassHashes;
use crate::islands::LibraryCallIslands;
use crate::mass::LibraryCallMass;
use crate::solver::LibraryCallSolver;

/// `BasicStepConfig` (balls, cuboids, convex polygons and half-spaces; no sensor, no composite
/// shape, no joint) with every stage but the step's orchestration in a declared class, at `H`'s
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
pub impl SplitStepConfig<impl H: ClassHashes> of StepConfig {
    impl Dispatcher = FamilyDispatcher<H>;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = PairLoopNarrowPhase<FamilyDispatcher<H>, NoSensors, NoComposites>;
    impl Broad = LibraryCallBroadPhase<H>;
    impl Islands = LibraryCallIslands<H>;
    impl Advance = LibraryCallSolveAdvance<H>;
    impl Mass = LibraryCallMass<H>;
}

/// [`SplitStepConfig`] with the contact generation batched per shape-pair family
/// (`BatchedNarrowPhase<FamilyBatch>`: one call of each family class per step).
pub impl SplitBatchedStepConfig<impl H: ClassHashes> of StepConfig {
    impl Dispatcher = FamilyDispatcher<H>;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = BatchedNarrowPhase<FamilyBatch<H>, NoSensors>;
    impl Broad = LibraryCallBroadPhase<H>;
    impl Islands = LibraryCallIslands<H>;
    impl Advance = LibraryCallSolveAdvance<H>;
    impl Mass = LibraryCallMass<H>;
}

/// [`SplitStepConfig`] with `HybridSolveAdvance`: no solve-and-advance call on a step without a
/// touching pair (its free bodies move in the caller).
pub impl SplitHybridStepConfig<impl H: ClassHashes> of StepConfig {
    impl Dispatcher = FamilyDispatcher<H>;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = PairLoopNarrowPhase<FamilyDispatcher<H>, NoSensors, NoComposites>;
    impl Broad = LibraryCallBroadPhase<H>;
    impl Islands = LibraryCallIslands<H>;
    impl Advance = HybridSolveAdvance<H>;
    impl Mass = LibraryCallMass<H>;
}

/// CS4's layout: `BasicStepConfig` with the contact generation of each pair in the family classes
/// (`FamilyDispatcher`) and the island solve in `SolverClass` (`LibraryCallSolver`), every other
/// stage in the caller.
pub impl ContactSolveStepConfig<impl H: ClassHashes> of StepConfig {
    impl Dispatcher = FamilyDispatcher<H>;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = LibraryCallSolver<H>;
    impl Narrow = PairLoopNarrowPhase<FamilyDispatcher<H>, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<LibraryCallSolver<H>>;
    impl Mass = InProcessMass;
}
