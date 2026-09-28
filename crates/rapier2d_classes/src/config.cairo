//! The step configuration of a contract whose step library-calls the declared classes.

use rapier2d::pipeline::config::{NoComposites, NoSensors, StepConfig};
use crate::contact::FamilyDispatcher;
use crate::hashes::ClassHashes;
use crate::solver::LibraryCallSolver;

/// `BasicStepConfig` (balls, cuboids, convex polygons and half-spaces; no sensor, no composite
/// shape, no joint) with the contact generation in `ContactBallClass` / `ContactPolygonClass` and
/// the island solve in `SolverClass`, the classes at `H`'s hashes. Results are bit-identical to
/// `BasicStepConfig`'s.
///
/// # Panics
/// As `BasicStepConfig`; when a library call fails (a class hash that is not declared).
pub impl SplitStepConfig<impl H: ClassHashes> of StepConfig {
    impl Dispatcher = FamilyDispatcher<H>;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = LibraryCallSolver<H>;
}
