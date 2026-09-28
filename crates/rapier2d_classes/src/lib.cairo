//! The `rapier2d` step split across declared Starknet classes (work package CS4, analysis in
//! `docs/research/class-split.md`), so that no class of a game's step exceeds the size limit of a
//! declared class.
//!
//! `StepConfig` (`rapier2d::pipeline::config`) hands the stages of the step to strategies: the
//! contact generation of each pair (its `ContactDispatcher`), the island solve (its
//! `JointStrategy`) and, since CS5, the narrow phase's pair loop, the broad phase, the island
//! stage, the fused solve and position update and the mass properties (`Narrow`, `Broad`,
//! `Islands`, `Advance`, `Mass`). This crate provides the declared classes that run them and the
//! strategies that library-call those classes:
//!
//! * `ContactBallClass` (pairs with a ball) and `ContactPolygonClass` (cuboid, convex polygon and
//!   half-space pairs), called by [`contact::FamilyDispatcher`] once per pair whose AABBs overlap,
//!   or by [`contact::FamilyBatch`] once per step with all of them;
//! * `SolveAdvanceClass` ([`advance`]), `IslandsClass` ([`islands`]), `BroadPhaseClass`
//!   ([`broad_phase`]) and `MassClass` ([`mass`]): the other stages;
//! * `SolverClass`, called by [`solver::LibraryCallSolver`] (CS4's layout);
//! * [`config::SplitStepConfig`]: `BasicStepConfig` with every stage library-called, generic over
//!   the [`hashes::ClassHashes`] of the declared classes (constants the game supplies after
//!   declaring them); its measured alternatives and CS4's layout.
//!
//! Results are bit-identical to `BasicStepConfig`'s (`tests/split.cairo`).

pub mod advance;
pub mod arena;
pub mod broad_phase;
pub mod config;
pub mod contact;
pub mod hashes;
pub mod islands;
pub mod mass;
pub mod solver;

pub use advance::{HybridSolveAdvance, LibraryCallSolveAdvance};
pub use broad_phase::LibraryCallBroadPhase;
pub use config::{
    ContactSolveStepConfig, SplitBatchedStepConfig, SplitHybridStepConfig, SplitStepConfig,
};
pub use contact::{FamilyBatch, FamilyDispatcher, ManifoldGeometry};
pub use hashes::ClassHashes;
pub use islands::LibraryCallIslands;
pub use mass::LibraryCallMass;
pub use solver::{LibraryCallSolver, SolverManifold};
