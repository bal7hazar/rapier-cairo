//! The `rapier2d` step split across declared Starknet classes (work packages CS4–CS6, analysis in
//! `docs/research/class-split.md`), so that no class of a game's step exceeds the size limit of a
//! declared class.
//!
//! `StepConfig` (`rapier2d::pipeline::config`) hands the contact generation of each pair (its
//! `ContactDispatcher`) and the island solve (its `JointStrategy`) to strategies; `StageConfig`
//! (`rapier2d::pipeline::stages`, CS5 / CS6) hands them the narrow phase's pair loop, the broad
//! phase, the island stage, the fused solve and position update, the mass properties, the rebuild
//! of the active set and the force events, and chooses what the caller class compiles. This crate
//! provides the declared classes that run the stages and the strategies that library-call them:
//!
//! * `ContactBallClass` (pairs with a ball) and `ContactPolygonClass` (cuboid, convex polygon and
//!   half-space pairs), called by [`contact::FamilyDispatcher`] once per pair whose AABBs overlap,
//!   or by [`contact::FamilyBatch`] once per step with all of them;
//! * `NarrowPhaseClass` ([`narrow`], CS6, CX2): the pair loop and the contacts of the pairs without
//!   a ball (the polygon family, computed in the class); it calls `ContactBallClass` once per step
//!   for the pairs with a ball;
//! * `SolveAdvanceClass` ([`advance`]), `IslandsClass` ([`islands`]), `BroadPhaseClass`
//!   ([`broad_phase`]), `MassClass` ([`mass`]), `ActiveSetClass` ([`active_set`], CS6) and
//!   `ForceEventsClass` ([`forces`], CS6): the other stages;
//! * `SolverClass`, called by [`solver::LibraryCallSolver`] (CS4's layout);
//! * [`config::SlimSplitStages`] (CS6): the stages of a caller class under the limit, generic over
//!   the [`hashes::ClassHashes`] of the declared classes (constants the game supplies after
//!   declaring them); CS5's [`config::SplitStages`], the measured alternatives and CS4's layout;
//! * [`orchestrator`]: CS6's route (b), measured and not shipped.
//!
//! Results are bit-identical to `BasicStepConfig`'s (`tests/split.cairo`, `tests/slim.cairo`).

pub mod active_set;
pub mod advance;
pub mod arena;
pub mod broad_phase;
pub mod config;
pub mod contact;
pub mod forces;
pub mod hashes;
pub mod islands;
pub mod mass;
pub mod narrow;
pub mod orchestrator;
pub mod solver;

pub use active_set::LibraryCallActiveSet;
pub use advance::{HybridSolveAdvance, LibraryCallSolveAdvance};
pub use broad_phase::LibraryCallBroadPhase;
pub use config::{
    ContactSolveStepConfig, SlimSplitStages, SplitBatchedStages, SplitHybridStages, SplitStages,
};
pub use contact::{FamilyBatch, FamilyDispatcher, ManifoldGeometry};
pub use forces::LibraryCallForceEvents;
pub use hashes::ClassHashes;
pub use islands::LibraryCallIslands;
pub use mass::LibraryCallMass;
pub use narrow::LibraryCallNarrowPhase;
pub use solver::{LibraryCallSolver, SolverManifold};
