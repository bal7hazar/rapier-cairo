//! The `rapier2d` step split across declared Starknet classes (work packages CS4–CS7, analysis in
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
//!   a ball (the polygon family, computed in the class); it calls `ContactBallClass` once per ball
//!   pair;
//! * `SolveAdvanceClass` ([`advance`]), `IslandsClass` ([`islands`]), `BroadPhaseClass`
//!   ([`broad_phase`]), `MassClass` ([`mass`]), `ActiveSetClass` ([`active_set`], CS6) and
//!   `ForceEventsClass` ([`forces`], CS6): the other stages;
//! * `SolverClass`, called by [`solver::LibraryCallSolver`] (CS4's layout);
//! * [`config::SlimSplitStages`] (CS6): the stages of a caller class under the limit, generic over
//!   the [`hashes::ClassHashes`] of the declared classes (constants the game supplies after
//!   declaring them); CS5's [`config::SplitStages`], the measured alternatives and CS4's layout;
//! * `WorldEditClass` ([`edits`], CS7): the World edits a game applies between steps (insert a
//!   body with its collider, remove bodies, put bodies to sleep), the world crossing with the
//!   basic codec ([`edits::edit_world`]).
//!
//! Only classes a game declares live here (CS7): the measured route (b)'s `OrchestratorClass`
//! is a fixture of `rapier_sink`.
//!
//! Results are bit-identical to `BasicStepConfig`'s (`tests/split.cairo`, `tests/slim.cairo`).

pub mod active_set;
pub mod advance;
pub mod arena;
pub mod broad_phase;
pub mod config;
pub mod contact;
pub mod edits;
pub mod forces;
pub mod hashes;
pub mod islands;
pub mod mass;
pub mod narrow;
pub mod solver;

pub use active_set::LibraryCallActiveSet;
pub use advance::{HybridSolveAdvance, LibraryCallSolveAdvance};
pub use broad_phase::LibraryCallBroadPhase;
pub use config::{
    ContactSolveStepConfig, SlimSplitStages, SplitBatchedStages, SplitHybridStages, SplitStages,
};
pub use contact::{FamilyBatch, FamilyDispatcher, ManifoldGeometry};
pub use edits::{BodyInsert, WorldEdit, apply_edits, edit_world};
pub use forces::LibraryCallForceEvents;
pub use hashes::ClassHashes;
pub use islands::LibraryCallIslands;
pub use mass::LibraryCallMass;
pub use narrow::LibraryCallNarrowPhase;
pub use solver::{LibraryCallSolver, SolverManifold};
