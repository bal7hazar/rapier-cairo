//! The stages of the step as strategies (work package CS5): the broad phase, the narrow phase's
//! pair loop, the island stage, the fused solve and advance and the mass properties, each a slot
//! of `StepConfig` (`config`), so that a contract can run a stage in another declared class
//! (`rapier2d_classes`) while the step keeps the world.
//!
//! Every slot has an in-process implementation, which is what `DefaultStepConfig` and
//! `BasicStepConfig` name: an `#[inline(always)]` forward to the function the step called before
//! the slot existed, with the same arguments, so that a program that names them compiles the same
//! code (same Cairo steps, same program felts). The other implementations must give the same
//! results (bit-identical worlds and events on the worlds they support).
//!
//! * [`BroadPhaseStage`]: the candidate pairs of the whole step (`find_pairs`) and of the
//!   active-set step (`find_pairs_sparse`, on the static proxies near the active ones);
//! * [`NarrowPhaseStage`]: the pair loop (`compute_contacts_from_scratch_with`): carry-over,
//!   filters, contact generation, solver data, collision events. [`PairLoopNarrowPhase`] calls
//!   the dispatcher once per pair; `narrow::BatchedNarrowPhase` collects the pairs first and
//!   hands their contact generation to a `narrow::ContactBatch` at once (one call per shape-pair
//!   family when the batch is library-called);
//! * [`IslandStage`]: `islands::update_islands` and `sleeping::islands_after_insertions` (sleep and
//!   wake-up decisions, written to the bodies);
//! * [`SolveAdvanceStage`]: `fused::solve_and_advance_sleeping_with` (constraints, island solve,
//!   free bodies, position update, sleep timers, collider poses);
//! * [`MassStage`]: `user_changes::recompute_mass_properties_from_colliders` (the user-change
//!   stage's mass recomputation).

use fixed::Fixed;
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_dynamics2d::collider_set::ColliderSet;
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::joint::{ImpulseJoint, ImpulseJointSet};
use rapier_dynamics2d::narrow_phase::strategies::{CompositeStrategy, IntersectionStrategy};
use rapier_dynamics2d::narrow_phase::{
    ContactDispatcher, ContactPair, NarrowPhase, PairCollider, compute_contacts_from_scratch_with,
};
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet};
use rapier_geometry2d::broad_phase::{BroadPhaseProxy, find_pairs, find_pairs_sparse};
use super::config::JointStrategy;
use super::fused::solve_and_advance_sleeping_with;
use super::islands::{SleepCensus, update_islands};
use super::sleeping::islands_after_insertions;
use super::user_changes::recompute_mass_properties_from_colliders;

/// The batched narrow phase (`BatchedNarrowPhase`, `ContactBatch`).
pub mod narrow;
#[cfg(test)]
mod tests;

/// The broad phase of the step.
pub trait BroadPhaseStage {
    /// The candidate pairs of `proxies` (`rapier_geometry2d::broad_phase::find_pairs`: indices
    /// into `proxies`, `i < j`, ascending).
    fn find_pairs(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)>;
    /// The candidate pairs of the active-set step (`broad_phase::find_pairs_sparse`: indices
    /// into `statics ++ dynamic`).
    fn find_pairs_sparse(
        statics: Span<BroadPhaseProxy>, dynamic: Span<BroadPhaseProxy>,
    ) -> Array<(u32, u32)>;
}

/// The broad phase in process.
pub impl InProcessBroadPhase of BroadPhaseStage {
    #[inline(always)]
    fn find_pairs(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
        find_pairs(proxies)
    }

    #[inline(always)]
    fn find_pairs_sparse(
        statics: Span<BroadPhaseProxy>, dynamic: Span<BroadPhaseProxy>,
    ) -> Array<(u32, u32)> {
        find_pairs_sparse(statics, dynamic)
    }
}

/// The narrow phase's pair loop of the step.
pub trait NarrowPhaseStage {
    /// Rebuilds `narrow_phase.pairs` from the broad-phase `pairs` (indices into `scratch`) and
    /// returns the collision events, as `narrow_phase::compute_contacts_from_scratch_with`.
    fn compute_contacts(
        ref narrow_phase: NarrowPhase,
        prediction: Fixed,
        scratch: Span<PairCollider>,
        pairs: Span<(u32, u32)>,
        ref colliders: ColliderSet,
    ) -> Array<CollisionEvent>;
}

/// The pair loop in process, one dispatcher call per pair (`D`), sensors by `S`, composite pairs
/// by `K`: what the step ran before CS5.
pub impl PairLoopNarrowPhase<
    impl D: ContactDispatcher, impl S: IntersectionStrategy, impl K: CompositeStrategy,
> of NarrowPhaseStage {
    #[inline(always)]
    fn compute_contacts(
        ref narrow_phase: NarrowPhase,
        prediction: Fixed,
        scratch: Span<PairCollider>,
        pairs: Span<(u32, u32)>,
        ref colliders: ColliderSet,
    ) -> Array<CollisionEvent> {
        compute_contacts_from_scratch_with::<
            D, S, K,
        >(ref narrow_phase, prediction, scratch, pairs, ref colliders)
    }
}

/// The island stage of the step (sleep and wake-up decisions).
pub trait IslandStage {
    /// As `islands::update_islands`: every body whose activation or velocities change is written
    /// to `bodies`; returns the entries with those bodies, whether a member sleeps after the
    /// stage and whether a body woke up.
    fn update_islands(
        ref bodies: RigidBodySet,
        pairs: Span<ContactPair>,
        dormant: Span<ContactPair>,
        joints: Span<(Handle, ImpulseJoint)>,
        entries: Span<(Handle, RigidBody)>,
        census: SleepCensus,
    ) -> (Span<(Handle, RigidBody)>, bool, bool);
    /// As `sleeping::islands_after_insertions`: the island stage of a step that met colliders
    /// inserted since the last step (`fresh`).
    fn islands_after_insertions(
        ref bodies: RigidBodySet,
        pairs: Span<ContactPair>,
        dormant: Span<ContactPair>,
        joints: Span<(Handle, ImpulseJoint)>,
        entries: Span<(Handle, RigidBody)>,
        census: SleepCensus,
        fresh: Span<Handle>,
    ) -> (Span<(Handle, RigidBody)>, bool, bool);
}

/// The island stage in process.
pub impl InProcessIslands of IslandStage {
    #[inline(always)]
    fn update_islands(
        ref bodies: RigidBodySet,
        pairs: Span<ContactPair>,
        dormant: Span<ContactPair>,
        joints: Span<(Handle, ImpulseJoint)>,
        entries: Span<(Handle, RigidBody)>,
        census: SleepCensus,
    ) -> (Span<(Handle, RigidBody)>, bool, bool) {
        update_islands(ref bodies, pairs, dormant, joints, entries, census)
    }

    #[inline(always)]
    fn islands_after_insertions(
        ref bodies: RigidBodySet,
        pairs: Span<ContactPair>,
        dormant: Span<ContactPair>,
        joints: Span<(Handle, ImpulseJoint)>,
        entries: Span<(Handle, RigidBody)>,
        census: SleepCensus,
        fresh: Span<Handle>,
    ) -> (Span<(Handle, RigidBody)>, bool, bool) {
        islands_after_insertions(ref bodies, pairs, dormant, joints, entries, census, fresh)
    }
}

/// The fused solve and position update of the step.
pub trait SolveAdvanceStage {
    /// As `fused::solve_and_advance_sleeping_with` (see `solve_and_advance_sleeping`): the
    /// active pairs of `narrow_phase` get their solved impulses, every moving body of `entries`
    /// its velocities, pose, world mass properties and sleep timer, its colliders their poses
    /// (read from `snapshot` when it has them, from `colliders` otherwise).
    fn solve_and_advance(
        gravity: Vec2,
        params: IntegrationParameters,
        ref bodies: RigidBodySet,
        ref colliders: ColliderSet,
        ref narrow_phase: NarrowPhase,
        ref impulse_joints: ImpulseJointSet,
        entries: Span<(Handle, RigidBody)>,
        snapshot: Span<(Handle, rapier_dynamics2d::collider::Collider)>,
        joint_entries: Span<(Handle, ImpulseJoint)>,
        sleeping: bool,
    );
}

/// The solve and position update in process, joints handled by `J`.
pub impl InProcessSolveAdvance<impl J: JointStrategy> of SolveAdvanceStage {
    #[inline(always)]
    fn solve_and_advance(
        gravity: Vec2,
        params: IntegrationParameters,
        ref bodies: RigidBodySet,
        ref colliders: ColliderSet,
        ref narrow_phase: NarrowPhase,
        ref impulse_joints: ImpulseJointSet,
        entries: Span<(Handle, RigidBody)>,
        snapshot: Span<(Handle, rapier_dynamics2d::collider::Collider)>,
        joint_entries: Span<(Handle, ImpulseJoint)>,
        sleeping: bool,
    ) {
        solve_and_advance_sleeping_with::<
            J,
        >(
            gravity,
            params,
            ref bodies,
            ref colliders,
            ref narrow_phase,
            ref impulse_joints,
            entries,
            snapshot,
            joint_entries,
            sleeping,
        );
    }
}

/// The mass properties of the user-change stage.
pub trait MassStage {
    /// As `recompute_mass_properties_from_colliders`: `body.mprops` (local and world mass
    /// properties, `max_extent`) from the enabled colliders of `body` found in `colliders`.
    fn recompute_mass_properties(ref body: RigidBody, ref colliders: ColliderSet);
}

/// The mass properties in process.
pub impl InProcessMass of MassStage {
    #[inline(always)]
    fn recompute_mass_properties(ref body: RigidBody, ref colliders: ColliderSet) {
        recompute_mass_properties_from_colliders(ref body, ref colliders);
    }
}
