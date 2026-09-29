//! The stages of the step as strategies (work package CS5): the broad phase, the narrow phase's
//! pair loop, the island stage, the fused solve and advance and the mass properties, each a slot
//! of a [`StageConfig`], so that a contract can run a stage in another declared class
//! (`rapier2d_classes`) while the step keeps the world. The configuration is the second generic of
//! `step_with_stages::<C, S>` / `step_with_force_events_with_stages::<C, S>`, next to the
//! `StepConfig` `C` (`config`: what the step supports).
//!
//! Every slot has an in-process implementation, which is what [`InProcessStages`] names (and
//! what `step_with::<C>` runs): an `#[inline(always)]` forward to the function the step called
//! before the slot existed, with the same arguments, so that a program that names them compiles
//! the same code (same Cairo steps, same program felts). The other implementations must give the
//! same results (bit-identical worlds and events on the worlds they support).
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
//!
//! # What a slim caller leaves out (CS6)
//!
//! A contract whose step class must stay under the declared-class limit also chooses what the
//! step's own code compiles:
//!
//! * [`ShapeStage`]: the bounding boxes of the broad-phase proxies ([`InProcessShapes`]: every
//!   shape; [`BasicShapeKernels`]: balls, cuboids, convex polygons and half-spaces, the others
//!   rejected with [`errors::NOT_BASIC`]);
//! * [`ForceEventStage`]: the contact-force event collection after the solve
//!   (`force_events::collect` / `collect_convex`);
//! * [`ActiveSetStage`]: the rebuild of the active set after a whole step
//!   (`active_set::rebuild`);
//! * [`FreePathStage`]: the pair-free fast path (`free_path`; [`NoFreePath`]: a step without
//!   pairs takes the ordinary path, same results);
//! * [`StageConfig::KINEMATIC`]: the velocity interpolation of the position-based kinematic
//!   bodies (`kinematic::prepare_existing`; off, a world with one is rejected with
//!   [`errors::KINEMATIC`] at the next step).

use fixed::Fixed;
use glam_core::Vec2;
use rapier_core::Handle;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_dynamics2d::collider_set::ColliderSet;
use rapier_dynamics2d::events::{CollisionEvent, ContactForceEvent};
use rapier_dynamics2d::joint::{ImpulseJoint, ImpulseJointSet};
use rapier_dynamics2d::narrow_phase::strategies::{CompositeStrategy, IntersectionStrategy};
use rapier_dynamics2d::narrow_phase::{
    ContactDispatcher, ContactPair, NarrowPhase, PairCollider, compute_contacts_from_scratch_with,
};
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet};
use rapier_geometry2d::aabb::Aabb;
use rapier_geometry2d::broad_phase::{BroadPhaseProxy, find_pairs, find_pairs_sparse};
use rapier_geometry2d::shape::{
    BallTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, Shape, ShapeTrait,
};
use rapier_math::pose2::Pose2;
use super::active_set::{ActiveSet, rebuild};
use super::config::{JointStrategy, StepConfig};
use super::force_events::collect_either;
use super::fused::solve_and_advance_sleeping_with;
use super::islands::{SleepCensus, update_islands};
use super::sleeping::islands_after_insertions;
use super::user_changes::recompute_mass_properties_from_colliders;

/// The batched narrow phase (`BatchedNarrowPhase`, `ContactBatch`).
pub mod narrow;

/// Panics of the slim stages (CS6).
pub mod errors {
    /// A shape other than a ball, a cuboid, a convex polygon or a half-space met
    /// [`super::BasicShapeKernels`].
    pub const NOT_BASIC: felt252 = 'Step: not a basic shape';
    /// A position-based kinematic body in a step compiled without `KINEMATIC`.
    pub const KINEMATIC: felt252 = 'Step: kinematic disabled';
}
#[cfg(test)]
mod slim_tests;
#[cfg(test)]
mod tests;

/// Where the stages of a step run (see the module documentation), selected statically as the
/// `StepConfig`: `step_with_stages::<C, S>`.
pub trait StageConfig {
    /// The narrow phase's pair loop: `PairLoopNarrowPhase<Dispatcher, Sensors, Composites>` of the
    /// step's `StepConfig` in process.
    impl Narrow: NarrowPhaseStage;
    /// The broad phase: [`InProcessBroadPhase`] in process.
    impl Broad: BroadPhaseStage;
    /// The island stage, sleep and wake-up: [`InProcessIslands`] in process.
    impl Islands: IslandStage;
    /// The fused solve and position update: `InProcessSolveAdvance<Joints>` in process.
    impl Advance: SolveAdvanceStage;
    /// The mass properties of the user changes: [`InProcessMass`] in process.
    impl Mass: MassStage;
    /// The bounding boxes of the proxies (CS6): [`InProcessShapes`] in process.
    impl Shapes: ShapeStage;
    /// The contact-force event collection (CS6): [`InProcessForceEvents`] in process.
    impl Forces: ForceEventStage;
    /// The rebuild of the active set (CS6): `InProcessActiveSet<Shapes>` in process.
    impl Active: ActiveSetStage;
    /// The pair-free fast path (CS6): [`InProcessFreePath`] in process.
    impl Free: FreePathStage;
    /// The position-based kinematic bodies are supported (CS6): `true` in process.
    const KINEMATIC: bool;
}

/// Every stage in process, with the dispatcher and strategies of `C`: what `step_with::<C>`
/// runs.
pub impl InProcessStages<impl C: StepConfig> of StageConfig {
    impl Narrow = PairLoopNarrowPhase<C::Dispatcher, C::Sensors, C::Composites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<C::Joints>;
    impl Mass = InProcessMass;
    impl Shapes = InProcessShapes;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<InProcessShapes>;
    impl Free = InProcessFreePath;
    const KINEMATIC: bool = true;
}

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

/// The shape kernels the step's own code calls (CS6).
pub trait ShapeStage {
    /// The bounding box of `shape` at `pose` (`ShapeTrait::compute_aabb`).
    fn compute_aabb(shape: Shape, pose: Pose2) -> Aabb;
}

/// Every shape: `ShapeTrait::compute_aabb`.
pub impl InProcessShapes of ShapeStage {
    #[inline(always)]
    fn compute_aabb(shape: Shape, pose: Pose2) -> Aabb {
        shape.compute_aabb(pose)
    }
}

/// Balls, cuboids, convex polygons and half-spaces (`BasicShapesDispatcher`'s shapes): the
/// same boxes as `ShapeTrait::compute_aabb`, the other arms not compiled.
///
/// # Panics
/// [`errors::NOT_BASIC`] on any other shape.
pub impl BasicShapeKernels of ShapeStage {
    #[inline(always)]
    fn compute_aabb(shape: Shape, pose: Pose2) -> Aabb {
        match shape {
            Shape::Ball(s) => s.compute_aabb(pose),
            Shape::Cuboid(s) => s.compute_aabb(pose),
            Shape::HalfSpace(s) => s.compute_aabb(pose),
            Shape::ConvexPolygon(s) => s.unbox().compute_aabb(pose),
            _ => core::panic_with_felt252(errors::NOT_BASIC),
        }
    }
}

/// The contact-force event collection of the step (CS6).
pub trait ForceEventStage {
    /// As `force_events::collect` (`groups`) / `collect_convex`: the force events of the pairs of
    /// `narrow`, whose event status is updated.
    fn collect(
        groups: bool, dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
    ) -> Array<ContactForceEvent>;
}

/// The force events in process.
pub impl InProcessForceEvents of ForceEventStage {
    #[inline(always)]
    fn collect(
        groups: bool, dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
    ) -> Array<ContactForceEvent> {
        collect_either(groups, dt, ref narrow, ref colliders)
    }
}

/// The rebuild of the active set after a whole step (CS6).
pub trait ActiveSetStage {
    /// As `active_set::rebuild` (see there for the arguments).
    fn rebuild(
        snapshot: Span<(Handle, rapier_dynamics2d::collider::Collider)>,
        entries: Span<(Handle, RigidBody)>,
        pairs: Span<ContactPair>,
        force_events: bool,
        prediction: Fixed,
    ) -> ActiveSet;
}

/// The rebuild in process, the static proxies' boxes by `A`.
pub impl InProcessActiveSet<impl A: ShapeStage> of ActiveSetStage {
    #[inline(always)]
    fn rebuild(
        snapshot: Span<(Handle, rapier_dynamics2d::collider::Collider)>,
        entries: Span<(Handle, RigidBody)>,
        pairs: Span<ContactPair>,
        force_events: bool,
        prediction: Fixed,
    ) -> ActiveSet {
        rebuild::<A>(snapshot, entries, pairs, force_events, prediction)
    }
}

/// Whether a step compiles the pair-free fast path (`free_path`, CS6).
pub trait FreePathStage {
    /// `false`: a step without pairs takes the ordinary path (same results).
    const ENABLED: bool;
}

/// The pair-free fast path.
pub impl InProcessFreePath of FreePathStage {
    const ENABLED: bool = true;
}

/// No pair-free fast path: its code is not compiled, a step without pairs takes the ordinary
/// path (same results, more Cairo steps on a flight tick).
pub impl NoFreePath of FreePathStage {
    const ENABLED: bool = false;
}
