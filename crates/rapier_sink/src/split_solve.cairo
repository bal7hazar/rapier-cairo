//! PROTOTYPE (branch `proto/cs3-phase-dispatch`, never merged): the island solve in its own
//! declared class. Needs `Serde` on `SolverInput` / `SolvedIsland` (`rapier_dynamics2d`), the only
//! engine change: `StepConfig::Joints` already hands the solve to a strategy
//! (`JointStrategy::solve`), so [`LibraryCallSolver`] (the `NoJoints` strategy with its `solve`
//! library-called) moves the constraint generation, the sweeps and the integration out of the
//! caller class. The solver runs only when the step has a touching manifold (or a joint): a flight
//! tick makes no call.
//!
//! * `SolverClass`: `solve_island_input_contacts` behind one entry point.
//! * `SplitSolveStep`: `BasicGameStep` with the solve in `SolverClass` (two classes).
//! * `Split3Step`: the contact generation in `ContactClass` too (three classes).
//! * `Split4Step`: the contact generation in `ContactBallClass` / `ContactPolygonClass` and the
//!   solve in `SolverClass` (four classes).

use core::dict::Felt252Dict;
use rapier2d::dispatcher::BasicShapesDispatcher;
use rapier2d::pipeline::config::{JointStrategy, NoComposites, NoSensors, StepConfig};
use rapier2d::prelude::{Fixed, Handle, IntegrationParameters, RigidBody};
use rapier_dynamics2d::joint::{ImpulseJoint, ImpulseJointSet, ImpulseJointSetTrait};
use rapier_dynamics2d::solver::body::SolverBody;
use rapier_dynamics2d::solver::island::{ManifoldImpulses, SolvedIsland, SolverInput};
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldData, TrackedContact};
use starknet::SyscallResultTrait;
use starknet::syscalls::library_call_syscall;
use crate::family::FamilyDispatcher;
use crate::split::{LibraryCallDispatcher, class_at, errors};

/// Storage slot of the solver class hash (`selector!("solver_class")`).
pub const SOLVER_CLASS_SLOT: felt252 = selector!("solver_class");

/// What the solve reads of a manifold (`sweeps::split::generation`): its solver data, its point
/// count and the warm-start impulses of its two points; about 2/3 of a manifold's felts.
#[derive(Copy, Drop, Serde)]
pub struct SolverManifold {
    pub data: ContactManifoldData,
    pub num_points: u8,
    pub warmstart: [(Fixed, Fixed); 2],
}

fn warmstart(point: TrackedContact) -> (Fixed, Fixed) {
    (point.data.warmstart_impulse, point.data.warmstart_tangent_impulse)
}

fn tracked(warmstart: (Fixed, Fixed)) -> TrackedContact {
    let (impulse, tangent) = warmstart;
    let mut point: TrackedContact = Default::default();
    point.data.warmstart_impulse = impulse;
    point.data.warmstart_tangent_impulse = tangent;
    point
}

/// `manifold`'s solver view.
pub fn solver_manifold(manifold: @ContactManifold) -> SolverManifold {
    let [p0, p1] = *manifold.points;
    SolverManifold {
        data: *manifold.data,
        num_points: *manifold.num_points,
        warmstart: [warmstart(p0), warmstart(p1)],
    }
}

/// A manifold the solve reads as it reads the original (default geometry, which it does not read).
pub fn from_solver_manifold(manifold: SolverManifold) -> ContactManifold {
    let [w0, w1] = manifold.warmstart;
    let mut out: ContactManifold = Default::default();
    out.points = [tracked(w0), tracked(w1)];
    out.num_points = manifold.num_points;
    out.data = manifold.data;
    out
}

/// `NoJoints`, the solve library-called in the class at [`SOLVER_CLASS_SLOT`]
/// (`SolverClass::solve_compact`): the manifolds cross as [`SolverManifold`]s and the frozen
/// body steps stay in the caller. Measured against the whole crossing
/// ([`FullLibraryCallSolver`]).
pub impl LibraryCallSolver of JointStrategy {
    #[inline(always)]
    fn joint_free(joints: @ImpulseJointSet) -> bool {
        joints.len() == 0
    }

    #[inline(always)]
    fn entries(ref joints: ImpulseJointSet) -> Array<(Handle, ImpulseJoint)> {
        assert(joints.len() == 0, rapier2d::pipeline::config::errors::JOINTS);
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

    fn solve(
        params: IntegrationParameters,
        input: SolverInput,
        manifolds: Span<ContactManifold>,
        ref joints: Array<ImpulseJoint>,
    ) -> SolvedIsland {
        let steps = input.steps.span();
        let mut calldata = array![];
        params.serialize(ref calldata);
        input.serialize(ref calldata);
        calldata.append(manifolds.len().into());
        for manifold in manifolds {
            solver_manifold(manifold).serialize(ref calldata);
        }
        let mut ret = library_call_syscall(
            class_at(SOLVER_CLASS_SLOT), selector!("solve_compact"), calldata.span(),
        )
            .unwrap_syscall();
        let (bodies, impulses): (Span<SolverBody>, Span<ManifoldImpulses>) = Serde::deserialize(
            ref ret,
        )
            .expect(errors::DECODE);
        SolvedIsland { bodies, steps, impulses }
    }

    #[inline(always)]
    fn write(
        joint_entries: Span<(Handle, ImpulseJoint)>,
        joints: Span<ImpulseJoint>,
        ref impulse_joints: ImpulseJointSet,
    ) {}
}

/// Measured and rejected: [`LibraryCallSolver`] with the whole `SolverInput`, manifolds and
/// `SolvedIsland` crossing (`SolverClass::solve`).
pub impl FullLibraryCallSolver of JointStrategy {
    #[inline(always)]
    fn joint_free(joints: @ImpulseJointSet) -> bool {
        joints.len() == 0
    }

    #[inline(always)]
    fn entries(ref joints: ImpulseJointSet) -> Array<(Handle, ImpulseJoint)> {
        assert(joints.len() == 0, rapier2d::pipeline::config::errors::JOINTS);
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

    fn solve(
        params: IntegrationParameters,
        input: SolverInput,
        manifolds: Span<ContactManifold>,
        ref joints: Array<ImpulseJoint>,
    ) -> SolvedIsland {
        let mut calldata = array![];
        params.serialize(ref calldata);
        input.serialize(ref calldata);
        manifolds.serialize(ref calldata);
        let mut ret = library_call_syscall(
            class_at(SOLVER_CLASS_SLOT), selector!("solve"), calldata.span(),
        )
            .unwrap_syscall();
        Serde::deserialize(ref ret).expect(errors::DECODE)
    }

    #[inline(always)]
    fn write(
        joint_entries: Span<(Handle, ImpulseJoint)>,
        joints: Span<ImpulseJoint>,
        ref impulse_joints: ImpulseJointSet,
    ) {}
}

/// `Split4StepConfig` with [`FullLibraryCallSolver`].
pub impl Split4FullSolveStepConfig of StepConfig {
    impl Dispatcher = FamilyDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = FullLibraryCallSolver;
}

/// `BasicStepConfig` with the solve in another class.
pub impl SplitSolveStepConfig of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = LibraryCallSolver;
}

/// `BasicStepConfig` with the contact generation and the solve in two other classes.
pub impl Split3StepConfig of StepConfig {
    impl Dispatcher = LibraryCallDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = LibraryCallSolver;
}

/// `BasicStepConfig` with the contact generation in the two family classes and the solve in a
/// third one.
pub impl Split4StepConfig of StepConfig {
    impl Dispatcher = FamilyDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = LibraryCallSolver;
}

#[starknet::contract]
pub mod SolverClass {
    use rapier2d::prelude::IntegrationParameters;
    use rapier_dynamics2d::solver::body::SolverBody;
    use rapier_dynamics2d::solver::island::{
        ManifoldImpulses, SolvedIsland, SolverInput, solve_island_input_contacts,
    };
    use rapier_geometry2d::contact::ContactManifold;

    #[storage]
    struct Storage {}

    /// `solve_island_input_contacts`: the solved bodies and the impulses of every manifold.
    #[external(v0)]
    fn solve(
        self: @ContractState,
        params: IntegrationParameters,
        input: SolverInput,
        manifolds: Span<ContactManifold>,
    ) -> SolvedIsland {
        solve_island_input_contacts(params, input, manifolds)
    }

    /// [`solve`] on [`super::SolverManifold`]s: the solved bodies and the impulses.
    #[external(v0)]
    fn solve_compact(
        self: @ContractState,
        params: IntegrationParameters,
        input: SolverInput,
        manifolds: Span<super::SolverManifold>,
    ) -> (Span<SolverBody>, Span<ManifoldImpulses>) {
        let mut full = array![];
        for manifold in manifolds {
            full.append(super::from_solver_manifold(*manifold));
        }
        let SolvedIsland {
            bodies, steps: _, impulses,
        } = solve_island_input_contacts(params, input, full.span());
        (bodies, impulses)
    }
}

#[starknet::contract]
pub mod SplitSolveStep {
    use rapier2d::prelude::{WorldState, WorldTrait};
    use starknet::ClassHash;
    use starknet::storage::StoragePointerWriteAccess;
    use super::SplitSolveStepConfig;

    #[storage]
    struct Storage {
        solver_class: ClassHash,
    }

    #[constructor]
    fn constructor(ref self: ContractState, solver_class: ClassHash) {
        self.solver_class.write(solver_class);
    }

    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world.step_with_force_events_with::<SplitSolveStepConfig>();
            i += 1;
        }
        world.into_state()
    }
}

#[starknet::contract]
pub mod Split3Step {
    use rapier2d::prelude::{WorldState, WorldTrait};
    use starknet::ClassHash;
    use starknet::storage::StoragePointerWriteAccess;
    use super::Split3StepConfig;

    #[storage]
    struct Storage {
        contact_class: ClassHash,
        solver_class: ClassHash,
    }

    #[constructor]
    fn constructor(ref self: ContractState, contact_class: ClassHash, solver_class: ClassHash) {
        self.contact_class.write(contact_class);
        self.solver_class.write(solver_class);
    }

    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world.step_with_force_events_with::<Split3StepConfig>();
            i += 1;
        }
        world.into_state()
    }
}

#[starknet::contract]
pub mod Split4Step {
    use rapier2d::prelude::{WorldState, WorldTrait};
    use starknet::ClassHash;
    use starknet::storage::StoragePointerWriteAccess;
    use super::Split4StepConfig;

    #[storage]
    struct Storage {
        contact_ball_class: ClassHash,
        contact_polygon_class: ClassHash,
        solver_class: ClassHash,
    }

    #[constructor]
    fn constructor(
        ref self: ContractState,
        contact_ball_class: ClassHash,
        contact_polygon_class: ClassHash,
        solver_class: ClassHash,
    ) {
        self.contact_ball_class.write(contact_ball_class);
        self.contact_polygon_class.write(contact_polygon_class);
        self.solver_class.write(solver_class);
    }

    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world.step_with_force_events_with::<Split4StepConfig>();
            i += 1;
        }
        world.into_state()
    }
}
