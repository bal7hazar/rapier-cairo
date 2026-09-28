//! The island solve (constraint generation, sweeps, integration) in a declared class.
//!
//! `StepConfig::Joints` hands the solve to a strategy (`JointStrategy::solve`):
//! [`LibraryCallSolver`] is `NoJoints` with its `solve` library-called. The solve runs only when
//! the step has a touching manifold: a tick without one makes no call. The manifolds cross as
//! [`SolverManifold`]s (what the solve reads of them) and the frozen body steps stay in the caller
//! (CS3: 16,598 Cairo steps per call against 19,047 for the whole `SolverInput`, manifolds and
//! `SolvedIsland`).

use core::dict::Felt252Dict;
use rapier2d::pipeline::config::JointStrategy;
use rapier2d::prelude::{Fixed, Handle, IntegrationParameters, RigidBody};
use rapier_dynamics2d::joint::{ImpulseJoint, ImpulseJointSet, ImpulseJointSetTrait};
use rapier_dynamics2d::solver::body::SolverBody;
use rapier_dynamics2d::solver::island::{ManifoldImpulses, SolvedIsland, SolverInput};
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldData, TrackedContact};
use starknet::SyscallResultTrait;
use starknet::syscalls::library_call_syscall;
use crate::hashes::{ClassHashes, errors};

/// What the solve reads of a manifold (`sweeps::split::generation`): its solver data, its point
/// count and the warm-start impulses (normal, tangent) of its two points.
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

/// A manifold the solve reads as it reads the original (default geometry, which it does not
/// read).
pub fn from_solver_manifold(manifold: SolverManifold) -> ContactManifold {
    let [w0, w1] = manifold.warmstart;
    let mut out: ContactManifold = Default::default();
    out.points = [tracked(w0), tracked(w1)];
    out.num_points = manifold.num_points;
    out.data = manifold.data;
    out
}

/// `NoJoints` with the island solve library-called in `SolverClass` (at `H::solver()`).
///
/// # Panics
/// `rapier2d::pipeline::config::errors::JOINTS` at the first step of a world with an impulse
/// joint; `errors::DECODE` when the class returns something else than its result.
pub impl LibraryCallSolver<impl H: ClassHashes> of JointStrategy {
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
        let mut ret = library_call_syscall(H::solver(), selector!("solve"), calldata.span())
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

/// The island solve of the contacts (`solve_island_input_contacts`).
#[starknet::contract]
pub mod SolverClass {
    use rapier2d::prelude::IntegrationParameters;
    use rapier_dynamics2d::solver::body::SolverBody;
    use rapier_dynamics2d::solver::island::{
        ManifoldImpulses, SolvedIsland, SolverInput, solve_island_input_contacts,
    };
    use super::{SolverManifold, from_solver_manifold};

    #[storage]
    struct Storage {}

    /// `solve_island_input_contacts` on the manifolds of `manifolds`: the solved bodies in input
    /// order and the impulses of every manifold.
    #[external(v0)]
    fn solve(
        self: @ContractState,
        params: IntegrationParameters,
        input: SolverInput,
        manifolds: Span<SolverManifold>,
    ) -> (Span<SolverBody>, Span<ManifoldImpulses>) {
        let mut full = array![];
        for manifold in manifolds {
            full.append(from_solver_manifold(*manifold));
        }
        let SolvedIsland {
            bodies, steps: _, impulses,
        } = solve_island_input_contacts(params, input, full.span());
        (bodies, impulses)
    }
}
