//! Rejected candidates of `crate::pipeline::config`, kept for re-measurement (AGENTS §5).
//!
//! [`AssertingNoJoints`]: `NoJoints` with its check in `joint_free` (a constant `true` for the
//! rest of the step). Exact Cairo steps of three steps of `free_fall(8)` (the pair-free path,
//! `tests/gas_scenes.cairo`, `steps_basic_free_fall8` against `steps_full_free_fall8`): 135,589
//! against the full step's 135,586 (+1 per step); the shipped `NoJoints` checks in `entries`,
//! which the pair-free path and the sparse step never call: 135,586 (equal). Same results.

use core::dict::Felt252Dict;
use rapier_core::Handle;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_dynamics2d::joint::{ImpulseJoint, ImpulseJointSet, ImpulseJointSetTrait};
use rapier_dynamics2d::rigid_body_set::RigidBody;
use rapier_dynamics2d::solver::island::{SolvedIsland, SolverInput, solve_island_input_contacts};
use rapier_geometry2d::contact::ContactManifold;
use super::{BasicShapesDispatcher, JointStrategy, NoComposites, NoSensors, StepConfig, errors};

pub impl AssertingNoJoints of JointStrategy {
    #[inline(always)]
    fn joint_free(joints: @ImpulseJointSet) -> bool {
        assert(joints.len() == 0, errors::JOINTS);
        true
    }

    #[inline(always)]
    fn entries(ref joints: ImpulseJointSet) -> Array<(Handle, ImpulseJoint)> {
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

    #[inline(always)]
    fn solve(
        params: IntegrationParameters,
        input: SolverInput,
        manifolds: Span<ContactManifold>,
        ref joints: Array<ImpulseJoint>,
    ) -> SolvedIsland {
        solve_island_input_contacts(params, input, manifolds)
    }

    #[inline(always)]
    fn write(
        joint_entries: Span<(Handle, ImpulseJoint)>,
        joints: Span<ImpulseJoint>,
        ref impulse_joints: ImpulseJointSet,
    ) {}
}

/// `BasicStepConfig` with [`AssertingNoJoints`].
pub impl AssertingBasicStep of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = AssertingNoJoints;
}

#[cfg(test)]
mod tests {
    use rapier_dynamics2d::rigid_body_set::RigidBodySetTrait;
    use rapier_testing::opaque;
    use crate::pipeline::fixtures::free_fall;
    use crate::world::WorldTrait;
    use super::AssertingBasicStep;
    use super::super::BasicStepConfig;

    #[test]
    fn gas_baseline() {
        let _ = opaque(1_u32);
    }

    /// Both candidates agree on the pair-free path; run with `--tracked-resource cairo-steps`.
    #[test]
    fn steps_free_fall8_shipped_and_asserting() {
        let mut shipped = free_fall(opaque(8));
        let mut asserting = free_fall(opaque(8));
        let mut k = 0;
        while k != 3 {
            let _ = shipped.step_with::<BasicStepConfig>();
            let _ = asserting.step_with::<AssertingBasicStep>();
            k += 1;
        }
        assert!(shipped.bodies.iter() == asserting.bodies.iter());
    }
}
