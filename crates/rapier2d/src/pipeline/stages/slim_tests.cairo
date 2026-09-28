//! The slim stages of a caller class (CS6) in process against the full step: no pair-free fast
//! path, the bounding boxes of the basic shapes only, no position-based kinematic body, the
//! batched pair loop (what `NarrowPhaseClass` runs). Raw equality of bodies, colliders, pairs,
//! events and the active set after every step (`config::tests::run_both_stages`), and the
//! rejections.

use fixed::{HALF, ONE};
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::narrow_phase::strategies::NoSensors;
use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
use crate::dispatcher::BasicShapesDispatcher;
use crate::pipeline::config::BasicStepConfig;
use crate::pipeline::config::tests::{at, ball_over, basic_level, run_both_stages};
use crate::pipeline::fixtures::{free_fall, p3_scene};
use crate::world::WorldTrait;
use super::narrow::{BatchedNarrowPhase, InProcessBatch};
use super::{
    BasicShapeKernels, InProcessActiveSet, InProcessBroadPhase, InProcessForceEvents,
    InProcessIslands, InProcessMass, InProcessSolveAdvance, NoFreePath, StageConfig,
};

/// `rapier2d_classes::SlimSplitStages` with every stage in process.
impl SlimInProcess of StageConfig {
    impl Narrow = BatchedNarrowPhase<InProcessBatch<BasicShapesDispatcher>, NoSensors>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<crate::pipeline::config::NoJoints>;
    impl Mass = InProcessMass;
    impl Shapes = BasicShapeKernels;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<BasicShapeKernels>;
    impl Free = NoFreePath;
    const KINEMATIC: bool = false;
}

/// Random game-shaped levels with user changes (wake-ups, removals, insertions, moves).
#[test]
#[fuzzer(runs: 4, seed: 20260929)]
fn fuzz_slim_agrees(seed: u16) {
    let mut full = basic_level(seed.into());
    let mut slim = basic_level(seed.into());
    let _ = run_both_stages::<
        BasicStepConfig, SlimInProcess,
    >(ref full, ref slim, seed.into(), 14, true, false);
}

/// Levels without user changes (the pair-free flight before the first contact, sleep, the
/// active set), the P3 scenes of basic shapes and a free fall (the pair-free path in process).
#[test]
fn test_slim_agrees_on_levels_and_scenes() {
    let mut taken = 0;
    for seed in array![0_u32, 1, 2].span() {
        let mut full = basic_level(*seed);
        let mut slim = basic_level(*seed);
        taken +=
            run_both_stages::<
                BasicStepConfig, SlimInProcess,
            >(ref full, ref slim, *seed, 16, false, *seed == 1);
    }
    assert!(taken != 0, "the active set was never taken");
    for (id, n) in array![('balls', 8_u32), ('stack', 5), ('cubes', 4), ('row', 4)].span() {
        let mut full = p3_scene(*id, *n);
        let mut slim = p3_scene(*id, *n);
        let _ = run_both_stages::<
            BasicStepConfig, SlimInProcess,
        >(ref full, ref slim, 0, 4, false, true);
    }
    let mut full = free_fall(6);
    let mut slim = free_fall(6);
    let _ = run_both_stages::<
        BasicStepConfig, SlimInProcess,
    >(ref full, ref slim, 0, 4, false, true);
}

#[test]
#[should_panic(expected: 'Step: not a basic shape')]
fn test_slim_rejects_other_shapes() {
    let mut world = ball_over(ColliderBuilderTrait::capsule_y(HALF, HALF));
    let _ = world.step_with_stages::<BasicStepConfig, SlimInProcess>();
}

#[test]
#[should_panic(expected: 'Step: kinematic disabled')]
fn test_slim_rejects_position_based_kinematic_bodies() {
    let mut world = basic_level(0);
    let _ = world
        .insert(
            RigidBodyTrait::kinematic_position_based(at(ONE, ONE)),
            ColliderBuilderTrait::cuboid(HALF, HALF).build(),
        );
    let _ = world.step_with_stages::<BasicStepConfig, SlimInProcess>();
}
