//! The batched narrow phase (`narrow::BatchedNarrowPhase`) in process against the full step:
//! raw equality of bodies, colliders, pairs and events after every step (`config::tests::
//! run_both`), on the game-shaped levels (user changes, sleep, the active set), the P3 scenes and
//! a level with a sensor; and the rejection of composite pairs.

use fixed::ONE;
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::narrow_phase::strategies::{NoComposites, NoSensors, SensorIntersections};
use crate::dispatcher::{BasicShapesDispatcher, DefaultDispatcher};
use crate::pipeline::config::tests::{ball_over, basic_level, run_both, v};
use crate::pipeline::config::{NoJoints, StepConfig};
use crate::pipeline::fixtures::p3_scene;
use crate::world::WorldTrait;
use super::narrow::{BatchedNarrowPhase, InProcessBatch};
use super::{InProcessBroadPhase, InProcessIslands, InProcessMass, InProcessSolveAdvance};

/// `BasicStepConfig` with the batched narrow phase in process.
impl BatchedBasic of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = BatchedNarrowPhase<InProcessBatch<BasicShapesDispatcher>, NoSensors>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
}

/// [`BatchedBasic`] with sensors.
impl BatchedWithSensors of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = SensorIntersections;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = BatchedNarrowPhase<InProcessBatch<BasicShapesDispatcher>, SensorIntersections>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
}

/// Every shape pair of the full dispatcher, batched (composite pairs rejected).
impl BatchedDefault of StepConfig {
    impl Dispatcher = DefaultDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
    impl Narrow = BatchedNarrowPhase<InProcessBatch<DefaultDispatcher>, NoSensors>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<NoJoints>;
    impl Mass = InProcessMass;
}

/// Random game-shaped levels with user changes (wake-ups, removals, insertions, moves).
#[test]
#[fuzzer(runs: 4, seed: 20260928)]
fn fuzz_batched_agrees(seed: u16) {
    let mut full = basic_level(seed.into());
    let mut batched = basic_level(seed.into());
    let _ = run_both::<BatchedBasic>(ref full, ref batched, seed.into(), 14, true, false);
}

/// Levels without user changes (sleep, then the active set's flight) and the P3 scenes of basic
/// shapes.
#[test]
fn test_batched_agrees_on_levels_and_scenes() {
    let mut taken = 0;
    for seed in array![0_u32, 1].span() {
        let mut full = basic_level(*seed);
        let mut batched = basic_level(*seed);
        taken += run_both::<BatchedBasic>(ref full, ref batched, *seed, 14, false, *seed == 1);
    }
    assert!(taken != 0, "the active set was never taken");
    for (id, n) in array![('balls', 8_u32), ('stack', 5), ('cubes', 4), ('row', 4)].span() {
        let mut full = p3_scene(*id, *n);
        let mut batched = p3_scene(*id, *n);
        let _ = run_both::<BatchedBasic>(ref full, ref batched, 0, 4, false, true);
    }
}

/// A sensor over a level: the intersection pairs and their events agree.
#[test]
fn test_batched_with_sensors_agrees() {
    let sensor = ColliderBuilderTrait::ball(ONE)
        .sensor(true)
        .active_events(COLLISION_EVENTS)
        .translation(v(fixed::ZERO, ONE))
        .build();
    let mut full = basic_level(3);
    let _ = full.insert_collider(sensor, None);
    let mut batched = basic_level(3);
    let _ = batched.insert_collider(sensor, None);
    let _ = run_both::<BatchedWithSensors>(ref full, ref batched, 3, 6, false, false);
}

#[test]
#[should_panic(expected: 'Narrow phase: no composites')]
fn test_batched_rejects_composite_pairs() {
    let points = array![v(-ONE, fixed::ZERO), v(fixed::ZERO, fixed::ZERO), v(ONE, fixed::ZERO)];
    let mut world = ball_over(ColliderBuilderTrait::polyline(points.span(), None));
    let _ = world.step_with::<BatchedDefault>();
}
