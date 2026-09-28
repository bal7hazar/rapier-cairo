//! `#[executable]` fixtures (work package CS2): the game's world (`scene::game_world`) stepped
//! with contact-force events, one program per step configuration, so that
//! `scripts/bytecode_size.py` reports the program felts a proof of the game hashes (the settled
//! proof path bootloads the whole program and Pedersen-hashes it).
//!
//! Not part of the `rapier_sink` package: `scripts/bytecode_size.py` copies this file and
//! `../src/scene.cairo` into a temporary package with one `[[target.executable]]` per function
//! below (`enable-gas = false`, as the game's executable). Inputs come from the arguments, so that
//! nothing is constant-folded.
//!
//! * `full`: `World::step_with_force_events` (every strategy: `DefaultStepConfig`);
//! * `basic`: `step_with_force_events_with::<BasicStepConfig>` (the game-shaped step);
//! * `no_joints`, `no_sensors`, `no_composites`, `basic_dispatcher`: `DefaultStepConfig` with
//!   one strategy replaced, so that `full` minus each gives that strategy's share.

use rapier2d::pipeline::config::{
    BasicShapesDispatcher, BasicStepConfig, CompositeManifolds, DefaultDispatcher,
    ImpulseJointSolver, NoComposites, NoJoints, NoSensors, SensorIntersections, StepConfig,
};
use rapier2d::prelude::{World, WorldTrait};

mod scene;

impl NoJointsStep of StepConfig {
    impl Dispatcher = DefaultDispatcher;
    impl Sensors = SensorIntersections;
    impl Composites = CompositeManifolds;
    impl Joints = NoJoints;
}

impl NoSensorsStep of StepConfig {
    impl Dispatcher = DefaultDispatcher;
    impl Sensors = NoSensors;
    impl Composites = CompositeManifolds;
    impl Joints = ImpulseJointSolver;
}

impl NoCompositesStep of StepConfig {
    impl Dispatcher = DefaultDispatcher;
    impl Sensors = SensorIntersections;
    impl Composites = NoComposites;
    impl Joints = ImpulseJointSolver;
}

impl BasicDispatcherStep of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = SensorIntersections;
    impl Composites = CompositeManifolds;
    impl Joints = ImpulseJointSolver;
}

/// `(collision events, contact-force events)` of `steps` configured steps of the game world.
fn run<impl C: StepConfig>(dx: i64, dy: i64, steps: u32) -> (u32, u32) {
    let mut world: World = scene::game_world(dx, dy);
    let (mut collisions, mut forces) = (0, 0);
    let mut i = 0;
    while i != steps {
        let (c, f) = world.step_with_force_events_with::<C>();
        collisions += c.len();
        forces += f.len();
        i += 1;
    }
    (collisions, forces)
}

#[executable]
fn full(dx: i64, dy: i64, steps: u32) -> (u32, u32) {
    let mut world = scene::game_world(dx, dy);
    scene::run_with_forces(ref world, steps)
}

#[executable]
fn basic(dx: i64, dy: i64, steps: u32) -> (u32, u32) {
    run::<BasicStepConfig>(dx, dy, steps)
}

#[executable]
fn no_joints(dx: i64, dy: i64, steps: u32) -> (u32, u32) {
    run::<NoJointsStep>(dx, dy, steps)
}

#[executable]
fn no_sensors(dx: i64, dy: i64, steps: u32) -> (u32, u32) {
    run::<NoSensorsStep>(dx, dy, steps)
}

#[executable]
fn no_composites(dx: i64, dy: i64, steps: u32) -> (u32, u32) {
    run::<NoCompositesStep>(dx, dy, steps)
}

#[executable]
fn basic_dispatcher(dx: i64, dy: i64, steps: u32) -> (u32, u32) {
    run::<BasicDispatcherStep>(dx, dy, steps)
}
