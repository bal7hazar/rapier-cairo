//! Tests and `gas_*` probes of `crate::pipeline::config`: every configuration against the full
//! step on the worlds it supports (raw equality of bodies, colliders, pairs and events after
//! every step), and the rejection of the worlds it does not.

use fixed::{Fixed, HALF, ONE, ZERO};
use glam_core::Vec2;
use rapier_core::collider::events::{COLLISION_EVENTS, CONTACT_FORCE_EVENTS};
use rapier_dynamics2d::collider::{ColliderBuilder, ColliderBuilderTrait};
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::joint::RevoluteJointBuilderTrait;
use rapier_dynamics2d::narrow_phase::strategies::{NoComposites, SensorIntersections};
use rapier_dynamics2d::rigid_body_set::{RigidBodyBuilderTrait, RigidBodySetTrait, RigidBodyTrait};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::dispatcher::{BasicShapesDispatcher, DefaultDispatcher};
use crate::pipeline::active_set::usable;
use crate::pipeline::ccd::{CCDSolver, CCDSolverTrait};
use crate::pipeline::fixtures::{draw, free_fall, p3_scene};
use crate::world::{World, WorldTrait};
use super::{
    BasicStepConfig, DefaultStepConfig, ImpulseJointSolver, InProcessStages, NoJoints, StageConfig,
    StepConfig,
};

/// Basic shapes and sensors (a mix a game may pick).
impl BasicWithSensors of StepConfig {
    impl Dispatcher = BasicShapesDispatcher;
    impl Sensors = SensorIntersections;
    impl Composites = NoComposites;
    impl Joints = NoJoints;
}

/// Every shape but composites, with joints.
impl NoCompositeShapes of StepConfig {
    impl Dispatcher = DefaultDispatcher;
    impl Sensors = SensorIntersections;
    impl Composites = NoComposites;
    impl Joints = ImpulseJointSolver;
}

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

pub(crate) fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

pub(crate) fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: v(x, y), rotation: Rot2 { re: ONE, im: ZERO } }
}

fn pentagon() -> ColliderBuilder {
    let q = f(1073741824);
    let points = array![
        v(-HALF, -HALF), v(HALF, -HALF), v(HALF + q, ZERO), v(ZERO, HALF), v(-HALF - q, ZERO),
    ];
    ColliderBuilderTrait::convex_polygon(points.span()).unwrap()
}

/// A game-shaped level for `seed`: a half-space ground and a fixed cuboid platform, 3–6 blocks
/// (cuboids and pentagons, stacked or side by side) that fall asleep after two calm steps, and a
/// ball fired at them from the left; every collider reports collision and force events.
/// Blocks stand 2 apart (a pentagon is 1.5 wide).
pub(crate) fn basic_level(seed: u32) -> World {
    let mut state: u64 = seed.into();
    let mut world: World = Default::default();
    let events = COLLISION_EVENTS | CONTACT_FORCE_EVENTS;
    let _ = world
        .insert_collider(
            ColliderBuilderTrait::halfspace(v(ZERO, ONE)).active_events(events).build(), None,
        );
    let _ = world
        .insert(
            RigidBodyTrait::fixed(at(f(42949672960), HALF)),
            ColliderBuilderTrait::cuboid(ONE, HALF).active_events(events).build(),
        );
    let n = 3 + draw(ref state) % 4;
    let stacked = draw(ref state) % 2 == 0;
    let mut k: u32 = 0;
    while k != n {
        let (column, row) = if stacked {
            (k / 2, k % 2)
        } else {
            (k, 0)
        };
        let mut body = RigidBodyTrait::dynamic(
            at(f(8589934592 * column.into()), HALF + f(4294967296 * row.into())),
        );
        body.activation.time_until_sleep = f(143165577);
        // Pentagons (pointed top) stand on the ground only.
        let builder = if !stacked && (k + seed) % 3 == 0 {
            pentagon()
        } else {
            ColliderBuilderTrait::cuboid(HALF, HALF)
        };
        let _ = world
            .insert(
                body,
                builder.active_events(events).contact_force_event_threshold(f(4294967296)).build(),
            );
        k += 1;
    }
    let speed: i64 = (8 + draw(ref state) % 6).into();
    let height: i64 = (1 + draw(ref state) % 3).into();
    let pebble = RigidBodyBuilderTrait::dynamic()
        .translation(v(f(-21474836480), f(2147483648 * height)))
        .linvel(v(f(4294967296 * speed), f(2147483648)))
        .build();
    let _ = world
        .insert(pebble, ColliderBuilderTrait::ball(f(1288490189)).active_events(events).build());
    world
}

/// A user change on both worlds at step `t` (from `seed`): wake a block up, remove a body,
/// insert a sleeping block, move a block.
fn change(ref a: World, ref b: World, t: u32, seed: u32) {
    let roll = (seed + t * 7) % 23;
    let bodies = a.bodies.iter();
    let (block, _) = *bodies.at(2 + roll % 3);
    if roll == 1 || roll == 12 {
        a.wake_up(block);
        b.wake_up(block);
    } else if roll == 3 || roll == 15 {
        let _ = a.remove_body(block);
        let _ = b.remove_body(block);
    } else if roll == 5 || roll == 18 {
        let body = RigidBodyBuilderTrait::dynamic()
            .translation(v(f(-8589934592), HALF))
            .sleeping(true)
            .build();
        let _ = a.insert(body, pentagon().build());
        let _ = b.insert(body, pentagon().build());
    } else if roll == 9 {
        let mut body = a.body(block).unwrap();
        body.set_position(at(f(-12884901888), HALF));
        assert!(a.set_body(block, body) && b.set_body(block, body));
    }
}

/// Both worlds hold the same bodies, colliders and pairs, raw.
fn assert_same(ref a: World, ref b: World, t: u32) {
    assert!(a.bodies.iter() == b.bodies.iter(), "step {} bodies", t);
    assert!(a.colliders.iter() == b.colliders.iter(), "step {} colliders", t);
    assert!(a.narrow_phase.pairs == b.narrow_phase.pairs, "step {} pairs", t);
}

/// Steps `full` with `step_with_force_events` and `configured` with
/// `step_with_force_events_with::<C>` (`step` / `step_with::<C>` on odd steps when `plain`)
/// `steps` times, with the user changes of `seed` when `changes`; asserts equality after every
/// step; returns how many steps took the active set.
pub(crate) fn run_both<impl C: StepConfig>(
    ref full: World, ref configured: World, seed: u32, steps: u32, changes: bool, plain: bool,
) -> u32 {
    run_both_stages::<C, InProcessStages<C>>(ref full, ref configured, seed, steps, changes, plain)
}

/// [`run_both`] with `step_with_stages::<C, S>` / `step_with_force_events_with_stages::<C, S>`.
pub(crate) fn run_both_stages<impl C: StepConfig, impl S: StageConfig>(
    ref full: World, ref configured: World, seed: u32, steps: u32, changes: bool, plain: bool,
) -> u32 {
    let mut sparse = 0;
    let mut t: u32 = 0;
    while t != steps {
        if changes && t > 4 {
            change(ref full, ref configured, t, seed);
        }
        if usable(ref configured) {
            sparse += 1;
        }
        if plain && t % 2 == 1 {
            let expected = full.step();
            let got = configured.step_with_stages::<C, S>();
            assert!(got == expected, "step {} events", t);
        } else {
            let (expected, expected_forces) = full.step_with_force_events();
            let (got, got_forces) = configured.step_with_force_events_with_stages::<C, S>();
            assert!(got == expected, "step {} events", t);
            assert!(got_forces == expected_forces, "step {} force events", t);
        }
        assert_same(ref full, ref configured, t);
        t += 1;
    }
    sparse
}

fn level_both(seed: u32, steps: u32, changes: bool, plain: bool) -> u32 {
    let mut full = basic_level(seed);
    let mut configured = basic_level(seed);
    run_both::<BasicStepConfig>(ref full, ref configured, seed, steps, changes, plain)
}

/// Random game-shaped levels with user changes: `BasicStepConfig` agrees with the full step.
#[test]
#[fuzzer(runs: 4, seed: 20260927)]
fn fuzz_basic_config_agrees(seed: u16) {
    let _ = level_both(seed.into(), 14, true, false);
}

/// Without user changes, the blocks fall asleep and the pebble's flight takes the active set
/// (sparse step); both outputs (with and without force events) agree.
#[test]
fn test_basic_config_agrees_on_levels() {
    let mut taken = 0;
    for seed in array![0_u32, 1].span() {
        taken += level_both(*seed, 14, false, *seed % 2 == 1);
    }
    assert!(taken != 0, "the active set was never taken");
}

/// The P3 scenes of basic shapes (resting balls, a cuboid stack, the free fall of the pair-free
/// path) and a basic-shape mix with sensors.
#[test]
fn test_configs_agree_on_p3_scenes() {
    for (id, n) in array![('balls', 8_u32), ('stack', 5), ('cubes', 4), ('row', 4)].span() {
        let mut full = p3_scene(*id, *n);
        let mut configured = p3_scene(*id, *n);
        let _ = run_both::<BasicStepConfig>(ref full, ref configured, 0, 4, false, true);
    }
    let mut full = free_fall(4);
    let mut configured = free_fall(4);
    let _ = run_both::<BasicStepConfig>(ref full, ref configured, 0, 3, false, true);
    let mut full = p3_scene('mixed', 0);
    let mut configured = p3_scene('mixed', 0);
    let _ = run_both::<NoCompositeShapes>(ref full, ref configured, 0, 4, false, true);
    let mut full = p3_scene('mixed', 0);
    let mut configured = p3_scene('mixed', 0);
    let _ = run_both::<DefaultStepConfig>(ref full, ref configured, 0, 3, false, true);
}

/// Sensors kept by a configuration that names them: the events of the sensor pair agree.
#[test]
fn test_basic_with_sensors_agrees() {
    let sensor = ColliderBuilderTrait::ball(ONE)
        .sensor(true)
        .active_events(COLLISION_EVENTS)
        .translation(v(ZERO, ONE))
        .build();
    let mut full = basic_level(3);
    let _ = full.insert_collider(sensor, None);
    let mut configured = basic_level(3);
    let _ = configured.insert_collider(sensor, None);
    let _ = run_both::<BasicWithSensors>(ref full, ref configured, 3, 6, false, false);
}

/// CCD: a bullet ball fired through a thin fixed wall, with the CCD pass (`step_with_ccd_with`)
/// and without (`step_with`), agrees with the full step's CCD entry points.
#[test]
fn test_basic_config_agrees_with_ccd() {
    let make = || -> World {
        let mut world: World = Default::default();
        let _ = world
            .insert(
                RigidBodyTrait::fixed(at(f(12884901888), ONE)),
                ColliderBuilderTrait::cuboid(f(214748364), ONE)
                    .active_events(COLLISION_EVENTS | CONTACT_FORCE_EVENTS)
                    .build(),
            );
        let bullet = RigidBodyBuilderTrait::dynamic()
            .translation(v(ZERO, ONE))
            .linvel(v(f(429496729600), ZERO))
            .ccd_enabled(true)
            .build();
        let _ = world
            .insert(
                bullet,
                ColliderBuilderTrait::ball(f(429496729)).active_events(COLLISION_EVENTS).build(),
            );
        world
    };
    let mut full = make();
    let mut configured = make();
    let mut full_ccd: CCDSolver = CCDSolverTrait::new();
    let mut configured_ccd: CCDSolver = CCDSolverTrait::new();
    let mut t: u32 = 0;
    while t != 3 {
        if t == 1 {
            let expected = full.step_with_ccd(ref full_ccd);
            let got = configured.step_with_ccd_with::<BasicStepConfig>(ref configured_ccd);
            assert!(got == expected, "step {} events", t);
        } else {
            let (expected, expected_forces) = full.step_with_ccd_and_force_events(ref full_ccd);
            let (got, got_forces) = configured
                .step_with_ccd_and_force_events_with::<BasicStepConfig>(ref configured_ccd);
            assert!(got == expected, "step {} events", t);
            assert!(got_forces == expected_forces, "step {} force events", t);
        }
        assert_same(ref full, ref configured, t);
        t += 1;
    }
    // The wall stopped the bullet (the CCD pass ran).
    let (_, bullet) = *full.bodies.iter().at(1);
    assert!(bullet.pos.position.translation.x < f(12884901888), "bullet crossed the wall");
}

/// A ball dropped on `builder`'s collider (at the origin, fixed): the pair meets the
/// dispatcher at the first step.
pub(crate) fn ball_over(builder: ColliderBuilder) -> World {
    let mut world: World = Default::default();
    let _ = world.insert_collider(builder.build(), None);
    let _ = world
        .insert(RigidBodyTrait::dynamic(at(ZERO, HALF)), ColliderBuilderTrait::ball(HALF).build());
    world
}

#[test]
#[should_panic(expected: 'Step: joints disabled')]
fn test_basic_config_rejects_joints() {
    let mut world = basic_level(0);
    let a = world.insert_body(RigidBodyTrait::fixed(at(ZERO, f(42949672960))));
    let b = world.insert_body(RigidBodyTrait::dynamic(at(ONE, f(42949672960))));
    let _ = world.insert_impulse_joint(a, b, RevoluteJointBuilderTrait::new().build());
    let _ = world.step_with_force_events_with::<BasicStepConfig>();
}

#[test]
#[should_panic(expected: 'Narrow phase: sensors disabled')]
fn test_basic_config_rejects_sensor_pairs() {
    let mut world = ball_over(ColliderBuilderTrait::cuboid(ONE, HALF).sensor(true));
    let _ = world.step_with::<BasicStepConfig>();
}

#[test]
#[should_panic(expected: 'Dispatch: not a basic shape')]
fn test_basic_config_rejects_other_shapes() {
    let mut world = ball_over(ColliderBuilderTrait::capsule_x(HALF, HALF));
    let _ = world.step_with_force_events_with::<BasicStepConfig>();
}

#[test]
#[should_panic(expected: 'Narrow phase: no composites')]
fn test_no_composites_rejects_composite_pairs() {
    let points = array![v(-ONE, ZERO), v(ZERO, ZERO), v(ONE, ZERO)];
    let mut world = ball_over(ColliderBuilderTrait::polyline(points.span(), None));
    let _ = world.step_with::<NoCompositeShapes>();
}

/// A basic world where no pair meets a disabled feature steps without panicking even though it
/// holds a capsule far away (rejection happens at the step that meets the feature).
#[test]
fn test_unmet_features_are_not_rejected() {
    let mut world = basic_level(1);
    let _ = world
        .insert(
            RigidBodyTrait::fixed(at(f(429496729600), ZERO)),
            ColliderBuilderTrait::capsule_x(HALF, HALF).build(),
        );
    let _ = world.step_with_force_events_with::<BasicStepConfig>();
}

// Probes: one step of the settled `cuboid_stack(3)` scene, full vs basic configuration.

fn settled_stack() -> World {
    let mut world = p3_scene('stack', opaque(3));
    let _ = world.step_with_force_events();
    world
}

#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}

#[test]
fn gas_setup_stack3() {
    let world = settled_stack();
    let _ = opaque(world.gravity);
}

#[test]
fn gas_step_default_stack3() {
    let mut world = settled_stack();
    let _ = world.step_with_force_events();
}

#[test]
fn gas_step_basic_stack3() {
    let mut world = settled_stack();
    let _ = world.step_with_force_events_with::<BasicStepConfig>();
}
