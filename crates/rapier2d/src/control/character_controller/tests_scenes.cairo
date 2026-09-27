//! Scene tests and `gas_*` probes of `move_shape` (the scenes of the `character_moves` golden
//! family): flat ground, a 30 degree ramp, a step, a wall. Subtract `gas_setup_<scene>` from
//! `gas_move_<scene>` to get the move alone.

use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::vec2::Vec2;
use rapier_core::Handle;
use rapier_dynamics2d::collider::builder::ColliderBuilderTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBodyBuilderTrait, RigidBodyTrait};
use rapier_geometry2d::shape::{Ball, CuboidTrait, Shape};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::queries::QueryFilterTrait;
use crate::queries::pipeline::{QueryPipeline, QueryPipelineTrait};
use crate::world::{World, WorldTrait};
use super::{
    CharacterAutostep, CharacterCollision, CharacterLength, EffectiveCharacterMovement,
    KinematicCharacterController, KinematicCharacterControllerTrait,
};

fn ratio(n: i64, d: i64) -> Fixed {
    FixedTrait::from_ratio(n, d)
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
}

fn cuboid(hx: Fixed, hy: Fixed) -> Shape {
    Shape::Cuboid(CuboidTrait::new(v(hx, hy)))
}

fn ball() -> Shape {
    Shape::Ball(Ball { radius: HALF })
}

fn fixed_collider(ref world: World, shape: Shape, pose: Pose2) -> Handle {
    world.insert_collider(ColliderBuilderTrait::new(shape).position(pose).build(), None)
}

/// `0` flat ground, `1` + 30 degree ramp, `2` + step (0.2 high from x = 1), `3` + wall (x = 2).
fn scene(kind: u32) -> World {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    let _ = fixed_collider(ref world, cuboid(ratio(10, 1), HALF), at(ZERO, -HALF));
    if kind == 1 {
        let ramp = Pose2Trait::new(
            v(ratio(5, 2), ratio(3, 5)), Rot2 { re: Fixed { raw: 3719550787 }, im: HALF },
        );
        let _ = fixed_collider(ref world, cuboid(ratio(2, 1), ratio(1, 10)), ramp);
    } else if kind == 2 {
        let _ = fixed_collider(ref world, cuboid(ONE, ratio(1, 10)), at(ratio(2, 1), ratio(1, 10)));
    } else if kind == 3 {
        let _ = fixed_collider(
            ref world, cuboid(ratio(1, 4), ratio(2, 1)), at(ratio(9, 4), ratio(2, 1)),
        );
    }
    world
}

fn stepper() -> KinematicCharacterController {
    KinematicCharacterController {
        autostep: Some(
            CharacterAutostep {
                max_height: CharacterLength::Absolute(ratio(3, 10)),
                min_width: CharacterLength::Absolute(ratio(1, 5)),
                include_dynamic_bodies: true,
            },
        ),
        ..Default::default(),
    }
}

/// `(scene, controller, shape, pose, desired)` of the probe of `kind`.
fn setup(kind: u32) -> (World, KinematicCharacterController, Shape, Pose2, Vec2) {
    let world = scene(kind);
    if kind == 0 {
        (
            world,
            Default::default(),
            ball(),
            at(ZERO, ratio(101, 200)),
            v(ratio(3, 10), -ratio(1, 10)),
        )
    } else if kind == 1 {
        (
            world,
            Default::default(),
            ball(),
            at(ZERO, ratio(101, 200)),
            v(ratio(3, 2), -ratio(1, 20)),
        )
    } else if kind == 2 {
        (
            world,
            stepper(),
            cuboid(ratio(3, 10), HALF),
            at(ratio(2, 5), ratio(51, 100)),
            v(ratio(3, 5), -ratio(1, 50)),
        )
    } else {
        (
            world,
            Default::default(),
            ball(),
            at(ratio(13, 10), ratio(101, 200)),
            v(ratio(3, 5), -ratio(1, 20)),
        )
    }
}

fn run(
    kind: u32, queries: QueryPipeline,
) -> (EffectiveCharacterMovement, Array<CharacterCollision>) {
    let (mut world, c, shape, pos, desired) = setup(kind);
    let mut events = array![];
    let m = c.move_shape(ratio(1, 60), ref world, queries, shape, pos, desired, ref events);
    (m, events)
}

fn close(a: Fixed, b: Fixed, tol: i64) -> bool {
    (a - b).abs() <= Fixed { raw: tol }
}

#[test]
fn test_flat_walk() {
    let (m, events) = run(0, QueryPipelineTrait::new());
    assert!(m.grounded && close(m.translation.x, ratio(3, 10), 64), "{:?}", m);
    // Starting within the offset of the ground: one hit at time zero, the fall removed.
    assert_eq!(events.len(), 1);
    assert_eq!(*events.at(0).handle.index, 0);
    assert!(close(m.translation.y, Fixed { raw: 429497 }, 64), "{:?}", m);
}

#[test]
fn test_slope_climb_and_step() {
    let (m, _) = run(1, QueryPipelineTrait::new());
    // Climbs the 30 degree ramp: up by about tan(30) of the run on it.
    assert!(m.grounded && m.translation.y > ratio(1, 10), "{:?}", m);
    let (m, events) = run(2, QueryPipelineTrait::new());
    assert!(close(m.translation.y, ratio(1, 5), 1048576), "{:?}", m);
    assert_eq!(events.len(), 3);
    // Without autostep the step blocks.
    let (mut world, _, shape, pos, desired) = setup(2);
    let c: KinematicCharacterController = Default::default();
    let mut events = array![];
    let m = c
        .move_shape(
            ratio(1, 60), ref world, QueryPipelineTrait::new(), shape, pos, desired, ref events,
        );
    assert!(m.translation.x < ratio(3, 10) && m.translation.y < ratio(1, 100), "{:?}", m);
}

#[test]
fn test_wall_and_filter() {
    let (m, events) = run(3, QueryPipelineTrait::new());
    assert!(close(m.translation.x, ratio(1899, 10000), 64), "{:?}", m);
    assert_eq!(*events.at(1).handle.index, 1);
    // Excluding the wall: the character walks through its place.
    let pass = QueryPipelineTrait::new()
        .with_filter(QueryFilterTrait::new().exclude_collider(Handle { index: 1, generation: 0 }));
    let (m, events) = run(3, pass);
    assert!(close(m.translation.x, ratio(3, 5), 64) && events.len() == 1, "{:?}", m);
    // Without sliding, the first hit stops the move.
    let (mut world, _, shape, pos, desired) = setup(3);
    let c = KinematicCharacterController { slide: false, ..Default::default() };
    let mut events = array![];
    let m = c
        .move_shape(
            ratio(1, 60), ref world, QueryPipelineTrait::new(), shape, pos, desired, ref events,
        );
    assert!(close(m.translation.x, ZERO, 64) && events.len() == 1, "{:?}", m);
}

/// A zero move out of the ground: pushed up to the offset gap; a sensor is not pushed out of.
#[test]
fn test_depenetration() {
    let mut world = scene(0);
    let c: KinematicCharacterController = Default::default();
    let mut events = array![];
    let m = c
        .move_shape(
            ratio(1, 60),
            ref world,
            QueryPipelineTrait::new(),
            ball(),
            at(ZERO, ratio(2, 5)),
            v(ZERO, ZERO),
            ref events,
        );
    assert!(m.grounded && close(m.translation.y, ratio(11, 100), 4096), "{:?}", m);
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    let _ = world
        .insert_collider(ColliderBuilderTrait::new(cuboid(ONE, ONE)).sensor(true).build(), None);
    let m = c
        .move_shape(
            ratio(1, 60),
            ref world,
            QueryPipelineTrait::new(),
            ball(),
            at(ZERO, ZERO),
            v(ZERO, ZERO),
            ref events,
        );
    assert_eq!(m.translation, v(ZERO, ZERO));
}

/// Pushing a dynamic box transfers an impulse along the hit normal; a fixed obstacle gets none.
#[test]
fn test_collision_impulses() {
    let mut world = scene(0);
    let rb = RigidBodyBuilderTrait::dynamic().position(at(ratio(6, 5), ratio(3, 10))).build();
    let (body, _) = world
        .insert(rb, ColliderBuilderTrait::new(cuboid(ratio(3, 10), ratio(3, 10))).build());
    let c: KinematicCharacterController = Default::default();
    let shape = cuboid(ratio(3, 10), HALF);
    let mut events = array![];
    let queries = QueryPipelineTrait::new();
    let _ = c
        .move_shape(
            ratio(1, 60),
            ref world,
            queries,
            shape,
            at(ratio(3, 10), ratio(51, 100)),
            v(HALF, -ratio(1, 100)),
            ref events,
        );
    assert_eq!(events.len(), 2);
    c
        .solve_character_collision_impulses(
            ratio(1, 60), ref world, queries, shape, ratio(2, 1), events.span(),
        );
    let pushed = world.body(body).unwrap();
    assert!(pushed.linvel().x > ratio(10, 1), "{:?}", pushed.linvel());
}

// ---------------------------------------------------------------------------------------------
// Gas probes: `gas_setup_<scene>` builds the scene and the inputs, `gas_move_<scene>` also moves.

#[test]
fn gas_baseline() {
    let _ = opaque(v(ZERO, ZERO));
}

fn probe(kind: u32, moving: bool) {
    let (mut world, c, shape, pos, desired) = setup(opaque(kind));
    let (c, shape, pos, desired) = (opaque(c), opaque(shape), opaque(pos), opaque(desired));
    if moving {
        let mut events = array![];
        let m = c
            .move_shape(
                opaque(ratio(1, 60)),
                ref world,
                QueryPipelineTrait::new(),
                shape,
                pos,
                desired,
                ref events,
            );
        let _ = opaque(m);
        let _ = opaque(events);
    }
}

#[test]
fn gas_setup_flat() {
    probe(0, false);
}

#[test]
fn gas_move_flat() {
    probe(0, true);
}

#[test]
fn gas_setup_slope() {
    probe(1, false);
}

#[test]
fn gas_move_slope() {
    probe(1, true);
}

#[test]
fn gas_setup_step() {
    probe(2, false);
}

#[test]
fn gas_move_step() {
    probe(2, true);
}

#[test]
fn gas_setup_wall() {
    probe(3, false);
}

#[test]
fn gas_move_wall() {
    probe(3, true);
}
