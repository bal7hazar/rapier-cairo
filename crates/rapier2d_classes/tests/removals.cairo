//! The split layouts against the in-process step on a scene pile10 does not cover (CX1, from the
//! audit of CS5): despawns in the middle of the run (arena holes, then spawns that reuse the freed
//! slots with new generations), a body with several colliders, a position-based and a
//! velocity-based kinematic body, a collider without parent and a sleeping island woken by an
//! impact. After every tick the whole `WorldState` and the tick's collision events must be
//! identical (`digest`). `SlimSplitStages` (CS6) has no position-based kinematic body: its run
//! drives the paddle by its velocity instead.
//!
//! The scene, in insertion order (slots): the ground (fixed body, half-space), a static box with
//! no parent, a three-box stack inserted asleep on the ground, a dumbbell (one dynamic body, two
//! balls and a bar), a kinematic paddle driven by its next position, a kinematic slider driven by
//! its velocity, and two falling boxes. The events of [`change`] remove bodies and colliders and
//! spawn new ones, among which the ball that wakes the stack.

use rapier2d::prelude::{
    BasicStepConfig, ColliderBuilderTrait, CollisionEvent, Fixed, Handle, IntegrationParameters,
    Pose2, RigidBodyBuilderTrait, RigidBodyTrait, Rot2, Vec2, World, WorldTrait,
};
use rapier2d_classes::{
    ContactSolveStepConfig, SlimSplitStages, SplitBatchedStages, SplitHybridStages, SplitStages,
};
use rapier_core::rigid_body::RigidBodyType;
use crate::hashes::{StoredHashes, install};
use crate::pile10::{InProcess, Layout, Staged};

const ONE: i64 = 0x100000000;
const HALF: i64 = 0x80000000;
/// `-9.81` in raw Q32.32.
const GRAVITY_Y: i64 = -42133629174;
/// `floor(2^32 / 60)`.
const DT: i64 = 71582788;
/// Ticks of the run.
const TICKS: u32 = 110;

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

fn v(x: i64, y: i64) -> Vec2 {
    Vec2 { x: f(x), y: f(y) }
}

fn at(x: i64, y: i64) -> Pose2 {
    Pose2 { translation: v(x, y), rotation: Rot2 { re: f(ONE), im: f(0) } }
}

/// The scene's handles that [`change`] and [`drive`] use.
#[derive(Destruct)]
struct Scene {
    world: World,
    /// The static box without parent (a collider handle).
    ledge: Handle,
    /// The boxes of the stack, bottom first.
    stack: Array<Handle>,
    /// The dumbbell body and its middle collider.
    dumbbell: Handle,
    bar: Handle,
    /// The dumbbell's balls (colliders).
    balls: Array<Handle>,
    /// The kinematic bodies, and the paddle's collider.
    paddle: Handle,
    paddle_collider: Handle,
    slider: Handle,
    /// The falling boxes, and the first one's collider.
    faller1: Handle,
    faller1_collider: Handle,
    faller2: Handle,
}

fn dynamic_box(ref world: World, x: i64, y: i64, sleeping: bool) -> Handle {
    let (handle, _) = dynamic_box_collider(ref world, x, y, sleeping);
    handle
}

fn dynamic_box_collider(ref world: World, x: i64, y: i64, sleeping: bool) -> (Handle, Handle) {
    let body = RigidBodyBuilderTrait::dynamic().position(at(x, y)).sleeping(sleeping).build();
    let collider = ColliderBuilderTrait::cuboid(f(HALF), f(HALF)).friction(f(HALF)).build();
    world.insert(body, collider)
}

fn build(kinematic_position: bool) -> Scene {
    let mut params: IntegrationParameters = Default::default();
    params.dt = f(DT);
    params.num_solver_iterations = 4;
    let mut world = WorldTrait::new(v(0, GRAVITY_Y), params);
    let _ = world
        .insert(
            RigidBodyBuilderTrait::fixed().build(),
            ColliderBuilderTrait::halfspace(v(0, ONE)).friction(f(HALF)).build(),
        );
    // A static box without parent, where the first faller lands.
    let ledge = world
        .insert_collider(
            ColliderBuilderTrait::cuboid(f(ONE), f(HALF)).translation(v(-6 * ONE, 2 * ONE)).build(),
            None,
        );
    // A stack of three boxes, asleep on the ground.
    let stack = array![
        dynamic_box(ref world, 12 * ONE, HALF, true),
        dynamic_box(ref world, 12 * ONE, 3 * HALF, true),
        dynamic_box(ref world, 12 * ONE, 5 * HALF, true),
    ];
    // A dumbbell: two balls and a bar on one body.
    let dumbbell = world
        .insert_body(
            RigidBodyBuilderTrait::dynamic().position(at(0, 3 * ONE)).angvel(f(HALF)).build(),
        );
    let balls = array![
        world
            .insert_collider(
                ColliderBuilderTrait::ball(f(HALF)).translation(v(-ONE, 0)).build(), Some(dumbbell),
            ),
        world
            .insert_collider(
                ColliderBuilderTrait::ball(f(HALF)).translation(v(ONE, 0)).build(), Some(dumbbell),
            ),
    ];
    let bar = world
        .insert_collider(ColliderBuilderTrait::cuboid(f(ONE), f(HALF / 4)).build(), Some(dumbbell));
    // A paddle driven by its next position (`drive`), or by the same speed, sweeping towards
    // the dumbbell.
    let paddle_body = if kinematic_position {
        RigidBodyBuilderTrait::kinematic_position_based().position(at(-3 * ONE, HALF)).build()
    } else {
        RigidBodyBuilderTrait::kinematic_velocity_based()
            .position(at(-3 * ONE, HALF))
            .linvel(v(60 * ONE / 32, 0))
            .build()
    };
    let (paddle, paddle_collider) = world
        .insert(paddle_body, ColliderBuilderTrait::cuboid(f(HALF), f(HALF)).build());
    // A slider driven by its velocity, moving towards the stack from the far side.
    let (slider, _) = world
        .insert(
            RigidBodyBuilderTrait::kinematic_velocity_based()
                .position(at(14 * ONE + HALF, HALF))
                .linvel(v(-ONE, 0))
                .build(),
            ColliderBuilderTrait::cuboid(f(HALF), f(HALF)).build(),
        );
    let (faller1, faller1_collider) = dynamic_box_collider(
        ref world, -6 * ONE, 3 * ONE + HALF / 2, false,
    );
    let faller2 = dynamic_box(ref world, 4 * ONE, 2 * ONE, false);
    Scene {
        world,
        ledge,
        stack,
        dumbbell,
        bar,
        balls,
        paddle,
        paddle_collider,
        slider,
        faller1,
        faller1_collider,
        faller2,
    }
}

/// [`build`], then the settle of a game level (as pile10): one `dt = 0` step with `L` (the
/// inserted colliders meet), then the stack put back to sleep.
fn settled<impl L: Layout>(kinematic_position: bool) -> Scene {
    let mut scene = build(kinematic_position);
    scene.world.integration_parameters.dt = f(0);
    let _ = scene.world.step_with_stages::<L::Step, L::Stages>();
    scene.world.integration_parameters.dt = f(DT);
    for handle in scene.stack.span() {
        let mut body = scene.world.body(*handle).unwrap();
        if !body.is_sleeping() {
            body.sleep();
            let _ = scene.world.set_body(*handle, body);
        }
    }
    scene
}

/// The paddle's next position at tick `t` (position-based): right by `1 / 32` per tick.
fn drive(ref scene: Scene, t: u32) {
    if let Some(mut body) = scene.world.body(scene.paddle) {
        if body.body_type != RigidBodyType::KinematicPositionBased {
            return;
        }
        let x = -3 * ONE + (t.into() * ONE) / 32;
        body.set_next_kinematic_position(at(x, HALF));
        let _ = scene.world.set_body(scene.paddle, body);
    }
}

/// The despawns and spawns of the run: a falling box removed (a hole), a ball spawned in its
/// slot (new generation) and launched at the sleeping stack, a collider of the dumbbell removed,
/// then the dumbbell itself, the ledge (no parent) and the second faller removed, and two boxes
/// spawned into the freed slots.
fn change(ref scene: Scene, t: u32) {
    if t == 30 {
        let _ = scene.world.remove_body(scene.faller1);
    } else if t == 34 {
        let body = RigidBodyBuilderTrait::dynamic()
            .position(at(6 * ONE, 2 * ONE))
            .linvel(v(8 * ONE, 0))
            .build();
        let collider = ColliderBuilderTrait::ball(f(HALF))
            .density(f(4 * ONE))
            .restitution(f(HALF / 2))
            .build();
        let (ball, _) = scene.world.insert(body, collider);
        assert!(ball.index == scene.faller1.index, "the ball reuses the freed slot");
        assert!(ball.generation != scene.faller1.generation, "with a new generation");
    } else if t == 40 {
        let _ = scene.world.remove_collider(scene.bar);
    } else if t == 55 {
        let _ = scene.world.remove_body(scene.dumbbell);
        let _ = scene.world.remove_collider(scene.ledge);
        let _ = scene.world.remove_body(scene.faller2);
    } else if t == 60 {
        let _ = dynamic_box(ref scene.world, 2 * ONE, 4 * ONE, false);
        let _ = dynamic_box(ref scene.world, 2 * ONE + HALF / 2, 6 * ONE, false);
    } else if t == 100 {
        if let Some(mut body) = scene.world.body(scene.slider) {
            body.set_linvel(v(0, 0));
            let _ = scene.world.set_body(scene.slider, body);
        }
    }
}

/// Whether the colliders `a` and `b` have a touching contact pair.
fn touching(ref scene: Scene, a: Handle, b: Handle) -> bool {
    let (a, b) = if a.index < b.index {
        (a, b)
    } else {
        (b, a)
    };
    match scene.world.contact_pair(a, b) {
        Some(pair) => pair.manifold.data.num_solver_contacts != 0,
        None => false,
    }
}

/// Poseidon digest of the whole world (`to_state`) and of the tick's events.
fn digest(ref world: World, events: Span<CollisionEvent>) -> felt252 {
    let mut felts: Array<felt252> = array![];
    world.to_state().serialize(ref felts);
    events.serialize(ref felts);
    core::poseidon::poseidon_hash_span(felts.span())
}

/// The tick-by-tick digests of the run with `L`, the initial world first; checks along the way
/// that the stack sleeps until the ball reaches it and is woken by it.
fn trace<impl L: Layout>(kinematic_position: bool) -> Array<felt252> {
    let mut scene = settled::<L>(kinematic_position);
    let mut out = array![digest(ref scene.world, array![].span())];
    let mut t = 0;
    let mut woken_at = None;
    let (mut ledge_touched, mut paddle_pushed) = (false, false);
    while t != TICKS {
        change(ref scene, t);
        drive(ref scene, t);
        let events = scene.world.step_with_stages::<L::Step, L::Stages>();
        out.append(digest(ref scene.world, events.span()));
        if touching(ref scene, scene.ledge, scene.faller1_collider) {
            ledge_touched = true;
        }
        for ball in scene.balls.span() {
            if touching(ref scene, scene.paddle_collider, *ball) {
                paddle_pushed = true;
            }
        }
        if woken_at.is_none() && !scene.world.is_sleeping(*scene.stack[0]).unwrap() {
            woken_at = Some(t);
        }
        t += 1;
    }
    assert!(ledge_touched, "a box lands on the collider without parent");
    assert!(paddle_pushed, "the kinematic paddle pushes the dumbbell");
    let woken_at = woken_at.expect('the impact wakes the stack');
    assert!(woken_at > 34, "the stack sleeps until the impact (woken at {woken_at})");
    out
}

fn assert_same(layout: ByteArray, got: Span<felt252>, expected: Span<felt252>) {
    assert_eq!(got.len(), expected.len());
    let mut tick = 0;
    for (a, b) in got.into_iter().zip(expected) {
        assert!(a == b, "{layout}: tick {tick} differs");
        tick += 1;
    }
}

#[test]
fn test_removals_split_bit_identical() {
    let expected = trace::<InProcess<BasicStepConfig>>(true);
    install(true);
    assert_same(
        "split",
        trace::<Staged<BasicStepConfig, SplitStages<StoredHashes>>>(true).span(),
        expected.span(),
    );
    assert_same(
        "hybrid",
        trace::<Staged<BasicStepConfig, SplitHybridStages<StoredHashes>>>(true).span(),
        expected.span(),
    );
}

#[test]
fn test_removals_variants_bit_identical() {
    let expected = trace::<InProcess<BasicStepConfig>>(true);
    install(true);
    assert_same(
        "batched",
        trace::<Staged<BasicStepConfig, SplitBatchedStages<StoredHashes>>>(true).span(),
        expected.span(),
    );
    assert_same(
        "CS4 layout",
        trace::<InProcess<ContactSolveStepConfig<StoredHashes>>>(true).span(),
        expected.span(),
    );
}

#[test]
fn test_removals_slim_bit_identical() {
    let expected = trace::<InProcess<BasicStepConfig>>(false);
    install(true);
    assert_same(
        "slim",
        trace::<Staged<BasicStepConfig, SlimSplitStages<StoredHashes>>>(false).span(),
        expected.span(),
    );
}
