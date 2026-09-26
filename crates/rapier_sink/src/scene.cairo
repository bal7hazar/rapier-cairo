//! The worlds the fixture contracts step. Positions are offset by calldata (`dx`, `dy`, raw
//! Q32.32) so that nothing is constant-folded.
//!
//! * [`game_world`]: the game's configuration (`Slingfall`): a half-space ground, a cuboid, a
//!   convex polygon and a ball, no joint, no sensor; every collider reports collision and
//!   contact-force events.
//! * [`full_world`]: every shape type (ball, cuboid, capsule, half-space, convex polygon,
//!   segment), a sensor and every joint builder (fixed, revolute, prismatic, rope, spring).

use rapier2d::prelude::*;

/// `1` in raw Q32.32.
const ONE: i64 = 4294967296;
/// `1/2` in raw Q32.32.
const HALF: i64 = 2147483648;
/// Earth gravity, `-9.81` in raw Q32.32.
const GRAVITY_Y: i64 = -42133629174;
/// `1/60` in raw Q32.32.
const DT: i64 = 71582788;

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

fn v(x: i64, y: i64) -> Vec2 {
    Vec2 { x: f(x), y: f(y) }
}

fn at(x: i64, y: i64) -> Pose2 {
    Pose2 { translation: v(x, y), rotation: Rot2 { re: f(ONE), im: f(0) } }
}

/// Collision and contact-force events, zero force threshold.
fn with_events(builder: ColliderBuilder) -> Collider {
    builder
        .active_events(COLLISION_EVENTS | CONTACT_FORCE_EVENTS)
        .contact_force_event_threshold(f(0))
        .build()
}

fn empty_world() -> World {
    let params = IntegrationParameters { dt: f(DT), ..Default::default() };
    WorldTrait::new(v(0, GRAVITY_Y), params)
}

/// A convex pentagon about one unit wide, relative to its body.
fn pentagon() -> ColliderBuilder {
    let points = array![
        v(-HALF, -HALF), v(HALF, -HALF), v(HALF + HALF / 2, 0), v(0, HALF), v(-HALF - HALF / 2, 0),
    ];
    ColliderBuilderTrait::convex_polygon(points.span()).expect('sink: pentagon')
}

/// The game's configuration: a half-space `y ≥ dy`, a cuboid, a pentagon and a ball stacked
/// above it, offset by `dx`.
pub fn game_world(dx: i64, dy: i64) -> World {
    let mut world = empty_world();
    let _ = world
        .insert_collider(
            with_events(ColliderBuilderTrait::halfspace(v(0, ONE)).position(at(dx, dy))), None,
        );
    let _ = world
        .insert(
            RigidBodyTrait::dynamic(at(dx, dy + HALF)),
            with_events(ColliderBuilderTrait::cuboid(f(HALF), f(HALF))),
        );
    let _ = world.insert(RigidBodyTrait::dynamic(at(dx, dy + 3 * HALF)), with_events(pentagon()));
    let _ = world
        .insert(
            RigidBodyTrait::dynamic(at(dx + HALF / 4, dy + 5 * HALF)),
            with_events(ColliderBuilderTrait::ball(f(HALF))),
        );
    world
}

/// Every shape type, a sensor and every joint builder, offset by `dx`, `dy`.
pub fn full_world(dx: i64, dy: i64) -> World {
    let mut world = game_world(dx, dy);
    let (capsule, _) = world
        .insert(
            RigidBodyTrait::dynamic(at(dx + 3 * ONE, dy + ONE)),
            with_events(ColliderBuilderTrait::capsule_x(f(HALF), f(HALF / 2))),
        );
    let (segment, _) = world
        .insert(
            RigidBodyTrait::dynamic(at(dx + 3 * ONE, dy + 2 * ONE)),
            with_events(ColliderBuilderTrait::segment(v(-HALF, 0), v(HALF, 0))),
        );
    let sensor = ColliderBuilderTrait::ball(f(ONE))
        .sensor(true)
        .position(at(dx + 3 * ONE, dy + ONE))
        .active_events(COLLISION_EVENTS)
        .build();
    let _ = world.insert_collider(sensor, None);
    let pivot = world.insert_body(RigidBodyTrait::fixed(at(dx - 3 * ONE, dy + 3 * ONE)));
    let (bob1, _) = world
        .insert(
            RigidBodyTrait::dynamic(at(dx - 2 * ONE, dy + 3 * ONE)),
            with_events(ColliderBuilderTrait::ball(f(HALF / 2))),
        );
    let (bob2, _) = world
        .insert(
            RigidBodyTrait::dynamic(at(dx - ONE, dy + 3 * ONE)),
            with_events(ColliderBuilderTrait::cuboid(f(HALF / 2), f(HALF / 2))),
        );
    let revolute = RevoluteJointBuilderTrait::new().local_anchor2(v(-ONE, 0)).build();
    let _ = world.insert_impulse_joint(pivot, bob1, revolute);
    let prismatic = PrismaticJointBuilderTrait::new(v(ONE, 0))
        .local_anchor2(v(-ONE, 0))
        .limits([f(-HALF), f(HALF)])
        .build();
    let _ = world.insert_impulse_joint(bob1, bob2, prismatic);
    let fixed = FixedJointBuilderTrait::new().local_anchor2(v(0, -ONE)).build();
    let _ = world.insert_impulse_joint(capsule, segment, fixed);
    let rope = RopeJointBuilderTrait::new(f(2 * ONE)).build();
    let _ = world.insert_impulse_joint(pivot, bob2, rope);
    let spring = SpringJointBuilderTrait::new(f(ONE), f(10 * ONE), f(HALF)).build();
    let _ = world.insert_impulse_joint(bob2, capsule, spring);
    world
}

/// Runs `steps` calls of `World::step_with_force_events`; returns the number of collision and
/// contact-force events.
pub fn run_with_forces(ref world: World, steps: u32) -> (u32, u32) {
    let (mut collisions, mut forces) = (0, 0);
    let mut i = 0;
    while i != steps {
        let (c, fe) = world.step_with_force_events();
        collisions += c.len();
        forces += fe.len();
        i += 1;
    }
    (collisions, forces)
}

/// Runs `steps` calls of `World::step`; returns the number of collision events.
pub fn run(ref world: World, steps: u32) -> u32 {
    let mut events = 0;
    let mut i = 0;
    while i != steps {
        events += world.step().len();
        i += 1;
    }
    events
}
