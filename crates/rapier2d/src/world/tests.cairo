//! Tests and gas probes of `World` (`super`).

use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::events::{COLLISION_EVENTS, REMOVED, SENSOR};
use rapier_dynamics2d::collider::{ColliderBuilderTrait, ColliderTrait};
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::joint::RevoluteJointBuilderTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBodyBuilderTrait, RigidBodySetTrait, RigidBodyTrait};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use super::{DEFAULT_GRAVITY_Y, PhysicsWorld, World, WorldTrait};

fn at(x: fixed::Fixed, y: fixed::Fixed) -> Pose2 {
    Pose2 { translation: Vec2 { x, y }, rotation: Rot2 { re: ONE, im: ZERO } }
}

/// Two dynamic balls joined by a revolute joint, and a standalone ground collider.
fn pair_world() -> (World, Handle, Handle, Handle) {
    let mut world = WorldTrait::new(Vec2 { x: ZERO, y: -ONE }, Default::default());
    let (a, _) = world
        .insert(RigidBodyTrait::dynamic(at(ZERO, ONE)), ColliderBuilderTrait::ball(HALF).build());
    let (b, _) = world
        .insert(RigidBodyTrait::dynamic(at(ONE, ONE)), ColliderBuilderTrait::ball(HALF).build());
    let joint = world.insert_impulse_joint(a, b, RevoluteJointBuilderTrait::new().build());
    let _ = world
        .insert_collider(ColliderBuilderTrait::halfspace(Vec2 { x: ZERO, y: ONE }).build(), None);
    (world, a, b, joint)
}

#[test]
fn test_insert_links_and_poses() {
    let (mut world, a, _, joint) = pair_world();
    assert_eq!(world.bodies.len(), 2);
    let body = world.body(a).unwrap();
    assert_eq!(body.colliders.len(), 1);
    let collider = world.collider(*body.colliders.at(0)).unwrap();
    assert_eq!(collider.parent(), Some(a));
    assert_eq!(collider.position(), at(ZERO, ONE));
    let standalone = world.collider(rapier_core::Handle { index: 2, generation: 0 }).unwrap();
    assert_eq!(standalone.parent(), None);
    assert_eq!(world.impulse_joint(joint).unwrap().body1, a);
}

/// `remove_body` removes the body's colliders and every joint attached to it; the other
/// body and the standalone collider stay.
#[test]
fn test_remove_body_detaches_colliders_and_joints() {
    let (mut world, a, b, joint) = pair_world();
    let co = *world.body(a).unwrap().colliders.at(0);
    assert!(world.remove_body(a).is_some());
    assert!(world.remove_body(a).is_none());
    assert!(world.collider(co).is_none());
    assert!(world.impulse_joint(joint).is_none());
    assert!(world.body(b).is_some());
    assert_eq!(world.colliders.len(), 2);
    let _ = world.step();
}

#[test]
fn test_remove_collider_and_joint() {
    let (mut world, a, _, joint) = pair_world();
    let co = *world.body(a).unwrap().colliders.at(0);
    assert!(world.remove_collider(co).is_some());
    assert!(world.remove_collider(co).is_none());
    assert_eq!(world.body(a).unwrap().colliders.len(), 0);
    assert_eq!(world.remove_impulse_joint(joint), Some(RevoluteJointBuilderTrait::new().build()));
    assert!(world.remove_impulse_joint(joint).is_none());
}

#[test]
fn test_setters_write_back() {
    let (mut world, a, _, _) = pair_world();
    let mut body = world.body(a).unwrap();
    body.set_linvel(Vec2 { x: ONE, y: ZERO });
    assert!(world.set_body(a, body));
    assert_eq!(world.body(a).unwrap().linvel(), Vec2 { x: ONE, y: ZERO });
    let co = *body.colliders.at(0);
    let mut collider = world.collider(co).unwrap();
    collider.set_friction(ONE);
    assert!(world.set_collider(co, collider));
    assert_eq!(world.collider(co).unwrap().friction(), ONE);
    let stale = Handle { index: a.index, generation: a.generation + 1 };
    assert!(!world.set_body(stale, body));
    assert!(world.contact_pair(co, co).is_none());
}

// Gas probes: `gas_<op>` − `gas_setup` is one call on the `pair_world` state.

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_new() {
    let _ = WorldTrait::new(opaque(Vec2 { x: ZERO, y: -ONE }), Default::default());
}

#[test]
fn gas_setup() {
    let _ = pair_world();
}

#[test]
fn gas_insert_body() {
    let (mut world, _, _, _) = pair_world();
    let _ = world.insert_body(RigidBodyTrait::dynamic(opaque(at(ZERO, ZERO))));
}

#[test]
fn gas_insert_collider_attached() {
    let (mut world, a, _, _) = pair_world();
    let _ = world.insert_collider(ColliderBuilderTrait::ball(opaque(HALF)).build(), Some(a));
}

/// `pair_world` plus a standalone sensor ball around the first ball, after one step.
fn sensor_world() -> (World, Handle, Handle) {
    let (mut world, _, _, _) = pair_world();
    let sensor = world
        .insert_collider(
            ColliderBuilderTrait::ball(ONE).sensor(true).position(at(ZERO, ONE)).build(), None,
        );
    let _ = world.step();
    (world, sensor, Handle { index: 0, generation: 0 })
}

#[test]
fn test_intersection_queries_skip_removed_colliders() {
    let (mut world, sensor, ball) = sensor_world();
    assert_eq!(world.intersection_pair(sensor, ball), Some(true));
    assert_eq!(world.intersection_pairs().len(), 2);
    assert_eq!(world.intersection_pairs_with(ball), array![(ball, sensor, true)]);
    let _ = world.remove_collider(ball);
    assert_eq!(world.intersection_pairs_with(sensor).len(), 1);
}

#[test]
fn gas_intersection_pair() {
    let (world, sensor, ball) = sensor_world();
    let _ = opaque(world.intersection_pair(opaque(sensor), ball));
}

#[test]
fn gas_intersection_pairs_with() {
    let (mut world, sensor, _) = sensor_world();
    let _ = opaque(world.intersection_pairs_with(opaque(sensor)));
}

#[test]
fn gas_insert_collider_standalone() {
    let (mut world, _, _, _) = pair_world();
    let _ = world.insert_collider(ColliderBuilderTrait::ball(opaque(HALF)).build(), None);
}

#[test]
fn gas_insert() {
    let (mut world, _, _, _) = pair_world();
    let _ = world
        .insert(
            RigidBodyTrait::dynamic(opaque(at(ZERO, ZERO))),
            ColliderBuilderTrait::ball(HALF).build(),
        );
}

#[test]
fn gas_insert_impulse_joint() {
    let (mut world, a, b, _) = pair_world();
    let _ = world.insert_impulse_joint(opaque(a), b, RevoluteJointBuilderTrait::new().build());
}

#[test]
fn gas_remove_body() {
    let (mut world, a, _, _) = pair_world();
    assert!(world.remove_body(opaque(a)).is_some());
}

#[test]
fn gas_remove_collider() {
    let (mut world, _, _, _) = pair_world();
    assert!(world.remove_collider(opaque(Handle { index: 2, generation: 0 })).is_some());
}

#[test]
fn gas_remove_impulse_joint() {
    let (mut world, _, _, joint) = pair_world();
    assert!(world.remove_impulse_joint(opaque(joint)).is_some());
}

#[test]
fn gas_body() {
    let (mut world, a, _, _) = pair_world();
    assert!(world.body(opaque(a)).is_some());
}

#[test]
fn gas_set_body() {
    let (mut world, a, _, _) = pair_world();
    let body = world.body(a).unwrap();
    assert!(world.set_body(opaque(a), body));
}

#[test]
fn gas_collider() {
    let (mut world, _, _, _) = pair_world();
    assert!(world.collider(opaque(Handle { index: 2, generation: 0 })).is_some());
}

#[test]
fn gas_set_collider() {
    let (mut world, _, _, _) = pair_world();
    let h = Handle { index: 2, generation: 0 };
    let collider = world.collider(h).unwrap();
    assert!(world.set_collider(opaque(h), collider));
}

#[test]
fn gas_impulse_joint() {
    let (mut world, _, _, joint) = pair_world();
    assert!(world.impulse_joint(opaque(joint)).is_some());
}

#[test]
fn gas_contact_pair() {
    let (mut world, _, _, _) = pair_world();
    let _ = world.step();
    let h = Handle { index: 2, generation: 0 };
    let _ = world.contact_pair(opaque(h), h);
}

#[test]
fn gas_step() {
    let (mut world, _, _, _) = pair_world();
    world.gravity = opaque(world.gravity);
    let _ = world.step();
}
#[test]
fn gas_step_wrapped() {
    let (mut world, _, _, _) = pair_world();
    world.gravity = opaque(world.gravity);
    let _ = super::alternatives::step_wrapped(ref world);
}

/// `0.1` s: a body at rest sleeps after a few steps.
const SHORT_SLEEP: Fixed = Fixed { raw: 429496730 };

/// A ball resting on a standalone ground (default gravity) and a standalone sensor around it
/// (collision events on), stepped until the ball sleeps: `(world, ball, ground, ball
/// collider, sensor)`. The sensor pair is dormant then.
fn sleeping_ball_world() -> (World, Handle, Handle, Handle, Handle) {
    let mut world: World = Default::default();
    let ground = world
        .insert_collider(ColliderBuilderTrait::halfspace(Vec2 { x: ZERO, y: ONE }).build(), None);
    let mut rb = RigidBodyTrait::dynamic(at(ZERO, HALF));
    rb.activation.time_until_sleep = SHORT_SLEEP;
    let (ball, ball_co) = world.insert(rb, ColliderBuilderTrait::ball(HALF).build());
    let sensor = world
        .insert_collider(
            ColliderBuilderTrait::ball(ONE)
                .sensor(true)
                .active_events(COLLISION_EVENTS)
                .position(at(ZERO, HALF))
                .build(),
            None,
        );
    let events = world.step();
    assert_eq!(events, array![CollisionEvent::Started((ball_co, sensor, SENSOR))]);
    let mut i = 0;
    while i != 20 {
        let _ = world.step();
        i += 1;
    }
    assert!(world.body(ball).unwrap().is_sleeping());
    assert_eq!(world.intersection_pair(ball_co, sensor), Some(true));
    (world, ball, ground, ball_co, sensor)
}

/// Upstream-exact removal: removing a sensor wakes none of its partners and its dormant pair
/// ends at the next step with `Stopped | SENSOR | REMOVED`; removing a contact partner wakes the
/// sleeping body at once.
#[test]
fn test_removal_wakes_contact_partners_only() {
    let (mut world, ball, _, ball_co, sensor) = sleeping_ball_world();
    assert!(world.remove_collider(sensor).is_some());
    assert!(world.body(ball).unwrap().is_sleeping());
    let events = world.step();
    assert_eq!(events, array![CollisionEvent::Stopped((ball_co, sensor, SENSOR | REMOVED))]);
    assert!(world.body(ball).unwrap().is_sleeping());
    assert_eq!(world.intersection_pairs(), array![]);
    assert_eq!(world.narrow_phase.pairs.len(), 1);
    let (mut world, ball, ground, ball_co, _) = sleeping_ball_world();
    assert!(world.remove_collider(ground).is_some());
    assert!(!world.body(ball).unwrap().is_sleeping());
    let _ = world.step();
    assert!(world.contact_pair(ground, ball_co).is_none());
}

/// `crowd(n)`: 20 dynamic balls far apart without gravity and one fixed body, stepped once
/// (change flags cleared); then every ball from the `n`-th on is made eligible and a second step
/// puts those to sleep. Returns the world and the fixed body.
fn crowd(awake: u32) -> (World, Handle) {
    let mut world = WorldTrait::new(Vec2 { x: ZERO, y: ZERO }, Default::default());
    let mut i: u32 = 0;
    while i != 20 {
        let x = FixedTrait::from_int((3 * i).try_into().unwrap());
        let _ = world
            .insert(RigidBodyTrait::dynamic(at(x, ZERO)), ColliderBuilderTrait::ball(HALF).build());
        i += 1;
    }
    let fixed = world.insert_body(RigidBodyTrait::fixed(at(ZERO, -FixedTrait::from_int(5))));
    let _ = world.step();
    for (handle, body) in world.bodies.iter() {
        if handle.index >= awake {
            let mut body = body;
            body.activation.time_since_can_sleep = body.activation.time_until_sleep;
            let _ = world.bodies.set(handle, body);
        }
    }
    let _ = world.step();
    (world, fixed)
}

/// Awake counts after sleep, then `wake_up_all` (weak: sleeping bodies strongly, awake timers
/// kept; strong: every timer reset); a fixed body is never active.
#[test]
fn test_active_bodies_and_wake_up_all() {
    // (awake balls before, strong)
    let cases = array![(8_u32, false), (8, true), (0, true), (20, false)];
    for (awake, strong) in cases {
        let (mut world, fixed) = crowd(awake);
        assert_eq!(world.num_active_bodies(), awake);
        let active = world.active_bodies();
        assert_eq!(active.len(), awake);
        let mut k: u32 = 0;
        for (handle, body) in active {
            assert_eq!(handle.index, k);
            assert!(!body.is_sleeping());
            k += 1;
        }
        // Awake balls have been moving for one step: a running timer.
        let mut timers = array![];
        for (_, body) in world.bodies.iter() {
            timers.append(body.activation.time_since_can_sleep);
        }
        world.wake_up_all(strong);
        assert_eq!(world.num_active_bodies(), 20);
        for (handle, body) in world.bodies.iter() {
            if handle == fixed {
                assert!(!body.is_sleeping());
            } else if handle.index >= awake || strong {
                assert_eq!(body.activation.time_since_can_sleep, ZERO);
            } else {
                assert_eq!(body.activation.time_since_can_sleep, *timers.at(handle.index));
            }
        }
        assert_eq!(world.active_bodies().len(), 20);
    }
}

/// `Default` (upstream gravity and parameters), the `PhysicsWorld` alias and the copy-out
/// arrays in handle order.
#[test]
fn test_default_alias_and_copy_out_arrays() {
    let mut world: PhysicsWorld = Default::default();
    assert_eq!(world.gravity, Vec2 { x: ZERO, y: DEFAULT_GRAVITY_Y });
    assert_eq!(DEFAULT_GRAVITY_Y.raw, -42133629174);
    assert_eq!(world.integration_parameters, Default::default());
    assert!(world.rigid_bodies().is_empty() && world.all_colliders().is_empty());
    let (mut world, _, _, _) = pair_world();
    assert_eq!(world.rigid_bodies(), world.bodies.iter());
    assert_eq!(world.all_colliders(), world.colliders.iter());
    assert_eq!(world.all_colliders().len(), 3);
}

#[test]
fn gas_crowd_setup() {
    let _ = crowd(opaque(8));
}

#[test]
fn gas_active_bodies_20_8() {
    let (mut world, _) = crowd(opaque(8));
    assert!(world.active_bodies().len() == 8);
}

#[test]
fn gas_num_active_bodies_20_8() {
    let (mut world, _) = crowd(opaque(8));
    assert!(world.num_active_bodies() == 8);
}

#[test]
fn gas_num_active_bodies_via_array_20_8() {
    let (mut world, _) = crowd(opaque(8));
    assert!(super::alternatives::num_active_bodies_via_array(ref world) == 8);
}

#[test]
fn gas_wake_up_all_20_8() {
    let (mut world, _) = crowd(opaque(8));
    world.wake_up_all(opaque(false));
}

#[test]
fn gas_rigid_bodies() {
    let (mut world, _, _, _) = pair_world();
    let _ = opaque(world.rigid_bodies());
}

#[test]
fn gas_all_colliders() {
    let (mut world, _, _, _) = pair_world();
    let _ = opaque(world.all_colliders());
}

#[test]
fn gas_default() {
    let world: World = Default::default();
    let _ = opaque(world.gravity);
}

/// Six unit cuboids built `sleeping(true)` (3 columns of 2, spacing `1 + gap`, lowest at
/// `y = 0.5 + gap` above a fixed ground whose top is `y = 0`, or no ground), an awake ball far
/// away. Returns the world and the six block handles.
fn insert_asleep_world(gap: Fixed, ground: bool) -> (World, Array<Handle>) {
    let mut world: World = Default::default();
    if ground {
        let _ = world
            .insert(
                RigidBodyTrait::fixed(at(ZERO, ZERO)),
                ColliderBuilderTrait::cuboid(FixedTrait::from_int(10), HALF).build(),
            );
    }
    let step = ONE + gap;
    let mut blocks = array![];
    let mut i: u32 = 0;
    while i != 6 {
        let column: i32 = (i / 2).try_into().unwrap();
        let row: i32 = (i % 2).try_into().unwrap();
        let x = step * FixedTrait::from_int(column);
        let y = ONE + gap + step * FixedTrait::from_int(row);
        let body = RigidBodyBuilderTrait::dynamic()
            .translation(Vec2 { x, y })
            .sleeping(true)
            .build();
        let (h, _) = world.insert(body, ColliderBuilderTrait::cuboid(HALF, HALF).build());
        blocks.append(h);
        i += 1;
    }
    let pebble = RigidBodyTrait::dynamic(at(FixedTrait::from_int(-8), FixedTrait::from_int(5)));
    let _ = world.insert(pebble, ColliderBuilderTrait::ball(Fixed { raw: 858993459 }).build());
    (world, blocks)
}

fn asleep_blocks(ref world: World, blocks: Span<Handle>) -> u32 {
    let mut n = 0;
    for h in blocks {
        if world.body(*h).unwrap().is_sleeping() {
            n += 1;
        }
    }
    n
}

/// BT2 addendum A, against upstream rapier2d-f64 0.35.3 (the same scene run in `tools/golden`,
/// 2026-09-25): bodies inserted asleep stay asleep while nothing touches them (with or without
/// the ground: a collider inserted since the last step wakes nobody), and a pile inserted asleep
/// in contact wakes at its first step (the contacts start: upstream `strong_wake_sleeping_side`).
/// Upstream after 1 and 10 steps: gap 0 → 0 of 6 blocks asleep; gap 0.5 → 6 of 6. A later
/// friction change wakes nobody (no change flag, as upstream).
fn insert_asleep_case(gap: Fixed, ground: bool, expected: u32, steps: u32) {
    let (mut world, blocks) = insert_asleep_world(gap, ground);
    let _ = world.step();
    assert_eq!(asleep_blocks(ref world, blocks.span()), expected, "step 1");
    let mut t = 1;
    while t != steps {
        let _ = world.step();
        t += 1;
    }
    assert_eq!(asleep_blocks(ref world, blocks.span()), expected, "later step");
    let co = *world.body(*blocks.at(0)).unwrap().colliders.at(0);
    let mut collider = world.collider(co).unwrap();
    collider.set_friction(Fixed { raw: 1288490189 });
    assert!(world.set_collider(co, collider));
    let _ = world.step();
    assert_eq!(asleep_blocks(ref world, blocks.span()), expected, "friction change");
}

/// Apart: every block stays asleep (10 steps, as upstream's record).
#[test]
fn test_insert_asleep_apart_stays_asleep() {
    insert_asleep_case(HALF, true, 6, 10);
    insert_asleep_case(HALF, false, 6, 10);
}

/// In contact: the pile wakes at its first step (2 steps here: the step budget of a unit test).
#[test]
fn test_insert_asleep_pile_wakes() {
    insert_asleep_case(ZERO, true, 0, 2);
    insert_asleep_case(ZERO, false, 0, 2);
}

/// Addendum B: the cheap activation reads against a whole-body copy (steps are the measure,
/// `--tracked-resource cairo-steps`): `probe(k)` reads one body of a 3-body world `k` ways.
fn read_probe(kind: u8) {
    let (mut world, a, _, _) = pair_world();
    let h = opaque(a);
    if kind == 1 {
        let _ = opaque(world.body(h));
    } else if kind == 2 {
        let _ = opaque(world.body(h).unwrap().is_sleeping());
    } else if kind == 3 {
        let _ = opaque(world.is_sleeping(h));
    } else if kind == 4 {
        let _ = opaque(world.linvel(h));
    } else if kind == 5 {
        let _ = opaque(world.angvel(h));
    } else if kind == 6 {
        let _ = opaque(Some(h.index == 7));
    }
}

#[test]
fn gas_read_setup() {
    read_probe(0);
}

#[test]
fn gas_read_body() {
    read_probe(1);
}

#[test]
fn gas_read_body_is_sleeping() {
    read_probe(2);
}

#[test]
fn gas_read_is_sleeping() {
    read_probe(3);
}

#[test]
fn gas_read_linvel() {
    read_probe(4);
}

#[test]
fn gas_read_angvel() {
    read_probe(5);
}

#[test]
fn gas_read_overhead() {
    read_probe(6);
}

#[test]
fn test_activation_reads() {
    let (mut world, a, _, _) = pair_world();
    let mut body = world.body(a).unwrap();
    body.set_linvel(Vec2 { x: ONE, y: HALF });
    body.set_angvel(HALF);
    assert!(world.set_body(a, body));
    assert_eq!(world.is_sleeping(a), Some(false));
    assert_eq!(world.linvel(a), Some(Vec2 { x: ONE, y: HALF }));
    assert_eq!(world.angvel(a), Some(HALF));
    let mut body = world.body(a).unwrap();
    body.sleep();
    assert!(world.set_body(a, body));
    assert_eq!(world.is_sleeping(a), Some(true));
    let stale = Handle { index: a.index, generation: a.generation + 1 };
    assert_eq!(world.is_sleeping(stale), None);
    assert_eq!(world.linvel(stale), None);
    assert_eq!(world.angvel(stale), None);
}
