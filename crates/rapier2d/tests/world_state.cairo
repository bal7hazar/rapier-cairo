//! WS acceptance: chunked execution through the versioned world state is bit-exact.
//!
//! For every scene and chunk size `K`, a reference world is stepped `n` times while a second one
//! is stepped by chunks of `K` steps, each chunk ending with
//! `from_state ∘ deserialize ∘ serialize ∘ to_state`. After every step both worlds must hold
//! the same state (every field of `World`, through `to_state`) and the step must have returned the
//! same events. Scripted mutations (joint insertion, body and collider removals, then new
//! insertions reusing the freed slots) run on both worlds and must return the same handles.
//!
//! Scenes (`rapier_golden::scenes`): `box_stack3` (resting contacts, warm start), `pendulum`
//! (revolute joint impulses), `box_stack3` with removals and reinsertions, and `ball_drop_sleep`
//! plus a standalone sensor slab around the resting ball (the ball falls asleep inside the sensor
//! at step 66, `ball2` crosses the sensor and wakes it up at step 87).

use rapier2d::prelude::{
    ColliderBuilderTrait, CollisionEvent, CollisionEventTrait, Fixed, Handle, IntegrationParameters,
    RevoluteJointBuilderTrait, RigidBodyTrait, Vec2, World, WorldTrait,
};
use rapier2d::world::state::{WORLD_STATE_VERSION, WorldState};
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_core::rigid_body::RigidBodyActivationTrait;
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::joint::ImpulseJointSetTrait;
use rapier_dynamics2d::rigid_body_set::RigidBodySetTrait;
use rapier_golden::scenes;
use rapier_golden::types::{BodyKindRaw, PoseRaw, SceneCase, ShapeRaw, Vec2Raw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;

const ONE_RAW: i64 = 4294967296;
const HALF_RAW: i64 = 2147483648;

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

fn vr(raw: Vec2Raw) -> Vec2 {
    Vec2 { x: f(raw.x), y: f(raw.y) }
}

fn at(x: i64, y: i64) -> Pose2 {
    Pose2 { translation: Vec2 { x: f(x), y: f(y) }, rotation: Rot2 { re: f(ONE_RAW), im: f(0) } }
}

fn pose(raw: PoseRaw) -> Pose2 {
    Pose2 {
        translation: vr(raw.translation),
        rotation: Rot2 { re: f(raw.rotation.re), im: f(raw.rotation.im) },
    }
}

fn h(index: u32, generation: u32) -> Handle {
    Handle { index, generation }
}

/// The world of a golden scene (as `golden_scenes::builder::build_world`): bodies in scene
/// order with their first collider, then the revolute joints.
fn build_world(scene: SceneCase, can_sleep: bool) -> World {
    let params = IntegrationParameters {
        dt: f(scene.dt), num_solver_iterations: 4, ..Default::default(),
    };
    let mut world = WorldTrait::new(vr(scene.gravity), params);
    let mut i = 0;
    for desc in scene.bodies.span() {
        if i == scene.num_bodies {
            break;
        }
        let mut body = match desc.kind {
            BodyKindRaw::Fixed => RigidBodyTrait::fixed(pose(*desc.pose)),
            BodyKindRaw::Dynamic => RigidBodyTrait::dynamic(pose(*desc.pose)),
        };
        if !can_sleep {
            body.activation = RigidBodyActivationTrait::cannot_sleep();
        }
        body.damping.linear_damping = f(*desc.linear_damping);
        body.damping.angular_damping = f(*desc.angular_damping);
        body.forces.gravity_scale = f(*desc.gravity_scale);
        let handle = world.insert_body(body);
        if *desc.num_colliders != 0 {
            let co = *desc.colliders.span().at(0);
            let builder = match co.shape {
                ShapeRaw::Ball(radius) => ColliderBuilderTrait::ball(f(radius)),
                ShapeRaw::Cuboid(half) => ColliderBuilderTrait::cuboid(f(half.x), f(half.y)),
                _ => panic!("unexpected scene shape"),
            };
            let collider = builder
                .position(pose(co.pose_wrt_parent))
                .density(f(co.density))
                .friction(f(co.friction))
                .restitution(f(co.restitution))
                .build();
            let _ = world.insert_collider(collider, Some(handle));
        }
        i += 1;
    }
    let mut j = 0;
    for joint in scene.joints.span() {
        if j == scene.num_joints {
            break;
        }
        let data = RevoluteJointBuilderTrait::new()
            .local_anchor1(vr(*joint.local_anchor1))
            .local_anchor2(vr(*joint.local_anchor2))
            .build();
        let _ = world.insert_impulse_joint(h(*joint.body1, 0), h(*joint.body2, 0), data);
        j += 1;
    }
    world
}

/// The scenes of the acceptance table.
#[derive(Copy, Drop, PartialEq, Debug)]
enum Scene {
    Stack,
    Pendulum,
    Removals,
    SleepSensor,
}

fn build(scene: Scene) -> World {
    match scene {
        Scene::Stack | Scene::Removals => build_world(scenes::BOX_STACK3, false),
        Scene::Pendulum => build_world(scenes::PENDULUM, false),
        Scene::SleepSensor => {
            let mut world = build_world(scenes::BALL_DROP_SLEEP, true);
            // Slab over y in [0.25, 1.75]: holds the resting ball, crossed by `ball2`.
            let sensor = ColliderBuilderTrait::cuboid(f(ONE_RAW), f(3 * HALF_RAW / 2))
                .position(at(0, ONE_RAW))
                .sensor(true)
                .active_events(COLLISION_EVENTS)
                .build();
            let _ = world.insert_collider(sensor, None);
            world
        },
    }
}

/// Mutations applied before step `step` (0-based); returns the handles they issued.
fn act(ref world: World, scene: Scene, step: u32) -> Array<Handle> {
    let mut issued = array![];
    if scene != Scene::Removals {
        return issued;
    }
    if step == 2 {
        let joint = RevoluteJointBuilderTrait::new()
            .local_anchor1(Vec2 { x: f(0), y: f(HALF_RAW) })
            .local_anchor2(Vec2 { x: f(0), y: f(-HALF_RAW) })
            .build();
        issued.append(world.insert_impulse_joint(h(2, 0), h(3, 0), joint));
    } else if step == 4 {
        // Its collider and its joint go with it.
        assert!(world.remove_body(h(3, 0)).is_some());
    } else if step == 5 {
        assert!(world.remove_collider(h(1, 0)).is_some());
    } else if step == 9 {
        let (body, collider) = world
            .insert(
                RigidBodyTrait::dynamic(at(2 * ONE_RAW, 4 * ONE_RAW)),
                ColliderBuilderTrait::ball(f(HALF_RAW)).build(),
            );
        issued.append(body);
        issued.append(collider);
        issued
            .append(
                world
                    .insert_collider(
                        ColliderBuilderTrait::ball(f(HALF_RAW / 2)).build(), Some(h(1, 0)),
                    ),
            );
        issued
            .append(
                world.insert_impulse_joint(h(2, 0), body, RevoluteJointBuilderTrait::new().build()),
            );
        // The freed slots are reused with bumped generations, as the uninterrupted run does.
        assert_eq!(issued, array![h(3, 1), h(1, 2), h(3, 2), h(0, 1)]);
    }
    issued
}

/// `from_state ∘ deserialize ∘ serialize ∘ to_state`.
fn round_trip(ref world: World) -> World {
    let state = world.to_state();
    let mut felts = array![];
    state.serialize(ref felts);
    assert_eq!(*felts.at(0), WORLD_STATE_VERSION.into());
    let mut span = felts.span();
    let restored: WorldState = Serde::deserialize(ref span).unwrap();
    assert!(span.is_empty());
    assert!(restored == state);
    WorldTrait::from_state(restored)
}

/// What a run went through, to prove each scene covers what it claims.
#[derive(Copy, Drop, Default)]
struct Seen {
    sensor_events: u32,
    sleeping_steps: u32,
    /// Some body sleeps after the last step.
    sleeping_at_end: bool,
    joint_impulse: bool,
}

fn observe(ref world: World, events: Span<CollisionEvent>, ref seen: Seen) {
    for event in events {
        if (*event).sensor() {
            seen.sensor_events += 1;
        }
    }
    seen.sleeping_at_end = false;
    for (_, body) in world.bodies.iter() {
        if body.is_sleeping() {
            seen.sleeping_steps += 1;
            seen.sleeping_at_end = true;
            break;
        }
    }
    for (_, joint) in world.impulse_joints.to_array() {
        let [a, b, c] = joint.impulses;
        if a.raw != 0 || b.raw != 0 || c.raw != 0 {
            seen.joint_impulse = true;
        }
    }
}

/// `step^n ≡ (from_state ∘ deserialize ∘ serialize ∘ to_state ∘ step^k)^(n / k)`, checked
/// after every step on the whole state and the events.
fn run(scene: Scene, k: u32, steps: u32) -> Seen {
    let mut reference = build(scene);
    let mut chunked = build(scene);
    let mut seen: Seen = Default::default();
    let mut step = 0;
    while step != steps {
        assert_eq!(act(ref reference, scene, step), act(ref chunked, scene, step));
        let expected = reference.step();
        let got = chunked.step();
        assert!(expected == got, "events differ at step {}", step);
        observe(ref reference, expected.span(), ref seen);
        step += 1;
        if step % k == 0 {
            let restored = round_trip(ref chunked);
            chunked = restored;
        }
        assert!(reference.to_state() == chunked.to_state(), "state differs after step {}", step);
    }
    // Both worlds keep issuing the same handles.
    let collider = ColliderBuilderTrait::ball(f(HALF_RAW)).build();
    let body = RigidBodyTrait::dynamic(at(0, 8 * ONE_RAW));
    assert_eq!(reference.insert(body, collider), chunked.insert(body, collider));
    assert_eq!(reference.colliders.len(), chunked.colliders.len());
    seen
}

#[test]
fn test_chunked_stack_k1() {
    let _ = run(Scene::Stack, 1, 20);
}

#[test]
fn test_chunked_stack_k7() {
    let _ = run(Scene::Stack, 7, 30);
}

#[test]
fn test_chunked_pendulum_k1() {
    let seen = run(Scene::Pendulum, 1, 20);
    assert!(seen.joint_impulse);
}

#[test]
fn test_chunked_pendulum_k7() {
    let _ = run(Scene::Pendulum, 7, 30);
}

#[test]
fn test_chunked_removals_k1() {
    let seen = run(Scene::Removals, 1, 20);
    assert!(seen.joint_impulse);
}

#[test]
fn test_chunked_removals_k7() {
    let _ = run(Scene::Removals, 7, 30);
}

#[test]
fn test_chunked_sleep_sensor_k1() {
    let seen = run(Scene::SleepSensor, 1, 95);
    assert!(seen.sleeping_steps > 10 && !seen.sleeping_at_end && seen.sensor_events != 0);
}

#[test]
fn test_chunked_sleep_sensor_k7() {
    let seen = run(Scene::SleepSensor, 7, 95);
    assert!(seen.sleeping_steps > 10 && !seen.sleeping_at_end && seen.sensor_events != 0);
}

/// Set-level round trips after removals: the restored set issues the next handle the original
/// issues (free list and generation included).
#[test]
fn test_set_round_trips_after_removals() {
    let mut world = build_world(scenes::BOX_STACK3, false);
    let _ = world.insert_impulse_joint(h(1, 0), h(2, 0), RevoluteJointBuilderTrait::new().build());
    let _ = world.insert_impulse_joint(h(2, 0), h(3, 0), RevoluteJointBuilderTrait::new().build());
    assert!(world.remove_body(h(2, 0)).is_some());
    assert!(world.remove_collider(h(0, 0)).is_some());

    let bodies_state = world.bodies.to_state();
    let mut bodies = RigidBodySetTrait::from_state(bodies_state);
    assert!(bodies.to_state() == bodies_state);
    let body = RigidBodyTrait::dynamic(at(0, 0));
    assert_eq!(bodies.insert(body), world.bodies.insert(body));

    let colliders_state = world.colliders.to_state();
    assert_eq!(colliders_state.free_list, array![0, 2].span());
    let mut colliders = ColliderSetTrait::from_state(colliders_state);
    assert!(colliders.to_state() == colliders_state);
    let collider = ColliderBuilderTrait::ball(f(HALF_RAW)).build();
    let issued = colliders.insert(collider);
    assert_eq!(issued, world.colliders.insert(collider));
    assert_eq!(issued, h(0, 2));

    let joints_state = world.impulse_joints.to_state();
    assert_eq!((joints_state.generation, joints_state.capacity), (2, 2));
    let mut joints = ImpulseJointSetTrait::from_state(joints_state);
    assert!(joints.to_state() == joints_state);
    let data = RevoluteJointBuilderTrait::new().build();
    assert_eq!(
        joints.insert(h(1, 0), h(3, 0), data), world.impulse_joints.insert(h(1, 0), h(3, 0), data),
    );
}

#[test]
#[should_panic(expected: 'world state: version')]
fn test_version_mismatch_panics() {
    let mut world = build(Scene::Pendulum);
    let _ = world.step();
    let mut state = world.to_state();
    state.version = 2;
    let _ = WorldTrait::from_state(state);
}
