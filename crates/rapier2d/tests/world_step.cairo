//! `World` / `step()` behaviour scenarios through the public API only, and the headline
//! per-step gas probes (`gas_step_*` minus the matching `gas_setup_*` is one step).
//!
//! Tolerances follow `tools/golden/README.md`: `2^12 · step` ulp on the positions of
//! single-contact and joint scenes, twice that on the velocities.

use core::num::traits::DivRem;
use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier2d::world::{World, WorldTrait};
use rapier_core::Handle;
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_core::rigid_body::{RigidBodyActivationTrait, RigidBodyType};
use rapier_dynamics2d::collider::{ColliderBuilderTrait, ColliderTrait};
use rapier_dynamics2d::events::{CollisionEvent, CollisionEventTrait};
use rapier_dynamics2d::joint::RevoluteJointBuilderTrait;
use rapier_dynamics2d::narrow_phase::ContactPairTrait;
use rapier_dynamics2d::rigid_body::RigidBodyMassPropsTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodyTrait};
use rapier_geometry2d::contact::NEW_CONTACT_BIT;
use rapier_geometry2d::shape::{BallTrait, CuboidTrait, Shape};
use rapier_golden::compare::within;
use rapier_golden::scenes;
use rapier_golden::types::{BodyKindRaw, PoseRaw, SceneCase, ShapeRaw, Vec2Raw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

const GRAVITY_Y: i64 = -42133629174;

// KD blocker reproducer. Two touching unit boxes, no gravity or friction. The left
// body moves right at 1 m/s. A contact must transfer that motion to the right body
// in the first step when the left body is dynamic or kinematic with equal dominance.
// The dominant case records upstream's distinct world-attached endpoint semantics.
fn kd_pusher(kind: RigidBodyType, dominant: bool) -> (RigidBody, RigidBody) {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    let mut driver = RigidBodyTrait::new(kind, at(-ONE, ZERO));
    driver.activation = RigidBodyActivationTrait::cannot_sleep();
    driver.set_linvel(v(ONE, ZERO));
    if dominant {
        driver.dominance.group = 1;
    }
    let collider = ColliderBuilderTrait::cuboid(HALF, HALF).friction(ZERO).build();
    let (left, _) = world.insert(driver, collider);
    let mut passenger = RigidBodyTrait::dynamic(at(ZERO, ZERO));
    passenger.activation = RigidBodyActivationTrait::cannot_sleep();
    let (right, _) = world.insert(passenger, collider);
    let _ = world.step();
    (world.body(left).unwrap(), world.body(right).unwrap())
}

#[test]
fn test_kd_dynamic_pusher_control() {
    let (driver, passenger) = kd_pusher(RigidBodyType::Dynamic, false);
    assert!(passenger.linvel().x > ZERO, "ordinary dynamic contact transfers motion");
    assert!(driver.linvel().x < ONE, "ordinary dynamic driver receives reaction");
}

/// Blocked by contact.cairo: immovable endpoints become WORLD and lose their velocity.
/// Run with `snforge test -p rapier2d test_kd_ --include-ignored` to reproduce.
/// Intentionally ignored until the orchestrator authorizes the contact-solver prerequisite.
#[test]
#[ignore]
fn test_kd_kinematic_pusher_transfers_motion() {
    let (driver, passenger) = kd_pusher(RigidBodyType::KinematicVelocityBased, false);
    assert_eq!(driver.linvel().x, ONE, "kinematic velocity is unaffected");
    assert!(driver.position().translation.x > -ONE, "kinematic driver advances");
    assert!(
        passenger.linvel().x > ZERO,
        "moving kinematic contact must push: {}",
        passenger.linvel().x.raw,
    );
}

/// Pinned upstream treats the dominance-superior endpoint as world-attached: its
/// velocity does not reach this contact. Keep this parity check distinct from kinematics.
#[test]
fn test_kd_dominant_pusher_matches_upstream() {
    let (driver, passenger) = kd_pusher(RigidBodyType::Dynamic, true);
    assert_eq!(driver.linvel().x, ONE, "dominant velocity is unaffected");
    assert_eq!(passenger.linvel().x, ZERO, "upstream dominant endpoint is world-attached");
}

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn vr(raw: Vec2Raw) -> Vec2 {
    v(f(raw.x), f(raw.y))
}

fn pose(raw: PoseRaw) -> Pose2 {
    Pose2 {
        translation: vr(raw.translation),
        rotation: Rot2 { re: f(raw.rotation.re), im: f(raw.rotation.im) },
    }
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: v(x, y), rotation: Rot2 { re: ONE, im: ZERO } }
}

fn gravity() -> Vec2 {
    v(ZERO, f(GRAVITY_Y))
}

/// A golden scene rebuilt through the public API; the body handles in scene order.
fn scene_world(scene: SceneCase) -> (World, Array<Handle>) {
    let params = IntegrationParameters { dt: f(scene.dt), ..Default::default() };
    let mut world = WorldTrait::new(vr(scene.gravity), params);
    let mut handles = array![];
    for desc in scene.bodies.span() {
        if handles.len() == scene.num_bodies {
            break;
        }
        let mut body = match desc.kind {
            BodyKindRaw::Fixed => RigidBodyTrait::fixed(pose(*desc.pose)),
            BodyKindRaw::Dynamic => RigidBodyTrait::dynamic(pose(*desc.pose)),
        };
        // The traces replayed here were recorded with `can_sleep(false)`.
        body.activation = RigidBodyActivationTrait::cannot_sleep();
        body.damping.linear_damping = f(*desc.linear_damping);
        body.damping.angular_damping = f(*desc.angular_damping);
        body.forces.gravity_scale = f(*desc.gravity_scale);
        let handle = world.insert_body(body);
        if *desc.num_colliders != 0 {
            let co = *desc.colliders.span().at(0);
            let shape = match co.shape {
                ShapeRaw::Ball(radius) => Shape::Ball(BallTrait::new(f(radius))),
                ShapeRaw::Cuboid(half) => Shape::Cuboid(CuboidTrait::new(vr(half))),
                _ => panic!("unexpected scene shape"),
            };
            let collider = ColliderBuilderTrait::new(shape)
                .position(pose(co.pose_wrt_parent))
                .density(f(co.density))
                .friction(f(co.friction))
                .restitution(f(co.restitution))
                .build();
            let _ = world.insert_collider(collider, Some(handle));
        }
        handles.append(handle);
    }
    if scene.num_joints != 0 {
        let joint = *scene.joints.span().at(0);
        let data = RevoluteJointBuilderTrait::new()
            .local_anchor1(vr(joint.local_anchor1))
            .local_anchor2(vr(joint.local_anchor2))
            .build();
        let _ = world
            .insert_impulse_joint(*handles.at(joint.body1), *handles.at(joint.body2), data);
    }
    (world, handles)
}

/// A half-space `y ≥ 0` and a ball of radius 1/2 at height `height` (collision events on).
fn ball_on_ground(height: Fixed) -> (World, Handle, Handle, Handle) {
    let mut world = WorldTrait::new(gravity(), Default::default());
    let ground = world.insert_collider(ColliderBuilderTrait::halfspace(v(ZERO, ONE)).build(), None);
    let (ball, ball_collider) = world
        .insert(
            RigidBodyTrait::dynamic(at(ZERO, height)),
            ColliderBuilderTrait::ball(HALF).active_events(COLLISION_EVENTS).build(),
        );
    (world, ground, ball, ball_collider)
}

/// Steps `n` times and returns every event, in order.
fn run(ref world: World, n: u32) -> Array<CollisionEvent> {
    let mut events = array![];
    let mut i = 0;
    while i != n {
        events.append_span(world.step().span());
        i += 1;
    }
    events
}

fn body(ref world: World, handle: Handle) -> RigidBody {
    world.body(handle).unwrap()
}

/// Free fall matches DA's closed form of symplectic Euler: every substep of `h = dt / 4` adds
/// `g·h` to the velocity, then `v·h` to the height (`v = g·h·n`, `y = y0 + g·h²·n(n+1)/2` up
/// to one floor per product); 2 ulp per substep.
#[test]
fn test_free_fall_matches_the_closed_form() {
    let mut world = WorldTrait::new(gravity(), Default::default());
    let y0 = FixedTrait::from_int(10);
    let (ball, _) = world
        .insert(RigidBodyTrait::dynamic(at(ZERO, y0)), ColliderBuilderTrait::ball(HALF).build());
    let h = world.integration_parameters.substep_dt();
    let increment = f(GRAVITY_Y) * h;
    let (mut vy, mut y) = (ZERO, y0);
    let mut n: u64 = 0;
    while n != 48 {
        vy = vy + increment;
        y = y + vy * h;
        n += 1;
        let (k, r) = DivRem::div_rem(n, 4);
        if r != 0 {
            continue;
        }
        assert_eq!(world.step().len(), 0);
        let rb = body(ref world, ball);
        assert!(within(rb.linvel().y.raw, vy.raw, 2 * n), "vy after {} steps", k);
        assert!(within(rb.position().translation.y.raw, y.raw, 2 * n), "y after {} steps", k);
        assert_eq!(rb.linvel().x, ZERO);
        assert_eq!(rb.position().rotation, Rot2 { re: ONE, im: ZERO });
    }
    // The discrete chain is the closed form up to rounding.
    let exact = y0 + increment * h * FixedTrait::from_int(48 * 49 / 2);
    assert!(within(y.raw, exact.raw, 2048));
}

/// A ball dropped on a half-space comes to rest on it and emits exactly one `Started`; removing
/// the ball then emits one `Stopped` flagged `REMOVED`, and no pair is left.
#[test]
fn test_ball_rests_on_halfspace_then_removal_stops_the_pair() {
    let (mut world, ground, ball, ball_collider) = ball_on_ground(FixedTrait::from_int(1));
    let events = run(ref world, 90);
    assert_eq!(events.len(), 1);
    let event = *events.at(0);
    assert!(event.started());
    assert_eq!((event.collider1(), event.collider2()), (ground, ball_collider));
    let rb = body(ref world, ball);
    let allowed = world.integration_parameters.allowed_linear_error();
    assert!(within(rb.position().translation.y.raw, HALF.raw, allowed.raw.try_into().unwrap()));
    assert!(within(rb.linvel().y.raw, 0, 1_u64 * 4294967), "at rest (|vy| < 1e-3)");
    assert!(world.contact_pair(ground, ball_collider).is_some());

    assert!(world.remove_body(ball).is_some());
    assert!(world.body(ball).is_none());
    assert!(world.collider(ball_collider).is_none());
    let events = world.step();
    assert_eq!(events.len(), 1);
    let event = *events.at(0);
    assert!(event.stopped() && event.removed());
    assert_eq!((event.collider1(), event.collider2()), (ground, ball_collider));
    assert!(world.contact_pair(ground, ball_collider).is_none());
    assert_eq!(world.step().len(), 0);
}

/// Warm start: the first contact step marks its point new; the next resting step finds it again
/// (no `NEW_CONTACT_BIT`) with a positive accumulated impulse.
#[test]
fn test_resting_contact_is_warm_started() {
    let (mut world, ground, _, ball_collider) = ball_on_ground(HALF);
    let _ = world.step();
    let first = world.contact_pair(ground, ball_collider).unwrap().manifold;
    assert_eq!(first.data.num_solver_contacts, 1);
    let [c0, _] = first.data.solver_contacts;
    assert!(c0.contact_id >= NEW_CONTACT_BIT);
    let [p0, _] = first.points;
    assert!(p0.data.impulse > ZERO);
    let _ = world.step();
    let second = world.contact_pair(ground, ball_collider).unwrap().manifold;
    let [c0, _] = second.data.solver_contacts;
    assert_eq!(c0.contact_id, 0);
    let [p0, _] = second.points;
    assert!(p0.data.impulse > ZERO);
}

/// A collider attached (and removed) after the first step changes the body mass at the next
/// step, as upstream's `recompute_mass_properties_from_colliders`.
#[test]
fn test_attached_collider_changes_the_body_mass() {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    let (ball, _) = world
        .insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)), ColliderBuilderTrait::ball(HALF).build());
    let _ = world.step();
    let m1 = body(ref world, ball).mprops.mass();
    let extra = world
        .insert_collider(
            ColliderBuilderTrait::cuboid(HALF, HALF).translation(v(ONE, ZERO)).build(), Some(ball),
        );
    let _ = world.step();
    let rb = body(ref world, ball);
    let m2 = rb.mprops.mass();
    // Ball π/4 plus unit box at density 1; the centre of mass moves toward the box.
    assert!(within(m2.raw, (m1 + ONE).raw, 64));
    assert!(rb.mprops.world_com.x > ZERO);
    assert!(world.collider(extra).unwrap().position().translation.x == ONE);
    assert!(world.remove_collider(extra).is_some());
    let _ = world.step();
    let rb = body(ref world, ball);
    assert!(within(rb.mprops.mass().raw, m1.raw, 64));
    assert!(within(rb.mprops.world_com.x.raw, 0, 64));
}

/// A fixed body never moves, even when a dynamic box lands on it and pushes it.
#[test]
fn test_fixed_body_never_moves() {
    let mut world = WorldTrait::new(gravity(), Default::default());
    let start = at(ONE, -HALF);
    let (ground, ground_collider) = world
        .insert(
            RigidBodyTrait::fixed(start), ColliderBuilderTrait::cuboid(ONE + ONE, HALF).build(),
        );
    let (_, box_collider) = world
        .insert(
            RigidBodyTrait::dynamic(at(ONE, HALF + HALF)),
            ColliderBuilderTrait::cuboid(HALF, HALF).build(),
        );
    let _ = run(ref world, 40);
    let rb = body(ref world, ground);
    assert_eq!(rb.position(), start);
    assert_eq!(rb.pos.next_position, start);
    assert_eq!(rb.linvel(), v(ZERO, ZERO));
    assert_eq!(world.collider(ground_collider).unwrap().position(), start);
    assert!(world.contact_pair(ground_collider, box_collider).unwrap().has_any_active_contact());
}

/// Golden replays through `World::step`: `(scene, last step compared)`.
#[test]
fn test_golden_scene_replays() {
    for (scene, last) in array![(scenes::BALL_DROP, 30_u32), (scenes::PENDULUM, 10)].span() {
        replay(*scene, *last);
    }
}

fn replay(scene: SceneCase, last: u32) {
    let (mut world, handles) = scene_world(scene);
    let mut done = 0;
    for sample in scene.samples.span() {
        if *sample.step > last {
            break;
        }
        let _ = run(ref world, *sample.step - done);
        done = *sample.step;
        let tol: u64 = 4096 * (*sample.step).into();
        let mut k = 0;
        while k != scene.num_dynamic {
            let expected = *sample.states.span().at(k);
            let rb = body(ref world, *handles.at(expected.body));
            let p = rb.position();
            for (got, want, budget) in array![
                (p.translation.x.raw, expected.translation.x, tol),
                (p.translation.y.raw, expected.translation.y, tol),
                (p.rotation.re.raw, expected.rotation.re, tol),
                (p.rotation.im.raw, expected.rotation.im, tol),
                (rb.linvel().x.raw, expected.linvel.x, 2 * tol),
                (rb.linvel().y.raw, expected.linvel.y, 2 * tol),
                (rb.vels.angvel.raw, expected.angvel, 2 * tol),
            ]
                .span() {
                assert!(
                    within(*got, *want, *budget),
                    "{} step {} got {} want {}",
                    scene.id,
                    *sample.step,
                    *got,
                    *want,
                );
            }
            k += 1;
        }
    }
    assert_eq!(done, last);
}

// Sleeping (SL): behaviour through the public API.

/// A unit box resting on the half-space `y ≥ 0` at `x`, gravity `(0, -9.81)`.
fn resting_box(ref world: World, x: Fixed) -> (Handle, Handle) {
    world
        .insert(
            RigidBodyTrait::dynamic(at(x, HALF)), ColliderBuilderTrait::cuboid(HALF, HALF).build(),
        )
}

/// Who sleeps after 60 steps at rest on the ground: a dynamic body with the default thresholds;
/// not one that `cannot_sleep` (negative thresholds), not a fixed body, not a moving kinematic
/// body (velocity-based, `linvel.x = 0.01`); a motionless kinematic body does (upstream's
/// `update_energy`: exactly zero velocities). Rows: (kind, sleeps).
#[test]
fn test_who_sleeps_after_a_second_at_rest() {
    let rows: Array<(felt252, bool)> = array![
        ('default', true), ('cannot_sleep', false), ('fixed', false), ('kinematic_moving', false),
        ('kinematic_still', true),
    ];
    for (kind, sleeps) in rows {
        let mut world = WorldTrait::new(gravity(), Default::default());
        let _ = world.insert_collider(ColliderBuilderTrait::halfspace(v(ZERO, ONE)).build(), None);
        let mut sleeper = if kind == 'fixed' {
            RigidBodyTrait::fixed(at(ZERO, HALF))
        } else if kind == 'kinematic_moving' {
            let mut b = RigidBodyTrait::new(
                rapier_core::rigid_body::RigidBodyType::KinematicVelocityBased, at(ZERO, HALF),
            );
            b.vels.linvel = v(f(42949673), ZERO);
            b
        } else if kind == 'kinematic_still' {
            RigidBodyTrait::kinematic_position_based(at(ZERO, HALF))
        } else {
            RigidBodyTrait::dynamic(at(ZERO, HALF))
        };
        if kind == 'cannot_sleep' {
            sleeper.activation = RigidBodyActivationTrait::cannot_sleep();
        }
        let (handle, _) = world.insert(sleeper, ColliderBuilderTrait::cuboid(HALF, HALF).build());
        let _ = run(ref world, 60);
        let rb = body(ref world, handle);
        assert_eq!(rb.is_sleeping(), sleeps, "{}", kind);
        if sleeps {
            assert_eq!(rb.linvel(), v(ZERO, ZERO), "{}: velocities zeroed", kind);
            assert!(rb.activation.is_eligible_for_sleep(), "{}: timer pinned", kind);
        }
    }
}

/// A tenth of a second: `time_until_sleep` of the stacks below (6 steps instead of 30, VM step
/// budget of the tests).
const SHORT_SLEEP: Fixed = Fixed { raw: 429496730 };

/// A sleeping stack keeps its poses, velocities and contact pairs step after step, emits no
/// event, and the whole island wakes up (strongly) when one of its bodies is woken up by hand
/// through `set_body`: a force, an impulse, a velocity, a pose, `wake_up`. Rows: the wake-up.
fn sleeping_stack_wakes_as_an_island(hows: Span<felt252>) {
    for how in hows {
        let mut world = WorldTrait::new(gravity(), Default::default());
        let ground = world
            .insert_collider(
                ColliderBuilderTrait::halfspace(v(ZERO, ONE))
                    .active_events(COLLISION_EVENTS)
                    .build(),
                None,
            );
        let mut lower = RigidBodyTrait::dynamic(at(ZERO, HALF));
        lower.activation.time_until_sleep = SHORT_SLEEP;
        let mut upper = RigidBodyTrait::dynamic(at(ZERO, ONE + HALF));
        upper.activation.time_until_sleep = SHORT_SLEEP;
        let (bottom, bottom_collider) = world
            .insert(lower, ColliderBuilderTrait::cuboid(HALF, HALF).build());
        let (top, _) = world.insert(upper, ColliderBuilderTrait::cuboid(HALF, HALF).build());
        let events = run(ref world, 20);
        assert_eq!(events.len(), 1, "one Started");
        let (b0, t0) = (body(ref world, bottom), body(ref world, top));
        assert!(b0.is_sleeping() && t0.is_sleeping(), "{}: the stack sleeps", *how);
        let events = run(ref world, 3);
        assert_eq!(events.len(), 0);
        let (b1, t1) = (body(ref world, bottom), body(ref world, top));
        assert!(b1.position() == b0.position() && t1.position() == t0.position());
        assert!(b1.linvel() == v(ZERO, ZERO) && t1.vels.angvel == ZERO);
        assert!(world.contact_pair(ground, bottom_collider).unwrap().has_any_active_contact());
        // Wake the top box by hand: the bottom one follows at the next step.
        let mut rb = body(ref world, top);
        if *how == 'force' {
            rb.add_force(v(ZERO, ONE), true);
        } else if *how == 'impulse' {
            rb.apply_impulse(v(HALF, ZERO), true);
        } else if *how == 'linvel' {
            rb.set_linvel(v(ONE, ZERO));
        } else if *how == 'position' {
            rb.set_position(at(HALF, ONE + HALF));
        } else {
            rb.wake_up(false);
        }
        assert!(world.set_body(top, rb));
        let _ = world.step();
        let (b2, t2) = (body(ref world, bottom), body(ref world, top));
        assert!(!b2.is_sleeping() && !t2.is_sleeping(), "{}: island awake", *how);
        assert!(!b2.activation.is_eligible_for_sleep(), "{}: strong wake-up", *how);
        assert!(!t2.activation.is_eligible_for_sleep(), "{}: strong wake-up of the target", *how);
        if *how != 'wake_up' {
            assert!(t2.position() != t1.position() || t2.linvel() != v(ZERO, ZERO), "{}", *how);
        }
    }
}

#[test]
fn test_sleeping_stack_wakes_on_forces_and_impulses() {
    sleeping_stack_wakes_as_an_island(array!['force', 'impulse'].span());
}

#[test]
fn test_sleeping_stack_wakes_on_setters() {
    sleeping_stack_wakes_as_an_island(array!['linvel', 'position', 'wake_up'].span());
}

/// Two balls resting 3 apart, linked by a revolute joint: they fall asleep at the same step (one
/// island); waking one wakes the other. Removing the ground collider under a sleeping body wakes
/// it (and its island), and the step ends the pair with a `Stopped` event.
#[test]
fn test_joint_linked_islands_and_removal_wake_ups() {
    let mut world = WorldTrait::new(gravity(), Default::default());
    let ground = world
        .insert_collider(
            ColliderBuilderTrait::halfspace(v(ZERO, ONE)).active_events(COLLISION_EVENTS).build(),
            None,
        );
    let three = FixedTrait::from_int(3);
    let mut left = RigidBodyTrait::dynamic(at(ZERO, HALF));
    left.activation.time_until_sleep = SHORT_SLEEP;
    let mut right = RigidBodyTrait::dynamic(at(three, HALF));
    right.activation.time_until_sleep = SHORT_SLEEP;
    let (a, _) = world.insert(left, ColliderBuilderTrait::ball(HALF).build());
    let (b, b_collider) = world.insert(right, ColliderBuilderTrait::ball(HALF).build());
    let joint = RevoluteJointBuilderTrait::new()
        .local_anchor1(v(ONE + HALF, ZERO))
        .local_anchor2(v(-(ONE + HALF), ZERO))
        .build();
    let _ = world.insert_impulse_joint(a, b, joint);
    let mut step: u32 = 0;
    let mut slept_at: u32 = 0;
    while step != 25 {
        let _ = world.step();
        step += 1;
        let (ra, rb) = (body(ref world, a), body(ref world, b));
        assert_eq!(ra.is_sleeping(), rb.is_sleeping(), "step {}: one island", step);
        if ra.is_sleeping() && slept_at == 0 {
            slept_at = step;
        }
    }
    assert!(slept_at != 0 && slept_at <= 20, "asleep at {}", slept_at);
    let mut rb = body(ref world, b);
    rb.wake_up(true);
    assert!(world.set_body(b, rb));
    let _ = world.step();
    assert!(!body(ref world, a).is_sleeping(), "joint partner woken");
    let _ = run(ref world, 15);
    assert!(body(ref world, a).is_sleeping() && body(ref world, b).is_sleeping());
    // The ground goes: both bodies wake up and fall; the pairs end.
    assert!(world.remove_collider(ground).is_some());
    let events = world.step();
    assert!(!body(ref world, a).is_sleeping() && !body(ref world, b).is_sleeping());
    assert_eq!(events.len(), 2);
    assert!(world.contact_pair(ground, b_collider).is_none());
    let _ = run(ref world, 3);
    assert!(body(ref world, b).position().translation.y < HALF, "falling");
}

/// A body woken up by a contact that starts: a ball dropped on a sleeping ball wakes it (strong)
/// at the step the pair starts touching, and the sleeping ball's carried-over ground pair is
/// solved in that same step (the ball is pushed into the ground and back, not through it).
#[test]
fn test_contact_start_wakes_a_sleeping_body() {
    let mut world = WorldTrait::new(gravity(), Default::default());
    let _ = world.insert_collider(ColliderBuilderTrait::halfspace(v(ZERO, ONE)).build(), None);
    let mut resting = RigidBodyTrait::dynamic(at(ZERO, HALF));
    resting.activation.time_until_sleep = SHORT_SLEEP;
    let (low, _) = world.insert(resting, ColliderBuilderTrait::ball(HALF).build());
    let _ = run(ref world, 15);
    assert!(body(ref world, low).is_sleeping());
    let (high, _) = world
        .insert(
            RigidBodyTrait::dynamic(at(ZERO, ONE + HALF + HALF)),
            ColliderBuilderTrait::ball(HALF).build(),
        );
    let mut step: u32 = 0;
    let mut woken_at: u32 = 0;
    while step != 30 && woken_at == 0 {
        let _ = world.step();
        step += 1;
        if !body(ref world, low).is_sleeping() {
            woken_at = step;
        }
    }
    assert!(woken_at != 0, "woken");
    let rb = body(ref world, low);
    // Strong wake-up in the step (timer reset), then that step's own update (+dt at most).
    assert!(rb.activation.time_since_can_sleep <= world.integration_parameters.dt);
    assert!(rb.position().translation.y > ZERO, "kept above the ground");
    let _ = run(ref world, 10);
    assert!(
        body(ref world, high)
            .position()
            .translation
            .y > body(ref world, low)
            .position()
            .translation
            .y,
    );
}

// ---------------------------------------------------------------------------------------------
// Gas probes: `gas_step_<scene>` − `gas_setup_<scene>` = one `World::step` in that state.

/// Builds the world of probe `id` and steps it `warmup` times, then `measured` more times.
#[inline(never)]
fn probe(id: felt252, warmup: u32, measured: u32) {
    let mut world = if id == 'free_fall' {
        let (world, _, _, _) = ball_on_ground(FixedTrait::from_int(100));
        world
    } else if id == 'resting' {
        let (world, _, _, _) = ball_on_ground(HALF);
        world
    } else if id == 'stack3' {
        let (world, _) = scene_world(scenes::BOX_STACK3);
        world
    } else {
        let (world, _) = scene_world(scenes::PENDULUM);
        world
    };
    let _ = run(ref world, warmup + measured);
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

/// Ball far above the half-space (no broad-phase pair).
#[test]
fn gas_setup_free_fall() {
    probe(opaque('free_fall'), opaque(2), 0);
}

#[test]
fn gas_step_free_fall() {
    probe(opaque('free_fall'), opaque(2), 1);
}

/// Ball resting on the half-space, warm-started.
#[test]
fn gas_setup_resting() {
    probe(opaque('resting'), opaque(3), 0);
}

#[test]
fn gas_step_resting() {
    probe(opaque('resting'), opaque(3), 1);
}

/// `BOX_STACK3` after 60 steps (settled: ground–box, box–box, box–box touching pairs).
#[test]
fn gas_setup_stack3() {
    probe(opaque('stack3'), opaque(60), 0);
}

#[test]
fn gas_step_stack3() {
    probe(opaque('stack3'), opaque(60), 1);
}

/// `PENDULUM` after 3 steps (one revolute joint, no contact).
#[test]
fn gas_setup_pendulum() {
    probe(opaque('pendulum'), opaque(3), 0);
}

#[test]
fn gas_step_pendulum() {
    probe(opaque('pendulum'), opaque(3), 1);
}
