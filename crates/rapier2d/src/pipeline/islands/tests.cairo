//! Unit tests and gas probes of the island stage, the sleep timer and the extent helpers.

use fixed::{Fixed, FixedTrait, HALF, MAX, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::joint::{ImpulseJointSetTrait, RevoluteJointBuilderTrait};
use rapier_dynamics2d::rigid_body_set::{RigidBodySetTrait, RigidBodyTrait};
use rapier_geometry2d::shape::{
    BallTrait, CapsuleTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait, Shape,
};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::world::{World, WorldTrait};
use super::super::fixtures::{at, p3_scene, v};
use super::{
    SleepCensusTrait, local_bounding_sphere, max_extent, relative_pose_drift, update_islands,
};

/// A quarter turn, exactly.
fn quarter() -> Rot2 {
    Rot2 { re: ZERO, im: ONE }
}

#[test]
fn test_local_bounding_spheres_follow_parry() {
    let cases: Array<(Shape, Vec2, Fixed)> = array![
        (Shape::Ball(BallTrait::new(HALF)), v(ZERO, ZERO), HALF),
        // |(3, 4)| = 5.
        (
            Shape::Cuboid(CuboidTrait::new(v(FixedTrait::from_int(3), FixedTrait::from_int(4)))),
            v(ZERO, ZERO),
            FixedTrait::from_int(5),
        ),
        // Segment (-1, 0)–(1, 0): midpoint 0, half length 1, plus the radius for the capsule.
        (
            Shape::Capsule(CapsuleTrait::new(v(-ONE, ZERO), v(ONE, ZERO), HALF)),
            v(ZERO, ZERO),
            ONE + HALF,
        ),
        (Shape::Segment(SegmentTrait::new(v(ZERO, ZERO), v(TWO, ZERO))), v(ONE, ZERO), ONE),
        (Shape::HalfSpace(HalfSpaceTrait::new(v(ZERO, ONE))), v(ZERO, ZERO), MAX),
    ];
    for (shape, center, radius) in cases {
        assert_eq!(local_bounding_sphere(shape), (center, radius));
    }
}

/// `max_extent`: the farthest sphere point from the local centre of mass; a half-space makes it
/// `MAX`; no collider gives `0`.
#[test]
fn test_max_extent_is_the_farthest_sphere_point() {
    let ball = Shape::Ball(BallTrait::new(HALF));
    let two_away = Pose2 { translation: v(TWO, ZERO), rotation: Rot2 { re: ONE, im: ZERO } };
    let origin = Pose2 { translation: v(ZERO, ZERO), rotation: Rot2 { re: ONE, im: ZERO } };
    assert_eq!(max_extent(v(ZERO, ZERO), array![].span()), ZERO);
    assert_eq!(max_extent(v(ZERO, ZERO), array![(ball, origin)].span()), HALF);
    // Ball centred 2 away from a centre of mass at x = 0.5: 1.5 + 0.5.
    assert_eq!(max_extent(v(HALF, ZERO), array![(ball, two_away), (ball, origin)].span()), TWO);
    let half = Shape::HalfSpace(HalfSpaceTrait::new(v(ZERO, ONE)));
    assert_eq!(max_extent(v(ZERO, ZERO), array![(ball, origin), (half, origin)].span()), MAX);
}

/// `relative_pose_drift`: translation only, a quarter turn (chord `2 sin(45°) r = sqrt(2) r`,
/// floor), a half turn (chord `2 r`), an infinite extent.
#[test]
fn test_relative_pose_drift() {
    let identity = Rot2 { re: ONE, im: ZERO };
    let base = Pose2 { translation: v(ZERO, ZERO), rotation: identity };
    let moved = Pose2 {
        translation: v(FixedTrait::from_int(3), FixedTrait::from_int(4)), rotation: identity,
    };
    assert_eq!(relative_pose_drift(base, moved, TWO), FixedTrait::from_int(5));
    let turned = Pose2 { translation: v(ZERO, ZERO), rotation: quarter() };
    // sqrt(2) = 1.4142135623… → raw 6074000999 (floor); `2 · |1| / sqrt(2) · 1`.
    let chord = relative_pose_drift(base, turned, ONE);
    assert!(chord.raw >= 6074000990 && chord.raw <= 6074001010, "quarter turn chord {:?}", chord);
    let flipped = Pose2 { translation: v(ZERO, ZERO), rotation: Rot2 { re: -ONE, im: ZERO } };
    assert_eq!(relative_pose_drift(base, flipped, HALF), ONE);
    assert_eq!(relative_pose_drift(base, moved, MAX), MAX);
}

/// The P3 cuboid stack of size `n` on a standalone half-space (no gravity: the boxes never
/// move), `time_until_sleep` cut to a tenth of a second (6 steps; VM step budget of the probes;
/// the cost of a sleeping step does not depend on it), stepped `steps` times.
fn stack(n: u32, steps: u32) -> World {
    let mut world = p3_scene('stack', n);
    for (handle, body) in world.bodies.iter() {
        let mut body = body;
        body.activation.time_until_sleep = Fixed { raw: 429496730 };
        assert!(world.bodies.set(handle, body));
    }
    let mut i = 0;
    while i != steps {
        let _ = world.step();
        i += 1;
    }
    world
}

fn sleeping_count(ref world: World) -> u32 {
    let mut count = 0;
    for (_, body) in world.bodies.iter() {
        if body.activation.sleeping {
            count += 1;
        }
    }
    count
}

/// `update_islands` on the sets directly: with no eligible body nothing changes; with every
/// member eligible the whole touching island sleeps (velocities zeroed); a sleeping body linked
/// by a joint to an awake one wakes up strongly; a sleeping stack with one member woken by hand
/// wakes entirely.
#[test]
fn test_update_islands_rules() {
    let mut world = stack(3, 1);
    let entries = world.bodies.iter();
    let (out, sleeping, woken) = update_islands(
        ref world.bodies,
        world.narrow_phase.pairs.span(),
        array![].span(),
        world.impulse_joints.to_array().span(),
        entries.span(),
        SleepCensusTrait::taken(entries.span()),
    );
    assert!(out == entries.span() && !sleeping && !woken, "nothing eligible");
    // Make every box eligible: the island sleeps at once.
    let mut bodies = world.bodies.iter();
    for (handle, body) in bodies.span() {
        let mut body = *body;
        body.activation.time_since_can_sleep = body.activation.time_until_sleep;
        body.vels.linvel = v(ONE, ZERO);
        assert!(world.bodies.set(*handle, body));
    }
    bodies = world.bodies.iter();
    let (out, sleeping, woken) = update_islands(
        ref world.bodies,
        world.narrow_phase.pairs.span(),
        array![].span(),
        world.impulse_joints.to_array().span(),
        bodies.span(),
        SleepCensusTrait::taken(bodies.span()),
    );
    assert!(sleeping && !woken);
    assert_eq!(sleeping_count(ref world), 3);
    for (_, body) in out {
        assert!(*body.activation.sleeping && body.vels.linvel == @v(ZERO, ZERO));
    }
    // Wake the middle box by hand: the whole island wakes (strong: timers reset).
    let (mid, mut body) = *world.bodies.iter().at(1);
    body.wake_up(false);
    assert!(world.bodies.set(mid, body));
    let entries = world.bodies.iter();
    let (out, sleeping, woken) = update_islands(
        ref world.bodies,
        world.narrow_phase.pairs.span(),
        array![].span(),
        world.impulse_joints.to_array().span(),
        entries.span(),
        SleepCensusTrait::taken(entries.span()),
    );
    assert!(!sleeping && woken);
    assert_eq!(sleeping_count(ref world), 0);
    for (_, body) in out {
        assert_eq!(*body.activation.time_since_can_sleep, ZERO);
    }
}

/// Two balls linked by a revolute joint, no contact: the sleeping one wakes up because the other
/// is awake; a disabled joint links nothing; a fixed pivot links nothing either.
#[test]
fn test_joints_link_islands() {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    let (a, _) = world
        .insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)), ColliderBuilderTrait::ball(HALF).build());
    let (b, _) = world
        .insert(
            RigidBodyTrait::dynamic(at(FixedTrait::from_int(5), ZERO)),
            ColliderBuilderTrait::ball(HALF).build(),
        );
    let pivot = world.insert_body(RigidBodyTrait::fixed(at(ZERO, FixedTrait::from_int(5))));
    let mut asleep = world.body(b).unwrap();
    asleep.sleep();
    assert!(world.set_body(b, asleep));
    let mut bodies = world.bodies.iter();
    let joints = world.impulse_joints.to_array();
    let (_, _, woken) = update_islands(
        ref world.bodies,
        array![].span(),
        array![].span(),
        joints.span(),
        bodies.span(),
        SleepCensusTrait::taken(bodies.span()),
    );
    assert!(!woken, "no link");
    let mut joint = RevoluteJointBuilderTrait::new().build();
    joint.enabled = rapier_dynamics2d::joint::JointEnabled::Disabled;
    let _ = world.impulse_joints.insert(a, b, joint);
    let _ = world.impulse_joints.insert(pivot, b, RevoluteJointBuilderTrait::new().build());
    bodies = world.bodies.iter();
    let joints = world.impulse_joints.to_array();
    let (_, _, woken) = update_islands(
        ref world.bodies,
        array![].span(),
        array![].span(),
        joints.span(),
        bodies.span(),
        SleepCensusTrait::taken(bodies.span()),
    );
    assert!(!woken, "disabled joint and fixed pivot link nothing");
    let _ = world.impulse_joints.insert(a, b, RevoluteJointBuilderTrait::new().build());
    bodies = world.bodies.iter();
    let joints = world.impulse_joints.to_array();
    let (_, sleeping, woken) = update_islands(
        ref world.bodies,
        array![].span(),
        array![].span(),
        joints.span(),
        bodies.span(),
        SleepCensusTrait::taken(bodies.span()),
    );
    assert!(woken && !sleeping);
    assert!(!world.body(b).unwrap().is_sleeping());
}

/// `gas_islands_<state>` − `gas_walks_<state>` is one `update_islands` call.
#[test]
fn gas_walks_stack3_awake() {
    probe(opaque(3), opaque(1), WALKS);
}

#[test]
fn gas_walks_stack3_eligible() {
    probe(opaque(3), opaque(7), WALKS);
}

#[test]
fn gas_walks_stack10_asleep() {
    probe(opaque(10), opaque(12), WALKS);
}

/// Both candidates give the same result on the three probe states.
#[test]
fn test_metered_candidate_agrees() {
    for (n, steps) in array![(3_u32, 1_u32), (3, 7), (10, 12)].span() {
        let mut a = stack(*n, *steps);
        let mut b = stack(*n, *steps);
        let ea = a.bodies.iter();
        let eb = b.bodies.iter();
        let ja = a.impulse_joints.to_array();
        let jb = b.impulse_joints.to_array();
        let (oa, sa, wa) = update_islands(
            ref a.bodies,
            a.narrow_phase.pairs.span(),
            array![].span(),
            ja.span(),
            ea.span(),
            SleepCensusTrait::taken(ea.span()),
        );
        let (ob, sb, wb) = super::alternatives::update_islands_metered(
            ref b.bodies,
            b.narrow_phase.pairs.span(),
            array![].span(),
            jb.span(),
            eb.span(),
            SleepCensusTrait::taken(eb.span()),
        );
        assert!(oa == ob && sa == sb && wa == wb);
        assert!(a.bodies.iter() == b.bodies.iter());
    }
}

/// Kinematic bodies: a moving one never becomes eligible and keeps its island awake; the
/// `Fixed` arm of the gate is never reached (fixed bodies are not members).
#[test]
fn test_moving_kinematic_body_never_sleeps() {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    let mut platform = RigidBodyTrait::new(RigidBodyType::KinematicVelocityBased, at(ZERO, ZERO));
    platform.vels.linvel = v(Fixed { raw: 42949673 }, ZERO);
    let (k, _) = world.insert(platform, ColliderBuilderTrait::cuboid(TWO, HALF).build());
    let mut i = 0;
    while i != 40 {
        let _ = world.step();
        i += 1;
    }
    let body = world.body(k).unwrap();
    assert!(!body.is_sleeping());
    assert_eq!(body.activation.time_since_can_sleep, ZERO);
}

// Gas probes: `gas_islands_<scene>` − `gas_walks_<scene>` is one `update_islands` call on the
// stepped world (`gas_setup_<scene>` is the world alone); `gas_step_<scene>` −
// `gas_setup_<scene>` is one whole `World::step` (the P3 `cuboid_stack10` awake step is in
// `tests/gas_scenes.cairo`).

/// Probe modes.
const SETUP: u8 = 0;
const ISLANDS: u8 = 1;
const STEP: u8 = 2;
const WAKE_SETUP: u8 = 3;
const WAKE_STEP: u8 = 4;
const METERED: u8 = 5;
/// The setup of the `ISLANDS` probes without the call (the body and joint walks).
const WALKS: u8 = 6;

/// `stack(n, steps)`, then per `mode`: nothing, `update_islands`, one step, the top box woken by
/// hand (`apply_impulse` through `set_body`), the same then one step.
#[inline(never)]
fn probe(n: u32, steps: u32, mode: u8) {
    let mut world = stack(n, steps);
    if mode == ISLANDS || mode == METERED || mode == WALKS {
        let entries = world.bodies.iter();
        let joints = world.impulse_joints.to_array();
        if mode == WALKS {
            let _ = opaque(entries.len() + joints.len());
            return;
        }
        let _ = if mode == ISLANDS {
            update_islands(
                ref world.bodies,
                world.narrow_phase.pairs.span(),
                array![].span(),
                joints.span(),
                entries.span(),
                SleepCensusTrait::taken(entries.span()),
            )
        } else {
            super::alternatives::update_islands_metered(
                ref world.bodies,
                world.narrow_phase.pairs.span(),
                array![].span(),
                joints.span(),
                entries.span(),
                SleepCensusTrait::taken(entries.span()),
            )
        };
    } else if mode == STEP {
        let _ = world.step();
    } else if mode >= WAKE_SETUP {
        let (top, mut body) = *world.bodies.iter().at(n - 1);
        body.apply_impulse(v(ONE, ZERO), true);
        assert!(world.set_body(top, body));
        if mode == WAKE_STEP {
            let _ = world.step();
        }
    }
}

/// Ten boxes asleep (after 12 steps): one fully sleeping step (target: ≤ 15 % of the awake
/// step of `gas_scenes::gas_step_cuboid_stack10`).
#[test]
fn gas_step_stack10_asleep() {
    probe(opaque(10), opaque(12), STEP);
}

/// Ten boxes asleep, the top one woken by hand: the wake-up step (island wake, revived pairs,
/// full solve).
#[test]
fn gas_setup_stack10_wake() {
    probe(opaque(10), opaque(12), WAKE_SETUP);
}

#[test]
fn gas_step_stack10_wake() {
    probe(opaque(10), opaque(12), WAKE_STEP);
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

/// `relative_pose_drift` of a translated body (no turn: one square root).
#[test]
fn gas_drift_translated() {
    let base = Pose2 { translation: v(ZERO, ZERO), rotation: Rot2 { re: ONE, im: ZERO } };
    let cur = Pose2 { translation: v(ONE, TWO), rotation: Rot2 { re: ONE, im: ZERO } };
    let _ = relative_pose_drift(opaque(base), cur, HALF);
}

/// `relative_pose_drift` of a turned body (two square roots, one division).
#[test]
fn gas_drift_turned() {
    let base = Pose2 { translation: v(ZERO, ZERO), rotation: Rot2 { re: ONE, im: ZERO } };
    let cur = Pose2 { translation: v(ONE, TWO), rotation: quarter() };
    let _ = relative_pose_drift(opaque(base), cur, HALF);
}

/// A falling ball (default thresholds, no turn) for the timer probes: `gas_update_sleep_timer`
/// − `gas_timer_setup` is one `update_sleep_timer`.
#[inline(never)]
fn falling_ball() -> rapier_dynamics2d::rigid_body_set::RigidBody {
    let mut body = RigidBodyTrait::dynamic(opaque(at(ZERO, ONE)));
    body.mprops.max_extent = HALF;
    body.vels.linvel = v(ZERO, -ONE);
    body
}

#[test]
fn gas_timer_setup() {
    let body = falling_ball();
    let _ = opaque(body.activation.time_since_can_sleep);
}

#[test]
fn gas_update_sleep_timer() {
    let mut body = falling_ball();
    let previous = at(ZERO, ONE + HALF);
    super::update_sleep_timer(ref body, previous, Default::default());
    let _ = opaque(body.activation.time_since_can_sleep);
}

/// The timer of a body that cannot sleep (negative threshold: no drift computed).
#[test]
fn gas_update_sleep_timer_cannot_sleep() {
    let mut body = falling_ball();
    body.activation = rapier_core::rigid_body::RigidBodyActivationTrait::cannot_sleep();
    let previous = at(ZERO, ONE + HALF);
    super::update_sleep_timer(ref body, previous, Default::default());
    let _ = opaque(body.activation.time_since_can_sleep);
}

/// Three awake boxes, no eligible body (the fast path: one walk over the bodies).
#[test]
fn gas_setup_stack3_awake() {
    probe(opaque(3), opaque(1), SETUP);
}

#[test]
fn gas_islands_stack3_awake() {
    probe(opaque(3), opaque(1), ISLANDS);
}

/// The metered candidate on the fast path …
#[test]
fn gas_islands_stack3_awake_metered() {
    probe(opaque(3), opaque(1), METERED);
}

/// … and on the slow path.
#[test]
fn gas_islands_stack3_eligible_metered() {
    probe(opaque(3), opaque(7), METERED);
}

/// Three boxes after 7 steps at rest: eligible, the union-find runs and the island sleeps.
#[test]
fn gas_setup_stack3_eligible() {
    probe(opaque(3), opaque(7), SETUP);
}

#[test]
fn gas_islands_stack3_eligible() {
    probe(opaque(3), opaque(7), ISLANDS);
}

/// Ten boxes asleep (after 12 steps): the fast path (no awake member).
#[test]
fn gas_setup_stack10_asleep() {
    probe(opaque(10), opaque(12), SETUP);
}

#[test]
fn gas_islands_stack10_asleep() {
    probe(opaque(10), opaque(12), ISLANDS);
}
