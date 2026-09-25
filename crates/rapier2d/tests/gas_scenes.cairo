//! `gas_scene_*` per-step budgets in Sierra gas and Cairo steps (work package P3).
//!
//! Each scene has a matching `gas_setup_*` and `gas_step_*`; subtracting them gives one
//! settled `World::step` at the same warm-started state. `gas_step_*` carries the Sierra-gas
//! ceiling (measured + 10 %); its uncapped `steps_step_*` twin runs the same probe so that
//! `--tracked-resource cairo-steps` measures every scene (the ceiling is in Sierra gas and fails
//! a cairo-steps run).

use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier2d::world::{World, WorldTrait};
use rapier_dynamics2d::collider::{ColliderBuilder, ColliderBuilderTrait};
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::joint::{
    ImpulseJointSetTrait, RevoluteJointBuilderTrait, RopeJointBuilderTrait, SpringJointBuilderTrait,
};
use rapier_dynamics2d::rigid_body_set::{RigidBodySetTrait, RigidBodyTrait};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

const WARMUP_CONTACTS: u32 = 1;
const WARMUP_FREE_FALL: u32 = 2;
const WARMUP_PENDULUM: u32 = 3;
const GRAVITY_Y: i64 = -42133629174;

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: v(x, y), rotation: Rot2 { re: ONE, im: ZERO } }
}

fn i(n: u32) -> Fixed {
    FixedTrait::from_int(n.try_into().unwrap())
}

fn gravity() -> Vec2 {
    v(ZERO, f(GRAVITY_Y))
}

fn add_ground(ref world: World) {
    let _ = world.insert_collider(ColliderBuilderTrait::halfspace(v(ZERO, ONE)).build(), None);
}

fn body_shape(kind: u32) -> ColliderBuilder {
    if kind == 0 {
        ColliderBuilderTrait::ball(HALF)
    } else if kind == 1 {
        ColliderBuilderTrait::cuboid(HALF, HALF)
    } else {
        ColliderBuilderTrait::capsule_x(HALF, HALF)
    }
}

fn free_fall(n: u32) -> World {
    let mut world = WorldTrait::new(gravity(), Default::default());
    let mut k = 0;
    while k != n {
        let x = i(k) * FixedTrait::from_int(4);
        let _ = world.insert(RigidBodyTrait::dynamic(at(x, i(100))), body_shape(0).build());
        k += 1;
    }
    world
}

fn balls_on_halfspace(n: u32) -> World {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    add_ground(ref world);
    let mut k = 0;
    while k != n {
        let x = i(k) * FixedTrait::from_int(2);
        let _ = world.insert(RigidBodyTrait::dynamic(at(x, HALF)), body_shape(0).build());
        k += 1;
    }
    world
}

fn cuboid_stack(n: u32) -> World {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    add_ground(ref world);
    let mut k = 0;
    while k != n {
        let _ = world.insert(RigidBodyTrait::dynamic(at(ZERO, HALF + i(k))), body_shape(1).build());
        k += 1;
    }
    world
}

fn mixed_pile() -> World {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    add_ground(ref world);
    for (x, kind) in array![
        (ZERO, 0_u32), (ONE, 1_u32), (FixedTrait::from_int(2) + HALF, 2_u32),
        (FixedTrait::from_int(4), 0_u32), (FixedTrait::from_int(5), 1_u32),
        (FixedTrait::from_int(6) + HALF, 2_u32), (FixedTrait::from_int(8), 0_u32),
        (FixedTrait::from_int(9), 1_u32),
    ]
        .span() {
        let _ = world.insert(RigidBodyTrait::dynamic(at(*x, HALF)), body_shape(*kind).build());
    }
    world
}

fn pendulum_chain(joints: u32) -> World {
    let mut world = WorldTrait::new(gravity(), Default::default());
    let pivot = world.insert_body(RigidBodyTrait::fixed(at(ZERO, ZERO)));
    let mut parent = pivot;
    let mut k = 0;
    while k != joints {
        let (body, _) = world
            .insert(
                RigidBodyTrait::dynamic(at(i(k + 1) * FixedTrait::from_int(2), ZERO)),
                ColliderBuilderTrait::ball(HALF).build(),
            );
        let joint = RevoluteJointBuilderTrait::new()
            .local_anchor1(v(ONE, ZERO))
            .local_anchor2(v(-ONE, ZERO))
            .build();
        let _ = world.insert_impulse_joint(parent, body, joint);
        parent = body;
        k += 1;
    }
    world
}

/// One pendulum link of `pendulum_chain` with the golden `pendulum_limited` range [-1/2, 1/2].
fn pendulum_limited() -> World {
    let mut world = WorldTrait::new(gravity(), Default::default());
    let pivot = world.insert_body(RigidBodyTrait::fixed(at(ZERO, ZERO)));
    let (body, _) = world
        .insert(
            RigidBodyTrait::dynamic(at(FixedTrait::from_int(2), ZERO)),
            ColliderBuilderTrait::ball(HALF).build(),
        );
    let joint = RevoluteJointBuilderTrait::new()
        .local_anchor1(v(ONE, ZERO))
        .local_anchor2(v(-ONE, ZERO))
        .limits([-HALF, HALF])
        .build();
    let _ = world.insert_impulse_joint(pivot, body, joint);
    world
}

/// A ball pinned to a fixed pivot by the golden `wheel_motor` velocity motor (4 rad/s, factor
/// 10, max force 2).
fn wheel_motor() -> World {
    let mut world = WorldTrait::new(gravity(), Default::default());
    let pivot = world.insert_body(RigidBodyTrait::fixed(at(ZERO, ZERO)));
    let (body, _) = world
        .insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)), ColliderBuilderTrait::ball(HALF).build());
    let joint = RevoluteJointBuilderTrait::new()
        .motor_velocity(FixedTrait::from_int(4), FixedTrait::from_int(10))
        .motor_max_force(FixedTrait::from_int(2))
        .build();
    let _ = world.insert_impulse_joint(pivot, body, joint);
    world
}

/// RJ: a ball on a rope of length 2 from a fixed pivot, released taut and horizontal (it stays
/// taut while swinging).
fn rope() -> World {
    let mut world = WorldTrait::new(gravity(), Default::default());
    let pivot = world.insert_body(RigidBodyTrait::fixed(at(ZERO, ZERO)));
    let (body, _) = world
        .insert(
            RigidBodyTrait::dynamic(at(FixedTrait::from_int(2), ZERO)),
            ColliderBuilderTrait::ball(HALF).build(),
        );
    let joint = RopeJointBuilderTrait::new(FixedTrait::from_int(2)).build();
    let _ = world.insert_impulse_joint(pivot, body, joint);
    world
}

/// RJ: a ball hanging 2 below a fixed pivot on a force-based spring (rest 1, stiffness 20,
/// damping 1/2), released stretched.
fn spring() -> World {
    let mut world = WorldTrait::new(gravity(), Default::default());
    let pivot = world.insert_body(RigidBodyTrait::fixed(at(ZERO, ZERO)));
    let (body, _) = world
        .insert(
            RigidBodyTrait::dynamic(at(ZERO, FixedTrait::from_int(-2))),
            ColliderBuilderTrait::ball(HALF).build(),
        );
    let joint = SpringJointBuilderTrait::new(ONE, FixedTrait::from_int(20), HALF).build();
    let _ = world.insert_impulse_joint(pivot, body, joint);
    world
}

fn scene(id: felt252, size: u32) -> World {
    if id == 'free' {
        free_fall(size)
    } else if id == 'balls' {
        balls_on_halfspace(size)
    } else if id == 'stack' {
        cuboid_stack(size)
    } else if id == 'mixed' {
        mixed_pile()
    } else if id == 'plim' {
        pendulum_limited()
    } else if id == 'wheel' {
        wheel_motor()
    } else if id == 'rope' {
        rope()
    } else if id == 'spring' {
        spring()
    } else {
        pendulum_chain(size)
    }
}

fn expected_pairs(id: felt252, size: u32) -> u32 {
    if id == 'free'
        || id == 'pend'
        || id == 'plim'
        || id == 'wheel'
        || id == 'rope'
        || id == 'spring' {
        0
    } else if id == 'mixed' {
        15
    } else {
        size
    }
}

fn expected_joints(id: felt252, size: u32) -> u32 {
    if id == 'pend' || id == 'plim' || id == 'wheel' || id == 'rope' || id == 'spring' {
        size
    } else {
        0
    }
}

fn count_active_pairs(world: @World) -> (u32, u32) {
    let mut pairs = world.narrow_phase.pairs.span();
    let mut active = 0;
    let mut points = 0;
    while let Some(pair) = pairs.pop_front() {
        let n = *pair.manifold.data.num_solver_contacts;
        if n != 0 {
            active += 1;
            points += n.into();
        }
    }
    (active, points)
}

fn run(ref world: World, steps: u32) {
    let mut k = 0;
    while k != steps {
        let _ = world.step();
        k += 1;
    }
}

#[inline(never)]
fn probe(id: felt252, size: u32, warmup: u32, measured: u32) {
    let mut world = scene(id, size);
    run(ref world, warmup);
    let (pairs, points) = count_active_pairs(@world);
    assert_eq!(pairs, expected_pairs(id, size));
    assert!(points >= pairs);
    assert_eq!(world.impulse_joints.len(), expected_joints(id, size));
    run(ref world, measured);
    let (pairs, points) = count_active_pairs(@world);
    assert_eq!(pairs, expected_pairs(id, size));
    assert!(points >= pairs);
    assert_eq!(
        world.bodies.len(),
        if id == 'pend' || id == 'plim' || id == 'wheel' || id == 'rope' || id == 'spring' {
            size + 1
        } else {
            size
        },
    );
    let _ = opaque(world.colliders.len());
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_setup_free_fall1() {
    probe(opaque('free'), opaque(1), WARMUP_FREE_FALL, 0);
}

#[test]
#[available_gas(l2_gas: 2851681)]
fn gas_step_free_fall1() {
    probe(opaque('free'), opaque(1), WARMUP_FREE_FALL, 1);
}

#[test]
fn gas_setup_free_fall8() {
    probe(opaque('free'), opaque(8), WARMUP_FREE_FALL, 0);
}

#[test]
#[available_gas(l2_gas: 17276353)]
fn gas_step_free_fall8() {
    probe(opaque('free'), opaque(8), WARMUP_FREE_FALL, 1);
}

#[test]
fn gas_setup_free_fall32() {
    probe(opaque('free'), opaque(32), WARMUP_FREE_FALL, 0);
}

#[test]
#[available_gas(l2_gas: 67390649)]
fn gas_step_free_fall32() {
    probe(opaque('free'), opaque(32), WARMUP_FREE_FALL, 1);
}

#[test]
fn gas_setup_balls_halfspace1() {
    probe(opaque('balls'), opaque(1), WARMUP_CONTACTS, 0);
}

#[test]
#[available_gas(l2_gas: 9430516)]
fn gas_step_balls_halfspace1() {
    probe(opaque('balls'), opaque(1), WARMUP_CONTACTS, 1);
}

#[test]
fn gas_setup_balls_halfspace8() {
    probe(opaque('balls'), opaque(8), WARMUP_CONTACTS, 0);
}

#[test]
#[available_gas(l2_gas: 63878916)]
fn gas_step_balls_halfspace8() {
    probe(opaque('balls'), opaque(8), WARMUP_CONTACTS, 1);
}

#[test]
fn gas_setup_balls_halfspace32() {
    probe(opaque('balls'), opaque(32), WARMUP_CONTACTS, 0);
}

#[test]
#[available_gas(l2_gas: 254543397)]
fn gas_step_balls_halfspace32() {
    probe(opaque('balls'), opaque(32), WARMUP_CONTACTS, 1);
}

#[test]
fn gas_setup_cuboid_stack1() {
    probe(opaque('stack'), opaque(1), WARMUP_CONTACTS, 0);
}

#[test]
#[available_gas(l2_gas: 9973715)]
fn gas_step_cuboid_stack1() {
    probe(opaque('stack'), opaque(1), WARMUP_CONTACTS, 1);
}

#[test]
fn gas_setup_cuboid_stack3() {
    probe(opaque('stack'), opaque(3), WARMUP_CONTACTS, 0);
}

#[test]
#[available_gas(l2_gas: 28923788)]
fn gas_step_cuboid_stack3() {
    probe(opaque('stack'), opaque(3), WARMUP_CONTACTS, 1);
}

#[test]
fn gas_setup_cuboid_stack5() {
    probe(opaque('stack'), opaque(5), WARMUP_CONTACTS, 0);
}

#[test]
#[available_gas(l2_gas: 47959925)]
fn gas_step_cuboid_stack5() {
    probe(opaque('stack'), opaque(5), WARMUP_CONTACTS, 1);
}

#[test]
fn gas_setup_cuboid_stack10() {
    probe(opaque('stack'), opaque(10), WARMUP_CONTACTS, 0);
}

#[test]
#[available_gas(l2_gas: 95926795)]
fn gas_step_cuboid_stack10() {
    probe(opaque('stack'), opaque(10), WARMUP_CONTACTS, 1);
}

#[test]
fn gas_setup_mixed_pile8() {
    probe(opaque('mixed'), opaque(8), WARMUP_CONTACTS, 0);
}

#[test]
#[available_gas(l2_gas: 108595478)]
fn gas_step_mixed_pile8() {
    probe(opaque('mixed'), opaque(8), WARMUP_CONTACTS, 1);
}

#[test]
fn gas_setup_pendulum_chain1() {
    probe(opaque('pend'), opaque(1), WARMUP_PENDULUM, 0);
}

#[test]
#[available_gas(l2_gas: 17225559)]
fn gas_step_pendulum_chain1() {
    probe(opaque('pend'), opaque(1), WARMUP_PENDULUM, 1);
}

#[test]
fn gas_setup_pendulum_chain3() {
    probe(opaque('pend'), opaque(3), WARMUP_PENDULUM, 0);
}

#[test]
#[available_gas(l2_gas: 46138701)]
fn gas_step_pendulum_chain3() {
    probe(opaque('pend'), opaque(3), WARMUP_PENDULUM, 1);
}

#[test]
fn gas_setup_pendulum_limited() {
    probe(opaque('plim'), opaque(1), WARMUP_PENDULUM, 0);
}

#[test]
#[available_gas(l2_gas: 18826723)]
fn gas_step_pendulum_limited() {
    probe(opaque('plim'), opaque(1), WARMUP_PENDULUM, 1);
}

#[test]
fn gas_setup_wheel_motor() {
    probe(opaque('wheel'), opaque(1), WARMUP_PENDULUM, 0);
}

#[test]
#[available_gas(l2_gas: 21444085)]
fn gas_step_wheel_motor() {
    probe(opaque('wheel'), opaque(1), WARMUP_PENDULUM, 1);
}

#[test]
fn gas_setup_rope() {
    probe(opaque('rope'), opaque(1), WARMUP_PENDULUM, 0);
}

#[test]
#[available_gas(l2_gas: 19324219)]
fn gas_step_rope() {
    probe(opaque('rope'), opaque(1), WARMUP_PENDULUM, 1);
}

#[test]
fn gas_setup_spring() {
    probe(opaque('spring'), opaque(1), WARMUP_PENDULUM, 0);
}

#[test]
#[available_gas(l2_gas: 19757399)]
fn gas_step_spring() {
    probe(opaque('spring'), opaque(1), WARMUP_PENDULUM, 1);
}

// Uncapped twins of the `gas_step_*` probes, for `--tracked-resource cairo-steps`.

#[test]
fn steps_step_free_fall1() {
    probe(opaque('free'), opaque(1), WARMUP_FREE_FALL, 1);
}

#[test]
fn steps_step_free_fall8() {
    probe(opaque('free'), opaque(8), WARMUP_FREE_FALL, 1);
}

#[test]
fn steps_step_free_fall32() {
    probe(opaque('free'), opaque(32), WARMUP_FREE_FALL, 1);
}

#[test]
fn steps_step_balls_halfspace1() {
    probe(opaque('balls'), opaque(1), WARMUP_CONTACTS, 1);
}

#[test]
fn steps_step_balls_halfspace8() {
    probe(opaque('balls'), opaque(8), WARMUP_CONTACTS, 1);
}

#[test]
fn steps_step_balls_halfspace32() {
    probe(opaque('balls'), opaque(32), WARMUP_CONTACTS, 1);
}

#[test]
fn steps_step_cuboid_stack1() {
    probe(opaque('stack'), opaque(1), WARMUP_CONTACTS, 1);
}

#[test]
fn steps_step_cuboid_stack3() {
    probe(opaque('stack'), opaque(3), WARMUP_CONTACTS, 1);
}

#[test]
fn steps_step_cuboid_stack5() {
    probe(opaque('stack'), opaque(5), WARMUP_CONTACTS, 1);
}

#[test]
fn steps_step_cuboid_stack10() {
    probe(opaque('stack'), opaque(10), WARMUP_CONTACTS, 1);
}

#[test]
fn steps_step_mixed_pile8() {
    probe(opaque('mixed'), opaque(8), WARMUP_CONTACTS, 1);
}

#[test]
fn steps_step_pendulum_chain1() {
    probe(opaque('pend'), opaque(1), WARMUP_PENDULUM, 1);
}

#[test]
fn steps_step_pendulum_chain3() {
    probe(opaque('pend'), opaque(3), WARMUP_PENDULUM, 1);
}

#[test]
fn steps_step_pendulum_limited() {
    probe(opaque('plim'), opaque(1), WARMUP_PENDULUM, 1);
}

#[test]
fn steps_step_wheel_motor() {
    probe(opaque('wheel'), opaque(1), WARMUP_PENDULUM, 1);
}

#[test]
fn steps_step_rope() {
    probe(opaque('rope'), opaque(1), WARMUP_PENDULUM, 1);
}

#[test]
fn steps_step_spring() {
    probe(opaque('spring'), opaque(1), WARMUP_PENDULUM, 1);
}
