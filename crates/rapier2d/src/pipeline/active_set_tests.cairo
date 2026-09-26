//! Equivalence of the active-set step (BT2, `active_set`) against the whole step: the same
//! world stepped twice, once as shipped and once with its active set invalidated before every
//! step (the whole path, which never reads it), compared raw after every step: events, bodies,
//! colliders, pairs. The worlds sleep, wake through contacts and user changes, lose bodies and
//! colliders, have sensors, a disabled body and bodies inserted asleep.

use fixed::{Fixed, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::collider::events::{COLLISION_EVENTS, CONTACT_FORCE_EVENTS};
use rapier_dynamics2d::collider::{ColliderBuilderTrait, ColliderTrait};
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBodyBuilderTrait, RigidBodySetTrait, RigidBodyTrait};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use crate::world::{World, WorldTrait};
use super::active_set::usable;
use super::fixtures::draw;

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: v(x, y), rotation: Rot2 { re: ONE, im: ZERO } }
}

/// A small level: a fixed ground, `3 + seed % 4` unit blocks resting on it (side by side or
/// stacked by pairs) that fall asleep after two calm steps, a standalone sensor over the first
/// block, a disabled body far away, and a pebble fired at the blocks from the left (it arrives
/// after about ten steps; slower or higher shots miss, from `seed`).
fn level(seed: u32) -> World {
    let mut state: u64 = seed.into();
    let mut world: World = Default::default();
    let _ = world
        .insert(
            RigidBodyTrait::fixed(at(ZERO, f(-2147483648))),
            ColliderBuilderTrait::cuboid(f(85899345920), HALF).build(),
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
        let x = f(4294967296 * column.into());
        let y = HALF + f(4294967296 * row.into());
        let mut body = RigidBodyTrait::dynamic(at(x, y));
        body.activation.time_until_sleep = f(143165577);
        let _ = world
            .insert(
                body,
                ColliderBuilderTrait::cuboid(HALF, HALF).active_events(COLLISION_EVENTS).build(),
            );
        k += 1;
    }
    let _ = world
        .insert_collider(
            ColliderBuilderTrait::ball(HALF)
                .sensor(true)
                .active_events(COLLISION_EVENTS)
                .translation(v(ZERO, ONE + HALF))
                .build(),
            None,
        );
    let mut disabled = RigidBodyTrait::dynamic(at(f(-429496729600), f(42949672960)));
    disabled.enabled = false;
    let _ = world.insert(disabled, ColliderBuilderTrait::ball(HALF).build());
    let speed: i64 = (8 + draw(ref state) % 6).into();
    let height: i64 = (1 + draw(ref state) % 3).into();
    let pebble = RigidBodyBuilderTrait::dynamic()
        .translation(v(f(-42949672960), f(2147483648 * height)))
        .linvel(v(f(4294967296 * speed), f(2147483648)))
        .build();
    let pebble_collider = ColliderBuilderTrait::ball(f(1288490189))
        .active_events(COLLISION_EVENTS | CONTACT_FORCE_EVENTS)
        .build();
    let _ = world.insert(pebble, pebble_collider);
    world
}

/// A user change on both worlds at step `t` (from `seed`): wake a block up, remove a body,
/// insert a block asleep on the ground, disable a collider, move a sleeping block.
fn change(ref a: World, ref b: World, t: u32, seed: u32) {
    let roll = (seed + t * 7) % 23;
    let bodies = a.bodies.iter();
    let (block, _) = *bodies.at(1 + roll % 3);
    if roll == 1 || roll == 12 {
        a.wake_up(block);
        b.wake_up(block);
    } else if roll == 3 || roll == 15 {
        let _ = a.remove_body(block);
        let _ = b.remove_body(block);
    } else if roll == 5 || roll == 18 {
        let body = RigidBodyBuilderTrait::dynamic()
            .translation(v(f(-17179869184), HALF))
            .sleeping(true)
            .build();
        let _ = a.insert(body, ColliderBuilderTrait::cuboid(HALF, HALF).build());
        let _ = b.insert(body, ColliderBuilderTrait::cuboid(HALF, HALF).build());
    } else if roll == 7 {
        let (co, mut collider) = *a.colliders.iter().at(1);
        collider.set_enabled(false);
        assert!(a.set_collider(co, collider) && b.set_collider(co, collider));
    } else if roll == 9 {
        let mut body = a.body(block).unwrap();
        body.set_position(at(f(-8589934592), HALF));
        assert!(a.set_body(block, body) && b.set_body(block, body));
    }
}

/// Steps `shipped` (active set as maintained) and `reference` (invalidated before every step)
/// `steps` times with the user changes of `seed`; returns how many steps took the active set.
fn run_both(seed: u32, steps: u32, changes: bool) -> u32 {
    let mut shipped = level(seed);
    let mut reference = level(seed);
    let mut sparse = 0;
    let mut t: u32 = 0;
    while t != steps {
        if changes && t > 4 {
            change(ref shipped, ref reference, t, seed);
        }
        if usable(ref shipped) {
            sparse += 1;
        }
        super::active_set::invalidate(ref reference);
        let (expected, expected_forces) = reference.step_with_force_events();
        let (got, got_forces) = shipped.step_with_force_events();
        assert!(got == expected, "seed {} step {} events", seed, t);
        assert!(got_forces == expected_forces, "seed {} step {} force events", seed, t);
        assert!(
            shipped.bodies.iter() == reference.bodies.iter(), "seed {} step {} bodies", seed, t,
        );
        assert!(
            shipped.colliders.iter() == reference.colliders.iter(),
            "seed {} step {} colliders",
            seed,
            t,
        );
        assert!(
            shipped.narrow_phase.pairs == reference.narrow_phase.pairs,
            "seed {} step {} pairs",
            seed,
            t,
        );
        t += 1;
    }
    sparse
}

/// Random levels with user changes.
#[test]
#[fuzzer(runs: 6, seed: 20260925)]
fn fuzz_active_set_agrees(seed: u16) {
    let _ = run_both(seed.into(), 12, true);
}

/// Without user changes the structure falls asleep and the pebble's flight takes the active
/// set (the shot misses or hits, from the seed).
#[test]
fn test_active_set_is_taken_and_agrees() {
    let mut taken = 0;
    for seed in array![0_u32, 1, 2].span() {
        taken += run_both(*seed, 16, false);
    }
    assert!(taken != 0, "the active set was never taken");
}

/// A world restored from its state keeps its active set (`WorldState` version 2).
#[test]
fn test_active_set_survives_state() {
    let mut world = level(0);
    let mut t = 0;
    while t != 8 {
        let _ = world.step();
        t += 1;
    }
    let valid = super::active_set::is_valid(@world);
    let state = world.to_state();
    let mut restored = WorldTrait::from_state(state);
    assert_eq!(super::active_set::is_valid(@restored), valid);
    assert!(usable(ref restored) == usable(ref world));
    let a = world.step();
    let b = restored.step();
    assert!(a == b);
    assert!(world.bodies.iter() == restored.bodies.iter());
    // A write before saving invalidates the saved set.
    let (h, body) = *world.bodies.iter().at(1);
    assert!(world.set_body(h, body));
    let state = world.to_state();
    assert!(!state.active_set.valid);
}
