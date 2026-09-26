//! `gas_*` probes of the CCD pass: one fast bullet (radius 1/10, 30 m/s) towards 1 or 10 fixed
//! planks (the first on its path), stepped by `step_with_ccd`; `World::step` on the same worlds for
//! comparison. Subtract the `setup` twin (the same world after the same warm-up) to get one step:
//! * `sweep`: step 2, the ball fast, swept, free (no clamp);
//! * `clamp`: step 3, swept and clamped at the plank;
//! * `idle`: a world without CCD body (the default solver's cache check);
//! * `nocache`: the fixed targets and candidates rebuilt before the step (`invalidate`).

use fixed::{Fixed, FixedTrait, ONE, ZERO};
use glam::Vec2;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::rigid_body_set::RigidBodyBuilderTrait;
use rapier_testing::opaque;
use crate::world::{World, WorldTrait};
use super::{CCDSolver, CCDSolverTrait};

fn ratio(n: i64, d: i64) -> Fixed {
    FixedTrait::from_ratio(n, d)
}

fn world_of(planks: u32, ccd: bool) -> World {
    let mut world = WorldTrait::new(Vec2 { x: ZERO, y: ZERO }, Default::default());
    let mut i: u32 = 0;
    while i != planks {
        let offset: i32 = i.try_into().unwrap();
        let at = FixedTrait::from_int(offset * 3);
        let _ = world
            .insert_collider(
                ColliderBuilderTrait::cuboid(ratio(1, 20), ONE)
                    .translation(Vec2 { x: at, y: at })
                    .build(),
                None,
            );
        i += 1;
    }
    let body = RigidBodyBuilderTrait::dynamic()
        .translation(Vec2 { x: ratio(-13, 10), y: ZERO })
        .linvel(Vec2 { x: FixedTrait::from_int(30), y: ZERO })
        .ccd_enabled(ccd)
        .build();
    let _ = world.insert(body, ColliderBuilderTrait::ball(ratio(1, 10)).build());
    world
}

/// `warmup` CCD steps, then `measured` steps: with CCD (`mode` 0), with CCD after `invalidate`
/// (1), or `World::step` (2).
#[inline(never)]
fn probe(planks: u32, ccd: bool, warmup: u32, measured: u32, mode: u32) {
    let mut world = world_of(planks, ccd);
    let mut solver: CCDSolver = CCDSolverTrait::new();
    let mut k = 0;
    while k != warmup {
        let _ = world.step_with_ccd(ref solver);
        k += 1;
    }
    let mut k = 0;
    while k != measured {
        if mode == 2 {
            let _ = world.step();
        } else {
            if mode == 1 {
                solver.invalidate();
            }
            let _ = world.step_with_ccd(ref solver);
        }
        k += 1;
    }
    let _ = opaque(world.colliders.len());
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_setup_sweep1() {
    probe(opaque(1), true, opaque(1), 0, 0);
}

#[test]
fn gas_ccd_sweep1() {
    probe(opaque(1), true, opaque(1), 1, 0);
}

#[test]
fn gas_plain_sweep1() {
    probe(opaque(1), true, opaque(1), 1, 2);
}

#[test]
fn gas_setup_sweep10() {
    probe(opaque(10), true, opaque(1), 0, 0);
}

#[test]
fn gas_ccd_sweep10() {
    probe(opaque(10), true, opaque(1), 1, 0);
}

#[test]
fn gas_plain_sweep10() {
    probe(opaque(10), true, opaque(1), 1, 2);
}

#[test]
fn gas_ccd_sweep10_nocache() {
    probe(opaque(10), true, opaque(1), 1, 1);
}

#[test]
fn gas_setup_clamp1() {
    probe(opaque(1), true, opaque(2), 0, 0);
}

#[test]
fn gas_ccd_clamp1() {
    probe(opaque(1), true, opaque(2), 1, 0);
}

#[test]
fn gas_plain_clamp1() {
    probe(opaque(1), true, opaque(2), 1, 2);
}

#[test]
fn gas_setup_clamp10() {
    probe(opaque(10), true, opaque(2), 0, 0);
}

#[test]
fn gas_ccd_clamp10() {
    probe(opaque(10), true, opaque(2), 1, 0);
}

#[test]
fn gas_plain_clamp10() {
    probe(opaque(10), true, opaque(2), 1, 2);
}

#[test]
fn gas_setup_idle10() {
    probe(opaque(10), false, opaque(1), 0, 0);
}

#[test]
fn gas_ccd_idle10() {
    probe(opaque(10), false, opaque(1), 1, 0);
}

#[test]
fn gas_plain_idle10() {
    probe(opaque(10), false, opaque(1), 1, 2);
}
