//! CC2: what CCD costs on the G0 levels (`golden_scenes::levels`, 60 Hz, 4 iterations), in Sierra
//! gas and Cairo steps (`--tracked-resource cairo-steps`). Each probe loads a level and runs its
//! window (flight: ticks 1–25; impact: ticks 1–30) with `World::step_with_ccd`:
//! * `ccd_default`: the default solver, no `ccd_enabled` body (the cost of the CCD entry point);
//! * `ccd_auto`: upstream's automatic mode (every fast dynamic body sweeps the fixed colliders);
//! * `ccd_pebble`: the default solver with the pebble `ccd_enabled` (a bullet).
//! `gas_load_level10_ccd_auto` is the load with its setup step taken with the automatic mode.
//! Compare with `level_budget::{gas,steps}_{load,flight,impact}_level*` (`World::step`).

use core::num::traits::Zero;
use rapier2d::pipeline::ccd::{CCDSolver, CCDSolverTrait};
use rapier2d::prelude::{IntegrationParameters, RigidBodyTrait, Vec2, World, WorldTrait};
use rapier_dynamics2d::rigid_body_set::RigidBodyCcdApiTrait;
use rapier_golden::generated::level_scenes;
use rapier_testing::opaque;
use crate::golden_scenes::builder::{f, pose, vr};
use crate::golden_scenes::levels::{collider, despawn, handle, level, load_level};

/// Ticks of the pebble's flight (its first contact is at tick 26).
const FLIGHT: u32 = 25;
/// Flight plus the first five impact ticks.
const IMPACT: u32 = 30;

/// Level `blocks` at 60 Hz with 4 iterations, then `ticks` ticks with `step_with_ccd`; `mode` 0:
/// default solver, 1: automatic, 2: default solver and a `ccd_enabled` pebble.
fn run_ccd(blocks: u32, mode: u32, ticks: u32) -> World {
    let (bodies, linvel, bounds) = level(blocks);
    let n = bodies.len();
    let mut world = load_level(bodies, linvel, level_scenes::level10_hz60_sub4::DT, 4);
    let mut solver: CCDSolver = CCDSolverTrait::new();
    if mode == 1 {
        solver.set_automatic(true);
    } else if mode == 2 {
        let mut pebble = world.body(handle(n - 1)).unwrap();
        pebble.enable_ccd(true);
        assert!(world.set_body(handle(n - 1), pebble));
    }
    let mut t = 0;
    while t != ticks {
        let _ = world.step_with_ccd(ref solver);
        let _ = despawn(ref world, n, bounds);
        t += 1;
    }
    world
}

/// `load_level` with its setup step taken by `step_with_ccd` in the automatic mode.
fn load_ccd_auto(blocks: u32) -> World {
    let (bodies, linvel, _) = level(blocks);
    let params = IntegrationParameters {
        dt: f(level_scenes::level10_hz60_sub4::DT), num_solver_iterations: 4, ..Default::default(),
    };
    let mut world = WorldTrait::new(vr(level_scenes::GRAVITY), params);
    let n = bodies.len();
    let mut i = 0;
    while i != n - 1 {
        let desc = *bodies.at(i);
        let body = if desc.role == 'ground' {
            RigidBodyTrait::fixed(pose(desc.pose))
        } else {
            RigidBodyTrait::dynamic(pose(desc.pose))
        };
        let _ = world.insert(body, collider(desc));
        i += 1;
    }
    world.gravity = Vec2 { x: Zero::zero(), y: Zero::zero() };
    let mut solver: CCDSolver = CCDSolverTrait::new();
    solver.set_automatic(true);
    let _ = world.step_with_ccd(ref solver);
    world.gravity = vr(level_scenes::GRAVITY);
    let mut i = 1;
    while i != n - 1 {
        let mut rb = world.body(handle(i)).unwrap();
        rb.sleep();
        assert!(world.set_body(handle(i), rb));
        i += 1;
    }
    let desc = *bodies.at(n - 1);
    let mut pebble = RigidBodyTrait::dynamic(pose(desc.pose));
    pebble.set_linvel(vr(linvel));
    let _ = world.insert(pebble, collider(desc));
    world
}

fn probe(blocks: u32, mode: u32, ticks: u32) {
    let world = run_ccd(opaque(blocks), opaque(mode), opaque(ticks));
    let _ = opaque(world.gravity);
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_load_level10_ccd_auto() {
    let world = load_ccd_auto(opaque(10));
    let _ = opaque(world.gravity);
}

#[test]
fn gas_flight_level10_ccd_default() {
    probe(10, 0, FLIGHT);
}

#[test]
fn gas_flight_level10_ccd_auto() {
    probe(10, 1, FLIGHT);
}

#[test]
fn gas_flight_level10_ccd_pebble() {
    probe(10, 2, FLIGHT);
}

#[test]
fn gas_impact_level10_ccd_default() {
    probe(10, 0, IMPACT);
}

#[test]
fn gas_impact_level10_ccd_auto() {
    probe(10, 1, IMPACT);
}

#[test]
fn gas_impact_level10_ccd_pebble() {
    probe(10, 2, IMPACT);
}

#[test]
fn gas_flight_level20_ccd_auto() {
    probe(20, 1, FLIGHT);
}

#[test]
fn gas_impact_level20_ccd_auto() {
    probe(20, 1, IMPACT);
}
