//! CC2: the `ccd_scenes` family (rapier2d-f64 0.35.3 with CCD, `tools/golden/src/ccd_scenes.rs`)
//! against `World::step_with_ccd`: the ball's pose and velocity within [`BAND`] at every step, and
//! its `is_ccd_active` flag exact.
//!
//! Upstream's automatic CCD (`plank_auto_*`) runs with `CCDSolverTrait::set_automatic(true)`; the
//! port's default (automatic off) is checked on the same scenes against upstream without CCD
//! (`plank_off_*`: the ball tunnels). `slow_bullet` is also stepped by `World::step`: CCD changes
//! no bit there.

use rapier2d::pipeline::ccd::{CCDSolver, CCDSolverTrait};
use rapier_core::Handle;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBodyBuilderTrait, RigidBodyCcdApiTrait};
use rapier_golden::generated::ccd_scenes;
use super::*;

/// Position band, in raw Q32.32 units (`2^10`, velocities twice as much). Measured: 408 on the
/// positions and 881 on the velocities (`plank_*_sub1`, the clamp step and the slide after it).
const BAND: u64 = 1024;

/// The world of scene `(kind, iterations, max_ccd_substeps, ccd_enabled)` and the ball's handle.
fn scene_world(kind: felt252, iterations: u32, substeps: u32, ccd: bool) -> (World, Handle) {
    let params = IntegrationParameters {
        dt: f(ccd_scenes::DT),
        num_solver_iterations: iterations,
        max_ccd_substeps: substeps,
        ..Default::default(),
    };
    let gravity = if kind == 'box' {
        Vec2 { x: Zero::zero(), y: Zero::zero() }
    } else {
        Vec2 { x: Zero::zero(), y: f(-42133629174) }
    };
    let mut world = WorldTrait::new(gravity, params);
    if kind == 'plank' {
        let half = vr(ccd_scenes::PLANK_HALF_EXTENTS);
        let _ = world.insert_collider(ColliderBuilderTrait::cuboid(half.x, half.y).build(), None);
    } else if kind == 'box' {
        let half = vr(ccd_scenes::BOX_HALF_EXTENTS);
        let _ = world
            .insert(
                RigidBodyBuilderTrait::dynamic().build(),
                ColliderBuilderTrait::cuboid(half.x, half.y).build(),
            );
    } else {
        let half = vr(ccd_scenes::GROUND_HALF_EXTENTS);
        let _ = world.insert_collider(ColliderBuilderTrait::cuboid(half.x, half.y).build(), None);
    }
    let (radius, start, linvel) = if kind == 'slow' {
        (
            ccd_scenes::SLOW_RADIUS,
            vr(ccd_scenes::SLOW_START),
            Vec2 { x: Zero::zero(), y: Zero::zero() },
        )
    } else {
        (ccd_scenes::BALL_RADIUS, vr(ccd_scenes::BALL_START), vr(ccd_scenes::BALL_LINVEL))
    };
    let body = RigidBodyBuilderTrait::dynamic()
        .translation(start)
        .linvel(linvel)
        .ccd_enabled(ccd)
        .build();
    let (ball, _) = world.insert(body, ColliderBuilderTrait::ball(f(radius)).build());
    (world, ball)
}

/// Steps scene `index` with `automatic` and checks every sample of scene `expected`; returns the
/// largest deviation of the positions and of the velocities, in raw units.
fn check(index: u32, expected: u32, automatic: bool) -> (u64, u64) {
    let (_, kind, iterations, substeps, ccd) = *ccd_scenes::scenes().at(index);
    let (mut world, ball) = scene_world(kind, iterations, substeps, ccd);
    let mut solver: CCDSolver = CCDSolverTrait::new();
    solver.set_automatic(automatic);
    let (mut max_pos, mut max_vel) = (0, 0);
    for sample in ccd_scenes::samples(expected) {
        let (step, x, y, vx, vy, active, other_x, other_y, _) = *sample;
        let _ = world.step_with_ccd(ref solver);
        let body = world.body(ball).unwrap();
        let t = body.pos.position.translation;
        let dp = max(abs_diff(t.x.raw, x), abs_diff(t.y.raw, y));
        let dv = max(abs_diff(body.vels.linvel.x.raw, vx), abs_diff(body.vels.linvel.y.raw, vy));
        assert!(dp <= BAND, "scene {} step {}: position off by {}", index, step, dp);
        assert!(dv <= 2 * BAND, "scene {} step {}: velocity off by {}", index, step, dv);
        if automatic || ccd || substeps == 0 {
            assert_eq!(body.is_ccd_active(), active, "scene {} step {}: ccd_active", index, step);
        }
        if kind == 'box' {
            let other = world.body(body_handle(0)).unwrap();
            let o = other.pos.position.translation;
            let d = max(abs_diff(o.x.raw, other_x), abs_diff(o.y.raw, other_y));
            assert!(d <= BAND, "scene {} step {}: box off by {}", index, step, d);
        }
        max_pos = max(max_pos, dp);
        max_vel = max(max_vel, dv);
    }
    println!("scene {}: max dpos {} max dvel {}", index, max_pos, max_vel);
    (max_pos, max_vel)
}

fn max(a: u64, b: u64) -> u64 {
    if a > b {
        a
    } else {
        b
    }
}

/// `(scene, expected samples, automatic)`: each scene against its own trace (upstream's automatic
/// CCD for `plank_auto_*`), then the port's default on the `plank_auto_*` inputs against
/// upstream without CCD (`plank_off_*`).
#[test]
fn test_ccd_scenes_match_upstream() {
    let cases = array![
        (0, 0, false), (1, 1, false), (2, 2, true), (3, 3, true), (4, 4, true), (5, 5, true),
        (6, 6, false), (7, 7, false), (8, 8, false), (2, 4, false), (3, 5, false),
    ];
    for (index, expected, automatic) in cases {
        let _ = check(index, expected, automatic);
    }
}

/// A scene where no body is ever fast: `step_with_ccd` leaves every bit as `World::step` does.
#[test]
fn test_slow_scene_is_bit_identical_to_step() {
    let (_, kind, iterations, substeps, ccd) = *ccd_scenes::scenes().at(8);
    let (mut with_ccd, ball) = scene_world(kind, iterations, substeps, ccd);
    let (mut plain, _) = scene_world(kind, iterations, substeps, ccd);
    let mut solver: CCDSolver = CCDSolverTrait::new();
    let mut i = 0;
    while i != 30 {
        let _ = with_ccd.step_with_ccd(ref solver);
        let _ = plain.step();
        let a = with_ccd.body(ball).unwrap();
        let b = plain.body(ball).unwrap();
        assert_eq!(a.pos, b.pos);
        assert_eq!(a.vels, b.vels);
        i += 1;
    }
}
