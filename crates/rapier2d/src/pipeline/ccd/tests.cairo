//! Tests of the CCD pass (`step_with_ccd`): tunnelling and clamping against a thin fixed plank,
//! the automatic tier, the per-pair casts and target tiers, the substep splitter, sensor
//! crossings, the caches, and the invariance of `max_ccd_substeps = 0`.

use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_core::interaction_groups::InteractionGroupsTrait;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::events::CollisionEventTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBodyBuilderTrait, RigidBodyCcdApiTrait};
use rapier_geometry2d::shape::{Ball, Cuboid, HalfSpace, Shape, ShapeTrait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use crate::world::{World, WorldTrait};
use super::sweeps::{FastCollider, Target, cast_pair, fast_colliders, sweep_body};
use super::{CCDSolver, CCDSolverTrait, next_dt};

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

fn ratio(n: i64, d: i64) -> Fixed {
    FixedTrait::from_ratio(n, d)
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
}

fn h(index: u32) -> Handle {
    Handle { index, generation: 0 }
}

/// A ball of radius `1/10` at `(-13/10, 0)` moving at `speed` m/s along `+x` without gravity,
/// towards `planks` fixed planks of half extents `(1/20, 1)`, the first at the origin, the others
/// off the ball's path (`(3i, 3i)`). Returns the world and the ball's handle.
fn plank(speed: i32, ccd: bool, planks: u32) -> (World, Handle) {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    let mut i: u32 = 0;
    while i != planks {
        let offset: i32 = i.try_into().unwrap();
        let _ = world
            .insert_collider(
                ColliderBuilderTrait::cuboid(ratio(1, 20), ONE)
                    .translation(v(int(offset * 3), int(offset * 3)))
                    .build(),
                None,
            );
        i += 1;
    }
    let body = RigidBodyBuilderTrait::dynamic()
        .translation(v(ratio(-13, 10), ZERO))
        .linvel(v(int(speed), ZERO))
        .ccd_enabled(ccd)
        .build();
    let (ball, _) = world.insert(body, ColliderBuilderTrait::ball(ratio(1, 10)).build());
    (world, ball)
}

fn x_of(ref world: World, handle: Handle) -> Fixed {
    world.body(handle).unwrap().pos.position.translation.x
}

#[test]
fn test_fast_ball_tunnels_without_ccd_and_stops_with_it() {
    // (speed, ccd_enabled, automatic, planks, stops before the plank)
    let cases = array![
        (30, false, false, 1, false), (30, true, false, 1, true), (30, false, true, 1, true),
        (30, true, false, 10, true), (6, true, false, 1, true), (72, true, true, 10, true),
    ];
    for (speed, ccd, automatic, planks, stops) in cases {
        let (mut world, ball) = plank(speed, ccd, planks);
        let mut solver: CCDSolver = CCDSolverTrait::new();
        solver.set_automatic(automatic);
        let mut i = 0;
        while i != 12 {
            let _ = world.step_with_ccd(ref solver);
            i += 1;
        }
        let x = x_of(ref world, ball);
        assert_eq!(x < ZERO, stops, "ccd {} automatic {} x {}", ccd, automatic, x.raw);
        if stops {
            // Stopped at the contact distance (0.15) within the linear slop.
            assert!(x > -ratio(16, 100) && x < -ratio(14, 100), "x {}", x.raw);
        }
    }
}

/// A ball of radius 1/10 swept from `(start_x, 0)` to `(1, 0)`.
fn fast_ball(start_x: Fixed) -> FastCollider {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    let (_, ch) = world
        .insert(
            RigidBodyBuilderTrait::dynamic().build(),
            ColliderBuilderTrait::ball(ratio(1, 10)).build(),
        );
    let mut out = fast_colliders(
        array![ch].span(),
        ref world.colliders,
        at(start_x, ZERO),
        at(ONE, ZERO),
        v(ZERO, ZERO),
        false,
    );
    let fc = out.pop_front().unwrap();
    assert_eq!(fc.shape, Shape::Ball(Ball { radius: ratio(1, 10) }));
    fc
}

/// `(target, start x, pseudo, expected fraction range)`: a thin box at the origin is hit when
/// the ball's centre is `0.05 + 0.1 - slop` away (the sweep's target distance); an initial overlap
/// retries with the core ball (radius 1/40: impact when its centre is 0.075 away, or none when the
/// core already overlaps); a pseudo pair reports the overlap at 0; a half-space never hits.
#[test]
fn test_cast_pair_clamping_cases() {
    let slop = ratio(1, 500);
    let plank = Shape::Cuboid(Cuboid { half_extents: v(ratio(1, 20), ONE) });
    let half_space = Shape::HalfSpace(HalfSpace { normal: v(ONE, ZERO) });
    let cases: Array<(Shape, Fixed, bool, Option<(Fixed, Fixed)>)> = array![
        (plank, -ONE, false, Some((ratio(424, 1000), ratio(428, 1000)))),
        (plank, -ratio(1, 10), false, Some((ratio(1, 1000), ratio(25, 1000)))),
        (plank, ZERO, false, None), (plank, -ratio(1, 10), true, Some((ZERO, ZERO))),
        (half_space, -ONE, false, None),
    ];
    for (shape, start, pseudo, expected) in cases {
        let fc = fast_ball(start);
        let got = cast_pair(@fc, shape, at(ZERO, ZERO), ONE, slop, pseudo);
        match expected {
            Some((
                low, high,
            )) => {
                let f = got.expect('expected a hit');
                assert!(f >= low && f <= high, "start {} fraction {}", start.raw, f.raw);
            },
            None => assert!(got.is_none(), "start {}: unexpected hit", start.raw),
        }
    }
}

/// `(bullet, target fixed, target bullet, groups match, same body, hits)`: a bullet hits every
/// target but a bullet's, a non-bullet the fixed targets only; groups and the own body filter.
#[test]
fn test_sweep_body_tiers() {
    let plank = Shape::Cuboid(Cuboid { half_extents: v(ratio(1, 20), ONE) });
    let cases = array![
        (true, true, false, true, false, true), (true, false, false, true, false, true),
        (true, false, true, true, false, false), (false, true, false, true, false, true),
        (false, false, false, true, false, false), (true, true, false, false, false, false),
        (true, false, false, true, true, false),
    ];
    for (bullet, fixed, target_bullet, groups, same, hits) in cases {
        let fc = fast_ball(-ONE);
        let target = Target {
            handle: h(7),
            shape: plank,
            pose: at(ZERO, ZERO),
            aabb: plank.compute_aabb(at(ZERO, ZERO)),
            body: if same {
                Some(h(3))
            } else {
                Some(h(5))
            },
            fixed,
            bullet: target_bullet,
            sensor: false,
            collision_groups: if groups {
                InteractionGroupsTrait::all()
            } else {
                InteractionGroupsTrait::none()
            },
            solver_groups: InteractionGroupsTrait::all(),
            active_events: Default::default(),
        };
        let mut pseudo = array![];
        let f = sweep_body(
            array![fc].span(), h(3), bullet, array![target].span(), ratio(1, 500), true, ref pseudo,
        );
        assert_eq!(
            f < ONE, hits, "bullet {} fixed {} bullet target {}", bullet, fixed, target_bullet,
        );
        assert!(pseudo.is_empty());
    }
}

/// Upstream's splitter on a 1/60 s step: `(substeps left, speed, expected dt, substeps left
/// after)`. Without an impact the rest of the step is taken at once (0 left); a slow bullet is
/// not fast; the plank's impact at about 0.0128 s (1.15 m at 90 m/s, after the first interval of
/// 1/240 s) gives `toi + rest / n`.
#[test]
fn test_substep_splitter() {
    let dt = ratio(1, 60);
    let cases = array![(1_u32, 30, dt, 0_u32), (4, 1, dt, 0), (4, 90, ZERO, 3)];
    for (substeps, speed, expected, left) in cases {
        let (mut world, _) = plank(speed, true, 1);
        let mut solver: CCDSolver = CCDSolverTrait::new();
        solver.refresh(ref world);
        let mut remaining_time = dt;
        let mut remaining = substeps;
        let got = next_dt(ref solver, ref world, ref remaining_time, ref remaining, ratio(1, 6000));
        assert_eq!(remaining, left);
        if expected != ZERO {
            assert_eq!(got, expected);
        } else {
            assert!(got > ratio(1, 240) && got < dt, "dt {}", got.raw);
            assert_eq!(remaining_time, dt - got);
        }
    }
}

/// A bullet crossing a thin sensor slab within one step: the narrow phase never sees the pair,
/// the pass emits `Started` then `Stopped`, flagged `SENSOR`, `(ball collider, sensor)`.
#[test]
fn test_sensor_crossing_emits_both_events() {
    // (ccd_enabled, events expected)
    let cases = array![(true, 2_u32), (false, 0)];
    for (ccd, expected) in cases {
        let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
        let sensor = world
            .insert_collider(
                ColliderBuilderTrait::cuboid(ratio(1, 20), ONE)
                    .sensor(true)
                    .active_events(COLLISION_EVENTS)
                    .build(),
                None,
            );
        let body = RigidBodyBuilderTrait::dynamic()
            .translation(v(ratio(-13, 10), ZERO))
            .linvel(v(int(30), ZERO))
            .ccd_enabled(ccd)
            .build();
        let (_, ball) = world.insert(body, ColliderBuilderTrait::ball(ratio(1, 10)).build());
        let mut solver: CCDSolver = CCDSolverTrait::new();
        let mut count: u32 = 0;
        let mut i = 0;
        while i != 4 {
            for event in world.step_with_ccd(ref solver) {
                assert!(event.sensor());
                assert_eq!((event.collider1(), event.collider2()), (ball, sensor));
                assert_eq!(event.started(), count % 2 == 0);
                count += 1;
            }
            i += 1;
        }
        assert_eq!(count, expected);
        // The sensor never stops the ball.
        assert!(world.body(h(0)).unwrap().pos.position.translation.x > HALF);
    }
}

/// The caches follow the writes: CCD switched on by `set_body`, a removal, the automatic switch
/// and an insertion.
#[test]
fn test_caches_follow_writes() {
    let (mut world, ball) = plank(30, false, 1);
    let mut solver: CCDSolver = CCDSolverTrait::new();
    let _ = world.step_with_ccd(ref solver);
    assert!(solver.candidates().is_empty());
    let mut body = world.body(ball).unwrap();
    body.enable_ccd(true);
    assert!(world.set_body(ball, body));
    let _ = world.step_with_ccd(ref solver);
    assert_eq!(solver.candidates(), array![ball].span());
    let _ = world.step_with_ccd(ref solver);
    assert_eq!(solver.candidates(), array![ball].span());
    assert!(world.body(ball).unwrap().is_ccd_active());
    let _ = world.remove_body(ball);
    let _ = world.step_with_ccd(ref solver);
    assert!(solver.candidates().is_empty());
    solver.set_automatic(true);
    let (other, _) = world
        .insert(RigidBodyBuilderTrait::dynamic().build(), ColliderBuilderTrait::ball(HALF).build());
    let _ = world.step_with_ccd(ref solver);
    assert_eq!(solver.candidates(), array![other].span());
}

/// `max_ccd_substeps = 0` disables CCD: `step_with_ccd` is `World::step`, bit for bit.
#[test]
fn test_zero_substeps_is_step() {
    let (mut with_ccd, ball) = plank(30, true, 1);
    let (mut plain, _) = plank(30, true, 1);
    with_ccd.integration_parameters.max_ccd_substeps = 0;
    plain.integration_parameters.max_ccd_substeps = 0;
    let mut solver: CCDSolver = CCDSolverTrait::new();
    solver.set_automatic(true);
    let mut i = 0;
    while i != 4 {
        let _ = with_ccd.step_with_ccd(ref solver);
        let _ = plain.step();
        i += 1;
    }
    assert_eq!(with_ccd.body(ball).unwrap(), plain.body(ball).unwrap());
    assert!(x_of(ref with_ccd, ball) > ZERO);
}

/// The solver serializes its switch alone; a restored solver rebuilds its caches.
#[test]
fn test_solver_serde() {
    let cases = array![false, true];
    for automatic in cases {
        let mut solver: CCDSolver = CCDSolverTrait::new();
        solver.set_automatic(automatic);
        let (mut world, _) = plank(30, true, 1);
        let _ = world.step_with_ccd(ref solver);
        let mut out = array![];
        solver.serialize(ref out);
        assert_eq!(out.len(), 1);
        let mut span = out.span();
        let back: CCDSolver = Serde::deserialize(ref span).unwrap();
        assert_eq!(back.automatic(), automatic);
        assert!(back.candidates().is_empty());
    }
}

/// The pass's short-circuited activation test answers upstream's on the interpolated velocity:
/// `(next x, next rotation im, thickness)` from rest at the origin, rotation about `(1/4, 0)`.
#[test]
fn test_moving_fast_matches_upstream_test() {
    let dt = ratio(1, 60);
    let inv_dt = rapier_math::math_ext::scalar::inv(dt);
    let cases = array![
        (ZERO, ZERO, ONE), (ratio(1, 2), ZERO, ONE), (ratio(51, 100), ZERO, ONE),
        (ZERO, ratio(6, 10), ratio(1, 10)), (ratio(1, 100), ratio(1, 100), ratio(1, 100)),
        (ratio(3, 1000), -ratio(2, 100), ratio(1, 20)), (ZERO, ratio(1, 1000), ratio(1, 1000)),
    ];
    for (x, im, thickness) in cases {
        let re = (ONE - im * im).sqrt();
        let pos = rapier_dynamics2d::rigid_body::RigidBodyPosition {
            position: at(ZERO, ZERO), next_position: Pose2Trait::new(v(x, ZERO), Rot2 { re, im }),
        };
        let ccd = rapier_dynamics2d::rigid_body::RigidBodyCcd {
            ccd_thickness: thickness, ..Default::default(),
        };
        let local_com = v(ratio(1, 4), ZERO);
        let vels = rapier_dynamics2d::rigid_body::RigidBodyPositionTrait::interpolate_velocity(
            pos, inv_dt, local_com,
        );
        let expected =
            rapier_dynamics2d::rigid_body::RigidBodyCcdTrait::is_moving_fast_with_next_position(
            @ccd, dt, vels, pos, local_com, HALF,
        );
        assert_eq!(super::moving_fast(ccd, dt, inv_dt, pos, local_com, HALF), expected);
    }
}
