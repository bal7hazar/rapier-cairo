//! G0: the level-shaped scenes (`rapier_golden::generated::level_scenes`) against upstream's
//! traces. A level is loaded as the game loads it: the ground, the blocks and the cores are
//! inserted, one setup step creates their contacts (whose start wakes every body, in both
//! engines), then every block and core is put to sleep and the pebble is inserted with its
//! launch velocity. Each tick is one `World::step` followed by the game's despawn rule (a
//! dynamic body whose centre leaves the level's `x` bounds is removed).
//!
//! Judgement (`tools/golden/README.md`, "level_scenes"): every sample up to the run's strict
//! window within the scene tolerances (`2^12 · tick` ulp on poses, twice that on velocities),
//! the tick at which the structure wakes equal to upstream's; after the window the pebble impact
//! is chaotic (a 14- or 24-body pile hit at 18 m/s), so only invariants are checked: no body
//! sinks under the ground, every body stays within bounds, and the run's awake statistics are
//! printed next to upstream's.

use core::num::traits::Zero;
use rapier2d::prelude::{
    ColliderBuilderTrait, Handle, IntegrationParameters, RigidBodyTrait, Vec2, World, WorldTrait,
};
use rapier_dynamics2d::collider::Collider;
use rapier_golden::compare::abs_diff;
use rapier_golden::generated::level_scenes;
use rapier_golden::types::{LevelBodyRaw, PolygonContactShapeRaw, ShapeRaw, Vec2Raw};
use super::builder::{f, pose, vr};

/// One sampled state `(tick, body, sleeping, x, y, re, im, vx, vy, angvel)`.
pub type State = (u32, u32, bool, i64, i64, i64, i64, i64, i64, i64);

/// Handle of the `index`-th body of a level (fresh arena, insertion order, nothing removed
/// before the pebble is inserted).
pub fn handle(index: u32) -> Handle {
    Handle { index, generation: 0 }
}

/// The bodies, pebble launch velocity and despawn bounds of level `10` or `20`.
pub fn level(blocks: u32) -> (Span<LevelBodyRaw>, Vec2Raw, (i64, i64)) {
    if blocks == 10 {
        (
            level_scenes::LEVEL10_BODIES.span(),
            level_scenes::LEVEL10_PEBBLE_LINVEL,
            level_scenes::LEVEL10_BOUNDS_X,
        )
    } else {
        (
            level_scenes::LEVEL20_BODIES.span(),
            level_scenes::LEVEL20_PEBBLE_LINVEL,
            level_scenes::LEVEL20_BOUNDS_X,
        )
    }
}

fn collider(desc: LevelBodyRaw) -> Collider {
    let builder = match desc.shape {
        PolygonContactShapeRaw::Polygon(polygon) => {
            let mut points = array![];
            let mut k: u8 = 0;
            for v in polygon.vertices.span() {
                if k == polygon.count {
                    break;
                }
                points.append(vr(*v));
                k += 1;
            }
            ColliderBuilderTrait::convex_polygon(points.span()).expect('level polygon')
        },
        PolygonContactShapeRaw::Other(shape) => match shape {
            ShapeRaw::Ball(radius) => ColliderBuilderTrait::ball(f(radius)),
            ShapeRaw::Cuboid(half) => ColliderBuilderTrait::cuboid(f(half.x), f(half.y)),
            ShapeRaw::HalfSpace(normal) => ColliderBuilderTrait::halfspace(vr(normal)),
            _ => panic!("unexpected level shape"),
        },
    };
    builder
        .density(f(desc.density))
        .friction(f(desc.friction))
        .restitution(f(desc.restitution))
        .build()
}

/// The world of a level after its load (see the module documentation), with `dt` and
/// `iterations` solver iterations; the other parameters at their defaults.
pub fn load_level(
    bodies: Span<LevelBodyRaw>, pebble_linvel: Vec2Raw, dt: i64, iterations: u32,
) -> World {
    let params = IntegrationParameters {
        dt: f(dt), num_solver_iterations: iterations, ..Default::default(),
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
        let (h, _) = world.insert(body, collider(desc));
        assert!(h == handle(i), "level handles follow insertion order");
        i += 1;
    }
    // Setup step without gravity: contacts created, nothing moves (exact contact, no load).
    world.gravity = Vec2 { x: Zero::zero(), y: Zero::zero() };
    let _ = world.step();
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
    pebble.set_linvel(vr(pebble_linvel));
    let (h, _) = world.insert(pebble, collider(desc));
    assert!(h == handle(n - 1), "pebble handle");
    world
}

/// The despawn rule: removes every dynamic body (indices `1..n`) whose centre is outside
/// `bounds`; returns how many were removed.
pub fn despawn(ref world: World, n: u32, bounds: (i64, i64)) -> u32 {
    let (x_min, x_max) = bounds;
    let mut removed = 0;
    let mut i = 1;
    while i != n {
        if let Some(rb) = world.body(handle(i)) {
            let x = rb.position().translation.x.raw;
            if x < x_min || x > x_max {
                let _ = world.remove_body(handle(i));
                removed += 1;
            }
        }
        i += 1;
    }
    removed
}

/// One tick: `World::step` then the despawn rule.
pub fn tick(ref world: World, n: u32, bounds: (i64, i64)) {
    let _ = world.step();
    let _ = despawn(ref world, n, bounds);
}

/// Awake dynamic bodies of the level (removed ones excluded).
pub fn awake_count(ref world: World, n: u32) -> u32 {
    let mut awake = 0;
    let mut i = 1;
    while i != n {
        if let Some(rb) = world.body(handle(i)) {
            if !rb.is_sleeping() {
                awake += 1;
            }
        }
        i += 1;
    }
    awake
}

/// Upstream's record of one run.
#[derive(Copy, Drop)]
pub struct Trace {
    pub dt: i64,
    pub iterations: u32,
    pub ticks: u32,
    pub all_asleep: u32,
    pub calm_end: u32,
    pub awake: Span<u8>,
    pub samples: Span<State>,
}

/// What a replay measured.
#[derive(Copy, Drop, Debug)]
pub struct Replay {
    /// First tick with more than one awake body (the structure woke), in the port / upstream.
    pub wake: u32,
    pub wake_upstream: u32,
    /// Last sampled tick up to which every sample is within the tolerances.
    pub within_until: u32,
    /// Samples beyond the tolerances (or with a body present on one side only).
    pub violations: u32,
    /// Largest pose / velocity deviation over the samples up to `within_until`, ulps.
    pub max_pose: u64,
    pub max_vel: u64,
    /// Awake body-ticks, max awake, first all-asleep tick (0: none) in the port.
    pub awake_ticks: u32,
    pub max_awake: u32,
    pub all_asleep: u32,
    pub awake_ticks_upstream: u32,
    /// First tick whose awake count differs from upstream's (0: none).
    pub awake_differs: u32,
}

fn trace_awake(trace: Trace) -> (u32, u32) {
    let mut total = 0;
    let mut wake = 0;
    let mut t = 1;
    for a in trace.awake {
        let a: u32 = (*a).into();
        total += a;
        if wake == 0 && a > 1 {
            wake = t;
        }
        t += 1;
    }
    (total, wake)
}

fn max(a: u64, b: u64) -> u64 {
    if a > b {
        a
    } else {
        b
    }
}

/// Replays the first `until` ticks of `trace` on level `blocks`, comparing every sample (see
/// the module documentation).
pub fn replay(blocks: u32, trace: Trace, until: u32) -> Replay {
    let (bodies, linvel, bounds) = level(blocks);
    let n = bodies.len();
    let mut world = load_level(bodies, linvel, trace.dt, trace.iterations);
    let (awake_ticks_upstream, wake_upstream) = trace_awake(trace);
    let mut report = Replay {
        wake: 0,
        wake_upstream,
        within_until: 0,
        violations: 0,
        max_pose: 0,
        max_vel: 0,
        awake_ticks: 0,
        max_awake: 0,
        all_asleep: 0,
        awake_ticks_upstream,
        awake_differs: 0,
    };
    let mut samples = trace.samples;
    let mut diverged = false;
    let mut t: u32 = 0;
    loop {
        // Compare the samples of tick `t`.
        let mut pose_dev: u64 = 0;
        let mut vel_dev: u64 = 0;
        let mut seen: u32 = 0;
        let mut mismatch = false;
        while let Some(s) = samples.get(0) {
            let (tick, body, sleeping, x, y, re, im, vx, vy, w) = *s.unbox();
            if tick != t {
                break;
            }
            let _ = samples.pop_front();
            seen += 1;
            match world.body(handle(body)) {
                Some(rb) => {
                    let p = rb.position();
                    let v = rb.linvel();
                    pose_dev =
                        max(
                            pose_dev,
                            max(
                                max(
                                    abs_diff(p.translation.x.raw, x),
                                    abs_diff(p.translation.y.raw, y),
                                ),
                                max(
                                    abs_diff(p.rotation.re.raw, re),
                                    abs_diff(p.rotation.im.raw, im),
                                ),
                            ),
                        );
                    vel_dev =
                        max(
                            vel_dev,
                            max(
                                max(abs_diff(v.x.raw, vx), abs_diff(v.y.raw, vy)),
                                abs_diff(rb.vels.angvel.raw, w),
                            ),
                        );
                    if rb.is_sleeping() != sleeping {
                        mismatch = true;
                    }
                },
                None => { mismatch = true; },
            }
        }
        if seen != 0 {
            if seen + 1 != n - removed_count(ref world, n) {
                mismatch = true;
            }
            let tol: u64 = 4096 * (if t == 0 {
                1
            } else {
                t.into()
            });
            let ok = !mismatch && pose_dev <= tol && vel_dev <= 2 * tol;
            let flag: ByteArray = if ok {
                ""
            } else {
                " (beyond)"
            };
            println!("tick {}: pose {} vel {} ulp{}", t, pose_dev, vel_dev, flag);
            if ok && !diverged {
                report.within_until = t;
                report.max_pose = max(report.max_pose, pose_dev);
                report.max_vel = max(report.max_vel, vel_dev);
            } else if !ok {
                diverged = true;
                report.violations += 1;
            }
        }
        if t == until {
            break;
        }
        tick(ref world, n, bounds);
        t += 1;
        let awake = awake_count(ref world, n);
        if report.awake_differs == 0 && awake != (*trace.awake.at(t - 1)).into() {
            report.awake_differs = t;
        }
        report.awake_ticks += awake;
        if awake > report.max_awake {
            report.max_awake = awake;
        }
        if report.wake == 0 && awake > 1 {
            report.wake = t;
        }
        if report.all_asleep == 0 && awake == 0 {
            report.all_asleep = t;
        }
        // Invariants: no dynamic body sinks under the ground (its centre stays above y = 0).
        let mut i = 1;
        while i != n {
            if let Some(rb) = world.body(handle(i)) {
                assert!(rb.position().translation.y.raw > 0, "body {} under the ground", i);
            }
            i += 1;
        }
    }
    println!("{:?}", report);
    report
}

fn removed_count(ref world: World, n: u32) -> u32 {
    let mut removed = 0;
    let mut i = 1;
    while i != n {
        if world.body(handle(i)).is_none() {
            removed += 1;
        }
        i += 1;
    }
    removed
}

fn mk(
    dt: i64,
    iterations: u32,
    ticks: u32,
    all_asleep: u32,
    calm_end: u32,
    awake: Span<u8>,
    samples: Span<State>,
) -> Trace {
    Trace { dt, iterations, ticks, all_asleep, calm_end, awake, samples }
}

/// Upstream's trace of level `blocks` under setting `k` (60 Hz with 4, 2, 1 solver
/// iterations, then 30 Hz with 4).
pub fn trace(blocks: u32, k: u32) -> Trace {
    let index = if blocks == 10 {
        k
    } else {
        4 + k
    };
    if index == 0 {
        mk(
            level_scenes::level10_hz60_sub4::DT,
            level_scenes::level10_hz60_sub4::NUM_SOLVER_ITERATIONS,
            level_scenes::level10_hz60_sub4::NUM_TICKS,
            level_scenes::level10_hz60_sub4::ALL_ASLEEP,
            level_scenes::level10_hz60_sub4::CALM_END,
            level_scenes::level10_hz60_sub4::awake(),
            level_scenes::level10_hz60_sub4::samples(),
        )
    } else if index == 1 {
        mk(
            level_scenes::level10_hz60_sub2::DT,
            level_scenes::level10_hz60_sub2::NUM_SOLVER_ITERATIONS,
            level_scenes::level10_hz60_sub2::NUM_TICKS,
            level_scenes::level10_hz60_sub2::ALL_ASLEEP,
            level_scenes::level10_hz60_sub2::CALM_END,
            level_scenes::level10_hz60_sub2::awake(),
            level_scenes::level10_hz60_sub2::samples(),
        )
    } else if index == 2 {
        mk(
            level_scenes::level10_hz60_sub1::DT,
            level_scenes::level10_hz60_sub1::NUM_SOLVER_ITERATIONS,
            level_scenes::level10_hz60_sub1::NUM_TICKS,
            level_scenes::level10_hz60_sub1::ALL_ASLEEP,
            level_scenes::level10_hz60_sub1::CALM_END,
            level_scenes::level10_hz60_sub1::awake(),
            level_scenes::level10_hz60_sub1::samples(),
        )
    } else if index == 3 {
        mk(
            level_scenes::level10_hz30_sub4::DT,
            level_scenes::level10_hz30_sub4::NUM_SOLVER_ITERATIONS,
            level_scenes::level10_hz30_sub4::NUM_TICKS,
            level_scenes::level10_hz30_sub4::ALL_ASLEEP,
            level_scenes::level10_hz30_sub4::CALM_END,
            level_scenes::level10_hz30_sub4::awake(),
            level_scenes::level10_hz30_sub4::samples(),
        )
    } else if index == 4 {
        mk(
            level_scenes::level20_hz60_sub4::DT,
            level_scenes::level20_hz60_sub4::NUM_SOLVER_ITERATIONS,
            level_scenes::level20_hz60_sub4::NUM_TICKS,
            level_scenes::level20_hz60_sub4::ALL_ASLEEP,
            level_scenes::level20_hz60_sub4::CALM_END,
            level_scenes::level20_hz60_sub4::awake(),
            level_scenes::level20_hz60_sub4::samples(),
        )
    } else if index == 5 {
        mk(
            level_scenes::level20_hz60_sub2::DT,
            level_scenes::level20_hz60_sub2::NUM_SOLVER_ITERATIONS,
            level_scenes::level20_hz60_sub2::NUM_TICKS,
            level_scenes::level20_hz60_sub2::ALL_ASLEEP,
            level_scenes::level20_hz60_sub2::CALM_END,
            level_scenes::level20_hz60_sub2::awake(),
            level_scenes::level20_hz60_sub2::samples(),
        )
    } else if index == 6 {
        mk(
            level_scenes::level20_hz60_sub1::DT,
            level_scenes::level20_hz60_sub1::NUM_SOLVER_ITERATIONS,
            level_scenes::level20_hz60_sub1::NUM_TICKS,
            level_scenes::level20_hz60_sub1::ALL_ASLEEP,
            level_scenes::level20_hz60_sub1::CALM_END,
            level_scenes::level20_hz60_sub1::awake(),
            level_scenes::level20_hz60_sub1::samples(),
        )
    } else {
        mk(
            level_scenes::level20_hz30_sub4::DT,
            level_scenes::level20_hz30_sub4::NUM_SOLVER_ITERATIONS,
            level_scenes::level20_hz30_sub4::NUM_TICKS,
            level_scenes::level20_hz30_sub4::ALL_ASLEEP,
            level_scenes::level20_hz30_sub4::CALM_END,
            level_scenes::level20_hz30_sub4::awake(),
            level_scenes::level20_hz30_sub4::samples(),
        )
    }
}

/// The strict window: the pebble's flight and the wake of the structure. Every sample before
/// the wake tick within the tolerances, the same wake tick and the same awake counts up to it;
/// the two ticks after it are replayed and printed (see the module documentation).
fn window(blocks: u32, k: u32) {
    let trace = trace(blocks, k);
    let (_, wake) = trace_awake(trace);
    let report = replay(blocks, trace, wake + 2);
    assert_eq!(report.wake, report.wake_upstream);
    assert!(report.within_until + 1 >= wake, "flight beyond the tolerances");
    assert!(report.awake_differs == 0 || report.awake_differs > wake, "awake counts");
}

/// A whole run: the window's checks, then the invariants only.
fn whole(blocks: u32, k: u32) {
    let trace = trace(blocks, k);
    let (_, wake) = trace_awake(trace);
    let report = replay(blocks, trace, trace.ticks);
    assert_eq!(report.wake, report.wake_upstream);
    assert!(report.within_until + 1 >= wake, "flight beyond the tolerances");
    println!("upstream: all asleep at {}, calm end at {}", trace.all_asleep, trace.calm_end);
}

#[test]
fn test_level10_hz60_sub4_window() {
    window(10, 0);
}

#[test]
fn test_level10_hz60_sub2_window() {
    window(10, 1);
}

#[test]
fn test_level10_hz60_sub1_window() {
    window(10, 2);
}

#[test]
fn test_level10_hz30_sub4_window() {
    window(10, 3);
}

#[test]
fn test_level20_hz60_sub4_window() {
    window(20, 0);
}

#[test]
fn test_level20_hz60_sub2_window() {
    window(20, 1);
}

#[test]
fn test_level20_hz60_sub1_window() {
    window(20, 2);
}

#[test]
fn test_level20_hz30_sub4_window() {
    window(20, 3);
}

#[test]
#[ignore]
fn test_level10_hz60_sub4_whole() {
    whole(10, 0);
}

#[test]
#[ignore]
fn test_level10_hz60_sub2_whole() {
    whole(10, 1);
}

#[test]
#[ignore]
fn test_level10_hz60_sub1_whole() {
    whole(10, 2);
}

#[test]
#[ignore]
fn test_level10_hz30_sub4_whole() {
    whole(10, 3);
}

#[test]
#[ignore]
fn test_level20_hz60_sub4_whole() {
    whole(20, 0);
}

#[test]
#[ignore]
fn test_level20_hz60_sub2_whole() {
    whole(20, 1);
}

#[test]
#[ignore]
fn test_level20_hz60_sub1_whole() {
    whole(20, 2);
}

#[test]
#[ignore]
fn test_level20_hz30_sub4_whole() {
    whole(20, 3);
}
