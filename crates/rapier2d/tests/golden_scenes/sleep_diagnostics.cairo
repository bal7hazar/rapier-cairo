//! SI: the wake-step solver omits the dormant ground pair upstream, but revives it in Cairo.
//! Counterfactuals use public pipeline stages; no production engine code is modified.
use rapier2d::pipeline::{self, SleepCensusTrait};
use rapier2d::prelude::{World, WorldTrait};
use rapier_dynamics2d::joint::ImpulseJointSetTrait;
use rapier_dynamics2d::rigid_body_set::RigidBodySetTrait;
use rapier_golden::compare::abs_diff;
use rapier_golden::generated::sleep_impact;
use rapier_golden::scenes;
use rapier_golden::types::SleepImpactRaw;
use super::builder::{body_handle, build_world, f, reseed};
use super::{Stats, compare};

#[derive(Copy, Drop, PartialEq, Debug)]
enum Mode {
    Engine,
    DelayGround,
    WeakToucher,
    ReferenceGeometry,
    ReferenceCache,
    ZeroSleepingVelocity,
    OldContact,
}

// Start from the generated upstream step 80, with its dormant ground manifold and impulse.
fn seeded() -> World {
    let scene = scenes::BALL_DROP_SLEEP;
    let mut w = build_world(scene);
    for sample in scene.samples.span() {
        if *sample.step == 80 {
            reseed(ref w, scene, *sample);
        }
    }
    pipeline::handle_user_changes(ref w.bodies, ref w.colliders, w.narrow_phase.pairs.span());
    let _ = pipeline::detect_collisions(
        w.integration_parameters, ref w.bodies, ref w.colliders, ref w.narrow_phase,
    );
    let up = sleep_impact::STEP_80;
    let [ground, _] = up.pairs;
    let mut pairs = array![];
    for p in w.narrow_phase.pairs.span() {
        let mut p = *p;
        let [mut a, b] = p.manifold.points;
        a.dist = f(ground.dist);
        a.data.impulse = f(ground.impulse);
        a.data.warmstart_impulse = f(ground.warmstart);
        p.manifold.points = [a, b];
        let [mut a, b] = p.manifold.data.solver_contacts;
        a.dist = f(ground.dist);
        a.contact_id = ground.contact_id;
        p.manifold.data.solver_contacts = [a, b];
        pairs.append(p);
    }
    w.narrow_phase.pairs = pairs;
    let mut i = 1;
    for state in up.bodies.span() {
        let h = body_handle(i);
        let mut b = w.body(h).unwrap();
        b.activation.sleeping = *state.sleeping;
        b.activation.time_since_can_sleep = f(*state.timer);
        assert!(w.bodies.set(h, b));
        i += 1;
    }
    w
}

/// The impact step split at the island/solver boundary. Only the named intervention differs.
fn staged(ref w: World, mode: Mode) {
    let p = w.integration_parameters;
    let lower = w.body(body_handle(1)).unwrap();
    let upper = w.body(body_handle(2)).unwrap();
    assert!(lower.activation.sleeping && !upper.activation.sleeping);
    assert_eq!(lower.vels.linvel.y.raw, 0);
    assert_eq!(upper.activation.time_since_can_sleep.raw, 0);
    if mode == Mode::ZeroSleepingVelocity {
        let mut lower = lower;
        lower.vels.linvel.y = f(0);
        assert!(w.bodies.set(body_handle(1), lower));
    }
    pipeline::handle_user_changes(ref w.bodies, ref w.colliders, w.narrow_phase.pairs.span());
    let _ = pipeline::detect_collisions(p, ref w.bodies, ref w.colliders, ref w.narrow_phase);
    let entries = w.bodies.iter().span();
    let joints = w.impulse_joints.to_array();
    let (_, sleeping, woken) = pipeline::update_islands(
        ref w.bodies,
        w.narrow_phase.pairs.span(),
        array![].span(),
        joints.span(),
        entries,
        SleepCensusTrait::taken(entries),
    );
    assert!(woken && !sleeping);
    // Both bodies stay awake for all four solver substeps: no activation update inside solve.
    for i in array![1, 2] {
        let b = w.body(body_handle(i)).unwrap();
        assert!(!b.activation.sleeping);
        assert_eq!(b.activation.time_since_can_sleep.raw, 0);
    }
    if mode == Mode::WeakToucher {
        // Upstream weak-wakes the already-awake toucher, preserving its previous timer.
        let mut b = w.body(body_handle(2)).unwrap();
        b.activation.time_since_can_sleep = upper.activation.time_since_can_sleep;
        assert!(w.bodies.set(body_handle(2), b));
    }
    let mut dormant = array![];
    let mut active = array![];
    let [ground_ref, ball_ref] = sleep_impact::STEP_87.pairs;
    for pair in w.narrow_phase.pairs.span() {
        let mut pair = *pair;
        assert_eq!(pair.manifold.data.restitution.raw, 0);
        if pair.collider1.index == 0 && mode == Mode::DelayGround {
            dormant.append(pair);
            continue;
        }
        let [mut point, other] = pair.manifold.points;
        let [mut sc, other_sc] = pair.manifold.data.solver_contacts;
        if pair.collider1.index == 1 {
            assert_eq!(pair.manifold.num_points, 1);
            assert_eq!(pair.manifold.data.num_solver_contacts, 1);
            assert_eq!(sc.contact_id, ball_ref.contact_id);
            assert_eq!(sc.contact_id, 0x80000000);
            assert!(sc.dist.raw < 0); // Already penetrating: no speculative-contact threshold.
            assert!(abs_diff(sc.dist.raw, ball_ref.dist) <= 1024);
            assert_eq!(pair.manifold.local_n1.y.raw, 0x100000000);
            assert_eq!(point.local_p1.y.raw, 0x80000000);
            assert_eq!(point.local_p2.y.raw, -0x80000000);
            assert_eq!(point.data.warmstart_impulse.raw, 0);
            if mode == Mode::ReferenceGeometry {
                sc.dist = f(ball_ref.dist);
                point.dist = f(ball_ref.dist);
            }
            if mode == Mode::OldContact {
                // Clears NEW: removes the restitution seed and enables an already-zero warm start.
                sc.contact_id = 0;
            }
        } else if mode == Mode::ReferenceCache {
            point.data.impulse = f(ground_ref.impulse);
            point.data.warmstart_impulse = f(ground_ref.warmstart);
        }
        pair.manifold.points = [point, other];
        pair.manifold.data.solver_contacts = [sc, other_sc];
        active.append(pair);
    }
    assert_eq!(dormant.len(), if mode == Mode::DelayGround {
        1
    } else {
        0
    });
    w.narrow_phase.pairs = active;
    pipeline::solve(w.gravity, p, ref w.bodies, ref w.narrow_phase, ref w.impulse_joints);
    w.narrow_phase.pairs = pipeline::merge_pairs(w.narrow_phase.pairs.span(), dormant.span());
    pipeline::advance_to_final_positions(ref w.bodies, ref w.colliders, p);
}

/// Worst vertical position/velocity error over the two balls.
fn errors(ref w: World, up: SleepImpactRaw) -> (u64, u64) {
    let mut y = 0;
    let mut v = 0;
    let mut i = 1;
    for expected in up.bodies.span() {
        let b = w.body(body_handle(i)).unwrap();
        y = super::max(y, abs_diff(b.pos.position.translation.y.raw, *expected.y));
        v = super::max(v, abs_diff(b.vels.linvel.y.raw, *expected.vy));
        assert_eq!(b.activation.sleeping, *expected.sleeping);
        i += 1;
    }
    (y, v)
}

/// Full replay also checks the reverse Rust experiment (prewake before step 87).
fn recovery(mode: Mode, seed: bool) -> Stats {
    let mut w = if seed {
        seeded()
    } else {
        build_world(scenes::BALL_DROP_SLEEP)
    };
    let scene = scenes::BALL_DROP_SLEEP;
    let mut stats: Stats = Default::default();
    let mut step = if seed {
        80_u32
    } else {
        0
    };
    while step != 120 {
        step += 1;
        if step == 87 {
            staged(ref w, mode);
        } else {
            let _ = w.step();
        }
        for up in sleep_impact::cases() {
            if *up.step == step {
                let (y, v) = errors(ref w, *up);
                if step <= 86 || mode == Mode::DelayGround {
                    assert!(y <= 4096 * step.into() && v <= 8192 * step.into());
                }
                if step == 87 || step == 110 {
                    println!("SI {:?} seed {} step {} y {} vy {}", mode, seed, step, y, v);
                }
                if step == 87 && mode == Mode::Engine {
                    assert!(y > 500000000 && v > 38000000000);
                }
            }
        }
        if mode == Mode::Engine {
            for up in sleep_impact::prewake_cases() {
                if *up.step == step {
                    let (y, v) = errors(ref w, *up);
                    println!("Rust prewake step {} y {} vy {}", step, y, v);
                    assert!(y <= 4096 * step.into() && v <= 8192 * step.into());
                }
            }
        }
        for sample in scene.samples.span() {
            if *sample.step == step && step >= 90 {
                stats = compare(ref w, scene, *sample, stats, true);
            }
        }
    }
    println!(
        "SI {:?} seed {} violations {} max y {} vy {}",
        mode,
        seed,
        stats.violations,
        stats.ty.ulps,
        stats.vy.ulps,
    );
    stats
}

#[test]
fn test_full_engine_and_reverse_rust_control() {
    let stats = recovery(Mode::Engine, false);
    assert_eq!(stats.violations, 4);
    assert!(stats.ty.ulps > 1590000000 && stats.ty.ulps < 1592000000);
}

#[test]
fn test_full_delay_ground_recovery() {
    let stats = recovery(Mode::DelayGround, false);
    assert_eq!(stats.violations, 0);
    assert_eq!(stats.ty.ulps, 53057);
    assert_eq!(stats.vy.ulps, 52326);
}

#[test]
fn test_seeded_delay_ground_recovery() {
    let stats = recovery(Mode::DelayGround, true);
    assert_eq!(stats.violations, 0);
    assert_eq!(stats.ty.ulps, 50);
    assert_eq!(stats.vy.ulps, 81);
}

#[test]
fn test_seeded_control() {
    let stats = recovery(Mode::Engine, true);
    assert_eq!(stats.violations, 4);
    assert_eq!(stats.ty.ulps, 1591205862);
}

/// Replacing the incident geometry or the retained ground cache does not recover the gap.
#[test]
fn test_geometry_control() {
    let stats = recovery(Mode::ReferenceGeometry, true);
    assert_eq!(stats.violations, 4);
    assert_eq!(stats.ty.ulps, 1591205863);
}

#[test]
fn test_cache_control() {
    let stats = recovery(Mode::ReferenceCache, false);
    assert_eq!(stats.violations, 4);
    assert_eq!(stats.ty.ulps, 1591205861);
}

/// These interventions are exact no-ops on the solver state: zero recovery at every later step.
#[test]
fn test_wake_velocity_and_restitution_controls() {
    for mode in array![Mode::WeakToucher, Mode::ZeroSleepingVelocity, Mode::OldContact] {
        let mut control = seeded();
        let mut engine = seeded();
        let mut step = 80;
        while step != 86 {
            let _ = control.step();
            let _ = engine.step();
            step += 1;
        }
        staged(ref control, mode);
        staged(ref engine, Mode::Engine);
        for i in array![1, 2] {
            assert_eq!(control.body(body_handle(i)), engine.body(body_handle(i)));
        }
        // Clearing NEW only changes that consumed solver tag; after regeneration it agrees too.
        let _ = control.step();
        let _ = engine.step();
        assert_eq!(control.bodies.iter().span(), engine.bodies.iter().span());
        assert_eq!(control.narrow_phase.pairs.span(), engine.narrow_phase.pairs.span());
    }
}

/// Ensure the diagnostic stage split reproduces the actual fused wake step bit for bit.
#[test]
fn test_staged_matches_engine() {
    let mut staged_world = seeded();
    let mut fused_world = seeded();
    let mut step = 80;
    while step != 86 {
        let _ = staged_world.step();
        let _ = fused_world.step();
        step += 1;
    }
    staged(ref staged_world, Mode::Engine);
    let _ = fused_world.step();
    assert_eq!(staged_world.bodies.iter().span(), fused_world.bodies.iter().span());
    assert_eq!(staged_world.narrow_phase.pairs.span(), fused_world.narrow_phase.pairs.span());
}
