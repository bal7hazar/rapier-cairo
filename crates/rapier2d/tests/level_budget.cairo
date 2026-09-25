//! G0: the cost of a level, in Sierra gas and Cairo steps. The levels are the golden ones
//! (`golden_scenes::levels`: 10 or 20 sleeping blocks, 3 cores, an 18.4 m/s pebble, the game's
//! despawn rule), loaded as the game loads them; one tick is `World::step` plus the despawn
//! check.
//!
//! Probes run by CI (`gas_*` with a Sierra-gas ceiling of measured + 10 %, and their uncapped
//! `steps_*` twins for `--tracked-resource cairo-steps`): the level load, the pebble's flight
//! (ticks 1–25, the structure asleep), and the flight plus the first five impact ticks (26–30).
//! Subtracting them gives the load, a flight tick and an impact tick.
//!
//! A whole run (300 ticks at 60 Hz, 150 at 30 Hz) exceeds snforge's default step limit, so the
//! matrix is `#[ignore]`d: `run_*` (one run per level and setting, measured with both tracked
//! resources) and `profile_*` (the per-tick gas, awake bodies, sleep and calm-rule ticks, and
//! the per-stage split of the first 60 ticks). Reproduce with
//! `snforge test -p rapier2d level_budget --include-ignored --max-n-steps 4000000000`
//! (add `--tracked-resource cairo-steps` for steps); the figures are in `docs/BUDGETS.md`.

use core::testing::get_available_gas;
use rapier2d::dispatcher::DefaultDispatcher;
use rapier2d::pipeline::{
    collision_inputs_sleeping, merge_pairs, solve_and_advance_sleeping, split_dormant,
    update_islands, user_changes_bodies,
};
use rapier2d::prelude::{Fixed, Handle, RigidBodyTrait, World, WorldTrait};
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::joint::ImpulseJointSetTrait;
use rapier_dynamics2d::narrow_phase::{ContactDispatcher, compute_contacts_from_scratch};
use rapier_dynamics2d::rigid_body_set::RigidBody;
use rapier_dynamics2d::solver::body_store::SolverBodyStoreTrait;
use rapier_dynamics2d::solver::contact::ContactConstraintsSetTrait;
use rapier_dynamics2d::solver::island::solve_island;
use rapier_geometry2d::broad_phase::find_pairs;
use rapier_geometry2d::contact::ContactManifold;
use rapier_geometry2d::shape::Shape;
use rapier_golden::generated::level_scenes;
use rapier_math::pose2::Pose2;
use rapier_testing::opaque;
use crate::golden_scenes::levels::{awake_count, despawn, handle, level, load_level, tick};

/// Ticks of the pebble's flight (its first contact is at tick 26 in every 60 Hz setting).
const FLIGHT: u32 = 25;
/// Flight plus the first five impact ticks.
const IMPACT: u32 = 30;

/// `(dt, solver iterations, ticks of a run)` of setting `k`: 60 Hz with 4, 2, 1 iterations,
/// then 30 Hz with 4.
fn setting(k: u32) -> (i64, u32, u32) {
    if k == 0 {
        (level_scenes::level10_hz60_sub4::DT, 4, 300)
    } else if k == 1 {
        (level_scenes::level10_hz60_sub2::DT, 2, 300)
    } else if k == 2 {
        (level_scenes::level10_hz60_sub1::DT, 1, 300)
    } else {
        (level_scenes::level10_hz30_sub4::DT, 4, 150)
    }
}

/// Ticks of a whole run under setting `k` (5 simulated seconds).
fn ticks_of(k: u32) -> u32 {
    let (_, _, ticks) = setting(k);
    ticks
}

/// Loads level `blocks` under setting `k` and runs `ticks` ticks.
fn run(blocks: u32, k: u32, ticks: u32) -> World {
    let (bodies, linvel, bounds) = level(blocks);
    let (dt, iterations, _) = setting(k);
    let n = bodies.len();
    let mut world = load_level(bodies, linvel, dt, iterations);
    let mut t = 0;
    while t != ticks {
        tick(ref world, n, bounds);
        t += 1;
    }
    world
}

fn probe(blocks: u32, k: u32, ticks: u32) {
    let world = run(opaque(blocks), opaque(k), opaque(ticks));
    let _ = opaque(world.gravity);
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
#[available_gas(l2_gas: 97082438)]
fn gas_load_level10() {
    probe(10, 0, 0);
}

#[test]
#[available_gas(l2_gas: 278333479)]
fn gas_flight_level10() {
    probe(10, 0, FLIGHT);
}

#[test]
#[available_gas(l2_gas: 772587322)]
fn gas_impact_level10() {
    probe(10, 0, IMPACT);
}

#[test]
#[available_gas(l2_gas: 177012177)]
fn gas_load_level20() {
    probe(20, 0, 0);
}

#[test]
#[available_gas(l2_gas: 479231241)]
fn gas_flight_level20() {
    probe(20, 0, FLIGHT);
}

#[test]
#[available_gas(l2_gas: 1015399572)]
fn gas_impact_level20() {
    probe(20, 0, IMPACT);
}

#[test]
fn steps_load_level10() {
    probe(10, 0, 0);
}

#[test]
fn steps_flight_level10() {
    probe(10, 0, FLIGHT);
}

#[test]
fn steps_impact_level10() {
    probe(10, 0, IMPACT);
}

#[test]
fn steps_load_level20() {
    probe(20, 0, 0);
}

#[test]
fn steps_flight_level20() {
    probe(20, 0, FLIGHT);
}

#[test]
fn steps_impact_level20() {
    probe(20, 0, IMPACT);
}

// --- The matrix: whole runs (ignored, see the module documentation). ------------------------

fn full(blocks: u32, k: u32) {
    probe(blocks, k, ticks_of(k));
}

/// Ticks of the prefix measured when a whole run does not fit the machine (20 blocks at 60 Hz
/// with 4 or 2 iterations: the VM holds about 73 bytes per Cairo step, 15 GB for 150 ticks).
const PREFIX: u32 = 150;

#[test]
#[ignore]
fn run_level10_hz60_sub4() {
    full(10, 0);
}

#[test]
#[ignore]
fn run_level10_hz60_sub2() {
    full(10, 1);
}

#[test]
#[ignore]
fn run_level10_hz60_sub1() {
    full(10, 2);
}

#[test]
#[ignore]
fn run_level10_hz30_sub4() {
    full(10, 3);
}

#[test]
#[ignore]
fn run_level20_hz60_sub4() {
    full(20, 0);
}

#[test]
#[ignore]
fn run_level20_hz60_sub4_prefix() {
    probe(20, 0, PREFIX);
}

#[test]
#[ignore]
fn run_level20_hz60_sub2() {
    full(20, 1);
}

#[test]
#[ignore]
fn run_level20_hz60_sub2_prefix() {
    probe(20, 1, PREFIX);
}

#[test]
#[ignore]
fn run_level20_hz60_sub1() {
    full(20, 2);
}

#[test]
#[ignore]
fn run_level20_hz30_sub4() {
    full(20, 3);
}

// --- Per-tick profile: gas of each tick, awake bodies, sleep and calm ticks. ---------------

/// Is every present dynamic body asleep or below the calm thresholds?
fn calm(ref world: World, n: u32) -> bool {
    let lin = Fixed { raw: level_scenes::CALM_LINEAR };
    let ang = Fixed { raw: level_scenes::CALM_ANGULAR };
    let (lin2, ang2) = (lin * lin, ang * ang);
    let mut i = 1;
    while i != n {
        if let Some(rb) = world.body(handle(i)) {
            if !rb.is_sleeping() {
                let v = rb.linvel();
                let w = rb.vels.angvel;
                if !((v.x * v.x + v.y * v.y) < lin2 && (w * w) < ang2) {
                    return false;
                }
            }
        }
        i += 1;
    }
    true
}

/// Runs level `blocks` under setting `k`, measuring the Sierra gas of every tick (step and
/// despawn check, nothing else); prints one `tick gas awake` line per tick, then the summary
/// (the calm-rule check is measured apart, `calm_gas`). With `fast_sleep` (the engine-side
/// alternative to the calm rule), every body gets `time_until_sleep` = the calm window, 20 ticks,
/// instead of the default half second.
fn profile(blocks: u32, k: u32, ticks: u32, fast_sleep: bool) {
    let (bodies, linvel, bounds) = level(blocks);
    let (dt, iterations, _) = setting(k);
    let n = bodies.len();
    let before = get_available_gas();
    let mut world = load_level(bodies, linvel, dt, iterations);
    let load = before - get_available_gas();
    if fast_sleep {
        let mut i = 1;
        while i != n {
            let mut rb = world.body(handle(i)).unwrap();
            rb.activation.time_until_sleep = Fixed { raw: dt * level_scenes::CALM_TICKS.into() };
            assert!(world.set_body(handle(i), rb));
            i += 1;
        }
    }
    let mut calm_gas: i128 = 0;
    let mut total: u128 = 0;
    let mut max_gas: u128 = 0;
    let mut awake_ticks: u32 = 0;
    let mut max_awake: u32 = 0;
    let mut all_asleep: u32 = 0;
    let mut calm_run: u32 = 0;
    let mut calm_end: u32 = 0;
    let mut gas_at_calm: u128 = 0;
    let mut gas_at_asleep: u128 = 0;
    let mut sleeping_gas: u128 = 0;
    let mut sleeping_ticks: u32 = 0;
    let mut despawn_gas: i128 = 0;
    let mut t: u32 = 0;
    while t != ticks {
        let before = get_available_gas();
        let _ = world.step();
        let middle = get_available_gas();
        let _ = despawn(ref world, n, bounds);
        let after = get_available_gas();
        despawn_gas += spent(middle, after);
        let gas = before - after;
        t += 1;
        total += gas;
        if gas > max_gas {
            max_gas = gas;
        }
        let awake = awake_count(ref world, n);
        awake_ticks += awake;
        if awake > max_awake {
            max_awake = awake;
        }
        if awake == 0 {
            sleeping_gas += gas;
            sleeping_ticks += 1;
        }
        if all_asleep == 0 && awake == 0 {
            all_asleep = t;
            gas_at_asleep = total;
        }
        let before = get_available_gas();
        let is_calm = calm(ref world, n);
        calm_gas += spent(before, get_available_gas());
        calm_run = if is_calm {
            calm_run + 1
        } else {
            0
        };
        if calm_end == 0 && calm_run == level_scenes::CALM_TICKS {
            calm_end = t;
            gas_at_calm = total;
        }
        println!("tick {} gas {} awake {}", t, gas, awake);
    }
    println!(
        "summary blocks {} setting {} load {} total {} max_tick {} awake_ticks {} max_awake {} all_asleep {} gas_at_asleep {} calm_end {} gas_at_calm {} sleeping_ticks {} sleeping_gas {} despawn_gas {} calm_gas {}",
        blocks,
        k,
        load,
        total,
        max_gas,
        awake_ticks,
        max_awake,
        all_asleep,
        gas_at_asleep,
        calm_end,
        gas_at_calm,
        sleeping_ticks,
        sleeping_gas,
        despawn_gas,
        calm_gas,
    );
}

#[test]
#[ignore]
fn profile_level10_hz60_sub4() {
    profile(10, 0, ticks_of(0), false);
}

#[test]
#[ignore]
fn profile_level10_hz60_sub2() {
    profile(10, 1, ticks_of(1), false);
}

#[test]
#[ignore]
fn profile_level10_hz60_sub1() {
    profile(10, 2, ticks_of(2), false);
}

#[test]
#[ignore]
fn profile_level10_hz30_sub4() {
    profile(10, 3, ticks_of(3), false);
}

#[test]
#[ignore]
fn profile_level10_hz60_sub4_fast_sleep() {
    profile(10, 0, ticks_of(0), true);
}

#[test]
#[ignore]
fn profile_level10_hz30_sub4_fast_sleep() {
    profile(10, 3, ticks_of(3), true);
}

#[test]
#[ignore]
fn profile_level20_hz60_sub1_fast_sleep() {
    profile(20, 2, ticks_of(2), true);
}

#[test]
#[ignore]
fn profile_level20_hz60_sub4() {
    profile(20, 0, ticks_of(0), false);
}

#[test]
#[ignore]
fn profile_level20_hz60_sub4_prefix() {
    profile(20, 0, PREFIX, false);
}

#[test]
#[ignore]
fn profile_level20_hz60_sub2() {
    profile(20, 1, ticks_of(1), false);
}

#[test]
#[ignore]
fn profile_level20_hz60_sub2_prefix() {
    profile(20, 1, PREFIX, false);
}

#[test]
#[ignore]
fn profile_level20_hz60_sub1() {
    profile(20, 2, ticks_of(2), false);
}

#[test]
#[ignore]
fn profile_level20_hz30_sub4() {
    profile(20, 3, ticks_of(3), false);
}

// --- Per-stage split of the first 60 ticks. -------------------------------------------------

/// Gas of the stages of one step: user changes, broad phase (proxies, dormant split,
/// `find_pairs`), narrow phase, islands and sleeping bookkeeping (island update, pair revival
/// and merge), solver (solve and position update).
#[derive(Copy, Drop, Default)]
struct Stages {
    user_changes: i128,
    broad: i128,
    narrow: i128,
    islands: i128,
    solver: i128,
}

/// Gas spent between two readings; negative when the stretch ended with a refund (a skipped
/// branch whose cost was withdrawn in advance).
fn spent(before: u128, after: u128) -> i128 {
    before.try_into().unwrap() - after.try_into().unwrap()
}

/// `World::step` (collision events only, no force-event collider) written out with the public
/// stage functions of `rapier2d::pipeline`, in the same order and with the same inputs, and a
/// gas reading between the stages. The world contains no kinematic body, so
/// `user_changes_bodies` is the step's own user-change stage.
fn profiled_step(ref world: World, ref stages: Stages, upto: u8, solver: u8) {
    let mut g = get_available_gas();
    let (snapshot, infos, entries, census) = user_changes_bodies(
        ref world.bodies, ref world.colliders, world.narrow_phase.pairs.span(),
    );
    let mut now = get_available_gas();
    stages.user_changes += spent(g, now);
    g = now;
    if upto == 1 {
        return;
    }
    let prediction = world.integration_parameters.prediction_distance();
    let (proxies, scratch, sleeping) = collision_inputs_sleeping(
        snapshot, infos, ref world.bodies, prediction,
    );
    let mut dormant = array![];
    if sleeping {
        let (active, asleep) = split_dormant(world.narrow_phase.pairs.span(), entries);
        world.narrow_phase.pairs = active;
        dormant = asleep;
    }
    let pairs = find_pairs(proxies.span());
    now = get_available_gas();
    stages.broad += spent(g, now);
    g = now;
    if upto == 2 {
        return;
    }
    if upto == 3 && solver == 1 {
        let _ = compute_contacts_from_scratch::<
            KeepDispatcher,
        >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
    } else if upto == 3 && solver == 3 {
        let _ = compute_contacts_from_scratch::<
            TmpCc,
        >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
    } else if upto == 3 && solver == 4 {
        let _ = compute_contacts_from_scratch::<
            TmpHalf,
        >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
    } else if upto == 3 && solver == 5 {
        let _ = compute_contacts_from_scratch::<
            TmpBall,
        >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
    } else if upto == 3 && solver == 2 {
        let _ = compute_contacts_from_scratch::<
            NullDispatcher,
        >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
    } else {
        let _ = compute_contacts_from_scratch::<
            DefaultDispatcher,
        >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
    }
    now = get_available_gas();
    stages.narrow += spent(g, now);
    g = now;
    if upto == 3 {
        return;
    }
    let joint_entries = world.impulse_joints.to_array();
    let (entries, sleeping, woken) = update_islands(
        ref world.bodies,
        world.narrow_phase.pairs.span(),
        dormant.span(),
        joint_entries.span(),
        entries,
        census,
    );
    if woken && !dormant.is_empty() {
        let (revived, asleep) = split_dormant(dormant.span(), entries);
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), revived.span());
        dormant = asleep;
    }
    now = get_available_gas();
    stages.islands += spent(g, now);
    g = now;
    if upto == 4 {
        if solver != 0 {
            solver_parts(ref world, entries, sleeping, solver);
        }
        return;
    }
    solve_and_advance_sleeping(
        world.gravity,
        world.integration_parameters,
        ref world.bodies,
        ref world.colliders,
        ref world.narrow_phase,
        ref world.impulse_joints,
        entries,
        snapshot,
        joint_entries.span(),
        sleeping,
    );
    now = get_available_gas();
    stages.solver += spent(g, now);
    g = now;
    if !dormant.is_empty() {
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
    }
    stages.islands += spent(g, get_available_gas());
}

/// The per-stage split of the first 60 ticks of level `blocks` at 60 Hz / 4 iterations; checks
/// that the written-out step leaves the same world as `World::step`.
fn stages(blocks: u32) {
    let (bodies, linvel, bounds) = level(blocks);
    let (dt, iterations, _) = setting(0);
    let n = bodies.len();
    let mut world = load_level(bodies, linvel, dt, iterations);
    let mut reference = load_level(bodies, linvel, dt, iterations);
    let mut stages: Stages = Default::default();
    let mut windows = array![];
    let mut t = 0;
    while t != 60 {
        profiled_step(ref world, ref stages, 5, 0);
        let _ = despawn(ref world, n, bounds);
        tick(ref reference, n, bounds);
        t += 1;
        if t == 25 || t == 60 {
            windows.append(stages);
        }
    }
    let mut i = 1;
    while i != n {
        let (a, b) = (world.body(handle(i)), reference.body(handle(i)));
        assert!(a.is_some() == b.is_some(), "presence of body {}", i);
        if let Some(a) = a {
            let b = b.unwrap();
            assert!(a.position() == b.position() && a.linvel() == b.linvel(), "body {}", i);
            assert!(a.vels.angvel == b.vels.angvel && a.is_sleeping() == b.is_sleeping());
        }
        i += 1;
    }
    for s in windows {
        println!(
            "stages blocks {} user_changes {} broad {} narrow {} islands {} solver {}",
            blocks,
            s.user_changes,
            s.broad,
            s.narrow,
            s.islands,
            s.solver,
        );
    }
}

#[test]
#[ignore]
fn profile_stages_level10() {
    stages(10);
}

#[test]
#[ignore]
fn profile_stages_level20() {
    stages(20);
}

/// Narrow-phase stubs of the stage probes: keep the previous manifold, or report an unsupported
/// pair (the narrow phase clears its manifold).
impl KeepDispatcher of ContactDispatcher {
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        true
    }
}
impl NullDispatcher of ContactDispatcher {
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        false
    }
}

// --- Stage probes (BT1): exact Cairo steps of one impact tick, by stage and solver part. ----

/// The tick the stage probes measure (the third impact tick of every 60 Hz setting).
const STAGE_TICK: u32 = 28;

/// The solver input of `solve_and_advance_sleeping` (the level has no joint): the touching
/// manifolds and the bodies they reference, sleeping ones as immovable copies. The manifold
/// order differs from the pipeline's (no fixed-last partition), not the work.
fn solver_parts(ref world: World, entries: Span<(Handle, RigidBody)>, sleeping: bool, part: u8) {
    let mut params = world.integration_parameters;
    let mut manifolds = array![];
    let mut points = 0;
    for pair in world.narrow_phase.pairs.span() {
        if *pair.manifold.data.num_solver_contacts != 0 {
            manifolds.append(*pair.manifold);
            points += (*pair.manifold.data.num_solver_contacts).into();
        }
    }
    let mut members = array![];
    let mut awake = 0;
    for entry in entries {
        let (handle, body) = entry;
        let mut referenced = false;
        for m in manifolds.span() {
            if *m.data.rigid_body1 == Some(*handle) || *m.data.rigid_body2 == Some(*handle) {
                referenced = true;
            }
        }
        if referenced {
            let mut body = *body;
            if sleeping && body.activation.sleeping {
                body.enabled = false;
            } else if body.enabled && RigidBodyTrait::is_dynamic(@body) {
                awake += 1;
            }
            members.append((*handle, body));
        }
    }
    println!(
        "solver input manifolds {} points {} members {} awake {}",
        manifolds.len(),
        points,
        members.len(),
        awake,
    );
    let mut store = SolverBodyStoreTrait::from_entries(members.span(), world.gravity, params);
    let mut joints = array![];
    if part == 2 {
        let mut bodies = array![];
        let mut i = 0;
        while i != store.len() {
            bodies.append(store.get(i));
            i += 1;
        }
        let cs = ContactConstraintsSetTrait::generate(
            manifolds.span(), bodies.span(), params, params.substep_dt(),
        );
        let _ = opaque(cs.constraints.len());
        return;
    }
    if part == 4 {
        params.num_solver_iterations = 3;
    } else if part == 5 {
        params.num_internal_pgs_iterations = 2;
    } else if part == 6 {
        params.num_internal_stabilization_iterations = 2;
    } else if part == 7 {
        params.num_solver_iterations = 1;
    } else if part == 8 {
        params.num_internal_pgs_iterations = 3;
    } else if part == 9 {
        params.num_internal_stabilization_iterations = 3;
    } else if part == 10 {
        params.num_solver_iterations = 2;
    } else if part == 11 {
        params.num_internal_pgs_iterations = 0;
    }
    if part >= 3 {
        solve_island(params, ref store, ref manifolds, ref joints);
    }
    let _ = opaque(manifolds.len());
}

/// Level `blocks` run to the tick before `STAGE_TICK`, then that tick's stages up to `upto`
/// (1 user changes, 2 broad phase, 3 narrow phase, 4 islands, 5 solver and position update);
/// with `upto == 4`, `part` runs a piece of the solver on the tick's solver input: 1 the input
/// alone, 2 constraint generation, 3 `solve_island`, 4 with 3 substeps, 5 with two biased
/// sweeps, 6 with two relaxation sweeps, 7 with 1 substep.
fn stage(blocks: u32, upto: u8, part: u8) {
    let (bodies, _, bounds) = level(blocks);
    let n = bodies.len();
    let mut world = run(opaque(blocks), 0, opaque(STAGE_TICK - 1));
    if upto != 0 {
        let mut stages: Stages = Default::default();
        profiled_step(ref world, ref stages, opaque(upto), opaque(part));
    }
    let _ = opaque((world.gravity, n, bounds));
}

// Reproduce: `snforge test -p rapier2d level_budget::stage --include-ignored --detailed-resources
// --tracked-resource cairo-steps`; differences of neighbours give each stage and part.

#[test]
#[ignore]
fn stage10_setup() {
    stage(10, 0, 0);
}

#[test]
#[ignore]
fn stage10_user_changes() {
    stage(10, 1, 0);
}

#[test]
#[ignore]
fn stage10_broad() {
    stage(10, 2, 0);
}

#[test]
#[ignore]
fn stage10_narrow() {
    stage(10, 3, 0);
}

#[test]
#[ignore]
fn stage10_islands() {
    stage(10, 4, 0);
}

#[test]
#[ignore]
fn stage10_solver() {
    stage(10, 5, 0);
}

#[test]
#[ignore]
fn solver10_input() {
    stage(10, 4, 1);
}

#[test]
#[ignore]
fn solver10_generate() {
    stage(10, 4, 2);
}

#[test]
#[ignore]
fn solver10_island() {
    stage(10, 4, 3);
}

#[test]
#[ignore]
fn solver10_sub3() {
    stage(10, 4, 4);
}

#[test]
#[ignore]
fn solver10_biased2() {
    stage(10, 4, 5);
}

#[test]
#[ignore]
fn solver10_relax2() {
    stage(10, 4, 6);
}

#[test]
#[ignore]
fn solver10_sub1() {
    stage(10, 4, 7);
}

#[test]
#[ignore]
fn stage20_setup() {
    stage(20, 0, 0);
}

#[test]
#[ignore]
fn stage20_user_changes() {
    stage(20, 1, 0);
}

#[test]
#[ignore]
fn stage20_broad() {
    stage(20, 2, 0);
}

#[test]
#[ignore]
fn stage20_narrow() {
    stage(20, 3, 0);
}

#[test]
#[ignore]
fn stage20_islands() {
    stage(20, 4, 0);
}

#[test]
#[ignore]
fn stage20_solver() {
    stage(20, 5, 0);
}

#[test]
#[ignore]
fn solver20_input() {
    stage(20, 4, 1);
}

#[test]
#[ignore]
fn solver20_generate() {
    stage(20, 4, 2);
}

#[test]
#[ignore]
fn solver20_island() {
    stage(20, 4, 3);
}

#[test]
#[ignore]
fn solver20_sub3() {
    stage(20, 4, 4);
}

#[test]
#[ignore]
fn solver20_biased2() {
    stage(20, 4, 5);
}

#[test]
#[ignore]
fn solver20_relax2() {
    stage(20, 4, 6);
}

#[test]
#[ignore]
fn solver20_sub1() {
    stage(20, 4, 7);
}

#[test]
#[ignore]
fn solver10_tmp_pgs3() {
    stage(10, 4, 8);
}

#[test]
#[ignore]
fn solver10_tmp_stab3() {
    stage(10, 4, 9);
}

#[test]
#[ignore]
fn solver10_tmp_sub2() {
    stage(10, 4, 10);
}

#[test]
#[ignore]
fn solver10_tmp_pgs0() {
    stage(10, 4, 11);
}

/// Poseidon digest of the serialized world state after the impact window of level `blocks`.
fn impact_digest(blocks: u32) -> felt252 {
    let world = run(blocks, 0, IMPACT);
    let mut out = array![];
    world.into_state().serialize(ref out);
    core::poseidon::poseidon_hash_span(out.span())
}

#[test]
fn test_impact_digest_level10() {
    assert_eq!(
        impact_digest(10),
        2461782582709462485446536548234317870668566793754787276166295666873025636411,
    );
}

#[test]
fn test_impact_digest_level20() {
    assert_eq!(
        impact_digest(20),
        2536114172100514642348097745032330135272153081367581830010923825855114040884,
    );
}

#[test]
#[ignore]
fn stage10_narrow_keep() {
    stage(10, 3, 1);
}

#[test]
#[ignore]
fn stage10_narrow_null() {
    stage(10, 3, 2);
}

#[test]
#[ignore]
fn tmp_pairs() {
    let mut world = run(10, 0, STAGE_TICK - 1);
    for pair in world.narrow_phase.pairs.span() {
        let c1 = world.colliders.get(*pair.collider1).unwrap();
        let c2 = world.colliders.get(*pair.collider2).unwrap();
        let k1: felt252 = match c1.shape {
            Shape::Ball(_) => 'ball',
            Shape::Cuboid(_) => 'cuboid',
            Shape::HalfSpace(_) => 'half',
            Shape::ConvexPolygon(_) => 'poly',
            _ => 'other',
        };
        let k2: felt252 = match c2.shape {
            Shape::Ball(_) => 'ball',
            Shape::Cuboid(_) => 'cuboid',
            Shape::HalfSpace(_) => 'half',
            Shape::ConvexPolygon(_) => 'poly',
            _ => 'other',
        };
        println!(
            "pair {} {} points {} solver {}",
            k1,
            k2,
            *pair.manifold.num_points,
            *pair.manifold.data.num_solver_contacts,
        );
    }
}

impl TmpCc of ContactDispatcher {
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        match (shape1, shape2) {
            (
                Shape::Cuboid(_), Shape::Cuboid(_),
            ) => rapier_geometry2d::dispatch::contact_manifold_step(
                pos12, shape1, shape2, prediction, ref manifold,
            ),
            _ => true,
        }
    }
}
impl TmpHalf of ContactDispatcher {
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        match shape1 {
            Shape::HalfSpace(_) => rapier_geometry2d::dispatch::contact_manifold_step(
                pos12, shape1, shape2, prediction, ref manifold,
            ),
            _ => true,
        }
    }
}
impl TmpBall of ContactDispatcher {
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        match (shape1, shape2) {
            (
                Shape::Cuboid(_), Shape::Ball(_),
            ) => rapier_geometry2d::dispatch::contact_manifold_step(
                pos12, shape1, shape2, prediction, ref manifold,
            ),
            _ => true,
        }
    }
}

#[test]
#[ignore]
fn stage10_narrow_tmp_cc() {
    stage(10, 3, 3);
}

#[test]
#[ignore]
fn stage10_narrow_tmp_half() {
    stage(10, 3, 4);
}

#[test]
#[ignore]
fn stage10_narrow_tmp_ball() {
    stage(10, 3, 5);
}
