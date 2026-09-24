//! Unit tests of the pipeline stages, and equivalence of every candidate in `alternatives` with
//! the shipped stage (same state in, same state and events out).

use fixed::{HALF, ONE, ZERO};
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_dynamics2d::collider::{ColliderBuilderTrait, ColliderTrait};
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::joint::ImpulseJointSetTrait;
use rapier_dynamics2d::narrow_phase::{ContactPair, NarrowPhaseTrait, pair_colliders};
use rapier_dynamics2d::rigid_body::RigidBodyMassPropsTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBodySetTrait, RigidBodyTrait};
use rapier_geometry2d::broad_phase::find_pairs;
use rapier_geometry2d::contact::ContactManifold;
use rapier_golden::scenes;
use crate::dispatcher::DefaultDispatcher;
use crate::world::{World, WorldTrait};
use super::alternatives::{
    InlineDispatcher, OutlinedDispatcher, ProxyCacheTrait, compute_contacts_bucketed,
    compute_contacts_by_kind, compute_contacts_metered, handle_user_changes_propagate,
    solve_all_manifolds, step_with_cache,
};
use super::fixtures::{at, ball_on_ground, mixed, random_world, row, scene_world, statics, v};
use super::fused_alternatives::{
    advance_with_snapshot_outlined, collision_inputs_field_reads,
    collision_inputs_outlined_fallback, step_staged,
};
use super::{
    advance_with_snapshot, collision_inputs, contacts_from_scratch, handle_user_changes,
    scatter_touching, solve, touching_manifolds, user_changes_snapshot,
};

fn world_of(id: felt252) -> World {
    if id == 'stack3' {
        let (world, _) = scene_world(scenes::BOX_STACK3);
        world
    } else if id == 'balls' {
        row(5, true)
    } else if id == 'cuboids' {
        row(3, false)
    } else if id == 'mixed' {
        mixed()
    } else if id == 'statics' {
        statics(4)
    } else if id == 'pendulum' {
        let (world, _) = scene_world(scenes::PENDULUM);
        world
    } else {
        let (world, _, _) = ball_on_ground(ONE, true);
        world
    }
}

/// Bodies, colliders and pairs of both worlds are identical.
fn same_state(ref a: World, ref b: World) -> bool {
    a.bodies.iter() == b.bodies.iter()
        && a.colliders.iter() == b.colliders.iter()
        && a.narrow_phase.pairs == b.narrow_phase.pairs
}

/// Stages 1–2 of a step, returning the broad-phase pairs.
fn prepare(ref world: World) -> Array<(u32, u32)> {
    handle_user_changes(ref world.bodies, ref world.colliders);
    let prediction = world.integration_parameters.prediction_distance();
    find_pairs(world.colliders.broad_phase_proxies(ref world.bodies, prediction).span())
}

/// The narrow phase of candidate `variant` on `world` (0 = shipped).
fn narrow_phase(ref world: World, pairs: Span<(u32, u32)>, variant: u8) -> Array<CollisionEvent> {
    let p = world.integration_parameters.prediction_distance();
    if variant == 1 {
        compute_contacts_by_kind(
            ref world.narrow_phase, p, ref world.bodies, ref world.colliders, pairs,
        )
    } else if variant == 2 {
        compute_contacts_bucketed(
            ref world.narrow_phase, p, ref world.bodies, ref world.colliders, pairs,
        )
    } else if variant == 3 {
        compute_contacts_metered(
            ref world.narrow_phase, p, ref world.bodies, ref world.colliders, pairs,
        )
    } else if variant == 4 {
        world
            .narrow_phase
            .compute_contacts::<InlineDispatcher>(p, ref world.bodies, ref world.colliders, pairs)
    } else if variant == 5 {
        world
            .narrow_phase
            .compute_contacts::<OutlinedDispatcher>(p, ref world.bodies, ref world.colliders, pairs)
    } else {
        world
            .narrow_phase
            .compute_contacts::<DefaultDispatcher>(p, ref world.bodies, ref world.colliders, pairs)
    }
}

/// Every narrow-phase candidate gives the shipped pairs (manifolds, warm-start data, event
/// status) and events, on scenes covering ball–ball, cuboid–cuboid, ball–half-space and
/// non-touching pairs, over steps that start and keep contacts.
#[test]
fn test_narrow_phase_candidates_agree_on_rows() {
    narrow_phase_candidates_agree(array!['balls', 'cuboids'].span());
}

#[test]
fn test_narrow_phase_candidates_agree_on_ground() {
    narrow_phase_candidates_agree(array!['mixed', 'resting'].span());
}

fn narrow_phase_candidates_agree(ids: Span<felt252>) {
    for id in ids {
        let mut variant = 1;
        while variant != 6 {
            let mut shipped = world_of(*id);
            let mut candidate = world_of(*id);
            let mut step = 0;
            while step != 2 {
                let pairs = prepare(ref shipped);
                let _ = prepare(ref candidate);
                let expected = narrow_phase(ref shipped, pairs.span(), 0);
                let got = narrow_phase(ref candidate, pairs.span(), variant);
                assert!(got == expected, "{} variant {} events", *id, variant);
                assert!(same_state(ref shipped, ref candidate), "{} variant {}", *id, variant);
                let _ = shipped.step();
                let _ = candidate.step();
                step += 1;
            }
            variant += 1;
        }
    }
}

/// Handing every manifold to the solver changes nothing but the cost: non-touching manifolds
/// produce inert constraints.
#[test]
fn test_all_manifolds_solver_agrees() {
    for id in array!['mixed', 'stack3', 'pendulum'].span() {
        let mut shipped = world_of(*id);
        let mut candidate = world_of(*id);
        let mut step = 0;
        while step != 3 {
            let pairs = prepare(ref shipped);
            let _ = prepare(ref candidate);
            let _ = narrow_phase(ref shipped, pairs.span(), 0);
            let _ = narrow_phase(ref candidate, pairs.span(), 0);
            let (g, p) = (shipped.gravity, shipped.integration_parameters);
            solve(g, p, ref shipped.bodies, ref shipped.narrow_phase, ref shipped.impulse_joints);
            solve_all_manifolds(
                g,
                p,
                ref candidate.bodies,
                ref candidate.narrow_phase,
                ref candidate.impulse_joints,
            );
            assert!(same_state(ref shipped, ref candidate), "{} step {}", *id, step);
            assert!(shipped.impulse_joints.to_array() == candidate.impulse_joints.to_array());
            super::advance_to_final_positions(ref shipped.bodies, ref shipped.colliders);
            super::advance_to_final_positions(ref candidate.bodies, ref candidate.colliders);
            step += 1;
        }
    }
}

/// DD's `propagate_modified_body_positions_to_colliders` path leaves the same state, on fresh
/// worlds (every flag raised) and after user edits (moved body, added collider).
#[test]
fn test_propagate_user_changes_agree() {
    for id in array!['stack3', 'statics', 'resting'].span() {
        let mut shipped = world_of(*id);
        let mut candidate = world_of(*id);
        let mut round = 0;
        while round != 2 {
            handle_user_changes(ref shipped.bodies, ref shipped.colliders);
            handle_user_changes_propagate(ref candidate.bodies, ref candidate.colliders);
            assert!(same_state(ref shipped, ref candidate), "{} round {}", *id, round);
            // Move the last body and attach a collider to it, in both worlds.
            let (handle, mut body) = *shipped.bodies.iter().at(shipped.bodies.len() - 1);
            body.set_position(at(HALF, ONE + ONE));
            assert!(shipped.set_body(handle, body) && candidate.set_body(handle, body));
            let extra = ColliderBuilderTrait::ball(HALF).translation(v(ONE, ZERO)).build();
            let _ = shipped.insert_collider(extra, Some(handle));
            let _ = candidate.insert_collider(extra, Some(handle));
            round += 1;
        }
    }
}

/// The proxy cache yields the same simulation, including after a static collider is removed
/// and a fixed body is moved (both invalidate it).
#[test]
fn test_proxy_cache_agrees() {
    for id in array!['statics', 'stack3'].span() {
        let mut shipped = world_of(*id);
        let mut candidate = world_of(*id);
        let mut cache = ProxyCacheTrait::new();
        let mut step = 0;
        while step != 6 {
            if step == 2 {
                let (first, _) = *shipped.colliders.iter().at(0);
                let _ = shipped.remove_collider(first);
                let _ = candidate.remove_collider(first);
            }
            if step == 4 {
                let (handle, mut body) = *shipped.bodies.iter().at(1);
                body.set_position(at(ONE, -ONE));
                assert!(shipped.set_body(handle, body) && candidate.set_body(handle, body));
            }
            let expected = shipped.step();
            let got = step_with_cache(ref candidate, ref cache);
            assert!(got == expected, "{} step {} events", *id, step);
            assert!(same_state(ref shipped, ref candidate), "{} step {}", *id, step);
            step += 1;
        }
    }
}

/// `scatter_touching` puts the solved manifolds back on the touching pairs, in order, and
/// leaves the others untouched.
#[test]
fn test_touching_manifolds_round_trip() {
    let mut world = mixed();
    let _ = world.step();
    let pairs = world.narrow_phase.pairs.span();
    assert!(pairs.len() >= 2);
    let touching = touching_manifolds(pairs);
    assert_eq!(touching.len(), 1);
    let mut solved: ContactManifold = *touching.at(0);
    solved.subshape1 = 7;
    let out = scatter_touching(pairs, array![solved].span());
    assert_eq!(out.len(), pairs.len());
    let mut i = 0;
    while i != pairs.len() {
        let (before, after): (ContactPair, ContactPair) = (*pairs.at(i), *out.at(i));
        if before.manifold.data.num_solver_contacts != 0 {
            assert_eq!(after.manifold, solved);
        } else {
            assert!(after == before);
        }
        i += 1;
    }
}

/// A collider added to a body, then disabled, then removed: the mass follows at each step.
#[test]
fn test_mass_follows_collider_changes() {
    let mut world = WorldTrait::new(v(ZERO, ZERO), Default::default());
    let (body, _) = world
        .insert(
            RigidBodyTrait::dynamic(at(ZERO, ZERO)),
            ColliderBuilderTrait::cuboid(HALF, HALF).build(),
        );
    let _ = world.step();
    assert_eq!(world.body(body).unwrap().mprops.mass(), ONE);
    let extra = world
        .insert_collider(
            ColliderBuilderTrait::cuboid(HALF, HALF).translation(v(ONE, ZERO)).build(), Some(body),
        );
    let _ = world.step();
    let rb = world.body(body).unwrap();
    assert_eq!(rb.mprops.mass(), ONE + ONE);
    assert_eq!(rb.mprops.world_com, v(HALF, ZERO));
    let mut collider = world.collider(extra).unwrap();
    collider.set_enabled(false);
    assert!(world.set_collider(extra, collider));
    let _ = world.step();
    let rb = world.body(body).unwrap();
    assert_eq!(rb.mprops.mass(), ONE);
    assert_eq!(rb.mprops.world_com, v(ZERO, ZERO));
    assert_eq!(world.narrow_phase.len(), 0);
}

/// The fused stages build exactly the proxies of `broad_phase_proxies` and the scratch of
/// `pair_colliders`, dense or sparse sets alike.
#[test]
fn test_collision_inputs_match_the_stage_functions() {
    let mut seed: u32 = 0;
    let mut overlaps = 0;
    while seed != 6 {
        let mut world = random_world(seed);
        let mut reference = random_world(seed);
        let (snapshot, infos) = user_changes_snapshot(ref world.bodies, ref world.colliders);
        handle_user_changes(ref reference.bodies, ref reference.colliders);
        assert!(same_state(ref world, ref reference), "seed {} user changes", seed);
        assert!(snapshot == world.colliders.iter().span(), "seed {} snapshot", seed);
        let p = world.integration_parameters.prediction_distance();
        let (proxies, scratch) = collision_inputs(snapshot, infos, ref world.bodies, p);
        let expected = reference.colliders.broad_phase_proxies(ref reference.bodies, p);
        assert!(proxies == expected, "seed {} proxies", seed);
        overlaps += find_pairs(proxies.span()).len();
        let expected = pair_colliders(ref reference.bodies, ref reference.colliders);
        assert!(scratch == expected.span(), "seed {} scratch", seed);
        seed += 1;
    }
    assert!(overlaps != 0);
}

/// `World::step` (fused) and every fused-stage candidate against the staged step, raw compare
/// of bodies, colliders, pairs and events, on scenes with contacts, joints and events.
#[test]
fn test_fused_step_agrees_on_scenes() {
    for id in array!['stack3', 'pendulum', 'mixed', 'statics', 'resting'].span() {
        let mut variant = 0;
        while variant != 4 {
            let mut staged = world_of(*id);
            let mut fused = world_of(*id);
            let mut step = 0;
            while step != 3 {
                let expected = step_staged(ref staged);
                let got = fused_step(ref fused, variant);
                assert!(got == expected, "{} variant {} step {} events", *id, variant, step);
                assert!(
                    same_state(ref staged, ref fused), "{} variant {} step {}", *id, variant, step,
                );
                step += 1;
            }
            variant += 1;
        }
    }
}

/// The fused step against the staged one on random worlds (free slots, stale generations,
/// sensors, disabled colliders, fixed and kinematic parents), with user changes between steps.
#[test]
#[fuzzer(runs: 8, seed: 20260923)]
fn fuzz_fused_step_agrees(seed: u16) {
    let mut staged = random_world(seed.into());
    let mut fused = random_world(seed.into());
    let mut step = 0;
    while step != 3 {
        if step == 1 {
            // Move the last body and disable the first collider, in both worlds.
            let (handle, mut body) = *staged.bodies.iter().at(staged.bodies.len() - 1);
            body.set_position(at(ONE, ONE + ONE));
            assert!(staged.set_body(handle, body) && fused.set_body(handle, body));
            let (handle, mut collider) = *staged.colliders.iter().at(0);
            collider.set_enabled(false);
            assert!(staged.set_collider(handle, collider) && fused.set_collider(handle, collider));
        }
        let expected = step_staged(ref staged);
        let got = fused.step();
        assert!(got == expected, "step {} events", step);
        assert!(same_state(ref staged, ref fused), "step {}", step);
        step += 1;
    }
}

/// One step through the fused stages, `variant` selecting a `fused_alternatives` candidate
/// (0 = shipped `World::step`).
fn fused_step(ref world: World, variant: u8) -> Array<CollisionEvent> {
    if variant == 0 {
        return world.step();
    }
    let (snapshot, infos) = user_changes_snapshot(ref world.bodies, ref world.colliders);
    let p = world.integration_parameters.prediction_distance();
    let (proxies, scratch) = if variant == 1 {
        collision_inputs_outlined_fallback(snapshot, infos, ref world.bodies, p)
    } else if variant == 2 {
        collision_inputs_field_reads(snapshot, infos, ref world.bodies, p)
    } else {
        collision_inputs(snapshot, infos, ref world.bodies, p)
    };
    let pairs = find_pairs(proxies.span());
    let events = contacts_from_scratch(
        ref world.narrow_phase, p, scratch, pairs.span(), ref world.colliders,
    );
    solve(
        world.gravity,
        world.integration_parameters,
        ref world.bodies,
        ref world.narrow_phase,
        ref world.impulse_joints,
    );
    if variant == 3 {
        advance_with_snapshot_outlined(ref world.bodies, ref world.colliders, snapshot);
    } else {
        advance_with_snapshot(ref world.bodies, ref world.colliders, snapshot);
    }
    events
}
