//! Gas probes of the pipeline stages and candidates. Every probe builds a world, steps it
//! `warmup` times, then runs the first `stages` stages of one more step:
//! `gas_<scene>_<stage>` − `gas_<scene>_<previous stage>` is the cost of that stage
//! (`setup` = nothing run). Stages: 1 user changes, 2 broad phase (proxies + `find_pairs`),
//! 3 narrow phase, 4 solver, 5 position update (= a whole step).
//!
//! Candidates are selected by `variant`: see `crate::pipeline::alternatives` and the pipeline
//! module documentation for the ranking.

use fixed::HALF;
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::narrow_phase::NarrowPhaseTrait;
use rapier_geometry2d::broad_phase::find_pairs;
use rapier_golden::scenes;
use rapier_testing::opaque;
use crate::dispatcher::DefaultDispatcher;
use crate::world::{World, WorldTrait};
use super::alternatives::{
    InlineDispatcher, OutlinedDispatcher, ProxyCacheTrait, compute_contacts_bucketed,
    compute_contacts_by_kind, compute_contacts_metered, handle_user_changes_propagate,
    solve_all_manifolds,
};
use super::fixtures::{ball_on_ground, mixed, row, scene_world, statics};
use super::{advance_to_final_positions, handle_user_changes, solve};

/// Shipped pipeline.
const SHIPPED: u8 = 0;
/// Narrow phase through `compute_contacts_by_kind` (one `process_pair` per shape-pair kind).
const BY_KIND: u8 = 1;
/// Narrow phase through an `#[inline(never)]` dispatcher.
const OUTLINED: u8 = 2;
/// Every manifold to the solver, touching or not.
const ALL_MANIFOLDS: u8 = 3;
/// User changes through DD's `propagate_modified_body_positions_to_colliders`.
const PROPAGATE: u8 = 4;
/// Broad phase with the static-proxy cache.
const PROXY_CACHE: u8 = 5;
/// Narrow phase through `compute_contacts_bucketed` (one loop per shape-pair kind).
const BUCKETED: u8 = 6;
/// Narrow phase through `compute_contacts_metered` (per-kind arms behind one-iteration loops).
const METERED: u8 = 7;
/// Narrow phase `compute_contacts::<InlineDispatcher>` (GG's `dispatch::contact_manifold`).
const INLINE: u8 = 8;

fn world_of(id: felt252) -> World {
    if id == 'stack3' {
        let (world, _) = scene_world(scenes::BOX_STACK3);
        world
    } else if id == 'balls' {
        row(5, true)
    } else if id == 'cuboids' {
        row(5, false)
    } else if id == 'mixed' {
        mixed()
    } else if id == 'statics' {
        statics(8)
    } else {
        let (world, _, _) = ball_on_ground(HALF, false);
        world
    }
}

#[inline(never)]
fn run(id: felt252, warmup: u32, stages: u8, variant: u8) {
    let mut world = world_of(id);
    let mut cache = ProxyCacheTrait::new();
    let mut i = 0;
    while i != warmup {
        if variant == PROXY_CACHE {
            let _ = super::alternatives::step_with_cache(ref world, ref cache);
        } else {
            let _ = world.step();
        }
        i += 1;
    }
    if stages == 0 {
        return;
    }
    if variant == PROPAGATE {
        handle_user_changes_propagate(ref world.bodies, ref world.colliders);
    } else {
        handle_user_changes(ref world.bodies, ref world.colliders);
    }
    if stages == 1 {
        return;
    }
    let prediction = world.integration_parameters.prediction_distance();
    let proxies = if variant == PROXY_CACHE {
        cache.proxies(ref world.bodies, ref world.colliders, prediction)
    } else {
        world.colliders.broad_phase_proxies(ref world.bodies, prediction)
    };
    let pairs = find_pairs(proxies.span());
    if stages == 2 {
        return;
    }
    let _ = if variant == INLINE {
        world
            .narrow_phase
            .compute_contacts::<
                InlineDispatcher,
            >(prediction, ref world.bodies, ref world.colliders, pairs.span())
    } else if variant == METERED {
        compute_contacts_metered(
            ref world.narrow_phase, prediction, ref world.bodies, ref world.colliders, pairs.span(),
        )
    } else if variant == BUCKETED {
        compute_contacts_bucketed(
            ref world.narrow_phase, prediction, ref world.bodies, ref world.colliders, pairs.span(),
        )
    } else if variant == BY_KIND {
        compute_contacts_by_kind(
            ref world.narrow_phase, prediction, ref world.bodies, ref world.colliders, pairs.span(),
        )
    } else if variant == OUTLINED {
        world
            .narrow_phase
            .compute_contacts::<
                OutlinedDispatcher,
            >(prediction, ref world.bodies, ref world.colliders, pairs.span())
    } else {
        world
            .narrow_phase
            .compute_contacts::<
                DefaultDispatcher,
            >(prediction, ref world.bodies, ref world.colliders, pairs.span())
    };
    if stages == 3 {
        return;
    }
    if variant == ALL_MANIFOLDS {
        solve_all_manifolds(
            world.gravity,
            world.integration_parameters,
            ref world.bodies,
            ref world.narrow_phase,
            ref world.impulse_joints,
        );
    } else {
        solve(
            world.gravity,
            world.integration_parameters,
            ref world.bodies,
            ref world.narrow_phase,
            ref world.impulse_joints,
        );
    }
    if stages == 4 {
        return;
    }
    advance_to_final_positions(ref world.bodies, ref world.colliders);
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

// `BOX_STACK3` settled (60 steps): the per-stage split of one step.

#[test]
fn gas_stack3_setup() {
    run(opaque('stack3'), 60, 0, SHIPPED);
}

#[test]
fn gas_stack3_user_changes() {
    run(opaque('stack3'), 60, 1, SHIPPED);
}

#[test]
fn gas_stack3_broad_phase() {
    run(opaque('stack3'), 60, 2, SHIPPED);
}

#[test]
fn gas_stack3_narrow_phase() {
    run(opaque('stack3'), 60, 3, SHIPPED);
}

#[test]
fn gas_stack3_solve() {
    run(opaque('stack3'), 60, 4, SHIPPED);
}

#[test]
fn gas_stack3_advance() {
    run(opaque('stack3'), 60, 5, SHIPPED);
}

// Candidates on `BOX_STACK3`: stage costs of each variant, same state.

#[test]
fn gas_stack3_narrow_phase_by_kind() {
    run(opaque('stack3'), 60, 3, BY_KIND);
}

#[test]
fn gas_stack3_solve_all_manifolds() {
    run(opaque('stack3'), 60, 4, ALL_MANIFOLDS);
}

#[test]
fn gas_stack3_user_changes_propagate() {
    run(opaque('stack3'), 60, 1, PROPAGATE);
}

#[test]
fn gas_stack3_setup_proxy_cache() {
    run(opaque('stack3'), 60, 1, PROXY_CACHE);
}

#[test]
fn gas_stack3_broad_phase_proxy_cache() {
    run(opaque('stack3'), 60, 2, PROXY_CACHE);
}

// Narrow phase per pair kind: 5 balls vs 5 cuboids in a touching row (4 pairs each), second
// step (warm-started manifolds).

#[test]
fn gas_balls_broad_phase() {
    run(opaque('balls'), 1, 2, SHIPPED);
}

#[test]
fn gas_balls_narrow_phase() {
    run(opaque('balls'), 1, 3, SHIPPED);
}

#[test]
fn gas_balls_narrow_phase_by_kind() {
    run(opaque('balls'), 1, 3, BY_KIND);
}

#[test]
fn gas_balls_narrow_phase_outlined() {
    run(opaque('balls'), 1, 3, OUTLINED);
}

#[test]
fn gas_cuboids_broad_phase() {
    run(opaque('cuboids'), 1, 2, SHIPPED);
}

#[test]
fn gas_cuboids_narrow_phase() {
    run(opaque('cuboids'), 1, 3, SHIPPED);
}

#[test]
fn gas_cuboids_narrow_phase_by_kind() {
    run(opaque('cuboids'), 1, 3, BY_KIND);
}

#[test]
fn gas_cuboids_narrow_phase_outlined() {
    run(opaque('cuboids'), 1, 3, OUTLINED);
}

// First step of a fresh `BOX_STACK3`: every change flag raised.

#[test]
fn gas_stack3_fresh_setup() {
    run(opaque('stack3'), 0, 0, SHIPPED);
}

#[test]
fn gas_stack3_fresh_user_changes() {
    run(opaque('stack3'), 0, 1, SHIPPED);
}

#[test]
fn gas_stack3_fresh_user_changes_propagate() {
    run(opaque('stack3'), 0, 1, PROPAGATE);
}

// Solver input: touching manifolds only (shipped) vs all of them, on `mixed` (1 touching pair,
// 1 non-touching pair), second step.

#[test]
fn gas_mixed_narrow_phase() {
    run(opaque('mixed'), 1, 3, SHIPPED);
}

#[test]
fn gas_mixed_solve() {
    run(opaque('mixed'), 1, 4, SHIPPED);
}

#[test]
fn gas_mixed_solve_all_manifolds() {
    run(opaque('mixed'), 1, 4, ALL_MANIFOLDS);
}

// Narrow phase, bucketed candidate.

#[test]
fn gas_balls_narrow_phase_bucketed() {
    run(opaque('balls'), 1, 3, BUCKETED);
}

#[test]
fn gas_cuboids_narrow_phase_bucketed() {
    run(opaque('cuboids'), 1, 3, BUCKETED);
}

#[test]
fn gas_stack3_narrow_phase_bucketed() {
    run(opaque('stack3'), 60, 3, BUCKETED);
}

#[test]
fn gas_resting_broad_phase() {
    run(opaque('resting'), 2, 2, SHIPPED);
}

#[test]
fn gas_resting_narrow_phase() {
    run(opaque('resting'), 2, 3, SHIPPED);
}

#[test]
fn gas_resting_narrow_phase_bucketed() {
    run(opaque('resting'), 2, 3, BUCKETED);
}

// Proxy cache on a world of 8 fixed platforms and one falling ball.

#[test]
fn gas_statics_user_changes() {
    run(opaque('statics'), 2, 1, SHIPPED);
}

#[test]
fn gas_statics_broad_phase() {
    run(opaque('statics'), 2, 2, SHIPPED);
}

#[test]
fn gas_statics_user_changes_proxy_cache() {
    run(opaque('statics'), 2, 1, PROXY_CACHE);
}

#[test]
fn gas_statics_broad_phase_proxy_cache() {
    run(opaque('statics'), 2, 2, PROXY_CACHE);
}

// Narrow phase, metered candidate.

#[test]
fn gas_balls_narrow_phase_metered() {
    run(opaque('balls'), 1, 3, METERED);
}

#[test]
fn gas_cuboids_narrow_phase_metered() {
    run(opaque('cuboids'), 1, 3, METERED);
}

#[test]
fn gas_stack3_narrow_phase_metered() {
    run(opaque('stack3'), 60, 3, METERED);
}

#[test]
fn gas_resting_narrow_phase_metered() {
    run(opaque('resting'), 2, 3, METERED);
}

// Narrow phase, GG's dispatcher inlined (the brief's option (a)).

#[test]
fn gas_balls_narrow_phase_inline() {
    run(opaque('balls'), 1, 3, INLINE);
}

#[test]
fn gas_cuboids_narrow_phase_inline() {
    run(opaque('cuboids'), 1, 3, INLINE);
}

#[test]
fn gas_stack3_narrow_phase_inline() {
    run(opaque('stack3'), 60, 3, INLINE);
}

#[test]
fn gas_resting_narrow_phase_inline() {
    run(opaque('resting'), 2, 3, INLINE);
}
