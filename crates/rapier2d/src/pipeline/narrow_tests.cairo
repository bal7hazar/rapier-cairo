//! Work package ON: the shipped narrow phase (`compute_contacts_from_scratch::<StepDispatcher>`)
//! against the loop shipped before ON (`narrow_alternatives::compute_contacts_outlined` with
//! `DefaultDispatcher`) and against the other candidates, raw compare of the events and of every
//! pair (manifold points, warm-start data, solver data, event status) after each step. Between
//! steps both worlds get the same user edits: collision events and every combine rule on, then a
//! collision-group filter and a solver-group filter, then a removed collider and a moved body.

use core::num::traits::Zero;
use fixed::Fixed;
use rapier_core::Handle;
use rapier_core::collider::CoefficientCombineRule;
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_core::interaction_groups::InteractionGroupsTrait;
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::events::CollisionEvent;
use rapier_dynamics2d::narrow_phase::{SortedMerge, compute_contacts_from_scratch};
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySetTrait};
use rapier_geometry2d::broad_phase::find_pairs;
use crate::dispatcher::DefaultDispatcher;
use crate::world::{World, WorldTrait};
use super::fixtures::{p3_scene, random_world};
use super::narrow_alternatives::{
    PersistentDispatcher, compute_contacts_inlined, compute_contacts_outlined,
};
use super::step_dispatcher::StepDispatcher;
use super::{collision_inputs, solve_and_advance, user_changes_bodies};

/// 0.1 in Q32.32.
const ONE_TENTH: Fixed = Fixed { raw: 429496730 };

/// Shipped narrow phase.
const SHIPPED: u8 = 0;
/// The pre-ON loop (reference).
const OUTLINED: u8 = 1;
/// The shipped loop with the metered `DefaultDispatcher`.
const METERED: u8 = 2;
/// The shipped loop with `PersistentDispatcher`.
const PERSISTENT: u8 = 3;
/// `compute_contacts_inlined`.
const INLINED: u8 = 4;

/// One fused step (as `step`) with the narrow phase `variant`; returns the events.
fn step_with(ref world: World, variant: u8) -> Array<CollisionEvent> {
    let (snapshot, infos, entries) = user_changes_bodies(ref world.bodies, ref world.colliders);
    let prediction = world.integration_parameters.prediction_distance();
    let (proxies, scratch) = collision_inputs(snapshot, infos, ref world.bodies, prediction);
    let pairs = find_pairs(proxies.span()).span();
    let events = if variant == SHIPPED {
        compute_contacts_from_scratch::<
            StepDispatcher,
        >(ref world.narrow_phase, prediction, scratch, pairs, ref world.colliders)
    } else if variant == METERED {
        compute_contacts_from_scratch::<
            DefaultDispatcher,
        >(ref world.narrow_phase, prediction, scratch, pairs, ref world.colliders)
    } else if variant == PERSISTENT {
        compute_contacts_from_scratch::<
            PersistentDispatcher,
        >(ref world.narrow_phase, prediction, scratch, pairs, ref world.colliders)
    } else if variant == INLINED {
        compute_contacts_inlined::<
            DefaultDispatcher,
        >(ref world.narrow_phase, prediction, scratch, pairs, ref world.colliders)
    } else {
        compute_contacts_outlined::<
            DefaultDispatcher, SortedMerge,
        >(ref world.narrow_phase, prediction, scratch, pairs, ref world.colliders)
    };
    solve_and_advance(
        world.gravity,
        world.integration_parameters,
        ref world.bodies,
        ref world.colliders,
        ref world.narrow_phase,
        ref world.impulse_joints,
        entries,
        snapshot,
    );
    events
}

/// The combine rule of index `k` (every rule in turn).
fn rule(k: u32) -> CoefficientCombineRule {
    let r = k % 6;
    if r == 0 {
        CoefficientCombineRule::Average
    } else if r == 1 {
        CoefficientCombineRule::Min
    } else if r == 2 {
        CoefficientCombineRule::Multiply
    } else if r == 3 {
        CoefficientCombineRule::Max
    } else if r == 4 {
        CoefficientCombineRule::ClampedSum
    } else {
        CoefficientCombineRule::GeometricMean
    }
}

/// The user edit before step `step` (see the module documentation).
fn edit(ref world: World, step: u32) {
    let entries = world.colliders.iter();
    let n = entries.len();
    if step == 1 {
        let mut k = 0;
        while k != n {
            let (handle, mut collider) = *entries.at(k);
            collider.flags.active_events = COLLISION_EVENTS;
            if k % 3 == 1 {
                collider.material.friction_combine_rule = rule(k);
                collider.material.restitution_combine_rule = rule(k + 1);
                collider.material.restitution = ONE_TENTH;
            }
            assert!(world.set_collider(handle, collider));
            k += 1;
        }
    } else if step == 3 && n > 2 {
        let (handle, mut collider) = *entries.at(1);
        collider.flags.collision_groups = InteractionGroupsTrait::none();
        assert!(world.set_collider(handle, collider));
        let (handle, mut collider) = *entries.at(n - 1);
        collider.flags.solver_groups = InteractionGroupsTrait::none();
        assert!(world.set_collider(handle, collider));
    } else if step == 5 && n > 3 {
        let (handle, _) = *entries.at(2);
        let _ = world.remove_collider(handle);
        let bodies: Span<(Handle, RigidBody)> = world.bodies.iter().span();
        let (handle, mut body) = *bodies.at(bodies.len() - 1);
        let mut position = body.pos.position;
        position.translation.x = position.translation.x + ONE_TENTH;
        body.pos.position = position;
        assert!(world.set_body(handle, body));
    }
}

/// Steps `expected` (variant `reference`) and `got` (variant `variant`) `steps` times with the
/// same edits; events and pairs raw-equal after every step. Returns the number of touching
/// pairs and of events seen, so that callers can check the scene exercised them.
fn agree(
    ref expected: World, ref got: World, reference: u8, variant: u8, steps: u32,
) -> (u32, u32) {
    let mut touching = 0;
    let mut events = 0;
    let mut step = 0;
    while step != steps {
        edit(ref expected, step);
        edit(ref got, step);
        let e = step_with(ref expected, reference);
        let g = step_with(ref got, variant);
        assert!(g == e, "variant {} step {} events", variant, step);
        assert!(
            got.narrow_phase.pairs == expected.narrow_phase.pairs,
            "variant {} step {} pairs",
            variant,
            step,
        );
        for pair in got.narrow_phase.pairs.span() {
            if !pair.manifold.data.num_solver_contacts.is_zero() {
                touching += 1;
            }
        }
        events += g.len();
        step += 1;
    }
    (touching, events)
}

/// `variant` against the pre-ON loop on each `(scene, size)` of `scenes` (`fixtures::p3_scene`),
/// through contact, filter, removal and motion edits; every scene must touch and emit events.
fn matches_pre_on(scenes: Span<(felt252, u32)>, variant: u8) {
    for (id, n) in scenes {
        let mut expected = p3_scene(*id, *n);
        let mut got = p3_scene(*id, *n);
        let (touching, events) = agree(ref expected, ref got, OUTLINED, variant, 7);
        assert!(touching != 0 && events != 0, "{} variant {}", *id, variant);
    }
}

/// Shipped: ball–ball, cuboid–cuboid and ball–half-space rows.
#[test]
fn test_shipped_matches_pre_on_on_rows() {
    matches_pre_on(array![('row', 3), ('cubes', 3), ('balls', 3)].span(), SHIPPED);
}

/// Shipped: the cuboid stack and the mixed pile (ball–cuboid, cuboid–capsule, capsule–ball
/// and half-space pairs).
#[test]
fn test_shipped_matches_pre_on_on_piles() {
    matches_pre_on(array![('stack', 3), ('mixed', 8)].span(), SHIPPED);
}

/// The losing candidates, on a cuboid row (persistence fast path) and on balls on a half-space.
#[test]
fn test_candidates_match_pre_on() {
    for variant in array![METERED, PERSISTENT, INLINED].span() {
        matches_pre_on(array![('cubes', 3), ('balls', 3)].span(), *variant);
    }
}

/// The shipped narrow phase against the pre-ON loop on random worlds (fixed, kinematic and
/// dynamic bodies, sensors, disabled colliders, several colliders per body, free slots).
#[test]
#[fuzzer(runs: 8, seed: 20260924)]
fn fuzz_narrow_phase_matches_pre_on(seed: u16) {
    let mut expected = random_world(seed.into());
    let mut got = random_world(seed.into());
    let _ = agree(ref expected, ref got, OUTLINED, SHIPPED, 7);
}

/// `World::step` is the shipped narrow phase (same events and pairs as `step_with`).
#[test]
fn test_world_step_is_shipped() {
    let mut expected = p3_scene('mixed', 8);
    let mut got = p3_scene('mixed', 8);
    let mut step = 0;
    while step != 3 {
        let e = step_with(ref expected, SHIPPED);
        let g = got.step();
        assert!(g == e && got.narrow_phase.pairs == expected.narrow_phase.pairs);
        step += 1;
    }
}
