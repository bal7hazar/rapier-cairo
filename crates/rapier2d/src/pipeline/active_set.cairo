//! The persistent active set (work package BT2): a step whose world has sleeping bodies walks
//! only the awake ones (upstream's island manager keeps an active set and the broad phase keeps
//! the proxies of the colliders that do not move; `docs/PLAN.md` D7 / D9 amended).
//!
//! [`ActiveSet`] lives in `World` (and in `WorldState`). A step that took the whole path fills it
//! ([`rebuild`]) when the next step can do without the walks over every body, collider and pair:
//! some body sleeps, no body woke, the world has no joint, no position-based kinematic body and no
//! disabled sleeping body. It records the bodies the step moves or whose proxies are not static
//! (every non-fixed body that is not asleep, awake or disabled), their colliders, the proxies of
//! every other collider (loosened AABBs: those colliders do not move while nothing wakes), the
//! positions in the pair list of the pairs that are not dormant (exactly the pairs with an active
//! collider: a pair left by a step has a non-static side or a sleeping one), the number of
//! sleeping members and whether a collider enables force events.
//!
//! The next step ([`sparse_step`]) trusts it when neither set was written since
//! (`RigidBodySetTrait::is_modified`, `ColliderSetTrait::is_modified`: the user changes, a
//! removal, a wake-up, the setters all write), the world still has no joint and the prediction
//! distance is the recorded one. Then, with the same results as the whole step:
//! * no user change to apply (every flag was cleared by the step that filled the set);
//! * the proxies of the active colliders are computed, the static ones reused, and only the
//!   pairs with an active proxy are looked for (`broad_phase::find_pairs_sparse`, the pairs and
//!   the order of `find_pairs`);
//! * the narrow phase gets the recorded live pairs as its previous pairs (the reference's
//!   `split_dormant` gives the same) and only the new candidates (built from the colliders of the
//!   pairs found); the pair list is left untouched when the live pairs come out unchanged, merged
//!   again otherwise;
//! * the island stage has nothing to do when no awake member is eligible for sleep and no touching
//!   pair links an awake body to a sleeping one (`islands::links_awake_to_sleeping`); otherwise
//!   the step falls back to the whole island stage and solver on every body (the set is then
//!   invalidated);
//! * the solver and the position update see the active bodies and the bodies their touching pairs
//!   reference (`solve_and_advance_sleeping` on that ascending sub-list: the same members in the
//!   same order);
//! * the set stays valid (with the new pair positions); the sets' flags are cleared.
//!
//! An all-asleep world therefore steps in a constant number of Cairo steps, and a flight tick
//! costs its awake bodies and their candidate pairs. Equivalence against the whole step:
//! `active_set_tests` (random worlds: sleep, contact wake-ups, user changes, removals, sensors).

use core::dict::{Felt252Dict, Felt252DictTrait};
use fixed::{Fixed, HALF};
use rapier_core::Handle;
use rapier_core::collider::events::CONTACT_FORCE_EVENTS;
use rapier_core::collider::{ActiveEventsTrait, ColliderEnabled, ColliderType};
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_core::rigid_body::{RigidBodyDominanceTrait, RigidBodyType};
use rapier_dynamics2d::collider::{Collider, ColliderTrait};
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::joint::ImpulseJointSetTrait;
use rapier_dynamics2d::narrow_phase::{
    ContactPair, PairCollider, compute_contacts_from_scratch, key_before,
};
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySet, RigidBodySetTrait};
use rapier_dynamics2d::solver::island::FreeBodySolverTrait;
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::broad_phase::{BroadPhaseProxy, find_pairs_sparse};
use rapier_geometry2d::shape::ShapeTrait;
use crate::dispatcher::DefaultDispatcher;
use crate::world::World;
use super::force_events::StepOutput;
use super::free_path::no_body_info;
use super::islands::{SleepCensus, SleepCensusTrait, links_awake_to_sleeping};
use super::ordering::{BODY_SLEEPING, body_status};
use super::{merge_pairs, solve_and_advance_sleeping, split_dormant, update_islands};

/// What a step needs to skip the sleeping bodies (see the module documentation). `valid` is
/// `false` until a step fills it.
#[derive(Drop, Clone, Serde, PartialEq, Debug)]
pub struct ActiveSet {
    /// The set describes the world as the last step left it.
    pub valid: bool,
    /// Every non-fixed body that is not asleep (awake or disabled), ascending slot.
    pub bodies: Array<Handle>,
    /// The colliders of those bodies, ascending slot, each with its parent's position in `bodies`.
    pub colliders: Array<(Handle, u32)>,
    /// The broad-phase proxies of every other collider (fixed, sleeping or no parent), ascending
    /// slot, loosened by half of `prediction`.
    pub statics: Array<BroadPhaseProxy>,
    /// Positions, ascending, of the pairs of `narrow_phase.pairs` that are not dormant.
    pub pairs: Array<u32>,
    /// Island members (enabled, non-fixed) asleep.
    pub sleeping: u32,
    /// A collider enables contact-force events.
    pub force_events: bool,
    /// The prediction distance of the static proxies.
    pub prediction: Fixed,
}

pub impl ActiveSetDefault of Default<ActiveSet> {
    fn default() -> ActiveSet {
        ActiveSet {
            valid: false,
            bodies: array![],
            colliders: array![],
            statics: array![],
            pairs: array![],
            sleeping: 0,
            force_events: false,
            prediction: Fixed { raw: 0 },
        }
    }
}

/// Whether `world` can take [`sparse_step`].
#[inline(always)]
pub(crate) fn usable(ref world: World) -> bool {
    if world.bodies.is_modified() || world.colliders.is_modified() {
        return false;
    }
    let set: @ActiveSet = world.active_set.as_snapshot().unbox();
    *set.valid
        && world.impulse_joints.len() == 0
        && *set.prediction == world.integration_parameters.prediction_distance()
}

/// The end of a whole step: the active set rebuilt from the sets when `fill` (and the sets'
/// flags cleared), marked invalid otherwise. Reading the sets again here (rather than keeping
/// the step's walks alive) costs only the steps that fill the set.
#[inline(never)]
pub(crate) fn refresh(ref world: World, fill: bool) {
    if fill {
        let prediction = world.integration_parameters.prediction_distance();
        let snapshot = world.colliders.iter().span();
        let entries = world.bodies.iter().span();
        let force_events = any_force_events(snapshot);
        world
            .active_set =
                BoxTrait::new(
                    rebuild(
                        snapshot,
                        entries,
                        world.narrow_phase.pairs.span(),
                        force_events,
                        prediction,
                    ),
                );
        world.bodies.clear_modified();
        world.colliders.clear_modified();
    } else {
        invalidate(ref world);
    }
}

/// Whether the world's active set is marked valid.
#[inline(always)]
pub fn is_valid(world: @World) -> bool {
    *world.active_set.as_snapshot().unbox().valid
}

/// Marks the world's active set invalid (the next step takes the whole path).
pub fn invalidate(ref world: World) {
    let mut set = world.active_set.unbox();
    set.valid = false;
    world.active_set = BoxTrait::new(set);
}

/// The active set of a world after a whole step (see the module documentation for when it is
/// valid): `snapshot` every collider in ascending slot (the static proxies are theirs, as the
/// step computed them: those colliders did not move since),
/// `entries` every body after the islands stage, `pairs` the pair list the step leaves,
/// `force_events` whether a collider enables force events. An invalid set when a body is a
/// position-based kinematic one or disabled and asleep, or when none sleeps.
pub(crate) fn rebuild(
    snapshot: Span<(Handle, Collider)>,
    entries: Span<(Handle, RigidBody)>,
    pairs: Span<ContactPair>,
    force_events: bool,
    prediction: Fixed,
) -> ActiveSet {
    let mut bodies = array![];
    // Slot of an active body → its position in `bodies` plus one.
    let mut positions: Felt252Dict<u32> = Default::default();
    let mut sleeping: u32 = 0;
    for (handle, body) in entries {
        if *body.body_type == RigidBodyType::Fixed {
            continue;
        }
        if *body.body_type == RigidBodyType::KinematicPositionBased {
            return Default::default();
        }
        if *body.activation.sleeping {
            if !*body.enabled {
                return Default::default();
            }
            sleeping += 1;
        } else {
            positions.insert((*handle.index).into(), bodies.len() + 1);
            bodies.append(*handle);
        }
    }
    if sleeping == 0 {
        return Default::default();
    }
    let bodies_span = bodies.span();
    let mut colliders = array![];
    let mut statics = array![];
    // Slot of an active collider → true.
    let mut live: Felt252Dict<bool> = Default::default();
    let margin = prediction * HALF;
    for (handle, collider) in snapshot {
        let mut active = false;
        if let Some(parent) = collider.parent() {
            let position = positions.get(parent.index.into());
            if position != 0 && *bodies_span.at(position - 1) == parent {
                colliders.append((*handle, position - 1));
                live.insert((*handle.index).into(), true);
                active = true;
            }
        }
        if !active {
            let aabb = collider.shape.compute_aabb(*collider.pos.pose).loosened(margin);
            statics.append(BroadPhaseProxy { collider: *handle, aabb, is_static: true });
        }
    }
    let mut positions = array![];
    let mut k: u32 = 0;
    for pair in pairs {
        if live.get((*pair.collider1.index).into()) || live.get((*pair.collider2.index).into()) {
            positions.append(k);
        }
        k += 1;
    }
    ActiveSet {
        valid: true,
        bodies,
        colliders,
        statics,
        pairs: positions,
        sleeping,
        force_events,
        prediction,
    }
}

/// `live` (ascending key) merged into `dormant` (ascending key, disjoint keys), with the
/// positions of the `live` pairs in the result.
fn merge_live(
    live: Span<ContactPair>, dormant: Span<ContactPair>,
) -> (Array<ContactPair>, Array<u32>) {
    let mut live = live;
    let mut dormant = dormant;
    let mut out = array![];
    let mut positions = array![];
    while let Some(a) = live.pop_front() {
        while let Some(d) = dormant.get(0) {
            let d = d.unbox();
            if key_before(*d.collider1, *d.collider2, *a.collider1, *a.collider2) {
                out.append(*d);
                dormant.pop_front().unwrap();
            } else {
                break;
            }
        }
        positions.append(out.len());
        out.append(*a);
    }
    out.append_span(dormant);
    (out, positions)
}

/// `pairs` split by `positions` (ascending): the pairs at those positions, and the others.
fn split_at_positions(
    pairs: Span<ContactPair>, positions: Span<u32>,
) -> (Array<ContactPair>, Array<ContactPair>) {
    let mut live = array![];
    let mut dormant = array![];
    let mut positions = positions;
    let mut k: u32 = 0;
    for pair in pairs {
        let is_live = match positions.get(0) {
            Some(p) => *p.unbox() == k,
            None => false,
        };
        if is_live {
            positions.pop_front().unwrap();
            live.append(*pair);
        } else {
            dormant.append(*pair);
        }
        k += 1;
    }
    (live, dormant)
}

/// The [`PairCollider`] of `collider` for the narrow phase (as `collision_inputs`).
#[inline(always)]
fn pair_collider(
    handle: Handle,
    collider: @Collider,
    body_type: RigidBodyType,
    world_com: glam::Vec2,
    dominance: i16,
) -> PairCollider {
    let (solid, sensor) = match *collider.flags.enabled {
        ColliderEnabled::Enabled => match *collider.co_type {
            ColliderType::Solid => (true, false),
            ColliderType::Sensor => (false, true),
        },
        _ => (false, false),
    };
    PairCollider {
        handle,
        solid,
        sensor,
        shape: *collider.shape,
        pose: *collider.pos.pose,
        friction: *collider.material.friction,
        restitution: *collider.material.restitution,
        friction_combine_rule: *collider.material.friction_combine_rule,
        restitution_combine_rule: *collider.material.restitution_combine_rule,
        active_collision_types: *collider.flags.active_collision_types,
        collision_groups: *collider.flags.collision_groups,
        solver_groups: *collider.flags.solver_groups,
        active_events: *collider.flags.active_events,
        one_way: *collider.one_way,
        body: collider.parent(),
        body_type,
        world_com,
        dominance,
    }
}

/// The [`PairCollider`] of a static collider (fixed, sleeping or no parent), read from the sets.
fn static_pair_collider(handle: Handle, ref world: World) -> PairCollider {
    let collider = world.colliders.get(handle).unwrap();
    let (body_type, world_com, dominance, _) = match collider.parent() {
        Some(parent) => match world.bodies.get(parent) {
            Some(body) => (
                body.body_type,
                body.mprops.world_com,
                body.dominance.effective_group(body.body_type),
                body.activation.sleeping,
            ),
            None => no_body_info(),
        },
        None => no_body_info(),
    };
    pair_collider(handle, @collider, body_type, world_com, dominance)
}

/// `awake` (ascending slot) with the bodies the touching pairs of `pairs` reference that it lacks,
/// in ascending slot.
fn with_referenced(
    awake: Span<(Handle, RigidBody)>, pairs: Span<ContactPair>, ref bodies: RigidBodySet,
) -> Span<(Handle, RigidBody)> {
    let mut touching = false;
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            touching = true;
            break;
        }
    }
    if !touching {
        return awake;
    }
    let mut seen: Felt252Dict<bool> = Default::default();
    for (handle, _) in awake {
        seen.insert((*handle.index).into(), true);
    }
    // The missing handles, kept sorted (few: grounds and sleeping neighbours).
    let mut extra: Array<Handle> = array![];
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            for side in array![*pair.manifold.data.rigid_body1, *pair.manifold.data.rigid_body2]
                .span() {
                if let Some(h) = *side {
                    let key: felt252 = h.index.into();
                    if !seen.get(key) {
                        seen.insert(key, true);
                        let mut sorted = array![];
                        let mut placed = false;
                        for e in extra.span() {
                            if !placed && e.index > @h.index {
                                sorted.append(h);
                                placed = true;
                            }
                            sorted.append(*e);
                        }
                        if !placed {
                            sorted.append(h);
                        }
                        extra = sorted;
                    }
                }
            }
        }
    }
    if extra.is_empty() {
        return awake;
    }
    let mut out = array![];
    let mut awake = awake;
    for h in extra.span() {
        while let Some(entry) = awake.get(0) {
            let (a, _) = entry.unbox();
            if a.index > h.index {
                break;
            }
            out.append(*awake.pop_front().unwrap());
        }
        if let Some(body) = bodies.get(*h) {
            out.append((*h, body));
        }
    }
    out.append_span(awake);
    out.span()
}

/// One step of a world whose [`ActiveSet`] is [`usable`] (see the module documentation).
#[inline(never)]
pub(crate) fn sparse_step<T, impl Output: StepOutput<T>, +Drop<T>>(ref world: World) -> T {
    let params = world.integration_parameters;
    let mut set = world.active_set.unbox();
    world.active_set = BoxTrait::new(Default::default());
    let prediction = set.prediction;
    let margin = prediction * HALF;
    let cache = set.snapshot_spans();
    let (active_bodies, active_colliders, statics) = cache;
    // The active bodies and the proxies of their colliders.
    let mut awake = array![];
    let mut census: SleepCensus = SleepCensus { sleeping: set.sleeping, awake: 0, eligible: false };
    for handle in active_bodies {
        let body = world.bodies.get(*handle).unwrap();
        census.count(@body);
        awake.append((*handle, body));
    }
    let awake = awake.span();
    let mut dynamic = array![];
    let mut dynamic_colliders = array![];
    for (handle, position) in active_colliders {
        let collider = world.colliders.get(*handle).unwrap();
        let (_, body) = awake.at(*position);
        dynamic
            .append(
                BroadPhaseProxy {
                    collider: *handle,
                    aabb: collider.shape.compute_aabb(collider.pos.pose).loosened(margin),
                    is_static: false,
                },
            );
        dynamic_colliders
            .append(
                pair_collider(
                    *handle,
                    @collider,
                    *body.body_type,
                    *body.mprops.world_com,
                    body.dominance.effective_group(*body.body_type),
                ),
            );
    }
    let candidates = find_pairs_sparse(statics, dynamic.span());
    // The previous live pairs are the narrow phase's previous pairs; the others are dormant.
    let previous = world.narrow_phase.pairs;
    let mut previous_live = array![];
    for position in set.pairs.span() {
        previous_live.append(*previous.at(*position));
    }
    world.narrow_phase.pairs = previous_live.clone();
    let mut events = array![];
    if !candidates.is_empty() || !previous_live.is_empty() {
        let n_s = statics.len();
        let dynamic_colliders = dynamic_colliders.span();
        // One scratch entry per collider of the pairs found, in first-use order.
        let mut scratch = array![];
        let mut at: Felt252Dict<u32> = Default::default();
        let mut pairs = array![];
        for (a, b) in candidates.span() {
            let mut ends = array![];
            for index in array![*a, *b].span() {
                let key: felt252 = (*index).into();
                let mut slot = at.get(key);
                if slot == 0 {
                    if *index < n_s {
                        scratch
                            .append(
                                static_pair_collider((*statics.at(*index)).collider, ref world),
                            );
                    } else {
                        scratch.append(*dynamic_colliders.at(*index - n_s));
                    }
                    slot = scratch.len();
                    at.insert(key, slot);
                }
                ends.append(slot - 1);
            }
            pairs.append((*ends.at(0), *ends.at(1)));
        }
        events =
            compute_contacts_from_scratch::<
                DefaultDispatcher,
            >(
                ref world.narrow_phase,
                prediction,
                scratch.span(),
                pairs.span(),
                ref world.colliders,
            );
    }
    // The bodies the touching pairs reference join the active ones.
    let entries = with_referenced(awake, world.narrow_phase.pairs.span(), ref world.bodies);
    let active_pairs = world.narrow_phase.pairs.span();
    let islands = census.awake != 0
        && (census.eligible
            || (census.sleeping != 0
                && links_awake_to_sleeping(active_pairs, array![].span(), entries)));
    let mut still_valid = true;
    let mut dormant = array![];
    if islands {
        // The whole island stage and solver, on every body. The set stays valid when nobody woke
        // up nor fell asleep (only the awake bodies can fall asleep).
        let (_, rest) = split_at_positions(previous.span(), set.pairs.span());
        dormant = rest;
        let all = world.bodies.iter().span();
        let (all, sleeping, woken) = update_islands(
            ref world.bodies,
            active_pairs,
            dormant.span(),
            array![].span(),
            all,
            SleepCensusTrait::taken(all),
        );
        still_valid = !woken;
        for handle in active_bodies {
            if still_valid && body_status(all, *handle) == BODY_SLEEPING {
                still_valid = false;
            }
        }
        if woken && !dormant.is_empty() {
            let (revived, asleep) = split_dormant(dormant.span(), all);
            world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), revived.span());
            dormant = asleep;
        }
        solve_and_advance_sleeping(
            world.gravity,
            params,
            ref world.bodies,
            ref world.colliders,
            ref world.narrow_phase,
            ref world.impulse_joints,
            all,
            array![].span(),
            array![].span(),
            sleeping,
        );
    } else if entries.is_empty() {
        // Nothing moves: only the parameters are validated, as the solver stage does.
        let _ = FreeBodySolverTrait::new(params, world.gravity);
    } else {
        solve_and_advance_sleeping(
            world.gravity,
            params,
            ref world.bodies,
            ref world.colliders,
            ref world.narrow_phase,
            ref world.impulse_joints,
            entries,
            array![].span(),
            array![].span(),
            true,
        );
    }
    let output = Output::finish(
        events, set.force_events, params.dt, ref world.narrow_phase, ref world.colliders,
    );
    if !still_valid {
        if !dormant.is_empty() {
            world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
        }
    } else if world.narrow_phase.pairs == previous_live {
        // The live pairs came out unchanged: the list stays as it was.
        world.narrow_phase.pairs = previous;
    } else {
        let (_, rest) = split_at_positions(previous.span(), set.pairs.span());
        let (merged, positions) = merge_live(world.narrow_phase.pairs.span(), rest.span());
        world.narrow_phase.pairs = merged;
        set.pairs = positions;
    }
    set.valid = still_valid;
    world.active_set = BoxTrait::new(set);
    world.bodies.clear_modified();
    world.colliders.clear_modified();
    output
}

#[generate_trait]
impl ActiveSetSpans of ActiveSetSpansTrait {
    /// The three lists as spans.
    #[inline(always)]
    fn snapshot_spans(
        self: @ActiveSet,
    ) -> (Span<Handle>, Span<(Handle, u32)>, Span<BroadPhaseProxy>) {
        (self.bodies.span(), self.colliders.span(), self.statics.span())
    }
}

/// Whether a collider of `snapshot` enables contact-force events (as the step's census).
fn any_force_events(snapshot: Span<(Handle, Collider)>) -> bool {
    for (_, collider) in snapshot {
        if *collider.flags.active_events.bits != 0
            && (*collider.flags.active_events).contains(CONTACT_FORCE_EVENTS) {
            return true;
        }
    }
    false
}
