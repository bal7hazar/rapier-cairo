//! Rejected force-event dispatch candidates, including their complete step context.
use fixed::{FixedTrait, ZERO};
use rapier_core::collider::ActiveEventsTrait;
use rapier_core::collider::events::CONTACT_FORCE_EVENTS;
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::events::{CollisionEvent, ContactForceEventTrait};
use rapier_dynamics2d::joint::ImpulseJointSetTrait;
use rapier_dynamics2d::narrow_phase::composite::group_len;
use rapier_dynamics2d::narrow_phase::compute_contacts_from_scratch;
use rapier_geometry2d::broad_phase::find_pairs;
use rapier_geometry2d::shape::ShapeTrait;
use crate::dispatcher::DefaultDispatcher;
use crate::pipeline::{
    collision_inputs_with_events, force_events, merge_pairs, solve_and_advance_sleeping,
    split_dormant, update_islands, user_changes_bodies_for_step,
};
use crate::world::World;
use super::{
    ColliderSet, ContactForceEvent, Fixed, NarrowPhase, collect, convex_event, group_event,
    threshold,
};
/// Keeps the metered branch's live state to the collider and pair sets, not the whole World.
#[inline(never)]
pub fn collect_if_enabled(
    enabled: bool, dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    let mut events = array![];
    let mut pending = enabled;
    while pending {
        events = collect(dt, ref narrow, ref colliders);
        pending = false;
    }
    events
}
/// Initial candidate: the metered loop carries the full World across its back edge.
#[inline(never)]
pub fn collect_world(ref world: World, enabled: bool) -> Array<ContactForceEvent> {
    let mut events = array![];
    let mut pending = enabled;
    while pending {
        events =
            collect(world.integration_parameters.dt, ref world.narrow_phase, ref world.colliders);
        pending = false;
    }
    events
}

/// Whole-world metering at the original call site.
pub fn step_world(ref world: World) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
    let (snapshot, infos, entries, census, _) = user_changes_bodies_for_step(
        ref world.bodies,
        ref world.colliders,
        world.narrow_phase.pairs.span(),
        Some(world.integration_parameters.dt),
        true,
    );
    let prediction = world.integration_parameters.prediction_distance();
    let (proxies, scratch, sleeping, force_events) = collision_inputs_with_events(
        snapshot, infos, ref world.bodies, prediction,
    );
    let mut dormant = array![];
    if sleeping {
        let (active, asleep) = split_dormant(world.narrow_phase.pairs.span(), entries);
        world.narrow_phase.pairs = active;
        dormant = asleep;
    }
    let pairs = find_pairs(proxies.span());
    let events = compute_contacts_from_scratch::<
        DefaultDispatcher,
    >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
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
        // The dormant pairs of the woken bodies join the solver input of this step.
        let (revived, asleep) = split_dormant(dormant.span(), entries);
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), revived.span());
        dormant = asleep;
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
    let mut forces = array![];
    let mut pending = force_events;
    while pending {
        forces =
            force_events::collect(
                world.integration_parameters.dt, ref world.narrow_phase, ref world.colliders,
            );
        pending = false;
    }
    if !dormant.is_empty() {
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
    }
    (events, forces)
}

/// Outlined reduced-argument metering at the original call site.
pub fn step_reduced(ref world: World) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
    let (snapshot, infos, entries, census, _) = user_changes_bodies_for_step(
        ref world.bodies,
        ref world.colliders,
        world.narrow_phase.pairs.span(),
        Some(world.integration_parameters.dt),
        true,
    );
    let prediction = world.integration_parameters.prediction_distance();
    let (proxies, scratch, sleeping, force_events) = collision_inputs_with_events(
        snapshot, infos, ref world.bodies, prediction,
    );
    let mut dormant = array![];
    if sleeping {
        let (active, asleep) = split_dormant(world.narrow_phase.pairs.span(), entries);
        world.narrow_phase.pairs = active;
        dormant = asleep;
    }
    let pairs = find_pairs(proxies.span());
    let events = compute_contacts_from_scratch::<
        DefaultDispatcher,
    >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
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
        // The dormant pairs of the woken bodies join the solver input of this step.
        let (revived, asleep) = split_dormant(dormant.span(), entries);
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), revived.span());
        dormant = asleep;
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
    let forces = collect_if_enabled(
        force_events, world.integration_parameters.dt, ref world.narrow_phase, ref world.colliders,
    );
    if !dormant.is_empty() {
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
    }
    (events, forces)
}

/// Direct conditional without an explicit refund boundary.
pub fn step_unwalleted(ref world: World) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
    let (snapshot, infos, entries, census, _) = user_changes_bodies_for_step(
        ref world.bodies,
        ref world.colliders,
        world.narrow_phase.pairs.span(),
        Some(world.integration_parameters.dt),
        true,
    );
    let prediction = world.integration_parameters.prediction_distance();
    let (proxies, scratch, sleeping, force_events) = collision_inputs_with_events(
        snapshot, infos, ref world.bodies, prediction,
    );
    let mut dormant = array![];
    if sleeping {
        let (active, asleep) = split_dormant(world.narrow_phase.pairs.span(), entries);
        world.narrow_phase.pairs = active;
        dormant = asleep;
    }
    let pairs = find_pairs(proxies.span());
    let events = compute_contacts_from_scratch::<
        DefaultDispatcher,
    >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
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
        // The dormant pairs of the woken bodies join the solver input of this step.
        let (revived, asleep) = split_dormant(dormant.span(), entries);
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), revived.span());
        dormant = asleep;
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
    let forces = if force_events {
        force_events::collect(
            world.integration_parameters.dt, ref world.narrow_phase, ref world.colliders,
        )
    } else {
        array![]
    };
    if !dormant.is_empty() {
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
    }
    (events, forces)
}

/// Direct conditional with an explicit gas wallet.
pub fn step_wallet(ref world: World) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
    let (snapshot, infos, entries, census, _) = user_changes_bodies_for_step(
        ref world.bodies,
        ref world.colliders,
        world.narrow_phase.pairs.span(),
        Some(world.integration_parameters.dt),
        true,
    );
    let prediction = world.integration_parameters.prediction_distance();
    let (proxies, scratch, sleeping, force_events) = collision_inputs_with_events(
        snapshot, infos, ref world.bodies, prediction,
    );
    let mut dormant = array![];
    if sleeping {
        let (active, asleep) = split_dormant(world.narrow_phase.pairs.span(), entries);
        world.narrow_phase.pairs = active;
        dormant = asleep;
    }
    let pairs = find_pairs(proxies.span());
    let events = compute_contacts_from_scratch::<
        DefaultDispatcher,
    >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
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
        // The dormant pairs of the woken bodies join the solver input of this step.
        let (revived, asleep) = split_dormant(dormant.span(), entries);
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), revived.span());
        dormant = asleep;
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
    force_events::gas_wallet();
    let forces = if force_events {
        force_events::collect(
            world.integration_parameters.dt, ref world.narrow_phase, ref world.colliders,
        )
    } else {
        array![]
    };
    if !dormant.is_empty() {
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
    }
    (events, forces)
}

/// Tied candidate: capture a unit payload before merging, assemble the return afterwards.
trait UnitOutput<T, Forces> {
    fn capture(
        enabled: bool, dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
    ) -> Forces;
    fn finish(events: Array<CollisionEvent>, forces: Forces) -> T;
}

impl UnitOnly of UnitOutput<Array<CollisionEvent>, ()> {
    #[inline(always)]
    fn capture(enabled: bool, dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet) {
        if enabled {
            let _ = collect(dt, ref narrow, ref colliders);
        }
    }
    fn finish(events: Array<CollisionEvent>, forces: ()) -> Array<CollisionEvent> {
        events
    }
}

fn step_unit_internal<T, Forces, impl Output: UnitOutput<T, Forces>, +Drop<Forces>>(
    ref world: World,
) -> T {
    let (snapshot, infos, entries, census, _) = user_changes_bodies_for_step(
        ref world.bodies,
        ref world.colliders,
        world.narrow_phase.pairs.span(),
        Some(world.integration_parameters.dt),
        true,
    );
    let prediction = world.integration_parameters.prediction_distance();
    let (proxies, scratch, sleeping, force_events) = collision_inputs_with_events(
        snapshot, infos, ref world.bodies, prediction,
    );
    let mut dormant = array![];
    if sleeping {
        let (active, asleep) = split_dormant(world.narrow_phase.pairs.span(), entries);
        world.narrow_phase.pairs = active;
        dormant = asleep;
    }
    let pairs = find_pairs(proxies.span());
    let events = compute_contacts_from_scratch::<
        DefaultDispatcher,
    >(ref world.narrow_phase, prediction, scratch, pairs.span(), ref world.colliders);
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
        // The dormant pairs of the woken bodies join the solver input of this step.
        let (revived, asleep) = split_dormant(dormant.span(), entries);
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), revived.span());
        dormant = asleep;
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
    // Dormant manifolds retain old impulses; emit only from this step's active pairs.
    let forces = Output::capture(
        force_events, world.integration_parameters.dt, ref world.narrow_phase, ref world.colliders,
    );
    if !dormant.is_empty() {
        world.narrow_phase.pairs = merge_pairs(world.narrow_phase.pairs.span(), dormant.span());
    }
    Output::finish(events, forces)
}

pub fn step_unit_payload(ref world: World) -> Array<CollisionEvent> {
    step_unit_internal::<Array<CollisionEvent>, (), UnitOnly>(ref world)
}

/// SH2a's collect (shipped in `0.1.0-alpha.5`): every entry pays the group scan
/// (`group_len`) and the loop carries the entry index and the group countdown (RG1: +1.5 % Cairo
/// steps on a level whose colliders all have force events).
pub fn collect_group_scan(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    collect_counted(dt, ref narrow, ref colliders, false)
}

/// [`collect_group_scan`] with the scan gated by `ShapeTrait::is_composite` (the loop still
/// carries the index and the countdown).
pub fn collect_gated_scan(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    collect_counted(dt, ref narrow, ref colliders, true)
}

#[inline(always)]
fn collect_counted(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet, gated: bool,
) -> Array<ContactForceEvent> {
    let inv_dt = if dt == ZERO {
        ZERO
    } else {
        dt.recip()
    };
    let mut events = array![];
    let mut pairs = array![];
    let span = narrow.pairs.span();
    let mut k: u32 = 0;
    let mut members: u32 = 0;
    for old in span {
        let mut pair = *old;
        k += 1;
        if members != 0 {
            members -= 1;
            pairs.append(pair);
            continue;
        }
        let co1 = colliders.get(pair.collider1).unwrap();
        let co2 = colliders.get(pair.collider2).unwrap();
        let group = if !gated || co1.shape.is_composite() || co2.shape.is_composite() {
            group_len(span, k - 1)
        } else {
            1
        };
        if (co1.flags.active_events | co2.flags.active_events).contains(CONTACT_FORCE_EVENTS) {
            let limit = threshold(co1).min(threshold(co2));
            if group != 1 {
                members = group - 1;
                match group_event(inv_dt, @pair, span.slice(k, group - 1)) {
                    Some(event) => if event.total_force_magnitude > limit {
                        events.append(event);
                        pair.event_status.bits = pair.event_status.bits | 2;
                    } else {
                        pair.event_status.bits = pair.event_status.bits & 253;
                    },
                    None => { pair.event_status.bits = pair.event_status.bits & 253; },
                }
                pairs.append(pair);
                continue;
            }
            let magnitude = if pair.manifold.data.num_solver_contacts == 0
                || pair.manifold.data.solver_flags.bits == 0 {
                ZERO
            } else {
                let [a, b] = pair.manifold.points;
                let first = if pair.manifold.num_points != 0 {
                    a.data.impulse
                } else {
                    ZERO
                };
                let second = if pair.manifold.num_points > 1 {
                    b.data.impulse
                } else {
                    ZERO
                };
                (first + second) * inv_dt
            };
            if magnitude > limit
                && pair.manifold.data.num_solver_contacts != 0
                && pair.manifold.data.solver_flags.bits != 0 {
                events.append(ContactForceEventTrait::from_contact_pair(dt, @pair, magnitude));
                pair.event_status.bits = pair.event_status.bits | 2;
            } else {
                pair.event_status.bits = pair.event_status.bits & 253;
            }
        } else {
            members = group - 1;
        }
        pairs.append(pair);
    }
    narrow.pairs = pairs;
    events
}

/// Reference floor: `0.1.0-alpha.4`'s loop, which knows no composite group (wrong for them).
pub fn collect_alpha4(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    let inv_dt = if dt == ZERO {
        ZERO
    } else {
        dt.recip()
    };
    let mut events = array![];
    let mut pairs = array![];
    for old in narrow.pairs.span() {
        let mut pair = *old;
        let co1 = colliders.get(pair.collider1).unwrap();
        let co2 = colliders.get(pair.collider2).unwrap();
        if (co1.flags.active_events | co2.flags.active_events).contains(CONTACT_FORCE_EVENTS) {
            let limit = threshold(co1).min(threshold(co2));
            convex_event(dt, inv_dt, limit, ref pair, ref events);
        }
        pairs.append(pair);
    }
    narrow.pairs = pairs;
    events
}

/// The shipped loop with the group found by peeking at the next entry's key instead of the
/// colliders' shapes.
pub fn collect_peek(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    let inv_dt = if dt == ZERO {
        ZERO
    } else {
        dt.recip()
    };
    let mut events = array![];
    let mut pairs = array![];
    let mut rest = narrow.pairs.span();
    while let Some(old) = rest.pop_front() {
        let mut pair = *old;
        let co1 = colliders.get(pair.collider1).unwrap();
        let co2 = colliders.get(pair.collider2).unwrap();
        if (co1.flags.active_events | co2.flags.active_events).contains(CONTACT_FORCE_EVENTS) {
            let limit = threshold(co1).min(threshold(co2));
            let grouped = match rest.get(0) {
                Some(next) => {
                    let next = next.unbox();
                    *next.collider1 == pair.collider1 && *next.collider2 == pair.collider2
                },
                None => false,
            };
            if grouped
                && super::composite_group(inv_dt, limit, pair, ref rest, ref pairs, ref events) {
                continue;
            }
            convex_event(dt, inv_dt, limit, ref pair, ref events);
        }
        pairs.append(pair);
    }
    narrow.pairs = pairs;
    events
}

/// The shipped loop with the composite branch as a call returning values (the lead, its event
/// and the member count) instead of `ref` arrays.
pub fn collect_by_value(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    let inv_dt = if dt == ZERO {
        ZERO
    } else {
        dt.recip()
    };
    let mut events = array![];
    let mut pairs = array![];
    let mut rest = narrow.pairs.span();
    while let Some(old) = rest.pop_front() {
        let mut pair = *old;
        let co1 = colliders.get(pair.collider1).unwrap();
        let co2 = colliders.get(pair.collider2).unwrap();
        if (co1.flags.active_events | co2.flags.active_events).contains(CONTACT_FORCE_EVENTS) {
            let limit = threshold(co1).min(threshold(co2));
            if co1.shape.is_composite() || co2.shape.is_composite() {
                let (lead, event, n) = group_by_value(inv_dt, limit, pair, rest);
                if n != 0 {
                    if let Some(event) = event {
                        events.append(event);
                    }
                    pairs.append(lead);
                    pairs.append_span(rest.slice(0, n));
                    rest = rest.slice(n, rest.len() - n);
                    continue;
                }
            }
            convex_event(dt, inv_dt, limit, ref pair, ref events);
        }
        pairs.append(pair);
    }
    narrow.pairs = pairs;
    events
}

#[inline(never)]
fn group_by_value(
    inv_dt: Fixed,
    limit: Fixed,
    pair: rapier_dynamics2d::narrow_phase::ContactPair,
    rest: Span<rapier_dynamics2d::narrow_phase::ContactPair>,
) -> (rapier_dynamics2d::narrow_phase::ContactPair, Option<ContactForceEvent>, u32) {
    let mut rest = rest;
    let mut pairs = array![];
    let mut events = array![];
    let before = rest.len();
    if !super::composite_group(inv_dt, limit, pair, ref rest, ref pairs, ref events) {
        return (pair, None, 0);
    }
    (*pairs.at(0), events.pop_front(), before - rest.len())
}

/// Reference: the alpha.4 loop written with `pop_front` (the shipped loop's form).
pub fn collect_alpha4_pop(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    let inv_dt = if dt == ZERO {
        ZERO
    } else {
        dt.recip()
    };
    let mut events = array![];
    let mut pairs = array![];
    let mut rest = narrow.pairs.span();
    while let Some(old) = rest.pop_front() {
        let mut pair = *old;
        let co1 = colliders.get(pair.collider1).unwrap();
        let co2 = colliders.get(pair.collider2).unwrap();
        if (co1.flags.active_events | co2.flags.active_events).contains(CONTACT_FORCE_EVENTS) {
            let limit = threshold(co1).min(threshold(co2));
            convex_event(dt, inv_dt, limit, ref pair, ref events);
        }
        pairs.append(pair);
    }
    narrow.pairs = pairs;
    events
}

/// The alpha.4 loop that gives up at the first enabled pair with a composite collider and
/// reruns the whole list through the group-aware loop (`collect_by_value`).
pub fn collect_abort(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    let inv_dt = if dt == ZERO {
        ZERO
    } else {
        dt.recip()
    };
    let mut events = array![];
    let mut pairs = array![];
    let mut composite = false;
    for old in narrow.pairs.span() {
        let mut pair = *old;
        let co1 = colliders.get(pair.collider1).unwrap();
        let co2 = colliders.get(pair.collider2).unwrap();
        if (co1.flags.active_events | co2.flags.active_events).contains(CONTACT_FORCE_EVENTS) {
            if co1.shape.is_composite() || co2.shape.is_composite() {
                composite = true;
                break;
            }
            let limit = threshold(co1).min(threshold(co2));
            convex_event(dt, inv_dt, limit, ref pair, ref events);
        }
        pairs.append(pair);
    }
    if composite {
        return collect_by_value_outlined(dt, ref narrow, ref colliders);
    }
    narrow.pairs = pairs;
    events
}

#[inline(never)]
fn collect_by_value_outlined(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    collect_by_value(dt, ref narrow, ref colliders)
}

/// RG1's first candidate: one group-aware loop (the composite test and the out-of-line group
/// call on every enabled pair), no hand-off.
pub fn collect_single_loop(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    let inv_dt = if dt == ZERO {
        ZERO
    } else {
        dt.recip()
    };
    let mut events = array![];
    let mut pairs = array![];
    let mut rest = narrow.pairs.span();
    while let Some(old) = rest.pop_front() {
        let mut pair = *old;
        let co1 = colliders.get(pair.collider1).unwrap();
        let co2 = colliders.get(pair.collider2).unwrap();
        if (co1.flags.active_events | co2.flags.active_events).contains(CONTACT_FORCE_EVENTS) {
            let limit = threshold(co1).min(threshold(co2));
            if (co1.shape.is_composite() || co2.shape.is_composite())
                && super::composite_group(inv_dt, limit, pair, ref rest, ref pairs, ref events) {
                continue;
            }
            convex_event(dt, inv_dt, limit, ref pair, ref events);
        }
        // Upstream visits only enabled force-event pairs. Preserve inactive bookkeeping,
        // including when some unrelated collider keeps the global collection pass enabled.
        pairs.append(pair);
    }
    narrow.pairs = pairs;
    events
}
