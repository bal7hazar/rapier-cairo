//! Post-solver force events, in ascending collider-pair order.
use fixed::{Fixed, FixedTrait, ZERO};
use glam_core::{Vec2, Vec2Trait};
use rapier_core::collider::ActiveEventsTrait;
use rapier_core::collider::events::CONTACT_FORCE_EVENTS;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::{CollisionEvent, ContactForceEvent, ContactForceEventTrait};
use rapier_dynamics2d::narrow_phase::strategies::errors;
use rapier_dynamics2d::narrow_phase::{ContactPair, NarrowPhase};
use rapier_geometry2d::contact::ContactManifold;
use rapier_geometry2d::shape::ShapeTrait;
use super::stages::{ForceEventStage, InProcessForceEvents};

/// The in-process outputs (what every step but `step_with_stages` returns).
pub(crate) impl CollisionOnly = CollisionOnlyWith<InProcessForceEvents>;
pub(crate) impl WithForces = WithForcesBy<InProcessForceEvents>;


/// Specialize only the return shape: both modes execute the same stages and event bookkeeping.
pub(crate) trait StepOutput<T> {
    /// The step's output. `groups`: composite pairs are grouped (CS2: a constant of the step's
    /// `CompositeStrategy`, so that the other collection pass is not compiled).
    fn finish(
        events: Array<CollisionEvent>,
        enabled: bool,
        dt: Fixed,
        ref narrow: NarrowPhase,
        ref colliders: ColliderSet,
        groups: bool,
    ) -> T;
    /// `output` with `events` appended to its collision events (CC2: the CCD pass's sensor
    /// events).
    fn with_events(output: T, events: Array<CollisionEvent>) -> T;
    /// The outputs of two consecutive substeps, concatenated (CC2).
    fn merge(first: T, second: T) -> T;
}

/// The collision events alone, force events collected by `F` (CS6) for their status bits.
pub(crate) impl CollisionOnlyWith<impl F: ForceEventStage> of StepOutput<Array<CollisionEvent>> {
    #[inline(always)]
    fn finish(
        events: Array<CollisionEvent>,
        enabled: bool,
        dt: Fixed,
        ref narrow: NarrowPhase,
        ref colliders: ColliderSet,
        groups: bool,
    ) -> Array<CollisionEvent> {
        if enabled {
            let _ = F::collect(groups, dt, ref narrow, ref colliders);
        }
        events
    }

    fn with_events(
        output: Array<CollisionEvent>, events: Array<CollisionEvent>,
    ) -> Array<CollisionEvent> {
        let mut output = output;
        output.append_span(events.span());
        output
    }

    fn merge(first: Array<CollisionEvent>, second: Array<CollisionEvent>) -> Array<CollisionEvent> {
        let mut first = first;
        first.append_span(second.span());
        first
    }
}

/// The collision and force events, force events collected by `F` (CS6).
pub(crate) impl WithForcesBy<
    impl F: ForceEventStage,
> of StepOutput<(Array<CollisionEvent>, Array<ContactForceEvent>)> {
    #[inline(always)]
    fn finish(
        events: Array<CollisionEvent>,
        enabled: bool,
        dt: Fixed,
        ref narrow: NarrowPhase,
        ref colliders: ColliderSet,
        groups: bool,
    ) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
        // Inlined dispatch: only the changed sets cross the branch merge.
        let forces = if enabled {
            F::collect(groups, dt, ref narrow, ref colliders)
        } else {
            array![]
        };
        (events, forces)
    }

    fn with_events(
        output: (Array<CollisionEvent>, Array<ContactForceEvent>), events: Array<CollisionEvent>,
    ) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
        let (mut collisions, forces) = output;
        collisions.append_span(events.span());
        (collisions, forces)
    }

    fn merge(
        first: (Array<CollisionEvent>, Array<ContactForceEvent>),
        second: (Array<CollisionEvent>, Array<ContactForceEvent>),
    ) -> (Array<CollisionEvent>, Array<ContactForceEvent>) {
        let (mut collisions, mut forces) = first;
        let (more_collisions, more_forces) = second;
        collisions.append_span(more_collisions.span());
        forces.append_span(more_forces.span());
        (collisions, forces)
    }
}


#[cfg(test)]
/// Refund boundary for the optional event branch, as the JM solver gas-wallet pattern.
#[inline(never)]
pub(crate) fn gas_wallet() {
    let mut pending = false;
    while pending {
        pending = false;
    }
}

pub(crate) fn threshold(collider: Collider) -> Fixed {
    if collider.flags.active_events.contains(CONTACT_FORCE_EVENTS) {
        collider.contact_force_event_threshold
    } else {
        Fixed { raw: 9223372036854775807 }
    }
}

/// Running sums of a composite group's force event (see [`group_event`]).
#[derive(Copy, Drop)]
struct GroupForce {
    total_force: Vec2,
    total: Fixed,
    max: Fixed,
    max_direction: Vec2,
    any: bool,
}

/// Adds the impulses of manifold `m` to `acc` when it has solver contacts (and solver flags).
#[inline(always)]
fn accumulate(ref acc: GroupForce, m: ContactManifold) {
    if m.data.num_solver_contacts == 0 || m.data.solver_flags.bits == 0 {
        return;
    }
    acc.any = true;
    let [a, b] = m.points;
    let first = if m.num_points != 0 {
        a.data.impulse
    } else {
        ZERO
    };
    let second = if m.num_points > 1 {
        b.data.impulse
    } else {
        ZERO
    };
    acc.total = acc.total + first + second;
    acc.total_force = acc.total_force + m.data.normal.mul_scalar(first + second);
    let strongest = first.max(second);
    if strongest > acc.max {
        acc.max = strongest;
        acc.max_direction = m.data.normal;
    }
}

/// The force event of the composite group made of `lead` and its `members` (upstream
/// `ContactForceEvent::from_contact_pair` over the pair's manifolds): the manifolds with solver
/// contacts (and solver flags) contribute their points' impulses along their normal; the strongest
/// point gives `max_force_*`. `None` when no manifold contributes (the pair is not in contact).
pub(crate) fn group_event(
    inv_dt: Fixed, lead: @ContactPair, members: Span<ContactPair>,
) -> Option<ContactForceEvent> {
    let mut acc = GroupForce {
        total_force: Default::default(),
        total: ZERO,
        max: ZERO,
        max_direction: Default::default(),
        any: false,
    };
    accumulate(ref acc, *lead.manifold);
    for member in members {
        accumulate(ref acc, *member.manifold);
    }
    if !acc.any {
        return None;
    }
    Some(
        ContactForceEvent {
            collider1: *lead.collider1,
            collider2: *lead.collider2,
            total_force: acc.total_force.mul_scalar(inv_dt),
            total_force_magnitude: acc.total * inv_dt,
            max_force_direction: acc.max_direction,
            max_force_magnitude: acc.max * inv_dt,
            started: *lead.event_status.bits & 2 == 0,
        },
    )
}

/// The entries of `rest` that follow `lead` in its composite group (same collider pair).
fn members_len(lead: @ContactPair, rest: Span<ContactPair>) -> u32 {
    let mut n: u32 = 0;
    for next in rest {
        if !(*next.collider1 == *lead.collider1 && *next.collider2 == *lead.collider2) {
            break;
        }
        n += 1;
    }
    n
}

/// The composite branch of [`collect`] for the enabled pair `pair`, whose collider has a
/// composite shape, followed by `rest`: when its group has more than one entry, appends the
/// group's event (if above `limit`), the lead with its updated status and the members, advances
/// `rest` past them and returns `true`; `false` (nothing done) for a one-entry group, which takes
/// the convex path.
#[inline(never)]
pub(crate) fn composite_group(
    inv_dt: Fixed,
    limit: Fixed,
    pair: ContactPair,
    ref rest: Span<ContactPair>,
    ref pairs: Array<ContactPair>,
    ref events: Array<ContactForceEvent>,
) -> bool {
    let n = members_len(@pair, rest);
    if n == 0 {
        return false;
    }
    let members = rest.slice(0, n);
    rest = rest.slice(n, rest.len() - n);
    let mut pair = pair;
    match group_event(inv_dt, @pair, members) {
        Some(event) => if event.total_force_magnitude > limit {
            events.append(event);
            pair.event_status.bits = pair.event_status.bits | 2;
        } else {
            pair.event_status.bits = pair.event_status.bits & 253;
        },
        None => { pair.event_status.bits = pair.event_status.bits & 253; },
    }
    pairs.append(pair);
    pairs.append_span(members);
    true
}

/// Emits normal-force events after impulse writeback, preserving the collision-event bit.
/// A composite pair (a group of entries, `rapier_dynamics2d::narrow_phase::composite`) emits one
/// event for the collider pair, from its lead entry, as upstream.
/// Only enabled sides contribute thresholds (minimum when both enable); strict `>`.
/// `dt == 0` means zero force, as upstream's safe inverse. Division rounds nearest-even.
/// Panics on fixed overflow. Called only when a collider has enabled force events.
///
/// Cost (RG1): the loop is `0.1.0-alpha.4`'s, plus a composite-shape test on the enabled pairs;
/// at the first enabled pair with a composite collider it hands the rest of the list to the
/// group-aware [`collect_groups`] (out of line), so a world without composite colliders pays no
/// group bookkeeping.
pub fn collect(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    collect_body(dt, ref narrow, ref colliders, true)
}

/// [`collect`] for a world without composite colliders (CS2, `NoComposites`: the narrow phase
/// rejected every composite pair): the group-aware pass is not compiled.
///
/// # Panics
/// As [`collect`], and `'Narrow phase: no composites'` on a pair with a composite collider.
pub fn collect_convex(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    collect_body(dt, ref narrow, ref colliders, false)
}

/// [`collect`] when `groups` (a constant after inlining), [`collect_convex`] otherwise.
#[inline(always)]
pub(crate) fn collect_either(
    groups: bool, dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet,
) -> Array<ContactForceEvent> {
    if groups {
        collect(dt, ref narrow, ref colliders)
    } else {
        collect_convex(dt, ref narrow, ref colliders)
    }
}

/// The body of [`collect`] (`groups`, a constant after inlining) and [`collect_convex`].
#[inline(always)]
fn collect_body(
    dt: Fixed, ref narrow: NarrowPhase, ref colliders: ColliderSet, groups: bool,
) -> Array<ContactForceEvent> {
    let inv_dt = if dt == ZERO {
        ZERO
    } else {
        dt.recip()
    };
    let mut events = array![];
    let mut pairs = array![];
    let mut rest = narrow.pairs.span();
    let mut composite = false;
    while let Some(old) = rest.pop_front() {
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
        // Upstream visits only enabled force-event pairs. Preserve inactive bookkeeping,
        // including when some unrelated collider keeps the global collection pass enabled.
        pairs.append(pair);
    }
    if composite {
        if !groups {
            core::panic_with_felt252(errors::COMPOSITE);
        }
        // Back to the composite pair: `rest` starts after it.
        let all = narrow.pairs.span();
        let start = all.len() - rest.len() - 1;
        collect_groups(
            dt, inv_dt, all.slice(start, rest.len() + 1), ref pairs, ref events, ref colliders,
        );
    }
    narrow.pairs = pairs;
    events
}

/// The force event of the one-entry pair `pair`, if its force is above `limit`, and its status
/// bit (the convex path of [`collect`] and [`collect_groups`]).
#[inline(always)]
pub(crate) fn convex_event(
    dt: Fixed,
    inv_dt: Fixed,
    limit: Fixed,
    ref pair: ContactPair,
    ref events: Array<ContactForceEvent>,
) {
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
}

/// [`collect`] from the entry `rest[0]` on, composite groups included: appends to `pairs` and
/// `events`. A disabled composite group's entries go through the loop one by one, each disabled
/// and left unchanged.
#[inline(never)]
fn collect_groups(
    dt: Fixed,
    inv_dt: Fixed,
    rest: Span<ContactPair>,
    ref pairs: Array<ContactPair>,
    ref events: Array<ContactForceEvent>,
    ref colliders: ColliderSet,
) {
    let mut rest = rest;
    while let Some(old) = rest.pop_front() {
        let mut pair = *old;
        let co1 = colliders.get(pair.collider1).unwrap();
        let co2 = colliders.get(pair.collider2).unwrap();
        if (co1.flags.active_events | co2.flags.active_events).contains(CONTACT_FORCE_EVENTS) {
            let limit = threshold(co1).min(threshold(co2));
            if (co1.shape.is_composite() || co2.shape.is_composite())
                && composite_group(inv_dt, limit, pair, ref rest, ref pairs, ref events) {
                continue;
            }
            convex_event(dt, inv_dt, limit, ref pair, ref events);
        }
        pairs.append(pair);
    }
}

#[cfg(test)]
mod tests {
    use fixed::{HALF, ONE, ZERO};
    use glam_core::Vec2;
    use rapier_dynamics2d::collider::ColliderBuilderTrait;
    use rapier_dynamics2d::narrow_phase::{ContactPairTrait, NarrowPhaseTrait};
    use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
    use rapier_testing::opaque;
    use crate::world::{World, WorldTrait};
    use super::*;

    fn fixture(limit: Fixed) -> (NarrowPhase, ColliderSet) {
        let mut colliders = ColliderSetTrait::new();
        let a = colliders
            .insert(
                ColliderBuilderTrait::ball(HALF)
                    .active_events(CONTACT_FORCE_EVENTS)
                    .contact_force_event_threshold(limit)
                    .build(),
            );
        let b = colliders.insert(ColliderBuilderTrait::ball(HALF).build());
        let mut pair = ContactPairTrait::new(a, b);
        pair.manifold.data.normal = Vec2 { x: ZERO, y: ONE };
        pair.manifold.num_points = 1;
        pair.manifold.data.num_solver_contacts = 1;
        pair.manifold.data.solver_flags.bits = 1;
        let [mut point, other] = pair.manifold.points;
        point.data.impulse = ONE;
        point.data.tangent_impulse = ONE;
        pair.manifold.points = [point, other];
        let mut narrow = NarrowPhaseTrait::new();
        narrow.pairs.append(pair);
        (narrow, colliders)
    }

    #[test]
    fn test_strict_threshold_zero_dt_and_crossings() {
        let (mut narrow, mut colliders) = fixture(ONE);
        assert!(collect(ONE, ref narrow, ref colliders).is_empty());
        let event = collect(HALF, ref narrow, ref colliders);
        assert!(*event.at(0).started);
        assert_eq!(*event.at(0).total_force_magnitude, ONE + ONE);
        let event = collect(HALF, ref narrow, ref colliders);
        assert!(!*event.at(0).started);
        assert!(collect(ZERO, ref narrow, ref colliders).is_empty());
        let event = collect(HALF, ref narrow, ref colliders);
        assert!(*event.at(0).started);
    }
    #[test]
    fn test_disabled_pair_status_is_independent_of_other_enabled_colliders() {
        let (mut narrow, mut colliders) = fixture(ZERO);
        assert!(*collect(ONE, ref narrow, ref colliders).at(0).started);
        let h = rapier_core::Handle { index: 0, generation: 0 };
        let mut co = colliders.get(h).unwrap();
        co.flags.active_events = Default::default();
        colliders.set(h, co);
        // A third collider enables the pass without enabling this pair.
        colliders
            .insert(ColliderBuilderTrait::ball(HALF).active_events(CONTACT_FORCE_EVENTS).build());
        assert!(collect(ONE, ref narrow, ref colliders).is_empty());
        co.flags.active_events = CONTACT_FORCE_EVENTS;
        colliders.set(h, co);
        assert!(!*collect(ONE, ref narrow, ref colliders).at(0).started);
    }

    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }
    #[test]
    fn gas_collect() {
        let (mut narrow, mut colliders) = fixture(opaque(ZERO));
        let _ = collect(opaque(ONE), ref narrow, ref colliders);
    }
    fn world_fixture() -> World {
        let (narrow, colliders) = fixture(opaque(ZERO));
        let mut world = WorldTrait::new(Vec2 { x: ZERO, y: ZERO }, Default::default());
        world.integration_parameters.dt = opaque(ONE);
        world.narrow_phase = narrow;
        world.colliders = colliders;
        world
    }
    #[test]
    fn gas_tail_reduced_off() {
        let mut w = world_fixture();
        let events = super::alternatives::collect_if_enabled(
            opaque(false), w.integration_parameters.dt, ref w.narrow_phase, ref w.colliders,
        );
        assert!(events.is_empty());
    }
    #[test]
    fn gas_tail_world_off() {
        let mut w = world_fixture();
        let events = super::alternatives::collect_world(ref w, opaque(false));
        assert!(events.is_empty());
    }
    #[test]
    fn gas_tail_reduced_on() {
        let mut w = world_fixture();
        let events = super::alternatives::collect_if_enabled(
            opaque(true), w.integration_parameters.dt, ref w.narrow_phase, ref w.colliders,
        );
        assert_eq!(events.len(), 1);
    }
    #[test]
    fn gas_tail_world_on() {
        let mut w = world_fixture();
        let events = super::alternatives::collect_world(ref w, opaque(true));
        assert_eq!(events.len(), 1);
    }
    fn falling() -> World {
        let mut w = WorldTrait::new(Vec2 { x: ZERO, y: opaque(-ONE) }, Default::default());
        w
            .insert(
                RigidBodyTrait::dynamic(Default::default()),
                ColliderBuilderTrait::ball(HALF).build(),
            );
        w.step();
        w.step();
        w
    }
    #[test]
    fn gas_step_dispatch_setup() {
        let _ = falling();
    }
    #[test]
    fn gas_step_dispatch_collision_only() {
        let mut w = falling();
        let _ = w.step();
    }
    #[test]
    fn gas_step_dispatch_unit_payload() {
        let mut w = falling();
        let _ = super::alternatives::step_unit_payload(ref w);
    }
    #[test]
    fn gas_step_dispatch_shipped() {
        let mut w = falling();
        let _ = w.step_with_force_events();
    }
    #[test]
    fn gas_step_dispatch_wallet() {
        let mut w = falling();
        let _ = super::alternatives::step_wallet(ref w);
    }
    #[test]
    fn gas_step_dispatch_world() {
        let mut w = falling();
        let _ = super::alternatives::step_world(ref w);
    }
    #[test]
    fn gas_step_dispatch_reduced() {
        let mut w = falling();
        let _ = super::alternatives::step_reduced(ref w);
    }
    #[test]
    fn gas_step_dispatch_unwalleted() {
        let mut w = falling();
        let _ = super::alternatives::step_unwalleted(ref w);
    }
    #[test]
    fn test_step_dispatch_variants() {
        let mut a = falling();
        let mut b = falling();
        let mut c = falling();
        let mut d = falling();
        let mut e = falling();
        let _ = a.step_with_force_events();
        let _ = super::alternatives::step_world(ref b);
        let _ = super::alternatives::step_reduced(ref c);
        let _ = super::alternatives::step_unwalleted(ref d);
        let _ = super::alternatives::step_unit_payload(ref e);
        let h = rapier_core::Handle { index: 0, generation: 0 };
        let expected = a.body(h);
        assert_eq!(b.body(h), expected);
        assert_eq!(c.body(h), expected);
        assert_eq!(d.body(h), expected);
        assert_eq!(e.body(h), expected);
    }
}

#[cfg(test)]
mod alternatives;

#[cfg(test)]
mod collect_tests;
