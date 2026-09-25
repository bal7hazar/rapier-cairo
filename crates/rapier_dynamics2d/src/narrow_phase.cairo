//! Narrow-phase bookkeeping (upstream `geometry/narrow_phase/`, `NarrowPhase::compute_contacts`
//! and `process_pair`; `geometry/contact_pair.rs`, `ContactPair`).
//!
//! The pair list is rebuilt every step from the stateless broad phase (`docs/PLAN.md` D7): the
//! only state carried from one step to the next is the previous step's [`ContactPair`] of each
//! `(collider1, collider2)` key, whose manifold is handed to the dispatcher for warm-start
//! matching and whose event status drives `CollisionEvent`s (D9).
//!
//! Sensors (work package SE): a pair of enabled colliders of which one is a sensor is an
//! *intersection pair* (upstream `IntersectionPair`, a separate graph upstream). It lives in the
//! same ascending list, as a [`ContactPair`] with an empty manifold (its parent bodies in
//! `manifold.data.rigid_body1/2`, which the sleeping split reads) whose `event_status` carries
//! `INTERSECTION_PAIR` and `INTERSECTING` (`crate::events`): no solver contact, so it never
//! reaches the solver, the islands or the force events, and the step's walks need no second list
//! (a second `Array` in [`NarrowPhase`] measured +4.5k gas per step on every scene).
//!
//! One step of [`NarrowPhaseTrait::compute_contacts`]:
//!
//! 1. one scratch [`PairCollider`] per collider, in the order of `ColliderSetTrait::iter`
//!    (one dict read per collider and per parent body — the only set reads of the step);
//! 2. for each broad-phase pair `(i, j)` (ascending, D8), the previous pair with the same key is
//!    looked up (sorted merge on the previous ascending list: no dict), then upstream's
//!    `process_pair` runs: pairs of disabled colliders are skipped; for an intersection pair
//!    (upstream `compute_intersections`' `process_pair`), the same filters as below make it
//!    non-intersecting, otherwise `rapier_geometry2d::dispatch::intersection_test` (exact, no
//!    prediction distance; an unsupported pair is not intersecting) decides; same-parent,
//!    `ActiveCollisionTypes` and collision-group filters clear the manifold; otherwise the
//!    dispatcher updates the manifold and the solver data is rebuilt (combined friction /
//!    restitution, dominance, solver flags, world normal, solver contacts with anchors relative to
//!    each body's world centre of mass, points at `dist >= prediction` skipped, `NEW_CONTACT_BIT`
//!    on points without warm-start impulse);
//! 3. events: first `Stopped` for the previous pairs absent from this step (ascending key;
//!    `REMOVED` when a collider no longer exists), then the `Started` / `Stopped` transitions of
//!    this step's pairs (ascending key). Transitions follow upstream: emitted when the pair's
//!    "has a solver contact" state (an intersection pair: `intersecting`) flips and either
//!    collider has `COLLISION_EVENTS`. The events of intersection pairs carry `SENSOR`.
//!
//! The dispatcher is a trait bound, as upstream's `&dyn PersistentQueryDispatcher` (static
//! dispatch here): `rapier2d` plugs in `rapier_geometry2d::dispatch::contact_manifold`.
//!
//! # Cost
//!
//! Sierra gas charges a loop-free function its costliest path, and a loop body only the path it
//! takes. The step's loop, [`compute_contacts_from_scratch`], therefore runs the whole pair
//! (carry-over, filters, dispatcher, solver data, event) inlined in its body, and keeps large
//! values off branch merges (work package ON). Narrow-phase stage of one P3 step, Sierra gas |
//! Cairo steps, before ON (outlined `process_pair`, `CarryOver::take` copying the pair out) →
//! after, with `rapier2d`'s `StepDispatcher`: `cuboid_stack(3)` 2 883 645 | 16 952 →
//! 1 642 485 | 11 680, `balls_on_halfspace(8)` 2 897 650 | 20 150 → 2 116 880 | 12 357,
//! `mixed_pile` 6 722 645 | 47 762 → 4 583 105 | 32 156. Per pair: ball–ball 324 585 | 2 564
//! →
//! 220 845 | 1 529, ball–half-space 385 945 | 2 748 → 281 305 | 1 703, resting cuboid–cuboid
//! 758 095 | 3 271 → 274 285 | 2 000, cuboid–half-space 527 065 | 4 544 → 412 405 | 3 398
//! (`rapier2d::pipeline::narrow_benches`, which also ranks the losing candidates).
//!
//! Deviations from upstream: one manifold per pair (every supported shape is convex); no
//! contact skin, no velocity-based speculative contacts (upstream also keeps a point beyond
//! `prediction` when the bodies approach it within `dt`), no solver-contact modification hooks,
//! no contact recycling; force events are collected by the world; an unsupported pair
//! (`contact_manifold` returning `false`) gets its manifold cleared. Sensors: one list for both
//! kinds (upstream: two graphs), so a sensor's events are interleaved with the contact events in
//! key order (upstream: after them); the intersection test is not pluggable (upstream: the same
//! `QueryDispatcher`); a dropped intersection pair emits its `Stopped` from `start_event_emitted`
//! (upstream: from `intersecting` and the current event flags); a pair whose collider switches
//! between solid and sensor ends (`Stopped` if started) and restarts as the other kind.

use core::num::traits::Zero;
use fixed::Fixed;
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::events::{COLLISION_EVENTS, REMOVED, SENSOR};
use rapier_core::collider::{
    ActiveCollisionTypes, ActiveCollisionTypesTrait, ActiveEvents, ActiveEventsTrait,
    CoefficientCombineRule, CoefficientCombineRuleTrait, CollisionEventFlags,
    CollisionEventFlagsTrait,
};
use rapier_core::interaction_groups::{InteractionGroups, InteractionGroupsTrait};
use rapier_core::rigid_body::{RigidBodyDominanceTrait, RigidBodyType};
use rapier_geometry2d::contact::{
    ContactManifold, NEW_CONTACT_BIT, SOLVER_COMPUTE_RIGID_IMPULSES, SolverContact, SolverFlags,
    TrackedContact,
};
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::collider::components::BoxedOneWayPlatformPartialEq;
use crate::collider::{Collider, ColliderTrait};
use crate::collider_set::{ColliderSet, ColliderSetTrait};
use crate::events::{
    CollisionEvent, PairEventStatus, PairEventStatusTrait, START_EVENT_EMITTED, started, stopped,
};
use crate::rigid_body_set::{RigidBodySet, RigidBodySetTrait};

#[cfg(test)]
mod alternatives;
#[cfg(test)]
mod benches;

pub mod intersections;
#[cfg(test)]
pub(crate) mod mock;
use intersections::intersection_pair_step;
pub mod one_way;
#[cfg(test)]
mod tests;

/// Contact-manifold generation for one convex pair: the frozen GG entry point
/// (`docs/interfaces/geometry-dynamics.md` §2), abstracted as upstream's
/// `PersistentQueryDispatcher`.
pub trait ContactDispatcher {
    /// Updates `manifold` for the pose `pos12` of shape 2 in the frame of shape 1. On entry
    /// `manifold` is the previous step's manifold of the pair (or a default one); the
    /// implementation transfers its `ContactData` to the new points (`match_contacts`). Returns
    /// `false` when the pair of shapes is unsupported.
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool;
}

/// The contact state of two colliders whose AABBs overlap (upstream `ContactPair`, with one
/// manifold instead of a list).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ContactPair {
    pub collider1: Handle,
    pub collider2: Handle,
    /// The manifold and its solver data; `data.num_solver_contacts == 0` when the pair is not
    /// touching (or was filtered out this step).
    pub manifold: ContactManifold,
    pub event_status: PairEventStatus,
}

/// Queries of [`ContactPair`].
#[generate_trait]
pub impl ContactPairImpl of ContactPairTrait {
    /// A pair with an empty manifold and no event emitted.
    #[inline(always)]
    fn new(collider1: Handle, collider2: Handle) -> ContactPair {
        ContactPair {
            collider1,
            collider2,
            manifold: Default::default(),
            event_status: PairEventStatusTrait::empty(),
        }
    }

    /// `true` when the manifold holds at least one solver contact (upstream
    /// `has_any_active_contact`).
    #[inline(always)]
    fn has_any_active_contact(self: @ContactPair) -> bool {
        *self.manifold.data.num_solver_contacts != 0
    }

    /// `true` for an intersection (sensor) pair (see the module documentation).
    #[inline(always)]
    fn is_intersection_pair(self: @ContactPair) -> bool {
        (*self.event_status).is_intersection_pair()
    }

    /// `true` for an intersection pair whose shapes intersect (upstream
    /// `IntersectionPair::intersecting`); `false` for a contact pair.
    #[inline(always)]
    fn intersecting(self: @ContactPair) -> bool {
        (*self.event_status).intersecting()
    }
}

/// The narrow phase: this step's contact and intersection (sensor) pairs, in ascending
/// `(collider1, collider2)` slot index. Its whole content is persistent state (D9), saved as is
/// by `rapier2d`'s `WorldState`.
#[derive(Drop, Default, Serde, PartialEq, Debug)]
pub struct NarrowPhase {
    pub pairs: Array<ContactPair>,
}

/// Per-step scratch copy of what `process_pair` reads from a collider and its parent body.
/// Built once per collider, so that the pair loop does no set read.
#[derive(Copy, Drop, Debug, PartialEq)]
pub struct PairCollider {
    pub handle: Handle,
    /// `false` for disabled and sensor colliders, which take part in no contact pair.
    pub solid: bool,
    /// `true` for an enabled sensor, which takes part in intersection pairs only.
    pub sensor: bool,
    pub shape: Shape,
    pub pose: Pose2,
    pub friction: Fixed,
    pub restitution: Fixed,
    pub friction_combine_rule: CoefficientCombineRule,
    pub restitution_combine_rule: CoefficientCombineRule,
    pub active_collision_types: ActiveCollisionTypes,
    pub collision_groups: InteractionGroups,
    pub solver_groups: InteractionGroups,
    pub active_events: ActiveEvents,
    /// Optional platform cone, copied without expanding its boxed data.
    pub one_way: Box<Option<crate::collider::components::OneWayPlatform>>,
    /// Parent body; `None` for a standalone collider, which behaves as attached to a fixed body.
    pub body: Option<Handle>,
    pub body_type: RigidBodyType,
    /// Parent's world centre of mass (the origin without a parent: anchors stay world points).
    pub world_com: Vec2,
    /// Parent's effective dominance group (upstream `effective_group`).
    pub dominance: i16,
}

/// Builds the [`PairCollider`] of `collider`. Cost: one dict read for a parented collider.
pub fn pair_collider(handle: Handle, collider: Collider, ref bodies: RigidBodySet) -> PairCollider {
    let (body_type, world_com, dominance) = match collider.parent() {
        Some(parent) => match bodies.get(parent) {
            Some(body) => (body.body_type, body.mprops.world_com, body.dominance),
            None => (RigidBodyType::Fixed, Default::default(), Default::default()),
        },
        None => (RigidBodyType::Fixed, Default::default(), Default::default()),
    };
    PairCollider {
        handle,
        solid: collider.is_enabled() && !collider.is_sensor(),
        sensor: collider.is_enabled() && collider.is_sensor(),
        shape: collider.shape,
        pose: collider.pos.pose,
        friction: collider.material.friction,
        restitution: collider.material.restitution,
        friction_combine_rule: collider.material.friction_combine_rule,
        restitution_combine_rule: collider.material.restitution_combine_rule,
        active_collision_types: collider.flags.active_collision_types,
        collision_groups: collider.flags.collision_groups,
        solver_groups: collider.flags.solver_groups,
        active_events: collider.flags.active_events,
        one_way: collider.one_way,
        body: collider.parent(),
        body_type,
        world_com,
        dominance: dominance.effective_group(body_type),
    }
}

/// One [`PairCollider`] per collider, in the order of `ColliderSetTrait::iter`.
pub fn pair_colliders(ref bodies: RigidBodySet, ref colliders: ColliderSet) -> Array<PairCollider> {
    let mut out = array![];
    for (handle, collider) in colliders.iter() {
        out.append(pair_collider(handle, collider, ref bodies));
    }
    out
}

/// How the previous step's pairs are found again by key: the interface of the candidate pair
/// loops (`alternatives`, `rapier2d::pipeline::alternatives`); the shipped loop,
/// [`compute_contacts_from_scratch`], walks [`SortedMerge`] inline.
pub trait CarryOver<S> {
    /// Starts a lookup session over the previous pairs (ascending key).
    fn begin(previous: Span<ContactPair>) -> S;
    /// The previous pair of key `(collider1, collider2)`, if any. Keys are queried in
    /// strictly ascending order.
    fn take(ref self: S, collider1: Handle, collider2: Handle) -> Option<ContactPair>;
    /// The previous pairs never taken, in ascending key.
    fn finish(self: S) -> Array<ContactPair>;
}

/// Carry-over by sorted merge: both lists are ascending, so one forward walk over the previous
/// pairs finds every key; no dict, O(previous + current) comparisons. Beats the dict (see
/// `benches`); the shipped loop runs the same walk by index.
#[derive(Drop)]
pub struct SortedMerge {
    previous: Span<ContactPair>,
    dropped: Array<ContactPair>,
}

/// `true` when the key `(a1, a2)` sorts before `(b1, b2)` (slot indices, D8).
#[inline(always)]
pub fn key_before(a1: Handle, a2: Handle, b1: Handle, b2: Handle) -> bool {
    a1.index < b1.index || (a1.index == b1.index && a2.index < b2.index)
}

pub impl SortedMergeCarryOver of CarryOver<SortedMerge> {
    #[inline(always)]
    fn begin(previous: Span<ContactPair>) -> SortedMerge {
        SortedMerge { previous, dropped: array![] }
    }

    fn take(ref self: SortedMerge, collider1: Handle, collider2: Handle) -> Option<ContactPair> {
        loop {
            let Some(head) = self.previous.get(0) else {
                break None;
            };
            let pair = *head.unbox();
            if key_before(pair.collider1, pair.collider2, collider1, collider2) {
                self.dropped.append(pair);
                let _ = self.previous.pop_front();
                continue;
            }
            if pair.collider1.index == collider1.index && pair.collider2.index == collider2.index {
                let _ = self.previous.pop_front();
                // Same slots, other generations: a collider was replaced; the old pair is gone.
                if pair.collider1 == collider1 && pair.collider2 == collider2 {
                    break Some(pair);
                }
                self.dropped.append(pair);
            }
            break None;
        }
    }

    fn finish(self: SortedMerge) -> Array<ContactPair> {
        let SortedMerge { previous, mut dropped } = self;
        dropped.append_span(previous);
        dropped
    }
}

/// Operations of [`NarrowPhase`].
#[generate_trait]
pub impl NarrowPhaseImpl of NarrowPhaseTrait {
    /// An empty narrow phase.
    #[inline(always)]
    fn new() -> NarrowPhase {
        NarrowPhase { pairs: array![] }
    }

    /// Rebuilds `self.pairs` for this step and returns the collision events (see the module
    /// documentation for the steps and the event order).
    ///
    /// `pairs` are the index pairs returned by `rapier_geometry2d::broad_phase::find_pairs` on
    /// `ColliderSetTrait::broad_phase_proxies`: indices into `colliders.iter()`, `i < j`, in
    /// ascending order. `prediction` is the prediction distance (upstream
    /// `IntegrationParameters::prediction_distance`).
    ///
    /// # Panics
    /// When an index is out of range, and as the dispatcher and the fixed-point pose kernels.
    fn compute_contacts<impl D: ContactDispatcher>(
        ref self: NarrowPhase,
        prediction: Fixed,
        ref bodies: RigidBodySet,
        ref colliders: ColliderSet,
        pairs: Span<(u32, u32)>,
    ) -> Array<CollisionEvent> {
        let scratch = pair_colliders(ref bodies, ref colliders).span();
        compute_contacts_from_scratch::<D>(ref self, prediction, scratch, pairs, ref colliders)
    }

    /// Number of pairs, contact and intersection.
    #[inline(always)]
    fn len(self: @NarrowPhase) -> u32 {
        self.pairs.len()
    }

    /// The contact pair of key `(collider1, collider2)` (slot-index order), if any; `None` for
    /// an intersection pair (upstream: another graph). Linear scan: for tests and user queries,
    /// not for the step.
    fn contact_pair(
        self: @NarrowPhase, collider1: Handle, collider2: Handle,
    ) -> Option<ContactPair> {
        let mut pairs = self.pairs.span();
        loop {
            match pairs.pop_front() {
                Some(pair) => if *pair.collider1 == collider1 && *pair.collider2 == collider2 {
                    if pair.is_intersection_pair() {
                        break None;
                    }
                    break Some(*pair);
                },
                None => { break None; },
            }
        }
    }

    /// Upstream `NarrowPhase::intersection_pair`: `Some(intersecting)` for the intersection pair
    /// of `collider1` and `collider2` (either order), `None` when they form none. Linear scan.
    fn intersection_pair(self: @NarrowPhase, collider1: Handle, collider2: Handle) -> Option<bool> {
        intersections::intersection_pair(self.pairs.span(), collider1, collider2)
    }

    /// Upstream `NarrowPhase::intersection_pairs_with`: `(collider1, collider2, intersecting)`
    /// of every intersection pair involving `collider`, ascending.
    fn intersection_pairs_with(
        self: @NarrowPhase, collider: Handle,
    ) -> Array<(Handle, Handle, bool)> {
        intersections::intersection_pairs_with(self.pairs.span(), Some(collider))
    }

    /// Upstream `NarrowPhase::intersection_pairs`: every intersection pair as
    /// `(collider1, collider2, intersecting)`, ascending.
    fn intersection_pairs(self: @NarrowPhase) -> Array<(Handle, Handle, bool)> {
        intersections::intersection_pairs_with(self.pairs.span(), None)
    }
}

/// `compute_contacts` on a prebuilt scratch: one [`PairCollider`] per collider, in the order of
/// `ColliderSetTrait::iter` (as [`pair_colliders`]; `rapier2d`'s fused step builds it with the
/// broad-phase proxies). `pairs` index into `scratch`; `colliders` is only read for the
/// `REMOVED` flag of the pairs that are dropped.
///
/// The body is [`process_pair`] and [`SortedMerge`] spelled out in the loop (an impl-generic
/// function cannot be `#[inline(always)]`): in a loop body every branch is charged only when
/// taken, whereas an outlined loop-free function is charged its costliest path on every pair.
/// For the same reason nothing large crosses a branch: the previous pair is a one-felt `Box`
/// into the previous list (a fresh default pair when absent), a filtered pair leaves on its own
/// `continue`, the event transition works on scalars and each pair is built once, in `append`.
/// The dispatcher `D` is inlined too when its `contact_manifold` is `#[inline(always)]`.
///
/// # Panics
/// As `NarrowPhaseTrait::compute_contacts`.
pub fn compute_contacts_from_scratch<impl D: ContactDispatcher>(
    ref self: NarrowPhase,
    prediction: Fixed,
    scratch: Span<PairCollider>,
    pairs: Span<(u32, u32)>,
    ref colliders: ColliderSet,
) -> Array<CollisionEvent> {
    let previous = self.pairs.span();
    let fresh_pair = ContactPairTrait::new(Default::default(), Default::default());
    let fresh = BoxTrait::new(@fresh_pair);
    let mut cursor: u32 = 0;
    let mut current = array![];
    let mut events = array![];
    let mut transitions = array![];
    for (i, j) in pairs {
        let co1 = *scratch.at(*i);
        let co2 = *scratch.at(*j);
        if !(co1.solid && co2.solid) {
            if (co1.solid || co1.sensor) && (co2.solid || co2.sensor) {
                intersection_pair_step(
                    co1,
                    co2,
                    previous,
                    ref cursor,
                    ref colliders,
                    ref events,
                    ref transitions,
                    ref current,
                );
            }
            continue;
        }
        let h1 = co1.handle;
        let h2 = co2.handle;
        let mut found = fresh;
        while let Some(boxed) = previous.get(cursor) {
            let head = boxed.unbox();
            let a1 = *head.collider1;
            let a2 = *head.collider2;
            if key_before(a1, a2, h1, h2) {
                dropped_event(a1, a2, *head.event_status, ref colliders, ref events);
                cursor += 1;
                continue;
            }
            if a1.index == h1.index && a2.index == h2.index {
                cursor += 1;
                // Same slots, other generations, or a sensor pair turned solid: a new pair.
                if a1 == h1 && a2 == h2 && !(*head.event_status).is_intersection_pair() {
                    found = boxed;
                } else {
                    dropped_event(a1, a2, *head.event_status, ref colliders, ref events);
                }
            }
            break;
        }
        let prev = found.unbox();
        let status = *prev.event_status;
        let had_contact = *prev.manifold.data.num_solver_contacts != 0;
        if pair_filtered(co1, co2) {
            let mut event_status = status;
            if had_contact && events_on(co1, co2) {
                event_status.bits = event_status.bits & 252;
                transitions.append(stopped(h1, h2, CollisionEventFlagsTrait::empty()));
            }
            current
                .append(
                    ContactPair {
                        collider1: h1, collider2: h2, manifold: Default::default(), event_status,
                    },
                );
            continue;
        }
        let mut manifold = *prev.manifold;
        let supported = D::contact_manifold(
            pair_pose(co1, co2), co1.shape, co2.shape, prediction, ref manifold,
        );
        let manifold = solver_data(prediction, co1, co2, manifold, supported);
        let has_contact = manifold.data.num_solver_contacts != 0;
        let mut event_status = status;
        if has_contact != had_contact && events_on(co1, co2) {
            if has_contact {
                event_status.bits = event_status.bits | START_EVENT_EMITTED.bits;
                transitions.append(started(h1, h2));
            } else {
                event_status.bits = event_status.bits & 252;
                transitions.append(stopped(h1, h2, CollisionEventFlagsTrait::empty()));
            }
        }
        current.append(ContactPair { collider1: h1, collider2: h2, manifold, event_status });
    }
    while let Some(boxed) = previous.get(cursor) {
        let head = boxed.unbox();
        dropped_event(
            *head.collider1, *head.collider2, *head.event_status, ref colliders, ref events,
        );
        cursor += 1;
    }
    events.append_span(transitions.span());
    self.pairs = current;
    events
}

/// `true` when either collider has `COLLISION_EVENTS`.
#[inline(always)]
fn events_on(co1: PairCollider, co2: PairCollider) -> bool {
    (co1.active_events | co2.active_events).contains(COLLISION_EVENTS)
}

/// The `Stopped` event of a dropped previous pair, if its `Started` was emitted (see
/// [`dropped_events`]); flagged `SENSOR` for an intersection pair.
#[inline(always)]
fn dropped_event(
    collider1: Handle,
    collider2: Handle,
    status: PairEventStatus,
    ref colliders: ColliderSet,
    ref events: Array<CollisionEvent>,
) {
    if status.start_event_emitted() {
        events
            .append(
                stopped(
                    collider1,
                    collider2,
                    dropped_flags(collider1, collider2, status, ref colliders),
                ),
            );
    }
}

/// Flags of the `Stopped` event of a dropped pair: `REMOVED` when a collider no longer exists,
/// `SENSOR` for an intersection pair. One or two dict reads.
#[inline(always)]
fn dropped_flags(
    collider1: Handle, collider2: Handle, status: PairEventStatus, ref colliders: ColliderSet,
) -> CollisionEventFlags {
    let flags = if colliders.contains(collider1) && colliders.contains(collider2) {
        CollisionEventFlagsTrait::empty()
    } else {
        REMOVED
    };
    if status.is_intersection_pair() {
        flags | SENSOR
    } else {
        flags
    }
}

/// `Stopped` events of the previous pairs that are not pairs any more: flags empty when both
/// colliders still exist, `REMOVED` otherwise (upstream emits the latter from `remove`).
/// Only pairs whose `Started` was emitted produce an event. One or two dict reads per such pair.
pub fn dropped_events(
    dropped: Array<ContactPair>, ref colliders: ColliderSet,
) -> Array<CollisionEvent> {
    let mut events = array![];
    for pair in dropped {
        if pair.event_status.start_event_emitted() {
            let flags = dropped_flags(
                pair.collider1, pair.collider2, pair.event_status, ref colliders,
            );
            events.append(stopped(pair.collider1, pair.collider2, flags));
        }
    }
    events
}

/// Upstream `process_pair` for one pair of solid colliders: filters, manifold update, solver
/// data, event transition. Returns the new pair and its event, if any. The pair loop
/// ([`compute_contacts_from_scratch`]) runs the same composition inlined.
pub fn process_pair<impl D: ContactDispatcher>(
    prediction: Fixed, co1: PairCollider, co2: PairCollider, previous: Option<ContactPair>,
) -> (ContactPair, Option<CollisionEvent>) {
    let (manifold, status) = previous_state(previous);
    let had_contact = manifold.data.num_solver_contacts != 0;
    let manifold = if pair_filtered(co1, co2) {
        Default::default()
    } else {
        update_manifold::<D>(prediction, co1, co2, manifold)
    };
    pair_transition(co1, co2, manifold, status, had_contact)
}

/// The manifold and event status carried over from `previous`: a default manifold and no
/// event emitted for a new pair.
#[inline(always)]
pub fn previous_state(previous: Option<ContactPair>) -> (ContactManifold, PairEventStatus) {
    match previous {
        Some(pair) => (pair.manifold, pair.event_status),
        None => (Default::default(), PairEventStatusTrait::empty()),
    }
}

/// The pair built from its new `manifold`, with its `Started` / `Stopped` transition: emitted
/// when the "has a solver contact" state differs from `had_contact` and either collider has
/// `COLLISION_EVENTS`.
#[inline(always)]
pub fn pair_transition(
    co1: PairCollider,
    co2: PairCollider,
    manifold: ContactManifold,
    status: PairEventStatus,
    had_contact: bool,
) -> (ContactPair, Option<CollisionEvent>) {
    let mut pair = ContactPair {
        collider1: co1.handle, collider2: co2.handle, manifold, event_status: status,
    };
    let has_contact = manifold.data.num_solver_contacts != 0;
    let mut event = None;
    if has_contact != had_contact && events_on(co1, co2) {
        if has_contact {
            pair.event_status.bits = pair.event_status.bits | START_EVENT_EMITTED.bits;
            event = Some(started(co1.handle, co2.handle));
        } else {
            pair.event_status.bits = pair.event_status.bits & 252;
            event = Some(stopped(co1.handle, co2.handle, CollisionEventFlagsTrait::empty()));
        }
    }
    (pair, event)
}

/// Upstream's filters, in order: same parent body, `ActiveCollisionTypes` (neither collider
/// accepts the pair of body types), collision groups.
#[inline(always)]
pub fn pair_filtered(co1: PairCollider, co2: PairCollider) -> bool {
    if let Some(body1) = co1.body {
        if co2.body == Some(body1) {
            return true;
        }
    }
    if !co1.active_collision_types.test(co1.body_type, co2.body_type)
        && !co2.active_collision_types.test(co1.body_type, co2.body_type) {
        return true;
    }
    !co1.collision_groups.test(co2.collision_groups)
}

/// The pose of collider 2 in the frame of collider 1 (`pos12` of the dispatcher).
#[inline(always)]
pub fn pair_pose(co1: PairCollider, co2: PairCollider) -> Pose2 {
    co1.pose.inv_mul(co2.pose)
}

/// Runs the dispatcher on `manifold` (the previous one) and rebuilds its solver data.
pub fn update_manifold<impl D: ContactDispatcher>(
    prediction: Fixed, co1: PairCollider, co2: PairCollider, manifold: ContactManifold,
) -> ContactManifold {
    let mut manifold = manifold;
    let supported = D::contact_manifold(
        pair_pose(co1, co2), co1.shape, co2.shape, prediction, ref manifold,
    );
    solver_data(prediction, co1, co2, manifold, supported)
}

/// The solver data of `manifold`, which the dispatcher just updated (`supported == false`
/// clears its points): bodies, solver flags, combined friction and restitution, relative
/// dominance, world normal, and one solver contact per point at `dist < prediction`.
#[inline(always)]
pub fn solver_data(
    prediction: Fixed,
    co1: PairCollider,
    co2: PairCollider,
    manifold: ContactManifold,
    supported: bool,
) -> ContactManifold {
    let mut manifold = manifold;
    if !supported {
        manifold.num_points = 0;
    }
    manifold.data.rigid_body1 = co1.body;
    manifold.data.rigid_body2 = co2.body;
    manifold
        .data
        .solver_flags =
            SolverFlags {
                bits: if co1.solver_groups.test(co2.solver_groups) {
                    SOLVER_COMPUTE_RIGID_IMPULSES
                } else {
                    0
                },
            };
    manifold
        .data
        .friction =
            CoefficientCombineRuleTrait::combine(
                co1.friction, co2.friction, co1.friction_combine_rule, co2.friction_combine_rule,
            );
    manifold
        .data
        .restitution =
            CoefficientCombineRuleTrait::combine(
                co1.restitution,
                co2.restitution,
                co1.restitution_combine_rule,
                co2.restitution_combine_rule,
            );
    manifold.data.relative_dominance = co1.dominance - co2.dominance;
    manifold.data.normal = co1.pose.rotation.rotate(manifold.local_n1);

    let [p0, p1] = manifold.points;
    let mut first: SolverContact = Default::default();
    let mut second: SolverContact = Default::default();
    let mut count: u8 = 0;
    if manifold.num_points != 0 && p0.dist < prediction {
        first = solver_contact(p0, 0, co1, co2);
        count = 1;
    }
    if manifold.num_points > 1 && p1.dist < prediction {
        let contact = solver_contact(p1, 1, co1, co2);
        if count == 0 {
            first = contact;
        } else {
            second = contact;
        }
        count += 1;
    }
    manifold.data.solver_contacts = [first, second];
    manifold.data.num_solver_contacts = count;
    if co1.one_way.unbox().is_some() || co2.one_way.unbox().is_some() {
        one_way::filter(ref manifold, co1, co2);
    }
    manifold
}

/// The solver contact of point `id`: world points relative to each body's world centre of
/// mass, `NEW_CONTACT_BIT` when the point carries no warm-start impulse (upstream: `impulse ==
/// 0`), no tangent velocity.
#[inline(always)]
pub fn solver_contact(
    point: TrackedContact, id: u32, co1: PairCollider, co2: PairCollider,
) -> SolverContact {
    let world1 = co1.pose.transform_point(point.local_p1);
    let world2 = co2.pose.transform_point(point.local_p2);
    SolverContact {
        anchor1: world1 - co1.world_com,
        anchor2: world2 - co2.world_com,
        dist: point.dist,
        tangent_velocity: Default::default(),
        contact_id: if point.data.impulse.is_zero() {
            id + NEW_CONTACT_BIT
        } else {
            id
        },
    }
}
