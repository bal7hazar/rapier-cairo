//! Narrow-phase bookkeeping (upstream `geometry/narrow_phase/`, `NarrowPhase::compute_contacts`
//! and `process_pair`; `geometry/contact_pair.rs`, `ContactPair`).
//!
//! The pair list is rebuilt every step from the stateless broad phase (`docs/PLAN.md` D7): the
//! only state carried from one step to the next is the previous step's [`ContactPair`] of each
//! `(collider1, collider2)` key, whose manifold is handed to the dispatcher for warm-start
//! matching and whose event status drives `CollisionEvent`s (D9).
//!
//! One step of [`NarrowPhaseTrait::compute_contacts`]:
//!
//! 1. one scratch [`PairCollider`] per collider, in the order of `ColliderSetTrait::iter`
//!    (one dict read per collider and per parent body — the only set reads of the step);
//! 2. for each broad-phase pair `(i, j)` (ascending, D8), the previous pair with the same key is
//!    looked up (sorted merge on the previous ascending list: no dict), then upstream's
//!    `process_pair` runs: pairs of disabled or sensor colliders are not contact pairs (sensor
//!    intersections are deferred); same-parent, `ActiveCollisionTypes` and collision-group
//!    filters clear the manifold; otherwise the dispatcher updates the manifold and the solver
//!    data is rebuilt (combined friction / restitution, dominance, solver flags, world normal,
//!    solver contacts with anchors relative to each body's world centre of mass, points at
//!    `dist >= prediction` skipped, `NEW_CONTACT_BIT` on points without warm-start impulse);
//! 3. events: first `Stopped` for the previous pairs absent from this step (ascending key;
//!    `REMOVED` when a collider no longer exists), then the `Started` / `Stopped` transitions of
//!    this step's pairs (ascending key). Transitions follow upstream: emitted when the pair's
//!    "has a solver contact" state flips and either collider has `COLLISION_EVENTS`.
//!
//! The dispatcher is a trait bound, as upstream's `&dyn PersistentQueryDispatcher` (static
//! dispatch here): `rapier_geometry2d::dispatch::contact_manifold` plugs in once it exists.
//!
//! Deviations from upstream: one manifold per pair (every supported shape is convex); no
//! contact skin, no velocity-based speculative contacts (upstream also keeps a point beyond
//! `prediction` when the bodies approach it within `dt`), no solver-contact modification hooks,
//! no contact recycling, no sensor intersection pairs, no contact-force events; an unsupported
//! pair (`contact_manifold` returning `false`) gets its manifold cleared.

use core::num::traits::Zero;
use fixed::Fixed;
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::events::{COLLISION_EVENTS, REMOVED};
use rapier_core::collider::{
    ActiveCollisionTypes, ActiveCollisionTypesTrait, ActiveEvents, ActiveEventsTrait,
    CoefficientCombineRule, CoefficientCombineRuleTrait, CollisionEventFlagsTrait,
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
#[cfg(test)]
pub(crate) mod mock;
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
}

/// The narrow phase: this step's contact pairs, in ascending `(collider1, collider2)` slot index.
#[derive(Drop, Default)]
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

/// How the previous step's pairs are found again by key.
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
/// pairs finds every key; no dict, O(previous + current) comparisons. **Winner** (see
/// `benches`).
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
        compute_contacts_with::<
            D, SortedMerge,
        >(ref self, prediction, ref bodies, ref colliders, pairs)
    }

    /// Number of contact pairs.
    #[inline(always)]
    fn len(self: @NarrowPhase) -> u32 {
        self.pairs.len()
    }

    /// The pair of key `(collider1, collider2)` (slot-index order), if any. Linear scan: for
    /// tests and user queries, not for the step.
    fn contact_pair(
        self: @NarrowPhase, collider1: Handle, collider2: Handle,
    ) -> Option<ContactPair> {
        let mut pairs = self.pairs.span();
        loop {
            match pairs.pop_front() {
                Some(pair) => if *pair.collider1 == collider1 && *pair.collider2 == collider2 {
                    break Some(*pair);
                },
                None => { break None; },
            }
        }
    }
}

/// `compute_contacts` with the carry-over strategy `C`.
pub fn compute_contacts_with<impl D: ContactDispatcher, S, impl C: CarryOver<S>, +Destruct<S>>(
    ref self: NarrowPhase,
    prediction: Fixed,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    pairs: Span<(u32, u32)>,
) -> Array<CollisionEvent> {
    let scratch = pair_colliders(ref bodies, ref colliders).span();
    let mut carry = C::begin(self.pairs.span());
    let mut current = array![];
    let mut transitions = array![];
    for (i, j) in pairs {
        let co1 = *scratch.at(*i);
        let co2 = *scratch.at(*j);
        if !(co1.solid && co2.solid) {
            continue;
        }
        let previous = carry.take(co1.handle, co2.handle);
        let (pair, event) = process_pair::<D>(prediction, co1, co2, previous);
        current.append(pair);
        if let Some(event) = event {
            transitions.append(event);
        }
    }
    let mut events = dropped_events(carry.finish(), ref colliders);
    events.append_span(transitions.span());
    self.pairs = current;
    events
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
            let flags = if colliders.contains(pair.collider1)
                && colliders.contains(pair.collider2) {
                CollisionEventFlagsTrait::empty()
            } else {
                REMOVED
            };
            events.append(stopped(pair.collider1, pair.collider2, flags));
        }
    }
    events
}

/// Upstream `process_pair` for one pair of solid colliders: filters, manifold update, solver
/// data, event transition. Returns the new pair and its event, if any.
pub fn process_pair<impl D: ContactDispatcher>(
    prediction: Fixed, co1: PairCollider, co2: PairCollider, previous: Option<ContactPair>,
) -> (ContactPair, Option<CollisionEvent>) {
    let (manifold, status) = match previous {
        Some(pair) => (pair.manifold, pair.event_status),
        None => (Default::default(), PairEventStatusTrait::empty()),
    };
    let had_contact = manifold.data.num_solver_contacts != 0;
    let manifold = if pair_filtered(co1, co2) {
        Default::default()
    } else {
        update_manifold::<D>(prediction, co1, co2, manifold)
    };
    let mut pair = ContactPair {
        collider1: co1.handle, collider2: co2.handle, manifold, event_status: status,
    };
    let has_contact = manifold.data.num_solver_contacts != 0;
    let mut event = None;
    if has_contact != had_contact
        && (co1.active_events | co2.active_events).contains(COLLISION_EVENTS) {
        if has_contact {
            pair.event_status = START_EVENT_EMITTED;
            event = Some(started(co1.handle, co2.handle));
        } else {
            pair.event_status = PairEventStatusTrait::empty();
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

/// Runs the dispatcher on `manifold` (the previous one) and rebuilds its solver data.
pub fn update_manifold<impl D: ContactDispatcher>(
    prediction: Fixed, co1: PairCollider, co2: PairCollider, manifold: ContactManifold,
) -> ContactManifold {
    let mut manifold = manifold;
    let pos12 = co1.pose.inv_mul(co2.pose);
    if !D::contact_manifold(pos12, co1.shape, co2.shape, prediction, ref manifold) {
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
    manifold
}

/// The solver contact of point `id`: world points relative to each body's world centre of
/// mass, `NEW_CONTACT_BIT` when the point carries no warm-start impulse (upstream: `impulse ==
/// 0`), no tangent velocity.
#[inline(never)]
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
