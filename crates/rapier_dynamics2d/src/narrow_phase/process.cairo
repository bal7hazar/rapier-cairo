//! Upstream's `process_pair` as standalone functions (the pair loop,
//! `super::compute_contacts_from_scratch`, runs the same composition inlined): used by the
//! candidate loops and the tests.

use fixed::Fixed;
use rapier_core::collider::CollisionEventFlagsTrait;
use rapier_geometry2d::contact::ContactManifold;
use crate::events::{
    CollisionEvent, PairEventStatus, PairEventStatusTrait, START_EVENT_EMITTED, started, stopped,
};
use super::{
    ContactDispatcher, ContactPair, PairCollider, events_on, pair_filtered, pair_pose, solver_data,
};

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
