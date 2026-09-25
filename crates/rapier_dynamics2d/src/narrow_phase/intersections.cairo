//! Intersection (sensor) pairs of the narrow phase (work package SE, see the parent module):
//! the step's per-pair update and upstream's intersection queries. The pairs live in the contact
//! pair list, marked by `INTERSECTION_PAIR` in their event status.

use rapier_core::Handle;
use rapier_core::collider::events::SENSOR;
use rapier_geometry2d::contact::ContactManifold;
use rapier_geometry2d::dispatch::intersection_test;
use crate::collider_set::ColliderSet;
use crate::events::{
    CollisionEvent, INTERSECTING, INTERSECTION_PAIR, PairEventStatus, PairEventStatusTrait,
    START_EVENT_EMITTED,
};
use super::{
    ContactPair, ContactPairTrait, PairCollider, dropped_event, events_on, key_before,
    pair_filtered, pair_pose,
};

/// One intersection pair of the step's walk (see the module documentation): finds the previous
/// pair of the key by sorted merge from `cursor` (emitting the `Stopped` of the pairs it passes),
/// runs the filters and `intersection_test`, emits the `SENSOR` transition and appends the pair.
/// Inlined into the pair loop's body, so a world without sensors pays nothing for it.
#[inline(never)]
pub(crate) fn intersection_pair_step(
    co1: PairCollider,
    co2: PairCollider,
    previous: Span<ContactPair>,
    ref cursor: u32,
    ref colliders: ColliderSet,
    ref events: Array<CollisionEvent>,
    ref transitions: Array<CollisionEvent>,
    ref current: Array<ContactPair>,
) {
    let h1 = co1.handle;
    let h2 = co2.handle;
    let mut status = PairEventStatusTrait::empty();
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
            // Same pair and still an intersection pair: carry its status over.
            if a1 == h1 && a2 == h2 && (*head.event_status).is_intersection_pair() {
                status = *head.event_status;
            } else {
                dropped_event(a1, a2, *head.event_status, ref colliders, ref events);
            }
        }
        break;
    }
    let was_intersecting = status.intersecting();
    let mut emitted = status.start_event_emitted();
    let intersecting = !pair_filtered(co1, co2)
        && intersection_test(pair_pose(co1, co2), co1.shape, co2.shape) == Some(true);
    if intersecting != was_intersecting && events_on(co1, co2) {
        emitted = intersecting;
        transitions
            .append(
                if intersecting {
                    CollisionEvent::Started((h1, h2, SENSOR))
                } else {
                    CollisionEvent::Stopped((h1, h2, SENSOR))
                },
            );
    }
    let mut bits = INTERSECTION_PAIR.bits;
    if intersecting {
        bits += INTERSECTING.bits;
    }
    if emitted {
        bits += START_EVENT_EMITTED.bits;
    }
    let mut manifold: ContactManifold = Default::default();
    manifold.data.rigid_body1 = co1.body;
    manifold.data.rigid_body2 = co2.body;
    current
        .append(
            ContactPair {
                collider1: h1, collider2: h2, manifold, event_status: PairEventStatus { bits },
            },
        );
}

/// `NarrowPhaseTrait::intersection_pair` on `pairs`.
pub fn intersection_pair(
    pairs: Span<ContactPair>, collider1: Handle, collider2: Handle,
) -> Option<bool> {
    let mut found = None;
    for pair in pairs {
        let (a, b) = (*pair.collider1, *pair.collider2);
        if (a == collider1 && b == collider2) || (a == collider2 && b == collider1) {
            if pair.is_intersection_pair() {
                found = Some(pair.intersecting());
            }
            break;
        }
    }
    found
}

/// `(collider1, collider2, intersecting)` of the intersection pairs of `pairs` that involve
/// `collider` (every one for `None`), ascending.
pub fn intersection_pairs_with(
    pairs: Span<ContactPair>, collider: Option<Handle>,
) -> Array<(Handle, Handle, bool)> {
    let mut out = array![];
    for pair in pairs {
        let involved = match collider {
            Some(c) => *pair.collider1 == c || *pair.collider2 == c,
            None => true,
        };
        if involved && pair.is_intersection_pair() {
            out.append((*pair.collider1, *pair.collider2, pair.intersecting()));
        }
    }
    out
}
