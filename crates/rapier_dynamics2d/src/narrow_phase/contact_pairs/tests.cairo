//! Tests of the contact-pair read API on hand-built pair lists.

use fixed::{FixedTrait, HALF, ONE, TWO, ZERO};
use glam_core::Vec2;
use rapier_core::Handle;
use rapier_geometry2d::contact::{ContactManifold, TrackedContact};
use rapier_testing::opaque;
use crate::events::{PairEventStatus, PairEventStatusTrait, START_EVENT_EMITTED};
use super::super::{ContactPair, ContactPairTrait, NarrowPhase, NarrowPhaseTrait};
use super::{ContactPairView, ContactPairViewTrait, NarrowPhaseContactPairsTrait};

fn h(index: u32) -> Handle {
    Handle { index, generation: 0 }
}

/// A manifold with `points` `(dist, impulse)` and world normal `(0, 1)`; solver contacts
/// counted as points.
fn manifold(points: Array<(fixed::Fixed, fixed::Fixed)>) -> ContactManifold {
    let mut m: ContactManifold = Default::default();
    let mut list = array![];
    for (dist, impulse) in points {
        let mut p: TrackedContact = Default::default();
        p.dist = dist;
        p.data.impulse = impulse;
        list.append(p);
    }
    m.num_points = list.len().try_into().unwrap();
    let p0 = *list.at(0);
    let p1 = if list.len() > 1 {
        *list.at(1)
    } else {
        Default::default()
    };
    m.points = [p0, p1];
    m.data.num_solver_contacts = m.num_points;
    m.data.normal = Vec2 { x: ZERO, y: ONE };
    m
}

fn entry(c1: u32, c2: u32, bits: u8, m: ContactManifold) -> ContactPair {
    let mut pair = ContactPairTrait::new(h(c1), h(c2));
    pair.event_status = PairEventStatus { bits };
    pair.manifold = m;
    pair
}

/// Entries: (0,1) one manifold with two contacts; (0,2) a sensor pair; (1,3) a composite run of
/// three manifolds (the first holding the event status); (2,3) a pair without contact.
fn narrow_phase() -> NarrowPhase {
    let deep = FixedTrait::from_int(-1);
    let a = manifold(array![(ZERO, ONE), (deep, TWO)]);
    let b = manifold(array![(HALF, HALF)]);
    let c = manifold(array![(deep * TWO, ONE)]);
    let d: ContactManifold = Default::default();
    NarrowPhase {
        pairs: array![
            entry(0, 1, START_EVENT_EMITTED.bits, a), entry(0, 2, 13, Default::default()),
            entry(1, 3, START_EVENT_EMITTED.bits, b), entry(1, 3, 0, c), entry(1, 3, 0, d),
            entry(2, 3, 0, d),
        ],
    }
}

#[test]
fn test_contact_pairs() {
    let np = narrow_phase();
    let all = np.contact_pairs();
    let keys: Array<(u32, u32, u32)> = array![(0, 1, 1), (1, 3, 3), (2, 3, 1)];
    assert_eq!(all.len(), keys.len());
    let mut i = 0;
    for (c1, c2, n) in keys {
        let pair = all.at(i);
        assert_eq!((*pair.collider1, *pair.collider2, pair.manifolds().len()), (h(c1), h(c2), n));
        i += 1;
    }
    // The run leader's status is the pair's.
    assert!(all.at(1).event_status.start_event_emitted());
    // Sensor pairs are not contact pairs.
    assert!(np.contact_pair_view(h(0), h(2)).is_none());
    // Involving a collider, by handle and by slot.
    assert_eq!(np.contact_pairs_with(h(3)).len(), 2);
    assert_eq!(np.contact_pairs_with(h(0)).len(), 1);
    assert_eq!(np.contact_pairs_with(h(9)).len(), 0);
    assert_eq!(np.contact_pairs_with(Handle { index: 3, generation: 5 }).len(), 0);
    assert_eq!(np.contact_pairs_with_unknown_gen(3).len(), 2);
    assert_eq!(np.contact_pairs_with_unknown_gen(1).len(), 2);
    assert_eq!(np.contact_pairs_with_unknown_gen(9).len(), 0);
}

#[test]
fn test_contact_pair_lookups() {
    let np = narrow_phase();
    // (collider1, collider2, exact match, unknown_gen match, manifolds)
    let cases: Array<(u32, u32, bool, bool, u32)> = array![
        (0, 1, true, true, 1), (1, 0, false, true, 1), (1, 3, true, true, 3),
        (3, 1, false, true, 3), (0, 2, false, false, 0), (2, 3, true, true, 1),
        (0, 3, false, false, 0), (7, 8, false, false, 0),
    ];
    for (c1, c2, exact, unknown, n) in cases {
        let by_handle = np.contact_pair_view(h(c1), h(c2));
        assert_eq!(by_handle.is_some(), exact);
        let by_slot = np.contact_pair_unknown_gen(c1, c2);
        assert_eq!(by_slot.is_some(), unknown);
        if unknown {
            assert_eq!(by_slot.unwrap().manifolds().len(), n);
        }
    }
    // A stale generation only matches through `unknown_gen`.
    let stale = Handle { index: 1, generation: 4 };
    assert!(np.contact_pair_view(h(0), stale).is_none());
    // The frozen `contact_pair` still answers the run's first entry.
    assert_eq!(np.contact_pair(h(1), h(3)).unwrap().manifold.num_points, 1);
}

#[test]
fn test_contact_pair_at_index() {
    let np = narrow_phase();
    // Entry 1 is a sensor pair, entries 3 and 4 continue the run led by entry 2.
    let cases: Array<(u32, Option<u32>)> = array![
        (0, Some(1)), (1, None), (2, Some(3)), (3, None), (4, None), (5, Some(1)), (6, None),
    ];
    for (i, expected) in cases {
        let got = np.contact_pair_at_index(i);
        match expected {
            Some(n) => assert_eq!(got.unwrap().manifolds().len(), n),
            None => assert!(got.is_none()),
        }
    }
}

#[test]
fn test_contact_pair_accessors() {
    let np = narrow_phase();
    let mut single = np.contact_pair_view(h(0), h(1)).unwrap();
    assert!(single.has_any_active_contact());
    assert_eq!(single.total_impulse_magnitude(), ONE + TWO);
    assert_eq!(single.total_impulse(), Vec2 { x: ZERO, y: ONE + TWO });
    assert_eq!(single.max_impulse(), (ONE + TWO, Vec2 { x: ZERO, y: ONE }));
    let (m, deepest) = single.find_deepest_contact().unwrap();
    assert_eq!((m.num_points, deepest.dist), (2, FixedTrait::from_int(-1)));
    assert_eq!(single.solver_manifolds().len(), 1);
    assert_eq!(single.rigid().unwrap().manifolds().len(), 1);

    // The composite run: impulses 0.5 + 1 + 0, deepest contact in the second manifold.
    let mut run = np.contact_pair_view(h(1), h(3)).unwrap();
    assert_eq!(run.total_impulse_magnitude(), HALF + ONE);
    assert_eq!(run.max_impulse(), (ONE, Vec2 { x: ZERO, y: ONE }));
    let (m, deepest) = run.find_deepest_contact().unwrap();
    assert_eq!((m.num_points, deepest.dist), (1, FixedTrait::from_int(-2)));

    // A pair without contact.
    let mut idle = np.contact_pair_view(h(2), h(3)).unwrap();
    assert!(!idle.has_any_active_contact());
    assert_eq!(idle.max_impulse(), (ZERO, Default::default()));
    assert!(idle.find_deepest_contact().is_none());
    assert_eq!(idle.total_impulse(), Default::default());

    run.clear();
    assert_eq!(run.manifolds().len(), 0);
    assert!(!run.has_any_active_contact());
    let empty: ContactPairView = Default::default();
    assert_eq!(empty.manifolds().len(), 0);
    assert_eq!(empty.event_status, PairEventStatusTrait::empty());
}

#[test]
fn test_empty_narrow_phase() {
    let np = NarrowPhaseTrait::new();
    assert_eq!(np.contact_pairs().len(), 0);
    assert!(np.contact_pair_at_index(0).is_none());
    assert!(np.contact_pair_unknown_gen(0, 1).is_none());
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_contact_pairs() {
    let np = narrow_phase();
    let _ = opaque(np.contact_pairs());
}

#[test]
fn gas_contact_pairs_with() {
    let np = narrow_phase();
    let _ = opaque(np.contact_pairs_with(opaque(h(3))));
}

#[test]
fn gas_contact_pairs_with_unknown_gen() {
    let np = narrow_phase();
    let _ = opaque(np.contact_pairs_with_unknown_gen(opaque(3)));
}

#[test]
fn gas_contact_pair_unknown_gen() {
    let np = narrow_phase();
    let _ = opaque(np.contact_pair_unknown_gen(opaque(3), 1));
}

#[test]
fn gas_contact_pair_at_index() {
    let np = narrow_phase();
    let _ = opaque(np.contact_pair_at_index(opaque(2)));
}

#[test]
fn gas_contact_pair_view() {
    let np = narrow_phase();
    let _ = opaque(np.contact_pair_view(opaque(h(1)), h(3)));
}

#[test]
fn gas_view_accessors() {
    let np = narrow_phase();
    let run = np.contact_pair_view(h(1), h(3)).unwrap();
    let _ = opaque(run.total_impulse());
    let _ = opaque(run.total_impulse_magnitude());
    let _ = opaque(run.max_impulse());
    let _ = opaque(run.find_deepest_contact());
    let _ = opaque(run.has_any_active_contact());
}
