//! Tests of the interaction graph views on hand-built pair lists.

use rapier_core::Handle;
use rapier_geometry2d::contact::ContactManifold;
use rapier_testing::opaque;
use crate::events::{
    INTERSECTING, INTERSECTION_PAIR, PairEventStatus, PairEventStatusTrait, START_EVENT_EMITTED,
};
use super::super::{ContactPair, ContactPairTrait, NarrowPhase};
use super::{
    ContactPairView, InteractionGraph, InteractionGraphTrait, IntersectionPair,
    NarrowPhaseInteractionGraphTrait,
};

fn h(index: u32) -> Handle {
    Handle { index, generation: 0 }
}

fn entry(c1: u32, c2: u32, bits: u8, points: u8) -> ContactPair {
    let mut pair = ContactPairTrait::new(h(c1), h(c2));
    pair.event_status = PairEventStatus { bits };
    let mut m: ContactManifold = Default::default();
    m.num_points = points;
    pair.manifold = m;
    pair
}

/// Entries: (0,1) a contact; (0,2) an intersecting sensor pair with its event emitted; (1,3) a
/// composite run of three manifolds; (2,3) a contact; (2,4) a sensor pair not intersecting.
fn narrow_phase() -> NarrowPhase {
    let sensor = INTERSECTION_PAIR.bits + INTERSECTING.bits + START_EVENT_EMITTED.bits;
    NarrowPhase {
        pairs: array![
            entry(0, 1, 0, 1), entry(0, 2, sensor, 0), entry(1, 3, 1, 1), entry(1, 3, 0, 2),
            entry(1, 3, 0, 0), entry(2, 3, 0, 1), entry(2, 4, INTERSECTION_PAIR.bits, 0),
        ],
    }
}

#[test]
fn test_contact_graph() {
    let graph = narrow_phase().contact_graph();
    // One edge per collider pair, sensors excluded.
    let edges = graph.interactions_with_endpoints();
    let keys: Array<(u32, u32, u32)> = array![(0, 1, 1), (1, 3, 3), (2, 3, 1)];
    assert_eq!(edges.len(), keys.len());
    let mut i = 0;
    for (c1, c2, n) in keys {
        let (a, b, view) = *edges.at(i);
        assert_eq!((a, b, view.manifolds.len()), (h(c1), h(c2), n));
        i += 1;
    }
    assert_eq!(graph.interactions().len(), 3);
    assert!((*graph.interactions().at(1)).event_status.start_event_emitted());
}

#[test]
fn test_intersection_graph() {
    let graph = narrow_phase().intersection_graph();
    let edges = graph.interactions_with_endpoints();
    assert_eq!(edges.len(), 2);
    let (a, b, first) = *edges.at(0);
    assert_eq!((a, b), (h(0), h(2)));
    assert_eq!(first, @IntersectionPair { intersecting: true, start_event_emitted: true });
    let (a, b, second) = *edges.at(1);
    assert_eq!((a, b), (h(2), h(4)));
    assert_eq!(second, @IntersectionPair { intersecting: false, start_event_emitted: false });
    assert_eq!(graph.interactions().len(), 2);
}

#[test]
fn test_interaction_pair_and_between() {
    let graph = narrow_phase().contact_graph();
    // (id1, id2, found), in both orders.
    let cases: Array<(u32, u32, bool)> = array![
        (0, 1, true), (1, 0, true), (1, 3, true), (3, 1, true), (2, 3, true), (0, 2, false),
        (0, 3, false), (7, 8, false),
    ];
    for (id1, id2, found) in cases {
        assert_eq!(graph.interaction_pair(id1, id2).is_some(), found);
        assert_eq!(graph.interactions_between(id1, id2).len(), if found {
            1
        } else {
            0
        });
    }
    let (a, b, view) = graph.interaction_pair(3, 1).unwrap();
    assert_eq!((a, b, view.manifolds.len()), (h(1), h(3), 3));
    // The sensor pair is on the intersection graph only.
    let sensors = narrow_phase().intersection_graph();
    assert!(sensors.interaction_pair(2, 0).is_some());
    assert!(sensors.interaction_pair(2, 4).is_some());
    assert!(sensors.interaction_pair(0, 1).is_none());
}

#[test]
fn test_interactions_with() {
    let graph = narrow_phase().contact_graph();
    // (collider slot, interactions)
    let cases: Array<(u32, u32)> = array![(0, 1), (1, 2), (2, 1), (3, 2), (4, 0), (9, 0)];
    for (id, n) in cases {
        assert_eq!(graph.interactions_with(id).len(), n);
    }
    let sensors = narrow_phase().intersection_graph();
    assert_eq!(sensors.interactions_with(2).len(), 2);
    assert_eq!(sensors.interactions_with(4).len(), 1);
    assert_eq!(sensors.interactions_with(3).len(), 0);
}

#[test]
fn test_index_interaction() {
    let graph = narrow_phase().contact_graph();
    let (a, b, _) = graph.index_interaction(2).unwrap();
    assert_eq!((a, b), (h(2), h(3)));
    let (a, b, view) = graph.index_interaction(1).unwrap();
    assert_eq!((a, b, view.manifolds.len()), (h(1), h(3), 3));
    assert!(graph.index_interaction(3).is_none());
    assert!(graph.index_interaction(opaque(0xffffffff_u32)).is_none());
    assert!(narrow_phase().intersection_graph().index_interaction(2).is_none());
}

#[test]
fn test_empty_graph() {
    let empty: InteractionGraph<ContactPairView> = InteractionGraphTrait::new();
    assert_eq!(empty, Default::default());
    assert_eq!(empty.interactions().len(), 0);
    assert_eq!(empty.interactions_with_endpoints().len(), 0);
    assert!(empty.interaction_pair(0, 1).is_none());
    assert_eq!(empty.interactions_between(0, 1).len(), 0);
    assert_eq!(empty.interactions_with(0).len(), 0);
    assert!(empty.index_interaction(0).is_none());
    let np: NarrowPhase = Default::default();
    assert_eq!(np.contact_graph(), empty);
    assert_eq!(np.intersection_graph().interactions().len(), 0);
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

#[test]
fn gas_contact_graph() {
    let np = narrow_phase();
    let _ = opaque(np.contact_graph());
}

#[test]
fn gas_intersection_graph() {
    let np = narrow_phase();
    let _ = opaque(np.intersection_graph());
}

#[test]
fn gas_interactions() {
    let graph = narrow_phase().contact_graph();
    let _ = opaque(graph.interactions().len());
}

#[test]
fn gas_interactions_with_endpoints() {
    let graph = narrow_phase().contact_graph();
    let _ = opaque(graph.interactions_with_endpoints().len());
}

#[test]
fn gas_interaction_pair() {
    let graph = narrow_phase().contact_graph();
    let _ = opaque(graph.interaction_pair(opaque(3), 1).is_some());
}

#[test]
fn gas_interactions_between() {
    let graph = narrow_phase().contact_graph();
    let _ = opaque(graph.interactions_between(opaque(3), 1).len());
}

#[test]
fn gas_interactions_with() {
    let graph = narrow_phase().contact_graph();
    let _ = opaque(graph.interactions_with(opaque(3)).len());
}

#[test]
fn gas_index_interaction() {
    let graph = narrow_phase().contact_graph();
    let _ = opaque(graph.index_interaction(opaque(1)).is_some());
}
