//! A read-only `InteractionGraph` over the step's pair list (work package IG1, off the step path):
//! upstream's `geometry/interaction_graph.rs` and `NarrowPhase::{contact_graph,
//! intersection_graph}`.
//!
//! # Representation
//!
//! There is no persistent graph (ADR 0001 D7: the pair list is rebuilt every step). The graph is a
//! value built from the pair list by [`NarrowPhaseInteractionGraphTrait::contact_graph`] /
//! [`NarrowPhaseInteractionGraphTrait::intersection_graph`]: a list of edges, each holding the two
//! colliders and an edge value, in ascending `(collider1, collider2)` order (the order of the pair
//! list). It is generic over the edge value, the node value being always a [`Handle`]:
//!
//! * `InteractionGraph<ContactPairView>`: one edge per collider pair (a composite pair is a run of
//!   entries, ADR 0001 entry 35, gathered as in `contact_pairs`); sensor pairs are not edges.
//! * `InteractionGraph<IntersectionPair>`: one edge per sensor pair.
//!
//! The graph is a copy: it does not follow the narrow phase, and is meant for tests and user
//! queries, not for the step.
//!
//! # Indices
//!
//! A node (a collider) is identified by its slot index ([`ColliderGraphIndex`]); an edge by its
//! position in the graph ([`TemporaryInteractionIndex`]), i.e. its rank among the pairs of the
//! step's pair list. Both are indices into the pair list of the step the graph was built from,
//! valid until the next step, not persistent graph indices.
//!
//! # Deviations
//!
//! * Reads return values: `&E` becomes a snapshot `@E`, iterators become arrays.
//! * Edges are undirected, as upstream's lookups (`find_edge`, `edges_between`) are; a collider
//!   pair is stored once, as `(collider1, collider2)` of the pair list.
//! * `interactions_with` lists the edges of a collider in graph order (upstream: reverse order of
//!   insertion).
//! * `raw_graph`, `interaction_pair_mut`, `interactions_with_mut` and `InteractionsWithMut` are not
//!   ported (mutable references into the step's storage).

use rapier_core::Handle;
use crate::events::PairEventStatusTrait;
use super::contact_pairs::{ContactPairView, contact_pairs};
use super::{ContactPair, ContactPairTrait, NarrowPhase};

#[cfg(test)]
mod tests;

/// The index of a collider in an [`InteractionGraph`]: its slot index. An index into the step's
/// pair list, valid until the next step; not a persistent graph index (upstream `NodeIndex`).
pub type ColliderGraphIndex = u32;

/// The index of a rigid body in an [`InteractionGraph`] of joints (upstream `NodeIndex`): its slot
/// index, as [`ColliderGraphIndex`] for a collider.
pub type RigidBodyGraphIndex = u32;

/// The index of an edge of an [`InteractionGraph`]: its rank in the graph, i.e. among the pairs of
/// the step's pair list. Valid until the next step (upstream `EdgeIndex`).
pub type TemporaryInteractionIndex = u32;

/// The state of one sensor pair (upstream `IntersectionPair`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct IntersectionPair {
    /// `true` while the two shapes intersect.
    pub intersecting: bool,
    /// `true` once the started event of the current intersection was emitted.
    pub start_event_emitted: bool,
}

/// One edge of the graph: two colliders and their interaction.
#[derive(Drop, Serde, PartialEq, Debug)]
pub struct InteractionEdge<E> {
    pub collider1: Handle,
    pub collider2: Handle,
    pub weight: E,
}

/// A read-only view of the interactions of the step's pair list (see the module documentation).
#[derive(Drop, Serde, PartialEq, Debug)]
pub struct InteractionGraph<E> {
    pub edges: Array<InteractionEdge<E>>,
}

pub impl InteractionGraphDefault<E, +Drop<E>> of Default<InteractionGraph<E>> {
    /// A graph without edge (upstream `Default`).
    fn default() -> InteractionGraph<E> {
        InteractionGraph { edges: array![] }
    }
}

/// `true` when the edge joins the colliders of slot `id1` and `id2`, in either order.
fn joins<E>(edge: @InteractionEdge<E>, id1: ColliderGraphIndex, id2: ColliderGraphIndex) -> bool {
    let (a, b) = (*edge.collider1.index, *edge.collider2.index);
    (a == id1 && b == id2) || (a == id2 && b == id1)
}

/// Queries of [`InteractionGraph`] (upstream `InteractionGraph`, read-only part).
#[generate_trait]
pub impl InteractionGraphImpl<E, +Drop<E>> of InteractionGraphTrait<E> {
    /// An empty graph (upstream `new`).
    fn new() -> InteractionGraph<E> {
        Default::default()
    }

    /// Every interaction, in graph order (upstream `interactions`).
    fn interactions(self: @InteractionGraph<E>) -> Array<@E> {
        let mut out = array![];
        for edge in self.edges.span() {
            out.append(edge.weight);
        }
        out
    }

    /// Every interaction with its two colliders, in graph order (upstream
    /// `interactions_with_endpoints`).
    fn interactions_with_endpoints(self: @InteractionGraph<E>) -> Array<(Handle, Handle, @E)> {
        let mut out = array![];
        for edge in self.edges.span() {
            out.append((*edge.collider1, *edge.collider2, edge.weight));
        }
        out
    }

    /// The first interaction between the colliders of slot `id1` and `id2`, in either order
    /// (upstream `interaction_pair`).
    fn interaction_pair(
        self: @InteractionGraph<E>, id1: ColliderGraphIndex, id2: ColliderGraphIndex,
    ) -> Option<(Handle, Handle, @E)> {
        let mut found = None;
        for edge in self.edges.span() {
            if joins(edge, id1, id2) {
                found = Some((*edge.collider1, *edge.collider2, edge.weight));
                break;
            }
        }
        found
    }

    /// Every interaction between the colliders of slot `id1` and `id2`, in either order (upstream
    /// `interactions_between`).
    fn interactions_between(
        self: @InteractionGraph<E>, id1: ColliderGraphIndex, id2: ColliderGraphIndex,
    ) -> Array<(Handle, Handle, @E)> {
        let mut out = array![];
        for edge in self.edges.span() {
            if joins(edge, id1, id2) {
                out.append((*edge.collider1, *edge.collider2, edge.weight));
            }
        }
        out
    }

    /// Every interaction involving the collider of slot `id` (upstream `interactions_with`).
    fn interactions_with(
        self: @InteractionGraph<E>, id: ColliderGraphIndex,
    ) -> Array<(Handle, Handle, @E)> {
        let mut out = array![];
        for edge in self.edges.span() {
            if *edge.collider1.index == id || *edge.collider2.index == id {
                out.append((*edge.collider1, *edge.collider2, edge.weight));
            }
        }
        out
    }

    /// The interaction of edge `id`, `None` when out of range (upstream `index_interaction`). `id`
    /// is an index into the step's pair list, valid until the next step, not a persistent graph
    /// index.
    fn index_interaction(
        self: @InteractionGraph<E>, id: TemporaryInteractionIndex,
    ) -> Option<(Handle, Handle, @E)> {
        let edge = self.edges.get(id)?.unbox();
        Some((*edge.collider1, *edge.collider2, edge.weight))
    }
}

/// The intersection (sensor) pairs of `pairs` as a graph.
pub fn intersection_graph(pairs: Span<ContactPair>) -> InteractionGraph<IntersectionPair> {
    let mut edges = array![];
    for pair in pairs {
        if pair.is_intersection_pair() {
            edges
                .append(
                    InteractionEdge {
                        collider1: *pair.collider1,
                        collider2: *pair.collider2,
                        weight: IntersectionPair {
                            intersecting: pair.intersecting(),
                            start_event_emitted: (*pair.event_status).start_event_emitted(),
                        },
                    },
                );
        }
    }
    InteractionGraph { edges }
}

/// The contact pairs of `pairs` as a graph.
pub fn contact_graph(pairs: Span<ContactPair>) -> InteractionGraph<ContactPairView> {
    let mut edges = array![];
    for view in contact_pairs(pairs) {
        edges
            .append(
                InteractionEdge {
                    collider1: view.collider1, collider2: view.collider2, weight: view,
                },
            );
    }
    InteractionGraph { edges }
}

/// Graph reads of [`NarrowPhase`] (upstream `NarrowPhase`, `geometry/narrow_phase/queries.rs`).
#[generate_trait]
pub impl NarrowPhaseInteractionGraphImpl of NarrowPhaseInteractionGraphTrait {
    /// Upstream `NarrowPhase::contact_graph`: a copy of the contact pairs as a graph (see the
    /// module documentation).
    fn contact_graph(self: @NarrowPhase) -> InteractionGraph<ContactPairView> {
        contact_graph(self.pairs.span())
    }

    /// Upstream `NarrowPhase::intersection_graph`: a copy of the intersection pairs as a graph.
    fn intersection_graph(self: @NarrowPhase) -> InteractionGraph<IntersectionPair> {
        intersection_graph(self.pairs.span())
    }
}
