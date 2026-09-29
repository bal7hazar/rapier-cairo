//! The contact-pair read API (work package CP3, off the step path): upstream's `ContactPair`
//! accessors and `NarrowPhase::contact_pairs*` over the port's pair list.
//!
//! # Representation
//!
//! The pair list holds one [`ContactPair`] entry per manifold (a composite pair is a run of
//! consecutive entries, ADR 0001 entry 35), so upstream's `ContactPair` (one pair, all its
//! manifolds) is [`ContactPairView`]: the collider pair, the run leader's event status and the
//! manifolds of the whole run, in the order the step stores them (ascending part order, the
//! first one holding a solver contact when the pair has one). Every read of this module returns
//! one view per collider pair; intersection (sensor) pairs are not contact pairs and are never
//! returned. The reads copy the manifolds out: they are for tests and user queries, not for the
//! step.
//!
//! `NarrowPhaseTrait::contact_pair` keeps answering the run's first entry (a frozen body); the
//! gathered twin is `contact_pair_view`.
//!
//! # Deviations
//!
//! * The port has rigid contacts only: `rigid` is always `Some` and answers the view itself,
//!   which carries `solver_manifolds` and `has_any_active_contact`; there is no contact
//!   clustering, so `solver_manifolds` is `manifolds`.
//! * `total_impulse`, `max_impulse` and the like return `Fixed` / `Vec2` values.
//! * `find_deepest_contact` copies the manifold and the contact out.
//! * Handles are matched in the order given by the caller for the `Handle` reads
//!   (`contact_pair_view`), in either order for the `unknown_gen` reads (upstream's graph is
//!   undirected; `contact_pair` keeps its exact-order match).
//! * `contact_pair_at_index(i)` indexes the pair storage (`NarrowPhase::pairs`, one entry per
//!   manifold, intersection pairs included), not a graph edge: it answers the pair whose run
//!   starts at entry `i`, and `None` when `i` is out of range, holds a sensor pair or is a later
//!   entry of a composite run.

use core::num::traits::Zero;
use fixed::Fixed;
use glam_core::Vec2;
use rapier_core::Handle;
use rapier_geometry2d::contact::{
    ContactManifold, ContactManifoldExt, ContactManifoldTrait, TrackedContact,
};
use rapier_geometry2d::manifold::ManifoldTrait;
use crate::events::{PairEventStatus, PairEventStatusTrait};
use super::{ContactPair, NarrowPhase};

#[cfg(test)]
mod tests;

/// One contact pair as upstream's `ContactPair`: the colliders, the pair's event status and the
/// manifolds of the whole run (see the module documentation).
#[derive(Drop, Serde, PartialEq, Debug)]
pub struct ContactPairView {
    pub collider1: Handle,
    pub collider2: Handle,
    pub event_status: PairEventStatus,
    pub manifolds: Array<ContactManifold>,
}

pub impl ContactPairViewDefault of Default<ContactPairView> {
    /// A pair of default handles without manifold (upstream `ContactPair::default`).
    fn default() -> ContactPairView {
        ContactPairView {
            collider1: Default::default(),
            collider2: Default::default(),
            event_status: PairEventStatusTrait::empty(),
            manifolds: array![],
        }
    }
}

/// Queries of [`ContactPairView`] (upstream `ContactPair` and `RigidPairContacts`).
#[generate_trait]
pub impl ContactPairViewImpl of ContactPairViewTrait {
    /// The pair's contact manifolds, in step order (upstream `manifolds`).
    fn manifolds(self: @ContactPairView) -> Span<ContactManifold> {
        self.manifolds.span()
    }

    /// The manifolds the solver sees (upstream `solver_manifolds`): every manifold, the port has
    /// no contact clustering.
    fn solver_manifolds(self: @ContactPairView) -> Span<ContactManifold> {
        self.manifolds.span()
    }

    /// The pair's rigid contacts (upstream `rigid`): always `Some`, the view itself (no soft
    /// pairs).
    fn rigid(self: @ContactPairView) -> Option<@ContactPairView> {
        Some(self)
    }

    /// `true` when a manifold holds at least one solver contact (upstream
    /// `has_any_active_contact`).
    fn has_any_active_contact(self: @ContactPairView) -> bool {
        let mut found = false;
        for m in self.manifolds.span() {
            if *m.data.num_solver_contacts != 0 {
                found = true;
                break;
            }
        }
        found
    }

    /// The total impulse (sum over the solver manifolds of the manifold's total normal impulse
    /// times its world normal, upstream `total_impulse`).
    fn total_impulse(self: @ContactPairView) -> Vec2 {
        let mut sum: Vec2 = Default::default();
        for m in self.manifolds.span() {
            let impulse = m.total_impulse();
            let normal = *m.data.normal;
            sum = Vec2 { x: sum.x + impulse * normal.x, y: sum.y + impulse * normal.y };
        }
        sum
    }

    /// The sum of the manifolds' total normal impulses (upstream `total_impulse_magnitude`).
    fn total_impulse_magnitude(self: @ContactPairView) -> Fixed {
        let mut sum: Fixed = Zero::zero();
        for m in self.manifolds.span() {
            sum = sum + m.total_impulse();
        }
        sum
    }

    /// `(impulse, normal)` of the manifold with the strongest total impulse (upstream
    /// `max_impulse`; the first one on a tie, `(0, 0)` when no manifold has a positive impulse).
    fn max_impulse(self: @ContactPairView) -> (Fixed, Vec2) {
        let mut best: Fixed = Zero::zero();
        let mut normal: Vec2 = Default::default();
        for m in self.manifolds.span() {
            let impulse = m.total_impulse();
            if impulse > best {
                best = impulse;
                normal = *m.data.normal;
            }
        }
        (best, normal)
    }

    /// The contact with the smallest signed distance and its manifold (upstream
    /// `find_deepest_contact`; the first one on a tie).
    fn find_deepest_contact(self: @ContactPairView) -> Option<(ContactManifold, TrackedContact)> {
        let mut deepest: Option<(ContactManifold, TrackedContact)> = None;
        for m in self.manifolds.span() {
            let Some(i) = (*m).find_deepest_contact() else {
                continue;
            };
            let candidate = (*m).point(i);
            deepest = match deepest {
                Some((_, current)) => if current.dist <= candidate.dist {
                    deepest
                } else {
                    Some((*m, candidate))
                },
                None => Some((*m, candidate)),
            };
        }
        deepest
    }

    /// Forgets every manifold (upstream `clear`).
    fn clear(ref self: ContactPairView) {
        self.manifolds = array![];
    }
}

/// `true` when entry `i` of `pairs` is a contact entry that starts a run (see the module
/// documentation).
fn starts_run(pairs: Span<ContactPair>, i: u32) -> bool {
    let pair = *pairs.at(i);
    if pair.event_status.is_intersection_pair() {
        return false;
    }
    if i == 0 {
        return true;
    }
    let before = *pairs.at(i - 1);
    !(before.collider1 == pair.collider1
        && before.collider2 == pair.collider2
        && !before.event_status.is_intersection_pair())
}

/// The view of the run that starts at entry `start` of `pairs` (a contact entry).
fn gather(pairs: Span<ContactPair>, start: u32) -> ContactPairView {
    let head = *pairs.at(start);
    let mut manifolds = array![];
    let mut i = start;
    while let Some(entry) = pairs.get(i) {
        let entry = *entry.unbox();
        if entry.collider1 != head.collider1
            || entry.collider2 != head.collider2
            || entry.event_status.is_intersection_pair() {
            break;
        }
        manifolds.append(entry.manifold);
        i += 1;
    }
    ContactPairView {
        collider1: head.collider1,
        collider2: head.collider2,
        event_status: head.event_status,
        manifolds,
    }
}

/// The views of the runs of `pairs`, ascending: every one for `None`, else those involving the
/// collider (exact handle when the flag is set, else slot index only).
fn views(pairs: Span<ContactPair>, collider: Option<(Handle, bool)>) -> Array<ContactPairView> {
    let mut out = array![];
    let mut i = 0;
    while i != pairs.len() {
        if starts_run(pairs, i) {
            let pair = *pairs.at(i);
            let involved = match collider {
                None => true,
                Some((c, exact)) => if exact {
                    pair.collider1 == c || pair.collider2 == c
                } else {
                    pair.collider1.index == c.index || pair.collider2.index == c.index
                },
            };
            if involved {
                out.append(gather(pairs, i));
            }
        }
        i += 1;
    }
    out
}

/// Every contact pair of `pairs` (upstream `NarrowPhase::contact_pairs`).
pub fn contact_pairs(pairs: Span<ContactPair>) -> Array<ContactPairView> {
    views(pairs, None)
}

/// The contact pairs involving `collider` (upstream `contact_pairs_with`).
pub fn contact_pairs_with(pairs: Span<ContactPair>, collider: Handle) -> Array<ContactPairView> {
    views(pairs, Some((collider, true)))
}

/// The contact pairs involving the collider of slot `collider`, whatever its generation
/// (upstream `contact_pairs_with_unknown_gen`).
pub fn contact_pairs_with_unknown_gen(
    pairs: Span<ContactPair>, collider: u32,
) -> Array<ContactPairView> {
    views(pairs, Some((Handle { index: collider, generation: 0 }, false)))
}

/// The pair of `(collider1, collider2)`, in this order (exact handles unless `unknown_gen`,
/// then slot indices in either order).
fn find(
    pairs: Span<ContactPair>, collider1: Handle, collider2: Handle, unknown_gen: bool,
) -> Option<ContactPairView> {
    let mut i = 0;
    let mut found = None;
    while i != pairs.len() {
        let pair = *pairs.at(i);
        let hit = if unknown_gen {
            (pair.collider1.index == collider1.index && pair.collider2.index == collider2.index)
                || (pair.collider1.index == collider2.index
                    && pair.collider2.index == collider1.index)
        } else {
            pair.collider1 == collider1 && pair.collider2 == collider2
        };
        if hit && starts_run(pairs, i) {
            found = Some(gather(pairs, i));
            break;
        }
        i += 1;
    }
    found
}

/// The contact pair of `(collider1, collider2)` with all its manifolds (see the module
/// documentation).
pub fn contact_pair_view(
    pairs: Span<ContactPair>, collider1: Handle, collider2: Handle,
) -> Option<ContactPairView> {
    find(pairs, collider1, collider2, false)
}

/// Upstream `contact_pair_unknown_gen`.
pub fn contact_pair_unknown_gen(
    pairs: Span<ContactPair>, collider1: u32, collider2: u32,
) -> Option<ContactPairView> {
    find(
        pairs,
        Handle { index: collider1, generation: 0 },
        Handle { index: collider2, generation: 0 },
        true,
    )
}

/// Upstream `contact_pair_at_index` (see the module documentation).
pub fn contact_pair_at_index(pairs: Span<ContactPair>, index: u32) -> Option<ContactPairView> {
    if index < pairs.len() && starts_run(pairs, index) {
        Some(gather(pairs, index))
    } else {
        None
    }
}

/// Read API of [`NarrowPhase`] (upstream `NarrowPhase`, `geometry/narrow_phase/queries.rs`).
#[generate_trait]
pub impl NarrowPhaseContactPairsImpl of NarrowPhaseContactPairsTrait {
    /// The contact pair of `(collider1, collider2)` (slot-index order) with all its manifolds:
    /// the gathered twin of `NarrowPhaseTrait::contact_pair`. Linear scan.
    fn contact_pair_view(
        self: @NarrowPhase, collider1: Handle, collider2: Handle,
    ) -> Option<ContactPairView> {
        contact_pair_view(self.pairs.span(), collider1, collider2)
    }

    /// Upstream `NarrowPhase::contact_pair_unknown_gen`: the pair of the two slot indices,
    /// whatever the generations, in either order.
    fn contact_pair_unknown_gen(
        self: @NarrowPhase, collider1: u32, collider2: u32,
    ) -> Option<ContactPairView> {
        contact_pair_unknown_gen(self.pairs.span(), collider1, collider2)
    }

    /// Upstream `NarrowPhase::contact_pair_at_index`: see the module documentation.
    fn contact_pair_at_index(self: @NarrowPhase, index: u32) -> Option<ContactPairView> {
        contact_pair_at_index(self.pairs.span(), index)
    }

    /// Upstream `NarrowPhase::contact_pairs`: every contact pair, ascending.
    fn contact_pairs(self: @NarrowPhase) -> Array<ContactPairView> {
        contact_pairs(self.pairs.span())
    }

    /// Upstream `NarrowPhase::contact_pairs_with`: the contact pairs involving `collider`.
    fn contact_pairs_with(self: @NarrowPhase, collider: Handle) -> Array<ContactPairView> {
        contact_pairs_with(self.pairs.span(), collider)
    }

    /// Upstream `NarrowPhase::contact_pairs_with_unknown_gen`: as `contact_pairs_with` with the
    /// collider identified by slot index.
    fn contact_pairs_with_unknown_gen(self: @NarrowPhase, collider: u32) -> Array<ContactPairView> {
        contact_pairs_with_unknown_gen(self.pairs.span(), collider)
    }
}
