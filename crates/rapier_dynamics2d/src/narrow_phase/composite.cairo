//! Composite collider pairs (work package SH2a): a polyline or a heightfield against a convex
//! collider has one manifold per part (upstream: several manifolds in one `ContactPair`).
//!
//! # Representation: one entry per manifold
//!
//! The pair list keeps its one-manifold [`ContactPair`]: a composite pair is a **group** of
//! consecutive entries with the same `(collider1, collider2)` key, one per manifold of
//! `rapier_geometry2d::dispatch::composite::contact_manifolds_composite` (at least one: an empty
//! placeholder when no part meets the other collider), so the solver, the islands, the sleeping
//! split and the state stay untouched: a group shares its colliders and bodies, so every walk
//! keeps it together, and every manifold with solver contacts is one constraint, as upstream.
//!
//! The group's **first entry** carries the pair's state: its `event_status` (the other entries'
//! is empty, so a dropped group emits one `Stopped`), and it is a manifold with solver contacts
//! whenever the group has one (the others follow in ascending part order), so "the first entry
//! has a solver contact" is upstream's `has_any_active_contact` for the whole pair, which the
//! pair loop's filtered path and the carried-over state read. Collision events are therefore per
//! collider pair, as upstream: one `Started` when the first manifold of the pair gets a solver
//! contact, one `Stopped` when the last one loses it.
//!
//! # Entry
//!
//! The dispatcher answers "unsupported" for a composite pair (`contact_manifold_step`'s composite
//! arm), and [`composite_pair_step`] runs from the pair loop's existing unsupported branch, so the
//! convex pairs' path is unchanged.
//!
//! # Deviations
//!
//! * `NarrowPhaseTrait::contact_pair` answers the group's first entry (a manifold with solver
//!   contacts when there is one), upstream the pair with all its manifolds.
//! * A pair of two composites has no manifold (see `rapier_geometry2d::dispatch::composite`).

use core::num::traits::DivRem;
use fixed::Fixed;
use rapier_core::collider::CollisionEventFlagsTrait;
use rapier_core::interaction_groups::{InteractionGroups, InteractionTestMode};
use rapier_geometry2d::contact::ContactManifold;
use rapier_geometry2d::dispatch::composite::contact_manifolds_composite;
use rapier_geometry2d::shape::ShapeTrait;
use crate::events::{CollisionEvent, PairEventStatusTrait, started, stopped};
use super::{
    CoefficientCombineRuleTrait, ContactPair, PairCollider, SOLVER_COMPUTE_RIGID_IMPULSES,
    SolverContact, SolverFlags, Vec2, dot2, mul_sub, one_way, pair_pose, solver_contact,
};

/// Runs the pair `(co1, co2)` when one of their shapes is composite (see the module
/// documentation): `Some((group, transition, skip))`, the group's entries in order, its
/// `Started` / `Stopped` event if any and the number of previous entries past `cursor` that
/// belonged to the pair's previous group; `None` (nothing done) otherwise.
///
/// The pair loop has just walked `previous` up to `cursor`: the group's first previous entry, if
/// any, is `previous[cursor - 1]` (same key, not an intersection pair).
///
/// # Cost of the hook on the other pairs
///
/// The loop calls this from its "unsupported" branch only, but its signature still shapes the
/// loop body (measured on `rapier2d::pipeline::narrow_benches`, Cairo steps per pair): a callee
/// that needs the bitwise builtin (+14), `ref` arrays or cursor (+10), a bitwise test computed
/// by the loop before the call (+18), or a manifold kept alive around the call (+75) moves every
/// pair. So the inputs are values the loop keeps alive anyway, `manifold` goes through the call,
/// the results come back by value, nothing below uses the bitwise builtin (event bits, the
/// event flag and the solver-groups test by `DivRem`), and the work runs behind a one-iteration
/// `while` (AGENTS §7 metering): its Sierra gas is withdrawn there, not charged to the loop body,
/// whose other branches would otherwise pay refunds (+10).
#[inline(never)]
pub(crate) fn composite_pair_step(
    prediction: Fixed,
    co1: PairCollider,
    co2: PairCollider,
    previous: Span<ContactPair>,
    cursor: u32,
    ref manifold: ContactManifold,
) -> Option<(Span<ContactPair>, Option<CollisionEvent>, u32)> {
    let mut out = None;
    let mut pending = true;
    while pending {
        out = composite_pair_inner(prediction, co1, co2, previous, cursor, ref manifold);
        pending = false;
    }
    out
}

/// The body of [`composite_pair_step`], behind its one-iteration loop.
fn composite_pair_inner(
    prediction: Fixed,
    co1: PairCollider,
    co2: PairCollider,
    previous: Span<ContactPair>,
    cursor: u32,
    ref manifold: ContactManifold,
) -> Option<(Span<ContactPair>, Option<CollisionEvent>, u32)> {
    if !co1.shape.is_composite() && !co2.shape.is_composite() {
        return None;
    }
    let solver_ok = groups_test(co1.solver_groups, co2.solver_groups);
    let events_enabled = low_bit(co1.active_events.bits) || low_bit(co2.active_events.bits);
    let h1 = co1.handle;
    let h2 = co2.handle;
    let mut old: Array<ContactManifold> = array![];
    let mut status = PairEventStatusTrait::empty();
    let mut had_contact = false;
    let mut skip: u32 = 0;
    if cursor != 0 {
        let head = previous.at(cursor - 1);
        if *head.collider1 == h1
            && *head.collider2 == h2
            && !(*head.event_status).is_intersection_pair() {
            status = *head.event_status;
            had_contact = *head.manifold.data.num_solver_contacts != 0;
            old.append(*head.manifold);
            while let Some(boxed) = previous.get(cursor + skip) {
                let next = boxed.unbox();
                if !(*next.collider1 == h1 && *next.collider2 == h2)
                    || (*next.event_status).is_intersection_pair() {
                    break;
                }
                old.append(*next.manifold);
                skip += 1;
            }
        }
    }
    let manifolds = contact_manifolds_composite(
        pair_pose(co1, co2), co1.shape, co2.shape, prediction, old.span(),
    )
        .unwrap_or_default();
    let mut solved = array![];
    let mut first: Option<u32> = None;
    let mut k: u32 = 0;
    for m in manifolds {
        let m = solver_data_composite(prediction, co1, co2, m, true, solver_ok);
        if first.is_none() && m.data.num_solver_contacts != 0 {
            first = Some(k);
        }
        solved.append(m);
        k += 1;
    }
    let has_contact = first.is_some();
    let mut event_status = status;
    let mut event = None;
    if has_contact != had_contact && events_enabled {
        let (_, low1) = DivRem::div_rem(event_status.bits, TWO);
        let (_, low2) = DivRem::div_rem(event_status.bits, FOUR);
        if has_contact {
            // `bits | START_EVENT_EMITTED`.
            event_status.bits = event_status.bits + 1 - low1;
            event = Some(started(h1, h2));
        } else {
            // `bits & 252`.
            event_status.bits = event_status.bits - low2;
            event = Some(stopped(h1, h2, CollisionEventFlagsTrait::empty()));
        }
    }
    let solved = solved.span();
    let mut group = array![];
    if solved.is_empty() {
        let placeholder = solver_data_composite(
            prediction, co1, co2, Default::default(), false, solver_ok,
        );
        group
            .append(
                ContactPair { collider1: h1, collider2: h2, manifold: placeholder, event_status },
            );
        return Some((group.span(), event, skip));
    }
    let lead = first.unwrap_or_default();
    group
        .append(
            ContactPair { collider1: h1, collider2: h2, manifold: *solved.at(lead), event_status },
        );
    let rest = PairEventStatusTrait::empty();
    let mut n: u32 = 0;
    for m in solved {
        if n != lead {
            group
                .append(
                    ContactPair { collider1: h1, collider2: h2, manifold: *m, event_status: rest },
                );
        }
        n += 1;
    }
    Some((group.span(), event, skip))
}

const TWO: NonZero<u8> = 2;
const FOUR: NonZero<u8> = 4;
const TWO_U32: NonZero<u32> = 2;

/// Bit 0 of `bits` (`COLLISION_EVENTS`), by `DivRem`.
#[inline(always)]
fn low_bit(bits: u32) -> bool {
    let (_, bit) = DivRem::div_rem(bits, TWO_U32);
    bit == 1
}

/// `(x & y) != 0` without the bitwise builtin: the low bits peeled together.
fn intersects(x: u32, y: u32) -> bool {
    let (mut x, mut y) = (x, y);
    loop {
        if x == 0 || y == 0 {
            break false;
        }
        let (qx, rx) = DivRem::div_rem(x, TWO_U32);
        let (qy, ry) = DivRem::div_rem(y, TWO_U32);
        if rx == 1 && ry == 1 {
            break true;
        }
        x = qx;
        y = qy;
    }
}

/// `InteractionGroupsTrait::test` without the bitwise builtin: the `Or` rule when both sides use
/// it, the `And` rule otherwise.
fn groups_test(a: InteractionGroups, b: InteractionGroups) -> bool {
    let ab = intersects(a.memberships.bits, b.filter.bits);
    let ba = intersects(b.memberships.bits, a.filter.bits);
    match (a.test_mode, b.test_mode) {
        (InteractionTestMode::Or, InteractionTestMode::Or) => ab || ba,
        _ => ab && ba,
    }
}

/// `super::solver_data` with the solver-groups test given (`solver_ok`), so that nothing here
/// needs the bitwise builtin.
fn solver_data_composite(
    prediction: Fixed,
    co1: PairCollider,
    co2: PairCollider,
    manifold: ContactManifold,
    supported: bool,
    solver_ok: bool,
) -> ContactManifold {
    let mut manifold = manifold;
    if !supported {
        manifold.num_points = 0;
    }
    solver_data_with_flags(prediction, co1, co2, manifold, solver_ok)
}

/// The number of entries of the group starting at `pairs[start]` (one for a convex pair).
pub fn group_len(pairs: Span<ContactPair>, start: u32) -> u32 {
    let head = pairs.at(start);
    let mut n: u32 = 1;
    while let Some(next) = pairs.get(start + n) {
        let next = next.unbox();
        if !(*next.collider1 == *head.collider1 && *next.collider2 == *head.collider2) {
            break;
        }
        n += 1;
    }
    n
}

/// The manifolds of the pair `(collider1, collider2)` (slot-index order), every entry of its
/// group; empty when the colliders form no contact pair. Linear scan, for tests and user queries
/// (upstream `ContactPair::manifolds`).
pub fn contact_pair_manifolds(
    pairs: Span<ContactPair>, collider1: rapier_core::Handle, collider2: rapier_core::Handle,
) -> Array<ContactManifold> {
    let mut out = array![];
    for pair in pairs {
        if *pair.collider1 == collider1
            && *pair.collider2 == collider2
            && !(*pair.event_status).is_intersection_pair() {
            out.append(*pair.manifold);
        }
    }
    out
}

/// `super::solver_data_supported` with the solver-groups test given (`solver_ok`).
#[inline(always)]
pub fn solver_data_with_flags(
    prediction: Fixed,
    co1: PairCollider,
    co2: PairCollider,
    manifold: ContactManifold,
    solver_ok: bool,
) -> ContactManifold {
    let mut manifold = manifold;
    manifold.data.rigid_body1 = co1.body;
    manifold.data.rigid_body2 = co2.body;
    manifold
        .data
        .solver_flags =
            SolverFlags { bits: if solver_ok {
                SOLVER_COMPUTE_RIGID_IMPULSES
            } else {
                0
            } };
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
    let (r, n1) = (co1.pose.rotation, manifold.local_n1);
    // `Rot2::rotate` inlined (BT3).
    manifold
        .data
        .normal = Vec2 { x: mul_sub(r.re, n1.x, r.im, n1.y), y: dot2(r.im, n1.x, r.re, n1.y) };

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

#[cfg(test)]
mod tests;
