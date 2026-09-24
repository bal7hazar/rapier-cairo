//! The narrow-phase pair loop as shipped before work package ON, kept for the gas ranking
//! (`narrow_benches`) and the old-vs-new equivalence tests (`tests`).
//!
//! Also the losing ON candidates: [`compute_contacts_inlined`] (the first step, composition
//! inlined but carry-over and pair copies unchanged) and [`PersistentDispatcher`] (the fast path
//! of `StepDispatcher` in front of the metered `DefaultDispatcher`); the ranking is in
//! `crate::pipeline`.
//!
//! [`compute_contacts_outlined`] calls the outlined [`process_pair_outlined`] →
//! [`update_manifold_outlined`] → [`solver_contact_outlined`] once per pair: Sierra gas charges
//! each of these loop-free functions its costliest path (both solver contacts, the costliest
//! combine rule, both event arms) on every pair. The shipped loop,
//! `rapier_dynamics2d::narrow_phase::compute_contacts_from_scratch`, inlines the same
//! composition in its loop body, where each branch is charged only when taken.

use core::num::traits::Zero;
use fixed::Fixed;
use rapier_core::collider::events::COLLISION_EVENTS;
use rapier_core::collider::{
    ActiveEventsTrait, CoefficientCombineRuleTrait, CollisionEventFlagsTrait,
};
use rapier_core::interaction_groups::InteractionGroupsTrait;
use rapier_dynamics2d::collider_set::ColliderSet;
use rapier_dynamics2d::events::{
    CollisionEvent, PairEventStatusTrait, START_EVENT_EMITTED, started, stopped,
};
use rapier_dynamics2d::narrow_phase::{
    CarryOver, ContactDispatcher, ContactPair, NarrowPhase, PairCollider, SortedMerge,
    dropped_events, pair_filtered, pair_pose, pair_transition, previous_state, solver_data,
};
use rapier_geometry2d::contact::{
    ContactManifold, NEW_CONTACT_BIT, SOLVER_COMPUTE_RIGID_IMPULSES, SolverContact, SolverFlags,
    TrackedContact,
};
use rapier_geometry2d::manifold::ManifoldTrait;
use rapier_geometry2d::shape::Shape;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::dispatcher::DefaultDispatcher;

/// The pair loop of `compute_contacts_from_scratch` as shipped before ON: one call of
/// the outlined [`process_pair_outlined`] per pair.
pub fn compute_contacts_outlined<impl D: ContactDispatcher, S, impl C: CarryOver<S>, +Destruct<S>>(
    ref self: NarrowPhase,
    prediction: Fixed,
    scratch: Span<PairCollider>,
    pairs: Span<(u32, u32)>,
    ref colliders: ColliderSet,
) -> Array<CollisionEvent> {
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
        let (pair, event) = process_pair_outlined::<D>(prediction, co1, co2, previous);
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

/// `process_pair` as shipped before ON: outlined, so Sierra gas charges it its costliest path
/// (both solver contacts, the costliest combine rule, both event arms) on every pair.
pub fn process_pair_outlined<impl D: ContactDispatcher>(
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
        update_manifold_outlined::<D>(prediction, co1, co2, manifold)
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

/// Runs the dispatcher on `manifold` (the previous one) and rebuilds its solver data.
pub fn update_manifold_outlined<impl D: ContactDispatcher>(
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
        first = solver_contact_outlined(p0, 0, co1, co2);
        count = 1;
    }
    if manifold.num_points > 1 && p1.dist < prediction {
        let contact = solver_contact_outlined(p1, 1, co1, co2);
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
pub fn solver_contact_outlined(
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

/// The first ON candidate: the pre-ON loop with the outlined `process_pair` replaced by its
/// composition inlined in the loop body (`previous_state`, `pair_filtered`, the dispatcher,
/// `solver_data`, `pair_transition`), carry-over still through [`CarryOver::take`] (an
/// `Option<ContactPair>` copy) and the pair built before the event branch.
pub fn compute_contacts_inlined<impl D: ContactDispatcher>(
    ref self: NarrowPhase,
    prediction: Fixed,
    scratch: Span<PairCollider>,
    pairs: Span<(u32, u32)>,
    ref colliders: ColliderSet,
) -> Array<CollisionEvent> {
    let mut carry: SortedMerge = CarryOver::begin(self.pairs.span());
    let mut current = array![];
    let mut transitions = array![];
    for (i, j) in pairs {
        let co1 = *scratch.at(*i);
        let co2 = *scratch.at(*j);
        if !(co1.solid && co2.solid) {
            continue;
        }
        let (manifold, status) = previous_state(carry.take(co1.handle, co2.handle));
        let had_contact = manifold.data.num_solver_contacts != 0;
        let manifold = if pair_filtered(co1, co2) {
            Default::default()
        } else {
            let mut manifold = manifold;
            let supported = D::contact_manifold(
                pair_pose(co1, co2), co1.shape, co2.shape, prediction, ref manifold,
            );
            solver_data(prediction, co1, co2, manifold, supported)
        };
        let (pair, event) = pair_transition(co1, co2, manifold, status, had_contact);
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

/// `DefaultDispatcher` (metered arms) behind the persistence fast path of
/// `crate::pipeline::step_dispatcher::StepDispatcher`: loses to `StepDispatcher`, whose plain
/// arms need no loop frame once inlined in the pair loop, and to plain `DefaultDispatcher` on
/// pairs without fast path (the extra shape `match`).
pub impl PersistentDispatcher of ContactDispatcher {
    #[inline(always)]
    fn contact_manifold(
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
        ref manifold: ContactManifold,
    ) -> bool {
        if persistent_pair(shape1, shape2) && manifold.try_update_contacts(pos12) {
            return true;
        }
        DefaultDispatcher::contact_manifold(pos12, shape1, shape2, prediction, ref manifold)
    }
}

/// `true` for the pairs whose generator starts with `try_update_contacts(pos12)`.
#[inline(always)]
fn persistent_pair(shape1: Shape, shape2: Shape) -> bool {
    match (shape1, shape2) {
        (Shape::Cuboid(_), Shape::Cuboid(_)) | (Shape::Cuboid(_), Shape::Capsule(_)) |
        (Shape::Capsule(_), Shape::Cuboid(_)) | (Shape::Cuboid(_), Shape::Segment(_)) |
        (Shape::Segment(_), Shape::Cuboid(_)) => true,
        _ => false,
    }
}
