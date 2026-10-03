//! The pair kinds of the step's pair loop beyond the dispatcher's convex pairs, as strategies
//! (work package CS2): sensors ([`IntersectionStrategy`]) and composite shapes
//! ([`CompositeStrategy`]). `compute_contacts_from_scratch_with` takes one of each, with the
//! `ContactDispatcher`; a program compiles only the strategies it names (upstream plugs the same
//! choices in at run time: `&dyn QueryDispatcher`, the collider types it builds).
//!
//! * [`SensorIntersections`] / [`CompositeManifolds`]: the narrow phase's own handling
//!   (`intersections::intersection_pair_step`, `composite::composite_pair_step`), what
//!   `compute_contacts_from_scratch` uses;
//! * [`NoSensors`] / [`NoComposites`]: neither is compiled; a world that uses the feature is
//!   rejected at the step that meets it (a documented panic, never a silently skipped pair).
//!
//! Both hooks run from branches the convex pairs never take (a pair with a sensor, a pair the
//! dispatcher does not support), so a world without sensors or composites pays the same Cairo
//! steps with either strategy.

use fixed::Fixed;
use rapier_geometry2d::contact::ContactManifold;
use rapier_geometry2d::shape::ShapeTrait;
use crate::collider_set::ColliderSet;
use crate::events::CollisionEvent;
use super::composite::{composite_pair_step, composite_pair_step_constrained};
use super::intersections::intersection_pair_step;
use super::{ContactPair, PairCollider};

/// Panics of the disabled strategies.
pub mod errors {
    /// A pair with a sensor met a step compiled with [`super::NoSensors`].
    pub const SENSOR: felt252 = 'Narrow phase: sensors disabled';
    /// A pair with a composite shape met a step compiled with [`super::NoComposites`].
    pub const COMPOSITE: felt252 = 'Narrow phase: no composites';
}

/// What the pair loop does with an enabled pair of which one collider is a sensor (an
/// intersection pair, see the parent module). Selected statically, as `ContactDispatcher`.
pub trait IntersectionStrategy {
    /// Updates the intersection pair `(co1, co2)`: walks `previous` from `cursor` to its key
    /// (emitting the `Stopped` of the pairs it passes into `events`), appends its transition to
    /// `transitions` and the pair to `current` (as `intersections::intersection_pair_step`).
    fn intersection_pair(
        co1: PairCollider,
        co2: PairCollider,
        previous: Span<ContactPair>,
        ref cursor: u32,
        ref colliders: ColliderSet,
        ref events: Array<CollisionEvent>,
        ref transitions: Array<CollisionEvent>,
        ref current: Array<ContactPair>,
    );
}

/// Sensors as the narrow phase handles them (`intersections::intersection_pair_step`).
pub impl SensorIntersections of IntersectionStrategy {
    #[inline(always)]
    fn intersection_pair(
        co1: PairCollider,
        co2: PairCollider,
        previous: Span<ContactPair>,
        ref cursor: u32,
        ref colliders: ColliderSet,
        ref events: Array<CollisionEvent>,
        ref transitions: Array<CollisionEvent>,
        ref current: Array<ContactPair>,
    ) {
        intersection_pair_step(
            co1, co2, previous, ref cursor, ref colliders, ref events, ref transitions, ref current,
        );
    }
}

/// No sensor: the intersection test is not compiled.
///
/// # Panics
/// `errors::SENSOR` when an enabled sensor's AABB meets another enabled collider's.
pub impl NoSensors of IntersectionStrategy {
    #[inline(always)]
    fn intersection_pair(
        co1: PairCollider,
        co2: PairCollider,
        previous: Span<ContactPair>,
        ref cursor: u32,
        ref colliders: ColliderSet,
        ref events: Array<CollisionEvent>,
        ref transitions: Array<CollisionEvent>,
        ref current: Array<ContactPair>,
    ) {
        core::panic_with_felt252(errors::SENSOR)
    }
}

/// What the pair loop does with a pair the dispatcher does not support (`contact_manifold`
/// returned `false`): a composite pair becomes a group of entries, any other keeps an empty
/// manifold.
pub trait CompositeStrategy {
    /// `true` when composite pairs are handled (the force-event pass then groups their entries).
    const ENABLED: bool;
    /// `Some((group, transition, skip))` for a composite pair (as
    /// `composite::composite_pair_step`), `None` for any other unsupported pair.
    fn composite_pair(
        prediction: Fixed,
        co1: PairCollider,
        co2: PairCollider,
        previous: Span<ContactPair>,
        cursor: u32,
        ref manifold: ContactManifold,
    ) -> Option<(Span<ContactPair>, Option<CollisionEvent>, u32)>;
}

/// Polylines, heightfields and compounds as the narrow phase handles them
/// (`composite::composite_pair_step`).
pub impl CompositeManifolds of CompositeStrategy {
    const ENABLED: bool = true;

    #[inline(always)]
    fn composite_pair(
        prediction: Fixed,
        co1: PairCollider,
        co2: PairCollider,
        previous: Span<ContactPair>,
        cursor: u32,
        ref manifold: ContactManifold,
    ) -> Option<(Span<ContactPair>, Option<CollisionEvent>, u32)> {
        composite_pair_step(prediction, co1, co2, previous, cursor, ref manifold)
    }
}

/// [`CompositeManifolds`] with the compound parts' normal constraints (lot CE, parry 0.31's
/// `CompoundFlags::FIX_INTERNAL_EDGES`): a compound built with that flag clamps its parts' contact
/// normals to the outline of the union, so a body sliding across the join of two parts does not
/// catch on it. A compound without the flag, a polyline and a heightfield get
/// [`CompositeManifolds`]'s manifolds. Opt-in: `DefaultStepConfig` keeps [`CompositeManifolds`], so
/// `World::step` does not compile this path; a step selects it with its own `StepConfig`
/// (`impl Composites = ConstrainedCompositeManifolds;`) through `World::step_with::<C>`.
pub impl ConstrainedCompositeManifolds of CompositeStrategy {
    const ENABLED: bool = true;

    #[inline(always)]
    fn composite_pair(
        prediction: Fixed,
        co1: PairCollider,
        co2: PairCollider,
        previous: Span<ContactPair>,
        cursor: u32,
        ref manifold: ContactManifold,
    ) -> Option<(Span<ContactPair>, Option<CollisionEvent>, u32)> {
        composite_pair_step_constrained(prediction, co1, co2, previous, cursor, ref manifold)
    }
}

/// No composite shape: the composite manifolds are not compiled; any other unsupported pair
/// keeps an empty manifold, as with [`CompositeManifolds`].
///
/// # Panics
/// `errors::COMPOSITE` when a pair of colliders, one of them composite, reaches the dispatcher.
pub impl NoComposites of CompositeStrategy {
    const ENABLED: bool = false;

    #[inline(always)]
    fn composite_pair(
        prediction: Fixed,
        co1: PairCollider,
        co2: PairCollider,
        previous: Span<ContactPair>,
        cursor: u32,
        ref manifold: ContactManifold,
    ) -> Option<(Span<ContactPair>, Option<CollisionEvent>, u32)> {
        assert(!co1.shape.is_composite() && !co2.shape.is_composite(), errors::COMPOSITE);
        None
    }
}
