//! The narrow phase's pair loop (`StageConfig::Narrow`) in a declared class (CS6, route (a);
//! CX2's crossing): the caller sends the previous pairs, the pair colliders and the candidate
//! pairs, `NarrowPhaseClass` runs the batched narrow phase (`rapier2d::pipeline::stages::narrow`:
//! the jobs, their contact generation, the loop with its filters, one-way platforms, solver data
//! and events) and returns the new pairs and the collision events.
//!
//! CX2: the class is also the polygon family's: the pairs without a ball get their contacts in
//! the class, only the pairs with a ball call `ContactBallClass` (whose hash crosses with the
//! call: a declared class cannot be compiled with the game's constants before they exist). CX3:
//! the class runs its own pair loop ([`pair_loop`]) on the previous pairs as they cross, each
//! pair's contacts generated where the loop reaches it (a pair with a ball by one call), instead
//! of the batched narrow phase's jobs then loop. A previous pair crosses as a [`PreviousPair`]: the
//! loop rewrites every field of its solver data but the contact count and the user data. CS6's
//! crossing (whole previous pairs, both families called) and the packed wires of
//! [`alternatives`] are the measured losers (`docs/research/class-split.md`, section 11).
//!
//! The pair loop reads the collider set only to flag the `Stopped` event of a dropped pair whose
//! `Started` was emitted (`REMOVED` when a collider no longer exists): the caller sends the
//! colliders of such pairs that still exist ([`alive`]), and the class answers from a set holding
//! exactly those (`crate::arena::partial_state`). A world without collision events sends none.

use core::dict::{Felt252Dict, Felt252DictTrait};
use rapier2d::pipeline::stages::NarrowPhaseStage;
use rapier2d::pipeline::stages::narrow::{ManifoldGeometry, geometry, with_geometry};
use rapier2d::prelude::{Fixed, Handle, Pose2, Shape, ShapeTrait};
use rapier2d::world::basic_state::{deserialize_basic_shape, serialize_basic_shape};
use rapier_core::collider::CollisionEventFlagsTrait;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::collider::components::BoxedOneWayPlatformSerde;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::{
    CollisionEvent, PairEventStatus, PairEventStatusTrait, START_EVENT_EMITTED, started, stopped,
};
use rapier_dynamics2d::narrow_phase::strategies::errors::{COMPOSITE, SENSOR};
use rapier_dynamics2d::narrow_phase::{
    ContactPair, ContactPairTrait, NarrowPhase, PairCollider, dropped_event, events_on, key_before,
    pair_filtered, pair_pose, solver_data_supported,
};
use rapier_geometry2d::contact::ContactManifoldData;
use starknet::syscalls::library_call_syscall;
use starknet::{ClassHash, SyscallResultTrait};
use crate::contact::{ball_family, contact_manifold_polygon_family};
use crate::hashes::{ClassHashes, errors};

/// Measured and rejected: the packed wires (CX2).
#[cfg(test)]
mod alternatives;

/// The crossing of a pair collider: its fields in order, the shape by the basic codec.
///
/// # Panics
/// `'State: not a basic shape'` on another shape.
pub impl PairColliderSerde of Serde<PairCollider> {
    fn serialize(self: @PairCollider, ref output: Array<felt252>) {
        self.handle.serialize(ref output);
        self.solid.serialize(ref output);
        self.sensor.serialize(ref output);
        serialize_basic_shape(self.shape, ref output);
        self.pose.serialize(ref output);
        self.friction.serialize(ref output);
        self.restitution.serialize(ref output);
        self.friction_combine_rule.serialize(ref output);
        self.restitution_combine_rule.serialize(ref output);
        self.active_collision_types.serialize(ref output);
        self.collision_groups.serialize(ref output);
        self.solver_groups.serialize(ref output);
        self.active_events.serialize(ref output);
        self.one_way.serialize(ref output);
        self.body.serialize(ref output);
        self.body_type.serialize(ref output);
        self.world_com.serialize(ref output);
        self.dominance.serialize(ref output);
    }

    fn deserialize(ref serialized: Span<felt252>) -> Option<PairCollider> {
        Some(
            PairCollider {
                handle: Serde::deserialize(ref serialized)?,
                solid: Serde::deserialize(ref serialized)?,
                sensor: Serde::deserialize(ref serialized)?,
                shape: deserialize_basic_shape(ref serialized)?,
                pose: Serde::deserialize(ref serialized)?,
                friction: Serde::deserialize(ref serialized)?,
                restitution: Serde::deserialize(ref serialized)?,
                friction_combine_rule: Serde::deserialize(ref serialized)?,
                restitution_combine_rule: Serde::deserialize(ref serialized)?,
                active_collision_types: Serde::deserialize(ref serialized)?,
                collision_groups: Serde::deserialize(ref serialized)?,
                solver_groups: Serde::deserialize(ref serialized)?,
                active_events: Serde::deserialize(ref serialized)?,
                one_way: Serde::deserialize(ref serialized)?,
                body: Serde::deserialize(ref serialized)?,
                body_type: Serde::deserialize(ref serialized)?,
                world_com: Serde::deserialize(ref serialized)?,
                dominance: Serde::deserialize(ref serialized)?,
            },
        )
    }
}

/// The colliders of the previous pairs whose `Started` event was emitted that still exist in
/// `colliders` (duplicates possible, any order): all the pair loop asks the set.
pub fn alive(previous: Span<ContactPair>, ref colliders: ColliderSet) -> Array<Handle> {
    let mut out = array![];
    for pair in previous {
        if (*pair.event_status).start_event_emitted() {
            if colliders.contains(*pair.collider1) {
                out.append(*pair.collider1);
            }
            if colliders.contains(*pair.collider2) {
                out.append(*pair.collider2);
            }
        }
    }
    out
}

/// A set that contains exactly the handles of `alive` (duplicates and order ignored), each with
/// `placeholder` as its value.
pub fn set_of(alive: Span<Handle>, placeholder: Collider) -> ColliderSet {
    if alive.is_empty() {
        return Default::default();
    }
    // Slot → generation plus one.
    let mut slots: Felt252Dict<u32> = Default::default();
    let mut last: u32 = 0;
    for handle in alive {
        slots.insert((*handle.index).into(), *handle.generation + 1);
        if *handle.index > last {
            last = *handle.index;
        }
    }
    let mut entries = array![];
    let mut index: u32 = 0;
    while index != last + 1 {
        let generation = slots.get(index.into());
        if generation != 0 {
            entries.append((Handle { index, generation: generation - 1 }, placeholder));
        }
        index += 1;
    }
    ColliderSetTrait::from_state(crate::arena::partial_state(entries.span()))
}


/// All the pair loop reads of a previous pair (CX2): its handles, event status, solver contact
/// count, user data and manifold geometry (36 felts instead of 64). The loop rewrites every other
/// field of its solver data (`solver_data_supported`), so they stay in the caller.
#[derive(Copy, Drop, Serde)]
pub struct PreviousPair {
    pub collider1: Handle,
    pub collider2: Handle,
    pub event_status: PairEventStatus,
    pub num_solver_contacts: u8,
    pub user_data: u32,
    pub geometry: ManifoldGeometry,
}

/// What crosses of `pair`.
#[inline(always)]
pub fn previous_of(pair: @ContactPair) -> PreviousPair {
    PreviousPair {
        collider1: *pair.collider1,
        collider2: *pair.collider2,
        event_status: *pair.event_status,
        num_solver_contacts: *pair.manifold.data.num_solver_contacts,
        user_data: *pair.manifold.data.user_data,
        geometry: crate::contact::geometry(pair.manifold),
    }
}

/// The pair the loop sees for `previous`: the default solver data but the contact count and the
/// user data (the loop's results are those of the whole pair).
#[inline(always)]
pub fn pair_of(previous: PreviousPair) -> ContactPair {
    let mut data: ContactManifoldData = Default::default();
    data.num_solver_contacts = previous.num_solver_contacts;
    data.user_data = previous.user_data;
    ContactPair {
        collider1: previous.collider1,
        collider2: previous.collider2,
        manifold: crate::contact::with_geometry(previous.geometry, data),
        event_status: previous.event_status,
    }
}

/// The pair loop of `BatchedNarrowPhase<FamilyBatch, NoSensors>` (`rapier2d::pipeline::stages::
/// narrow::compute_contacts_with_results`) on the previous pairs as they cross (CX3): a pair's
/// contact generation runs where the loop reaches it, the pairs without a ball here
/// (`contact_manifold_polygon_family`), a pair with a ball by one call of `ContactBallClass` (at
/// `contact_ball`). The generator of each pair gets what its batch job carried (the pose of
/// collider 2 in collider 1's frame, the shapes, the geometry of the previous manifold of the pair
/// found by the loop's walk, or a default one) and the new manifold is the geometry it leaves with
/// the previous solver data ([`pair_of`]): same pairs and events. CX2's batched body (the jobs,
/// then the loop over their results) is the measured loser (`alternatives::batched_in_class`).
///
/// # Panics
/// As `compute_contacts_with_results::<NoSensors>`: `errors::SENSOR` on a sensor pair,
/// `UNSUPPORTED` on a shape that is not basic; `crate::hashes::errors::DECODE` when the ball class
/// returns something else than its result.
pub fn pair_loop(
    contact_ball: ClassHash,
    previous: Span<PreviousPair>,
    prediction: Fixed,
    scratch: Span<PairCollider>,
    pairs: Span<(u32, u32)>,
    ref colliders: ColliderSet,
) -> (Array<ContactPair>, Array<CollisionEvent>) {
    let fresh = previous_of(@ContactPairTrait::new(Default::default(), Default::default()));
    let mut cursor: u32 = 0;
    let mut current = array![];
    let mut events = array![];
    let mut transitions = array![];
    for (i, j) in pairs {
        let co1 = *scratch.at(*i);
        let co2 = *scratch.at(*j);
        if !(co1.solid && co2.solid) {
            if (co1.solid || co1.sensor) && (co2.solid || co2.sensor) {
                core::panic_with_felt252(SENSOR);
            }
            continue;
        }
        let h1 = co1.handle;
        let h2 = co2.handle;
        let mut found = fresh;
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
                if a1 == h1 && a2 == h2 && !(*head.event_status).is_intersection_pair() {
                    found = *head;
                } else {
                    dropped_event(a1, a2, *head.event_status, ref colliders, ref events);
                }
            }
            break;
        }
        let status = found.event_status;
        let had_contact = found.num_solver_contacts != 0;
        if pair_filtered(co1, co2) {
            let mut event_status = status;
            if had_contact && events_on(co1, co2) {
                event_status.bits = event_status.bits & 252;
                transitions.append(stopped(h1, h2, CollisionEventFlagsTrait::empty()));
            }
            current
                .append(
                    ContactPair {
                        collider1: h1, collider2: h2, manifold: Default::default(), event_status,
                    },
                );
            continue;
        }
        let pos12 = pair_pose(co1, co2);
        let (supported, new_geometry) = if ball_family(co1.shape, co2.shape) {
            ball_geometry(contact_ball, pos12, co1.shape, co2.shape, prediction, found.geometry)
        } else {
            let mut manifold = with_geometry(found.geometry, Default::default());
            let supported = contact_manifold_polygon_family(
                pos12, co1.shape, co2.shape, prediction, ref manifold,
            );
            (supported, geometry(@manifold))
        };
        let mut manifold = pair_of(PreviousPair { geometry: new_geometry, ..found }).manifold;
        if !supported {
            assert(!co1.shape.is_composite() && !co2.shape.is_composite(), COMPOSITE);
            manifold.num_points = 0;
        }
        let manifold = solver_data_supported(prediction, co1, co2, manifold);
        let has_contact = manifold.data.num_solver_contacts != 0;
        let mut event_status = status;
        if has_contact != had_contact && events_on(co1, co2) {
            if has_contact {
                event_status.bits = event_status.bits | START_EVENT_EMITTED.bits;
                transitions.append(started(h1, h2));
            } else {
                event_status.bits = event_status.bits & 252;
                transitions.append(stopped(h1, h2, CollisionEventFlagsTrait::empty()));
            }
        }
        current.append(ContactPair { collider1: h1, collider2: h2, manifold, event_status });
    }
    while let Some(boxed) = previous.get(cursor) {
        let head = boxed.unbox();
        dropped_event(
            *head.collider1, *head.collider2, *head.event_status, ref colliders, ref events,
        );
        cursor += 1;
    }
    events.append_span(transitions.span());
    (current, events)
}

/// The geometry of a pair with a ball, by `ContactBallClass` (at `contact_ball`, its
/// `contact_geometry` entry).
fn ball_geometry(
    contact_ball: ClassHash,
    pos12: Pose2,
    shape1: Shape,
    shape2: Shape,
    prediction: Fixed,
    geometry: ManifoldGeometry,
) -> (bool, ManifoldGeometry) {
    let mut calldata = array![];
    pos12.serialize(ref calldata);
    shape1.serialize(ref calldata);
    shape2.serialize(ref calldata);
    prediction.serialize(ref calldata);
    geometry.serialize(ref calldata);
    let mut ret = library_call_syscall(contact_ball, selector!("contact_geometry"), calldata.span())
        .unwrap_syscall();
    Serde::deserialize(ref ret).expect(errors::DECODE)
}

/// The pair loop library-called in `NarrowPhaseClass` (at `H::narrow_phase()`), one call per step
/// with a candidate or a previous pair, the pairs with a ball in `ContactBallClass` (at
/// `H::contact_ball()`, one call per pair, CX3), the others in `NarrowPhaseClass` (CX2). Same
/// results as `PairLoopNarrowPhase<FamilyDispatcher<H>, NoSensors, NoComposites>`.
///
/// # Panics
/// `errors::DECODE` when the class returns something else than its result; as the class (a shape
/// that is not basic, a sensor or a composite pair).
pub impl LibraryCallNarrowPhase<impl H: ClassHashes> of NarrowPhaseStage {
    fn compute_contacts(
        ref narrow_phase: NarrowPhase,
        prediction: Fixed,
        scratch: Span<PairCollider>,
        pairs: Span<(u32, u32)>,
        ref colliders: ColliderSet,
    ) -> Array<CollisionEvent> {
        if pairs.is_empty() && narrow_phase.pairs.is_empty() {
            return array![];
        }
        let mut calldata = array![];
        H::contact_ball().serialize(ref calldata);
        calldata.append(narrow_phase.pairs.len().into());
        for pair in narrow_phase.pairs.span() {
            previous_of(pair).serialize(ref calldata);
        }
        prediction.serialize(ref calldata);
        scratch.serialize(ref calldata);
        pairs.serialize(ref calldata);
        alive(narrow_phase.pairs.span(), ref colliders).serialize(ref calldata);
        let mut ret = library_call_syscall(
            H::narrow_phase(), selector!("compute_contacts"), calldata.span(),
        )
            .unwrap_syscall();
        let (current, events): (Array<ContactPair>, Array<CollisionEvent>) = Serde::deserialize(
            ref ret,
        )
            .expect(errors::DECODE);
        narrow_phase.pairs = current;
        events
    }
}

/// The narrow phase's pair loop of the basic shapes (`rapier2d::pipeline::stages::narrow`) and
/// the contact generation of the pairs without a ball; the pairs with a ball in the class whose
/// hash crosses.
#[starknet::contract]
pub mod NarrowPhaseClass {
    use fixed::ONE;
    use rapier2d::prelude::{ColliderBuilderTrait, Fixed, Handle};
    use rapier_dynamics2d::events::CollisionEvent;
    use rapier_dynamics2d::narrow_phase::{ContactPair, PairCollider};
    use starknet::ClassHash;
    use super::{PairColliderSerde, PreviousPair};

    #[storage]
    struct Storage {}

    /// [`super::pair_loop`] on `previous` (the step's previous pairs as they cross): the new
    /// pairs and the collision events. `alive`: see [`super::alive`].
    #[external(v0)]
    fn compute_contacts(
        self: @ContractState,
        contact_ball: ClassHash,
        previous: Span<PreviousPair>,
        prediction: Fixed,
        scratch: Span<PairCollider>,
        pairs: Span<(u32, u32)>,
        alive: Span<Handle>,
    ) -> (Array<ContactPair>, Array<CollisionEvent>) {
        let mut colliders = super::set_of(alive, ColliderBuilderTrait::ball(ONE).build());
        super::pair_loop(contact_ball, previous, prediction, scratch, pairs, ref colliders)
    }
}
