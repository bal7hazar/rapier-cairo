//! The narrow phase's pair loop (`StageConfig::Narrow`) in a declared class (CS6, route (a)): the
//! caller sends the previous pairs, the pair colliders and the candidate pairs, `NarrowPhaseClass`
//! runs the batched narrow phase (`rapier2d::pipeline::stages::narrow`: the jobs, one call of each
//! contact family class, the loop with its filters, one-way platforms, solver data and events) and
//! returns the new pairs and the collision events.
//!
//! The family classes' hashes cross with the call (a declared class cannot be compiled with the
//! game's constants before they exist). The pair loop reads the collider set only to flag the
//! `Stopped` event of a dropped pair whose `Started` was emitted (`REMOVED` when a collider no
//! longer exists): the caller sends the colliders of such pairs that still exist ([`alive`]),
//! and the class answers from a set holding exactly those (`crate::arena::partial_state`). A world
//! without collision events sends none.

use core::dict::{Felt252Dict, Felt252DictTrait};
use rapier2d::pipeline::stages::NarrowPhaseStage;
use rapier2d::prelude::{Fixed, Handle};
use rapier2d::world::basic_state::{deserialize_basic_shape, serialize_basic_shape};
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::collider::components::BoxedOneWayPlatformSerde;
use rapier_dynamics2d::collider_set::{ColliderSet, ColliderSetTrait};
use rapier_dynamics2d::events::{CollisionEvent, PairEventStatusTrait};
use rapier_dynamics2d::narrow_phase::{ContactPair, NarrowPhase, PairCollider};
use starknet::SyscallResultTrait;
use starknet::syscalls::library_call_syscall;
use crate::hashes::{ClassHashes, errors};

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

/// The pair loop library-called in `NarrowPhaseClass` (at `H::narrow_phase()`), one call per step
/// with a candidate or a previous pair, the families at `H::contact_ball()` and
/// `H::contact_polygon()`. Same results as `PairLoopNarrowPhase<FamilyDispatcher<H>, NoSensors,
/// NoComposites>`.
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
        H::contact_polygon().serialize(ref calldata);
        narrow_phase.pairs.serialize(ref calldata);
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

/// The narrow phase's pair loop of the basic shapes (`rapier2d::pipeline::stages::narrow`, the
/// contact generation in the family classes whose hashes cross).
#[starknet::contract]
pub mod NarrowPhaseClass {
    use fixed::ONE;
    use rapier2d::pipeline::stages::narrow::{compute_contacts_with_results, contact_jobs};
    use rapier2d::prelude::{ColliderBuilderTrait, Fixed, Handle};
    use rapier_dynamics2d::events::CollisionEvent;
    use rapier_dynamics2d::narrow_phase::strategies::NoSensors;
    use rapier_dynamics2d::narrow_phase::{ContactPair, NarrowPhase, PairCollider};
    use starknet::ClassHash;
    use super::PairColliderSerde;

    #[storage]
    struct Storage {}

    /// `BatchedNarrowPhase<FamilyBatch, NoSensors>` on `previous` (the step's previous pairs):
    /// the new pairs and the collision events. `alive`: see [`super::alive`].
    #[external(v0)]
    fn compute_contacts(
        self: @ContractState,
        contact_ball: ClassHash,
        contact_polygon: ClassHash,
        previous: Array<ContactPair>,
        prediction: Fixed,
        scratch: Span<PairCollider>,
        pairs: Span<(u32, u32)>,
        alive: Span<Handle>,
    ) -> (Array<ContactPair>, Array<CollisionEvent>) {
        let mut narrow = NarrowPhase { pairs: previous };
        let mut colliders = super::set_of(alive, ColliderBuilderTrait::ball(ONE).build());
        let jobs = contact_jobs(narrow.pairs.span(), scratch, pairs);
        let results = if jobs.is_empty() {
            array![].span()
        } else {
            crate::contact::family_geometries(
                contact_ball, contact_polygon, prediction, jobs.span(),
            )
        };
        let events = compute_contacts_with_results::<
            NoSensors,
        >(ref narrow, prediction, scratch, pairs, ref colliders, results);
        (narrow.pairs, events)
    }
}
