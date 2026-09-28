//! The island stage (`StepConfig::Islands`: sleep and wake-up decisions) in a declared class
//! (CS5).
//!
//! [`LibraryCallIslands`] keeps `update_islands`' fast checks in the caller (no awake member, or
//! no eligible one and no link from an awake body to a sleeping one: nothing changes, no call) and
//! library-calls [`IslandsClass`] for the union-find. What crosses:
//! * in: an [`IslandBody`] per body (the entries of the stage: all the stage reads of a body) and
//!   the links of the touching pairs (their two bodies: all the union-find reads of a pair),
//!   active pairs first, then the dormant ones, in the stage's order; a step that met colliders
//!   inserted since the last step (`islands_after_insertions`, whose contact-start wake-up reads
//!   the pairs' event state) sends the whole pairs instead;
//! * out: the bodies the stage changed (position in the entries, new activation and change
//!   flags), whether a member sleeps and whether a body woke up.
//!
//! The class runs the stage on bodies rebuilt from the [`IslandBody`]s (zero velocities, handles
//! renumbered densely as in `crate::arena` on the per-step path); the
//! caller writes each changed body's activation and flags, and zero velocities when it went to
//! sleep (`RigidBody::sleep`, the only write that sets the sleeping flag), to its set as the stage
//! does. A body the stage writes with unchanged activation and flags (woken, then put back to
//! sleep in the same step, `SLEEP` already raised) is not written back: its velocities are
//! those of a sleeping body already, only the set's `is_modified` flag, which the step clears or
//! discards at its end, can differ.

use rapier2d::pipeline::config::errors::JOINTS;
use rapier2d::pipeline::islands::{SleepCensus, links_awake_to_sleeping, update_islands_slow};
use rapier2d::pipeline::sleeping::islands_after_insertions;
use rapier2d::pipeline::stages::IslandStage;
use rapier2d::prelude::{
    ContactPair, ContactPairTrait, Handle, ImpulseJoint, RigidBody, RigidBodySet, RigidBodySetTrait,
};
use rapier_core::rigid_body::{RigidBodyActivation, RigidBodyChanges, RigidBodyType};
use rapier_dynamics2d::rigid_body::RigidBodyVelocityTrait;
use starknet::SyscallResultTrait;
use starknet::syscalls::library_call_syscall;
use crate::arena::{dense_option, positions};
use crate::hashes::{ClassHashes, errors};

/// The two bodies of every touching pair of `pairs`, appended to `links`.
fn append_links(ref links: Array<(Option<Handle>, Option<Handle>)>, pairs: Span<ContactPair>) {
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            links.append((*pair.manifold.data.rigid_body1, *pair.manifold.data.rigid_body2));
        }
    }
}

/// A touching pair between the bodies of `link`: all the union-find reads of a pair.
fn link_pair(link: (Option<Handle>, Option<Handle>)) -> ContactPair {
    let (body1, body2) = link;
    let mut pair = ContactPairTrait::new(Default::default(), Default::default());
    pair.manifold.data.rigid_body1 = body1;
    pair.manifold.data.rigid_body2 = body2;
    pair.manifold.data.num_solver_contacts = 1;
    pair
}

/// All the island stage reads and writes of a body but its velocities (which it only zeroes).
#[derive(Copy, Drop, Serde)]
pub struct IslandBody {
    pub enabled: bool,
    pub body_type: RigidBodyType,
    pub activation: RigidBodyActivation,
    pub changes: RigidBodyChanges,
}

/// What the stage changed of a body: its activation and change flags.
#[derive(Copy, Drop, Serde)]
pub struct IslandChange {
    pub position: u32,
    pub activation: RigidBodyActivation,
    pub changes: RigidBodyChanges,
}

fn island_bodies(entries: Span<(Handle, RigidBody)>) -> Array<(Handle, IslandBody)> {
    let mut out = array![];
    for (handle, body) in entries {
        out
            .append(
                (
                    *handle,
                    IslandBody {
                        enabled: *body.enabled,
                        body_type: *body.body_type,
                        activation: *body.activation,
                        changes: *body.changes,
                    },
                ),
            );
    }
    out
}

/// `entries` with the class's `changed` bodies written to `bodies` and in place.
fn apply(
    ref bodies: RigidBodySet,
    entries: Span<(Handle, RigidBody)>,
    changed: Span<IslandChange>,
    sleeping: bool,
    woken: bool,
) -> (Span<(Handle, RigidBody)>, bool, bool) {
    if changed.is_empty() {
        return (entries, sleeping, woken);
    }
    let mut changed = changed;
    let mut out = array![];
    let mut position: u32 = 0;
    for entry in entries {
        let (handle, body) = *entry;
        let mut replaced = false;
        if let Some(head) = changed.get(0) {
            let change = *head.unbox();
            if change.position == position {
                let mut body = body;
                body.activation = change.activation;
                body.changes = change.changes;
                if change.activation.sleeping {
                    body.vels = RigidBodyVelocityTrait::zero();
                }
                let _ = bodies.set(handle, body);
                out.append((handle, body));
                let _ = changed.pop_front();
                replaced = true;
            }
        }
        if !replaced {
            out.append(*entry);
        }
        position += 1;
    }
    (out.span(), sleeping, woken)
}

fn decode(mut ret: Span<felt252>) -> (Span<IslandChange>, bool, bool) {
    Serde::deserialize(ref ret).expect(errors::DECODE)
}

/// `InProcessIslands` with the union-find library-called in `IslandsClass` (at `H::islands()`).
///
/// # Panics
/// `rapier2d::pipeline::config::errors::JOINTS` when joints reach the stage; `errors::DECODE`
/// when the class returns something else than its result; as the stage.
pub impl LibraryCallIslands<impl H: ClassHashes> of IslandStage {
    fn update_islands(
        ref bodies: RigidBodySet,
        pairs: Span<ContactPair>,
        dormant: Span<ContactPair>,
        joints: Span<(Handle, ImpulseJoint)>,
        entries: Span<(Handle, RigidBody)>,
        census: SleepCensus,
    ) -> (Span<(Handle, RigidBody)>, bool, bool) {
        assert(joints.is_empty(), JOINTS);
        if !(census.awake != 0
            && (census.eligible
                || (census.sleeping != 0 && links_awake_to_sleeping(pairs, joints, entries)))) {
            return (entries, census.sleeping != 0, false);
        }
        let mut links = array![];
        append_links(ref links, pairs);
        append_links(ref links, dormant);
        let mut calldata = array![];
        links.span().serialize(ref calldata);
        island_bodies(entries).span().serialize(ref calldata);
        let ret = library_call_syscall(H::islands(), selector!("update_islands"), calldata.span())
            .unwrap_syscall();
        let (changed, sleeping, woken) = decode(ret);
        apply(ref bodies, entries, changed, sleeping, woken)
    }

    fn islands_after_insertions(
        ref bodies: RigidBodySet,
        pairs: Span<ContactPair>,
        dormant: Span<ContactPair>,
        joints: Span<(Handle, ImpulseJoint)>,
        entries: Span<(Handle, RigidBody)>,
        census: SleepCensus,
        fresh: Span<Handle>,
    ) -> (Span<(Handle, RigidBody)>, bool, bool) {
        assert(joints.is_empty(), JOINTS);
        let mut calldata = array![];
        pairs.serialize(ref calldata);
        dormant.serialize(ref calldata);
        island_bodies(entries).span().serialize(ref calldata);
        census.serialize(ref calldata);
        fresh.serialize(ref calldata);
        let ret = library_call_syscall(
            H::islands(), selector!("islands_after_insertions"), calldata.span(),
        )
            .unwrap_syscall();
        let (changed, sleeping, woken) = decode(ret);
        apply(ref bodies, entries, changed, sleeping, woken)
    }
}

/// The bodies the stage sees: `IslandBody`s with zero velocities, at slots `0..n` when `renumber`
/// (the caller's handles otherwise).
fn rebuilt(entries: Span<(Handle, IslandBody)>, renumber: bool) -> Array<(Handle, RigidBody)> {
    let mut out = array![];
    let mut index: u32 = 0;
    for (handle, island) in entries {
        let mut body: RigidBody = Default::default();
        body.enabled = *island.enabled;
        body.body_type = *island.body_type;
        body.activation = *island.activation;
        body.changes = *island.changes;
        out.append((if renumber {
            Handle { index, generation: 0 }
        } else {
            *handle
        }, body));
        index += 1;
    }
    out
}

/// The bodies of `after` whose activation or change flags differ from `before`'s.
fn changes(
    before: Span<(Handle, RigidBody)>, after: Span<(Handle, RigidBody)>,
) -> Span<IslandChange> {
    let mut out = array![];
    let mut position: u32 = 0;
    let mut after = after;
    for (_, body) in before {
        let (_, new) = *after.pop_front().unwrap();
        if new.activation != *body.activation || new.changes != *body.changes {
            out.append(IslandChange { position, activation: new.activation, changes: new.changes });
        }
        position += 1;
    }
    out.span()
}

/// `update_islands_slow` on the links (see the module documentation): the changed bodies, whether
/// a member sleeps, whether a body woke up.
pub fn update_islands_values(
    links: Span<(Option<Handle>, Option<Handle>)>, entries: Span<(Handle, IslandBody)>,
) -> (Span<IslandChange>, bool, bool) {
    // Dense handles (`crate::arena`): the bodies take the slots of their positions.
    let n = entries.len();
    let mut at = positions(entries);
    let mut pairs = array![];
    for link in links {
        let (body1, body2) = *link;
        pairs.append(link_pair((dense_option(ref at, n, body1), dense_option(ref at, n, body2))));
    }
    let entries = rebuilt(entries, true).span();
    let mut bodies = RigidBodySetTrait::from_state(crate::arena::partial_state(entries));
    let (after, sleeping, woken) = update_islands_slow(
        ref bodies, pairs.span(), array![].span(), array![].span(), entries,
    );
    (changes(entries, after), sleeping, woken)
}

/// `islands_after_insertions` on the values: the changed bodies, whether a member sleeps, whether
/// a body woke up.
pub fn islands_after_insertions_values(
    pairs: Span<ContactPair>,
    dormant: Span<ContactPair>,
    entries: Span<(Handle, IslandBody)>,
    census: SleepCensus,
    fresh: Span<Handle>,
) -> (Span<IslandChange>, bool, bool) {
    let entries = rebuilt(entries, false).span();
    let mut bodies = RigidBodySetTrait::from_state(crate::arena::partial_state(entries));
    let (after, sleeping, woken) = islands_after_insertions(
        ref bodies, pairs, dormant, array![].span(), entries, census, fresh,
    );
    (changes(entries, after), sleeping, woken)
}

/// The island stage's union-find and rules (no joint).
#[starknet::contract]
pub mod IslandsClass {
    use rapier2d::pipeline::islands::SleepCensus;
    use rapier2d::prelude::{ContactPair, Handle};
    use super::{IslandBody, IslandChange};

    #[storage]
    struct Storage {}

    /// [`super::update_islands_values`].
    #[external(v0)]
    fn update_islands(
        self: @ContractState,
        links: Span<(Option<Handle>, Option<Handle>)>,
        entries: Span<(Handle, IslandBody)>,
    ) -> (Span<IslandChange>, bool, bool) {
        super::update_islands_values(links, entries)
    }

    /// [`super::islands_after_insertions_values`].
    #[external(v0)]
    fn islands_after_insertions(
        self: @ContractState,
        pairs: Span<ContactPair>,
        dormant: Span<ContactPair>,
        entries: Span<(Handle, IslandBody)>,
        census: SleepCensus,
        fresh: Span<Handle>,
    ) -> (Span<IslandChange>, bool, bool) {
        super::islands_after_insertions_values(pairs, dormant, entries, census, fresh)
    }
}
