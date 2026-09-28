//! CS5's crossing of the island stage's per-step path, measured against CX1's (`super`) and
//! rejected: the links as two `Option<Handle>`s, an [`IslandBody`] per body both ways (as
//! `IslandChange`s back), and in the class the bodies rebuilt at dense handles (`crate::arena`)
//! into a set. [`Cs5ValuesIslands`] runs its gather, class work and write-back in process
//! (`tests/steps.cairo`, `steps_islands_cs5_values_*`).
//!
//! Also CX1's first class work on CX1's wire: `update_islands_slow` itself on bodies rebuilt from
//! the packed values ([`update_islands_values_slow`], [`SlowValuesIslands`]).

use rapier2d::pipeline::config::errors::JOINTS;
use rapier2d::pipeline::islands::{SleepCensus, links_awake_to_sleeping, update_islands_slow};
use rapier2d::pipeline::sleeping::islands_after_insertions;
use rapier2d::pipeline::stages::IslandStage;
use rapier2d::prelude::{
    ContactPair, ContactPairTrait, Handle, ImpulseJoint, RigidBody, RigidBodySet, RigidBodySetTrait,
};
use rapier_core::rigid_body::changes::SLEEP;
use rapier_core::rigid_body::{RigidBodyActivation, RigidBodyChanges, RigidBodyType};
use rapier_dynamics2d::rigid_body::RigidBodyVelocityTrait;
use crate::arena::{dense_option, partial_state, positions};
use super::{
    ENABLED, FIXED, IslandBody, SLEEPING, SLEEP_RAISED, fixed_of, island_bodies, needs_slow_path,
    pack_islands, replay,
};

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

/// Measurement only: CS5's `LibraryCallIslands` per-step path with `update_islands_values`
/// called in process instead of library-called.
pub impl Cs5ValuesIslands of IslandStage {
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
        let (changed, sleeping, woken) = update_islands_values(
            links.span(), island_bodies(entries).span(),
        );
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
        islands_after_insertions(ref bodies, pairs, dormant, joints, entries, census, fresh)
    }
}

/// CS5's class work: `update_islands_slow` on the links, the bodies at dense handles in a set.
pub fn update_islands_values(
    links: Span<(Option<Handle>, Option<Handle>)>, entries: Span<(Handle, IslandBody)>,
) -> (Span<IslandChange>, bool, bool) {
    let n = entries.len();
    let mut at = positions(entries);
    let mut pairs = array![];
    for link in links {
        let (body1, body2) = *link;
        pairs.append(link_pair((dense_option(ref at, n, body1), dense_option(ref at, n, body2))));
    }
    let mut rebuilt = array![];
    let mut index: u32 = 0;
    for (_, island) in entries {
        let mut body: RigidBody = Default::default();
        body.enabled = *island.enabled;
        body.body_type = *island.body_type;
        body.activation = *island.activation;
        body.changes = *island.changes;
        rebuilt.append((Handle { index, generation: 0 }, body));
        index += 1;
    }
    let entries = rebuilt.span();
    let mut bodies = RigidBodySetTrait::from_state(partial_state(entries));
    let (after, sleeping, woken) = update_islands_slow(
        ref bodies, pairs.span(), array![].span(), array![].span(), entries,
    );
    (changes(entries, after), sleeping, woken)
}

/// Measurement only: `super::ValuesIslands` with [`update_islands_values_slow`] instead of the
/// decisions.
pub impl SlowValuesIslands of IslandStage {
    fn update_islands(
        ref bodies: RigidBodySet,
        pairs: Span<ContactPair>,
        dormant: Span<ContactPair>,
        joints: Span<(Handle, ImpulseJoint)>,
        entries: Span<(Handle, RigidBody)>,
        census: SleepCensus,
    ) -> (Span<(Handle, RigidBody)>, bool, bool) {
        assert(joints.is_empty(), JOINTS);
        if !needs_slow_path(pairs, joints, entries, census) {
            return (entries, census.sleeping != 0, false);
        }
        let mut links = array![];
        super::append_links(ref links, pairs);
        super::append_links(ref links, dormant);
        let (changed, sleeping, woken) = update_islands_values_slow(
            links.span(), pack_islands(island_bodies(entries).span()),
        );
        (replay(ref bodies, entries, changed), sleeping, woken)
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
        islands_after_insertions(ref bodies, pairs, dormant, joints, entries, census, fresh)
    }
}

/// CX1's first class work, measured against `super::update_islands_decisions` and rejected:
/// `update_islands_slow` on bodies rebuilt from the packed links and bodies (default other
/// fields, the caller's slots) and an empty set (its writes are no-ops, its result is read from
/// the entries it returns): one felt
/// per changed body (`position · 4 + decision`), whether a member sleeps, whether a body woke up.
pub fn update_islands_values_slow(
    links: Span<felt252>, bodies: Span<felt252>,
) -> (Span<felt252>, bool, bool) {
    let mut pairs = array![];
    for link in links {
        let link: u64 = (*link).try_into().unwrap();
        let (slot1, slot2) = DivRem::div_rem(link, 0x100000000_u64.try_into().unwrap());
        pairs
            .append(
                super::link_pair(
                    Some(Handle { index: slot1.try_into().unwrap(), generation: 0 }),
                    Some(Handle { index: slot2.try_into().unwrap(), generation: 0 }),
                ),
            );
    }
    let mut entries = array![];
    let mut bodies = bodies;
    while let Some(packed) = bodies.pop_front() {
        let packed: u64 = (*packed).try_into().unwrap();
        let (slot, flags) = DivRem::div_rem(packed, 16_u64.try_into().unwrap());
        let flags: u32 = flags.try_into().unwrap();
        let mut body: RigidBody = Default::default();
        body.enabled = flags & ENABLED != 0;
        if flags & FIXED != 0 {
            body.body_type = RigidBodyType::Fixed;
        }
        body.activation.sleeping = flags & SLEEPING != 0;
        if flags & SLEEP_RAISED != 0 {
            body.changes = SLEEP;
        }
        if body.enabled && flags & FIXED == 0 {
            body.activation.time_until_sleep = fixed_of(*bodies.pop_front().unwrap());
            body.activation.time_since_can_sleep = fixed_of(*bodies.pop_front().unwrap());
        }
        entries.append((Handle { index: slot.try_into().unwrap(), generation: 0 }, body));
    }
    let entries = entries.span();
    let mut set = RigidBodySetTrait::new();
    let (after, sleeping, woken) = update_islands_slow(
        ref set, pairs.span(), array![].span(), array![].span(), entries,
    );
    (super::decisions_of(entries, after), sleeping, woken)
}

/// What the stage changed of a body: its activation and change flags.
#[derive(Copy, Drop, Serde)]
pub struct IslandChange {
    pub position: u32,
    pub activation: RigidBodyActivation,
    pub changes: RigidBodyChanges,
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
