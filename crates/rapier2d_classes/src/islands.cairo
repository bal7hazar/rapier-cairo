//! The island stage (`StepConfig::Islands`: sleep and wake-up decisions) in a declared class
//! (CS5, CX1).
//!
//! [`LibraryCallIslands`] keeps `update_islands`' fast checks in the caller (no awake member, or
//! no eligible one and no link from an awake body to a sleeping one: nothing changes, no call) and
//! library-calls [`IslandsClass`] for the union-find, `update_islands_slow`. What crosses (CX1):
//! * in: one link per touching pair whose two sides have a body (the slots of the two bodies in
//!   one felt, [`pack_link`]: the union-find and the membership lookups only read the slot
//!   index), active pairs first, then the dormant ones, in the stage's order; an [`IslandBody`]
//!   per body (every entry of the stage, ascending slot), which the class packs
//!   ([`pack_islands`]: its slot and flags in one felt and, for an island member, its two sleep
//!   timers). Measured against the caller packing them (fewer felts, 283 more CASM felts in the
//!   slim caller, whose margin CX1 must keep: `REPORT`, margin options);
//! * out: one felt per body the stage changed (its position in the entries and whether it was
//!   woken up and / or put to sleep), whether a member sleeps and whether a body woke up.
//!
//! The class takes `update_islands_slow`'s decisions on those values alone
//! ([`update_islands_decisions`]: the same union-find, per-island flags and rules, no body
//! rebuilt), and the caller replays them on its own bodies with the stage's own calls,
//! `RigidBody::wake_up(true)` then `RigidBody::sleep()`, and writes them to its set, as the stage
//! does. Measured against `update_islands_slow` itself run in the class on bodies rebuilt from the
//! same values (`alternatives::update_islands_values`): the decisions cost fewer Cairo steps than
//! the in-process stage.
//!
//! A step that met colliders inserted since the last step (`islands_after_insertions`, whose
//! contact-start wake-up reads the pairs' event state and the bodies' types from a set) sends the
//! whole pairs and the same [`IslandBody`]s. CS5's crossing of the
//! per-step path is the measured loser, in [`alternatives`].

use core::dict::{Felt252Dict, Felt252DictTrait};
use rapier2d::pipeline::config::errors::JOINTS;
use rapier2d::pipeline::islands::{SleepCensus, links_awake_to_sleeping};
use rapier2d::pipeline::sleeping::islands_after_insertions;
use rapier2d::pipeline::stages::IslandStage;
use rapier2d::prelude::{
    ContactPair, ContactPairTrait, Fixed, Handle, ImpulseJoint, RigidBody, RigidBodySet,
    RigidBodySetTrait, RigidBodyTrait,
};
use rapier_core::data::union_find::{UnionFind, UnionFindTrait};
use rapier_core::rigid_body::changes::SLEEP;
use rapier_core::rigid_body::{
    RigidBodyActivation, RigidBodyChanges, RigidBodyChangesTrait, RigidBodyType,
};
use starknet::SyscallResultTrait;
use starknet::syscalls::library_call_syscall;
use crate::hashes::{ClassHashes, errors};

/// Measured and rejected: CS5's crossing of the per-step path.
pub mod alternatives;

/// `2^32`: the weight of the first slot of a packed link.
const TWO_POW_32: felt252 = 0x100000000;
/// The weight of a body's slot in its packed felt (four flag bits below it).
const SLOT: felt252 = 16;
/// The flag bits of a packed body.
pub(crate) const ENABLED: u32 = 1;
pub(crate) const FIXED: u32 = 2;
pub(crate) const SLEEPING: u32 = 4;
pub(crate) const SLEEP_RAISED: u32 = 8;
/// The decision bits of a packed change (the position in the entries weighs 4).
pub(crate) const WOKEN: u32 = 1;
pub(crate) const PUT_TO_SLEEP: u32 = 2;

/// A touching pair's link: the slots of its two bodies in one felt (`slot1 · 2^32 + slot2`).
#[inline(always)]
pub fn pack_link(body1: Handle, body2: Handle) -> felt252 {
    body1.index.into() * TWO_POW_32 + body2.index.into()
}

/// The links of the touching pairs of `pairs` whose two sides have a body, appended to `links`
/// (a side without a body never links: `islands::links`).
pub(crate) fn append_links(ref links: Array<felt252>, pairs: Span<ContactPair>) {
    for pair in pairs {
        if *pair.manifold.data.num_solver_contacts != 0 {
            if let (Some(h1), Some(h2)) =
                (*pair.manifold.data.rigid_body1, *pair.manifold.data.rigid_body2) {
                links.append(pack_link(h1, h2));
            }
        }
    }
}

/// All the island stage reads of the bodies of `entries`: per body its slot and flags in one felt
/// (`slot · 16 + flags`: enabled, fixed, sleeping, `SLEEP` raised) then, for an island member
/// (enabled, not fixed), its `time_until_sleep` and `time_since_can_sleep` (raw).
pub fn pack_islands(entries: Span<(Handle, IslandBody)>) -> Span<felt252> {
    let mut out = array![];
    for (handle, body) in entries {
        let enabled = *body.enabled;
        let fixed = *body.body_type == RigidBodyType::Fixed;
        let mut flags: u32 = 0;
        if enabled {
            flags += ENABLED;
        }
        if fixed {
            flags += FIXED;
        }
        if *body.activation.sleeping {
            flags += SLEEPING;
        }
        if body.changes.contains(SLEEP) {
            flags += SLEEP_RAISED;
        }
        out.append((*handle).index.into() * SLOT + flags.into());
        if enabled && !fixed {
            out.append((*body.activation.time_until_sleep.raw).into());
            out.append((*body.activation.time_since_can_sleep.raw).into());
        }
    }
    out.span()
}

/// `entries` with the class's decisions (`changed`, ascending position) replayed on their bodies
/// and written to `bodies`: `wake_up(true)` then `sleep()`, as the stage calls them.
pub(crate) fn replay(
    ref bodies: RigidBodySet, entries: Span<(Handle, RigidBody)>, changed: Span<felt252>,
) -> Span<(Handle, RigidBody)> {
    if changed.is_empty() {
        return entries;
    }
    let mut changed = changed;
    let (mut next, mut decision) = unpack_change(*changed.pop_front().unwrap());
    let mut out = array![];
    let mut position: u32 = 0;
    for entry in entries {
        if position == next {
            let (handle, mut body) = *entry;
            if decision & WOKEN != 0 {
                body.wake_up(true);
            }
            if decision & PUT_TO_SLEEP != 0 {
                body.sleep();
            }
            let _ = bodies.set_internal(handle, body);
            out.append((handle, body));
            match changed.pop_front() {
                Some(packed) => {
                    let (p, d) = unpack_change(*packed);
                    next = p;
                    decision = d;
                },
                None => { next = entries.len(); },
            }
        } else {
            out.append(*entry);
        }
        position += 1;
    }
    // `set` per body in the stage: the same values, and one `is_modified` flag.
    bodies.mark_modified();
    out.span()
}

#[inline(always)]
fn unpack_change(packed: felt252) -> (u32, u32) {
    let packed: u32 = packed.try_into().unwrap();
    let (position, decision) = DivRem::div_rem(packed, 4_u32.try_into().unwrap());
    (position, decision)
}

/// All the island stage reads and writes of a body but its velocities (which it only zeroes):
/// the crossing of `islands_after_insertions`.
#[derive(Copy, Drop, Serde)]
pub struct IslandBody {
    pub enabled: bool,
    pub body_type: RigidBodyType,
    pub activation: RigidBodyActivation,
    pub changes: RigidBodyChanges,
}

pub(crate) fn island_bodies(entries: Span<(Handle, RigidBody)>) -> Array<(Handle, IslandBody)> {
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

/// The fast checks of `islands::update_islands`: whether the slow path runs.
#[inline(always)]
pub(crate) fn needs_slow_path(
    pairs: Span<ContactPair>,
    joints: Span<(Handle, ImpulseJoint)>,
    entries: Span<(Handle, RigidBody)>,
    census: SleepCensus,
) -> bool {
    census.awake != 0
        && (census.eligible
            || (census.sleeping != 0 && links_awake_to_sleeping(pairs, joints, entries)))
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
        if !needs_slow_path(pairs, joints, entries, census) {
            return (entries, census.sleeping != 0, false);
        }
        let mut links = array![];
        append_links(ref links, pairs);
        append_links(ref links, dormant);
        let mut calldata = array![];
        links.span().serialize(ref calldata);
        island_bodies(entries).span().serialize(ref calldata);
        let mut ret = library_call_syscall(
            H::islands(), selector!("update_islands"), calldata.span(),
        )
            .unwrap_syscall();
        let (changed, sleeping, woken): (Span<felt252>, bool, bool) = Serde::deserialize(ref ret)
            .expect(errors::DECODE);
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
        assert(joints.is_empty(), JOINTS);
        let mut calldata = array![];
        pairs.serialize(ref calldata);
        dormant.serialize(ref calldata);
        island_bodies(entries).span().serialize(ref calldata);
        census.serialize(ref calldata);
        fresh.serialize(ref calldata);
        let mut ret = library_call_syscall(
            H::islands(), selector!("islands_after_insertions"), calldata.span(),
        )
            .unwrap_syscall();
        let (changed, sleeping, woken): (Span<felt252>, bool, bool) = Serde::deserialize(ref ret)
            .expect(errors::DECODE);
        (replay(ref bodies, entries, changed), sleeping, woken)
    }
}

/// Measurement only: [`LibraryCallIslands`]' per-step path with `update_islands_decisions`
/// called in process instead of library-called (the packing, the class's work and the replay
/// without the call), to split a call's cost.
pub impl ValuesIslands of IslandStage {
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
        append_links(ref links, pairs);
        append_links(ref links, dormant);
        let (changed, sleeping, woken) = update_islands_decisions(
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

/// A touching pair between the bodies of `link`: all the union-find reads of a pair.
pub(crate) fn link_pair(body1: Option<Handle>, body2: Option<Handle>) -> ContactPair {
    let mut pair = ContactPairTrait::new(Default::default(), Default::default());
    pair.manifold.data.rigid_body1 = body1;
    pair.manifold.data.rigid_body2 = body2;
    pair.manifold.data.num_solver_contacts = 1;
    pair
}

#[inline(always)]
pub(crate) fn fixed_of(raw: felt252) -> Fixed {
    Fixed { raw: raw.try_into().unwrap() }
}

/// `ordering::body_status` codes (`BODY_*`), stored plus nothing: 0 is a slot no body has,
/// which the stage counts as awake.
const STATUS_FIXED: u8 = 1;
const STATUS_DISABLED: u8 = 2;
const STATUS_SLEEPING: u8 = 3;
const STATUS_AWAKE: u8 = 4;

/// A packed body's island data: slot, member (enabled, not fixed), sleeping, eligible for sleep
/// (`time_since_can_sleep >= time_until_sleep`), positive `time_until_sleep`, non-zero
/// `time_since_can_sleep`.
#[derive(Copy, Drop)]
struct IslandFlags {
    slot: u32,
    member: bool,
    sleeping: bool,
    eligible: bool,
    positive_ttl: bool,
    since_nonzero: bool,
}

/// `update_islands_slow`'s decisions taken on the packed values alone (no body rebuilt): the
/// union-find of the members linked by `links`, the per-island flags, then per member of an
/// island with an awake member whether the stage wakes it up (`wake_up(true)`: a mixed island
/// and the body sleeps or has a non-zero sleep timer) and whether it puts it to sleep (a mixed
/// island without a positive `time_until_sleep`, or an island without an ineligible awake
/// member). Measured against `update_islands_slow` run on bodies rebuilt from the same values
/// (`alternatives::update_islands_values`), which reports the bodies whose values changed only.
pub fn update_islands_decisions(
    links: Span<felt252>, bodies: Span<felt252>,
) -> (Span<felt252>, bool, bool) {
    let mut status: Felt252Dict<u8> = Default::default();
    let mut decoded = array![];
    let mut bodies = bodies;
    while let Some(packed) = bodies.pop_front() {
        let packed: u64 = (*packed).try_into().unwrap();
        let (slot, flags) = DivRem::div_rem(packed, 16_u64.try_into().unwrap());
        let flags: u32 = flags.try_into().unwrap();
        let slot: u32 = slot.try_into().unwrap();
        let sleeping = flags & SLEEPING != 0;
        let code = if flags & FIXED != 0 {
            STATUS_FIXED
        } else if flags & ENABLED == 0 {
            STATUS_DISABLED
        } else if sleeping {
            STATUS_SLEEPING
        } else {
            STATUS_AWAKE
        };
        status.insert(slot.into(), code);
        let member = code >= STATUS_SLEEPING;
        let mut body = IslandFlags {
            slot, member, sleeping, eligible: false, positive_ttl: false, since_nonzero: false,
        };
        if member {
            let until = fixed_of(*bodies.pop_front().unwrap());
            let since = fixed_of(*bodies.pop_front().unwrap());
            body.eligible = since >= until;
            body.positive_ttl = until > Default::default();
            body.since_nonzero = since != Default::default();
        }
        decoded.append(body);
    }
    let mut forest: UnionFind = Default::default();
    for link in links {
        let link: u64 = (*link).try_into().unwrap();
        let (slot1, slot2) = DivRem::div_rem(link, 0x100000000_u64.try_into().unwrap());
        let (slot1, slot2): (u32, u32) = (slot1.try_into().unwrap(), slot2.try_into().unwrap());
        let s1 = status.get(slot1.into());
        let s2 = status.get(slot2.into());
        // `islands::links`: both sides are members (or slots without a body, counted awake).
        if (s1 == 0 || s1 >= STATUS_SLEEPING) && (s2 == 0 || s2 >= STATUS_SLEEPING) {
            let _ = forest.union(slot1, slot2);
        }
    }
    let mut has_awake: Felt252Dict<bool> = Default::default();
    let mut has_sleeping: Felt252Dict<bool> = Default::default();
    let mut blocked: Felt252Dict<bool> = Default::default();
    let mut positive_ttl: Felt252Dict<bool> = Default::default();
    let mut roots = array![];
    for body in decoded.span() {
        let mut root = 0;
        if *body.member {
            root = forest.find(*body.slot);
            let key: felt252 = root.into();
            if *body.sleeping {
                has_sleeping.insert(key, true);
            } else {
                has_awake.insert(key, true);
                if !*body.eligible {
                    blocked.insert(key, true);
                }
            }
            if *body.positive_ttl {
                positive_ttl.insert(key, true);
            }
        }
        roots.append(root);
    }
    let mut roots = roots.span();
    let mut changed = array![];
    let mut position: u32 = 0;
    let mut asleep_after: u32 = 0;
    let mut woken = false;
    for body in decoded.span() {
        let key: felt252 = (*roots.pop_front().unwrap()).into();
        if *body.member && has_awake.get(key) {
            let mixed = has_sleeping.get(key);
            let wakes = mixed && (*body.sleeping || *body.since_nonzero);
            if mixed && *body.sleeping {
                woken = true;
            }
            let sleeps = if mixed {
                !positive_ttl.get(key)
            } else {
                !blocked.get(key)
            };
            let mut decision: u32 = 0;
            if wakes {
                decision += WOKEN;
            }
            if sleeps {
                decision += PUT_TO_SLEEP;
            }
            if decision != 0 {
                changed.append((position * 4 + decision).into());
            }
            if sleeps || (*body.sleeping && !wakes) {
                asleep_after += 1;
            }
        } else if *body.member && *body.sleeping {
            asleep_after += 1;
        }
        position += 1;
    }
    (changed.span(), asleep_after != 0, woken)
}

/// The bodies the stage sees: `IslandBody`s with zero velocities at the caller's handles.
fn rebuilt(entries: Span<(Handle, IslandBody)>) -> Array<(Handle, RigidBody)> {
    let mut out = array![];
    for (handle, island) in entries {
        let mut body: RigidBody = Default::default();
        body.enabled = *island.enabled;
        body.body_type = *island.body_type;
        body.activation = *island.activation;
        body.changes = *island.changes;
        out.append((*handle, body));
    }
    out
}

/// The decisions (`position · 4 + decision`, see [`replay`]) that turn `before` into `after`
/// (the same bodies, in order): a body whose activation or change flags differ was woken up
/// (`wake_up(true)`) when it slept or is awake after the stage, and put to sleep (`sleep()`) when
/// it sleeps after it (a `wake_up(true)` of an awake body before changes nothing `sleep()` does
/// not overwrite; a second `wake_up(true)` nothing the first did not). A body written unchanged
/// is not reported.
pub(crate) fn decisions_of(
    before: Span<(Handle, RigidBody)>, after: Span<(Handle, RigidBody)>,
) -> Span<felt252> {
    let mut changed = array![];
    let mut position: u32 = 0;
    let mut after = after;
    for (_, body) in before {
        let (_, new) = *after.pop_front().unwrap();
        if new.activation != *body.activation || new.changes != *body.changes {
            let mut decision = 0;
            if *body.activation.sleeping || !new.activation.sleeping {
                decision += WOKEN;
            }
            if new.activation.sleeping {
                decision += PUT_TO_SLEEP;
            }
            changed.append((position * 4 + decision).into());
        }
        position += 1;
    }
    changed.span()
}

/// `islands_after_insertions` on the values: the decisions of the changed bodies
/// ([`decisions_of`]), whether a member sleeps, whether a body woke up.
pub fn islands_after_insertions_values(
    pairs: Span<ContactPair>,
    dormant: Span<ContactPair>,
    entries: Span<(Handle, IslandBody)>,
    census: SleepCensus,
    fresh: Span<Handle>,
) -> (Span<felt252>, bool, bool) {
    let entries = rebuilt(entries).span();
    let mut bodies = RigidBodySetTrait::from_state(crate::arena::partial_state(entries));
    let (after, sleeping, woken) = islands_after_insertions(
        ref bodies, pairs, dormant, array![].span(), entries, census, fresh,
    );
    (decisions_of(entries, after), sleeping, woken)
}

/// The island stage's union-find and rules (no joint).
#[starknet::contract]
pub mod IslandsClass {
    use rapier2d::pipeline::islands::SleepCensus;
    use rapier2d::prelude::{ContactPair, Handle};
    use super::IslandBody;

    #[storage]
    struct Storage {}

    /// [`super::update_islands_decisions`] of the packed bodies ([`super::pack_islands`]).
    #[external(v0)]
    fn update_islands(
        self: @ContractState, links: Span<felt252>, bodies: Span<(Handle, IslandBody)>,
    ) -> (Span<felt252>, bool, bool) {
        super::update_islands_decisions(links, super::pack_islands(bodies))
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
    ) -> (Span<felt252>, bool, bool) {
        super::islands_after_insertions_values(pairs, dormant, entries, census, fresh)
    }
}
