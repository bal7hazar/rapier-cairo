//! Rejected candidates, kept for the gas ranking (`benches`) and the equivalence tests
//! (`tests`).
//!
//! [`compute_contacts_with`]: the pair loop shipped before ON, generic over the carry-over
//! strategy: [`CarryOver::take`] copies the previous pair out as an `Option<ContactPair>` and
//! the outlined [`process_pair`] is charged its costliest path on every pair.
//!
//! [`DictCarryOver`]: the previous pairs indexed in a `Felt252Dict` keyed by the packed handle
//! pair, value `index + 1` (0 = absent); a found entry is zeroed so that the pairs left non-zero
//! at the end are the dropped ones, scanned in ascending key to keep the event order of the
//! sorted merge. Costs one dict write per previous pair, one entry per current pair, one read
//! per previous pair at the end, and the squash.

use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
use fixed::Fixed;
use rapier_core::Handle;
use rapier_core::data::handle::HandleTrait;
use crate::collider_set::ColliderSet;
use crate::events::CollisionEvent;
use crate::rigid_body_set::RigidBodySet;
use super::{
    CarryOver, ContactDispatcher, ContactPair, NarrowPhase, PairCollider, dropped_events,
    pair_colliders, process_pair,
};

/// `2^64`, the weight of the first handle in the packed pair key.
const TWO_POW_64: felt252 = 0x10000000000000000;

/// Injective felt key of a pair of handles: `key(h1) * 2^64 + key(h2)` (each key < 2^64).
#[inline(always)]
pub fn pair_key(collider1: Handle, collider2: Handle) -> felt252 {
    collider1.key() * TWO_POW_64 + collider2.key()
}

/// Dict-based lookup state.
#[derive(Destruct)]
pub struct DictCarryOver {
    previous: Span<ContactPair>,
    index: Felt252Dict<u32>,
}

pub impl DictCarryOverImpl of CarryOver<DictCarryOver> {
    fn begin(previous: Span<ContactPair>) -> DictCarryOver {
        let mut index: Felt252Dict<u32> = Default::default();
        let n = previous.len();
        let mut i = 0;
        while i != n {
            let pair = *previous.at(i);
            i += 1;
            index.insert(pair_key(pair.collider1, pair.collider2), i);
        }
        DictCarryOver { previous, index }
    }

    fn take(ref self: DictCarryOver, collider1: Handle, collider2: Handle) -> Option<ContactPair> {
        let (entry, slot) = self.index.entry(pair_key(collider1, collider2));
        self.index = entry.finalize(0);
        if slot == 0 {
            None
        } else {
            Some(*self.previous.at(slot - 1))
        }
    }

    fn finish(self: DictCarryOver) -> Array<ContactPair> {
        let DictCarryOver { previous, mut index } = self;
        let mut dropped = array![];
        for pair in previous {
            if index.get(pair_key(*pair.collider1, *pair.collider2)) != 0 {
                dropped.append(*pair);
            }
        }
        dropped
    }
}

/// `compute_contacts` with the dict carry-over.
pub fn compute_contacts_dict<impl D: ContactDispatcher>(
    ref narrow_phase: NarrowPhase,
    prediction: Fixed,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    pairs: Span<(u32, u32)>,
) -> Array<CollisionEvent> {
    compute_contacts_with::<
        D, DictCarryOver,
    >(ref narrow_phase, prediction, ref bodies, ref colliders, pairs)
}

/// The pre-ON pair loop with the carry-over strategy `C`: `compute_contacts` built on
/// [`CarryOver`] and the outlined [`process_pair`].
pub fn compute_contacts_with<impl D: ContactDispatcher, S, impl C: CarryOver<S>, +Destruct<S>>(
    ref self: NarrowPhase,
    prediction: Fixed,
    ref bodies: RigidBodySet,
    ref colliders: ColliderSet,
    pairs: Span<(u32, u32)>,
) -> Array<CollisionEvent> {
    let scratch: Span<PairCollider> = pair_colliders(ref bodies, ref colliders).span();
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
        let (pair, event) = process_pair::<D>(prediction, co1, co2, previous);
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
