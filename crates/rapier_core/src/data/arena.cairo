//! Generational arena (upstream `rapier::data::Arena`, itself adapted from `generational-arena`).
//!
//! The arena hands out [`Handle`]s and recycles the slots of removed values. Semantics follow
//! upstream so that handles are bit-identical to the Rust engine for the same call sequence:
//!
//! * one **arena-wide generation counter**, bumped by every successful `remove`; a slot filled by
//!   `insert` is stamped with the current counter, so a handle kept across a remove + reinsert
//!   of its slot is stale;
//! * freed slots are reused **last freed, first reused** (intrusive LIFO free list); when the
//!   list is empty the next never-used index is allocated;
//! * iteration ([`ArenaTrait::to_array`]) is in ascending slot index, never in dict order.
//!
//! Deviation from upstream: `capacity` is the number of slots ever allocated (the high-water
//! mark), not a reserved `Vec` length — there is nothing to reserve in Cairo — and values must
//! be `Copy` because `get` returns them by value.
//!
//! Candidates (all behind [`ArenaTrait`], ranked by the `gas_*` probes of this module):
//!
//! 1. [`Arena`] — one `Felt252Dict<Nullable<Slot<T>>>`; the generation and the free-list link
//!    live inside the boxed slot, every operation is a single dict access (`entry` /
//!    `finalize` for read-modify-write). **Winner**: cheapest dict layout on insert, get,
//!    replace, remove, slot reuse and iteration, and O(1) on every mutation.
//! 2. `alternatives::SplitArena` — `Felt252Dict<Nullable<T>>` for values plus a
//!    `Felt252Dict<u64>` for generations and free-list links. Only wins `contains` (no unboxing)
//!    and, by a few percent, `set`; pays a second dict everywhere else.
//! 3. `alternatives::ArrayArena` — `Array<Slot<T>>`, rebuilt on every in-place mutation. Cheapest
//!    on fresh insert, get, iteration and bulk `set_all` (no dict, no squash), but `set`,
//!    `remove` and slot reuse are O(capacity): already 4× the dict cost at 8 items, 10× at 32,
//!    and a handle-by-handle write-back of all items is quadratic. Rejected as the general
//!    container; the ranking is a reminder that read-mostly per-step data belongs in arrays.

use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
use core::nullable::{FromNullableResult, NullableTrait, match_nullable};
use core::num::traits::CheckedAdd;
use super::handle::Handle;

#[cfg(test)]
mod alternatives;

/// Panic messages of the arena.
pub mod errors {
    /// All `2^32 - 1` slot indices are allocated.
    pub const CAPACITY_OVERFLOW: felt252 = 'Arena: capacity overflow';
    /// The generation counter cannot be bumped past `2^32 - 1`.
    pub const GENERATION_OVERFLOW: felt252 = 'Arena: generation overflow';
    /// Internal invariant broken: the free list reached a live or unallocated slot.
    pub const CORRUPT_FREE_LIST: felt252 = 'Arena: corrupt free list';
    /// `set_all` received a number of values different from `len`.
    pub const LENGTH_MISMATCH: felt252 = 'Arena: length mismatch';
    /// `from_state`: entries are not in strictly ascending slot index.
    pub const STATE_UNORDERED: felt252 = 'Arena: state entries unordered';
    /// `from_state`: a slot index is not below `capacity`.
    pub const STATE_OUT_OF_RANGE: felt252 = 'Arena: state index too large';
    /// `from_state`: an entry generation is above the arena generation.
    pub const STATE_GENERATION: felt252 = 'Arena: state generation too new';
    /// `from_state`: a free slot is also live, or listed twice.
    pub const STATE_SLOT_REUSED: felt252 = 'Arena: state slot listed twice';
    /// `from_state`: `entries.len() + free_list.len() != capacity`.
    pub const STATE_INCOMPLETE: felt252 = 'Arena: state misses slots';
}

/// Free-list terminator; also the only `u32` that is never a valid slot index.
pub(crate) const NO_SLOT: u32 = 0xffffffff;

/// Content of an allocated arena slot.
#[derive(Copy, Drop)]
pub(crate) enum Slot<T> {
    /// Vacant slot, holding the next free slot index (or `NO_SLOT`).
    Free: u32,
    /// Live slot: generation it was filled at, and its value.
    Occupied: (u32, T),
}

/// Operations of a generational arena of `T` stored in the container `A`.
///
/// Reads take `ref self` because `Felt252Dict` reads mutate the dict's access log.
pub trait ArenaTrait<A, T> {
    /// Returns an empty arena (length 0, capacity 0, generation 0).
    fn new() -> A;

    /// Stores `value` and returns its handle.
    ///
    /// Reuses the most recently freed slot if any, else allocates index `capacity`. The handle
    /// generation is the current arena generation.
    ///
    /// # Panics
    /// `Arena: capacity overflow` when all `2^32 - 1` slots are allocated.
    fn insert(ref self: A, value: T) -> Handle;

    /// Returns the value behind `handle`, or `None` if the handle is stale, was removed, or was
    /// never issued by this arena.
    fn get(ref self: A, handle: Handle) -> Option<T>;

    /// Returns `true` when `handle` resolves to a live value.
    fn contains(ref self: A, handle: Handle) -> bool;

    /// Overwrites the value behind `handle`. Returns `false` (and changes nothing) when the
    /// handle does not resolve.
    fn set(ref self: A, handle: Handle, value: T) -> bool;

    /// Same as [`set`](ArenaTrait::set) but returns the previous value, `None` when the handle
    /// does not resolve.
    fn replace(ref self: A, handle: Handle, value: T) -> Option<T>;

    /// Overwrites every live value, `values` being in ascending slot index like the output of
    /// [`to_array`](ArenaTrait::to_array). Handles, generations and the free list are untouched.
    ///
    /// # Panics
    /// `Arena: length mismatch` when `values.len() != self.len()`.
    fn set_all(ref self: A, values: Span<T>);

    /// Removes and returns the value behind `handle`, `None` when the handle does not resolve
    /// (in particular on a second removal). A successful removal bumps the arena generation
    /// and pushes the slot on the free list.
    ///
    /// # Panics
    /// `Arena: generation overflow` on the `2^32`-th removal.
    fn remove(ref self: A, handle: Handle) -> Option<T>;

    /// Number of live values.
    fn len(self: @A) -> u32;

    /// `true` when no value is live.
    fn is_empty(self: @A) -> bool;

    /// Number of slots ever allocated; every live handle has `index < capacity`.
    fn capacity(self: @A) -> u32;

    /// Returns every live `(handle, value)` in **ascending slot index**. This is the iteration
    /// primitive of the arena; its order only depends on the sequence of inserts and removes.
    fn to_array(ref self: A) -> Array<(Handle, T)>;
}

/// Flat, serialisable image of an [`Arena`], for persistence between steps.
///
/// Restoring it with [`ArenaStateTrait::from_state`] yields an arena that behaves exactly like
/// the original one (same handles for the same future calls).
#[derive(Copy, Drop, Serde)]
pub struct ArenaState<T> {
    /// Arena generation counter.
    pub generation: u32,
    /// Number of allocated slots.
    pub capacity: u32,
    /// Free slot indices, next slot to be reused first.
    pub free_list: Span<u32>,
    /// Live `(handle, value)` pairs in ascending slot index.
    pub entries: Span<(Handle, T)>,
}

/// Generational arena backed by a single `Felt252Dict`.
///
/// Holds a dict, hence `Destruct` instead of `Drop`: pass it by `ref`.
pub struct Arena<T> {
    /// Slot index → boxed slot; null for never-allocated indices.
    slots: Felt252Dict<Nullable<Slot<T>>>,
    /// Arena generation counter (number of successful removals).
    generation: u32,
    /// Most recently freed slot, or `NO_SLOT`.
    free_head: u32,
    /// Number of live values.
    len: u32,
    /// Number of allocated slots.
    capacity: u32,
}

/// Squashes the backing dict when the arena goes out of scope.
pub impl ArenaDestruct<T, +Drop<T>> of Destruct<Arena<T>> {
    fn destruct(self: Arena<T>) nopanic {
        self.slots.squash();
    }
}

/// The default arena is the empty one.
pub impl ArenaDefault<T, +Copy<T>, +Drop<T>> of Default<Arena<T>> {
    fn default() -> Arena<T> {
        ArenaTrait::new()
    }
}

/// [`ArenaTrait`] for the shipped, single-dict [`Arena`]: every operation below costs one
/// dict access, except `to_array` / `set_all` which cost one per allocated slot.
pub impl ArenaImpl<T, +Copy<T>, +Drop<T>> of ArenaTrait<Arena<T>, T> {
    fn new() -> Arena<T> {
        Arena { slots: Default::default(), generation: 0, free_head: NO_SLOT, len: 0, capacity: 0 }
    }

    fn insert(ref self: Arena<T>, value: T) -> Handle {
        let generation = self.generation;
        let slot = NullableTrait::new(Slot::Occupied((generation, value)));
        let index = self.free_head;
        let index = if index == NO_SLOT {
            let index = self.capacity;
            assert(index != NO_SLOT, errors::CAPACITY_OVERFLOW);
            self.capacity = index + 1;
            self.slots.insert(index.into(), slot);
            index
        } else {
            let (entry, previous) = self.slots.entry(index.into());
            self.slots = entry.finalize(slot);
            self.free_head = match match_nullable(previous) {
                FromNullableResult::NotNull(previous) => match previous.unbox() {
                    Slot::Free(next) => next,
                    Slot::Occupied(_) => core::panic_with_felt252(errors::CORRUPT_FREE_LIST),
                },
                FromNullableResult::Null => core::panic_with_felt252(errors::CORRUPT_FREE_LIST),
            };
            index
        };
        self.len += 1;
        Handle { index, generation }
    }

    fn get(ref self: Arena<T>, handle: Handle) -> Option<T> {
        match match_nullable(self.slots.get(handle.index.into())) {
            FromNullableResult::NotNull(slot) => match slot.unbox() {
                Slot::Occupied((generation, value)) => if generation == handle.generation {
                    Option::Some(value)
                } else {
                    Option::None
                },
                Slot::Free(_) => Option::None,
            },
            FromNullableResult::Null => Option::None,
        }
    }

    fn contains(ref self: Arena<T>, handle: Handle) -> bool {
        self.get(handle).is_some()
    }

    fn set(ref self: Arena<T>, handle: Handle, value: T) -> bool {
        self.replace(handle, value).is_some()
    }

    fn replace(ref self: Arena<T>, handle: Handle, value: T) -> Option<T> {
        let (entry, previous) = self.slots.entry(handle.index.into());
        if let FromNullableResult::NotNull(slot) = match_nullable(previous) {
            if let Slot::Occupied((generation, old)) = slot.unbox() {
                if generation == handle.generation {
                    self
                        .slots = entry
                        .finalize(NullableTrait::new(Slot::Occupied((generation, value))));
                    return Option::Some(old);
                }
            }
        }
        self.slots = entry.finalize(previous);
        Option::None
    }

    fn set_all(ref self: Arena<T>, mut values: Span<T>) {
        assert(values.len() == self.len, errors::LENGTH_MISMATCH);
        let capacity = self.capacity;
        let mut index = 0;
        while index != capacity {
            let (entry, previous) = self.slots.entry(index.into());
            let mut next = previous;
            if let FromNullableResult::NotNull(slot) = match_nullable(previous) {
                if let Slot::Occupied((generation, _)) = slot.unbox() {
                    // Exactly `len == values.len()` slots are live: the pop cannot fail.
                    if let Option::Some(value) = values.pop_front() {
                        next = NullableTrait::new(Slot::Occupied((generation, *value)));
                    }
                }
            }
            self.slots = entry.finalize(next);
            index += 1;
        }
    }

    fn remove(ref self: Arena<T>, handle: Handle) -> Option<T> {
        let (entry, previous) = self.slots.entry(handle.index.into());
        if let FromNullableResult::NotNull(slot) = match_nullable(previous) {
            if let Slot::Occupied((generation, value)) = slot.unbox() {
                if generation == handle.generation {
                    self.slots = entry.finalize(NullableTrait::new(Slot::Free(self.free_head)));
                    self.free_head = handle.index;
                    self.len -= 1;
                    self
                        .generation = self
                        .generation
                        .checked_add(1)
                        .expect(errors::GENERATION_OVERFLOW);
                    return Option::Some(value);
                }
            }
        }
        self.slots = entry.finalize(previous);
        Option::None
    }

    #[inline(always)]
    fn len(self: @Arena<T>) -> u32 {
        *self.len
    }

    #[inline(always)]
    fn is_empty(self: @Arena<T>) -> bool {
        *self.len == 0
    }

    #[inline(always)]
    fn capacity(self: @Arena<T>) -> u32 {
        *self.capacity
    }

    fn to_array(ref self: Arena<T>) -> Array<(Handle, T)> {
        let mut entries = array![];
        let capacity = self.capacity;
        let mut index = 0;
        while index != capacity {
            if let FromNullableResult::NotNull(slot) =
                match_nullable(self.slots.get(index.into())) {
                if let Slot::Occupied((generation, value)) = slot.unbox() {
                    entries.append((Handle { index, generation }, value));
                }
            }
            index += 1;
        }
        entries
    }
}

/// Persistence of an [`Arena`]: flatten to / rebuild from an [`ArenaState`].
#[generate_trait]
pub impl ArenaStateImpl<T, +Copy<T>, +Drop<T>> of ArenaStateTrait<T> {
    /// Returns the current generation counter (the generation the next insert is stamped with).
    #[inline(always)]
    fn generation(self: @Arena<T>) -> u32 {
        *self.generation
    }

    /// Flattens the arena. Cost: one dict read per allocated slot plus one per free slot.
    fn to_state(ref self: Arena<T>) -> ArenaState<T> {
        let entries = self.to_array().span();
        let mut free_list = array![];
        let mut index = self.free_head;
        while index != NO_SLOT {
            free_list.append(index);
            index = match match_nullable(self.slots.get(index.into())) {
                FromNullableResult::NotNull(slot) => match slot.unbox() {
                    Slot::Free(next) => next,
                    Slot::Occupied(_) => core::panic_with_felt252(errors::CORRUPT_FREE_LIST),
                },
                FromNullableResult::Null => core::panic_with_felt252(errors::CORRUPT_FREE_LIST),
            };
        }
        ArenaState {
            generation: self.generation,
            capacity: self.capacity,
            free_list: free_list.span(),
            entries,
        }
    }

    /// Rebuilds an arena from its flat image. Cost: one dict write per allocated slot.
    ///
    /// # Panics
    /// When the state is not a valid image: entries not strictly ascending, an index
    /// `>= capacity`, an entry generation above `generation`, a slot listed twice, or
    /// `entries.len() + free_list.len() != capacity`.
    fn from_state(state: ArenaState<T>) -> Arena<T> {
        let ArenaState { generation, capacity, mut free_list, mut entries } = state;
        assert(capacity != NO_SLOT, errors::CAPACITY_OVERFLOW);
        assert(entries.len() + free_list.len() == capacity, errors::STATE_INCOMPLETE);
        let len = entries.len();
        let mut slots: Felt252Dict<Nullable<Slot<T>>> = Default::default();

        // Entries are strictly ascending, hence pairwise distinct.
        let mut lower_bound = 0;
        while let Option::Some(entry) = entries.pop_front() {
            let (handle, value) = *entry;
            assert(handle.index >= lower_bound, errors::STATE_UNORDERED);
            assert(handle.index < capacity, errors::STATE_OUT_OF_RANGE);
            assert(handle.generation <= generation, errors::STATE_GENERATION);
            slots
                .insert(
                    handle.index.into(),
                    NullableTrait::new(Slot::Occupied((handle.generation, value))),
                );
            lower_bound = handle.index + 1;
        }

        // Free slots must not collide with entries nor with each other; together with the
        // count check above this proves every slot below `capacity` is listed exactly once.
        let free_head = match free_list.get(0) {
            Option::Some(head) => *head.unbox(),
            Option::None => NO_SLOT,
        };
        while let Option::Some(index) = free_list.pop_front() {
            let index = *index;
            assert(index < capacity, errors::STATE_OUT_OF_RANGE);
            let next = match free_list.get(0) {
                Option::Some(next) => *next.unbox(),
                Option::None => NO_SLOT,
            };
            let (entry, previous) = slots.entry(index.into());
            let vacant = previous.is_null();
            slots = entry.finalize(NullableTrait::new(Slot::Free(next)));
            assert(vacant, errors::STATE_SLOT_REUSED);
        }

        Arena { slots, generation, free_head, len, capacity }
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use crate::data::handle::{Handle, HandleTrait, INVALID_HANDLE};
    use super::alternatives::{ArrayArena, SplitArena};
    use super::{Arena, ArenaState, ArenaStateTrait, ArenaTrait};

    /// Stand-in for a small engine component (eight scalars).
    #[derive(Copy, Drop, Serde, PartialEq, Debug)]
    struct Item {
        a: u64,
        b: u64,
        c: u64,
        d: u64,
        e: u64,
        f: u64,
        g: u64,
        h: u64,
    }

    fn item(seed: u64) -> Item {
        Item {
            a: seed,
            b: seed + 1,
            c: seed + 2,
            d: seed + 3,
            e: seed + 4,
            f: seed + 5,
            g: seed + 6,
            h: seed + 7,
        }
    }

    fn h(index: u32, generation: u32) -> Handle {
        HandleTrait::new(index, generation)
    }

    /// Arena with `n` live items `item(10 * i)` at handles `(i, 0)`. Never inlined so that every
    /// probe pays the exact same setup.
    #[inline(never)]
    fn filled<A, +ArenaTrait<A, Item>, +Destruct<A>>(n: u32) -> A {
        let mut arena: A = ArenaTrait::new();
        let mut i: u32 = 0;
        while i != n {
            arena.insert(item(opaque(i.into() * 10)));
            i += 1;
        }
        arena
    }

    // Behaviour, checked identically on every candidate.

    fn check_empty<A, +ArenaTrait<A, Item>, +Destruct<A>>() {
        let mut arena: A = ArenaTrait::new();
        assert_eq!(arena.len(), 0);
        assert!(arena.is_empty());
        assert_eq!(arena.capacity(), 0);
        assert_eq!(arena.get(h(0, 0)), Option::None);
        assert_eq!(arena.get(INVALID_HANDLE), Option::None);
        assert!(!arena.contains(h(0, 0)));
        assert!(!arena.set(h(0, 0), item(1)));
        assert_eq!(arena.replace(h(0, 0), item(1)), Option::None);
        assert_eq!(arena.remove(h(0, 0)), Option::None);
        assert_eq!(arena.remove(INVALID_HANDLE), Option::None);
        assert_eq!(arena.to_array(), array![]);
        arena.set_all([].span());
        assert_eq!(arena.len(), 0);
        assert_eq!(arena.capacity(), 0);
    }

    fn check_insert_get<A, +ArenaTrait<A, Item>, +Destruct<A>>() {
        let mut arena: A = ArenaTrait::new();
        assert_eq!(arena.insert(item(10)), h(0, 0));
        assert_eq!(arena.insert(item(20)), h(1, 0));
        assert_eq!(arena.insert(item(30)), h(2, 0));
        assert_eq!(arena.len(), 3);
        assert!(!arena.is_empty());
        assert_eq!(arena.capacity(), 3);
        assert_eq!(arena.get(h(0, 0)), Option::Some(item(10)));
        assert_eq!(arena.get(h(1, 0)), Option::Some(item(20)));
        assert_eq!(arena.get(h(2, 0)), Option::Some(item(30)));
        assert!(arena.contains(h(2, 0)));
        // Never issued: index past capacity, generation from the future, invalid handle.
        assert_eq!(arena.get(h(3, 0)), Option::None);
        assert_eq!(arena.get(h(1, 1)), Option::None);
        assert_eq!(arena.get(INVALID_HANDLE), Option::None);
        assert!(!arena.contains(h(3, 0)));
        assert!(!arena.contains(h(1, 1)));
    }

    fn check_set_replace<A, +ArenaTrait<A, Item>, +Destruct<A>>() {
        let mut arena: A = filled(3);
        assert!(arena.set(h(1, 0), item(11)));
        assert_eq!(arena.get(h(1, 0)), Option::Some(item(11)));
        assert_eq!(arena.replace(h(1, 0), item(12)), Option::Some(item(11)));
        assert_eq!(arena.get(h(1, 0)), Option::Some(item(12)));
        // Misses change nothing.
        assert!(!arena.set(h(1, 1), item(99)));
        assert!(!arena.set(h(7, 0), item(99)));
        assert_eq!(arena.replace(h(1, 1), item(99)), Option::None);
        assert_eq!(arena.get(h(1, 0)), Option::Some(item(12)));
        assert_eq!(arena.get(h(0, 0)), Option::Some(item(0)));
        assert_eq!(arena.get(h(2, 0)), Option::Some(item(20)));
        assert_eq!(arena.len(), 3);
        assert_eq!(arena.capacity(), 3);
    }

    fn check_stale_handle<A, +ArenaTrait<A, Item>, +Destruct<A>>() {
        let mut arena: A = filled(2);
        let stale = h(0, 0);
        assert_eq!(arena.remove(stale), Option::Some(item(0)));
        assert_eq!(arena.len(), 1);
        assert_eq!(arena.get(stale), Option::None);
        assert!(!arena.contains(stale));

        // The slot is reused under the bumped generation.
        let fresh = arena.insert(item(77));
        assert_eq!(fresh, h(0, 1));
        assert_eq!(arena.len(), 2);
        assert_eq!(arena.capacity(), 2);

        // The stale handle keeps missing and cannot touch the new occupant.
        assert_eq!(arena.get(stale), Option::None);
        assert!(!arena.contains(stale));
        assert!(!arena.set(stale, item(99)));
        assert_eq!(arena.replace(stale, item(99)), Option::None);
        assert_eq!(arena.remove(stale), Option::None);
        assert_eq!(arena.get(fresh), Option::Some(item(77)));
        assert_eq!(arena.len(), 2);
    }

    fn check_double_remove<A, +ArenaTrait<A, Item>, +Destruct<A>>() {
        let mut arena: A = filled(3);
        assert_eq!(arena.remove(h(1, 0)), Option::Some(item(10)));
        assert_eq!(arena.remove(h(1, 0)), Option::None);
        assert_eq!(arena.len(), 2);
        assert_eq!(arena.capacity(), 3);
        // The failed removal neither bumped the generation nor pushed the slot twice.
        assert_eq!(arena.insert(item(1)), h(1, 1));
        assert_eq!(arena.insert(item(2)), h(3, 1));
        assert_eq!(arena.len(), 4);
    }

    fn check_lifo_reuse<A, +ArenaTrait<A, Item>, +Destruct<A>>() {
        let mut arena: A = filled(5);
        assert!(arena.remove(h(1, 0)).is_some());
        assert!(arena.remove(h(3, 0)).is_some());
        assert!(arena.remove(h(0, 0)).is_some());
        // Last freed first, then fresh indices; generation = number of removals so far.
        assert_eq!(arena.insert(item(1)), h(0, 3));
        assert_eq!(arena.insert(item(2)), h(3, 3));
        assert!(arena.remove(h(4, 0)).is_some());
        assert_eq!(arena.insert(item(3)), h(4, 4));
        assert_eq!(arena.insert(item(4)), h(1, 4));
        assert_eq!(arena.insert(item(5)), h(5, 4));
        assert_eq!(arena.len(), 6);
        assert_eq!(arena.capacity(), 6);
    }

    fn check_iteration_order<A, +ArenaTrait<A, Item>, +Destruct<A>>() {
        let mut arena: A = filled(5);
        assert!(arena.remove(h(3, 0)).is_some());
        assert!(arena.remove(h(1, 0)).is_some());
        assert_eq!(
            arena.to_array(), array![(h(0, 0), item(0)), (h(2, 0), item(20)), (h(4, 0), item(40))],
        );
        // Reuse happens in LIFO order (1 then 3), iteration stays in ascending index.
        assert_eq!(arena.insert(item(11)), h(1, 2));
        assert_eq!(arena.insert(item(33)), h(3, 2));
        assert_eq!(
            arena.to_array(),
            array![
                (h(0, 0), item(0)), (h(1, 2), item(11)), (h(2, 0), item(20)), (h(3, 2), item(33)),
                (h(4, 0), item(40)),
            ],
        );
    }

    fn check_remove_all<A, +ArenaTrait<A, Item>, +Destruct<A>>() {
        let mut arena: A = filled(3);
        assert!(arena.remove(h(0, 0)).is_some());
        assert!(arena.remove(h(1, 0)).is_some());
        assert!(arena.remove(h(2, 0)).is_some());
        assert!(arena.is_empty());
        assert_eq!(arena.capacity(), 3);
        assert_eq!(arena.to_array(), array![]);
        assert_eq!(arena.insert(item(5)), h(2, 3));
    }

    fn check_set_all<A, +ArenaTrait<A, Item>, +Destruct<A>>() {
        let mut arena: A = filled(4);
        assert!(arena.remove(h(0, 0)).is_some());
        assert!(arena.remove(h(2, 0)).is_some());
        arena.set_all([item(100), item(300)].span());
        assert_eq!(arena.to_array(), array![(h(1, 0), item(100)), (h(3, 0), item(300))]);
        // The free list survived.
        assert_eq!(arena.insert(item(200)), h(2, 2));
        assert_eq!(arena.insert(item(0)), h(0, 2));
        assert_eq!(arena.len(), 4);
    }

    fn check_set_all_length_mismatch<A, +ArenaTrait<A, Item>, +Destruct<A>>() {
        let mut arena: A = filled(2);
        arena.set_all([item(1)].span());
    }

    #[test]
    fn test_dict_empty() {
        check_empty::<Arena<Item>>();
    }

    #[test]
    fn test_split_empty() {
        check_empty::<SplitArena<Item>>();
    }

    #[test]
    fn test_array_empty() {
        check_empty::<ArrayArena<Item>>();
    }

    #[test]
    fn test_dict_insert_get() {
        check_insert_get::<Arena<Item>>();
    }

    #[test]
    fn test_split_insert_get() {
        check_insert_get::<SplitArena<Item>>();
    }

    #[test]
    fn test_array_insert_get() {
        check_insert_get::<ArrayArena<Item>>();
    }

    #[test]
    fn test_dict_set_replace() {
        check_set_replace::<Arena<Item>>();
    }

    #[test]
    fn test_split_set_replace() {
        check_set_replace::<SplitArena<Item>>();
    }

    #[test]
    fn test_array_set_replace() {
        check_set_replace::<ArrayArena<Item>>();
    }

    #[test]
    fn test_dict_stale_handle() {
        check_stale_handle::<Arena<Item>>();
    }

    #[test]
    fn test_split_stale_handle() {
        check_stale_handle::<SplitArena<Item>>();
    }

    #[test]
    fn test_array_stale_handle() {
        check_stale_handle::<ArrayArena<Item>>();
    }

    #[test]
    fn test_dict_double_remove() {
        check_double_remove::<Arena<Item>>();
    }

    #[test]
    fn test_split_double_remove() {
        check_double_remove::<SplitArena<Item>>();
    }

    #[test]
    fn test_array_double_remove() {
        check_double_remove::<ArrayArena<Item>>();
    }

    #[test]
    fn test_dict_lifo_reuse() {
        check_lifo_reuse::<Arena<Item>>();
    }

    #[test]
    fn test_split_lifo_reuse() {
        check_lifo_reuse::<SplitArena<Item>>();
    }

    #[test]
    fn test_array_lifo_reuse() {
        check_lifo_reuse::<ArrayArena<Item>>();
    }

    #[test]
    fn test_dict_iteration_order() {
        check_iteration_order::<Arena<Item>>();
    }

    #[test]
    fn test_split_iteration_order() {
        check_iteration_order::<SplitArena<Item>>();
    }

    #[test]
    fn test_array_iteration_order() {
        check_iteration_order::<ArrayArena<Item>>();
    }

    #[test]
    fn test_dict_remove_all() {
        check_remove_all::<Arena<Item>>();
    }

    #[test]
    fn test_split_remove_all() {
        check_remove_all::<SplitArena<Item>>();
    }

    #[test]
    fn test_array_remove_all() {
        check_remove_all::<ArrayArena<Item>>();
    }

    #[test]
    fn test_dict_set_all() {
        check_set_all::<Arena<Item>>();
    }

    #[test]
    fn test_split_set_all() {
        check_set_all::<SplitArena<Item>>();
    }

    #[test]
    fn test_array_set_all() {
        check_set_all::<ArrayArena<Item>>();
    }

    #[test]
    #[should_panic(expected: 'Arena: length mismatch')]
    fn test_dict_set_all_length_mismatch() {
        check_set_all_length_mismatch::<Arena<Item>>();
    }

    #[test]
    #[should_panic(expected: 'Arena: length mismatch')]
    fn test_split_set_all_length_mismatch() {
        check_set_all_length_mismatch::<SplitArena<Item>>();
    }

    #[test]
    #[should_panic(expected: 'Arena: length mismatch')]
    fn test_array_set_all_length_mismatch() {
        check_set_all_length_mismatch::<ArrayArena<Item>>();
    }

    #[test]
    fn test_default_is_new() {
        let mut arena: Arena<Item> = Default::default();
        assert!(arena.is_empty());
        assert_eq!(arena.generation(), 0);
        assert_eq!(arena.insert(item(1)), h(0, 0));
    }

    // Persistence.

    #[test]
    fn test_state_of_empty_arena() {
        let mut arena: Arena<Item> = ArenaTrait::new();
        let state = arena.to_state();
        assert_eq!(state.generation, 0);
        assert_eq!(state.capacity, 0);
        assert_eq!(state.free_list, [].span());
        assert_eq!(state.entries, [].span());
        let mut restored = ArenaStateTrait::from_state(state);
        assert!(restored.is_empty());
        assert_eq!(restored.insert(item(1)), h(0, 0));
    }

    #[test]
    fn test_state_layout() {
        let mut arena: Arena<Item> = filled(5);
        assert!(arena.remove(h(1, 0)).is_some());
        assert!(arena.remove(h(3, 0)).is_some());
        let state = arena.to_state();
        assert_eq!(state.generation, 2);
        assert_eq!(state.capacity, 5);
        assert_eq!(state.free_list, [3, 1].span());
        assert_eq!(
            state.entries, [(h(0, 0), item(0)), (h(2, 0), item(20)), (h(4, 0), item(40))].span(),
        );
    }

    #[test]
    fn test_state_roundtrip_behaves_identically() {
        let mut arena: Arena<Item> = filled(6);
        assert!(arena.remove(h(4, 0)).is_some());
        assert!(arena.remove(h(0, 0)).is_some());
        assert_eq!(arena.insert(item(7)), h(0, 2));
        assert!(arena.remove(h(2, 0)).is_some());

        let mut restored = ArenaStateTrait::from_state(arena.to_state());
        assert_eq!(restored.len(), arena.len());
        assert_eq!(restored.capacity(), arena.capacity());
        assert_eq!(restored.generation(), arena.generation());
        assert_eq!(restored.to_array(), arena.to_array());
        assert_eq!(restored.get(h(0, 0)), Option::None);
        assert_eq!(restored.get(h(0, 2)), Option::Some(item(7)));

        // Same future: identical handles from both arenas, including free-list order.
        let mut i: u64 = 0;
        while i != 4 {
            assert_eq!(restored.insert(item(i)), arena.insert(item(i)));
            i += 1;
        }
        assert_eq!(restored.to_array(), arena.to_array());
    }

    #[test]
    fn test_state_serde_roundtrip() {
        let mut arena: Arena<Item> = filled(3);
        assert!(arena.remove(h(1, 0)).is_some());
        let mut serialized = array![];
        arena.to_state().serialize(ref serialized);
        let mut serialized = serialized.span();
        let state: ArenaState<Item> = Serde::deserialize(ref serialized).unwrap();
        let mut restored = ArenaStateTrait::from_state(state);
        assert_eq!(restored.to_array(), arena.to_array());
        assert_eq!(restored.insert(item(9)), h(1, 1));
    }

    fn state(
        generation: u32, capacity: u32, free_list: Span<u32>, entries: Span<(Handle, Item)>,
    ) -> ArenaState<Item> {
        ArenaState { generation, capacity, free_list, entries }
    }

    #[test]
    #[should_panic(expected: 'Arena: state entries unordered')]
    fn test_from_state_rejects_unordered_entries() {
        let entries = [(h(1, 0), item(1)), (h(0, 0), item(0))].span();
        ArenaStateTrait::from_state(state(0, 2, [].span(), entries));
    }

    #[test]
    #[should_panic(expected: 'Arena: state entries unordered')]
    fn test_from_state_rejects_duplicate_entries() {
        let entries = [(h(0, 0), item(1)), (h(0, 0), item(0))].span();
        ArenaStateTrait::from_state(state(0, 2, [].span(), entries));
    }

    #[test]
    #[should_panic(expected: 'Arena: state index too large')]
    fn test_from_state_rejects_entry_out_of_range() {
        let entries = [(h(2, 0), item(1))].span();
        ArenaStateTrait::from_state(state(0, 1, [].span(), entries));
    }

    #[test]
    #[should_panic(expected: 'Arena: state index too large')]
    fn test_from_state_rejects_free_slot_out_of_range() {
        ArenaStateTrait::<Item>::from_state(state(1, 1, [4].span(), [].span()));
    }

    #[test]
    #[should_panic(expected: 'Arena: state generation too new')]
    fn test_from_state_rejects_future_generation() {
        let entries = [(h(0, 3), item(1))].span();
        ArenaStateTrait::from_state(state(2, 1, [].span(), entries));
    }

    #[test]
    #[should_panic(expected: 'Arena: state slot listed twice')]
    fn test_from_state_rejects_free_slot_colliding_with_entry() {
        let entries = [(h(0, 0), item(1))].span();
        ArenaStateTrait::from_state(state(1, 2, [0].span(), entries));
    }

    #[test]
    #[should_panic(expected: 'Arena: state slot listed twice')]
    fn test_from_state_rejects_duplicate_free_slot() {
        ArenaStateTrait::<Item>::from_state(state(2, 2, [1, 1].span(), [].span()));
    }

    #[test]
    #[should_panic(expected: 'Arena: state misses slots')]
    fn test_from_state_rejects_missing_slots() {
        let entries = [(h(0, 0), item(1))].span();
        ArenaStateTrait::from_state(state(1, 3, [1].span(), entries));
    }

    // Equivalence of the candidates on pseudo-random operation sequences.

    const LCG_MUL: u128 = 6364136223846793005;
    const LCG_INC: u128 = 1442695040888963407;
    const NZ_TWO_POW_64: NonZero<u128> = 0x10000000000000000;
    const NZ_TWO_POW_32: NonZero<u128> = 0x100000000;

    /// Advances the generator and returns 32 well-mixed bits.
    fn next(ref state: u64) -> u32 {
        let (_, low) = DivRem::div_rem(state.into() * LCG_MUL + LCG_INC, NZ_TWO_POW_64);
        state = low.try_into().unwrap();
        let (high, _) = DivRem::div_rem(low, NZ_TWO_POW_32);
        high.try_into().unwrap()
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_candidates_equivalent(seed: u64) {
        let mut rng = seed;
        let mut dict: Arena<Item> = ArenaTrait::new();
        let mut split: SplitArena<Item> = ArenaTrait::new();
        let mut array: ArrayArena<Item> = ArenaTrait::new();
        // Every handle ever issued, so that stale ones keep being exercised.
        let mut issued: Array<Handle> = array![];
        let mut step: u64 = 0;
        while step != 48 {
            let op = next(ref rng) % 8;
            let value = item(step);
            if op < 3 || issued.len() == 0 {
                let handle = dict.insert(value);
                assert_eq!(split.insert(value), handle);
                assert_eq!(array.insert(value), handle);
                issued.append(handle);
            } else {
                let handle = *issued.at(next(ref rng) % issued.len());
                if op < 5 {
                    let removed = dict.remove(handle);
                    assert_eq!(split.remove(handle), removed);
                    assert_eq!(array.remove(handle), removed);
                } else if op == 5 {
                    let replaced = dict.replace(handle, value);
                    assert_eq!(split.replace(handle, value), replaced);
                    assert_eq!(array.replace(handle, value), replaced);
                } else if op == 6 {
                    let done = dict.set(handle, value);
                    assert_eq!(split.set(handle, value), done);
                    assert_eq!(array.set(handle, value), done);
                } else {
                    let found = dict.get(handle);
                    assert_eq!(split.get(handle), found);
                    assert_eq!(array.get(handle), found);
                    assert_eq!(dict.contains(handle), found.is_some());
                    assert_eq!(split.contains(handle), found.is_some());
                    assert_eq!(array.contains(handle), found.is_some());
                }
            }
            assert_eq!(split.len(), dict.len());
            assert_eq!(array.len(), dict.len());
            assert_eq!(split.capacity(), dict.capacity());
            assert_eq!(array.capacity(), dict.capacity());
            step += 1;
        }
        let entries = dict.to_array();
        assert_eq!(split.to_array(), entries.clone());
        assert_eq!(array.to_array(), entries.clone());
        let mut restored = ArenaStateTrait::from_state(dict.to_state());
        assert_eq!(restored.to_array(), entries);
        assert_eq!(restored.insert(item(0)), dict.insert(item(0)));
    }

    // Gas probes. Every `gas_<candidate>_<op>_<n>` probe first builds an arena of `n` live
    // items: subtract `gas_<candidate>_fill_<n>` to get the net cost of the operation on a
    // populated arena (dict squashing of the extra accesses included), and `gas_baseline` from
    // the `fill` probes themselves. `set_each` and `set_all` probes also build their input:
    // subtract `gas_values_<n>` too.

    /// Handle the single-operation probes act on (the cost does not depend on the index).
    const PROBED: Handle = Handle { index: 3, generation: 0 };

    #[inline(never)]
    fn values(n: u32) -> Span<Item> {
        let mut values = array![];
        let mut i: u32 = 0;
        while i != n {
            values.append(item(opaque(i.into() * 7)));
            i += 1;
        }
        values.span()
    }

    fn probe_fill<A, +ArenaTrait<A, Item>, +Destruct<A>>(n: u32) {
        let arena: A = filled(n);
        assert!(!arena.is_empty());
    }

    fn probe_get<A, +ArenaTrait<A, Item>, +Destruct<A>>(n: u32) {
        let mut arena: A = filled(n);
        assert!(arena.get(opaque(PROBED)).is_some());
    }

    fn probe_contains<A, +ArenaTrait<A, Item>, +Destruct<A>>(n: u32) {
        let mut arena: A = filled(n);
        assert!(arena.contains(opaque(PROBED)));
    }

    fn probe_set<A, +ArenaTrait<A, Item>, +Destruct<A>>(n: u32) {
        let mut arena: A = filled(n);
        assert!(arena.set(opaque(PROBED), item(opaque(5))));
    }

    fn probe_replace<A, +ArenaTrait<A, Item>, +Destruct<A>>(n: u32) {
        let mut arena: A = filled(n);
        assert!(arena.replace(opaque(PROBED), item(opaque(5))).is_some());
    }

    fn probe_remove<A, +ArenaTrait<A, Item>, +Destruct<A>>(n: u32) {
        let mut arena: A = filled(n);
        assert!(arena.remove(opaque(PROBED)).is_some());
    }

    /// Remove then insert into the freed slot: subtract the `remove` probe for the net cost of
    /// an insert that reuses a slot.
    fn probe_reinsert<A, +ArenaTrait<A, Item>, +Destruct<A>>(n: u32) {
        let mut arena: A = filled(n);
        assert!(arena.remove(opaque(PROBED)).is_some());
        assert!(arena.insert(item(opaque(5))).generation == 1);
    }

    fn probe_iter<A, +ArenaTrait<A, Item>, +Destruct<A>>(n: u32) {
        let mut arena: A = filled(n);
        assert!(arena.to_array().len() == n);
    }

    /// One `set` per live item: the write-back of a simulation step done handle by handle.
    fn probe_set_each<A, +ArenaTrait<A, Item>, +Destruct<A>>(n: u32) {
        let mut arena: A = filled(n);
        let mut values = values(n);
        let mut index: u32 = 0;
        while let Option::Some(value) = values.pop_front() {
            arena.set(Handle { index, generation: 0 }, *value);
            index += 1;
        }
        assert!(arena.len() == n);
    }

    /// The same write-back done in bulk.
    fn probe_set_all<A, +ArenaTrait<A, Item>, +Destruct<A>>(n: u32) {
        let mut arena: A = filled(n);
        arena.set_all(values(n));
        assert!(arena.len() == n);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_values_8() {
        assert!(values(8).len() == 8);
    }

    #[test]
    fn gas_values_32() {
        assert!(values(32).len() == 32);
    }

    #[test]
    fn gas_dict_fill_8() {
        probe_fill::<Arena<Item>>(8);
    }

    #[test]
    fn gas_dict_fill_32() {
        probe_fill::<Arena<Item>>(32);
    }

    #[test]
    fn gas_split_fill_8() {
        probe_fill::<SplitArena<Item>>(8);
    }

    #[test]
    fn gas_split_fill_32() {
        probe_fill::<SplitArena<Item>>(32);
    }

    #[test]
    fn gas_array_fill_8() {
        probe_fill::<ArrayArena<Item>>(8);
    }

    #[test]
    fn gas_array_fill_32() {
        probe_fill::<ArrayArena<Item>>(32);
    }

    #[test]
    fn gas_dict_get_8() {
        probe_get::<Arena<Item>>(8);
    }

    #[test]
    fn gas_dict_get_32() {
        probe_get::<Arena<Item>>(32);
    }

    #[test]
    fn gas_split_get_8() {
        probe_get::<SplitArena<Item>>(8);
    }

    #[test]
    fn gas_split_get_32() {
        probe_get::<SplitArena<Item>>(32);
    }

    #[test]
    fn gas_array_get_8() {
        probe_get::<ArrayArena<Item>>(8);
    }

    #[test]
    fn gas_array_get_32() {
        probe_get::<ArrayArena<Item>>(32);
    }

    #[test]
    fn gas_dict_contains_8() {
        probe_contains::<Arena<Item>>(8);
    }

    #[test]
    fn gas_dict_contains_32() {
        probe_contains::<Arena<Item>>(32);
    }

    #[test]
    fn gas_split_contains_8() {
        probe_contains::<SplitArena<Item>>(8);
    }

    #[test]
    fn gas_split_contains_32() {
        probe_contains::<SplitArena<Item>>(32);
    }

    #[test]
    fn gas_array_contains_8() {
        probe_contains::<ArrayArena<Item>>(8);
    }

    #[test]
    fn gas_array_contains_32() {
        probe_contains::<ArrayArena<Item>>(32);
    }

    #[test]
    fn gas_dict_set_8() {
        probe_set::<Arena<Item>>(8);
    }

    #[test]
    fn gas_dict_set_32() {
        probe_set::<Arena<Item>>(32);
    }

    #[test]
    fn gas_split_set_8() {
        probe_set::<SplitArena<Item>>(8);
    }

    #[test]
    fn gas_split_set_32() {
        probe_set::<SplitArena<Item>>(32);
    }

    #[test]
    fn gas_array_set_8() {
        probe_set::<ArrayArena<Item>>(8);
    }

    #[test]
    fn gas_array_set_32() {
        probe_set::<ArrayArena<Item>>(32);
    }

    #[test]
    fn gas_dict_replace_8() {
        probe_replace::<Arena<Item>>(8);
    }

    #[test]
    fn gas_dict_replace_32() {
        probe_replace::<Arena<Item>>(32);
    }

    #[test]
    fn gas_split_replace_8() {
        probe_replace::<SplitArena<Item>>(8);
    }

    #[test]
    fn gas_split_replace_32() {
        probe_replace::<SplitArena<Item>>(32);
    }

    #[test]
    fn gas_array_replace_8() {
        probe_replace::<ArrayArena<Item>>(8);
    }

    #[test]
    fn gas_array_replace_32() {
        probe_replace::<ArrayArena<Item>>(32);
    }

    #[test]
    fn gas_dict_remove_8() {
        probe_remove::<Arena<Item>>(8);
    }

    #[test]
    fn gas_dict_remove_32() {
        probe_remove::<Arena<Item>>(32);
    }

    #[test]
    fn gas_split_remove_8() {
        probe_remove::<SplitArena<Item>>(8);
    }

    #[test]
    fn gas_split_remove_32() {
        probe_remove::<SplitArena<Item>>(32);
    }

    #[test]
    fn gas_array_remove_8() {
        probe_remove::<ArrayArena<Item>>(8);
    }

    #[test]
    fn gas_array_remove_32() {
        probe_remove::<ArrayArena<Item>>(32);
    }

    #[test]
    fn gas_dict_reinsert_8() {
        probe_reinsert::<Arena<Item>>(8);
    }

    #[test]
    fn gas_dict_reinsert_32() {
        probe_reinsert::<Arena<Item>>(32);
    }

    #[test]
    fn gas_split_reinsert_8() {
        probe_reinsert::<SplitArena<Item>>(8);
    }

    #[test]
    fn gas_split_reinsert_32() {
        probe_reinsert::<SplitArena<Item>>(32);
    }

    #[test]
    fn gas_array_reinsert_8() {
        probe_reinsert::<ArrayArena<Item>>(8);
    }

    #[test]
    fn gas_array_reinsert_32() {
        probe_reinsert::<ArrayArena<Item>>(32);
    }

    #[test]
    fn gas_dict_iter_8() {
        probe_iter::<Arena<Item>>(8);
    }

    #[test]
    fn gas_dict_iter_32() {
        probe_iter::<Arena<Item>>(32);
    }

    #[test]
    fn gas_split_iter_8() {
        probe_iter::<SplitArena<Item>>(8);
    }

    #[test]
    fn gas_split_iter_32() {
        probe_iter::<SplitArena<Item>>(32);
    }

    #[test]
    fn gas_array_iter_8() {
        probe_iter::<ArrayArena<Item>>(8);
    }

    #[test]
    fn gas_array_iter_32() {
        probe_iter::<ArrayArena<Item>>(32);
    }

    #[test]
    fn gas_dict_set_each_8() {
        probe_set_each::<Arena<Item>>(8);
    }

    #[test]
    fn gas_dict_set_each_32() {
        probe_set_each::<Arena<Item>>(32);
    }

    #[test]
    fn gas_split_set_each_8() {
        probe_set_each::<SplitArena<Item>>(8);
    }

    #[test]
    fn gas_split_set_each_32() {
        probe_set_each::<SplitArena<Item>>(32);
    }

    #[test]
    fn gas_array_set_each_8() {
        probe_set_each::<ArrayArena<Item>>(8);
    }

    #[test]
    fn gas_array_set_each_32() {
        probe_set_each::<ArrayArena<Item>>(32);
    }

    #[test]
    fn gas_dict_set_all_8() {
        probe_set_all::<Arena<Item>>(8);
    }

    #[test]
    fn gas_dict_set_all_32() {
        probe_set_all::<Arena<Item>>(32);
    }

    #[test]
    fn gas_split_set_all_8() {
        probe_set_all::<SplitArena<Item>>(8);
    }

    #[test]
    fn gas_split_set_all_32() {
        probe_set_all::<SplitArena<Item>>(32);
    }

    #[test]
    fn gas_array_set_all_8() {
        probe_set_all::<ArrayArena<Item>>(8);
    }

    #[test]
    fn gas_array_set_all_32() {
        probe_set_all::<ArrayArena<Item>>(32);
    }

    // Remaining public functions of the shipped arena.

    #[test]
    fn gas_dict_new() {
        let arena: Arena<Item> = ArenaTrait::new();
        assert!(opaque(arena.len()) == 0);
    }

    #[test]
    fn gas_dict_len_is_empty_capacity_generation() {
        let arena: Arena<Item> = ArenaTrait::new();
        assert!(opaque(arena.len()) == 0);
        assert!(opaque(arena.is_empty()));
        assert!(opaque(arena.capacity()) == 0);
        assert!(opaque(arena.generation()) == 0);
    }

    #[test]
    fn gas_dict_to_state_8() {
        let mut arena: Arena<Item> = filled(8);
        assert!(arena.to_state().entries.len() == 8);
    }

    #[test]
    fn gas_dict_to_state_32() {
        let mut arena: Arena<Item> = filled(32);
        assert!(arena.to_state().entries.len() == 32);
    }

    /// Subtract `gas_dict_to_state_<n>` for the net cost of `from_state`.
    #[test]
    fn gas_dict_state_roundtrip_8() {
        let mut arena: Arena<Item> = filled(8);
        let restored = ArenaStateTrait::from_state(opaque(arena.to_state()));
        assert!(restored.len() == 8);
    }

    #[test]
    fn gas_dict_state_roundtrip_32() {
        let mut arena: Arena<Item> = filled(32);
        let restored = ArenaStateTrait::from_state(opaque(arena.to_state()));
        assert!(restored.len() == 32);
    }
}
