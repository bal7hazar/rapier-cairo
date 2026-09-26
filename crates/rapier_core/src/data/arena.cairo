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

/// Bit of [`Arena`]'s `stamp` above the 32-bit generation: set by every successful write.
const MODIFIED: u64 = 0x100000000;

/// The generation held in `stamp` (the stamp without its [`MODIFIED`] bit).
#[inline(always)]
fn generation_of(stamp: u64) -> u32 {
    let generation = if stamp >= MODIFIED {
        stamp - MODIFIED
    } else {
        stamp
    };
    // A stamp is a `u32` generation plus at most `MODIFIED`.
    generation.try_into().unwrap()
}

/// `stamp` with its [`MODIFIED`] bit set.
#[inline(always)]
fn marked(stamp: u64) -> u64 {
    if stamp >= MODIFIED {
        stamp
    } else {
        stamp + MODIFIED
    }
}

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
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
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
    /// Arena generation counter (number of successful removals), plus [`MODIFIED`] when the
    /// arena was written since [`ArenaStateTrait::clear_modified`] (BT4: the bit rides in this
    /// cold field, read by `insert` and `remove` only, rather than in a field of each set).
    stamp: u64,
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
        Arena { slots: Default::default(), stamp: 0, free_head: NO_SLOT, len: 0, capacity: 0 }
    }

    fn insert(ref self: Arena<T>, value: T) -> Handle {
        let generation = generation_of(self.stamp);
        self.stamp = generation.into() + MODIFIED;
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
        // Matched in place (BT4): `get(handle).is_some()` returns, hence copies, the value.
        match match_nullable(self.slots.get(handle.index.into())) {
            FromNullableResult::NotNull(slot) => match slot.unbox() {
                Slot::Occupied((generation, _)) => generation == handle.generation,
                Slot::Free(_) => false,
            },
            FromNullableResult::Null => false,
        }
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
                    self.stamp = marked(self.stamp);
                    return Option::Some(old);
                }
            }
        }
        self.slots = entry.finalize(previous);
        Option::None
    }

    fn set_all(ref self: Arena<T>, mut values: Span<T>) {
        assert(values.len() == self.len, errors::LENGTH_MISMATCH);
        self.stamp = marked(self.stamp);
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
                    let generation = generation_of(self.stamp)
                        .checked_add(1)
                        .expect(errors::GENERATION_OVERFLOW);
                    self.stamp = generation.into() + MODIFIED;
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

/// One component `F` of an arena value `T`, read by [`ArenaFieldTrait::get_field`] (BT4). The
/// projection is inlined into the read, so only the cells of the component are copied out of the
/// slot (an [`ArenaTrait::get`] returns, hence copies, the whole value).
pub trait ArenaField<T, F> {
    /// The component of `value`.
    fn read(value: T) -> F;
}

/// Component reads of an [`Arena`].
#[generate_trait]
pub impl ArenaFieldImpl<T, +Copy<T>, +Drop<T>> of ArenaFieldTrait<T> {
    /// The component `P` selects of the value behind `handle`, `None` when the handle is stale,
    /// was removed, or was never issued by this arena. One dict read.
    fn get_field<F, impl P: ArenaField<T, F>, +Drop<F>>(
        ref self: Arena<T>, handle: Handle,
    ) -> Option<F> {
        match match_nullable(self.slots.get(handle.index.into())) {
            FromNullableResult::NotNull(slot) => match slot.unbox() {
                Slot::Occupied((generation, value)) => if generation == handle.generation {
                    Option::Some(P::read(value))
                } else {
                    Option::None
                },
                Slot::Free(_) => Option::None,
            },
            FromNullableResult::Null => Option::None,
        }
    }
}

/// Persistence of an [`Arena`]: flatten to / rebuild from an [`ArenaState`].
#[generate_trait]
pub impl ArenaStateImpl<T, +Copy<T>, +Drop<T>> of ArenaStateTrait<T> {
    /// Returns the current generation counter (the generation the next insert is stamped with).
    #[inline(always)]
    fn generation(self: @Arena<T>) -> u32 {
        generation_of(*self.stamp)
    }

    /// Whether a value was inserted, overwritten (`set`, `replace`, `set_all`) or removed since
    /// [`clear_modified`](ArenaStateTrait::clear_modified) or since the arena was created or
    /// restored ([`from_state`](ArenaStateTrait::from_state)). Reads and failed writes (a handle
    /// that does not resolve) leave it unchanged.
    #[inline(always)]
    fn is_modified(self: @Arena<T>) -> bool {
        *self.stamp >= MODIFIED
    }

    /// Raises the [`is_modified`](ArenaStateTrait::is_modified) bit.
    #[inline(always)]
    fn mark_modified(ref self: Arena<T>) {
        self.stamp = marked(self.stamp);
    }

    /// [`ArenaTrait::set`] without raising the [`is_modified`](ArenaStateTrait::is_modified) bit
    /// (upstream `get_mut_internal`): for a caller that tracks its own writes.
    fn set_untracked(ref self: Arena<T>, handle: Handle, value: T) -> bool {
        let (entry, previous) = self.slots.entry(handle.index.into());
        if let FromNullableResult::NotNull(slot) = match_nullable(previous) {
            if let Slot::Occupied((generation, _)) = slot.unbox() {
                if generation == handle.generation {
                    self
                        .slots = entry
                        .finalize(NullableTrait::new(Slot::Occupied((generation, value))));
                    return true;
                }
            }
        }
        self.slots = entry.finalize(previous);
        false
    }

    /// Clears the [`is_modified`](ArenaStateTrait::is_modified) bit.
    #[inline(always)]
    fn clear_modified(ref self: Arena<T>) {
        let stamp = self.stamp;
        if stamp >= MODIFIED {
            self.stamp = stamp - MODIFIED;
        }
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
            generation: generation_of(self.stamp),
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

        Arena { slots, stamp: generation.into(), free_head, len, capacity }
    }
}

#[cfg(test)]
mod tests;
