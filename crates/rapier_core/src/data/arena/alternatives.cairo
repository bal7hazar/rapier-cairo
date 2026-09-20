//! Rejected arena candidates, kept compiled (tests only) so the ranking can be re-measured on
//! every toolchain bump. Both implement `ArenaTrait` with the exact semantics of `Arena`.

use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
use core::nullable::{FromNullableResult, NullableTrait, match_nullable};
use core::num::traits::CheckedAdd;
use super::super::handle::Handle;
use super::{ArenaTrait, NO_SLOT, Slot, errors};

/// `SplitArena` metadata of a free slot is `FREE_TAG + next`, of a live slot `generation + 1`,
/// of a never-allocated slot `0`.
const FREE_TAG: u64 = 0x100000001;

/// Candidate: values in a `Felt252Dict<Nullable<T>>`, generations and free-list links in a
/// separate `Felt252Dict<u64>`. `contains` never touches the boxed value, every other
/// operation pays two dict accesses.
pub struct SplitArena<T> {
    values: Felt252Dict<Nullable<T>>,
    meta: Felt252Dict<u64>,
    generation: u32,
    free_head: u32,
    len: u32,
    capacity: u32,
}

impl SplitArenaDestruct<T, +Drop<T>> of Destruct<SplitArena<T>> {
    fn destruct(self: SplitArena<T>) nopanic {
        let SplitArena { values, meta, .. } = self;
        values.squash();
        meta.squash();
    }
}

#[inline(always)]
fn live_meta(generation: u32) -> u64 {
    generation.into() + 1
}

pub impl SplitArenaImpl<T, +Copy<T>, +Drop<T>> of ArenaTrait<SplitArena<T>, T> {
    fn new() -> SplitArena<T> {
        SplitArena {
            values: Default::default(),
            meta: Default::default(),
            generation: 0,
            free_head: NO_SLOT,
            len: 0,
            capacity: 0,
        }
    }

    fn insert(ref self: SplitArena<T>, value: T) -> Handle {
        let generation = self.generation;
        let index = self.free_head;
        let index = if index == NO_SLOT {
            let index = self.capacity;
            assert(index != NO_SLOT, errors::CAPACITY_OVERFLOW);
            self.capacity = index + 1;
            self.meta.insert(index.into(), live_meta(generation));
            index
        } else {
            let (entry, previous) = self.meta.entry(index.into());
            self.meta = entry.finalize(live_meta(generation));
            assert(previous >= FREE_TAG, errors::CORRUPT_FREE_LIST);
            self.free_head = (previous - FREE_TAG).try_into().expect(errors::CORRUPT_FREE_LIST);
            index
        };
        self.values.insert(index.into(), NullableTrait::new(value));
        self.len += 1;
        Handle { index, generation }
    }

    fn get(ref self: SplitArena<T>, handle: Handle) -> Option<T> {
        if self.meta.get(handle.index.into()) != live_meta(handle.generation) {
            return Option::None;
        }
        match match_nullable(self.values.get(handle.index.into())) {
            FromNullableResult::NotNull(value) => Option::Some(value.unbox()),
            FromNullableResult::Null => Option::None,
        }
    }

    fn contains(ref self: SplitArena<T>, handle: Handle) -> bool {
        self.meta.get(handle.index.into()) == live_meta(handle.generation)
    }

    fn set(ref self: SplitArena<T>, handle: Handle, value: T) -> bool {
        if self.meta.get(handle.index.into()) != live_meta(handle.generation) {
            return false;
        }
        self.values.insert(handle.index.into(), NullableTrait::new(value));
        true
    }

    fn replace(ref self: SplitArena<T>, handle: Handle, value: T) -> Option<T> {
        if self.meta.get(handle.index.into()) != live_meta(handle.generation) {
            return Option::None;
        }
        let (entry, previous) = self.values.entry(handle.index.into());
        self.values = entry.finalize(NullableTrait::new(value));
        match match_nullable(previous) {
            FromNullableResult::NotNull(previous) => Option::Some(previous.unbox()),
            FromNullableResult::Null => Option::None,
        }
    }

    fn set_all(ref self: SplitArena<T>, mut values: Span<T>) {
        assert(values.len() == self.len, errors::LENGTH_MISMATCH);
        let mut index: u32 = 0;
        while let Option::Some(value) = values.pop_front() {
            loop {
                let meta = self.meta.get(index.into());
                index += 1;
                if meta != 0 && meta < FREE_TAG {
                    break;
                }
            }
            self.values.insert((index - 1).into(), NullableTrait::new(*value));
        }
    }

    fn remove(ref self: SplitArena<T>, handle: Handle) -> Option<T> {
        let (entry, previous) = self.meta.entry(handle.index.into());
        if previous != live_meta(handle.generation) {
            self.meta = entry.finalize(previous);
            return Option::None;
        }
        self.meta = entry.finalize(FREE_TAG + self.free_head.into());
        self.free_head = handle.index;
        self.len -= 1;
        self.generation = self.generation.checked_add(1).expect(errors::GENERATION_OVERFLOW);
        let (entry, previous) = self.values.entry(handle.index.into());
        self.values = entry.finalize(Default::default());
        match match_nullable(previous) {
            FromNullableResult::NotNull(previous) => Option::Some(previous.unbox()),
            FromNullableResult::Null => Option::None,
        }
    }

    fn len(self: @SplitArena<T>) -> u32 {
        *self.len
    }

    fn is_empty(self: @SplitArena<T>) -> bool {
        *self.len == 0
    }

    fn capacity(self: @SplitArena<T>) -> u32 {
        *self.capacity
    }

    fn to_array(ref self: SplitArena<T>) -> Array<(Handle, T)> {
        let mut entries = array![];
        let capacity = self.capacity;
        let mut index = 0;
        while index != capacity {
            let meta = self.meta.get(index.into());
            if meta != 0 && meta < FREE_TAG {
                if let FromNullableResult::NotNull(value) =
                    match_nullable(self.values.get(index.into())) {
                    let generation = (meta - 1).try_into().expect(errors::CORRUPT_FREE_LIST);
                    entries.append((Handle { index, generation }, value.unbox()));
                }
            }
            index += 1;
        }
        entries
    }
}

/// Candidate: slots in an append-only `Array`. Reads and fresh inserts are O(1) and avoid the
/// dict entirely; any mutation of an existing slot rebuilds the whole array.
#[derive(Drop)]
pub struct ArrayArena<T> {
    slots: Array<Slot<T>>,
    generation: u32,
    free_head: u32,
    len: u32,
}

#[generate_trait]
impl ArrayArenaInternalImpl<T, +Copy<T>, +Drop<T>> of ArrayArenaInternalTrait<T> {
    /// Returns the live value behind `handle`.
    fn lookup(self: @ArrayArena<T>, handle: Handle) -> Option<T> {
        match self.slots.get(handle.index) {
            Option::Some(slot) => match *slot.unbox() {
                Slot::Occupied((generation, value)) => if generation == handle.generation {
                    Option::Some(value)
                } else {
                    Option::None
                },
                Slot::Free(_) => Option::None,
            },
            Option::None => Option::None,
        }
    }

    /// Rebuilds the slot array with `slot` at `index` (`index < slots.len()`).
    fn rebuild_with(ref self: ArrayArena<T>, index: u32, slot: Slot<T>) {
        let slots = self.slots.span();
        let mut head = slots.slice(0, index);
        let mut tail = slots.slice(index + 1, slots.len() - index - 1);
        let mut rebuilt = array![];
        while let Option::Some(kept) = head.pop_front() {
            rebuilt.append(*kept);
        }
        rebuilt.append(slot);
        while let Option::Some(kept) = tail.pop_front() {
            rebuilt.append(*kept);
        }
        self.slots = rebuilt;
    }
}

pub impl ArrayArenaImpl<T, +Copy<T>, +Drop<T>> of ArenaTrait<ArrayArena<T>, T> {
    fn new() -> ArrayArena<T> {
        ArrayArena { slots: array![], generation: 0, free_head: NO_SLOT, len: 0 }
    }

    fn insert(ref self: ArrayArena<T>, value: T) -> Handle {
        let generation = self.generation;
        let slot = Slot::Occupied((generation, value));
        let index = self.free_head;
        let index = if index == NO_SLOT {
            let index = self.slots.len();
            assert(index != NO_SLOT, errors::CAPACITY_OVERFLOW);
            self.slots.append(slot);
            index
        } else {
            self.free_head = match *self.slots.at(index) {
                Slot::Free(next) => next,
                Slot::Occupied(_) => core::panic_with_felt252(errors::CORRUPT_FREE_LIST),
            };
            self.rebuild_with(index, slot);
            index
        };
        self.len += 1;
        Handle { index, generation }
    }

    fn get(ref self: ArrayArena<T>, handle: Handle) -> Option<T> {
        self.lookup(handle)
    }

    fn contains(ref self: ArrayArena<T>, handle: Handle) -> bool {
        self.lookup(handle).is_some()
    }

    fn set(ref self: ArrayArena<T>, handle: Handle, value: T) -> bool {
        self.replace(handle, value).is_some()
    }

    fn replace(ref self: ArrayArena<T>, handle: Handle, value: T) -> Option<T> {
        let previous = self.lookup(handle);
        if previous.is_some() {
            self.rebuild_with(handle.index, Slot::Occupied((handle.generation, value)));
        }
        previous
    }

    fn set_all(ref self: ArrayArena<T>, mut values: Span<T>) {
        assert(values.len() == self.len, errors::LENGTH_MISMATCH);
        let mut slots = self.slots.span();
        let mut rebuilt = array![];
        while let Option::Some(slot) = slots.pop_front() {
            match *slot {
                Slot::Occupied((
                    generation, _,
                )) => {
                    // `values.len() == len` live slots: the pop cannot fail.
                    if let Option::Some(value) = values.pop_front() {
                        rebuilt.append(Slot::Occupied((generation, *value)));
                    }
                },
                Slot::Free(next) => rebuilt.append(Slot::Free(next)),
            }
        }
        self.slots = rebuilt;
    }

    fn remove(ref self: ArrayArena<T>, handle: Handle) -> Option<T> {
        let previous = self.lookup(handle);
        if previous.is_some() {
            self.rebuild_with(handle.index, Slot::Free(self.free_head));
            self.free_head = handle.index;
            self.len -= 1;
            self.generation = self.generation.checked_add(1).expect(errors::GENERATION_OVERFLOW);
        }
        previous
    }

    fn len(self: @ArrayArena<T>) -> u32 {
        *self.len
    }

    fn is_empty(self: @ArrayArena<T>) -> bool {
        *self.len == 0
    }

    fn capacity(self: @ArrayArena<T>) -> u32 {
        self.slots.len()
    }

    fn to_array(ref self: ArrayArena<T>) -> Array<(Handle, T)> {
        let mut entries = array![];
        let mut slots = self.slots.span();
        let mut index = 0;
        while let Option::Some(slot) = slots.pop_front() {
            if let Slot::Occupied((generation, value)) = *slot {
                entries.append((Handle { index, generation }, value));
            }
            index += 1;
        }
        entries
    }
}
