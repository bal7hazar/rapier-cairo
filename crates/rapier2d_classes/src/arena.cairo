//! Sets rebuilt in a stage class from the entries that cross (CS5): the class runs the step's own
//! stage function on them, so that it computes what the caller's sets would.
//!
//! A set rebuilt at the caller's handles holds a free slot for every slot below the last handle
//! that did not cross (`partial_state`): a lone awake body at slot 30 costs 30 free-list entries,
//! checked and inserted by `from_state`. The classes of the per-step stages renumber the handles
//! instead ([`positions`], [`dense`]): the `n` entries that cross take slots `0..n` in their
//! order (ascending, as the caller's), generation 0, and every handle a value refers to is
//! renumbered the same way (a handle that did not cross becomes slot `n`, which no entry has). The
//! stages only compare, look up and order handles, so the renumbered world steps as the caller's.

use core::dict::{Felt252Dict, Felt252DictTrait};
use rapier_core::Handle;
use rapier_core::data::arena::ArenaState;

/// The renumbering of `entries`' handles: handle → its position plus one.
pub fn positions<T>(entries: Span<(Handle, T)>) -> Felt252Dict<u32> {
    let mut at: Felt252Dict<u32> = Default::default();
    let mut position: u32 = 0;
    for (handle, _) in entries {
        position += 1;
        at.insert((*handle).into(), position);
    }
    at
}

/// `handle` renumbered by `at` (of `len` entries): its position, or `len` when it did not cross.
pub fn dense(ref at: Felt252Dict<u32>, len: u32, handle: Handle) -> Handle {
    let position = at.get(handle.into());
    if position == 0 {
        Handle { index: len, generation: 0 }
    } else {
        Handle { index: position - 1, generation: 0 }
    }
}

/// `handle` renumbered by `at`, when there is one.
pub fn dense_option(ref at: Felt252Dict<u32>, len: u32, handle: Option<Handle>) -> Option<Handle> {
    match handle {
        Some(h) => Some(dense(ref at, len, h)),
        None => None,
    }
}

/// The flat image of a set holding exactly `entries` (strictly ascending slot) at their handles:
/// every other slot below the last one is free, the generation is the largest of the handles'.
/// `from_state` of it resolves each handle of `entries` to its value, and no other.
pub fn partial_state<T, +Copy<T>, +Drop<T>>(entries: Span<(Handle, T)>) -> ArenaState<T> {
    let mut free_list = array![];
    let mut next: u32 = 0;
    let mut generation: u32 = 0;
    for (handle, _) in entries {
        while next != *handle.index {
            free_list.append(next);
            next += 1;
        }
        next += 1;
        if *handle.generation > generation {
            generation = *handle.generation;
        }
    }
    ArenaState { generation, capacity: next, free_list: free_list.span(), entries }
}

/// `entries` in ascending slot (insertion sort: a body's colliders, a handful).
pub fn ascending<T, +Copy<T>, +Drop<T>>(entries: Span<(Handle, T)>) -> Span<(Handle, T)> {
    let mut sorted: Array<(Handle, T)> = array![];
    for entry in entries {
        let (handle, _) = *entry;
        let mut out = array![];
        let mut placed = false;
        for other in sorted.span() {
            let (h, _) = *other;
            if !placed && h.index > handle.index {
                out.append(*entry);
                placed = true;
            }
            out.append(*other);
        }
        if !placed {
            out.append(*entry);
        }
        sorted = out;
    }
    sorted.span()
}

#[cfg(test)]
mod tests {
    use rapier_core::Handle;
    use rapier_core::data::arena::{Arena, ArenaStateTrait, ArenaTrait};
    use super::{ascending, partial_state};

    fn h(index: u32, generation: u32) -> Handle {
        Handle { index, generation }
    }

    #[test]
    fn test_partial_state_resolves_exactly_the_entries() {
        let entries = array![(h(1, 0), 10_u32), (h(4, 2), 40), (h(5, 1), 50)];
        let mut arena: Arena<u32> = ArenaStateTrait::from_state(partial_state(entries.span()));
        assert_eq!(arena.get(h(1, 0)), Some(10));
        assert_eq!(arena.get(h(4, 2)), Some(40));
        assert_eq!(arena.get(h(5, 1)), Some(50));
        assert_eq!(arena.get(h(0, 0)), None);
        assert_eq!(arena.get(h(4, 1)), None);
        assert_eq!(arena.len(), 3);
    }

    #[test]
    fn test_ascending() {
        let entries = array![(h(7, 0), 7_u32), (h(2, 0), 2), (h(9, 0), 9), (h(0, 0), 0)];
        let sorted = ascending(entries.span());
        let mut indices = array![];
        for (handle, value) in sorted {
            assert_eq!(handle.index, value);
            indices.append(*handle.index);
        }
        assert_eq!(indices, array![0, 2, 7, 9]);
    }
}
