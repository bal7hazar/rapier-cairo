//! Unit tests of the arena: behaviour checked identically on every candidate, state persistence,
//! and equivalence of the candidates on pseudo-random operation sequences. Gas probes live in
//! `gas`.

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

#[cfg(test)]
mod gas;
