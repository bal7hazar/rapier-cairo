//! Gas probes of the arena candidates.

use rapier_testing::opaque;
use crate::data::handle::Handle;
use super::super::alternatives::{ArrayArena, SplitArena};
use super::super::{Arena, ArenaField, ArenaFieldTrait, ArenaState, ArenaStateTrait, ArenaTrait};
use super::{Item, filled, item};

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

// BT4: component read, untracked write and the modified bit (subtract `gas_dict_fill_8`).

/// The component `c` of an [`Item`].
impl ItemC of ArenaField<Item, u64> {
    #[inline(always)]
    fn read(value: Item) -> u64 {
        value.c
    }
}

/// Compare with `gas_dict_get_8`: one `u64` copied out instead of the item.
#[test]
fn gas_dict_get_field_8() {
    let mut arena: Arena<Item> = filled(8);
    assert!(arena.get_field::<u64, ItemC>(opaque(PROBED)).is_some());
}

/// Compare with `gas_dict_set_8`.
#[test]
fn gas_dict_set_untracked_8() {
    let mut arena: Arena<Item> = filled(8);
    assert!(arena.set_untracked(opaque(PROBED), item(opaque(5))));
}

/// `is_modified`, `clear_modified`, `mark_modified`, `is_modified`.
#[test]
fn gas_dict_modified_8() {
    let mut arena: Arena<Item> = filled(8);
    assert!(opaque(arena.is_modified()));
    arena.clear_modified();
    arena.mark_modified();
    assert!(opaque(arena.is_modified()));
}
