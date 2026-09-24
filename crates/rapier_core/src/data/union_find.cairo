//! Disjoint-set forest over `u32` ids (upstream `rapier::data::UnionFind`), used to group bodies
//! into simulation islands.
//!
//! **Determinism contract** (deviation from upstream): the representative of a set is its
//! **smallest id**. It therefore depends only on the partition, never on the order or the
//! orientation of the `union` calls that built it. Upstream picks the root of the larger set
//! (first argument on ties), which is order dependent; islands built from a contact list would
//! inherit the ordering of that list.
//!
//! Storage is sparse: a `Felt252Dict` keyed by `id + 1` holding the parent's `id + 1`, where the
//! default `0` means "root". A fresh structure therefore already contains every `u32` id as a
//! singleton: there is no `reset(len)`, and ids do not need to be dense.
//!
//! Candidates (all behind [`UnionFindTrait`], ranked by the `gas_*` probes of this module):
//!
//! 1. [`UnionFind`] — link the larger root under the smaller one, path halving. **Winner.**
//! 2. `alternatives::CompressionUnionFind` — same linking, full two-pass path compression.
//! 3. `alternatives::PlainUnionFind` — same linking, no compression at all.
//! 4. `alternatives::SizeUnionFind` — upstream's union by size + path halving, with a side
//!    table giving the smallest id of each root's set.
//! 5. `alternatives::RankUnionFind` — union by rank + path halving, same side table.

use core::dict::{Felt252Dict, Felt252DictTrait};

/// Panic messages of the union-find.
pub mod errors {
    /// Internal invariant broken: a stored parent is not a valid id.
    pub const CORRUPT_FOREST: felt252 = 'UnionFind: corrupt forest';
}

/// Operations of a disjoint-set structure stored in the container `U`.
///
/// All ids in `0..=0xffffffff` exist from the start, each alone in its set.
pub trait UnionFindTrait<U> {
    /// Returns the partition where every id is a singleton. O(1).
    fn new() -> U;

    /// Returns the representative of the set of `id`: the smallest id of that set.
    /// May shorten internal paths, hence `ref self`.
    fn find(ref self: U, id: u32) -> u32;

    /// Merges the sets of `a` and `b` and returns the representative of the merged set.
    /// Idempotent and symmetric: `union(a, b)`, `union(b, a)` and repeating either leave the
    /// same partition and return the same value.
    fn union(ref self: U, a: u32, b: u32) -> u32;

    /// Returns `true` when `a` and `b` are in the same set.
    fn connected(ref self: U, a: u32, b: u32) -> bool;
}

/// Disjoint-set forest: smaller root wins, path halving.
///
/// Holds a dict, hence `Destruct` instead of `Drop`: pass it by `ref`.
#[derive(Destruct)]
pub struct UnionFind {
    /// `id + 1` → parent's `id + 1`, `0` for roots.
    parents: Felt252Dict<felt252>,
}

/// Walks from `code` to its root with path halving and returns the root's code.
pub(crate) fn root_halving(ref parents: Felt252Dict<felt252>, mut code: felt252) -> felt252 {
    loop {
        let parent = parents.get(code);
        if parent == 0 {
            break code;
        }
        let grandparent = parents.get(parent);
        if grandparent == 0 {
            break parent;
        }
        parents.insert(code, grandparent);
        code = grandparent;
    }
}

/// Dictionary code of an id.
#[inline(always)]
pub(crate) fn encode(id: u32) -> felt252 {
    id.into() + 1
}

/// Id of a dictionary code.
#[inline(always)]
pub(crate) fn decode(code: felt252) -> u32 {
    (code - 1).try_into().expect(errors::CORRUPT_FOREST)
}

/// The default partition is the all-singletons one.
pub impl UnionFindDefault of Default<UnionFind> {
    fn default() -> UnionFind {
        UnionFindTrait::new()
    }
}

/// [`UnionFindTrait`] for the shipped [`UnionFind`] (smaller root wins, path halving).
pub impl UnionFindImpl of UnionFindTrait<UnionFind> {
    fn new() -> UnionFind {
        UnionFind { parents: Default::default() }
    }

    fn find(ref self: UnionFind, id: u32) -> u32 {
        decode(root_halving(ref self.parents, encode(id)))
    }

    fn union(ref self: UnionFind, a: u32, b: u32) -> u32 {
        let root_a = self.find(a);
        let root_b = self.find(b);
        if root_a == root_b {
            root_a
        } else if root_a < root_b {
            self.parents.insert(encode(root_b), encode(root_a));
            root_a
        } else {
            self.parents.insert(encode(root_a), encode(root_b));
            root_b
        }
    }

    fn connected(ref self: UnionFind, a: u32, b: u32) -> bool {
        self.find(a) == self.find(b)
    }
}

#[cfg(test)]
mod alternatives;

#[cfg(test)]
mod tests;
