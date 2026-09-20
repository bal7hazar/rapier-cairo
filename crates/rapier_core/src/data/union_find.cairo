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
mod alternatives {
    use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
    use super::{UnionFindTrait, decode, encode, root_halving};

    /// Links `root_b` and `root_a` (distinct roots) smaller-wins; returns the winner.
    fn link_min(ref parents: Felt252Dict<felt252>, root_a: u32, root_b: u32) -> u32 {
        if root_a < root_b {
            parents.insert(encode(root_b), encode(root_a));
            root_a
        } else {
            parents.insert(encode(root_a), encode(root_b));
            root_b
        }
    }

    /// Candidate: smaller root wins, full path compression in two passes (the second pass is
    /// one `entry` access per node and is skipped for paths of length < 2).
    #[derive(Destruct)]
    pub struct CompressionUnionFind {
        parents: Felt252Dict<felt252>,
    }

    pub impl CompressionUnionFindImpl of UnionFindTrait<CompressionUnionFind> {
        fn new() -> CompressionUnionFind {
            CompressionUnionFind { parents: Default::default() }
        }

        fn find(ref self: CompressionUnionFind, id: u32) -> u32 {
            let start = encode(id);
            let mut root = start;
            let mut hops: u32 = 0;
            loop {
                let parent = self.parents.get(root);
                if parent == 0 {
                    break;
                }
                root = parent;
                hops += 1;
            }
            if hops > 1 {
                let mut code = start;
                // The last node before the root already points to it.
                while hops != 1 {
                    let (entry, parent) = self.parents.entry(code);
                    self.parents = entry.finalize(root);
                    code = parent;
                    hops -= 1;
                }
            }
            decode(root)
        }

        fn union(ref self: CompressionUnionFind, a: u32, b: u32) -> u32 {
            let root_a = self.find(a);
            let root_b = self.find(b);
            if root_a == root_b {
                return root_a;
            }
            link_min(ref self.parents, root_a, root_b)
        }

        fn connected(ref self: CompressionUnionFind, a: u32, b: u32) -> bool {
            self.find(a) == self.find(b)
        }
    }

    /// Candidate: smaller root wins, paths are never shortened (reads only in `find`).
    #[derive(Destruct)]
    pub struct PlainUnionFind {
        parents: Felt252Dict<felt252>,
    }

    pub impl PlainUnionFindImpl of UnionFindTrait<PlainUnionFind> {
        fn new() -> PlainUnionFind {
            PlainUnionFind { parents: Default::default() }
        }

        fn find(ref self: PlainUnionFind, id: u32) -> u32 {
            let mut root = encode(id);
            loop {
                let parent = self.parents.get(root);
                if parent == 0 {
                    break;
                }
                root = parent;
            }
            decode(root)
        }

        fn union(ref self: PlainUnionFind, a: u32, b: u32) -> u32 {
            let root_a = self.find(a);
            let root_b = self.find(b);
            if root_a == root_b {
                return root_a;
            }
            link_min(ref self.parents, root_a, root_b)
        }

        fn connected(ref self: PlainUnionFind, a: u32, b: u32) -> bool {
            self.find(a) == self.find(b)
        }
    }

    /// Candidate: upstream's union by size with path halving. The root is no longer the
    /// smallest id, so `lead` stores `root - smallest id` for every root (default `0`).
    #[derive(Destruct)]
    pub struct SizeUnionFind {
        parents: Felt252Dict<felt252>,
        /// Root code → set size minus one.
        sizes: Felt252Dict<u32>,
        /// Root code → root id minus smallest id of the set.
        lead: Felt252Dict<u32>,
    }

    pub impl SizeUnionFindImpl of UnionFindTrait<SizeUnionFind> {
        fn new() -> SizeUnionFind {
            SizeUnionFind {
                parents: Default::default(), sizes: Default::default(), lead: Default::default(),
            }
        }

        fn find(ref self: SizeUnionFind, id: u32) -> u32 {
            let root = root_halving(ref self.parents, encode(id));
            decode(root) - self.lead.get(root)
        }

        fn union(ref self: SizeUnionFind, a: u32, b: u32) -> u32 {
            let code_a = root_halving(ref self.parents, encode(a));
            let code_b = root_halving(ref self.parents, encode(b));
            let min_a = decode(code_a) - self.lead.get(code_a);
            if code_a == code_b {
                return min_a;
            }
            let min_b = decode(code_b) - self.lead.get(code_b);
            let min = if min_a < min_b {
                min_a
            } else {
                min_b
            };
            let size_a = self.sizes.get(code_a);
            let size_b = self.sizes.get(code_b);
            // Ties attach `b` under `a`, as upstream.
            let (big, small) = if size_a >= size_b {
                (code_a, code_b)
            } else {
                (code_b, code_a)
            };
            self.parents.insert(small, big);
            self.sizes.insert(big, size_a + size_b + 1);
            self.lead.insert(big, decode(big) - min);
            min
        }

        fn connected(ref self: SizeUnionFind, a: u32, b: u32) -> bool {
            root_halving(ref self.parents, encode(a)) == root_halving(ref self.parents, encode(b))
        }
    }

    /// Candidate: union by rank with path halving, same `lead` side table as `SizeUnionFind`.
    #[derive(Destruct)]
    pub struct RankUnionFind {
        parents: Felt252Dict<felt252>,
        /// Root code → rank.
        ranks: Felt252Dict<u32>,
        /// Root code → root id minus smallest id of the set.
        lead: Felt252Dict<u32>,
    }

    pub impl RankUnionFindImpl of UnionFindTrait<RankUnionFind> {
        fn new() -> RankUnionFind {
            RankUnionFind {
                parents: Default::default(), ranks: Default::default(), lead: Default::default(),
            }
        }

        fn find(ref self: RankUnionFind, id: u32) -> u32 {
            let root = root_halving(ref self.parents, encode(id));
            decode(root) - self.lead.get(root)
        }

        fn union(ref self: RankUnionFind, a: u32, b: u32) -> u32 {
            let code_a = root_halving(ref self.parents, encode(a));
            let code_b = root_halving(ref self.parents, encode(b));
            let min_a = decode(code_a) - self.lead.get(code_a);
            if code_a == code_b {
                return min_a;
            }
            let min_b = decode(code_b) - self.lead.get(code_b);
            let min = if min_a < min_b {
                min_a
            } else {
                min_b
            };
            let rank_a = self.ranks.get(code_a);
            let rank_b = self.ranks.get(code_b);
            let (big, small) = if rank_a >= rank_b {
                (code_a, code_b)
            } else {
                (code_b, code_a)
            };
            self.parents.insert(small, big);
            if rank_a == rank_b {
                self.ranks.insert(big, rank_a + 1);
            }
            self.lead.insert(big, decode(big) - min);
            min
        }

        fn connected(ref self: RankUnionFind, a: u32, b: u32) -> bool {
            root_halving(ref self.parents, encode(a)) == root_halving(ref self.parents, encode(b))
        }
    }
}

#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::alternatives::{CompressionUnionFind, PlainUnionFind, RankUnionFind, SizeUnionFind};
    use super::{UnionFind, UnionFindTrait};

    const MAX: u32 = 0xffffffff;

    // Behaviour, checked identically on every candidate.

    fn check_singletons<U, +UnionFindTrait<U>, +Destruct<U>>() {
        let mut sets: U = UnionFindTrait::new();
        assert_eq!(sets.find(0), 0);
        assert_eq!(sets.find(7), 7);
        assert_eq!(sets.find(MAX), MAX);
        assert!(sets.connected(3, 3));
        assert!(!sets.connected(0, 1));
        assert!(!sets.connected(0, MAX));
    }

    fn check_smallest_id_wins<U, +UnionFindTrait<U>, +Destruct<U>>() {
        let mut sets: U = UnionFindTrait::new();
        assert_eq!(sets.union(5, 9), 5);
        assert_eq!(sets.union(9, 2), 2);
        assert_eq!(sets.union(7, 5), 2);
        assert_eq!(sets.find(9), 2);
        assert_eq!(sets.find(7), 2);
        assert_eq!(sets.find(5), 2);
        assert_eq!(sets.find(2), 2);
        // Extremes of the id range.
        assert_eq!(sets.union(MAX, MAX - 1), MAX - 1);
        assert_eq!(sets.union(MAX, 0), 0);
        assert_eq!(sets.find(MAX - 1), 0);
        assert!(!sets.connected(0, 2));
    }

    fn check_idempotent_and_symmetric<U, +UnionFindTrait<U>, +Destruct<U>>() {
        let mut sets: U = UnionFindTrait::new();
        assert_eq!(sets.union(4, 4), 4);
        assert_eq!(sets.find(4), 4);
        assert_eq!(sets.union(1, 3), 1);
        assert_eq!(sets.union(1, 3), 1);
        assert_eq!(sets.union(3, 1), 1);
        assert_eq!(sets.union(3, 3), 1);
        assert_eq!(sets.find(1), 1);
        assert_eq!(sets.find(3), 1);
        assert!(!sets.connected(1, 2));
        assert!(!sets.connected(3, 4));
    }

    fn check_transitive<U, +UnionFindTrait<U>, +Destruct<U>>() {
        let mut sets: U = UnionFindTrait::new();
        sets.union(0, 1);
        sets.union(2, 3);
        assert!(!sets.connected(0, 3));
        sets.union(1, 2);
        assert!(sets.connected(0, 3));
        assert!(sets.connected(3, 0));
        assert!(sets.connected(1, 3));
        assert!(!sets.connected(0, 4));
        assert!(!sets.connected(4, 5));
        assert_eq!(sets.find(3), 0);
    }

    /// Same partition built in different orders and orientations: same representatives.
    fn check_order_independent<U, +UnionFindTrait<U>, +Destruct<U>>() {
        let mut forward: U = UnionFindTrait::new();
        forward.union(1, 2);
        forward.union(2, 3);
        forward.union(3, 4);
        forward.union(10, 11);
        let mut backward: U = UnionFindTrait::new();
        backward.union(11, 10);
        backward.union(4, 3);
        backward.union(3, 2);
        backward.union(2, 1);
        let mut id = 0;
        while id != 12 {
            assert_eq!(forward.find(id), backward.find(id));
            id += 1;
        }
        assert_eq!(forward.find(4), 1);
        assert_eq!(forward.find(11), 10);
    }

    /// A long chain stays correct while `find` rewrites it.
    fn check_long_chain<U, +UnionFindTrait<U>, +Destruct<U>>() {
        let mut sets: U = UnionFindTrait::new();
        let mut id = 40;
        while id != 0 {
            sets.union(id - 1, id);
            id -= 1;
        }
        assert_eq!(sets.find(40), 0);
        assert_eq!(sets.find(40), 0);
        assert_eq!(sets.find(17), 0);
        assert_eq!(sets.find(0), 0);
        assert!(!sets.connected(40, 41));
    }

    fn check_all<U, +UnionFindTrait<U>, +Destruct<U>>() {
        check_singletons::<U>();
        check_smallest_id_wins::<U>();
        check_idempotent_and_symmetric::<U>();
        check_transitive::<U>();
        check_order_independent::<U>();
        check_long_chain::<U>();
    }

    #[test]
    fn test_halving() {
        check_all::<UnionFind>();
    }

    #[test]
    fn test_compression() {
        check_all::<CompressionUnionFind>();
    }

    #[test]
    fn test_plain() {
        check_all::<PlainUnionFind>();
    }

    #[test]
    fn test_size() {
        check_all::<SizeUnionFind>();
    }

    #[test]
    fn test_rank() {
        check_all::<RankUnionFind>();
    }

    #[test]
    fn test_default_is_new() {
        let mut sets: UnionFind = Default::default();
        assert_eq!(sets.find(12), 12);
    }

    /// Port of upstream's `union_find_components` test.
    #[test]
    fn test_upstream_components() {
        let mut sets: UnionFind = UnionFindTrait::new();
        sets.union(0, 1);
        sets.union(2, 3);
        sets.union(1, 2);
        assert_eq!(sets.find(0), sets.find(3));
        assert_ne!(sets.find(0), sets.find(4));
        assert_ne!(sets.find(4), sets.find(5));
    }

    // Equivalence of the candidates against a brute-force labelling.

    const NODES: u32 = 24;
    const LCG_MUL: u128 = 6364136223846793005;
    const LCG_INC: u128 = 1442695040888963407;
    const NZ_TWO_POW_64: NonZero<u128> = 0x10000000000000000;
    const NZ_TWO_POW_32: NonZero<u128> = 0x100000000;

    fn next(ref state: u64) -> u32 {
        let (_, low) = DivRem::div_rem(state.into() * LCG_MUL + LCG_INC, NZ_TWO_POW_64);
        state = low.try_into().unwrap();
        let (high, _) = DivRem::div_rem(low, NZ_TWO_POW_32);
        high.try_into().unwrap()
    }

    /// Reference: `labels[i]` is the smallest id of the set of `i`, relabelled on every union.
    fn relabel(labels: Span<u32>, a: u32, b: u32) -> Span<u32> {
        let (label_a, label_b) = (*labels.at(a), *labels.at(b));
        let (keep, drop) = if label_a < label_b {
            (label_a, label_b)
        } else {
            (label_b, label_a)
        };
        let mut labels = labels;
        let mut relabelled = array![];
        while let Option::Some(label) = labels.pop_front() {
            relabelled.append(if *label == drop {
                keep
            } else {
                *label
            });
        }
        relabelled.span()
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_candidates_equivalent(seed: u64) {
        let mut rng = seed;
        let mut halving: UnionFind = UnionFindTrait::new();
        let mut compression: CompressionUnionFind = UnionFindTrait::new();
        let mut plain: PlainUnionFind = UnionFindTrait::new();
        let mut size: SizeUnionFind = UnionFindTrait::new();
        let mut rank: RankUnionFind = UnionFindTrait::new();
        let mut labels = array![];
        let mut id = 0;
        while id != NODES {
            labels.append(id);
            id += 1;
        }
        let mut labels = labels.span();

        let mut step: u32 = 0;
        while step != 20 {
            let a = next(ref rng) % NODES;
            let b = next(ref rng) % NODES;
            labels = relabel(labels, a, b);
            let expected = *labels.at(a);
            assert_eq!(halving.union(a, b), expected);
            assert_eq!(compression.union(a, b), expected);
            assert_eq!(plain.union(a, b), expected);
            assert_eq!(size.union(a, b), expected);
            assert_eq!(rank.union(a, b), expected);

            let c = next(ref rng) % NODES;
            let d = next(ref rng) % NODES;
            let linked = *labels.at(c) == *labels.at(d);
            assert_eq!(halving.connected(c, d), linked);
            assert_eq!(compression.connected(c, d), linked);
            assert_eq!(plain.connected(c, d), linked);
            assert_eq!(size.connected(c, d), linked);
            assert_eq!(rank.connected(c, d), linked);
            step += 1;
        }

        let mut id = 0;
        while id != NODES {
            let expected = *labels.at(id);
            assert_eq!(halving.find(id), expected);
            assert_eq!(compression.find(id), expected);
            assert_eq!(plain.find(id), expected);
            assert_eq!(size.find(id), expected);
            assert_eq!(rank.find(id), expected);
            id += 1;
        }
    }

    // Gas probes. A scenario probe builds its edge list, unions every edge, then calls `find`
    // on each of the `n` ids (what island extraction does). Subtract `gas_edges_<scenario>`
    // for the net cost of the disjoint-set work.

    const N: u32 = 32;

    /// `union(i, i + 1)` for ascending `i`: the friendliest order for smaller-root-wins.
    #[inline(never)]
    fn edges_ascending() -> Span<(u32, u32)> {
        let mut edges = array![];
        let mut i: u32 = 0;
        while i != N - 1 {
            edges.append((opaque(i), i + 1));
            i += 1;
        }
        edges.span()
    }

    /// `union(i, i + 1)` for descending `i`: smaller-root-wins builds one path of length `n`.
    #[inline(never)]
    fn edges_descending() -> Span<(u32, u32)> {
        let mut edges = array![];
        let mut i: u32 = N - 1;
        while i != 0 {
            edges.append((opaque(i - 1), i));
            i -= 1;
        }
        edges.span()
    }

    /// Four stacks of eight bodies, contacts `(i, i + 1)` visited in a scattered order: the
    /// shape of a typical island-building pass.
    #[inline(never)]
    fn edges_stacks() -> Span<(u32, u32)> {
        let mut edges = array![];
        let mut k: u32 = 0;
        while k != N - 1 {
            let i = opaque(k * 13) % (N - 1);
            if i % 8 != 7 {
                edges.append((i, i + 1));
            }
            k += 1;
        }
        edges.span()
    }

    /// Balanced tournament: pairs, then pairs of pairs, … until one set remains.
    #[inline(never)]
    fn edges_tournament() -> Span<(u32, u32)> {
        let mut edges = array![];
        let mut stride: u32 = 1;
        while stride != N {
            let mut i: u32 = 0;
            while i != N {
                // Join through the last element of each block to force real walks.
                edges.append((opaque(i + stride - 1), i + 2 * stride - 1));
                i += 2 * stride;
            }
            stride *= 2;
        }
        edges.span()
    }

    /// Runs a scenario and returns the sum of the representatives of all ids.
    fn run<U, +UnionFindTrait<U>, +Destruct<U>>(mut edges: Span<(u32, u32)>) -> u32 {
        let mut sets: U = UnionFindTrait::new();
        while let Option::Some(edge) = edges.pop_front() {
            let (a, b) = *edge;
            sets.union(a, b);
        }
        let mut sum = 0;
        let mut id = 0;
        while id != N {
            sum += sets.find(id);
            id += 1;
        }
        sum
    }

    /// Four stacks rooted at 0, 8, 16 and 24.
    const STACKS_SUM: u32 = 384;

    #[test]
    fn test_scenarios_agree() {
        assert_eq!(run::<UnionFind>(edges_ascending()), 0);
        assert_eq!(run::<UnionFind>(edges_descending()), 0);
        assert_eq!(run::<UnionFind>(edges_tournament()), 0);
        assert_eq!(run::<UnionFind>(edges_stacks()), STACKS_SUM);
        assert_eq!(run::<CompressionUnionFind>(edges_stacks()), STACKS_SUM);
        assert_eq!(run::<PlainUnionFind>(edges_stacks()), STACKS_SUM);
        assert_eq!(run::<SizeUnionFind>(edges_stacks()), STACKS_SUM);
        assert_eq!(run::<RankUnionFind>(edges_stacks()), STACKS_SUM);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_edges_ascending() {
        assert!(edges_ascending().len() == 31);
    }

    #[test]
    fn gas_edges_descending() {
        assert!(edges_descending().len() == 31);
    }

    #[test]
    fn gas_edges_stacks() {
        assert!(edges_stacks().len() == 28);
    }

    #[test]
    fn gas_edges_tournament() {
        assert!(edges_tournament().len() == 31);
    }

    #[test]
    fn gas_halving_ascending() {
        assert!(run::<UnionFind>(edges_ascending()) == 0);
    }

    #[test]
    fn gas_compression_ascending() {
        assert!(run::<CompressionUnionFind>(edges_ascending()) == 0);
    }

    #[test]
    fn gas_plain_ascending() {
        assert!(run::<PlainUnionFind>(edges_ascending()) == 0);
    }

    #[test]
    fn gas_size_ascending() {
        assert!(run::<SizeUnionFind>(edges_ascending()) == 0);
    }

    #[test]
    fn gas_rank_ascending() {
        assert!(run::<RankUnionFind>(edges_ascending()) == 0);
    }

    #[test]
    fn gas_halving_descending() {
        assert!(run::<UnionFind>(edges_descending()) == 0);
    }

    #[test]
    fn gas_compression_descending() {
        assert!(run::<CompressionUnionFind>(edges_descending()) == 0);
    }

    #[test]
    fn gas_plain_descending() {
        assert!(run::<PlainUnionFind>(edges_descending()) == 0);
    }

    #[test]
    fn gas_size_descending() {
        assert!(run::<SizeUnionFind>(edges_descending()) == 0);
    }

    #[test]
    fn gas_rank_descending() {
        assert!(run::<RankUnionFind>(edges_descending()) == 0);
    }

    #[test]
    fn gas_halving_stacks() {
        assert!(run::<UnionFind>(edges_stacks()) == STACKS_SUM);
    }

    #[test]
    fn gas_compression_stacks() {
        assert!(run::<CompressionUnionFind>(edges_stacks()) == STACKS_SUM);
    }

    #[test]
    fn gas_plain_stacks() {
        assert!(run::<PlainUnionFind>(edges_stacks()) == STACKS_SUM);
    }

    #[test]
    fn gas_size_stacks() {
        assert!(run::<SizeUnionFind>(edges_stacks()) == STACKS_SUM);
    }

    #[test]
    fn gas_rank_stacks() {
        assert!(run::<RankUnionFind>(edges_stacks()) == STACKS_SUM);
    }

    #[test]
    fn gas_halving_tournament() {
        assert!(run::<UnionFind>(edges_tournament()) == 0);
    }

    #[test]
    fn gas_compression_tournament() {
        assert!(run::<CompressionUnionFind>(edges_tournament()) == 0);
    }

    #[test]
    fn gas_plain_tournament() {
        assert!(run::<PlainUnionFind>(edges_tournament()) == 0);
    }

    #[test]
    fn gas_size_tournament() {
        assert!(run::<SizeUnionFind>(edges_tournament()) == 0);
    }

    #[test]
    fn gas_rank_tournament() {
        assert!(run::<RankUnionFind>(edges_tournament()) == 0);
    }

    // Single calls on the shipped structure.

    #[test]
    fn gas_halving_new() {
        let _sets: UnionFind = UnionFindTrait::new();
    }

    /// Subtract `gas_halving_new`.
    #[test]
    fn gas_halving_find_singleton() {
        let mut sets: UnionFind = UnionFindTrait::new();
        assert!(sets.find(opaque(5)) == 5);
    }

    /// Subtract `gas_halving_new`.
    #[test]
    fn gas_halving_union_singletons() {
        let mut sets: UnionFind = UnionFindTrait::new();
        assert!(sets.union(opaque(5), opaque(9)) == 5);
    }

    /// Subtract `gas_halving_new`.
    #[test]
    fn gas_halving_connected_singletons() {
        let mut sets: UnionFind = UnionFindTrait::new();
        assert!(!sets.connected(opaque(5), opaque(9)));
    }
}
