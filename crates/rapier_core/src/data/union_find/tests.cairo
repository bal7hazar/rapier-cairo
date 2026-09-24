//! Unit tests of the union-find, equivalence of the candidates in `alternatives` and gas probes.

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
