//! Rejected union-find candidates, kept compiled (tests only) so the ranking can be re-measured on
//! every toolchain bump. All implement `UnionFindTrait` with the exact semantics of `UnionFind`.

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
