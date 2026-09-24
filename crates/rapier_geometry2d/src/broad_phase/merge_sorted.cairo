//! Rejected: the first grid and strip, which walk proxies in ascending index order, emit
//! `(j, i)` in discovery order, pair large boxes in a separate pass and restore `(i, j)` order
//! with a merge sort over every pair (≈ 37k gas per pair at 32 pairs, O(P log P)).
use super::{BroadPhaseProxy, overlaps_raw, pair_allowed};

pub(crate) mod grid {
    use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
    use core::num::traits::DivRem;
    use super::append_large_pairs;
    use super::super::ordering::merge_sort_pairs;
    use super::super::{BroadPhaseProxy, crowded_fallback, overlaps_raw, pair_allowed};

    #[derive(Copy, Drop, Serde, PartialEq, Debug)]
    struct Cells {
        x0: u32,
        x1: u32,
        y0: u32,
        y1: u32,
    }

    /// Floor into four-unit cells, offset by 2^29. Covers the complete i64 raw range.
    #[inline(always)]
    pub(crate) fn cell(raw: i64) -> u32 {
        let biased: felt252 = raw.into();
        let biased: u64 = (biased + 0x8000000000000000).try_into().unwrap();
        let (q, _) = DivRem::div_rem(biased, 0x400000000);
        q.try_into().unwrap()
    }

    #[inline(always)]
    fn key(x: u32, y: u32) -> felt252 {
        let x: felt252 = x.into();
        let y: felt252 = y.into();
        x + y * 0x40000000
    }

    #[inline(always)]
    fn cells(p: BroadPhaseProxy) -> Cells {
        Cells {
            x0: cell(p.aabb.mins.x.raw),
            x1: cell(p.aabb.maxs.x.raw),
            y0: cell(p.aabb.mins.y.raw),
            y1: cell(p.aabb.maxs.y.raw),
        }
    }

    #[inline(always)]
    fn small(c: Cells) -> bool {
        c.x1 >= c.x0 && c.y1 >= c.y0 && c.x1 - c.x0 <= 1 && c.y1 - c.y0 <= 1
    }

    /// Dict traversal follows explicit linked lists, never dict iteration order. A shared pair
    /// belongs to its lowest shared cell, eliminating duplicates before the final merge sort.
    pub(crate) fn find_pairs_grid(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
        let mut heads: Felt252Dict<u32> = Default::default();
        let mut nodes: Array<(u32, u32, Cells)> = array![];
        let mut large: Array<u32> = array![];
        let mut pairs = array![];
        let n = proxies.len();
        let mut i = 0;
        while i != n {
            let a = *proxies.at(i);
            let c = cells(a);
            if !small(c) {
                large.append(i);
                i += 1;
                continue;
            }
            let mut x = c.x0;
            while x != c.x1 + 1 {
                let mut y = c.y0;
                while y != c.y1 + 1 {
                    let (entry, head) = heads.entry(key(x, y));
                    nodes.append((i, head, c));
                    heads = entry.finalize(nodes.len());
                    let mut link = head;
                    let mut occupancy = 0;
                    while link != 0 {
                        if occupancy == 8 {
                            return crowded_fallback(proxies, pairs.len(), i);
                        }
                        occupancy += 1;
                        let (j, next, b_cells) = *nodes.at(link - 1);
                        // Only the lower-left shared cell owns the pair.
                        if (x == c.x0 || x == b_cells.x0) && (y == c.y0 || y == b_cells.y0) {
                            let b = *proxies.at(j);
                            if pair_allowed(a, b) && overlaps_raw(a.aabb, b.aabb) {
                                pairs.append((j, i));
                            }
                        }
                        link = next;
                    }
                    y += 1;
                }
                x += 1;
            }
            i += 1;
        }
        append_large_pairs(proxies, large.span(), ref pairs);
        merge_sort_pairs(pairs)
    }
}

pub(crate) mod strip {
    use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
    use core::num::traits::DivRem;
    use super::append_large_pairs;
    use super::super::ordering::merge_sort_pairs;
    use super::super::{BroadPhaseProxy, crowded_fallback, overlaps_raw, pair_allowed};

    /// Returns exact sorted pairs using one-dimensional cells; no input-order assumption.
    pub(crate) fn find_pairs_strip(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
        find_pairs_strip_limit(proxies, 2)
    }

    /// Same algorithm with a measured occupancy bound.
    pub(crate) fn find_pairs_strip_limit(
        proxies: Span<BroadPhaseProxy>, limit: u32,
    ) -> Array<(u32, u32)> {
        let mut heads: Felt252Dict<u32> = Default::default();
        let mut nodes = array![];
        let mut large: Array<u32> = array![];
        let mut pairs = array![];
        let mut i = 0;
        for a in proxies {
            let a = *a;
            let lo = cell(a.aabb.mins.x.raw);
            let hi = cell(a.aabb.maxs.x.raw);
            if !narrow(lo, hi) {
                large.append(i);
                i += 1;
                continue;
            }
            let mut k = lo;
            while k != hi + 1 {
                let (entry, head) = heads.entry(k.into());
                nodes.append((i, head, lo));
                heads = entry.finalize(nodes.len());
                let mut link = head;
                let mut occupancy = 0;
                while link != 0 {
                    if occupancy == limit {
                        return crowded_fallback(proxies, pairs.len(), i);
                    }
                    occupancy += 1;
                    let (j, next, other_lo) = *nodes.at(link - 1);
                    if k == lo || k == other_lo {
                        let b = *proxies.at(j);
                        if pair_allowed(a, b) && overlaps_raw(a.aabb, b.aabb) {
                            pairs.append((j, i));
                        }
                    }
                    link = next;
                }
                k += 1;
            }
            i += 1;
        }
        append_large_pairs(proxies, large.span(), ref pairs);
        merge_sort_pairs(pairs)
    }

    /// Floor into centred four-unit cells. Bias in the field, then divide as u128 so both i64
    /// extremes and the two-unit origin shift are representable without saturation or wrap.
    #[inline(always)]
    pub(crate) fn cell(raw: i64) -> u32 {
        let biased: felt252 = raw.into();
        let biased: u128 = (biased + 0x8000000200000000).try_into().unwrap();
        let (q, _) = DivRem::div_rem(biased, 0x400000000);
        q.try_into().unwrap()
    }

    #[inline(always)]
    fn narrow(lo: u32, hi: u32) -> bool {
        hi >= lo && hi - lo <= 1
    }
}

/// Pairs each large proxy with every small proxy and every later large proxy, exactly once.
/// O(n) per large proxy and independent of its extent, so an unbounded ground costs one scan.
/// `large` is ascending (built in index order), so a cursor tells large `j` apart without flags.
fn append_large_pairs(
    proxies: Span<BroadPhaseProxy>, large: Span<u32>, ref pairs: Array<(u32, u32)>,
) {
    let n = proxies.len();
    for big in large {
        let i = *big;
        let a = *proxies.at(i);
        let mut rest = large;
        let mut next_large = match rest.pop_front() {
            Option::Some(v) => *v,
            Option::None => n,
        };
        let mut j = 0;
        while j != n {
            let b = *proxies.at(j);
            if j == next_large {
                // Large-large once, from the lower index; also skips `j == i`.
                if j > i && pair_allowed(a, b) && overlaps_raw(a.aabb, b.aabb) {
                    pairs.append((i, j));
                }
                next_large = match rest.pop_front() {
                    Option::Some(v) => *v,
                    Option::None => n,
                };
            } else if pair_allowed(a, b) && overlaps_raw(a.aabb, b.aabb) {
                if i < j {
                    pairs.append((i, j));
                } else {
                    pairs.append((j, i));
                }
            }
            j += 1;
        }
    }
}
