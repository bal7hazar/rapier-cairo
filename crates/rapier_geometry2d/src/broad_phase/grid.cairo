//! Power-of-two cells sized from the proxies (`scale::cell_size`, four world units for unit
//! boxes); signed Q32.32 coordinates are biased before floor division.
//! Every small box occupies at most four cells. Larger boxes (including infinite grounds) are
//! never inserted: each scans its later proxies once and is tested by the earlier small ones,
//! so neither coordinate magnitude nor extent can cause unbounded loops.
use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
use super::ordering::{large_partners, large_row, rows_ascending};
use super::scale::{cell_size, floor_div};
use super::{BroadPhaseProxy, crowded_fallback, overlaps_raw, pair_allowed};

#[derive(Copy, Drop, Serde, PartialEq, Debug)]
struct Cells {
    x0: u64,
    x1: u64,
    y0: u64,
    y1: u64,
}

/// Raw coordinate offset by 2^63, so that floor division covers the complete i64 range.
#[inline(always)]
pub(crate) fn biased(raw: i64) -> u64 {
    let biased: felt252 = raw.into();
    (biased + 0x8000000000000000).try_into().unwrap()
}

#[inline(always)]
fn key(x: u64, y: u64) -> felt252 {
    let x: felt252 = x.into();
    let y: felt252 = y.into();
    x + y * 0x10000000000000000
}

#[inline(always)]
fn cells(p: BroadPhaseProxy, size: NonZero<u64>) -> Cells {
    Cells {
        x0: floor_div(biased(p.aabb.mins.x.raw), size),
        x1: floor_div(biased(p.aabb.maxs.x.raw), size),
        y0: floor_div(biased(p.aabb.mins.y.raw), size),
        y1: floor_div(biased(p.aabb.maxs.y.raw), size),
    }
}

#[inline(always)]
fn small(c: Cells) -> bool {
    c.x1 >= c.x0 && c.y1 >= c.y0 && c.x1 - c.x0 <= 1 && c.y1 - c.y0 <= 1
}

/// Dict traversal follows explicit linked lists, never dict iteration order. Proxies are
/// inserted in descending index order, so every partner found is a later proxy and each cell
/// list yields ascending indices: rows come out (nearly) sorted and are replayed in reverse.
/// A shared pair belongs to its lowest shared cell, so no pair is found twice.
pub(crate) fn find_pairs_grid(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    find_pairs_grid_sized(proxies, cell_size(proxies))
}

/// Same algorithm with square cells of `size` raw units.
pub(crate) fn find_pairs_grid_sized(
    proxies: Span<BroadPhaseProxy>, size: NonZero<u64>,
) -> Array<(u32, u32)> {
    let mut heads: Felt252Dict<u32> = Default::default();
    let mut nodes: Array<(u32, u32, Cells)> = array![];
    let mut large: Array<u32> = array![];
    let mut pairs = array![];
    let n = proxies.len();
    let mut rest = proxies;
    while let Option::Some(a) = rest.pop_back() {
        let a = *a;
        let i = rest.len();
        let c = cells(a, size);
        if !small(c) {
            large_row(proxies, i, a, ref pairs);
            large.append(i);
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
                        return crowded_fallback(proxies, pairs.len(), n - 1 - i);
                    }
                    occupancy += 1;
                    let (j, next, b_cells) = *nodes.at(link - 1);
                    // Only the lower-left shared cell owns the pair.
                    if (x == c.x0 || x == b_cells.x0) && (y == c.y0 || y == b_cells.y0) {
                        let b = *proxies.at(j);
                        if pair_allowed(a, b) && overlaps_raw(a.aabb, b.aabb) {
                            pairs.append((i, j));
                        }
                    }
                    link = next;
                }
                y += 1;
            }
            x += 1;
        }
        if large.len() != 0 {
            large_partners(proxies, i, a, large.span(), ref pairs);
        }
    }
    rows_ascending(pairs)
}
