//! Rejected scale-free cell formulations, kept to be re-measured on toolchain bumps.
//! * `match` on the exponent over constant divisors, once per proxy (`find_pairs_grid_match`);
//! * multiply-shift with a runtime reciprocal (`floor_wide_mul`);
//! * a cheaper statistic, the maximum of three sampled extents (`cell_size_max3`).
use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
use core::num::traits::{DivRem, WideMul};
use super::super::grid::biased;
use super::super::ordering::{large_partners, large_row, rows_ascending};
use super::super::scale::{cell_of, extent, median5};
use super::super::{BroadPhaseProxy, crowded_fallback, overlaps_raw, pair_allowed};

#[derive(Copy, Drop, Serde, PartialEq, Debug)]
struct Cells {
    x0: u64,
    x1: u64,
    y0: u64,
    y1: u64,
}

#[inline(always)]
fn floor_div(v: u64, d: NonZero<u64>) -> u64 {
    let (q, _) = DivRem::div_rem(v, d);
    q
}

/// `floor(v / 2^shift)` as the high word of `v * 2^(64 - shift)`; `recip` is that power of two.
#[inline(always)]
pub(crate) fn floor_wide_mul(v: u64, recip: u64) -> u64 {
    let (high, _) = DivRem::div_rem(v.wide_mul(recip), 0x10000000000000000);
    high.try_into().unwrap()
}

/// Maximum of three sampled extents, as a cell size (see `scale::cell_size`).
pub(crate) fn cell_size_max3(proxies: Span<BroadPhaseProxy>) -> NonZero<u64> {
    let n = proxies.len();
    let size = if n < 5 {
        0x400000000
    } else {
        let s0 = extent(*proxies.at(n / 4));
        let s1 = extent(*proxies.at(n / 2));
        let s2 = extent(*proxies.at(n * 3 / 4));
        let hi = if s0 > s1 {
            s0
        } else {
            s1
        };
        cell_of(if hi > s2 {
            hi
        } else {
            s2
        })
    };
    size.try_into().unwrap()
}

/// The exponent of `scale::cell_of` by five conditional steps, as `shift`.
#[inline(always)]
fn shift_of(s: u64) -> u32 {
    let mut v = s / 0x80000;
    let mut shift = 20;
    if v >= 0x100000000 {
        v = 0xffffffff;
    }
    if v >= 0x10000 {
        v = v / 0x10000;
        shift += 16;
    }
    if v >= 0x100 {
        v = v / 0x100;
        shift += 8;
    }
    if v >= 0x10 {
        v = v / 0x10;
        shift += 4;
    }
    if v >= 4 {
        v = v / 4;
        shift += 2;
    }
    if v >= 2 {
        shift += 1;
    }
    shift
}

pub(crate) fn cell_shift(proxies: Span<BroadPhaseProxy>) -> u32 {
    let n = proxies.len();
    if n < 5 {
        return 34;
    }
    let s0 = extent(*proxies.at(n / 10));
    let s1 = extent(*proxies.at(n * 3 / 10));
    let s2 = extent(*proxies.at(n / 2));
    let s3 = extent(*proxies.at(n * 7 / 10));
    let s4 = extent(*proxies.at(n * 9 / 10));
    shift_of(median5(s0, s1, s2, s3, s4))
}

#[inline(always)]
fn floor_quad(x0: u64, x1: u64, y0: u64, y1: u64, shift: u32) -> (u64, u64, u64, u64) {
    match shift {
        20 => (
            floor_div(x0, 0x100000),
            floor_div(x1, 0x100000),
            floor_div(y0, 0x100000),
            floor_div(y1, 0x100000),
        ),
        21 => (
            floor_div(x0, 0x200000),
            floor_div(x1, 0x200000),
            floor_div(y0, 0x200000),
            floor_div(y1, 0x200000),
        ),
        22 => (
            floor_div(x0, 0x400000),
            floor_div(x1, 0x400000),
            floor_div(y0, 0x400000),
            floor_div(y1, 0x400000),
        ),
        23 => (
            floor_div(x0, 0x800000),
            floor_div(x1, 0x800000),
            floor_div(y0, 0x800000),
            floor_div(y1, 0x800000),
        ),
        24 => (
            floor_div(x0, 0x1000000),
            floor_div(x1, 0x1000000),
            floor_div(y0, 0x1000000),
            floor_div(y1, 0x1000000),
        ),
        25 => (
            floor_div(x0, 0x2000000),
            floor_div(x1, 0x2000000),
            floor_div(y0, 0x2000000),
            floor_div(y1, 0x2000000),
        ),
        26 => (
            floor_div(x0, 0x4000000),
            floor_div(x1, 0x4000000),
            floor_div(y0, 0x4000000),
            floor_div(y1, 0x4000000),
        ),
        27 => (
            floor_div(x0, 0x8000000),
            floor_div(x1, 0x8000000),
            floor_div(y0, 0x8000000),
            floor_div(y1, 0x8000000),
        ),
        28 => (
            floor_div(x0, 0x10000000),
            floor_div(x1, 0x10000000),
            floor_div(y0, 0x10000000),
            floor_div(y1, 0x10000000),
        ),
        29 => (
            floor_div(x0, 0x20000000),
            floor_div(x1, 0x20000000),
            floor_div(y0, 0x20000000),
            floor_div(y1, 0x20000000),
        ),
        30 => (
            floor_div(x0, 0x40000000),
            floor_div(x1, 0x40000000),
            floor_div(y0, 0x40000000),
            floor_div(y1, 0x40000000),
        ),
        31 => (
            floor_div(x0, 0x80000000),
            floor_div(x1, 0x80000000),
            floor_div(y0, 0x80000000),
            floor_div(y1, 0x80000000),
        ),
        32 => (
            floor_div(x0, 0x100000000),
            floor_div(x1, 0x100000000),
            floor_div(y0, 0x100000000),
            floor_div(y1, 0x100000000),
        ),
        33 => (
            floor_div(x0, 0x200000000),
            floor_div(x1, 0x200000000),
            floor_div(y0, 0x200000000),
            floor_div(y1, 0x200000000),
        ),
        34 => (
            floor_div(x0, 0x400000000),
            floor_div(x1, 0x400000000),
            floor_div(y0, 0x400000000),
            floor_div(y1, 0x400000000),
        ),
        35 => (
            floor_div(x0, 0x800000000),
            floor_div(x1, 0x800000000),
            floor_div(y0, 0x800000000),
            floor_div(y1, 0x800000000),
        ),
        36 => (
            floor_div(x0, 0x1000000000),
            floor_div(x1, 0x1000000000),
            floor_div(y0, 0x1000000000),
            floor_div(y1, 0x1000000000),
        ),
        37 => (
            floor_div(x0, 0x2000000000),
            floor_div(x1, 0x2000000000),
            floor_div(y0, 0x2000000000),
            floor_div(y1, 0x2000000000),
        ),
        38 => (
            floor_div(x0, 0x4000000000),
            floor_div(x1, 0x4000000000),
            floor_div(y0, 0x4000000000),
            floor_div(y1, 0x4000000000),
        ),
        39 => (
            floor_div(x0, 0x8000000000),
            floor_div(x1, 0x8000000000),
            floor_div(y0, 0x8000000000),
            floor_div(y1, 0x8000000000),
        ),
        40 => (
            floor_div(x0, 0x10000000000),
            floor_div(x1, 0x10000000000),
            floor_div(y0, 0x10000000000),
            floor_div(y1, 0x10000000000),
        ),
        41 => (
            floor_div(x0, 0x20000000000),
            floor_div(x1, 0x20000000000),
            floor_div(y0, 0x20000000000),
            floor_div(y1, 0x20000000000),
        ),
        42 => (
            floor_div(x0, 0x40000000000),
            floor_div(x1, 0x40000000000),
            floor_div(y0, 0x40000000000),
            floor_div(y1, 0x40000000000),
        ),
        43 => (
            floor_div(x0, 0x80000000000),
            floor_div(x1, 0x80000000000),
            floor_div(y0, 0x80000000000),
            floor_div(y1, 0x80000000000),
        ),
        44 => (
            floor_div(x0, 0x100000000000),
            floor_div(x1, 0x100000000000),
            floor_div(y0, 0x100000000000),
            floor_div(y1, 0x100000000000),
        ),
        45 => (
            floor_div(x0, 0x200000000000),
            floor_div(x1, 0x200000000000),
            floor_div(y0, 0x200000000000),
            floor_div(y1, 0x200000000000),
        ),
        46 => (
            floor_div(x0, 0x400000000000),
            floor_div(x1, 0x400000000000),
            floor_div(y0, 0x400000000000),
            floor_div(y1, 0x400000000000),
        ),
        47 => (
            floor_div(x0, 0x800000000000),
            floor_div(x1, 0x800000000000),
            floor_div(y0, 0x800000000000),
            floor_div(y1, 0x800000000000),
        ),
        48 => (
            floor_div(x0, 0x1000000000000),
            floor_div(x1, 0x1000000000000),
            floor_div(y0, 0x1000000000000),
            floor_div(y1, 0x1000000000000),
        ),
        49 => (
            floor_div(x0, 0x2000000000000),
            floor_div(x1, 0x2000000000000),
            floor_div(y0, 0x2000000000000),
            floor_div(y1, 0x2000000000000),
        ),
        50 => (
            floor_div(x0, 0x4000000000000),
            floor_div(x1, 0x4000000000000),
            floor_div(y0, 0x4000000000000),
            floor_div(y1, 0x4000000000000),
        ),
        51 => (
            floor_div(x0, 0x8000000000000),
            floor_div(x1, 0x8000000000000),
            floor_div(y0, 0x8000000000000),
            floor_div(y1, 0x8000000000000),
        ),
        _ => (
            floor_div(x0, 0x400000000),
            floor_div(x1, 0x400000000),
            floor_div(y0, 0x400000000),
            floor_div(y1, 0x400000000),
        ),
    }
}

#[inline(always)]
fn cells(p: BroadPhaseProxy, shift: u32) -> Cells {
    let (x0, x1, y0, y1) = floor_quad(
        biased(p.aabb.mins.x.raw),
        biased(p.aabb.maxs.x.raw),
        biased(p.aabb.mins.y.raw),
        biased(p.aabb.maxs.y.raw),
        shift,
    );
    Cells { x0, x1, y0, y1 }
}

#[inline(always)]
fn key(x: u64, y: u64) -> felt252 {
    let x: felt252 = x.into();
    let y: felt252 = y.into();
    x + y * 0x10000000000000000
}

#[inline(always)]
fn small(c: Cells) -> bool {
    c.x1 >= c.x0 && c.y1 >= c.y0 && c.x1 - c.x0 <= 1 && c.y1 - c.y0 <= 1
}

/// The shipped grid with the cell exponent dispatched per proxy by a `match` over constant
/// divisors (`floor_quad`). The linear arm chain is charged once per proxy on its worst path.
pub(crate) fn find_pairs_grid_match(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    let shift = cell_shift(proxies);
    let mut heads: Felt252Dict<u32> = Default::default();
    let mut nodes: Array<(u32, u32, Cells)> = array![];
    let mut large: Array<u32> = array![];
    let mut pairs = array![];
    let n = proxies.len();
    let mut rest = proxies;
    while let Option::Some(a) = rest.pop_back() {
        let a = *a;
        let i = rest.len();
        let c = cells(a, shift);
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
