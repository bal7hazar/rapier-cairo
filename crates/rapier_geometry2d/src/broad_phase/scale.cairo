//! Scale-free cell size for the strip and the grid.
//!
//! The cell is a power of two of raw units, `2^shift`, chosen once per call from the proxies
//! themselves: `shift = floor(log2(w + h)) + 1` of the median of five sampled extents, clamped to
//! `MIN_SHIFT..=MAX_SHIFT`. A box of side `L` therefore sits in a cell of size between `2 L` and
//! `4 L` whatever the world scale (a one-unit box gets four-unit cells, as BG's fixed cell). The
//! size is built by conditional constant multiplications while the shift is found (no runtime
//! `pow`, no `match`) and is then a runtime `NonZero` divisor, converted once per call. The choice
//! only changes cost: any size yields the exact pair set (wide boxes are scanned, crowded cells
//! fall back).
use core::num::traits::DivRem;
use super::BroadPhaseProxy;

/// Four world units, the cell size used below five proxies (and BG's fixed cell).
pub(crate) const UNIT_CELL: u64 = 0x400000000;

/// `w + h` of a proxy, saturated to `u64` (an inverted box saturates too: it is scanned anyway).
#[inline(always)]
pub(crate) fn extent(p: BroadPhaseProxy) -> u64 {
    let w: felt252 = p.aabb.maxs.x.raw.into() - p.aabb.mins.x.raw.into();
    let h: felt252 = p.aabb.maxs.y.raw.into() - p.aabb.mins.y.raw.into();
    (w + h).try_into().unwrap_or(0xffffffffffffffff)
}

#[inline(always)]
fn sort2(a: u64, b: u64) -> (u64, u64) {
    if a > b {
        (b, a)
    } else {
        (a, b)
    }
}

/// Median of five with seven comparisons.
#[inline(always)]
pub(crate) fn median5(a: u64, b: u64, c: u64, d: u64, e: u64) -> u64 {
    let (a, b) = sort2(a, b);
    let (d, e) = sort2(d, e);
    // Order the pairs by their minima: `a` is then below three values and cannot be the median.
    let (d, e, b) = if a > d {
        (a, b, e)
    } else {
        (d, e, b)
    };
    let (b, c) = sort2(b, c);
    // Second smallest of the sorted pairs (b, c) and (d, e).
    let lo = if b > d {
        b
    } else {
        d
    };
    let hi = if c < e {
        c
    } else {
        e
    };
    if lo < hi {
        lo
    } else {
        hi
    }
}

/// `2^clamp(floor(log2(s)) + 1, 20, 51)`: cells from about 0.00024 to 524 288 world units.
#[inline(always)]
pub(crate) fn cell_of(s: u64) -> u64 {
    let mut v = s / 0x80000;
    let mut cell = 0x100000;
    if v >= 0x100000000 {
        v = 0xffffffff;
    }
    if v >= 0x10000 {
        v = v / 0x10000;
        cell *= 0x10000;
    }
    if v >= 0x100 {
        v = v / 0x100;
        cell *= 0x100;
    }
    if v >= 0x10 {
        v = v / 0x10;
        cell *= 0x10;
    }
    if v >= 4 {
        v = v / 4;
        cell *= 4;
    }
    if v >= 2 {
        cell *= 2;
    }
    cell
}

/// Cell size (a power of two of raw units) of a proxy set. Deterministic: five fixed positions,
/// median of their extents.
pub(crate) fn cell_size(proxies: Span<BroadPhaseProxy>) -> NonZero<u64> {
    let n = proxies.len();
    let size = if n < 5 {
        UNIT_CELL
    } else {
        let s0 = extent(*proxies.at(n / 10));
        let s1 = extent(*proxies.at(n * 3 / 10));
        let s2 = extent(*proxies.at(n / 2));
        let s3 = extent(*proxies.at(n * 7 / 10));
        let s4 = extent(*proxies.at(n * 9 / 10));
        cell_of(median5(s0, s1, s2, s3, s4))
    };
    size.try_into().unwrap()
}

/// Floors a coordinate biased by `2^63` into cells of `size`.
#[inline(always)]
pub(crate) fn floor_div(v: u64, size: NonZero<u64>) -> u64 {
    let (q, _) = DivRem::div_rem(v, size);
    q
}
