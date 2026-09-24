//! X strips centred on multiples of the cell size (`scale::cell_size`, four world units for unit
//! boxes). This is the cheap small-grid candidate:
//! streaming insertion tests earlier occupants, so there is no counting-range walk or sort
//! of proxies. Two-strip boxes are deduplicated by their lowest shared strip. Wider boxes
//! (e.g. a static ground) are never inserted and pair by scans, as in the grid; crowded
//! strips fall back to a density-selected scan; large worlds use the 2D grid.
use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
use core::num::traits::DivRem;
use super::ordering::{large_partners, large_row, rows_ascending};
use super::scale::cell_size;
use super::{BroadPhaseProxy, crowded_fallback, overlaps_raw, pair_allowed};

/// Returns exact sorted pairs using one-dimensional cells; no input-order assumption.
pub(crate) fn find_pairs_strip(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    find_pairs_strip_limit(proxies, 2)
}

/// Same algorithm with strips of `size` raw units (even), centred on multiples of `size`.
pub(crate) fn find_pairs_strip_sized(
    proxies: Span<BroadPhaseProxy>, size: NonZero<u64>,
) -> Array<(u32, u32)> {
    strip_pairs(proxies, 2, size)
}

/// Same algorithm with a measured occupancy bound. Descending insertion as in the grid.
pub(crate) fn find_pairs_strip_limit(
    proxies: Span<BroadPhaseProxy>, limit: u32,
) -> Array<(u32, u32)> {
    strip_pairs(proxies, limit, cell_size(proxies))
}

fn strip_pairs(
    proxies: Span<BroadPhaseProxy>, limit: u32, size: NonZero<u64>,
) -> Array<(u32, u32)> {
    let size: u64 = size.into();
    let wide: u128 = size.into();
    let half = wide / 2;
    let wide: NonZero<u128> = wide.try_into().unwrap();
    let mut heads: Felt252Dict<u32> = Default::default();
    let mut nodes = array![];
    let mut large: Array<u32> = array![];
    let mut pairs = array![];
    let n = proxies.len();
    let mut rest = proxies;
    while let Option::Some(a) = rest.pop_back() {
        let a = *a;
        let i = rest.len();
        let lo = floor_div128(biased(a.aabb.mins.x.raw) + half, wide);
        let hi = floor_div128(biased(a.aabb.maxs.x.raw) + half, wide);
        if !narrow(lo, hi) {
            large_row(proxies, i, a, ref pairs);
            large.append(i);
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
                    return crowded_fallback(proxies, pairs.len(), n - 1 - i);
                }
                occupancy += 1;
                let (j, next, other_lo) = *nodes.at(link - 1);
                if k == lo || k == other_lo {
                    let b = *proxies.at(j);
                    if pair_allowed(a, b) && overlaps_raw(a.aabb, b.aabb) {
                        pairs.append((i, j));
                    }
                }
                link = next;
            }
            k += 1;
        }
        if large.len() != 0 {
            large_partners(proxies, i, a, large.span(), ref pairs);
        }
    }
    rows_ascending(pairs)
}

/// Raw coordinate offset by 2^63; u128 keeps the centring shift representable at both extremes.
#[inline(always)]
fn biased(raw: i64) -> u128 {
    let biased: felt252 = raw.into();
    (biased + 0x8000000000000000).try_into().unwrap()
}

#[inline(always)]
fn narrow(lo: u128, hi: u128) -> bool {
    hi >= lo && hi - lo <= 1
}

#[inline(always)]
fn floor_div128(v: u128, size: NonZero<u128>) -> u128 {
    let (q, _) = DivRem::div_rem(v, size);
    q
}
