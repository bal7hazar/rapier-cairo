//! Four-unit x strips centred on multiples of four. This is the cheap small-grid candidate:
//! streaming insertion tests earlier occupants, so there is no counting-range walk or sort
//! of proxies. Two-strip boxes are deduplicated by their lowest shared strip. Wider boxes
//! (e.g. a static ground) are never inserted and pair by scans, as in the grid; crowded
//! strips fall back to a density-selected scan; large worlds use the 2D grid.
use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
use core::num::traits::DivRem;
use super::ordering::{large_partners, large_row, rows_ascending};
use super::{BroadPhaseProxy, crowded_fallback, overlaps_raw, pair_allowed};

/// Returns exact sorted pairs using one-dimensional cells; no input-order assumption.
pub(crate) fn find_pairs_strip(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    find_pairs_strip_limit(proxies, 2)
}

/// Same algorithm with a measured occupancy bound. Descending insertion as in the grid.
pub(crate) fn find_pairs_strip_limit(
    proxies: Span<BroadPhaseProxy>, limit: u32,
) -> Array<(u32, u32)> {
    let mut heads: Felt252Dict<u32> = Default::default();
    let mut nodes = array![];
    let mut large: Array<u32> = array![];
    let mut pairs = array![];
    let n = proxies.len();
    let mut rest = proxies;
    while let Option::Some(a) = rest.pop_back() {
        let a = *a;
        let i = rest.len();
        let lo = cell(a.aabb.mins.x.raw);
        let hi = cell(a.aabb.maxs.x.raw);
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
