use super::ordering::merge_sort_pairs;
use super::{BroadPhaseProxy, append_pair, find_pairs_brute, overlaps_fields_flat};

#[derive(Copy, Drop)]
struct SortProxy {
    index: u32,
    proxy: BroadPhaseProxy,
}

#[inline(always)]
fn before(a: SortProxy, b: SortProxy) -> bool {
    a.proxy.aabb.mins.x.raw < b.proxy.aabb.mins.x.raw
        || (a.proxy.aabb.mins.x.raw == b.proxy.aabb.mins.x.raw && a.index < b.index)
}

fn merge_proxy_runs(
    src: Span<SortProxy>, left: u32, mid: u32, right: u32, ref dst: Array<SortProxy>,
) {
    let mut i = left;
    let mut j = mid;
    while i != mid && j != right {
        let a = *src.at(i);
        let b = *src.at(j);
        if before(a, b) {
            dst.append(a);
            i += 1;
        } else {
            dst.append(b);
            j += 1;
        }
    }
    while i != mid {
        dst.append(*src.at(i));
        i += 1;
    }
    while j != right {
        dst.append(*src.at(j));
        j += 1;
    }
}

fn merge_sort_proxies(mut src: Array<SortProxy>) -> Array<SortProxy> {
    let n = src.len();
    let mut width = 1;
    while width < n {
        let span = src.span();
        let mut dst = array![];
        let mut left = 0;
        while left != n {
            let mut mid = left + width;
            if mid > n {
                mid = n;
            }
            let mut right = mid + width;
            if right > n {
                right = n;
            }
            merge_proxy_runs(span, left, mid, right, ref dst);
            left = right;
        }
        src = dst;
        width = width * 2;
    }
    src
}

fn sorted_by_min_x(proxies: Span<BroadPhaseProxy>) -> Array<SortProxy> {
    let mut unsorted = array![];
    let n = proxies.len();
    let mut i = 0;
    while i != n {
        unsorted.append(SortProxy { index: i, proxy: *proxies.at(i) });
        i += 1;
    }
    merge_sort_proxies(unsorted)
}

fn all_dynamic_sorted_x_disjoint(proxies: Span<BroadPhaseProxy>) -> bool {
    let n = proxies.len();
    if n < 16 {
        return false;
    }
    let first = *proxies.at(0);
    if first.is_static {
        return false;
    }
    let mut max_x = first.aabb.maxs.x.raw;
    let mut i = 1;
    while i != n {
        let proxy = *proxies.at(i);
        if proxy.is_static {
            return false;
        }
        let aabb = proxy.aabb;
        if aabb.mins.x.raw <= max_x {
            return false;
        }
        if aabb.maxs.x.raw > max_x {
            max_x = aabb.maxs.x.raw;
        }
        i += 1;
    }
    true
}

/// Benchmark-shaped sorted-disjoint guard kept out of the shipped path.
pub fn find_pairs_sorted_x_then_brute(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    let pairs = array![];
    if all_dynamic_sorted_x_disjoint(proxies) {
        pairs
    } else {
        find_pairs_brute(proxies)
    }
}

/// Sort-and-prune candidate: bottom-up merge sort by `mins.x`, sweep x, test y only.
///
/// Output pairs are merge-sorted back into the public `(i, j)` order.
pub fn find_pairs_sort_and_prune(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    let sorted = sorted_by_min_x(proxies);
    let n = sorted.len();
    let mut pairs = array![];
    let mut i = 0;
    while i != n {
        let a = *sorted.at(i);
        let mut j = i + 1;
        while j != n {
            let b = *sorted.at(j);
            if b.proxy.aabb.mins.x.raw > a.proxy.aabb.maxs.x.raw {
                j = n;
            } else {
                let a_aabb = a.proxy.aabb;
                if !(a.proxy.is_static && b.proxy.is_static)
                    && a_aabb.mins.y.raw <= b.proxy.aabb.maxs.y.raw
                    && b.proxy.aabb.mins.y.raw <= a_aabb.maxs.y.raw {
                    if a.index < b.index {
                        pairs.append((a.index, b.index));
                    } else {
                        pairs.append((b.index, a.index));
                    }
                }
                j += 1;
            }
        }
        i += 1;
    }
    merge_sort_pairs(pairs)
}
#[derive(Drop)]
struct ProxySoa {
    min_xs: Array<i64>,
    max_xs: Array<i64>,
    min_ys: Array<i64>,
    max_ys: Array<i64>,
    statics: Array<bool>,
}

fn build_soa(proxies: Span<BroadPhaseProxy>) -> ProxySoa {
    let mut min_xs = array![];
    let mut max_xs = array![];
    let mut min_ys = array![];
    let mut max_ys = array![];
    let mut statics = array![];
    let mut tail = proxies;
    while let Option::Some(value) = tail.pop_front() {
        let proxy = *value;
        let aabb = proxy.aabb;
        min_xs.append(aabb.mins.x.raw);
        max_xs.append(aabb.maxs.x.raw);
        min_ys.append(aabb.mins.y.raw);
        max_ys.append(aabb.maxs.y.raw);
        statics.append(proxy.is_static);
    }
    ProxySoa { min_xs, max_xs, min_ys, max_ys, statics }
}

fn find_pairs_soa_scan(
    proxies: Span<BroadPhaseProxy>, metered: bool, split_static: bool,
) -> Array<(u32, u32)> {
    let n = proxies.len();
    let ProxySoa { min_xs, max_xs, min_ys, max_ys, statics } = build_soa(proxies);
    let min_xs = min_xs.span();
    let max_xs = max_xs.span();
    let min_ys = min_ys.span();
    let max_ys = max_ys.span();
    let statics = statics.span();
    let mut pairs = array![];
    let mut i = 0;
    while i != n {
        let a_min_x = *min_xs.at(i);
        let a_max_x = *max_xs.at(i);
        let a_min_y = *min_ys.at(i);
        let a_max_y = *max_ys.at(i);
        let a_static = *statics.at(i);
        let mut j = i + 1;
        let len = n - j;
        let mut b_min_xs = min_xs.slice(j, len);
        let mut b_max_xs = max_xs.slice(j, len);
        let mut b_min_ys = min_ys.slice(j, len);
        let mut b_max_ys = max_ys.slice(j, len);
        let mut b_statics = statics.slice(j, len);
        while j != n {
            let b_min_x = *b_min_xs.pop_front().unwrap();
            let b_max_x = *b_max_xs.pop_front().unwrap();
            let b_min_y = *b_min_ys.pop_front().unwrap();
            let b_max_y = *b_max_ys.pop_front().unwrap();
            let b_static = *b_statics.pop_front().unwrap();
            if !(a_static && b_static)
                && (!split_static || !a_static || !b_static)
                && overlaps_fields_flat(
                    a_min_x, a_max_x, a_min_y, a_max_y, b_min_x, b_max_x, b_min_y, b_max_y,
                ) {
                if metered {
                    append_pair(ref pairs, i, j);
                } else {
                    pairs.append((i, j));
                }
            }
            j += 1;
        }
        i += 1;
    }
    pairs
}

/// Struct-of-arrays brute-force candidate with direct append on overlap.
pub(crate) fn find_pairs_soa(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    find_pairs_soa_scan(proxies, false, false)
}

/// Struct-of-arrays brute-force candidate with append behind a metered one-iteration loop.
pub(crate) fn find_pairs_soa_metered(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    find_pairs_soa_scan(proxies, true, false)
}

/// Rejected struct-of-arrays static-aware candidate.
pub(crate) fn find_pairs_soa_static_split(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    find_pairs_soa_scan(proxies, true, true)
}

/// Rejected signed-wide coordinate bias; compare gas_cell_signed with gas_cell_felt.
#[inline(always)]
pub(crate) fn cell_signed(raw: i64) -> u32 {
    let biased: i128 = raw.into();
    let biased: u64 = (biased + 0x8000000000000000).try_into().unwrap();
    let (q, _) = core::num::traits::DivRem::div_rem(biased, 0x400000000);
    q.try_into().unwrap()
}

mod previous_counting {
    //! Counting-sort quantised min-x, O(n + cell range); fall back when the range exceeds 4n.
    //! Within a bucket order is arbitrary: the sweep must use cell bounds, then exact overlap.
    use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
    use super::super::grid::cell;
    use super::super::ordering::merge_sort_pairs;
    use super::super::{BroadPhaseProxy, find_pairs_tail_static_split, overlaps_raw, pair_allowed};

    pub(crate) fn find_pairs_counting(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
        let n = proxies.len();
        if n == 0 {
            return array![];
        }
        let mut lo = cell(*proxies.at(0).aabb.mins.x.raw);
        let mut hi = lo;
        let mut keys = array![];
        for p in proxies {
            let k = cell(*p.aabb.mins.x.raw);
            keys.append(k);
            if k < lo {
                lo = k;
            }
            if k > hi {
                hi = k;
            }
        }
        let range: u64 = (hi - lo).into();
        let count: u64 = n.into();
        if hi == lo || range > count * 4 {
            return find_pairs_tail_static_split(proxies);
        }
        let mut heads: Felt252Dict<u32> = Default::default();
        let mut links = array![];
        let mut i = 0;
        while i != n {
            let (entry, head) = heads.entry((*keys.at(i)).into());
            links.append(head);
            heads = entry.finalize(i + 1);
            i += 1;
        }
        let mut sorted = array![];
        let mut k = lo;
        while k != hi + 1 {
            let mut link = heads.get(k.into());
            while link != 0 {
                let i = link - 1;
                sorted.append((i, k));
                link = *links.at(i);
            }
            k += 1;
        }
        let mut pairs = array![];
        let mut i = 0;
        while i != n {
            let (ai, _) = *sorted.at(i);
            let a = *proxies.at(ai);
            let end = cell(a.aabb.maxs.x.raw);
            let mut j = i + 1;
            while j != n {
                let (bi, start) = *sorted.at(j);
                if start > end {
                    break;
                }
                let b = *proxies.at(bi);
                if pair_allowed(a, b) && overlaps_raw(a.aabb, b.aabb) {
                    if ai < bi {
                        pairs.append((ai, bi));
                    } else {
                        pairs.append((bi, ai));
                    }
                }
                j += 1;
            }
            i += 1;
        }
        merge_sort_pairs(pairs)
    }
}
/// Rejected counting sort that materialises the sorted proxy order.
pub(crate) fn find_pairs_counting_materialized(
    proxies: Span<BroadPhaseProxy>,
) -> Array<(u32, u32)> {
    previous_counting::find_pairs_counting(proxies)
}

mod previous_grid {
    //! Fixed four-world-unit cells; signed Q32.32 coordinates are biased before floor division.
    //! Every small box occupies at most four cells. Larger boxes (including infinite grounds)
    //! use a separate list, so neither coordinate magnitude nor extent can cause unbounded loops.
    use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
    use core::num::traits::DivRem;
    use super::super::ordering::merge_sort_pairs;
    use super::super::{BroadPhaseProxy, find_pairs_tail_static_split, overlaps_raw, pair_allowed};

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
            if small(c) {
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
                                return find_pairs_tail_static_split(proxies);
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
            } else {
                large.append(i);
            }
            i += 1;
        }
        // Large-small and large-large exactly once, with no cell expansion.
        for big in large.span() {
            let i = *big;
            let a = *proxies.at(i);
            let mut j = 0;
            while j != n {
                if j != i {
                    let b = *proxies.at(j);
                    if pair_allowed(a, b)
                        && (j > i || small(cells(b)))
                        && overlaps_raw(a.aabb, b.aabb) {
                        if i < j {
                            pairs.append((i, j));
                        } else {
                            pairs.append((j, i));
                        }
                    }
                }
                j += 1;
            }
        }
        merge_sort_pairs(pairs)
    }
}
/// Rejected grid that recomputes cell ranges in the large-box pass and always falls back to
/// the tail scan when a cell is crowded.
pub(crate) fn find_pairs_grid_uncached(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    previous_grid::find_pairs_grid(proxies)
}

/// Less conservative strip occupancy limit, retained for crossover measurements.
pub(crate) fn find_pairs_strip_eight(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    super::strip::find_pairs_strip_limit(proxies, 8)
}

mod strip_wide_fallback {
    //! Shipped strip before large boxes got their own list: any box wider than two strips
    //! (a ground) abandons the strips for the tail scan.
    use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
    use super::super::ordering::merge_sort_pairs;
    use super::super::strip::cell;
    use super::super::{
        BroadPhaseProxy, crowded_fallback, find_pairs_tail_static_split, overlaps_raw, pair_allowed,
    };

    pub(crate) fn find_pairs_strip(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
        let mut heads: Felt252Dict<u32> = Default::default();
        let mut nodes = array![];
        let mut pairs = array![];
        let mut i = 0;
        for a in proxies {
            let a = *a;
            let lo = cell(a.aabb.mins.x.raw);
            let hi = cell(a.aabb.maxs.x.raw);
            if hi < lo || hi - lo > 1 {
                return find_pairs_tail_static_split(proxies);
            }
            let mut k = lo;
            while k != hi + 1 {
                let (entry, head) = heads.entry(k.into());
                nodes.append((i, head, lo));
                heads = entry.finalize(nodes.len());
                let mut link = head;
                let mut occupancy = 0;
                while link != 0 {
                    if occupancy == 2 {
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
        merge_sort_pairs(pairs)
    }
}

/// Rejected: strips that fall back to the tail scan as soon as one box is wider than two strips.
pub(crate) fn find_pairs_strip_wide_fallback(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    strip_wide_fallback::find_pairs_strip(proxies)
}

mod counting {
    //! Counting-sort into quantised min-x buckets; sweep the linked buckets directly instead of
    //! materialising another sorted array. Four-unit cells are centred on multiples of four.
    //! Bound the counting range by 4n and retain the tail scan for a single occupied x bucket.
    use core::dict::{Felt252Dict, Felt252DictEntryTrait, Felt252DictTrait};
    use super::super::ordering::merge_sort_pairs;
    use super::super::strip::cell;
    use super::super::{BroadPhaseProxy, find_pairs_tail_static_split, overlaps_raw, pair_allowed};

    #[inline(always)]
    fn check_pair(
        proxies: Span<BroadPhaseProxy>,
        a: BroadPhaseProxy,
        ai: u32,
        bi: u32,
        ref pairs: Array<(u32, u32)>,
    ) {
        let b = *proxies.at(bi);
        if pair_allowed(a, b) && overlaps_raw(a.aabb, b.aabb) {
            if ai < bi {
                pairs.append((ai, bi));
            } else {
                pairs.append((bi, ai));
            }
        }
    }

    /// Returns exact lexicographically sorted pairs; raw overlap decides closed boundaries.
    pub(crate) fn find_pairs_counting(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
        let n = proxies.len();
        if n == 0 {
            return array![];
        }
        let mut lo_raw = *proxies.at(0).aabb.mins.x.raw;
        let mut hi_raw = lo_raw;
        for p in proxies {
            let x = *p.aabb.mins.x.raw;
            if x < lo_raw {
                lo_raw = x;
            }
            if x > hi_raw {
                hi_raw = x;
            }
        }
        let lo = cell(lo_raw);
        let hi = cell(hi_raw);
        let range: u64 = (hi - lo).into();
        let count: u64 = n.into();
        if hi == lo || range > count * 4 {
            return find_pairs_tail_static_split(proxies);
        }
        let mut heads: Felt252Dict<u32> = Default::default();
        let mut links = array![];
        let mut i = 0;
        while i != n {
            let k = cell(*proxies.at(i).aabb.mins.x.raw);
            let (entry, head) = heads.entry(k.into());
            links.append(head);
            heads = entry.finalize(i + 1);
            i += 1;
        }
        let mut pairs = array![];
        let mut k = lo;
        while k != hi + 1 {
            let mut link = heads.get(k.into());
            while link != 0 {
                let ai = link - 1;
                let a = *proxies.at(ai);
                let next = *links.at(ai);
                let mut other = next;
                while other != 0 {
                    let bi = other - 1;
                    check_pair(proxies, a, ai, bi, ref pairs);
                    other = *links.at(bi);
                }
                let mut end = cell(a.aabb.maxs.x.raw);
                if end > hi {
                    end = hi;
                }
                let mut bucket = k + 1;
                while bucket <= end {
                    let mut other = heads.get(bucket.into());
                    while other != 0 {
                        let bi = other - 1;
                        check_pair(proxies, a, ai, bi, ref pairs);
                        other = *links.at(bi);
                    }
                    bucket += 1;
                }
                link = next;
            }
            k += 1;
        }
        merge_sort_pairs(pairs)
    }
}
/// Rejected counting sort over linked x buckets.
pub(crate) fn find_pairs_counting(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    counting::find_pairs_counting(proxies)
}

/// Original tail scan, including its final empty tail; retained as the before baseline.
pub(crate) fn find_pairs_tail_full(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    let n = proxies.len();
    let mut pairs = array![];
    let mut i = 0;
    while i != n {
        let a = *proxies.at(i);
        let a_aabb = a.aabb;
        let a_min_x = a_aabb.mins.x.raw;
        let a_max_x = a_aabb.maxs.x.raw;
        let a_min_y = a_aabb.mins.y.raw;
        let a_max_y = a_aabb.maxs.y.raw;
        let mut j = i + 1;
        let mut tail = proxies.slice(j, n - j);
        if a.is_static {
            while let Option::Some(value) = tail.pop_front() {
                let b = *value;
                let b_aabb = b.aabb;
                if !b.is_static
                    && overlaps_fields_flat(
                        a_min_x,
                        a_max_x,
                        a_min_y,
                        a_max_y,
                        b_aabb.mins.x.raw,
                        b_aabb.maxs.x.raw,
                        b_aabb.mins.y.raw,
                        b_aabb.maxs.y.raw,
                    ) {
                    append_pair(ref pairs, i, j);
                }
                j += 1;
            }
        } else {
            while let Option::Some(value) = tail.pop_front() {
                let b = *value;
                let b_aabb = b.aabb;
                if overlaps_fields_flat(
                    a_min_x,
                    a_max_x,
                    a_min_y,
                    a_max_y,
                    b_aabb.mins.x.raw,
                    b_aabb.maxs.x.raw,
                    b_aabb.mins.y.raw,
                    b_aabb.maxs.y.raw,
                ) {
                    append_pair(ref pairs, i, j);
                }
                j += 1;
            }
        }
        i += 1;
    }
    pairs
}

/// Rejected metered static-row append; dynamic rows remain metered in both candidates.
pub(crate) fn find_pairs_tail_metered(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    let n = proxies.len();
    let mut pairs = array![];
    if n == 0 {
        return pairs;
    }
    let end = n - 1;
    let mut i = 0;
    while i != end {
        let a = *proxies.at(i);
        let a_aabb = a.aabb;
        let a_min_x = a_aabb.mins.x.raw;
        let a_max_x = a_aabb.maxs.x.raw;
        let a_min_y = a_aabb.mins.y.raw;
        let a_max_y = a_aabb.maxs.y.raw;
        let mut j = i + 1;
        let mut tail = proxies.slice(j, n - j);
        if a.is_static {
            while let Option::Some(value) = tail.pop_front() {
                let b = *value;
                let b_aabb = b.aabb;
                if !b.is_static
                    && overlaps_fields_flat(
                        a_min_x,
                        a_max_x,
                        a_min_y,
                        a_max_y,
                        b_aabb.mins.x.raw,
                        b_aabb.maxs.x.raw,
                        b_aabb.mins.y.raw,
                        b_aabb.maxs.y.raw,
                    ) {
                    append_pair(ref pairs, i, j);
                }
                j += 1;
            }
        } else {
            while let Option::Some(value) = tail.pop_front() {
                let b = *value;
                let b_aabb = b.aabb;
                if overlaps_fields_flat(
                    a_min_x,
                    a_max_x,
                    a_min_y,
                    a_max_y,
                    b_aabb.mins.x.raw,
                    b_aabb.maxs.x.raw,
                    b_aabb.mins.y.raw,
                    b_aabb.maxs.y.raw,
                ) {
                    append_pair(ref pairs, i, j);
                }
                j += 1;
            }
        }
        i += 1;
    }
    pairs
}
