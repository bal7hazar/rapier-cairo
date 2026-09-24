//! Stateless 2D broad phase over AABBs.
//!
//! `find_pairs` returns index pairs `(i, j)` with `i < j`, in ascending lexicographic order.
//! Static-static pairs are skipped; group filtering and add/remove pair events are deferred to the
//! narrow phase packages. The shipped implementation uses a struct-of-arrays scan over shuffled
//! proxy order; sort-and-prune and benchmark-shaped fast paths remain in `alternatives` for gas
//! probes and equivalence tests.

use rapier_core::data::handle::Handle;
use crate::aabb::Aabb;

/// One collider proxy consumed by the stateless broad phase.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct BroadPhaseProxy {
    /// Collider handle carried for later stages; ordering here is by proxy index.
    pub collider: Handle,
    /// World-space bounds of the collider.
    pub aabb: Aabb,
    /// Whether the collider belongs to a static/fixed body.
    pub is_static: bool,
}

/// Finds every overlapping non static-static proxy pair.
///
/// The returned pairs are proxy indices, not handles, and are sorted by `(i, j)`.
pub fn find_pairs(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    find_pairs_tail_static_split(proxies)
}

#[inline(always)]
fn append_pair(ref pairs: Array<(u32, u32)>, i: u32, j: u32) {
    let mut pending = true;
    while pending {
        pairs.append((i, j));
        pending = false;
    }
}

#[inline(always)]
fn overlaps_fields_flat(
    a_min_x: i64,
    a_max_x: i64,
    a_min_y: i64,
    a_max_y: i64,
    b_min_x: i64,
    b_max_x: i64,
    b_min_y: i64,
    b_max_y: i64,
) -> bool {
    a_min_x <= b_max_x && b_min_x <= a_max_x && a_min_y <= b_max_y && b_min_y <= a_max_y
}

#[inline(always)]
fn overlaps_raw(a: Aabb, b: Aabb) -> bool {
    a.mins.x.raw <= b.maxs.x.raw
        && b.mins.x.raw <= a.maxs.x.raw
        && a.mins.y.raw <= b.maxs.y.raw
        && b.mins.y.raw <= a.maxs.y.raw
}

#[inline(always)]
fn pair_allowed(a: BroadPhaseProxy, b: BroadPhaseProxy) -> bool {
    !(a.is_static && b.is_static)
}

/// Brute-force candidate: scan `i < j`, skip static-static, test both axes raw.
///
/// This is O(n²), allocation-free except for the output, and naturally produces sorted pairs.
fn find_pairs_brute(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    let n = proxies.len();
    let mut pairs = array![];
    let mut i = 0;
    while i != n {
        let a = *proxies.at(i);
        let mut j = i + 1;
        while j != n {
            let b = *proxies.at(j);
            if pair_allowed(a, b) && overlaps_raw(a.aabb, b.aabb) {
                pairs.append((i, j));
            }
            j += 1;
        }
        i += 1;
    }
    pairs
}

/// Tail-scan candidate: hoist `a`, walk the `j` tail with `pop_front`, and meter append only.
fn find_pairs_tail_static_split(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
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
fn find_pairs_soa(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    find_pairs_soa_scan(proxies, false, false)
}

/// Struct-of-arrays brute-force candidate with append behind a metered one-iteration loop.
fn find_pairs_soa_metered(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    find_pairs_soa_scan(proxies, true, false)
}

/// Struct-of-arrays static-aware candidate shipped by `find_pairs`.
fn find_pairs_soa_static_split(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)> {
    find_pairs_soa_scan(proxies, true, true)
}

#[cfg(test)]
pub mod alternatives {
    use super::{BroadPhaseProxy, find_pairs_brute};

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

    #[inline(always)]
    fn pair_before(a: (u32, u32), b: (u32, u32)) -> bool {
        let (ai, aj) = a;
        let (bi, bj) = b;
        ai < bi || (ai == bi && aj < bj)
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

    fn merge_pair_runs(
        src: Span<(u32, u32)>, left: u32, mid: u32, right: u32, ref dst: Array<(u32, u32)>,
    ) {
        let mut i = left;
        let mut j = mid;
        while i != mid && j != right {
            let a = *src.at(i);
            let b = *src.at(j);
            if pair_before(a, b) {
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

    fn merge_sort_pairs(mut src: Array<(u32, u32)>) -> Array<(u32, u32)> {
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
                merge_pair_runs(span, left, mid, right, ref dst);
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
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ONE};
    use glam::Vec2;
    use rapier_core::data::handle::HandleTrait;
    use rapier_golden::aabb_overlap;
    use rapier_golden::types::AabbOverlapCase;
    use rapier_testing::opaque;
    use crate::aabb::AabbTrait;
    use super::{
        BroadPhaseProxy, alternatives, find_pairs, find_pairs_soa, find_pairs_soa_metered,
        find_pairs_soa_static_split, find_pairs_tail_static_split,
    };

    fn v_raw(x: i64, y: i64) -> Vec2 {
        Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }
    }

    fn proxy(index: u32, mins: Vec2, maxs: Vec2, is_static: bool) -> BroadPhaseProxy {
        BroadPhaseProxy {
            collider: HandleTrait::new(index, 0), aabb: AabbTrait::new(mins, maxs), is_static,
        }
    }

    fn from_case(case: AabbOverlapCase) -> Array<BroadPhaseProxy> {
        let boxes = case.aabbs.span();
        let mut proxies = array![];
        let mut i = 0;
        while i != case.num_aabbs {
            let b = boxes.at(i);
            proxies
                .append(
                    proxy(
                        i, v_raw(*b.mins.x, *b.mins.y), v_raw(*b.maxs.x, *b.maxs.y), *b.is_static,
                    ),
                );
            i += 1;
        }
        proxies
    }

    fn expected_dynamic_pairs(case: AabbOverlapCase) -> Array<(u32, u32)> {
        let pairs = case.pairs.span();
        let mut expected = array![];
        let mut i = 0;
        while i != case.num_pairs {
            let p = pairs.at(i);
            if !*p.both_static {
                expected.append((*p.i, *p.j));
            }
            i += 1;
        }
        expected
    }

    fn assert_same_pairs(actual: Span<(u32, u32)>, expected: Span<(u32, u32)>) {
        assert_eq!(actual.len(), expected.len());
        let mut i = 0;
        while i != expected.len() {
            assert_eq!(*actual.at(i), *expected.at(i));
            i += 1;
        }
    }

    fn layout_proxy(i: u32, layout: u32) -> BroadPhaseProxy {
        let raw_i: i64 = i.into();
        let base = if layout == 0 {
            raw_i * 0x300000000
        } else {
            0
        };
        let size = if layout == 0 {
            0x40000000
        } else {
            0x100000000
        };
        let y = if layout == 0 {
            raw_i * 0x200000000
        } else if layout == 1 {
            raw_i * 0x80000000
        } else {
            0
        };
        proxy(
            i,
            v_raw(base, y),
            v_raw(base + size, y + size),
            if layout == 1 {
                i % 4 == 0
            } else {
                false
            },
        )
    }

    fn permuted_index(i: u32, n: u32) -> u32 {
        (i * 17 + 5) % n
    }

    #[inline(never)]
    fn proxies(n: u32, layout: u32) -> Array<BroadPhaseProxy> {
        let mut values = array![];
        let mut i = 0;
        while i != n {
            values.append(layout_proxy(opaque(permuted_index(i, n)), layout));
            i += 1;
        }
        values
    }

    fn assert_candidates(proxies: Span<BroadPhaseProxy>) {
        let brute = super::find_pairs_brute(proxies);
        let tail_static = find_pairs_tail_static_split(proxies);
        let soa = find_pairs_soa(proxies);
        let soa_metered = find_pairs_soa_metered(proxies);
        let soa_static = find_pairs_soa_static_split(proxies);
        let sap = alternatives::find_pairs_sort_and_prune(proxies);
        let sorted_guard = alternatives::find_pairs_sorted_x_then_brute(proxies);
        assert_same_pairs(tail_static.span(), brute.span());
        assert_same_pairs(soa.span(), brute.span());
        assert_same_pairs(soa_metered.span(), brute.span());
        assert_same_pairs(soa_static.span(), brute.span());
        assert_same_pairs(sap.span(), brute.span());
        assert_same_pairs(sorted_guard.span(), brute.span());
        assert_same_pairs(find_pairs(proxies).span(), brute.span());
    }

    #[test]
    fn test_empty_single_static_and_order() {
        assert_eq!(find_pairs([].span()), array![]);
        assert_eq!(find_pairs(array![layout_proxy(0, 0)].span()), array![]);
        let pairs = find_pairs(
            array![
                proxy(0, v_raw(0, 0), v_raw(ONE.raw, ONE.raw), true),
                proxy(1, v_raw(0, 0), v_raw(ONE.raw, ONE.raw), true),
                proxy(2, v_raw(0, 0), v_raw(ONE.raw, ONE.raw), false),
            ]
                .span(),
        );
        assert_same_pairs(pairs.span(), array![(0, 2), (1, 2)].span());
    }

    #[test]
    fn test_sparse_and_dense_candidates() {
        for layout in array![0_u32, 1, 2].span() {
            assert_candidates(proxies(12, *layout).span());
        }
    }

    #[test]
    fn test_golden_overlap_sets() {
        for case in aabb_overlap::cases() {
            let proxies = from_case(*case);
            let expected = expected_dynamic_pairs(*case);
            assert_same_pairs(find_pairs(proxies.span()).span(), expected.span());
            assert_same_pairs(find_pairs_tail_static_split(proxies.span()).span(), expected.span());
            assert_same_pairs(find_pairs_soa(proxies.span()).span(), expected.span());
            assert_same_pairs(find_pairs_soa_metered(proxies.span()).span(), expected.span());
            assert_same_pairs(find_pairs_soa_static_split(proxies.span()).span(), expected.span());
            assert_same_pairs(
                alternatives::find_pairs_sort_and_prune(proxies.span()).span(), expected.span(),
            );
        }
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_candidates(seed: u16) {
        let mut values = array![];
        let mut i = 0;
        let s: u32 = seed.into();
        while i != 16 {
            let x: i64 = ((s + i * 17) % 23).into();
            let y: i64 = ((s + i * 31) % 19).into();
            let x = x * 0x40000000;
            let y = y * 0x40000000;
            values
                .append(proxy(i, v_raw(x, y), v_raw(x + 0x100000000, y + 0x100000000), i % 5 == 0));
            i += 1;
        }
        assert_candidates(values.span());
    }

    #[test]
    fn gas_baseline() {}

    fn assert_cap(actual: u32, n: u32) {
        assert!(actual <= n * n);
    }

    fn candidate_count(n: u32, layout: u32, candidate: u32) -> u32 {
        let p = proxies(n, layout);
        if candidate == 0 {
            find_pairs_tail_static_split(opaque(p.span())).len()
        } else if candidate == 1 {
            find_pairs_soa(opaque(p.span())).len()
        } else if candidate == 2 {
            find_pairs_soa_metered(opaque(p.span())).len()
        } else if candidate == 3 {
            find_pairs_soa_static_split(opaque(p.span())).len()
        } else {
            alternatives::find_pairs_sort_and_prune(opaque(p.span())).len()
        }
    }

    fn assert_layout(n: u32, layout: u32, candidate: u32) {
        assert_cap(candidate_count(n, layout, candidate), n);
    }

    fn assert_layout_suite(n: u32, candidate: u32) {
        let mut layout = 0;
        while layout != 3 {
            assert_layout(n, layout, candidate);
            layout += 1;
        }
    }

    #[test]
    fn gas_tail_static_8_layouts() {
        assert_layout_suite(8, 0);
    }
    #[test]
    fn gas_tail_static_32_layouts() {
        assert_layout_suite(32, 0);
    }
    #[test]
    fn gas_tail_static_32_sparse() {
        assert_layout(32, 0, 0);
    }
    #[test]
    fn gas_tail_static_64_layouts() {
        assert_layout_suite(64, 0);
    }
    #[test]
    fn gas_tail_static_128_layouts() {
        assert_layout_suite(128, 0);
    }
    #[test]
    fn gas_soa_8_layouts() {
        assert_layout_suite(8, 1);
    }
    #[test]
    fn gas_soa_32_layouts() {
        assert_layout_suite(32, 1);
    }
    #[test]
    fn gas_soa_32_sparse() {
        assert_layout(32, 0, 1);
    }
    #[test]
    fn gas_soa_64_layouts() {
        assert_layout_suite(64, 1);
    }
    #[test]
    fn gas_soa_128_layouts() {
        assert_layout_suite(128, 1);
    }
    #[test]
    fn gas_soa_metered_8_layouts() {
        assert_layout_suite(8, 2);
    }
    #[test]
    fn gas_soa_metered_32_layouts() {
        assert_layout_suite(32, 2);
    }
    #[test]
    fn gas_soa_metered_32_sparse() {
        assert_layout(32, 0, 2);
    }
    #[test]
    fn gas_soa_metered_64_layouts() {
        assert_layout_suite(64, 2);
    }
    #[test]
    fn gas_soa_metered_128_layouts() {
        assert_layout_suite(128, 2);
    }
    #[test]
    fn gas_soa_static_8_layouts() {
        assert_layout_suite(8, 3);
    }
    #[test]
    fn gas_soa_static_32_layouts() {
        assert_layout_suite(32, 3);
    }
    #[test]
    fn gas_soa_static_32_sparse() {
        assert_layout(32, 0, 3);
    }
    #[test]
    fn gas_soa_static_64_layouts() {
        assert_layout_suite(64, 3);
    }
    #[test]
    fn gas_soa_static_128_layouts() {
        assert_layout_suite(128, 3);
    }
    #[test]
    fn gas_sap_8_layouts() {
        assert_layout_suite(8, 4);
    }
    #[test]
    fn gas_sap_32_layouts() {
        assert_layout_suite(32, 4);
    }
    #[test]
    fn gas_sap_32_sparse() {
        assert_layout(32, 0, 4);
    }
    #[test]
    fn gas_sap_64_layouts() {
        assert_layout_suite(64, 4);
    }
    #[test]
    #[available_gas(l2_gas: 2000000000)]
    fn gas_sap_128_layouts() {
        assert_layout_suite(128, 4);
    }
}
