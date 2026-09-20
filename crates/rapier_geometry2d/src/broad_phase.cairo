//! Stateless 2D broad phase over AABBs.
//!
//! `find_pairs` returns index pairs `(i, j)` with `i < j`, in ascending lexicographic order.
//! Static-static pairs are skipped; group filtering and add/remove pair events are deferred to the
//! narrow phase packages. The shipped implementation is the measured brute/static-skip candidate,
//! while sort-and-prune remains in `alternatives` for gas probes and equivalence tests.

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
    find_pairs_brute(proxies)
}

#[inline(always)]
fn overlaps_raw(a: Aabb, b: Aabb) -> bool {
    a.mins.x.raw <= b.maxs.x.raw
        && b.mins.x.raw <= a.maxs.x.raw
        && a.mins.y.raw <= b.maxs.y.raw
        && b.mins.y.raw <= a.maxs.y.raw
}

#[inline(always)]
fn y_overlaps_raw(a: Aabb, b: Aabb) -> bool {
    a.mins.y.raw <= b.maxs.y.raw && b.mins.y.raw <= a.maxs.y.raw
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

#[cfg(test)]
pub mod alternatives {
    use super::{BroadPhaseProxy, pair_allowed, y_overlaps_raw};

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

    fn insert_sorted_proxy(ref sorted: Array<SortProxy>, item: SortProxy) {
        let mut pos = 0;
        let len = sorted.len();
        while pos != len && before(*sorted.at(pos), item) {
            pos += 1;
        }
        let mut head = sorted.span().slice(0, pos);
        let mut tail = sorted.span().slice(pos, len - pos);
        let mut rebuilt = array![];
        while let Option::Some(value) = head.pop_front() {
            rebuilt.append(*value);
        }
        rebuilt.append(item);
        while let Option::Some(value) = tail.pop_front() {
            rebuilt.append(*value);
        }
        sorted = rebuilt;
    }

    fn insert_sorted_pair(ref pairs: Array<(u32, u32)>, pair: (u32, u32)) {
        let len = pairs.len();
        if len == 0 || pair_before(*pairs.at(len - 1), pair) {
            pairs.append(pair);
            return;
        }
        let mut pos = 0;
        while pos != len && pair_before(*pairs.at(pos), pair) {
            pos += 1;
        }
        let mut head = pairs.span().slice(0, pos);
        let mut tail = pairs.span().slice(pos, len - pos);
        let mut rebuilt = array![];
        while let Option::Some(value) = head.pop_front() {
            rebuilt.append(*value);
        }
        rebuilt.append(pair);
        while let Option::Some(value) = tail.pop_front() {
            rebuilt.append(*value);
        }
        pairs = rebuilt;
    }

    fn sorted_by_min_x(proxies: Span<BroadPhaseProxy>) -> Array<SortProxy> {
        let mut sorted = array![];
        let n = proxies.len();
        let mut i = 0;
        while i != n {
            insert_sorted_proxy(ref sorted, SortProxy { index: i, proxy: *proxies.at(i) });
            i += 1;
        }
        sorted
    }

    /// Sort-and-prune candidate: stable insertion sort by `mins.x`, sweep x, test y only.
    ///
    /// Output pairs are insertion-sorted back into the public `(i, j)` order.
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
                    if pair_allowed(a.proxy, b.proxy)
                        && y_overlaps_raw(a.proxy.aabb, b.proxy.aabb) {
                        if a.index < b.index {
                            insert_sorted_pair(ref pairs, (a.index, b.index));
                        } else {
                            insert_sorted_pair(ref pairs, (b.index, a.index));
                        }
                    }
                    j += 1;
                }
            }
            i += 1;
        }
        pairs
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
    use super::{BroadPhaseProxy, alternatives, find_pairs, find_pairs_brute};

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
        let base = if layout == 2 {
            raw_i * 0x300000000
        } else if layout == 3 {
            0
        } else {
            raw_i * 0x80000000
        };
        let size = if layout == 2 {
            0x40000000
        } else {
            0x100000000
        };
        let y = if layout == 2 {
            raw_i * 0x200000000
        } else {
            0
        };
        proxy(
            i,
            v_raw(base, y),
            v_raw(base + size, y + size),
            if layout == 1 {
                i % 4 != 0
            } else {
                false
            },
        )
    }

    #[inline(never)]
    fn proxies(n: u32, layout: u32) -> Array<BroadPhaseProxy> {
        let mut values = array![];
        let mut i = 0;
        while i != n {
            values.append(layout_proxy(opaque(i), layout));
            i += 1;
        }
        values
    }

    fn assert_candidates(proxies: Span<BroadPhaseProxy>) {
        let brute = find_pairs_brute(proxies);
        let sap = alternatives::find_pairs_sort_and_prune(proxies);
        assert_same_pairs(sap.span(), brute.span());
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
        for layout in array![0_u32, 1, 2, 3].span() {
            assert_candidates(proxies(12, *layout).span());
        }
    }

    #[test]
    fn test_golden_overlap_sets() {
        for case in aabb_overlap::cases() {
            let proxies = from_case(*case);
            let expected = expected_dynamic_pairs(*case);
            assert_same_pairs(find_pairs(proxies.span()).span(), expected.span());
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

    fn cap(n: u32) -> u32 {
        (n * (n - 1)) / 2
    }

    #[test]
    fn gas_brute_8_all_dynamic() {
        let p = proxies(8, 0);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(8));
    }
    #[test]
    fn gas_brute_16_all_dynamic() {
        let p = proxies(16, 0);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(16));
    }
    #[test]
    fn gas_brute_32_all_dynamic() {
        let p = proxies(32, 0);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(32));
    }
    #[test]
    fn gas_brute_64_all_dynamic() {
        let p = proxies(64, 0);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(64));
    }
    #[test]
    fn gas_brute_8_static75() {
        let p = proxies(8, 1);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(8));
    }
    #[test]
    fn gas_brute_16_static75() {
        let p = proxies(16, 1);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(16));
    }
    #[test]
    fn gas_brute_32_static75() {
        let p = proxies(32, 1);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(32));
    }
    #[test]
    fn gas_brute_64_static75() {
        let p = proxies(64, 1);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(64));
    }
    #[test]
    fn gas_brute_8_sparse() {
        let p = proxies(8, 2);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(8));
    }
    #[test]
    fn gas_brute_16_sparse() {
        let p = proxies(16, 2);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(16));
    }
    #[test]
    fn gas_brute_32_sparse() {
        let p = proxies(32, 2);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(32));
    }
    #[test]
    fn gas_brute_64_sparse() {
        let p = proxies(64, 2);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(64));
    }
    #[test]
    fn gas_brute_8_dense() {
        let p = proxies(8, 3);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(8));
    }
    #[test]
    fn gas_brute_16_dense() {
        let p = proxies(16, 3);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(16));
    }
    #[test]
    fn gas_brute_32_dense() {
        let p = proxies(32, 3);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(32));
    }
    #[test]
    fn gas_brute_64_dense() {
        let p = proxies(64, 3);
        assert!(find_pairs_brute(opaque(p.span())).len() <= cap(64));
    }
    #[test]
    fn gas_sap_8_all_dynamic() {
        let p = proxies(8, 0);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(8));
    }
    #[test]
    fn gas_sap_16_all_dynamic() {
        let p = proxies(16, 0);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(16));
    }
    #[test]
    fn gas_sap_32_all_dynamic() {
        let p = proxies(32, 0);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(32));
    }
    #[test]
    fn gas_sap_64_all_dynamic() {
        let p = proxies(64, 0);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(64));
    }
    #[test]
    fn gas_sap_8_static75() {
        let p = proxies(8, 1);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(8));
    }
    #[test]
    fn gas_sap_16_static75() {
        let p = proxies(16, 1);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(16));
    }
    #[test]
    fn gas_sap_32_static75() {
        let p = proxies(32, 1);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(32));
    }
    #[test]
    fn gas_sap_64_static75() {
        let p = proxies(64, 1);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(64));
    }
    #[test]
    fn gas_sap_8_sparse() {
        let p = proxies(8, 2);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(8));
    }
    #[test]
    fn gas_sap_16_sparse() {
        let p = proxies(16, 2);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(16));
    }
    #[test]
    fn gas_sap_32_sparse() {
        let p = proxies(32, 2);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(32));
    }
    #[test]
    fn gas_sap_64_sparse() {
        let p = proxies(64, 2);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(64));
    }
    #[test]
    fn gas_sap_8_dense() {
        let p = proxies(8, 3);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(8));
    }
    #[test]
    fn gas_sap_16_dense() {
        let p = proxies(16, 3);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(16));
    }
    #[test]
    fn gas_sap_32_dense() {
        let p = proxies(32, 3);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(32));
    }
    #[test]
    #[available_gas(l2_gas: 2000000000)]
    fn gas_sap_64_dense() {
        let p = proxies(64, 3);
        assert!(alternatives::find_pairs_sort_and_prune(opaque(p.span())).len() <= cap(64));
    }
}
