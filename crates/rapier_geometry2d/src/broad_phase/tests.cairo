use fixed::{Fixed, ONE};
use glam::Vec2;
use rapier_core::data::handle::HandleTrait;
use rapier_golden::aabb_overlap;
use rapier_golden::types::AabbOverlapCase;
use rapier_testing::opaque;
use crate::aabb::AabbTrait;
use super::alternatives::{find_pairs_soa, find_pairs_soa_metered, find_pairs_soa_static_split};
use super::{BroadPhaseProxy, alternatives, find_pairs, find_pairs_tail_static_split};

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
                proxy(i, v_raw(*b.mins.x, *b.mins.y), v_raw(*b.maxs.x, *b.maxs.y), *b.is_static),
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
    assert_same_pairs(alternatives::find_pairs_tail_metered(proxies).span(), brute.span());
    assert_same_pairs(alternatives::find_pairs_tail_full(proxies).span(), brute.span());
    assert_same_pairs(alternatives::find_pairs_strip_eight(proxies).span(), brute.span());
    assert_same_pairs(alternatives::find_pairs_strip_wide_fallback(proxies).span(), brute.span());
    assert_same_pairs(super::strip::find_pairs_strip(proxies).span(), brute.span());
    assert_same_pairs(alternatives::find_pairs_grid_uncached(proxies).span(), brute.span());
    assert_same_pairs(alternatives::find_pairs_counting_materialized(proxies).span(), brute.span());
    assert_same_pairs(super::grid::find_pairs_grid(proxies).span(), brute.span());
    assert_same_pairs(super::merge_sorted::grid::find_pairs_grid(proxies).span(), brute.span());
    assert_same_pairs(super::merge_sorted::strip::find_pairs_strip(proxies).span(), brute.span());
    assert_same_pairs(super::alternatives::find_pairs_counting(proxies).span(), brute.span());
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
    assert_candidates([].span());
    assert_candidates(array![layout_proxy(0, 0)].span());
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
        assert_candidates(proxies.span());
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
    while i != 40 {
        let x: i64 = ((s + i * 17) % 23).into();
        let y: i64 = ((s + i * 31) % 19).into();
        let x = (x - 11) * 0x100000000;
        let y = (y - 9) * 0x100000000;
        values
            .append(
                proxy(
                    i,
                    v_raw(x, y),
                    v_raw(x + ((s + i) % 8).into() * 0x100000000, y + 0x100000000),
                    i % 5 == 0,
                ),
            );
        i += 1;
    }
    values
        .append(
            proxy(
                90,
                v_raw(-0x8000000000000000, -0x8000000000000000),
                v_raw(0x7fffffffffffffff, 0x7fffffffffffffff),
                true,
            ),
        );
    values
        .append(
            proxy(
                91,
                v_raw(-0x8000000000000000, -0x8000000000000000),
                v_raw(0x7fffffffffffffff, 0x7fffffffffffffff),
                true,
            ),
        );
    assert_candidates(values.span());
}

#[test]
fn test_grid_boundaries_extremes_and_duplicate_cells() {
    let mut values = array![];
    for raw in array![
        -0x8000000000000000_i64, -0x400000001, -0x400000000, -1, 0, 0x3ffffffff, 0x400000000,
        0x7fffffffffffffff,
    ]
        .span() {
        assert_eq!(alternatives::cell_grid_fixed(*raw), alternatives::cell_signed(*raw));
        values.append(proxy(0, v_raw(*raw, *raw), v_raw(*raw, *raw), false));
        values.append(proxy(0, v_raw(*raw, *raw), v_raw(*raw, *raw), true));
    }
    values.append(proxy(0, v_raw(-1, -1), v_raw(1, 1), false));
    values.append(proxy(0, v_raw(-1, -1), v_raw(1, 1), false));
    assert_candidates(values.span());
}

#[test]
fn test_large_dispatch_and_large_static() {
    for n in array![32_u32, 64, 128].span() {
        for layout in array![0_u32, 1].span() {
            let mut values = proxies(*n, *layout);
            values
                .append(
                    proxy(
                        999,
                        v_raw(-0x8000000000000000, -0x8000000000000000),
                        v_raw(0x7fffffffffffffff, 0x7fffffffffffffff),
                        true,
                    ),
                );
            assert_same_pairs(
                find_pairs(values.span()).span(), super::find_pairs_brute(values.span()).span(),
            );
            // Same world with the ground first (arena order of a scene built ground-first).
            let mut ground_first = array![*values.at(values.len() - 1)];
            ground_first.append_span(values.span().slice(0, values.len() - 1));
            assert_same_pairs(
                find_pairs(ground_first.span()).span(),
                super::find_pairs_brute(ground_first.span()).span(),
            );
        }
    }
    let values = proxies(64, 2);
    assert_same_pairs(
        find_pairs(values.span()).span(), super::find_pairs_brute(values.span()).span(),
    );
}
