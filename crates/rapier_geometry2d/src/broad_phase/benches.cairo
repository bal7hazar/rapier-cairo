//! Each case shares the same opaque shuffled input and consumption with its setup twin.
//! Subtract setup to measure the algorithm; pair-test normalisation uses n(n-1)/2.
use core::num::traits::DivRem;
use fixed::Fixed;
use glam::Vec2;
use rapier_core::data::handle::HandleTrait;
use rapier_testing::opaque;
use crate::aabb::AabbTrait;
use super::alternatives::find_pairs_counting;
use super::grid::find_pairs_grid;
use super::{BroadPhaseProxy, find_pairs_tail_static_split};

#[inline(never)]
fn proxies(n: u32, layout: u32) -> Span<BroadPhaseProxy> {
    let mut values = array![];
    let mut i = 0;
    while i != n {
        let k = opaque((i * 17 + 5) % n);
        let (row_index, column_index) = DivRem::div_rem(k, 16);
        let x: i64 = column_index.into();
        let y: i64 = row_index.into();
        let row: i64 = k.into();
        let (x, y, w, h) = if layout == 4 {
            (row * 0x400000000 - 0x80000000, 0x6380000000, 0x100000000, 0x100000000)
        } else if layout == 5 {
            (x * 0x400000000 - 0x80000000, y * 0x400000000 - 0x80000000, 0x100000000, 0x100000000)
        } else if layout == 0 || layout == 3 {
            (x * 0x400000000, y * 0x400000000, 0x100000000, 0x100000000)
        } else if layout == 1 {
            (0, row * 0x100000000, 0x100000000, 0x100000000)
        } else {
            (0, 0, 0x100000000, 0x100000000)
        };
        let (x, y, w, h) = if layout == 3 && k == 0 {
            (-0x10000000000, -0x100000000, 0x20000000000, 0x100000000)
        } else {
            (x, y, w, h)
        };
        values
            .append(
                BroadPhaseProxy {
                    collider: HandleTrait::new(k, 0),
                    aabb: AabbTrait::new(
                        Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } },
                        Vec2 { x: Fixed { raw: x + w }, y: Fixed { raw: y + h } },
                    ),
                    is_static: layout == 3 && k == 0,
                },
            );
        i += 1;
    }
    opaque(values.span())
}

#[test]
fn gas_baseline() {}

#[test_case(32, 4)]
#[test_case(32, 5)]
#[test_case(8, 0)]
#[test_case(8, 1)]
#[test_case(8, 2)]
#[test_case(8, 3)]
#[test_case(32, 0)]
#[test_case(32, 1)]
#[test_case(32, 2)]
#[test_case(32, 3)]
#[test_case(64, 0)]
#[test_case(64, 1)]
#[test_case(64, 2)]
#[test_case(64, 3)]
#[test_case(128, 0)]
#[test_case(128, 1)]
#[test_case(128, 2)]
#[test_case(128, 3)]
#[test_case(256, 0)]
#[test_case(256, 1)]
#[test_case(256, 2)]
#[test_case(256, 3)]
fn gas_setup(n: u32, layout: u32) {
    let p = proxies(n, layout);
    assert!(opaque(p.len()) <= n * n);
}

#[test_case(32, 4)]
#[test_case(32, 5)]
#[test_case(8, 0)]
#[test_case(8, 1)]
#[test_case(8, 2)]
#[test_case(8, 3)]
#[test_case(32, 0)]
#[test_case(32, 1)]
#[test_case(32, 2)]
#[test_case(32, 3)]
#[test_case(64, 0)]
#[test_case(64, 1)]
#[test_case(64, 2)]
#[test_case(64, 3)]
#[test_case(128, 0)]
#[test_case(128, 1)]
#[test_case(128, 2)]
#[test_case(128, 3)]
#[test_case(256, 0)]
#[test_case(256, 1)]
#[test_case(256, 2)]
#[test_case(256, 3)]
fn gas_tail(n: u32, layout: u32) {
    let p = proxies(n, layout);
    assert!(opaque(super::alternatives::find_pairs_tail_full(p).len()) <= n * n);
}

#[test_case(32, 4)]
#[test_case(32, 5)]
#[test_case(8, 0)]
#[test_case(8, 1)]
#[test_case(8, 2)]
#[test_case(8, 3)]
#[test_case(32, 0)]
#[test_case(32, 1)]
#[test_case(32, 2)]
#[test_case(32, 3)]
#[test_case(64, 0)]
#[test_case(64, 1)]
#[test_case(64, 2)]
#[test_case(64, 3)]
#[test_case(128, 0)]
#[test_case(128, 1)]
#[test_case(128, 2)]
#[test_case(128, 3)]
#[test_case(256, 0)]
#[test_case(256, 1)]
#[test_case(256, 2)]
#[test_case(256, 3)]
fn gas_grid(n: u32, layout: u32) {
    let p = proxies(n, layout);
    assert!(opaque(find_pairs_grid(p).len()) <= n * n);
}

#[test_case(32, 4)]
#[test_case(32, 5)]
#[test_case(8, 0)]
#[test_case(8, 1)]
#[test_case(8, 2)]
#[test_case(8, 3)]
#[test_case(32, 0)]
#[test_case(32, 1)]
#[test_case(32, 2)]
#[test_case(32, 3)]
#[test_case(64, 0)]
#[test_case(64, 1)]
#[test_case(64, 2)]
#[test_case(64, 3)]
#[test_case(128, 0)]
#[test_case(128, 1)]
#[test_case(128, 2)]
#[test_case(128, 3)]
#[test_case(256, 0)]
#[test_case(256, 1)]
#[test_case(256, 2)]
#[test_case(256, 3)]
fn gas_counting(n: u32, layout: u32) {
    let p = proxies(n, layout);
    assert!(opaque(find_pairs_counting(p).len()) <= n * n);
}

#[test]
fn gas_cell_signed() {
    assert!(super::alternatives::cell_signed(opaque(-1)) == 0x1fffffff);
}
#[test]
fn gas_cell_felt() {
    assert!(super::alternatives::cell_grid_fixed(opaque(-1)) == 0x1fffffff);
}

#[test_case(32, 4)]
#[test_case(32, 5)]
#[test_case(8, 0)]
#[test_case(8, 1)]
#[test_case(8, 2)]
#[test_case(8, 3)]
#[test_case(32, 0)]
#[test_case(32, 1)]
#[test_case(32, 2)]
#[test_case(32, 3)]
#[test_case(64, 0)]
#[test_case(64, 1)]
#[test_case(64, 2)]
#[test_case(64, 3)]
#[test_case(128, 0)]
#[test_case(128, 1)]
#[test_case(128, 2)]
#[test_case(128, 3)]
#[test_case(256, 0)]
#[test_case(256, 1)]
#[test_case(256, 2)]
#[test_case(256, 3)]
fn gas_counting_materialized(n: u32, layout: u32) {
    let p = proxies(n, layout);
    assert!(opaque(super::alternatives::find_pairs_counting_materialized(p).len()) <= n * n);
}

#[test_case(32, 4)]
#[test_case(32, 5)]
#[test_case(8, 0)]
#[test_case(8, 1)]
#[test_case(8, 2)]
#[test_case(8, 3)]
#[test_case(32, 0)]
#[test_case(32, 1)]
#[test_case(32, 2)]
#[test_case(32, 3)]
#[test_case(64, 0)]
#[test_case(64, 1)]
#[test_case(64, 2)]
#[test_case(64, 3)]
#[test_case(128, 0)]
#[test_case(128, 1)]
#[test_case(128, 2)]
#[test_case(128, 3)]
#[test_case(256, 0)]
#[test_case(256, 1)]
#[test_case(256, 2)]
#[test_case(256, 3)]
fn gas_grid_uncached(n: u32, layout: u32) {
    let p = proxies(n, layout);
    assert!(opaque(super::alternatives::find_pairs_grid_uncached(p).len()) <= n * n);
}

// Retain gas coverage of BP's rejected algorithms without another size/layout matrix.
#[test_case(0)]
#[test_case(1)]
#[test_case(2)]
#[test_case(3)]
#[test_case(4)]
#[test_case(5)]
fn gas_legacy(candidate: u32) {
    let p = proxies(32, 0);
    let mut count = 0;
    let mut pending = true;
    while pending {
        count = match candidate {
            0 => super::alternatives::find_pairs_soa(p).len(),
            1 => super::alternatives::find_pairs_soa_metered(p).len(),
            2 => super::alternatives::find_pairs_soa_static_split(p).len(),
            3 => super::alternatives::find_pairs_sort_and_prune(p).len(),
            4 => super::alternatives::find_pairs_sorted_x_then_brute(p).len(),
            _ => super::find_pairs_brute(p).len(),
        };
        pending = false;
    }
    assert!(opaque(count) == 0);
}

#[test_case(32, 4)]
#[test_case(32, 5)]
#[test_case(8, 0)]
#[test_case(8, 1)]
#[test_case(8, 2)]
#[test_case(8, 3)]
#[test_case(32, 0)]
#[test_case(32, 1)]
#[test_case(32, 2)]
#[test_case(32, 3)]
#[test_case(64, 0)]
#[test_case(64, 1)]
#[test_case(64, 2)]
#[test_case(64, 3)]
#[test_case(128, 0)]
#[test_case(128, 1)]
#[test_case(128, 2)]
#[test_case(128, 3)]
#[test_case(256, 0)]
#[test_case(256, 1)]
#[test_case(256, 2)]
#[test_case(256, 3)]
fn gas_strip(n: u32, layout: u32) {
    let p = proxies(n, layout);
    assert!(opaque(super::strip::find_pairs_strip(p).len()) <= n * n);
}

#[test_case(0, 32, 0)]
#[test_case(0, 32, 1)]
#[test_case(0, 32, 2)]
#[test_case(0, 32, 3)]
#[test_case(0, 32, 4)]
#[test_case(0, 32, 5)]
#[test_case(0, 64, 0)]
#[test_case(0, 64, 1)]
#[test_case(0, 64, 2)]
#[test_case(0, 64, 3)]
#[test_case(1, 32, 0)]
#[test_case(1, 32, 1)]
#[test_case(1, 32, 2)]
#[test_case(1, 32, 3)]
#[test_case(1, 32, 4)]
#[test_case(1, 32, 5)]
#[test_case(1, 64, 0)]
#[test_case(1, 64, 1)]
#[test_case(1, 64, 2)]
#[test_case(1, 64, 3)]
#[test_case(2, 32, 0)]
#[test_case(2, 32, 1)]
#[test_case(2, 32, 2)]
#[test_case(2, 32, 3)]
#[test_case(2, 32, 4)]
#[test_case(2, 32, 5)]
#[test_case(3, 64, 0)]
#[test_case(3, 64, 1)]
#[test_case(3, 64, 2)]
#[test_case(3, 64, 3)]
#[test_case(3, 128, 0)]
#[test_case(3, 128, 1)]
#[test_case(3, 128, 2)]
#[test_case(3, 128, 3)]
#[test_case(3, 256, 0)]
#[test_case(3, 256, 1)]
#[test_case(3, 256, 2)]
#[test_case(3, 256, 3)]
fn gas_variant(variant: u32, n: u32, layout: u32) {
    let p = proxies(n, layout);
    let mut count = 0;
    let mut pending = true;
    while pending {
        count = match variant {
            0 => super::alternatives::find_pairs_strip_eight(p).len(),
            1 => super::alternatives::find_pairs_strip_wide_fallback(p).len(),
            2 => super::merge_sorted::strip::find_pairs_strip(p).len(),
            _ => super::merge_sorted::grid::find_pairs_grid(p).len(),
        };
        pending = false;
    }
    assert!(opaque(count) <= n * n);
}

#[test_case(32, 4)]
#[test_case(32, 5)]
#[test_case(8, 0)]
#[test_case(8, 1)]
#[test_case(8, 2)]
#[test_case(8, 3)]
#[test_case(32, 0)]
#[test_case(32, 1)]
#[test_case(32, 2)]
#[test_case(32, 3)]
#[test_case(64, 0)]
#[test_case(64, 1)]
#[test_case(64, 2)]
#[test_case(64, 3)]
#[test_case(128, 0)]
#[test_case(128, 1)]
#[test_case(128, 2)]
#[test_case(128, 3)]
#[test_case(256, 0)]
#[test_case(256, 1)]
#[test_case(256, 2)]
#[test_case(256, 3)]
fn gas_shipped(n: u32, layout: u32) {
    let p = proxies(n, layout);
    assert!(opaque(super::find_pairs(p).len()) <= n * n);
}

#[test_case(8)]
#[test_case(32)]
#[test_case(64)]
#[test_case(128)]
#[test_case(256)]
fn gas_dense_brute(n: u32) {
    let p = proxies(n, 2);
    assert!(opaque(super::find_pairs_brute(p).len()) <= n * n);
}

#[test_case(32, 4)]
#[test_case(32, 5)]
#[test_case(8, 0)]
#[test_case(8, 1)]
#[test_case(8, 2)]
#[test_case(8, 3)]
#[test_case(32, 0)]
#[test_case(32, 1)]
#[test_case(32, 2)]
#[test_case(32, 3)]
#[test_case(64, 0)]
#[test_case(64, 1)]
#[test_case(64, 2)]
#[test_case(64, 3)]
#[test_case(128, 0)]
#[test_case(128, 1)]
#[test_case(128, 2)]
#[test_case(128, 3)]
#[test_case(256, 0)]
#[test_case(256, 1)]
#[test_case(256, 2)]
#[test_case(256, 3)]
fn gas_tail_trimmed(n: u32, layout: u32) {
    let p = proxies(n, layout);
    assert!(opaque(find_pairs_tail_static_split(p).len()) <= n * n);
}

#[test_case(32, 4)]
#[test_case(32, 5)]
#[test_case(8, 0)]
#[test_case(8, 1)]
#[test_case(8, 2)]
#[test_case(8, 3)]
#[test_case(32, 0)]
#[test_case(32, 1)]
#[test_case(32, 2)]
#[test_case(32, 3)]
#[test_case(64, 0)]
#[test_case(64, 1)]
#[test_case(64, 2)]
#[test_case(64, 3)]
#[test_case(128, 0)]
#[test_case(128, 1)]
#[test_case(128, 2)]
#[test_case(128, 3)]
#[test_case(256, 0)]
#[test_case(256, 1)]
#[test_case(256, 2)]
#[test_case(256, 3)]
fn gas_static_metered(n: u32, layout: u32) {
    let p = proxies(n, layout);
    assert!(opaque(super::alternatives::find_pairs_tail_metered(p).len()) <= n * n);
}
