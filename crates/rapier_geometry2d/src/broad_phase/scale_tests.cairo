use fixed::Fixed;
use glam::Vec2;
use rapier_core::data::handle::HandleTrait;
use crate::aabb::AabbTrait;
use super::grid::{find_pairs_grid, find_pairs_grid_sized};
use super::scale::{cell_size, median5};
use super::strip::{find_pairs_strip, find_pairs_strip_sized};
use super::{BroadPhaseProxy, find_pairs, find_pairs_brute};

fn proxy_raw(index: u32, x0: i64, y0: i64, x1: i64, y1: i64, is_static: bool) -> BroadPhaseProxy {
    BroadPhaseProxy {
        collider: HandleTrait::new(index, 0),
        aabb: AabbTrait::new(
            Vec2 { x: Fixed { raw: x0 }, y: Fixed { raw: y0 } },
            Vec2 { x: Fixed { raw: x1 }, y: Fixed { raw: y1 } },
        ),
        is_static,
    }
}

fn assert_same(actual: Span<(u32, u32)>, expected: Span<(u32, u32)>) {
    assert_eq!(actual.len(), expected.len());
    let mut i = 0;
    while i != expected.len() {
        assert_eq!(*actual.at(i), *expected.at(i));
        i += 1;
    }
}

fn size_of(proxies: Span<BroadPhaseProxy>) -> u64 {
    cell_size(proxies).into()
}

/// `count` identical square boxes of side `side` raw.
fn squares(count: u32, side: i64) -> Array<BroadPhaseProxy> {
    let mut values = array![];
    let mut i = 0;
    while i != count {
        let x: i64 = i.into();
        values.append(proxy_raw(i, x * side, 0, x * side + side, side, false));
        i += 1;
    }
    values
}

#[test]
fn test_median5_of_every_multiset() {
    let mut code = 0_u32;
    while code != 3125 {
        let a: u64 = (code % 5).into();
        let b: u64 = (code / 5 % 5).into();
        let c: u64 = (code / 25 % 5).into();
        let d: u64 = (code / 125 % 5).into();
        let e: u64 = (code / 625).into();
        let values = array![a, b, c, d, e];
        let mut expected = 99;
        for x in values.span() {
            let mut below = 0_u32;
            let mut at_most = 0_u32;
            for y in values.span() {
                if *y < *x {
                    below += 1;
                }
                if *y <= *x {
                    at_most += 1;
                }
            }
            if below <= 2 && at_most >= 3 {
                expected = *x;
            }
        }
        assert_eq!(median5(a, b, c, d, e), expected);
        code += 1;
    }
}

#[test]
fn test_cell_size_follows_extent_and_clamps() {
    // A box of side `L` (a power of two) gets a cell of `4 L`, at every scale.
    for shift in array![26_u32, 28, 30, 32, 34, 36, 40, 44].span() {
        let side: i64 = core::num::traits::Pow::pow(2_i64, *shift);
        assert_eq!(
            size_of(squares(40, side).span()), 4 * core::num::traits::Pow::pow(2_u64, *shift),
        );
    }
    // The unit box keeps BG's four-unit cell, also below five proxies.
    assert_eq!(size_of(squares(40, 0x100000000).span()), 0x400000000);
    assert_eq!(size_of(squares(3, 0x100000000_i64 * 64).span()), 0x400000000);
    // 1.5 units: floor(log2(3)) + 1 = 2 -> 2^33 raw, i.e. two units.
    assert_eq!(size_of(squares(40, 0x180000000).span()), 0x400000000);
    // Clamps: points and unit-less boxes, then a box beyond the largest cell, then inverted boxes.
    assert_eq!(size_of(squares(40, 0).span()), 0x100000);
    assert_eq!(size_of(squares(40, 0x200000000000000).span()), 0x8000000000000);
    let inverted = proxy_raw(0, 8, 8, 0, 0, false);
    assert_eq!(
        size_of(array![inverted, inverted, inverted, inverted, inverted].span()), 0x8000000000000,
    );
}

#[test]
fn test_cell_size_ignores_outliers() {
    // Two grounds among the five sampled proxies do not move the median.
    let mut values = squares(60, 0x100000000);
    let ground = proxy_raw(0, -0x100000000000, -0x100000000, 0x100000000000, 0, true);
    let n = values.len();
    let mut with_grounds = array![];
    let mut i = 0;
    while i != n {
        with_grounds.append(if i == n / 2 || i == n * 3 / 10 {
            ground
        } else {
            *values.at(i)
        });
        i += 1;
    }
    assert_eq!(size_of(with_grounds.span()), 0x400000000);
    values.append(ground);
}

fn extremes() -> Array<BroadPhaseProxy> {
    let mut values = array![];
    for raw in array![
        -0x8000000000000000_i64, -0x400000001, -0x400000000, -1, 0, 0x3ffffffff, 0x400000000,
        0x7fffffffffffffff,
    ]
        .span() {
        values.append(proxy_raw(0, *raw, *raw, *raw, *raw, false));
        values.append(proxy_raw(0, *raw, *raw, *raw, *raw, true));
    }
    values.append(proxy_raw(0, -1, -1, 1, 1, false));
    values.append(proxy_raw(0, -1, -1, 1, 1, false));
    values
        .append(
            proxy_raw(
                0,
                -0x8000000000000000,
                -0x8000000000000000,
                0x7fffffffffffffff,
                0x7fffffffffffffff,
                true,
            ),
        );
    values
}

/// The cell size only changes cost: any positive even size gives the exact pairs.
#[test]
fn test_any_cell_size_is_exact() {
    let mut worlds = array![extremes(), squares(70, 0x100000000), squares(70, 0x30000000)];
    let mut dense = array![];
    let mut i = 0;
    while i != 70 {
        dense.append(proxy_raw(i, 0, 0, 0x100000000, 0x100000000, i % 7 == 0));
        i += 1;
    }
    worlds.append(dense);
    for world in worlds.span() {
        let brute = find_pairs_brute(world.span());
        for size in array![0x100000_u64, 0x1234568, 0x400000000, 0x300000000, 0x8000000000000]
            .span() {
            let size: NonZero<u64> = (*size).try_into().unwrap();
            assert_same(find_pairs_grid_sized(world.span(), size).span(), brute.span());
            assert_same(find_pairs_strip_sized(world.span(), size).span(), brute.span());
        }
        assert_same(find_pairs(world.span()).span(), brute.span());
    }
}

/// A random world of 33..72 proxies (strip below 64, grid from 64) at a scale of 1/16, 1, 16 or
/// 256; extents 1/4 to 1 3/4 units, static boxes and a ground on some seeds.
fn world(seed: u32) -> Array<BroadPhaseProxy> {
    let n = 33 + seed % 40;
    let scale: i64 = if seed / 40 % 4 == 0 {
        1
    } else if seed / 40 % 4 == 1 {
        16
    } else if seed / 40 % 4 == 2 {
        256
    } else {
        4096
    };
    let u: i64 = 0x1000000 * scale;
    let mut values = array![];
    let mut i = 0;
    while i != n {
        let cx: i64 = ((seed + i * 17) % 23).into();
        let cy: i64 = ((seed + i * 31) % 19).into();
        let fx: i64 = ((seed * 7 + i * 5) % 16).into();
        let fy: i64 = ((seed * 3 + i * 11) % 16).into();
        let w: i64 = (4 + (seed + i) % 24).into();
        let h: i64 = (4 + (seed + i * 3) % 24).into();
        let x = (cx - 11) * 16 * u + fx * u;
        let y = (cy - 9) * 16 * u + fy * u;
        values.append(proxy_raw(i, x, y, x + w * u, y + h * u, i % 5 == 0));
        i += 1;
    }
    if seed % 3 == 0 {
        values.append(proxy_raw(n, -256 * 16 * u, -32 * u, 256 * 16 * u, 0, true));
    }
    values
}

#[test]
#[fuzzer(runs: 96, seed: 20260924)]
fn fuzz_scale_sweep(seed: u16) {
    let values = world(seed.into());
    let brute = find_pairs_brute(values.span());
    assert_same(find_pairs(values.span()).span(), brute.span());
    assert_same(find_pairs_strip(values.span()).span(), brute.span());
    assert_same(find_pairs_grid(values.span()).span(), brute.span());
}

/// The same layout, coordinates and extents multiplied by 16, 256 and 4096: identical pairs, and
/// the cell size scales with it.
#[test]
fn test_scaled_worlds_have_identical_pairs_and_proportional_cells() {
    let mut seed = 0;
    while seed != 6 {
        let base = world(seed);
        let unit = size_of(base.span());
        let expected = find_pairs(base.span());
        let mut factor = 1_u64;
        let mut times = 0;
        while times != 3 {
            factor *= 16;
            let mut scaled = array![];
            for p in base.span() {
                let f: i64 = factor.try_into().unwrap();
                let a = *p.aabb.mins.x.raw * f;
                let b = *p.aabb.mins.y.raw * f;
                let c = *p.aabb.maxs.x.raw * f;
                let d = *p.aabb.maxs.y.raw * f;
                scaled.append(proxy_raw(0, a, b, c, d, *p.is_static));
            }
            assert_eq!(size_of(scaled.span()), unit * factor);
            assert_same(find_pairs(scaled.span()).span(), expected.span());
            times += 1;
        }
        seed += 1;
    }
}
