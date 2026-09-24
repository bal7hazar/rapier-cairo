//! Scale sweep of the strip/grid broad phase. `scale` is the world multiplier in 1/16 units:
//! 1 = x1/16, 16 = x1, 256 = x16, 4096 = x256. Layouts are BG's 0 (sparse), 1 (stack),
//! 2 (dense), 3 (sparse plus one huge static ground), with coordinates and extents multiplied.
//! The shipped probes run in a x16 world; subtract `gas_setup` (same input, no broad phase).
//! The full sweep (scales x sizes x layouts, gas and steps) is in the BS report.
use core::num::traits::DivRem;
use fixed::Fixed;
use glam::Vec2;
use rapier_core::data::handle::HandleTrait;
use rapier_testing::opaque;
use crate::aabb::AabbTrait;
use super::alternatives::scale::{cell_size_max3, find_pairs_grid_match, floor_wide_mul};
use super::grid::find_pairs_grid_sized;
use super::scale::{cell_size, floor_div};
use super::strip::find_pairs_strip_sized;
use super::{BroadPhaseProxy, find_pairs};

#[inline(never)]
pub(crate) fn scaled(n: u32, layout: u32, scale: i64) -> Span<BroadPhaseProxy> {
    // One unit `u` is 1/256 world unit times the scale multiplier.
    let u: i64 = 0x1000000 * scale;
    let mut values = array![];
    let mut i = 0;
    while i != n {
        let k = opaque((i * 17 + 5) % n);
        let (row_index, column_index) = DivRem::div_rem(k, 16);
        let column: i64 = column_index.into();
        let row: i64 = row_index.into();
        let stack: i64 = k.into();
        let (x, y, w, h) = if layout == 1 {
            (0, stack * 16 * u, 16 * u, 16 * u)
        } else if layout == 2 {
            (0, 0, 16 * u, 16 * u)
        } else {
            (column * 64 * u, row * 64 * u, 16 * u, 16 * u)
        };
        let ground = (layout == 3 && k == 0);
        let (x, y, w, h) = if ground {
            (-4096 * u, -16 * u, 8192 * u, 16 * u)
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
                    is_static: ground,
                },
            );
        i += 1;
    }
    opaque(values.span())
}

#[test]
fn gas_baseline() {}

#[test_case(64, 0)]
#[test_case(256, 0)]
fn gas_setup(n: u32, layout: u32) {
    let p = scaled(n, layout, 256);
    assert!(opaque(p.len()) <= n * n);
}

#[test_case(32, 0)]
#[test_case(64, 0)]
#[test_case(256, 0)]
#[test_case(32, 1)]
#[test_case(64, 1)]
#[test_case(256, 1)]
#[test_case(32, 3)]
#[test_case(64, 3)]
#[test_case(256, 3)]
fn gas_scaled(n: u32, layout: u32) {
    let p = scaled(n, layout, 256);
    assert!(opaque(find_pairs(p).len()) <= n * n);
}

/// Shipped statistic: median of five extents (setup twin: `gas_setup_64_0`).
#[test]
fn gas_cell_size() {
    let size: u64 = cell_size(scaled(64, 0, 256)).into();
    assert!(opaque(size) != 0);
}

/// Loser: maximum of three sampled extents. Cheaper, but a ground in the sample sizes the cells.
#[test]
fn gas_cell_size_max3() {
    let size: u64 = cell_size_max3(scaled(64, 0, 256)).into();
    assert!(opaque(size) != 0);
}

/// Loser: the exponent dispatched per proxy by a `match` over constant divisors.
#[test]
fn gas_grid_match() {
    let p = scaled(64, 0, 256);
    assert!(opaque(find_pairs_grid_match(p).len()) <= 64 * 64);
}

/// Loser: four cells by multiply-shift (compare with `gas_cell_size`'s divisor, see the report).
#[test]
fn gas_cells_wide_mul() {
    let (a, b, c, d) = (
        opaque(0x123456789abcdef_u64),
        opaque(0x223456789abcdef_u64),
        opaque(7),
        opaque(0x7fffffffffffff),
    );
    let r = opaque(0x40000000_u64);
    let sum = floor_wide_mul(a, r)
        + floor_wide_mul(b, r)
        + floor_wide_mul(c, r)
        + floor_wide_mul(d, r);
    assert!(opaque(sum) != 0);
}
