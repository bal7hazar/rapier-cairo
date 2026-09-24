//! Golden broad-phase checks against Parry's closed-interval AABB overlap fixtures.

use fixed::Fixed;
use glam::Vec2;
use rapier_core::data::handle::HandleTrait;
use rapier_geometry2d::aabb::AabbTrait;
use rapier_geometry2d::broad_phase::{BroadPhaseProxy, find_pairs};
use rapier_golden::aabb_overlap;
use rapier_golden::types::AabbOverlapCase;
use rapier_testing::opaque;

fn v(raw_x: i64, raw_y: i64) -> Vec2 {
    Vec2 { x: Fixed { raw: raw_x }, y: Fixed { raw: raw_y } }
}

fn proxies(case: AabbOverlapCase) -> Array<BroadPhaseProxy> {
    let boxes = case.aabbs.span();
    let mut out = array![];
    let mut i = 0;
    while i != case.num_aabbs {
        let b = boxes.at(i);
        out
            .append(
                BroadPhaseProxy {
                    collider: HandleTrait::new(i, 0),
                    aabb: AabbTrait::new(v(*b.mins.x, *b.mins.y), v(*b.maxs.x, *b.maxs.y)),
                    is_static: *b.is_static,
                },
            );
        i += 1;
    }
    out
}

/// The case with every coordinate multiplied by `factor` (exact, order-preserving); `None` when a
/// coordinate would leave the `i64` range.
fn scaled_proxies(case: AabbOverlapCase, factor: i64) -> Option<Array<BroadPhaseProxy>> {
    let limit = 0x7fffffffffffffff / factor;
    let boxes = case.aabbs.span();
    let mut out = array![];
    let mut i = 0;
    while i != case.num_aabbs {
        let b = boxes.at(i);
        let coords = [*b.mins.x, *b.mins.y, *b.maxs.x, *b.maxs.y];
        for c in coords.span() {
            if *c > limit || *c < -limit {
                return Option::None;
            }
        }
        out
            .append(
                BroadPhaseProxy {
                    collider: HandleTrait::new(i, 0),
                    aabb: AabbTrait::new(
                        v(*b.mins.x * factor, *b.mins.y * factor),
                        v(*b.maxs.x * factor, *b.maxs.y * factor),
                    ),
                    is_static: *b.is_static,
                },
            );
        i += 1;
    }
    Option::Some(out)
}

fn expected(case: AabbOverlapCase) -> Array<(u32, u32)> {
    let pairs = case.pairs.span();
    let mut out = array![];
    let mut i = 0;
    while i != case.num_pairs {
        let p = pairs.at(i);
        if !*p.both_static {
            out.append((*p.i, *p.j));
        }
        i += 1;
    }
    out
}

fn assert_same(actual: Span<(u32, u32)>, expected: Span<(u32, u32)>) {
    assert_eq!(actual.len(), expected.len());
    let mut i = 0;
    while i != expected.len() {
        assert_eq!(*actual.at(i), *expected.at(i));
        i += 1;
    }
}

#[test]
fn test_all_golden_overlap_sets() {
    for case in aabb_overlap::cases() {
        let actual = find_pairs(proxies(*case).span());
        let expected = expected(*case);
        assert_same(actual.span(), expected.span());
    }
}

/// The same golden layouts in worlds 16, 256 and 4096 times larger give the same pairs.
#[test]
fn test_golden_overlap_sets_are_scale_free() {
    let mut checked = 0_u32;
    for case in aabb_overlap::cases() {
        for factor in array![16_i64, 256, 4096].span() {
            if let Option::Some(scaled) = scaled_proxies(*case, *factor) {
                assert_same(find_pairs(scaled.span()).span(), expected(*case).span());
                checked += 1;
            }
        }
    }
    assert!(checked > 0);
}

#[test]
fn test_static_static_pairs_are_dropped_but_dynamic_static_remain() {
    for case in aabb_overlap::cases() {
        let actual = find_pairs(proxies(*case).span());
        let all = case.pairs.span();
        let mut dynamic_expected = 0;
        let mut i = 0;
        while i != *case.num_pairs {
            if !*all.at(i).both_static {
                dynamic_expected += 1;
            }
            i += 1;
        }
        assert_eq!(actual.len(), dynamic_expected, "{}", *case.id);
    }
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_find_pairs_golden_grid() {
    let case = opaque(aabb_overlap::SET_GRID_TOUCHING);
    let proxies = proxies(case);
    assert_eq!(find_pairs(opaque(proxies.span())).len(), expected(case).len());
}
