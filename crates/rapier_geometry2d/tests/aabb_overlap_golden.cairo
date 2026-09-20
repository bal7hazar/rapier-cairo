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
