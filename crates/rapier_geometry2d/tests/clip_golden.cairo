//! All 16 upstream cases, both clipping paths, exact order and feature identifiers.
use fixed::{Fixed, FixedTrait};
use glam::Vec2;
use rapier_geometry2d::clip::{
    ClippingPoints, clip_segment_segment, clip_segment_segment_with_features,
    clip_segment_segment_with_normal,
};
use rapier_golden::clip2d;
use rapier_golden::compare::within;
use rapier_golden::types::{ClipPointRaw, ClipResultRaw, Vec2Raw};
use rapier_testing::opaque;
fn v(p: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: p.x }, y: Fixed { raw: p.y } }
}
fn point(p: Vec2, q: Vec2Raw, tol: u64, id: felt252) {
    assert!(within(p.x.raw, q.x, tol), "{}: x {} vs {}", id, p.x.raw, q.x);
    assert!(within(p.y.raw, q.y, tol), "{}: y {} vs {}", id, p.y.raw, q.y);
}
fn check_point(a: ClippingPoints, b: ClipPointRaw, tol: u64, id: felt252) {
    let (p1, p2, f1, f2) = a;
    point(p1, b.p1, tol, id);
    point(p2, b.p2, tol, id);
    assert_eq!(f1, b.f1, "{}", id);
    assert_eq!(f2, b.f2, "{}", id);
}
fn check(a: Option<(ClippingPoints, ClippingPoints)>, b: ClipResultRaw, tol: u64, id: felt252) {
    assert_eq!(a.is_some(), b.clipped, "{}", id);
    if let Some((p, q)) = a {
        let [bp, bq] = b.points;
        check_point(p, bp, tol, id);
        check_point(q, bq, tol, id);
    }
}
#[test]
fn test_all_clip_vectors() {
    let mut count = 0;
    for c in clip2d::cases() {
        let a = (v(*c.seg1.a), v(*c.seg1.b));
        let b = (v(*c.seg2.a), v(*c.seg2.b));
        // README: 4 + 2*length ulp; Manhattan length bounds Euclidean length.
        let d1 = v(*c.seg1.b) - v(*c.seg1.a);
        let d2 = v(*c.seg2.b) - v(*c.seg2.a);
        let length = (d1.x.abs() + d1.y.abs()).max(d2.x.abs() + d2.y.abs());
        let raw: u64 = length.raw.try_into().unwrap();
        let tol = 4 + 2 * (raw / 4294967296 + 1);
        let actual = clip_segment_segment_with_features(a, b);
        check(actual, *c.plain, tol, *c.id);
        check(clip_segment_segment_with_normal(a, b, v(*c.normal)), *c.with_normal, tol, *c.id);
        let (a0, a1) = a;
        let (b0, b1) = b;
        let expected = match actual {
            Some(((p, q, _, _), (r, s, _, _))) => Some(((p, q), (r, s))),
            None => None,
        };
        assert_eq!(clip_segment_segment(a0, a1, b0, b1), expected);
        count += 1;
    }
    assert_eq!(count, 16);
}
#[test]
fn gas_baseline() {
    let _ = opaque(clip2d::CLIP_PARALLEL_PARTIAL);
}
#[test]
fn gas_golden_clip() {
    let c = opaque(clip2d::CLIP_PARALLEL_PARTIAL);
    let _ = clip_segment_segment_with_normal(
        (v(c.seg1.a), v(c.seg1.b)), (v(c.seg2.a), v(c.seg2.b)), v(c.normal),
    );
}
