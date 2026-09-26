//! Tests and `gas_*` probes of `crate::query::intersection` (the `intersection_test_*` entry
//! points and `ShapeIntersection`).
//!
//! The wrappers are checked against the exact answers of `crate::dispatch::intersection_test`
//! (the kernels they call) over every ordered pair of the shape set at several poses, which
//! pins the argument order and the inversion of `pos12`, and against hand-computed cases.

use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::aabb::{Aabb, AabbTrait};
use crate::dispatch::intersection_test;
use crate::shape::{Ball, Capsule, ConvexPolygonTrait, Cuboid, HalfSpace, Segment, Shape};
use super::{
    ShapeIntersection, ShapeIntersectionTrait, intersection_test_aabb_segment,
    intersection_test_ball_point_query, intersection_test_cuboid_segment,
    intersection_test_halfspace_support_map, intersection_test_point_query_ball,
    intersection_test_segment_cuboid, intersection_test_support_map_halfspace,
    intersection_test_support_map_support_map,
};

/// 30 degrees.
const R_30: Rot2 = Rot2 { re: Fixed { raw: 3719550787 }, im: HALF };
const R_90: Rot2 = Rot2 { re: ZERO, im: ONE };

fn f(num: i64, den: i64) -> Fixed {
    FixedTrait::from_ratio(num, den)
}
fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}
fn pose(x: Fixed, y: Fixed, rotation: Rot2) -> Pose2 {
    Pose2 { translation: v(x, y), rotation }
}

fn ball() -> Ball {
    Ball { radius: HALF }
}
fn cuboid() -> Cuboid {
    Cuboid { half_extents: v(ONE, HALF) }
}
fn segment() -> Segment {
    Segment { a: v(-ONE, ZERO), b: v(ONE, ZERO) }
}
fn halfspace() -> HalfSpace {
    HalfSpace { normal: v(ZERO, ONE) }
}

/// One shape of each kind, a capsule and a triangle included.
fn shapes() -> Array<Shape> {
    let triangle = ConvexPolygonTrait::from_convex_polyline(
        array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span(),
    )
        .unwrap();
    array![
        Shape::Ball(ball()), Shape::Cuboid(cuboid()),
        Shape::Capsule(
            Capsule { segment: Segment { a: v(ZERO, -HALF), b: v(ZERO, HALF) }, radius: HALF },
        ),
        Shape::Segment(segment()), Shape::ConvexPolygon(BoxTrait::new(triangle)),
    ]
}

fn poses() -> Array<Pose2> {
    array![
        pose(ZERO, ZERO, Default::default()), pose(ONE, HALF, Default::default()),
        pose(TWO, ZERO, R_90), pose(-ONE, f(3, 2), R_30), pose(f(5, 2), -TWO, R_30),
        pose(f(-1, 2), ZERO, R_90),
    ]
}

#[test]
fn test_wrappers_agree_with_the_dispatch_table() {
    let shapes = shapes();
    let poses = poses();
    let hs = halfspace();
    for pos12 in poses.span() {
        let pos12 = *pos12;
        for s1 in shapes.span() {
            let s1 = *s1;
            // Support-map pairs, every ordered pair.
            for s2 in shapes.span() {
                let s2 = *s2;
                let expected = intersection_test(pos12, s1, s2).unwrap();
                assert_eq!(intersection_test_support_map_support_map(pos12, s1, s2), expected);
            }
            // Half-space against a support map, both orders.
            let with_hs = intersection_test(pos12, Shape::HalfSpace(hs), s1).unwrap();
            assert_eq!(intersection_test_halfspace_support_map(pos12, hs, s1), with_hs);
            let hs_last = intersection_test(pos12, s1, Shape::HalfSpace(hs)).unwrap();
            assert_eq!(intersection_test_support_map_halfspace(pos12, s1, hs), hs_last);
            // Ball against a point-query shape, both orders (no sub-shapes).
            let ball_2 = intersection_test(pos12, s1, Shape::Ball(ball())).unwrap();
            assert_eq!(
                intersection_test_point_query_ball(pos12, s1, ball()),
                ShapeIntersectionTrait::new(ball_2),
            );
            let ball_1 = intersection_test(pos12, Shape::Ball(ball()), s1).unwrap();
            assert_eq!(
                intersection_test_ball_point_query(pos12, ball(), s1),
                ShapeIntersectionTrait::new(ball_1),
            );
        }
        // Cuboid and segment, both orders.
        let cs = intersection_test(pos12, Shape::Cuboid(cuboid()), Shape::Segment(segment()))
            .unwrap();
        assert_eq!(intersection_test_cuboid_segment(pos12, cuboid(), segment()), cs);
        let sc = intersection_test(pos12, Shape::Segment(segment()), Shape::Cuboid(cuboid()))
            .unwrap();
        assert_eq!(intersection_test_segment_cuboid(pos12, segment(), cuboid()), sc);
    }
}

/// `(pos12, expected)` of the cuboid `[-1, 1] x [-0.5, 0.5]` and the segment `(-1, 0)-(1, 0)`.
fn cuboid_segment_cases() -> Array<(Pose2, bool)> {
    array![
        (pose(ZERO, HALF, Default::default()), true),
        (pose(ZERO, HALF + f(1, 1024), Default::default()), false),
        (pose(TWO, ZERO, Default::default()), true),
        (pose(TWO + f(1, 1024), ZERO, Default::default()), false), (pose(ZERO, ONE, R_90), true),
        (pose(ZERO, f(3, 2), R_90), true), (pose(ZERO, f(3, 2) + f(1, 1024), R_90), false),
    ]
}

#[test]
fn test_cuboid_segment_orders() {
    for case in cuboid_segment_cases().span() {
        let (pos12, expected) = *case;
        assert_eq!(intersection_test_cuboid_segment(pos12, cuboid(), segment()), expected);
        // The segment first: the same pair seen from the other frame.
        assert_eq!(
            intersection_test_segment_cuboid(pos12.inverse(), segment(), cuboid()), expected,
        );
    }
}

/// `(a, b, expected)` against the AABB `[1, 3] x [1, 2]` (segment in the frame of the AABB's
/// parent, translated by `-center` as upstream does).
fn aabb_segment_cases() -> Array<(Vec2, Vec2, bool)> {
    array![
        (v(ZERO, ZERO), v(TWO, f(3, 2)), true), (v(ZERO, ZERO), v(HALF, f(3, 1)), false),
        (v(ONE + TWO, f(3, 2)), v(f(4, 1), f(3, 2)), true),
        (v(f(3, 1) + f(1, 1024), f(3, 2)), v(f(4, 1), f(3, 2)), false),
        (v(TWO, f(3, 2)), v(TWO, f(3, 2)), true), (v(ZERO, f(5, 2)), v(f(4, 1), f(5, 2)), false),
        (v(ZERO, f(5, 2)), v(f(4, 1), ONE), true),
    ]
}

#[test]
fn test_aabb_segment() {
    let aabb: Aabb = AabbTrait::new(v(ONE, ONE), v(f(3, 1), TWO));
    for case in aabb_segment_cases().span() {
        let (a, b, expected) = *case;
        assert_eq!(intersection_test_aabb_segment(aabb, Segment { a, b }), expected);
    }
}

/// The half-space `y <= 0` against a ball of radius 0.5, on both sides of the threshold.
#[test]
fn test_halfspace_support_map_thresholds() {
    let ball = Shape::Ball(ball());
    // Ball of radius 0.5: touching at y = 0.5.
    assert!(
        intersection_test_halfspace_support_map(
            pose(ZERO, HALF, Default::default()), halfspace(), ball,
        ),
    );
    assert!(
        !intersection_test_halfspace_support_map(
            pose(ZERO, HALF + f(1, 1024), Default::default()), halfspace(), ball,
        ),
    );
    // Seen from the ball: the half-space is at `-pos12`.
    assert!(
        intersection_test_support_map_halfspace(
            pose(ZERO, -HALF, Default::default()), ball, halfspace(),
        ),
    );
    assert!(
        !intersection_test_support_map_halfspace(
            pose(ZERO, -HALF - f(1, 1024), Default::default()), ball, halfspace(),
        ),
    );
}

#[test]
#[should_panic(expected: 'Query: not a support map')]
fn test_halfspace_support_map_rejects_a_halfspace() {
    intersection_test_halfspace_support_map(
        Default::default(), halfspace(), Shape::HalfSpace(halfspace()),
    );
}

#[test]
#[should_panic(expected: 'Query: not a support map')]
fn test_support_map_support_map_rejects_a_halfspace() {
    intersection_test_support_map_support_map(
        Default::default(), Shape::Ball(ball()), Shape::HalfSpace(halfspace()),
    );
}

#[test]
fn test_shape_intersection_helpers() {
    let base = ShapeIntersectionTrait::new(true);
    assert_eq!(base, ShapeIntersection { intersecting: true, subshape1: 0, subshape2: 0 });
    let tagged = base.with_subshapes(3, 7);
    assert_eq!(tagged, ShapeIntersection { intersecting: true, subshape1: 3, subshape2: 7 });
    assert_eq!(
        tagged.swapped(), ShapeIntersection { intersecting: true, subshape1: 7, subshape2: 3 },
    );
    assert_eq!(tagged.swapped().swapped(), tagged);
    let from_bool: ShapeIntersection = false.into();
    assert_eq!(from_bool, ShapeIntersectionTrait::new(false));
    assert!(!from_bool.intersecting);
}

#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}

#[test]
fn gas_cuboid_segment() {
    assert!(
        intersection_test_cuboid_segment(
            opaque(pose(ZERO, HALF, R_30)), opaque(cuboid()), opaque(segment()),
        ),
    );
}

#[test]
fn gas_segment_cuboid() {
    assert!(
        intersection_test_segment_cuboid(
            opaque(pose(ZERO, HALF, R_30)), opaque(segment()), opaque(cuboid()),
        ),
    );
}

#[test]
fn gas_aabb_segment() {
    let aabb: Aabb = AabbTrait::new(v(ONE, ONE), v(f(3, 1), TWO));
    assert!(
        intersection_test_aabb_segment(
            opaque(aabb), opaque(Segment { a: v(ZERO, ZERO), b: v(TWO, f(3, 2)) }),
        ),
    );
}

#[test]
fn gas_halfspace_support_map() {
    let pos12 = opaque(pose(ZERO, HALF, R_30));
    assert!(
        intersection_test_halfspace_support_map(
            pos12, opaque(halfspace()), Shape::Cuboid(opaque(cuboid())),
        ),
    );
}

#[test]
fn gas_support_map_support_map() {
    let pos12 = opaque(pose(ONE, HALF, R_30));
    assert!(
        intersection_test_support_map_support_map(
            pos12, Shape::Cuboid(opaque(cuboid())), Shape::Cuboid(opaque(cuboid())),
        ),
    );
}

#[test]
fn gas_point_query_ball() {
    let pos12 = opaque(pose(ONE, HALF, R_30));
    assert!(
        intersection_test_point_query_ball(pos12, Shape::Cuboid(opaque(cuboid())), opaque(ball()))
            .intersecting,
    );
}

#[test]
fn gas_ball_point_query() {
    let pos12 = opaque(pose(ONE, HALF, R_30));
    assert!(
        intersection_test_ball_point_query(pos12, opaque(ball()), Shape::Cuboid(opaque(cuboid())))
            .intersecting,
    );
}
