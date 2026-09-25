//! Tests and `gas_*` probes of `crate::dispatch::intersection`.
//!
//! Probes call the typed kernels with opaque inputs (a loop-free probe of the inlined table
//! would pay its costliest arm), plus the table inside a one-iteration loop body, as the narrow
//! phase's pair loop runs it.

use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::shape::{
    Ball, Capsule, ConvexPolygon, ConvexPolygonTrait, Cuboid, HalfSpace, Segment, Shape,
};
use super::alternatives::{
    cuboid_cuboid_upstream_sat, intersection_test_from_contacts, point_polygon_projection,
    segment_segment_endpoints,
};
use super::{
    ball_ball, cuboid_capsule, cuboid_cuboid, halfspace_cuboid, halfspace_polygon,
    halfspace_segment, intersection_test, point_cuboid, point_halfspace, point_polygon,
    point_segment, segment_segment,
};

const QUARTER: Fixed = Fixed { raw: 1073741824 };
/// 30 degrees.
const R_30: Rot2 = Rot2 { re: Fixed { raw: 3719550787 }, im: HALF };
const R_90: Rot2 = Rot2 { re: ZERO, im: ONE };

fn f(num: i64, den: i64) -> Fixed {
    FixedTrait::from_ratio(num, den)
}
fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}
fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: v(x, y), rotation: Default::default() }
}
fn rot(x: Fixed, y: Fixed, rotation: Rot2) -> Pose2 {
    Pose2 { translation: v(x, y), rotation }
}

fn ball() -> Shape {
    Shape::Ball(Ball { radius: HALF })
}
/// Half extents (1, 0.5).
fn cuboid() -> Shape {
    Shape::Cuboid(Cuboid { half_extents: v(ONE, HALF) })
}
/// Vertical core (0, -0.5)-(0, 0.5), radius 0.25.
fn capsule() -> Shape {
    Shape::Capsule(Capsule { segment: segment_y(), radius: QUARTER })
}
fn segment_y() -> Segment {
    Segment { a: v(ZERO, -HALF), b: v(ZERO, HALF) }
}
fn segment() -> Shape {
    Shape::Segment(Segment { a: v(-ONE, ZERO), b: v(ONE, ZERO) })
}
fn halfspace() -> Shape {
    Shape::HalfSpace(HalfSpace { normal: v(ZERO, ONE) })
}
/// Triangle (-1, -1), (1, -1), (0, 1).
fn triangle() -> ConvexPolygon {
    ConvexPolygonTrait::from_convex_polyline(
        array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span(),
    )
        .unwrap()
}
fn polygon() -> Shape {
    Shape::ConvexPolygon(BoxTrait::new(triangle()))
}

/// `(shape1, shape2, pos12, expected)`: every arm of the table, each on both sides of its
/// threshold, touching counted as intersecting.
fn cases() -> Array<(Shape, Shape, Pose2, bool)> {
    array![
        // ball–ball: radii 0.5 + 0.5.
        (ball(), ball(), at(ONE, ZERO), true), (ball(), ball(), at(ZERO, -ONE), true),
        (ball(), ball(), at(ONE + f(1, 1024), ZERO), false),
        // ball–cuboid (touching the +x face, the corner region, contained).
        (ball(), cuboid(), at(f(3, 2), ZERO), true),
        (ball(), cuboid(), at(f(3, 2) + f(1, 1024), ZERO), false),
        (cuboid(), ball(), at(ONE + f(3, 10), HALF + f(3, 10)), true),
        (cuboid(), ball(), at(ONE + f(4, 10), HALF + f(4, 10)), false),
        (
            cuboid(), ball(), at(ZERO, ZERO), true,
        ), // ball–capsule / segment / half-space / polygon.
        (ball(), capsule(), at(f(3, 4), ZERO), true),
        (capsule(), ball(), at(ZERO, f(3, 2) + f(1, 1024)), false),
        (segment(), ball(), at(ZERO, HALF), true),
        (ball(), segment(), at(f(3, 2), f(1, 1024)), false),
        (halfspace(), ball(), at(f(7, 1), HALF), true),
        (ball(), halfspace(), at(ZERO, -HALF - f(1, 1024)), false),
        (ball(), polygon(), at(ZERO, f(3, 2)), true),
        (polygon(), ball(), at(ZERO, -f(3, 2) - f(1, 1024)), false),
        // cuboid–cuboid.
        (cuboid(), cuboid(), at(TWO, ZERO), true), (cuboid(), cuboid(), at(ZERO, ONE), true),
        (cuboid(), cuboid(), at(TWO + f(1, 1024), ZERO), false),
        (cuboid(), cuboid(), rot(ONE + HALF, ZERO, R_90), true),
        (cuboid(), cuboid(), rot(TWO, ONE, R_30), false),
        // half-space–cuboid / capsule / segment / polygon, both orders.
        (halfspace(), cuboid(), at(ZERO, HALF), true),
        (halfspace(), cuboid(), at(ZERO, HALF + f(1, 1024)), false),
        (cuboid(), halfspace(), at(ZERO, -HALF), true),
        (halfspace(), capsule(), rot(ZERO, QUARTER, R_90), true),
        (halfspace(), capsule(), at(ZERO, HALF + QUARTER + f(1, 1024)), false),
        (segment(), halfspace(), at(ZERO, ZERO), true),
        (halfspace(), segment(), at(ZERO, f(1, 1024)), false),
        (halfspace(), polygon(), at(ZERO, ONE), true),
        (polygon(), halfspace(), at(ZERO, -ONE - f(1, 1024)), false),
        (halfspace(), halfspace(), at(ZERO, ZERO), false),
        // cuboid–capsule / segment: face contact, corner region, crossing core.
        (cuboid(), capsule(), at(ONE + QUARTER, ZERO), true),
        (capsule(), cuboid(), at(ONE + QUARTER + f(1, 1024), ZERO), false),
        (cuboid(), capsule(), at(ONE + f(1, 8), HALF + HALF + f(1, 8)), true),
        (cuboid(), capsule(), at(ONE + f(1, 4), HALF + HALF + f(1, 4)), false),
        (cuboid(), segment(), at(ZERO, HALF), true),
        (segment(), cuboid(), at(ZERO, HALF + f(1, 1024)), false),
        (cuboid(), segment(), rot(ZERO, ZERO, R_30), true),
        // segment / capsule pairs.
        (capsule(), capsule(), at(HALF, ZERO), true),
        (capsule(), capsule(), at(HALF + f(1, 1024), ZERO), false),
        (capsule(), capsule(), rot(ZERO, ONE, R_90), true),
        (segment(), segment(), rot(ZERO, ZERO, R_90), true),
        (segment(), segment(), at(TWO, ZERO), true),
        (segment(), segment(), at(TWO + f(1, 1024), ZERO), false),
        (segment(), capsule(), at(ONE + QUARTER, ZERO), true),
        (segment(), capsule(), at(ONE + QUARTER + f(1, 1024), ZERO), false),
        (capsule(), segment(), at(QUARTER, ZERO), true),
        // convex polygon pairs.
        (polygon(), polygon(), at(TWO, ZERO), true),
        (polygon(), polygon(), at(TWO + f(1, 1024), ZERO), false),
        (polygon(), polygon(), rot(ZERO, TWO, R_30), true),
        (polygon(), cuboid(), at(ZERO, f(3, 2)), true),
        (cuboid(), polygon(), at(ZERO, -f(3, 2) - f(1, 1024)), false),
        (polygon(), segment(), at(ZERO, ONE), true),
        (segment(), polygon(), at(ZERO, -ONE - f(1, 1024)), false),
        (polygon(), capsule(), at(ZERO, ONE + HALF + QUARTER), true),
        (capsule(), polygon(), at(ZERO, -ONE - HALF - QUARTER - f(1, 1024)), false),
        (polygon(), capsule(), at(ONE + f(1, 8), -ONE - HALF - f(1, 16)), true),
        (polygon(), capsule(), at(ONE + HALF, -ONE - HALF - HALF), false),
    ]
}

#[test]
fn test_cases_both_orders() {
    let mut k: u32 = 0;
    for (s1, s2, pose, expected) in cases() {
        k += 1;
        let supported = !(match (s1, s2) {
            (Shape::HalfSpace(_), Shape::HalfSpace(_)) => true,
            _ => false,
        });
        let answer = if supported {
            Some(expected)
        } else {
            None
        };
        assert!(intersection_test(pose, s1, s2) == answer, "case {}", k);
        assert!(intersection_test(pose.inverse(), s2, s1) == answer, "case {} swapped", k);
    }
}

#[test]
fn test_segments_cross_collinear_and_degenerate() {
    let s = Segment { a: v(-ONE, ZERO), b: v(ONE, ZERO) };
    let point = Segment { a: v(HALF, ZERO), b: v(HALF, ZERO) };
    // (other segment, pose, expected)
    let table: Array<(Segment, Pose2, bool)> = array![
        (s, at(ONE + HALF, ZERO), true), (s, at(TWO + f(1, 1024), ZERO), false),
        (point, at(ZERO, ZERO), true), (point, at(ZERO, f(1, 1024)), false),
        (point, at(ONE, ZERO), false),
    ];
    for (other, pose, expected) in table {
        assert_eq!(segment_segment(pose, s, other, ZERO), expected);
        assert_eq!(segment_segment_endpoints(pose, s, other, ZERO), expected);
    }
}

#[test]
#[fuzzer(runs: 48, seed: 20260925)]
fn fuzz_cuboid_cuboid_candidates(x: i16, y: i16) {
    let c1 = Cuboid { half_extents: v(ONE, HALF) };
    let c2 = Cuboid { half_extents: v(HALF + QUARTER, QUARTER) };
    for rotation in array![Default::default(), R_30, R_90].span() {
        let pose = rot(
            Fixed { raw: x.into() * 262147 }, Fixed { raw: y.into() * 131101 }, *rotation,
        );
        assert_eq!(cuboid_cuboid(pose, c1, c2), cuboid_cuboid_upstream_sat(pose, c1, c2));
    }
}

#[test]
#[fuzzer(runs: 48, seed: 20260926)]
fn fuzz_segment_and_polygon_candidates(x: i16, y: i16) {
    let s1 = Segment { a: v(-ONE, -QUARTER), b: v(ONE, HALF) };
    let s2 = segment_y();
    for rotation in array![Default::default(), R_30, R_90].span() {
        let pose = rot(
            Fixed { raw: x.into() * 262147 }, Fixed { raw: y.into() * 131101 }, *rotation,
        );
        for radius in array![ZERO, QUARTER, ONE].span() {
            assert_eq!(
                segment_segment(pose, s1, s2, *radius),
                segment_segment_endpoints(pose, s1, s2, *radius),
            );
        }
        let center = pose.translation;
        for radius in array![ZERO, QUARTER, ONE].span() {
            assert_eq!(
                point_polygon(triangle(), center, *radius),
                point_polygon_projection(triangle(), center, *radius),
            );
        }
    }
}

/// The table answers the same in both orders on every pair, and agrees with the contact
/// generators on the pairs whose generator is exact (ball pairs, cuboid–cuboid, half-space
/// pairs). The analytic cuboid–capsule and PFM polygon generators are not: they can miss a
/// rounded cap against a corner (`-21015, 15468` here: capsule–cuboid at distance 0.171 <
/// 0.25, no point with `dist <= 0`), which upstream's GJK and these kernels see.
#[test]
#[fuzzer(runs: 32, seed: 20260927)]
fn fuzz_table_symmetric_and_contacts(x: i16, y: i16) {
    let shapes = array![ball(), cuboid(), capsule(), segment(), polygon(), halfspace()];
    let pose = rot(Fixed { raw: x.into() * 262147 }, Fixed { raw: y.into() * 131101 }, R_30);
    for s1 in shapes.span() {
        for s2 in shapes.span() {
            let answer = intersection_test(pose, *s1, *s2);
            assert_eq!(answer, intersection_test(pose.inverse(), *s2, *s1));
            let exact_generator = match (*s1, *s2) {
                (Shape::Ball(_), _) | (_, Shape::Ball(_)) => true,
                (Shape::HalfSpace(_), _) | (_, Shape::HalfSpace(_)) => true,
                (Shape::Cuboid(_), Shape::Cuboid(_)) => true,
                _ => false,
            };
            if exact_generator {
                if let Some(derived) = intersection_test_from_contacts(pose, *s1, *s2) {
                    assert_eq!(answer, Some(derived));
                }
            }
        }
    }
}

#[test]
fn gas_baseline() {
    let _ = opaque(at(ONE, ZERO));
}

#[test]
fn gas_ball_ball() {
    let b = Ball { radius: opaque(HALF) };
    let _ = opaque(ball_ball(opaque(v(HALF, HALF)), b, b));
}

#[test]
fn gas_ball_cuboid() {
    let c = Cuboid { half_extents: opaque(v(ONE, HALF)) };
    let _ = opaque(point_cuboid(c, opaque(v(ONE + HALF, ONE)), HALF));
}

#[test]
fn gas_ball_polygon_projection() {
    let _ = opaque(point_polygon_projection(opaque(triangle()), opaque(v(ZERO, f(3, 2))), HALF));
}

#[test]
fn gas_ball_segment() {
    let _ = opaque(point_segment(opaque(segment_y()), opaque(v(HALF, QUARTER)), HALF));
}

#[test]
fn gas_ball_halfspace() {
    let h = HalfSpace { normal: opaque(v(ZERO, ONE)) };
    let _ = opaque(point_halfspace(h, opaque(v(HALF, QUARTER)), HALF));
}

#[test]
fn gas_ball_polygon() {
    let _ = opaque(point_polygon(opaque(triangle()), opaque(v(ZERO, f(3, 2))), HALF));
}

#[test]
fn gas_cuboid_cuboid() {
    let c = Cuboid { half_extents: opaque(v(ONE, HALF)) };
    let _ = opaque(cuboid_cuboid(opaque(rot(ONE, HALF, R_30)), c, c));
}

#[test]
fn gas_cuboid_cuboid_upstream_sat() {
    let c = Cuboid { half_extents: opaque(v(ONE, HALF)) };
    let _ = opaque(cuboid_cuboid_upstream_sat(opaque(rot(ONE, HALF, R_30)), c, c));
}

#[test]
fn gas_halfspace_cuboid() {
    let c = Cuboid { half_extents: opaque(v(ONE, HALF)) };
    let _ = opaque(halfspace_cuboid(opaque(rot(ZERO, HALF, R_30)), v(ZERO, ONE), c));
}

#[test]
fn gas_halfspace_capsule() {
    let _ = opaque(
        halfspace_segment(opaque(rot(ZERO, HALF, R_30)), v(ZERO, ONE), segment_y(), QUARTER),
    );
}

#[test]
fn gas_halfspace_polygon() {
    let _ = opaque(
        halfspace_polygon(opaque(rot(ZERO, TWO, R_30)), v(ZERO, ONE), opaque(triangle())),
    );
}

/// Overlapping core: the three-axis test answers.
#[test]
fn gas_cuboid_capsule_overlap() {
    let c = Cuboid { half_extents: opaque(v(ONE, HALF)) };
    let capsule = Capsule { segment: segment_y(), radius: QUARTER };
    let _ = opaque(cuboid_capsule(opaque(rot(ONE, ZERO, R_30)), c, capsule));
}

/// Separated core in the corner region: the six distances run.
#[test]
fn gas_cuboid_capsule_corner() {
    let c = Cuboid { half_extents: opaque(v(ONE, HALF)) };
    let capsule = Capsule { segment: segment_y(), radius: QUARTER };
    let _ = opaque(cuboid_capsule(opaque(at(ONE + HALF, TWO)), c, capsule));
}

#[test]
fn gas_cuboid_segment() {
    let c = Cuboid { half_extents: opaque(v(ONE, HALF)) };
    let s = Capsule { segment: segment_y(), radius: ZERO };
    let _ = opaque(cuboid_capsule(opaque(at(ONE + HALF, TWO)), c, s));
}

#[test]
fn gas_capsule_capsule() {
    let _ = opaque(segment_segment(opaque(at(ONE, QUARTER)), segment_y(), segment_y(), HALF));
}

#[test]
fn gas_capsule_capsule_endpoints() {
    let _ = opaque(
        segment_segment_endpoints(opaque(at(ONE, QUARTER)), segment_y(), segment_y(), HALF),
    );
}

#[test]
fn gas_segment_segment() {
    let _ = opaque(segment_segment(opaque(rot(ZERO, ZERO, R_30)), segment_y(), segment_y(), ZERO));
}

#[test]
fn gas_polygon_polygon() {
    let _ = opaque(intersection_test(opaque(at(TWO, HALF)), polygon(), polygon()));
}

#[test]
fn gas_polygon_cuboid() {
    let _ = opaque(intersection_test(opaque(at(TWO, HALF)), polygon(), cuboid()));
}

#[test]
fn gas_polygon_capsule_corner() {
    let _ = opaque(intersection_test(opaque(at(ONE + HALF, -TWO)), polygon(), capsule()));
}

/// The table in a one-iteration loop body (as the pair loop runs it): ball–ball.
#[test]
fn gas_table_in_loop_ball_ball() {
    let mut pending = opaque(true);
    let mut hit = None;
    while pending {
        hit = intersection_test(opaque(at(HALF, HALF)), opaque(ball()), opaque(ball()));
        pending = false;
    }
    let _ = opaque(hit);
}

/// The table in a one-iteration loop body: cuboid–cuboid.
#[test]
fn gas_table_in_loop_cuboid_cuboid() {
    let mut pending = opaque(true);
    let mut hit = None;
    while pending {
        hit = intersection_test(opaque(rot(ONE, HALF, R_30)), opaque(cuboid()), opaque(cuboid()));
        pending = false;
    }
    let _ = opaque(hit);
}

/// Derived from the contact generators (metered dispatcher, zero prediction): ball–ball.
#[test]
fn gas_from_contacts_ball_ball() {
    let _ = opaque(intersection_test_from_contacts(opaque(at(HALF, HALF)), ball(), ball()));
}

/// Derived from the contact generators: cuboid–cuboid.
#[test]
fn gas_from_contacts_cuboid_cuboid() {
    let _ = opaque(
        intersection_test_from_contacts(opaque(rot(ONE, HALF, R_30)), cuboid(), cuboid()),
    );
}

/// Derived from the contact generators: cuboid–capsule, corner region.
#[test]
fn gas_from_contacts_cuboid_capsule() {
    let _ = opaque(
        intersection_test_from_contacts(opaque(at(ONE + HALF, TWO)), cuboid(), capsule()),
    );
}

/// Derived from the contact generators: polygon–polygon.
#[test]
fn gas_from_contacts_polygon_polygon() {
    let _ = opaque(intersection_test_from_contacts(opaque(at(TWO, HALF)), polygon(), polygon()));
}
