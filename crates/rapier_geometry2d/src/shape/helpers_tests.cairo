//! Tests and gas probes of the MH1 shape helpers: per-shape `aabb` / `local_aabb` /
//! `bounding_sphere` / `local_bounding_sphere` / `scaled`, the polygon feature helpers and the
//! new `ShapeTrait` views. The original per-shape tests stay unchanged.

use fixed::{Fixed, FixedTrait, HALF, MAX, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::aabb::bounding_volume::{BoundingSphere, BoundingSphereTrait};
use crate::feature_id::{FEATURE_UNKNOWN, FeatureIdTrait};
use super::ball::alternatives as ball_alternatives;
use super::capsule::alternatives as capsule_alternatives;
use super::segment::alternatives as segment_alternatives;
use super::{
    Ball, BallTrait, Capsule, CapsuleTrait, ConvexPolygon, ConvexPolygonTrait, Cuboid, CuboidTrait,
    HalfSpace, HalfSpaceTrait, Segment, SegmentTrait, Shape, ShapeTrait,
};

fn i(n: i32) -> Fixed {
    FixedTrait::from_int(n)
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn raw(x: i64, y: i64) -> Vec2 {
    Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }
}

/// Quarter turn, then `(1, 2)`: exact, literal.
const POSE: Pose2 = Pose2 {
    translation: Vec2 { x: Fixed { raw: 4294967296 }, y: Fixed { raw: 8589934592 } },
    rotation: Rot2 { re: Fixed { raw: 0 }, im: Fixed { raw: 4294967296 } },
};

fn ball() -> Ball {
    BallTrait::new(HALF)
}
fn cuboid() -> Cuboid {
    CuboidTrait::new(v(TWO, ONE))
}
fn capsule() -> Capsule {
    CapsuleTrait::new(v(ZERO, -ONE), v(ZERO, ONE), HALF)
}
fn segment() -> Segment {
    SegmentTrait::new(v(-ONE, ZERO), v(ONE, ONE))
}
fn halfspace() -> HalfSpace {
    HalfSpaceTrait::new(v(ZERO, ONE))
}
fn square() -> ConvexPolygon {
    ConvexPolygonTrait::from_convex_polyline(
        array![v(-ONE, -ONE), v(ONE, -ONE), v(ONE, ONE), v(-ONE, ONE)].span(),
    )
        .unwrap()
}

const N1: Fixed = Fixed { raw: -4294967296 };
const P1: Fixed = Fixed { raw: 4294967296 };
const Z: Fixed = Fixed { raw: 0 };
const O: Vec2 = Vec2 { x: Z, y: Z };
/// `square()` as a literal, so that probes do not pay the construction.
const SQUARE: ConvexPolygon = ConvexPolygon {
    vertices: [
        Vec2 { x: N1, y: N1 }, Vec2 { x: P1, y: N1 }, Vec2 { x: P1, y: P1 }, Vec2 { x: N1, y: P1 },
        O, O, O, O,
    ],
    normals: [
        Vec2 { x: Z, y: N1 }, Vec2 { x: P1, y: Z }, Vec2 { x: Z, y: P1 }, Vec2 { x: N1, y: Z }, O,
        O, O, O,
    ],
    count: 4,
};

fn all() -> Span<Shape> {
    array![
        Shape::Ball(ball()), Shape::Cuboid(cuboid()), Shape::Capsule(capsule()),
        Shape::Segment(segment()), Shape::HalfSpace(halfspace()),
        Shape::ConvexPolygon(BoxTrait::new(square())),
    ]
        .span()
}

fn world_of(expected: Span<(BoundingSphere, BoundingSphere)>, k: usize) -> BoundingSphere {
    let (_, world) = *expected.at(k);
    world
}

#[test]
fn test_bounding_spheres_table() {
    let sqrt5 = Fixed { raw: 9603838834 };
    let sqrt2 = Fixed { raw: 6074000999 };
    // Segment (-1, 0)-(1, 1): centre (0, 1/2), radius |(1, 1/2)| floored.
    let seg_r = Fixed { raw: 4801919417 };
    // (local sphere, sphere at POSE) per shape of `all()`; the quarter turn maps (0, 1/2) to
    // (-1/2, 0).
    let expected: Span<(BoundingSphere, BoundingSphere)> = array![
        (
            BoundingSphereTrait::new(v(ZERO, ZERO), HALF),
            BoundingSphereTrait::new(v(ONE, TWO), HALF),
        ),
        (
            BoundingSphereTrait::new(v(ZERO, ZERO), sqrt5),
            BoundingSphereTrait::new(v(ONE, TWO), sqrt5),
        ),
        (
            BoundingSphereTrait::new(v(ZERO, ZERO), ONE + HALF),
            BoundingSphereTrait::new(v(ONE, TWO), ONE + HALF),
        ),
        (
            BoundingSphereTrait::new(v(ZERO, HALF), seg_r),
            BoundingSphereTrait::new(v(HALF, TWO), seg_r),
        ),
        (BoundingSphereTrait::new(v(ZERO, ZERO), MAX), BoundingSphereTrait::new(v(ONE, TWO), MAX)),
        (
            BoundingSphereTrait::new(v(ZERO, ZERO), sqrt2),
            BoundingSphereTrait::new(v(ONE, TWO), sqrt2),
        ),
    ]
        .span();
    let mut k = 0;
    for shape in all() {
        let (local, world) = *expected.at(k);
        assert_eq!((*shape).compute_local_bounding_sphere(), local);
        assert_eq!((*shape).compute_bounding_sphere(POSE), world);
        k += 1;
    }
    // The per-type methods agree with the dispatch.
    assert_eq!(
        ball().bounding_sphere(POSE), ball_alternatives::bounding_sphere_transformed(ball(), POSE),
    );
    assert_eq!(cuboid().bounding_sphere(POSE), cuboid().local_bounding_sphere().transform_by(POSE));
    assert_eq!(capsule().bounding_sphere(POSE), world_of(expected, 2));
    assert_eq!(segment().bounding_sphere(POSE), world_of(expected, 3));
    assert_eq!(halfspace().bounding_sphere(POSE).radius, MAX);
    assert_eq!(square().bounding_sphere(POSE), world_of(expected, 5));
    assert_eq!(
        segment_alternatives::local_bounding_sphere_point_cloud(segment()),
        segment().local_bounding_sphere(),
    );
}

#[test]
fn test_aabb_aliases_and_swept_box() {
    assert_eq!(ball().aabb(POSE), ball().compute_aabb(POSE));
    assert_eq!(ball().local_aabb(), ball().compute_local_aabb());
    assert_eq!(cuboid().aabb(POSE), cuboid().compute_aabb(POSE));
    assert_eq!(cuboid().local_aabb(), cuboid().compute_local_aabb());
    assert_eq!(capsule().aabb(POSE), capsule().compute_aabb(POSE));
    assert_eq!(capsule().local_aabb(), capsule().compute_local_aabb());
    assert_eq!(segment().aabb(POSE), segment().compute_aabb(POSE));
    assert_eq!(segment().local_aabb(), segment().compute_local_aabb());
    assert_eq!(halfspace().aabb(POSE), halfspace().compute_aabb(POSE));
    assert_eq!(halfspace().local_aabb(), halfspace().compute_local_aabb());
    assert_eq!(square().aabb(POSE), square().compute_aabb(POSE));
    assert_eq!(square().local_aabb(), square().compute_local_aabb());
    // Ball of radius 1/2 swept from the origin to (3, 1).
    let end = Pose2Trait::new(v(i(3), ONE), Rot2 { re: ONE, im: ZERO });
    let swept = Shape::Ball(ball()).compute_swept_aabb(Pose2Trait::IDENTITY, end);
    assert_eq!((swept.mins, swept.maxs), (v(-HALF, -HALF), v(i(3) + HALF, ONE + HALF)));
}

#[test]
fn test_feature_normals_at_point() {
    let diag = Fixed { raw: 3037000500 };
    let face0 = FeatureIdTrait::face(0);
    let vertex0 = FeatureIdTrait::vertex(0);
    // (shape, feature, point, normal).
    let cases: Span<(Shape, crate::feature_id::FeatureId, Vec2, Option<Vec2>)> = array![
        (Shape::Ball(ball()), FEATURE_UNKNOWN, v(ZERO, i(3)), Some(v(ZERO, ONE))),
        (Shape::Ball(ball()), FEATURE_UNKNOWN, v(ZERO, ZERO), None),
        (Shape::Cuboid(cuboid()), face0, v(TWO, ZERO), Some(v(ONE, ZERO))),
        (Shape::Capsule(capsule()), face0, v(HALF, ZERO), None),
        (
            Shape::Segment(SegmentTrait::new(v(ZERO, ZERO), v(TWO, ZERO))),
            face0,
            v(ONE, ZERO),
            Some(v(ZERO, -ONE)),
        ),
        (Shape::HalfSpace(halfspace()), face0, v(ZERO, ZERO), None),
        (Shape::ConvexPolygon(BoxTrait::new(square())), face0, v(ZERO, -ONE), Some(v(ZERO, -ONE))),
        (
            Shape::ConvexPolygon(BoxTrait::new(square())),
            vertex0,
            v(-ONE, -ONE),
            Some(v(-diag, -diag)),
        ),
    ]
        .span();
    for (shape, feature, point, normal) in cases {
        assert_eq!((*shape).feature_normal_at_point(0, *feature, *point), *normal);
    }
    // (3, 4) normalises to (0.6, 0.8) within 1 ulp.
    let n = Shape::Ball(ball()).feature_normal_at_point(0, FEATURE_UNKNOWN, v(i(3), i(4))).unwrap();
    assert!(n.x.abs_diff_eq(Fixed { raw: 2576980378 }, Fixed { raw: 1 }));
    assert!(n.y.abs_diff_eq(Fixed { raw: 3435973837 }, Fixed { raw: 1 }));
}

#[test]
fn test_polygon_feature_helpers() {
    let p = square();
    assert_eq!(p, SQUARE);
    // Faces: 0 = -y, 1 = +x, 2 = +y, 3 = -x (edge i joins vertex i to i + 1).
    assert_eq!(p.feature_normal(FeatureIdTrait::face(1)), Some(v(ONE, ZERO)));
    assert_eq!(p.feature_normal(FeatureIdTrait::face(4)), None);
    assert_eq!(p.feature_normal(FeatureIdTrait::vertex(8)), None);
    assert_eq!(p.feature_normal(FEATURE_UNKNOWN), None);
    // (direction, feature): half a degree off a normal picks the face, two degrees the vertex.
    let cases: Span<(Vec2, crate::feature_id::FeatureId)> = array![
        (v(ZERO, -ONE), FeatureIdTrait::face(0)), (v(ONE, ZERO), FeatureIdTrait::face(1)),
        (raw(37480185, -4294803757), FeatureIdTrait::face(0)),
        (raw(149892197, -4292350918), FeatureIdTrait::vertex(1)),
        (raw(3037000500, 3037000500), FeatureIdTrait::vertex(2)),
        (raw(-3037000500, 3037000500), FeatureIdTrait::vertex(3)),
    ]
        .span();
    for (dir, feature) in cases {
        assert_eq!(p.support_feature_id_toward(*dir), *feature);
    }
}

#[test]
fn test_scaled_half_height_and_support_face() {
    assert_eq!(cuboid().scaled(v(TWO, -ONE)).half_extents, v(i(4), -ONE));
    let s = segment().scaled(v(TWO, i(3)));
    assert_eq!((s.a, s.b), (v(-TWO, ZERO), v(TWO, i(3))));
    assert_eq!(halfspace().scaled(v(ONE, i(3))), Some(halfspace()));
    assert_eq!(halfspace().scaled(v(ONE, ZERO)), None);
    assert_eq!(
        HalfSpaceTrait::new(v(ONE, ZERO)).scaled(v(-TWO, ONE)),
        Some(HalfSpaceTrait::new(v(-ONE, ZERO))),
    );
    // (capsule, half height): floored length, halved.
    let cases: Span<(Capsule, Fixed)> = array![
        (capsule(), ONE), (CapsuleTrait::new(v(ZERO, ZERO), v(i(3), i(4)), ONE), i(5) * HALF),
        (CapsuleTrait::new(v(ONE, ONE), v(ONE, ONE), ONE), ZERO),
    ]
        .span();
    for (c, h) in cases {
        assert_eq!((*c).half_height(), *h);
        assert_eq!(capsule_alternatives::half_height_mul(*c), *h);
    }
    for dir in array![v(ONE, ZERO), v(ZERO, -ONE), v(-ONE, HALF)].span() {
        assert_eq!(cuboid().support_face(*dir), cuboid().support_feature(*dir));
    }
}

#[test]
fn test_support_and_feature_map_views() {
    for shape in all() {
        let expected = match *shape {
            Shape::HalfSpace(_) => None,
            _ => Some(*shape),
        };
        assert_eq!((*shape).as_support_map(), expected);
    }
    assert_eq!(Shape::Ball(ball()).as_polygonal_feature_map(), None);
    assert_eq!(Shape::HalfSpace(halfspace()).as_polygonal_feature_map(), None);
    assert_eq!(
        Shape::Cuboid(cuboid()).as_polygonal_feature_map(), Some((Shape::Cuboid(cuboid()), ZERO)),
    );
    assert_eq!(
        Shape::Capsule(capsule()).as_polygonal_feature_map(),
        Some((Shape::Segment(capsule().segment), HALF)),
    );
}

#[test]
fn gas_baseline() {}
#[test]
fn gas_ball_local_bounding_sphere() {
    let _ = opaque(ball()).local_bounding_sphere();
}
#[test]
fn gas_ball_bounding_sphere() {
    let _ = opaque(ball()).bounding_sphere(opaque(POSE));
}
#[test]
fn gas_ball_bounding_sphere_transformed() {
    let _ = ball_alternatives::bounding_sphere_transformed(opaque(ball()), opaque(POSE));
}
#[test]
fn gas_cuboid_bounding_sphere() {
    let _ = opaque(cuboid()).bounding_sphere(opaque(POSE));
}
#[test]
fn gas_capsule_bounding_sphere() {
    let _ = opaque(capsule()).bounding_sphere(opaque(POSE));
}
#[test]
fn gas_capsule_half_height_div() {
    let _ = opaque(capsule()).half_height();
}
#[test]
fn gas_capsule_half_height_mul() {
    let _ = capsule_alternatives::half_height_mul(opaque(capsule()));
}
#[test]
fn gas_segment_local_bounding_sphere() {
    let _ = opaque(segment()).local_bounding_sphere();
}
#[test]
fn gas_segment_local_bounding_sphere_point_cloud() {
    let _ = segment_alternatives::local_bounding_sphere_point_cloud(opaque(segment()));
}
#[test]
fn gas_halfspace_bounding_sphere() {
    let _ = opaque(halfspace()).bounding_sphere(opaque(POSE));
}
#[test]
fn gas_polygon_local_bounding_sphere() {
    let _ = opaque(SQUARE).local_bounding_sphere();
}
#[test]
fn gas_polygon_feature_normal_vertex() {
    let _ = opaque(SQUARE).feature_normal(opaque(FeatureIdTrait::vertex(0)));
}
#[test]
fn gas_polygon_support_feature_id_toward_vertex() {
    let _ = opaque(SQUARE).support_feature_id_toward(opaque(raw(3037000500, 3037000500)));
}
#[test]
fn gas_shape_compute_bounding_sphere_cuboid() {
    let _ = opaque(Shape::Cuboid(cuboid())).compute_bounding_sphere(opaque(POSE));
}
#[test]
fn gas_shape_compute_swept_aabb_ball() {
    let _ = opaque(Shape::Ball(ball()))
        .compute_swept_aabb(opaque(Pose2Trait::IDENTITY), opaque(POSE));
}
#[test]
fn gas_shape_feature_normal_at_point_cuboid() {
    let _ = opaque(Shape::Cuboid(cuboid()))
        .feature_normal_at_point(0, opaque(FeatureIdTrait::face(0)), opaque(v(TWO, ZERO)));
}
#[test]
fn gas_cuboid_scaled() {
    let _ = opaque(cuboid()).scaled(opaque(v(TWO, -ONE)));
}
#[test]
fn gas_halfspace_scaled() {
    let _ = opaque(halfspace()).scaled(opaque(v(ONE, i(3))));
}
