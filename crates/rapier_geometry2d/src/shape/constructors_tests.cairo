//! PX2: `ShapeTrait`'s upstream `SharedShape::*` constructors build exactly the value their
//! `XTrait::new` counterpart does, wrapped in the matching `Shape` variant;
//! `crates/rapier_dynamics2d/src/collider/builder.cairo`'s `ColliderBuilderTrait` wraps the same
//! calls, so both sides stay in lock step without a cross-crate test dependency.

use fixed::{Fixed, HALF, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_math::pose2::IDENTITY;
use rapier_testing::opaque;
use super::{
    BallTrait, CapsuleTrait, CompoundTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait,
    HeightFieldTrait, PolylineTrait, RoundCuboid, RoundShapeTrait, RoundTriangle, SegmentTrait,
    Shape, ShapeTrait, TriangleTrait,
};

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

/// The simple (non-`Option`) constructors: each matches the `Shape` variant built directly.
#[test]
fn test_simple_constructors() {
    let (a, b, c) = (v(ZERO, ZERO), v(TWO, ZERO), v(ZERO, TWO));
    let round_cuboid: RoundCuboid = RoundShapeTrait::new(CuboidTrait::new(v(ONE, TWO)), HALF);
    let round_triangle: RoundTriangle = RoundShapeTrait::new(TriangleTrait::new(a, b, c), HALF);
    let cases = array![
        (ShapeTrait::ball(TWO), Shape::Ball(BallTrait::new(TWO))),
        (ShapeTrait::cuboid(TWO, ONE), Shape::Cuboid(CuboidTrait::new(v(TWO, ONE)))),
        (ShapeTrait::capsule(a, b, HALF), Shape::Capsule(CapsuleTrait::new(a, b, HALF))),
        (ShapeTrait::capsule_x(TWO, HALF), Shape::Capsule(CapsuleTrait::new_x(TWO, HALF))),
        (ShapeTrait::capsule_y(TWO, HALF), Shape::Capsule(CapsuleTrait::new_y(TWO, HALF))),
        (ShapeTrait::segment(a, b), Shape::Segment(SegmentTrait::new(a, b))),
        (ShapeTrait::halfspace(b), Shape::HalfSpace(HalfSpaceTrait::new(b))),
        (
            ShapeTrait::triangle(a, b, c),
            Shape::Triangle(BoxTrait::new(TriangleTrait::new(a, b, c))),
        ),
        (ShapeTrait::round_cuboid(ONE, TWO, HALF), Shape::RoundCuboid(round_cuboid)),
        (
            ShapeTrait::round_triangle(a, b, c, HALF),
            Shape::RoundTriangle(BoxTrait::new(round_triangle)),
        ),
    ];
    for (built, expected) in cases {
        assert_eq!(built, expected);
    }
}

/// The `Option`-returning polygon constructors: `Some` with the expected polygon, `None` in the
/// same degenerate cases as `ConvexPolygonTrait::from_convex_hull` / `from_convex_polyline`.
#[test]
fn test_polygon_constructors() {
    let square = [v(-ONE, -ONE), v(ONE, -ONE), v(ONE, ONE), v(-ONE, ONE)];
    let polygon = ConvexPolygonTrait::from_convex_polyline(square.span()).unwrap();
    let scrambled = [v(ONE, ONE), v(-ONE, ONE), v(ONE, -ONE), v(-ONE, -ONE)];
    let polygon_shape: Shape = polygon.into();
    assert_eq!(ShapeTrait::convex_polyline(square.span()).unwrap(), polygon_shape);
    assert_eq!(ShapeTrait::convex_hull(scrambled.span()).unwrap(), polygon_shape);
    let round_polygon = RoundShapeTrait::new(polygon, HALF);
    let round_shape: Shape = round_polygon.into();
    assert_eq!(ShapeTrait::round_convex_polyline(square.span(), HALF).unwrap(), round_shape);
    assert_eq!(ShapeTrait::round_convex_hull(scrambled.span(), HALF).unwrap(), round_shape);
    // Degenerate input (a segment): every polygon constructor answers `None`.
    let degenerate = [v(ZERO, ZERO), v(ONE, ONE)].span();
    assert!(ShapeTrait::convex_polyline(degenerate).is_none());
    assert!(ShapeTrait::convex_hull(degenerate).is_none());
    assert!(ShapeTrait::round_convex_polyline(degenerate, HALF).is_none());
    assert!(ShapeTrait::round_convex_hull(degenerate, HALF).is_none());
}

/// The composite constructors (upstream `polyline`, `heightfield`, `compound`): the shape they
/// wrap and their panic on an empty compound.
#[test]
fn test_composite_constructors() {
    let vertices = [v(ZERO, ZERO), v(ONE, ZERO), v(ONE, ONE)].span();
    let polyline_shape: Shape = PolylineTrait::new(vertices, Option::None).into();
    assert_eq!(ShapeTrait::polyline(vertices, Option::None), polyline_shape);
    let heights = [ONE, TWO, ONE].span();
    let heightfield_shape: Shape = HeightFieldTrait::new(heights, v(TWO, ONE)).into();
    assert_eq!(ShapeTrait::heightfield(heights, v(TWO, ONE)), heightfield_shape);
    let parts = array![(IDENTITY, ShapeTrait::ball(ONE))].span();
    let compound_shape: Shape = CompoundTrait::new(parts).into();
    assert_eq!(ShapeTrait::compound(parts), compound_shape);
}

#[test]
#[should_panic(expected: ('Compound: no part',))]
fn test_compound_rejects_no_part() {
    let _ = ShapeTrait::compound(array![].span());
}

#[test]
fn gas_baseline() {}
#[test]
fn gas_ball() {
    let _ = ShapeTrait::ball(opaque(ONE));
}
#[test]
fn gas_cuboid() {
    let _ = ShapeTrait::cuboid(opaque(ONE), opaque(TWO));
}
#[test]
fn gas_capsule() {
    let _ = ShapeTrait::capsule(opaque(v(ZERO, ZERO)), opaque(v(ONE, ZERO)), opaque(HALF));
}
#[test]
fn gas_capsule_x() {
    let _ = ShapeTrait::capsule_x(opaque(ONE), opaque(HALF));
}
#[test]
fn gas_capsule_y() {
    let _ = ShapeTrait::capsule_y(opaque(ONE), opaque(HALF));
}
#[test]
fn gas_segment() {
    let _ = ShapeTrait::segment(opaque(v(ZERO, ZERO)), opaque(v(ONE, ZERO)));
}
#[test]
fn gas_halfspace() {
    let _ = ShapeTrait::halfspace(opaque(v(ZERO, ONE)));
}
#[test]
fn gas_triangle() {
    let _ = ShapeTrait::triangle(opaque(v(ZERO, ZERO)), opaque(v(TWO, ZERO)), opaque(v(ZERO, TWO)));
}
#[test]
fn gas_round_cuboid() {
    let _ = ShapeTrait::round_cuboid(opaque(ONE), opaque(TWO), opaque(HALF));
}
#[test]
fn gas_round_triangle() {
    let _ = ShapeTrait::round_triangle(
        opaque(v(ZERO, ZERO)), opaque(v(TWO, ZERO)), opaque(v(ZERO, TWO)), opaque(HALF),
    );
}
#[test]
fn gas_convex_hull() {
    let points = opaque([v(ONE, ONE), v(-ONE, ONE), v(ONE, -ONE), v(-ONE, -ONE)]);
    let _ = ShapeTrait::convex_hull(points.span());
}
#[test]
fn gas_round_convex_hull() {
    let points = opaque([v(ONE, ONE), v(-ONE, ONE), v(ONE, -ONE), v(-ONE, -ONE)]);
    let _ = ShapeTrait::round_convex_hull(points.span(), opaque(HALF));
}
#[test]
fn gas_convex_polyline() {
    let points = opaque([v(-ONE, -ONE), v(ONE, -ONE), v(ONE, ONE), v(-ONE, ONE)]);
    let _ = ShapeTrait::convex_polyline(points.span());
}
#[test]
fn gas_round_convex_polyline() {
    let points = opaque([v(-ONE, -ONE), v(ONE, -ONE), v(ONE, ONE), v(-ONE, ONE)]);
    let _ = ShapeTrait::round_convex_polyline(points.span(), opaque(HALF));
}
#[test]
fn gas_polyline() {
    let vertices = opaque([v(ZERO, ZERO), v(ONE, ZERO), v(ONE, ONE)]);
    let _ = ShapeTrait::polyline(vertices.span(), Option::None);
}
#[test]
fn gas_heightfield() {
    let heights = opaque([ONE, TWO, ONE]);
    let _ = ShapeTrait::heightfield(heights.span(), opaque(v(TWO, ONE)));
}
#[test]
fn gas_compound() {
    let parts = opaque(array![(IDENTITY, ShapeTrait::ball(ONE))].span());
    let _ = ShapeTrait::compound(parts);
}
