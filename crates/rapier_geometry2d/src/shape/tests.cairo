use fixed::{Fixed, HALF, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::mass::MassProperties;
use super::{
    Ball, BallTrait, Capsule, CapsuleTrait, ConvexPolygonTrait, Cuboid, CuboidTrait, HalfSpace,
    HalfSpaceTrait, Segment, SegmentTrait, Shape, ShapeTrait, ShapeType,
};

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn pose() -> Pose2 {
    Pose2Trait::new(v(ONE, TWO), Rot2 { re: ZERO, im: ONE })
}

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

fn count(b: bool) -> u8 {
    if b {
        1
    } else {
        0
    }
}

fn all() -> Span<Shape> {
    array![
        Shape::Ball(ball()), Shape::Cuboid(cuboid()), Shape::Capsule(capsule()),
        Shape::Segment(segment()), Shape::HalfSpace(halfspace()),
    ]
        .span()
}

#[test]
fn test_shape_type_and_accessors() {
    let types = array![
        ShapeType::Ball, ShapeType::Cuboid, ShapeType::Capsule, ShapeType::Segment,
        ShapeType::HalfSpace,
    ];
    let mut k = 0;
    for shape in all() {
        assert_eq!((*shape).shape_type(), *types.at(k));
        // Exactly one accessor answers, and it returns the wrapped value.
        let hits = count((*shape).as_ball().is_some())
            + count((*shape).as_cuboid().is_some())
            + count((*shape).as_capsule().is_some())
            + count((*shape).as_segment().is_some())
            + count((*shape).as_halfspace().is_some());
        assert_eq!(hits, 1_u8);
        k += 1;
    }
    assert_eq!(Shape::Ball(ball()).as_ball(), Some(ball()));
    assert_eq!(Shape::Cuboid(cuboid()).as_cuboid(), Some(cuboid()));
    assert_eq!(Shape::Capsule(capsule()).as_capsule(), Some(capsule()));
    assert_eq!(Shape::Segment(segment()).as_segment(), Some(segment()));
    assert_eq!(Shape::HalfSpace(halfspace()).as_halfspace(), Some(halfspace()));
}

#[test]
fn test_dispatch_matches_the_shape_methods() {
    let p = pose();
    assert_eq!(Shape::Ball(ball()).compute_aabb(p), ball().compute_aabb(p));
    assert_eq!(Shape::Cuboid(cuboid()).compute_aabb(p), cuboid().compute_aabb(p));
    assert_eq!(Shape::Capsule(capsule()).compute_aabb(p), capsule().compute_aabb(p));
    assert_eq!(Shape::Segment(segment()).compute_aabb(p), segment().compute_aabb(p));
    assert_eq!(Shape::HalfSpace(halfspace()).compute_aabb(p), halfspace().compute_aabb(p));
    assert_eq!(Shape::Ball(ball()).compute_local_aabb(), ball().compute_local_aabb());
    assert_eq!(Shape::Cuboid(cuboid()).compute_local_aabb(), cuboid().compute_local_aabb());
    assert_eq!(Shape::Capsule(capsule()).compute_local_aabb(), capsule().compute_local_aabb());
    assert_eq!(Shape::Segment(segment()).compute_local_aabb(), segment().compute_local_aabb());
    assert_eq!(
        Shape::HalfSpace(halfspace()).compute_local_aabb(), halfspace().compute_local_aabb(),
    );
    assert_eq!(Shape::Ball(ball()).mass_properties(ONE), ball().mass_properties(ONE));
    assert_eq!(Shape::Cuboid(cuboid()).mass_properties(ONE), cuboid().mass_properties(ONE));
    assert_eq!(Shape::Capsule(capsule()).mass_properties(ONE), capsule().mass_properties(ONE));
}

#[test]
fn test_every_shape_is_convex_and_converts_into_shape() {
    let polygon = ConvexPolygonTrait::from_convex_polyline(
        [v(ZERO, ZERO), v(ONE, ZERO), v(ZERO, ONE)].span(),
    )
        .unwrap();
    let shapes: Array<Shape> = array![
        ball().into(), cuboid().into(), capsule().into(), segment().into(), halfspace().into(),
        polygon.into(),
    ];
    let expected = array![
        Shape::Ball(ball()), Shape::Cuboid(cuboid()), Shape::Capsule(capsule()),
        Shape::Segment(segment()), Shape::HalfSpace(halfspace()),
        Shape::ConvexPolygon(BoxTrait::new(polygon)),
    ];
    let mut k = 0;
    for shape in shapes {
        assert_eq!(shape, *expected.at(k));
        assert!(shape.is_convex());
        k += 1;
    }
}

#[test]
fn test_segments_and_half_spaces_are_massless() {
    let zero: MassProperties = Default::default();
    assert_eq!(Shape::Segment(segment()).mass_properties(TWO), zero);
    assert_eq!(Shape::HalfSpace(halfspace()).mass_properties(TWO), zero);
    // Every local box is well formed.
    for shape in all() {
        let local = (*shape).compute_local_aabb();
        assert!(local.mins.x <= local.maxs.x && local.mins.y <= local.maxs.y);
    }
}

#[test]
fn gas_baseline() {}
#[test]
fn gas_shape_type() {
    let _ = opaque(Shape::Cuboid(cuboid())).shape_type();
}
#[test]
fn gas_is_convex() {
    let _ = opaque(Shape::Cuboid(cuboid())).is_convex();
}
#[test]
fn gas_as_cuboid() {
    let _ = opaque(Shape::Cuboid(cuboid())).as_cuboid();
}
#[test]
fn gas_compute_local_aabb() {
    let _ = opaque(Shape::Cuboid(cuboid())).compute_local_aabb();
}
// Out of line, Sierra gas equalises the branches of a `match`: every variant costs as much
// as the most expensive one (the `_outlined` probes are equal); inlined, each pays its arm.
#[test]
fn gas_compute_aabb_ball() {
    let _ = opaque(Shape::Ball(ball())).compute_aabb(opaque(pose()));
}
#[test]
fn gas_compute_aabb_capsule() {
    let _ = opaque(Shape::Capsule(capsule())).compute_aabb(opaque(pose()));
}
#[test]
fn gas_compute_aabb_ball_outlined() {
    let _ = super::alternatives::compute_aabb_outlined(opaque(Shape::Ball(ball())), opaque(pose()));
}
#[test]
fn gas_compute_aabb_capsule_outlined() {
    let _ = super::alternatives::compute_aabb_outlined(
        opaque(Shape::Capsule(capsule())), opaque(pose()),
    );
}
#[test]
fn test_compute_aabb_outlined_agrees() {
    for shape in all() {
        assert_eq!(
            super::alternatives::compute_aabb_outlined(*shape, pose()),
            (*shape).compute_aabb(pose()),
        );
    }
}
#[test]
fn gas_mass_properties_ball() {
    let _ = opaque(Shape::Ball(ball())).mass_properties(opaque(ONE));
}
#[test]
fn gas_mass_properties_capsule() {
    let _ = opaque(Shape::Capsule(capsule())).mass_properties(opaque(ONE));
}
