//! Tests and `gas_*` probes of `crate::dispatch::basic`.

use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam_core::Vec2;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::contact::ContactManifold;
use crate::shape::{Ball, Capsule, ConvexPolygonTrait, Cuboid, HalfSpace, Segment, Shape};
use super::contact_manifold_step_basic;
use super::super::contact_manifold_step;

const PREDICTION: Fixed = Fixed { raw: 85899346 };
/// 30 degrees.
const R_30: Rot2 = Rot2 { re: Fixed { raw: 3719550787 }, im: HALF };

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn pentagon() -> Shape {
    let quarter = FixedTrait::from_ratio(1, 4);
    let points = array![
        v(-HALF, -HALF), v(HALF, -HALF), v(HALF + quarter, ZERO), v(ZERO, HALF),
        v(-HALF - quarter, ZERO),
    ];
    Shape::ConvexPolygon(
        BoxTrait::new(ConvexPolygonTrait::from_convex_polyline(points.span()).unwrap()),
    )
}

/// The four basic shapes.
fn basic_shapes() -> Array<Shape> {
    array![
        Shape::Ball(Ball { radius: HALF }), Shape::Cuboid(Cuboid { half_extents: v(ONE, HALF) }),
        pentagon(), Shape::HalfSpace(HalfSpace { normal: v(ZERO, ONE) }),
    ]
}

/// `pos12` moved by `(dx, dy)` raw.
fn moved(pos12: Pose2, dx: i64, dy: i64) -> Pose2 {
    Pose2 {
        translation: pos12.translation + v(Fixed { raw: dx }, Fixed { raw: dy }),
        rotation: pos12.rotation,
    }
}

/// Both tables from the same manifolds: same result, same manifold.
fn same(
    pos12: Pose2, shape1: Shape, shape2: Shape, ref a: ContactManifold, ref b: ContactManifold,
) -> bool {
    let got = contact_manifold_step_basic(pos12, shape1, shape2, PREDICTION, ref a);
    let expected = contact_manifold_step(pos12, shape1, shape2, PREDICTION, ref b);
    got == expected && a == b
}

/// Every ordered pair of basic shapes at a separated, a resting, a shallow-rotated (both sides)
/// and a deep pose, three calls on the same manifold (cold, warm and barely moved, moved beyond
/// the persistence tolerance): bit-identical to `contact_manifold_step` on every call.
#[test]
fn test_basic_matches_step_on_every_basic_pair() {
    let level = Rot2 { re: ONE, im: ZERO };
    let poses = array![
        Pose2 { translation: v(ZERO, FixedTrait::from_int(3)), rotation: level },
        Pose2 { translation: v(ZERO, ONE), rotation: level },
        Pose2 { translation: v(HALF, FixedTrait::from_ratio(1, 8)), rotation: R_30 },
        Pose2 { translation: v(HALF, FixedTrait::from_ratio(-1, 8)), rotation: R_30 },
        Pose2 { translation: v(FixedTrait::from_ratio(1, 10), HALF), rotation: R_30 },
    ];
    let shapes = basic_shapes();
    let mut supported: u32 = 0;
    for pos12 in poses.span() {
        for s1 in shapes.span() {
            for s2 in shapes.span() {
                let mut a: ContactManifold = Default::default();
                let mut b: ContactManifold = Default::default();
                assert!(same(*pos12, *s1, *s2, ref a, ref b));
                assert!(same(moved(*pos12, 4294, 0), *s1, *s2, ref a, ref b));
                assert!(same(moved(*pos12, 42949672, -4294967), *s1, *s2, ref a, ref b));
                if a.num_points != 0 {
                    supported += 1;
                }
            }
        }
    }
    // The poses do produce contacts (the comparison is not between empty manifolds only).
    assert!(supported > 40, "pairs with contacts {supported}");
}

/// A half-space pair is unsupported, as in the full table.
#[test]
fn test_halfspace_pair_is_unsupported() {
    let h = Shape::HalfSpace(HalfSpace { normal: v(ZERO, ONE) });
    let mut m: ContactManifold = Default::default();
    let pos12 = Pose2 { translation: v(ZERO, ZERO), rotation: R_30 };
    assert!(!contact_manifold_step_basic(pos12, h, h, PREDICTION, ref m));
    assert_eq!(m.num_points, 0);
}

#[test]
#[should_panic(expected: 'Dispatch: not a basic shape')]
fn test_capsule_ball_panics() {
    let capsule = Shape::Capsule(
        Capsule { segment: Segment { a: v(-HALF, ZERO), b: v(HALF, ZERO) }, radius: HALF },
    );
    let mut m: ContactManifold = Default::default();
    let pos12 = Pose2 { translation: v(ZERO, HALF), rotation: R_30 };
    let _ = contact_manifold_step_basic(
        pos12, capsule, Shape::Ball(Ball { radius: HALF }), PREDICTION, ref m,
    );
}

#[test]
#[should_panic(expected: 'Dispatch: not a basic shape')]
fn test_cuboid_segment_panics() {
    let segment = Shape::Segment(Segment { a: v(-HALF, ZERO), b: v(HALF, ZERO) });
    let mut m: ContactManifold = Default::default();
    let pos12 = Pose2 { translation: v(ZERO, HALF), rotation: R_30 };
    let cuboid = Shape::Cuboid(Cuboid { half_extents: v(ONE, HALF) });
    let _ = contact_manifold_step_basic(pos12, cuboid, segment, PREDICTION, ref m);
}

// Opaque inputs of the probes.
fn pd() -> Pose2 {
    opaque(Pose2 { translation: v(HALF, FixedTrait::from_ratio(1, 8)), rotation: R_30 })
}
fn ball() -> Shape {
    opaque(Shape::Ball(Ball { radius: HALF }))
}
fn cuboid() -> Shape {
    opaque(Shape::Cuboid(Cuboid { half_extents: v(ONE, HALF) }))
}
fn polygon() -> Shape {
    opaque(pentagon())
}

#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}

#[test]
fn gas_basic_ball_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_step_basic(pd(), ball(), ball(), PREDICTION, ref m);
}

#[test]
fn gas_step_ball_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_step(pd(), ball(), ball(), PREDICTION, ref m);
}

#[test]
fn gas_basic_cuboid_cuboid() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_step_basic(pd(), cuboid(), cuboid(), PREDICTION, ref m);
}

#[test]
fn gas_step_cuboid_cuboid() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_step(pd(), cuboid(), cuboid(), PREDICTION, ref m);
}

#[test]
fn gas_basic_polygon_polygon() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_step_basic(pd(), polygon(), polygon(), PREDICTION, ref m);
}

#[test]
fn gas_step_polygon_polygon() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_step(pd(), polygon(), polygon(), PREDICTION, ref m);
}
