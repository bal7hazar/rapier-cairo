use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::query::ClosestPoints;
use crate::shape::{
    Capsule, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait, Shape,
};
use super::alternatives::{distance_exhaustive, distance_witness};
use super::{
    closest_points_support_map_support_map, contact_support_map_support_map,
    distance_support_map_support_map, witness,
};

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
}

fn quarter_at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2Trait::new(v(x, y), Rot2 { re: ZERO, im: ONE })
}

fn cuboid() -> Shape {
    Shape::Cuboid(CuboidTrait::new(v(ONE, HALF)))
}

fn capsule() -> Capsule {
    CapsuleTrait::new_y(HALF, HALF)
}

fn segment() -> Shape {
    Shape::Segment(SegmentTrait::new(v(-ONE, ZERO), v(ONE, ZERO)))
}

fn triangle() -> Shape {
    Shape::ConvexPolygon(
        BoxTrait::new(
            ConvexPolygonTrait::from_convex_polyline(
                array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span(),
            )
                .unwrap(),
        ),
    )
}

/// `(shape1, shape2, pos12, dist, normal1)`: exact answers of the analytic kernel.
#[test]
fn test_witness_table() {
    let cases: Span<(Shape, Shape, Pose2, Fixed, Vec2)> = array![
        // Separated faces: 3 - 1 - 1 = 1 along +x.
        (cuboid(), cuboid(), at(int(3), ZERO), ONE, v(ONE, ZERO)),
        // Overlapping faces: depth 0.25 along +y.
        (
            cuboid(),
            cuboid(),
            at(ZERO, HALF + HALF / FixedTrait::from_int(2)),
            -HALF / FixedTrait::from_int(2),
            v(ZERO, ONE),
        ),
        // Capsule above a segment: 2 - 0.5 - 0.5 = 1 along +y.
        (segment(), Shape::Capsule(capsule()), at(ZERO, TWO), ONE, v(ZERO, ONE)),
        // Collinear segments end to end, one unit apart: the direction axis separates them.
        (segment(), segment(), at(int(3), ZERO), ONE, v(ONE, ZERO)),
        // Crossing segments: their difference is a square, depth 1 (upstream's EPA agrees).
        (segment(), segment(), quarter_at(ZERO, ZERO), -ONE, v(ZERO, -ONE)),
        // Triangle apex under a cuboid: 2 - 1 - 0.5 = 0.5 along +y.
        (triangle(), cuboid(), at(ZERO, TWO), HALF, v(ZERO, ONE)),
    ]
        .span();
    for (s1, s2, pos12, dist, normal) in cases {
        let w = witness(*pos12, *s1, *s2);
        assert_eq!(w.dist, *dist);
        assert_eq!(w.normal1, *normal);
        // `point2 - point1 = dist * normal1`.
        assert_eq!(w.point2 - w.point1, v(*normal.x * *dist, *normal.y * *dist));
    }
}

#[test]
fn test_queries_follow_the_witness() {
    let pos12 = at(ZERO, TWO);
    let (s1, s2) = (segment(), Shape::Capsule(capsule()));
    assert_eq!(distance_support_map_support_map(pos12, s1, s2), ONE);
    assert_eq!(
        closest_points_support_map_support_map(pos12, s1, s2, HALF), ClosestPoints::Disjoint,
    );
    assert_eq!(
        closest_points_support_map_support_map(pos12, s1, s2, ONE),
        ClosestPoints::WithinMargin((v(ZERO, ZERO), v(ZERO, -ONE))),
    );
    assert!(contact_support_map_support_map(pos12, s1, s2, HALF).is_none());
    let c = contact_support_map_support_map(pos12, s1, s2, ONE).unwrap();
    assert_eq!(
        (c.point1, c.point2, c.normal2, c.dist), (v(ZERO, ZERO), v(ZERO, -ONE), v(ZERO, -ONE), ONE),
    );
    // Overlapping: `Intersecting`, zero distance.
    let deep = at(ZERO, HALF);
    assert_eq!(distance_support_map_support_map(deep, s1, s2), ZERO);
    assert_eq!(
        closest_points_support_map_support_map(deep, s1, s2, ONE), ClosestPoints::Intersecting,
    );
}

#[test]
#[should_panic(expected: 'Query: not a support map')]
fn test_halfspace_is_not_a_support_map() {
    let _ = witness(at(ZERO, ZERO), cuboid(), Shape::HalfSpace(HalfSpaceTrait::new(v(ZERO, ONE))));
}

/// The segment-pair candidate and the kernel agree on capsule distances.
#[test]
#[fuzzer(runs: 48, seed: 20260926)]
fn fuzz_capsule_distance_candidates_agree(x: i8, y: i8, turn: bool) {
    let pos12 = if turn {
        quarter_at(
            FixedTrait::from_int(x.into()) / FixedTrait::from_int(4),
            FixedTrait::from_int(y.into()) / FixedTrait::from_int(4),
        )
    } else {
        at(
            FixedTrait::from_int(x.into()) / FixedTrait::from_int(4),
            FixedTrait::from_int(y.into()) / FixedTrait::from_int(4),
        )
    };
    let kernel = distance_support_map_support_map(
        pos12, Shape::Capsule(capsule()), Shape::Capsule(capsule()),
    );
    let candidate = distance_witness(pos12, Shape::Capsule(capsule()), Shape::Capsule(capsule()));
    assert!(kernel.abs_diff_eq(candidate, Fixed { raw: 4 }));
}

/// The pruned vertex search answers what the exhaustive one does.
#[test]
#[fuzzer(runs: 48, seed: 20260926)]
fn fuzz_pruning_is_exact(x: i8, y: i8, turn: bool) {
    let tx = FixedTrait::from_int(x.into()) / FixedTrait::from_int(8);
    let ty = FixedTrait::from_int(y.into()) / FixedTrait::from_int(8);
    let pos12 = if turn {
        Pose2Trait::new(v(tx, ty), Rot2 { re: Fixed { raw: 3719550787 }, im: HALF })
    } else {
        at(tx, ty)
    };
    assert_eq!(
        distance_support_map_support_map(pos12, triangle(), cuboid()),
        distance_exhaustive(pos12, triangle(), cuboid()),
    );
}

/// Every answer is a witness: the distance is never below the distance of the two points
/// the kernel reports, and a separated pair is not penetrating.
#[test]
#[fuzzer(runs: 48, seed: 20260926)]
fn fuzz_witness_is_consistent(x: i8, y: i8) {
    let pos12 = at(
        FixedTrait::from_int(x.into()) / FixedTrait::from_int(8),
        FixedTrait::from_int(y.into()) / FixedTrait::from_int(8),
    );
    let w = witness(pos12, triangle(), cuboid());
    let d = w.point2 - w.point1;
    assert!(d.x.abs_diff_eq(w.normal1.x * w.dist, Fixed { raw: 16 }));
    assert!(d.y.abs_diff_eq(w.normal1.y * w.dist, Fixed { raw: 16 }));
}

#[test]
fn gas_baseline() {}

// Separated pairs (vertex–edge search), then overlapping ones (SAT witness).
#[test]
fn gas_distance_support_map_cuboid_cuboid() {
    let _ = distance_support_map_support_map(
        opaque(at(int(3), HALF)), opaque(cuboid()), opaque(cuboid()),
    );
}

#[test]
fn gas_distance_exhaustive_cuboid_cuboid() {
    let _ = distance_exhaustive(opaque(at(int(3), HALF)), opaque(cuboid()), opaque(cuboid()));
}

#[test]
fn gas_distance_exhaustive_triangle_triangle() {
    let _ = distance_exhaustive(opaque(at(int(3), ZERO)), opaque(triangle()), opaque(triangle()));
}

#[test]
fn gas_distance_support_map_capsule_capsule() {
    let _ = distance_support_map_support_map(
        opaque(quarter_at(TWO, ONE)),
        opaque(Shape::Capsule(capsule())),
        opaque(Shape::Capsule(capsule())),
    );
}

#[test]
fn gas_distance_witness_capsule_capsule() {
    let _ = distance_witness(
        opaque(quarter_at(TWO, ONE)),
        opaque(Shape::Capsule(capsule())),
        opaque(Shape::Capsule(capsule())),
    );
}

#[test]
fn gas_distance_support_map_triangle_triangle() {
    let _ = distance_support_map_support_map(
        opaque(at(int(3), ZERO)), opaque(triangle()), opaque(triangle()),
    );
}

#[test]
fn gas_contact_support_map_cuboid_cuboid_overlapping() {
    let _ = contact_support_map_support_map(
        opaque(at(ONE, HALF)), opaque(cuboid()), opaque(cuboid()), opaque(ZERO),
    );
}

#[test]
fn gas_contact_support_map_segment_capsule_overlapping() {
    let _ = contact_support_map_support_map(
        opaque(at(HALF, ZERO)), opaque(segment()), opaque(Shape::Capsule(capsule())), opaque(ZERO),
    );
}

#[test]
fn gas_contact_support_map_triangle_cuboid_overlapping() {
    let _ = contact_support_map_support_map(
        opaque(at(HALF, HALF)), opaque(triangle()), opaque(cuboid()), opaque(ZERO),
    );
}

#[test]
fn gas_closest_points_support_map_cuboid_capsule() {
    let _ = closest_points_support_map_support_map(
        opaque(at(int(3), ZERO)),
        opaque(cuboid()),
        opaque(Shape::Capsule(capsule())),
        opaque(int(10)),
    );
}
