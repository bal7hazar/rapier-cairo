use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::contact::ContactManifold;
use crate::shape::{
    BallTrait, CapsuleTrait, CuboidTrait, HeightFieldTrait, PolylineTrait, SegmentTrait, Shape,
};
use super::{contact_manifold_part, contact_manifolds_composite};

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
}

const PREDICTION: Fixed = Fixed { raw: 85899346 };

/// A "V" whose arms have slope 1: `(-2, 2) -> (0, 0) -> (2, 2)`.
fn vee() -> Shape {
    PolylineTrait::new(array![v(int(-2), TWO), v(ZERO, ZERO), v(TWO, TWO)].span(), None).into()
}

/// Flat ground at `y = 0` over `x in [-4, 4]`, four cells.
fn flat() -> Shape {
    HeightFieldTrait::new(array![ZERO, ZERO, ZERO, ZERO, ZERO].span(), v(int(8), ONE)).into()
}

fn points(manifolds: Span<ContactManifold>) -> Array<(u32, u32, u8)> {
    let mut out = array![];
    for m in manifolds {
        out.append((*m.subshape1, *m.subshape2, *m.num_points));
    }
    out
}

#[test]
fn test_ball_in_a_vee_touches_both_arms() {
    let ball = Shape::Ball(BallTrait::new(ONE));
    // Centre on the axis at `sqrt(2)`: tangent to both arms.
    let pos = at(ZERO, FixedTrait::from_raw(6074001000));
    let ms = contact_manifolds_composite(pos, vee(), ball, PREDICTION, array![].span()).unwrap();
    assert_eq!(points(ms.span()), array![(0, 0, 1), (1, 0, 1)]);
    // Composite second: the ids move to `subshape2`, the normals point from the ball.
    let ms2 = contact_manifolds_composite(pos.inverse(), ball, vee(), PREDICTION, array![].span())
        .unwrap();
    assert_eq!(points(ms2.span()), array![(0, 0, 1), (0, 1, 1)]);
    let [c, _] = (*ms2.at(1)).points;
    let [c1, _] = (*ms.at(1)).points;
    assert_eq!(c.dist, c1.dist);
}

#[test]
fn test_cuboid_on_heightfield_cells() {
    // A box over `x in [-1, 1.5]` resting on the ground, across the seam `x = 0`.
    let cuboid = Shape::Cuboid(CuboidTrait::new(v(ONE + FixedTrait::from_ratio(1, 4), HALF)));
    let pos = at(FixedTrait::from_ratio(1, 4), HALF);
    let ms = contact_manifolds_composite(pos, flat(), cuboid, PREDICTION, array![].span()).unwrap();
    // Cells 1 ([-2, 0]) and 2 ([0, 2]), two points each (the box bottom clipped to each cell).
    assert_eq!(points(ms.span()), array![(1, 0, 2), (2, 0, 2)]);
    // Warm start: the previous manifolds are found again by sub-shape id.
    let mut previous = ms;
    let mut m0 = *previous.at(0);
    let [mut p0, p1] = m0.points;
    p0.data.impulse = ONE;
    m0.points = [p0, p1];
    let again = contact_manifolds_composite(
        pos, flat(), cuboid, PREDICTION, array![*previous.at(1), m0].span(),
    )
        .unwrap();
    let [q0, _] = (*again.at(0)).points;
    assert_eq!(q0.data.impulse, ONE);
    // Far above: no part in the box, no manifold.
    let none = contact_manifolds_composite(
        at(ZERO, int(10)), flat(), cuboid, PREDICTION, array![].span(),
    )
        .unwrap();
    assert_eq!(none.len(), 0);
}

#[test]
fn test_unsupported_and_parts() {
    let ball = Shape::Ball(BallTrait::new(ONE));
    assert!(
        contact_manifolds_composite(at(ZERO, ZERO), ball, ball, PREDICTION, array![].span())
            .is_none(),
    );
    assert!(
        contact_manifolds_composite(at(ZERO, ZERO), vee(), flat(), PREDICTION, array![].span())
            .is_none(),
    );
    // Segment–capsule and segment–segment parts: the capsule generator with a zero radius.
    let seg = Shape::Segment(SegmentTrait::new(v(-ONE, ZERO), v(ONE, ZERO)));
    let cap = Shape::Capsule(CapsuleTrait::new_x(ONE, HALF));
    let mut m: ContactManifold = Default::default();
    assert!(contact_manifold_part(at(ZERO, HALF), seg, cap, PREDICTION, ref m));
    assert_eq!(m.num_points, 2);
    let mut m: ContactManifold = Default::default();
    assert!(contact_manifold_part(at(ZERO, ZERO), seg, seg, PREDICTION, ref m));
    let mut m: ContactManifold = Default::default();
    assert!(contact_manifold_part(at(ZERO, HALF), cap, seg, PREDICTION, ref m));
    assert_eq!(m.num_points, 2);
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_manifolds_ball_on_heightfield() {
    let _ = contact_manifolds_composite(
        opaque(at(ZERO, ONE)),
        opaque(flat()),
        opaque(Shape::Ball(BallTrait::new(ONE))),
        opaque(PREDICTION),
        array![].span(),
    );
}

#[test]
fn gas_manifolds_cuboid_on_heightfield() {
    let _ = contact_manifolds_composite(
        opaque(at(HALF, HALF)),
        opaque(flat()),
        opaque(Shape::Cuboid(CuboidTrait::new(v(ONE + HALF, HALF)))),
        opaque(PREDICTION),
        array![].span(),
    );
}

#[test]
fn gas_manifolds_ball_in_vee() {
    let _ = contact_manifolds_composite(
        opaque(at(ZERO, FixedTrait::from_raw(6074001000))),
        opaque(vee()),
        opaque(Shape::Ball(BallTrait::new(ONE))),
        opaque(PREDICTION),
        array![].span(),
    );
}
