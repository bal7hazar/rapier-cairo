use fixed::{Fixed, HALF, ONE};
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::contact::{ContactData, ContactManifold, ContactManifoldTrait, TrackedContact};
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::shape::{Ball, Capsule, Cuboid, HalfSpace, Segment, Shape};
use super::alternatives::{
    contact_manifold_convex_ball_early_reject, contact_manifold_convex_ball_inlined,
    contact_manifold_convex_ball_separate_norm,
};
use super::{
    contact_manifold_ball_convex, contact_manifold_convex_ball, contact_manifold_convex_ball_shapes,
};

const UNIT: i64 = 0x1_0000_0000;
const PRED_RAW: i64 = 0x1000_0000;
/// `2^-4`, exactly representable: the prediction of the boundary cases.
const PREDICTION: Fixed = Fixed { raw: PRED_RAW };
/// Tolerance of the non-dyadic normals: `1 ulp / |dpos|` plus the rounding of two products.
const TOL: i64 = 3;

#[derive(Copy, Drop)]
struct Case {
    shape: Shape,
    centre: Vec2,
    radius: Fixed,
    num_points: u8,
    /// Manifold distance (already net of the ball radius).
    dist: i64,
    /// Normal in the convex frame, from the shape towards the ball.
    n: Vec2,
    /// Contact point on the convex shape.
    p: Vec2,
    fid: FeatureId,
}

fn f(raw: i64) -> Fixed {
    Fixed { raw }
}

fn v(x: i64, y: i64) -> Vec2 {
    Vec2 { x: f(x), y: f(y) }
}

fn ball(radius: Fixed) -> Ball {
    Ball { radius }
}

fn translation(x: i64, y: i64) -> Pose2 {
    Pose2Trait::new(v(x, y), Rot2 { re: f(UNIT), im: f(0) })
}

fn fresh() -> ContactManifold {
    Default::default()
}

fn cuboid() -> Shape {
    Shape::Cuboid(Cuboid { half_extents: v(UNIT, UNIT / 2) })
}

fn segment() -> Shape {
    Shape::Segment(Segment { a: v(-UNIT, 0), b: v(UNIT, 0) })
}

fn halfspace() -> Shape {
    Shape::HalfSpace(HalfSpace { normal: v(0, UNIT) })
}

fn capsule() -> Shape {
    Shape::Capsule(Capsule { segment: Segment { a: v(0, -UNIT), b: v(0, UNIT) }, radius: HALF })
}

fn unit_ball() -> Shape {
    Shape::Ball(ball(ONE))
}

/// `x / 4` and `y / 4` of the 3-4-5 direction, i.e. `(0.6, 0.8)` in raw units.
const N35: Vec2 = Vec2 { x: Fixed { raw: 2576980378 }, y: Fixed { raw: 3435973837 } };
const NONE: Vec2 = Vec2 { x: Fixed { raw: 0 }, y: Fixed { raw: 0 } };

fn case(
    shape: Shape,
    cx: i64,
    cy: i64,
    radius: i64,
    dist: i64,
    n: Vec2,
    px: i64,
    py: i64,
    fid: FeatureId,
) -> Case {
    let num_points = if n == NONE {
        0
    } else {
        1
    };
    Case { shape, centre: v(cx, cy), radius: f(radius), num_points, dist, n, p: v(px, py), fid }
}

fn close(actual: Fixed, expected: i64, tol: i64) -> bool {
    let d = actual.raw - expected;
    -tol <= d && d <= tol
}

fn close_vec(actual: Vec2, expected: Vec2, tol: i64) -> bool {
    close(actual.x, expected.x.raw, tol) && close(actual.y, expected.y.raw, tol)
}

fn cases() -> Span<Case> {
    let up = v(0, UNIT);
    let right = v(UNIT, 0);
    let (u1, u2, u4, u8) = (UNIT, UNIT / 2, UNIT / 4, UNIT / 8);
    let f0 = FeatureIdTrait::face(0);
    array![
        // Cuboid, half extents (1, 0.5): beyond the reach, exactly at it, one ulp past it.
        case(cuboid(), 3 * u1, 0, u2, 0, NONE, 0, 0, f0),
        case(cuboid(), 6710886400, u4, u2, PRED_RAW, right, u1, u4, f0),
        case(cuboid(), 6710886401, u4, u2, 0, NONE, 0, 0, f0),
        // Touching a face, deep inside (through the -y face), touching a vertex.
        case(cuboid(), 3 * u2, u4, u2, 0, right, u1, u4, f0),
        case(cuboid(), u4, -u8, u2, -7 * u8, v(0, -u1), u4, -u2, FeatureIdTrait::face(3)),
        case(cuboid(), 7 * u4, 3 * u2, 5 * u4, 0, N35, u1, u2, FeatureIdTrait::vertex(0)),
        // Segment: above (side 1), vertex region, centre exactly on it (fallback normal).
        case(segment(), u2, u1, u1, 0, up, u2, 0, FeatureIdTrait::face(1)),
        case(segment(), 7 * u4, u1, 5 * u4, 0, N35, u1, 0, FeatureIdTrait::vertex(1)),
        case(segment(), u2, 0, u1, -u1, v(-u1, 0), u2, 0, f0),
        // Half-space: touching, far inside, centre on the origin (fallback normal, inside).
        case(halfspace(), u4, u1, u1, 0, up, u4, 0, f0),
        case(halfspace(), 2 * u1, -3 * u1, u1, -4 * u1, up, 2 * u1, 0, f0),
        case(halfspace(), 0, 0, u1, -u1, v(0, -u1), 0, 0, f0),
        // Capsule: touching the side, centre inside.
        case(capsule(), 3 * u2, u4, u1, 0, right, u2, u4, f0),
        case(capsule(), u4, 0, u2, -3 * u4, right, u2, 0, f0),
        // Ball as shape 1: touching (3-4-5), centre at the centre (projects on +x).
        case(unit_ball(), 3 * u2, 2 * u1, 3 * u2, 0, N35, 2576980378, 3435973837, f0),
        case(unit_ball(), 0, 0, u2, -3 * u2, right, u1, 0, f0),
    ]
        .span()
}

/// Checks one manifold of `case`; `flipped` tells that the ball is shape 1 of `m` and that
/// the pose was the identity-rotation translation of the ball in the convex frame.
fn check(case: Case, m: ContactManifold, flipped: bool) {
    assert_eq!(m.num_points, case.num_points);
    if case.num_points == 0 {
        return;
    }
    let c = m.point(0);
    let face = FeatureIdTrait::face(0);
    assert!(close(c.dist, case.dist, TOL), "dist {} vs {}", c.dist.raw, case.dist);
    let (n_convex, n_ball) = if flipped {
        (m.local_n2, m.local_n1)
    } else {
        (m.local_n1, m.local_n2)
    };
    let (p_convex, p_ball, fid_convex, fid_ball) = if flipped {
        (c.local_p2, c.local_p1, c.fid2, c.fid1)
    } else {
        (c.local_p1, c.local_p2, c.fid1, c.fid2)
    };
    assert!(close_vec(n_convex, case.n, TOL), "n {:?} vs {:?}", n_convex, case.n);
    assert!(close_vec(p_convex, case.p, TOL), "p {:?} vs {:?}", p_convex, case.p);
    // Identity rotation: the ball normal is the opposite one, and its point sits on the circle.
    assert!(close_vec(n_ball, Vec2 { x: -case.n.x, y: -case.n.y }, TOL));
    assert!(close_vec(p_ball, Vec2 { x: n_ball.x * case.radius, y: n_ball.y * case.radius }, 1));
    assert_eq!(fid_convex, case.fid);
    assert_eq!(fid_ball, face);
    assert_eq!(c.data, Default::default());
}

#[test]
fn test_regimes_convex_first() {
    for case in cases() {
        let mut m = fresh();
        contact_manifold_convex_ball(
            translation((*case).centre.x.raw, (*case).centre.y.raw),
            (*case).shape,
            ball((*case).radius),
            PREDICTION,
            ref m,
        );
        check(*case, m, false);
    }
}

#[test]
fn test_regimes_ball_first() {
    // The same scenes seen from the ball: `pos12` is the convex shape in the ball frame.
    for case in cases() {
        let mut m = fresh();
        contact_manifold_ball_convex(
            translation(-(*case).centre.x.raw, -(*case).centre.y.raw),
            ball((*case).radius),
            (*case).shape,
            PREDICTION,
            ref m,
        );
        check(*case, m, true);
    }
}

#[test]
fn test_normals_follow_the_rotation() {
    // Ball touching the +x face of the cuboid, turned a quarter turn: n2 = -R^-1 n1 = +y.
    let pos12 = Pose2Trait::new(v(3 * UNIT / 2, UNIT / 4), Rot2 { re: f(0), im: f(UNIT) });
    let mut m = fresh();
    contact_manifold_convex_ball(pos12, cuboid(), ball(HALF), PREDICTION, ref m);
    assert_eq!(m.local_n1, v(UNIT, 0));
    assert_eq!(m.local_n2, v(0, UNIT));
    assert_eq!(m.point(0).local_p2, v(0, UNIT / 2));
    // Flipped, the same scene: the ball is shape 1, `pos12` becomes the inverse pose.
    let pos21 = pos12.inverse();
    let mut m = fresh();
    contact_manifold_ball_convex(pos21, ball(HALF), cuboid(), PREDICTION, ref m);
    assert_eq!(m.local_n2, v(UNIT, 0));
    assert_eq!(m.local_n1, v(0, UNIT));
    assert_eq!(m.point(0).local_p1, v(0, UNIT / 2));
    assert_eq!(m.point(0).fid1, FeatureIdTrait::face(0));
    assert_eq!(m.point(0).fid2, FeatureIdTrait::face(0));
}

#[test]
fn test_warm_start_data_and_stale_state() {
    let data = ContactData { impulse: ONE, ..Default::default() };
    let old = TrackedContact { data, ..Default::default() };
    let pose = translation(3 * UNIT / 2, UNIT / 4);
    // Exactly one point: the data survives, the geometry is refreshed.
    let mut m = fresh();
    m.points = [old, old];
    m.num_points = 1;
    contact_manifold_convex_ball(pose, cuboid(), ball(HALF), PREDICTION, ref m);
    assert_eq!((m.num_points, m.point(0).data), (1, data));
    // Zero or two points: cleared first, so the data restarts from default.
    for stale in array![0_u8, 2].span() {
        let mut m = fresh();
        m.points = [old, old];
        m.num_points = *stale;
        contact_manifold_convex_ball(pose, cuboid(), ball(HALF), PREDICTION, ref m);
        assert_eq!((m.num_points, m.point(0).data), (1, Default::default()));
    }
    // Beyond the reach: cleared, normals kept.
    let mut m = fresh();
    contact_manifold_convex_ball(pose, cuboid(), ball(HALF), PREDICTION, ref m);
    let n1 = m.local_n1;
    contact_manifold_convex_ball(translation(9 * UNIT, 0), cuboid(), ball(HALF), PREDICTION, ref m);
    assert_eq!((m.num_points, m.local_n1), (0, n1));
}

#[test]
fn test_shapes_wrapper_dispatch() {
    let pose = translation(3 * UNIT / 2, UNIT / 4);
    let ball_shape = Shape::Ball(ball(HALF));
    let mut expected = fresh();
    contact_manifold_convex_ball(pose, cuboid(), ball(HALF), PREDICTION, ref expected);
    let mut m = fresh();
    assert!(contact_manifold_convex_ball_shapes(pose, cuboid(), ball_shape, PREDICTION, ref m));
    assert_eq!(m, expected);
    let mut expected = fresh();
    contact_manifold_ball_convex(pose, ball(HALF), cuboid(), PREDICTION, ref expected);
    let mut m = fresh();
    assert!(contact_manifold_convex_ball_shapes(pose, ball_shape, cuboid(), PREDICTION, ref m));
    assert_eq!(m, expected);
    // Neither is a ball: unhandled, untouched.
    let mut m = fresh();
    for other in array![segment(), halfspace(), capsule(), cuboid()].span() {
        assert!(!contact_manifold_convex_ball_shapes(pose, cuboid(), *other, PREDICTION, ref m));
    }
    assert_eq!(m, fresh());
}

#[test]
#[fuzzer(runs: 64, seed: 20260921)]
fn fuzz_alternatives_match(x: i16, y: i16, r: u8) {
    let radius = ball(f(r.into() * 0x0100_0000));
    let pose = Pose2Trait::new(
        v(x.into() * 0x2_0000 + 3, y.into() * 0x1_8000 - 5),
        Rot2 { re: f(2576980378), im: f(3435973837) },
    );
    for shape in array![cuboid(), segment(), halfspace(), capsule(), unit_ball()].span() {
        let mut a = fresh();
        let mut b = fresh();
        let mut c = fresh();
        let mut d = fresh();
        contact_manifold_convex_ball(pose, *shape, radius, PREDICTION, ref a);
        contact_manifold_convex_ball_separate_norm(pose, *shape, radius, PREDICTION, false, ref b);
        contact_manifold_convex_ball_early_reject(pose, *shape, radius, PREDICTION, false, ref c);
        contact_manifold_convex_ball_inlined(pose, *shape, radius, PREDICTION, ref d);
        assert_eq!(a, b);
        assert_eq!(a, c);
        assert_eq!(a, d);
    }
}

#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}

/// A ball of radius 0.5 whose centre is `0.03` outside the reference shape, so that every
/// shape produces a point and none is in a degenerate branch.
fn probe(shape: Shape, x: i64, y: i64) {
    let mut m = fresh();
    contact_manifold_convex_ball(
        opaque(translation(x, y)), opaque(shape), opaque(ball(HALF)), opaque(PREDICTION), ref m,
    );
}

#[test]
fn gas_convex_ball_cuboid() {
    probe(cuboid(), 3 * UNIT / 2, UNIT / 4);
}

#[test]
fn gas_convex_ball_capsule() {
    probe(capsule(), UNIT, UNIT / 4);
}

#[test]
fn gas_convex_ball_segment() {
    probe(segment(), UNIT / 2, UNIT / 2);
}

#[test]
fn gas_convex_ball_halfspace() {
    probe(halfspace(), UNIT / 2, UNIT / 2);
}

#[test]
fn gas_convex_ball_ball() {
    probe(unit_ball(), 3 * UNIT / 2, UNIT / 4);
}

#[test]
fn gas_convex_ball_cuboid_separated() {
    probe(cuboid(), 5 * UNIT, UNIT / 4);
}

/// Opaque `(pose, ball, prediction)` of the alternative and wrapper probes: the ball touches
/// the +x face of the cuboid, `x` is its centre abscissa.
fn opaque_args(x: i64) -> (Pose2, Ball, Fixed) {
    (opaque(translation(x, UNIT / 4)), opaque(ball(HALF)), opaque(PREDICTION))
}

#[test]
fn gas_ball_convex_cuboid() {
    let (pose, b, pred) = opaque_args(-3 * UNIT / 2);
    let mut m = fresh();
    contact_manifold_ball_convex(pose, b, opaque(cuboid()), pred, ref m);
}

#[test]
fn gas_convex_ball_shapes_flipped() {
    let (pose, b, pred) = opaque_args(-3 * UNIT / 2);
    let mut m = fresh();
    let _ = contact_manifold_convex_ball_shapes(
        pose, opaque(Shape::Ball(b)), opaque(cuboid()), pred, ref m,
    );
}

#[test]
fn gas_convex_ball_shapes_direct() {
    let (pose, b, pred) = opaque_args(3 * UNIT / 2);
    let mut m = fresh();
    let _ = contact_manifold_convex_ball_shapes(
        pose, opaque(cuboid()), opaque(Shape::Ball(b)), pred, ref m,
    );
}

#[test]
fn gas_convex_ball_cuboid_separate_norm() {
    let (pose, b, pred) = opaque_args(3 * UNIT / 2);
    let mut m = fresh();
    contact_manifold_convex_ball_separate_norm(pose, opaque(cuboid()), b, pred, false, ref m);
}

#[test]
fn gas_convex_ball_cuboid_early_reject() {
    let (pose, b, pred) = opaque_args(3 * UNIT / 2);
    let mut m = fresh();
    contact_manifold_convex_ball_early_reject(pose, opaque(cuboid()), b, pred, false, ref m);
}

#[test]
fn gas_convex_ball_cuboid_early_reject_separated() {
    let (pose, b, pred) = opaque_args(5 * UNIT);
    let mut m = fresh();
    contact_manifold_convex_ball_early_reject(pose, opaque(cuboid()), b, pred, false, ref m);
}

#[test]
fn gas_convex_ball_cuboid_inlined() {
    let (pose, b, pred) = opaque_args(3 * UNIT / 2);
    let mut m = fresh();
    contact_manifold_convex_ball_inlined(pose, opaque(cuboid()), b, pred, ref m);
}

#[test]
fn gas_convex_ball_halfspace_inlined() {
    let (pose, b, pred) = opaque_args(3 * UNIT / 2);
    let mut m = fresh();
    contact_manifold_convex_ball_inlined(pose, opaque(halfspace()), b, pred, ref m);
}
