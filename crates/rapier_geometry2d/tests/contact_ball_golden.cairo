//! Every ball pair of `rapier_golden::contact_manifolds` against the ball generators: ball–ball,
//! ball–cuboid, ball–capsule, halfspace–ball, segment–ball and the flipped-order cases.
//!
//! Tolerances are the analytic-pair ones of `tools/golden/README.md`: 64 ulp on `dist`, on both
//! points and on every normal component. The number of points is exact. Feature ids are exact
//! unless the case is tagged `ambiguous`, in which case only the point count and `dist` are
//! compared (the discrete outputs hinge on an exact tie or on a fallback branch upstream).
//! When upstream keeps no point the normals are meaningless and are not compared.

use fixed::Fixed;
use glam::vec2::Vec2;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait};
use rapier_geometry2d::contact_generators::ball_ball::contact_manifold_ball_ball_shapes;
use rapier_geometry2d::contact_generators::convex_ball::contact_manifold_convex_ball_shapes;
use rapier_geometry2d::shape::{Ball, Capsule, Cuboid, HalfSpace, Segment, Shape};
use rapier_golden::compare::{vec2_within, within};
use rapier_golden::contact_manifolds;
use rapier_golden::types::{ManifoldCase, PoseRaw, ShapeRaw, Vec2Raw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

const TOLERANCE: u64 = 64;
/// Ball pairs in the fixture: 6 ball–ball, 7 ball–cuboid, 6 ball–capsule, 7 halfspace–ball,
/// 7 segment–ball, and the flipped cuboid–ball, capsule–ball, ball–halfspace,
/// ball–segment.
const BALL_CASES: u32 = 37;

fn vector(v: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: v.x }, y: Fixed { raw: v.y } }
}

fn raw(v: Vec2) -> Vec2Raw {
    Vec2Raw { x: v.x.raw, y: v.y.raw }
}

/// The pose as handed to upstream: the rotation pair is not renormalised.
fn pose(p: PoseRaw) -> Pose2 {
    Pose2 {
        translation: vector(p.translation),
        rotation: Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    }
}

fn shape(s: ShapeRaw) -> Shape {
    match s {
        ShapeRaw::Ball(r) => Shape::Ball(Ball { radius: Fixed { raw: r } }),
        ShapeRaw::Cuboid(he) => Shape::Cuboid(Cuboid { half_extents: vector(he) }),
        ShapeRaw::Capsule(c) => Shape::Capsule(
            Capsule {
                segment: Segment { a: vector(c.a), b: vector(c.b) },
                radius: Fixed { raw: c.radius },
            },
        ),
        ShapeRaw::HalfSpace(n) => Shape::HalfSpace(HalfSpace { normal: vector(n) }),
        ShapeRaw::Segment(s) => Shape::Segment(Segment { a: vector(s.a), b: vector(s.b) }),
    }
}

/// Runs the generator upstream's dispatcher would pick, on a fresh manifold. `None` when the pair
/// has no ball.
fn generate(case: ManifoldCase) -> Option<ContactManifold> {
    let (s1, s2, pos12) = (shape(case.shape1), shape(case.shape2), pose(case.pos12));
    let prediction = Fixed { raw: contact_manifolds::PREDICTION };
    let mut m: ContactManifold = Default::default();
    if contact_manifold_ball_ball_shapes(pos12, s1, s2, prediction, ref m)
        || contact_manifold_convex_ball_shapes(pos12, s1, s2, prediction, ref m) {
        Some(m)
    } else {
        None
    }
}

fn check(case: ManifoldCase, m: ContactManifold) {
    let id = case.id;
    assert_eq!(m.num_points.into(), case.num_points, "{} num_points", id);
    if case.num_points == 0 {
        return;
    }
    let [e0, _] = case.points;
    let c = m.point(0);
    assert!(within(c.dist.raw, e0.dist, TOLERANCE), "{} dist {} vs {}", id, c.dist.raw, e0.dist);
    if case.ambiguous {
        return;
    }
    assert!(vec2_within(raw(m.local_n1), case.local_n1, TOLERANCE), "{} n1", id);
    assert!(vec2_within(raw(m.local_n2), case.local_n2, TOLERANCE), "{} n2", id);
    assert!(vec2_within(raw(c.local_p1), e0.local_p1, TOLERANCE), "{} p1", id);
    assert!(vec2_within(raw(c.local_p2), e0.local_p2, TOLERANCE), "{} p2", id);
    assert_eq!(c.fid1.packed, e0.fid1, "{} fid1", id);
    assert_eq!(c.fid2.packed, e0.fid2, "{} fid2", id);
}

#[test]
fn test_every_ball_case() {
    let mut tested = 0;
    for c in contact_manifolds::cases() {
        if let Some(m) = generate(*c) {
            check(*c, m);
            tested += 1;
        }
    }
    assert_eq!(tested, BALL_CASES);
}

/// Cases whose point count is 0 upstream leave the manifold empty here too, and a manifold that
/// is regenerated over its own output (warm-start path) gives the same geometry.
#[test]
fn test_regeneration_over_the_previous_manifold() {
    let prediction = Fixed { raw: contact_manifolds::PREDICTION };
    for c in contact_manifolds::cases() {
        let (s1, s2, pos12) = (shape((*c).shape1), shape((*c).shape2), pose((*c).pos12));
        if let Some(first) = generate(*c) {
            let mut m = first;
            if !contact_manifold_ball_ball_shapes(pos12, s1, s2, prediction, ref m) {
                contact_manifold_convex_ball_shapes(pos12, s1, s2, prediction, ref m);
            }
            // Two balls keep the data of the live point, so the whole manifold is identical; the
            // convex path does too (one point). Either way nothing may drift.
            assert_eq!(m.num_points, first.num_points);
            if m.num_points != 0 {
                assert_eq!(m.point(0), first.point(0));
            }
        }
    }
}

#[test]
fn gas_baseline() {
    let _ = opaque(contact_manifolds::BALL_BALL_SHALLOW.pos12);
}

#[test]
fn gas_golden_ball_ball_shallow() {
    let case = opaque(contact_manifolds::BALL_BALL_SHALLOW);
    let _ = generate(case);
}

#[test]
fn gas_golden_ball_cuboid_shallow() {
    let case = opaque(contact_manifolds::BALL_CUBOID_SHALLOW);
    let _ = generate(case);
}

#[test]
fn gas_golden_cuboid_ball_shallow() {
    let case = opaque(contact_manifolds::CUBOID_BALL_SHALLOW);
    let _ = generate(case);
}
