//! Family `intersection_tests`: `DefaultQueryDispatcher::intersection_test` (the query behind
//! sensor pairs, work package SE) on every shape pair that has a contact generator, plus the
//! segment pairs upstream answers with GJK, over four regimes: separated, touching, overlapping,
//! contained. `pos12` is the pose of shape 2 in the frame of shape 1.
//!
//! Touching poses are exact in Q32.32 and in f64. Upstream answers the touching case of a pair
//! it sends to GJK (every pair without a ball, a half-space, or two cuboids) within GJK's
//! tolerance: such cases are tagged `gjk_touching`, and a port with an exact kernel may answer
//! `true` where upstream answers `false`.

use crate::q::{jqpose, QPose, QRot, QVec};
use crate::shapes::ShapeSpec;
use rapier2d_f64::parry::query::{DefaultQueryDispatcher, QueryDispatcher};
use serde_json::{json, Value};

pub(crate) struct Case {
    pub(crate) pair: &'static str,
    pub(crate) regime: &'static str,
    pub(crate) shape1: ShapeSpec,
    pub(crate) shape2: ShapeSpec,
    pub(crate) pos12: QPose,
}

fn pose(x: f64, y: f64, rot: QRot) -> QPose {
    QPose::new(QVec::snap(x, y), rot)
}

/// Upstream sends the pair to GJK (support map against support map).
pub(crate) fn gjk(s1: &ShapeSpec, s2: &ShapeSpec) -> bool {
    let special = |s: &ShapeSpec| matches!(s, ShapeSpec::Ball { .. } | ShapeSpec::HalfSpace { .. });
    let cuboids = matches!((s1, s2), (ShapeSpec::Cuboid { .. }, ShapeSpec::Cuboid { .. }));
    !(special(s1) || special(s2) || cuboids)
}

/// The case table, shared with the `shape_queries` family.
#[rustfmt::skip]
pub(crate) fn cases() -> Vec<Case> {
    let id = QRot::IDENTITY;
    let q = QRot::QUARTER;
    let r30 = QRot::from_degrees(30.0);
    let r45 = QRot::from_degrees(45.0);
    let rm30 = QRot::from_degrees(-30.0);
    let ball = ShapeSpec::ball;
    let cub = ShapeSpec::cuboid;
    let cap = ShapeSpec::capsule_y;
    let seg = ShapeSpec::segment;
    let hs = ShapeSpec::halfspace_up();
    let tri = ShapeSpec::polygon(&[(-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)]);
    let small_tri = ShapeSpec::polygon(&[(-0.5, -0.5), (0.5, -0.5), (0.0, 0.5)]);
    let pent = ShapeSpec::polygon(&[(-1.0, -1.0), (1.0, -1.0), (1.5, 0.0), (0.0, 1.5), (-1.5, 0.0)]);
    let long = seg((-1.0, 0.0), (1.0, 0.0));
    let short = seg((-0.5, 0.0), (0.5, 0.0));
    let (b, c, p) = (ball(0.5), cub(1.0, 0.5), cap(0.5, 0.25));
    let mut v = Vec::new();
    let mut add = |pair, regime, shape1, shape2, pos12| v.push(Case { pair, regime, shape1, shape2, pos12 });
    // (pair, [separated, touching, overlapping, contained] as (shape1, shape2, pos12))
    add("ball_ball", "separated", b, b, pose(1.5, 0.25, r30));
    add("ball_ball", "touching", b, b, pose(1.0, 0.0, id));
    add("ball_ball", "overlapping", b, b, pose(0.3, 0.4, r45));
    add("ball_ball", "contained", b, ball(0.25), pose(0.1, 0.1, id));
    add("ball_cuboid", "separated", b, c, pose(2.5, 0.0, id));
    add("ball_cuboid", "touching", b, c, pose(0.0, 1.0, id));
    add("ball_cuboid", "overlapping", b, c, pose(1.3, 0.8, id));
    add("ball_cuboid", "contained", b, cub(2.0, 1.0), pose(0.2, 0.1, r30));
    add("ball_capsule", "separated", b, p, pose(2.0, 0.0, id));
    add("ball_capsule", "touching", b, p, pose(0.75, 0.0, id));
    add("ball_capsule", "overlapping", b, p, pose(0.8, 0.9, rm30));
    add("ball_capsule", "contained", ball(1.5), p, pose(0.1, 0.0, id));
    add("ball_segment", "separated", b, long, pose(0.0, 1.0, id));
    add("ball_segment", "touching", b, long, pose(0.0, 0.5, id));
    add("ball_segment", "overlapping", b, long, pose(0.5, 0.3, r30));
    add("ball_segment", "contained", ball(2.0), long, pose(0.2, 0.1, id));
    add("ball_halfspace", "separated", b, hs, pose(0.0, -1.0, id));
    add("ball_halfspace", "touching", b, hs, pose(0.0, -0.5, id));
    add("ball_halfspace", "overlapping", b, hs, pose(0.0, -0.3, r30));
    add("ball_halfspace", "contained", b, hs, pose(0.0, 2.0, id));
    add("ball_polygon", "separated", b, tri, pose(0.0, 3.0, id));
    add("ball_polygon", "touching", b, tri, pose(0.0, -1.5, id));
    add("ball_polygon", "overlapping", b, tri, pose(0.3, -1.2, id));
    add("ball_polygon", "contained", ball(0.25), tri, pose(0.0, 0.2, id));
    add("cuboid_cuboid", "separated", c, cub(0.5, 0.5), pose(3.0, 0.0, id));
    add("cuboid_cuboid", "touching", c, cub(0.5, 0.5), pose(1.5, 0.0, id));
    add("cuboid_cuboid", "overlapping", c, cub(0.5, 0.5), pose(1.6, 0.3, r30));
    add("cuboid_cuboid", "contained", c, cub(0.25, 0.25), pose(0.2, 0.0, r45));
    add("cuboid_capsule", "separated", c, p, pose(3.0, 0.0, id));
    add("cuboid_capsule", "touching", c, p, pose(1.25, 0.0, id));
    add("cuboid_capsule", "overlapping", c, p, pose(1.3, 0.2, r30));
    add("cuboid_capsule", "contained", c, p, pose(0.1, 0.0, q));
    add("cuboid_segment", "separated", c, long, pose(0.0, 1.0, id));
    add("cuboid_segment", "touching", c, long, pose(0.0, 0.5, id));
    add("cuboid_segment", "overlapping", c, long, pose(0.5, 0.5, r30));
    add("cuboid_segment", "contained", c, short, pose(0.2, 0.1, id));
    add("capsule_capsule", "separated", p, p, pose(2.0, 0.0, id));
    add("capsule_capsule", "touching", p, p, pose(0.5, 0.0, id));
    add("capsule_capsule", "overlapping", p, p, pose(0.9, 0.0, q));
    add("capsule_capsule", "contained", cap(1.0, 1.0), cap(0.25, 0.25), pose(0.1, 0.2, id));
    add("halfspace_cuboid", "separated", hs, c, pose(0.0, 2.0, id));
    add("halfspace_cuboid", "touching", hs, c, pose(3.0, 0.5, id));
    add("halfspace_cuboid", "overlapping", hs, c, pose(0.0, 0.9, r30));
    add("halfspace_cuboid", "contained", hs, c, pose(0.0, -2.0, r45));
    add("halfspace_capsule", "separated", hs, p, pose(0.0, 1.5, id));
    add("halfspace_capsule", "touching", hs, p, pose(0.0, 0.75, id));
    add("halfspace_capsule", "overlapping", hs, p, pose(0.5, 0.5, r30));
    add("halfspace_capsule", "contained", hs, p, pose(0.0, -2.0, id));
    add("halfspace_segment", "separated", hs, long, pose(0.0, 0.5, id));
    add("halfspace_segment", "touching", hs, long, pose(0.0, 0.0, id));
    add("halfspace_segment", "overlapping", hs, long, pose(0.0, 0.0, r30));
    add("halfspace_segment", "contained", hs, long, pose(0.0, -1.0, id));
    add("halfspace_polygon", "separated", hs, tri, pose(0.0, 1.5, id));
    add("halfspace_polygon", "touching", hs, tri, pose(0.0, 1.0, id));
    add("halfspace_polygon", "overlapping", hs, tri, pose(0.0, 0.5, r30));
    add("halfspace_polygon", "contained", hs, tri, pose(0.0, -2.0, id));
    add("polygon_capsule", "separated", tri, p, pose(0.0, 3.0, id));
    add("polygon_capsule", "touching", tri, p, pose(0.0, 1.75, id));
    add("polygon_capsule", "overlapping", tri, p, pose(0.5, 0.5, id));
    add("polygon_capsule", "contained", pent, cap(0.25, 0.25), pose(0.0, 0.0, id));
    add("polygon_cuboid", "separated", tri, c, pose(3.0, 0.0, id));
    add("polygon_cuboid", "touching", tri, c, pose(0.0, 1.5, id));
    add("polygon_cuboid", "overlapping", tri, c, pose(0.5, 0.5, r30));
    add("polygon_cuboid", "contained", pent, cub(0.5, 0.25), pose(0.0, 0.1, id));
    add("polygon_segment", "separated", tri, long, pose(0.0, 2.0, id));
    add("polygon_segment", "touching", tri, long, pose(0.0, -1.0, id));
    add("polygon_segment", "overlapping", tri, long, pose(0.0, 0.0, r30));
    add("polygon_segment", "contained", pent, short, pose(0.0, 0.2, id));
    add("polygon_polygon", "separated", tri, tri, pose(3.0, 0.0, id));
    add("polygon_polygon", "touching", tri, tri, pose(2.0, 0.0, id));
    add("polygon_polygon", "overlapping", tri, tri, pose(0.5, 0.5, r30));
    add("polygon_polygon", "contained", pent, small_tri, pose(0.0, 0.0, id));
    add("segment_segment", "separated", long, long, pose(0.0, 1.0, id));
    add("segment_segment", "touching", long, long, pose(2.0, 0.0, id));
    add("segment_segment", "overlapping", long, long, pose(0.0, 0.0, q));
    add("segment_segment", "contained", long, short, pose(0.2, 0.0, id));
    add("segment_capsule", "separated", long, p, pose(0.0, 2.0, id));
    add("segment_capsule", "touching", long, p, pose(1.25, 0.0, id));
    add("segment_capsule", "overlapping", long, p, pose(0.5, 0.0, id));
    add("segment_capsule", "contained", seg((0.0, -0.25), (0.0, 0.25)), p, pose(0.0, 0.0, id));
    add("halfspace_halfspace", "unsupported", hs, hs, pose(0.0, 0.0, id));
    v
}

fn run(case: &Case) -> Value {
    let (shape1, shape2) = (case.shape1.shared(), case.shape2.shared());
    let answer = DefaultQueryDispatcher.intersection_test(&case.pos12.p(), &*shape1.0, &*shape2.0);
    json!({
        "id": format!("{}/{}", case.pair, case.regime),
        "pair": case.pair,
        "regime": case.regime,
        "gjk_touching": case.regime == "touching" && gjk(&case.shape1, &case.shape2),
        "shape1": case.shape1.json(),
        "shape2": case.shape2.json(),
        "pos12": jqpose(case.pos12),
        "expected": match answer {
            Ok(intersecting) => json!({ "supported": true, "intersecting": intersecting }),
            Err(_) => json!({ "supported": false, "intersecting": false }),
        },
    })
}

pub fn generate() -> Value {
    json!({
        "family": "intersection_tests",
        "cases": cases().iter().map(run).collect::<Vec<_>>(),
    })
}
