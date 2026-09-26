//! Work package CC1: shape casts. Two families, in files of their own:
//!
//! * `shape_casts`: `query::cast_shapes` (linear motions) on every pair of the closed shape set
//!   (SH1's triangle and round shapes included) and the unsupported half-space pair, five regimes
//!   each, answered under five option sets ([`OPTIONS`]);
//! * `nonlinear_shape_casts`: `query::cast_shapes_nonlinear` (rotating motions) on the same pairs,
//!   five regimes each, with `stop_at_penetration` true and false;
//! * `sweep_toi`: `query::sweep_toi::sweep_time_of_impact` (the swept proxies of Rapier's CCD) on
//!   the same pairs but the half-space ones (no proxy), shape 1 standing still as Rapier's target
//!   and shape 2 swept from its placement, five regimes each (`SWEEP_REGIMES`).
//!
//! Placement (as `sh1`): shape 2 sits below shape 1 (`pos12 = (dx, -(bottom1 + top2 + gap))`),
//! except for a half-space first, where it sits above the plane (`pos12 = (dx, bottom2 + gap)`);
//! "toward" is the direction that closes the gap. Shape 1 is at `pos1 = translation(1, -2)` and
//! shape 2 at `pos2 = pos1 * pos12` (exact). Regimes:
//!
//! * `hit`: gap 0.5, shape 2 turned by 10 degrees, closing at `(0.1, 1)` while shape 1 drifts at
//!   `(0.05, -0.25)` (rotating: angular velocities 0.5 and -0.25);
//! * `miss`: gap 0.5, shape 2 moving away;
//! * `touching`: gap 0 (exact), closing;
//! * `penetrating`: gap -0.05, turned by 10 degrees, closing;
//! * `grazing`: gap 0 (exact), shape 2 1.5 units to the side, moving sideways back (rotating:
//!   turning at 0.2 too).
//!
//! `iterative` tells that upstream answers the linear cast with GJK (every supported pair but
//! ball–ball and the half-space pairs); every nonlinear cast is iterative upstream (conservative
//! advancement on GJK's closest points).

use crate::q::{jf, jq, jqpose, jqvec, jvec, QPose, QRot, QVec, Q};
use crate::sh1::Sh1Shape;
use crate::shapes::ShapeSpec;
use rapier2d_f64::parry::query::{
    self, NonlinearRigidMotion, ShapeCastHit, ShapeCastOptions, ShapeCastStatus,
};
use serde_json::{json, Value};

/// `(name, max_time_of_impact raw or None for Real::MAX, target_distance raw,
/// stop_at_penetration, compute_impact_geometry_on_penetration)`.
type Opt = (&'static str, Option<i64>, i64, bool, bool);

/// The option sets of the linear family, in the order of every case's `answers`.
pub const OPTIONS: [Opt; 5] = [
    ("default", None, 0, true, true),
    ("pass_through", None, 0, false, true),
    ("target", None, 429_496_730, true, true),
    ("max_toi", Some(1 << 30), 0, true, true),
    ("no_geometry", None, 0, true, false),
];

/// Nonlinear time interval, raw.
const START_TIME: i64 = 0;
const END_TIME: i64 = 2 << 32;

fn b(s: ShapeSpec) -> Sh1Shape {
    Sh1Shape::Base(s)
}

fn pentagon() -> ShapeSpec {
    ShapeSpec::polygon(&[(-0.5, -0.4), (0.5, -0.4), (0.6, 0.1), (0.0, 0.5), (-0.6, 0.1)])
}

fn tri() -> Sh1Shape {
    Sh1Shape::triangle((-1.0, -0.5), (1.0, -0.5), (0.25, 0.75))
}

fn small_tri() -> Sh1Shape {
    Sh1Shape::triangle((-0.4, -0.3), (0.4, -0.3), (0.0, 0.4))
}

/// The pairs, both families.
fn pairs() -> Vec<(&'static str, Sh1Shape, Sh1Shape)> {
    let ball = b(ShapeSpec::ball(0.4));
    let cub = b(ShapeSpec::cuboid(0.5, 0.3));
    let cap = b(ShapeSpec::capsule_y(0.3, 0.2));
    let seg = b(ShapeSpec::segment((-0.4, 0.0), (0.4, 0.0)));
    let hs = b(ShapeSpec::halfspace_up());
    let poly = b(pentagon());
    let rcub = b(ShapeSpec::cuboid(0.4, 0.25)).rounded(0.1);
    let rtri = tri().rounded(0.1);
    let rpoly = b(pentagon()).rounded(0.1);
    vec![
        ("ball_ball", ball, ball),
        ("ball_cuboid", ball, cub),
        ("cuboid_ball", cub, ball),
        ("ball_capsule", ball, cap),
        ("ball_segment", ball, seg),
        ("ball_polygon", ball, poly),
        ("cuboid_cuboid", cub, cub),
        ("cuboid_capsule", cub, cap),
        ("cuboid_segment", cub, seg),
        ("capsule_capsule", cap, cap),
        ("capsule_segment", cap, seg),
        ("segment_segment", seg, seg),
        ("polygon_polygon", poly, poly),
        ("polygon_capsule", poly, cap),
        ("halfspace_ball", hs, ball),
        ("ball_halfspace", ball, hs),
        ("halfspace_cuboid", hs, cub),
        ("cuboid_halfspace", cub, hs),
        ("halfspace_polygon", hs, poly),
        ("halfspace_rcub", hs, rcub),
        ("tri_ball", tri(), ball),
        ("tri_cuboid", tri(), cub),
        ("tri_tri", tri(), small_tri()),
        ("rcub_cuboid", rcub, cub),
        ("rcub_rcub", rcub, rcub),
        ("rtri_ball", rtri, ball),
        ("rpoly_poly", rpoly, poly),
        ("halfspace_halfspace", hs, hs),
    ]
}

/// `(regime, gap, dx, degrees of shape 2)`.
const REGIMES: [(&str, f64, f64, f64); 5] = [
    ("hit", 0.5, 0.0, 10.0),
    ("miss", 0.5, 0.0, 10.0),
    ("touching", 0.0, 0.0, 0.0),
    ("penetrating", -0.05, 0.0, 10.0),
    ("grazing", 0.0, 1.5, 0.0),
];

/// The `x` of the topmost point of shape 2 when it is a triangle's apex, so that it comes under
/// shape 1's centre (as `sh1`).
fn apex_x(shape: &Sh1Shape) -> f64 {
    match *shape {
        Sh1Shape::Triangle { c, .. } | Sh1Shape::RoundTriangle { c, .. } => c.x.f(),
        _ => 0.0,
    }
}

struct Case {
    pair: &'static str,
    regime: &'static str,
    shape1: Sh1Shape,
    shape2: Sh1Shape,
    pos12: QPose,
    /// +1 when shape 2 closes the gap moving along +y.
    toward: f64,
}

fn cases() -> Vec<Case> {
    let mut out = Vec::new();
    for (pair, shape1, shape2) in pairs() {
        let first_hs = shape1.is_halfspace();
        let dx0 = if first_hs { 0.0 } else { -apex_x(&shape2) };
        for (regime, gap, dx, deg) in REGIMES {
            let (y, toward) = if first_hs {
                (-shape2.y_range().0 + gap, -1.0)
            } else {
                (-(-shape1.y_range().0 + shape2.y_range().1 + gap), 1.0)
            };
            let rot = if deg == 0.0 { QRot::IDENTITY } else { QRot::from_degrees(deg) };
            out.push(Case {
                pair,
                regime,
                shape1,
                shape2,
                pos12: QPose::new(QVec::snap(dx0 + dx, y), rot),
                toward,
            });
        }
    }
    out
}

fn pos1() -> QPose {
    QPose::translation(1.0, -2.0)
}

/// `pos1 * pos12` for the pure integer translation `pos1`: exact.
fn pos2(pos12: QPose) -> QPose {
    let t = pos12.translation;
    QPose::new(QVec { x: Q(t.x.0 + (1 << 32)), y: Q(t.y.0 - (2 << 32)) }, pos12.rotation)
}

/// `(vel1, vel2)` of a linear case.
fn velocities(case: &Case) -> (QVec, QVec) {
    let k = case.toward;
    match case.regime {
        "hit" => (QVec::snap(0.05, -0.25 * k), QVec::snap(0.1, k)),
        "miss" => (QVec::ZERO, QVec::snap(0.0, -k)),
        "grazing" => (QVec::ZERO, QVec::snap(-1.0, 0.0)),
        _ => (QVec::ZERO, QVec::snap(0.0, k)),
    }
}

/// `(angvel1, angvel2)` of a nonlinear case; the linear velocities are those of [`velocities`].
fn angular_velocities(regime: &str) -> (Q, Q) {
    match regime {
        "hit" => (Q::snap(-0.25), Q::snap(0.5)),
        "miss" => (Q::ZERO, Q::snap(1.0)),
        "grazing" => (Q::ZERO, Q::snap(0.2)),
        _ => (Q::ZERO, Q::snap(0.3)),
    }
}

/// Local rotation centre of shape 2's motion (shape 1 turns about its origin).
fn center2() -> QVec {
    QVec::snap(0.1, 0.0)
}

fn status(s: ShapeCastStatus) -> &'static str {
    match s {
        ShapeCastStatus::OutOfIterations => "out_of_iterations",
        ShapeCastStatus::Converged => "converged",
        ShapeCastStatus::Failed => "failed",
        ShapeCastStatus::PenetratingOrWithinTargetDist => "penetrating",
    }
}

fn hit_json(hit: Option<ShapeCastHit>) -> Value {
    match hit {
        None => Value::Null,
        Some(h) => json!({
            "toi": jf(h.time_of_impact),
            "witness1": jvec(h.witness1),
            "witness2": jvec(h.witness2),
            "normal1": jvec(h.normal1),
            "normal2": jvec(h.normal2),
            "status": status(h.status),
        }),
    }
}

fn options(o: &Opt) -> ShapeCastOptions {
    ShapeCastOptions {
        max_time_of_impact: o.1.map(|r| Q(r).f()).unwrap_or(f64::MAX),
        target_distance: Q(o.2).f(),
        stop_at_penetration: o.3,
        compute_impact_geometry_on_penetration: o.4,
    }
}

fn options_json(o: &Opt) -> Value {
    json!({
        "name": o.0,
        "max_time_of_impact": o.1.map(|r| jq(Q(r))).unwrap_or(Value::Null),
        "target_distance": jq(Q(o.2)),
        "stop_at_penetration": o.3,
        "compute_impact_geometry_on_penetration": o.4,
    })
}

fn is_ball(s: &Sh1Shape) -> bool {
    matches!(s, Sh1Shape::Base(ShapeSpec::Ball { .. }))
}

fn linear(case: &Case) -> Value {
    let (shape1, shape2) = (case.shape1.shared(), case.shape2.shared());
    let (g1, g2) = (&*shape1.0, &*shape2.0);
    let (p1, p2) = (pos1(), pos2(case.pos12));
    let (vel1, vel2) = velocities(case);
    let answers: Vec<Value> = OPTIONS
        .iter()
        .map(|o| match query::cast_shapes(&p1.p(), vel1.v(), g1, &p2.p(), vel2.v(), g2, options(o)) {
            Err(_) => json!({ "supported": false, "hit": Value::Null }),
            Ok(hit) => json!({ "supported": true, "hit": hit_json(hit) }),
        })
        .collect();
    let any_hs = case.shape1.is_halfspace() || case.shape2.is_halfspace();
    json!({
        "id": format!("{}/{}", case.pair, case.regime),
        "pair": case.pair,
        "regime": case.regime,
        "iterative": !(any_hs || (is_ball(&case.shape1) && is_ball(&case.shape2))),
        "shape1": case.shape1.json(),
        "shape2": case.shape2.json(),
        "pos1": jqpose(p1),
        "vel1": jqvec(vel1),
        "pos2": jqpose(p2),
        "vel2": jqvec(vel2),
        "answers": answers,
    })
}

fn motion_json(start: QPose, center: QVec, linvel: QVec, angvel: Q) -> Value {
    json!({
        "start": jqpose(start),
        "local_center": jqvec(center),
        "linvel": jqvec(linvel),
        "angvel": jq(angvel),
    })
}

fn nonlinear(case: &Case) -> Value {
    let (shape1, shape2) = (case.shape1.shared(), case.shape2.shared());
    let (g1, g2) = (&*shape1.0, &*shape2.0);
    let (p1, p2) = (pos1(), pos2(case.pos12));
    let (vel1, vel2) = velocities(case);
    let (w1, w2) = angular_velocities(case.regime);
    let m1 = NonlinearRigidMotion::new(p1.p(), QVec::ZERO.v(), vel1.v(), w1.f());
    let m2 = NonlinearRigidMotion::new(p2.p(), center2().v(), vel2.v(), w2.f());
    let (t0, t1) = (Q(START_TIME).f(), Q(END_TIME).f());
    let answers: Vec<Value> = [true, false]
        .iter()
        .map(|&stop| match query::cast_shapes_nonlinear(&m1, g1, &m2, g2, t0, t1, stop) {
            Err(_) => json!({ "stop_at_penetration": stop, "supported": false, "hit": Value::Null }),
            Ok(hit) => json!({ "stop_at_penetration": stop, "supported": true, "hit": hit_json(hit) }),
        })
        .collect();
    json!({
        "id": format!("{}/{}", case.pair, case.regime),
        "pair": case.pair,
        "regime": case.regime,
        "shape1": case.shape1.json(),
        "shape2": case.shape2.json(),
        "motion1": motion_json(p1, QVec::ZERO, vel1, w1),
        "motion2": motion_json(p2, center2(), vel2, w2),
        "answers": answers,
    })
}

/// Linear slop of the sweep family: `0.005`, snapped (Rapier's `normalized_linear_slop`).
const SLOP: i64 = 21_474_836;

/// `(regime, gap, dx, degrees of shape 2 at the start, end offset (x, y) towards shape 1 along
/// `toward`, end turn in degrees)`.
const SWEEP_REGIMES: [(&str, f64, f64, f64, (f64, f64), f64); 5] = [
    ("hit", 0.5, 0.0, 10.0, (0.1, 1.0), 0.0),
    ("rotating", 0.5, 0.0, 0.0, (0.0, 1.0), 90.0),
    ("miss", 0.5, 0.0, 10.0, (0.0, -1.0), 0.0),
    ("overlapped", -0.05, 0.0, 10.0, (0.0, 1.0), 0.0),
    ("grazing", 0.0, 1.5, 0.0, (-2.0, 0.0), 0.0),
];

fn sweep_status(s: query::SweepToiStatus) -> &'static str {
    match s {
        query::SweepToiStatus::Overlapped => "overlapped",
        query::SweepToiStatus::Hit => "hit",
        query::SweepToiStatus::Separated => "separated",
        query::SweepToiStatus::Failed => "failed",
    }
}

fn sweep_cases() -> Vec<Value> {
    let mut out = Vec::new();
    for (pair, shape1, shape2) in pairs() {
        if shape1.is_halfspace() || shape2.is_halfspace() {
            continue;
        }
        let (s1, s2) = (shape1.shared(), shape2.shared());
        let proxy1 = query::ToiProxy::from_shape(&*s1.0).unwrap();
        let proxy2 = query::ToiProxy::from_shape(&*s2.0).unwrap();
        for (regime, gap, dx, deg, (ex, ey), turn) in SWEEP_REGIMES {
            let y = -(-shape1.y_range().0 + shape2.y_range().1 + gap);
            let dx0 = -apex_x(&shape2);
            let rot = if deg == 0.0 { QRot::IDENTITY } else { QRot::from_degrees(deg) };
            let end_rot = if turn == 0.0 { rot } else { QRot::from_degrees(deg + turn) };
            let start1 = pos1();
            let start2 = pos2(QPose::new(QVec::snap(dx0 + dx, y), rot));
            let end2 = QPose::new(
                QVec { x: Q(start2.translation.x.0 + Q::snap(ex).0), y: Q(start2.translation.y.0 + Q::snap(ey).0) },
                end_rot,
            );
            let center2 = center2();
            let sweep1 = query::Sweep::constant(&start1.p(), QVec::ZERO.v());
            let sweep2 = query::Sweep::from_poses(&start2.p(), &end2.p(), center2.v());
            let r = query::sweep_time_of_impact(&proxy1, &sweep1, &proxy2, &sweep2, 1.0, Q(SLOP).f());
            out.push(json!({
                "id": format!("{pair}/{regime}"),
                "pair": pair,
                "regime": regime,
                "shape1": shape1.json(),
                "shape2": shape2.json(),
                "pose1": jqpose(start1),
                "start2": jqpose(start2),
                "end2": jqpose(end2),
                "local_center2": jqvec(center2),
                "status": sweep_status(r.status),
                "fraction": jf(r.fraction),
                "point": jvec(r.point),
                "normal": jvec(r.normal),
            }));
        }
    }
    out
}

pub fn sweep_toi() -> Value {
    json!({
        "family": "sweep_toi",
        "max_fraction": jq(Q::ONE),
        "linear_slop": jq(Q(SLOP)),
        "cases": sweep_cases(),
    })
}

pub fn shape_casts() -> Value {
    json!({
        "family": "shape_casts",
        "options": OPTIONS.iter().map(options_json).collect::<Vec<_>>(),
        "cases": cases().iter().map(linear).collect::<Vec<_>>(),
    })
}

pub fn nonlinear_shape_casts() -> Value {
    json!({
        "family": "nonlinear_shape_casts",
        "start_time": jq(Q(START_TIME)),
        "end_time": jq(Q(END_TIME)),
        "cases": cases().iter().map(nonlinear).collect::<Vec<_>>(),
    })
}
