//! Family `ray_casts`: `RayCast::cast_ray` and `RayCast::cast_ray_and_get_normal` on ball,
//! cuboid, capsule, segment and half-space, both `solid` and hollow.
//!
//! Every case places the shape at a pose (the identity for most of them) and casts a world-space
//! ray, so the `Pose2` wrappers are exercised with the local casts. Both upstream entry points are
//! recorded because they do not always agree: the cuboid's `cast_local_ray` (slab loop) and
//! `cast_local_ray_and_get_normal` (`clip_aabb_line`) answer differently for a hollow ray that
//! starts inside or on the boundary (see README).

use crate::leaf::qv;
use crate::q::{jf, jq, jqpose, jqvec, jvec, QPose, QRot, QVec, Q};
use crate::shapes::ShapeSpec;
use rapier2d_f64::parry::query::{Ray, RayCast, RayIntersection};
use rapier2d_f64::parry::shape::{Ball, Capsule, Cuboid, FeatureId, HalfSpace, Segment};
use serde_json::{json, Value};

struct Case {
    name: String,
    shape: ShapeSpec,
    pose: QPose,
    origin: QVec,
    dir: QVec,
    max_toi: Q,
    note: &'static str,
}

const DEFAULT_MAX: f64 = 100.0;

#[allow(clippy::too_many_arguments)]
fn c(
    group: &str,
    name: &str,
    shape: ShapeSpec,
    origin: (f64, f64),
    dir: (f64, f64),
    max_toi: f64,
    note: &'static str,
) -> Case {
    Case {
        name: format!("{group}/{name}"),
        shape,
        pose: QPose::new(QVec::ZERO, QRot::IDENTITY),
        origin: qv(origin.0, origin.1),
        dir: qv(dir.0, dir.1),
        max_toi: Q::snap(max_toi),
        note,
    }
}

fn posed(mut case: Case, pose: QPose) -> Case {
    case.pose = pose;
    case
}

#[rustfmt::skip]
fn cases() -> Vec<Case> {
    let m = DEFAULT_MAX;
    let ball = ShapeSpec::ball(0.5);
    // Half extents (1.0, 0.5).
    let cub = ShapeSpec::cuboid(1.0, 0.5);
    // Segment (0, -0.5)-(0, 0.5), radius 0.25.
    let cap = ShapeSpec::capsule_y(0.5, 0.25);
    let cap_oblique = ShapeSpec::capsule((-0.5, -0.25), (1.0, 0.75), 0.3);
    let seg = ShapeSpec::segment((-1.0, 0.0), (1.0, 0.0));
    let seg_oblique = ShapeSpec::segment((0.0, 0.0), (2.0, 1.0));
    let hs = ShapeSpec::halfspace_up();
    let rot30 = QPose::new(qv(1.0, 2.0), QRot::from_degrees(30.0));
    let rotm135 = QPose::new(qv(-1.5, 0.25), QRot::from_degrees(-135.0));

    vec![
        // --- ball (radius 0.5) ---------------------------------------------------------------
        c("ball", "hit", ball, (-2.0, 0.1), (1.0, 0.0), m, "enters from -x, off-centre"),
        c("ball", "oblique", ball, (-2.0, -1.5), (0.8, 0.6), m, "diagonal hit"),
        c("ball", "unnormalized_dir", ball, (-2.0, 0.0), (4.0, 0.0), m, "|dir| = 4: toi is in units of dir (0.375)"),
        c("ball", "grazing", ball, (-2.0, 0.5), (1.0, 0.0), m, "tangent: delta == 0, single root"),
        c("ball", "inside", ball, (0.1, 0.2), (1.0, 0.5), m, "solid: 0 with the inward normal; hollow: the exit"),
        c("ball", "on_surface_out", ball, (0.5, 0.0), (1.0, 0.0), m, "on the circle pointing out: c == 0, b > 0, counts as inside"),
        c("ball", "miss", ball, (-2.0, 1.0), (1.0, 0.0), m, "passes above: delta < 0"),
        c("ball", "pointing_away", ball, (2.0, 0.0), (1.0, 0.0), m, "c > 0 and b > 0: early exit"),
        c("ball", "max_toi_cut", ball, (-2.0, 0.0), (1.0, 0.0), 1.0, "hit at 1.5 > max 1.0"),
        c("ball", "max_toi_equal", ball, (-2.0, 0.0), (1.0, 0.0), 1.5, "hit at exactly max: kept (<=)"),
        c("ball", "zero_dir_inside", ball, (0.1, 0.2), (0.0, 0.0), m, "a == 0 inside: toi 0 for solid and hollow"),
        c("ball", "zero_dir_outside", ball, (2.0, 0.0), (0.0, 0.0), m, "a == 0 outside: miss"),
        posed(c("ball", "posed", ball, (-1.0, 2.2), (1.0, 0.0), m, "ball at (1, 2), rotated 30 deg"), rot30),
        // --- cuboid (half extents 1.0, 0.5) --------------------------------------------------
        c("cuboid", "hit_face", cub, (-3.0, 0.2), (1.0, 0.0), m, "-x face"),
        c("cuboid", "hit_face_y", cub, (0.3, 2.0), (0.0, -1.0), m, "+y face"),
        c("cuboid", "oblique", cub, (-3.0, -1.0), (1.0, 0.4), m, "enters the -x face at an angle"),
        c("cuboid", "corner_diag", cub, (-2.0, -1.5), (1.0, 1.0), m, "reaches both slabs at t = 1: near_diag, normal -dir/|dir|"),
        c("cuboid", "grazing_edge", cub, (-3.0, 0.5), (1.0, 0.0), m, "slides along the +y face: y == maxs is not outside"),
        c("cuboid", "inside", cub, (0.2, 0.1), (1.0, 0.25), m, "solid: 0 with no normal; hollow: the exit"),
        c("cuboid", "boundary_in", cub, (-1.0, 0.0), (1.0, 0.0), m, "on the -x face pointing in: near == 0 (cast_local_ray hollow returns the exit)"),
        c("cuboid", "parallel_miss", cub, (-3.0, 0.6), (1.0, 0.0), m, "parallel to x above the box"),
        c("cuboid", "pointing_away", cub, (3.0, 0.0), (1.0, 0.0), m, "far slab behind the origin"),
        c("cuboid", "max_toi_cut", cub, (-3.0, 0.0), (1.0, 0.0), 1.5, "hit at 2 > max 1.5"),
        c("cuboid", "inside_max_cut", cub, (0.0, 0.0), (1.0, 0.0), 0.5, "hollow exit at 1 > max 0.5: cast_local_ray returns max"),
        c("cuboid", "zero_dir_inside", cub, (0.2, 0.1), (0.0, 0.0), m, "zero direction inside: toi 0, feature Unknown"),
        c("cuboid", "zero_dir_outside", cub, (2.0, 0.0), (0.0, 0.0), m, "zero direction outside: miss"),
        posed(c("cuboid", "posed", cub, (-2.0, 2.0), (1.0, 0.0), m, "box at (1, 2) rotated 30 deg"), rot30),
        // --- capsule (GJK upstream) ---------------------------------------------------------
        c("capsule", "hit_side", cap, (-2.0, 0.1), (1.0, 0.0), m, "flat side"),
        c("capsule", "hit_cap", cap, (0.1, 3.0), (0.0, -1.0), m, "top cap"),
        c("capsule", "oblique", cap, (-2.0, -2.0), (1.0, 1.2), m, "diagonal, bottom cap region"),
        c("capsule", "inside", cap, (0.1, 0.2), (1.0, 0.5), m, "solid: 0 with -dir/|dir|; hollow: exit through the side, WRONG upstream (|dir| != 1, see README)"),
        c("capsule", "inside_cap_exit", cap, (0.0, 0.3), (0.0, 1.0), m, "hollow exit through the top cap"),
        c("capsule", "parallel_miss", cap, (0.5, -3.0), (0.0, 1.0), m, "parallel to the axis, outside"),
        c("capsule", "miss", cap, (-2.0, 2.0), (1.0, 0.0), m, "passes above the top cap"),
        c("capsule", "pointing_away", cap, (2.0, 0.0), (1.0, 0.0), m, "behind the origin"),
        c("capsule", "max_toi_cut", cap, (-2.0, 0.0), (1.0, 0.0), 1.5, "hit at 1.75 > max 1.5"),
        c("capsule", "unnormalized_dir", cap, (0.0, -3.0), (0.0, 2.0), m, "|dir| = 2, bottom cap at t = 1.125"),
        c("capsule", "zero_dir_inside", cap, (0.0, 0.0), (0.0, 0.0), m, "zero direction: GJK answers None even inside"),
        c("capsule", "oblique_shape", cap_oblique, (1.5, -1.0), (-1.0, 1.0), m, "oblique capsule, side hit"),
        posed(c("capsule", "posed", cap, (-3.0, 0.5), (1.0, 0.0), m, "capsule at (-1.5, 0.25) rotated -135 deg"), rotm135),
        // --- segment (-1, 0)-(1, 0) ---------------------------------------------------------
        c("segment", "from_above", seg, (0.3, 2.0), (0.0, -1.0), m, "normal . dir > 0: Face(1) with -normal"),
        c("segment", "from_below", seg, (0.3, -2.0), (0.0, 1.0), m, "normal . dir < 0: Face(0)"),
        c("segment", "oblique", seg_oblique, (1.5, -1.0), (-0.5, 1.0), m, "oblique segment"),
        c("segment", "end_point", seg, (1.0, 2.0), (0.0, -1.0), m, "hits b exactly: t == 1 is kept"),
        c("segment", "beyond_end", seg, (1.5, 2.0), (0.0, -1.0), m, "line hit beyond b: t > 1"),
        c("segment", "collinear_ahead", seg, (-3.0, 0.0), (1.0, 0.0), m, "collinear, segment ahead: Vertex(0)"),
        c("segment", "collinear_rev", seg, (3.0, 0.0), (-2.0, 0.0), m, "collinear from the b side, |dir| = 2: Vertex(1)"),
        c("segment", "collinear_on", seg, (0.0, 0.0), (1.0, 0.0), m, "collinear, origin on the segment: toi 0, Face(0)"),
        c("segment", "collinear_behind", seg, (3.0, 0.0), (1.0, 0.0), m, "collinear, segment behind: miss"),
        c("segment", "parallel_miss", seg, (-3.0, 1.0), (1.0, 0.0), m, "parallel, not collinear"),
        c("segment", "behind", seg, (0.3, 2.0), (0.0, 1.0), m, "s < 0"),
        c("segment", "max_toi_cut", seg, (0.3, 2.0), (0.0, -1.0), 1.5, "hit at 2 > max 1.5"),
        c("segment", "zero_dir", seg, (0.3, 0.0), (0.0, 0.0), m, "zero direction on the segment: dot == 0, miss"),
        posed(c("segment", "posed", seg_oblique, (0.5, 4.0), (0.5, -1.0), m, "oblique segment at (1, 2) rotated 30 deg, interior hit"), rot30),
        // --- half-space, normal +y ----------------------------------------------------------
        c("halfspace", "from_above", hs, (0.3, 2.0), (0.5, -1.0), m, "outside, hits the plane at t = 2"),
        c("halfspace", "inside_solid", hs, (0.0, -1.0), (1.0, 0.0), m, "inside, parallel: solid 0 with a zero normal; hollow 1/0 miss"),
        c("halfspace", "inside_up", hs, (0.0, -1.0), (0.0, 2.0), m, "inside pointing out: hollow exit at 0.5 with -normal"),
        c("halfspace", "parallel_miss", hs, (0.0, 1.0), (1.0, 0.0), m, "outside, parallel: -1/0 miss"),
        c("halfspace", "pointing_away", hs, (0.0, 1.0), (0.0, 1.0), m, "t < 0"),
        c("halfspace", "on_plane", hs, (0.0, 0.0), (1.0, -1.0), m, "origin on the plane: t = 0 (not inside, so normal +n)"),
        c("halfspace", "on_plane_parallel", hs, (0.0, 0.0), (1.0, 0.0), m, "0/0: miss"),
        c("halfspace", "max_toi_cut", hs, (0.3, 2.0), (0.5, -1.0), 1.5, "hit at 2 > max 1.5"),
        c("halfspace", "zero_dir_outside", hs, (0.0, 1.0), (0.0, 0.0), m, "zero direction outside: miss"),
        posed(c("halfspace", "posed", hs, (-2.0, 4.0), (1.0, -1.0), m, "plane through (1, 2) rotated 30 deg"), rot30),
    ]
}

fn feature_json(f: FeatureId) -> Value {
    match f {
        FeatureId::Vertex(i) => json!({ "kind": "vertex", "code": i }),
        FeatureId::Face(i) => json!({ "kind": "face", "code": i }),
        FeatureId::Unknown => json!({ "kind": "unknown", "code": 0 }),
    }
}

pub(crate) fn hit_json(hit: Option<RayIntersection>) -> Value {
    match hit {
        None => Value::Null,
        Some(h) => json!({
            "time_of_impact": jf(h.time_of_impact),
            "normal": jvec(h.normal),
            "feature": feature_json(h.feature),
        }),
    }
}

fn ask<S: RayCast>(s: &S, case: &Case, solid: bool) -> Value {
    let pose = case.pose.p();
    let ray = Ray::new(case.origin.v(), case.dir.v());
    let max = case.max_toi.f();
    let toi = s.cast_ray(&pose, &ray, max, solid);
    let hit = s.cast_ray_and_get_normal(&pose, &ray, max, solid);
    json!({
        "toi": toi.map(jf).unwrap_or(Value::Null),
        "hit": hit_json(hit),
    })
}

fn run(case: &Case) -> Value {
    let both = |f: &dyn Fn(bool) -> Value| json!({ "solid": f(true), "hollow": f(false) });
    let expected = match case.shape {
        ShapeSpec::ConvexPolygon { vertices, count } => both(&|s| {
            ask(
                &rapier2d_f64::parry::shape::ConvexPolygon::from_convex_polyline(
                    vertices[..count].iter().map(|p| p.v()).collect(),
                )
                .unwrap(),
                case,
                s,
            )
        }),
        ShapeSpec::Ball { radius } => both(&|s| ask(&Ball::new(radius.f()), case, s)),
        ShapeSpec::Cuboid { half_extents } => {
            both(&|s| ask(&Cuboid::new(half_extents.v()), case, s))
        }
        ShapeSpec::Capsule { a, b, radius } => {
            both(&|s| ask(&Capsule::new(a.v(), b.v(), radius.f()), case, s))
        }
        ShapeSpec::Segment { a, b } => both(&|s| ask(&Segment::new(a.v(), b.v()), case, s)),
        ShapeSpec::HalfSpace { normal } => both(&|s| ask(&HalfSpace::new(normal.v()), case, s)),
    };
    json!({
        "id": case.name,
        "note": case.note,
        "shape": case.shape.json(),
        "pose": jqpose(case.pose),
        "origin": jqvec(case.origin),
        "dir": jqvec(case.dir),
        "max_toi": jq(case.max_toi),
        "expected": expected,
    })
}

/// A ray starting at the exact centre of a ball: the normal is `normalize(0) = NaN` upstream.
fn ball_center_probe() -> Value {
    let ray = Ray::new(QVec::ZERO.v(), qv(1.0, 0.0).v());
    let hit = Ball::new(0.5).cast_local_ray_and_get_normal(&ray, DEFAULT_MAX, true);
    let n = hit.map(|h| h.normal).unwrap();
    json!({ "id": "probe/ball_center_solid", "normal_is_nan": n.x.is_nan() || n.y.is_nan() })
}

pub fn generate() -> Value {
    json!({
        "family": "ray_casts",
        "cases": cases().iter().map(run).collect::<Vec<_>>(),
        "polygons": ShapeSpec::polygons().into_iter().flat_map(|(name, shape)| {
            [("outside", (-4.0, 0.003)), ("inside", (0.0, 0.003))].into_iter().map(move |(tag,o)|
                run(&c(name, tag, shape, o, (1.0, 0.0), DEFAULT_MAX, "unit ray; analytic clipping vs GJK")))
        }).collect::<Vec<_>>(),
        "non_finite_probes": [ball_center_probe()],
    })
}
