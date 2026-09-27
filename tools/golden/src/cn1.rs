//! Work package CN1: start contacts of the `shape_casts` family, appended after CC1's cases.
//!
//! A character-shaped shape 2 rests `GAP = 2^-16` away from a face of shape 1 (a floor, a wall,
//! or the floor as shape 2 under a ball), so that every option set of
//! [`crate::shape_casts::OPTIONS`] answers through upstream's contact-geometry fallback
//! (`t < 1e-4`, or `t = 0` within the target distance) on a GJK contact that does not fail (the
//! shapes are not exactly touching). Regimes, with `n` the face normal from shape 1 towards shape
//! 2 and `e = perp(n)`: `into` (shape 2 moving at `-n + e / 3`), `along` (exactly along the face,
//! `e`: the move of KC1's `wall_slide`) and `away` (`n + e / 4`). Shape 1 stands still.
//!
//! After the audit of #205, appended after the first seven pairs ([`audit_pairs`]): exact zero
//! gaps (face on face, ball on face), a hexagon on the floor, rotated face contacts (floor at 30
//! degrees, a box at -60 and a ball at -17), vertices on a face (a triangle's apex, a box corner at
//! -17 degrees on the 30-degree floor) and a box whose corner is 3 or 12 ulps past the end of the
//! floor's face, `2^-16` above it (the closest pair is then corner against corner).

use crate::q::{jqpose, jqvec, QPose, QRot, QVec, Q};
use crate::sh1::Sh1Shape;
use crate::shape_casts::{hit_json, options, OPTIONS};
use crate::shapes::ShapeSpec;
use rapier2d_f64::parry::query;
use serde_json::{json, Value};

/// `2^-16`: the start gap, exact in Q32.32 and f64.
const GAP: f64 = 1.0 / 65536.0;

fn b(s: ShapeSpec) -> Sh1Shape {
    Sh1Shape::Base(s)
}

fn floor() -> (Sh1Shape, QPose) {
    (b(ShapeSpec::cuboid(8.0, 0.5)), QPose::translation(0.0, -0.5))
}

/// `(pair, shape1, pos1, shape2, pos2, n)`.
type StartPair = (&'static str, Sh1Shape, QPose, Sh1Shape, QPose, (f64, f64));

fn start_pairs() -> Vec<StartPair> {
    let x = Q::snap(1.0 / 3.0).f();
    let (fl, fl_pos) = floor();
    let on_floor = |pair, shape, bottom: f64| -> StartPair {
        (pair, fl, fl_pos, shape, QPose::translation(x, bottom + GAP), (0.0, 1.0))
    };
    let ball = b(ShapeSpec::ball(0.5));
    vec![
        on_floor("start_ball_floor", ball, 0.5),
        on_floor("start_cuboid_floor", b(ShapeSpec::cuboid(0.25, 0.5)), 0.5),
        on_floor("start_capsule_floor", b(ShapeSpec::capsule_y(0.25, 0.25)), 0.5),
        on_floor("start_tri_floor", Sh1Shape::triangle((-0.5, -0.25), (0.5, -0.25), (0.0, 0.5)), 0.25),
        on_floor("start_rcub_floor", b(ShapeSpec::cuboid(0.25, 0.25)).rounded(0.125), 0.375),
        (
            "start_ball_wall",
            b(ShapeSpec::cuboid(0.25, 2.0)),
            QPose::translation(2.25, 2.0),
            ball,
            QPose::translation(1.5 - GAP, 0.75),
            (-1.0, 0.0),
        ),
        ("start_floor_ball", ball, QPose::translation(x, 0.5 + GAP), fl, fl_pos, (0.0, -1.0)),
    ]
}

fn rot(deg: f64) -> QRot {
    if deg == 0.0 {
        QRot::IDENTITY
    } else {
        QRot::from_degrees(deg)
    }
}

/// Shape 2 (a core of `verts` dilated by `radius`) turned by `deg2`, resting `gap` above the top
/// face of the floor centred at the origin and turned by `deg1`, `dx` along that face. The
/// placement is computed in f64 from the snapped rotations, then snapped: a nonzero gap is exact
/// within about `1e-10`, a zero gap with both rotations zero exactly.
#[allow(clippy::too_many_arguments)]
fn resting(
    pair: &'static str,
    shape2: Sh1Shape,
    verts: &[(f64, f64)],
    radius: f64,
    deg1: f64,
    deg2: f64,
    dx: f64,
    gap: f64,
) -> StartPair {
    let (fl, _) = floor();
    let (r1, r2) = (rot(deg1), rot(deg2));
    let (c1, s1) = (r1.re.f(), r1.im.f());
    let (c2, s2) = (r2.re.f(), r2.im.f());
    let n = (-s1, c1);
    let depth = verts
        .iter()
        .map(|&(x, y)| -((c2 * x - s2 * y) * n.0 + (s2 * x + c2 * y) * n.1))
        .fold(f64::MIN, f64::max)
        + radius;
    let h = 0.5 + depth + gap;
    let centre = QVec::snap(c1 * dx + n.0 * h, s1 * dx + n.1 * h);
    (pair, fl, QPose::new(QVec::ZERO, r1), shape2, QPose::new(centre, r2), n)
}

fn corners(hx: f64, hy: f64) -> [(f64, f64); 4] {
    [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
}

/// The audit's pairs (see the module documentation).
fn audit_pairs() -> Vec<StartPair> {
    let x = Q::snap(1.0 / 3.0).f();
    let ulp = 1.0 / 4_294_967_296.0;
    let ball = b(ShapeSpec::ball(0.5));
    let bx = b(ShapeSpec::cuboid(0.25, 0.5));
    let hexagon = [(-0.5, -0.25), (0.5, -0.25), (0.75, 0.1), (0.4, 0.45), (-0.4, 0.45), (-0.75, 0.1)];
    let apex_down = [(-0.5, 0.5), (0.0, -0.25), (0.5, 0.5)];
    let (a, bb, c) = (apex_down[0], apex_down[1], apex_down[2]);
    vec![
        resting("start0_ball_floor", ball, &[(0.0, 0.0)], 0.5, 0.0, 0.0, x, 0.0),
        resting("start0_cuboid_floor", bx, &corners(0.25, 0.5), 0.0, 0.0, 0.0, x, 0.0),
        resting("start_hexagon_floor", b(ShapeSpec::polygon(&hexagon)), &hexagon, 0.0, 0.0, 0.0, x, GAP),
        resting("start_rot_cuboid", bx, &corners(0.25, 0.5), 0.0, 30.0, -60.0, x, GAP),
        resting("start_rot_ball", ball, &[(0.0, 0.0)], 0.5, 30.0, -17.0, x, GAP),
        resting("start_vertex_tri", Sh1Shape::triangle(a, bb, c), &apex_down, 0.0, 0.0, 0.0, x, GAP),
        resting("start_vertex_box", b(ShapeSpec::cuboid(0.25, 0.25)), &corners(0.25, 0.25), 0.0, 30.0, -17.0, x, GAP),
        resting("start_corner_in", bx, &corners(0.25, 0.5), 0.0, 0.0, 0.0, 8.25 + 3.0 * ulp, GAP),
        resting("start_corner_out", bx, &corners(0.25, 0.5), 0.0, 0.0, 0.0, 8.25 + 12.0 * ulp, GAP),
    ]
}

/// `(regime, velocity of shape 2 as (along -n, along e) coefficients)`.
const REGIMES: [(&str, f64, f64); 3] = [("into", 1.0, 1.0 / 3.0), ("along", 0.0, 1.0), ("away", -1.0, 0.25)];

pub fn start_cases() -> Vec<Value> {
    let mut out = Vec::new();
    for (pair, shape1, pos1, shape2, pos2, (nx, ny)) in start_pairs().into_iter().chain(audit_pairs()) {
        let (s1, s2) = (shape1.shared(), shape2.shared());
        let (g1, g2) = (&*s1.0, &*s2.0);
        for (regime, k_in, k_e) in REGIMES {
            let vel1 = QVec::ZERO;
            let vel2 = QVec::snap(-k_in * nx - k_e * ny, -k_in * ny + k_e * nx);
            let answers: Vec<Value> = OPTIONS
                .iter()
                .map(|o| match query::cast_shapes(&pos1.p(), vel1.v(), g1, &pos2.p(), vel2.v(), g2, options(o)) {
                    Err(_) => json!({ "supported": false, "hit": Value::Null }),
                    Ok(hit) => json!({ "supported": true, "hit": hit_json(hit) }),
                })
                .collect();
            out.push(json!({
                "id": format!("{pair}/{regime}"),
                "pair": pair,
                "regime": regime,
                "iterative": true,
                "shape1": shape1.json(),
                "shape2": shape2.json(),
                "pos1": jqpose(pos1),
                "vel1": jqvec(vel1),
                "pos2": jqpose(pos2),
                "vel2": jqvec(vel2),
                "answers": answers,
            }));
        }
    }
    out
}
