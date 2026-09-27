//! Work package CN1: start contacts of the `shape_casts` family, appended after CC1's cases.
//!
//! A character-shaped shape 2 rests `GAP = 2^-16` away from a face of shape 1 (a floor, a wall,
//! or the floor as shape 2 under a ball), so that every option set of
//! [`crate::shape_casts::OPTIONS`] answers through upstream's contact-geometry fallback
//! (`t < 1e-4`, or `t = 0` within the target distance) on a GJK contact that does not fail (the
//! shapes are not exactly touching). Regimes, with `n` the face normal from shape 1 towards shape
//! 2 and `e = perp(n)`: `into` (shape 2 moving at `-n + e / 3`), `along` (exactly along the face,
//! `e`: the move of KC1's `wall_slide`) and `away` (`n + e / 4`). Shape 1 stands still.

use crate::q::{jqpose, jqvec, QPose, QVec, Q};
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

/// `(regime, velocity of shape 2 as (along -n, along e) coefficients)`.
const REGIMES: [(&str, f64, f64); 3] = [("into", 1.0, 1.0 / 3.0), ("along", 0.0, 1.0), ("away", -1.0, 0.25)];

pub fn start_cases() -> Vec<Value> {
    let mut out = Vec::new();
    for (pair, shape1, pos1, shape2, pos2, (nx, ny)) in start_pairs() {
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
