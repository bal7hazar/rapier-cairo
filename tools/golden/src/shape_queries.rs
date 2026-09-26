//! Family `shape_queries`: Parry's top-level shape-pair queries (`query::distance`,
//! `query::closest_points`, `query::contact`, work package QY1) on the case table of
//! `intersection_tests` (every pair of the closed shape set over four regimes, plus the
//! unsupported half-space pair).
//!
//! Shape 1 sits at `pos1 = translation(1, -2)` and shape 2 at `pos2 = pos1 * pos12`, which is
//! exact in Q32.32 (a pure integer translation), so `pos1.inv_mul(pos2)` is the recorded `pos12`
//! on both sides and every answer is in world space. `closest_points` runs with three margins
//! (0, 0.1, 10) and `contact` with two predictions (0, 0.1); both `0.1` are snapped to Q32.32.
//!
//! Upstream's `contact_support_map_halfspace` (a half-space second) hands `pos12` to the
//! half-space kernel without inverting it (parry 0.30.2 and 0.31.1), so it reports contacts for
//! separated pairs and misses penetrating ones. For those cases (`contact_swapped`), the recorded
//! contacts are upstream's answer for the swapped pair, flipped back: the half-space-first
//! kernel is correct.
//!
//! The `iterative_*` flags tell which queries upstream answers with GJK (and EPA for a
//! penetrating contact) rather than with an analytic kernel, per its dispatcher order: those
//! answers carry GJK's tolerance and, for a penetrating contact, EPA's polygonal approximation
//! of a rounded shape; the port answers every pair analytically.

use crate::intersection_tests::{cases, Case};
use crate::q::{jf, jq, jqpose, jvec, QPose, QVec, Q};
use crate::shapes::ShapeSpec;
use rapier2d_f64::parry::query::{self, ClosestPoints};
use serde_json::{json, Value};

/// Margins of `closest_points` and predictions of `contact`, as Q32.32 raws.
const MARGINS: [i64; 3] = [0, 429_496_730, 10 << 32];
const PREDICTIONS: [i64; 2] = [0, 429_496_730];

fn is_ball(s: &ShapeSpec) -> bool {
    matches!(s, ShapeSpec::Ball { .. })
}

fn is_halfspace(s: &ShapeSpec) -> bool {
    matches!(s, ShapeSpec::HalfSpace { .. })
}

fn is_cuboid(s: &ShapeSpec) -> bool {
    matches!(s, ShapeSpec::Cuboid { .. })
}

fn is_segment(s: &ShapeSpec) -> bool {
    matches!(s, ShapeSpec::Segment { .. })
}

/// `DefaultQueryDispatcher::distance` reaches `distance_support_map_support_map` (GJK).
fn iterative_distance(s1: &ShapeSpec, s2: &ShapeSpec) -> bool {
    let analytic = is_ball(s1)
        || is_ball(s2)
        || (is_cuboid(s1) && is_cuboid(s2))
        || (is_segment(s1) && is_segment(s2))
        || is_halfspace(s1)
        || is_halfspace(s2);
    !analytic
}

/// `DefaultQueryDispatcher::closest_points` reaches `closest_points_support_map_support_map`.
fn iterative_closest_points(s1: &ShapeSpec, s2: &ShapeSpec) -> bool {
    let analytic = is_ball(s1)
        || is_ball(s2)
        || (is_segment(s1) && is_segment(s2))
        || is_halfspace(s1)
        || is_halfspace(s2);
    !analytic
}

/// `DefaultQueryDispatcher::contact` reaches `contact_support_map_support_map` (GJK + EPA).
fn iterative_contact(s1: &ShapeSpec, s2: &ShapeSpec) -> bool {
    !(is_ball(s1) || is_ball(s2) || is_halfspace(s1) || is_halfspace(s2))
}

/// A half-space second behind anything but a half-space: upstream's buggy flipped kernel.
fn contact_swapped(case: &Case) -> bool {
    is_halfspace(&case.shape2) && !is_halfspace(&case.shape1)
}

fn pos1() -> QPose {
    QPose::translation(1.0, -2.0)
}

/// `pos1 * pos12` for the pure integer translation `pos1`: exact.
fn pos2(pos12: QPose) -> QPose {
    let t = pos12.translation;
    let shifted = QVec { x: Q(t.x.0 + (1 << 32)), y: Q(t.y.0 - (2 << 32)) };
    QPose::new(shifted, pos12.rotation)
}

fn closest(answer: ClosestPoints, margin: i64) -> Value {
    let (kind, points) = match answer {
        ClosestPoints::Intersecting => ("intersecting", Value::Null),
        ClosestPoints::WithinMargin(p1, p2) => ("within_margin", json!([jvec(p1), jvec(p2)])),
        ClosestPoints::Disjoint => ("disjoint", Value::Null),
    };
    json!({ "margin": jq(Q(margin)), "kind": kind, "points": points })
}

fn run(case: &Case) -> Value {
    let (shape1, shape2) = (case.shape1.shared(), case.shape2.shared());
    let (g1, g2) = (&*shape1.0, &*shape2.0);
    let (p1, p2) = (pos1(), pos2(case.pos12));
    let (w1, w2) = (p1.p(), p2.p());
    let distance = query::distance(&w1, g1, &w2, g2);
    let expected = match distance {
        Err(_) => json!({ "supported": false }),
        Ok(distance) => {
            let closest_points: Vec<Value> = MARGINS
                .iter()
                .map(|&m| closest(query::closest_points(&w1, g1, &w2, g2, Q(m).f()).unwrap(), m))
                .collect();
            let contacts: Vec<Value> = PREDICTIONS
                .iter()
                .map(|&p| {
                    let answer = if contact_swapped(case) {
                        query::contact(&w2, g2, &w1, g1, Q(p).f()).unwrap().map(|c| c.flipped())
                    } else {
                        query::contact(&w1, g1, &w2, g2, Q(p).f()).unwrap()
                    };
                    let contact = match answer {
                        None => Value::Null,
                        Some(c) => json!({
                            "point1": jvec(c.point1),
                            "point2": jvec(c.point2),
                            "normal1": jvec(c.normal1),
                            "normal2": jvec(c.normal2),
                            "dist": jf(c.dist),
                        }),
                    };
                    json!({ "prediction": jq(Q(p)), "contact": contact })
                })
                .collect();
            json!({
                "supported": true,
                "distance": jf(distance),
                "closest_points": closest_points,
                "contacts": contacts,
            })
        }
    };
    json!({
        "id": format!("{}/{}", case.pair, case.regime),
        "pair": case.pair,
        "regime": case.regime,
        "iterative_distance": iterative_distance(&case.shape1, &case.shape2),
        "iterative_closest_points": iterative_closest_points(&case.shape1, &case.shape2),
        "iterative_contact": iterative_contact(&case.shape1, &case.shape2),
        "contact_swapped": contact_swapped(case),
        "shape1": case.shape1.json(),
        "shape2": case.shape2.json(),
        "pos1": jqpose(p1),
        "pos2": jqpose(p2),
        "expected": expected,
    })
}

pub fn generate() -> Value {
    json!({
        "family": "shape_queries",
        "margins": MARGINS.iter().map(|&m| jq(Q(m))).collect::<Vec<_>>(),
        "predictions": PREDICTIONS.iter().map(|&p| jq(Q(p))).collect::<Vec<_>>(),
        "cases": cases().iter().map(run).collect::<Vec<_>>(),
    })
}
