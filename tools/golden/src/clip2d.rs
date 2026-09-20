//! Family `clip2d`: `parry::query::details::{clip_segment_segment, clip_segment_segment_with_normal}`,
//! the polygonal-feature clipping Parry runs on the two reference/incident edges of a cuboid
//! manifold.
//!
//! Both functions return `Option<(cp_a, cp_b)>` where each clipping point is
//! `(p1, p2, feature1, feature2)`: `p1` lies on segment 1, `p2` on segment 2, and a feature is
//! `0` = first vertex of the segment as passed, `2` = second vertex, `1` = interior.

use crate::leaf::{jqseg, qv};
use crate::q::{jqvec, jvec, QVec};
use rapier2d_f64::math::Vector;
use rapier2d_f64::parry::query::details::{clip_segment_segment, clip_segment_segment_with_normal};
use serde_json::{json, Value};

type Clip = Option<(
    (Vector, Vector, usize, usize),
    (Vector, Vector, usize, usize),
)>;

struct Case {
    name: &'static str,
    seg1: (QVec, QVec),
    seg2: (QVec, QVec),
    /// Direction the segments are projected along for `clip_segment_segment_with_normal`.
    normal: QVec,
    note: &'static str,
}

const Y: (f64, f64) = (0.0, 1.0);
const X: (f64, f64) = (1.0, 0.0);

fn seg(a: (f64, f64), b: (f64, f64)) -> (QVec, QVec) {
    (qv(a.0, a.1), qv(b.0, b.1))
}

fn case(
    name: &'static str,
    seg1: (QVec, QVec),
    seg2: (QVec, QVec),
    normal: (f64, f64),
    note: &'static str,
) -> Case {
    Case {
        name,
        seg1,
        seg2,
        normal: qv(normal.0, normal.1),
        note,
    }
}

fn cases() -> Vec<Case> {
    let base = seg((0.0, 0.0), (2.0, 0.0));
    // A raw unit, 2^-32.
    let u = 1.0 / 4_294_967_296.0;
    vec![
        case("parallel_partial", base, seg((1.0, 0.25), (3.0, 0.25)), Y, "parallel, offset, overlapping on the second half of segment 1"),
        case("seg2_inside", base, seg((0.5, 0.25), (1.5, 0.25)), Y, "segment 2 inside segment 1: both clip points come from segment 2"),
        case("seg1_inside", seg((0.5, 0.0), (1.5, 0.0)), seg((0.0, 0.25), (2.0, 0.25)), Y, "segment 1 inside segment 2"),
        case("identical", base, base, Y, "same segment twice: both features are vertices"),
        case("reversed", base, seg((3.0, 0.25), (1.0, 0.25)), Y, "segment 2 runs against segment 1: features are swapped"),
        case("collinear_overlap", base, seg((1.0, 0.0), (3.0, 0.0)), Y, "collinear, overlapping"),
        case("collinear_disjoint", base, seg((3.0, 0.0), (4.0, 0.0)), Y, "collinear, disjoint: no clip"),
        case("disjoint_parallel", base, seg((3.0, 0.25), (4.0, 0.25)), Y, "parallel and offset, projections disjoint: no clip"),
        case("touch_point", seg((0.0, 0.0), (1.0, 0.0)), seg((1.0, 0.0), (2.0, 0.0)), Y, "end to end: the interval degenerates to a single point (both clip points equal)"),
        case("touch_reversed", seg((0.0, 0.0), (1.0, 0.0)), seg((2.0, 0.0), (1.0, 0.0)), Y, "single point, second segment reversed"),
        case("crossing_perp", seg((-1.0, 0.0), (1.0, 0.0)), seg((0.0, -1.0), (0.0, 1.0)), Y, "perpendicular crossing: with a y normal segment 2 projects to a single point"),
        case("oblique", seg((0.0, 0.0), (2.0, 1.0)), seg((0.5, 1.0), (2.5, 1.5)), (-0.4472135954999579, 0.8944271909999159), "oblique parallel segments, normal perpendicular to them"),
        case("seg2_point_inside", base, seg((1.0, 0.25), (1.0, 0.25)), Y, "zero-length segment 2 in the interior of segment 1"),
        case("sliver", seg((0.0, 0.0), (1.0, 0.0)), seg((1.0 - u, 0.25), (2.0, 0.25)), Y, "overlap of a single raw unit"),
        case("vertical", seg((0.0, 0.0), (0.0, 2.0)), seg((0.25, -1.0), (0.25, 1.0)), X, "vertical segments, x normal: the tangent is y"),
        case("normal_not_parallel", seg((0.0, 0.0), (2.0, 0.0)), seg((0.5, 1.0), (1.5, 0.0)), Y, "segment 2 slanted: plain clipping projects on segment 1, the normal variant on the x axis"),
    ]
}

fn clip_json(c: Clip) -> Value {
    match c {
        None => Value::Null,
        Some((a, b)) => {
            let point = |p: (Vector, Vector, usize, usize)| json!({ "p1": jvec(p.0), "p2": jvec(p.1), "f1": p.2, "f2": p.3 });
            json!({ "points": [point(a), point(b)] })
        }
    }
}

fn v(q: QVec) -> Vector {
    q.v()
}

fn finite(c: &Clip) -> bool {
    match c {
        None => true,
        Some((a, b)) => [a, b].iter().all(|p| p.0.is_finite() && p.1.is_finite()),
    }
}

fn run(c: &Case) -> Value {
    let (s1, s2) = ((v(c.seg1.0), v(c.seg1.1)), (v(c.seg2.0), v(c.seg2.1)));
    let plain = clip_segment_segment(s1, s2);
    let with_normal = clip_segment_segment_with_normal(s1, s2, v(c.normal));
    assert!(
        finite(&plain) && finite(&with_normal),
        "{}: non-finite clip",
        c.name
    );
    json!({
        "id": format!("clip/{}", c.name),
        "note": c.note,
        "seg1": jqseg(c.seg1.0, c.seg1.1),
        "seg2": jqseg(c.seg2.0, c.seg2.1),
        "normal": jqvec(c.normal),
        "expected": { "plain": clip_json(plain), "with_normal": clip_json(with_normal) },
    })
}

/// Inputs that make upstream divide by zero. They are *not* exported to Cairo: the JSON records
/// which variant produced non-finite points so the README can state it.
fn probes() -> Vec<Value> {
    let probe = |name: &str, s1: (QVec, QVec), s2: (QVec, QVec), n: (f64, f64)| {
        let (a, b) = ((v(s1.0), v(s1.1)), (v(s2.0), v(s2.1)));
        json!({
            "id": format!("probe/{name}"),
            "seg1": jqseg(s1.0, s1.1),
            "seg2": jqseg(s2.0, s2.1),
            "plain_finite": finite(&clip_segment_segment(a, b)),
            "with_normal_finite": finite(&clip_segment_segment_with_normal(a, b, v(qv(n.0, n.1)))),
        })
    };
    vec![
        probe(
            "seg1_zero_length",
            seg((1.0, 0.0), (1.0, 0.0)),
            seg((0.0, 0.0), (2.0, 0.0)),
            Y,
        ),
        probe(
            "seg2_zero_at_start",
            seg((0.0, 0.0), (2.0, 0.0)),
            seg((0.0, 0.0), (0.0, 0.0)),
            Y,
        ),
        probe(
            "both_zero_length",
            seg((1.0, 0.0), (1.0, 0.0)),
            seg((1.0, 0.0), (1.0, 0.0)),
            Y,
        ),
    ]
}

pub fn generate() -> Value {
    json!({
        "family": "clip2d",
        "cases": cases().iter().map(run).collect::<Vec<_>>(),
        "non_finite_probes": probes(),
    })
}
