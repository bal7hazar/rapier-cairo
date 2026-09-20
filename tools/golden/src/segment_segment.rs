//! Family `segment_segment`: `closest_points_segment_segment_with_locations` and
//! `closest_points_segment_segment` (Christer Ericson's routine, as ported by Parry).
//!
//! `pos12` is the pose of segment 2 in the frame of segment 1. The reported `p1` lies in the
//! frame of segment 1, `p2` in the frame of segment 2; the squared distance is measured between
//! `p1` and `pos12 * p2`.

use crate::leaf::{jloc, jqseg, qv};
use crate::q::{jf, jqpose, jvec, QPose, QRot, QVec};
use rapier2d_f64::parry::query::details::{
    closest_points_segment_segment, closest_points_segment_segment_with_locations,
};
use rapier2d_f64::parry::query::ClosestPoints;
use rapier2d_f64::parry::shape::Segment;
use serde_json::{json, Value};

type Seg = (QVec, QVec);

struct Case {
    name: &'static str,
    seg1: Seg,
    seg2: Seg,
    pos12: QPose,
    /// The closest pair is not unique (parallel or collinear overlap): only the distance is
    /// comparable, the points and locations are whichever member of the tie upstream picks.
    ambiguous: bool,
    note: &'static str,
}

fn seg(a: (f64, f64), b: (f64, f64)) -> Seg {
    (qv(a.0, a.1), qv(b.0, b.1))
}

const ID: QPose = QPose {
    translation: QVec::ZERO,
    rotation: QRot::IDENTITY,
};

fn case(name: &'static str, seg1: Seg, seg2: Seg, note: &'static str) -> Case {
    Case {
        name,
        seg1,
        seg2,
        pos12: ID,
        ambiguous: false,
        note,
    }
}

fn posed(name: &'static str, seg1: Seg, seg2: Seg, pos12: QPose, note: &'static str) -> Case {
    Case {
        name,
        seg1,
        seg2,
        pos12,
        ambiguous: false,
        note,
    }
}

/// The closest pair is not unique.
fn tie(mut c: Case) -> Case {
    c.ambiguous = true;
    c
}

fn cases() -> Vec<Case> {
    let s1 = seg((-1.0, 0.0), (1.0, 0.0));
    let mut v = vec![
        case("crossing", s1, seg((0.0, -1.0), (0.0, 1.0)), "perpendicular crossing at the middle of both: distance 0, both on the edge"),
        case("crossing_oblique", s1, seg((-0.5, -1.0), (1.0, 1.0)), "oblique crossing"),
        tie(case("parallel", s1, seg((-0.5, 1.0), (1.5, 1.0)), "parallel, overlapping ranges: the closest pair is not unique, upstream picks s = 0 (vertex a of segment 1)")),
        tie(case("parallel_reversed", s1, seg((1.5, 1.0), (-0.5, 1.0)), "parallel, second segment reversed: another member of the tie")),
        tie(case("collinear_overlap", s1, seg((0.5, 0.0), (2.5, 0.0)), "collinear and overlapping: distance 0, the pair is not unique")),
        case("collinear_disjoint", s1, seg((2.0, 0.0), (3.0, 0.0)), "collinear, gap 1: vertex b to vertex a"),
        case("collinear_touching", s1, seg((1.0, 0.0), (2.0, 0.0)), "collinear, touching at a vertex"),
        tie(case("collinear_same", s1, s1, "identical segments: denominator 0")),
        case("endpoint_interior", s1, seg((0.3, 0.5), (0.3, 2.0)), "T configuration: end point of 2 against the interior of 1"),
        case("endpoint_endpoint", s1, seg((2.0, 1.0), (3.0, 2.0)), "vertex b of 1 against vertex a of 2"),
        case("skew_beyond", s1, seg((1.5, -1.0), (1.5, 1.0)), "segment 2 crosses the extension of 1: vertex b of 1 against the interior of 2"),
        case("nearly_parallel", s1, seg((-1.0, 1.0), (1.0, 1.0 + 1.0 / 1024.0)), "slope 2^-10: the denominator is small but above the epsilon"),
        case("zero_len_first", seg((0.0, 0.0), (0.0, 0.0)), seg((1.0, -1.0), (1.0, 1.0)), "segment 1 is a point"),
        case("zero_len_second", s1, seg((0.3, 0.7), (0.3, 0.7)), "segment 2 is a point"),
        case("both_zero_len", seg((0.0, 0.0), (0.0, 0.0)), seg((1.0, 1.0), (1.0, 1.0)), "two points"),
        posed("pose_rot30", s1, seg((0.0, -1.0), (0.0, 1.0)),
            QPose::new(qv(0.5, 0.25), QRot::from_degrees(30.0)), "segment 2 rotated by 30 degrees and moved before the query"),
        tie(posed("pose_quarter", s1, seg((0.0, -1.0), (0.0, 1.0)),
            QPose::new(qv(0.0, 2.0), QRot::QUARTER), "exact quarter turn: segment 2 becomes horizontal, parallel to 1 at height 2")),
        posed("pose_quarter_vertical", s1, seg((-1.0, 0.0), (1.0, 0.0)),
            QPose::new(qv(1.5, 0.0), QRot::QUARTER), "horizontal segment 2 turned vertical at x = 1.5: same geometry as skew_beyond through a pose"),
    ];
    // Swapped copies (identity pose): the closest distance must not depend on the order.
    let swapped: Vec<Case> = [
        "crossing_oblique",
        "parallel",
        "collinear_overlap",
        "endpoint_interior",
        "endpoint_endpoint",
        "skew_beyond",
    ]
    .iter()
    .map(|n| {
        let c = v.iter().find(|c| c.name == *n).unwrap();
        Case {
            name: match *n {
                "crossing_oblique" => "crossing_oblique_swap",
                "parallel" => "parallel_swap",
                "collinear_overlap" => "collinear_overlap_swap",
                "endpoint_interior" => "endpoint_interior_swap",
                "endpoint_endpoint" => "endpoint_endpoint_swap",
                _ => "skew_beyond_swap",
            },
            seg1: c.seg2,
            seg2: c.seg1,
            pos12: ID,
            ambiguous: c.ambiguous,
            note: "segments of the case without the suffix, in the opposite order",
        }
    })
    .collect();
    v.extend(swapped);
    v
}

fn to_segment(s: Seg) -> Segment {
    Segment::new(s.0.v(), s.1.v())
}

fn run(c: &Case) -> Value {
    let pos12 = c.pos12.p();
    let (s1, s2) = (to_segment(c.seg1), to_segment(c.seg2));
    let (loc1, loc2) = closest_points_segment_segment_with_locations(&pos12, &s1, &s2);
    let (p1, p2) = (s1.point_at(&loc1), s2.point_at(&loc2));

    // The public wrapper must give the same points (margin large enough to always answer).
    match closest_points_segment_segment(&pos12, &s1, &s2, 1.0e6) {
        ClosestPoints::WithinMargin(q1, q2) => assert!(q1 == p1 && q2 == p2),
        _ => panic!("{}: unexpected closest points variant", c.name),
    }

    let d = p1 - pos12.transform_point(p2);
    json!({
        "id": format!("seg/{}", c.name),
        "note": c.note,
        "seg1": jqseg(c.seg1.0, c.seg1.1),
        "seg2": jqseg(c.seg2.0, c.seg2.1),
        "pos12": jqpose(c.pos12),
        "ambiguous": c.ambiguous,
        "expected": {
            "loc1": jloc(&loc1),
            "loc2": jloc(&loc2),
            "p1": jvec(p1),
            "p2": jvec(p2),
            "dist_sq": jf(d.x * d.x + d.y * d.y),
        },
    })
}

pub fn generate() -> Value {
    json!({
        "family": "segment_segment",
        "cases": cases().iter().map(run).collect::<Vec<_>>(),
    })
}
