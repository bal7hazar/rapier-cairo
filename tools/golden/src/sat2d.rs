//! Family `sat2d`: the separating-axis helpers of `parry::query::sat` that Parry's polygonal
//! manifold generators are built from, in both directions.
//!
//! * cuboid–cuboid: `cuboid_cuboid_find_local_separating_normal_oneway(c1, c2, pos12)` and
//!   `(c2, c1, pos21)`;
//! * cuboid–segment: `cuboid_support_map_find_local_separating_normal_oneway(cuboid, segment,
//!   pos12)` (the four face normals of the cuboid) and
//!   `segment_cuboid_find_local_separating_normal_oneway(segment, cuboid, pos21)` (the one
//!   normal of the segment);
//! * cuboid–triangle: `cuboid_support_map_find_local_separating_normal_oneway(cuboid, triangle,
//!   pos12)` and `triangle_cuboid_find_local_separating_normal_oneway(triangle, cuboid, pos21)`
//!   (the three edge normals of the triangle).
//!
//! Each call reports `(separation, axis)`: the axis is expressed in the local frame of the shape
//! whose normals were tested (shape 1 for `sep1`, shape 2 for `sep2`), separation is positive
//! when the shapes are apart along it.

use crate::leaf::{jqseg, qv};
use crate::q::{jf, jqpose, jqvec, jvec, QPose, QRot, QVec, Q};
use rapier2d_f64::parry::query::sat;
use rapier2d_f64::parry::shape::{Cuboid, Segment, Triangle};
use serde_json::{json, Value};

#[derive(Copy, Clone)]
enum Shape {
    Cuboid(QVec),
    Segment(QVec, QVec),
    Triangle(QVec, QVec, QVec),
}

impl Shape {
    fn json(self) -> Value {
        match self {
            Shape::Cuboid(h) => json!({ "type": "cuboid", "half_extents": jqvec(h) }),
            Shape::Segment(a, b) => {
                let mut v = jqseg(a, b);
                v["type"] = "segment".into();
                v
            }
            Shape::Triangle(a, b, c) => {
                json!({ "type": "triangle", "a": jqvec(a), "b": jqvec(b), "c": jqvec(c) })
            }
        }
    }
}

struct Case {
    pair: &'static str,
    name: &'static str,
    shape1: Shape,
    shape2: Shape,
    /// Pose of shape 2 in the frame of shape 1.
    pos12: QPose,
    /// Two axes tie exactly (or the answer hinges on the sign of an exact zero).
    ambiguous: bool,
    note: &'static str,
}

fn pose(x: f64, y: f64, rot: QRot) -> QPose {
    QPose::new(qv(x, y), rot)
}

fn deg(d: f64) -> QRot {
    QRot::from_degrees(d)
}

const ID: QRot = QRot::IDENTITY;

#[rustfmt::skip]
fn cases() -> Vec<Case> {
    let mut v = Vec::new();
    let mut add = |pair, name, shape1, shape2, pos12, ambiguous, note| {
        v.push(Case { pair, name, shape1, shape2, pos12, ambiguous, note });
    };
    let cub = |x, y| Shape::Cuboid(qv(x, y));

    // --- cuboid / cuboid (same configurations as contact_manifolds/cuboid_cuboid) ------------
    let (c1, c2) = (cub(1.0, 0.5), cub(0.5, 0.5));
    add("cuboid_cuboid", "separated", c1, c2, pose(3.0, 0.0, ID), false, "gap 1.5 along x only");
    add("cuboid_cuboid", "within_pred", c1, c2, pose(1.51, 0.25, ID), false, "gap 0.01 along x");
    add("cuboid_cuboid", "touching", c1, c2, pose(1.5, 0.0, ID), false, "faces in exact contact: separation 0 along +x");
    add("cuboid_cuboid", "shallow", c1, c2, pose(1.6, 0.3, deg(30.0)), false, "vertex of cuboid 2 inside a face of cuboid 1");
    add("cuboid_cuboid", "deep", c1, c2, pose(0.3, 0.2, deg(45.0)), false, "centre of cuboid 2 inside cuboid 1");
    add("cuboid_cuboid", "degenerate", c1, c2, pose(0.0, 0.0, ID), true,
        "coincident centres: the orientation of every tested axis is decided by the sign of an exact zero translation (+)");
    add("cuboid_cuboid", "degen_corner", c1, c2, pose(1.5, 1.0, ID), true,
        "corner against corner, exactly touching: x and y separations are both 0");
    add("cuboid_cuboid", "degen_rot90", c1, cub(1.0, 0.5), pose(0.0, 1.49, QRot::QUARTER), false,
        "exact quarter turn: edges parallel through a rotation, penetration 0.01");
    add("cuboid_cuboid", "sep_diagonal", c1, c2, pose(2.5, 1.8, ID), false,
        "separated along x and y: upstream returns the separation along the weighted diagonal, not the maximum axis separation");

    // --- cuboid / segment ---------------------------------------------------------------------
    let (cu, sg) = (cub(1.0, 0.5), Shape::Segment(qv(0.0, -0.5), qv(0.0, 0.5)));
    add("cuboid_segment", "separated", cu, sg, pose(3.0, 0.0, ID), false, "vertical segment, gap 2 along x");
    add("cuboid_segment", "within_pred", cu, sg, pose(1.01, 0.0, ID), false, "gap 0.01");
    add("cuboid_segment", "touching", cu, sg, pose(1.0, 0.0, ID), false, "segment lying on the +x face: separation 0, parallel to the face");
    add("cuboid_segment", "shallow", cu, sg, pose(0.9, 0.2, deg(30.0)), false, "rotated segment crossing the +x face");
    add("cuboid_segment", "deep", cu, sg, pose(0.2, 0.1, QRot::QUARTER), false, "horizontal segment through the cuboid");
    add("cuboid_segment", "degenerate", cu, sg, pose(0.0, 0.0, ID), true,
        "segment through the cuboid centre along y: face normals tie");

    // --- cuboid / triangle --------------------------------------------------------------------
    let tri = Shape::Triangle(qv(-0.5, -0.5), qv(0.5, -0.5), qv(0.0, 0.5));
    add("cuboid_triangle", "separated", cu, tri, pose(3.0, 0.0, ID), false, "gap 1.5 along x");
    add("cuboid_triangle", "within_pred", cu, tri, pose(1.51, 0.0, ID), false, "gap 0.01, closest feature: vertex against face");
    add("cuboid_triangle", "touching", cu, tri, pose(1.5, 0.0, ID), true,
        "triangle vertex on the corner of the cuboid: exact contact");
    add("cuboid_triangle", "shallow", cu, tri, pose(1.2, 0.2, deg(30.0)), false, "rotated triangle vertex 0.05 inside the +x face");
    add("cuboid_triangle", "deep", cu, tri, pose(0.2, 0.1, deg(45.0)), false, "triangle centre inside the cuboid");
    add("cuboid_triangle", "degenerate", cu, tri, pose(0.0, 0.0, ID), true, "triangle inside the cuboid");
    add("cuboid_triangle", "degen_edge", cu, tri, pose(0.0, 1.0, ID), true,
        "triangle base edge lying on the +y face: parallel edges in exact contact");

    v
}

fn cuboid(s: Shape) -> Cuboid {
    match s {
        Shape::Cuboid(h) => Cuboid::new(h.v()),
        _ => panic!("cuboid expected"),
    }
}

fn segment(s: Shape) -> Segment {
    match s {
        Shape::Segment(a, b) => Segment::new(a.v(), b.v()),
        _ => panic!("segment expected"),
    }
}

fn triangle(s: Shape) -> Triangle {
    match s {
        Shape::Triangle(a, b, c) => Triangle::new(a.v(), b.v(), c.v()),
        _ => panic!("triangle expected"),
    }
}

/// The inverse pose, snapped to Q32.32 so that it is an exactly representable *input* of the
/// second direction (the rotation conjugate is exact; only the translation is rounded).
fn snapped_inverse(p: QPose) -> QPose {
    let inv = p.p().inverse();
    QPose::new(
        QVec::snap(inv.translation.x, inv.translation.y),
        QRot {
            re: p.rotation.re,
            im: Q(-p.rotation.im.0),
        },
    )
}

fn axis_json((separation, axis): (f64, rapier2d_f64::math::Vector)) -> Value {
    json!({ "separation": jf(separation), "axis": jvec(axis) })
}

fn run(case: &Case) -> Value {
    let pos12 = case.pos12;
    let pos21 = snapped_inverse(pos12);
    let (p12, p21) = (pos12.p(), pos21.p());
    let (sep1, sep2) = match (case.shape1, case.shape2) {
        (Shape::Cuboid(_), Shape::Cuboid(_)) => {
            let (c1, c2) = (cuboid(case.shape1), cuboid(case.shape2));
            (
                sat::cuboid_cuboid_find_local_separating_normal_oneway(&c1, &c2, &p12),
                sat::cuboid_cuboid_find_local_separating_normal_oneway(&c2, &c1, &p21),
            )
        }
        (Shape::Cuboid(_), Shape::Segment(..)) => {
            let (c, s) = (cuboid(case.shape1), segment(case.shape2));
            (
                sat::cuboid_support_map_find_local_separating_normal_oneway(&c, &s, &p12),
                sat::segment_cuboid_find_local_separating_normal_oneway(&s, &c, &p21),
            )
        }
        (Shape::Cuboid(_), Shape::Triangle(..)) => {
            let (c, t) = (cuboid(case.shape1), triangle(case.shape2));
            (
                sat::cuboid_support_map_find_local_separating_normal_oneway(&c, &t, &p12),
                sat::triangle_cuboid_find_local_separating_normal_oneway(&t, &c, &p21),
            )
        }
        _ => panic!("unsupported pair"),
    };
    json!({
        "id": format!("{}/{}", case.pair, case.name),
        "pair": case.pair,
        "ambiguous": case.ambiguous,
        "note": case.note,
        "shape1": case.shape1.json(),
        "shape2": case.shape2.json(),
        "pos12": jqpose(pos12),
        "pos21": jqpose(pos21),
        "expected": { "sep1": axis_json(sep1), "sep2": axis_json(sep2) },
    })
}

/// A zero-length segment has no normal: upstream answers `(-f64::MAX, 0)` instead of a
/// separation. Recorded, not exported (it cannot be quantised).
fn zero_length_segment_probe() -> Value {
    let c = Cuboid::new(qv(1.0, 0.5).v());
    let s = Segment::new(qv(0.5, 0.5).v(), qv(0.5, 0.5).v());
    let pos = QPose::translation(0.25, 0.0).p();
    let (separation, axis) = sat::segment_cuboid_find_local_separating_normal_oneway(&s, &c, &pos);
    json!({
        "id": "probe/segment_no_normal",
        "separation_is_minus_f64_max": separation == -f64::MAX,
        "axis_is_zero": axis == rapier2d_f64::math::Vector::ZERO,
    })
}

pub fn generate() -> Value {
    json!({
        "family": "sat2d",
        "cases": cases().iter().map(run).collect::<Vec<_>>(),
        "non_finite_probes": [zero_length_segment_probe()],
    })
}
