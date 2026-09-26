//! Family `point_projection`: `PointQuery::project_local_point` (solid and non-solid),
//! `PointQuery::project_local_point_and_get_feature`, `PointQuery::distance_to_local_point` on
//! ball, cuboid, capsule and segment, plus
//! `PointQueryWithLocation::project_local_point_and_get_location` on the segment.
//!
//! Everything is expressed in the local frame of the shape.

use crate::leaf::jloc;
use crate::q::{jf, jqvec, jvec, QVec};
use crate::shapes::ShapeSpec;
use rapier2d_f64::math::Vector;
use rapier2d_f64::parry::query::{PointProjection, PointQuery, PointQueryWithLocation};
use rapier2d_f64::parry::shape::{Ball, Capsule, Cuboid, FeatureId, Segment, SegmentPointLocation};
use serde_json::{json, Value};

struct Case {
    name: String,
    shape: ShapeSpec,
    point: QVec,
    note: &'static str,
}

fn c(shape_name: &str, name: &str, shape: ShapeSpec, x: f64, y: f64, note: &'static str) -> Case {
    Case {
        name: format!("{shape_name}/{name}"),
        shape,
        point: QVec::snap(x, y),
        note,
    }
}

#[rustfmt::skip]
fn cases() -> Vec<Case> {
    let ball = ShapeSpec::ball(0.5);
    // Half extents (1.0, 0.5).
    let cub = ShapeSpec::cuboid(1.0, 0.5);
    // Segment (0, -0.5)-(0, 0.5), radius 0.25.
    let cap = ShapeSpec::capsule_y(0.5, 0.25);
    let cap_oblique = ShapeSpec::capsule((-0.5, -0.25), (1.0, 0.75), 0.3);
    let cap_point = ShapeSpec::capsule((0.0, 0.0), (0.0, 0.0), 0.25);
    let seg = ShapeSpec::segment((-1.0, 0.0), (1.0, 0.0));
    let seg_oblique = ShapeSpec::segment((0.0, 0.0), (2.0, 1.0));
    let seg_point = ShapeSpec::segment((0.5, 0.5), (0.5, 0.5));

    vec![
        // --- ball (radius 0.5) ---------------------------------------------------------------
        c("ball", "outside", ball, 1.2, 0.9, "distance 1.5 from the centre"),
        c("ball", "inside", ball, 0.3, 0.2, "non-solid projection pushes the point out to the surface"),
        c("ball", "on_boundary", ball, 0.5, 0.0, "distance^2 == r^2 exactly: counts as inside"),
        c("ball", "near_center", ball, 2.0f64.powi(-20), 0.0, "2^-20 from the centre: projection amplifies by r / 2^-20"),
        // --- cuboid (half extents 1.0, 0.5) --------------------------------------------------
        c("cuboid", "outside_face", cub, 2.0, 0.2, "face region of +x"),
        c("cuboid", "outside_vertex", cub, 1.5, 1.0, "vertex region of (+x, +y)"),
        c("cuboid", "inside", cub, 0.3, 0.1, "nearest face is +y (0.4 away vs 0.7)"),
        c("cuboid", "inside_tie", cub, 0.5, 0.0, "0.5 from the +x face and from both y faces: upstream picks x on a tie (`diff.x <= diff.y`)"),
        c("cuboid", "center", cub, 0.0, 0.0, "the centre: an exact zero has sign +1 (copysign of +0.0), so the +y face wins (0.5 < 1.0)"),
        c("cuboid", "on_face", cub, 1.0, 0.25, "on the +x face: inside, projection is the point itself"),
        c("cuboid", "on_vertex", cub, 1.0, 0.5, "on the (+x, +y) vertex"),
        c("cuboid", "on_edge_ext", cub, 3.0, 0.5, "on the extension of the +y edge, beyond the +x vertex: face region of +x"),
        c("cuboid", "on_edge_ext_diag", cub, 3.0, 1.0, "diagonal beyond the vertex: vertex region"),
        // --- capsule -------------------------------------------------------------------------
        c("capsule", "outside_side", cap, 1.0, 0.2, "side region"),
        c("capsule", "outside_cap", cap, 0.3, 1.2, "cap region of the top end point"),
        c("capsule", "inside", cap, 0.1, 0.2, "inside the tube"),
        c("capsule", "on_boundary", cap, 0.25, 0.2, "exactly on the side surface"),
        c("capsule", "on_segment", cap, 0.0, 0.2, "on the core segment: zero distance, upstream takes the segment normal"),
        c("capsule", "on_vertex", cap, 0.0, -0.5, "on an end point of the core segment"),
        c("capsule", "axis_extension", cap, 0.0, 1.5, "on the extension of the core segment"),
        c("capsule", "oblique_outside", cap_oblique, 0.2, 1.2, "oblique capsule, cap region"),
        c("capsule", "zero_len_center", cap_point, 0.0, 0.0, "zero-length core on the point: no normal, upstream falls back to +y"),
        c("capsule", "zero_len_outside", cap_point, 1.0, 0.0, "zero-length core, outside"),
        // --- segment -------------------------------------------------------------------------
        c("segment", "above", seg, 0.3, 0.5, "interior region, left of the direction"),
        c("segment", "below", seg, 0.3, -0.5, "interior region, right of the direction"),
        c("segment", "on_interior", seg, 0.3, 0.0, "on the segment: is_inside, feature Face(0) (perp_dot == 0)"),
        c("segment", "beyond_b", seg, 2.0, 0.5, "vertex region of b"),
        c("segment", "before_a", seg, -2.0, 0.1, "vertex region of a"),
        c("segment", "on_vertex_a", seg, -1.0, 0.0, "on vertex a: ab.ap == 0 selects vertex 0"),
        c("segment", "on_vertex_b", seg, 1.0, 0.0, "on vertex b: ab.ap == |ab|^2 selects vertex 1"),
        c("segment", "edge_ext", seg, 3.0, 0.0, "on the line through the segment, beyond b: vertex 1, not inside"),
        c("segment", "oblique", seg_oblique, 0.5, 1.0, "oblique segment, interior region"),
        c("segment", "zero_length", seg_point, 1.0, 1.0, "zero-length segment: vertex 0"),
    ]
}

pub(crate) fn proj_json(p: &PointProjection) -> Value {
    json!({ "point": jvec(p.point), "is_inside": p.is_inside })
}

pub(crate) fn feature_json(f: FeatureId) -> Value {
    match f {
        FeatureId::Vertex(i) => json!({ "kind": "vertex", "code": i }),
        FeatureId::Face(i) => json!({ "kind": "face", "code": i }),
        FeatureId::Unknown => json!({ "kind": "unknown", "code": 0 }),
    }
}

fn location_json(l: Option<SegmentPointLocation>) -> Value {
    match l {
        None => json!({ "kind": "none" }),
        Some(loc) => jloc(&loc),
    }
}

struct Answer {
    projection: PointProjection,
    solid: PointProjection,
    distance: f64,
    feature: FeatureId,
    location: Option<SegmentPointLocation>,
}

fn ask<S: PointQuery>(s: &S, p: Vector) -> Answer {
    // The feature query re-runs the projection (non-solid): both must agree.
    let (from_feature, feature) = s.project_local_point_and_get_feature(p);
    let projection = s.project_local_point(p, false);
    assert_eq!(projection.point, from_feature.point);
    assert_eq!(projection.is_inside, from_feature.is_inside);
    Answer {
        projection,
        solid: s.project_local_point(p, true),
        distance: s.distance_to_local_point(p, false),
        feature,
        location: None,
    }
}

fn run(case: &Case) -> Value {
    let p = case.point.v();
    let answer = match case.shape {
        ShapeSpec::ConvexPolygon { vertices, count } => ask(
            &rapier2d_f64::parry::shape::ConvexPolygon::from_convex_polyline(
                vertices[..count].iter().map(|p| p.v()).collect(),
            )
            .unwrap(),
            p,
        ),
        ShapeSpec::Ball { radius } => ask(&Ball::new(radius.f()), p),
        ShapeSpec::Cuboid { half_extents } => ask(&Cuboid::new(half_extents.v()), p),
        ShapeSpec::Capsule { a, b, radius } => ask(&Capsule::new(a.v(), b.v(), radius.f()), p),
        ShapeSpec::Segment { a, b } => {
            let s = Segment::new(a.v(), b.v());
            let mut answer = ask(&s, p);
            let (with_loc, loc) = s.project_local_point_and_get_location(p, false);
            assert_eq!(with_loc.point, answer.projection.point);
            answer.location = Some(loc);
            answer
        }
        ShapeSpec::HalfSpace { .. } => panic!("half-space is not part of this family"),
    };
    json!({
        "id": case.name,
        "note": case.note,
        "shape": case.shape.json(),
        "point": jqvec(case.point),
        "expected": {
            "projection": proj_json(&answer.projection),
            "projection_solid": proj_json(&answer.solid),
            "distance": jf(answer.distance),
            "feature": feature_json(answer.feature),
            "location": location_json(answer.location),
        },
    })
}

/// Projecting exactly the centre of a ball divides by zero upstream (`pt * r / 0`): recorded, not
/// exported.
fn ball_center_probe() -> Value {
    let p = Ball::new(0.5).project_local_point(Vector::new(0.0, 0.0), false);
    json!({ "id": "probe/ball_center", "projection_is_nan": p.point.x.is_nan() || p.point.y.is_nan() })
}

pub fn generate() -> Value {
    json!({
        "family": "point_projection",
        "cases": cases().iter().map(run).collect::<Vec<_>>(),
        "polygons": ShapeSpec::polygons().into_iter().flat_map(|(name, shape)| {
            [("outside", 3.0, 0.123), ("inside", 0.1, 0.002), ("tie", 0.0, 0.0)].into_iter().map(move |(tag,x,y)| {
                let mut value = run(&c(name, tag, shape, x, y, "analytic vs GJK/EPA; ties ambiguous"));
                let degenerate = name == "poly_pent" && tag == "tie";
                value["ambiguous"] = json!(tag == "tie" && !degenerate);
                value["gjk_degenerate"] = json!(degenerate);
                if degenerate { value["note"] = json!("EPA degeneracy: returns top vertex (distance 1.5) instead of bottom edge (distance 1); analytic regression"); }
                value
            })
        }).collect::<Vec<_>>(),
        "non_finite_probes": [ball_center_probe()],
    })
}
