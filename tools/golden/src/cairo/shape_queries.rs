//! Fixtures of the QY1 family `shape_queries`: distance, closest points and contact on every
//! pair of the closed shape set.
use super::polygons::contact_shape;
use super::{boolean, id, load, pose, raw, vec2, zero_vec2, Module, Node};
use serde_json::Value;
use std::path::Path;

fn zero() -> Node {
    Node::Lit("0".into())
}

fn closest(v: &Value) -> Node {
    let kind = match v["kind"].as_str().unwrap() {
        "intersecting" => "0",
        "within_margin" => "1",
        "disjoint" => "2",
        other => panic!("unknown closest-points kind {other}"),
    };
    let (p1, p2) = match &v["points"] {
        Value::Null => (zero_vec2(), zero_vec2()),
        p => (vec2(&p[0]), vec2(&p[1])),
    };
    Node::Struct(
        "ClosestPointsRaw",
        vec![
            ("margin", raw(&v["margin"])),
            ("kind", Node::Lit(kind.into())),
            ("p1", p1),
            ("p2", p2),
        ],
    )
}

fn contact(v: &Value) -> Node {
    let c = &v["contact"];
    let fields = if c.is_null() {
        vec![
            ("prediction", raw(&v["prediction"])),
            ("some", Node::Lit("false".into())),
            ("point1", zero_vec2()),
            ("point2", zero_vec2()),
            ("normal1", zero_vec2()),
            ("normal2", zero_vec2()),
            ("dist", zero()),
        ]
    } else {
        vec![
            ("prediction", raw(&v["prediction"])),
            ("some", Node::Lit("true".into())),
            ("point1", vec2(&c["point1"])),
            ("point2", vec2(&c["point2"])),
            ("normal1", vec2(&c["normal1"])),
            ("normal2", vec2(&c["normal2"])),
            ("dist", raw(&c["dist"])),
        ]
    };
    Node::Struct("ContactAnswerRaw", fields)
}

pub fn shape_queries(vectors: &Path) -> String {
    let json = load(vectors, "shape_queries.json");
    let mut module = Module::new(
        "shape_queries.json",
        "Parry's `distance` / `closest_points` / `contact` on every pair of the closed set (QY1).",
    );
    let cases: Vec<(String, Node)> = json["cases"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| {
            let e = &c["expected"];
            let supported = e["supported"].as_bool().unwrap();
            let (distance, closest_points, contacts) = if supported {
                (
                    raw(&e["distance"]),
                    e["closest_points"].as_array().unwrap().iter().map(closest).collect(),
                    e["contacts"].as_array().unwrap().iter().map(contact).collect(),
                )
            } else {
                let none = |_: &i64| {
                    Node::Struct(
                        "ClosestPointsRaw",
                        vec![("margin", zero()), ("kind", zero()), ("p1", zero_vec2()), ("p2", zero_vec2())],
                    )
                };
                let no_contact = |_: &i64| {
                    contact(&serde_json::json!({ "prediction": { "raw": 0 }, "contact": null }))
                };
                (zero(), [0i64; 3].iter().map(none).collect(), [0i64; 2].iter().map(no_contact).collect())
            };
            let node = Node::Struct(
                "ShapeQueryCase",
                vec![
                    ("id", id(c)),
                    ("shape1", contact_shape(&c["shape1"])),
                    ("shape2", contact_shape(&c["shape2"])),
                    ("pos1", pose(&c["pos1"])),
                    ("pos2", pose(&c["pos2"])),
                    ("supported", boolean(&e["supported"])),
                    ("iterative_distance", boolean(&c["iterative_distance"])),
                    ("iterative_closest_points", boolean(&c["iterative_closest_points"])),
                    ("iterative_contact", boolean(&c["iterative_contact"])),
                    ("contact_swapped", boolean(&c["contact_swapped"])),
                    ("distance", distance),
                    ("closest_points", Node::Array(closest_points)),
                    ("contacts", Node::Array(contacts)),
                ],
            );
            (super::const_name(c), node)
        })
        .collect();
    module.table("ShapeQueryCase", "ALL", "cases", &cases);
    module.finish(&[
        "ShapeQueryCase",
        "ClosestPointsRaw",
        "ContactAnswerRaw",
        "PolygonContactShapeRaw",
        "ConvexPolygonRaw",
        "ShapeRaw",
        "Vec2Raw",
        "PoseRaw",
        "RotRaw",
        "SegmentRaw",
        "CapsuleRaw",
    ])
}
