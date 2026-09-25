//! Fixtures of the SE families: `intersection_tests` and the `sensor_trigger` scene.
use super::polygons::contact_shape;
use super::{boolean, id, konst, load, pose, raw, vec2, Module, Node};
use std::path::Path;

pub fn intersection_tests(vectors: &Path) -> String {
    let json = load(vectors, "intersection_tests.json");
    let mut module = Module::new(
        "intersection_tests.json",
        "`DefaultQueryDispatcher::intersection_test` on every supported pair, four regimes (SE).",
    );
    let cases: Vec<(String, Node)> = json["cases"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| {
            let e = &c["expected"];
            let node = Node::Struct(
                "IntersectionCase",
                vec![
                    ("id", id(c)),
                    ("shape1", contact_shape(&c["shape1"])),
                    ("shape2", contact_shape(&c["shape2"])),
                    ("pos12", pose(&c["pos12"])),
                    ("supported", boolean(&e["supported"])),
                    ("intersecting", boolean(&e["intersecting"])),
                    ("gjk_touching", boolean(&c["gjk_touching"])),
                ],
            );
            (super::const_name(c), node)
        })
        .collect();
    module.table("IntersectionCase", "ALL", "cases", &cases);
    module.finish(&[
        "IntersectionCase",
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

/// Event tuple: `(step, started, collider1, collider2, sensor, removed)`.
const EVENT: &str = "(u32, bool, u32, u32, bool, bool)";
const EVENT_DOC: &str = "(step, started, collider1, collider2, sensor, removed)";
/// Sample tuple: `(step, y, vy, has_pair, intersecting)`.
const SAMPLE: &str = "(u32, i64, i64, bool, bool)";
const SAMPLE_DOC: &str = "(step, y, vy, has_pair, intersecting)";

pub fn sensor_trigger(vectors: &Path) -> String {
    let json = load(vectors, "sensor_trigger.json");
    let mut module = Module::new(
        "sensor_trigger.json",
        "Scene `sensor_trigger`: a ball falling through a standalone sensor slab (SE).",
    );
    let body = &mut module.body;
    body.push_str(&konst("DT", "i64", &raw(&json["dt"])));
    body.push_str(&konst("GRAVITY", "Vec2Raw", &vec2(&json["gravity"])));
    let half = vec2(&json["slab_half_extents"]);
    body.push_str(&konst("SLAB_HALF_EXTENTS", "Vec2Raw", &half));
    body.push_str(&konst("SLAB_CENTER", "Vec2Raw", &vec2(&json["slab_center"])));
    body.push_str(&konst("BALL_RADIUS", "i64", &raw(&json["ball_radius"])));
    body.push_str(&konst("BALL_START", "Vec2Raw", &vec2(&json["ball_start"])));
    let lit = |v: &serde_json::Value| v.to_string();
    let events: Vec<Node> = json["events"]
        .as_array()
        .unwrap()
        .iter()
        .map(|e| {
            Node::Lit(format!(
                "({}, {}, {}, {}, {}, {})",
                lit(&e["step"]),
                lit(&e["started"]),
                lit(&e["collider1"]),
                lit(&e["collider2"]),
                lit(&e["sensor"]),
                lit(&e["removed"]),
            ))
        })
        .collect();
    let n_events = events.len();
    body.push_str(&konst(
        "EVENTS",
        &format!("[{EVENT}; {n_events}]"),
        &Node::Array(events),
    ));
    let samples: Vec<Node> = json["samples"]
        .as_array()
        .unwrap()
        .iter()
        .map(|s| {
            let pair = &s["intersecting"];
            let (Node::Lit(y), Node::Lit(vy)) = (raw(&s["y"]), raw(&s["vy"])) else {
                unreachable!()
            };
            Node::Lit(format!(
                "({}, {y}, {vy}, {}, {})",
                lit(&s["step"]),
                !pair.is_null(),
                pair.as_bool().unwrap_or(false),
            ))
        })
        .collect();
    let n_samples = samples.len();
    body.push_str(&konst(
        "SAMPLES",
        &format!("[{SAMPLE}; {n_samples}]"),
        &Node::Array(samples),
    ));
    body.push_str(&format!(
        "\n/// The recorded events, in order: `{EVENT_DOC}`.\npub fn events() -> Span<{EVENT}> {{\n    EVENTS.span()\n}}\n\n/// One sample per step, after the step: `{SAMPLE_DOC}`.\npub fn samples() -> Span<{SAMPLE}> {{\n    SAMPLES.span()\n}}\n",
    ));
    module.finish(&["Vec2Raw"])
}
