//! CC1 fixtures (shape casts): the families `shape_casts`, `nonlinear_shape_casts` and `sweep_toi` of
//! `crate::shape_casts`, each an index module and parts of at most 12 cases (compile budget).
use super::sh1::sh1_shape;
use super::{boolean, const_name, id, konst, load, pose, raw, vec2, zero_vec2, Module, Node};
use serde_json::Value;
use std::path::Path;

const PART: usize = 12;

const TYPES: [&str; 18] = [
    "CapsuleRaw",
    "ConvexPolygonRaw",
    "NonlinearMotionRaw",
    "NonlinearShapeCastCase",
    "PoseRaw",
    "RotRaw",
    "RoundCuboidRaw",
    "RoundPolygonRaw",
    "RoundTriangleRaw",
    "SegmentRaw",
    "ShapeCastCase",
    "ShapeCastHitRaw",
    "ShapeCastOptionsRaw",
    "ShapeRaw",
    "Sh1ShapeRaw",
    "SweepToiCase",
    "TriangleRaw",
    "Vec2Raw",
];

fn lit(s: &str) -> Node {
    Node::Lit(s.into())
}

/// `ShapeCastStatus` as its declaration index.
fn status(s: &str) -> Node {
    lit(match s {
        "out_of_iterations" => "0",
        "converged" => "1",
        "failed" => "2",
        "penetrating" => "3",
        other => panic!("unknown status {other}"),
    })
}

fn hit(a: &Value) -> Node {
    let h = &a["hit"];
    let fields = if h.is_null() {
        vec![
            ("some", lit("false")),
            ("toi", lit("0")),
            ("witness1", zero_vec2()),
            ("witness2", zero_vec2()),
            ("normal1", zero_vec2()),
            ("normal2", zero_vec2()),
            ("status", lit("0")),
        ]
    } else {
        vec![
            ("some", lit("true")),
            ("toi", raw(&h["toi"])),
            ("witness1", vec2(&h["witness1"])),
            ("witness2", vec2(&h["witness2"])),
            ("normal1", vec2(&h["normal1"])),
            ("normal2", vec2(&h["normal2"])),
            ("status", status(h["status"].as_str().unwrap())),
        ]
    };
    Node::Struct("ShapeCastHitRaw", fields)
}

fn answers(c: &Value) -> (Node, Node) {
    let a = c["answers"].as_array().unwrap();
    (boolean(&a[0]["supported"]), Node::Array(a.iter().map(hit).collect()))
}

fn linear_case(c: &Value) -> Node {
    let (supported, answers) = answers(c);
    Node::Struct(
        "ShapeCastCase",
        vec![
            ("id", id(c)),
            ("shape1", sh1_shape(&c["shape1"])),
            ("shape2", sh1_shape(&c["shape2"])),
            ("pos1", pose(&c["pos1"])),
            ("vel1", vec2(&c["vel1"])),
            ("pos2", pose(&c["pos2"])),
            ("vel2", vec2(&c["vel2"])),
            ("supported", supported),
            ("iterative", boolean(&c["iterative"])),
            ("answers", answers),
        ],
    )
}

fn motion(m: &Value) -> Node {
    Node::Struct(
        "NonlinearMotionRaw",
        vec![
            ("start", pose(&m["start"])),
            ("local_center", vec2(&m["local_center"])),
            ("linvel", vec2(&m["linvel"])),
            ("angvel", raw(&m["angvel"])),
        ],
    )
}

fn nonlinear_case(c: &Value) -> Node {
    let (supported, answers) = answers(c);
    Node::Struct(
        "NonlinearShapeCastCase",
        vec![
            ("id", id(c)),
            ("shape1", sh1_shape(&c["shape1"])),
            ("shape2", sh1_shape(&c["shape2"])),
            ("motion1", motion(&c["motion1"])),
            ("motion2", motion(&c["motion2"])),
            ("supported", supported),
            ("answers", answers),
        ],
    )
}

/// `SweepToiStatus` as its declaration index.
fn sweep_status(s: &str) -> Node {
    lit(match s {
        "overlapped" => "0",
        "hit" => "1",
        "separated" => "2",
        "failed" => "3",
        other => panic!("unknown sweep status {other}"),
    })
}

fn sweep_case(c: &Value) -> Node {
    Node::Struct(
        "SweepToiCase",
        vec![
            ("id", id(c)),
            ("shape1", sh1_shape(&c["shape1"])),
            ("shape2", sh1_shape(&c["shape2"])),
            ("pose1", pose(&c["pose1"])),
            ("start2", pose(&c["start2"])),
            ("end2", pose(&c["end2"])),
            ("local_center2", vec2(&c["local_center2"])),
            ("status", sweep_status(c["status"].as_str().unwrap())),
            ("fraction", raw(&c["fraction"])),
            ("point", vec2(&c["point"])),
            ("normal", vec2(&c["normal"])),
        ],
    )
}

fn options(o: &Value) -> Node {
    let max = match &o["max_time_of_impact"] {
        Value::Null => lit("0x7fffffffffffffff"),
        v => raw(v),
    };
    Node::Struct(
        "ShapeCastOptionsRaw",
        vec![
            ("max_time_of_impact", max),
            ("target_distance", raw(&o["target_distance"])),
            ("stop_at_penetration", boolean(&o["stop_at_penetration"])),
            (
                "compute_impact_geometry_on_penetration",
                boolean(&o["compute_impact_geometry_on_penetration"]),
            ),
        ],
    )
}

/// The types a part of family `ty` may import. `ShapeRaw` is a substring of `Sh1ShapeRaw` and
/// `ShapeCastCase` of `NonlinearShapeCastCase`: each is only kept where it is really used.
fn types(ty: &str, uses_other: bool) -> Vec<&'static str> {
    TYPES
        .iter()
        .copied()
        .filter(|t| uses_other || *t != "ShapeRaw")
        .filter(|t| *t != "ShapeCastCase" || ty == "ShapeCastCase")
        .collect()
}

/// One family as an index module `name` (with `header`) and its parts `name/part<i>`.
fn family(
    file: &'static str,
    name: &'static str,
    doc: &str,
    ty: &'static str,
    json: &Value,
    header: String,
    case: fn(&Value) -> Node,
) -> Vec<(String, String)> {
    let cases = json["cases"].as_array().unwrap();
    let mut files = Vec::new();
    let mut index = Module::new(file, doc);
    index.body.push_str(&header);
    let mut refs = Vec::new();
    for (part, chunk) in cases.chunks(PART).enumerate() {
        let mut module = Module::new(file, &format!("{doc} Part {part}."));
        let nodes: Vec<(String, Node)> = chunk.iter().map(|c| (const_name(c), case(c))).collect();
        module.table(ty, "ALL", "cases", &nodes);
        // `ShapeRaw::` is a substring of `Sh1ShapeRaw::`: import it only when a case uses it.
        let uses_other = nodes.iter().any(|(_, n)| n.flat().contains("Sh1ShapeRaw::Other"));
        files.push((format!("{name}/part{part}"), module.finish(&types(ty, uses_other))));
        index.body.push_str(&format!("pub mod part{part};\n"));
        refs.extend(
            chunk
                .iter()
                .map(|c| (const_name(c), Node::Lit(format!("part{part}::{}", const_name(c))))),
        );
    }
    index.table(ty, "ALL", "cases", &refs);
    files.push((name.to_string(), index.finish(&[ty, "ShapeCastOptionsRaw"])));
    files
}

pub fn files(vectors: &Path) -> Vec<(String, String)> {
    let json = load(vectors, "shape_casts.json");
    let opts: Vec<Node> = json["options"].as_array().unwrap().iter().map(options).collect();
    let header = konst("OPTIONS", "[ShapeCastOptionsRaw; 5]", &Node::Array(opts));
    let mut files = family(
        "shape_casts.json",
        "shape_casts",
        "Parry's linear `cast_shapes` on every pair of the closed set, five option sets (CC1).",
        "ShapeCastCase",
        &json,
        header,
        linear_case,
    );
    let json = load(vectors, "nonlinear_shape_casts.json");
    let header = format!(
        "{}{}",
        konst("START_TIME", "i64", &raw(&json["start_time"])),
        konst("END_TIME", "i64", &raw(&json["end_time"])),
    );
    files.extend(family(
        "nonlinear_shape_casts.json",
        "nonlinear_shape_casts",
        "Parry's `cast_shapes_nonlinear` on every pair of the closed set, stopping at penetration \
         or not (CC1).",
        "NonlinearShapeCastCase",
        &json,
        header,
        nonlinear_case,
    ));
    let json = load(vectors, "sweep_toi.json");
    let header = format!(
        "{}{}",
        konst("MAX_FRACTION", "i64", &raw(&json["max_fraction"])),
        konst("LINEAR_SLOP", "i64", &raw(&json["linear_slop"])),
    );
    files.extend(family(
        "sweep_toi.json",
        "sweep_toi",
        "Parry's `sweep_time_of_impact` on the TOI proxies of the closed set (CC1).",
        "SweepToiCase",
        &json,
        header,
        sweep_case,
    ));
    files
}
