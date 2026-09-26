//! SH1 fixtures (triangles and round shapes): the three families of `crate::sh1`, each in files
//! of its own, split in parts of at most 12 cases for the compile budget.
use super::leaf_families::{feature, projection, ray_answer};
use super::polygons::contact_shape;
use super::{
    boolean, const_name, id, konst, load, mass_properties, pose, raw, vec2, zero_vec2, Module,
    Node,
};
use serde_json::Value;
use std::path::Path;

const PART: usize = 12;

const TYPES: [&str; 22] = [
    "ConvexPolygonRaw",
    "CapsuleRaw",
    "ContactPointRaw",
    "MassPropertiesRaw",
    "PointFeatureRaw",
    "PoseRaw",
    "ProjectionRaw",
    "RayAnswerRaw",
    "RayHitRaw",
    "RotRaw",
    "RoundCuboidRaw",
    "RoundPolygonRaw",
    "RoundTriangleRaw",
    "SegmentRaw",
    "ShapeRaw",
    "Sh1AabbCase",
    "Sh1ManifoldCase",
    "Sh1MassCase",
    "Sh1ProjectionCase",
    "Sh1RayCase",
    "Sh1ShapeRaw",
    "Vec2Raw",
];

fn lit(s: &str) -> Node {
    Node::Lit(s.into())
}

fn triangle(v: &Value) -> Node {
    Node::Struct(
        "TriangleRaw",
        vec![("a", vec2(&v["a"])), ("b", vec2(&v["b"])), ("c", vec2(&v["c"]))],
    )
}

/// A shape of the SH1 families.
fn sh1_shape(v: &Value) -> Node {
    let r = || raw(&v["border_radius"]);
    match v["type"].as_str().unwrap() {
        "triangle" => Node::Variant("Sh1ShapeRaw::Triangle", Box::new(triangle(v))),
        "round_cuboid" => Node::Variant(
            "Sh1ShapeRaw::RoundCuboid",
            Box::new(Node::Struct(
                "RoundCuboidRaw",
                vec![("half_extents", vec2(&v["half_extents"])), ("border_radius", r())],
            )),
        ),
        "round_triangle" => Node::Variant(
            "Sh1ShapeRaw::RoundTriangle",
            Box::new(Node::Struct(
                "RoundTriangleRaw",
                vec![("triangle", triangle(v)), ("border_radius", r())],
            )),
        ),
        "round_convex_polygon" => {
            let inner = match contact_shape(&serde_json::json!({
                "type": "convex_polygon", "vertices": v["vertices"].clone() })) {
                Node::Variant(_, inner) => *inner,
                _ => unreachable!(),
            };
            Node::Variant(
                "Sh1ShapeRaw::RoundPolygon",
                Box::new(Node::Struct(
                    "RoundPolygonRaw",
                    vec![("polygon", inner), ("border_radius", r())],
                )),
            )
        }
        "convex_polygon" => match contact_shape(v) {
            Node::Variant(_, inner) => Node::Variant("Sh1ShapeRaw::Polygon", inner),
            _ => unreachable!(),
        },
        _ => match contact_shape(v) {
            Node::Variant(_, inner) => Node::Variant("Sh1ShapeRaw::Other", inner),
            _ => unreachable!(),
        },
    }
}

fn location(v: &Value) -> Node {
    match v["kind"].as_str().unwrap() {
        "none" => lit("TriangleLocationRaw::NoLocation"),
        "solid" => lit("TriangleLocationRaw::OnSolid"),
        "vertex" => Node::Variant(
            "TriangleLocationRaw::OnVertex",
            Box::new(Node::Lit(v["vertex"].as_u64().unwrap().to_string())),
        ),
        "edge" => Node::Variant(
            "TriangleLocationRaw::OnEdge",
            Box::new(Node::Struct(
                "TriangleEdgeRaw",
                vec![
                    ("edge", Node::Lit(v["edge"].as_u64().unwrap().to_string())),
                    ("u", raw(&v["u"])),
                    ("v", raw(&v["v"])),
                ],
            )),
        ),
        other => panic!("unknown location kind {other}"),
    }
}

fn manifold_case(c: &Value) -> Node {
    let fid = |v: &Value| match v {
        Value::Null => lit("0"),
        v => Node::Lit(format!("0x{:08x}", v["packed"].as_u64().unwrap())),
    };
    let empty_point = || {
        Node::Struct(
            "ContactPointRaw",
            vec![
                ("local_p1", zero_vec2()),
                ("local_p2", zero_vec2()),
                ("dist", lit("0")),
                ("fid1", lit("0")),
                ("fid2", lit("0")),
            ],
        )
    };
    let e = &c["expected"];
    let found = e["manifolds"].as_array().unwrap();
    assert!(found.len() <= 1, "convex pairs yield at most one manifold");
    let (n1, n2, mut points) = match found.first() {
        Some(m) => (
            vec2(&m["local_n1"]),
            vec2(&m["local_n2"]),
            m["points"]
                .as_array()
                .unwrap()
                .iter()
                .map(|p| {
                    Node::Struct(
                        "ContactPointRaw",
                        vec![
                            ("local_p1", vec2(&p["local_p1"])),
                            ("local_p2", vec2(&p["local_p2"])),
                            ("dist", raw(&p["dist"])),
                            ("fid1", fid(&p["fid1"])),
                            ("fid2", fid(&p["fid2"])),
                        ],
                    )
                })
                .collect::<Vec<_>>(),
        ),
        None => (zero_vec2(), zero_vec2(), vec![]),
    };
    let num_points = points.len();
    assert!(num_points <= 2);
    while points.len() < 2 {
        points.push(empty_point());
    }
    Node::Struct(
        "Sh1ManifoldCase",
        vec![
            ("id", id(c)),
            ("shape1", sh1_shape(&c["shape1"])),
            ("shape2", sh1_shape(&c["shape2"])),
            ("pos12", pose(&c["pos12"])),
            ("ambiguous", boolean(&c["ambiguous"])),
            ("num_points", Node::Lit(num_points.to_string())),
            ("local_n1", n1),
            ("local_n2", n2),
            ("points", Node::Array(points)),
            ("intersects", boolean(&e["intersects"])),
            ("distance", raw(&e["distance"])),
        ],
    )
}

fn projection_case(c: &Value) -> Node {
    let e = &c["expected"];
    Node::Struct(
        "Sh1ProjectionCase",
        vec![
            ("id", id(c)),
            ("shape", sh1_shape(&c["shape"])),
            ("point", vec2(&c["point"])),
            ("projection", projection(&e["projection"])),
            ("projection_solid", projection(&e["projection_solid"])),
            ("distance", raw(&e["distance"])),
            ("feature", feature(&e["feature"])),
            ("location", location(&e["location"])),
        ],
    )
}

fn ray_case(c: &Value) -> Node {
    let e = &c["expected"];
    Node::Struct(
        "Sh1RayCase",
        vec![
            ("id", id(c)),
            ("shape", sh1_shape(&c["shape"])),
            ("pose", pose(&c["pose"])),
            ("origin", vec2(&c["origin"])),
            ("dir", vec2(&c["dir"])),
            ("max_toi", raw(&c["max_toi"])),
            ("solid", ray_answer(&e["solid"])),
            ("hollow", ray_answer(&e["hollow"])),
        ],
    )
}

fn mass_case(c: &Value) -> Node {
    Node::Struct(
        "Sh1MassCase",
        vec![
            ("id", id(c)),
            ("shape", sh1_shape(&c["shape"])),
            ("density", raw(&c["density"])),
            ("expected", mass_properties(&c["expected"])),
        ],
    )
}

fn aabb_case(c: &Value) -> Node {
    Node::Struct(
        "Sh1AabbCase",
        vec![
            ("id", id(c)),
            ("shape", sh1_shape(&c["shape"])),
            ("pose", pose(&c["pose"])),
            ("mins", vec2(&c["expected"]["mins"])),
            ("maxs", vec2(&c["expected"]["maxs"])),
        ],
    )
}

fn all_types(uses_other: bool) -> Vec<&'static str> {
    let mut t: Vec<&'static str> = TYPES
        .iter()
        .copied()
        .filter(|ty| uses_other || *ty != "ShapeRaw")
        .collect();
    t.extend(["TriangleRaw", "TriangleLocationRaw", "TriangleEdgeRaw"]);
    t
}

/// One family (`key` of `file`) as an index module `name` and its parts `name/part<i>`.
#[allow(clippy::too_many_arguments)]
fn family(
    vectors: &Path,
    file: &'static str,
    key: &str,
    name: &'static str,
    doc: &str,
    ty: &'static str,
    prediction: bool,
    case: fn(&Value) -> Node,
) -> Vec<(String, String)> {
    let json = load(vectors, file);
    let cases = json[key].as_array().unwrap();
    let mut files = Vec::new();
    let mut index = Module::new(file, doc);
    if prediction {
        index.body.push_str(&konst("PREDICTION", "i64", &raw(&json["prediction"])));
    }
    let mut refs = Vec::new();
    for (part, chunk) in cases.chunks(PART).enumerate() {
        let mut module = Module::new(file, &format!("{doc} Part {part}."));
        let nodes: Vec<(String, Node)> = chunk.iter().map(|c| (const_name(c), case(c))).collect();
        module.table(ty, "ALL", "cases", &nodes);
        // `ShapeRaw::` is a substring of `Sh1ShapeRaw::`: import it only when a case uses it.
        let uses_other = nodes.iter().any(|(_, n)| n.flat().contains("Sh1ShapeRaw::Other"));
        files.push((format!("{name}/part{part}"), module.finish(&all_types(uses_other))));
        index.body.push_str(&format!("pub mod part{part};\n"));
        refs.extend(chunk.iter().map(|c| {
            (const_name(c), Node::Lit(format!("part{part}::{}", const_name(c))))
        }));
    }
    index.table(ty, "ALL", "cases", &refs);
    files.push((name.to_string(), index.finish(&[ty])));
    files
}

pub fn files(vectors: &Path) -> Vec<(String, String)> {
    let mut files = family(
        vectors,
        "triangle_contacts.json",
        "cases",
        "triangle_contacts",
        "Contact manifolds of a triangle against the closed shape set (SH1).",
        "Sh1ManifoldCase",
        true,
        manifold_case,
    );
    files.extend(family(
        vectors,
        "round_shape_contacts.json",
        "cases",
        "round_shape_contacts",
        "Contact manifolds of the round cuboid, triangle and polygon (SH1).",
        "Sh1ManifoldCase",
        true,
        manifold_case,
    ));
    for (key, name, doc, ty, case) in [
        (
            "points",
            "sh1_points",
            "Point projections on triangles and round shapes (SH1).",
            "Sh1ProjectionCase",
            projection_case as fn(&Value) -> Node,
        ),
        (
            "rays",
            "sh1_rays",
            "World-space ray casts on triangles and round shapes (SH1).",
            "Sh1RayCase",
            ray_case,
        ),
        (
            "masses",
            "sh1_mass",
            "Mass properties of triangles and round shapes (SH1).",
            "Sh1MassCase",
            mass_case,
        ),
        (
            "aabbs",
            "sh1_aabb",
            "World-space AABBs of triangles and round shapes (SH1).",
            "Sh1AabbCase",
            aabb_case,
        ),
    ] {
        files.extend(family(vectors, "sh1_queries.json", key, name, doc, ty, false, case));
    }
    files
}
