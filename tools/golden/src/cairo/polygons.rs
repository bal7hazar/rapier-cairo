//! Polygon additions in the four existing JSON families, emitted separately to keep the frozen
//! MVP enum and its consumers unchanged and the generated files below the compile budget.
use super::leaf_families::{feature, location, projection, ray_answer};
use super::{
    boolean, const_name, id, load, mass_properties, padded, pose, raw, vec2, zero_vec2, Module,
    Node, ALL_TYPES,
};
use serde_json::Value;
use std::path::Path;

fn polygon(v: &Value) -> Node {
    let vertices = v["vertices"].as_array().unwrap();
    Node::Struct(
        "ConvexPolygonRaw",
        vec![
            (
                "vertices",
                padded(vertices.iter().map(vec2).collect(), 8, "polygon", zero_vec2),
            ),
            ("count", Node::Lit(vertices.len().to_string())),
        ],
    )
}
fn emit(
    vectors: &Path,
    file: &'static str,
    ty: &'static str,
    fields: impl Fn(&Value) -> Vec<(&'static str, Node)>,
) -> String {
    let json = load(vectors, file);
    let mut module = Module::new(file, "Convex polygon golden additions (Parry f64 0.30.2).");
    let cases = json["polygons"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| {
            let mut values = vec![("id", id(c)), ("shape", polygon(&c["shape"]))];
            values.extend(fields(c));
            (const_name(c), Node::Struct(ty, values))
        })
        .collect::<Vec<_>>();
    module.table(ty, "ALL", "cases", &cases);
    let mut types: Vec<_> = ALL_TYPES
        .iter()
        .copied()
        .filter(|name| !["AabbCase", "ShapeMassCase"].contains(name))
        .collect();
    types.extend([
        "ConvexPolygonRaw",
        "PolygonAabbCase",
        "PolygonShapeMassCase",
        "PolygonProjectionCase",
        "PolygonRayCase",
        "ProjectionRaw",
        "PointFeatureRaw",
        "SegmentLocationRaw",
        "RayAnswerRaw",
        "RayHitRaw",
    ]);
    module.finish(&types)
}
pub fn aabb(v: &Path) -> String {
    emit(v, "aabb.json", "PolygonAabbCase", |c| {
        vec![
            ("pose", pose(&c["pose"])),
            ("mins", vec2(&c["expected"]["mins"])),
            ("maxs", vec2(&c["expected"]["maxs"])),
        ]
    })
}
pub fn mass(v: &Path) -> String {
    emit(v, "mass_properties.json", "PolygonShapeMassCase", |c| {
        vec![
            ("density", raw(&c["density"])),
            ("expected", mass_properties(&c["expected"])),
        ]
    })
}
pub fn point(v: &Path) -> String {
    emit(v, "point_projection.json", "PolygonProjectionCase", |c| {
        let e = &c["expected"];
        vec![
            ("point", vec2(&c["point"])),
            ("ambiguous", boolean(&c["ambiguous"])),
            ("gjk_degenerate", boolean(&c["gjk_degenerate"])),
            ("projection", projection(&e["projection"])),
            ("projection_solid", projection(&e["projection_solid"])),
            ("distance", raw(&e["distance"])),
            ("feature", feature(&e["feature"])),
            ("location", location(&e["location"])),
        ]
    })
}
pub fn ray(v: &Path) -> String {
    emit(v, "ray_casts.json", "PolygonRayCase", |c| {
        let e = &c["expected"];
        vec![
            ("pose", pose(&c["pose"])),
            ("origin", vec2(&c["origin"])),
            ("dir", vec2(&c["dir"])),
            ("max_toi", raw(&c["max_toi"])),
            ("solid", ray_answer(&e["solid"])),
            ("hollow", ray_answer(&e["hollow"])),
        ]
    })
}

use super::{konst, shape};
fn contact_shape(v: &Value) -> Node {
    if v["type"] == "convex_polygon" {
        Node::Variant("PolygonContactShapeRaw::Polygon", Box::new(polygon(v)))
    } else {
        Node::Variant("PolygonContactShapeRaw::Other", Box::new(shape(v)))
    }
}
fn manifolds(vectors: &Path, group: usize) -> String {
    let json = load(vectors, "contact_manifolds.json");
    let mut module = Module::new(
        "contact_manifolds.json",
        "Contact manifolds of every shape pair of the MVP matrix over six regimes.",
    );
    module
        .body
        .push_str(&konst("PREDICTION", "i64", &raw(&json["prediction"])));
    module.body.push('\n');

    let fid = |v: &Value| match v {
        // No trustworthy id for this point (see README): PackedFeatureId::UNKNOWN.
        Value::Null => Node::Lit("0".into()),
        v => Node::Lit(format!("0x{:08x}", v["packed"].as_u64().unwrap())),
    };
    let empty_point = || {
        Node::Struct(
            "ContactPointRaw",
            vec![
                ("local_p1", zero_vec2()),
                ("local_p2", zero_vec2()),
                ("dist", Node::Lit("0".into())),
                ("fid1", Node::Lit("0".into())),
                ("fid2", Node::Lit("0".into())),
            ],
        )
    };

    let cases: Vec<(String, Node)> = json["polygons"]
        .as_array()
        .unwrap()
        .iter()
        .skip(group * 12)
        .take(12)
        .map(|c| {
            let found = c["expected"]["manifolds"].as_array().unwrap();
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
            let node = Node::Struct(
                "PolygonManifoldCase",
                vec![
                    ("id", id(c)),
                    ("shape1", contact_shape(&c["shape1"])),
                    ("shape2", contact_shape(&c["shape2"])),
                    ("pos12", pose(&c["pos12"])),
                    ("ambiguous", boolean(&c["ambiguous"])),
                    ("num_points", Node::Lit(num_points.to_string())),
                    ("local_n1", n1),
                    ("local_n2", n2),
                    ("points", Node::Array(points)),
                ],
            );
            (const_name(c), node)
        })
        .collect();
    module.table("PolygonManifoldCase", "ALL", "cases", &cases);
    let types = [
        "PolygonManifoldCase",
        "PolygonContactShapeRaw",
        "ConvexPolygonRaw",
        "ShapeRaw",
        "Vec2Raw",
        "PoseRaw",
        "RotRaw",
        "SegmentRaw",
        "CapsuleRaw",
        "ContactPointRaw",
    ];
    module.finish(
        &types
            .into_iter()
            .filter(|ty| group != 0 || *ty != "ShapeRaw")
            .collect::<Vec<_>>(),
    )
}

pub fn manifold_files(vectors: &Path) -> Vec<(&'static str, String)> {
    let names = ["polygon", "cuboid", "segment", "capsule"];
    let paths = [
        "polygon_contacts/polygon",
        "polygon_contacts/cuboid",
        "polygon_contacts/segment",
        "polygon_contacts/capsule",
    ];
    let json = load(vectors, "contact_manifolds.json");
    let mut files = Vec::new();
    let mut module = Module::new(
        "contact_manifolds.json",
        "Polygon contact fixtures, split by pair for the compile budget.",
    );
    module
        .body
        .push_str(&konst("PREDICTION", "i64", &raw(&json["prediction"])));
    for (i, name) in names.iter().enumerate() {
        files.push((paths[i], manifolds(vectors, i)));
        module.body.push_str(&format!("pub mod {name};\n"));
    }
    let cases: Vec<_> = json["polygons"]
        .as_array()
        .unwrap()
        .iter()
        .enumerate()
        .map(|(i, c)| {
            (
                const_name(c),
                Node::Lit(format!("{}::{}", names[i / 12], const_name(c))),
            )
        })
        .collect();
    module.table("PolygonManifoldCase", "ALL", "cases", &cases);
    files.push(("polygon_contacts", module.finish(&["PolygonManifoldCase"])));
    files
}
