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
