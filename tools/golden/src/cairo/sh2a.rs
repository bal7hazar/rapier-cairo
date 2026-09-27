//! SH2a fixtures (polylines and 2D heightfields): the composite shapes of `crate::sh2a` as
//! constants and a `composite(index)` builder, the families `composite_contacts` and
//! `composite_queries` (index modules and parts of at most 12 cases, compile budget) and the
//! scenes `composite_scenes` (tuples, as `ccd_scenes`).
use super::leaf_families::{feature, projection, ray_answer};
use super::sh1::sh1_shape;
use super::{boolean, const_name, id, konst, load, pose, raw, vec2, zero_vec2, Module, Node};
use serde_json::Value;
use std::path::Path;

pub(super) const PART: usize = 12;

const TYPES: [&str; 26] = [
    "CapsuleRaw",
    "ClosestPointsRaw",
    "CompositeAabbCase",
    "CompositeManifoldCase",
    "CompositePairCase",
    "CompositePointCase",
    "CompositeRayCase",
    "ContactAnswerRaw",
    "ContactPointRaw",
    "ConvexPolygonRaw",
    "PartManifoldRaw",
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
    "ShapeCastHitRaw",
    "ShapeRaw",
    "Sh1ShapeRaw",
    "TriangleRaw",
    "Vec2Raw",
];

pub(super) fn lit(s: &str) -> Node {
    Node::Lit(s.into())
}

pub(super) fn zero() -> Node {
    lit("0")
}

fn types(uses_other: bool) -> Vec<&'static str> {
    TYPES.iter().copied().filter(|t| uses_other || *t != "ShapeRaw").collect()
}

/// `supported` and `value` of an optional answer, `(false, default)` when unsupported.
pub(super) fn opt<'a>(v: &'a Value) -> (bool, &'a Value) {
    (v["supported"].as_bool().unwrap(), &v["value"])
}

/// The constants of the composite shapes of `json["composites"]` (prefix `prefix`) and the
/// builder `name(index) -> CompositeRaw`.
pub(super) fn composites_module(json: &Value, prefix: &str, name: &str, body: &mut String) {
    let mut arms = String::new();
    for (i, c) in json.as_array().unwrap().iter().enumerate() {
        let shape = &c["shape"];
        let tag = format!("{prefix}{i}");
        let span = |key: &str, ty: &str, nodes: Vec<Node>, body: &mut String| -> String {
            if nodes.is_empty() {
                "array![].span()".to_string()
            } else {
                let n = nodes.len();
                body.push_str(&konst(&format!("{tag}_{key}"), &format!("[{ty}; {n}]"), &Node::Array(nodes)));
                format!("{tag}_{key}.span()")
            }
        };
        let arm = if shape["type"] == "polyline" {
            let vertices = shape["vertices"].as_array().unwrap().iter().map(vec2).collect();
            let v = span("VERTICES", "Vec2Raw", vertices, body);
            let indices: Vec<Node> = match &shape["indices"] {
                Value::Null => vec![],
                ix => ix
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|p| Node::Lit(format!("({}, {})", p[0], p[1])))
                    .collect(),
            };
            let ix = span("INDICES", "(u32, u32)", indices, body);
            format!(
                "CompositeRaw {{ is_heightfield: false, vertices: {v}, indices: {ix}, oriented: {}, heights: array![].span(), scale: Vec2Raw {{ x: 0, y: 0 }}, removed: array![].span() }}",
                shape["oriented"]
            )
        } else {
            let heights = shape["heights"].as_array().unwrap().iter().map(raw).collect();
            let h = span("HEIGHTS", "i64", heights, body);
            let removed = shape["removed"].as_array().unwrap().iter().map(|r| lit(&r.to_string())).collect();
            let r = span("REMOVED", "u32", removed, body);
            let scale = match vec2(&shape["scale"]) {
                Node::Struct(_, f) => {
                    let get = |k: &str| match &f.iter().find(|(n, _)| *n == k).unwrap().1 {
                        Node::Lit(s) => s.clone(),
                        _ => unreachable!(),
                    };
                    format!("Vec2Raw {{ x: {}, y: {} }}", get("x"), get("y"))
                }
                _ => unreachable!(),
            };
            format!(
                "CompositeRaw {{ is_heightfield: true, vertices: array![].span(), indices: array![].span(), oriented: false, heights: {h}, scale: {scale}, removed: {r} }}"
            )
        };
        arms.push_str(&format!("        {i} => {arm},\n"));
    }
    body.push_str(&format!(
        "\n/// Composite shape `index`, in the order of the JSON file.\npub fn {name}(index: u32) -> CompositeRaw {{\n    match index {{\n{arms}        _ => core::panic_with_felt252('{name}: no such shape'),\n    }}\n}}\n"
    ));
}

pub(super) fn contact_point(p: &Value) -> Node {
    let fid = |v: &Value| Node::Lit(format!("0x{:08x}", v["packed"].as_u64().unwrap()));
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
}

pub(super) fn empty_point() -> Node {
    Node::Struct(
        "ContactPointRaw",
        vec![
            ("local_p1", zero_vec2()),
            ("local_p2", zero_vec2()),
            ("dist", zero()),
            ("fid1", zero()),
            ("fid2", zero()),
        ],
    )
}

fn part_manifold(m: Option<&Value>) -> Node {
    let (part, n1, n2, mut points) = match m {
        Some(m) => (
            lit(&m["part"].to_string()),
            vec2(&m["local_n1"]),
            vec2(&m["local_n2"]),
            m["points"].as_array().unwrap().iter().map(contact_point).collect::<Vec<_>>(),
        ),
        None => (zero(), zero_vec2(), zero_vec2(), vec![]),
    };
    let n = points.len();
    while points.len() < 2 {
        points.push(empty_point());
    }
    Node::Struct(
        "PartManifoldRaw",
        vec![
            ("part", part),
            ("num_points", lit(&n.to_string())),
            ("local_n1", n1),
            ("local_n2", n2),
            ("points", Node::Array(points)),
        ],
    )
}

fn manifold_case(c: &Value) -> Node {
    let e = &c["expected"];
    let ms = e["manifolds"].as_array().unwrap();
    let manifolds = (0..4).map(|k| part_manifold(ms.get(k))).collect();
    let (is, iv) = opt(&e["intersects"]);
    let (ds, dv) = opt(&e["distance"]);
    Node::Struct(
        "CompositeManifoldCase",
        vec![
            ("id", id(c)),
            ("composite", lit(&c["composite"].to_string())),
            ("composite_first", boolean(&c["composite_first"])),
            ("other", sh1_shape(&c["other"])),
            ("pos12", pose(&c["pos12"])),
            ("num_manifolds", lit(&ms.len().to_string())),
            ("manifolds", Node::Array(manifolds)),
            ("intersects_supported", lit(&is.to_string())),
            ("intersects", if is { boolean(iv) } else { lit("false") }),
            ("distance_supported", lit(&ds.to_string())),
            ("distance", if ds { raw(dv) } else { zero() }),
        ],
    )
}

pub(super) fn point_case(c: &Value) -> Node {
    let e = &c["expected"];
    Node::Struct(
        "CompositePointCase",
        vec![
            ("id", id(c)),
            ("composite", lit(&c["composite"].to_string())),
            ("point", vec2(&c["point"])),
            ("projection", projection(&e["projection"])),
            ("projection_solid", projection(&e["projection_solid"])),
            ("feature_projection", projection(&e["feature_projection"])),
            ("feature", feature(&e["feature"])),
            ("distance", raw(&e["distance"])),
            ("contains", boolean(&e["contains"])),
        ],
    )
}

pub(super) fn ray_case(c: &Value) -> Node {
    let e = &c["expected"];
    Node::Struct(
        "CompositeRayCase",
        vec![
            ("id", id(c)),
            ("composite", lit(&c["composite"].to_string())),
            ("pose", pose(&c["pose"])),
            ("origin", vec2(&c["origin"])),
            ("dir", vec2(&c["dir"])),
            ("max_toi", raw(&c["max_toi"])),
            ("solid", ray_answer(&e["solid"])),
            ("hollow", ray_answer(&e["hollow"])),
        ],
    )
}

pub(super) fn contact_answer(prediction: &Value, v: &Value, supported: bool) -> Node {
    let some = supported && v["some"].as_bool().unwrap();
    let f = |k: &str| if some { vec2(&v[k]) } else { zero_vec2() };
    Node::Struct(
        "ContactAnswerRaw",
        vec![
            ("prediction", raw(prediction)),
            ("some", lit(&some.to_string())),
            ("point1", f("point1")),
            ("point2", f("point2")),
            ("normal1", f("normal1")),
            ("normal2", f("normal2")),
            ("dist", if some { raw(&v["dist"]) } else { zero() }),
        ],
    )
}

pub(super) fn closest(margin: &Value, v: &Value, supported: bool) -> Node {
    let kind = if supported { v["kind"].as_u64().unwrap() } else { 0 };
    let f = |k: &str| if supported && kind == 1 { vec2(&v[k]) } else { zero_vec2() };
    Node::Struct(
        "ClosestPointsRaw",
        vec![("margin", raw(margin)), ("kind", lit(&kind.to_string())), ("p1", f("p1")), ("p2", f("p2"))],
    )
}

pub(super) fn cast_hit(v: &Value, supported: bool) -> Node {
    let some = supported && v["some"].as_bool().unwrap();
    let f = |k: &str| if some { vec2(&v[k]) } else { zero_vec2() };
    Node::Struct(
        "ShapeCastHitRaw",
        vec![
            ("some", lit(&some.to_string())),
            ("toi", if some { raw(&v["toi"]) } else { zero() }),
            ("witness1", f("witness1")),
            ("witness2", f("witness2")),
            ("normal1", f("normal1")),
            ("normal2", f("normal2")),
            ("status", if some { lit(&v["status"].to_string()) } else { zero() }),
        ],
    )
}

pub(super) fn pair_case(c: &Value) -> Node {
    let e = &c["expected"];
    let (is, iv) = opt(&e["intersects"]);
    let (ds, dv) = opt(&e["distance"]);
    let (cs, cv) = opt(&e["contact"]);
    let (ps, pv) = opt(&e["closest_points"]);
    let (ks, kv) = opt(&e["cast"]);
    Node::Struct(
        "CompositePairCase",
        vec![
            ("id", id(c)),
            ("composite", lit(&c["composite"].to_string())),
            ("composite_first", boolean(&c["composite_first"])),
            ("other", sh1_shape(&c["other"])),
            ("pos1", pose(&c["pos1"])),
            ("pos2", pose(&c["pos2"])),
            ("vel", vec2(&c["vel"])),
            ("prediction", raw(&c["prediction"])),
            ("margin", raw(&c["margin"])),
            ("intersects_supported", lit(&is.to_string())),
            ("intersects", if is { boolean(iv) } else { lit("false") }),
            ("distance_supported", lit(&ds.to_string())),
            ("distance", if ds { raw(dv) } else { zero() }),
            ("contact_supported", lit(&cs.to_string())),
            ("contact", contact_answer(&c["prediction"], cv, cs)),
            ("closest_supported", lit(&ps.to_string())),
            ("closest", closest(&c["margin"], pv, ps)),
            ("cast_supported", lit(&ks.to_string())),
            ("cast", cast_hit(kv, ks)),
        ],
    )
}

fn aabb_case(c: &Value) -> Node {
    let e = &c["expected"];
    Node::Struct(
        "CompositeAabbCase",
        vec![
            ("id", id(c)),
            ("composite", lit(&c["composite"].to_string())),
            ("pose", pose(&c["pose"])),
            ("mins", vec2(&e["mins"])),
            ("maxs", vec2(&e["maxs"])),
            ("center", vec2(&e["center"])),
            ("radius", raw(&e["radius"])),
            ("mass", raw(&e["mass"])),
        ],
    )
}

/// One family (`cases` of `json[key]`) as an index module `name` and its parts `name/part<i>`.
pub(super) fn family(
    file: &'static str,
    json: &Value,
    key: &str,
    name: &'static str,
    doc: &str,
    ty: &'static str,
    header: String,
    case: fn(&Value) -> Node,
) -> Vec<(String, String)> {
    let cases = json[key].as_array().unwrap();
    let mut files = Vec::new();
    let mut index = Module::new(file, doc);
    index.body.push_str(&header);
    let mut refs = Vec::new();
    for (part, chunk) in cases.chunks(PART).enumerate() {
        let mut module = Module::new(file, &format!("{doc} Part {part}."));
        let nodes: Vec<(String, Node)> = chunk.iter().map(|c| (const_name(c), case(c))).collect();
        module.table(ty, "ALL", "cases", &nodes);
        let uses_other = nodes.iter().any(|(_, n)| n.flat().contains("Sh1ShapeRaw::Other"));
        files.push((format!("{name}/part{part}"), module.finish(&types(uses_other))));
        index.body.push_str(&format!("pub mod part{part};\n"));
        refs.extend(chunk.iter().map(|c| (const_name(c), Node::Lit(format!("part{part}::{}", const_name(c))))));
    }
    index.table(ty, "ALL", "cases", &refs);
    files.push((name.to_string(), index.finish(&[ty])));
    files
}

/// The scenes as tuples: `(id, ground)` per scene, the samples and events of each.
fn scenes_module(vectors: &Path) -> String {
    let json = load(vectors, "composite_scenes.json");
    let mut module = Module::new(
        "composite_scenes.json",
        "Family `composite_scenes`: a box over a heightfield, a ball along a polyline, a box across a polyline vertex (SH2a).",
    );
    let body = &mut module.body;
    body.push_str(&konst("DT", "i64", &raw(&json["dt"])));
    body.push_str(&konst("GRAVITY", "Vec2Raw", &vec2(&json["gravity"])));
    let scenes = json["scenes"].as_array().unwrap();
    let grounds: Value = Value::Array(scenes.iter().map(|s| serde_json::json!({ "shape": s["ground"].clone() })).collect());
    composites_module(&grounds, "GROUND", "ground", body);
    let lit_of = |n: Node| match n {
        Node::Lit(s) => s,
        _ => unreachable!(),
    };
    let mut table = Vec::new();
    let mut sample_arms = String::new();
    let mut event_arms = String::new();
    for (i, s) in scenes.iter().enumerate() {
        let shape = sh1_shape(&s["shape"]);
        let start = vec2(&s["start"]);
        let linvel = vec2(&s["linvel"]);
        table.push(Node::Lit(format!(
            "('{}', {}, {}, {})",
            s["id"].as_str().unwrap(),
            shape.flat(),
            start.flat(),
            linvel.flat()
        )));
        let samples: Vec<Node> = s["samples"]
            .as_array()
            .unwrap()
            .iter()
            .map(|x| {
                Node::Lit(format!(
                    "({}, {}, {}, {}, {}, {}, {}, {}, {})",
                    x["step"],
                    lit_of(raw(&x["x"])),
                    lit_of(raw(&x["y"])),
                    lit_of(raw(&x["re"])),
                    lit_of(raw(&x["im"])),
                    lit_of(raw(&x["vx"])),
                    lit_of(raw(&x["vy"])),
                    lit_of(raw(&x["w"])),
                    x["manifolds"]
                ))
            })
            .collect();
        let n = samples.len();
        body.push_str(&konst(
            &format!("SAMPLES{i}"),
            &format!("[(u32, i64, i64, i64, i64, i64, i64, i64, u32); {n}]"),
            &Node::Array(samples),
        ));
        sample_arms.push_str(&format!("        {i} => SAMPLES{i}.span(),\n"));
        let events: Vec<Node> = s["events"]
            .as_array()
            .unwrap()
            .iter()
            .map(|e| Node::Lit(format!("({}, {}, {}, {})", e["step"], e["started"], e["collider1"], e["collider2"])))
            .collect();
        if events.is_empty() {
            event_arms.push_str(&format!("        {i} => array![].span(),\n"));
        } else {
            let n = events.len();
            body.push_str(&konst(&format!("EVENTS{i}"), &format!("[(u32, bool, u32, u32); {n}]"), &Node::Array(events)));
            event_arms.push_str(&format!("        {i} => EVENTS{i}.span(),\n"));
        }
    }
    let n = table.len();
    body.push_str(&konst(
        "SCENES",
        &format!("[(felt252, Sh1ShapeRaw, Vec2Raw, Vec2Raw); {n}]"),
        &Node::Array(table),
    ));
    body.push_str(&format!(
        "\n/// Every scene: `(id, dynamic shape, start, linvel)`; its ground is `ground(index)`.\npub fn scenes() -> Span<(felt252, Sh1ShapeRaw, Vec2Raw, Vec2Raw)> {{\n    SCENES.span()\n}}\n\n/// The samples of scene `index`, one per step: `(step, x, y, re, im, vx, vy, w, manifolds with solver contacts)`.\npub fn samples(index: u32) -> Span<(u32, i64, i64, i64, i64, i64, i64, i64, u32)> {{\n    match index {{\n{sample_arms}        _ => core::panic_with_felt252('composite_scenes: no scene'),\n    }}\n}}\n\n/// The collision events of scene `index`: `(step, started, collider1, collider2)`, colliders 0\n/// (ground) and 1 (the body's).\npub fn events(index: u32) -> Span<(u32, bool, u32, u32)> {{\n    match index {{\n{event_arms}        _ => core::panic_with_felt252('composite_scenes: no scene'),\n    }}\n}}\n",
    ));
    module.finish(&["CompositeRaw", "Sh1ShapeRaw", "Vec2Raw", "RoundCuboidRaw", "SegmentRaw", "CapsuleRaw", "ConvexPolygonRaw", "ShapeRaw"])
}

pub fn files(vectors: &Path) -> Vec<(String, String)> {
    let contacts = load(vectors, "composite_contacts.json");
    let queries = load(vectors, "composite_queries.json");
    // The composite shapes, shared by both families (same list in both files).
    assert_eq!(contacts["composites"], queries["composites"]);
    let mut shapes = Module::new(
        "composite_contacts.json",
        "The composite shapes of the SH2a families, built from their raws.",
    );
    composites_module(&contacts["composites"], "C", "composite", &mut shapes.body);
    let mut files = vec![("composite_shapes".to_string(), shapes.finish(&["CompositeRaw", "Vec2Raw"]))];
    let header = konst("PREDICTION", "i64", &raw(&contacts["prediction"]));
    files.extend(family(
        "composite_contacts.json",
        &contacts,
        "cases",
        "composite_contacts",
        "Contact manifolds of the polyline and the heightfield against the convex shapes (SH2a).",
        "CompositeManifoldCase",
        header,
        manifold_case,
    ));
    for (key, name, doc, ty, case) in [
        ("points", "composite_points", "Point projections on the composite shapes (SH2a).", "CompositePointCase", point_case as fn(&Value) -> Node),
        ("rays", "composite_rays", "World-space ray casts on the composite shapes (SH2a).", "CompositeRayCase", ray_case),
        ("pairs", "composite_pairs", "Shape-pair queries with a composite shape (SH2a).", "CompositePairCase", pair_case),
        ("aabbs", "composite_aabbs", "AABBs, bounding spheres and mass of the composite shapes (SH2a).", "CompositeAabbCase", aabb_case),
    ] {
        files.extend(family("composite_queries.json", &queries, key, name, doc, ty, String::new(), case));
    }
    files.push(("composite_scenes".to_string(), scenes_module(vectors)));
    files
}
