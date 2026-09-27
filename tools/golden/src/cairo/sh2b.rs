//! SH2b fixtures (compound shapes): the compounds of `crate::sh2b` as constants and a
//! `compound(index)` builder, the families `compound_contacts`, `compound_queries` (points, rays and
//! pairs in the SH2a case types, mass properties in their own) as index modules and parts of at
//! most 12 cases (compile budget), and the scenes `compound_scenes` (tuples, as `composite_scenes`).
use super::sh1::sh1_shape;
use super::sh2a::{
    composites_module, contact_point, empty_point, family, lit, opt, pair_case, point_case, ray_case,
    zero, PART,
};
use super::{boolean, const_name, id, konst, load, pose, raw, vec2, zero_vec2, Module, Node};
use serde_json::Value;
use std::path::Path;

const TYPES: [&str; 19] = [
    "CapsuleRaw",
    "CompoundManifoldCase",
    "CompoundRaw",
    "CompoundShapeMassCase",
    "CompositeRaw",
    "ContactPointRaw",
    "ConvexPolygonRaw",
    "OtherRaw",
    "PairManifoldRaw",
    "PoseRaw",
    "RotRaw",
    "RoundCuboidRaw",
    "RoundPolygonRaw",
    "RoundTriangleRaw",
    "SegmentRaw",
    "ShapeRaw",
    "Sh1ShapeRaw",
    "TriangleRaw",
    "Vec2Raw",
];

/// The constants of the compounds of `json` (an array of `{ "shape": compound }`, prefix `prefix`)
/// and the builder `name(index) -> CompoundRaw`.
pub(super) fn compounds_module(json: &Value, prefix: &str, name: &str, body: &mut String) {
    let mut arms = String::new();
    for (i, c) in json.as_array().unwrap().iter().enumerate() {
        let parts: Vec<Node> = c["shape"]["parts"]
            .as_array()
            .unwrap()
            .iter()
            .map(|p| Node::Lit(format!("({}, {})", pose(&p["pose"]).flat(), sh1_shape(&p["shape"]).flat())))
            .collect();
        let n = parts.len();
        body.push_str(&konst(&format!("{prefix}{i}_PARTS"), &format!("[(PoseRaw, Sh1ShapeRaw); {n}]"), &Node::Array(parts)));
        arms.push_str(&format!("        {i} => CompoundRaw {{ parts: {prefix}{i}_PARTS.span() }},\n"));
    }
    body.push_str(&format!(
        "\n/// Compound `index`, in the order of the JSON file.\npub fn {name}(index: u32) -> CompoundRaw {{\n    match index {{\n{arms}        _ => core::panic_with_felt252('{name}: no such compound'),\n    }}\n}}\n"
    ));
}

fn pair_manifold(m: Option<&Value>) -> Node {
    let (s1, s2, n1, n2, mut points) = match m {
        Some(m) => (
            lit(&m["subshape1"].to_string()),
            lit(&m["subshape2"].to_string()),
            vec2(&m["local_n1"]),
            vec2(&m["local_n2"]),
            m["points"].as_array().unwrap().iter().map(contact_point).collect::<Vec<_>>(),
        ),
        None => (zero(), zero(), zero_vec2(), zero_vec2(), vec![]),
    };
    let n = points.len();
    while points.len() < 2 {
        points.push(empty_point());
    }
    Node::Struct(
        "PairManifoldRaw",
        vec![
            ("subshape1", s1),
            ("subshape2", s2),
            ("num_points", lit(&n.to_string())),
            ("local_n1", n1),
            ("local_n2", n2),
            ("points", Node::Array(points)),
        ],
    )
}

fn other(v: &Value) -> Node {
    match v["kind"].as_str().unwrap() {
        "convex" => Node::Variant("OtherRaw::Convex", Box::new(sh1_shape(&v["shape"]))),
        "composite" => Node::Variant("OtherRaw::Composite", Box::new(lit(&v["index"].to_string()))),
        "compound" => Node::Variant("OtherRaw::Compound", Box::new(lit(&v["index"].to_string()))),
        k => panic!("unknown other kind {k}"),
    }
}

fn manifold_case(c: &Value) -> Node {
    let e = &c["expected"];
    let ms = e["manifolds"].as_array().unwrap();
    let manifolds = (0..4).map(|k| pair_manifold(ms.get(k))).collect();
    let (is, iv) = opt(&e["intersects"]);
    let (ds, dv) = opt(&e["distance"]);
    let infinite = e["distance"]["infinite"].as_bool().unwrap_or(false);
    Node::Struct(
        "CompoundManifoldCase",
        vec![
            ("id", id(c)),
            ("compound", lit(&c["compound"].to_string())),
            ("compound_first", boolean(&c["compound_first"])),
            ("other", other(&c["other"])),
            ("pos12", pose(&c["pos12"])),
            ("num_manifolds", lit(&ms.len().to_string())),
            ("manifolds", Node::Array(manifolds)),
            ("intersects_supported", lit(&is.to_string())),
            ("intersects", if is { boolean(iv) } else { lit("false") }),
            ("distance_supported", lit(&ds.to_string())),
            ("distance_infinite", lit(&infinite.to_string())),
            ("distance", if ds && !infinite { raw(dv) } else { zero() }),
        ],
    )
}

fn mass_case(c: &Value) -> Node {
    let e = &c["expected"];
    Node::Struct(
        "CompoundShapeMassCase",
        vec![
            ("id", id(c)),
            ("compound", lit(&c["compound"].to_string())),
            ("pose", pose(&c["pose"])),
            ("density", raw(&c["density"])),
            ("mins", vec2(&e["mins"])),
            ("maxs", vec2(&e["maxs"])),
            ("center", vec2(&e["center"])),
            ("radius", raw(&e["radius"])),
            ("mass", raw(&e["mass"])),
            ("local_com", vec2(&e["local_com"])),
            ("inertia", raw(&e["inertia"])),
        ],
    )
}

/// One family of this module's types, as `sh2a::family`.
fn own_family(
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
        files.push((format!("{name}/part{part}"), module.finish(&TYPES)));
        index.body.push_str(&format!("pub mod part{part};\n"));
        refs.extend(chunk.iter().map(|c| (const_name(c), Node::Lit(format!("part{part}::{}", const_name(c))))));
    }
    index.table(ty, "ALL", "cases", &refs);
    files.push((name.to_string(), index.finish(&[ty])));
    files
}

/// The scenes: `(id, start, linvel)` per scene, its ground (`ground_kind`: 0 the convex
/// `ground_convex`, 1 the composite `ground`), its compound (`body`), samples and events.
fn scenes_module(vectors: &Path) -> String {
    let json = load(vectors, "compound_scenes.json");
    let mut module = Module::new(
        "compound_scenes.json",
        "Family `compound_scenes`: an L-shaped compound toppling on a half-space, an L across a polyline vertex, a three-part compound over a heightfield (SH2b).",
    );
    let body = &mut module.body;
    body.push_str(&konst("DT", "i64", &raw(&json["dt"])));
    body.push_str(&konst("GRAVITY", "Vec2Raw", &vec2(&json["gravity"])));
    let scenes = json["scenes"].as_array().unwrap();
    let dummy = serde_json::json!({ "type": "polyline", "vertices": [], "indices": null, "oriented": false });
    let grounds: Value = Value::Array(
        scenes
            .iter()
            .map(|s| {
                let g = &s["ground"];
                let shape = if g["kind"] == "composite" { g["shape"].clone() } else { dummy.clone() };
                serde_json::json!({ "shape": shape })
            })
            .collect(),
    );
    composites_module(&grounds, "GROUND", "ground", body);
    let compounds: Value = Value::Array(scenes.iter().map(|s| serde_json::json!({ "shape": s["compound"].clone() })).collect());
    compounds_module(&compounds, "BODY", "body", body);
    let lit_of = |n: Node| match n {
        Node::Lit(s) => s,
        _ => unreachable!(),
    };
    let mut table = Vec::new();
    let mut kind_arms = String::new();
    let mut convex_arms = String::new();
    let mut sample_arms = String::new();
    let mut event_arms = String::new();
    for (i, s) in scenes.iter().enumerate() {
        table.push(Node::Lit(format!(
            "('{}', {}, {})",
            s["id"].as_str().unwrap(),
            vec2(&s["start"]).flat(),
            vec2(&s["linvel"]).flat()
        )));
        let composite = s["ground"]["kind"] == "composite";
        kind_arms.push_str(&format!("        {i} => {},\n", if composite { 1 } else { 0 }));
        if !composite {
            convex_arms.push_str(&format!("        {i} => {},\n", sh1_shape(&s["ground"]["shape"]).flat()));
        }
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
    body.push_str(&konst("SCENES", &format!("[(felt252, Vec2Raw, Vec2Raw); {n}]"), &Node::Array(table)));
    body.push_str(&format!(
        "\n/// Every scene: `(id, start, linvel)`; its ground is `ground_kind(index)` and its compound\n/// `body(index)`.\npub fn scenes() -> Span<(felt252, Vec2Raw, Vec2Raw)> {{\n    SCENES.span()\n}}\n\n/// `0` when the ground of scene `index` is the convex `ground_convex(index)`, `1` when it is the\n/// composite `ground(index)`.\npub fn ground_kind(index: u32) -> u32 {{\n    match index {{\n{kind_arms}        _ => core::panic_with_felt252('compound_scenes: no scene'),\n    }}\n}}\n\n/// The convex ground of scene `index`.\npub fn ground_convex(index: u32) -> Sh1ShapeRaw {{\n    match index {{\n{convex_arms}        _ => core::panic_with_felt252('compound_scenes: no convex'),\n    }}\n}}\n\n/// The samples of scene `index`, one per step: `(step, x, y, re, im, vx, vy, w, manifolds with solver contacts)`.\npub fn samples(index: u32) -> Span<(u32, i64, i64, i64, i64, i64, i64, i64, u32)> {{\n    match index {{\n{sample_arms}        _ => core::panic_with_felt252('compound_scenes: no scene'),\n    }}\n}}\n\n/// The collision events of scene `index`: `(step, started, collider1, collider2)`, colliders 0\n/// (ground) and 1 (the body's).\npub fn events(index: u32) -> Span<(u32, bool, u32, u32)> {{\n    match index {{\n{event_arms}        _ => core::panic_with_felt252('compound_scenes: no scene'),\n    }}\n}}\n",
    ));
    module.finish(&TYPES)
}

pub fn files(vectors: &Path) -> Vec<(String, String)> {
    let contacts = load(vectors, "compound_contacts.json");
    let queries = load(vectors, "compound_queries.json");
    assert_eq!(contacts["compounds"], queries["compounds"]);
    let mut shapes = Module::new("compound_contacts.json", "The compounds of the SH2b families, built from their raws.");
    compounds_module(&contacts["compounds"], "K", "compound", &mut shapes.body);
    let mut files = vec![("compound_shapes".to_string(), shapes.finish(&TYPES))];
    let header = konst("PREDICTION", "i64", &raw(&contacts["prediction"]));
    files.extend(own_family(
        "compound_contacts.json",
        &contacts,
        "cases",
        "compound_contacts",
        "Contact manifolds of the compounds against convex shapes, half-spaces, composites and compounds (SH2b).",
        "CompoundManifoldCase",
        header,
        manifold_case,
    ));
    for (key, name, doc, ty, case) in [
        ("points", "compound_points", "Point projections on the compounds (SH2b).", "CompositePointCase", point_case as fn(&Value) -> Node),
        ("rays", "compound_rays", "World-space ray casts on the compounds (SH2b).", "CompositeRayCase", ray_case),
        ("pairs", "compound_pairs", "Shape-pair queries with a compound (SH2b).", "CompositePairCase", pair_case),
    ] {
        files.extend(family("compound_queries.json", &queries, key, name, doc, ty, String::new(), case));
    }
    files.extend(own_family(
        "compound_queries.json",
        &queries,
        "mass",
        "compound_mass",
        "AABBs, bounding spheres and mass properties of the compounds (SH2b).",
        "CompoundShapeMassCase",
        String::new(),
        mass_case,
    ));
    files.push(("compound_scenes".to_string(), scenes_module(vectors)));
    files
}
