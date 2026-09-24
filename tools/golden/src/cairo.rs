//! JSON vectors -> Cairo fixtures (`crates/rapier_golden/src/generated/*.cairo`).
//!
//! The fixtures only carry raw Q32.32 `i64` values inside the plain structs of
//! `rapier_golden::types`, as `const` items: no scalar type, no runtime construction cost.
//! The emitted source is wrapped at 100 columns, then `scarb fmt` runs as the last step and has
//! the final word on layout (it packs short fields and array items several per line, which this
//! printer does not try to imitate). Set `GOLDEN_SKIP_FMT=1` to skip that step.

use serde_json::Value;
use std::collections::BTreeSet;
use std::fs;
use std::path::Path;
use std::process::Command;

mod leaf_families;
mod scenes;
mod sleep_impact;

const MAX_WIDTH: usize = 100;
const MAX_SHORT_STRING: usize = 31;

// ---------------------------------------------------------------------------------------------
// A tiny expression tree and its pretty printer.
// ---------------------------------------------------------------------------------------------

enum Node {
    Lit(String),
    Struct(&'static str, Vec<(&'static str, Node)>),
    Variant(&'static str, Box<Node>),
    Array(Vec<Node>),
}

impl Node {
    fn flat(&self) -> String {
        match self {
            Node::Lit(s) => s.clone(),
            Node::Struct(name, fields) => {
                let parts: Vec<String> = fields
                    .iter()
                    .map(|(k, v)| format!("{k}: {}", v.flat()))
                    .collect();
                format!("{name} {{ {} }}", parts.join(", "))
            }
            Node::Variant(name, inner) => format!("{name}({})", inner.flat()),
            Node::Array(items) => {
                let parts: Vec<String> = items.iter().map(Node::flat).collect();
                format!("[{}]", parts.join(", "))
            }
        }
    }

    /// Renders the node starting at column `column` (the text already on the line), followed by
    /// `suffix` characters on its last line, with nested lines indented by `indent` spaces.
    fn render(&self, indent: usize, column: usize, suffix: usize) -> String {
        let flat = self.flat();
        if column + flat.len() + suffix <= MAX_WIDTH {
            return flat;
        }
        let pad = " ".repeat(indent + 4);
        let close = " ".repeat(indent);
        match self {
            Node::Lit(_) => flat,
            Node::Struct(name, fields) => {
                let mut out = format!("{name} {{\n");
                for (key, value) in fields {
                    let head = format!("{pad}{key}: ");
                    let body = value.render(indent + 4, head.len(), 1);
                    out.push_str(&format!("{head}{body},\n"));
                }
                out.push_str(&format!("{close}}}"));
                out
            }
            Node::Variant(name, inner) => {
                let body = inner.render(indent + 4, pad.len(), 1);
                format!("{name}(\n{pad}{body},\n{close})")
            }
            Node::Array(items) => {
                let mut out = "[\n".to_string();
                for item in items {
                    let body = item.render(indent + 4, pad.len(), 1);
                    out.push_str(&format!("{pad}{body},\n"));
                }
                out.push_str(&format!("{close}]"));
                out
            }
        }
    }
}

fn konst(name: &str, ty: &str, value: &Node) -> String {
    let head = format!("pub const {name}: {ty} = ");
    format!("{head}{};\n", value.render(0, head.len(), 1))
}

// ---------------------------------------------------------------------------------------------
// JSON -> nodes
// ---------------------------------------------------------------------------------------------

fn raw(v: &Value) -> Node {
    Node::Lit(v["raw"].as_i64().expect("scalar raw").to_string())
}

fn int(v: &Value) -> Node {
    Node::Lit(v.as_u64().expect("integer").to_string())
}

fn boolean(v: &Value) -> Node {
    Node::Lit(v.as_bool().expect("bool").to_string())
}

fn vec2(v: &Value) -> Node {
    let r = v["raw"].as_array().expect("vector raw");
    Node::Struct(
        "Vec2Raw",
        vec![
            ("x", Node::Lit(r[0].as_i64().unwrap().to_string())),
            ("y", Node::Lit(r[1].as_i64().unwrap().to_string())),
        ],
    )
}

fn zero_vec2() -> Node {
    Node::Struct(
        "Vec2Raw",
        vec![("x", Node::Lit("0".into())), ("y", Node::Lit("0".into()))],
    )
}

fn rot(v: &Value) -> Node {
    let r = v["raw"].as_array().expect("rotation raw");
    Node::Struct(
        "RotRaw",
        vec![
            ("re", Node::Lit(r[0].as_i64().unwrap().to_string())),
            ("im", Node::Lit(r[1].as_i64().unwrap().to_string())),
        ],
    )
}

fn pose(v: &Value) -> Node {
    Node::Struct(
        "PoseRaw",
        vec![
            ("translation", vec2(&v["translation"])),
            ("rotation", rot(&v["rotation"])),
        ],
    )
}

fn shape(v: &Value) -> Node {
    match v["type"].as_str().unwrap() {
        "ball" => Node::Variant("ShapeRaw::Ball", Box::new(raw(&v["radius"]))),
        "cuboid" => Node::Variant("ShapeRaw::Cuboid", Box::new(vec2(&v["half_extents"]))),
        "capsule" => Node::Variant(
            "ShapeRaw::Capsule",
            Box::new(Node::Struct(
                "CapsuleRaw",
                vec![
                    ("a", vec2(&v["a"])),
                    ("b", vec2(&v["b"])),
                    ("radius", raw(&v["radius"])),
                ],
            )),
        ),
        "halfspace" => Node::Variant("ShapeRaw::HalfSpace", Box::new(vec2(&v["normal"]))),
        "segment" => Node::Variant(
            "ShapeRaw::Segment",
            Box::new(Node::Struct(
                "SegmentRaw",
                vec![("a", vec2(&v["a"])), ("b", vec2(&v["b"]))],
            )),
        ),
        other => panic!("unknown shape type {other}"),
    }
}

/// A string as a Cairo short string.
fn short_string(s: &str) -> Node {
    assert!(
        s.len() <= MAX_SHORT_STRING && s.is_ascii() && !s.contains('\''),
        "`{s}` does not fit a felt252 short string"
    );
    Node::Lit(format!("'{s}'"))
}

/// A case id as a Cairo short string.
fn id(v: &Value) -> Node {
    short_string(v["id"].as_str().unwrap())
}

/// A case id as a Cairo constant name: upper case, every other character becomes `_`.
fn const_name(v: &Value) -> String {
    let s = v["id"].as_str().unwrap();
    s.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() {
                c.to_ascii_uppercase()
            } else {
                '_'
            }
        })
        .collect()
}

fn fields<'a>(
    v: &'a Value,
    names: &[&'static str],
    f: fn(&Value) -> Node,
) -> Vec<(&'static str, Node)> {
    names.iter().map(|n| (*n, f(&v[*n]))).collect()
}

fn mass_properties(v: &Value) -> Node {
    Node::Struct(
        "MassPropertiesRaw",
        vec![
            ("mass", raw(&v["mass"])),
            ("inv_mass", raw(&v["inv_mass"])),
            ("local_com", vec2(&v["local_com"])),
            ("principal_inertia", raw(&v["principal_inertia"])),
            ("inv_principal_inertia", raw(&v["inv_principal_inertia"])),
        ],
    )
}

// ---------------------------------------------------------------------------------------------
// Files
// ---------------------------------------------------------------------------------------------

struct Module {
    doc: String,
    source: &'static str,
    imports: BTreeSet<&'static str>,
    body: String,
}

impl Module {
    fn new(source: &'static str, doc: &str) -> Self {
        Module {
            doc: doc.to_string(),
            source,
            imports: BTreeSet::new(),
            body: String::new(),
        }
    }

    /// Emits one constant per case plus the `ALL` table and its `cases()` accessor.
    fn table(&mut self, ty: &'static str, table: &str, accessor: &str, cases: &[(String, Node)]) {
        let mut seen = BTreeSet::new();
        for (name, node) in cases {
            assert!(seen.insert(name.clone()), "duplicate constant {name}");
            self.body.push_str(&konst(name, ty, node));
            self.body.push('\n');
        }
        let all = Node::Array(cases.iter().map(|(n, _)| Node::Lit(n.clone())).collect());
        self.body
            .push_str(&konst(table, &format!("[{ty}; {}]", cases.len()), &all));
        self.body.push_str(&format!(
            "\n/// Every case of [`{table}`], in the order of the JSON file.\npub fn {accessor}() -> Span<{ty}> {{\n    {table}.span()\n}}\n"
        ));
    }

    fn finish(mut self, all_types: &[&'static str]) -> String {
        for ty in all_types {
            if self.body.contains(&format!("{ty} {{"))
                || self.body.contains(&format!("{ty}::"))
                || self.body.contains(&format!(": {ty} "))
                || self.body.contains(&format!("[{ty};"))
            {
                self.imports.insert(ty);
            }
        }
        let imports: Vec<&str> = self.imports.iter().copied().collect();
        let use_line = if imports.len() == 1 {
            format!("use crate::types::{};\n", imports[0])
        } else {
            let flat = format!("use crate::types::{{{}}};\n", imports.join(", "));
            if flat.len() - 1 <= MAX_WIDTH {
                flat
            } else {
                // Fill layout, as `scarb fmt` does for long import lists.
                let mut out = "use crate::types::{\n".to_string();
                let mut line = "   ".to_string();
                for import in &imports {
                    if line.len() + 1 + import.len() + 1 > MAX_WIDTH {
                        out.push_str(&line);
                        out.push('\n');
                        line = "   ".to_string();
                    }
                    line.push_str(&format!(" {import},"));
                }
                out.push_str(&line);
                out.push_str("\n};\n");
                out
            }
        };
        format!(
            "// Generated by tools/golden from tools/golden/vectors/{} — do not edit.\n// Regenerate with `cargo run --release` in tools/golden.\n\n//! {}\n\n{use_line}\n{}",
            self.source, self.doc, self.body
        )
    }
}

const ALL_TYPES: [&str; 24] = [
    "AabbCase",
    "BodyKindRaw",
    "BodyStateRaw",
    "CapsuleRaw",
    "CompoundMassCase",
    "ContactPointRaw",
    "IntegrationDefaultsRaw",
    "IntegrationDerivedRaw",
    "ManifoldCase",
    "MassPropertiesRaw",
    "PoseRaw",
    "RevoluteJointRaw",
    "RotRaw",
    "SceneBodyRaw",
    "SceneCase",
    "SceneColliderRaw",
    "SceneSampleRaw",
    "SegmentRaw",
    "ShapeMassCase",
    "ShapeRaw",
    "SpringDefaultsRaw",
    "SpringDerivedRaw",
    "Vec2Raw",
    "WeightedColliderRaw",
];

fn load(vectors: &Path, file: &str) -> Value {
    let text = fs::read_to_string(vectors.join(file)).unwrap_or_else(|e| panic!("{file}: {e}"));
    serde_json::from_str(&text).unwrap_or_else(|e| panic!("{file}: {e}"))
}

fn integration_parameters(vectors: &Path) -> String {
    let json = load(vectors, "integration_parameters.json");
    let mut module = Module::new(
        "integration_parameters.json",
        "`IntegrationParameters` defaults and the solver quantities derived from them.",
    );

    let d = &json["defaults"];
    let spring = |v: &Value| {
        Node::Struct(
            "SpringDefaultsRaw",
            fields(
                v,
                &["natural_frequency", "damping_ratio", "angular_frequency"],
                raw,
            ),
        )
    };
    let mut f = vec![
        ("dt", raw(&d["dt"])),
        ("min_ccd_dt", raw(&d["min_ccd_dt"])),
        ("contact_softness", spring(&d["contact_softness"])),
        (
            "static_contact_softness",
            spring(&d["static_contact_softness"]),
        ),
        ("joint_softness", spring(&d["joint_softness"])),
    ];
    f.extend(fields(
        d,
        &[
            "warmstart_coefficient",
            "length_unit",
            "normalized_allowed_linear_error",
            "normalized_max_corrective_velocity",
            "normalized_prediction_distance",
            "normalized_max_linear_velocity",
            "normalized_contact_recycle_distance",
        ],
        raw,
    ));
    f.extend(fields(
        d,
        &[
            "num_solver_iterations",
            "num_internal_pgs_iterations",
            "num_internal_stabilization_iterations",
            "max_ccd_substeps",
        ],
        int,
    ));
    f.extend(fields(
        d,
        &[
            "contact_clustering",
            "contact_recycling",
            "friction_in_bias_pass",
            "warmstart_joints",
        ],
        boolean,
    ));
    module.body.push_str(&konst(
        "DEFAULTS",
        "IntegrationDefaultsRaw",
        &Node::Struct("IntegrationDefaultsRaw", f),
    ));
    module.body.push('\n');

    let derived_spring = |v: &Value| {
        Node::Struct(
            "SpringDerivedRaw",
            fields(v, &["erp_inv_dt", "erp", "cfm_coeff", "cfm_factor"], raw),
        )
    };
    let cases: Vec<(String, Node)> = json["cases"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| {
            let mut f = vec![
                ("id", id(c)),
                ("dt", raw(&c["dt"])),
                ("num_solver_iterations", int(&c["num_solver_iterations"])),
            ];
            f.extend(fields(
                c,
                &[
                    "inv_dt",
                    "substep_dt",
                    "substep_inv_dt",
                    "allowed_linear_error",
                    "max_corrective_velocity",
                    "prediction_distance",
                    "max_linear_velocity",
                    "contact_recycle_distance",
                ],
                raw,
            ));
            f.push(("contact", derived_spring(&c["contact"])));
            f.push(("static_contact", derived_spring(&c["static_contact"])));
            f.push(("joint", derived_spring(&c["joint"])));
            (const_name(c), Node::Struct("IntegrationDerivedRaw", f))
        })
        .collect();
    module.table("IntegrationDerivedRaw", "ALL", "cases", &cases);
    module.finish(&ALL_TYPES)
}

fn mass(vectors: &Path) -> String {
    let json = load(vectors, "mass_properties.json");
    let mut module = Module::new(
        "mass_properties.json",
        "Mass properties of single shapes and of a two-collider body.",
    );

    let shapes: Vec<(String, Node)> = json["shapes"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| {
            let node = Node::Struct(
                "ShapeMassCase",
                vec![
                    ("id", id(c)),
                    ("shape", shape(&c["shape"])),
                    ("density", raw(&c["density"])),
                    ("expected", mass_properties(&c["expected"])),
                ],
            );
            (const_name(c), node)
        })
        .collect();
    module.table("ShapeMassCase", "ALL", "cases", &shapes);
    module.body.push('\n');

    let bodies: Vec<(String, Node)> = json["bodies"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| {
            let colliders = c["colliders"].as_array().unwrap();
            assert_eq!(
                colliders.len(),
                2,
                "compound fixtures hold exactly two colliders"
            );
            let collider = |v: &Value| {
                Node::Struct(
                    "WeightedColliderRaw",
                    vec![
                        ("shape", shape(&v["shape"])),
                        ("pose_wrt_parent", pose(&v["pose_wrt_parent"])),
                        ("density", raw(&v["density"])),
                    ],
                )
            };
            let e = &c["expected"];
            let node = Node::Struct(
                "CompoundMassCase",
                vec![
                    ("id", id(c)),
                    ("body_pose", pose(&c["body_pose"])),
                    (
                        "colliders",
                        Node::Array(vec![collider(&colliders[0]), collider(&colliders[1])]),
                    ),
                    ("expected", mass_properties(&e["local"])),
                    ("world_com", vec2(&e["world_com"])),
                    ("effective_inv_mass", vec2(&e["effective_inv_mass"])),
                    (
                        "effective_world_inv_inertia",
                        raw(&e["effective_world_inv_inertia"]),
                    ),
                ],
            );
            (const_name(c), node)
        })
        .collect();
    module.table("CompoundMassCase", "ALL_BODIES", "body_cases", &bodies);
    module.finish(&ALL_TYPES)
}

fn aabb(vectors: &Path) -> String {
    let json = load(vectors, "aabb.json");
    let mut module = Module::new("aabb.json", "World-space AABBs of posed shapes.");
    let cases: Vec<(String, Node)> = json["cases"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| {
            let node = Node::Struct(
                "AabbCase",
                vec![
                    ("id", id(c)),
                    ("shape", shape(&c["shape"])),
                    ("pose", pose(&c["pose"])),
                    ("mins", vec2(&c["expected"]["mins"])),
                    ("maxs", vec2(&c["expected"]["maxs"])),
                ],
            );
            (const_name(c), node)
        })
        .collect();
    module.table("AabbCase", "ALL", "cases", &cases);
    module.finish(&ALL_TYPES)
}

fn manifolds(vectors: &Path) -> String {
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

    let cases: Vec<(String, Node)> = json["cases"]
        .as_array()
        .unwrap()
        .iter()
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
                "ManifoldCase",
                vec![
                    ("id", id(c)),
                    ("shape1", shape(&c["shape1"])),
                    ("shape2", shape(&c["shape2"])),
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
    module.table("ManifoldCase", "ALL", "cases", &cases);
    module.finish(&ALL_TYPES)
}

fn zero() -> Node {
    Node::Lit("0".into())
}

fn zero_rot() -> Node {
    Node::Struct("RotRaw", vec![("re", zero()), ("im", zero())])
}

fn zero_pose() -> Node {
    Node::Struct(
        "PoseRaw",
        vec![("translation", zero_vec2()), ("rotation", zero_rot())],
    )
}

/// Pads `items` with `pad()` up to `len`, so that unused array slots are zeroed.
fn padded(mut items: Vec<Node>, len: usize, what: &str, pad: fn() -> Node) -> Node {
    assert!(
        items.len() <= len,
        "{what}: {} items exceed capacity {len}",
        items.len()
    );
    while items.len() < len {
        items.push(pad());
    }
    Node::Array(items)
}

fn write(path: &Path, content: &str) {
    if fs::read_to_string(path).ok().as_deref() != Some(content) {
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        fs::write(path, content).unwrap();
        println!("wrote     {}", path.display());
    } else {
        println!("unchanged {}", path.display());
    }
}

pub fn generate(vectors: &Path, crate_dir: &Path) {
    let src = crate_dir.join("src");
    let files = [
        ("aabb", aabb(vectors)),
        ("aabb_overlap", leaf_families::aabb_overlap(vectors)),
        ("clip2d", leaf_families::clip2d(vectors)),
        ("contact_manifolds", manifolds(vectors)),
        ("integration_parameters", integration_parameters(vectors)),
        ("mass_properties", mass(vectors)),
        ("point_projection", leaf_families::point_projection(vectors)),
        ("pose2", leaf_families::pose2(vectors)),
        ("ray_casts", leaf_families::ray_casts(vectors)),
        ("sat2d", leaf_families::sat2d(vectors)),
        ("scenes", scenes::generate(vectors)),
        ("sleep_impact", sleep_impact::generate(vectors)),
        ("segment_segment", leaf_families::segment_segment(vectors)),
    ];
    let mut index = String::from(
        "// Generated by tools/golden — do not edit.\n// Regenerate with `cargo run --release` in tools/golden.\n\n//! Golden fixtures, one module per vector family.\n\n",
    );
    for (name, content) in &files {
        write(
            &src.join("generated").join(format!("{name}.cairo")),
            content,
        );
        index.push_str(&format!("pub mod {name};\n"));
    }
    write(&src.join("generated.cairo"), &index);

    // `scarb fmt` owns the final layout, so that `scarb fmt --check --workspace` passes.
    if std::env::var_os("GOLDEN_SKIP_FMT").is_some() {
        return;
    }
    match Command::new("scarb")
        .args(["fmt", "--package", "rapier_golden"])
        .current_dir(crate_dir)
        .status()
    {
        Ok(status) if status.success() => println!("scarb fmt: ok"),
        Ok(status) => panic!("scarb fmt failed: {status}"),
        Err(e) => {
            eprintln!("error: could not run scarb ({e}); run `scarb fmt --workspace` manually");
            std::process::exit(1);
        }
    }
}
