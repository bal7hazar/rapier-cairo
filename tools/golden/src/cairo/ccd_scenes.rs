//! Fixtures of the CC2 family `ccd_scenes` (CCD in the step).
use super::{konst, load, raw, vec2, Module, Node};
use std::path::Path;

/// Scene tuple: `(id, kind, num_solver_iterations, max_ccd_substeps, ccd_enabled)`.
const SCENE: &str = "(felt252, felt252, u32, u32, bool)";
const SCENE_DOC: &str = "(id, kind, num_solver_iterations, max_ccd_substeps, ccd_enabled)";
/// Sample tuple of one step.
const SAMPLE: &str = "(u32, i64, i64, i64, i64, bool, i64, i64, i64)";
const SAMPLE_DOC: &str = "(step, x, y, vx, vy, ccd_active, other_x, other_y, other_angle)";

fn lit(node: Node) -> String {
    match node {
        Node::Lit(s) => s,
        _ => unreachable!(),
    }
}

pub fn ccd_scenes(vectors: &Path) -> String {
    let json = load(vectors, "ccd_scenes.json");
    let mut module = Module::new(
        "ccd_scenes.json",
        "Family `ccd_scenes`: CCD in the step (CC2), see `tools/golden/src/ccd_scenes.rs`.",
    );
    let body = &mut module.body;
    body.push_str(&konst("DT", "i64", &raw(&json["dt"])));
    body.push_str(&konst("BALL_RADIUS", "i64", &raw(&json["ball_radius"])));
    body.push_str(&konst("BALL_START", "Vec2Raw", &vec2(&json["ball_start"])));
    body.push_str(&konst(
        "BALL_LINVEL",
        "Vec2Raw",
        &vec2(&json["ball_linvel"]),
    ));
    let half = |k: &str| vec2(&json[k]);
    body.push_str(&konst(
        "PLANK_HALF_EXTENTS",
        "Vec2Raw",
        &half("plank_half_extents"),
    ));
    body.push_str(&konst(
        "BOX_HALF_EXTENTS",
        "Vec2Raw",
        &half("box_half_extents"),
    ));
    body.push_str(&konst("SLOW_RADIUS", "i64", &raw(&json["slow_radius"])));
    body.push_str(&konst("SLOW_START", "Vec2Raw", &vec2(&json["slow_start"])));
    body.push_str(&konst(
        "GROUND_HALF_EXTENTS",
        "Vec2Raw",
        &half("ground_half_extents"),
    ));
    let scenes = json["scenes"].as_array().unwrap();
    let mut table = Vec::new();
    let mut arms = String::new();
    for (i, scene) in scenes.iter().enumerate() {
        let id = scene["id"].as_str().unwrap();
        table.push(Node::Lit(format!(
            "('{id}', '{}', {}, {}, {})",
            scene["kind"].as_str().unwrap(),
            scene["num_solver_iterations"],
            scene["max_ccd_substeps"],
            scene["ccd_enabled"],
        )));
        let samples: Vec<Node> = scene["samples"]
            .as_array()
            .unwrap()
            .iter()
            .map(|s| {
                let other = |k: &str| {
                    if s[k].is_null() {
                        "0".to_string()
                    } else {
                        lit(raw(&s[k]))
                    }
                };
                Node::Lit(format!(
                    "({}, {}, {}, {}, {}, {}, {}, {}, {})",
                    s["step"],
                    lit(raw(&s["x"])),
                    lit(raw(&s["y"])),
                    lit(raw(&s["vx"])),
                    lit(raw(&s["vy"])),
                    s["ccd_active"],
                    other("other_x"),
                    other("other_y"),
                    other("other_angle"),
                ))
            })
            .collect();
        let name: String = id
            .chars()
            .map(|c| {
                if c.is_ascii_alphanumeric() {
                    c.to_ascii_uppercase()
                } else {
                    '_'
                }
            })
            .collect();
        let n = samples.len();
        body.push_str(&konst(
            &name,
            &format!("[{SAMPLE}; {n}]"),
            &Node::Array(samples),
        ));
        arms.push_str(&format!("        {i} => {name}.span(),\n"));
    }
    let n = table.len();
    body.push_str(&konst(
        "SCENES",
        &format!("[{SCENE}; {n}]"),
        &Node::Array(table),
    ));
    body.push_str(&format!(
        "\n/// Every scene: `{SCENE_DOC}`.\npub fn scenes() -> Span<{SCENE}> {{\n    SCENES.span()\n}}\n\n/// The samples of scene `index`, one per step: `{SAMPLE_DOC}`.\npub fn samples(index: u32) -> Span<{SAMPLE}> {{\n    match index {{\n{arms}        _ => core::panic_with_felt252('ccd_scenes: no such scene'),\n    }}\n}}\n",
    ));
    module.finish(&["Vec2Raw"])
}
