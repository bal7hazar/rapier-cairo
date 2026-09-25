//! Fixtures of the G0 family `level_scenes`: `level_scenes.cairo` describes the levels (bodies in
//! insertion order, pebble launch velocity, despawn bounds, calm rule), one child module per run
//! (`level_scenes/<level>_<config>.cairo`) holds its settings, per-tick awake counts, removals
//! and sampled states.
use super::polygons::contact_shape;
use super::{konst, load, pose, raw, short_string, vec2, Module, Node};
use serde_json::Value;
use std::path::Path;

/// Sampled state tuple.
const STATE: &str = "(u32, u32, bool, i64, i64, i64, i64, i64, i64, i64)";
const STATE_DOC: &str = "(tick, body, sleeping, x, y, re, im, vx, vy, angvel)";

fn opt_tick(v: &Value) -> String {
    v.as_u64().map_or("0".into(), |t| t.to_string())
}

fn run_module(run: &Value) -> (&'static str, String) {
    let level = run["level"].as_str().unwrap();
    let config = run["config"].as_str().unwrap();
    let name = format!("{level}_{config}");
    let mut module = Module::new(
        "level_scenes.json",
        &format!("Run `{config}` of `{level}` (G0): settings, awake counts, removals, samples."),
    );
    let lit = |v: &Value| Node::Lit(v.to_string());
    let body = &mut module.body;
    body.push_str(&konst("DT", "i64", &raw(&run["dt"])));
    body.push_str(&konst(
        "NUM_SOLVER_ITERATIONS",
        "u32",
        &lit(&run["num_solver_iterations"]),
    ));
    body.push_str(&konst("NUM_TICKS", "u32", &lit(&run["num_ticks"])));
    body.push_str("/// First tick after which the pebble has an active contact (0: none).\n");
    body.push_str(&konst(
        "FIRST_PEBBLE_CONTACT",
        "u32",
        &Node::Lit(opt_tick(&run["first_pebble_contact"])),
    ));
    body.push_str("/// First tick after which every remaining dynamic body sleeps (0: none).\n");
    body.push_str(&konst(
        "ALL_ASLEEP",
        "u32",
        &Node::Lit(opt_tick(&run["all_asleep"])),
    ));
    body.push_str("/// Tick at which the calm rule ends the shot upstream (0: never).\n");
    body.push_str(&konst(
        "CALM_END",
        "u32",
        &Node::Lit(opt_tick(&run["calm_end"])),
    ));
    let removals: Vec<Node> = run["removals"]
        .as_array()
        .unwrap()
        .iter()
        .map(|r| Node::Lit(format!("({}, {})", r["tick"], r["body"])))
        .collect();
    let n = removals.len();
    body.push_str("/// Out-of-bounds removals `(tick, body index)`, in order.\n");
    body.push_str(&konst(
        "REMOVALS",
        &format!("[(u32, u32); {n}]"),
        &Node::Array(removals),
    ));
    let awake: Vec<Node> = run["awake"].as_array().unwrap().iter().map(lit).collect();
    let n = awake.len();
    body.push_str("/// Awake dynamic bodies after each tick (index 0 = tick 1).\n");
    body.push_str(&konst("AWAKE", &format!("[u8; {n}]"), &Node::Array(awake)));
    let mut states = Vec::new();
    for sample in run["samples"].as_array().unwrap() {
        for (i, s) in sample["states"].as_array().unwrap().iter().enumerate() {
            if s.is_null() {
                continue;
            }
            let r = |v: &Value, k: usize| v["raw"][k].as_i64().unwrap();
            states.push(Node::Lit(format!(
                "({}, {}, {}, {}, {}, {}, {}, {}, {}, {})",
                sample["tick"],
                i + 1,
                s["sleeping"],
                r(&s["translation"], 0),
                r(&s["translation"], 1),
                r(&s["rotation"], 0),
                r(&s["rotation"], 1),
                r(&s["linvel"], 0),
                r(&s["linvel"], 1),
                s["angvel"]["raw"].as_i64().unwrap(),
            )));
        }
    }
    let n = states.len();
    body.push_str(&format!(
        "/// Sampled states `{STATE_DOC}` by tick then body (removed bodies omitted).\n"
    ));
    body.push_str(&konst(
        "SAMPLES",
        &format!("[{STATE}; {n}]"),
        &Node::Array(states),
    ));
    body.push_str(&format!(
        "\n/// Awake counts, one per tick.\npub fn awake() -> Span<u8> {{\n    AWAKE.span()\n}}\n\n/// Removals `(tick, body)`.\npub fn removals() -> Span<(u32, u32)> {{\n    REMOVALS.span()\n}}\n\n/// Sampled states `{STATE_DOC}`.\npub fn samples() -> Span<{STATE}> {{\n    SAMPLES.span()\n}}\n",
    ));
    (
        Box::leak(format!("level_scenes/{name}").into_boxed_str()),
        module.finish(&[]),
    )
}

pub fn generate(vectors: &Path) -> Vec<(&'static str, String)> {
    let json = load(vectors, "level_scenes.json");
    let mut module = Module::new(
        "level_scenes.json",
        "G0 level scenes: bodies in insertion order (ground, blocks, cores, pebble), pebble launch, despawn bounds and calm rule; one child module per run.",
    );
    let body = &mut module.body;
    body.push_str(&konst("GRAVITY", "Vec2Raw", &vec2(&json["gravity"])));
    let calm = &json["calm_rule"];
    let q = |x: f64| Node::Lit(crate::q::Q::snap(x).0.to_string());
    body.push_str("/// Calm rule: speed thresholds (raw) and consecutive ticks.\n");
    body.push_str(&konst(
        "CALM_LINEAR",
        "i64",
        &q(calm["linear"].as_f64().unwrap()),
    ));
    body.push_str(&konst(
        "CALM_ANGULAR",
        "i64",
        &q(calm["angular"].as_f64().unwrap()),
    ));
    body.push_str(&konst(
        "CALM_TICKS",
        "u32",
        &Node::Lit(calm["ticks"].to_string()),
    ));
    let mut runs = Vec::new();
    for level in json["levels"].as_array().unwrap() {
        let id = level["id"].as_str().unwrap().to_uppercase();
        let bodies: Vec<Node> = level["bodies"]
            .as_array()
            .unwrap()
            .iter()
            .map(|b| {
                Node::Struct(
                    "LevelBodyRaw",
                    vec![
                        ("role", short_string(b["role"].as_str().unwrap())),
                        ("shape", contact_shape(&b["shape"])),
                        ("pose", pose(&b["pose"])),
                        ("density", raw(&b["density"])),
                        ("friction", raw(&b["friction"])),
                        ("restitution", raw(&b["restitution"])),
                    ],
                )
            })
            .collect();
        let n = bodies.len();
        body.push_str(&format!(
            "\n/// `{}`: {}.\n",
            level["id"].as_str().unwrap(),
            level["note"].as_str().unwrap()
        ));
        body.push_str(&konst(
            &format!("{id}_BODIES"),
            &format!("[LevelBodyRaw; {n}]"),
            &Node::Array(bodies),
        ));
        body.push_str(&konst(
            &format!("{id}_PEBBLE_LINVEL"),
            "Vec2Raw",
            &vec2(&level["pebble_linvel"]),
        ));
    }
    for run in json["runs"].as_array().unwrap() {
        let (name, content) = run_module(run);
        let bounds = &run["bounds_x"];
        let level = run["level"].as_str().unwrap().to_uppercase();
        if run["config"] == "hz60_sub4" {
            body.push_str(&format!(
                "/// Despawn bounds `(x_min, x_max)` of `{}`.\n",
                level.to_lowercase()
            ));
            body.push_str(&konst(
                &format!("{level}_BOUNDS_X"),
                "(i64, i64)",
                &Node::Lit(format!("({}, {})", bounds[0]["raw"], bounds[1]["raw"])),
            ));
        }
        runs.push((name, content));
    }
    body.push('\n');
    for (name, _) in &runs {
        body.push_str(&format!(
            "pub mod {};\n",
            name.trim_start_matches("level_scenes/")
        ));
    }
    let mut files = vec![(
        "level_scenes",
        module.finish(&[
            "LevelBodyRaw",
            "PolygonContactShapeRaw",
            "ConvexPolygonRaw",
            "ShapeRaw",
            "Vec2Raw",
            "PoseRaw",
            "RotRaw",
        ]),
    )];
    files.extend(runs);
    files
}
