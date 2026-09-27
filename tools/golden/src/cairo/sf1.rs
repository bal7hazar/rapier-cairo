//! SF1 fixtures: the scenes of `tilted_landing` (tuples, as `compound_scenes`), the parts of each
//! scene's body as a `CompoundRaw` (plain colliders on one body, not a compound shape).
use super::sh2b::compounds_module;
use super::{konst, load, raw, vec2, Module, Node};
use std::path::Path;

const TYPES: [&str; 6] = ["CompoundRaw", "PoseRaw", "RotRaw", "ShapeRaw", "Sh1ShapeRaw", "Vec2Raw"];

pub fn tilted_landing(vectors: &Path) -> String {
    let json = load(vectors, "tilted_landing.json");
    let mut module = Module::new(
        "tilted_landing.json",
        "Family `tilted_landing`: an L made of two plain cuboid colliders on one body, dropped flat on the upward half-space (SF1).",
    );
    let body = &mut module.body;
    body.push_str(&konst("DT", "i64", &raw(&json["dt"])));
    body.push_str(&konst("GRAVITY", "Vec2Raw", &vec2(&json["gravity"])));
    let scenes = json["scenes"].as_array().unwrap();
    let parts: serde_json::Value = serde_json::Value::Array(
        scenes.iter().map(|s| serde_json::json!({ "shape": { "parts": s["parts"].clone() } })).collect(),
    );
    compounds_module(&parts, "BODY", "parts", body);
    let lit_of = |n: Node| match n {
        Node::Lit(s) => s,
        _ => unreachable!(),
    };
    let mut table = Vec::new();
    let mut sample_arms = String::new();
    let mut event_arms = String::new();
    for (i, s) in scenes.iter().enumerate() {
        table.push(Node::Lit(format!("('{}', {})", s["id"].as_str().unwrap(), vec2(&s["start"]).flat())));
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
        let n = events.len();
        body.push_str(&konst(&format!("EVENTS{i}"), &format!("[(u32, bool, u32, u32); {n}]"), &Node::Array(events)));
        event_arms.push_str(&format!("        {i} => EVENTS{i}.span(),\n"));
    }
    let n = table.len();
    body.push_str(&konst("SCENES", &format!("[(felt252, Vec2Raw); {n}]"), &Node::Array(table)));
    body.push_str(&format!(
        "\n/// Every scene: `(id, start)`; the ground is the upward half-space, the body's colliders are\n/// `parts(index)`.\npub fn scenes() -> Span<(felt252, Vec2Raw)> {{\n    SCENES.span()\n}}\n\n/// The samples of scene `index`, one per step: `(step, x, y, re, im, vx, vy, w, manifolds with solver contacts)`.\npub fn samples(index: u32) -> Span<(u32, i64, i64, i64, i64, i64, i64, i64, u32)> {{\n    match index {{\n{sample_arms}        _ => core::panic_with_felt252('tilted_landing: no scene'),\n    }}\n}}\n\n/// The collision events of scene `index`: `(step, started, collider1, collider2)`, colliders 0\n/// (ground), then 1, 2 (the body's parts).\npub fn events(index: u32) -> Span<(u32, bool, u32, u32)> {{\n    match index {{\n{event_arms}        _ => core::panic_with_felt252('tilted_landing: no scene'),\n    }}\n}}\n",
    ));
    module.finish(&TYPES)
}
