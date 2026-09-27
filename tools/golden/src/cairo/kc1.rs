//! KC1 fixtures: the families `pid_corrections` and `character_moves` of `crate::kc1`, as tuple
//! tables (as `tilted_landing`).
use super::{boolean, id, konst, load, pose, raw, shape, vec2, Module, Node};
use serde_json::Value;
use std::path::Path;

const TYPES: [&str; 5] = ["CapsuleRaw", "PoseRaw", "RotRaw", "ShapeRaw", "Vec2Raw"];

fn tuple(items: Vec<Node>) -> Node {
    Node::Lit(format!(
        "({})",
        items.iter().map(Node::flat).collect::<Vec<_>>().join(", ")
    ))
}

fn lit(s: String) -> Node {
    Node::Lit(s)
}

pub fn pid_corrections(vectors: &Path) -> String {
    let json = load(vectors, "pid_corrections.json");
    let mut module = Module::new(
        "pid_corrections.json",
        "Family `pid_corrections`: PD / PID corrections of a dynamic body towards a target (KC1).",
    );
    let cases = json["cases"].as_array().unwrap();
    let inputs: Vec<Node> = cases
        .iter()
        .map(|c| {
            tuple(vec![
                id(c),
                pose(&c["pose"]),
                vec2(&c["linvel"]),
                raw(&c["angvel"]),
                vec2(&c["offset"]),
                raw(&c["kp"]),
                raw(&c["ki"]),
                raw(&c["kd"]),
                lit(c["axes"].to_string()),
                raw(&c["dt"]),
                pose(&c["target"]),
                vec2(&c["target_linvel"]),
                raw(&c["target_angvel"]),
            ])
        })
        .collect();
    let outputs: Vec<Node> = cases
        .iter()
        .map(|c| {
            tuple(vec![
                vec2(&c["local_com"]),
                vec2(&c["pd"]["linvel"]),
                raw(&c["pd"]["angvel"]),
                vec2(&c["pd_linear"]),
                raw(&c["pd_angular"]),
                vec2(&c["pid_third"]["linvel"]),
                raw(&c["pid_third"]["angvel"]),
                vec2(&c["pid_lin_integral"]),
                raw(&c["pid_ang_integral"]),
                vec2(&c["pid_linear"]),
                raw(&c["pid_angular"]),
            ])
        })
        .collect();
    let n = cases.len();
    let body = &mut module.body;
    body.push_str(&konst(
        "INPUTS",
        &format!("[(felt252, PoseRaw, Vec2Raw, i64, Vec2Raw, i64, i64, i64, u8, i64, PoseRaw, Vec2Raw, i64); {n}]"),
        &Node::Array(inputs),
    ));
    body.push_str(&konst(
        "OUTPUTS",
        &format!("[(Vec2Raw, Vec2Raw, i64, Vec2Raw, i64, Vec2Raw, i64, Vec2Raw, i64, Vec2Raw, i64); {n}]"),
        &Node::Array(outputs),
    ));
    body.push_str(
        "\n/// Every case: `(id, body pose, linvel, angvel, collider offset, kp, ki, kd, axes bits, dt,\n/// target pose, target linvel, target angvel)`.\npub fn inputs() -> Span<(felt252, PoseRaw, Vec2Raw, i64, Vec2Raw, i64, i64, i64, u8, i64, PoseRaw, Vec2Raw, i64)> {\n    INPUTS.span()\n}\n\n/// Upstream's answers, same order: `(local com, PD linvel, PD angvel, PD linear, PD angular,\n/// third PID linvel, third PID angvel, PID linear integral, PID angular integral, fresh PID\n/// linear, fresh PID angular)`.\npub fn outputs() -> Span<(Vec2Raw, Vec2Raw, i64, Vec2Raw, i64, Vec2Raw, i64, Vec2Raw, i64, Vec2Raw, i64)> {\n    OUTPUTS.span()\n}\n",
    );
    module.finish(&TYPES)
}

fn kind(s: &str) -> Node {
    lit(match s {
        "fixed" => "0",
        "kinematic" => "1",
        "dynamic" => "2",
        other => panic!("unknown body kind {other}"),
    }
    .into())
}

/// `(slide, autostep?, max height, min width, include dynamic, climb, slide angle, snap?, snap,
/// offset, nudge)`.
fn options(o: &Value) -> Node {
    let a = &o["autostep"];
    let s = &o["snap_to_ground"];
    let zero = || lit("0".into());
    tuple(vec![
        boolean(&o["slide"]),
        lit((!a.is_null()).to_string()),
        if a.is_null() { zero() } else { raw(&a["max_height"]) },
        if a.is_null() { zero() } else { raw(&a["min_width"]) },
        if a.is_null() { lit("false".into()) } else { boolean(&a["include_dynamic_bodies"]) },
        raw(&o["max_slope_climb_angle"]),
        raw(&o["min_slope_slide_angle"]),
        lit((!s.is_null()).to_string()),
        if s.is_null() { zero() } else { raw(s) },
        raw(&o["offset"]),
        raw(&o["normal_nudge_factor"]),
    ])
}

const OPTIONS_TY: &str = "(bool, bool, i64, i64, bool, i64, i64, bool, i64, i64, i64)";

pub fn character_moves(vectors: &Path) -> String {
    let json = load(vectors, "character_moves.json");
    let mut module = Module::new(
        "character_moves.json",
        "Family `character_moves`: `KinematicCharacterController::move_shape` in small scenes (KC1).",
    );
    let mut colliders = vec![];
    for (i, s) in json["scenes"].as_array().unwrap().iter().enumerate() {
        for c in s["colliders"].as_array().unwrap() {
            colliders.push(tuple(vec![
                lit(i.to_string()),
                kind(c["kind"].as_str().unwrap()),
                pose(&c["pose"]),
                vec2(&c["linvel"]),
                shape(&c["shape"]),
            ]));
        }
    }
    let cases = json["cases"].as_array().unwrap();
    let mut inputs = vec![];
    let mut results = vec![];
    let mut hits = vec![];
    let mut pushed = None;
    for (i, c) in cases.iter().enumerate() {
        inputs.push(tuple(vec![
            id(c),
            lit(c["scene"].to_string()),
            shape(&c["shape"]),
            pose(&c["pos"]),
            vec2(&c["desired"]),
            raw(&c["dt"]),
            options(&c["options"]),
        ]));
        let collisions = c["collisions"].as_array().unwrap();
        results.push(tuple(vec![
            vec2(&c["translation"]),
            boolean(&c["grounded"]),
            boolean(&c["is_sliding_down_slope"]),
            lit(collisions.len().to_string()),
        ]));
        for h in collisions {
            hits.push(tuple(vec![
                lit(i.to_string()),
                lit(h["collider"].to_string()),
                raw(&h["toi"]),
                vec2(&h["normal1"]),
                vec2(&h["witness1"]),
                vec2(&h["translation_applied"]),
                vec2(&h["translation_remaining"]),
            ]));
        }
        if !c["pushed"].is_null() {
            let p = &c["pushed"];
            pushed = Some(tuple(vec![
                lit(i.to_string()),
                raw(&p["mass"]),
                vec2(&p["linvel"]),
                raw(&p["angvel"]),
            ]));
        }
    }
    let body = &mut module.body;
    let (nc, n, nh) = (colliders.len(), inputs.len(), hits.len());
    body.push_str(&konst(
        "COLLIDERS",
        &format!("[(u32, u8, PoseRaw, Vec2Raw, ShapeRaw); {nc}]"),
        &Node::Array(colliders),
    ));
    body.push_str(&konst(
        "CASES",
        &format!("[(felt252, u32, ShapeRaw, PoseRaw, Vec2Raw, i64, {OPTIONS_TY}); {n}]"),
        &Node::Array(inputs),
    ));
    body.push_str(&konst("RESULTS", &format!("[(Vec2Raw, bool, bool, u32); {n}]"), &Node::Array(results)));
    body.push_str(&konst(
        "HITS",
        &format!("[(u32, u32, i64, Vec2Raw, Vec2Raw, Vec2Raw, Vec2Raw); {nh}]"),
        &Node::Array(hits),
    ));
    body.push_str(&konst("PUSHED", "(u32, i64, Vec2Raw, i64)", &pushed.expect("one push case")));
    body.push_str(&format!(
        "\n/// Every scene collider, in handle order per scene: `(scene, kind (0 standalone fixed, 1\n/// velocity-based kinematic body, 2 dynamic body), pose, linvel, shape)`.\npub fn colliders() -> Span<(u32, u8, PoseRaw, Vec2Raw, ShapeRaw)> {{\n    COLLIDERS.span()\n}}\n\n/// Every case: `(id, scene, character shape, pose, desired translation, dt, options)`, the\n/// options `{OPTIONS_TY}` = `(slide, autostep?, max height (absolute), min width (absolute),\n/// include dynamic bodies, max climb angle, min slide angle, snap?, snap (relative), offset\n/// (relative), normal nudge)`.\npub fn cases() -> Span<(felt252, u32, ShapeRaw, PoseRaw, Vec2Raw, i64, {OPTIONS_TY})> {{\n    CASES.span()\n}}\n\n/// Upstream's answer per case: `(translation, grounded, sliding down, collisions)`.\npub fn results() -> Span<(Vec2Raw, bool, bool, u32)> {{\n    RESULTS.span()\n}}\n\n/// Every collision event, case by case in callback order: `(case, collider index, toi, normal1,\n/// witness1, translation applied, translation remaining)` (world space).\npub fn hits() -> Span<(u32, u32, i64, Vec2Raw, Vec2Raw, Vec2Raw, Vec2Raw)> {{\n    HITS.span()\n}}\n"
    ));
    module.finish(&TYPES)
}
