//! SI diagnostics only; existing scene fixtures stay byte-identical.
use super::*;

pub(super) fn generate(vectors: &Path) -> String {
    let json = load(vectors, "scenes.json");
    let scene = json["scenes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|s| s["id"] == "ball_drop_sleep")
        .unwrap();
    let mut module = Module::new("scenes.json", "SI: sleeping-ball impact diagnostics.");
    for prewake in [false, true] {
        let cases = scene["impact_diagnostics"][if prewake { "prewake_steps" } else { "steps" }]
            .as_array()
            .unwrap()
            .iter()
            .filter(|s| {
                if prewake {
                    [87, 88, 90, 110, 120].iter().any(|n| s["step"] == *n)
                } else {
                    s["step"].as_u64().unwrap() <= 95 || s["step"] == 110 || s["step"] == 120
                }
            })
            .map(|s| {
                let bodies = (0..2)
                    .map(|i| {
                        Node::Struct(
                            "SleepImpactBodyRaw",
                            vec![
                                ("y", raw_component(&s["bodies"][i]["translation"], 1)),
                                ("vy", raw_component(&s["bodies"][i]["linvel"], 1)),
                                ("timer", raw(&s["activation"][i]["timer"])),
                                (
                                    "sleeping",
                                    Node::Lit(s["activation"][i]["sleeping"].to_string()),
                                ),
                            ],
                        )
                    })
                    .collect();
                let pairs = [(0, 1), (1, 2)]
                    .iter()
                    .map(|&(a, b)| {
                        let pair = s["pairs"].as_array().unwrap().iter().find(|p| {
                            (p["collider1"] == a && p["collider2"] == b)
                                || (p["collider1"] == b && p["collider2"] == a)
                        });
                        let m = pair.and_then(|p| p["manifolds"].as_array().unwrap().first());
                        let m = m.filter(|m| !m["solver_contacts"].as_array().unwrap().is_empty());
                        Node::Struct(
                            "SleepImpactPairRaw",
                            vec![
                                ("present", Node::Lit(m.is_some().to_string())),
                                (
                                    "dist",
                                    m.map_or_else(zero, |m| raw(&m["solver_contacts"][0]["dist"])),
                                ),
                                (
                                    "contact_id",
                                    m.map_or_else(zero, |m| int(&m["solver_contact_ids"][0])),
                                ),
                                (
                                    "impulse",
                                    m.map_or_else(zero, |m| raw(&m["points"][0]["impulse"])),
                                ),
                                (
                                    "warmstart",
                                    m.map_or_else(zero, |m| {
                                        raw(&m["points"][0]["warmstart_impulse"])
                                    }),
                                ),
                            ],
                        )
                    })
                    .collect();
                (
                    format!(
                        "{}STEP_{}",
                        if prewake { "PREWAKE_" } else { "" },
                        s["step"]
                    ),
                    Node::Struct(
                        "SleepImpactRaw",
                        vec![
                            ("step", int(&s["step"])),
                            ("bodies", Node::Array(bodies)),
                            ("pairs", Node::Array(pairs)),
                        ],
                    ),
                )
            })
            .collect::<Vec<_>>();
        module.table(
            "SleepImpactRaw",
            if prewake { "PREWAKE_ALL" } else { "ALL" },
            if prewake { "prewake_cases" } else { "cases" },
            &cases,
        );
    }
    module.finish(&["SleepImpactBodyRaw", "SleepImpactPairRaw", "SleepImpactRaw"])
}

fn raw_component(v: &Value, i: usize) -> Node {
    Node::Lit(v["raw"][i].as_i64().unwrap().to_string())
}
