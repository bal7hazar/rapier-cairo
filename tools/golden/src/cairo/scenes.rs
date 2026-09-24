use super::*;

/// Capacities of the fixed-size arrays of `SceneCase` (see `types.cairo`).
const SCENE_MAX_BODIES: usize = 4;
const SCENE_MAX_COLLIDERS: usize = 1;
const SCENE_MAX_JOINTS: usize = 1;
const SCENE_MAX_DYNAMIC: usize = 3;
const SCENE_NUM_SAMPLES: usize = 22;

fn scene_collider(v: &Value) -> Node {
    Node::Struct(
        "SceneColliderRaw",
        vec![
            ("shape", shape(&v["shape"])),
            ("pose_wrt_parent", pose(&v["pose_wrt_parent"])),
            ("density", raw(&v["density"])),
            ("friction", raw(&v["friction"])),
            ("restitution", raw(&v["restitution"])),
        ],
    )
}

fn empty_scene_collider() -> Node {
    Node::Struct(
        "SceneColliderRaw",
        vec![
            ("shape", Node::Variant("ShapeRaw::Ball", Box::new(zero()))),
            ("pose_wrt_parent", zero_pose()),
            ("density", zero()),
            ("friction", zero()),
            ("restitution", zero()),
        ],
    )
}

fn scene_body(v: &Value) -> Node {
    let colliders: Vec<Node> = v["colliders"]
        .as_array()
        .unwrap()
        .iter()
        .map(scene_collider)
        .collect();
    let num_colliders = colliders.len();
    let kind = match v["type"].as_str().unwrap() {
        "fixed" => "BodyKindRaw::Fixed",
        "dynamic" => "BodyKindRaw::Dynamic",
        other => panic!("unknown body type {other}"),
    };
    Node::Struct(
        "SceneBodyRaw",
        vec![
            ("name", short_string(v["name"].as_str().unwrap())),
            ("kind", Node::Lit(kind.into())),
            ("pose", pose(&v["pose"])),
            ("linear_damping", raw(&v["linear_damping"])),
            ("angular_damping", raw(&v["angular_damping"])),
            ("gravity_scale", raw(&v["gravity_scale"])),
            ("num_colliders", Node::Lit(num_colliders.to_string())),
            (
                "colliders",
                padded(
                    colliders,
                    SCENE_MAX_COLLIDERS,
                    "colliders",
                    empty_scene_collider,
                ),
            ),
        ],
    )
}

fn empty_scene_body() -> Node {
    Node::Struct(
        "SceneBodyRaw",
        vec![
            ("name", zero()),
            ("kind", Node::Lit("BodyKindRaw::Fixed".into())),
            ("pose", zero_pose()),
            ("linear_damping", zero()),
            ("angular_damping", zero()),
            ("gravity_scale", zero()),
            ("num_colliders", zero()),
            (
                "colliders",
                padded(
                    vec![],
                    SCENE_MAX_COLLIDERS,
                    "colliders",
                    empty_scene_collider,
                ),
            ),
        ],
    )
}

fn empty_joint() -> Node {
    Node::Struct(
        "RevoluteJointRaw",
        vec![
            ("body1", zero()),
            ("body2", zero()),
            ("local_anchor1", zero_vec2()),
            ("local_anchor2", zero_vec2()),
        ],
    )
}

fn empty_state() -> Node {
    Node::Struct(
        "BodyStateRaw",
        vec![
            ("body", zero()),
            ("translation", zero_vec2()),
            ("rotation", zero_rot()),
            ("linvel", zero_vec2()),
            ("angvel", zero()),
        ],
    )
}

pub(super) fn generate(vectors: &Path) -> String {
    let json = load(vectors, "scenes.json");
    let mut module = Module::new(
        "scenes.json",
        "Full-engine traces: description of six scenes and the sampled states of their dynamic bodies.",
    );
    let num_steps = json["num_steps"].as_u64().unwrap();

    let mut joint_cases = Vec::new();
    let cases: Vec<(String, Node)> = json["scenes"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|c| {
            let bodies = c["bodies"].as_array().unwrap();
            let index_of = |v: &Value| -> Node {
                let index = bodies
                    .iter()
                    .position(|b| b["name"] == *v)
                    .unwrap_or_else(|| panic!("unknown body {v}"));
                Node::Lit(index.to_string())
            };
            let dynamic: Vec<&str> = bodies
                .iter()
                .filter(|b| b["type"] == "dynamic")
                .map(|b| b["name"].as_str().unwrap())
                .collect();

            let controlled = c["joints"].as_array().unwrap().iter().any(|j| {
                !j["limits"].is_null() || !j["motor"].is_null() || j["type"] == "prismatic"
            });
            let mut joints: Vec<Node> = c["joints"]
                .as_array()
                .unwrap()
                .iter()
                .map(|j| {
                    assert!(j["type"] == "revolute" || j["type"] == "prismatic");
                    Node::Struct(
                        "RevoluteJointRaw",
                        vec![
                            ("body1", index_of(&j["body1"])),
                            ("body2", index_of(&j["body2"])),
                            ("local_anchor1", vec2(&j["local_anchor1"])),
                            ("local_anchor2", vec2(&j["local_anchor2"])),
                        ],
                    )
                })
                .collect();
            if controlled {
                joints.clear();
            }
            let num_joints = joints.len();

            let samples = c["samples"].as_array().unwrap();
            assert_eq!(
                samples.len(),
                SCENE_NUM_SAMPLES,
                "sample count of {}",
                c["id"]
            );
            let samples: Vec<Node> = samples
                .iter()
                .map(|s| {
                    let states = s["bodies"].as_array().unwrap();
                    let names: Vec<&str> =
                        states.iter().map(|b| b["body"].as_str().unwrap()).collect();
                    assert_eq!(
                        names, dynamic,
                        "sampled bodies of {} follow body order",
                        c["id"]
                    );
                    let states: Vec<Node> = states
                        .iter()
                        .map(|b| {
                            Node::Struct(
                                "BodyStateRaw",
                                vec![
                                    ("body", index_of(&b["body"])),
                                    ("translation", vec2(&b["translation"])),
                                    ("rotation", rot(&b["rotation"])),
                                    ("linvel", vec2(&b["linvel"])),
                                    ("angvel", raw(&b["angvel"])),
                                ],
                            )
                        })
                        .collect();
                    Node::Struct(
                        "SceneSampleRaw",
                        vec![
                            ("step", int(&s["step"])),
                            (
                                "states",
                                padded(states, SCENE_MAX_DYNAMIC, "sampled states", empty_state),
                            ),
                        ],
                    )
                })
                .collect();

            let body_nodes: Vec<Node> = bodies.iter().map(scene_body).collect();
            let node = Node::Struct(
                "SceneCase",
                vec![
                    ("id", id(c)),
                    ("gravity", vec2(&json["gravity"])),
                    ("dt", raw(&json["dt"])),
                    ("num_steps", Node::Lit(num_steps.to_string())),
                    ("num_bodies", Node::Lit(bodies.len().to_string())),
                    (
                        "bodies",
                        padded(body_nodes, SCENE_MAX_BODIES, "bodies", empty_scene_body),
                    ),
                    ("num_dynamic", Node::Lit(dynamic.len().to_string())),
                    ("num_joints", Node::Lit(num_joints.to_string())),
                    (
                        "joints",
                        padded(joints, SCENE_MAX_JOINTS, "joints", empty_joint),
                    ),
                    ("samples", Node::Array(samples)),
                ],
            );
            if controlled {
                let j = &c["joints"][0];
                assert_eq!(c["joints"].as_array().unwrap().len(), 1);
                let field = |name: &str| {
                    if j["motor"].is_null() {
                        zero()
                    } else {
                        raw(&j["motor"][name])
                    }
                };
                let control = Node::Struct(
                    "crate::types::SceneJointControlRaw",
                    vec![
                        ("body1", index_of(&j["body1"])),
                        ("body2", index_of(&j["body2"])),
                        ("local_anchor1", vec2(&j["local_anchor1"])),
                        ("local_anchor2", vec2(&j["local_anchor2"])),
                        (
                            "prismatic",
                            Node::Lit((j["type"] == "prismatic").to_string()),
                        ),
                        (
                            "axis",
                            if j["axis"].is_null() {
                                zero_vec2()
                            } else {
                                vec2(&j["axis"])
                            },
                        ),
                        (
                            "has_limits",
                            Node::Lit((!j["limits"].is_null()).to_string()),
                        ),
                        (
                            "min",
                            if j["limits"].is_null() {
                                zero()
                            } else {
                                raw(&j["limits"][0])
                            },
                        ),
                        (
                            "max",
                            if j["limits"].is_null() {
                                zero()
                            } else {
                                raw(&j["limits"][1])
                            },
                        ),
                        ("has_motor", Node::Lit((!j["motor"].is_null()).to_string())),
                        ("target_pos", field("target_pos")),
                        ("target_vel", field("target_vel")),
                        ("stiffness", field("stiffness")),
                        ("damping", field("damping")),
                        ("max_force", field("max_force")),
                        (
                            "force_based",
                            Node::Lit(
                                j["motor"]["force_based"]
                                    .as_bool()
                                    .unwrap_or(false)
                                    .to_string(),
                            ),
                        ),
                    ],
                );
                joint_cases.push((
                    const_name(c),
                    Node::Struct(
                        "crate::types::JointSceneCase",
                        vec![("scene", node), ("joint", control)],
                    ),
                ));
                None
            } else {
                Some((const_name(c), node))
            }
        })
        .collect();
    module.table("SceneCase", "ALL", "cases", &cases);
    module.table(
        "crate::types::JointSceneCase",
        "JOINT_CASES",
        "joint_cases",
        &joint_cases,
    );
    module.finish(&ALL_TYPES)
}
