//! Cairo emitters of the leaf-level families (`pose2`, `aabb_overlap`, `sat2d`, `clip2d`,
//! `point_projection`, `segment_segment`, `ray_casts`). Child of `cairo`: it reuses the node
//! printer, the `Module` builder and the JSON readers of its parent.

use super::{
    boolean, const_name, id, int, load, padded, pose, raw, rot, vec2, zero_vec2, Module, Node,
    ALL_TYPES,
};
use serde_json::Value;
use std::path::Path;

/// Types introduced by these families (the existing ones are in `ALL_TYPES`).
const LEAF_TYPES: [&str; 21] = [
    "AabbOverlapCase",
    "ClipCase",
    "ClipPointRaw",
    "ClipResultRaw",
    "OverlapBoxRaw",
    "OverlapPairRaw",
    "PointFeatureRaw",
    "Pose2Case",
    "ProjectionCase",
    "ProjectionRaw",
    "RayAnswerRaw",
    "RayCase",
    "RayHitRaw",
    "RotChainCase",
    "RotChainSampleRaw",
    "SatAxisRaw",
    "SatCase",
    "SatOperandRaw",
    "SegmentLocationRaw",
    "SegmentPairCase",
    "TriangleRaw",
];

fn finish(module: Module) -> String {
    let all: Vec<&'static str> = ALL_TYPES.iter().chain(LEAF_TYPES.iter()).copied().collect();
    module.finish(&all)
}

/// Capacities of the fixed-size arrays of `AabbOverlapCase` (see `types.cairo`).
const OVERLAP_MAX_BOXES: usize = 32;
const OVERLAP_MAX_PAIRS: usize = 32;

fn lit(s: &str) -> Node {
    Node::Lit(s.to_string())
}

fn zero() -> Node {
    lit("0")
}

fn vecs(v: &Value) -> Node {
    Node::Array(v.as_array().unwrap().iter().map(vec2).collect())
}

fn segment(v: &Value) -> Node {
    Node::Struct(
        "SegmentRaw",
        vec![("a", vec2(&v["a"])), ("b", vec2(&v["b"]))],
    )
}

fn cases_of(json: &Value, key: &str, f: impl Fn(&Value) -> Node) -> Vec<(String, Node)> {
    json[key]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| (const_name(c), f(c)))
        .collect()
}

// --- pose2 -----------------------------------------------------------------------------------

pub fn pose2(vectors: &Path) -> String {
    let json = load(vectors, "pose2.json");
    let mut module = Module::new(
        "pose2.json",
        "2D pose algebra (`Pose`, `Rotation`) and the drift of a chain of small-rotation products.",
    );
    let cases = cases_of(&json, "cases", |c| {
        let e = &c["expected"];
        Node::Struct(
            "Pose2Case",
            vec![
                ("id", id(c)),
                ("a", pose(&c["a"])),
                ("b", pose(&c["b"])),
                ("points", vecs(&c["points"])),
                ("mul", pose(&e["mul"])),
                ("inverse", pose(&e["inverse"])),
                ("inv_mul", pose(&e["inv_mul"])),
                ("rot_mul", rot(&e["rot_mul"])),
                ("rot_inverse", rot(&e["rot_inverse"])),
                ("transform_point", vecs(&e["transform_point"])),
                (
                    "inverse_transform_point",
                    vecs(&e["inverse_transform_point"]),
                ),
                ("transform_vector", vecs(&e["transform_vector"])),
                (
                    "inverse_transform_vector",
                    vecs(&e["inverse_transform_vector"]),
                ),
            ],
        )
    });
    module.table("Pose2Case", "ALL", "cases", &cases);
    module.body.push('\n');

    let chains = cases_of(&json, "chains", |c| {
        let samples: Vec<Node> = c["samples"]
            .as_array()
            .unwrap()
            .iter()
            .map(|s| {
                Node::Struct(
                    "RotChainSampleRaw",
                    vec![
                        ("steps", int(&s["steps"])),
                        ("rotation", rot(&s["rotation"])),
                        ("norm_squared", raw(&s["norm_squared"])),
                        ("drift", raw(&s["drift"])),
                    ],
                )
            })
            .collect();
        assert_eq!(samples.len(), 4, "chain checkpoints");
        Node::Struct(
            "RotChainCase",
            vec![
                ("id", id(c)),
                ("step", rot(&c["step"])),
                ("samples", Node::Array(samples)),
            ],
        )
    });
    module.table("RotChainCase", "ALL_CHAINS", "chain_cases", &chains);
    finish(module)
}

// --- aabb_overlap ----------------------------------------------------------------------------

fn empty_box() -> Node {
    Node::Struct(
        "OverlapBoxRaw",
        vec![
            ("mins", zero_vec2()),
            ("maxs", zero_vec2()),
            ("is_static", lit("false")),
        ],
    )
}

fn empty_pair() -> Node {
    Node::Struct(
        "OverlapPairRaw",
        vec![("i", zero()), ("j", zero()), ("both_static", lit("false"))],
    )
}

pub fn aabb_overlap(vectors: &Path) -> String {
    let json = load(vectors, "aabb_overlap.json");
    let mut module = Module::new(
        "aabb_overlap.json",
        "Overlapping pairs of lists of AABBs; touching AABBs overlap (closed convention).",
    );
    let cases = cases_of(&json, "cases", |c| {
        let boxes: Vec<Node> = c["aabbs"]
            .as_array()
            .unwrap()
            .iter()
            .map(|b| {
                Node::Struct(
                    "OverlapBoxRaw",
                    vec![
                        ("mins", vec2(&b["mins"])),
                        ("maxs", vec2(&b["maxs"])),
                        ("is_static", boolean(&b["static"])),
                    ],
                )
            })
            .collect();
        let pairs: Vec<Node> = c["expected"]["pairs"]
            .as_array()
            .unwrap()
            .iter()
            .map(|p| {
                Node::Struct(
                    "OverlapPairRaw",
                    vec![
                        ("i", int(&p["i"])),
                        ("j", int(&p["j"])),
                        ("both_static", boolean(&p["both_static"])),
                    ],
                )
            })
            .collect();
        let (num_aabbs, num_pairs) = (boxes.len(), pairs.len());
        let merged = &c["expected"]["merged"];
        Node::Struct(
            "AabbOverlapCase",
            vec![
                ("id", id(c)),
                ("num_aabbs", Node::Lit(num_aabbs.to_string())),
                (
                    "aabbs",
                    padded(boxes, OVERLAP_MAX_BOXES, "aabbs", empty_box),
                ),
                ("num_pairs", Node::Lit(num_pairs.to_string())),
                (
                    "pairs",
                    padded(pairs, OVERLAP_MAX_PAIRS, "pairs", empty_pair),
                ),
                ("merged_mins", vec2(&merged["mins"])),
                ("merged_maxs", vec2(&merged["maxs"])),
            ],
        )
    });
    module.table("AabbOverlapCase", "ALL", "cases", &cases);
    finish(module)
}

// --- sat2d -----------------------------------------------------------------------------------

fn sat_shape(v: &Value) -> Node {
    match v["type"].as_str().unwrap() {
        "cuboid" => Node::Variant("SatOperandRaw::Cuboid", Box::new(vec2(&v["half_extents"]))),
        "segment" => Node::Variant("SatOperandRaw::Segment", Box::new(segment(v))),
        "triangle" => Node::Variant(
            "SatOperandRaw::Triangle",
            Box::new(Node::Struct(
                "TriangleRaw",
                vec![
                    ("a", vec2(&v["a"])),
                    ("b", vec2(&v["b"])),
                    ("c", vec2(&v["c"])),
                ],
            )),
        ),
        other => panic!("unknown SAT shape {other}"),
    }
}

fn sat_axis(v: &Value) -> Node {
    Node::Struct(
        "SatAxisRaw",
        vec![
            ("separation", raw(&v["separation"])),
            ("axis", vec2(&v["axis"])),
        ],
    )
}

pub fn sat2d(vectors: &Path) -> String {
    let json = load(vectors, "sat2d.json");
    let mut module = Module::new(
        "sat2d.json",
        "Separating-axis helpers of Parry (cuboid against cuboid, segment and triangle), both ways.",
    );
    let cases = cases_of(&json, "cases", |c| {
        let e = &c["expected"];
        Node::Struct(
            "SatCase",
            vec![
                ("id", id(c)),
                ("shape1", sat_shape(&c["shape1"])),
                ("shape2", sat_shape(&c["shape2"])),
                ("pos12", pose(&c["pos12"])),
                ("pos21", pose(&c["pos21"])),
                ("ambiguous", boolean(&c["ambiguous"])),
                ("sep1", sat_axis(&e["sep1"])),
                ("sep2", sat_axis(&e["sep2"])),
            ],
        )
    });
    module.table("SatCase", "ALL", "cases", &cases);
    finish(module)
}

// --- clip2d ----------------------------------------------------------------------------------

fn empty_clip_point() -> Node {
    Node::Struct(
        "ClipPointRaw",
        vec![
            ("p1", zero_vec2()),
            ("p2", zero_vec2()),
            ("f1", zero()),
            ("f2", zero()),
        ],
    )
}

fn clip_result(v: &Value) -> Node {
    let points = match v.get("points") {
        None => vec![empty_clip_point(), empty_clip_point()],
        Some(points) => points
            .as_array()
            .unwrap()
            .iter()
            .map(|p| {
                Node::Struct(
                    "ClipPointRaw",
                    vec![
                        ("p1", vec2(&p["p1"])),
                        ("p2", vec2(&p["p2"])),
                        ("f1", int(&p["f1"])),
                        ("f2", int(&p["f2"])),
                    ],
                )
            })
            .collect(),
    };
    assert_eq!(points.len(), 2, "a clip yields two points");
    Node::Struct(
        "ClipResultRaw",
        vec![
            ("clipped", boolean(&Value::Bool(!v.is_null()))),
            ("points", Node::Array(points)),
        ],
    )
}

pub fn clip2d(vectors: &Path) -> String {
    let json = load(vectors, "clip2d.json");
    let mut module = Module::new(
        "clip2d.json",
        "Segment-against-segment clipping, as used on the reference and incident edges of a manifold.",
    );
    let cases = cases_of(&json, "cases", |c| {
        let e = &c["expected"];
        Node::Struct(
            "ClipCase",
            vec![
                ("id", id(c)),
                ("seg1", segment(&c["seg1"])),
                ("seg2", segment(&c["seg2"])),
                ("normal", vec2(&c["normal"])),
                ("plain", clip_result(&e["plain"])),
                ("with_normal", clip_result(&e["with_normal"])),
            ],
        )
    });
    module.table("ClipCase", "ALL", "cases", &cases);
    finish(module)
}

// --- point_projection & segment_segment -----------------------------------------------------

pub(super) fn location(v: &Value) -> Node {
    match v["kind"].as_str().unwrap() {
        "none" => lit("SegmentLocationRaw::NoLocation"),
        "vertex" => Node::Variant("SegmentLocationRaw::OnVertex", Box::new(int(&v["vertex"]))),
        "edge" => Node::Variant("SegmentLocationRaw::OnEdge", Box::new(raw(&v["u"]))),
        other => panic!("unknown location kind {other}"),
    }
}

pub(super) fn projection(v: &Value) -> Node {
    Node::Struct(
        "ProjectionRaw",
        vec![
            ("point", vec2(&v["point"])),
            ("is_inside", boolean(&v["is_inside"])),
        ],
    )
}

pub(super) fn feature(v: &Value) -> Node {
    match v["kind"].as_str().unwrap() {
        "unknown" => lit("PointFeatureRaw::Unknown"),
        "vertex" => Node::Variant("PointFeatureRaw::Vertex", Box::new(int(&v["code"]))),
        "face" => Node::Variant("PointFeatureRaw::Face", Box::new(int(&v["code"]))),
        other => panic!("unknown feature kind {other}"),
    }
}

pub fn point_projection(vectors: &Path) -> String {
    let json = load(vectors, "point_projection.json");
    let mut module = Module::new(
        "point_projection.json",
        "Projection of points on a ball, cuboid, capsule and segment (local frame).",
    );
    let cases = cases_of(&json, "cases", |c| {
        let e = &c["expected"];
        Node::Struct(
            "ProjectionCase",
            vec![
                ("id", id(c)),
                ("shape", super::shape(&c["shape"])),
                ("point", vec2(&c["point"])),
                ("projection", projection(&e["projection"])),
                ("projection_solid", projection(&e["projection_solid"])),
                ("distance", raw(&e["distance"])),
                ("feature", feature(&e["feature"])),
                ("location", location(&e["location"])),
            ],
        )
    });
    module.table("ProjectionCase", "ALL", "cases", &cases);
    finish(module)
}

pub fn segment_segment(vectors: &Path) -> String {
    let json = load(vectors, "segment_segment.json");
    let mut module = Module::new(
        "segment_segment.json",
        "Closest points between two segments, with their locations and the squared distance.",
    );
    let cases = cases_of(&json, "cases", |c| {
        let e = &c["expected"];
        Node::Struct(
            "SegmentPairCase",
            vec![
                ("id", id(c)),
                ("seg1", segment(&c["seg1"])),
                ("seg2", segment(&c["seg2"])),
                ("pos12", pose(&c["pos12"])),
                ("ambiguous", boolean(&c["ambiguous"])),
                ("loc1", location(&e["loc1"])),
                ("loc2", location(&e["loc2"])),
                ("p1", vec2(&e["p1"])),
                ("p2", vec2(&e["p2"])),
                ("dist_sq", raw(&e["dist_sq"])),
            ],
        )
    });
    module.table("SegmentPairCase", "ALL", "cases", &cases);
    finish(module)
}

// --- ray_casts -------------------------------------------------------------------------------

fn ray_hit(v: &Value) -> Node {
    if v.is_null() {
        return Node::Struct(
            "RayHitRaw",
            vec![
                ("hit", lit("false")),
                ("time_of_impact", zero()),
                ("normal", zero_vec2()),
                ("feature", lit("PointFeatureRaw::Unknown")),
            ],
        );
    }
    Node::Struct(
        "RayHitRaw",
        vec![
            ("hit", lit("true")),
            ("time_of_impact", raw(&v["time_of_impact"])),
            ("normal", vec2(&v["normal"])),
            ("feature", feature(&v["feature"])),
        ],
    )
}

pub(super) fn ray_answer(v: &Value) -> Node {
    let toi = &v["toi"];
    Node::Struct(
        "RayAnswerRaw",
        vec![
            ("has_toi", lit(if toi.is_null() { "false" } else { "true" })),
            ("toi", if toi.is_null() { zero() } else { raw(toi) }),
            ("hit", ray_hit(&v["hit"])),
        ],
    )
}

pub fn ray_casts(vectors: &Path) -> String {
    let json = load(vectors, "ray_casts.json");
    let mut module = Module::new(
        "ray_casts.json",
        "World-space ray casts (`cast_ray`, `cast_ray_and_get_normal`) on the five shapes, solid and hollow.",
    );
    let cases = cases_of(&json, "cases", |c| {
        let e = &c["expected"];
        Node::Struct(
            "RayCase",
            vec![
                ("id", id(c)),
                ("shape", super::shape(&c["shape"])),
                ("pose", pose(&c["pose"])),
                ("origin", vec2(&c["origin"])),
                ("dir", vec2(&c["dir"])),
                ("max_toi", raw(&c["max_toi"])),
                ("solid", ray_answer(&e["solid"])),
                ("hollow", ray_answer(&e["hollow"])),
            ],
        )
    });
    module.table("RayCase", "ALL", "cases", &cases);
    finish(module)
}
