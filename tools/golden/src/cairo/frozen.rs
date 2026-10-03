//! Cairo emitter of the frozen parry 0.30.2 copy (`vectors/frozen/parry_0_30_2.json`, OB): the
//! fields that parry 0.31.1 moved and that the port keeps at their 0.30.2 value. The JSON was
//! written once from the 0.30.2 vectors and is never regenerated; this emitter only transcribes
//! it, as tables of `(id, ...)` tuples plus the lookups the golden tests call. Child of `cairo`.

use super::leaf_families::feature;
use super::{konst, load, raw, short_string, vec2, Module, Node, ALL_TYPES};
use serde_json::Value;
use std::path::Path;

const FILE: &str = "frozen/parry_0_30_2.json";

/// The entries of `family` / `field`, in the order of the JSON file.
fn entries<'a>(values: &'a [Value], family: &str, field: &str) -> Vec<&'a Value> {
    values
        .iter()
        .filter(|v| v["family"] == family && v["field"] == field)
        .collect()
}

/// One `(id[, solid][, value])` tuple.
fn tuple(v: &Value, value: Option<Node>) -> Node {
    let mut parts = vec![short_string(v["id"].as_str().unwrap()).flat()];
    if let Some(solid) = v.get("solid") {
        parts.push(solid.to_string());
    }
    if let Some(value) = value {
        parts.push(value.flat());
    }
    Node::Lit(format!("({})", parts.join(", ")))
}

/// Emits `pub const <name>: [<ty>; n]` from the entries of `family` / `field`.
fn table(
    module: &mut Module,
    values: &[Value],
    (family, field): (&str, &str),
    (name, ty): (&str, &str),
    value: fn(&Value) -> Option<Node>,
) {
    let rows = entries(values, family, field);
    let items = rows.iter().map(|v| tuple(v, value(&v["value"]))).collect();
    module.body.push_str(&format!(
        "/// `{family}` / `{field}`: the 0.30.2 value of every case whose value moved.\n"
    ));
    module.body.push_str(&konst(
        name,
        &format!("[{ty}; {}]", rows.len()),
        &Node::Array(items),
    ));
    module.body.push('\n');
}

/// A miss: both answers `null` in 0.30.2 (the only `answer` entries), so the tuple holds no value.
fn miss(v: &Value) -> Option<Node> {
    assert!(v["toi"].is_null() && v["hit"].is_null(), "a frozen answer is a miss");
    None
}

const LOOKUPS: &str = r#"fn point_feature(
    table: Span<(felt252, PointFeatureRaw)>, id: felt252, current: PointFeatureRaw,
) -> PointFeatureRaw {
    for entry in table {
        let (k, f) = *entry;
        if k == id {
            return f;
        }
    }
    current
}

fn ray_feature(
    table: Span<(felt252, bool, PointFeatureRaw)>,
    id: felt252,
    solid: bool,
    current: PointFeatureRaw,
) -> PointFeatureRaw {
    for entry in table {
        let (k, s, f) = *entry;
        if k == id && s == solid {
            return f;
        }
    }
    current
}

/// The expected feature of `composite_points` case `id`: the frozen 0.30.2 one if it moved,
/// else `current` (the 0.31.1 one, which equals it).
pub fn composite_point_feature(id: felt252, current: PointFeatureRaw) -> PointFeatureRaw {
    point_feature(COMPOSITE_POINT_FEATURES.span(), id, current)
}

/// The expected hit feature of `composite_rays` case `id`, as [`composite_point_feature`].
pub fn composite_ray_feature(id: felt252, solid: bool, current: PointFeatureRaw) -> PointFeatureRaw {
    ray_feature(COMPOSITE_RAY_FEATURES.span(), id, solid, current)
}

/// The expected feature of `compound_points` case `id`, as [`composite_point_feature`].
pub fn compound_point_feature(id: felt252, current: PointFeatureRaw) -> PointFeatureRaw {
    point_feature(COMPOUND_POINT_FEATURES.span(), id, current)
}

/// The expected answer of `ray_casts` case `id`: `current` (0.31.1) with the frozen feature,
/// normal and miss of 0.30.2 laid over it. Times of impact are not laid over: the port follows
/// 0.31.1's (see [`ray_cast_toi`]).
pub fn ray_cast_answer(id: felt252, solid: bool, current: RayAnswerRaw) -> RayAnswerRaw {
    for entry in RAY_CAST_MISSES.span() {
        let (k, s) = *entry;
        if k == id && s == solid {
            let zero = Vec2Raw { x: 0, y: 0 };
            let hit = RayHitRaw {
                hit: false, time_of_impact: 0, normal: zero, feature: PointFeatureRaw::Unknown,
            };
            return RayAnswerRaw { has_toi: false, toi: 0, hit };
        }
    }
    let mut hit = current.hit;
    hit.feature = ray_feature(RAY_CAST_FEATURES.span(), id, solid, hit.feature);
    for entry in RAY_CAST_NORMALS.span() {
        let (k, s, n) = *entry;
        if k == id && s == solid {
            hit.normal = n;
        }
    }
    RayAnswerRaw { has_toi: current.has_toi, toi: current.toi, hit }
}

/// The 0.30.2 time of impact of `ray_casts` case `id`, if it moved.
pub fn ray_cast_toi(id: felt252, solid: bool) -> Option<i64> {
    for entry in RAY_CAST_TOIS.span() {
        let (k, s, t) = *entry;
        if k == id && s == solid {
            return Some(t);
        }
    }
    None
}
"#;

pub fn generate(vectors: &Path) -> String {
    let json = load(vectors, FILE);
    assert_eq!(json["parry"], "parry2d-f64 0.30.2", "the frozen copy is of parry 0.30.2");
    let values = json["values"].as_array().unwrap();
    let mut module = Module::new(
        FILE,
        "Frozen parry 0.30.2 values: the fields parry 0.31.1 moved that the port keeps at their \
         0.30.2 value (`tools/golden/README.md`, \"Frozen parry 0.30.2 values\"). The golden tests \
         compare these fields with this copy and every other field with the 0.31.1 fixtures.",
    );
    let feature_table = [
        (("composite_points", "feature"), ("COMPOSITE_POINT_FEATURES", "(felt252, PointFeatureRaw)")),
        (("composite_rays", "feature"), ("COMPOSITE_RAY_FEATURES", "(felt252, bool, PointFeatureRaw)")),
        (("compound_points", "feature"), ("COMPOUND_POINT_FEATURES", "(felt252, PointFeatureRaw)")),
        (("ray_casts", "feature"), ("RAY_CAST_FEATURES", "(felt252, bool, PointFeatureRaw)")),
    ];
    for (key, konst) in feature_table {
        table(&mut module, values, key, konst, |v| Some(feature(v)));
    }
    table(
        &mut module,
        values,
        ("ray_casts", "normal"),
        ("RAY_CAST_NORMALS", "(felt252, bool, Vec2Raw)"),
        |v| Some(vec2(v)),
    );
    table(
        &mut module,
        values,
        ("ray_casts", "toi"),
        ("RAY_CAST_TOIS", "(felt252, bool, i64)"),
        |v| Some(raw(v)),
    );
    table(
        &mut module,
        values,
        ("ray_casts", "answer"),
        ("RAY_CAST_MISSES", "(felt252, bool)"),
        miss,
    );
    let known = [
        "composite_points/feature",
        "composite_rays/feature",
        "compound_points/feature",
        "ray_casts/feature",
        "ray_casts/normal",
        "ray_casts/toi",
        "ray_casts/answer",
    ];
    for v in values {
        let key = format!("{}/{}", v["family"].as_str().unwrap(), v["field"].as_str().unwrap());
        assert!(known.contains(&key.as_str()), "frozen entry {key} has no table");
    }
    module.body.push_str(LOOKUPS);
    let types = ["PointFeatureRaw", "RayAnswerRaw", "RayHitRaw"];
    let all: Vec<&'static str> = ALL_TYPES.iter().chain(types.iter()).copied().collect();
    module.finish(&all)
}
