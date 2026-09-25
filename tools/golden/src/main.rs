//! Golden-vector harness for rapier-cairo. See `README.md`.
//!
//! `cargo run --release` regenerates `vectors/*.json` and the Cairo fixtures under
//! `crates/rapier_golden/src/generated/`. The output is a pure function of the pinned upstream
//! crates: no clock, no RNG, no hash-map iteration.

mod aabb;
mod aabb_overlap;
mod cairo;
mod clip2d;
mod intersection_tests;
mod jsonfmt;
mod leaf;
mod level_scenes;
mod manifolds;
mod mass;
mod params;
mod point_projection;
mod pose2;
mod q;
mod ray_casts;
mod sat2d;
mod scenes;
mod segment_segment;
mod sensor_trigger;
mod shapes;

use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};

/// Pinned upstream versions, repeated in every vector file. Must match `Cargo.toml`.
pub const RAPIER_VERSION: &str = "rapier2d-f64 0.35.3";
pub const PARRY_VERSION: &str = "parry2d-f64 0.30.2";

fn tool_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn write_if_changed(path: &Path, content: &str) {
    if fs::read_to_string(path).ok().as_deref() != Some(content) {
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        fs::write(path, content).unwrap();
        println!("wrote     {}", path.display());
    } else {
        println!("unchanged {}", path.display());
    }
}

fn with_header(mut value: Value) -> Value {
    let body = value.as_object_mut().unwrap();
    let mut out = serde_json::Map::new();
    out.insert(
        "generated_by".into(),
        "tools/golden — do not edit, run `cargo run --release` in tools/golden".into(),
    );
    out.insert("rapier".into(), RAPIER_VERSION.into());
    out.insert("parry".into(), PARRY_VERSION.into());
    out.insert(
        "encoding".into(),
        "scalar = {f64, raw}; raw = round(f64 * 2^32) as i64 (Q32.32)".into(),
    );
    out.append(body);
    Value::Object(out)
}

fn main() {
    let mode = std::env::args().nth(1).unwrap_or_else(|| "all".into());
    let vectors_dir = tool_dir().join("vectors");

    if mode == "all" || mode == "vectors" {
        type Family = (&'static str, fn() -> Value);
        let families: [Family; 15] = [
            ("integration_parameters", params::generate),
            ("mass_properties", mass::generate),
            ("aabb", aabb::generate),
            ("contact_manifolds", manifolds::generate),
            ("scenes", scenes::generate),
            ("pose2", pose2::generate),
            ("aabb_overlap", aabb_overlap::generate),
            ("sat2d", sat2d::generate),
            ("clip2d", clip2d::generate),
            ("point_projection", point_projection::generate),
            ("segment_segment", segment_segment::generate),
            ("ray_casts", ray_casts::generate),
            ("intersection_tests", intersection_tests::generate),
            ("sensor_trigger", sensor_trigger::generate),
            ("level_scenes", level_scenes::generate),
        ];
        for (name, generate) in families {
            let value = with_header(generate());
            write_if_changed(
                &vectors_dir.join(format!("{name}.json")),
                &jsonfmt::render(&value),
            );
        }
    }

    if mode == "all" || mode == "cairo" {
        let crate_dir = tool_dir().join("../../crates/rapier_golden");
        cairo::generate(&vectors_dir, &crate_dir);
    }

    if !["all", "vectors", "cairo"].contains(&mode.as_str()) {
        eprintln!("usage: golden [all|vectors|cairo]");
        std::process::exit(2);
    }
}
