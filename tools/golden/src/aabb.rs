//! Family `aabb`: `Shape::compute_aabb(&pose)` for the MVP shapes under several poses.

use crate::q::{jqpose, jvec, QPose, QRot, QVec};
use crate::shapes::ShapeSpec;
use serde_json::{json, Value};

pub fn generate() -> Value {
    let shapes = [
        ("ball", ShapeSpec::ball(0.5)),
        ("cuboid", ShapeSpec::cuboid(1.0, 0.5)),
        ("capsule", ShapeSpec::capsule_y(0.5, 0.25)),
        (
            "capsule_oblique",
            ShapeSpec::capsule((-0.5, -0.25), (1.0, 0.75), 0.3),
        ),
    ];
    let poses = [
        ("identity", QPose::new(QVec::ZERO, QRot::IDENTITY)),
        ("translated", QPose::translation(3.0, -2.0)),
        ("rot90", QPose::new(QVec::snap(-1.0, 4.0), QRot::QUARTER)),
        ("rot180", QPose::new(QVec::snap(0.5, 0.5), QRot::HALF)),
        ("rot30", QPose::new(QVec::ZERO, QRot::from_degrees(30.0))),
        (
            "rot45",
            QPose::new(QVec::snap(10.0, 10.0), QRot::from_degrees(45.0)),
        ),
        (
            "rot-135",
            QPose::new(QVec::snap(-1.5, 0.25), QRot::from_degrees(-135.0)),
        ),
        (
            "rot1",
            QPose::new(QVec::snap(100.0, -250.0), QRot::from_degrees(1.0)),
        ),
    ];

    let mut cases = Vec::new();
    for (shape_name, shape) in shapes {
        for (pose_name, pose) in poses {
            let aabb = shape.shared().compute_aabb(&pose.p());
            cases.push(json!({
                "id": format!("{shape_name}/{pose_name}"),
                "shape": shape.json(),
                "pose": jqpose(pose),
                "expected": { "mins": jvec(aabb.mins), "maxs": jvec(aabb.maxs) },
            }));
        }
    }

    json!({ "family": "aabb", "cases": cases })
}
