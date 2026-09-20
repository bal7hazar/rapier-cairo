//! Family `pose2`: 2D pose algebra of `parry2d_f64::math::{Pose, Rotation}` (glam's `Pose2` /
//! `Rot2`) on Q32.32 inputs, plus a chain of small-rotation multiplications for the drift study.
//!
//! Upstream functions: `Pose * Pose`, `Pose::inverse`, `Pose::inv_mul`, `Pose::transform_point`,
//! `Pose::inverse_transform_point`, `Pose::transform_vector`, `Pose::inverse_transform_vector`,
//! `Rotation * Rotation`, `Rotation::inverse`.

use crate::leaf::{jpose, jvecs, qv};
use crate::q::{jf, jqpose, jqrot, jrot, QPose, QRot, QVec, Q};
use rapier2d_f64::math::{Rotation, Vector};
use serde_json::{json, Value};

struct Case {
    name: &'static str,
    a: QPose,
    b: QPose,
    note: &'static str,
}

fn pose(x: f64, y: f64, rot: QRot) -> QPose {
    QPose::new(qv(x, y), rot)
}

fn deg(d: f64) -> QRot {
    QRot::from_degrees(d)
}

/// The points every pose is applied to (exactly representable).
fn points() -> [QVec; 3] {
    [qv(1.0, 0.0), qv(0.5, -2.0), qv(-3.0, 4.25)]
}

fn cases() -> Vec<Case> {
    let a = pose(-1.0, 4.0, QRot::QUARTER);
    let a_inv = {
        // The inverse of `a`, snapped to Q32.32 (rotation conjugate is exact).
        let up = a.p().inverse();
        QPose::new(
            QVec::snap(up.translation.x, up.translation.y),
            QRot {
                re: a.rotation.re,
                im: Q(-a.rotation.im.0),
            },
        )
    };
    vec![
        Case {
            name: "identity_translated",
            a: QPose::new(QVec::ZERO, QRot::IDENTITY),
            b: QPose::translation(3.0, -2.0),
            note: "identity first: every operation is the identity on the second pose",
        },
        Case {
            name: "rot90_rot30",
            a,
            b: pose(2.0, 0.5, deg(30.0)),
            note: "exact quarter turn against a generic angle",
        },
        Case {
            name: "rot45_rot-135",
            a: pose(10.0, 10.0, deg(45.0)),
            b: pose(-1.5, 0.25, deg(-135.0)),
            note: "opposite generic rotations: the relative rotation is 180 degrees",
        },
        Case {
            name: "rot180_rot1",
            a: pose(0.5, 0.5, QRot::HALF),
            b: pose(100.0, -250.0, deg(1.0)),
            note: "exact half turn, far translation with a 1 degree rotation",
        },
        Case {
            name: "far_small_rot",
            a: pose(100.0, -250.0, deg(1.0)),
            b: pose(101.0, -249.5, deg(1.5)),
            note: "large translations, tiny relative rotation: the point transform loses precision",
        },
        Case {
            name: "same_pose",
            a: pose(0.75, -0.25, deg(30.0)),
            b: pose(0.75, -0.25, deg(30.0)),
            note: "a == b: inv_mul is the identity up to the norm error of the rotation",
        },
        Case {
            name: "pose_times_inverse",
            a,
            b: a_inv,
            note: "b is the inverse of a snapped to Q32.32: mul is the identity up to 1-2 ulp",
        },
    ]
}

fn norm_sq(r: Rotation) -> f64 {
    r.re * r.re + r.im * r.im
}

fn case_json(c: &Case) -> Value {
    let (a, b) = (c.a.p(), c.b.p());
    let pts = points();
    let ps: Vec<Vector> = pts.iter().map(|p| p.v()).collect();
    let map = |f: &dyn Fn(Vector) -> Vector| -> Vec<Vector> { ps.iter().map(|p| f(*p)).collect() };

    // Does upstream renormalise anything? Compare with the plain complex product.
    let plain = Rotation {
        re: a.rotation.re * b.rotation.re - a.rotation.im * b.rotation.im,
        im: a.rotation.im * b.rotation.re + a.rotation.re * b.rotation.im,
    };
    let rot_mul = a.rotation * b.rotation;
    let plain_product = rot_mul.re == plain.re && rot_mul.im == plain.im;

    json!({
        "id": format!("pair/{}", c.name),
        "note": c.note,
        "a": jqpose(c.a),
        "b": jqpose(c.b),
        "a_rotation_used": jrot(a.rotation),
        "b_rotation_used": jrot(b.rotation),
        "points": jvecs(&ps),
        "expected": {
            "mul": jpose(a * b),
            "inverse": jpose(a.inverse()),
            "inv_mul": jpose(a.inv_mul(&b)),
            "rot_mul": jrot(rot_mul),
            "rot_mul_is_plain_complex_product": plain_product,
            "rot_inverse": jrot(a.rotation.inverse()),
            "transform_point": jvecs(&map(&|p| a.transform_point(p))),
            "inverse_transform_point": jvecs(&map(&|p| a.inverse_transform_point(p))),
            "transform_vector": jvecs(&map(&|p| a.transform_vector(p))),
            "inverse_transform_vector": jvecs(&map(&|p| a.inverse_transform_vector(p))),
        },
    })
}

/// 1 000 successive products `acc = acc * r` from the identity, sampled at a few checkpoints.
fn chain(name: &str, degrees: f64) -> Value {
    const CHECKPOINTS: [u32; 4] = [1, 10, 100, 1000];
    let step = QRot::from_degrees(degrees);
    let r = step.r();
    let mut acc = Rotation { re: 1.0, im: 0.0 };
    let mut samples = Vec::new();
    for i in 1..=1000u32 {
        acc = acc * r;
        if CHECKPOINTS.contains(&i) {
            let n = norm_sq(acc);
            samples.push(json!({
                "steps": i,
                "rotation": jrot(acc),
                "norm_squared": jf(n),
                "drift": jf(n - 1.0),
            }));
        }
    }
    let input_norm = norm_sq(r);
    json!({
        "id": format!("chain/{name}"),
        "degrees": degrees,
        "step": jqrot(step),
        "step_norm_squared_minus_one": input_norm - 1.0,
        "samples": samples,
    })
}

pub fn generate() -> Value {
    let cases: Vec<Value> = cases().iter().map(case_json).collect();
    let chains = vec![chain("deg0p1", 0.1), chain("deg1", 1.0), chain("deg5", 5.0)];
    json!({ "family": "pose2", "cases": cases, "chains": chains })
}
