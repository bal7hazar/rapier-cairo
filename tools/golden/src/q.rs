//! Q32.32 quantisation helpers and JSON encoders.
//!
//! Every *input* handed to Rapier/Parry is built from a raw `i64` (`value = raw / 2^32`), so the
//! Cairo port can be fed bit-identical inputs. Every *output* is emitted both as the upstream
//! `f64` and as the nearest raw `i64`.

use rapier2d_f64::math::{Pose, Rotation, Vector};
use serde_json::{json, Value};

/// 2^32 as an `f64`.
pub const SCALE: f64 = 4_294_967_296.0;

/// A Q32.32 number, stored as its raw `i64`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Q(pub i64);

impl Q {
    pub const ZERO: Q = Q(0);
    pub const ONE: Q = Q(1 << 32);

    /// Snaps a decimal to the nearest Q32.32 value (ties away from zero).
    pub fn snap(x: f64) -> Q {
        Q(to_raw(x))
    }

    /// The exact `f64` value of this Q32.32 number.
    ///
    /// Exact as long as `|raw| < 2^53`, which is asserted.
    pub fn f(self) -> f64 {
        assert!(
            self.0.unsigned_abs() < (1u64 << 53),
            "raw {} is not exactly representable as f64",
            self.0
        );
        self.0 as f64 / SCALE
    }
}

/// Rounds `x * 2^32` to the nearest integer, ties away from zero. Panics when the value is not
/// finite or does not fit a signed 64-bit raw: such outputs must be handled explicitly.
pub fn to_raw(x: f64) -> i64 {
    assert!(x.is_finite(), "non-finite value cannot be quantised: {x}");
    let scaled = (x * SCALE).round();
    assert!(
        scaled.abs() < 9.223_372_036_854_775e18,
        "value {x} overflows Q32.32"
    );
    scaled as i64
}

/// A 2D vector with Q32.32 components.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct QVec {
    pub x: Q,
    pub y: Q,
}

impl QVec {
    pub const ZERO: QVec = QVec {
        x: Q::ZERO,
        y: Q::ZERO,
    };

    pub fn snap(x: f64, y: f64) -> QVec {
        QVec {
            x: Q::snap(x),
            y: Q::snap(y),
        }
    }

    pub fn v(self) -> Vector {
        Vector::new(self.x.f(), self.y.f())
    }
}

/// A rotation given as a complex number whose two components are Q32.32 values.
///
/// There is no non-trivial pair of Q32.32 numbers with `re² + im² = 1` exactly (2^64 is not a
/// sum of two non-zero squares), so apart from the multiples of 90° the norm is `1 ± ~2^-32`.
/// The pair is handed to upstream **as is, without renormalisation**, so that both engines see
/// the same input.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct QRot {
    pub re: Q,
    pub im: Q,
}

impl QRot {
    pub const IDENTITY: QRot = QRot {
        re: Q::ONE,
        im: Q::ZERO,
    };

    /// Quarter turn (+90°), exact.
    pub const QUARTER: QRot = QRot {
        re: Q::ZERO,
        im: Q::ONE,
    };

    /// Half turn (180°), exact.
    pub const HALF: QRot = QRot {
        re: Q(-(1 << 32)),
        im: Q::ZERO,
    };

    /// `(cos, sin)` of an angle in degrees, each snapped to Q32.32.
    pub fn from_degrees(deg: f64) -> QRot {
        let a = deg.to_radians();
        QRot {
            re: Q::snap(a.cos()),
            im: Q::snap(a.sin()),
        }
    }

    pub fn r(self) -> Rotation {
        Rotation {
            re: self.re.f(),
            im: self.im.f(),
        }
    }
}

/// A rigid transformation with Q32.32 components.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct QPose {
    pub translation: QVec,
    pub rotation: QRot,
}

impl QPose {
    pub fn new(translation: QVec, rotation: QRot) -> QPose {
        QPose {
            translation,
            rotation,
        }
    }

    pub fn translation(x: f64, y: f64) -> QPose {
        QPose::new(QVec::snap(x, y), QRot::IDENTITY)
    }

    pub fn p(self) -> Pose {
        Pose::from_parts(self.translation.v(), self.rotation.r())
    }
}

// ---------------------------------------------------------------------------------------------
// JSON encoders. A scalar is `{"f64": x, "raw": n}`, a vector `{"f64": [x, y], "raw": [rx, ry]}`.
// ---------------------------------------------------------------------------------------------

/// Encodes an exactly representable input scalar.
pub fn jq(q: Q) -> Value {
    json!({ "f64": q.f(), "raw": q.0 })
}

/// Encodes an input vector.
pub fn jqvec(v: QVec) -> Value {
    json!({ "f64": [v.x.f(), v.y.f()], "raw": [v.x.0, v.y.0] })
}

/// Encodes an input rotation.
pub fn jqrot(r: QRot) -> Value {
    json!({ "f64": [r.re.f(), r.im.f()], "raw": [r.re.0, r.im.0] })
}

/// Encodes an input pose.
pub fn jqpose(p: QPose) -> Value {
    json!({ "translation": jqvec(p.translation), "rotation": jqrot(p.rotation) })
}

/// Encodes an output scalar (upstream `f64` + nearest Q32.32 raw).
pub fn jf(x: f64) -> Value {
    // Normalise -0.0 so the JSON never contains a negative zero.
    let x = if x == 0.0 { 0.0 } else { x };
    json!({ "f64": x, "raw": to_raw(x) })
}

/// Encodes an output vector.
pub fn jvec(v: Vector) -> Value {
    let (x, y) = (nz(v.x), nz(v.y));
    json!({ "f64": [x, y], "raw": [to_raw(x), to_raw(y)] })
}

/// Encodes an output rotation as `[re, im]`.
pub fn jrot(r: Rotation) -> Value {
    let (re, im) = (nz(r.re), nz(r.im));
    json!({ "f64": [re, im], "raw": [to_raw(re), to_raw(im)] })
}

fn nz(x: f64) -> f64 {
    if x == 0.0 {
        0.0
    } else {
        x
    }
}
