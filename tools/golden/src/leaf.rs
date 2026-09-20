//! Helpers shared by the leaf-level families (`pose2`, `aabb_overlap`, `sat2d`, `clip2d`,
//! `point_projection`, `segment_segment`): JSON encoders for upstream outputs and a few input
//! shorthands. The quantisation rule itself lives in `q.rs`.

use crate::q::{jf, jqvec, jrot, jvec, QVec};
use rapier2d_f64::math::{Pose, Vector};
use rapier2d_f64::parry::shape::SegmentPointLocation;
use serde_json::{json, Value};

/// Input vector from decimals, snapped to Q32.32.
pub fn qv(x: f64, y: f64) -> QVec {
    QVec::snap(x, y)
}

/// Encodes an output pose (upstream `f64` + nearest raw).
pub fn jpose(p: Pose) -> Value {
    json!({ "translation": jvec(p.translation), "rotation": jrot(p.rotation) })
}

/// Encodes an input segment given by two Q32.32 end points.
pub fn jqseg(a: QVec, b: QVec) -> Value {
    json!({ "a": jqvec(a), "b": jqvec(b) })
}

/// Encodes a list of output vectors.
pub fn jvecs(vs: &[Vector]) -> Value {
    Value::Array(vs.iter().map(|v| jvec(*v)).collect())
}

/// Encodes a location on a segment: `{"kind": "vertex", "vertex": i}` or
/// `{"kind": "edge", "u": u}` where the point is `a + u (b - a)` (upstream stores `[1 - u, u]`).
pub fn jloc(l: &SegmentPointLocation) -> Value {
    match l {
        SegmentPointLocation::OnVertex(i) => json!({ "kind": "vertex", "vertex": i }),
        SegmentPointLocation::OnEdge([_, u]) => json!({ "kind": "edge", "u": jf(*u) }),
    }
}
