//! Rejected candidates of `super::support_map`, kept for the `gas_*` ranking of `super::tests`.

use fixed::wide::dot2;
use fixed::{Fixed, MAX, ZERO};
use glam_core::{Vec2, Vec2Trait};
use rapier_math::consts::GJK_EPS_TOL;
use rapier_math::math_ext::norm2::is_zero2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::point::wide2::dot_wide;
use crate::ray::quotient::div_wide;
use crate::shape::{Segment, Shape};
use super::support_map::{CsoCast, CsoFace, classify, cso_faces, in_slab};
use super::super::support_map::{Core, Witness, core_witness, local_core, transformed};
use super::{ShapeCastHit, ShapeCastOptions, ShapeCastStatus};

/// Step bound of [`cast_conservative_advancement`].
const MAX_STEPS: u32 = 64;
const SCALE: i128 = 0x100000000;

/// `core` translated by `d`.
fn shifted(core: Core, d: Vec2) -> Core {
    let mut vertices = array![];
    for v in core.vertices {
        vertices.append(*v + d);
    }
    let mut faces = array![];
    for f in core.faces {
        faces.append(Segment { a: *f.a + d, b: *f.b + d });
    }
    Core { vertices: vertices.span(), axes: core.axes, faces: faces.span(), radius: core.radius }
}

/// Conservative advancement on the exact witness: from `t`, the shapes cannot meet before
/// `t + gap / approach` (`gap` the distance less the target, `approach` the closing speed along
/// the normal), stop when `gap <= GJK_EPS_TOL`. Same answer as the winner within the tolerance
/// on the hits it finds (status `Converged` or `OutOfIterations`, no contact-geometry fallback).
pub fn cast_conservative_advancement(
    pos12: Pose2, vel12: Vec2, g1: Shape, g2: Shape, options: ShapeCastOptions,
) -> Option<ShapeCastHit> {
    if is_zero2(vel12.x, vel12.y) {
        return None;
    }
    let core1 = local_core(g1);
    let core2 = transformed(local_core(g2), pos12);
    let mut t = ZERO;
    let mut step = 0;
    while step != MAX_STEPS {
        let w = core_witness(core1, shifted(core2, vel12.mul_scalar(t)));
        let gap = w.dist - options.target_distance;
        if gap <= GJK_EPS_TOL {
            return Some(hit(pos12, t, w.point1, w.point2 - vel12.mul_scalar(t), w.normal1, true));
        }
        let approach = -dot_wide(w.normal1.x, w.normal1.y, vel12.x, vel12.y);
        if approach <= 0 {
            return None;
        }
        let num: i128 = gap.raw.into() * SCALE;
        let dt = div_wide(num, approach)?;
        t = t + dt;
        if t > options.max_time_of_impact {
            return None;
        }
        step += 1;
    }
    let w = core_witness(core1, shifted(core2, vel12.mul_scalar(t)));
    Some(hit(pos12, t, w.point1, w.point2 - vel12.mul_scalar(t), w.normal1, false))
}

fn hit(pos12: Pose2, t: Fixed, p1: Vec2, p2: Vec2, normal1: Vec2, converged: bool) -> ShapeCastHit {
    ShapeCastHit {
        time_of_impact: t,
        witness1: p1,
        witness2: pos12.inverse_transform_point(p2),
        normal1,
        normal2: -pos12.rotation.inverse_rotate(normal1),
        status: if converged {
            ShapeCastStatus::Converged
        } else {
            ShapeCastStatus::OutOfIterations
        },
    }
}

/// Full Cyrus–Beck clipping: every face built, the exit bounds `t_out` divided too, `t_in >
/// t_out`
/// a miss before the span test (the winner's `super::support_map::cso_cast` builds only the faces
/// the ray moves into and lets the span test reject the misses).
pub fn cso_cast_clipped(
    core1: Core, core2: Core, radius: Fixed, dir: Vec2, max_toi: Fixed,
) -> CsoCast {
    let faces = cso_faces(core1, core2);
    let mut t_in = ZERO;
    let mut t_out = MAX;
    let mut entering: Option<CsoFace> = None;
    for face in faces.span() {
        let face = *face;
        let h = face.offset + radius;
        let den = dot_wide(face.normal.x, face.normal.y, dir.x, dir.y);
        if den == 0 {
            if h < ZERO {
                return CsoCast::Miss;
            }
            continue;
        }
        let num: i128 = h.raw.into() * SCALE;
        if den < 0 {
            if h < ZERO {
                let Some(t) = div_wide(num, den) else {
                    return CsoCast::Miss;
                };
                if entering.is_none() || t > t_in {
                    t_in = t;
                    entering = Some(face);
                }
            }
        } else {
            if h < ZERO {
                return CsoCast::Miss;
            }
            if let Some(t) = div_wide(num, den) {
                if t < t_out {
                    t_out = t;
                }
            }
        }
    }
    let Some(face) = entering else {
        return CsoCast::Inside;
    };
    if t_in > t_out || t_in > max_toi {
        return CsoCast::Miss;
    }
    classify(face, t_in, radius, dir)
}

/// `super::support_map::on_face` without choosing the face by its direction: every face of core1
/// is tested with core2's point, then every face of core2 with core1's (same answers: the outer
/// slabs of a convex core's faces are disjoint).
pub fn on_face_every_face(core1: Core, core2: Core, w: Witness) -> Witness {
    let (r1, r2) = (core1.radius, core2.radius);
    if w.dist + r1 + r2 <= ZERO {
        return w;
    }
    let n = w.normal1;
    let p1 = w.point1 - n.mul_scalar(r1);
    let p2 = w.point2 + n.mul_scalar(r2);
    let mut normal: Option<Vec2> = None;
    let mut faces = core1.faces;
    for u in core1.axes {
        let f = *faces.pop_front().unwrap();
        let d = p2 - f.a;
        if normal.is_none() && in_slab(*u, f, p2, dot2(d.x, *u.x, d.y, *u.y)) {
            normal = Some(*u);
        }
    }
    let mut faces = core2.faces;
    for w2 in core2.axes {
        let f = *faces.pop_front().unwrap();
        let m = -*w2;
        let d = f.a - p1;
        if normal.is_none() && in_slab(m, f, p1, dot2(d.x, m.x, d.y, m.y)) {
            normal = Some(m);
        }
    }
    match normal {
        Some(m) => Witness {
            point1: p1 + m.mul_scalar(r1), point2: p2 - m.mul_scalar(r2), normal1: m, dist: w.dist,
        },
        None => w,
    }
}
