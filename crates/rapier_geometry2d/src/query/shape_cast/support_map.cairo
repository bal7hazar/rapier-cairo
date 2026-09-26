//! Support-map shape casts (Parry `shape_cast_support_map_support_map.rs`): every pair upstream
//! sends to GJK's ray cast on the Minkowski difference, answered by one exact kernel (ADR 0001
//! entries 14, 17, 24: analytic kernels instead of GJK / EPA).
//!
//! # The kernel
//!
//! Shape 2 translated by `d` touches shape 1 exactly when `d` lies in the *configuration
//! obstacle* `core1 - core2` dilated by `R = r1 + r2 + target_distance`, where `core1` and
//! `core2` are the polygonal cores of `super::super::support_map` (both in the frame of shape 1)
//! and `r1`, `r2` their radii. The time of impact is the entry time of the ray `t vel12` into
//! that rounded polygon:
//!
//! 1. **Faces.** The edges of the difference are the faces of core1 against the support set of
//!    core2 along the opposite normal, and the faces of core2 against the support set of core1
//!    (ties kept as a span, so that parallel faces give one long edge). Each is a line
//!    `m . x <= h` with the unit normal `m` of its core.
//! 2. **Entry** into the polygon of the lines moved out by `R`: the latest entry `t_in` (one
//!    correctly rounded wide quotient each) over the lines the origin violates and the ray moves
//!    into; a violated line the ray leaves or runs along is a miss. Only the faces the ray moves
//!    into are built (their support sets are the cost).
//! 3. **Span test.** The entry point lies on the dilated face (a flat piece of the boundary) when
//!    it is within the span of the face. Beyond it, the point is either outside the polygon (the
//!    ray misses it: every point of the dilated face that bounds the polygon is within the span)
//!    or in the corner region of the end vertex it passed, where the boundary is the circle of
//!    radius `R` around that vertex: the ray is cast on that circle (exact kernel of
//!    `crate::ray::ball`), and a miss there is a miss of the shape (a ray that enters a corner
//!    region can only reach the shape through that arc, and the circle lies in the polygon). With
//!    `R = 0` a point beyond the span is a miss.
//! 4. **Start inside the clipped polygon.** The exact witness of the two cores at the start
//!    (`super::super::support_map::core_witness`) decides: within `target_distance` answers
//!    `t = 0`; otherwise the origin is in the corner region of the closest vertex pair, whose
//!    circle is cast as in step 3.
//!
//! The witnesses are the vertex pair (circle) or the point of the face at the entry (flat piece,
//! split between the two spans), pushed out by the radii along the normal. The time of impact is
//! exact up to the rounding of the pose transform, of the unit normals and of one quotient or
//! square root; GJK stops within its tolerance. For a face–face impact the witnesses are one
//! member of the family of contact points, as GJK's.
//!
//! # Candidates
//!
//! Ranked by the `gas_*` probes of `super::tests` (cuboid against a turned cuboid, raw Sierra gas
//! and exact Cairo steps); the rejected ones live in `super::alternatives`.
//!
//! 1. **Minkowski ray cast, lazy entry** (this module): only the faces the ray moves into are
//!    built, the span test rejects the misses. **Winner**: 685k gas / 5,295 steps for the whole
//!    cast (the kernel alone, `gas_cso_cast_*`: 650k / 4,991).
//! 2. `alternatives::cso_cast_clipped`: full Cyrus–Beck clipping (every face built, exit bounds
//!    divided too). Same answers (`tests::test_clipped_agrees`); kernel 1.02M / 8,041.
//! 3. `alternatives::cast_conservative_advancement`: conservative advancement on the exact
//!    witness of `super::super::support_map` (upstream's ray-cast family): each step moves by the
//!    distance over the approach speed until the distance is within `GJK_EPS_TOL`. 1.50M / 12,516:
//!    a full witness per step, and only linear convergence on rounded shapes.

use fixed::wide::dot2;
use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::math_ext::norm2::is_zero2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::point::wide2::dot_wide;
use crate::ray::Ray;
use crate::ray::ball::ray_toi_with_ball;
use crate::ray::quotient::div_wide;
use crate::shape::{Segment, Shape};
use super::super::normalize_and_length;
use super::super::support_map::{Core, Witness, core_witness, local_core, transformed};
use super::{ShapeCastHit, ShapeCastOptions, ShapeCastStatus, TOI_AT_START};

/// Four ulps: the rounding slack of a position read along a face.
const SLACK: Fixed = Fixed { raw: 4 };
/// `2^32`: moves a `Fixed` raw to the Q64.64 scale of a wide product.
const SCALE: i128 = 0x100000000;

/// An edge of `core1 - core2`: the points `p - q` with `p` in the span `[p_lo, p_hi]` of core1 and
/// `q` in the span `[q_lo, q_hi]` of core2, both spans ordered along `e = perp(normal)`.
#[derive(Copy, Drop, Debug)]
pub struct CsoFace {
    /// Unit outward normal of the difference, from shape 1 towards shape 2.
    pub normal: Vec2,
    /// `max(normal . x)` over the difference of the cores (not dilated).
    pub offset: Fixed,
    pub p_lo: Vec2,
    pub p_hi: Vec2,
    pub q_lo: Vec2,
    pub q_hi: Vec2,
}

/// Where the ray meets the dilated difference.
#[derive(Copy, Drop, Debug, PartialEq)]
pub struct CsoHit {
    pub time_of_impact: Fixed,
    /// Unit outward normal of the dilated difference at the entry (frame of shape 1).
    pub normal: Vec2,
    /// The point of core1 and the point of core2 (translated to the impact) that meet.
    pub p: Vec2,
    pub q: Vec2,
}

/// The answer of [`cso_cast`].
#[derive(Copy, Drop, Debug, PartialEq)]
pub enum CsoCast {
    Miss,
    /// The origin satisfies every dilated face line: inside the dilated difference or in one of
    /// its corner regions.
    Inside,
    Hit: CsoHit,
}

#[inline(always)]
fn dot(a: Vec2, b: Vec2) -> Fixed {
    dot2(a.x, b.x, a.y, b.y)
}

/// `e = perp(m)`, the direction along a face of normal `m`.
#[inline(always)]
fn along(m: Vec2) -> Vec2 {
    Vec2 { x: -m.y, y: m.x }
}

/// The two points ordered along `e`.
#[inline(always)]
fn ordered(a: Vec2, b: Vec2, e: Vec2) -> (Vec2, Vec2) {
    if dot(b, e) < dot(a, e) {
        (b, a)
    } else {
        (a, b)
    }
}

/// The support set of `vertices` along `m` (`maximize`) or `-m`: `(extreme m . v, lo, hi)`, the
/// tied vertices' extremes along `e`.
fn support_span(vertices: Span<Vec2>, m: Vec2, e: Vec2, maximize: bool) -> (Fixed, Vec2, Vec2) {
    let first = *vertices[0];
    let mut best = dot(first, m);
    let (mut lo, mut hi) = (first, first);
    let (mut lo_e, mut hi_e) = (dot(first, e), dot(first, e));
    let n = vertices.len();
    let mut i = 1;
    while i != n {
        let v = *vertices[i];
        let d = dot(v, m);
        if (maximize && d > best) || (!maximize && d < best) {
            best = d;
            lo = v;
            hi = v;
            lo_e = dot(v, e);
            hi_e = lo_e;
        } else if d == best {
            let de = dot(v, e);
            if de < lo_e {
                lo = v;
                lo_e = de;
            } else if de > hi_e {
                hi = v;
                hi_e = de;
            }
        }
        i += 1;
    }
    (best, lo, hi)
}

/// The edge of `core1 - core2` along the face `f` (axis `u`) of core1.
#[inline(always)]
fn face_of_core1(u: Vec2, f: Segment, vertices2: Span<Vec2>) -> CsoFace {
    let e = along(u);
    let (p_lo, p_hi) = ordered(f.a, f.b, e);
    let (min2, q_lo, q_hi) = support_span(vertices2, u, e, false);
    let (da, db) = (dot(f.a, u), dot(f.b, u));
    let max1 = if da > db {
        da
    } else {
        db
    };
    CsoFace { normal: u, offset: max1 - min2, p_lo, p_hi, q_lo, q_hi }
}

/// The edge of `core1 - core2` along the face `f` (outward axis `w`) of core2.
#[inline(always)]
fn face_of_core2(w: Vec2, f: Segment, vertices1: Span<Vec2>) -> CsoFace {
    let m = -w;
    let e = along(m);
    let (q_lo, q_hi) = ordered(f.a, f.b, e);
    let (max1, p_lo, p_hi) = support_span(vertices1, m, e, true);
    let (da, db) = (dot(f.a, m), dot(f.b, m));
    let min2 = if da < db {
        da
    } else {
        db
    };
    CsoFace { normal: m, offset: max1 - min2, p_lo, p_hi, q_lo, q_hi }
}

/// Every edge of `core1 - core2` (both cores in the same frame), core1's faces first.
pub fn cso_faces(core1: Core, core2: Core) -> Array<CsoFace> {
    let mut out = array![];
    let mut faces = core1.faces;
    for u in core1.axes {
        out.append(face_of_core1(*u, *faces.pop_front().unwrap(), core2.vertices));
    }
    let mut faces = core2.faces;
    for w in core2.axes {
        out.append(face_of_core2(*w, *faces.pop_front().unwrap(), core1.vertices));
    }
    out
}

/// The entering candidate of `face` for the ray `t dir`: its entry time when the origin violates
/// the dilated line, `None` otherwise; `Err` when the time overflows (a miss).
#[inline(always)]
fn entry_of(face: CsoFace, radius: Fixed, den: i128) -> Result<Option<Fixed>, ()> {
    let h = face.offset + radius;
    if h >= ZERO {
        return Ok(None);
    }
    let num: i128 = h.raw.into() * SCALE;
    match div_wide(num, den) {
        Some(t) => Ok(Some(t)),
        None => Err(()),
    }
}

/// The ray `t dir` on the circle of `radius` around `p - q`: the hit, normal and vertex pair.
pub fn cast_on_vertex(p: Vec2, q: Vec2, radius: Fixed, dir: Vec2, fallback: Vec2) -> CsoCast {
    let center = p - q;
    let ray = Ray { origin: Vec2 { x: ZERO, y: ZERO }, dir };
    let (_, toi) = ray_toi_with_ball(center, radius, ray, true);
    match toi {
        Some(t) => {
            let (n, _) = normalize_and_length(dir.mul_scalar(t) - center);
            CsoCast::Hit(CsoHit { time_of_impact: t, normal: n.unwrap_or(fallback), p, q })
        },
        None => CsoCast::Miss,
    }
}

/// The entry of the ray `t dir` (`t >= 0`) into `core1 - core2` dilated by `radius`, both cores
/// in the same frame; `Miss` also when the entry is beyond `max_toi`. See the module
/// documentation.
///
/// Only the faces the ray moves into (`normal . dir < 0`) can bound the entry: their support
/// sets are computed first, and the latest entry among the lines the origin violates is kept.
/// A convex polygon's entry point lies on that face's edge exactly when the ray enters the
/// polygon, so the span test of [`classify`] replaces the exit bounds of full clipping
/// (`super::alternatives::cso_cast_clipped` measures them). Only when
/// no such face exists are the other faces read, to tell a start inside from a ray moving away.
/// #### Panics
/// * The overflow panics of the wide products (coordinates bounded by 2^20 are safe).
pub fn cso_cast(core1: Core, core2: Core, radius: Fixed, dir: Vec2, max_toi: Fixed) -> CsoCast {
    let mut t_in = ZERO;
    let mut entering: Option<CsoFace> = None;
    let mut faces = core1.faces;
    for u in core1.axes {
        let f = *faces.pop_front().unwrap();
        let den = dot_wide(*u.x, *u.y, dir.x, dir.y);
        if den < 0 {
            let face = face_of_core1(*u, f, core2.vertices);
            match entry_of(face, radius, den) {
                Ok(Some(t)) => if entering.is_none() || t > t_in {
                    t_in = t;
                    entering = Some(face);
                },
                Ok(None) => {},
                Err(()) => { return CsoCast::Miss; },
            }
        }
    }
    let mut faces = core2.faces;
    for w in core2.axes {
        let f = *faces.pop_front().unwrap();
        let den = -dot_wide(*w.x, *w.y, dir.x, dir.y);
        if den < 0 {
            let face = face_of_core2(*w, f, core1.vertices);
            match entry_of(face, radius, den) {
                Ok(Some(t)) => if entering.is_none() || t > t_in {
                    t_in = t;
                    entering = Some(face);
                },
                Ok(None) => {},
                Err(()) => { return CsoCast::Miss; },
            }
        }
    }
    match entering {
        Some(face) => if t_in > max_toi {
            CsoCast::Miss
        } else {
            classify(face, t_in, radius, dir)
        },
        None => if violates_other_faces(core1, core2, radius, dir) {
            CsoCast::Miss
        } else {
            CsoCast::Inside
        },
    }
}

/// Whether the origin violates a dilated face line the ray does not move into (it then never
/// enters: a miss).
fn violates_other_faces(core1: Core, core2: Core, radius: Fixed, dir: Vec2) -> bool {
    let mut faces = core1.faces;
    for u in core1.axes {
        let f = *faces.pop_front().unwrap();
        if dot_wide(*u.x, *u.y, dir.x, dir.y) >= 0 && face_of_core1(*u, f, core2.vertices).offset
            + radius < ZERO {
            return true;
        }
    }
    let mut faces = core2.faces;
    for w in core2.axes {
        let f = *faces.pop_front().unwrap();
        if dot_wide(*w.x, *w.y, dir.x, dir.y) <= 0 && face_of_core2(*w, f, core1.vertices).offset
            + radius < ZERO {
            return true;
        }
    }
    false
}

/// The hit of the ray entering the dilated line of `face` at `t_in`: the flat piece when the
/// entry is within the face's span (four ulps of slack; always for a zero `radius`), the circle
/// of the end vertex it passed otherwise.
pub fn classify(face: CsoFace, t_in: Fixed, radius: Fixed, dir: Vec2) -> CsoCast {
    let e = along(face.normal);
    let start = face.p_lo - face.q_hi;
    let x = dir.mul_scalar(t_in);
    let s = dot(x - start, e);
    let len_p = dot(face.p_hi - face.p_lo, e);
    let len = len_p + dot(face.q_hi - face.q_lo, e);
    if s < -SLACK {
        return if radius == ZERO {
            CsoCast::Miss
        } else {
            cast_on_vertex(face.p_lo, face.q_hi, radius, dir, face.normal)
        };
    }
    if s > len + SLACK {
        return if radius == ZERO {
            CsoCast::Miss
        } else {
            cast_on_vertex(face.p_hi, face.q_lo, radius, dir, face.normal)
        };
    }
    // The flat piece: split `s` between the span of core1 and the span of core2.
    let s = if s < ZERO {
        ZERO
    } else if s > len {
        len
    } else {
        s
    };
    let s_p = if s < len_p {
        s
    } else {
        len_p
    };
    let p = face.p_lo + e.mul_scalar(s_p);
    let q = face.q_hi - e.mul_scalar(s - s_p);
    CsoCast::Hit(CsoHit { time_of_impact: t_in, normal: face.normal, p, q })
}

/// The first impact of the support-map shape `g2`, placed at `pos12` and moving at `vel12`, with
/// the support-map shape `g1` (upstream `cast_shapes_support_map_support_map`); see the module
/// documentation for the kernel and `super` for the semantics.
///
/// A start within the target distance answers `t = 0`. An impact below `1e-4` reports the
/// contact geometry of the start (`compute_impact_geometry_on_penetration`, or
/// `stop_at_penetration = false`, which then drops a separating motion, `normal1 . vel12 >= 0`
/// exact). Otherwise `witness1` is on the real surface of `g1` (not inflated) and `witness2` on
/// that of `g2`; a `t = 0` impact without contact geometry reports upstream's `-vel12 / |vel12|`
/// normal and its origin witnesses.
/// #### Panics
/// * `'Query: not a support map'` when either shape is a half-space; the overflow panics of the
///   transforms and wide products (coordinates bounded by 2^20 are safe).
pub fn cast_shapes_support_map_support_map(
    pos12: Pose2, vel12: Vec2, g1: Shape, g2: Shape, options: ShapeCastOptions,
) -> Option<ShapeCastHit> {
    if is_zero2(vel12.x, vel12.y) {
        return None;
    }
    let core1 = local_core(g1);
    let core2 = transformed(local_core(g2), pos12);
    let td = options.target_distance;
    let inflate = if td > ZERO {
        td
    } else {
        ZERO
    };
    let radius = core1.radius + core2.radius + inflate;
    let mut start: Option<Witness> = None;
    let cast = match cso_cast(core1, core2, radius, vel12, options.max_time_of_impact) {
        CsoCast::Inside => {
            let w = core_witness(core1, core2);
            start = Some(w);
            if w.dist <= inflate {
                None
            } else {
                // The corner region of the closest vertex pair.
                let p = w.point1 - w.normal1.mul_scalar(core1.radius);
                let q = w.point2 + w.normal1.mul_scalar(core2.radius);
                match cast_on_vertex(p, q, radius, vel12, w.normal1) {
                    CsoCast::Hit(h) => Some(h),
                    _ => { return None; },
                }
            }
        },
        CsoCast::Hit(h) => Some(h),
        CsoCast::Miss => { return None; },
    };
    let time_of_impact = match cast {
        Some(h) => h.time_of_impact,
        None => ZERO,
    };
    if time_of_impact > options.max_time_of_impact {
        return None;
    }
    let geometry = options.compute_impact_geometry_on_penetration || !options.stop_at_penetration;
    if geometry && time_of_impact < TOI_AT_START {
        let w = start.unwrap_or_else(|| core_witness(core1, core2));
        if !options.stop_at_penetration
            && dot_wide(w.normal1.x, w.normal1.y, vel12.x, vel12.y) >= 0 {
            return None;
        }
        return Some(
            ShapeCastHit {
                time_of_impact,
                witness1: w.point1,
                witness2: pos12.inverse_transform_point(w.point2),
                normal1: w.normal1,
                normal2: -pos12.rotation.inverse_rotate(w.normal1),
                status: status_at_start(w, td),
            },
        );
    }
    match cast {
        Some(h) => {
            let status = if time_of_impact == ZERO {
                let w = start.unwrap_or_else(|| core_witness(core1, core2));
                status_at_start(w, td)
            } else {
                ShapeCastStatus::Converged
            };
            Some(
                ShapeCastHit {
                    time_of_impact,
                    // Upstream: the witness of the inflated shape less `normal1 * td`.
                    witness1: h.p + h.normal.mul_scalar(core1.radius + inflate - td),
                    witness2: pos12
                        .inverse_transform_point(h.q - h.normal.mul_scalar(core2.radius)),
                    normal1: h.normal,
                    normal2: -pos12.rotation.inverse_rotate(h.normal),
                    status,
                },
            )
        },
        None => {
            // `t = 0` without contact geometry: upstream's initial direction and its origin
            // witnesses.
            let (dir, _) = normalize_and_length(-vel12);
            let normal1 = dir.unwrap();
            Some(
                ShapeCastHit {
                    time_of_impact,
                    witness1: -normal1.mul_scalar(td),
                    witness2: pos12.inverse_transform_point(Vec2 { x: ZERO, y: ZERO }),
                    normal1,
                    normal2: -pos12.rotation.inverse_rotate(normal1),
                    status: status_at_start(start.unwrap(), td),
                },
            )
        },
    }
}

/// `PenetratingOrWithinTargetDist` when the start is strictly closer than `target_distance`.
#[inline(always)]
fn status_at_start(w: Witness, target_distance: Fixed) -> ShapeCastStatus {
    if w.dist < target_distance {
        ShapeCastStatus::PenetratingOrWithinTargetDist
    } else {
        ShapeCastStatus::Converged
    }
}
