//! 2D SAT in upstream axis order. Nonnegative half extents/radii and unit poses
//! are required. Products floor once; inputs, displacements, projections and results must fit
//! Q32.32, otherwise the fixed arithmetic panics. No allocation or support loops.
use fixed::wide::{dot2, normalize2};
use fixed::{Fixed, FixedTrait, MAX, ONE, ZERO};
use glam::Vec2;
use rapier_math::consts::DEFAULT_EPSILON;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::shape::{Capsule, Cuboid, Segment};

const X: Vec2 = Vec2 { x: ONE, y: ZERO };
const Y: Vec2 = Vec2 { x: ZERO, y: ONE };
const O: Vec2 = Vec2 { x: ZERO, y: ZERO };
fn dot(a: Vec2, b: Vec2) -> Fixed {
    dot2(a.x, b.x, a.y, b.y)
}
fn signed(v: Fixed, sign: Fixed) -> Fixed {
    if sign < ZERO {
        -v
    } else {
        v
    }
}
fn support(c: Cuboid, d: Vec2) -> Vec2 {
    Vec2 { x: signed(c.half_extents.x, d.x), y: signed(c.half_extents.y, d.y) }
}
fn world_support(c: Cuboid, p: Pose2, d: Vec2) -> Vec2 {
    p.transform_point(support(c, p.inverse_transform_vector(d)))
}
fn best(a: (Fixed, Vec2), b: (Fixed, Vec2)) -> (Fixed, Vec2) {
    let (sa, _) = a;
    let (sb, _) = b;
    if sb > sa {
        b
    } else {
        a
    }
}

/// Separation on `axis1`, oriented from c1 to c2; axis is assumed unit.
/// Extends the upstream 3D helper to 2D. Floors dot/pose products; panics on overflow.
pub fn cuboid_cuboid_compute_separation_wrt_local_line(
    c1: Cuboid, c2: Cuboid, pos12: Pose2, axis1: Vec2,
) -> (Fixed, Vec2) {
    let axis = if dot(pos12.translation, axis1) < ZERO {
        -axis1
    } else {
        axis1
    };
    (dot(world_support(c2, pos12, -axis) - support(c1, axis), axis), axis)
}

fn finish_cuboids(
    c1: Cuboid, c2: Cuboid, p: Pose2, sx: Fixed, sy: Fixed, ax: Vec2, ay: Vec2,
) -> (Fixed, Vec2) {
    if sx >= ZERO && sy >= ZERO {
        // Rescale before normalizing: (epsilon, epsilon) must give a unit 45°
        // axis, not the short-vector precision cliff of a Q32.32 length.
        let wx = sx.max(DEFAULT_EPSILON);
        let wy = sy.max(DEFAULT_EPSILON);
        let scale = wx.max(wy);
        let (nx, ny) = normalize2(wx / scale, wy / scale);
        let axis = Vec2 { x: signed(nx, ax.x), y: signed(ny, ay.y) };
        (dot(world_support(c2, p, -axis) - support(c1, axis), axis), axis)
    } else {
        best((sx, ax), (sy, ay))
    }
}

/// Tests c1's x then y face axis (first wins ties). Two nonnegative gaps produce
/// upstream's weighted diagonal with DEFAULT_EPSILON nudges. At most one normalize.
/// Q32.32 floor products; same valid range and overflow policy as this module.
pub fn cuboid_cuboid_find_local_separating_normal_oneway(
    c1: Cuboid, c2: Cuboid, pos12: Pose2,
) -> (Fixed, Vec2) {
    let ax = Vec2 { x: signed(ONE, pos12.translation.x), y: ZERO };
    let ay = Vec2 { x: ZERO, y: signed(ONE, pos12.translation.y) };
    // Project the two radii directly. The positive-facing axis must floor a
    // NEGATIVE radius; negating an already floored positive radius is off by one
    // ulp and can change an exact tie. This equals the direct support transforms.
    let re = pos12.rotation.re.abs();
    let im = pos12.rotation.im.abs();
    let h = c2.half_extents;
    let rx = dot2(signed(re, -ax.x), h.x, signed(im, -ax.x), h.y);
    let ry = dot2(signed(im, -ay.y), h.x, signed(re, -ay.y), h.y);
    let sx = signed(pos12.translation.x + rx, ax.x) - c1.half_extents.x;
    let sy = signed(pos12.translation.y + ry, ay.y) - c1.half_extents.y;
    finish_cuboids(c1, c2, pos12, sx, sy, ax, ay)
}

fn segment_search(c1: Cuboid, seg: Segment, p: Pose2, radius: Fixed) -> (Fixed, Vec2) {
    let a = p.transform_point(seg.a);
    let b = p.transform_point(seg.b);
    let nx = -a.x.max(b.x) - c1.half_extents.x - radius;
    let px = a.x.min(b.x) - c1.half_extents.x - radius;
    let ny = -a.y.max(b.y) - c1.half_extents.y - radius;
    let py = a.y.min(b.y) - c1.half_extents.y - radius;
    best(best(best((nx, -X), (px, X)), (ny, -Y)), (py, Y))
}

/// Tests the cuboid's -x,+x,-y,+y axes against a segment; first wins ties.
/// Zero-length segments are valid. Floors pose products; panics on overflow.
pub fn cuboid_segment_find_local_separating_normal_oneway(
    c1: Cuboid, segment2: Segment, pos12: Pose2,
) -> (Fixed, Vec2) {
    segment_search(c1, segment2, pos12, ZERO)
}

/// Capsule support-map specialization, testing -x,+x,-y,+y with first-axis ties.
/// Nonnegative radius and a unit pose required; subtracts radius from each core
/// separation (rotation invariant). Floors pose products; panics on overflow.
pub fn cuboid_support_map_find_local_separating_normal_oneway(
    c1: Cuboid, capsule2: Capsule, pos12: Pose2,
) -> (Fixed, Vec2) {
    segment_search(c1, capsule2.segment, pos12, capsule2.radius)
}

/// Tests an optional unit normal, oriented from point1 toward the cuboid.
/// `None` returns `(-fixed::MAX, zero)` as the finite upstream sentinel.
/// Floors products; panics on overflow under the module's bounds.
pub fn point_cuboid_find_local_separating_normal_oneway(
    point1: Vec2, normal1: Option<Vec2>, shape2: Cuboid, pos12: Pose2,
) -> (Fixed, Vec2) {
    match normal1 {
        None => (-MAX, O),
        Some(n) => {
            let axis = if dot(pos12.translation - point1, n) >= ZERO {
                n
            } else {
                -n
            };
            (dot(world_support(shape2, pos12, -axis) - point1, axis), axis)
        },
    }
}

/// Tests the segment's normal against a cuboid (reverse one-way query).
/// A zero-length segment returns `(-fixed::MAX, zero)` without normalization.
/// One normalization at most; short vectors are scaled up first to preserve unit
/// length. Nearest normal components, floored dot/pose products; overflow panics.
pub fn segment_cuboid_find_local_separating_normal_oneway(
    segment1: Segment, shape2: Cuboid, pos12: Pose2,
) -> (Fixed, Vec2) {
    let d = segment1.b - segment1.a;
    let scale = d.x.abs().max(d.y.abs());
    let n = if scale == ZERO {
        None
    } else {
        // Normalizing a raw (1,1) directly gives (1,1), not a unit vector.
        // Scale tiny segments up before the single normalization.
        let (x, y) = if scale < ONE {
            normalize2(d.y / scale, -d.x / scale)
        } else {
            normalize2(d.y, -d.x)
        };
        Some(Vec2 { x, y })
    };
    point_cuboid_find_local_separating_normal_oneway(segment1.a, n, shape2, pos12)
}

#[cfg(test)]
mod alternatives {
    //! Measured candidates retained for reproducibility.
    use core::num::traits::WideMul;
    use fixed::{Fixed, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use super::{Cuboid, Segment, X, Y, best, finish_cuboids, signed};

    // Literal unrolled upstream cuboid support search.
    pub fn cuboids_direct(c1: Cuboid, c2: Cuboid, pos12: Pose2) -> (Fixed, Vec2) {
        let ax = Vec2 { x: signed(fixed::ONE, pos12.translation.x), y: ZERO };
        let ay = Vec2 { x: ZERO, y: signed(fixed::ONE, pos12.translation.y) };
        let px = super::world_support(c2, pos12, -ax);
        let py = super::world_support(c2, pos12, -ay);
        let sx = signed(px.x, ax.x) - c1.half_extents.x;
        let sy = signed(py.y, ay.y) - c1.half_extents.y;
        finish_cuboids(c1, c2, pos12, sx, sy, ax, ay)
    }

    fn on_axis(c: Cuboid, s: Segment, p: Pose2, axis: Vec2) -> (Fixed, Vec2) {
        let dir = p.inverse_transform_vector(-axis);
        let a = s.a.x.raw.wide_mul(dir.x.raw) + s.a.y.raw.wide_mul(dir.y.raw);
        let b = s.b.x.raw.wide_mul(dir.x.raw) + s.b.y.raw.wide_mul(dir.y.raw);
        let point = p.transform_point(if a > b {
            s.a
        } else {
            s.b
        });
        let sep = if axis.x != ZERO {
            signed(point.x, axis.x) - c.half_extents.x
        } else {
            signed(point.y, axis.y) - c.half_extents.y
        };
        (sep, axis)
    }

    pub fn segment_direct(c: Cuboid, s: Segment, p: Pose2) -> (Fixed, Vec2) {
        best(
            best(best(on_axis(c, s, p, -X), on_axis(c, s, p, X)), on_axis(c, s, p, -Y)),
            on_axis(c, s, p, Y),
        )
    }
}
#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, MAX, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{IDENTITY, Pose2};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use super::{
        Capsule, Cuboid, O, Segment, X, Y, alternatives,
        cuboid_cuboid_compute_separation_wrt_local_line,
        cuboid_cuboid_find_local_separating_normal_oneway,
        cuboid_segment_find_local_separating_normal_oneway,
        cuboid_support_map_find_local_separating_normal_oneway,
        point_cuboid_find_local_separating_normal_oneway,
        segment_cuboid_find_local_separating_normal_oneway,
    };
    const C: Cuboid = Cuboid { half_extents: Vec2 { x: ONE, y: HALF } };
    const S: Segment = Segment { a: Vec2 { x: Fixed { raw: -4294967296 }, y: ZERO }, b: X };
    const P: Pose2 = Pose2 {
        translation: Vec2 { x: TWO, y: HALF }, rotation: Rot2 { re: ONE, im: ZERO },
    };

    #[test]
    fn test_ties_degeneracy_capsule_and_point() {
        assert_eq!(cuboid_segment_find_local_separating_normal_oneway(C, S, IDENTITY), (-HALF, -Y));
        let square = Cuboid { half_extents: Vec2 { x: ONE, y: ONE } };
        assert_eq!(
            cuboid_cuboid_find_local_separating_normal_oneway(square, square, IDENTITY), (-TWO, X),
        );
        assert_eq!(point_cuboid_find_local_separating_normal_oneway(O, None, C, P), (-MAX, O));
        assert_eq!(
            segment_cuboid_find_local_separating_normal_oneway(Segment { a: O, b: O }, C, P),
            (-MAX, O),
        );
        for radius in array![ZERO, HALF, ONE].span() {
            let (sep, axis) = cuboid_segment_find_local_separating_normal_oneway(C, S, P);
            assert_eq!(
                cuboid_support_map_find_local_separating_normal_oneway(
                    C, Capsule { segment: S, radius: *radius }, P,
                ),
                (sep - *radius, axis),
            );
        }
        assert_eq!(cuboid_cuboid_compute_separation_wrt_local_line(C, C, P, -X), (ZERO, X));
    }

    #[test]
    fn test_weighted_corner_and_large_gap() {
        for raw in array![0_i64, 4294967296, 429496729600000].span() {
            let gap = Fixed { raw: *raw };
            let p = Pose2 { translation: Vec2 { x: TWO + gap, y: ONE + gap }, ..IDENTITY };
            let (sep, n) = cuboid_cuboid_find_local_separating_normal_oneway(C, C, p);
            assert!(sep >= ZERO);
            assert!(n.x.abs_diff_eq(n.y, Fixed { raw: 2 }));
            assert!(n.x.abs_diff_eq(fixed::FRAC_1_SQRT_2, Fixed { raw: 2 }));
        }
    }

    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_candidates(x: i16, y: i16) {
        for rotation in array![
            IDENTITY.rotation,
            Rot2 { re: Fixed { raw: 2576980378 }, im: Fixed { raw: 3435973837 } },
            Rot2 { re: Fixed { raw: -3037000500 }, im: Fixed { raw: 3037000500 } },
        ]
            .span() {
            let p = Pose2 {
                translation: Vec2 {
                    x: Fixed { raw: x.into() * 65537 }, y: Fixed { raw: y.into() * 131071 },
                },
                rotation: *rotation,
            };
            let c = Cuboid { half_extents: Vec2 { x: Fixed { raw: 4294967297 }, y: HALF } };
            assert_eq!(
                cuboid_segment_find_local_separating_normal_oneway(c, S, p),
                alternatives::segment_direct(c, S, p),
            );
            assert_eq!(
                cuboid_cuboid_find_local_separating_normal_oneway(c, c, p),
                alternatives::cuboids_direct(c, c, p),
            );
        }
    }
    #[test]
    fn test_projection_rounding_keeps_axis_sign() {
        let a = Cuboid { half_extents: O };
        let b = Cuboid { half_extents: Vec2 { x: Fixed { raw: 1 }, y: ZERO } };
        for sign in array![ONE, -ONE].span() {
            let p = Pose2 {
                translation: Vec2 { x: *sign, y: ZERO },
                rotation: Rot2 { re: Fixed { raw: 2576980378 }, im: Fixed { raw: 3435973837 } },
            };
            let actual = cuboid_cuboid_find_local_separating_normal_oneway(a, b, p);
            assert_eq!(actual, alternatives::cuboids_direct(a, b, p));
            let expected = if *sign == ONE {
                Fixed { raw: 4294967295 }
            } else {
                ONE
            };
            assert_eq!(actual, (expected, Vec2 { x: *sign, y: ZERO }));
        }
    }
    #[test]
    fn test_tiny_segment_normal_is_unit() {
        let s = Segment { a: O, b: Vec2 { x: Fixed { raw: 1 }, y: Fixed { raw: 1 } } };
        let (_, n) = segment_cuboid_find_local_separating_normal_oneway(s, C, P);
        assert!(
            rapier_math::math_ext::norm2::is_unit2_raw(
                n.x, n.y, rapier_math::consts::UNIT_TOL_SQ_RAW,
            ),
        );
    }
    #[test]
    #[fuzzer(runs: 64, seed: 20260921)]
    fn fuzz_separated_candidates(x: i16, y: i16) {
        let p = Pose2 {
            translation: Vec2 {
                x: FixedTrait::from_int(x.into()), y: FixedTrait::from_int(y.into()),
            },
            rotation: Rot2 { re: Fixed { raw: 2576980378 }, im: Fixed { raw: -3435973837 } },
        };
        assert_eq!(
            cuboid_segment_find_local_separating_normal_oneway(C, S, p),
            alternatives::segment_direct(C, S, p),
        );
        assert_eq!(
            cuboid_cuboid_find_local_separating_normal_oneway(C, C, p),
            alternatives::cuboids_direct(C, C, p),
        );
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(P);
    }
    #[test]
    fn gas_cuboids_minmax() {
        let _ = cuboid_cuboid_find_local_separating_normal_oneway(C, C, opaque(P));
    }
    #[test]
    fn gas_cuboids_direct() {
        let _ = alternatives::cuboids_direct(C, C, opaque(P));
    }
    #[test]
    fn gas_cuboids_diagonal() {
        let p = Pose2 { translation: Vec2 { x: Fixed { raw: 12884901888 }, y: TWO }, ..P };
        let _ = cuboid_cuboid_find_local_separating_normal_oneway(C, C, opaque(p));
    }
    #[test]
    fn gas_cuboids_line() {
        let _ = cuboid_cuboid_compute_separation_wrt_local_line(C, C, opaque(P), X);
    }
    #[test]
    fn gas_segment_minmax() {
        let _ = cuboid_segment_find_local_separating_normal_oneway(C, S, opaque(P));
    }
    #[test]
    fn gas_segment_direct() {
        let _ = alternatives::segment_direct(C, S, opaque(P));
    }
    #[test]
    fn gas_capsule() {
        let _ = cuboid_support_map_find_local_separating_normal_oneway(
            C, Capsule { segment: S, radius: HALF }, opaque(P),
        );
    }
    #[test]
    fn gas_point_cuboid() {
        let _ = point_cuboid_find_local_separating_normal_oneway(O, Some(Y), C, opaque(P));
    }
    #[test]
    fn gas_segment_cuboid() {
        let _ = segment_cuboid_find_local_separating_normal_oneway(S, C, opaque(P));
    }
}
