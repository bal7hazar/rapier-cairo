//! Point queries on a [`Cuboid`] (Parry `query/point/point_cuboid.rs`, which forwards to
//! `point_aabb.rs` on the AABB `[-half_extents, half_extents]`).
//!
//! No square root and no division: the whole family is comparisons, differences and one length
//! for the distance. The centre of the box is the local origin, so upstream's `self.center()`
//! is exactly zero here and the `(mins + maxs) / 2` of the AABB version disappears.
//!
//! # Candidates
//!
//! Ranked by the `gas_*` probes of this module; the rejected one lives in
//! `#[cfg(test)] mod alternatives`.
//!
//! 1. **clamp / abs** (this module): the outside shift is `clamp(pt, mins, maxs) - pt` and the
//!    inside one is `(half_extents - |pt|)` with the sign copied back. **Winner**.
//! 2. `alternatives::do_project_sign_mul`: the literal port,
//!    `max(mins - pt, 0) - max(pt - maxs, 0)` and `diff * sign(pt)`. Same answers (proved by
//!    `fuzz_candidates_agree`), two extra products and two extra comparisons.

use fixed::wide::distance2;
use fixed::{Fixed, FixedTrait, ZERO};
use glam::vec2::Vec2;
use rapier_math::consts::DEFAULT_EPSILON;
use crate::feature_id::{FEATURE_UNKNOWN, FeatureId, FeatureIdTrait};
use super::PointProjection;
use super::shapes_shim::Cuboid;

/// Projects `pt` on `cuboid` and also returns the shift that was applied, which
/// [`project_local_point_and_get_feature_cuboid`] reads to name the feature.
///
/// Mirrors `Aabb::do_project_local_point`.
fn do_project_local_point(cuboid: Cuboid, pt: Vec2, solid: bool) -> (bool, Vec2, Vec2) {
    let he = cuboid.half_extents;
    // `clamp(pt, -he, he) - pt`: zero exactly when the point is in the box, boundary included.
    let shift = Vec2 {
        x: pt.x.clamp(ZERO - he.x, he.x) - pt.x, y: pt.y.clamp(ZERO - he.y, he.y) - pt.y,
    };
    if shift.x != ZERO || shift.y != ZERO {
        return (false, Vec2 { x: pt.x + shift.x, y: pt.y + shift.y }, shift);
    }
    if solid {
        return (true, pt, Vec2 { x: ZERO, y: ZERO });
    }
    // Inside and hollow: push the point to the nearest face. `diff` is the distance to the face
    // on the side of `pt`; the axis with the smaller one wins, `x` on a tie, and a zero
    // coordinate is treated as positive (upstream's `copy_sign_to` on `+0.0`).
    let diff_x = he.x - pt.x.abs();
    let diff_y = he.y - pt.y.abs();
    let shift = if diff_x <= diff_y {
        Vec2 { x: diff_x.copysign(pt.x), y: ZERO }
    } else {
        Vec2 { x: ZERO, y: diff_y.copysign(pt.y) }
    };
    (true, Vec2 { x: pt.x + shift.x, y: pt.y + shift.y }, shift)
}

/// Projects `pt` on `cuboid`.
///
/// Mirrors `PointQuery::project_local_point` for `Cuboid`. A point on a face or on a vertex
/// counts as inside. With `solid = true` an inside point projects to itself; with `solid = false`
/// it is pushed to the nearest face — the axis whose face is closer wins, `x` on a tie, and an
/// exactly zero coordinate is pushed towards `+`.
/// #### Panics
/// * `'i64_neg Underflow'` if a half extent is `fixed::MIN`.
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if a difference leaves the scalar range (a
///   point and a box at opposite ends of the range).
/// #### Deviations
/// * Exact: every step is a comparison or a difference of representable values. Upstream's
///   `-0.0` bias is not representable in Q32.32, which is why a zero coordinate reads as `+`;
///   upstream's own comment says the choice is arbitrary and the golden vectors agree with `+`.
#[inline(always)]
pub fn project_local_point_cuboid(cuboid: Cuboid, pt: Vec2, solid: bool) -> PointProjection {
    let (is_inside, point, _) = do_project_local_point(cuboid, pt, solid);
    PointProjection { is_inside, point }
}

/// Projects `pt` on `cuboid` and names the feature it landed on.
///
/// Mirrors `PointQuery::project_local_point_and_get_feature` for `Cuboid`, with the **f32**
/// feature codes of Parry: `Face(0)`/`Face(1)` for `+x`/`+y`, `Face(2)`/`Face(3)` for `-x`/`-y`,
/// and `Vertex(code)` with bit `i` set when coordinate `i` is below the centre. A point that was
/// not shifted at all (on the boundary, or inside on a face tie) reports the first face it
/// touches within `rapier_math::consts::DEFAULT_EPSILON`, so a point exactly on a vertex is
/// `Face(0)`, not a vertex.
/// #### Panics
/// * See [`project_local_point_cuboid`].
/// #### Deviations
/// * None; the `DEFAULT_EPSILON` band is upstream's, re-expressed in Q32.32 (512 ulp).
pub fn project_local_point_and_get_feature_cuboid(
    cuboid: Cuboid, pt: Vec2,
) -> (PointProjection, FeatureId) {
    let (is_inside, point, shift) = do_project_local_point(cuboid, pt, false);
    let proj = PointProjection { is_inside, point };
    let he = cuboid.half_extents;
    let zero_x = shift.x == ZERO;
    let zero_y = shift.y == ZERO;
    if zero_x && zero_y {
        // The point was not moved: report the first face it lies on.
        let feature = if point.x > he.x - DEFAULT_EPSILON {
            FeatureIdTrait::face(0)
        } else if point.x <= DEFAULT_EPSILON - he.x {
            FeatureIdTrait::face(2)
        } else if point.y > he.y - DEFAULT_EPSILON {
            FeatureIdTrait::face(1)
        } else if point.y <= DEFAULT_EPSILON - he.y {
            FeatureIdTrait::face(3)
        } else {
            FEATURE_UNKNOWN
        };
        (proj, feature)
    } else if zero_x || zero_y {
        // Exactly one axis moved: the projection is on the face of that axis.
        let (axis, coord) = if zero_x {
            (1, point.y)
        } else {
            (0, point.x)
        };
        let feature = if coord < ZERO {
            FeatureIdTrait::face(axis + 2)
        } else {
            FeatureIdTrait::face(axis)
        };
        (proj, feature)
    } else {
        let mut code = 0_u32;
        if point.x < ZERO {
            code += 1;
        }
        if point.y < ZERO {
            code += 2;
        }
        (proj, FeatureIdTrait::vertex(code))
    }
}

/// Returns the distance from `pt` to `cuboid`, negative inside when `solid = false`.
///
/// Mirrors `PointQuery::distance_to_local_point` for `Aabb`: outside it is the length of the
/// per-axis overshoot, inside (and hollow) it is minus the distance to the nearest face.
/// #### Panics
/// * `'i64_neg Underflow'` if a half extent is `fixed::MIN`.
/// * `'Fixed: overflow'` if the length does not fit the scalar range.
/// #### Deviations
/// * `fixed::wide::distance2` floors the length, so the magnitude is at most 1 ulp below the
///   exact distance; the differences it squares are exact (no intermediate rescale).
pub fn distance_to_local_point_cuboid(cuboid: Cuboid, pt: Vec2, solid: bool) -> Fixed {
    let he = cuboid.half_extents;
    // `max(mins - pt, pt - maxs, 0)`: the per-axis overshoot, zero inside.
    let over_x = (ZERO - he.x - pt.x).max(pt.x - he.x).max(ZERO);
    let over_y = (ZERO - he.y - pt.y).max(pt.y - he.y).max(ZERO);
    if solid || over_x != ZERO || over_y != ZERO {
        fixed::wide::norm2(over_x, over_y)
    } else {
        let (_, point, _) = do_project_local_point(cuboid, pt, false);
        ZERO - distance2(pt.x, pt.y, point.x, point.y)
    }
}

/// Returns `true` when `pt` is inside `cuboid`, boundary included.
///
/// Mirrors `PointQuery::contains_local_point` for `Aabb`.
/// #### Panics
/// * `'i64_neg Underflow'` if a half extent is `fixed::MIN`.
/// #### Deviations
/// * None (exact comparisons).
#[inline(always)]
pub fn contains_local_point_cuboid(cuboid: Cuboid, pt: Vec2) -> bool {
    let he = cuboid.half_extents;
    pt.x >= ZERO - he.x && pt.x <= he.x && pt.y >= ZERO - he.y && pt.y <= he.y
}

/// Rejected candidates, kept for the `gas_*` ranking.
#[cfg(test)]
pub mod alternatives {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use glam::vec2::Vec2;
    use rapier_math::math_ext::scalar::copy_sign_to;
    use super::super::PointProjection;
    use super::super::shapes_shim::Cuboid;

    /// The literal port of `Aabb::do_project_local_point`: `max(mins - pt, 0) - max(pt - maxs, 0)`
    /// for the outside shift and `diff * sign(pt)` for the inside one. Same answers as the
    /// shipped version, two products and two comparisons more.
    pub fn do_project_sign_mul(cuboid: Cuboid, pt: Vec2, solid: bool) -> (bool, Vec2, Vec2) {
        let he = cuboid.half_extents;
        let mins_pt = Vec2 { x: ZERO - he.x - pt.x, y: ZERO - he.y - pt.y };
        let pt_maxs = Vec2 { x: pt.x - he.x, y: pt.y - he.y };
        let shift = Vec2 {
            x: mins_pt.x.max(ZERO) - pt_maxs.x.max(ZERO),
            y: mins_pt.y.max(ZERO) - pt_maxs.y.max(ZERO),
        };
        if shift.x != ZERO || shift.y != ZERO {
            return (false, Vec2 { x: pt.x + shift.x, y: pt.y + shift.y }, shift);
        }
        if solid {
            return (true, pt, Vec2 { x: ZERO, y: ZERO });
        }
        let sgn = Vec2 { x: copy_sign_to(pt.x, ONE), y: copy_sign_to(pt.y, ONE) };
        let diff = Vec2 { x: he.x - sgn.x * pt.x, y: he.y - sgn.y * pt.y };
        let shift = if diff.x <= diff.y {
            Vec2 { x: diff.x * sgn.x, y: ZERO }
        } else {
            Vec2 { x: ZERO, y: diff.y * sgn.y }
        };
        (true, Vec2 { x: pt.x + shift.x, y: pt.y + shift.y }, shift)
    }

    /// [`do_project_sign_mul`] behind the public signature, for the `gas_*` comparison.
    pub fn project_sign_mul(cuboid: Cuboid, pt: Vec2, solid: bool) -> PointProjection {
        let (is_inside, point, _) = do_project_sign_mul(cuboid, pt, solid);
        PointProjection { is_inside, point }
    }

    /// `contains_local_point` through the full projection, as the default trait method does.
    pub fn contains_via_projection(cuboid: Cuboid, pt: Vec2) -> bool {
        let (is_inside, _, _) = do_project_sign_mul(cuboid, pt, true);
        let _: Fixed = ZERO;
        is_inside
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use crate::feature_id::{FEATURE_UNKNOWN, FeatureId, FeatureIdTrait};
    use super::alternatives::{contains_via_projection, do_project_sign_mul, project_sign_mul};
    use super::super::PointProjection;
    use super::super::shapes_shim::Cuboid;
    use super::{
        contains_local_point_cuboid, distance_to_local_point_cuboid,
        project_local_point_and_get_feature_cuboid, project_local_point_cuboid,
    };

    const UNIT: i64 = 0x1_0000_0000;
    const HALF: i64 = 0x8000_0000;

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }
    }

    fn box2(x: i64, y: i64) -> Cuboid {
        Cuboid { half_extents: v(x, y) }
    }

    #[test]
    fn test_projection_table() {
        let b = box2(UNIT, HALF);
        // (point, solid, expected point, expected is_inside)
        let cases: Span<(Vec2, bool, Vec2, bool)> = array![
            // Outside a face, then outside a vertex region.
            (v(2 * UNIT, 0), false, v(UNIT, 0), false),
            (v(2 * UNIT, UNIT), false, v(UNIT, HALF), false),
            (v(-2 * UNIT, -UNIT), true, v(-UNIT, -HALF), false),
            // On a face and on a vertex: inside, untouched by both flags.
            (v(UNIT, 0), false, v(UNIT, 0), true), (v(UNIT, HALF), false, v(UNIT, HALF), true),
            (
                v(UNIT, HALF), true, v(UNIT, HALF), true,
            ), // Inside: solid keeps the point, hollow pushes it to the nearest face.
            (v(0, 0), true, v(0, 0), true), (v(0, 0), false, v(0, HALF), true),
            (v(HALF, 0), false, v(UNIT, 0), true), // tie on `diff`: `x` wins
            (v(-HALF, 0), false, v(-UNIT, 0), true), (v(0, -HALF / 2), false, v(0, -HALF), true),
        ]
            .span();
        for (pt, solid, point, is_inside) in cases {
            assert_eq!(
                project_local_point_cuboid(b, *pt, *solid),
                PointProjection { is_inside: *is_inside, point: *point },
            );
        }
    }

    #[test]
    fn test_feature_table() {
        let b = box2(UNIT, HALF);
        // (point, expected feature)
        let cases: Span<(Vec2, FeatureId)> = array![
            (v(2 * UNIT, 0), FeatureIdTrait::face(0)), (v(-2 * UNIT, 0), FeatureIdTrait::face(2)),
            (v(0, UNIT), FeatureIdTrait::face(1)), (v(0, -UNIT), FeatureIdTrait::face(3)),
            // Vertex regions: the code has bit `i` set when coordinate `i` is negative.
            (v(2 * UNIT, UNIT), FeatureIdTrait::vertex(0)),
            (v(-2 * UNIT, UNIT), FeatureIdTrait::vertex(1)),
            (v(2 * UNIT, -UNIT), FeatureIdTrait::vertex(2)),
            (v(-2 * UNIT, -UNIT), FeatureIdTrait::vertex(3)),
            // Zero shift: the first face touched wins, so a vertex reads as `Face(0)`.
            (v(UNIT, HALF), FeatureIdTrait::face(0)), (v(-UNIT, -HALF), FeatureIdTrait::face(2)),
            // Inside: the face it was pushed to.
            (v(0, 0), FeatureIdTrait::face(1)), (v(HALF, 0), FeatureIdTrait::face(0)),
            (v(0, -HALF / 2), FeatureIdTrait::face(3)),
        ]
            .span();
        for (pt, expected) in cases {
            let (_, feature) = project_local_point_and_get_feature_cuboid(b, *pt);
            assert_eq!(feature, *expected);
        }
    }

    /// A box thinner than `DEFAULT_EPSILON` has no interior the epsilon band can miss: every
    /// inside point touches a face, so `Unknown` is unreachable for it.
    #[test]
    fn test_feature_unknown_is_unreachable_for_a_thick_box() {
        let b = box2(UNIT, UNIT);
        let (_, feature) = project_local_point_and_get_feature_cuboid(b, v(0, 0));
        assert!(feature != FEATURE_UNKNOWN);
    }

    #[test]
    fn test_distance_and_containment() {
        let b = box2(UNIT, HALF);
        // (point, expected distance with solid = false, contains)
        let cases: Span<(Vec2, i64, bool)> = array![
            (v(2 * UNIT, 0), UNIT, false), (v(UNIT, 0), 0, true), (v(0, 0), -HALF, true),
            (v(HALF, 0), -HALF, true),
            (v(2 * UNIT, UNIT), 4801919417, false), // |(1, 0.5)| = sqrt(1.25)
            (v(0, HALF), 0, true),
        ]
            .span();
        for (pt, expected, contains) in cases {
            assert_eq!(distance_to_local_point_cuboid(b, *pt, false), Fixed { raw: *expected });
            assert_eq!(contains_local_point_cuboid(b, *pt), *contains);
            assert_eq!(contains_via_projection(b, *pt), *contains);
            assert!(distance_to_local_point_cuboid(b, *pt, true) >= ZERO);
        }
    }

    /// Huge and tiny boxes: the projection is a chain of exact differences, so nothing under- or
    /// overflows between `1 ulp` and `2^30`.
    #[test]
    fn test_extreme_extents() {
        let tiny = box2(1, 1);
        assert_eq!(project_local_point_cuboid(tiny, v(5, 5), false).point, v(1, 1));
        // A square box ties on `diff`, and `x` wins the tie.
        assert_eq!(project_local_point_cuboid(tiny, v(0, 0), false).point, v(1, 0));
        let huge = box2(0x4000_0000_0000_0000, 0x4000_0000_0000_0000);
        assert_eq!(
            project_local_point_cuboid(huge, v(0x7000_0000_0000_0000, 0), false).point,
            v(0x4000_0000_0000_0000, 0),
        );
        assert!(contains_local_point_cuboid(huge, v(0, 0x3fff_ffff_ffff_ffff)));
    }

    /// The two formulations of the projection answer identically everywhere.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_candidates_agree(x: i32, y: i32, hx: u16, hy: u16) {
        let b = box2(hx.into() * 65536, hy.into() * 65536);
        let pt = v(x.into(), y.into());
        for solid in array![true, false].span() {
            let shipped = project_local_point_cuboid(b, pt, *solid);
            let other = project_sign_mul(b, pt, *solid);
            assert_eq!(shipped, other);
        }
        let (_, _, s1) = super::do_project_local_point(b, pt, false);
        let (_, _, s2) = do_project_sign_mul(b, pt, false);
        assert_eq!(s1, s2);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_project_cuboid_outside() {
        let _ = project_local_point_cuboid(opaque(box2(UNIT, HALF)), opaque(v(2 * UNIT, 0)), false);
    }

    #[test]
    fn gas_project_cuboid_inside_hollow() {
        let _ = project_local_point_cuboid(opaque(box2(UNIT, HALF)), opaque(v(HALF / 2, 0)), false);
    }

    #[test]
    fn gas_project_cuboid_inside_solid() {
        let _ = project_local_point_cuboid(opaque(box2(UNIT, HALF)), opaque(v(HALF / 2, 0)), true);
    }

    #[test]
    fn gas_project_cuboid_sign_mul_outside() {
        let _ = project_sign_mul(opaque(box2(UNIT, HALF)), opaque(v(2 * UNIT, 0)), false);
    }

    #[test]
    fn gas_project_cuboid_sign_mul_inside_hollow() {
        let _ = project_sign_mul(opaque(box2(UNIT, HALF)), opaque(v(HALF / 2, 0)), false);
    }

    #[test]
    fn gas_project_and_get_feature_cuboid() {
        let _ = project_local_point_and_get_feature_cuboid(
            opaque(box2(UNIT, HALF)), opaque(v(2 * UNIT, UNIT)),
        );
    }

    #[test]
    fn gas_distance_to_local_point_cuboid_outside() {
        let _ = distance_to_local_point_cuboid(
            opaque(box2(UNIT, HALF)), opaque(v(2 * UNIT, UNIT)), false,
        );
    }

    #[test]
    fn gas_distance_to_local_point_cuboid_inside() {
        let _ = distance_to_local_point_cuboid(opaque(box2(UNIT, HALF)), opaque(v(0, 0)), false);
    }

    #[test]
    fn gas_contains_local_point_cuboid() {
        let _ = contains_local_point_cuboid(opaque(box2(UNIT, HALF)), opaque(v(2 * UNIT, UNIT)));
    }

    #[test]
    fn gas_contains_cuboid_via_projection() {
        let _ = contains_via_projection(opaque(box2(UNIT, HALF)), opaque(v(2 * UNIT, UNIT)));
    }
}
