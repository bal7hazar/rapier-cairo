//! Point queries on a [`HalfSpace`] (Parry `query/point/point_halfspace.rs`).
//!
//! The half space is `{ p : normal . p <= 0 }` with the boundary through the local origin, so
//! every query is one dot product: no square root, no division, no squared length — and hence
//! none of the fixed-point hazards the rest of this module answers.

use fixed::wide::{dot2, mul_add};
use fixed::{Fixed, ZERO};
use glam::vec2::Vec2;
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::shape::HalfSpace;
use super::PointProjection;

/// Projects `pt` on `halfspace`.
///
/// Mirrors `PointQuery::project_local_point` for `HalfSpace`. A point on the boundary counts as
/// inside. With `solid = true` an inside point projects to itself; with `solid = false` it is
/// pushed to the boundary plane.
/// #### Panics
/// * `'Fixed: overflow'` if the dot product or a projected component leaves the scalar range.
/// #### Deviations
/// * `normal` is assumed unit, as upstream does; the projection is `pt - normal * (normal . pt)`
///   with one rescale per component (the floor of the exact value).
pub fn project_local_point_halfspace(
    halfspace: HalfSpace, pt: Vec2, solid: bool,
) -> PointProjection {
    let d = dot2(halfspace.normal.x, pt.x, halfspace.normal.y, pt.y);
    let is_inside = d <= ZERO;
    if is_inside && solid {
        PointProjection { is_inside: true, point: pt }
    } else {
        PointProjection {
            is_inside,
            point: Vec2 {
                x: mul_add(ZERO - halfspace.normal.x, d, pt.x),
                y: mul_add(ZERO - halfspace.normal.y, d, pt.y),
            },
        }
    }
}

/// Projects `pt` on `halfspace` and names the feature it landed on.
///
/// Mirrors `PointQuery::project_local_point_and_get_feature` for `HalfSpace`: the boundary is one
/// face, so the answer is always `Face(0)`.
/// #### Panics
/// * See [`project_local_point_halfspace`].
/// #### Deviations
/// * None.
#[inline(always)]
pub fn project_local_point_and_get_feature_halfspace(
    halfspace: HalfSpace, pt: Vec2,
) -> (PointProjection, FeatureId) {
    (project_local_point_halfspace(halfspace, pt, false), FeatureIdTrait::face(0))
}

/// Returns the signed distance from `pt` to the boundary of `halfspace`: negative inside when
/// `solid = false`, clamped to 0 inside when `solid = true`.
///
/// Mirrors `PointQuery::distance_to_local_point` for `HalfSpace`.
/// #### Panics
/// * `'Fixed: overflow'` if the dot product leaves the scalar range.
/// #### Deviations
/// * The dot product is fused (one rescale), so the result is the floor of the exact value.
pub fn distance_to_local_point_halfspace(halfspace: HalfSpace, pt: Vec2, solid: bool) -> Fixed {
    let dist = dot2(halfspace.normal.x, pt.x, halfspace.normal.y, pt.y);
    if solid && dist < ZERO {
        ZERO
    } else {
        dist
    }
}

/// Returns `true` when `pt` is inside `halfspace`, boundary included.
///
/// Mirrors `PointQuery::contains_local_point` for `HalfSpace`.
/// #### Panics
/// * `'Fixed: overflow'` if the dot product leaves the scalar range.
/// #### Deviations
/// * None.
#[inline(always)]
pub fn contains_local_point_halfspace(halfspace: HalfSpace, pt: Vec2) -> bool {
    dot2(halfspace.normal.x, pt.x, halfspace.normal.y, pt.y) <= ZERO
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use crate::feature_id::FeatureIdTrait;
    use crate::shape::HalfSpace;
    use super::super::PointProjection;
    use super::{
        contains_local_point_halfspace, distance_to_local_point_halfspace,
        project_local_point_and_get_feature_halfspace, project_local_point_halfspace,
    };

    const UNIT: i64 = 0x1_0000_0000;
    const HALF: i64 = 0x8000_0000;
    /// `cos(45°) = sin(45°)`.
    const SQRT2_2: i64 = 3037000500;

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }
    }

    fn up() -> HalfSpace {
        HalfSpace { normal: v(0, UNIT) }
    }

    #[test]
    fn test_projection_table() {
        // (half space, point, solid, expected point, expected is_inside)
        let diag = HalfSpace { normal: v(SQRT2_2, SQRT2_2) };
        let cases: Span<(HalfSpace, Vec2, bool, Vec2, bool)> = array![
            (up(), v(UNIT, UNIT), false, v(UNIT, 0), false),
            (up(), v(UNIT, UNIT), true, v(UNIT, 0), false),
            (up(), v(UNIT, 0), false, v(UNIT, 0), true),
            (up(), v(UNIT, -UNIT), true, v(UNIT, -UNIT), true),
            (up(), v(UNIT, -UNIT), false, v(UNIT, 0), true),
            (diag, v(UNIT, UNIT), false, v(0, 0), false),
            (diag, v(-UNIT, -UNIT), true, v(-UNIT, -UNIT), true),
        ]
            .span();
        for (h, pt, solid, point, is_inside) in cases {
            let got = project_local_point_halfspace(*h, *pt, *solid);
            assert_eq!(got.is_inside, *is_inside);
            assert!(got.point.x.abs_diff_eq(*point.x, Fixed { raw: 4 }));
            assert!(got.point.y.abs_diff_eq(*point.y, Fixed { raw: 4 }));
        }
    }

    #[test]
    fn test_distance_and_containment() {
        // (point, expected distance with solid = false, contains)
        let cases: Span<(Vec2, i64, bool)> = array![
            (v(0, UNIT), UNIT, false), (v(0, 0), 0, true), (v(3 * UNIT, -HALF), -HALF, true),
        ]
            .span();
        for (pt, expected, contains) in cases {
            assert_eq!(
                distance_to_local_point_halfspace(up(), *pt, false), Fixed { raw: *expected },
            );
            assert_eq!(contains_local_point_halfspace(up(), *pt), *contains);
            assert!(distance_to_local_point_halfspace(up(), *pt, true) >= ZERO);
        }
    }

    #[test]
    fn test_feature_is_always_face_zero() {
        let (proj, feature) = project_local_point_and_get_feature_halfspace(up(), v(UNIT, UNIT));
        assert_eq!(feature, FeatureIdTrait::face(0));
        assert_eq!(proj, PointProjection { is_inside: false, point: v(UNIT, 0) });
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_project_halfspace_outside() {
        let _ = project_local_point_halfspace(opaque(up()), opaque(v(UNIT, UNIT)), false);
    }

    #[test]
    fn gas_project_halfspace_inside_solid() {
        let _ = project_local_point_halfspace(opaque(up()), opaque(v(UNIT, -UNIT)), true);
    }

    #[test]
    fn gas_project_and_get_feature_halfspace() {
        let _ = project_local_point_and_get_feature_halfspace(opaque(up()), opaque(v(UNIT, UNIT)));
    }

    #[test]
    fn gas_distance_to_local_point_halfspace() {
        let _ = distance_to_local_point_halfspace(opaque(up()), opaque(v(UNIT, UNIT)), false);
    }

    #[test]
    fn gas_contains_local_point_halfspace() {
        let _ = contains_local_point_halfspace(opaque(up()), opaque(v(UNIT, UNIT)));
    }
}
