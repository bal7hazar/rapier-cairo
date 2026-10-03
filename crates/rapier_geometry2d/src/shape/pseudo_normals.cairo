//! The pseudo-normal cone projection shared by the segment and triangle pseudo-normals (Parry
//! `shape/pseudo_normals.rs`, `project_into_cone`; PX6).
//!
//! # Fixed point
//!
//! * Every product and sum is a Q32.32 operation of `Vec2` / `Fixed`; the lengths are the
//!   floored wide norms of `fixed::wide`, the normalisations `v * recip(|v|)` (exact for an
//!   axis-aligned vector).
//! * Upstream's `length <= 1.0e-6` guard is `length <= LENGTH_EPS` on the floored length: a floored
//!   raw length is at most `floor(1.0e-6 * 2^32) = 4294` exactly when it is at most `1.0e-6`.

use fixed::{Fixed, ONE, TWO, ZERO};
use glam_core::{Vec2, Vec2Trait};
use crate::query::normalize_and_length;

/// `floor(1.0e-6 * 2^32)` raw: upstream's `1.0e-6` length threshold (see the module doc).
const LENGTH_EPS: Fixed = Fixed { raw: 4294 };

/// Scales a vector by a scalar (component-wise `Fixed` products).
#[inline(always)]
fn scale(v: Vec2, s: Fixed) -> Vec2 {
    Vec2 { x: v.x * s, y: v.y * s }
}

/// Projects `dir` into the cone of normals around `face` bounded by the pseudo-normal
/// `closest_edge` (upstream `project_into_cone`); answers `(accepted, projected direction)`.
///
/// `accepted` is `dir . face >= 0`, the value upstream returns whatever the projection did.
/// The direction is replaced by `face` when the cone is degenerate (`closest_edge == face`), kept
/// when it already lies in the cone or when no correction can be computed (its component
/// orthogonal to `face` is zero, or the adjusted pseudo-normal is shorter than `1.0e-6`), and
/// otherwise reflected onto the cone's boundary.
///
/// Like upstream, `face`, `closest_edge` and `dir` are assumed unit-sized.
/// #### Panics
/// * `'Fixed: overflow'` if a product leaves Q32.32 (not for unit inputs).
pub(crate) fn project_into_cone(face: Vec2, closest_edge: Vec2, dir: Vec2) -> (bool, Vec2) {
    let dot_face = dir.dot(face);
    let accepted = dot_face >= ZERO;

    if closest_edge == face {
        // The normal cone is degenerate, there is only one possible direction.
        return (accepted, face);
    }

    let dot_edge_face = face.dot(closest_edge);
    let dot_dir_face = face.dot(dir);
    // cos(2 * angle(closest_edge, face))
    let dot_corrected_dir_face = TWO * dot_edge_face * dot_edge_face - ONE;

    if dot_dir_face >= dot_corrected_dir_face {
        // The direction is in the pseudo-normal cone. No correction to apply.
        return (true, dir);
    }

    // We need to correct.
    let edge_on_normal = scale(face, dot_edge_face);
    let edge_orthogonal_to_normal = closest_edge - edge_on_normal;

    let dir_on_normal = scale(face, dot_dir_face);
    let dir_orthogonal_to_normal = dir - dir_on_normal;
    let Some(unit_dir_orthogonal_to_normal) = dir_orthogonal_to_normal.try_normalize() else {
        return (accepted, dir);
    };

    let adjusted_pseudo_normal = edge_on_normal
        + scale(unit_dir_orthogonal_to_normal, edge_orthogonal_to_normal.length());
    let (adjusted, length) = normalize_and_length(adjusted_pseudo_normal);
    let Some(adjusted) = adjusted else {
        return (accepted, dir);
    };
    if length <= LENGTH_EPS {
        return (accepted, dir);
    }

    // The reflection of the face normal wrt. the adjusted pseudo-normal gives us the second end
    // of the pseudo-normal cone the direction is projected on.
    (accepted, scale(adjusted, TWO * face.dot(adjusted)) - face)
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use glam_core::Vec2;
    use rapier_testing::opaque;
    use super::project_into_cone;

    fn v(x: i32, y: i32) -> Vec2 {
        Vec2 { x: FixedTrait::from_int(x), y: FixedTrait::from_int(y) }
    }

    const Y: Vec2 = Vec2 { x: ZERO, y: ONE };

    /// `(1, 1) / sqrt(2)` to 1 ulp.
    fn diag() -> Vec2 {
        let h: Fixed = Fixed { raw: 3037000500 };
        Vec2 { x: h, y: h }
    }

    #[test]
    fn test_degenerate_cone_collapses_to_face() {
        // (face, dir, accepted, projected): the cone is one direction, +Y; a direction orthogonal
        // to the face is accepted (`>= 0`).
        let cases: Span<(Vec2, Vec2, bool, Vec2)> = array![
            (Y, v(1, 1), true, Y), (Y, v(-1, 0), true, Y), (Y, v(0, -1), false, Y),
        ]
            .span();
        for (face, dir, accepted, projected) in cases {
            let (ok, out) = project_into_cone(*face, *face, *dir);
            assert_eq!((ok, out), (*accepted, *projected));
        }
    }

    #[test]
    fn test_direction_inside_the_cone_is_kept() {
        // A wide cone (the edge pseudo-normal at 45 degrees): +Y itself, and the edge, are inside.
        let edge = diag();
        let (ok, out) = project_into_cone(Y, edge, Y);
        assert_eq!((ok, out), (true, Y));
        let (ok, out) = project_into_cone(Y, edge, edge);
        assert_eq!((ok, out), (true, edge));
    }

    #[test]
    fn test_direction_outside_the_cone_is_clamped() {
        // The cone around +Y is bounded at +-90 degrees (`cos(2 * 45) = 0`): the direction
        // (1, -1) / sqrt(2) is clamped onto the boundary, +X, and rejected (it points away).
        let edge = diag();
        let dir = Vec2 { x: edge.x, y: -edge.y };
        let (ok, out) = project_into_cone(Y, edge, dir);
        assert!(!ok);
        assert!(out.x.raw >= ONE.raw - 4 && out.x.raw <= ONE.raw + 4, "x");
        assert!(out.y.raw >= -4 && out.y.raw <= 4, "y");
    }

    #[test]
    fn test_direction_along_the_face_has_no_correction_axis() {
        // The cone around +Y is bounded at +-90 degrees; -Y is outside and has no component
        // orthogonal to the face, so upstream keeps it (and rejects it).
        let (ok, out) = project_into_cone(Y, diag(), v(0, -1));
        assert_eq!((ok, out), (false, v(0, -1)));
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_project_into_cone() {
        let _ = project_into_cone(opaque(Y), opaque(diag()), opaque(v(1, -1)));
    }
}
