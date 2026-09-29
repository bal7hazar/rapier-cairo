//! Closest points between two lines (Parry `query/closest_points/closest_points_line_line.rs`,
//! Ericson's routine without the segment clamping; PX3).
//!
//! The lines are given by an origin and a direction (not required to be unit), both in the same
//! frame. The fixed-point answers to the three hazards are those of the segment routine of
//! [`crate::closest_points`], which the module documentation there details:
//!
//! 1. the degeneracy tests (`|dir|^2 <= eps`) are wide: the raw Q64.64 sum of squares against
//!    `eps` lifted to that scale;
//! 2. the determinant `a e - b b` is the exact `i128` difference of two raw products, and
//! upstream's
//!    absolute threshold (`denom <= eps`, a fourth-degree quantity against a first-degree
//!    tolerance) is the relative test `denom <= a e * eps`, i.e. `sin^2(angle) <= eps`;
//!    `!ulps_eq!(ae, bb)`, a floating-point cancellation guard, has nothing left to catch;
//! 3. every quotient is one correctly rounded [`div_wide`] of two exact quantities of the same
//!    scale, saturated to the scalar range for lines nearly parallel to each other.

use fixed::wide::{dot2, norm2_squared};
use fixed::{Fixed, MAX, MIN, ZERO};
use glam_core::{Vec2, Vec2Trait};
use rapier_math::consts::DEFAULT_EPSILON;
use rapier_math::math_ext::norm2::norm2_sq_wide;
use crate::ray::quotient::div_wide;

/// `2^32`, the factor that lifts a `Fixed` raw to the Q64.64 scale of the other operand.
const SCALE: i128 = 0x1_0000_0000;

/// `num / den` (`den != 0`, both at the same scale), saturated to the scalar range.
#[inline(always)]
fn ratio(num: i128, den: i128) -> Fixed {
    let saturated = if (num < 0) != (den < 0) {
        MIN
    } else {
        MAX
    };
    div_wide(num, den).unwrap_or(saturated)
}

/// The parameters `(s, t)` of the closest points `orig1 + dir1 * s` and `orig2 + dir2 * t` of
/// two lines, and whether the lines are parallel (upstream
/// `closest_points_line_line_parameters_eps`). `eps` is the tolerance of upstream: a direction
/// whose squared length is at most `eps` is a point, and two directions whose sine squared is at
/// most `eps` are parallel (`s = 0`, `t` the projection of `orig1` on the second line).
///
/// For a degenerate first direction `s = 0` and `t = dir2 . r / |dir2|^2`; for a degenerate
/// second one `t = 0` and `s = -dir1 . r / |dir1|^2`; for both `(0, 0)`; `r = orig1 - orig2`.
/// The flag is `false` in every degenerate case, as upstream.
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `orig1 - orig2` leaves the scalar range.
/// * `'Fixed: overflow'` if a squared length or a dot product does not fit the scalar range
///   (`|dir| >= 2^15.5`).
/// * `'i128_mul Overflow'` for `eps > 1`.
/// #### Deviations
/// * See the module documentation.
pub fn closest_points_line_line_parameters_eps(
    orig1: Vec2, dir1: Vec2, orig2: Vec2, dir2: Vec2, eps: Fixed,
) -> (Fixed, Fixed, bool) {
    let r = orig1 - orig2;
    let eps_sq: i128 = eps.raw.into() * SCALE;
    let degenerate1 = norm2_sq_wide(dir1.x, dir1.y) <= eps_sq;
    let degenerate2 = norm2_sq_wide(dir2.x, dir2.y) <= eps_sq;
    if degenerate1 && degenerate2 {
        return (ZERO, ZERO, false);
    }
    let e: i128 = norm2_squared(dir2.x, dir2.y).raw.into();
    let f: i128 = dot2(dir2.x, r.x, dir2.y, r.y).raw.into();
    if degenerate1 {
        return (ZERO, ratio(f, e), false);
    }
    let a: i128 = norm2_squared(dir1.x, dir1.y).raw.into();
    let c: i128 = dot2(dir1.x, r.x, dir1.y, r.y).raw.into();
    if degenerate2 {
        return (ratio(-c, a), ZERO, false);
    }
    let b: i128 = dot2(dir1.x, dir2.x, dir1.y, dir2.y).raw.into();
    let ae = a * e;
    let denom = ae - b * b;
    // `denom <= a e * eps`, the relative form of upstream's `denom <= eps`.
    let parallel = denom <= ae / SCALE * eps.raw.into();
    let s = if parallel {
        ZERO
    } else {
        ratio(b * f - c * e, denom)
    };
    // `t = (b s + f) / e`, numerator and denominator both at the raw Q64.64 scale.
    let t = ratio(b * s.raw.into() + f * SCALE, e * SCALE);
    (s, t, parallel)
}

/// The parameters `(s, t)` of the closest points of two lines (upstream
/// `closest_points_line_line_parameters`, tolerance `DEFAULT_EPSILON`).
/// #### Panics
/// * See [`closest_points_line_line_parameters_eps`].
pub fn closest_points_line_line_parameters(
    orig1: Vec2, dir1: Vec2, orig2: Vec2, dir2: Vec2,
) -> (Fixed, Fixed) {
    let (s, t, _) = closest_points_line_line_parameters_eps(
        orig1, dir1, orig2, dir2, DEFAULT_EPSILON,
    );
    (s, t)
}

/// The closest points `(orig1 + dir1 * s, orig2 + dir2 * t)` of two lines (upstream
/// `closest_points_line_line`).
/// #### Panics
/// * See [`closest_points_line_line_parameters_eps`]; `'Fixed: overflow'` if a point leaves the
///   scalar range.
pub fn closest_points_line_line(orig1: Vec2, dir1: Vec2, orig2: Vec2, dir2: Vec2) -> (Vec2, Vec2) {
    let (s, t) = closest_points_line_line_parameters(orig1, dir1, orig2, dir2);
    (orig1 + dir1.mul_scalar(s), orig2 + dir2.mul_scalar(t))
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, HALF, ONE, ZERO};
    use glam_core::Vec2;
    use rapier_math::consts::DEFAULT_EPSILON;
    use rapier_testing::opaque;
    use super::{
        closest_points_line_line, closest_points_line_line_parameters,
        closest_points_line_line_parameters_eps,
    };

    fn f(n: i64) -> Fixed {
        Fixed { raw: n * 0x1_0000_0000 }
    }

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: f(x), y: f(y) }
    }

    #[test]
    fn test_parameters_table() {
        // (origin1, dir1, origin2, dir2, s, t, parallel)
        let cases: Span<(Vec2, Vec2, Vec2, Vec2, Fixed, Fixed, bool)> = array![
            // Perpendicular lines crossing at (0, 0): the origins are 1 before the crossing.
            (v(-1, 0), v(1, 0), v(0, -1), v(0, 1), ONE, ONE, false),
            // Oblique: (0, 0) + s (2, 1) meets (1, 3) + t (-1, 1) at s = 0.8 * 2... exact table
            // below.
            (v(0, 0), v(1, 1), v(0, 2), v(1, -1), ONE, ONE, false),
            // Parallel: `s = 0` and `t` is the projection of the first origin.
            (v(0, 0), v(1, 0), v(3, 1), v(2, 0), ZERO, -f(3) / f(2), true),
            // Both directions zero, then each one alone.
            (v(1, 1), v(0, 0), v(2, 3), v(0, 0), ZERO, ZERO, false),
            (v(1, 1), v(0, 0), v(0, 0), v(2, 0), ZERO, HALF, false),
            (v(0, 0), v(2, 0), v(1, 1), v(0, 0), HALF, ZERO, false),
        ]
            .span();
        for case in cases {
            let (o1, d1, o2, d2, s, t, parallel) = *case;
            let (ps, pt, flag) = closest_points_line_line_parameters_eps(
                o1, d1, o2, d2, DEFAULT_EPSILON,
            );
            assert_eq!((ps, pt, flag), (s, t, parallel));
            assert_eq!(closest_points_line_line_parameters(o1, d1, o2, d2), (s, t));
        }
    }

    #[test]
    fn test_points_and_symmetry() {
        let (p1, p2) = closest_points_line_line(v(-1, 0), v(1, 0), v(0, -1), v(0, 1));
        assert_eq!((p1, p2), (v(0, 0), v(0, 0)));
        // Skew in 2D never happens; swapping the lines swaps the answers.
        let (a, b) = closest_points_line_line(v(0, 0), v(2, 1), v(1, 3), v(-1, 1));
        let (c, d) = closest_points_line_line(v(1, 3), v(-1, 1), v(0, 0), v(2, 1));
        assert_eq!((a, b), (d, c));
    }

    #[test]
    fn test_parallel_threshold_is_relative() {
        // Slope 2^-10: `sin^2 = 2^-20` is above the default `2^-23`, so not parallel...
        let slight = Vec2 { x: ONE, y: Fixed { raw: 0x40_0000 } };
        let (_, _, parallel) = closest_points_line_line_parameters_eps(
            v(0, 0), v(1, 0), v(0, 1), slight, DEFAULT_EPSILON,
        );
        assert!(!parallel);
        // ... and parallel for a looser epsilon.
        let (_, _, parallel) = closest_points_line_line_parameters_eps(
            v(0, 0), v(1, 0), v(0, 1), slight, Fixed { raw: 0x1_0000 },
        );
        assert!(parallel);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_closest_points_line_line_parameters_eps() {
        let _ = closest_points_line_line_parameters_eps(
            opaque(v(0, 0)),
            opaque(v(2, 1)),
            opaque(v(1, 3)),
            opaque(v(-1, 1)),
            opaque(DEFAULT_EPSILON),
        );
    }

    #[test]
    fn gas_closest_points_line_line_parameters() {
        let _ = closest_points_line_line_parameters(
            opaque(v(0, 0)), opaque(v(2, 1)), opaque(v(1, 3)), opaque(v(-1, 1)),
        );
    }

    #[test]
    fn gas_closest_points_line_line() {
        let _ = closest_points_line_line(
            opaque(v(0, 0)), opaque(v(2, 1)), opaque(v(1, 3)), opaque(v(-1, 1)),
        );
    }

    #[test]
    fn gas_closest_points_line_line_parallel() {
        let _ = closest_points_line_line(
            opaque(v(0, 0)), opaque(v(1, 0)), opaque(v(3, 1)), opaque(v(2, 0)),
        );
    }
}
