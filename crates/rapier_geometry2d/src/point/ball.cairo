//! Point queries on a [`Ball`] (Parry `query/point/point_ball.rs`).
//!
//! # Candidates
//!
//! Ranked by the `gas_*` probes of this module; the rejected one lives in
//! `#[cfg(test)] mod alternatives`.
//!
//! 1. **normalise, then scale** (this module): `normalize(pt) * radius`, two `Recip::mul` and two
//!    products. **Winner** on robustness — it is defined for every non-zero `pt`.
//! 2. `alternatives::project_scale_first`: upstream's literal `pt * (radius / |pt|)`, one
//!    `Recip::mul` less. **Wrong** near the centre: `radius / |pt|` leaves the scalar range as
//!    soon as `|pt| < radius * 2^-31`, where the winner is exact.

use fixed::wide::{NormTrait, RecipTrait, norm2, norm2_wide};
use fixed::{Fixed, ZERO};
use glam::vec2::Vec2;
use rapier_math::math_ext::norm2::is_norm2_le;
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::shape::Ball;
use super::PointProjection;

/// Projects `pt` on `ball`.
///
/// Mirrors `PointQuery::project_local_point` for `Ball`. A point at distance exactly `radius`
/// counts as inside. With `solid = true` an inside point projects to itself; with `solid = false`
/// it is pushed to the circle.
/// #### Panics
/// * `'Fixed: overflow'` if a projected component leaves the scalar range (`radius` close to
///   `2^31`).
/// #### Deviations
/// * Upstream evaluates `pt * (radius / sqrt(|pt|^2))`, which is `0 * inf = NaN` at the exact
///   centre. Here the centre projects to `(radius, 0)` — the `+x` point of the circle — with
///   `is_inside = true`. Any other point of the circle would be as good; the rule is
///   deterministic, which upstream's is not.
/// * The inside test compares the squared length wide (`rapier_math::math_ext::norm2`), so it is
///   exact for radii below `2^-16`, where `radius * radius` rescales to 0.
/// * The projected point is `normalize(pt) * radius`: each component is within about
///   `1 ulp / |pt| + 2 ulp` of the exact projection.
pub fn project_local_point_ball(ball: Ball, pt: Vec2, solid: bool) -> PointProjection {
    let is_inside = is_norm2_le(pt.x, pt.y, ball.radius);
    if is_inside && solid {
        return PointProjection { is_inside: true, point: pt };
    }
    match norm2_wide(pt.x, pt.y).try_recip() {
        Some(r) => PointProjection {
            is_inside, point: Vec2 { x: r.mul(pt.x) * ball.radius, y: r.mul(pt.y) * ball.radius },
        },
        None => PointProjection { is_inside: true, point: Vec2 { x: ball.radius, y: ZERO } },
    }
}

/// Projects `pt` on `ball` and names the feature it landed on.
///
/// Mirrors `PointQuery::project_local_point_and_get_feature` for `Ball`: a circle has a single
/// face, so the answer is always `Face(0)` and the projection is the `solid = false` one.
/// #### Panics
/// * See [`project_local_point_ball`].
/// #### Deviations
/// * None.
#[inline(always)]
pub fn project_local_point_and_get_feature_ball(
    ball: Ball, pt: Vec2,
) -> (PointProjection, FeatureId) {
    (project_local_point_ball(ball, pt, false), FeatureIdTrait::face(0))
}

/// Returns the distance from `pt` to `ball`: `|pt| - radius`, negative inside when
/// `solid = false`, clamped to 0 inside when `solid = true`.
///
/// Mirrors `PointQuery::distance_to_local_point` for `Ball`.
/// #### Panics
/// * `'Fixed: overflow'` if `|pt|` itself leaves the scalar range (`|pt| >= 2^31`).
/// #### Deviations
/// * `fixed::wide::norm2` floors the length, so the result is at most 1 ulp below the exact
///   distance. It is computed from the raw Q64.64 sum of squares: no rescale, hence no underflow
///   for a point a few ulp from the centre.
pub fn distance_to_local_point_ball(ball: Ball, pt: Vec2, solid: bool) -> Fixed {
    let dist = norm2(pt.x, pt.y) - ball.radius;
    if solid && dist < ZERO {
        ZERO
    } else {
        dist
    }
}

/// Returns `true` when `pt` is inside `ball`, boundary included.
///
/// Mirrors `PointQuery::contains_local_point` for `Ball` (`|pt|^2 <= radius^2`).
/// #### Panics
/// * `'i128_add Overflow'` for `pt = (fixed::MIN, fixed::MIN)`.
/// #### Deviations
/// * The comparison is wide, so it is exact for every radius instead of degenerating below
///   `2^-16`.
#[inline(always)]
pub fn contains_local_point_ball(ball: Ball, pt: Vec2) -> bool {
    is_norm2_le(pt.x, pt.y, ball.radius)
}

/// Rejected candidates, kept for the `gas_*` ranking.
#[cfg(test)]
pub mod alternatives {
    use fixed::wide::{NormTrait, RecipTrait, norm2_wide};
    use glam::vec2::Vec2;
    use rapier_math::math_ext::norm2::is_norm2_le;
    use crate::shape::Ball;
    use super::super::PointProjection;

    /// Upstream's literal `pt * (radius / |pt|)`: one `Recip::mul` instead of two, but the scale
    /// `radius / |pt|` overflows the scalar range for a point closer than `radius * 2^-31` to
    /// the centre, where the shipped version still answers.
    pub fn project_scale_first(ball: Ball, pt: Vec2, solid: bool) -> PointProjection {
        let is_inside = is_norm2_le(pt.x, pt.y, ball.radius);
        if is_inside && solid {
            return PointProjection { is_inside: true, point: pt };
        }
        match norm2_wide(pt.x, pt.y).try_recip() {
            Some(r) => {
                let scale = r.mul(ball.radius);
                PointProjection { is_inside, point: Vec2 { x: pt.x * scale, y: pt.y * scale } }
            },
            None => PointProjection {
                is_inside: true, point: Vec2 { x: ball.radius, y: fixed::ZERO },
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use crate::feature_id::FeatureIdTrait;
    use crate::shape::Ball;
    use super::alternatives::project_scale_first;
    use super::super::PointProjection;
    use super::{
        contains_local_point_ball, distance_to_local_point_ball,
        project_local_point_and_get_feature_ball, project_local_point_ball,
    };

    const UNIT: i64 = 0x1_0000_0000;

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }
    }

    fn ball(radius: i64) -> Ball {
        Ball { radius: Fixed { raw: radius } }
    }

    #[test]
    fn test_projection_table() {
        // (radius, point, solid, expected point, expected is_inside)
        let cases: Span<(i64, Vec2, bool, Vec2, bool)> = array![
            // Outside: both flags agree.
            (UNIT, v(2 * UNIT, 0), false, v(UNIT, 0), false),
            (UNIT, v(2 * UNIT, 0), true, v(UNIT, 0), false),
            // Exactly on the boundary counts as inside.
            (UNIT, v(UNIT, 0), false, v(UNIT, 0), true),
            (UNIT, v(0, -UNIT), true, v(0, -UNIT), true),
            // Inside: solid keeps the point, hollow pushes it to the circle.
            (2 * UNIT, v(UNIT, 0), true, v(UNIT, 0), true),
            (2 * UNIT, v(UNIT, 0), false, v(2 * UNIT, 0), true),
            // A point a few ulp from the centre: the wide length keeps the direction.
            (UNIT, v(4096, 0), false, v(UNIT, 0), true), (UNIT, v(0, 3), false, v(0, UNIT), true),
            // The exact centre: deterministic +x fallback.
            (UNIT, v(0, 0), false, v(UNIT, 0), true), (UNIT, v(0, 0), true, v(0, 0), true),
            // A radius below one raw unit still classifies exactly.
            (1, v(0, 2), false, v(0, 1), false),
        ]
            .span();
        for (radius, pt, solid, point, is_inside) in cases {
            assert_eq!(
                project_local_point_ball(ball(*radius), *pt, *solid),
                PointProjection { is_inside: *is_inside, point: *point },
            );
        }
    }

    #[test]
    fn test_distance_and_containment() {
        // (radius, point, expected distance (solid = false), contains)
        let cases: Span<(i64, Vec2, i64, bool)> = array![
            (UNIT, v(2 * UNIT, 0), UNIT, false), (UNIT, v(UNIT, 0), 0, true),
            (UNIT, v(0, 0), -UNIT, true), (2 * UNIT, v(0, UNIT), -UNIT, true),
            (UNIT, v(3 * UNIT, 4 * UNIT), 4 * UNIT, false),
        ]
            .span();
        for (radius, pt, expected, contains) in cases {
            assert_eq!(
                distance_to_local_point_ball(ball(*radius), *pt, false), Fixed { raw: *expected },
            );
            assert_eq!(contains_local_point_ball(ball(*radius), *pt), *contains);
            // `solid = true` never reports a negative distance.
            assert!(distance_to_local_point_ball(ball(*radius), *pt, true) >= ZERO);
        }
    }

    /// The rejected candidate loses accuracy far from the centre, which is why it is rejected.
    #[test]
    fn test_scale_first_candidate_drifts_far_from_the_centre() {
        let b = ball(UNIT);
        let pt = v(21981 * UNIT, -30201 * UNIT);
        let shipped = project_local_point_ball(b, pt, false).point;
        let other = project_scale_first(b, pt, false).point;
        assert!(!shipped.x.abs_diff_eq(other.x, Fixed { raw: 4096 }));
        // The shipped one still lands on the circle.
        let len = fixed::wide::norm2(shipped.x, shipped.y);
        assert!(len.abs_diff_eq(b.radius, Fixed { raw: 4 }));
    }

    #[test]
    fn test_feature_is_always_face_zero() {
        let (proj, feature) = project_local_point_and_get_feature_ball(ball(UNIT), v(2 * UNIT, 0));
        assert_eq!(feature, FeatureIdTrait::face(0));
        assert_eq!(proj.point, v(UNIT, 0));
        let (proj, feature) = project_local_point_and_get_feature_ball(ball(2 * UNIT), v(0, 0));
        assert_eq!(feature, FeatureIdTrait::face(0));
        assert!(proj.is_inside);
    }

    /// The projection lies on the circle. The length of `pt` is floored to 1 ulp before the
    /// division, so the radius it comes back with carries up to `radius / |pt|` ulp of error;
    /// the fuzz keeps `|pt| >= 1` and `radius <= 256`, which bounds it by 258 ulp.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_projection_lies_on_the_circle(x: i16, y: i16, radius: u8) {
        if x == 0 && y == 0 {
            return;
        }
        let b = Ball { radius: FixedTrait::from_int(radius.into()) + ONE };
        let pt = v(x.into() * UNIT, y.into() * UNIT);
        let proj = project_local_point_ball(b, pt, false).point;
        let len = fixed::wide::norm2(proj.x, proj.y);
        assert!(len.abs_diff_eq(b.radius, Fixed { raw: 512 }));
    }

    /// Both candidates agree while the query point stays within a couple of units of the
    /// centre. Further out they must not: `radius / |pt|` is rounded to 1 ulp before it
    /// multiplies `pt`, so the rejected candidate loses `|pt| / 2` ulp per component — a
    /// thousandth of a unit at `|pt| = 4000` — while the shipped one loses `radius / 2` ulp.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_candidates_agree(x: i8, y: i8) {
        if x == 0 && y == 0 {
            return;
        }
        let b = ball(UNIT);
        let pt = v(x.into() * 0x400_0000, y.into() * 0x400_0000);
        let a = project_local_point_ball(b, pt, false);
        let c = project_scale_first(b, pt, false);
        assert_eq!(a.is_inside, c.is_inside);
        assert!(a.point.x.abs_diff_eq(c.point.x, Fixed { raw: 4 }));
        assert!(a.point.y.abs_diff_eq(c.point.y, Fixed { raw: 4 }));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_project_ball_outside() {
        let _ = project_local_point_ball(opaque(ball(UNIT)), opaque(v(2 * UNIT, UNIT)), false);
    }

    #[test]
    fn gas_project_ball_inside_solid() {
        let _ = project_local_point_ball(opaque(ball(4 * UNIT)), opaque(v(UNIT, UNIT)), true);
    }

    #[test]
    fn gas_project_ball_scale_first() {
        let _ = project_scale_first(opaque(ball(UNIT)), opaque(v(2 * UNIT, UNIT)), false);
    }

    #[test]
    fn gas_project_and_get_feature_ball() {
        let _ = project_local_point_and_get_feature_ball(
            opaque(ball(UNIT)), opaque(v(2 * UNIT, UNIT)),
        );
    }

    #[test]
    fn gas_distance_to_local_point_ball() {
        let _ = distance_to_local_point_ball(opaque(ball(UNIT)), opaque(v(2 * UNIT, UNIT)), false);
    }

    #[test]
    fn gas_contains_local_point_ball() {
        let _ = contains_local_point_ball(opaque(ball(UNIT)), opaque(v(2 * UNIT, UNIT)));
    }
}
