//! Ball–ball shape cast (Parry `shape_cast_ball_ball.rs`): a ray cast from the origin along
//! `vel12` on the circle of radius `r1 + r2 + target_distance` centred at `-pos12.translation`,
//! with the exact circle kernel of `crate::ray::ball` (exact coefficients, one wide square root,
//! one correctly rounded quotient).

use fixed::ZERO;
use glam::{Vec2, Vec2Trait};
use rapier_math::math_ext::norm2::is_norm2_lt;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2Trait;
use crate::point::wide2::dot_wide;
use crate::ray::ball::ray_toi_with_ball;
use crate::ray::{Ray, RayTrait};
use crate::shape::Ball;
use super::super::X;
use super::{ShapeCastHit, ShapeCastOptions, ShapeCastStatus, TOI_AT_START};

/// The first impact of ball `b2`, placed at `pos12` and moving at `vel12`, with ball `b1`
/// (upstream `cast_shapes_ball_ball`).
///
/// A start within the inflated circle answers `t = 0` (a zero `vel12` included); `normal1 =
/// (hit point - centre) / radius` is the outward normal of the inflated circle, `+X` for a zero
/// radius. With `stop_at_penetration = false`, an impact below `1e-4` whose normal velocity is
/// not approaching (`normal1 . vel12 >= 0`, exact) is dropped.
/// #### Panics
/// * `'Fixed: overflow'` if the radius sum or the hit point leaves the scalar range; see
///   `crate::ray::ball::ray_toi_with_ball`.
pub fn cast_shapes_ball_ball(
    pos12: Pose2, vel12: Vec2, b1: Ball, b2: Ball, options: ShapeCastOptions,
) -> Option<ShapeCastHit> {
    let radius = b1.radius + b2.radius + options.target_distance;
    let center = -pos12.translation;
    let ray = Ray { origin: Vec2 { x: ZERO, y: ZERO }, dir: vel12 };
    let (inside, toi) = ray_toi_with_ball(center, radius, ray, true);
    let time_of_impact = toi?;
    if time_of_impact > options.max_time_of_impact {
        return None;
    }
    let (normal1, normal2, witness1, witness2) = if radius == ZERO {
        (X, pos12.rotation.inverse_rotate(-X), Vec2 { x: ZERO, y: ZERO }, Vec2 { x: ZERO, y: ZERO })
    } else {
        let dpt = ray.point_at(time_of_impact) - center;
        let normal1 = Vec2 { x: dpt.x / radius, y: dpt.y / radius };
        let normal2 = pos12.rotation.inverse_rotate(-normal1);
        (normal1, normal2, normal1.mul_scalar(b1.radius), normal2.mul_scalar(b2.radius))
    };
    if !options.stop_at_penetration
        && time_of_impact < TOI_AT_START
        && dot_wide(normal1.x, normal1.y, vel12.x, vel12.y) >= 0 {
        return None;
    }
    let status = if inside && is_norm2_lt(center.x, center.y, radius) {
        ShapeCastStatus::PenetratingOrWithinTargetDist
    } else {
        ShapeCastStatus::Converged
    };
    Some(ShapeCastHit { time_of_impact, witness1, witness2, normal1, normal2, status })
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, MAX, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::Pose2Trait;
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::shape::BallTrait;
    use super::cast_shapes_ball_ball;
    use super::super::{ShapeCastOptions, ShapeCastStatus};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    fn options(max: Fixed, target: Fixed, stop: bool) -> ShapeCastOptions {
        ShapeCastOptions {
            max_time_of_impact: max,
            target_distance: target,
            stop_at_penetration: stop,
            compute_impact_geometry_on_penetration: true,
        }
    }

    /// `(x of ball 2, vx of ball 2, max, target, stop) -> (hit, toi, status)`, radii 0.5 each.
    #[test]
    fn test_ball_ball_table() {
        let quarter = Rot2 { re: ZERO, im: ONE };
        let cases = array![
            // Head-on from x = 4 at -1: impact at 3.
            (int(4), -ONE, MAX, ZERO, true, true, int(3), ShapeCastStatus::Converged),
            // Moving away.
            (int(4), ONE, MAX, ZERO, true, false, ZERO, ShapeCastStatus::Converged),
            // Beyond max_time_of_impact, then exactly at it (inclusive).
            (int(4), -ONE, TWO, ZERO, true, false, ZERO, ShapeCastStatus::Converged),
            (int(4), -ONE, int(3), ZERO, true, true, int(3), ShapeCastStatus::Converged),
            // Target distance 0.5: impact half a unit earlier.
            (int(4), -ONE, MAX, HALF, true, true, int(3) - HALF, ShapeCastStatus::Converged),
            // Touching at the start: t = 0, converged.
            (ONE, -ONE, MAX, ZERO, true, true, ZERO, ShapeCastStatus::Converged),
            // Penetrating: t = 0, penetrating, whatever the motion.
            (
                HALF,
                ONE,
                MAX,
                ZERO,
                true,
                true,
                ZERO,
                ShapeCastStatus::PenetratingOrWithinTargetDist,
            ),
            (
                HALF,
                ZERO,
                MAX,
                ZERO,
                true,
                true,
                ZERO,
                ShapeCastStatus::PenetratingOrWithinTargetDist,
            ),
            // Not stopping at penetration: separating motion drops the hit, approaching keeps it.
            (HALF, ONE, MAX, ZERO, false, false, ZERO, ShapeCastStatus::Converged),
            (
                HALF,
                -ONE,
                MAX,
                ZERO,
                false,
                true,
                ZERO,
                ShapeCastStatus::PenetratingOrWithinTargetDist,
            ),
        ];
        for (x, vx, max, target, stop, hit, toi, status) in cases {
            let pos12 = Pose2Trait::new(v(x, ZERO), quarter);
            let answer = cast_shapes_ball_ball(
                pos12,
                v(vx, ZERO),
                BallTrait::new(HALF),
                BallTrait::new(HALF),
                options(max, target, stop),
            );
            assert_eq!(answer.is_some(), hit, "{:?} {:?}", x, vx);
            if let Some(h) = answer {
                assert_eq!(h.time_of_impact, toi);
                assert_eq!(h.status, status);
                if x > HALF {
                    // Head-on impacts: normal +X in frame 1, -X in the quarter-turned frame 2.
                    assert_eq!(h.normal1, v(ONE, ZERO));
                    assert_eq!(h.normal2, v(ZERO, ONE));
                    assert_eq!(h.witness1, v(HALF, ZERO));
                    assert_eq!(h.witness2, v(ZERO, HALF));
                }
            }
        }
    }

    /// Two points (zero radii) meeting: `+X` normal, zero witnesses.
    #[test]
    fn test_ball_ball_zero_radius() {
        let pos12 = Pose2Trait::new(v(TWO, ZERO), Rot2 { re: ONE, im: ZERO });
        let hit = cast_shapes_ball_ball(
            pos12, v(-ONE, ZERO), BallTrait::new(ZERO), BallTrait::new(ZERO), Default::default(),
        )
            .unwrap();
        assert_eq!(hit.time_of_impact, TWO);
        assert_eq!((hit.normal1, hit.normal2), (v(ONE, ZERO), v(-ONE, ZERO)));
        assert_eq!((hit.witness1, hit.witness2), (v(ZERO, ZERO), v(ZERO, ZERO)));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_cast_shapes_ball_ball() {
        let _ = cast_shapes_ball_ball(
            opaque(Pose2Trait::new(v(int(4), ONE), Rot2 { re: ONE, im: ZERO })),
            opaque(v(-ONE, ZERO)),
            opaque(BallTrait::new(HALF)),
            opaque(BallTrait::new(ONE)),
            opaque(Default::default()),
        );
    }
}
