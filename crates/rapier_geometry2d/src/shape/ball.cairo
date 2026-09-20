//! `Ball` (Parry `shape/ball.rs`, `bounding_volume/aabb_ball.rs`, `mass_properties_ball.rs`).

use fixed::Fixed;
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::Pose2;
use crate::aabb::{Aabb, AabbTrait};
use crate::mass::{MassProperties, MassPropertiesTrait};

/// A disc of the given radius centred on the local origin.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Ball {
    pub radius: Fixed,
}

#[generate_trait]
pub impl BallImpl of BallTrait {
    /// Ball of radius `radius` (`radius >= 0`, not checked, as upstream).
    #[inline(always)]
    fn new(radius: Fixed) -> Ball {
        Ball { radius }
    }

    /// `[-r, r]^2`.
    /// #### Panics
    /// * `'i64_neg Underflow'` for a radius of `fixed::MIN`.
    #[inline(always)]
    fn compute_local_aabb(self: Ball) -> Aabb {
        let r = Vec2 { x: self.radius, y: self.radius };
        AabbTrait::new(-r, r)
    }

    /// Only the translation of `pose` matters: a ball is rotation invariant. Division-free.
    /// #### Panics
    /// * `'i64_add Overflow'` / `'i64_sub Underflow'` when a corner leaves the scalar range.
    #[inline(always)]
    fn compute_aabb(self: Ball, pose: Pose2) -> Aabb {
        let r = Vec2 { x: self.radius, y: self.radius };
        AabbTrait::from_half_extents(pose.translation, r)
    }

    /// Mass properties for `density` (`from_ball`).
    fn mass_properties(self: Ball, density: Fixed) -> MassProperties {
        MassPropertiesTrait::from_ball(density, self.radius)
    }

    /// Support point of the ball in the (normalised on the fly) direction `dir`: `r * dir / |dir|`.
    /// A zero `dir` falls back to `+Y` (upstream returns NaN).
    fn local_support_point(self: Ball, dir: Vec2) -> Vec2 {
        Self::local_support_point_toward(self, dir.normalize_or(Vec2Trait::Y))
    }

    /// Support point for a unit direction `dir`: `r * dir` (no normalisation).
    #[inline(always)]
    fn local_support_point_toward(self: Ball, dir: Vec2) -> Vec2 {
        dir.mul_scalar(self.radius)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::{Rot2, Rot2Trait};
    use rapier_testing::opaque;
    use crate::aabb::Aabb;
    use super::BallTrait;

    const QUARTER_TURN: Rot2 = Rot2 { re: ZERO, im: ONE };

    fn v(x: i32, y: i32) -> Vec2 {
        Vec2 { x: FixedTrait::from_int(x), y: FixedTrait::from_int(y) }
    }

    fn pose(x: i32, y: i32, rotation: Rot2) -> Pose2 {
        Pose2Trait::new(v(x, y), rotation)
    }

    #[test]
    fn test_aabb_table() {
        let one = Vec2 { x: HALF, y: HALF };
        // (radius, pose, mins, maxs): the rotation never matters, a zero radius gives a point.
        let cases: Span<(Fixed, Pose2, Vec2, Vec2)> = array![
            (HALF, pose(0, 0, Rot2Trait::IDENTITY), -one, one),
            (
                HALF,
                pose(3, -2, QUARTER_TURN),
                Vec2 { x: FixedTrait::from_ratio(5, 2), y: FixedTrait::from_ratio(-5, 2) },
                Vec2 { x: FixedTrait::from_ratio(7, 2), y: FixedTrait::from_ratio(-3, 2) },
            ),
            (ZERO, pose(1, 1, QUARTER_TURN), v(1, 1), v(1, 1)),
        ]
            .span();
        for (radius, pose, mins, maxs) in cases {
            let ball = BallTrait::new(*radius);
            assert_eq!(ball.compute_aabb(*pose), Aabb { mins: *mins, maxs: *maxs });
            let local = ball.compute_local_aabb();
            assert_eq!(local.mins, -Vec2 { x: *radius, y: *radius });
            assert_eq!(local.maxs, Vec2 { x: *radius, y: *radius });
        }
    }

    #[test]
    fn test_support_points() {
        let ball = BallTrait::new(FixedTrait::from_int(5));
        // Axis-aligned directions are exact whatever their length; a zero direction falls back to
        // +Y.
        assert_eq!(ball.local_support_point(v(0, 7)), v(0, 5));
        assert_eq!(ball.local_support_point(v(-2, 0)), v(-5, 0));
        assert_eq!(ball.local_support_point(v(0, 0)), v(0, 5));
        assert_eq!(ball.local_support_point_toward(v(1, 0)), v(5, 0));
        // (3, 4) / 5 * 5 within 2 ulp.
        let p = ball.local_support_point(v(3, 4));
        assert!(p.x.abs_diff_eq(FixedTrait::from_int(3), Fixed { raw: 2 }));
        assert!(p.y.abs_diff_eq(FixedTrait::from_int(4), Fixed { raw: 2 }));
    }

    #[test]
    fn test_mass_properties_delegate() {
        let ball = BallTrait::new(ONE);
        let props = ball.mass_properties(ONE);
        // mass = pi, inertia = pi / 2: inverse mass 1/pi, inverse inertia 2/pi.
        assert!(props.inv_mass.abs_diff_eq(Fixed { raw: 1367130551 }, Fixed { raw: 2 }));
        assert!(
            props.inv_principal_inertia.abs_diff_eq(Fixed { raw: 2734261102 }, Fixed { raw: 4 }),
        );
        assert_eq!(BallTrait::new(ZERO).mass_properties(ONE).inv_mass, ZERO);
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_new() {
        let _ = BallTrait::new(opaque(HALF));
    }
    #[test]
    fn gas_compute_local_aabb() {
        let _ = BallTrait::new(opaque(HALF)).compute_local_aabb();
    }
    #[test]
    fn gas_compute_aabb() {
        let _ = BallTrait::new(opaque(HALF)).compute_aabb(opaque(pose(3, -2, QUARTER_TURN)));
    }
    #[test]
    fn gas_local_support_point() {
        let _ = BallTrait::new(opaque(HALF)).local_support_point(opaque(v(3, 4)));
    }
    #[test]
    fn gas_local_support_point_toward() {
        let _ = BallTrait::new(opaque(HALF)).local_support_point_toward(opaque(v(1, 0)));
    }
}
