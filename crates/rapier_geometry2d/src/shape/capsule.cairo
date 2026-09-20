//! `Capsule` (Parry `shape/capsule.rs`, `bounding_volume/aabb_capsule.rs`,
//! `mass_properties_capsule.rs`).

use fixed::{Fixed, FixedTrait};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::Pose2;
use crate::mass::{MassProperties, MassPropertiesTrait};
use crate::shape::aabb_shim::{Aabb, segment_aabb};
use crate::shape::segment::{Segment, SegmentTrait};

/// The segment `segment` dilated by `radius`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Capsule {
    pub segment: Segment,
    /// `>= 0`, not checked, as upstream.
    pub radius: Fixed,
}

#[generate_trait]
pub impl CapsuleImpl of CapsuleTrait {
    /// Capsule of core segment `a`–`b`.
    #[inline(always)]
    fn new(a: Vec2, b: Vec2, radius: Fixed) -> Capsule {
        Capsule { segment: SegmentTrait::new(a, b), radius }
    }

    /// Capsule along X: `(-half_height, 0)`–`(half_height, 0)`.
    fn new_x(half_height: Fixed, radius: Fixed) -> Capsule {
        let zero = FixedTrait::from_raw(0);
        Self::new(Vec2 { x: -half_height, y: zero }, Vec2 { x: half_height, y: zero }, radius)
    }

    /// Capsule along Y: `(0, -half_height)`–`(0, half_height)`.
    fn new_y(half_height: Fixed, radius: Fixed) -> Capsule {
        let zero = FixedTrait::from_raw(0);
        Self::new(Vec2 { x: zero, y: -half_height }, Vec2 { x: zero, y: half_height }, radius)
    }

    /// The core segment.
    #[inline(always)]
    fn segment(self: Capsule) -> Segment {
        self.segment
    }

    /// Length of the core segment (excluding the caps).
    #[inline(always)]
    fn height(self: Capsule) -> Fixed {
        self.segment.length()
    }

    /// Midpoint of the core segment.
    #[inline(always)]
    fn center(self: Capsule) -> Vec2 {
        self.segment.a.midpoint(self.segment.b)
    }

    /// The capsule moved by `pose`.
    #[inline(always)]
    fn transform_by(self: Capsule, pose: Pose2) -> Capsule {
        Capsule { segment: self.segment.transformed(pose), radius: self.radius }
    }

    /// `[min(a, b) - r, max(a, b) + r]`.
    #[inline(always)]
    fn compute_local_aabb(self: Capsule) -> Aabb {
        segment_aabb(self.segment.a, self.segment.b, self.radius)
    }

    /// Box of the transformed core segment, grown by the radius (upstream
    /// `transform_by(pose).local_aabb()`).
    #[inline(always)]
    fn compute_aabb(self: Capsule, pose: Pose2) -> Aabb {
        let s = self.segment.transformed(pose);
        segment_aabb(s.a, s.b, self.radius)
    }

    /// Mass properties for `density` (`from_capsule`).
    fn mass_properties(self: Capsule, density: Fixed) -> MassProperties {
        MassPropertiesTrait::from_capsule(density, self.segment.a, self.segment.b, self.radius)
    }

    /// Support point along `dir`, normalised on the fly (`+Y` for a zero `dir`, as upstream).
    fn local_support_point(self: Capsule, dir: Vec2) -> Vec2 {
        Self::local_support_point_toward(self, dir.normalize_or(Vec2Trait::Y))
    }

    /// Support point for a unit `dir`: the farther end point plus `dir * radius` (`b` on a tie).
    #[inline(always)]
    fn local_support_point_toward(self: Capsule, dir: Vec2) -> Vec2 {
        let end = if (self.segment.a - self.segment.b).dot(dir) > FixedTrait::from_raw(0) {
            self.segment.a
        } else {
            self.segment.b
        };
        end + dir.mul_scalar(self.radius)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::{Rot2, Rot2Trait};
    use rapier_testing::opaque;
    use crate::shape::aabb_shim::Aabb;
    use super::CapsuleTrait;

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn i(n: i32) -> Fixed {
        FixedTrait::from_int(n)
    }

    /// A 45 degree pose built from literals: no runtime normalisation inside a gas probe.
    fn probe_pose() -> Pose2 {
        Pose2 {
            translation: v(ONE, ONE),
            rotation: Rot2 { re: Fixed { raw: 3037000500 }, im: Fixed { raw: 3037000500 } },
        }
    }

    fn pose(x: Fixed, y: Fixed, re: Fixed, im: Fixed) -> Pose2 {
        Pose2Trait::new(v(x, y), Rot2Trait::from_cos_sin(re, im))
    }

    #[test]
    fn test_constructors_height_center() {
        let y = CapsuleTrait::new_y(i(1), HALF);
        assert_eq!(y.segment(), CapsuleTrait::new(v(ZERO, i(-1)), v(ZERO, i(1)), HALF).segment);
        assert_eq!((y.height(), y.center()), (i(2), v(ZERO, ZERO)));
        let x = CapsuleTrait::new_x(HALF, HALF);
        assert_eq!((x.height(), x.center()), (ONE, v(ZERO, ZERO)));
        let oblique = CapsuleTrait::new(v(i(-2), i(-1)), v(i(4), i(3)), HALF);
        assert_eq!(oblique.center(), v(i(1), i(1)));
        // Zero-length capsule: a disc.
        let disc = CapsuleTrait::new(v(ONE, ONE), v(ONE, ONE), HALF);
        assert_eq!((disc.height(), disc.center()), (ZERO, v(ONE, ONE)));
    }

    #[test]
    fn test_aabb_table() {
        let capsule = CapsuleTrait::new(v(i(-2), i(-1)), v(i(4), i(3)), HALF);
        let local = capsule.compute_local_aabb();
        assert_eq!(
            local,
            Aabb {
                mins: v(FixedTrait::from_ratio(-5, 2), FixedTrait::from_ratio(-3, 2)),
                maxs: v(FixedTrait::from_ratio(9, 2), FixedTrait::from_ratio(7, 2)),
            },
        );
        // Identity, quarter turn (exact) with a translation, half turn.
        let cases: Span<(Pose2, Aabb)> = array![
            (pose(ZERO, ZERO, ONE, ZERO), local),
            (
                pose(i(10), i(-10), ZERO, ONE),
                Aabb {
                    mins: v(FixedTrait::from_ratio(13, 2), FixedTrait::from_ratio(-25, 2)),
                    maxs: v(FixedTrait::from_ratio(23, 2), FixedTrait::from_ratio(-11, 2)),
                },
            ),
            (
                pose(ZERO, ZERO, -ONE, ZERO),
                Aabb {
                    mins: v(FixedTrait::from_ratio(-9, 2), FixedTrait::from_ratio(-7, 2)),
                    maxs: v(FixedTrait::from_ratio(5, 2), FixedTrait::from_ratio(3, 2)),
                },
            ),
        ]
            .span();
        for (p, expected) in cases {
            assert_eq!(capsule.compute_aabb(*p), *expected);
        }
        // Zero radius: the segment box; zero-length zero radius: a point.
        let segment = CapsuleTrait::new(v(ONE, i(2)), v(i(3), ZERO), ZERO).compute_local_aabb();
        assert_eq!((segment.mins, segment.maxs), (v(ONE, ZERO), v(i(3), i(2))));
        let point = CapsuleTrait::new(v(ONE, ONE), v(ONE, ONE), ZERO)
            .compute_aabb(pose(ONE, ZERO, ZERO, ONE));
        assert_eq!((point.mins, point.maxs), (v(ZERO, ONE), v(ZERO, ONE)));
    }

    #[test]
    fn test_support_points() {
        let x = CapsuleTrait::new_x(i(2), HALF);
        // Axis-aligned directions are exact whatever their length; a tie picks `b`; zero -> +Y.
        assert_eq!(x.local_support_point(v(i(3), ZERO)), v(FixedTrait::from_ratio(5, 2), ZERO));
        assert_eq!(x.local_support_point(v(i(-3), ZERO)), v(FixedTrait::from_ratio(-5, 2), ZERO));
        assert_eq!(x.local_support_point(v(ZERO, i(1))), v(i(2), HALF));
        assert_eq!(x.local_support_point(v(ZERO, ZERO)), v(i(2), HALF));
        assert_eq!(x.local_support_point_toward(v(ZERO, -ONE)), v(i(2), -HALF));
    }

    #[test]
    fn test_transform_by_moves_the_core() {
        let moved = CapsuleTrait::new_x(i(2), HALF).transform_by(pose(ONE, ZERO, ZERO, ONE));
        assert_eq!(
            (moved.segment.a, moved.segment.b, moved.radius), (v(ONE, i(-2)), v(ONE, i(2)), HALF),
        );
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_new() {
        let _ = CapsuleTrait::new(opaque(v(ZERO, i(-1))), opaque(v(ZERO, i(1))), opaque(HALF));
    }
    #[test]
    fn gas_height() {
        let _ = opaque(CapsuleTrait::new_y(i(1), HALF)).height();
    }
    #[test]
    fn gas_center() {
        let _ = opaque(CapsuleTrait::new_y(i(1), HALF)).center();
    }
    #[test]
    fn gas_compute_local_aabb() {
        let _ = opaque(CapsuleTrait::new_y(i(1), HALF)).compute_local_aabb();
    }
    #[test]
    fn gas_compute_aabb() {
        let _ = opaque(CapsuleTrait::new_y(i(1), HALF)).compute_aabb(opaque(probe_pose()));
    }
    #[test]
    fn gas_local_support_point() {
        let _ = opaque(CapsuleTrait::new_y(i(1), HALF)).local_support_point(opaque(v(ONE, HALF)));
    }
    #[test]
    fn gas_local_support_point_toward() {
        let _ = opaque(CapsuleTrait::new_y(i(1), HALF))
            .local_support_point_toward(opaque(v(ONE, ZERO)));
    }
    #[test]
    fn gas_mass_properties() {
        let _ = opaque(CapsuleTrait::new_y(i(1), HALF)).mass_properties(opaque(ONE));
    }
    #[test]
    fn gas_transform_by() {
        let _ = opaque(CapsuleTrait::new_y(i(1), HALF)).transform_by(opaque(probe_pose()));
    }
}
