//! `HalfSpace` (Parry `shape/half_space.rs`, `bounding_volume/aabb_halfspace.rs`).

use core::num::traits::Zero;
use fixed::Fixed;
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::Pose2;
use crate::aabb::bounding_volume::{BoundingSphere, UNBOUNDED_RADIUS, centered_bounding_sphere};
use crate::aabb::{Aabb, AabbTrait};
use crate::mass::MassProperties;

/// Raw of `Vector::MAX * 0.5`: half of `fixed::MAX`, the extent of the AABB of a half-space.
const HALF_MAX_RAW: i64 = 0x3fffffffffffffff;

/// The region behind the plane through the local origin with outward normal `normal`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct HalfSpace {
    /// Outward unit normal (not checked, as upstream).
    pub normal: Vec2,
}

#[generate_trait]
pub impl HalfSpaceImpl of HalfSpaceTrait {
    /// Half-space with outward `normal`, expected unit.
    #[inline(always)]
    fn new(normal: Vec2) -> HalfSpace {
        HalfSpace { normal }
    }

    /// `[-MAX/2, MAX/2]^2`: effectively unbounded, yet leaves room to loosen and merge.
    #[inline(always)]
    fn compute_local_aabb(self: HalfSpace) -> Aabb {
        let max = Fixed { raw: HALF_MAX_RAW };
        let min = Fixed { raw: -HALF_MAX_RAW };
        AabbTrait::new(Vec2 { x: min, y: min }, Vec2 { x: max, y: max })
    }

    /// The pose is ignored, as upstream: the box is the same "everything" box.
    #[inline(always)]
    fn compute_aabb(self: HalfSpace, pose: Pose2) -> Aabb {
        Self::compute_local_aabb(self)
    }

    /// Upstream name of [`HalfSpaceTrait::compute_local_aabb`].
    #[inline(always)]
    fn local_aabb(self: HalfSpace) -> Aabb {
        Self::compute_local_aabb(self)
    }

    /// Upstream name of [`HalfSpaceTrait::compute_aabb`] (the pose is ignored).
    #[inline(always)]
    fn aabb(self: HalfSpace, pose: Pose2) -> Aabb {
        Self::compute_local_aabb(self)
    }

    /// Unbounded: centred on the origin with radius `fixed::MAX` (upstream `Real::max_value()`).
    #[inline(always)]
    fn local_bounding_sphere(self: HalfSpace) -> BoundingSphere {
        BoundingSphere { center: Vec2Trait::ZERO, radius: UNBOUNDED_RADIUS }
    }

    /// The unbounded sphere centred on the translation of `pose`.
    #[inline(always)]
    fn bounding_sphere(self: HalfSpace, pose: Pose2) -> BoundingSphere {
        centered_bounding_sphere(pose, UNBOUNDED_RADIUS)
    }

    /// The half-space with normal `normal * scale` renormalised (rounded to nearest), `None`
    /// when the scaled normal is zero.
    fn scaled(self: HalfSpace, scale: Vec2) -> Option<HalfSpace> {
        match (self.normal * scale).try_normalize() {
            Some(normal) => Some(HalfSpace { normal }),
            None => None,
        }
    }

    /// Zero: a half-space has infinite mass, which the inverse-mass representation encodes as 0.
    #[inline(always)]
    fn mass_properties(self: HalfSpace, density: Fixed) -> MassProperties {
        Zero::<MassProperties>::zero()
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::Pose2Trait;
    use rapier_math::rot2::Rot2Trait;
    use rapier_testing::opaque;
    use super::{HALF_MAX_RAW, HalfSpaceTrait};

    fn up() -> Vec2 {
        Vec2 { x: ZERO, y: ONE }
    }

    #[test]
    fn test_aabb_is_everything_and_ignores_the_pose() {
        let h = HalfSpaceTrait::new(up());
        let big = Fixed { raw: HALF_MAX_RAW };
        let local = h.compute_local_aabb();
        assert_eq!(local.maxs, Vec2 { x: big, y: big });
        assert_eq!(local.mins, Vec2 { x: -big, y: -big });
        // Half of `fixed::MAX`: room to loosen and merge without overflow.
        assert_eq!(big + big + Fixed { raw: 1 }, FixedTrait::from_raw(0x7fffffffffffffff));
        let pose = Pose2Trait::new(Vec2 { x: ONE, y: ONE }, Rot2Trait::from_cos_sin(ZERO, ONE));
        assert_eq!(h.compute_aabb(pose), local);
    }

    #[test]
    fn test_mass_properties_are_zero() {
        assert_eq!(HalfSpaceTrait::new(up()).mass_properties(ONE), Default::default());
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_compute_aabb() {
        let _ = HalfSpaceTrait::new(opaque(up())).compute_aabb(opaque(Pose2Trait::IDENTITY));
    }
    #[test]
    fn gas_compute_local_aabb() {
        let _ = HalfSpaceTrait::new(opaque(up())).compute_local_aabb();
    }
}
