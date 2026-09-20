//! Private stand-in for `rapier_geometry2d::aabb` (work package GA, not merged yet).
//!
//! Only what `compute_aabb` needs: the `Aabb` value type with GA's exact layout and derives, its
//! `new` / `from_half_extents` constructors, and the two kernels behind the shape AABBs. When GA
//! lands, replace `use crate::shape::aabb_shim::{Aabb, AabbTrait}` by `use crate::aabb::{Aabb,
//! AabbTrait}` in the shape modules and delete this file (listed under "Escalations" in the PR).

use fixed::wide::dot2;
use fixed::{Fixed, FixedTrait};
use glam::Vec2;
use rapier_math::pose2::Pose2;

/// Axis-aligned bounding box (`mins <= maxs` component-wise for a valid box).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct Aabb {
    pub mins: Vec2,
    pub maxs: Vec2,
}

#[generate_trait]
pub impl AabbImpl of AabbTrait {
    /// Box from its corners; no validation (upstream `Aabb::new`).
    #[inline(always)]
    fn new(mins: Vec2, maxs: Vec2) -> Aabb {
        Aabb { mins, maxs }
    }

    /// Box `[center - half_extents, center + half_extents]` (upstream `Aabb::from_half_extents`).
    /// #### Panics
    /// * `'i64_add Overflow'` / `'i64_sub Underflow'` when a corner leaves the scalar range.
    #[inline(always)]
    fn from_half_extents(center: Vec2, half_extents: Vec2) -> Aabb {
        Aabb {
            mins: Vec2 { x: center.x - half_extents.x, y: center.y - half_extents.y },
            maxs: Vec2 { x: center.x + half_extents.x, y: center.y + half_extents.y },
        }
    }
}

/// `|R| * h`: the half extents of the box of half extents `h` rotated by `pose` (upstream
/// `PoseOps::absolute_transform_vector`). One rescale per component, so an exact quarter turn
/// gives an exact box.
/// #### Panics
/// * `'i64_neg Underflow'` for a rotation component equal to `fixed::MIN` (not unit).
/// * `'Fixed: overflow'` if a component of the result leaves the scalar range.
#[inline(always)]
pub fn absolute_transform_vector(pose: Pose2, h: Vec2) -> Vec2 {
    let c = pose.rotation.re.abs();
    let s = pose.rotation.im.abs();
    Vec2 { x: dot2(c, h.x, s, h.y), y: dot2(s, h.x, c, h.y) }
}

/// Box of the segment `a`–`b` grown by `margin` on every side.
/// #### Panics
/// * `'i64_add Overflow'` / `'i64_sub Underflow'` when a corner leaves the scalar range.
#[inline(always)]
pub fn segment_aabb(a: Vec2, b: Vec2, margin: Fixed) -> Aabb {
    let (min_x, max_x) = if a.x < b.x {
        (a.x, b.x)
    } else {
        (b.x, a.x)
    };
    let (min_y, max_y) = if a.y < b.y {
        (a.y, b.y)
    } else {
        (b.y, a.y)
    };
    Aabb {
        mins: Vec2 { x: min_x - margin, y: min_y - margin },
        maxs: Vec2 { x: max_x + margin, y: max_y + margin },
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::Pose2Trait;
    use rapier_math::rot2::{Rot2, Rot2Trait};
    use rapier_testing::opaque;
    use super::{Aabb, AabbTrait, absolute_transform_vector, segment_aabb};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    #[test]
    fn test_constructors() {
        let b = AabbTrait::from_half_extents(v(ONE, ZERO), v(HALF, ONE));
        assert_eq!(b, AabbTrait::new(v(HALF, -ONE), v(ONE + HALF, ONE)));
        assert_eq!(Default::<Aabb>::default(), AabbTrait::new(v(ZERO, ZERO), v(ZERO, ZERO)));
    }

    #[test]
    fn test_absolute_transform_vector() {
        let h = v(FixedTrait::from_int(2), ONE);
        let turn = |
            re: Fixed, im: Fixed,
        | Pose2Trait::new(v(ZERO, ZERO), Rot2Trait::from_cos_sin(re, im));
        assert_eq!(absolute_transform_vector(turn(ONE, ZERO), h), h);
        assert_eq!(absolute_transform_vector(turn(ZERO, ONE), h), v(ONE, FixedTrait::from_int(2)));
        assert_eq!(absolute_transform_vector(turn(-ONE, ZERO), h), h);
        assert_eq!(absolute_transform_vector(turn(ZERO, -ONE), h), v(ONE, FixedTrait::from_int(2)));
    }

    #[test]
    fn test_segment_aabb_orders_end_points() {
        let (a, b) = (v(ONE, -ONE), v(-ONE, ONE));
        assert_eq!(segment_aabb(a, b, HALF), segment_aabb(b, a, HALF));
        assert_eq!(segment_aabb(a, b, ZERO), AabbTrait::new(v(-ONE, -ONE), v(ONE, ONE)));
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_absolute_transform_vector() {
        let rotation = Rot2 { re: Fixed { raw: 3037000500 }, im: Fixed { raw: 3037000500 } };
        let p = Pose2Trait::new(opaque(v(ONE, ONE)), opaque(rotation));
        let _ = absolute_transform_vector(p, opaque(v(ONE, HALF)));
    }
    #[test]
    fn gas_segment_aabb() {
        let _ = segment_aabb(opaque(v(ONE, -ONE)), opaque(v(-ONE, ONE)), opaque(HALF));
    }
    #[test]
    fn gas_from_half_extents() {
        let _ = AabbTrait::from_half_extents(opaque(v(ONE, ZERO)), opaque(v(HALF, ONE)));
    }
}
