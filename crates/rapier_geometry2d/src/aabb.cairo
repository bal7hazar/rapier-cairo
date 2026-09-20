//! Axis-aligned bounding boxes (Parry `Aabb`) for 2D broad-phase and shape bounds.
//!
//! Intersections and containment use closed intervals, so touching edges and corners overlap.
//! `transform_by` uses Parry's absolute-rotation formula; the four-corner implementation is kept
//! in the test-only alternatives for equivalence checks and gas ranking.

use fixed::wide::dot2;
use fixed::{Fixed, FixedTrait, HALF};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::{Pose2, Pose2Trait};

/// An axis-aligned bounding box, represented by minimum and maximum corners.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct Aabb {
    /// Minimum x/y corner.
    pub mins: Vec2,
    /// Maximum x/y corner.
    pub maxs: Vec2,
}

/// `|R| * h`: the half extents of the box of half extents `h` rotated by `pose` (upstream
/// `PoseOps::absolute_transform_vector`). One rescale per component, so an exact quarter turn
/// gives an exact box. Shared by [`AabbTrait::transform_by`] and the shape AABBs.
/// #### Panics
/// * `'i64_neg Underflow'` for a rotation component equal to `fixed::MIN` (not unit).
/// * `'Fixed: overflow'` if a component of the result leaves the scalar range.
#[inline(always)]
pub fn absolute_transform_vector(pose: Pose2, h: Vec2) -> Vec2 {
    let c = pose.rotation.re.abs();
    let s = pose.rotation.im.abs();
    Vec2 { x: dot2(c, h.x, s, h.y), y: dot2(s, h.x, c, h.y) }
}

#[generate_trait]
pub impl AabbImpl of AabbTrait {
    /// Creates a new AABB from its minimum and maximum corners.
    ///
    /// The bounds are stored as-is; callers are responsible for `mins <= maxs` on each axis.
    #[inline(always)]
    fn new(mins: Vec2, maxs: Vec2) -> Aabb {
        Aabb { mins, maxs }
    }

    /// Creates an AABB from its center and half-extents.
    ///
    /// Negative half-extents invert that axis, matching direct vector arithmetic.
    #[inline(always)]
    fn from_half_extents(center: Vec2, half_extents: Vec2) -> Aabb {
        Aabb { mins: center - half_extents, maxs: center + half_extents }
    }

    /// Returns the midpoint between `mins` and `maxs`, floored component-wise.
    fn center(self: Aabb) -> Vec2 {
        (self.mins + self.maxs).mul_scalar(HALF)
    }

    /// Returns half the full extents, floored component-wise.
    fn half_extents(self: Aabb) -> Vec2 {
        self.extents().mul_scalar(HALF)
    }

    /// Returns the full extents `maxs - mins`.
    fn extents(self: Aabb) -> Vec2 {
        self.maxs - self.mins
    }

    /// Returns `true` when `self` and `other` overlap on both axes, including edge touches.
    fn intersects(self: Aabb, other: Aabb) -> bool {
        self.mins.x.raw <= other.maxs.x.raw
            && other.mins.x.raw <= self.maxs.x.raw
            && self.mins.y.raw <= other.maxs.y.raw
            && other.mins.y.raw <= self.maxs.y.raw
    }

    /// Returns `true` when `self` fully contains `other`, including equal bounds.
    fn contains(self: Aabb, other: Aabb) -> bool {
        self.mins.x.raw <= other.mins.x.raw
            && other.maxs.x.raw <= self.maxs.x.raw
            && self.mins.y.raw <= other.mins.y.raw
            && other.maxs.y.raw <= self.maxs.y.raw
    }

    /// Returns `true` when `p` lies inside this AABB or on its boundary.
    fn contains_local_point(self: Aabb, p: Vec2) -> bool {
        self.mins.x.raw <= p.x.raw
            && p.x.raw <= self.maxs.x.raw
            && self.mins.y.raw <= p.y.raw
            && p.y.raw <= self.maxs.y.raw
    }

    /// Returns the smallest AABB containing both inputs.
    fn merged(self: Aabb, other: Aabb) -> Aabb {
        Aabb { mins: self.mins.min(other.mins), maxs: self.maxs.max(other.maxs) }
    }

    /// Expands every side by `margin`.
    ///
    /// A negative margin tightens the box and may invert empty axes, matching upstream arithmetic.
    #[inline(always)]
    fn loosened(self: Aabb, margin: Fixed) -> Aabb {
        let v = Vec2 { x: margin, y: margin };
        Aabb { mins: self.mins - v, maxs: self.maxs + v }
    }

    /// Shrinks every side by `margin`.
    ///
    /// A negative margin loosens the box and may invert empty axes, matching upstream arithmetic.
    fn tightened(self: Aabb, margin: Fixed) -> Aabb {
        let v = Vec2 { x: margin, y: margin };
        Aabb { mins: self.mins + v, maxs: self.maxs - v }
    }

    /// Bounds this AABB after applying `pose`.
    ///
    /// Uses Parry's `abs(rotation) * half_extents` formulation instead of transforming four
    /// corners. Each product pair is accumulated wide and floored once per output component.
    fn transform_by(self: Aabb, pose: Pose2) -> Aabb {
        let center = pose.transform_point(self.center());
        let half = self.half_extents();
        let half = absolute_transform_vector(pose, half);
        Aabb { mins: center - half, maxs: center + half }
    }

    /// Returns the 2D area of this AABB (`extent.x * extent.y`).
    fn volume(self: Aabb) -> Fixed {
        let e = self.extents();
        e.x * e.y
    }

    /// Scales `mins` and `maxs` component-wise, reordering axes for negative scale components.
    fn scaled(self: Aabb, scale: Vec2) -> Aabb {
        let a = self.mins * scale;
        let b = self.maxs * scale;
        Aabb { mins: a.min(b), maxs: a.max(b) }
    }
}

#[cfg(test)]
pub mod alternatives {
    use glam::{Vec2, Vec2Trait};
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use super::Aabb;

    /// Transforms all four corners and merges them. Same semantics as `transform_by`, but costlier.
    pub fn transform_by_corners(aabb: Aabb, pose: Pose2) -> Aabb {
        let p0 = pose.transform_point(aabb.mins);
        let p1 = pose.transform_point(Vec2 { x: aabb.mins.x, y: aabb.maxs.y });
        let p2 = pose.transform_point(Vec2 { x: aabb.maxs.x, y: aabb.mins.y });
        let p3 = pose.transform_point(aabb.maxs);
        Aabb { mins: p0.min(p1).min(p2).min(p3), maxs: p0.max(p1).max(p2).max(p3) }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::{Vec2, Vec2Trait};
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::{IDENTITY, Rot2};
    use rapier_testing::opaque;
    use super::{Aabb, AabbTrait, absolute_transform_vector, alternatives};

    const QUARTER: Fixed = Fixed { raw: 1073741824 };
    const NEG_ONE: Fixed = Fixed { raw: -4294967296 };
    const THREE: Fixed = Fixed { raw: 12884901888 };
    const FOUR: Fixed = Fixed { raw: 17179869184 };
    const R: Rot2 = Rot2 { re: Fixed { raw: 3037000500 }, im: Fixed { raw: 3037000500 } };
    const P: Pose2 = Pose2 {
        translation: Vec2 { x: Fixed { raw: 8589934592 }, y: NEG_ONE }, rotation: R,
    };
    const A: Aabb = Aabb {
        mins: Vec2 { x: NEG_ONE, y: Fixed { raw: -8589934592 } }, maxs: Vec2 { x: THREE, y: TWO },
    };
    const B: Aabb = Aabb {
        mins: Vec2 { x: ONE, y: TWO }, maxs: Vec2 { x: FOUR, y: Fixed { raw: 21474836480 } },
    };

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn close(a: Vec2, b: Vec2, ulps: i64) {
        assert!(a.x.abs_diff_eq(b.x, Fixed { raw: ulps }));
        assert!(a.y.abs_diff_eq(b.y, Fixed { raw: ulps }));
    }

    #[test]
    fn test_constructors_center_and_extents() {
        let a = AabbTrait::new(v(NEG_ONE, -TWO), v(THREE, TWO));
        assert_eq!(a.center(), v(ONE, ZERO));
        assert_eq!(a.extents(), v(FOUR, FOUR));
        assert_eq!(a.half_extents(), v(TWO, TWO));
        assert_eq!(AabbTrait::from_half_extents(v(ONE, ZERO), v(TWO, TWO)), a);

        let empty = AabbTrait::new(v(ONE, TWO), v(ONE, TWO));
        assert_eq!(empty.center(), v(ONE, TWO));
        assert_eq!(empty.volume(), ZERO);
    }

    #[test]
    fn test_intersections_are_closed() {
        for (lhs, rhs, hit) in array![
            (A, B, true),
            (A, AabbTrait::new(v(FOUR + FixedTrait::from_raw(1), ZERO), v(FOUR, ONE)), false),
            (A, AabbTrait::new(v(THREE, TWO), v(FOUR, THREE)), true),
            (
                AabbTrait::new(v(FOUR, FOUR), v(FixedTrait::from_int(5), FixedTrait::from_int(5))),
                A,
                false,
            ),
        ]
            .span() {
            assert_eq!((*lhs).intersects(*rhs), *hit);
            assert_eq!((*rhs).intersects(*lhs), *hit);
        }
    }

    #[test]
    fn test_containment_point_merge_margin_and_scale() {
        assert!(A.contains_local_point(v(NEG_ONE, -TWO)));
        assert!(A.contains_local_point(v(THREE, TWO)));
        assert!(!A.contains_local_point(v(THREE + FixedTrait::from_raw(1), ZERO)));
        assert!(A.contains(AabbTrait::new(v(ZERO, NEG_ONE), v(ONE, ONE))));
        assert!(!A.contains(B));
        assert_eq!(A.merged(B), AabbTrait::new(v(NEG_ONE, -TWO), v(FOUR, FixedTrait::from_int(5))));
        assert_eq!(
            A.loosened(HALF),
            AabbTrait::new(v(NEG_ONE - HALF, -TWO - HALF), v(THREE + HALF, TWO + HALF)),
        );
        assert_eq!(
            A.tightened(QUARTER),
            AabbTrait::new(v(NEG_ONE + QUARTER, -TWO + QUARTER), v(THREE - QUARTER, TWO - QUARTER)),
        );
        assert_eq!(
            A.scaled(v(-ONE, TWO)),
            AabbTrait::new(v(-THREE, Fixed { raw: -17179869184 }), v(ONE, FOUR)),
        );
    }

    #[test]
    fn test_transform_by_abs_rotation_matches_corners() {
        for a in array![A, B, AabbTrait::new(v(ZERO, ZERO), v(ZERO, ZERO))].span() {
            let fast = (*a).transform_by(P);
            let corners = alternatives::transform_by_corners(*a, P);
            close(fast.mins, corners.mins, 8);
            close(fast.maxs, corners.maxs, 8);
            assert_eq!(
                (*a).transform_by(Pose2Trait::new(v(ONE, TWO), IDENTITY)),
                AabbTrait::new((*a).mins + v(ONE, TWO), (*a).maxs + v(ONE, TWO)),
            );
        }
    }

    #[test]
    #[fuzzer(runs: 128, seed: 20260920)]
    fn fuzz_transform_candidates(x0: i16, y0: i16, x1: i16, y1: i16) {
        let lo = v(Fixed { raw: x0.into() }, Fixed { raw: y0.into() });
        let hi = v(Fixed { raw: x1.into() }, Fixed { raw: y1.into() });
        let a = AabbTrait::new(lo.min(hi), lo.max(hi));
        let fast = a.transform_by(P);
        let corners = alternatives::transform_by_corners(a, P);
        close(fast.mins, corners.mins, 8);
        close(fast.maxs, corners.maxs, 8);
    }

    #[test]
    fn test_absolute_transform_vector_quarter_turns_are_exact() {
        let h = v(TWO, ONE);
        let turn = |
            re: Fixed, im: Fixed,
        | Pose2 { translation: v(ZERO, ZERO), rotation: Rot2 { re, im } };
        assert_eq!(absolute_transform_vector(turn(ONE, ZERO), h), h);
        assert_eq!(absolute_transform_vector(turn(ZERO, ONE), h), v(ONE, TWO));
        assert_eq!(absolute_transform_vector(turn(NEG_ONE, ZERO), h), h);
        assert_eq!(absolute_transform_vector(turn(ZERO, NEG_ONE), h), v(ONE, TWO));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_absolute_transform_vector() {
        let _ = absolute_transform_vector(opaque(P), opaque(v(ONE, HALF)));
    }
    #[test]
    fn gas_new() {
        assert_eq!(AabbTrait::new(opaque(A.mins), opaque(A.maxs)), A);
    }
    #[test]
    fn gas_from_half_extents() {
        assert_eq!(
            AabbTrait::from_half_extents(opaque(v(ONE, ZERO)), opaque(v(TWO, TWO))),
            AabbTrait::new(v(NEG_ONE, -TWO), v(THREE, TWO)),
        );
    }
    #[test]
    fn gas_center() {
        assert_eq!(opaque(A).center(), v(ONE, ZERO));
    }
    #[test]
    fn gas_half_extents() {
        assert_eq!(opaque(A).half_extents(), v(TWO, TWO));
    }
    #[test]
    fn gas_extents() {
        assert_eq!(opaque(A).extents(), v(FOUR, FOUR));
    }
    #[test]
    fn gas_intersects() {
        assert!(opaque(A).intersects(opaque(B)));
    }
    #[test]
    fn gas_contains() {
        assert!(opaque(A).contains(opaque(A)));
    }
    #[test]
    fn gas_contains_local_point() {
        assert!(opaque(A).contains_local_point(opaque(v(ONE, ZERO))));
    }
    #[test]
    fn gas_merged() {
        assert_eq!(opaque(A).merged(opaque(B)).maxs.y, FixedTrait::from_int(5));
    }
    #[test]
    fn gas_loosened() {
        assert_eq!(opaque(A).loosened(opaque(HALF)).mins.x, NEG_ONE - HALF);
    }
    #[test]
    fn gas_tightened() {
        assert_eq!(opaque(A).tightened(opaque(HALF)).mins.x, NEG_ONE + HALF);
    }
    #[test]
    fn gas_transform_by_abs_rotation() {
        assert!(
            opaque(A).transform_by(opaque(P)).contains_local_point(P.transform_point(A.center())),
        );
    }
    #[test]
    fn gas_transform_by_corners() {
        assert!(
            alternatives::transform_by_corners(opaque(A), opaque(P))
                .contains_local_point(P.transform_point(A.center())),
        );
    }
    #[test]
    fn gas_volume() {
        assert_eq!(opaque(A).volume(), FixedTrait::from_int(16));
    }
    #[test]
    fn gas_scaled() {
        assert_eq!(opaque(A).scaled(opaque(v(-ONE, TWO))).mins.x, -THREE);
    }
}
