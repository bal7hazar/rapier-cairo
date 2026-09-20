//! `Segment` (Parry `shape/segment.rs`, `bounding_volume/aabb_support_map.rs`).

use core::num::traits::Zero;
use fixed::{Fixed, FixedTrait};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::{DEFAULT_EPSILON, try_normalize2, try_normalize2_eps};
use crate::aabb::Aabb;
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::mass::MassProperties;

/// The segment from `a` to `b`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Segment {
    pub a: Vec2,
    pub b: Vec2,
}

#[generate_trait]
pub impl SegmentImpl of SegmentTrait {
    #[inline(always)]
    fn new(a: Vec2, b: Vec2) -> Segment {
        Segment { a, b }
    }

    /// `b - a`.
    #[inline(always)]
    fn scaled_direction(self: Segment) -> Vec2 {
        self.b - self.a
    }

    /// `|b - a|`, floored to 1 ulp.
    #[inline(always)]
    fn length(self: Segment) -> Fixed {
        Self::scaled_direction(self).length()
    }

    /// Exchanges the end points.
    fn swap(ref self: Segment) {
        let a = self.a;
        self.a = self.b;
        self.b = a;
    }

    /// Unit direction `(b - a) / |b - a|`, `None` for a zero-length segment. Rounded to nearest
    /// per component, exact for axis-aligned segments.
    fn direction(self: Segment) -> Option<Vec2> {
        let d = Self::scaled_direction(self);
        match try_normalize2(d.x, d.y) {
            Some((x, y)) => Some(Vec2 { x, y }),
            None => None,
        }
    }

    /// `(d.y, -d.x)` with `d = b - a`: the right-hand normal, not normalised.
    #[inline(always)]
    fn scaled_normal(self: Segment) -> Vec2 {
        let d = Self::scaled_direction(self);
        Vec2 { x: d.y, y: -d.x }
    }

    /// Unit right-hand normal, `None` when the segment is not longer than `DEFAULT_EPSILON`
    /// (upstream `length > DEFAULT_EPSILON`, here the wide comparison).
    #[inline(always)]
    fn normal(self: Segment) -> Option<Vec2> {
        let n = Self::scaled_normal(self);
        match try_normalize2_eps(n.x, n.y, DEFAULT_EPSILON) {
            Some((x, y)) => Some(Vec2 { x, y }),
            None => None,
        }
    }

    /// The segment moved by `pose`.
    #[inline(always)]
    fn transformed(self: Segment, pose: Pose2) -> Segment {
        Segment { a: pose.transform_point(self.a), b: pose.transform_point(self.b) }
    }

    /// Normal of a feature: `Vertex(0)` gives the direction, `Vertex(1)` its opposite,
    /// `Face(0)` / `Face(1)` the right / left normal; anything else is `None`. A zero-length
    /// segment answers `+Y`, as upstream.
    fn feature_normal(self: Segment, feature: FeatureId) -> Option<Vec2> {
        match Self::direction(self) {
            Some(dir) => {
                if feature.is_vertex() {
                    if feature.code() == 0 {
                        Some(dir)
                    } else {
                        Some(-dir)
                    }
                } else if feature.is_face() {
                    if feature.code() == 0 {
                        Some(Vec2 { x: dir.y, y: -dir.x })
                    } else {
                        Some(Vec2 { x: -dir.y, y: dir.x })
                    }
                } else {
                    None
                }
            },
            None => Some(Vec2Trait::Y),
        }
    }

    /// `[min(a, b), max(a, b)]` component-wise.
    #[inline(always)]
    fn compute_local_aabb(self: Segment) -> Aabb {
        let (min_x, max_x) = if self.a.x < self.b.x {
            (self.a.x, self.b.x)
        } else {
            (self.b.x, self.a.x)
        };
        let (min_y, max_y) = if self.a.y < self.b.y {
            (self.a.y, self.b.y)
        } else {
            (self.b.y, self.a.y)
        };
        Aabb { mins: Vec2 { x: min_x, y: min_y }, maxs: Vec2 { x: max_x, y: max_y } }
    }

    /// Box of the transformed end points.
    #[inline(always)]
    fn compute_aabb(self: Segment, pose: Pose2) -> Aabb {
        Self::compute_local_aabb(Self::transformed(self, pose))
    }

    /// Zero: a segment has no area.
    #[inline(always)]
    fn mass_properties(self: Segment, density: Fixed) -> MassProperties {
        Zero::<MassProperties>::zero()
    }

    /// The end point farther along `dir`, `a` on a tie (upstream `a.dot(dir) > b.dot(dir)`,
    /// evaluated as one dot of the difference).
    #[inline(always)]
    fn local_support_point(self: Segment, dir: Vec2) -> Vec2 {
        if (self.a - self.b).dot(dir) > FixedTrait::from_raw(0) {
            self.a
        } else {
            self.b
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::feature_id::{FEATURE_UNKNOWN, FeatureId, FeatureIdTrait};
    use super::{Segment, SegmentTrait};

    fn v(x: i32, y: i32) -> Vec2 {
        Vec2 { x: FixedTrait::from_int(x), y: FixedTrait::from_int(y) }
    }

    fn seg(ax: i32, ay: i32, bx: i32, by: i32) -> Segment {
        SegmentTrait::new(v(ax, ay), v(bx, by))
    }

    fn quarter_turn() -> Pose2 {
        Pose2Trait::new(v(10, 0), Rot2 { re: ZERO, im: ONE })
    }

    #[test]
    fn test_direction_normal_length_table() {
        // (segment, scaled direction, length, unit direction, unit normal); the axis-aligned cases
        // are exact, the zero-length one has neither direction nor normal.
        let cases: Span<(Segment, Vec2, i32, Option<Vec2>, Option<Vec2>)> = array![
            (seg(0, 0, 3, 0), v(3, 0), 3, Some(v(1, 0)), Some(v(0, -1))),
            (seg(1, 1, 1, -3), v(0, -4), 4, Some(v(0, -1)), Some(v(-1, 0))),
            (seg(3, 4, 0, 0), v(-3, -4), 5, None, None), (seg(2, 2, 2, 2), v(0, 0), 0, None, None),
        ]
            .span();
        for (s, scaled, length, direction, normal) in cases {
            assert_eq!(s.scaled_direction(), *scaled);
            assert_eq!(s.length(), FixedTrait::from_int(*length));
            assert_eq!(s.scaled_normal(), Vec2 { x: *scaled.y, y: -*scaled.x });
            if let Some(expected) = direction {
                assert_eq!(s.direction(), Some(*expected));
                assert_eq!(s.normal(), *normal);
            }
        }
        // (3, 4): unit direction (0.6, 0.8) within 1 ulp; zero length: no direction, no normal.
        let d = seg(3, 4, 0, 0).direction().unwrap();
        assert!(d.x.abs_diff_eq(Fixed { raw: -2576980378 }, Fixed { raw: 1 }));
        assert!(d.y.abs_diff_eq(Fixed { raw: -3435973837 }, Fixed { raw: 1 }));
        assert_eq!((seg(2, 2, 2, 2).direction(), seg(2, 2, 2, 2).normal()), (None, None));
        // Shorter than DEFAULT_EPSILON (512 ulp): no normal, but a direction.
        let tiny = SegmentTrait::new(v(0, 0), Vec2 { x: Fixed { raw: 512 }, y: ZERO });
        assert_eq!(tiny.normal(), None);
        assert_eq!(tiny.direction(), Some(v(1, 0)));
    }

    #[test]
    fn test_swap_transformed_support() {
        let mut s = seg(0, 0, 3, 1);
        s.swap();
        assert_eq!(s, seg(3, 1, 0, 0));
        // Quarter turn then translation (10, 0): (x, y) -> (10 - y, x).
        assert_eq!(seg(0, 0, 3, 1).transformed(quarter_turn()), seg(10, 0, 9, 3));
        // The end point farther along dir; a tie picks `b`.
        let s = seg(-1, 0, 2, 0);
        assert_eq!(s.local_support_point(v(1, 0)), v(2, 0));
        assert_eq!(s.local_support_point(v(-1, 5)), v(-1, 0));
        assert_eq!(s.local_support_point(v(0, 1)), v(2, 0));
        assert_eq!(s.local_support_point(v(0, 0)), v(2, 0));
    }

    #[test]
    fn test_feature_normal_table() {
        let s = seg(0, 0, 0, 2);
        let cases: Span<(FeatureId, Option<Vec2>)> = array![
            (FeatureIdTrait::vertex(0), Some(v(0, 1))), (FeatureIdTrait::vertex(1), Some(v(0, -1))),
            (FeatureIdTrait::face(0), Some(v(1, 0))), (FeatureIdTrait::face(1), Some(v(-1, 0))),
            (FEATURE_UNKNOWN, None),
        ]
            .span();
        for (feature, expected) in cases {
            assert_eq!(s.feature_normal(*feature), *expected);
        }
        // Zero length: +Y whatever the feature.
        assert_eq!(seg(1, 1, 1, 1).feature_normal(FeatureIdTrait::face(1)), Some(v(0, 1)));
    }

    #[test]
    fn test_aabb_and_mass() {
        let s = seg(3, -1, -2, 4);
        let local = s.compute_local_aabb();
        assert_eq!((local.mins, local.maxs), (v(-2, -1), v(3, 4)));
        // Quarter turn: (3, -1) -> (11, 3), (-2, 4) -> (6, -2).
        let placed = s.compute_aabb(quarter_turn());
        assert_eq!((placed.mins, placed.maxs), (v(6, -2), v(11, 3)));
        // End points given in the opposite order box the same.
        assert_eq!(seg(-2, 4, 3, -1).compute_local_aabb(), local);
        let point = seg(1, 1, 1, 1).compute_local_aabb();
        assert_eq!((point.mins, point.maxs), (v(1, 1), v(1, 1)));
        assert_eq!(s.mass_properties(HALF), Default::default());
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_scaled_direction() {
        let _ = opaque(seg(0, 0, 3, 4)).scaled_direction();
    }
    #[test]
    fn gas_length() {
        let _ = opaque(seg(0, 0, 3, 4)).length();
    }
    #[test]
    fn gas_direction() {
        let _ = opaque(seg(0, 0, 3, 4)).direction();
    }
    #[test]
    fn gas_normal() {
        let _ = opaque(seg(0, 0, 3, 4)).normal();
    }
    #[test]
    fn gas_transformed() {
        let _ = opaque(seg(0, 0, 3, 4)).transformed(opaque(quarter_turn()));
    }
    #[test]
    fn gas_feature_normal() {
        let _ = opaque(seg(0, 0, 3, 4)).feature_normal(opaque(FeatureIdTrait::face(0)));
    }
    #[test]
    fn gas_compute_local_aabb() {
        let _ = opaque(seg(0, 0, 3, 4)).compute_local_aabb();
    }
    #[test]
    fn gas_compute_aabb() {
        let _ = opaque(seg(0, 0, 3, 4)).compute_aabb(opaque(quarter_turn()));
    }
    #[test]
    fn gas_local_support_point() {
        let _ = opaque(seg(0, 0, 3, 4)).local_support_point(opaque(v(1, 1)));
    }
}
