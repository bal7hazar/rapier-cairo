//! Segment–segment kernels of the shape-pair queries (Parry
//! `closest_points_segment_segment.rs`, `distance_segment_segment.rs`), on top of the exact
//! routine of [`crate::closest_points`].
//!
//! The second segment is moved into the frame of the first to find the locations, and each
//! point is then read on its own segment in its own frame, as upstream does. Like upstream,
//! crossing or touching segments answer `WithinMargin` with (nearly) equal points, never
//! `Intersecting`.

use fixed::{Fixed, ZERO};
use rapier_math::math_ext::norm2::is_norm2_gt;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::closest_points::closest_points_segment_segment_with_locations as locations;
use crate::point::segment_point_at;
use crate::shape::{Segment, SegmentTrait};
use super::{ClosestPoints, normalize_and_length};

/// Closest points of two segments when they are within `margin` of each other (upstream
/// `closest_points_segment_segment`, with `pos12`).
/// #### Panics
/// * See `crate::closest_points::closest_points_segment_segment_with_locations`.
pub fn closest_points_segment_segment(
    pos12: Pose2, seg1: Segment, seg2: Segment, margin: Fixed,
) -> ClosestPoints {
    let (loc1, loc2) = locations(seg1, seg2.transformed(pos12));
    let p1 = segment_point_at(seg1, loc1);
    let p2 = segment_point_at(seg2, loc2);
    let d = p1 - pos12.transform_point(p2);
    if margin < ZERO || is_norm2_gt(d.x, d.y, margin) {
        ClosestPoints::Disjoint
    } else {
        ClosestPoints::WithinMargin((p1, p2))
    }
}

/// Distance between two segments (upstream `distance_segment_segment`): the length between the
/// closest points.
/// #### Panics
/// * See [`closest_points_segment_segment`].
pub fn distance_segment_segment(pos12: Pose2, seg1: Segment, seg2: Segment) -> Fixed {
    let (loc1, loc2) = locations(seg1, seg2.transformed(pos12));
    let d = segment_point_at(seg1, loc1) - pos12.transform_point(segment_point_at(seg2, loc2));
    let (_, len) = normalize_and_length(d);
    len
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::query::ClosestPoints;
    use crate::query::support_map::distance_support_map_support_map;
    use crate::shape::{Segment, SegmentTrait, Shape};
    use super::{closest_points_segment_segment, distance_segment_segment};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    fn seg() -> Segment {
        SegmentTrait::new(v(-ONE, ZERO), v(ONE, ZERO))
    }

    /// `(pos12, distance, closest points at margin 1)`; crossing segments are `WithinMargin`.
    #[test]
    fn test_segment_segment_table() {
        let quarter = Rot2 { re: ZERO, im: ONE };
        let identity = Rot2 { re: ONE, im: ZERO };
        let cases: Span<(Pose2, Fixed, ClosestPoints)> = array![
            (Pose2Trait::new(v(int(4), ZERO), identity), int(2), ClosestPoints::Disjoint),
            (
                Pose2Trait::new(v(ZERO, int(2)), quarter),
                ONE,
                ClosestPoints::WithinMargin((v(ZERO, ZERO), v(-ONE, ZERO))),
            ),
            (
                Pose2Trait::new(v(ZERO, ZERO), quarter),
                ZERO,
                ClosestPoints::WithinMargin((v(ZERO, ZERO), v(ZERO, ZERO))),
            ),
        ]
            .span();
        for (pos12, distance, closest) in cases {
            assert_eq!(distance_segment_segment(*pos12, seg(), seg()), *distance);
            assert_eq!(closest_points_segment_segment(*pos12, seg(), seg(), ONE), *closest);
            // Same distance as the generic kernel (which reports crossing as penetrating).
            assert_eq!(
                distance_support_map_support_map(
                    *pos12, Shape::Segment(seg()), Shape::Segment(seg()),
                ),
                *distance,
            );
        }
        assert_eq!(
            closest_points_segment_segment(
                Pose2Trait::new(v(ZERO, HALF), identity), seg(), seg(), -ONE,
            ),
            ClosestPoints::Disjoint,
        );
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_distance_segment_segment() {
        let _ = distance_segment_segment(
            opaque(Pose2Trait::new(v(ZERO, int(2)), Rot2 { re: ZERO, im: ONE })),
            opaque(seg()),
            opaque(seg()),
        );
    }

    #[test]
    fn gas_distance_support_map_segment_segment() {
        let _ = distance_support_map_support_map(
            opaque(Pose2Trait::new(v(ZERO, int(2)), Rot2 { re: ZERO, im: ONE })),
            opaque(Shape::Segment(seg())),
            opaque(Shape::Segment(seg())),
        );
    }

    #[test]
    fn gas_closest_points_segment_segment() {
        let _ = closest_points_segment_segment(
            opaque(Pose2Trait::new(v(ZERO, int(2)), Rot2 { re: ZERO, im: ONE })),
            opaque(seg()),
            opaque(seg()),
            opaque(ONE),
        );
    }
}
