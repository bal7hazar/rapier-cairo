//! Rejected candidates of `crate::dispatch::intersection`, kept for the `gas_*` ranking.

use fixed::{Fixed, ZERO};
use rapier_math::math_ext::norm2::is_norm2_le;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::contact::ContactManifold;
use crate::dispatch::contact_manifold;
use crate::point::convex_polygon::project_local_point_convex_polygon;
use crate::point::project_local_point_segment;
use crate::sat::cuboid_cuboid_find_local_separating_normal_oneway;
use crate::shape::{ConvexPolygon, Cuboid, Segment, Shape};
use super::segments_cross;

/// The answer derived from the contact generators: the metered dispatcher at zero prediction,
/// intersecting when a point has `dist <= 0`. `None` for a pair without generator.
pub fn intersection_test_from_contacts(pos12: Pose2, shape1: Shape, shape2: Shape) -> Option<bool> {
    let mut manifold: ContactManifold = Default::default();
    if !contact_manifold(pos12, shape1, shape2, ZERO, ref manifold) {
        return None;
    }
    let [p0, p1] = manifold.points;
    Some(
        (manifold.num_points > 0 && p0.dist <= ZERO)
            || (manifold.num_points > 1 && p1.dist <= ZERO),
    )
}

/// Upstream's cuboid–cuboid test: the two one-way SAT searches (`sat`), separated when either
/// separation is positive.
pub fn cuboid_cuboid_upstream_sat(pos12: Pose2, cuboid1: Cuboid, cuboid2: Cuboid) -> bool {
    let (sep1, _) = cuboid_cuboid_find_local_separating_normal_oneway(cuboid1, cuboid2, pos12);
    if sep1 > ZERO {
        return false;
    }
    let (sep2, _) = cuboid_cuboid_find_local_separating_normal_oneway(
        cuboid2, cuboid1, pos12.inverse(),
    );
    sep2 <= ZERO
}

/// Segment / capsule pairs without the closest-points kernel: crossing test, then the four
/// endpoint–segment distances (two disjoint segments are closest at an endpoint).
pub fn segment_segment_endpoints(
    pos12: Pose2, segment1: Segment, segment2: Segment, radius: Fixed,
) -> bool {
    let a2 = pos12.transform_point(segment2.a);
    let b2 = pos12.transform_point(segment2.b);
    if segments_cross(segment1.a, segment1.b, a2, b2) {
        return true;
    }
    if radius.raw == 0 {
        return false;
    }
    let other = Segment { a: a2, b: b2 };
    within(segment1, a2, radius)
        || within(segment1, b2, radius)
        || within(other, segment1.a, radius)
        || within(other, segment1.b, radius)
}

fn within(segment: Segment, pt: glam::Vec2, radius: Fixed) -> bool {
    let proj = project_local_point_segment(segment, pt, true);
    is_norm2_le(pt.x - proj.point.x, pt.y - proj.point.y, radius)
}

/// Ball–polygon through `crate::point`'s solid projection (exact containment, then the closest
/// point over every edge).
pub fn point_polygon_projection(polygon: ConvexPolygon, pt: glam::Vec2, radius: Fixed) -> bool {
    let proj = project_local_point_convex_polygon(polygon, pt, true);
    proj.is_inside || is_norm2_le(pt.x - proj.point.x, pt.y - proj.point.y, radius)
}
