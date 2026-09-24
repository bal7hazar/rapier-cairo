//! Analytic polygon projection: test exact edge half-planes and minimise distance to segments.
//! Unlike GJK/EPA, equal rounded squared distances choose the first edge. Feature ids name the
//! exact Voronoi region (plain vertex/face index, unlike the packed PFM `2*i`/`2*i+1` ids).

use fixed::Fixed;
use fixed::wide::distance2;
use glam::Vec2;
use rapier_math::math_ext::norm2::norm2_sq_wide;
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::shape::{ConvexPolygon, ConvexPolygonTrait, Segment};
use super::{
    PointProjection, SegmentPointLocation, cross_wide, project_local_point_and_get_location_segment,
};

/// Tests all edge half-planes. Boundary is inside; no rounded normals enter this decision.
/// Panics if coordinate differences or exact i128 cross products overflow.
pub fn contains_local_point_convex_polygon(polygon: ConvexPolygon, pt: Vec2) -> bool {
    let mut i = 0;
    while i != polygon.count {
        let a = polygon.vertex(i);
        let e = polygon.vertex(polygon.next(i)) - a;
        let d = pt - a;
        if cross_wide(e.x, e.y, d.x, d.y) < 0 {
            return false;
        }
        i += 1;
    }
    true
}

/// Closest boundary point and plain vertex/face id. Equal distances choose the lowest edge.
/// Segment interpolation floors once after the correctly rounded wide barycentric ratio.
/// Panics on coordinate differences or wide product overflow; see segment projection.
pub fn project_local_point_and_get_feature_convex_polygon(
    polygon: ConvexPolygon, pt: Vec2,
) -> (PointProjection, FeatureId) {
    let inside = contains_local_point_convex_polygon(polygon, pt);
    let mut point = polygon.vertex(0);
    let d = point - pt;
    let mut best = norm2_sq_wide(d.x, d.y);
    let mut feature = FeatureIdTrait::vertex(0);
    let mut i = 0;
    while i != polygon.count {
        let next = polygon.next(i);
        let segment = Segment { a: polygon.vertex(i), b: polygon.vertex(next) };
        let (projection, location) = project_local_point_and_get_location_segment(
            segment, pt, false,
        );
        let d = projection.point - pt;
        let sq = norm2_sq_wide(d.x, d.y);
        if sq < best || i == 0 {
            best = sq;
            point = projection.point;
            feature = match location {
                SegmentPointLocation::OnVertex(v) => FeatureIdTrait::vertex(
                    if v == 0 {
                        i.into()
                    } else {
                        next.into()
                    },
                ),
                SegmentPointLocation::OnEdge(_) => FeatureIdTrait::face(i.into()),
            };
        }
        i += 1;
    }
    (PointProjection { is_inside: inside, point }, feature)
}

/// Projects onto the filled polygon if `solid`, otherwise onto its boundary.
/// Rounding and panics as `project_local_point_and_get_feature_convex_polygon`.
pub fn project_local_point_convex_polygon(
    polygon: ConvexPolygon, pt: Vec2, solid: bool,
) -> PointProjection {
    if solid && contains_local_point_convex_polygon(polygon, pt) {
        return PointProjection { is_inside: true, point: pt };
    }
    let (projection, _) = project_local_point_and_get_feature_convex_polygon(polygon, pt);
    projection
}

/// Signed distance to the boundary (negative inside); zero inside a solid polygon.
/// Distance floors the wide norm. Panics if the length is unrepresentable, as other point queries.
pub fn distance_to_local_point_convex_polygon(
    polygon: ConvexPolygon, pt: Vec2, solid: bool,
) -> Fixed {
    let projection = project_local_point_convex_polygon(polygon, pt, solid);
    let distance = distance2(pt.x, pt.y, projection.point.x, projection.point.y);
    if projection.is_inside && !solid {
        -distance
    } else {
        distance
    }
}
