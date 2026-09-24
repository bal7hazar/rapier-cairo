//! Point projection on the MVP shapes (Parry `query/point/`, work package GC).
//!
//! Upstream exposes `PointQuery::{project_local_point, project_local_point_and_get_feature,
//! distance_to_local_point, contains_local_point}` as a trait object; a closed shape set needs no
//! dynamic dispatch, so every shape gets a free function named after it
//! (`project_local_point_ball`, ...) and [`super::shape::Shape`] will dispatch by `match` once GB
//! lands. All queries are in the local frame of the shape.
//!
//! # Fixed-point hazards answered here
//!
//! * **No squared length is ever rescaled before it is compared.** `pt.length_squared() <=
//!   radius * radius` would be `0 <= 0` for everything shorter than `2^-16`, so every such test
//!   goes through the wide helpers of `rapier_math::math_ext::norm2` (raw Q64.64, exact over the
//!   whole scalar range).
//! * **One division per query, at the end.** The segment ratio `u = (ab . ap) / |ab|^2` is the
//!   only division of the module and it is computed from the two *wide* dot products by
//!   [`ratio::clamped_ratio`], which never rescales either operand; the ball and the capsule share
//!   the single reciprocal of `fixed::wide::Norm`.
//! * **Discrete answers are decided exactly.** `is_inside`, the Voronoi region of a segment and
//!   the `Face(0)` / `Face(1)` side are all read off exact `i128` quantities, never off a rounded
//!   projection, so they never flip because the projected point landed 1 ulp away.
//!
//! Bounded convex polygons use analytic edge projection in `convex_polygon`.
//! Deferred (see `docs/PLAN.md`): support-map projection (GJK/EPA) and triangles.

pub mod ball;
pub mod capsule;
pub mod convex_polygon;
pub mod cuboid;
pub mod halfspace;
pub mod ratio;
pub mod segment;
pub mod wide2;
pub use ball::{
    contains_local_point_ball, distance_to_local_point_ball,
    project_local_point_and_get_feature_ball, project_local_point_ball,
};
pub use capsule::{
    contains_local_point_capsule, distance_to_local_point_capsule,
    project_local_point_and_get_feature_capsule, project_local_point_capsule,
};
pub use cuboid::{
    contains_local_point_cuboid, distance_to_local_point_cuboid,
    project_local_point_and_get_feature_cuboid, project_local_point_cuboid,
};
use fixed::{Fixed, ONE, ZERO};
use glam::vec2::Vec2;
pub use halfspace::{
    contains_local_point_halfspace, distance_to_local_point_halfspace,
    project_local_point_and_get_feature_halfspace, project_local_point_halfspace,
};
pub use ratio::clamped_ratio;
pub use segment::{
    contains_local_point_segment, distance_to_local_point_segment,
    project_local_point_and_get_feature_segment, project_local_point_and_get_location_segment,
    project_local_point_segment, segment_point_at,
};
pub use wide2::{cross_wide, dot_wide};

/// The result of projecting a point on a shape (Parry `PointProjection`).
///
/// `point` is the closest point of the shape, `is_inside` tells whether the query point was
/// inside it. The boundary counts as inside (`<=`) for every shape. Upstream's `subshape` field
/// is dropped: the MVP shape set has no composite shape.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PointProjection {
    pub is_inside: bool,
    pub point: Vec2,
}

/// Where a point projects on a segment (Parry `SegmentPointLocation`).
///
/// `OnVertex(0)` is `a`, `OnVertex(1)` is `b`; `OnEdge((u, v))` carries the barycentric
/// coordinates of an interior point, `u + v = 1` and `point = a * u + b * v`, with `0 < v < 1`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum SegmentPointLocation {
    OnVertex: u32,
    OnEdge: (Fixed, Fixed),
}

#[generate_trait]
pub impl SegmentPointLocationImpl of SegmentPointLocationTrait {
    /// Returns the barycentric coordinates `(u, v)` of the location, so that the point is
    /// `a * u + b * v`.
    ///
    /// Mirrors `SegmentPointLocation::barycentric_coordinates`.
    /// #### Panics
    /// * Never (an `OnVertex` code other than 0 reads as `b`, like upstream).
    /// #### Deviations
    /// * Upstream returns a `[Real; 2]` array.
    #[inline(always)]
    fn barycentric_coordinates(self: SegmentPointLocation) -> (Fixed, Fixed) {
        match self {
            SegmentPointLocation::OnVertex(i) => if i == 0 {
                (ONE, ZERO)
            } else {
                (ZERO, ONE)
            },
            SegmentPointLocation::OnEdge(uv) => uv,
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, HALF, ONE, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use super::{PointProjection, SegmentPointLocation, SegmentPointLocationTrait};

    #[test]
    fn test_barycentric_coordinates() {
        let quarter = Fixed { raw: 0x40000000 };
        let cases: Span<(SegmentPointLocation, (Fixed, Fixed))> = array![
            (SegmentPointLocation::OnVertex(0), (ONE, ZERO)),
            (SegmentPointLocation::OnVertex(1), (ZERO, ONE)),
            (SegmentPointLocation::OnEdge((HALF, HALF)), (HALF, HALF)),
            (SegmentPointLocation::OnEdge((quarter, ONE - quarter)), (quarter, ONE - quarter)),
        ]
            .span();
        for (loc, expected) in cases {
            assert_eq!((*loc).barycentric_coordinates(), *expected);
        }
    }

    #[test]
    fn test_projection_equality_is_structural() {
        let p = PointProjection { is_inside: true, point: Vec2 { x: ONE, y: ZERO } };
        assert_eq!(p, PointProjection { is_inside: true, point: Vec2 { x: ONE, y: ZERO } });
        assert!(p != PointProjection { is_inside: false, point: Vec2 { x: ONE, y: ZERO } });
        assert!(p != PointProjection { is_inside: true, point: Vec2 { x: ZERO, y: ZERO } });
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_barycentric_coordinates_edge() {
        let _ = opaque(SegmentPointLocation::OnEdge((HALF, HALF))).barycentric_coordinates();
    }

    #[test]
    fn gas_barycentric_coordinates_vertex() {
        let _ = opaque(SegmentPointLocation::OnVertex(1)).barycentric_coordinates();
    }
}
