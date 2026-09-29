//! Point queries on a [`BoundingSphere`] (Parry `query/point/point_bounding_sphere.rs`; PX3):
//! the ball of its radius, moved to its centre.

use fixed::Fixed;
use glam_core::Vec2;
use crate::aabb::bounding_volume::BoundingSphere;
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::shape::Ball;
use super::query::PointQuery;
use super::{
    PointProjection, contains_local_point_ball, distance_to_local_point_ball,
    project_local_point_ball,
};

/// `PointQuery` of a bounding sphere: the queries of the [`Ball`] of the same radius on the point
/// relative to the centre, the projection moved back. The feature is always `Face(0)`.
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_add Overflow'` if `pt - center` or the projected point leaves
///   the scalar range; see the ball queries.
pub impl BoundingSpherePointQuery of PointQuery<BoundingSphere> {
    fn project_local_point(self: BoundingSphere, pt: Vec2, solid: bool) -> PointProjection {
        let mut proj = project_local_point_ball(
            Ball { radius: self.radius }, pt - self.center, solid,
        );
        proj.point = proj.point + self.center;
        proj
    }

    fn project_local_point_and_get_feature(
        self: BoundingSphere, pt: Vec2,
    ) -> (PointProjection, FeatureId) {
        (Self::project_local_point(self, pt, false), FeatureIdTrait::face(0))
    }

    fn distance_to_local_point(self: BoundingSphere, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_ball(Ball { radius: self.radius }, pt - self.center, solid)
    }

    fn contains_local_point(self: BoundingSphere, pt: Vec2) -> bool {
        contains_local_point_ball(Ball { radius: self.radius }, pt - self.center)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ONE, ZERO};
    use glam_core::Vec2;
    use rapier_testing::opaque;
    use crate::aabb::bounding_volume::BoundingSphere;
    use crate::feature_id::FeatureIdTrait;
    use super::super::PointProjection;
    use super::{BoundingSpherePointQuery, PointQuery};

    fn f(n: i64) -> Fixed {
        Fixed { raw: n * 0x1_0000_0000 }
    }

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: f(x), y: f(y) }
    }

    /// The sphere of radius 2 centred on `(1, 1)`.
    fn sphere() -> BoundingSphere {
        BoundingSphere { center: v(1, 1), radius: f(2) }
    }

    #[test]
    fn test_projection_distance_and_containment() {
        // Outside, on the x axis of the centre: the surface point, distance 3 - 2.
        let p = PointQuery::project_local_point(sphere(), v(6, 1), true);
        assert_eq!(p, PointProjection { is_inside: false, point: v(3, 1) });
        assert_eq!(PointQuery::distance_to_local_point(sphere(), v(6, 1), true), f(3));
        // Inside: solid answers the point, hollow the surface; the boundary is inside.
        let inside = v(2, 1);
        let solid = PointQuery::project_local_point(sphere(), inside, true);
        assert_eq!(solid, PointProjection { is_inside: true, point: inside });
        let hollow = PointQuery::project_local_point(sphere(), inside, false);
        assert_eq!(hollow, PointProjection { is_inside: true, point: v(3, 1) });
        assert_eq!(PointQuery::distance_to_local_point(sphere(), inside, false), -ONE);
        assert_eq!(PointQuery::distance_to_local_point(sphere(), inside, true), ZERO);
        assert!(PointQuery::contains_local_point(sphere(), v(3, 1)));
        assert!(!PointQuery::contains_local_point(sphere(), v(4, 1)));
    }

    #[test]
    fn test_feature_is_the_face_and_the_projection_is_hollow() {
        let (proj, feature) = PointQuery::project_local_point_and_get_feature(sphere(), v(2, 1));
        assert_eq!(feature, FeatureIdTrait::face(0));
        assert_eq!(proj, PointQuery::project_local_point(sphere(), v(2, 1), false));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_project_local_point() {
        let _ = PointQuery::project_local_point(opaque(sphere()), opaque(v(6, 2)), opaque(true));
    }

    #[test]
    fn gas_project_local_point_and_get_feature() {
        let _ = PointQuery::project_local_point_and_get_feature(opaque(sphere()), opaque(v(6, 2)));
    }

    #[test]
    fn gas_distance_to_local_point() {
        let _ = PointQuery::distance_to_local_point(
            opaque(sphere()), opaque(v(6, 2)), opaque(true),
        );
    }

    #[test]
    fn gas_contains_local_point() {
        let _ = PointQuery::contains_local_point(opaque(sphere()), opaque(v(6, 2)));
    }
}
