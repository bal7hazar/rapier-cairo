//! Ray casts on a [`BoundingSphere`] (Parry `query/ray/ray_bounding_sphere.rs`; PX3): the ball of
//! its radius, with the ray moved so that the centre is the origin.

use fixed::Fixed;
use crate::aabb::bounding_volume::BoundingSphere;
use crate::shape::Ball;
use super::cast::RayCast;
use super::{
    Ray, RayIntersection, RayTrait, cast_local_ray_and_get_normal_ball, cast_local_ray_ball,
};

/// `RayCast` of a bounding sphere: the casts of the [`Ball`] of the same radius on the ray
/// translated by `-center`. The hit point is `origin + dir * t`, as for every shape.
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `origin - center` leaves the scalar range;
///   see the ball casts.
pub impl BoundingSphereRayCast of RayCast<BoundingSphere> {
    fn cast_local_ray(
        self: BoundingSphere, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        let centered = ray.translate_by(-self.center);
        cast_local_ray_ball(Ball { radius: self.radius }, centered, max_time_of_impact, solid)
    }

    fn cast_local_ray_and_get_normal(
        self: BoundingSphere, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        let centered = ray.translate_by(-self.center);
        cast_local_ray_and_get_normal_ball(
            Ball { radius: self.radius }, centered, max_time_of_impact, solid,
        )
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, ONE, ZERO};
    use glam_core::Vec2;
    use rapier_testing::opaque;
    use crate::aabb::bounding_volume::BoundingSphere;
    use super::super::cast::RayCast;
    use super::{BoundingSphereRayCast, Ray};

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
    fn test_casts_on_the_moved_sphere() {
        let ray = Ray { origin: v(-4, 1), dir: v(1, 0) };
        // Enters at x = -1, after 3; the normal points out of the sphere.
        assert_eq!(RayCast::cast_local_ray(sphere(), ray, f(10), true), Some(f(3)));
        let hit = RayCast::cast_local_ray_and_get_normal(sphere(), ray, f(10), true).unwrap();
        assert_eq!((hit.time_of_impact, hit.normal), (f(3), Vec2 { x: -ONE, y: ZERO }));
        // Beyond the maximum, missing, and from inside.
        assert!(RayCast::cast_local_ray(sphere(), ray, f(2), true).is_none());
        let above = Ray { origin: v(-4, 4), dir: v(1, 0) };
        assert!(!RayCast::intersects_local_ray(sphere(), above, f(10)));
        let inside = Ray { origin: v(1, 1), dir: v(1, 0) };
        assert_eq!(RayCast::cast_local_ray(sphere(), inside, f(10), true), Some(ZERO));
        assert_eq!(RayCast::cast_local_ray(sphere(), inside, f(10), false), Some(f(2)));
        assert!(RayCast::intersects_local_ray(sphere(), ray, f(10)));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_cast_local_ray() {
        let ray = Ray { origin: v(-4, 1), dir: v(1, 0) };
        let _ = RayCast::cast_local_ray(opaque(sphere()), opaque(ray), opaque(f(10)), opaque(true));
    }

    #[test]
    fn gas_cast_local_ray_and_get_normal() {
        let ray = Ray { origin: v(-4, 1), dir: v(1, 0) };
        let _ = RayCast::cast_local_ray_and_get_normal(
            opaque(sphere()), opaque(ray), opaque(f(10)), opaque(true),
        );
    }
}
