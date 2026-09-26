//! Parry's `RayCast` trait over the closed shape set and [`Aabb`] (work package QY1).
//!
//! The per-shape free functions of [`crate::ray`] stay the kernels; this module gives them
//! upstream's trait surface, generic over the shape type, with upstream's provided methods as
//! default implementations (`cast_ray*` move the ray into the local frame first,
//! `intersects_*` are solid casts). Every implementor overrides `cast_local_ray` with its own
//! kernel: the cuboid's (and hence the AABB's) differs from `cast_local_ray_and_get_normal` on a
//! hollow ray starting inside, as upstream's does. [`Shape`] dispatches by `match`, inlined.
//!
//! # Deviations
//!
//! * `self` is taken by value instead of `&self`.

use fixed::Fixed;
use rapier_math::pose2::Pose2;
use crate::aabb::{Aabb, cast_local_ray_aabb, cast_local_ray_and_get_normal_aabb};
use crate::shape::{
    Ball, Capsule, ConvexPolygon, Cuboid, HalfSpace, RoundConvexPolygon, RoundCuboid, RoundTriangle,
    Segment, Shape, Triangle,
};
use super::convex_polygon::{
    cast_local_ray_and_get_normal_convex_polygon, cast_local_ray_convex_polygon,
};
use super::round_shape::{
    cast_local_ray_and_get_normal_round_convex_polygon, cast_local_ray_and_get_normal_round_cuboid,
    cast_local_ray_and_get_normal_round_triangle,
};
use super::triangle::{cast_local_ray_and_get_normal_triangle, cast_local_ray_triangle};
use super::{
    Ray, RayIntersection, RayIntersectionTrait, RayTrait, cast_local_ray,
    cast_local_ray_and_get_normal, cast_local_ray_and_get_normal_ball,
    cast_local_ray_and_get_normal_capsule, cast_local_ray_and_get_normal_cuboid,
    cast_local_ray_and_get_normal_halfspace, cast_local_ray_and_get_normal_segment,
    cast_local_ray_ball, cast_local_ray_capsule, cast_local_ray_cuboid, cast_local_ray_halfspace,
    cast_local_ray_segment,
};

/// Ray casts on a shape (Parry `RayCast`). `max_time_of_impact` is inclusive and `solid`
/// follows the semantics of [`crate::ray`].
pub trait RayCast<T, +Drop<T>> {
    /// Time of impact of `ray` (local frame of the shape).
    fn cast_local_ray(
        self: T, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<
        Fixed,
    > {
        match Self::cast_local_ray_and_get_normal(self, ray, max_time_of_impact, solid) {
            Some(hit) => Some(hit.time_of_impact),
            None => None,
        }
    }

    /// Time of impact, normal and feature of `ray` (local frame of the shape).
    fn cast_local_ray_and_get_normal(
        self: T, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection>;

    /// Whether the solid shape is hit within `max_time_of_impact` (local frame).
    fn intersects_local_ray(
        self: T, ray: Ray, max_time_of_impact: Fixed,
    ) -> bool {
        Self::cast_local_ray(self, ray, max_time_of_impact, true).is_some()
    }

    /// [`RayCast::cast_local_ray`] for the shape placed at `m` and a ray in the frame of `m`.
    fn cast_ray(
        self: T, m: Pose2, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<
        Fixed,
    > {
        Self::cast_local_ray(self, ray.inverse_transform_by(m), max_time_of_impact, solid)
    }

    /// [`RayCast::cast_local_ray_and_get_normal`] for the shape placed at `m`; the normal is
    /// rotated back into the frame of `m`.
    fn cast_ray_and_get_normal(
        self: T, m: Pose2, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<
        RayIntersection,
    > {
        let hit = Self::cast_local_ray_and_get_normal(
            self, ray.inverse_transform_by(m), max_time_of_impact, solid,
        )?;
        Some(hit.transform_by(m))
    }

    /// [`RayCast::intersects_local_ray`] for the shape placed at `m`.
    fn intersects_ray(
        self: T, m: Pose2, ray: Ray, max_time_of_impact: Fixed,
    ) -> bool {
        Self::intersects_local_ray(self, ray.inverse_transform_by(m), max_time_of_impact)
    }
}

pub impl BallRayCast of RayCast<Ball> {
    fn cast_local_ray(
        self: Ball, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        cast_local_ray_ball(self, ray, max_time_of_impact, solid)
    }
    fn cast_local_ray_and_get_normal(
        self: Ball, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal_ball(self, ray, max_time_of_impact, solid)
    }
}

pub impl CuboidRayCast of RayCast<Cuboid> {
    fn cast_local_ray(
        self: Cuboid, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        cast_local_ray_cuboid(self, ray, max_time_of_impact, solid)
    }
    fn cast_local_ray_and_get_normal(
        self: Cuboid, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal_cuboid(self, ray, max_time_of_impact, solid)
    }
}

pub impl CapsuleRayCast of RayCast<Capsule> {
    fn cast_local_ray(
        self: Capsule, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        cast_local_ray_capsule(self, ray, max_time_of_impact, solid)
    }
    fn cast_local_ray_and_get_normal(
        self: Capsule, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal_capsule(self, ray, max_time_of_impact, solid)
    }
}

pub impl SegmentRayCast of RayCast<Segment> {
    fn cast_local_ray(
        self: Segment, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        cast_local_ray_segment(self, ray, max_time_of_impact, solid)
    }
    fn cast_local_ray_and_get_normal(
        self: Segment, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal_segment(self, ray, max_time_of_impact, solid)
    }
}

pub impl HalfSpaceRayCast of RayCast<HalfSpace> {
    fn cast_local_ray(
        self: HalfSpace, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        cast_local_ray_halfspace(self, ray, max_time_of_impact, solid)
    }
    fn cast_local_ray_and_get_normal(
        self: HalfSpace, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal_halfspace(self, ray, max_time_of_impact, solid)
    }
}

pub impl ConvexPolygonRayCast of RayCast<ConvexPolygon> {
    fn cast_local_ray(
        self: ConvexPolygon, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        cast_local_ray_convex_polygon(self, ray, max_time_of_impact, solid)
    }
    fn cast_local_ray_and_get_normal(
        self: ConvexPolygon, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal_convex_polygon(self, ray, max_time_of_impact, solid)
    }
}

pub impl TriangleRayCast of RayCast<Triangle> {
    fn cast_local_ray(
        self: Triangle, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        cast_local_ray_triangle(self, ray, max_time_of_impact, solid)
    }
    fn cast_local_ray_and_get_normal(
        self: Triangle, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal_triangle(self, ray, max_time_of_impact, solid)
    }
}

pub impl RoundCuboidRayCast of RayCast<RoundCuboid> {
    fn cast_local_ray(
        self: RoundCuboid, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        Some(
            Self::cast_local_ray_and_get_normal(self, ray, max_time_of_impact, solid)?
                .time_of_impact,
        )
    }
    fn cast_local_ray_and_get_normal(
        self: RoundCuboid, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal_round_cuboid(
            self.inner_shape, self.border_radius, ray, max_time_of_impact, solid,
        )
    }
}

pub impl RoundTriangleRayCast of RayCast<RoundTriangle> {
    fn cast_local_ray(
        self: RoundTriangle, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        Some(
            Self::cast_local_ray_and_get_normal(self, ray, max_time_of_impact, solid)?
                .time_of_impact,
        )
    }
    fn cast_local_ray_and_get_normal(
        self: RoundTriangle, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal_round_triangle(
            self.inner_shape, self.border_radius, ray, max_time_of_impact, solid,
        )
    }
}

pub impl RoundConvexPolygonRayCast of RayCast<RoundConvexPolygon> {
    fn cast_local_ray(
        self: RoundConvexPolygon, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        Some(
            Self::cast_local_ray_and_get_normal(self, ray, max_time_of_impact, solid)?
                .time_of_impact,
        )
    }
    fn cast_local_ray_and_get_normal(
        self: RoundConvexPolygon, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal_round_convex_polygon(
            self.inner_shape, self.border_radius, ray, max_time_of_impact, solid,
        )
    }
}

pub impl AabbRayCast of RayCast<Aabb> {
    fn cast_local_ray(
        self: Aabb, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        cast_local_ray_aabb(self, ray, max_time_of_impact, solid)
    }
    fn cast_local_ray_and_get_normal(
        self: Aabb, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal_aabb(self, ray, max_time_of_impact, solid)
    }
}

/// `match` dispatch over the closed set ([`cast_local_ray`] / [`cast_local_ray_and_get_normal`]
/// of the parent module), inlined.
pub impl ShapeRayCast of RayCast<Shape> {
    #[inline(always)]
    fn cast_local_ray(
        self: Shape, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<Fixed> {
        cast_local_ray(self, ray, max_time_of_impact, solid)
    }
    #[inline(always)]
    fn cast_local_ray_and_get_normal(
        self: Shape, ray: Ray, max_time_of_impact: Fixed, solid: bool,
    ) -> Option<RayIntersection> {
        cast_local_ray_and_get_normal(self, ray, max_time_of_impact, solid)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::aabb::{Aabb, AabbTrait};
    use crate::ray::{RayIntersectionTrait, RayTrait};
    use crate::shape::{
        BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait,
        Shape,
    };
    use super::{RayCast, ShapeRayCast};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    fn quarter() -> Pose2 {
        Pose2Trait::new(v(ONE, TWO), Rot2 { re: ZERO, im: ONE })
    }

    /// Every shape, a ray through the placed shape: the trait answers what the `Shape` free
    /// functions answer, and the posed casts are the local ones on the moved ray.
    #[test]
    fn test_trait_matches_free_functions() {
        let triangle = ConvexPolygonTrait::from_convex_polyline(
            array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span(),
        )
            .unwrap();
        let shapes = array![
            Shape::Ball(BallTrait::new(HALF)), Shape::Cuboid(CuboidTrait::new(v(HALF, HALF))),
            Shape::Capsule(CapsuleTrait::new_x(HALF, HALF)),
            Shape::Segment(SegmentTrait::new(v(ZERO, -ONE), v(ZERO, ONE))),
            Shape::HalfSpace(HalfSpaceTrait::new(v(-ONE, ZERO))),
            Shape::ConvexPolygon(BoxTrait::new(triangle)),
        ];
        let m = quarter();
        let world = RayTrait::new(v(ONE, int(-2)), v(ZERO, ONE));
        let local = world.inverse_transform_by(m);
        for shape in shapes.span() {
            let expected = crate::ray::cast_local_ray_and_get_normal(*shape, local, int(100), true);
            assert_eq!((*shape).cast_local_ray_and_get_normal(local, int(100), true), expected);
            match expected {
                Some(hit) => {
                    assert_eq!(
                        (*shape).cast_ray_and_get_normal(m, world, int(100), true),
                        Some(hit.transform_by(m)),
                    );
                    assert!((*shape).intersects_ray(m, world, int(100)));
                },
                None => assert!(!(*shape).intersects_ray(m, world, int(100))),
            }
            assert_eq!(
                (*shape).cast_ray(m, world, int(100), false),
                crate::ray::cast_local_ray(*shape, local, int(100), false),
            );
        }
    }

    /// `(aabb, ray, max, solid, toi)`: the AABB answers the cuboid of its half extents on the
    /// ray moved to its centre.
    #[test]
    fn test_aabb_casts_table() {
        let unit = AabbTrait::new(v(ZERO, ZERO), v(TWO, TWO));
        let cases: Span<(Aabb, Vec2, Vec2, bool, Option<Fixed>)> = array![
            (unit, v(int(-2), ONE), v(ONE, ZERO), true, Some(TWO)),
            (unit, v(ONE, ONE), v(ONE, ZERO), true, Some(ZERO)),
            (unit, v(ONE, ONE), v(ONE, ZERO), false, Some(ONE)),
            (unit, v(int(-2), int(5)), v(ONE, ZERO), true, None),
            (unit, v(int(-2), ONE), v(-ONE, ZERO), true, None),
        ]
            .span();
        for (aabb, origin, dir, solid, toi) in cases {
            let ray = RayTrait::new(*origin, *dir);
            assert_eq!((*aabb).cast_local_ray(ray, int(10), *solid), *toi);
            let hit = (*aabb).cast_local_ray_and_get_normal(ray, int(10), *solid);
            assert_eq!(hit.is_some(), toi.is_some());
        }
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_shape_cast_ray_and_get_normal_ball() {
        let _ = ShapeRayCast::cast_ray_and_get_normal(
            opaque(Shape::Ball(BallTrait::new(HALF))),
            opaque(quarter()),
            opaque(RayTrait::new(v(int(-2), TWO), v(ONE, ZERO))),
            opaque(int(100)),
            true,
        );
    }

    #[test]
    fn gas_aabb_cast_local_ray_and_get_normal() {
        let _ = opaque(AabbTrait::new(v(ZERO, ZERO), v(TWO, TWO)))
            .cast_local_ray_and_get_normal(
                opaque(RayTrait::new(v(int(-2), ONE), v(ONE, ZERO))), opaque(int(10)), true,
            );
    }

    #[test]
    fn gas_aabb_intersects_local_ray() {
        let _ = opaque(AabbTrait::new(v(ZERO, ZERO), v(TWO, TWO)))
            .intersects_local_ray(
                opaque(RayTrait::new(v(int(-2), ONE), v(ONE, ZERO))), opaque(int(10)),
            );
    }
}
