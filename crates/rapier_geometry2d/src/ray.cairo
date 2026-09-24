//! Ray casts against the closed shape set (Parry `query/ray/`, work package QP).
//!
//! Upstream exposes the `RayCast` trait on every shape; a closed shape set needs no dynamic
//! dispatch, so every shape gets free functions named after it (`cast_local_ray_ball`, …, one
//! submodule each) and [`cast_local_ray`] / [`cast_local_ray_and_get_normal`] dispatch on
//! [`Shape`] by `match`. The `Pose2` wrappers [`cast_ray`] and [`cast_ray_and_get_normal`] move the
//! ray into the local frame of the shape, as upstream's default trait methods do.
//!
//! # Semantics shared by every shape
//!
//! * `ray.dir` is **not** normalised: a time of impact `t` is the point `origin + dir * t`.
//! * `solid = true`: a ray starting inside answers `t = 0`; `solid = false` (hollow): it answers
//!   the exit point, with the normal pointing **into** the shape, as upstream.
//! * `max_time_of_impact` is inclusive (`t <= max`) for every shape. The world-level queries of
//!   `rapier2d` are exclusive, as upstream's BVH traversal is.
//! * Normals are unit vectors in the frame of the query, or zero where upstream answers zero
//!   (solid ray starting inside a cuboid or a half-space).
//!
//! # Fixed-point hazards answered here
//!
//! Every time of impact is one correctly rounded quotient of two exact wide quantities
//! ([`quotient::div_wide`]); the circle casts take the square root of an exact 256-bit
//! discriminant ([`quotient::isqrt_wide`]). Discrete decisions (inside / outside, which side, the
//! sign of `t`) are read off the exact numerators, never off a rounded time. See each submodule
//! for its deviations; the capsule is the notable one (analytic here, GJK upstream).

pub mod ball;
pub mod capsule;
pub mod convex_polygon;
pub mod cuboid;
pub mod halfspace;
pub mod quotient;
pub mod segment;
pub use ball::{cast_local_ray_and_get_normal_ball, cast_local_ray_ball};
pub use capsule::{cast_local_ray_and_get_normal_capsule, cast_local_ray_capsule};
pub use cuboid::{cast_local_ray_and_get_normal_cuboid, cast_local_ray_cuboid};
use fixed::Fixed;
use glam::vec2::{Vec2, Vec2Trait};
pub use halfspace::{cast_local_ray_and_get_normal_halfspace, cast_local_ray_halfspace};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
pub use segment::{cast_local_ray_and_get_normal_segment, cast_local_ray_segment};
use crate::feature_id::FeatureId;
use crate::shape::Shape;

/// A ray (Parry `Ray`): the half-line `origin + dir * t`, `t >= 0`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Ray {
    pub origin: Vec2,
    /// Not required to be unit: times of impact are in units of `dir`.
    pub dir: Vec2,
}

/// Where a ray hit a shape (Parry `RayIntersection`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RayIntersection {
    /// The hit point is `origin + dir * time_of_impact`.
    pub time_of_impact: Fixed,
    /// Unit normal at the hit point (see the module documentation for its orientation), zero
    /// where upstream answers zero.
    pub normal: Vec2,
    /// The feature hit, with the codes upstream reports for that shape.
    pub feature: FeatureId,
}

#[generate_trait]
pub impl RayImpl of RayTrait {
    #[inline(always)]
    fn new(origin: Vec2, dir: Vec2) -> Ray {
        Ray { origin, dir }
    }

    /// `origin + dir * t`.
    /// #### Panics
    /// * `'Fixed: overflow'` / `'i64_add Overflow'` when the point leaves the scalar range.
    #[inline(always)]
    fn point_at(self: Ray, t: Fixed) -> Vec2 {
        self.origin + self.dir.mul_scalar(t)
    }

    /// The ray moved by `pose` (upstream `transform_by`).
    #[inline(always)]
    fn transform_by(self: Ray, pose: Pose2) -> Ray {
        Ray { origin: pose.transform_point(self.origin), dir: pose.rotation.rotate(self.dir) }
    }

    /// The ray in the local frame of `pose` (upstream `inverse_transform_by`).
    #[inline(always)]
    fn inverse_transform_by(self: Ray, pose: Pose2) -> Ray {
        Ray {
            origin: pose.inverse_transform_point(self.origin),
            dir: pose.rotation.inverse_rotate(self.dir),
        }
    }
}

#[generate_trait]
pub impl RayIntersectionImpl of RayIntersectionTrait {
    /// The intersection with its normal rotated by `pose` (upstream `transform_by`).
    #[inline(always)]
    fn transform_by(self: RayIntersection, pose: Pose2) -> RayIntersection {
        RayIntersection {
            time_of_impact: self.time_of_impact,
            normal: pose.rotation.rotate(self.normal),
            feature: self.feature,
        }
    }
}

/// Time of impact of `ray` on `shape`, in the local frame of the shape.
///
/// Mirrors `RayCast::cast_local_ray`. Inlined so that the caller pays the arm it reaches.
/// #### Panics
/// * See the per-shape functions.
#[inline(always)]
pub fn cast_local_ray(
    shape: Shape, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    match shape {
        Shape::Ball(s) => cast_local_ray_ball(s, ray, max_time_of_impact, solid),
        Shape::Cuboid(s) => cast_local_ray_cuboid(s, ray, max_time_of_impact, solid),
        Shape::Capsule(s) => cast_local_ray_capsule(s, ray, max_time_of_impact, solid),
        Shape::Segment(s) => cast_local_ray_segment(s, ray, max_time_of_impact, solid),
        Shape::HalfSpace(s) => cast_local_ray_halfspace(s, ray, max_time_of_impact, solid),
        Shape::ConvexPolygon(s) => convex_polygon::cast_local_ray_convex_polygon(
            s.unbox(), ray, max_time_of_impact, solid,
        ),
    }
}

/// Time of impact, normal and feature of `ray` on `shape`, in the local frame of the shape.
///
/// Mirrors `RayCast::cast_local_ray_and_get_normal`. Inlined, see [`cast_local_ray`].
/// #### Panics
/// * See the per-shape functions.
#[inline(always)]
pub fn cast_local_ray_and_get_normal(
    shape: Shape, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    match shape {
        Shape::Ball(s) => cast_local_ray_and_get_normal_ball(s, ray, max_time_of_impact, solid),
        Shape::Cuboid(s) => cast_local_ray_and_get_normal_cuboid(s, ray, max_time_of_impact, solid),
        Shape::Capsule(s) => cast_local_ray_and_get_normal_capsule(
            s, ray, max_time_of_impact, solid,
        ),
        Shape::Segment(s) => cast_local_ray_and_get_normal_segment(
            s, ray, max_time_of_impact, solid,
        ),
        Shape::ConvexPolygon(s) => convex_polygon::cast_local_ray_and_get_normal_convex_polygon(
            s.unbox(), ray, max_time_of_impact, solid,
        ),
        Shape::HalfSpace(s) => cast_local_ray_and_get_normal_halfspace(
            s, ray, max_time_of_impact, solid,
        ),
    }
}

/// Time of impact of the world-space `ray` on `shape` placed at `pose`.
///
/// Mirrors `RayCast::cast_ray`: the ray is moved into the local frame first.
/// #### Panics
/// * See [`cast_local_ray`].
#[inline(always)]
pub fn cast_ray(
    shape: Shape, pose: Pose2, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<Fixed> {
    cast_local_ray(shape, ray.inverse_transform_by(pose), max_time_of_impact, solid)
}

/// Time of impact, world-space normal and feature of the world-space `ray` on `shape` placed at
/// `pose`.
///
/// Mirrors `RayCast::cast_ray_and_get_normal`.
/// #### Panics
/// * See [`cast_local_ray_and_get_normal`].
#[inline(always)]
pub fn cast_ray_and_get_normal(
    shape: Shape, pose: Pose2, ray: Ray, max_time_of_impact: Fixed, solid: bool,
) -> Option<RayIntersection> {
    let hit = cast_local_ray_and_get_normal(
        shape, ray.inverse_transform_by(pose), max_time_of_impact, solid,
    )?;
    Some(hit.transform_by(pose))
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::vec2::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::feature_id::FeatureIdTrait;
    use crate::shape::{BallTrait, CapsuleTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait, Shape};
    use super::{
        Ray, RayIntersection, RayIntersectionTrait, RayTrait, cast_local_ray,
        cast_local_ray_and_get_normal, cast_ray, cast_ray_and_get_normal,
    };

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    /// Quarter turn around `(1, 2)`.
    fn quarter() -> Pose2 {
        Pose2Trait::new(v(ONE, TWO), Rot2 { re: ZERO, im: ONE })
    }

    fn shapes() -> Span<Shape> {
        array![
            Shape::Ball(BallTrait::new(HALF)), Shape::Cuboid(CuboidTrait::new(v(HALF, HALF))),
            Shape::Capsule(CapsuleTrait::new_x(HALF, HALF)),
            Shape::Segment(SegmentTrait::new(v(ZERO, -ONE), v(ZERO, ONE))),
            Shape::HalfSpace(HalfSpaceTrait::new(v(-ONE, ZERO))),
        ]
            .span()
    }

    #[test]
    fn test_ray_transforms_round_trip() {
        let ray = RayTrait::new(v(int(3), int(-1)), v(ONE, TWO));
        let local = ray.inverse_transform_by(quarter());
        assert_eq!(local, Ray { origin: v(int(-3), int(-2)), dir: v(TWO, -ONE) });
        assert_eq!(local.transform_by(quarter()), ray);
        assert_eq!(ray.point_at(HALF), v(int(3) + HALF, ZERO));
        let hit = RayIntersection {
            time_of_impact: ONE, normal: v(ONE, ZERO), feature: FeatureIdTrait::face(0),
        };
        assert_eq!(hit.transform_by(quarter()).normal, v(ZERO, ONE));
    }

    /// Every shape, a ray from `(-3, 0)` along `+x`: the posed cast equals the local cast of the
    /// moved ray, and the normal is rotated back.
    #[test]
    fn test_posed_casts_match_local_casts() {
        let pose = quarter();
        let world = RayTrait::new(v(ONE, int(-1)), v(ZERO, ONE));
        let local = world.inverse_transform_by(pose);
        for shape in shapes() {
            let expected = cast_local_ray_and_get_normal(*shape, local, int(100), true);
            let posed = cast_ray_and_get_normal(*shape, pose, world, int(100), true);
            match expected {
                Some(hit) => assert_eq!(posed, Some(hit.transform_by(pose))),
                None => assert_eq!(posed, None),
            }
            assert_eq!(
                cast_ray(*shape, pose, world, int(100), true),
                cast_local_ray(*shape, local, int(100), true),
            );
        }
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_cast_ray_ball() {
        let _ = cast_ray(
            opaque(Shape::Ball(BallTrait::new(HALF))),
            opaque(quarter()),
            opaque(RayTrait::new(v(int(-2), TWO), v(ONE, ZERO))),
            opaque(int(100)),
            true,
        );
    }

    #[test]
    fn gas_cast_ray_and_get_normal_cuboid() {
        let _ = cast_ray_and_get_normal(
            opaque(Shape::Cuboid(CuboidTrait::new(v(HALF, HALF)))),
            opaque(quarter()),
            opaque(RayTrait::new(v(int(-2), TWO), v(ONE, ZERO))),
            opaque(int(100)),
            true,
        );
    }
}
