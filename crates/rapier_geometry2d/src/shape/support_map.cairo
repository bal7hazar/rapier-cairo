//! The support-map trait (Parry `shape/support_map.rs`) over the shapes of the closed set.
//!
//! Each shape keeps its inherent `local_support_point` (the step calls those); the trait
//! delegates to them so generic helpers (`local_support_map_aabb`) and the [`Shape`] view
//! returned by `ShapeTrait::as_support_map` can be used uniformly. `HalfSpace` is not a support
//! map (as upstream).

use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::shape::{
    Ball, BallTrait, Capsule, CapsuleTrait, ConvexPolygon, ConvexPolygonTrait, Cuboid, CuboidTrait,
    RoundConvexPolygonShapeTrait, Segment, SegmentTrait, Shape, Triangle, TriangleTrait,
};

/// A convex shape described by its support function (upstream `SupportMap`).
///
/// `dir` need not be unit for `local_support_point`; `*_toward` expects a unit direction (the
/// ball and capsule skip their normalisation).
pub trait SupportMap<T, +Drop<T>> {
    /// The point of the shape farthest along `dir`, in its local frame.
    fn local_support_point(self: T, dir: Vec2) -> Vec2;
    /// Same as `local_support_point` for a unit `dir`.
    fn local_support_point_toward(self: T, dir: Vec2) -> Vec2;
    /// The support point of the shape placed at `pose`, for a world-space `dir`: the direction
    /// is rotated back, the local support point transformed (one floor per component each).
    fn support_point(
        self: T, pose: Pose2, dir: Vec2,
    ) -> Vec2 {
        pose.transform_point(Self::local_support_point(self, pose.rotation.inverse_rotate(dir)))
    }
    /// `support_point` for a unit `dir`.
    fn support_point_toward(
        self: T, pose: Pose2, dir: Vec2,
    ) -> Vec2 {
        pose
            .transform_point(
                Self::local_support_point_toward(self, pose.rotation.inverse_rotate(dir)),
            )
    }
}

pub impl BallSupportMap of SupportMap<Ball> {
    #[inline(always)]
    fn local_support_point(self: Ball, dir: Vec2) -> Vec2 {
        BallTrait::local_support_point(self, dir)
    }
    #[inline(always)]
    fn local_support_point_toward(self: Ball, dir: Vec2) -> Vec2 {
        BallTrait::local_support_point_toward(self, dir)
    }
}

pub impl CuboidSupportMap of SupportMap<Cuboid> {
    #[inline(always)]
    fn local_support_point(self: Cuboid, dir: Vec2) -> Vec2 {
        CuboidTrait::local_support_point(self, dir)
    }
    #[inline(always)]
    fn local_support_point_toward(self: Cuboid, dir: Vec2) -> Vec2 {
        CuboidTrait::local_support_point(self, dir)
    }
}

pub impl CapsuleSupportMap of SupportMap<Capsule> {
    #[inline(always)]
    fn local_support_point(self: Capsule, dir: Vec2) -> Vec2 {
        CapsuleTrait::local_support_point(self, dir)
    }
    #[inline(always)]
    fn local_support_point_toward(self: Capsule, dir: Vec2) -> Vec2 {
        CapsuleTrait::local_support_point_toward(self, dir)
    }
}

pub impl SegmentSupportMap of SupportMap<Segment> {
    #[inline(always)]
    fn local_support_point(self: Segment, dir: Vec2) -> Vec2 {
        SegmentTrait::local_support_point(self, dir)
    }
    #[inline(always)]
    fn local_support_point_toward(self: Segment, dir: Vec2) -> Vec2 {
        SegmentTrait::local_support_point(self, dir)
    }
}

pub impl ConvexPolygonSupportMap of SupportMap<ConvexPolygon> {
    #[inline(always)]
    fn local_support_point(self: ConvexPolygon, dir: Vec2) -> Vec2 {
        ConvexPolygonTrait::local_support_point(self, dir)
    }
    #[inline(always)]
    fn local_support_point_toward(self: ConvexPolygon, dir: Vec2) -> Vec2 {
        ConvexPolygonTrait::local_support_point(self, dir)
    }
}

pub impl TriangleSupportMap of SupportMap<Triangle> {
    #[inline(always)]
    fn local_support_point(self: Triangle, dir: Vec2) -> Vec2 {
        TriangleTrait::local_support_point(self, dir)
    }
    #[inline(always)]
    fn local_support_point_toward(self: Triangle, dir: Vec2) -> Vec2 {
        TriangleTrait::local_support_point(self, dir)
    }
}

/// Dispatch over the closed set.
/// #### Panics
/// * `'Query: not a support map'` for a half-space.
pub impl ShapeSupportMap of SupportMap<Shape> {
    fn local_support_point(self: Shape, dir: Vec2) -> Vec2 {
        match self {
            Shape::Ball(s) => BallTrait::local_support_point(s, dir),
            Shape::Cuboid(s) => CuboidTrait::local_support_point(s, dir),
            Shape::Capsule(s) => CapsuleTrait::local_support_point(s, dir),
            Shape::Segment(s) => SegmentTrait::local_support_point(s, dir),
            Shape::HalfSpace(_) => core::panic_with_felt252(crate::query::errors::NOT_SUPPORT_MAP),
            Shape::ConvexPolygon(s) => ConvexPolygonTrait::local_support_point(s.unbox(), dir),
            Shape::Triangle(s) => TriangleTrait::local_support_point(s.unbox(), dir),
            Shape::RoundCuboid(s) => s.local_support_point(dir),
            Shape::RoundTriangle(s) => s.unbox().local_support_point(dir),
            Shape::RoundConvexPolygon(s) => s.unbox().to_round().local_support_point(dir),
        }
    }
    fn local_support_point_toward(self: Shape, dir: Vec2) -> Vec2 {
        crate::query::support_map::local_support_point_toward(self, dir)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::shape::{
        BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait,
        Shape,
    };
    use super::SupportMap;

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn shapes() -> Span<Shape> {
        let square = ConvexPolygonTrait::from_convex_polyline(
            array![v(-ONE, -ONE), v(ONE, -ONE), v(ONE, ONE), v(-ONE, ONE)].span(),
        )
            .unwrap();
        array![
            Shape::Ball(BallTrait::new(HALF)), Shape::Cuboid(CuboidTrait::new(v(TWO, ONE))),
            Shape::Capsule(CapsuleTrait::new_x(ONE, HALF)),
            Shape::Segment(SegmentTrait::new(v(-ONE, ZERO), v(ONE, ONE))),
            Shape::ConvexPolygon(BoxTrait::new(square)),
        ]
            .span()
    }

    #[test]
    fn test_local_support_points_table() {
        // (direction, expected per shape): axis-aligned directions are exact for every shape.
        let x = v(ONE, ZERO);
        let ny = v(ZERO, -ONE);
        let expected_x = array![
            v(HALF, ZERO), v(TWO, ONE), v(ONE + HALF, ZERO), v(ONE, ONE), v(ONE, -ONE),
        ];
        let expected_ny = array![
            v(ZERO, -HALF), v(TWO, -ONE), v(ONE, -HALF), v(-ONE, ZERO), v(-ONE, -ONE),
        ];
        let mut i = 0;
        for shape in shapes() {
            assert_eq!(SupportMap::local_support_point(*shape, x), *expected_x.at(i));
            assert_eq!(SupportMap::local_support_point_toward(*shape, x), *expected_x.at(i));
            assert_eq!(SupportMap::local_support_point(*shape, ny), *expected_ny.at(i));
            i += 1;
        }
        // Per-type impls agree with the dispatch.
        let ball = BallTrait::new(HALF);
        assert_eq!(SupportMap::local_support_point(ball, x), v(HALF, ZERO));
        let cuboid = CuboidTrait::new(v(TWO, ONE));
        assert_eq!(SupportMap::local_support_point_toward(cuboid, ny), v(TWO, -ONE));
    }

    #[test]
    fn test_support_point_in_world_frame() {
        // Quarter turn + (10, 0): world +X is local -Y.
        let pose = Pose2Trait::new(v(FixedTrait::from_int(10), ZERO), Rot2 { re: ZERO, im: ONE });
        let cuboid = CuboidTrait::new(v(TWO, ONE));
        // Local support along -Y is (2, -1) (zero counts positive), placed: (10 + 1, 2).
        let expected = v(FixedTrait::from_int(11), TWO);
        assert_eq!(SupportMap::support_point(cuboid, pose, v(ONE, ZERO)), expected);
        assert_eq!(SupportMap::support_point_toward(cuboid, pose, v(ONE, ZERO)), expected);
        let shape = Shape::Cuboid(cuboid);
        assert_eq!(SupportMap::support_point(shape, pose, v(ONE, ZERO)), expected);
    }

    #[test]
    #[should_panic(expected: 'Query: not a support map')]
    fn test_half_space_is_not_a_support_map() {
        let _ = SupportMap::local_support_point(
            Shape::HalfSpace(HalfSpaceTrait::new(v(ZERO, ONE))), v(ONE, ZERO),
        );
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_support_point_cuboid() {
        let pose = Pose2 { translation: v(ONE, ONE), rotation: Rot2 { re: ZERO, im: ONE } };
        let _ = SupportMap::support_point(
            opaque(CuboidTrait::new(v(TWO, ONE))), opaque(pose), opaque(v(ONE, ZERO)),
        );
    }
    #[test]
    fn gas_local_support_point_shape_cuboid() {
        let _ = SupportMap::local_support_point(
            opaque(Shape::Cuboid(CuboidTrait::new(v(TWO, ONE)))), opaque(v(ONE, ZERO)),
        );
    }
}
