//! The closed shape enum of the 2D pipeline (decision D6) and its dispatch.
//!
//! Ball, cuboid, capsule, segment and half-space, each with the upstream Parry API that Rapier's
//! step consumes: `compute_local_aabb`, `compute_aabb`, `mass_properties`, the support maps and the
//! cuboid feature ids. `match` on [`Shape`] replaces Parry's `dyn Shape`.
//!
//! Deferred: convex polygons, round shapes, compounds, `scaled`, ray casting.

pub mod aabb_shim;
pub mod ball;
pub mod capsule;
pub mod cuboid;
pub mod halfspace;
pub mod segment;
use fixed::Fixed;
use rapier_math::pose2::Pose2;
use crate::mass::MassProperties;
use crate::shape::aabb_shim::Aabb;
pub use crate::shape::ball::{Ball, BallTrait};
pub use crate::shape::capsule::{Capsule, CapsuleTrait};
pub use crate::shape::cuboid::{Cuboid, CuboidTrait, SupportFeature};
pub use crate::shape::halfspace::{HalfSpace, HalfSpaceTrait};
pub use crate::shape::segment::{Segment, SegmentTrait};

/// The kind of a [`Shape`] (upstream `ShapeType`, restricted to the supported shapes).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum ShapeType {
    Ball,
    Cuboid,
    Capsule,
    Segment,
    HalfSpace,
}

/// A collision shape in its local frame.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum Shape {
    Ball: Ball,
    Cuboid: Cuboid,
    Capsule: Capsule,
    Segment: Segment,
    HalfSpace: HalfSpace,
}

#[generate_trait]
pub impl ShapeImpl of ShapeTrait {
    /// The kind of the shape.
    #[inline(always)]
    fn shape_type(self: Shape) -> ShapeType {
        match self {
            Shape::Ball(_) => ShapeType::Ball,
            Shape::Cuboid(_) => ShapeType::Cuboid,
            Shape::Capsule(_) => ShapeType::Capsule,
            Shape::Segment(_) => ShapeType::Segment,
            Shape::HalfSpace(_) => ShapeType::HalfSpace,
        }
    }

    /// Bounding box in the local frame.
    fn compute_local_aabb(self: Shape) -> Aabb {
        match self {
            Shape::Ball(s) => s.compute_local_aabb(),
            Shape::Cuboid(s) => s.compute_local_aabb(),
            Shape::Capsule(s) => s.compute_local_aabb(),
            Shape::Segment(s) => s.compute_local_aabb(),
            Shape::HalfSpace(s) => s.compute_local_aabb(),
        }
    }

    /// Bounding box of the shape placed at `pose`. Division-free; exact for quarter-turn poses.
    fn compute_aabb(self: Shape, pose: Pose2) -> Aabb {
        match self {
            Shape::Ball(s) => s.compute_aabb(pose),
            Shape::Cuboid(s) => s.compute_aabb(pose),
            Shape::Capsule(s) => s.compute_aabb(pose),
            Shape::Segment(s) => s.compute_aabb(pose),
            Shape::HalfSpace(s) => s.compute_aabb(pose),
        }
    }

    /// Mass properties for a uniform `density`; segments and half-spaces have none (zero).
    fn mass_properties(self: Shape, density: Fixed) -> MassProperties {
        match self {
            Shape::Ball(s) => s.mass_properties(density),
            Shape::Cuboid(s) => s.mass_properties(density),
            Shape::Capsule(s) => s.mass_properties(density),
            Shape::Segment(s) => s.mass_properties(density),
            Shape::HalfSpace(s) => s.mass_properties(density),
        }
    }

    /// The wrapped `Ball`, `None` for any other shape.
    fn as_ball(self: Shape) -> Option<Ball> {
        match self {
            Shape::Ball(s) => Some(s),
            _ => None,
        }
    }

    /// The wrapped `Cuboid`, `None` for any other shape.
    fn as_cuboid(self: Shape) -> Option<Cuboid> {
        match self {
            Shape::Cuboid(s) => Some(s),
            _ => None,
        }
    }

    /// The wrapped `Capsule`, `None` for any other shape.
    fn as_capsule(self: Shape) -> Option<Capsule> {
        match self {
            Shape::Capsule(s) => Some(s),
            _ => None,
        }
    }

    /// The wrapped `Segment`, `None` for any other shape.
    fn as_segment(self: Shape) -> Option<Segment> {
        match self {
            Shape::Segment(s) => Some(s),
            _ => None,
        }
    }

    /// The wrapped `HalfSpace`, `None` for any other shape.
    fn as_halfspace(self: Shape) -> Option<HalfSpace> {
        match self {
            Shape::HalfSpace(s) => Some(s),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::mass::MassProperties;
    use super::{
        Ball, BallTrait, Capsule, CapsuleTrait, Cuboid, CuboidTrait, HalfSpace, HalfSpaceTrait,
        Segment, SegmentTrait, Shape, ShapeTrait, ShapeType,
    };

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn pose() -> Pose2 {
        Pose2Trait::new(v(ONE, TWO), Rot2 { re: ZERO, im: ONE })
    }

    fn ball() -> Ball {
        BallTrait::new(HALF)
    }

    fn cuboid() -> Cuboid {
        CuboidTrait::new(v(TWO, ONE))
    }

    fn capsule() -> Capsule {
        CapsuleTrait::new(v(ZERO, -ONE), v(ZERO, ONE), HALF)
    }

    fn segment() -> Segment {
        SegmentTrait::new(v(-ONE, ZERO), v(ONE, ONE))
    }

    fn halfspace() -> HalfSpace {
        HalfSpaceTrait::new(v(ZERO, ONE))
    }

    fn count(b: bool) -> u8 {
        if b {
            1
        } else {
            0
        }
    }

    fn all() -> Span<Shape> {
        array![
            Shape::Ball(ball()), Shape::Cuboid(cuboid()), Shape::Capsule(capsule()),
            Shape::Segment(segment()), Shape::HalfSpace(halfspace()),
        ]
            .span()
    }

    #[test]
    fn test_shape_type_and_accessors() {
        let types = array![
            ShapeType::Ball, ShapeType::Cuboid, ShapeType::Capsule, ShapeType::Segment,
            ShapeType::HalfSpace,
        ];
        let mut k = 0;
        for shape in all() {
            assert_eq!((*shape).shape_type(), *types.at(k));
            // Exactly one accessor answers, and it returns the wrapped value.
            let hits = count((*shape).as_ball().is_some())
                + count((*shape).as_cuboid().is_some())
                + count((*shape).as_capsule().is_some())
                + count((*shape).as_segment().is_some())
                + count((*shape).as_halfspace().is_some());
            assert_eq!(hits, 1_u8);
            k += 1;
        }
        assert_eq!(Shape::Ball(ball()).as_ball(), Some(ball()));
        assert_eq!(Shape::Cuboid(cuboid()).as_cuboid(), Some(cuboid()));
        assert_eq!(Shape::Capsule(capsule()).as_capsule(), Some(capsule()));
        assert_eq!(Shape::Segment(segment()).as_segment(), Some(segment()));
        assert_eq!(Shape::HalfSpace(halfspace()).as_halfspace(), Some(halfspace()));
    }

    #[test]
    fn test_dispatch_matches_the_shape_methods() {
        let p = pose();
        assert_eq!(Shape::Ball(ball()).compute_aabb(p), ball().compute_aabb(p));
        assert_eq!(Shape::Cuboid(cuboid()).compute_aabb(p), cuboid().compute_aabb(p));
        assert_eq!(Shape::Capsule(capsule()).compute_aabb(p), capsule().compute_aabb(p));
        assert_eq!(Shape::Segment(segment()).compute_aabb(p), segment().compute_aabb(p));
        assert_eq!(Shape::HalfSpace(halfspace()).compute_aabb(p), halfspace().compute_aabb(p));
        assert_eq!(Shape::Ball(ball()).compute_local_aabb(), ball().compute_local_aabb());
        assert_eq!(Shape::Cuboid(cuboid()).compute_local_aabb(), cuboid().compute_local_aabb());
        assert_eq!(Shape::Capsule(capsule()).compute_local_aabb(), capsule().compute_local_aabb());
        assert_eq!(Shape::Segment(segment()).compute_local_aabb(), segment().compute_local_aabb());
        assert_eq!(
            Shape::HalfSpace(halfspace()).compute_local_aabb(), halfspace().compute_local_aabb(),
        );
        assert_eq!(Shape::Ball(ball()).mass_properties(ONE), ball().mass_properties(ONE));
        assert_eq!(Shape::Cuboid(cuboid()).mass_properties(ONE), cuboid().mass_properties(ONE));
        assert_eq!(Shape::Capsule(capsule()).mass_properties(ONE), capsule().mass_properties(ONE));
    }

    #[test]
    fn test_segments_and_half_spaces_are_massless() {
        let zero: MassProperties = Default::default();
        assert_eq!(Shape::Segment(segment()).mass_properties(TWO), zero);
        assert_eq!(Shape::HalfSpace(halfspace()).mass_properties(TWO), zero);
        // Every local box is well formed.
        for shape in all() {
            let local = (*shape).compute_local_aabb();
            assert!(local.mins.x <= local.maxs.x && local.mins.y <= local.maxs.y);
        }
    }

    #[test]
    fn gas_baseline() {}
    #[test]
    fn gas_shape_type() {
        let _ = opaque(Shape::Cuboid(cuboid())).shape_type();
    }
    #[test]
    fn gas_as_cuboid() {
        let _ = opaque(Shape::Cuboid(cuboid())).as_cuboid();
    }
    #[test]
    fn gas_compute_local_aabb() {
        let _ = opaque(Shape::Cuboid(cuboid())).compute_local_aabb();
    }
    // Sierra gas equalises the branches of a `match`: every variant costs as much as the most
    // expensive one, so the dispatched probes below are equal (the per-shape kernels are ranked
    // in the shape modules).
    #[test]
    fn gas_compute_aabb_ball() {
        let _ = opaque(Shape::Ball(ball())).compute_aabb(opaque(pose()));
    }
    #[test]
    fn gas_compute_aabb_capsule() {
        let _ = opaque(Shape::Capsule(capsule())).compute_aabb(opaque(pose()));
    }
    #[test]
    fn gas_mass_properties_ball() {
        let _ = opaque(Shape::Ball(ball())).mass_properties(opaque(ONE));
    }
    #[test]
    fn gas_mass_properties_capsule() {
        let _ = opaque(Shape::Capsule(capsule())).mass_properties(opaque(ONE));
    }
}
