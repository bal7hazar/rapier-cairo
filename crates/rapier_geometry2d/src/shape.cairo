//! The closed shape enum of the 2D pipeline (decision D6) and its dispatch.
//!
//! Ball, cuboid, capsule, segment, half-space, bounded convex polygon, triangle and the round
//! cuboid, triangle and convex polygon (work package SH1), each with the upstream
//! Parry API that Rapier's step consumes: `compute_local_aabb`, `compute_aabb`, `mass_properties`,
//! the support maps and the cuboid feature ids. `match` on [`Shape`] replaces Parry's `dyn Shape`.
//!
//! The bounding-sphere, swept-box, feature-normal and support-map / feature-map views of the
//! `Shape` trait are ported too; they are not reached by the step.
//!
//! Deferred: compounds and `scaled` on `Shape`.
//!
//! The payloads wider than the capsule (triangle, round triangle, round polygon) are boxed like the
//! polygon, so that a `Shape` stays six felts; the round cuboid (three felts) is stored inline.

pub mod convex_polygon;
use convex_polygon::{BoxedConvexPolygonPartialEq, BoxedConvexPolygonSerde};
pub use convex_polygon::{ConvexPolygon, ConvexPolygonTrait};
pub mod ball;
pub mod capsule;
pub mod cuboid;
pub mod halfspace;
pub mod polygonal_feature_map;
pub mod round_shape;
pub mod segment;
pub mod support_map;
pub mod triangle;
use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::Pose2;
use crate::aabb::bounding_volume::{BoundingSphere, BoundingSphereTrait};
use crate::aabb::{Aabb, AabbTrait};
use crate::feature_id::{FeatureId, SubShapeId};
use crate::mass::MassProperties;
pub use crate::shape::ball::{Ball, BallTrait};
pub use crate::shape::capsule::{Capsule, CapsuleTrait};
pub use crate::shape::cuboid::{Cuboid, CuboidTrait};
pub use crate::shape::halfspace::{HalfSpace, HalfSpaceTrait};
use crate::shape::round_shape::{
    BoxedRoundConvexPolygonShapePartialEq, BoxedRoundConvexPolygonShapeSerde,
    BoxedRoundTrianglePartialEq, BoxedRoundTriangleSerde,
};
pub use crate::shape::round_shape::{
    RoundConvexPolygon, RoundConvexPolygonShape, RoundConvexPolygonShapeTrait,
    RoundConvexPolygonTrait, RoundCuboid, RoundCuboidTrait, RoundShape, RoundShapeTrait,
    RoundTriangle, RoundTriangleTrait,
};
pub use crate::shape::segment::{Segment, SegmentTrait};
use crate::shape::triangle::{BoxedTrianglePartialEq, BoxedTriangleSerde};
pub use crate::shape::triangle::{
    Triangle, TriangleOrientation, TrianglePointLocation, TrianglePointLocationTrait, TriangleTrait,
};

/// The kind of a [`Shape`] (upstream `ShapeType`, restricted to the supported shapes).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum ShapeType {
    Ball,
    Cuboid,
    Capsule,
    Segment,
    HalfSpace,
    ConvexPolygon,
    Triangle,
    RoundCuboid,
    RoundTriangle,
    RoundConvexPolygon,
}

/// A collision shape in its local frame.
///
/// The SH1 variants are declared right after `Ball` so that `Ball` stays the first variant (the
/// cheapest `match` branch) and every old variant keeps its relative position, hence the code
/// layout of the old `match` arms (see `crate::dispatch`). Serialization keeps the tags of the
/// six original variants (`0..=5`) and gives the SH1 ones `6..=9` ([`ShapeSerde`]).
#[derive(Copy, Drop, PartialEq, Debug)]
pub enum Shape {
    Ball: Ball,
    /// Boxed (six felts of vertices), as the polygon.
    Triangle: Box<Triangle>,
    /// Three felts: stored inline.
    RoundCuboid: RoundCuboid,
    /// Boxed, as the triangle.
    RoundTriangle: Box<RoundTriangle>,
    /// Boxed, as the polygon, with its inner polygon's local box (see [`RoundConvexPolygonShape`]).
    RoundConvexPolygon: Box<RoundConvexPolygonShape>,
    Cuboid: Cuboid,
    Capsule: Capsule,
    Segment: Segment,
    HalfSpace: HalfSpace,
    /// Boxed to preserve the six-felt representation of existing shapes; the polygon itself
    /// retains its fixed vertex/normal arrays. Serialization delegates to its value.
    ConvexPolygon: Box<ConvexPolygon>,
}

/// `Serde` with stable tags: the six original variants keep `0..=5` (their derived tags before
/// SH1), the SH1 variants take `6..=9`; the payload follows its tag, as a derived `Serde` does.
pub impl ShapeSerde of Serde<Shape> {
    fn serialize(self: @Shape, ref output: Array<felt252>) {
        match self {
            Shape::Ball(x) => {
                Serde::serialize(@0, ref output);
                Serde::serialize(x, ref output);
            },
            Shape::Triangle(x) => {
                Serde::serialize(@6, ref output);
                Serde::serialize(x, ref output);
            },
            Shape::RoundCuboid(x) => {
                Serde::serialize(@7, ref output);
                Serde::serialize(x, ref output);
            },
            Shape::RoundTriangle(x) => {
                Serde::serialize(@8, ref output);
                Serde::serialize(x, ref output);
            },
            Shape::RoundConvexPolygon(x) => {
                Serde::serialize(@9, ref output);
                Serde::serialize(x, ref output);
            },
            Shape::Cuboid(x) => {
                Serde::serialize(@1, ref output);
                Serde::serialize(x, ref output);
            },
            Shape::Capsule(x) => {
                Serde::serialize(@2, ref output);
                Serde::serialize(x, ref output);
            },
            Shape::Segment(x) => {
                Serde::serialize(@3, ref output);
                Serde::serialize(x, ref output);
            },
            Shape::HalfSpace(x) => {
                Serde::serialize(@4, ref output);
                Serde::serialize(x, ref output);
            },
            Shape::ConvexPolygon(x) => {
                Serde::serialize(@5, ref output);
                Serde::serialize(x, ref output);
            },
        }
    }
    fn deserialize(ref serialized: Span<felt252>) -> Option<Shape> {
        let idx: felt252 = Serde::deserialize(ref serialized)?;
        Some(
            match idx {
                0 => Shape::Ball(Serde::deserialize(ref serialized)?),
                1 => Shape::Cuboid(Serde::deserialize(ref serialized)?),
                2 => Shape::Capsule(Serde::deserialize(ref serialized)?),
                3 => Shape::Segment(Serde::deserialize(ref serialized)?),
                4 => Shape::HalfSpace(Serde::deserialize(ref serialized)?),
                5 => Shape::ConvexPolygon(Serde::deserialize(ref serialized)?),
                _ => { return deserialize_sh1(idx, ref serialized); },
            },
        )
    }
}

/// The SH1 tags (`6..=9`) of [`ShapeSerde`], out of line so that the six original tags keep the
/// derived `match`; `None` for any other tag.
#[inline(never)]
fn deserialize_sh1(idx: felt252, ref serialized: Span<felt252>) -> Option<Shape> {
    if idx == 6 {
        Some(Shape::Triangle(Serde::deserialize(ref serialized)?))
    } else if idx == 7 {
        Some(Shape::RoundCuboid(Serde::deserialize(ref serialized)?))
    } else if idx == 8 {
        Some(Shape::RoundTriangle(Serde::deserialize(ref serialized)?))
    } else if idx == 9 {
        Some(Shape::RoundConvexPolygon(Serde::deserialize(ref serialized)?))
    } else {
        None
    }
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
            Shape::ConvexPolygon(_) => ShapeType::ConvexPolygon,
            Shape::Triangle(_) => ShapeType::Triangle,
            Shape::RoundCuboid(_) => ShapeType::RoundCuboid,
            Shape::RoundTriangle(_) => ShapeType::RoundTriangle,
            Shape::RoundConvexPolygon(_) => ShapeType::RoundConvexPolygon,
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
            Shape::ConvexPolygon(s) => s.unbox().compute_local_aabb(),
            Shape::Triangle(s) => s.unbox().compute_local_aabb(),
            Shape::RoundCuboid(s) => s.compute_local_aabb(),
            Shape::RoundTriangle(s) => s.unbox().compute_local_aabb(),
            Shape::RoundConvexPolygon(s) => s.unbox().to_round().compute_local_aabb(),
        }
    }

    /// Bounding box of the shape placed at `pose`. Division-free; exact for quarter-turn poses.
    ///
    /// Inlined so that the caller pays the arm it reaches: out of line, Sierra gas charges every
    /// shape the most expensive arm (`alternatives::compute_aabb_outlined`; the broad phase of
    /// 32 balls pays 16.5k gas more per collider).
    #[inline(always)]
    fn compute_aabb(self: Shape, pose: Pose2) -> Aabb {
        match self {
            Shape::Ball(s) => s.compute_aabb(pose),
            Shape::Cuboid(s) => s.compute_aabb(pose),
            Shape::Capsule(s) => s.compute_aabb(pose),
            Shape::Segment(s) => s.compute_aabb(pose),
            Shape::HalfSpace(s) => s.compute_aabb(pose),
            Shape::ConvexPolygon(s) => s.unbox().compute_aabb(pose),
            _ => {
                let (mins, maxs) = sh1_aabb(self, pose);
                Aabb { mins, maxs }
            },
        }
    }

    /// Mass properties for a uniform `density`; segments and half-spaces have none (zero).
    ///
    /// Inlined: before SH1 the six-arm method was small enough for the compiler to inline it in
    /// the test builds; the SH1 arm (one out-of-line call) would otherwise make every caller pay a
    /// call (`gas_mass_properties_*`: 216 / 367 Cairo steps either way).
    #[inline(always)]
    fn mass_properties(self: Shape, density: Fixed) -> MassProperties {
        match self {
            Shape::Ball(s) => s.mass_properties(density),
            Shape::Cuboid(s) => s.mass_properties(density),
            Shape::Capsule(s) => s.mass_properties(density),
            Shape::Segment(s) => s.mass_properties(density),
            Shape::HalfSpace(s) => s.mass_properties(density),
            Shape::ConvexPolygon(s) => s.unbox().mass_properties(density),
            _ => {
                let (props, _) = sh1_mass_properties(self, density);
                props
            },
        }
    }

    /// Bounding sphere in the local frame (upstream `compute_local_bounding_sphere`, each
    /// shape's `local_bounding_sphere`).
    fn compute_local_bounding_sphere(self: Shape) -> BoundingSphere {
        match self {
            Shape::Ball(s) => s.local_bounding_sphere(),
            Shape::Cuboid(s) => s.local_bounding_sphere(),
            Shape::Capsule(s) => s.local_bounding_sphere(),
            Shape::Segment(s) => s.local_bounding_sphere(),
            Shape::HalfSpace(s) => s.local_bounding_sphere(),
            Shape::ConvexPolygon(s) => s.unbox().local_bounding_sphere(),
            Shape::Triangle(s) => s.unbox().local_bounding_sphere(),
            Shape::RoundCuboid(s) => s.local_bounding_sphere(),
            Shape::RoundTriangle(s) => s.unbox().local_bounding_sphere(),
            Shape::RoundConvexPolygon(s) => s.unbox().to_round().local_bounding_sphere(),
        }
    }

    /// Bounding sphere of the shape placed at `pose` (upstream default:
    /// `compute_local_bounding_sphere().transform_by(pose)`).
    fn compute_bounding_sphere(self: Shape, pose: Pose2) -> BoundingSphere {
        Self::compute_local_bounding_sphere(self).transform_by(pose)
    }

    /// Box swept by the shape from `start_pose` to `end_pose`: the union of both boxes (upstream
    /// default, not the continuous sweep).
    fn compute_swept_aabb(self: Shape, start_pose: Pose2, end_pose: Pose2) -> Aabb {
        Self::compute_aabb(self, start_pose).merged(Self::compute_aabb(self, end_pose))
    }

    /// Normal of the shape at `point` on `feature` (upstream `feature_normal_at_point`): a ball
    /// answers the normalised `point` (`None` at the centre), a cuboid, segment or polygon its
    /// `feature_normal`, a capsule, a half-space, a triangle and the round shapes `None` (upstream
    /// 2D answers). `subshape` is always `0` for the closed set and is ignored.
    fn feature_normal_at_point(
        self: Shape, subshape: SubShapeId, feature: FeatureId, point: Vec2,
    ) -> Option<Vec2> {
        match self {
            Shape::Ball(_) => point.try_normalize(),
            Shape::Cuboid(s) => s.feature_normal(feature),
            Shape::Capsule(_) => None,
            Shape::Segment(s) => s.feature_normal(feature),
            Shape::HalfSpace(_) => None,
            Shape::ConvexPolygon(s) => s.unbox().feature_normal(feature),
            _ => None,
        }
    }

    /// The shape as a support map (`SupportMap<Shape>`), `None` for a half-space (upstream
    /// `as_support_map`, returning the shape itself instead of a trait object).
    fn as_support_map(self: Shape) -> Option<Shape> {
        match self {
            Shape::HalfSpace(_) => None,
            _ => Some(self),
        }
    }

    /// The polygonal feature map of the shape and its rounding radius (upstream
    /// `as_polygonal_feature_map`): cuboids, segments, polygons and triangles are their own with
    /// radius 0, a capsule is its core segment with its radius, a round shape its inner shape with
    /// its border radius; `None` for a ball or a half-space. Use the returned shape through
    /// `PolygonalFeatureMap<Shape>`.
    fn as_polygonal_feature_map(self: Shape) -> Option<(Shape, Fixed)> {
        match self {
            Shape::Cuboid(_) => Some((self, ZERO)),
            Shape::Segment(_) => Some((self, ZERO)),
            Shape::ConvexPolygon(_) => Some((self, ZERO)),
            Shape::Capsule(s) => Some((Shape::Segment(s.segment), s.radius)),
            Shape::Triangle(_) => Some((self, ZERO)),
            Shape::RoundCuboid(s) => Some((Shape::Cuboid(s.inner_shape), s.border_radius)),
            Shape::RoundTriangle(s) => {
                let s = s.unbox();
                Some((Shape::Triangle(BoxTrait::new(s.inner_shape)), s.border_radius))
            },
            Shape::RoundConvexPolygon(s) => {
                let s = s.unbox();
                Some((Shape::ConvexPolygon(BoxTrait::new(s.inner_shape)), s.border_radius))
            },
            _ => None,
        }
    }

    /// Is the shape known to be convex (upstream `is_convex`)? Every shape of the closed set
    /// is: a half-space is convex too.
    #[inline(always)]
    fn is_convex(self: Shape) -> bool {
        match self {
            Shape::Ball(_) => true,
            Shape::Cuboid(_) => true,
            Shape::Capsule(_) => true,
            Shape::Segment(_) => true,
            Shape::HalfSpace(_) => true,
            Shape::ConvexPolygon(_) => true,
            Shape::Triangle(_) => true,
            Shape::RoundCuboid(_) => true,
            Shape::RoundTriangle(_) => true,
            Shape::RoundConvexPolygon(_) => true,
        }
    }

    /// The wrapped polygon, `None` for every other shape.
    fn as_convex_polygon(self: Shape) -> Option<ConvexPolygon> {
        match self {
            Shape::ConvexPolygon(s) => Some(s.unbox()),
            _ => None,
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

    /// The wrapped `Triangle`, `None` for any other shape (upstream `as_triangle`).
    fn as_triangle(self: Shape) -> Option<Triangle> {
        match self {
            Shape::Triangle(s) => Some(s.unbox()),
            _ => None,
        }
    }

    /// The wrapped `RoundCuboid`, `None` for any other shape (upstream `as_round_cuboid`).
    fn as_round_cuboid(self: Shape) -> Option<RoundCuboid> {
        match self {
            Shape::RoundCuboid(s) => Some(s),
            _ => None,
        }
    }

    /// The wrapped `RoundTriangle`, `None` for any other shape (upstream `as_round_triangle`).
    fn as_round_triangle(self: Shape) -> Option<RoundTriangle> {
        match self {
            Shape::RoundTriangle(s) => Some(s.unbox()),
            _ => None,
        }
    }

    /// The wrapped `RoundConvexPolygon`, `None` for any other shape (upstream
    /// `as_round_convex_polygon`).
    fn as_round_convex_polygon(self: Shape) -> Option<RoundConvexPolygon> {
        match self {
            Shape::RoundConvexPolygon(s) => Some(s.unbox().to_round()),
            _ => None,
        }
    }
}

/// The SH1 arms of the inlined `ShapeTrait::compute_aabb`, out of line and loop-free. Returns the
/// corners, not an `Aabb`: the polygon arm's call returns one, and the compiler merges identical
/// post-call blocks, which would move the polygon arm's code in the broad-phase loop.
#[inline(never)]
fn sh1_aabb(shape: Shape, pose: Pose2) -> (Vec2, Vec2) {
    let aabb = match shape {
        Shape::Triangle(s) => s.unbox().compute_aabb(pose),
        Shape::RoundCuboid(s) => AabbTrait::loosened(
            s.inner_shape.compute_aabb(pose), s.border_radius,
        ),
        Shape::RoundTriangle(s) => {
            let s = s.unbox();
            AabbTrait::loosened(s.inner_shape.compute_aabb(pose), s.border_radius)
        },
        Shape::RoundConvexPolygon(s) => s.unbox().compute_aabb_cached(pose),
        _ => Aabb { mins: pose.translation, maxs: pose.translation },
    };
    (aabb.mins, aabb.maxs)
}

/// The SH1 arm of `ShapeTrait::mass_properties`. Returns a pair, not a bare `MassProperties`: the
/// polygon arm's call returns one, and the compiler merges identical post-call blocks, which
/// would move the polygon arm's code.
#[inline(never)]
fn sh1_mass_properties(shape: Shape, density: Fixed) -> (MassProperties, bool) {
    let props = match shape {
        Shape::Triangle(s) => s.unbox().mass_properties(density),
        Shape::RoundCuboid(s) => s.mass_properties(density),
        Shape::RoundTriangle(s) => s.unbox().mass_properties(density),
        Shape::RoundConvexPolygon(s) => s.unbox().to_round().mass_properties(density),
        _ => Default::default(),
    };
    (props, true)
}

/// Upstream `impl Shape for Ball`: a ball is a shape.
pub impl BallIntoShape of Into<Ball, Shape> {
    #[inline(always)]
    fn into(self: Ball) -> Shape {
        Shape::Ball(self)
    }
}

/// Upstream `impl Shape for Cuboid`.
pub impl CuboidIntoShape of Into<Cuboid, Shape> {
    #[inline(always)]
    fn into(self: Cuboid) -> Shape {
        Shape::Cuboid(self)
    }
}

/// Upstream `impl Shape for Capsule`.
pub impl CapsuleIntoShape of Into<Capsule, Shape> {
    #[inline(always)]
    fn into(self: Capsule) -> Shape {
        Shape::Capsule(self)
    }
}

/// Upstream `impl Shape for Segment`.
pub impl SegmentIntoShape of Into<Segment, Shape> {
    #[inline(always)]
    fn into(self: Segment) -> Shape {
        Shape::Segment(self)
    }
}

/// Upstream `impl Shape for HalfSpace`.
pub impl HalfSpaceIntoShape of Into<HalfSpace, Shape> {
    #[inline(always)]
    fn into(self: HalfSpace) -> Shape {
        Shape::HalfSpace(self)
    }
}

/// Upstream `impl Shape for ConvexPolygon` (boxed, as the variant).
pub impl ConvexPolygonIntoShape of Into<ConvexPolygon, Shape> {
    #[inline(always)]
    fn into(self: ConvexPolygon) -> Shape {
        Shape::ConvexPolygon(BoxTrait::new(self))
    }
}

/// Upstream `impl Shape for Triangle` (boxed, as the variant).
pub impl TriangleIntoShape of Into<Triangle, Shape> {
    #[inline(always)]
    fn into(self: Triangle) -> Shape {
        Shape::Triangle(BoxTrait::new(self))
    }
}

/// Upstream `impl Shape for RoundShape<Cuboid>`.
pub impl RoundCuboidIntoShape of Into<RoundCuboid, Shape> {
    #[inline(always)]
    fn into(self: RoundCuboid) -> Shape {
        Shape::RoundCuboid(self)
    }
}

/// Upstream `impl Shape for RoundShape<Triangle>` (boxed, as the variant).
pub impl RoundTriangleIntoShape of Into<RoundTriangle, Shape> {
    #[inline(always)]
    fn into(self: RoundTriangle) -> Shape {
        Shape::RoundTriangle(BoxTrait::new(self))
    }
}

/// Upstream `impl Shape for RoundShape<ConvexPolygon>` (boxed, as the variant).
pub impl RoundConvexPolygonIntoShape of Into<RoundConvexPolygon, Shape> {
    #[inline(always)]
    fn into(self: RoundConvexPolygon) -> Shape {
        Shape::RoundConvexPolygon(BoxTrait::new(RoundConvexPolygonShapeTrait::new(self)))
    }
}

#[cfg(test)]
mod helpers_tests;

#[cfg(test)]
mod alternatives {
    use rapier_math::pose2::Pose2;
    use crate::aabb::Aabb;
    use super::{
        BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait,
        Shape,
    };

    /// Rejected direct polygon payload; increases every Shape value to 34 felts.
    #[derive(Copy, Drop)]
    pub enum UnboxedShape {
        Ball: super::Ball,
        Cuboid: super::Cuboid,
        Capsule: super::Capsule,
        Segment: super::Segment,
        HalfSpace: super::HalfSpace,
        ConvexPolygon: super::ConvexPolygon,
    }

    /// Rejected 34-felt enum representation: existing scene probes measured the copy overhead
    /// before boxing the polygon payload. This retained AABB path exercises that representation.
    #[inline(never)]
    pub fn compute_aabb_outlined(shape: Shape, pose: Pose2) -> Aabb {
        let shape = match shape {
            Shape::Ball(s) => UnboxedShape::Ball(s),
            Shape::Cuboid(s) => UnboxedShape::Cuboid(s),
            Shape::Capsule(s) => UnboxedShape::Capsule(s),
            Shape::Segment(s) => UnboxedShape::Segment(s),
            Shape::HalfSpace(s) => UnboxedShape::HalfSpace(s),
            Shape::ConvexPolygon(s) => UnboxedShape::ConvexPolygon(s.unbox()),
            // The SH1 shapes postdate this candidate.
            _ => core::panic_with_felt252('Shape: not in candidate'),
        };
        compute_unboxed_aabb(shape, pose)
    }

    #[inline(never)]
    fn compute_unboxed_aabb(shape: UnboxedShape, pose: Pose2) -> Aabb {
        match shape {
            UnboxedShape::Ball(s) => s.compute_aabb(pose),
            UnboxedShape::Capsule(s) => s.compute_aabb(pose),
            UnboxedShape::Cuboid(s) => s.compute_aabb(pose),
            UnboxedShape::HalfSpace(s) => s.compute_aabb(pose),
            UnboxedShape::Segment(s) => s.compute_aabb(pose),
            UnboxedShape::ConvexPolygon(s) => s.compute_aabb(pose),
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
        Ball, BallTrait, Capsule, CapsuleTrait, ConvexPolygonTrait, Cuboid, CuboidTrait, HalfSpace,
        HalfSpaceTrait, Segment, SegmentTrait, Shape, ShapeTrait, ShapeType,
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
    fn test_every_shape_is_convex_and_converts_into_shape() {
        let polygon = ConvexPolygonTrait::from_convex_polyline(
            [v(ZERO, ZERO), v(ONE, ZERO), v(ZERO, ONE)].span(),
        )
            .unwrap();
        let shapes: Array<Shape> = array![
            ball().into(), cuboid().into(), capsule().into(), segment().into(), halfspace().into(),
            polygon.into(),
        ];
        let expected = array![
            Shape::Ball(ball()), Shape::Cuboid(cuboid()), Shape::Capsule(capsule()),
            Shape::Segment(segment()), Shape::HalfSpace(halfspace()),
            Shape::ConvexPolygon(BoxTrait::new(polygon)),
        ];
        let mut k = 0;
        for shape in shapes {
            assert_eq!(shape, *expected.at(k));
            assert!(shape.is_convex());
            k += 1;
        }
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
    fn gas_is_convex() {
        let _ = opaque(Shape::Cuboid(cuboid())).is_convex();
    }
    #[test]
    fn gas_as_cuboid() {
        let _ = opaque(Shape::Cuboid(cuboid())).as_cuboid();
    }
    #[test]
    fn gas_compute_local_aabb() {
        let _ = opaque(Shape::Cuboid(cuboid())).compute_local_aabb();
    }
    // Out of line, Sierra gas equalises the branches of a `match`: every variant costs as much
    // as the most expensive one (the `_outlined` probes are equal); inlined, each pays its arm.
    #[test]
    fn gas_compute_aabb_ball() {
        let _ = opaque(Shape::Ball(ball())).compute_aabb(opaque(pose()));
    }
    #[test]
    fn gas_compute_aabb_capsule() {
        let _ = opaque(Shape::Capsule(capsule())).compute_aabb(opaque(pose()));
    }
    #[test]
    fn gas_compute_aabb_ball_outlined() {
        let _ = super::alternatives::compute_aabb_outlined(
            opaque(Shape::Ball(ball())), opaque(pose()),
        );
    }
    #[test]
    fn gas_compute_aabb_capsule_outlined() {
        let _ = super::alternatives::compute_aabb_outlined(
            opaque(Shape::Capsule(capsule())), opaque(pose()),
        );
    }
    #[test]
    fn test_compute_aabb_outlined_agrees() {
        for shape in all() {
            assert_eq!(
                super::alternatives::compute_aabb_outlined(*shape, pose()),
                (*shape).compute_aabb(pose()),
            );
        }
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
