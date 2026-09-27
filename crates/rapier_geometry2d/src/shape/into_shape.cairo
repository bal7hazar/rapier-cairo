//! Upstream `impl Shape for X`, one per concrete shape: what makes `X` usable as a `super::Shape`
//! (`X.into()`), out of line from `crate::shape` (re-exported there) to keep it under the file
//! budget.

use super::{
    Ball, Capsule, Compound, ConvexPolygon, Cuboid, HalfSpace, HeightField, Polyline,
    RoundConvexPolygon, RoundConvexPolygonShapeTrait, RoundCuboid, RoundTriangle, Segment, Shape,
    Triangle,
};

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

/// Upstream `impl Shape for Polyline` (boxed, as the variant).
pub impl PolylineIntoShape of Into<Polyline, Shape> {
    #[inline(always)]
    fn into(self: Polyline) -> Shape {
        Shape::Polyline(BoxTrait::new(self))
    }
}

/// Upstream `impl Shape for HeightField` (boxed, as the variant).
pub impl HeightFieldIntoShape of Into<HeightField, Shape> {
    #[inline(always)]
    fn into(self: HeightField) -> Shape {
        Shape::HeightField(BoxTrait::new(self))
    }
}

/// Upstream `impl Shape for Compound` (boxed, as the variant).
pub impl CompoundIntoShape of Into<Compound, Shape> {
    #[inline(always)]
    fn into(self: Compound) -> Shape {
        Shape::Compound(BoxTrait::new(self))
    }
}

/// Upstream `impl Shape for RoundShape<ConvexPolygon>` (boxed, as the variant).
pub impl RoundConvexPolygonIntoShape of Into<RoundConvexPolygon, Shape> {
    #[inline(always)]
    fn into(self: RoundConvexPolygon) -> Shape {
        Shape::RoundConvexPolygon(BoxTrait::new(RoundConvexPolygonShapeTrait::new(self)))
    }
}
