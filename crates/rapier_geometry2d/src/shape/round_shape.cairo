//! `RoundShape` (Parry `shape/round_shape.rs` and the `impl_shape_for_round_shape!` part of
//! `shape/shape.rs`): a shape dilated by a border radius, i.e. the Minkowski sum of the inner
//! shape and a disc of radius `border_radius`.
//!
//! Upstream's three 2D instances are aliases here: [`RoundCuboid`], [`RoundTriangle`] and
//! [`RoundConvexPolygon`]. As upstream, the bounding volumes are the inner ones loosened by the
//! radius and the mass properties are the inner shape's (the border adds no mass). The queries
//! (point projection, ray cast, contact manifolds, shape-pair queries) are exact analytic
//! kernels on the inner shape and the radius where upstream runs GJK on the support map (ADR 0001
//! entries 14 and 17); see `crate::point::round_shape`, `crate::ray::round_shape` and
//! `crate::contact_generators::pfm_pfm`.

use fixed::Fixed;
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::Pose2;
use crate::aabb::bounding_volume::{BoundingSphere, BoundingVolume};
use crate::aabb::{Aabb, AabbTrait};
use crate::mass::MassProperties;
use crate::shape::convex_polygon::{ConvexPolygon, ConvexPolygonTrait};
use crate::shape::cuboid::{Cuboid, CuboidTrait};
use crate::shape::support_map::SupportMap;
use crate::shape::triangle::{Triangle, TriangleTrait};

/// A shape with rounded borders (upstream `RoundShape<S>`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RoundShape<S> {
    /// The shape being rounded.
    pub inner_shape: S,
    /// The radius of the rounded border (expected `>= 0`, not checked, as upstream).
    pub border_radius: Fixed,
}

/// A cuboid with rounded corners (upstream `RoundCuboid`).
pub type RoundCuboid = RoundShape<Cuboid>;
/// A triangle with rounded corners (upstream `RoundTriangle`).
pub type RoundTriangle = RoundShape<Triangle>;
/// A convex polygon with rounded corners (upstream `RoundConvexPolygon`).
pub type RoundConvexPolygon = RoundShape<ConvexPolygon>;

#[generate_trait]
pub impl RoundShapeImpl<S, +Drop<S>> of RoundShapeTrait<S> {
    /// `inner_shape` dilated by `border_radius` (upstream builds the struct literally).
    #[inline(always)]
    fn new(inner_shape: S, border_radius: Fixed) -> RoundShape<S> {
        RoundShape { inner_shape, border_radius }
    }
}

/// The support point of the dilated shape: the inner support point toward the unit `dir`, moved
/// by `dir * border_radius` (upstream `impl SupportMap for RoundShape<S>`); `local_support_point`
/// normalises `dir` first (`+Y` for a zero direction, as the capsule).
pub impl RoundShapeSupportMap<S, +SupportMap<S>, +Drop<S>, +Copy<S>> of SupportMap<RoundShape<S>> {
    fn local_support_point(self: RoundShape<S>, dir: Vec2) -> Vec2 {
        Self::local_support_point_toward(self, dir.normalize_or(Vec2Trait::Y))
    }
    fn local_support_point_toward(self: RoundShape<S>, dir: Vec2) -> Vec2 {
        self.inner_shape.local_support_point_toward(dir) + dir.mul_scalar(self.border_radius)
    }
}

/// Box serialization is the round triangle value.
pub impl BoxedRoundTriangleSerde of Serde<Box<RoundTriangle>> {
    fn serialize(self: @Box<RoundTriangle>, ref output: Array<felt252>) {
        (*self).unbox().serialize(ref output);
    }
    fn deserialize(ref serialized: Span<felt252>) -> Option<Box<RoundTriangle>> {
        Some(BoxTrait::new(Serde::<RoundTriangle>::deserialize(ref serialized)?))
    }
}

/// Structural equality of boxed round triangles.
pub impl BoxedRoundTrianglePartialEq of PartialEq<Box<RoundTriangle>> {
    fn eq(lhs: @Box<RoundTriangle>, rhs: @Box<RoundTriangle>) -> bool {
        (*lhs).unbox() == (*rhs).unbox()
    }
    fn ne(lhs: @Box<RoundTriangle>, rhs: @Box<RoundTriangle>) -> bool {
        !Self::eq(lhs, rhs)
    }
}

/// The payload of `Shape::RoundConvexPolygon`: the round polygon's fields and the local box of
/// its inner polygon, cached at construction ([`RoundConvexPolygonShapeTrait::new`]).
///
/// The shape dispatch of `ShapeTrait::compute_aabb` must stay loop-free and no dearer than its
/// polygon arm (otherwise every broad-phase loop pays a gas redeposit on the polygon path), so the
/// round polygon's box is the cached local box moved by the pose and loosened by the radius:
/// conservative under rotation (it contains the tight box of
/// [`RoundConvexPolygonTrait::compute_aabb`]), exact for axis-aligned and quarter-turn poses.
#[derive(Copy, Drop, Serde, Debug)]
pub struct RoundConvexPolygonShape {
    pub inner_shape: ConvexPolygon,
    pub border_radius: Fixed,
    /// `inner_shape.compute_local_aabb()`.
    pub local_aabb: Aabb,
}

#[generate_trait]
pub impl RoundConvexPolygonShapeImpl of RoundConvexPolygonShapeTrait {
    /// The payload of `round`, its local box computed once.
    fn new(round: RoundConvexPolygon) -> RoundConvexPolygonShape {
        RoundConvexPolygonShape {
            inner_shape: round.inner_shape,
            border_radius: round.border_radius,
            local_aabb: round.inner_shape.compute_local_aabb(),
        }
    }
    /// The round polygon.
    #[inline(always)]
    fn to_round(self: RoundConvexPolygonShape) -> RoundConvexPolygon {
        RoundShape { inner_shape: self.inner_shape, border_radius: self.border_radius }
    }
    /// The conservative box of the dispatch (see the type).
    #[inline(always)]
    fn compute_aabb_cached(self: RoundConvexPolygonShape, pose: Pose2) -> Aabb {
        AabbTrait::loosened(AabbTrait::transform_by(self.local_aabb, pose), self.border_radius)
    }
}

/// Box serialization is the payload value, cache included: deserializing is plain field reads (no
/// loop, which would pull gas into the inlined `Shape` deserializer).
pub impl BoxedRoundConvexPolygonShapeSerde of Serde<Box<RoundConvexPolygonShape>> {
    fn serialize(self: @Box<RoundConvexPolygonShape>, ref output: Array<felt252>) {
        (*self).unbox().serialize(ref output);
    }
    fn deserialize(ref serialized: Span<felt252>) -> Option<Box<RoundConvexPolygonShape>> {
        Some(BoxTrait::new(Serde::<RoundConvexPolygonShape>::deserialize(ref serialized)?))
    }
}

/// Structural equality of the round polygons (the cache follows from them).
pub impl BoxedRoundConvexPolygonShapePartialEq of PartialEq<Box<RoundConvexPolygonShape>> {
    fn eq(lhs: @Box<RoundConvexPolygonShape>, rhs: @Box<RoundConvexPolygonShape>) -> bool {
        (*lhs).unbox().to_round() == (*rhs).unbox().to_round()
    }
    fn ne(lhs: @Box<RoundConvexPolygonShape>, rhs: @Box<RoundConvexPolygonShape>) -> bool {
        !Self::eq(lhs, rhs)
    }
}

#[inline(always)]
fn loosened_sphere(s: BoundingSphere, r: Fixed) -> BoundingSphere {
    BoundingVolume::<BoundingSphere>::loosened(s, r)
}

/// The shape views of upstream's `impl Shape for RoundShape<Cuboid>`.
#[generate_trait]
pub impl RoundCuboidImpl of RoundCuboidTrait {
    /// The inner box loosened by the radius (upstream `compute_local_aabb`).
    fn compute_local_aabb(self: RoundCuboid) -> Aabb {
        AabbTrait::loosened(self.inner_shape.compute_local_aabb(), self.border_radius)
    }
    /// The inner placed box loosened by the radius (upstream `compute_aabb`).
    fn compute_aabb(self: RoundCuboid, pose: Pose2) -> Aabb {
        AabbTrait::loosened(self.inner_shape.compute_aabb(pose), self.border_radius)
    }
    /// The inner sphere loosened by the radius (upstream `compute_local_bounding_sphere`).
    /// #### Panics
    /// * `'Bounding: negative margin'` for a negative radius.
    #[inline(never)]
    fn local_bounding_sphere(self: RoundCuboid) -> BoundingSphere {
        loosened_sphere(self.inner_shape.local_bounding_sphere(), self.border_radius)
    }
    /// The inner shape's mass properties: the border adds no mass (upstream).
    fn mass_properties(self: RoundCuboid, density: Fixed) -> MassProperties {
        self.inner_shape.mass_properties(density)
    }
}

/// The shape views of upstream's `impl Shape for RoundShape<Triangle>`.
#[generate_trait]
pub impl RoundTriangleImpl of RoundTriangleTrait {
    /// The inner box loosened by the radius.
    fn compute_local_aabb(self: RoundTriangle) -> Aabb {
        AabbTrait::loosened(self.inner_shape.compute_local_aabb(), self.border_radius)
    }
    /// The inner placed box loosened by the radius.
    fn compute_aabb(self: RoundTriangle, pose: Pose2) -> Aabb {
        AabbTrait::loosened(self.inner_shape.compute_aabb(pose), self.border_radius)
    }
    /// The inner point-cloud sphere loosened by the radius.
    /// #### Panics
    /// * `'Bounding: negative margin'` for a negative radius.
    #[inline(never)]
    fn local_bounding_sphere(self: RoundTriangle) -> BoundingSphere {
        loosened_sphere(self.inner_shape.local_bounding_sphere(), self.border_radius)
    }
    /// The inner triangle's mass properties.
    fn mass_properties(self: RoundTriangle, density: Fixed) -> MassProperties {
        self.inner_shape.mass_properties(density)
    }
}

/// The shape views of upstream's `impl Shape for RoundShape<ConvexPolygon>`.
#[generate_trait]
pub impl RoundConvexPolygonImpl of RoundConvexPolygonTrait {
    /// The inner box loosened by the radius.
    fn compute_local_aabb(self: RoundConvexPolygon) -> Aabb {
        AabbTrait::loosened(self.inner_shape.compute_local_aabb(), self.border_radius)
    }
    /// The inner placed box loosened by the radius (upstream `compute_aabb`, tight).
    fn compute_aabb(self: RoundConvexPolygon, pose: Pose2) -> Aabb {
        AabbTrait::loosened(self.inner_shape.compute_aabb(pose), self.border_radius)
    }
    /// The inner point-cloud sphere loosened by the radius.
    /// #### Panics
    /// * `'Bounding: negative margin'` for a negative radius.
    #[inline(never)]
    fn local_bounding_sphere(self: RoundConvexPolygon) -> BoundingSphere {
        loosened_sphere(self.inner_shape.local_bounding_sphere(), self.border_radius)
    }
    /// The inner polygon's mass properties.
    fn mass_properties(self: RoundConvexPolygon, density: Fixed) -> MassProperties {
        self.inner_shape.mass_properties(density)
    }
}
