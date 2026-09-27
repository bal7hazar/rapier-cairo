//! `Compound` (Parry `shape/compound.rs`, 2D; work package SH2b): the union of convex parts, each
//! placed by its own pose in the compound's frame.
//!
//! The parts are `(Pose2, Shape)` pairs of the closed enum; a composite part (polyline,
//! heightfield, compound) is rejected at construction, as upstream ("Nested composite shapes are
//! not allowed"). The box of every part in the compound's frame and their union are computed once
//! and serialised with the parts (derived data, as the polyline's tree).
//!
//! Upstream prunes the parts with a binned-SAH `Bvh`; the port scans the parts' boxes in index
//! order ([`CompoundTrait::parts_in_aabb`]), which the `gas_*` probes of `compound/tests.cairo`
//! measure against the implicit tree of the polyline (`alternatives::parts_in_aabb_tree`) for 2
//! to 16 parts. Parts therefore come out in ascending index, where upstream's order follows its
//! tree.
//!
//! Deviations: `bvh` is not ported (no tree, see above); `DEFAULT_WELD_TOLERANCE`, `flags`,
//! `set_flags`, `with_flags`, `part_normal_constraints`, `CompoundFlags` and
//! `CompoundPseudoNormals` (Parry 0.31's `FIX_INTERNAL_EDGES`) postdate the golden pin
//! (parry2d-f64 0.30.2); `decompose_trimesh` needs a triangle mesh.

use fixed::Fixed;
use rapier_math::pose2::Pose2;
use crate::aabb::bounding_volume::{BoundingSphere, BoundingSphereTrait};
use crate::aabb::{Aabb, AabbTrait};
use crate::mass::MassProperties;
use crate::shape::round_shape::{RoundConvexPolygonShapeTrait, RoundConvexPolygonTrait};
use crate::shape::{
    BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, RoundCuboidTrait,
    RoundTriangleTrait, SegmentTrait, Shape, ShapeTrait, TriangleTrait,
};

/// Failure modes of [`CompoundTrait`].
pub mod errors {
    /// `new` was given no part (upstream: "A compound shape must contain at least one shape.").
    pub const EMPTY: felt252 = 'Compound: no part';
    /// A part is a polyline, a heightfield or a compound (upstream: "Nested composite shapes are
    /// not allowed.").
    pub const NESTED: felt252 = 'Compound: nested composite';
    /// A part index is not below `num_parts`.
    pub const PART_INDEX: felt252 = 'Compound: part index';
}

/// A union of convex parts (upstream `Compound`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Compound {
    /// The parts and their poses in the compound's frame.
    shapes: Span<(Pose2, Shape)>,
    /// The box of each part in the compound's frame (`shape.compute_aabb(pose)`).
    aabbs: Span<Aabb>,
    /// The union of `aabbs`.
    aabb: Aabb,
}

/// `Serde` of the boxed variant payload.
pub impl BoxedCompoundSerde of Serde<Box<Compound>> {
    fn serialize(self: @Box<Compound>, ref output: Array<felt252>) {
        Serde::serialize(@(*self).unbox(), ref output);
    }
    fn deserialize(ref serialized: Span<felt252>) -> Option<Box<Compound>> {
        Some(BoxTrait::new(Serde::deserialize(ref serialized)?))
    }
}

pub impl BoxedCompoundPartialEq of PartialEq<Box<Compound>> {
    fn eq(lhs: @Box<Compound>, rhs: @Box<Compound>) -> bool {
        (*lhs).unbox() == (*rhs).unbox()
    }
    fn ne(lhs: @Box<Compound>, rhs: @Box<Compound>) -> bool {
        !Self::eq(lhs, rhs)
    }
}

#[generate_trait]
pub impl CompoundImpl of CompoundTrait {
    /// A compound of `shapes` (upstream `Compound::new`): each part's box in the compound's frame
    /// and their union are computed once.
    /// #### Panics
    /// * [`errors::EMPTY`] for no part.
    /// * [`errors::NESTED`] when a part is composite (polyline, heightfield or compound).
    fn new(shapes: Span<(Pose2, Shape)>) -> Compound {
        assert(!shapes.is_empty(), errors::EMPTY);
        let mut aabbs = array![];
        let mut aabb = AabbTrait::new_invalid();
        for part in shapes {
            let (pose, shape) = *part;
            assert(!shape.is_composite(), errors::NESTED);
            let bv = shape.compute_aabb(pose);
            aabb = aabb.merged(bv);
            aabbs.append(bv);
        }
        Compound { shapes, aabbs: aabbs.span(), aabb }
    }

    /// The parts and their poses (upstream `shapes`).
    #[inline(always)]
    fn shapes(self: @Compound) -> Span<(Pose2, Shape)> {
        *self.shapes
    }

    /// The number of parts.
    #[inline(always)]
    fn num_parts(self: @Compound) -> u32 {
        self.shapes.len()
    }

    /// Part `i` and its pose.
    /// #### Panics
    /// * [`errors::PART_INDEX`] when `i >= num_parts`.
    fn part(self: @Compound, i: u32) -> (Pose2, Shape) {
        assert(i < self.shapes.len(), errors::PART_INDEX);
        *self.shapes.at(i)
    }

    /// The pose of part `i` in the compound's frame (upstream: the manifolds' `subshape_pos`).
    /// #### Panics
    /// * [`errors::PART_INDEX`] when `i >= num_parts`.
    fn part_pose(self: @Compound, i: u32) -> Pose2 {
        let (pose, _) = self.part(i);
        pose
    }

    /// The box of every part in the compound's frame (upstream `aabbs`).
    #[inline(always)]
    fn aabbs(self: @Compound) -> Span<Aabb> {
        *self.aabbs
    }

    /// The union of the parts' boxes (upstream `local_aabb`).
    #[inline(always)]
    fn local_aabb(self: @Compound) -> Aabb {
        *self.aabb
    }

    /// The local box moved by `pose` (upstream `Shape::compute_aabb`:
    /// `local_aabb().transform_by(pose)`, conservative under rotation).
    #[inline(always)]
    fn aabb(self: @Compound, pose: Pose2) -> Aabb {
        self.local_aabb().transform_by(pose)
    }

    /// Upstream name of [`CompoundTrait::local_aabb`] on `Shape`.
    #[inline(always)]
    fn compute_local_aabb(self: @Compound) -> Aabb {
        self.local_aabb()
    }

    /// Upstream name of [`CompoundTrait::aabb`] on `Shape`.
    #[inline(always)]
    fn compute_aabb(self: @Compound, pose: Pose2) -> Aabb {
        self.aabb(pose)
    }

    /// The bounding sphere of the local box (upstream `local_bounding_sphere`).
    fn local_bounding_sphere(self: @Compound) -> BoundingSphere {
        self.local_aabb().bounding_sphere()
    }

    /// [`CompoundTrait::local_bounding_sphere`] placed at `pose`.
    fn bounding_sphere(self: @Compound, pose: Pose2) -> BoundingSphere {
        self.local_bounding_sphere().transform_by(pose)
    }

    /// The sum of the parts' mass properties, each moved by its pose (upstream
    /// `MassProperties::from_compound`).
    fn mass_properties(self: @Compound, density: Fixed) -> MassProperties {
        crate::mass::MassPropertiesTrait::from_compound(density, *self.shapes)
    }

    /// The indices of the parts whose box intersects `aabb` (the compound's frame), ascending
    /// (upstream: `bvh().intersect_aabb`; a scan of [`CompoundTrait::aabbs`], see the module
    /// documentation).
    fn parts_in_aabb(self: @Compound, aabb: Aabb) -> Array<u32> {
        let mut out = array![];
        let mut i: u32 = 0;
        for bv in *self.aabbs {
            if (*bv).intersects(aabb) {
                out.append(i);
            }
            i += 1;
        }
        out
    }
}

/// The mass properties of a part: `ShapeTrait::mass_properties` without its compound arm (an
/// inlined dispatcher cannot sit on the recursion compound → part → compound). Zero for a
/// composite, which `CompoundTrait::new` rejects.
#[inline(never)]
pub fn part_mass_properties(shape: Shape, density: Fixed) -> MassProperties {
    match shape {
        Shape::Ball(s) => s.mass_properties(density),
        Shape::Cuboid(s) => s.mass_properties(density),
        Shape::Capsule(s) => s.mass_properties(density),
        Shape::Segment(s) => s.mass_properties(density),
        Shape::HalfSpace(s) => s.mass_properties(density),
        Shape::ConvexPolygon(s) => s.unbox().mass_properties(density),
        Shape::Triangle(s) => s.unbox().mass_properties(density),
        Shape::RoundCuboid(s) => s.mass_properties(density),
        Shape::RoundTriangle(s) => s.unbox().mass_properties(density),
        Shape::RoundConvexPolygon(s) => s.unbox().to_round().mass_properties(density),
        _ => Default::default(),
    }
}

/// Rejected candidates, kept for the `gas_*` ranking and as oracles.
#[cfg(test)]
pub mod alternatives {
    use core::num::traits::DivRem;
    use crate::aabb::{Aabb, AabbTrait};
    use crate::shape::polyline::build_tree;
    use super::{Compound, CompoundTrait};

    /// The prefilter over the polyline's implicit tree (built here from the part boxes; a stored
    /// tree would be built once).
    pub fn tree(compound: @Compound) -> (Span<Aabb>, u32) {
        build_tree(compound.aabbs())
    }

    /// The parts meeting `aabb` through the implicit tree `(nodes, leaf_base)` of [`tree`]:
    /// `PolylineTrait::segments_in_aabb`'s stackless walk.
    pub fn parts_in_aabb_tree(nodes: Span<Aabb>, leaf_base: u32, n: u32, aabb: Aabb) -> Array<u32> {
        let mut out = array![];
        let mut k: u32 = 1;
        loop {
            if (*nodes.at(k)).intersects(aabb) {
                if k < leaf_base {
                    k = 2 * k;
                    continue;
                }
                let leaf = k - leaf_base;
                if leaf < n {
                    out.append(leaf);
                }
            }
            let mut done = false;
            loop {
                if k == 1 {
                    done = true;
                    break;
                }
                let (parent, right) = DivRem::div_rem(k, 2);
                if right == 0 {
                    k += 1;
                    break;
                }
                k = parent;
            }
            if done {
                break;
            }
        }
        out
    }
}

#[cfg(test)]
mod tests;
