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
//! # Internal edges (lot CE)
//!
//! [`FIX_INTERNAL_EDGES`] (Parry 0.31, opt-in: `new` sets no flag) computes, once, the normal cones
//! of every polygonal part from the outline of the union ([`internal_edges`]); a step that selects
//! the constrained composite strategy projects each part's contact normals into them
//! ([`CompoundTrait::part_normal_constraints`]). The flags are not stored apart: a compound is
//! flagged exactly when it holds cones (one entry per part), so a flag-free compound keeps its
//! width but one span, and its serialised felts ([`CompoundSerde`]).
//!
//! Deviations: `bvh` is not ported (no tree, see above); `decompose_trimesh` needs a triangle mesh;
//! `DEFAULT_WELD_TOLERANCE` is an absolute distance in raw Q32.32 units (upstream: ULPs relative
//! to each corner's magnitude, ADR 0001); `part_normal_constraints` answers the cones by value.

use core::nullable::{FromNullableResult, NullableTrait, match_nullable, null};
use core::traits::BitOr;
use fixed::Fixed;
use rapier_math::pose2::Pose2;
use crate::aabb::bounding_volume::{BoundingSphere, BoundingSphereTrait};
use crate::aabb::{Aabb, AabbTrait};
use crate::mass::MassProperties;
pub use crate::shape::compound::pseudo_normals::{
    CompoundEdgeCone, CompoundEdgeConeTrait, CompoundPseudoNormals, turns_clockwise,
};
use crate::shape::round_shape::{RoundConvexPolygonShapeTrait, RoundConvexPolygonTrait};
use crate::shape::{
    BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, RoundCuboidTrait,
    RoundTriangleTrait, SegmentTrait, Shape, ShapeTrait, TriangleTrait,
};

pub mod internal_edges;
pub mod pseudo_normals;

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

/// Controls how a [`Compound`] is loaded (upstream `CompoundFlags`, a bit set).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct CompoundFlags {
    pub bits: u8,
}

/// The edges where two parts meet are interior to the union, and contact normals are clamped to
/// the surviving outline (upstream `CompoundFlags::FIX_INTERNAL_EDGES`): it removes the ledge a
/// body catches on when it slides across the cut between two parts of a decomposition. Costs a
/// one-off pass over the parts' edges when set, and is honoured by a step that selects the
/// constrained composite strategy.
pub const FIX_INTERNAL_EDGES: CompoundFlags = CompoundFlags { bits: 1 };

#[generate_trait]
pub impl CompoundFlagsImpl of CompoundFlagsTrait {
    /// No flag set.
    #[inline(always)]
    fn empty() -> CompoundFlags {
        CompoundFlags { bits: 0 }
    }

    /// The [`FIX_INTERNAL_EDGES`] flag.
    #[inline(always)]
    fn fix_internal_edges() -> CompoundFlags {
        FIX_INTERNAL_EDGES
    }

    /// Whether every flag of `other` is set in `self`.
    #[inline(always)]
    fn contains(self: CompoundFlags, other: CompoundFlags) -> bool {
        (self.bits & other.bits) == other.bits
    }
}

pub impl CompoundFlagsBitOr of BitOr<CompoundFlags> {
    #[inline(always)]
    fn bitor(lhs: CompoundFlags, rhs: CompoundFlags) -> CompoundFlags {
        CompoundFlags { bits: lhs.bits | rhs.bits }
    }
}

/// A union of convex parts (upstream `Compound`).
#[derive(Copy, Drop, Debug)]
pub struct Compound {
    /// The parts and their poses in the compound's frame.
    shapes: Span<(Pose2, Shape)>,
    /// The box of each part in the compound's frame (`shape.compute_aabb(pose)`).
    aabbs: Span<Aabb>,
    /// The union of `aabbs`.
    aabb: Aabb,
    /// One entry per part when [`FIX_INTERNAL_EDGES`] is set (`None` for a part with no straight
    /// sides), null otherwise (upstream `flags` and `pseudo_normals`, see the module
    /// documentation). One felt, so that a flag-free compound costs the paths that copy it
    /// (every part access) one felt at most.
    pseudo_normals: Nullable<Span<Option<CompoundPseudoNormals>>>,
}

/// The cones of `pseudo_normals`, `None` when null.
#[inline(always)]
fn cones_of(
    pseudo_normals: Nullable<Span<Option<CompoundPseudoNormals>>>,
) -> Option<Span<Option<CompoundPseudoNormals>>> {
    match match_nullable(pseudo_normals) {
        FromNullableResult::Null => None,
        FromNullableResult::NotNull(cones) => Some(cones.unbox()),
    }
}

pub impl CompoundPartialEq of PartialEq<Compound> {
    fn eq(lhs: @Compound, rhs: @Compound) -> bool {
        lhs.shapes == rhs.shapes
            && lhs.aabbs == rhs.aabbs
            && lhs.aabb == rhs.aabb
            && cones_of(*lhs.pseudo_normals) == cones_of(*rhs.pseudo_normals)
    }
}

/// `2^32`: [`CompoundSerde`] packs the flags above the part count.
const FLAG_SHIFT: felt252 = 0x1_0000_0000;
const FLAG_SHIFT_NZ: NonZero<u64> = 0x1_0000_0000;

/// The serialised form of a compound: the part count, the parts, the boxes, the union box, as the
/// derived `Serde` of the fields; a flagged compound adds its flags above the part count
/// (`count + flags * 2^32`) and its cones after the union box. A flag-free compound therefore keeps
/// the felts it had before the flags existed, and is read back by the derived path; a flagged
/// header fails that path's `u32` part count, which rewinds and reads the flagged layout.
pub impl CompoundSerde of Serde<Compound> {
    fn serialize(self: @Compound, ref output: Array<felt252>) {
        let Some(cones) = cones_of(*self.pseudo_normals) else {
            Serde::serialize(self.shapes, ref output);
            Serde::serialize(self.aabbs, ref output);
            Serde::serialize(self.aabb, ref output);
            return;
        };
        let shapes = *self.shapes;
        output.append(shapes.len().into() + FIX_INTERNAL_EDGES.bits.into() * FLAG_SHIFT);
        for part in shapes {
            Serde::serialize(part, ref output);
        }
        Serde::serialize(self.aabbs, ref output);
        Serde::serialize(self.aabb, ref output);
        Serde::serialize(@cones, ref output);
    }

    fn deserialize(ref serialized: Span<felt252>) -> Option<Compound> {
        let start = serialized;
        if let Some(shapes) = Serde::deserialize(ref serialized) {
            let aabbs = Serde::deserialize(ref serialized)?;
            let aabb = Serde::deserialize(ref serialized)?;
            return Some(Compound { shapes, aabbs, aabb, pseudo_normals: null() });
        }
        serialized = start;
        deserialize_flagged(ref serialized)
    }
}

/// The flagged arm of [`CompoundSerde::deserialize`], out of line.
#[inline(never)]
fn deserialize_flagged(ref serialized: Span<felt252>) -> Option<Compound> {
    let head: u64 = (*serialized.pop_front()?).try_into()?;
    let (bits, count) = core::num::traits::DivRem::div_rem(head, FLAG_SHIFT_NZ);
    if bits != FIX_INTERNAL_EDGES.bits.into() {
        return None;
    }
    let count: u32 = count.try_into()?;
    let mut shapes = array![];
    let mut i: u32 = 0;
    while i != count {
        shapes.append(Serde::<(Pose2, Shape)>::deserialize(ref serialized)?);
        i += 1;
    }
    let aabbs = Serde::deserialize(ref serialized)?;
    let aabb = Serde::deserialize(ref serialized)?;
    let pseudo_normals: Span<Option<CompoundPseudoNormals>> = Serde::deserialize(ref serialized)?;
    if pseudo_normals.len() != count {
        return None;
    }
    Some(
        Compound {
            shapes: shapes.span(), aabbs, aabb, pseudo_normals: NullableTrait::new(pseudo_normals),
        },
    )
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
        Compound { shapes, aabbs: aabbs.span(), aabb, pseudo_normals: null() }
    }

    /// The tolerance [`CompoundTrait::set_flags`] welds part corners with when given `None`:
    /// 4 raw Q32.32 units (`4 * 2^-32`), an absolute Chebyshev distance (upstream: 4 ULPs relative
    /// to each corner's magnitude; ADR 0001).
    const DEFAULT_WELD_TOLERANCE: Fixed = Fixed { raw: 4 };

    /// [`CompoundTrait::new`] with `flags` applied (upstream `with_flags`); `weld_tolerance` is
    /// [`CompoundTrait::set_flags`]'s.
    /// #### Panics
    /// * As [`CompoundTrait::new`] and [`CompoundTrait::set_flags`].
    fn with_flags(
        shapes: Span<(Pose2, Shape)>, flags: CompoundFlags, weld_tolerance: Option<Fixed>,
    ) -> Compound {
        let mut compound = Self::new(shapes);
        compound.set_flags(flags, weld_tolerance);
        compound
    }

    /// Sets the flags, computing or discarding the cones (upstream `set_flags`).
    ///
    /// `weld_tolerance` says how far apart two part corners may be and still be the same point:
    /// an absolute distance on each axis, `None` selecting
    /// [`CompoundTrait::DEFAULT_WELD_TOLERANCE`]; zero welds only corners at the very same
    /// coordinates. Only [`FIX_INTERNAL_EDGES`] reads it.
    /// #### Panics
    /// * `'i64_sub Overflow'` / `'i64_sub Underflow'` when two corners are `2^31` or more apart.
    fn set_flags(ref self: Compound, flags: CompoundFlags, weld_tolerance: Option<Fixed>) {
        self
            .pseudo_normals =
                if flags.contains(FIX_INTERNAL_EDGES) {
                    NullableTrait::new(
                        internal_edges::compute_pseudo_normals(
                            self.shapes, weld_tolerance.unwrap_or(Self::DEFAULT_WELD_TOLERANCE),
                        ),
                    )
                } else {
                    null()
                };
    }

    /// The flags (upstream `flags`): [`FIX_INTERNAL_EDGES`] when the compound holds its cones.
    #[inline(always)]
    fn flags(self: @Compound) -> CompoundFlags {
        match cones_of(*self.pseudo_normals) {
            None => CompoundFlagsTrait::empty(),
            Some(_) => FIX_INTERNAL_EDGES,
        }
    }

    /// The cones of part `i` (upstream `part_normal_constraints`): `None` unless the compound was
    /// given [`FIX_INTERNAL_EDGES`] and the part is polygonal (a cuboid, a convex polygon or a
    /// triangle).
    /// #### Panics
    /// * [`errors::PART_INDEX`] when flagged and `i >= num_parts`.
    #[inline(always)]
    fn part_normal_constraints(self: @Compound, i: u32) -> Option<CompoundPseudoNormals> {
        let cones = cones_of(*self.pseudo_normals)?;
        assert(i < cones.len(), errors::PART_INDEX);
        *cones.at(i)
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
