//! `Polyline` (Parry `shape/polyline.rs`, 2D; work package SH2a): a composite shape made of
//! segments sharing a vertex buffer, with the optional `ORIENTED` flag (outward pseudo-normals).
//!
//! Upstream prunes the segments with a binned-SAH `Bvh`. The port keeps a deterministic implicit
//! tree instead: a complete binary tree over the segments in index order, stored as the boxes of
//! its nodes in heap layout (node `k` has children `2k` and `2k + 1`, the leaves are the
//! `leaf_base` last nodes, `leaf_base` the smallest power of two `>= num_segments`, padding leaves
//! hold an inverted box that intersects nothing). A query walks it left child first without a
//! stack ([`PolylineTrait::segments_in_aabb`]), so the segments come out in ascending index,
//! where upstream's order follows its tree. The linear scan it replaces is
//! `alternatives::segments_in_aabb_linear` (see the `gas_*` probes of `polyline/tests.cairo`).
//!
//! The tree and the pseudo-normals are derived data, serialised with the rest (`Serde` is
//! derived): rebuilding them on deserialisation would put the constructor's dictionary (and its
//! `SegmentArena` builtin) on every `Shape` deserialisation, which the world state runs.
//!
//! Deviations: `update_vertices` (a closure argument) is not ported,
//! [`PolylineTrait::set_vertices`]
//! replaces it; `bvh` answers the node boxes of the implicit tree.

use core::dict::{Felt252Dict, Felt252DictTrait};
use core::nullable::NullableTrait;
use core::num::traits::DivRem;
use core::traits::BitOr;
use fixed::Fixed;
use glam_core::{Vec2, Vec2Trait};
use rapier_math::pose2::Pose2;
use crate::aabb::bounding_volume::{BoundingSphere, BoundingSphereTrait};
use crate::aabb::{Aabb, AabbTrait};
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::mass::MassProperties;
use crate::shape::segment::{Segment, SegmentPseudoNormals, SegmentTrait};

/// Failure modes of [`PolylineTrait`].
pub mod errors {
    /// A segment index refers to a vertex that does not exist.
    pub const VERTEX_INDEX: felt252 = 'Polyline: vertex index';
    /// A segment index is not below `num_segments`.
    pub const SEGMENT_INDEX: felt252 = 'Polyline: segment index';
    /// `set_vertices` was given another number of vertices.
    pub const VERTEX_COUNT: felt252 = 'Polyline: vertex count';
}

/// Flags of a [`Polyline`] (upstream `PolylineFlags`, a bit set).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct PolylineFlags {
    pub bits: u8,
}

/// The polyline is closed and counter-clockwise: its interior is solid and its segments
/// collide from their outward side (upstream `PolylineFlags::ORIENTED`).
pub const ORIENTED: PolylineFlags = PolylineFlags { bits: 1 };

#[generate_trait]
pub impl PolylineFlagsImpl of PolylineFlagsTrait {
    /// No flag set.
    #[inline(always)]
    fn empty() -> PolylineFlags {
        PolylineFlags { bits: 0 }
    }

    /// The `ORIENTED` flag.
    #[inline(always)]
    fn oriented() -> PolylineFlags {
        ORIENTED
    }

    /// `true` when every flag of `other` is set in `self`. Bit by bit with `DivRem`: a `&` would
    /// add the bitwise builtin to every query that tests the flags.
    fn contains(self: PolylineFlags, other: PolylineFlags) -> bool {
        let (mut a, mut b) = (self.bits, other.bits);
        while b != 0 {
            let (qa, ra) = DivRem::div_rem(a, TWO_U8_NZ);
            let (qb, rb) = DivRem::div_rem(b, TWO_U8_NZ);
            if rb == 1 && ra == 0 {
                return false;
            }
            a = qa;
            b = qb;
        }
        true
    }
}

pub impl PolylineFlagsBitOr of BitOr<PolylineFlags> {
    #[inline(always)]
    fn bitor(lhs: PolylineFlags, rhs: PolylineFlags) -> PolylineFlags {
        PolylineFlags { bits: lhs.bits | rhs.bits }
    }
}

/// A set of segments sharing a vertex buffer (upstream `Polyline`, 2D).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Polyline {
    vertices: Span<Vec2>,
    indices: Span<[u32; 2]>,
    flags: PolylineFlags,
    /// One outward pseudo-normal per vertex when `ORIENTED`, empty otherwise.
    pseudo_normals: Span<Vec2>,
    /// The implicit tree (see the module documentation); node 0 is unused.
    nodes: Span<Aabb>,
    /// Index of the first leaf node.
    leaf_base: u32,
}

/// `Serde` of the boxed variant payload.
pub impl BoxedPolylineSerde of Serde<Box<Polyline>> {
    fn serialize(self: @Box<Polyline>, ref output: Array<felt252>) {
        Serde::serialize(@(*self).unbox(), ref output);
    }
    fn deserialize(ref serialized: Span<felt252>) -> Option<Box<Polyline>> {
        Some(BoxTrait::new(Serde::deserialize(ref serialized)?))
    }
}

pub impl BoxedPolylinePartialEq of PartialEq<Box<Polyline>> {
    fn eq(lhs: @Box<Polyline>, rhs: @Box<Polyline>) -> bool {
        (*lhs).unbox() == (*rhs).unbox()
    }
    fn ne(lhs: @Box<Polyline>, rhs: @Box<Polyline>) -> bool {
        !Self::eq(lhs, rhs)
    }
}

const TWO_NZ: NonZero<u32> = 2;
const TWO_U8_NZ: NonZero<u8> = 2;

/// The box of a padding leaf: inverted, it intersects no box.
#[inline(always)]
fn empty_box() -> Aabb {
    AabbTrait::new_invalid()
}

/// The implicit tree over `boxes` (one per segment): `(nodes, leaf_base)`.
pub(crate) fn build_tree(boxes: Span<Aabb>) -> (Span<Aabb>, u32) {
    let n = boxes.len();
    if n == 0 {
        // An empty polyline: a zero root box, so that its AABB stays representable.
        return (array![empty_box(), Default::default()].span(), 1);
    }
    let mut leaf_base: u32 = 1;
    while leaf_base < n {
        leaf_base *= 2;
    }
    // Levels from the leaves up, then written root first.
    let mut level: Array<Aabb> = array![];
    level.append_span(boxes);
    let mut k = n;
    while k != leaf_base {
        level.append(empty_box());
        k += 1;
    }
    let mut levels: Array<Span<Aabb>> = array![level.span()];
    let mut current = level.span();
    while current.len() > 1 {
        let mut next = array![];
        let mut rest = current;
        while let Some(left) = rest.pop_front() {
            let right = rest.pop_front().unwrap();
            next.append((*left).merged(*right));
        }
        current = next.span();
        levels.append(current);
    }
    let mut nodes = array![empty_box()];
    let mut depth = levels.len();
    while depth != 0 {
        depth -= 1;
        nodes.append_span(*levels.at(depth));
    }
    (nodes.span(), leaf_base)
}

/// The outward pseudo-normal of every vertex (upstream `compute_pseudo_normals`): the normalised
/// sum of the unit outward normals of its segments (zero for an isolated vertex).
fn compute_pseudo_normals(vertices: Span<Vec2>, indices: Span<[u32; 2]>) -> Span<Vec2> {
    let mut dict: Felt252Dict<Nullable<Vec2>> = Default::default();
    for idx in indices {
        let [a, b] = *idx;
        let n = face_normal(*vertices.at(a), *vertices.at(b));
        let (ka, kb): (felt252, felt252) = (a.into(), b.into());
        let na = dict.get(ka).deref_or(Vec2Trait::ZERO);
        dict.insert(ka, NullableTrait::new(na + n));
        let nb = dict.get(kb).deref_or(Vec2Trait::ZERO);
        dict.insert(kb, NullableTrait::new(nb + n));
    }
    let mut out = array![];
    let mut i: u32 = 0;
    while i != vertices.len() {
        let key: felt252 = i.into();
        out.append(dict.get(key).deref_or(Vec2Trait::ZERO).normalize_or_zero());
        i += 1;
    }
    out.span()
}

/// Upstream `ccw_face_normal([a, b])` or zero for a degenerate segment.
#[inline(always)]
fn face_normal(a: Vec2, b: Vec2) -> Vec2 {
    let ab = b - a;
    Vec2 { x: ab.y, y: -ab.x }.normalize_or_zero()
}

/// The boxes of the segments of `indices`.
fn segment_boxes(vertices: Span<Vec2>, indices: Span<[u32; 2]>) -> Span<Aabb> {
    let mut boxes = array![];
    for idx in indices {
        let [a, b] = *idx;
        boxes.append(SegmentTrait::new(*vertices.at(a), *vertices.at(b)).compute_local_aabb());
    }
    boxes.span()
}

#[generate_trait]
pub impl PolylineImpl of PolylineTrait {
    /// A polyline (upstream `Polyline::new`): `indices` pairs vertex indices into segments;
    /// `None` chains the vertices (`[0, 1], [1, 2], …`, no segment for fewer than two vertices).
    /// #### Panics
    /// * [`errors::VERTEX_INDEX`] when an index is not below `vertices.len()`.
    fn new(vertices: Span<Vec2>, indices: Option<Span<[u32; 2]>>) -> Polyline {
        Self::with_flags(vertices, indices, PolylineFlagsTrait::empty())
    }

    /// [`PolylineTrait::new`] with `flags` (upstream `with_flags`): `ORIENTED` computes the
    /// outward pseudo-normals.
    /// #### Panics
    /// * As [`PolylineTrait::new`].
    fn with_flags(
        vertices: Span<Vec2>, indices: Option<Span<[u32; 2]>>, flags: PolylineFlags,
    ) -> Polyline {
        let indices = match indices {
            Some(indices) => indices,
            None => {
                let mut chain = array![];
                let mut i: u32 = 1;
                while i < vertices.len() {
                    chain.append([i - 1, i]);
                    i += 1;
                }
                chain.span()
            },
        };
        let n_vertices = vertices.len();
        for idx in indices {
            let [a, b] = *idx;
            assert(a < n_vertices && b < n_vertices, errors::VERTEX_INDEX);
        }
        let (nodes, leaf_base) = build_tree(segment_boxes(vertices, indices));
        let pseudo_normals = if flags.contains(ORIENTED) {
            compute_pseudo_normals(vertices, indices)
        } else {
            array![].span()
        };
        Polyline { vertices, indices, flags, pseudo_normals, nodes, leaf_base }
    }

    /// Replaces the flags (upstream `set_flags`), computing or dropping the pseudo-normals.
    fn set_flags(ref self: Polyline, flags: PolylineFlags) {
        self.flags = flags;
        self
            .pseudo_normals =
                if flags.contains(ORIENTED) {
                    compute_pseudo_normals(self.vertices, self.indices)
                } else {
                    array![].span()
                };
    }

    /// The flags (upstream `flags`).
    #[inline(always)]
    fn flags(self: @Polyline) -> PolylineFlags {
        *self.flags
    }

    /// `true` when the polyline is `ORIENTED` (its lowest flag bit).
    #[inline(always)]
    fn is_oriented(self: @Polyline) -> bool {
        let (_, bit) = DivRem::div_rem(*self.flags.bits, TWO_U8_NZ);
        bit == 1
    }

    /// The outward pseudo-normal of every vertex when `ORIENTED` (upstream `pseudo_normals`).
    fn pseudo_normals(self: @Polyline) -> Option<Span<Vec2>> {
        if self.is_oriented() {
            Some(*self.pseudo_normals)
        } else {
            None
        }
    }

    /// The pseudo-normals of segment `i` when `ORIENTED` (upstream
    /// `segment_normal_constraints`); `None` otherwise or for a degenerate segment.
    fn segment_normal_constraints(self: @Polyline, i: u32) -> Option<SegmentPseudoNormals> {
        if !self.is_oriented() {
            return None;
        }
        let [a, b] = *self.indices.at(i);
        let ab = *self.vertices.at(b) - *self.vertices.at(a);
        let face = Vec2 { x: ab.y, y: -ab.x }.try_normalize()?;
        Some(
            SegmentPseudoNormals {
                face, edges: [*self.pseudo_normals.at(a), *self.pseudo_normals.at(b)],
            },
        )
    }

    /// The vertex buffer (upstream `vertices`).
    #[inline(always)]
    fn vertices(self: @Polyline) -> Span<Vec2> {
        *self.vertices
    }

    /// The segments' vertex indices (upstream `indices`).
    #[inline(always)]
    fn indices(self: @Polyline) -> Span<[u32; 2]> {
        *self.indices
    }

    /// The indices flattened, two per segment (upstream `flat_indices`).
    fn flat_indices(self: @Polyline) -> Array<u32> {
        let mut out = array![];
        for idx in *self.indices {
            let [a, b] = *idx;
            out.append(a);
            out.append(b);
        }
        out
    }

    /// The number of segments (upstream `num_segments`).
    #[inline(always)]
    fn num_segments(self: @Polyline) -> u32 {
        self.indices.len()
    }

    /// Segment `i` (upstream `segment`).
    /// #### Panics
    /// * [`errors::SEGMENT_INDEX`] when `i >= num_segments`.
    fn segment(self: @Polyline, i: u32) -> Segment {
        assert(i < self.indices.len(), errors::SEGMENT_INDEX);
        let [a, b] = *self.indices.at(i);
        SegmentTrait::new(*self.vertices.at(a), *self.vertices.at(b))
    }

    /// Every segment, in index order (upstream `segments`).
    fn segments(self: @Polyline) -> Array<Segment> {
        let mut out = array![];
        let vertices = *self.vertices;
        for idx in *self.indices {
            let [a, b] = *idx;
            out.append(SegmentTrait::new(*vertices.at(a), *vertices.at(b)));
        }
        out
    }

    /// The polyline feature of `feature` on segment `segment` (upstream
    /// `segment_feature_to_polyline_feature`): always the segment's face, as upstream.
    #[inline(always)]
    fn segment_feature_to_polyline_feature(
        self: @Polyline, segment: u32, feature: FeatureId,
    ) -> FeatureId {
        FeatureIdTrait::face(segment)
    }

    /// The local bounding box (upstream `local_aabb`, the tree's root box). An empty polyline
    /// answers an inverted box.
    #[inline(always)]
    fn local_aabb(self: @Polyline) -> Aabb {
        *self.nodes.at(1)
    }

    /// The root box moved by `pose` (upstream `aabb`: `local_aabb().transform_by(pose)`,
    /// conservative under rotation).
    #[inline(always)]
    fn aabb(self: @Polyline, pose: Pose2) -> Aabb {
        self.local_aabb().transform_by(pose)
    }

    /// Upstream name of [`PolylineTrait::local_aabb`] on `Shape`.
    #[inline(always)]
    fn compute_local_aabb(self: @Polyline) -> Aabb {
        self.local_aabb()
    }

    /// Upstream name of [`PolylineTrait::aabb`] on `Shape`.
    #[inline(always)]
    fn compute_aabb(self: @Polyline, pose: Pose2) -> Aabb {
        self.aabb(pose)
    }

    /// The bounding sphere of the local box (upstream `local_bounding_sphere`).
    fn local_bounding_sphere(self: @Polyline) -> BoundingSphere {
        self.local_aabb().bounding_sphere()
    }

    /// [`PolylineTrait::local_bounding_sphere`] placed at `pose` (upstream `bounding_sphere`).
    fn bounding_sphere(self: @Polyline, pose: Pose2) -> BoundingSphere {
        self.local_bounding_sphere().transform_by(pose)
    }

    /// Zero: a polyline has no area (upstream `Shape::mass_properties`).
    #[inline(always)]
    fn mass_properties(self: @Polyline, density: Fixed) -> MassProperties {
        Default::default()
    }

    /// The node boxes of the implicit tree, root at index 1 (upstream `bvh`, see the module
    /// documentation for the layout).
    #[inline(always)]
    fn bvh(self: @Polyline) -> Span<Aabb> {
        *self.nodes
    }

    /// The polyline with every vertex multiplied component-wise by `scale` (upstream `scaled`),
    /// tree and pseudo-normals recomputed.
    fn scaled(self: @Polyline, scale: Vec2) -> Polyline {
        let mut vertices = array![];
        for p in *self.vertices {
            vertices.append(*p * scale);
        }
        Self::with_flags(vertices.span(), Some(*self.indices), *self.flags)
    }

    /// Reverses the polyline (upstream `reverse`): every segment swapped, the segment order
    /// reversed, tree and pseudo-normals recomputed.
    fn reverse(ref self: Polyline) {
        let mut reversed = array![];
        let mut indices = self.indices;
        while let Some(idx) = indices.pop_back() {
            let [a, b] = *idx;
            reversed.append([b, a]);
        }
        self = Self::with_flags(self.vertices, Some(reversed.span()), self.flags);
    }

    /// Replaces the vertices, keeping the indices (upstream `set_vertices`); tree and
    /// pseudo-normals recomputed.
    /// #### Panics
    /// * [`errors::VERTEX_COUNT`] when the number of vertices changes.
    fn set_vertices(ref self: Polyline, vertices: Span<Vec2>) {
        assert(vertices.len() == self.vertices.len(), errors::VERTEX_COUNT);
        self = Self::with_flags(vertices, Some(self.indices), self.flags);
    }

    /// The closed loops of a polyline whose segments are chained loop after loop, each loop
    /// ending on its first vertex (upstream `extract_connected_components`): one polyline per
    /// loop with its own vertices.
    fn extract_connected_components(self: @Polyline) -> Array<Polyline> {
        let vertices = *self.vertices;
        let indices = *self.indices;
        let mut components = array![];
        if indices.is_empty() {
            return components;
        }
        let mut start_i: u32 = 0;
        let [mut start_node, _] = *indices.at(0);
        let mut component_vertices = array![];
        let mut component_indices = array![];
        let mut i: u32 = 0;
        while i != indices.len() {
            let [a, b] = *indices.at(i);
            component_vertices.append(*vertices.at(a));
            if b != start_node {
                component_indices.append([i - start_i, i - start_i + 1]);
            } else {
                component_indices.append([i - start_i, 0]);
                components
                    .append(Self::new(component_vertices.span(), Some(component_indices.span())));
                component_vertices = array![];
                component_indices = array![];
                if i + 1 < indices.len() {
                    let [next, _] = *indices.at(i + 1);
                    start_node = next;
                    start_i = i + 1;
                }
            }
            i += 1;
        }
        components
    }

    /// The indices of the segments whose box intersects `aabb` (local frame), ascending: the
    /// stackless walk of the implicit tree (upstream: `bvh().intersect_aabb`).
    fn segments_in_aabb(self: @Polyline, aabb: Aabb) -> Array<u32> {
        let mut out = array![];
        let nodes = *self.nodes;
        let leaf_base = *self.leaf_base;
        let n = self.indices.len();
        if n == 0 {
            return out;
        }
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
            // Next subtree: climb while `k` is a right child, then step to the right sibling.
            let mut done = false;
            loop {
                if k == 1 {
                    done = true;
                    break;
                }
                let (parent, right) = DivRem::div_rem(k, TWO_NZ);
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

/// Rejected candidates, kept for the `gas_*` ranking and as oracles.
#[cfg(test)]
pub mod alternatives {
    use crate::aabb::{Aabb, AabbTrait};
    use crate::shape::segment::SegmentTrait;
    use super::{Polyline, PolylineTrait};

    /// The prefilter as a linear scan: each segment's box computed and tested, ascending.
    pub fn segments_in_aabb_linear(polyline: @Polyline, aabb: Aabb) -> Array<u32> {
        let mut out = array![];
        let vertices = polyline.vertices();
        let mut i: u32 = 0;
        for idx in polyline.indices() {
            let [a, b] = *idx;
            let seg = SegmentTrait::new(*vertices.at(a), *vertices.at(b));
            if seg.compute_local_aabb().intersects(aabb) {
                out.append(i);
            }
            i += 1;
        }
        out
    }
}

#[cfg(test)]
mod tests;
