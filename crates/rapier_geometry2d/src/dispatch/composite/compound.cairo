//! Contact manifolds of the pairs with a [`Compound`] (work package SH2b): Parry's
//! `contact_manifolds_composite_shape_shape` (a compound against a convex shape),
//! `contact_manifolds_composite_shape_composite_shape` (a compound against a polyline or another
//! compound) and `contact_manifolds_heightfield_composite_shape` (a heightfield against a
//! compound), one manifold per part pair.
//!
//! Every manifold is expressed in the frames of its **parts**, as upstream's: a compound part's
//! points and normal are in the part's frame (upstream records the part's pose in the manifold,
//! `subshape_pos1` / `subshape_pos2`; the port reads it back from the compound with
//! `CompoundTrait::part_pose(subshape)`, see `rapier_dynamics2d::narrow_phase::composite`). So a
//! part's previous manifold keeps its persistence data from one step to the next exactly as
//! upstream.
//!
//! The convex pair of two parts follows upstream's `contact_manifold_convex_convex`
//! ([`part_manifold`]): cuboid–cuboid through the cuboid generator, every other pair through
//! `super::contact_manifold_part` (balls, half-spaces and capsule pairs through the convex table,
//! the rest through the PFM–PFM generator).
//!
//! # Order
//!
//! As upstream: the candidate parts of the first composite (for two composites, the one with the
//! larger local box, `flipped` when that is shape 2), each followed by its candidates in the
//! second one; candidates in ascending index (upstream: BVH order).
//!
//! # Deviations
//!
//! * Polyline–polyline and polyline–heightfield pairs stay unsupported (SH2a: fixed level
//!   geometry); a heightfield–heightfield pair is unsupported upstream too.

use fixed::{Fixed, ZERO};
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::aabb::{Aabb, AabbTrait};
use crate::contact::ContactManifold;
use crate::contact_generators::cuboid_cuboid::contact_manifold_cuboid_cuboid;
use crate::shape::{
    Capsule, Compound, CompoundTrait, HeightFieldTrait, PolylineTrait, SegmentTrait, Shape,
    ShapeTrait,
};
use super::{contact_manifold_part, previous_or_new};

/// Upstream `contact_manifold_convex_convex` for two parts (see the module documentation);
/// `false` for an unsupported pair (manifold cleared).
pub fn part_manifold(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (
            Shape::Cuboid(c1), Shape::Cuboid(c2),
        ) => {
            contact_manifold_cuboid_cuboid(pos12, c1, c2, prediction, ref manifold);
            true
        },
        _ => contact_manifold_part(pos12, shape1, shape2, prediction, ref manifold),
    }
}

/// [`part_manifold`] behind a one-iteration loop (charged only when it runs).
fn metered_part_manifold(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) {
    let mut pending = true;
    while pending {
        let _ = part_manifold(pos12, shape1, shape2, prediction, ref manifold);
        pending = false;
    }
}

/// Upstream `contact_manifolds_composite_shape_shape` for a compound: `pos12` places `other` in
/// the compound's frame; `flipped` when the compound is shape 2 of the pair (its part index then
/// goes to `subshape2`, and each part pair is generated with `other` first).
pub fn contact_manifolds_compound_shape(
    pos12: Pose2,
    compound: @Compound,
    other: Shape,
    prediction: Fixed,
    previous: Span<ContactManifold>,
    flipped: bool,
) -> Array<ContactManifold> {
    let mut out = array![];
    let aabb = other.compute_aabb(pos12).loosened(prediction);
    let pos21 = if flipped {
        pos12.inverse()
    } else {
        pos12
    };
    for id in compound.parts_in_aabb(aabb) {
        let (part_pose, part) = compound.part(id);
        if flipped {
            let mut manifold = previous_or_new(previous, 0, id);
            metered_part_manifold(pos21.mul(part_pose), other, part, prediction, ref manifold);
            out.append(manifold);
        } else {
            let mut manifold = previous_or_new(previous, id, 0);
            metered_part_manifold(part_pose.inv_mul(pos12), part, other, prediction, ref manifold);
            out.append(manifold);
        }
    }
    out
}

/// The local box of a polyline or a compound (upstream `bvh().root_aabb()`).
fn root_aabb(shape: Shape) -> Aabb {
    match shape {
        Shape::Polyline(p) => p.unbox().local_aabb(),
        Shape::Compound(c) => c.unbox().local_aabb(),
        _ => shape.compute_local_aabb(),
    }
}

/// The parts of a polyline or a compound meeting `aabb` (its frame), with their poses (the
/// identity for a polyline's segments), ascending.
fn posed_parts_in_aabb(shape: Shape, aabb: Aabb) -> Array<(u32, Pose2, Shape)> {
    let mut out = array![];
    match shape {
        Shape::Polyline(p) => {
            let p = p.unbox();
            for id in p.segments_in_aabb(aabb) {
                out.append((id, Default::default(), Shape::Segment(p.segment(id))));
            }
        },
        Shape::Compound(c) => {
            let c = c.unbox();
            for id in c.parts_in_aabb(aabb) {
                let (pose, part) = c.part(id);
                out.append((id, pose, part));
            }
        },
        _ => {},
    }
    out
}

/// `|v|^2` of a box's half extents, exact.
fn half_extents_sq(aabb: Aabb) -> i128 {
    let he = aabb.half_extents();
    let (x, y): (i128, i128) = (he.x.raw.into(), he.y.raw.into());
    x * x + y * y
}

/// Upstream `contact_manifolds_composite_shape_composite_shape` for two composites of which one
/// is a compound (a polyline or a compound each). `pos12` places shape 2 in shape 1's frame.
fn contact_manifolds_two_composites(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, previous: Span<ContactManifold>,
) -> Array<ContactManifold> {
    let flipped = half_extents_sq(root_aabb(shape1)) < half_extents_sq(root_aabb(shape2));
    let (c1, c2, pos12, pos21) = if flipped {
        (shape2, shape1, pos12.inverse(), pos12)
    } else {
        (shape1, shape2, pos12, pos12.inverse())
    };
    let aabb2_1 = root_aabb(c2).transform_by(pos12).loosened(prediction);
    let mut out = array![];
    for (id1, part_pose1, part1) in posed_parts_in_aabb(c1, aabb2_1) {
        let pos211 = pos21.mul(part_pose1);
        let aabb1_2 = part1.compute_aabb(pos211).loosened(prediction);
        for (id2, part_pose2, part2) in posed_parts_in_aabb(c2, aabb1_2) {
            let pos2211 = part_pose2.inv_mul(pos211);
            if flipped {
                let mut manifold = previous_or_new(previous, id2, id1);
                metered_part_manifold(pos2211, part2, part1, prediction, ref manifold);
                out.append(manifold);
            } else {
                let mut manifold = previous_or_new(previous, id1, id2);
                metered_part_manifold(pos2211.inverse(), part1, part2, prediction, ref manifold);
                out.append(manifold);
            }
        }
    }
    out
}

/// Upstream `contact_manifolds_heightfield_composite_shape` for a compound: `pos12` places the
/// compound in the heightfield's frame; `flipped` when the heightfield is shape 2 of the pair.
fn contact_manifolds_heightfield_compound(
    pos12: Pose2,
    heightfield: Shape,
    compound: @Compound,
    prediction: Fixed,
    previous: Span<ContactManifold>,
    flipped: bool,
) -> Array<ContactManifold> {
    let mut out = array![];
    let Shape::HeightField(h) = heightfield else {
        return out;
    };
    let pos21 = pos12.inverse();
    let aabb2_1 = compound.local_aabb().transform_by(pos12).loosened(prediction);
    for (id1, segment) in h.unbox().elements_in_local_aabb(aabb2_1) {
        let cell = Shape::Capsule(Capsule { segment, radius: ZERO });
        let aabb1_2 = segment.compute_aabb(pos21).loosened(prediction);
        for id2 in compound.parts_in_aabb(aabb1_2) {
            let (part_pose2, part2) = compound.part(id2);
            if flipped {
                let mut manifold = previous_or_new(previous, id2, id1);
                metered_part_manifold(
                    part_pose2.inv_mul(pos21), part2, cell, prediction, ref manifold,
                );
                out.append(manifold);
            } else {
                let mut manifold = previous_or_new(previous, id1, id2);
                metered_part_manifold(pos12.mul(part_pose2), cell, part2, prediction, ref manifold);
                out.append(manifold);
            }
        }
    }
    out
}

/// The manifolds of two composite shapes when one is a compound (see the module
/// documentation); `None` otherwise (polyline–polyline, polyline–heightfield,
/// heightfield–heightfield).
pub fn contact_manifolds_composite_pair(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, previous: Span<ContactManifold>,
) -> Option<Array<ContactManifold>> {
    match (shape1, shape2) {
        (
            Shape::HeightField(_), Shape::Compound(c2),
        ) => Some(
            contact_manifolds_heightfield_compound(
                pos12, shape1, @c2.unbox(), prediction, previous, false,
            ),
        ),
        (
            Shape::Compound(c1), Shape::HeightField(_),
        ) => Some(
            contact_manifolds_heightfield_compound(
                pos12.inverse(), shape2, @c1.unbox(), prediction, previous, true,
            ),
        ),
        (Shape::Compound(_), _) |
        (
            _, Shape::Compound(_),
        ) => Some(contact_manifolds_two_composites(pos12, shape1, shape2, prediction, previous)),
        _ => None,
    }
}
