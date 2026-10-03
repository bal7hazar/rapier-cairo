//! Contact manifolds of the pairs with a [`Compound`] whose parts constrain their contact normals
//! (lot CE): Parry 0.31's `contact_manifolds_composite_shape_shape`,
//! `contact_manifolds_composite_shape_composite_shape` and
//! `contact_manifolds_heightfield_composite_shape` with each compound part's
//! `part_normal_constraints` handed to `contact_manifold_convex_convex`.
//!
//! These are copies of [`super::compound`]'s functions, reached only by
//! [`contact_manifolds_composite_constrained`], which a step selects through the constrained
//! composite strategy
//! (`rapier_dynamics2d::narrow_phase::strategies::ConstrainedCompositeManifolds`):
//! the default step does not compile them. A compound without `FIX_INTERNAL_EDGES` has no cones,
//! and every part pair then goes to [`super::compound::part_manifold`], the unconstrained
//! function: its manifolds are the unconstrained strategy's.
//!
//! # Which pairs are constrained
//!
//! As upstream's `contact_manifold_convex_convex`: a convex–ball pair
//! (`contact_manifold_convex_ball_constrained`) and every pair of the PFM–PFM generator
//! (`contact_manifold_pfm_pfm_part_constrained`, cuboid–triangle included). Ball–ball,
//! cuboid–cuboid, capsule–capsule and half-space pairs ignore the constraints, as upstream's.

use fixed::Fixed;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::aabb::AabbTrait;
use crate::contact::ContactManifold;
use crate::contact_generators::convex_ball::{
    contact_manifold_ball_convex_constrained, contact_manifold_convex_ball_constrained,
};
use crate::contact_generators::pfm_pfm::contact_manifold_pfm_pfm_part_constrained;
use crate::shape::{
    Capsule, Compound, CompoundPseudoNormals, CompoundTrait, HeightFieldTrait, SegmentTrait, Shape,
    ShapeTrait,
};
use super::compound::{half_extents_sq, part_manifold, posed_parts_in_aabb, root_aabb};
use super::{contact_manifolds_composite, previous_or_new};

/// [`part_manifold`] with the parts' normal constraints (see the module documentation); `false`
/// for an unsupported pair (manifold cleared).
pub fn part_manifold_constrained(
    pos12: Pose2,
    shape1: Shape,
    shape2: Shape,
    constraints1: Option<CompoundPseudoNormals>,
    constraints2: Option<CompoundPseudoNormals>,
    prediction: Fixed,
    ref manifold: ContactManifold,
) -> bool {
    if constraints1.is_none() && constraints2.is_none() {
        return part_manifold(pos12, shape1, shape2, prediction, ref manifold);
    }
    match (shape1, shape2) {
        (Shape::Ball(_), Shape::Ball(_)) | (Shape::Cuboid(_), Shape::Cuboid(_)) |
        (Shape::Capsule(_), Shape::Capsule(_)) | (Shape::HalfSpace(_), _) |
        (_, Shape::HalfSpace(_)) => part_manifold(pos12, shape1, shape2, prediction, ref manifold),
        (
            _, Shape::Ball(ball2),
        ) => {
            contact_manifold_convex_ball_constrained(
                pos12, shape1, ball2, constraints1, prediction, ref manifold,
            );
            true
        },
        (
            Shape::Ball(ball1), _,
        ) => {
            contact_manifold_ball_convex_constrained(
                pos12, ball1, shape2, constraints2, prediction, ref manifold,
            );
            true
        },
        _ => contact_manifold_pfm_pfm_part_constrained(
            pos12, shape1, shape2, constraints1, constraints2, prediction, ref manifold,
        ),
    }
}

/// [`part_manifold_constrained`] behind a one-iteration loop (charged only when it runs).
fn metered(
    pos12: Pose2,
    shape1: Shape,
    shape2: Shape,
    constraints1: Option<CompoundPseudoNormals>,
    constraints2: Option<CompoundPseudoNormals>,
    prediction: Fixed,
    ref manifold: ContactManifold,
) {
    let mut pending = true;
    while pending {
        let _ = part_manifold_constrained(
            pos12, shape1, shape2, constraints1, constraints2, prediction, ref manifold,
        );
        pending = false;
    }
}

/// The cones of part `id` of `shape` when it is a compound, `None` otherwise.
fn constraints_of(shape: Shape, id: u32) -> Option<CompoundPseudoNormals> {
    match shape {
        Shape::Compound(c) => c.unbox().part_normal_constraints(id),
        _ => None,
    }
}

/// `super::compound::contact_manifolds_compound_shape` with the parts' cones.
pub fn contact_manifolds_compound_shape_constrained(
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
        let cones = compound.part_normal_constraints(id);
        if flipped {
            let mut manifold = previous_or_new(previous, 0, id);
            metered(pos21.mul(part_pose), other, part, None, cones, prediction, ref manifold);
            out.append(manifold);
        } else {
            let mut manifold = previous_or_new(previous, id, 0);
            metered(part_pose.inv_mul(pos12), part, other, cones, None, prediction, ref manifold);
            out.append(manifold);
        }
    }
    out
}

/// `super::compound`'s two-composite manifolds with the parts' cones.
fn contact_manifolds_two_composites_constrained(
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
        let cones1 = constraints_of(c1, id1);
        for (id2, part_pose2, part2) in posed_parts_in_aabb(c2, aabb1_2) {
            let pos2211 = part_pose2.inv_mul(pos211);
            let cones2 = constraints_of(c2, id2);
            if flipped {
                let mut manifold = previous_or_new(previous, id2, id1);
                metered(pos2211, part2, part1, cones2, cones1, prediction, ref manifold);
                out.append(manifold);
            } else {
                let mut manifold = previous_or_new(previous, id1, id2);
                metered(pos2211.inverse(), part1, part2, cones1, cones2, prediction, ref manifold);
                out.append(manifold);
            }
        }
    }
    out
}

/// `super::compound`'s heightfield–compound manifolds with the compound parts' cones.
fn contact_manifolds_heightfield_compound_constrained(
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
        let cell = Shape::Capsule(Capsule { segment, radius: fixed::ZERO });
        let aabb1_2 = segment.compute_aabb(pos21).loosened(prediction);
        for id2 in compound.parts_in_aabb(aabb1_2) {
            let (part_pose2, part2) = compound.part(id2);
            let cones = compound.part_normal_constraints(id2);
            if flipped {
                let mut manifold = previous_or_new(previous, id2, id1);
                metered(
                    part_pose2.inv_mul(pos21), part2, cell, cones, None, prediction, ref manifold,
                );
                out.append(manifold);
            } else {
                let mut manifold = previous_or_new(previous, id1, id2);
                metered(pos12.mul(part_pose2), cell, part2, None, cones, prediction, ref manifold);
                out.append(manifold);
            }
        }
    }
    out
}

/// `super::contact_manifolds_composite` with the compound parts' normal constraints (see the
/// module documentation): every pair with a compound goes through the constrained copies, every
/// other one (a polyline or a heightfield against a convex shape, two composites without a
/// compound, two convex shapes) to `super::contact_manifolds_composite` itself.
/// #### Panics
/// * As `super::contact_manifolds_composite`, and the constrained generators'.
pub fn contact_manifolds_composite_constrained(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, previous: Span<ContactManifold>,
) -> Option<Array<ContactManifold>> {
    match (shape1, shape2) {
        (
            Shape::HeightField(_), Shape::Compound(c2),
        ) => Some(
            contact_manifolds_heightfield_compound_constrained(
                pos12, shape1, @c2.unbox(), prediction, previous, false,
            ),
        ),
        (
            Shape::Compound(c1), Shape::HeightField(_),
        ) => Some(
            contact_manifolds_heightfield_compound_constrained(
                pos12.inverse(), shape2, @c1.unbox(), prediction, previous, true,
            ),
        ),
        (Shape::Compound(c1), _) => if shape2.is_composite() {
            Some(
                contact_manifolds_two_composites_constrained(
                    pos12, shape1, shape2, prediction, previous,
                ),
            )
        } else {
            Some(
                contact_manifolds_compound_shape_constrained(
                    pos12, @c1.unbox(), shape2, prediction, previous, false,
                ),
            )
        },
        (_, Shape::Compound(c2)) => if shape1.is_composite() {
            Some(
                contact_manifolds_two_composites_constrained(
                    pos12, shape1, shape2, prediction, previous,
                ),
            )
        } else {
            Some(
                contact_manifolds_compound_shape_constrained(
                    pos12.inverse(), @c2.unbox(), shape1, prediction, previous, true,
                ),
            )
        },
        _ => contact_manifolds_composite(pos12, shape1, shape2, prediction, previous),
    }
}
