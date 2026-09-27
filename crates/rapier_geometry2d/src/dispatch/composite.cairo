//! Contact manifolds of a composite shape against a convex one (work package SH2a): Parry's
//! `contact_manifolds_composite_shape_shape` (polyline) and `contact_manifolds_heightfield_shape`
//! (2D heightfield), one manifold per part.
//!
//! The parts are the segments of a polyline whose box meets the other shape's box loosened by the
//! prediction (`PolylineTrait::segments_in_aabb`, the implicit tree), and the enabled cells of a
//! heightfield in that box's abscissa range (`HeightFieldTrait::elements_in_local_aabb`), each in
//! the composite's frame. A heightfield cell is a zero-radius capsule, as upstream
//! (`Capsule::new(a, b, 0.0)`), a polyline part a segment. Every part gets a manifold, empty or
//! not, as upstream keeps one per part of the box query; `subshape1` (composite first) or
//! `subshape2` (composite second) is the part's index, the other one zero. A part's previous
//! manifold (same sub-shape ids in `previous`) is updated in place, so its warm-start data follow
//! the feature ids.
//!
//! The convex pair of each part follows upstream's `contact_manifold_convex_convex`
//! (`contact_manifold_part`): a ball, a half-space or two capsules go to
//! [`crate::dispatch::contact_manifold`] (metered: charged the reached generator only), every
//! other pair to the PFM–PFM generator ([`contact_manifold_pfm_pfm_part`], SAT where upstream
//! runs GJK), whose contact order and feature ids the solver then sees as upstream's (the
//! convex table's cuboid–capsule and cuboid–segment generators answer the same points in
//! another order).
//!
//! # Deviations
//!
//! * Composite–composite pairs are unsupported (`None`): upstream's
//!   `contact_manifolds_{composite_shape_composite_shape, heightfield_composite_shape}` are not
//!   ported (the parts would need segment–segment manifolds on both sides; level geometry is
//!   fixed, and fixed–fixed pairs never reach the narrow phase).
//! * An `ORIENTED` polyline does not constrain the contact normals (upstream's
//!   `SegmentPseudoNormals` normal constraints): its point queries are one-sided, its contacts are
//!   the plain segments'.
//! * Parts in ascending index (upstream: BVH order for a polyline; ascending for a heightfield).

use fixed::{Fixed, ZERO};
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::aabb::{Aabb, AabbTrait};
use crate::contact::{ContactManifold, ContactManifoldTrait};
use crate::contact_generators::pfm_pfm::contact_manifold_pfm_pfm_part;
use crate::shape::{Capsule, HeightFieldTrait, PolylineTrait, Segment, Shape, ShapeTrait};
use super::contact_manifold;

/// A zero-radius capsule around `segment`.
#[inline(always)]
fn thin(segment: Segment) -> Capsule {
    Capsule { segment, radius: ZERO }
}

/// The convex contact manifold of one part pair (see the module documentation): upstream's
/// `contact_manifold_convex_convex` order, i.e. the convex table for a ball, a half-space or two
/// capsules, the PFM–PFM generator for every other pair (a segment or capsule part against a
/// segment, a capsule, a cuboid, a polygon, a triangle or a round shape). Returns `false` for an
/// unsupported pair (manifold cleared).
pub fn contact_manifold_part(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (Shape::Ball(_), _) | (_, Shape::Ball(_)) | (Shape::HalfSpace(_), _) |
        (_, Shape::HalfSpace(_)) |
        (
            Shape::Capsule(_), Shape::Capsule(_),
        ) => contact_manifold(pos12, shape1, shape2, prediction, ref manifold),
        _ => contact_manifold_pfm_pfm_part(pos12, shape1, shape2, prediction, ref manifold),
    }
}

/// The previous manifold of sub-shapes `(subshape1, subshape2)` in `previous`, or a fresh one.
fn previous_or_new(
    previous: Span<ContactManifold>, subshape1: u32, subshape2: u32,
) -> ContactManifold {
    for m in previous {
        if *m.subshape1 == subshape1 && *m.subshape2 == subshape2 {
            return *m;
        }
    }
    ContactManifoldTrait::with_data(subshape1, subshape2, Default::default())
}

/// The parts of the composite `shape` meeting `aabb` (its frame), as convex contact shapes:
/// segments for a polyline, zero-radius capsules for a heightfield.
fn contact_parts(shape: Shape, aabb: Aabb) -> Array<(u32, Shape)> {
    let mut out = array![];
    match shape {
        Shape::Polyline(p) => {
            let p = p.unbox();
            for id in p.segments_in_aabb(aabb) {
                out.append((id, Shape::Segment(p.segment(id))));
            }
        },
        Shape::HeightField(h) => {
            for (id, seg) in h.unbox().elements_in_local_aabb(aabb) {
                out.append((id, Shape::Capsule(thin(seg))));
            }
        },
        _ => {},
    }
    out
}

/// Parry `contact_manifolds` for a pair with exactly one composite shape (see the module
/// documentation): one manifold per part, ascending part index, each updated from its previous
/// manifold in `previous` (any order). `pos12` is the pose of `shape2` in the frame of `shape1`.
/// `None` when neither or both shapes are composite.
/// #### Panics
/// * The panics of the part generators and of `Pose2::inverse` (composite second).
pub fn contact_manifolds_composite(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, previous: Span<ContactManifold>,
) -> Option<Array<ContactManifold>> {
    let (first, second) = (shape1.is_composite(), shape2.is_composite());
    if first == second {
        return None;
    }
    let mut out = array![];
    if first {
        let aabb2 = shape2.compute_aabb(pos12).loosened(prediction);
        for (id, part) in contact_parts(shape1, aabb2) {
            let mut manifold = previous_or_new(previous, id, 0);
            let mut pending = true;
            while pending {
                let _ = contact_manifold_part(pos12, part, shape2, prediction, ref manifold);
                pending = false;
            }
            out.append(manifold);
        }
    } else {
        let aabb1 = shape1.compute_aabb(pos12.inverse()).loosened(prediction);
        for (id, part) in contact_parts(shape2, aabb1) {
            let mut manifold = previous_or_new(previous, 0, id);
            let mut pending = true;
            while pending {
                let _ = contact_manifold_part(pos12, shape1, part, prediction, ref manifold);
                pending = false;
            }
            out.append(manifold);
        }
    }
    Some(out)
}

/// The composite arms of both tables: the manifold cleared and `false` (a composite pair has one
/// manifold per part, see [`contact_manifolds_composite`]). Out of line, and the
/// composite combinations each have an arm right before the old wildcard arm they would
/// otherwise join (as the SH1 arms): an old arm's merge point gaining branches moves its code.
#[inline(never)]
pub(crate) fn composite_unsupported(ref manifold: ContactManifold) -> bool {
    manifold.clear();
    false
}

#[cfg(test)]
mod tests;
