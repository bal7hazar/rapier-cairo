//! Shape-pair queries with a [`Compound`] first (work package SH2b): Parry's
//! `CompositeShapeRef::{intersects_shape, distance_to_shape, closest_points_to_shape,
//! contact_with_shape, cast_shape, cast_shape_nonlinear}` on a compound, whose parts carry a pose.
//!
//! Each part answers through the convex (or, for a composite `shape2`, composite) tables of
//! `super` with the other shape placed in the part's frame (`part_pose.inv_mul(pos12)`, the
//! velocity rotated into it, a nonlinear motion `prepend`ed with the part's pose), and its answer
//! is moved back into the compound's frame (`transform1_by`), as upstream. The `*_part` functions
//! return the part's index with the answer (upstream's `(u32, _)` results).
//!
//! # Deviations
//!
//! * Parts are visited in ascending index (upstream: BVH order, best first), the first strictly
//!   better part winning: only exact ties may pick another part than upstream.
//! * Candidate parts: the box scan of `CompoundTrait::parts_in_aabb` where upstream filters by box
//!   (intersection, contact); every part where upstream's best-first search prunes by bound
//!   (distance, closest points, casts), which visits more parts for the same answer.

use fixed::{Fixed, MAX};
use glam_core::Vec2;
use rapier_math::math_ext::norm2::norm2_sq_wide;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::aabb::AabbTrait;
use crate::query::nonlinear_shape_cast::{NonlinearRigidMotion, NonlinearRigidMotionTrait};
use crate::query::shape_cast::{ShapeCastHit, ShapeCastHitTrait, ShapeCastOptions};
use crate::query::{ClosestPoints, Contact, ContactTrait};
use crate::shape::{Compound, CompoundTrait, Shape, ShapeTrait};

/// Upstream `intersects_shape` on a compound: a part whose box meets `shape2`'s box intersects
/// it (unsupported parts count as disjoint); the index of the first such part, ascending.
pub fn intersection_test_compound_part(
    compound: @Compound, pos12: Pose2, shape2: Shape,
) -> Option<u32> {
    let aabb2 = shape2.compute_aabb(pos12);
    for id in compound.parts_in_aabb(aabb2) {
        let (pose, part) = compound.part(id);
        if super::intersection_part(pose.inv_mul(pos12), part, shape2) == Some(true) {
            return Some(id);
        }
    }
    None
}

/// Upstream `distance_to_shape` on a compound: the smallest distance of the parts to `shape2`
/// (unsupported parts skipped) and its part, `None` for no supported part.
pub fn distance_compound_part(
    compound: @Compound, pos12: Pose2, shape2: Shape,
) -> Option<(u32, Fixed)> {
    let mut best: Option<(u32, Fixed)> = None;
    let mut i: u32 = 0;
    for part in compound.shapes() {
        let (pose, shape) = *part;
        if let Some(d) = super::distance_part(pose.inv_mul(pos12), shape, shape2) {
            let better = match best {
                Some((_, b)) => d < b,
                None => true,
            };
            if better {
                best = Some((i, d));
            }
        }
        i += 1;
    }
    best
}

/// Upstream `closest_points_to_shape` on a compound: the first part answering `Intersecting`, or
/// the closest `WithinMargin` pair strictly closer than `max_dist` (its first point moved into
/// the compound's frame); `None` for neither.
pub fn closest_points_compound_part(
    compound: @Compound, pos12: Pose2, shape2: Shape, max_dist: Fixed,
) -> Option<(u32, ClosestPoints)> {
    let bound: i128 = max_dist.raw.into();
    let mut best_sq = bound * bound;
    let mut best: Option<(u32, ClosestPoints)> = None;
    let mut i: u32 = 0;
    for part in compound.shapes() {
        let (pose, shape) = *part;
        match super::closest_points_part(pose.inv_mul(pos12), shape, shape2, max_dist) {
            Some(ClosestPoints::Intersecting) => { return Some((i, ClosestPoints::Intersecting)); },
            Some(ClosestPoints::WithinMargin((
                a, b,
            ))) => {
                let a = pose.transform_point(a);
                let d = a - pos12.transform_point(b);
                let sq = norm2_sq_wide(d.x, d.y);
                if sq < best_sq {
                    best_sq = sq;
                    best = Some((i, ClosestPoints::WithinMargin((a, b))));
                }
            },
            _ => {},
        }
        i += 1;
    }
    best
}

/// Upstream `contact_with_shape` on a compound: the deepest contact (strictly smaller `dist`)
/// among the parts whose box meets `shape2`'s box loosened by `prediction`, moved into the
/// compound's frame.
pub fn contact_compound_part(
    compound: @Compound, pos12: Pose2, shape2: Shape, prediction: Fixed,
) -> Option<(u32, Contact)> {
    let aabb2 = shape2.compute_aabb(pos12).loosened(prediction);
    let mut best: Option<(u32, Contact)> = None;
    for id in compound.parts_in_aabb(aabb2) {
        let (pose, part) = compound.part(id);
        if let Some(Some(c)) = super::contact_part(pose.inv_mul(pos12), part, shape2, prediction) {
            let replace = match best {
                Some((_, b)) => c.dist < b.dist,
                None => true,
            };
            if replace {
                let mut c = c;
                c.transform1_by_mut(pose);
                best = Some((id, c));
            }
        }
    }
    best
}

/// Keeps `(id, hit)` when its time of impact is strictly below the best one.
#[inline(always)]
fn keep_earlier(ref best: Option<(u32, ShapeCastHit)>, id: u32, hit: ShapeCastHit) {
    let replace = match best {
        Some((_, b)) => hit.time_of_impact < b.time_of_impact,
        None => true,
    };
    if replace {
        best = Some((id, hit));
    }
}

/// Upstream `cast_shape` on a compound: the earliest impact of the parts (strictly earlier wins,
/// strictly below `max_time_of_impact`, unsupported parts skipped), moved into the compound's
/// frame.
pub fn cast_shapes_compound_part(
    compound: @Compound, pos12: Pose2, vel12: Vec2, shape2: Shape, options: ShapeCastOptions,
) -> Option<(u32, ShapeCastHit)> {
    let mut best: Option<(u32, ShapeCastHit)> = None;
    let mut i: u32 = 0;
    for part in compound.shapes() {
        let (pose, shape) = *part;
        let vel = pose.rotation.inverse_rotate(vel12);
        if let Some(Some(hit)) =
            super::cast_part(pose.inv_mul(pos12), vel, shape, shape2, options) {
            if hit.time_of_impact < options.max_time_of_impact {
                keep_earlier(ref best, i, hit.transform1_by(pose));
            }
        }
        i += 1;
    }
    best
}

/// Upstream `cast_shape_nonlinear` on a compound: the earliest impact of the parts, each moving
/// with `motion1.prepend(part_pose)` (strictly earlier wins, strictly below `end_time`,
/// unsupported parts skipped), moved into the compound's frame.
pub fn cast_shapes_nonlinear_compound_part(
    motion1: NonlinearRigidMotion,
    compound: @Compound,
    motion2: NonlinearRigidMotion,
    shape2: Shape,
    start_time: Fixed,
    end_time: Fixed,
    stop_at_penetration: bool,
) -> Option<(u32, ShapeCastHit)> {
    let mut best: Option<(u32, ShapeCastHit)> = None;
    let mut i: u32 = 0;
    for part in compound.shapes() {
        let (pose, shape) = *part;
        if let Some(Some(hit)) =
            crate::query::nonlinear_shape_cast::cast_shapes_nonlinear(
                (@motion1).prepend(pose),
                shape,
                motion2,
                shape2,
                start_time,
                end_time,
                stop_at_penetration,
            ) {
            if hit.time_of_impact < end_time {
                keep_earlier(ref best, i, hit.transform1_by(pose));
            }
        }
        i += 1;
    }
    best
}

/// `fixed::MAX` when no part answers, as the polyline's.
#[inline(always)]
pub fn distance_or_max(answer: Option<(u32, Fixed)>) -> Fixed {
    match answer {
        Some((_, d)) => d,
        None => MAX,
    }
}
