//! Shape-pair queries with a composite shape (work package SH2a): Parry's
//! `*_composite_shape_shape` (polyline) and `*_heightfield_shape` (2D heightfield) functions, and
//! the parts both are made of.
//!
//! A composite answers through its parts, each a [`Segment`] in the composite's frame: every
//! segment of a polyline, every enabled cell of a heightfield. Each query runs the convex table
//! on the parts (`crate::dispatch::intersection_test`, `super::dispatcher::{distance,
//! closest_points, contact}`, `super::shape_cast::cast_shapes_local`,
//! `super::nonlinear_shape_cast::cast_shapes_nonlinear`), so a composite–composite pair recurses
//! exactly as upstream's dispatcher does, and keeps the best answer.
//!
//! # Upstream's support matrix, kept
//!
//! The heightfield is not a `CompositeShape` upstream: only the intersection test against a ball
//! (point query), `contact`, the linear cast and the contact manifolds support it; its
//! `distance`, `closest_points`, other intersection tests and nonlinear cast are unsupported
//! (`None`). A polyline supports every query.
//!
//! # Deviations
//!
//! * Parts are visited in ascending index (upstream: BVH order, best first), the first strictly
//!   better part winning: only exact ties may pick another part than upstream.
//! * Candidate parts: the polyline's implicit tree (`PolylineTrait::segments_in_aabb`) where
//!   upstream filters by box (intersection, contact); every part where upstream's best-first
//!   search prunes by bound (distance, closest points, casts), which visits more parts for the
//!   same answer.
//! * The heightfield's linear cast walks the cells along the ray of the moving box's centre as
//!   upstream, with the exact cell arithmetic of `HeightFieldTrait`.

use fixed::{Fixed, MAX, ZERO};
use glam_core::Vec2;
use rapier_math::math_ext::norm2::norm2_sq_wide;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::aabb::{Aabb, AabbTrait};
use crate::shape::{
    CompoundTrait, HeightField, HeightFieldTrait, Polyline, PolylineTrait, Shape, ShapeTrait,
};
use super::nonlinear_shape_cast::{NonlinearRigidMotion, cast_shapes_nonlinear};
use super::shape_cast::{ShapeCastHit, ShapeCastHitTrait, ShapeCastOptions};
use super::{ClosestPoints, ClosestPointsTrait, Contact, ContactTrait};

/// The segments of `polyline` whose box meets `aabb`, as parts.
fn polyline_parts_in_aabb(polyline: @Polyline, aabb: Aabb) -> Array<(u32, Shape)> {
    let mut out = array![];
    for id in polyline.segments_in_aabb(aabb) {
        out.append((id, Shape::Segment(polyline.segment(id))));
    }
    out
}

/// Every segment of `polyline`, as parts.
fn polyline_parts(polyline: @Polyline) -> Array<(u32, Shape)> {
    let mut out = array![];
    let mut i: u32 = 0;
    for seg in polyline.segments() {
        out.append((i, Shape::Segment(seg)));
        i += 1;
    }
    out
}

/// The enabled cells of `heightfield` that may meet `aabb`, as parts.
fn heightfield_parts_in_aabb(heightfield: @HeightField, aabb: Aabb) -> Array<(u32, Shape)> {
    let mut out = array![];
    for (id, seg) in heightfield.elements_in_local_aabb(aabb) {
        out.append((id, Shape::Segment(seg)));
    }
    out
}

/// The parts of a composite `shape` whose box may meet `aabb` (its local frame), ascending:
/// `(part index, segment)`. Empty for a convex shape.
pub fn parts_in_aabb(shape: Shape, aabb: Aabb) -> Array<(u32, Shape)> {
    match shape {
        Shape::Polyline(p) => polyline_parts_in_aabb(@p.unbox(), aabb),
        Shape::HeightField(h) => heightfield_parts_in_aabb(@h.unbox(), aabb),
        Shape::Compound(c) => {
            let c = c.unbox();
            let mut out = array![];
            for id in c.parts_in_aabb(aabb) {
                let (_, part) = c.part(id);
                out.append((id, part));
            }
            out
        },
        _ => array![],
    }
}

/// Every part of a composite `shape`, ascending (a heightfield's enabled cells). Empty for a
/// convex shape.
pub fn parts(shape: Shape) -> Array<(u32, Shape)> {
    match shape {
        Shape::Polyline(p) => polyline_parts(@p.unbox()),
        Shape::HeightField(h) => {
            let h = h.unbox();
            let mut out = array![];
            let mut i: u32 = 0;
            while i != h.num_cells() {
                if let Some(seg) = h.segment_at(i) {
                    out.append((i, Shape::Segment(seg)));
                }
                i += 1;
            }
            out
        },
        Shape::Compound(c) => {
            let mut out = array![];
            let mut i: u32 = 0;
            for part in c.unbox().shapes() {
                let (_, shape) = *part;
                out.append((i, shape));
                i += 1;
            }
            out
        },
        _ => array![],
    }
}

/// The part-level tables: the outlined copies of the public ones (an inlined table cannot be on
/// the recursion of a composite–composite pair).
fn intersection_part(pos12: Pose2, shape1: Shape, shape2: Shape) -> Option<bool> {
    crate::dispatch::intersection::intersection_test_outlined(pos12, shape1, shape2)
}

fn distance_part(pos12: Pose2, shape1: Shape, shape2: Shape) -> Option<Fixed> {
    super::dispatcher::distance_outlined(pos12, shape1, shape2)
}

fn closest_points_part(
    pos12: Pose2, shape1: Shape, shape2: Shape, max_dist: Fixed,
) -> Option<ClosestPoints> {
    super::dispatcher::closest_points_outlined(pos12, shape1, shape2, max_dist)
}

fn contact_part(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed,
) -> Option<Option<Contact>> {
    super::dispatcher::contact_outlined(pos12, shape1, shape2, prediction)
}

fn cast_part(
    pos12: Pose2, vel12: Vec2, shape1: Shape, shape2: Shape, options: ShapeCastOptions,
) -> Option<Option<ShapeCastHit>> {
    super::shape_cast::cast_shapes_local_outlined(pos12, vel12, shape1, shape2, options)
}

/// Upstream `intersection_test` for a pair with a composite shape and no ball (balls are point
/// queries, see `crate::dispatch::intersection`): a polyline intersects when one of its segments
/// meeting the other shape's box intersects it (upstream `intersection_test_composite_shape_shape`,
/// unsupported parts count as disjoint); a heightfield is unsupported (`None`).
#[inline(never)]
pub fn intersection_test_composite(pos12: Pose2, shape1: Shape, shape2: Shape) -> Option<bool> {
    match (shape1, shape2) {
        (
            Shape::Polyline(p1), _,
        ) => {
            let aabb2 = shape2.compute_aabb(pos12);
            for (_, part) in polyline_parts_in_aabb(@p1.unbox(), aabb2) {
                if intersection_part(pos12, part, shape2) == Some(true) {
                    return Some(true);
                }
            }
            Some(false)
        },
        (
            Shape::Compound(c1), _,
        ) => Some(compound::intersection_test_compound_part(@c1.unbox(), pos12, shape2).is_some()),
        (_, Shape::Polyline(_)) |
        (_, Shape::Compound(_)) => intersection_test_composite(pos12.inverse(), shape2, shape1),
        _ => None,
    }
}

/// Upstream `distance_composite_shape_shape`: the smallest distance between a polyline's
/// segments and the other shape (unsupported parts skipped, `fixed::MAX` for none); a
/// heightfield is unsupported (`None`).
#[inline(never)]
pub fn distance_composite(pos12: Pose2, shape1: Shape, shape2: Shape) -> Option<Fixed> {
    match (shape1, shape2) {
        (
            Shape::Polyline(p1), _,
        ) => {
            let mut best = MAX;
            for (_, part) in polyline_parts(@p1.unbox()) {
                if let Some(d) = distance_part(pos12, part, shape2) {
                    if d < best {
                        best = d;
                    }
                }
            }
            Some(best)
        },
        (
            Shape::Compound(c1), _,
        ) => Some(
            compound::distance_or_max(compound::distance_compound_part(@c1.unbox(), pos12, shape2)),
        ),
        (_, Shape::Polyline(_)) |
        (_, Shape::Compound(_)) => distance_composite(pos12.inverse(), shape2, shape1),
        _ => None,
    }
}

/// Upstream `closest_points_composite_shape_shape`: the first part answering `Intersecting`, or
/// the closest `WithinMargin` pair strictly closer than `max_dist`, `Disjoint` otherwise; a
/// heightfield is unsupported (`None`). The points of the second form are flipped back when the
/// polyline is shape 2.
#[inline(never)]
pub fn closest_points_composite(
    pos12: Pose2, shape1: Shape, shape2: Shape, max_dist: Fixed,
) -> Option<ClosestPoints> {
    match (shape1, shape2) {
        (
            Shape::Polyline(p1), _,
        ) => {
            let bound: i128 = max_dist.raw.into();
            let mut best_sq = bound * bound;
            let mut best = ClosestPoints::Disjoint;
            for (_, part) in polyline_parts(@p1.unbox()) {
                match closest_points_part(pos12, part, shape2, max_dist) {
                    Some(ClosestPoints::Intersecting) => {
                        return Some(ClosestPoints::Intersecting);
                    },
                    Some(ClosestPoints::WithinMargin((
                        a, b,
                    ))) => {
                        let d = a - pos12.transform_point(b);
                        let sq = norm2_sq_wide(d.x, d.y);
                        if sq < best_sq {
                            best_sq = sq;
                            best = ClosestPoints::WithinMargin((a, b));
                        }
                    },
                    _ => {},
                }
            }
            Some(best)
        },
        (
            Shape::Compound(c1), _,
        ) => Some(
            match compound::closest_points_compound_part(@c1.unbox(), pos12, shape2, max_dist) {
                Some((_, pts)) => pts,
                None => ClosestPoints::Disjoint,
            },
        ),
        (_, Shape::Polyline(_)) |
        (
            _, Shape::Compound(_),
        ) => Some(closest_points_composite(pos12.inverse(), shape2, shape1, max_dist)?.flipped()),
        _ => None,
    }
}

/// The deepest contact of the parts meeting `aabb` (strictly smaller `dist` wins).
fn best_contact(
    pos12: Pose2, parts: Array<(u32, Shape)>, shape2: Shape, prediction: Fixed,
) -> Option<Contact> {
    let mut best: Option<Contact> = None;
    for (_, part) in parts {
        if let Some(Some(c)) = contact_part(pos12, part, shape2, prediction) {
            let replace = match best {
                Some(b) => c.dist < b.dist,
                None => true,
            };
            if replace {
                best = Some(c);
            }
        }
    }
    best
}

/// Upstream `contact_heightfield_shape` / `contact_composite_shape_shape`: the deepest contact
/// among the parts meeting the other shape's box loosened by the prediction (a heightfield:
/// by `max(prediction, 0)`), flipped back when the composite is shape 2.
#[inline(never)]
pub fn contact_composite(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed,
) -> Option<Option<Contact>> {
    match (shape1, shape2) {
        (
            Shape::HeightField(h1), _,
        ) => {
            let margin = if prediction > ZERO {
                prediction
            } else {
                ZERO
            };
            let aabb2 = shape2.compute_aabb(pos12).loosened(margin);
            Some(
                best_contact(
                    pos12, heightfield_parts_in_aabb(@h1.unbox(), aabb2), shape2, prediction,
                ),
            )
        },
        (
            _, Shape::HeightField(_),
        ) => Some(
            match contact_composite(pos12.inverse(), shape2, shape1, prediction)? {
                Some(c) => Some(c.flipped()),
                None => None,
            },
        ),
        (
            Shape::Polyline(p1), _,
        ) => {
            let aabb2 = shape2.compute_aabb(pos12).loosened(prediction);
            Some(
                best_contact(pos12, polyline_parts_in_aabb(@p1.unbox(), aabb2), shape2, prediction),
            )
        },
        (
            Shape::Compound(c1), _,
        ) => Some(
            match compound::contact_compound_part(@c1.unbox(), pos12, shape2, prediction) {
                Some((_, c)) => Some(c),
                None => None,
            },
        ),
        (_, Shape::Polyline(_)) |
        (
            _, Shape::Compound(_),
        ) => Some(
            match contact_composite(pos12.inverse(), shape2, shape1, prediction)? {
                Some(c) => Some(c.flipped()),
                None => None,
            },
        ),
        _ => None,
    }
}

/// Keeps `hit` when its time of impact is strictly below the best one.
#[inline(always)]
fn keep_earlier(ref best: Option<ShapeCastHit>, hit: ShapeCastHit) {
    let replace = match best {
        Some(b) => hit.time_of_impact < b.time_of_impact,
        None => true,
    };
    if replace {
        best = Some(hit);
    }
}

/// `x` clamped to `[0, hi]`.
#[inline(always)]
fn clamp(x: i64, hi: i64) -> i64 {
    if x < 0 {
        0
    } else if x > hi {
        hi
    } else {
        x
    }
}

/// Upstream `cast_shapes_heightfield_shape` (2D): the cells of the moving box's abscissa range
/// (grown by one cell ahead), then the cells met by the ray of the box centre until the ray's
/// parameter reaches `max_time_of_impact`. `None` (unsupported) as soon as a part is.
fn cast_heightfield(
    pos12: Pose2, vel12: Vec2, h: @HeightField, shape2: Shape, options: ShapeCastOptions,
) -> Option<Option<ShapeCastHit>> {
    let aabb2 = shape2.compute_aabb(pos12).loosened(options.target_distance);
    let origin = aabb2.center();
    let (mut start, mut end) = h.unclamped_elements_range_in_local_aabb(aabb2);
    let right = vel12.x > ZERO;
    if right {
        end += 1;
    } else {
        start -= 1;
    }
    let cells: i64 = h.num_cells().into();
    let mut best: Option<ShapeCastHit> = None;
    let mut i: u32 = clamp(start, cells).try_into().unwrap();
    let stop: u32 = clamp(end, cells).try_into().unwrap();
    while i < stop {
        if let Some(seg) = h.segment_at(i) {
            if let Some(hit) = cast_part(pos12, vel12, Shape::Segment(seg), shape2, options)? {
                keep_earlier(ref best, hit);
            }
        }
        i += 1;
    }
    if vel12.x == ZERO {
        return Some(best);
    }
    let mut curr: i64 = if right {
        let e = end - 1;
        if e < 0 {
            0
        } else {
            e
        }
    } else if start < cells - 1 {
        start
    } else {
        cells - 1
    };
    while (right && curr < cells - 1) || (!right && curr > 0) {
        // The ray's parameter at the next cell boundary: `(x_boundary - origin.x) / vel.x`.
        let boundary = if right {
            curr += 1;
            h.x_at(curr.try_into().unwrap())
        } else {
            let b = h.x_at(curr.try_into().unwrap());
            curr -= 1;
            b
        };
        let param = (boundary - origin.x) / vel12.x;
        if param >= options.max_time_of_impact {
            break;
        }
        if let Some(seg) = h.segment_at(curr.try_into().unwrap()) {
            if let Some(hit) = cast_part(pos12, vel12, Shape::Segment(seg), shape2, options)? {
                keep_earlier(ref best, hit);
            }
        }
    }
    Some(best)
}

/// Upstream `cast_shapes` for a pair with a composite shape (no half-space–support-map pair
/// reaches it): the heightfield's cell walk ([`cast_heightfield`]) or the earliest impact of a
/// polyline's segments (strictly earlier wins, unsupported parts skipped), swapped back when the
/// composite is shape 2 (`pos12` inverted, `vel12` rotated into shape 2's frame and negated).
#[inline(never)]
pub fn cast_shapes_composite(
    pos12: Pose2, vel12: Vec2, shape1: Shape, shape2: Shape, options: ShapeCastOptions,
) -> Option<Option<ShapeCastHit>> {
    match (shape1, shape2) {
        (Shape::HeightField(h1), _) => cast_heightfield(pos12, vel12, @h1.unbox(), shape2, options),
        (
            _, Shape::HeightField(_),
        ) => {
            let vel21 = -pos12.rotation.inverse_rotate(vel12);
            Some(
                match cast_shapes_composite(pos12.inverse(), vel21, shape2, shape1, options)? {
                    Some(hit) => Some(hit.swapped()),
                    None => None,
                },
            )
        },
        (
            Shape::Polyline(p1), _,
        ) => {
            let mut best: Option<ShapeCastHit> = None;
            for (_, part) in polyline_parts(@p1.unbox()) {
                if let Some(Some(hit)) = cast_part(pos12, vel12, part, shape2, options) {
                    if hit.time_of_impact < options.max_time_of_impact {
                        keep_earlier(ref best, hit);
                    }
                }
            }
            Some(best)
        },
        (
            Shape::Compound(c1), _,
        ) => Some(
            match compound::cast_shapes_compound_part(@c1.unbox(), pos12, vel12, shape2, options) {
                Some((_, hit)) => Some(hit),
                None => None,
            },
        ),
        (_, Shape::Polyline(_)) |
        (
            _, Shape::Compound(_),
        ) => {
            let vel21 = -pos12.rotation.inverse_rotate(vel12);
            Some(
                match cast_shapes_composite(pos12.inverse(), vel21, shape2, shape1, options)? {
                    Some(hit) => Some(hit.swapped()),
                    None => None,
                },
            )
        },
        _ => None,
    }
}

/// Upstream `cast_shapes_nonlinear_composite_shape_shape`: the earliest impact of a polyline's
/// segments (strictly earlier wins, unsupported parts skipped), swapped back when the polyline is
/// shape 2; a heightfield is unsupported (`None`).
#[inline(never)]
pub fn cast_shapes_nonlinear_composite(
    motion1: NonlinearRigidMotion,
    shape1: Shape,
    motion2: NonlinearRigidMotion,
    shape2: Shape,
    start_time: Fixed,
    end_time: Fixed,
    stop_at_penetration: bool,
) -> Option<Option<ShapeCastHit>> {
    match (shape1, shape2) {
        (
            Shape::Polyline(p1), _,
        ) => {
            let mut best: Option<ShapeCastHit> = None;
            for (_, part) in polyline_parts(@p1.unbox()) {
                if let Some(Some(hit)) =
                    cast_shapes_nonlinear(
                        motion1, part, motion2, shape2, start_time, end_time, stop_at_penetration,
                    ) {
                    if hit.time_of_impact < end_time {
                        keep_earlier(ref best, hit);
                    }
                }
            }
            Some(best)
        },
        (
            Shape::Compound(c1), _,
        ) => Some(
            match compound::cast_shapes_nonlinear_compound_part(
                motion1, @c1.unbox(), motion2, shape2, start_time, end_time, stop_at_penetration,
            ) {
                Some((_, hit)) => Some(hit),
                None => None,
            },
        ),
        (_, Shape::Polyline(_)) |
        (
            _, Shape::Compound(_),
        ) => Some(
            match cast_shapes_nonlinear_composite(
                motion2, shape2, motion1, shape1, start_time, end_time, stop_at_penetration,
            )? {
                Some(hit) => Some(hit.swapped()),
                None => None,
            },
        ),
        _ => None,
    }
}

pub mod compound;
#[cfg(test)]
mod tests;
