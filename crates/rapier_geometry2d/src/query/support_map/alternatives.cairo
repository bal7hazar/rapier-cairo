//! Rejected candidates of [`super`], kept for the `gas_*` ranking.

use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_math::pose2::Pose2;
use crate::query::normalize_and_length;
use crate::shape::Shape;
use super::{Core, closest_on, local_core, sat, transformed};

/// [`super::closest_pair`] without the pruning: every vertex is projected.
pub fn closest_pair_exhaustive(core1: Core, core2: Core) -> (Vec2, Vec2) {
    let mut best: i128 = 0;
    let mut pair = (*core1.vertices[0], *core2.vertices[0]);
    let mut first = true;
    for v2 in core2.vertices {
        let (q, sq) = closest_on(core1.vertices, *v2);
        if first || sq < best {
            best = sq;
            pair = (q, *v2);
            first = false;
        }
    }
    for v1 in core1.vertices {
        let (q, sq) = closest_on(core2.vertices, *v1);
        if sq < best {
            best = sq;
            pair = (*v1, q);
        }
    }
    pair
}

/// The separated distance of the kernel with the exhaustive search.
pub fn distance_exhaustive(pos12: Pose2, shape1: Shape, shape2: Shape) -> Fixed {
    let core1 = local_core(shape1);
    let core2 = transformed(local_core(shape2), pos12);
    match sat(core1, core2) {
        Some(best) => if best.separation <= ZERO {
            return ZERO;
        },
        None => {},
    }
    let (p1, p2) = closest_pair_exhaustive(core1, core2);
    let (_, len) = normalize_and_length(p2 - p1);
    let dist = len - core1.radius - core2.radius;
    if dist > ZERO {
        dist
    } else {
        ZERO
    }
}

/// The full kernel ([`super::witness`]) for every pair, segment cores included: the loser of
/// [`super::distance_support_map_support_map`] on two capsules / segments.
pub fn distance_witness(pos12: Pose2, shape1: Shape, shape2: Shape) -> Fixed {
    let dist = super::witness(pos12, shape1, shape2).dist;
    if dist > ZERO {
        dist
    } else {
        ZERO
    }
}
