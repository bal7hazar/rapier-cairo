//! A dispatch table restricted to the basic shapes (work package CS2): balls, cuboids, convex
//! polygons and half-spaces, the pairs a game built from them meets. Parry composes its
//! `QueryDispatcher`s at run time (`DefaultQueryDispatcher.chain(..)`); here the choice is static,
//! so a program that dispatches through [`contact_manifold_step_basic`] only (e.g.
//! `rapier2d`'s `BasicShapesDispatcher`) does not compile the capsule, segment, triangle, round
//! and composite generators.

use fixed::Fixed;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::contact::{ContactManifold, ContactManifoldTrait};
use crate::contact_generators::ball_ball::contact_manifold_ball_ball;
use crate::contact_generators::convex_ball::{
    contact_manifold_ball_convex, contact_manifold_convex_ball,
};
use crate::contact_generators::cuboid_cuboid::cuboid_cuboid_fresh;
use crate::contact_generators::halfspace_pfm::contact_manifold_halfspace_pfm;
use crate::contact_generators::polygon_polygon::{
    contact_manifold_polygon_cuboid, polygon_cuboid_fresh, polygon_polygon_fresh,
};
use crate::manifold::ManifoldTrait;
use crate::shape::Shape;

/// Panics of [`contact_manifold_step_basic`].
pub mod errors {
    /// A shape other than a ball, a cuboid, a convex polygon or a half-space.
    pub const UNSUPPORTED: felt252 = 'Dispatch: not a basic shape';
}

/// `super::contact_manifold_step` on the pairs of balls, cuboids, convex polygons and
/// half-spaces: the same arms, generators and results (bit for bit) for each of those pairs;
/// a half-space pair is unsupported (`false`, manifold cleared) as there.
///
/// Arguments, returned value and cost: as `super::contact_manifold_step` (inline it into a loop
/// body).
///
/// # Panics
/// As `super::contact_manifold_step` on the same pair, and `errors::UNSUPPORTED` when either
/// shape is of another type (a world with such a shape must use the full table).
#[inline(always)]
pub fn contact_manifold_step_basic(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold,
) -> bool {
    match (shape1, shape2) {
        (
            Shape::Ball(ball1), Shape::Ball(ball2),
        ) => {
            contact_manifold_ball_ball(pos12, ball1, ball2, prediction, ref manifold);
            true
        },
        (
            Shape::Cuboid(cuboid1), Shape::Cuboid(cuboid2),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                cuboid_cuboid_fresh(pos12, cuboid1, cuboid2, prediction, ref manifold);
            }
            true
        },
        (Shape::Ball(ball1), Shape::Cuboid(_)) | (Shape::Ball(ball1), Shape::ConvexPolygon(_)) |
        (
            Shape::Ball(ball1), Shape::HalfSpace(_),
        ) => {
            contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
            true
        },
        (Shape::Cuboid(_), Shape::Ball(ball2)) | (Shape::ConvexPolygon(_), Shape::Ball(ball2)) |
        (
            Shape::HalfSpace(_), Shape::Ball(ball2),
        ) => {
            contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
            true
        },
        (Shape::HalfSpace(halfspace1), Shape::ConvexPolygon(_)) |
        (
            Shape::HalfSpace(halfspace1), Shape::Cuboid(_),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, halfspace1, shape2, prediction, ref manifold, false,
            );
            true
        },
        (Shape::ConvexPolygon(_), Shape::HalfSpace(halfspace2)) |
        (
            Shape::Cuboid(_), Shape::HalfSpace(halfspace2),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12.inverse(), halfspace2, shape1, prediction, ref manifold, true,
            );
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::ConvexPolygon(b),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                polygon_polygon_fresh(pos12, a.unbox(), b.unbox(), prediction, ref manifold);
            }
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Cuboid(b),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                polygon_cuboid_fresh(pos12, a.unbox(), b, prediction, false, ref manifold);
            }
            true
        },
        (
            Shape::Cuboid(b), Shape::ConvexPolygon(a),
        ) => {
            contact_manifold_polygon_cuboid(
                pos12.inverse(), a.unbox(), b, prediction, true, ref manifold,
            );
            true
        },
        (Shape::HalfSpace(_), Shape::HalfSpace(_)) => {
            manifold.clear();
            false
        },
        _ => core::panic_with_felt252(errors::UNSUPPORTED),
    }
}

#[cfg(test)]
mod tests;
