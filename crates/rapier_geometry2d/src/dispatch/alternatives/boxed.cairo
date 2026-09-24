//! Rejected CP2 boxed-argument helpers: measured scene costs equal the direct table.
use fixed::Fixed;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::contact::{ContactManifold, ContactManifoldTrait};
use crate::contact_generators::ball_ball::contact_manifold_ball_ball;
use crate::contact_generators::capsule_capsule::contact_manifold_capsule_capsule;
use crate::contact_generators::convex_ball::{
    contact_manifold_ball_convex, contact_manifold_convex_ball,
};
use crate::contact_generators::cuboid_capsule::{
    contact_manifold_cuboid_capsule, contact_manifold_cuboid_capsule_shapes,
};
use crate::contact_generators::cuboid_cuboid::contact_manifold_cuboid_cuboid;
use crate::contact_generators::cuboid_segment::{
    contact_manifold_cuboid_segment, contact_manifold_cuboid_segment_shapes,
};
use crate::contact_generators::halfspace_pfm::contact_manifold_halfspace_pfm;
use crate::contact_generators::polygon_polygon::{
    contact_manifold_polygon_cuboid, contact_manifold_polygon_polygon,
};
use crate::contact_generators::polygon_segment::{
    contact_manifold_polygon_capsule, contact_manifold_polygon_segment,
};
use crate::manifold::ManifoldTrait;
use crate::shape::{Capsule, ConvexPolygon, Cuboid, Segment, Shape};

#[inline(always)]
pub fn contact_manifold_step_boxed(
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
                contact_manifold_cuboid_cuboid(pos12, cuboid1, cuboid2, prediction, ref manifold);
            }
            true
        },
        (
            Shape::Capsule(capsule1), Shape::Capsule(capsule2),
        ) => {
            contact_manifold_capsule_capsule(pos12, capsule1, capsule2, prediction, ref manifold);
            true
        },
        (
            Shape::Ball(ball1), _,
        ) => {
            contact_manifold_ball_convex(pos12, ball1, shape2, prediction, ref manifold);
            true
        },
        (
            _, Shape::Ball(ball2),
        ) => {
            contact_manifold_convex_ball(pos12, shape1, ball2, prediction, ref manifold);
            true
        },
        (
            Shape::Cuboid(cuboid1), Shape::Capsule(capsule2),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                contact_manifold_cuboid_capsule(pos12, cuboid1, capsule2, prediction, ref manifold);
            }
            true
        },
        (
            Shape::Capsule(_), Shape::Cuboid(_),
        ) => {
            if manifold.try_update_contacts(pos12) {
                return true;
            }
            contact_manifold_cuboid_capsule_shapes(pos12, shape1, shape2, prediction, ref manifold)
        },
        (
            Shape::Cuboid(cuboid1), Shape::Segment(segment2),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                contact_manifold_cuboid_segment(pos12, cuboid1, segment2, prediction, ref manifold);
            }
            true
        },
        (
            Shape::Segment(_), Shape::Cuboid(_),
        ) => {
            if manifold.try_update_contacts(pos12) {
                return true;
            }
            contact_manifold_cuboid_segment_shapes(pos12, shape1, shape2, prediction, ref manifold)
        },
        (Shape::HalfSpace(halfspace1), Shape::ConvexPolygon(_)) |
        (Shape::HalfSpace(halfspace1), Shape::Cuboid(_)) |
        (Shape::HalfSpace(halfspace1), Shape::Segment(_)) |
        (
            Shape::HalfSpace(halfspace1), Shape::Capsule(_),
        ) => {
            contact_manifold_halfspace_pfm(
                pos12, halfspace1, shape2, prediction, ref manifold, false,
            );
            true
        },
        (Shape::ConvexPolygon(_), Shape::HalfSpace(halfspace2)) |
        (Shape::Cuboid(_), Shape::HalfSpace(halfspace2)) |
        (Shape::Segment(_), Shape::HalfSpace(halfspace2)) |
        (
            Shape::Capsule(_), Shape::HalfSpace(halfspace2),
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
                boxed_polygon_polygon(pos12, a, b, prediction, ref manifold);
            }
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Cuboid(b),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                boxed_polygon_cuboid(pos12, a, b, prediction, false, ref manifold);
            }
            true
        },
        (
            Shape::Cuboid(b), Shape::ConvexPolygon(a),
        ) => {
            boxed_polygon_cuboid(pos12.inverse(), a, b, prediction, true, ref manifold);
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Segment(b),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                boxed_polygon_segment(pos12, a, b, prediction, false, ref manifold);
            }
            true
        },
        (
            Shape::Segment(b), Shape::ConvexPolygon(a),
        ) => {
            boxed_polygon_segment(pos12.inverse(), a, b, prediction, true, ref manifold);
            true
        },
        (
            Shape::ConvexPolygon(a), Shape::Capsule(b),
        ) => {
            if !manifold.try_update_contacts(pos12) {
                boxed_polygon_capsule(pos12, a, b, prediction, false, ref manifold);
            }
            true
        },
        (
            Shape::Capsule(b), Shape::ConvexPolygon(a),
        ) => {
            boxed_polygon_capsule(pos12.inverse(), a, b, prediction, true, ref manifold);
            true
        },
        _ => {
            manifold.clear();
            false
        },
    }
}

#[inline(never)]
fn boxed_polygon_polygon(
    p: Pose2,
    a: Box<ConvexPolygon>,
    b: Box<ConvexPolygon>,
    prediction: Fixed,
    ref m: ContactManifold,
) {
    contact_manifold_polygon_polygon(p, a.unbox(), b.unbox(), prediction, ref m);
}
#[inline(never)]
fn boxed_polygon_cuboid(
    p: Pose2,
    a: Box<ConvexPolygon>,
    b: Cuboid,
    prediction: Fixed,
    flipped: bool,
    ref m: ContactManifold,
) {
    contact_manifold_polygon_cuboid(p, a.unbox(), b, prediction, flipped, ref m);
}
#[inline(never)]
fn boxed_polygon_segment(
    p: Pose2,
    a: Box<ConvexPolygon>,
    b: Segment,
    prediction: Fixed,
    flipped: bool,
    ref m: ContactManifold,
) {
    contact_manifold_polygon_segment(p, a.unbox(), b, prediction, flipped, ref m);
}
#[inline(never)]
fn boxed_polygon_capsule(
    p: Pose2,
    a: Box<ConvexPolygon>,
    b: Capsule,
    prediction: Fixed,
    flipped: bool,
    ref m: ContactManifold,
) {
    contact_manifold_polygon_capsule(p, a.unbox(), b, prediction, flipped, ref m);
}
