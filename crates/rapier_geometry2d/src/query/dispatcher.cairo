//! Local-frame query tables (Parry `DefaultQueryDispatcher::{distance, closest_points,
//! contact}`), in upstream's priority order. `pos12` is the pose of `shape2` in the frame of
//! `shape1` (unit rotation); `None` is upstream's `Err(Unsupported)`, i.e. the
//! half-space–half-space pair.
//!
//! | query | ball–ball | ball–other | cuboid–cuboid | segment–segment | half-space–other
//! | other |
//! |---|---|---|---|---|---|---|
//! | `distance` | `ball` | `ball` (projection) | `cuboid` (SAT) | `segment` | `halfspace` |
//! `support_map` |
//! | `closest_points` | `ball` | `ball` | `cuboid` (SAT) | `segment` | `halfspace` | `support_map`
//! |
//! | `contact` | `ball` | `ball`, after the half-spaces | `support_map` | `support_map` |
//! `halfspace` | `support_map` |
//!
//! Upstream sends cuboid–cuboid `closest_points` to GJK; its own exact SAT witness
//! (`closest_points_cuboid_cuboid`, the one behind `distance_cuboid_cuboid`) is four times
//! cheaper than [`super::support_map`] here (`query::cuboid::tests::gas_*`) and agrees with it.
//! Its `contact_cuboid_cuboid` is not used for `contact`, upstream nor here: a support point
//! projected on the nearest face of a rotated box can miss the penetration
//! (`cuboid_cuboid/overlapping` of the golden family answers `None`).
//!
//! Every table is `#[inline(always)]` (the convention of `crate::dispatch`): inlined into its
//! caller, each arm is charged only when taken.

use fixed::Fixed;
use rapier_math::pose2::Pose2;
use crate::shape::Shape;
use super::ball::{
    closest_points_ball_ball, closest_points_ball_convex_polyhedron,
    closest_points_convex_polyhedron_ball, contact_ball_ball, contact_ball_convex_polyhedron,
    contact_convex_polyhedron_ball, distance_ball_ball, distance_ball_convex_polyhedron,
    distance_convex_polyhedron_ball,
};
use super::cuboid::{closest_points_cuboid_cuboid, distance_cuboid_cuboid};
use super::halfspace::{
    closest_points_halfspace_support_map, closest_points_support_map_halfspace,
    contact_halfspace_support_map, contact_support_map_halfspace, distance_halfspace_support_map,
    distance_support_map_halfspace,
};
use super::segment::{closest_points_segment_segment, distance_segment_segment};
use super::support_map::{
    closest_points_support_map_support_map, contact_support_map_support_map,
    distance_support_map_support_map,
};
use super::{ClosestPoints, Contact};

/// Distance between `shape1` and `shape2` placed at `pos12`, zero when they touch or overlap
/// (upstream `DefaultQueryDispatcher::distance`).
/// #### Panics
/// * The panics of the selected kernel.
#[inline(always)]
pub fn distance(pos12: Pose2, shape1: Shape, shape2: Shape) -> Option<Fixed> {
    match (shape1, shape2) {
        (Shape::Ball(b1), Shape::Ball(b2)) => Some(distance_ball_ball(b1, pos12.translation, b2)),
        (Shape::Ball(b1), _) => Some(distance_ball_convex_polyhedron(pos12, b1, shape2)),
        (_, Shape::Ball(b2)) => Some(distance_convex_polyhedron_ball(pos12, shape1, b2)),
        (Shape::Cuboid(c1), Shape::Cuboid(c2)) => Some(distance_cuboid_cuboid(pos12, c1, c2)),
        (Shape::Segment(s1), Shape::Segment(s2)) => Some(distance_segment_segment(pos12, s1, s2)),
        (Shape::HalfSpace(_), Shape::HalfSpace(_)) => None,
        (Shape::HalfSpace(h), _) => Some(distance_halfspace_support_map(pos12, h, shape2)),
        (_, Shape::HalfSpace(h)) => Some(distance_support_map_halfspace(pos12, shape1, h)),
        _ => Some(distance_support_map_support_map(pos12, shape1, shape2)),
    }
}

/// Closest points of `shape1` and `shape2` placed at `pos12` within `max_dist`, each point in
/// its shape's frame (upstream `DefaultQueryDispatcher::closest_points`).
/// #### Panics
/// * The panics of the selected kernel; `'Query: negative margin'` for `max_dist < 0` on the
///   ball–ball and half-space pairs.
#[inline(always)]
pub fn closest_points(
    pos12: Pose2, shape1: Shape, shape2: Shape, max_dist: Fixed,
) -> Option<ClosestPoints> {
    match (shape1, shape2) {
        (
            Shape::Ball(b1), Shape::Ball(b2),
        ) => Some(closest_points_ball_ball(pos12, b1, b2, max_dist)),
        (
            Shape::Ball(b1), _,
        ) => Some(closest_points_ball_convex_polyhedron(pos12, b1, shape2, max_dist)),
        (
            _, Shape::Ball(b2),
        ) => Some(closest_points_convex_polyhedron_ball(pos12, shape1, b2, max_dist)),
        (
            Shape::Cuboid(c1), Shape::Cuboid(c2),
        ) => Some(closest_points_cuboid_cuboid(pos12, c1, c2, max_dist)),
        (
            Shape::Segment(s1), Shape::Segment(s2),
        ) => Some(closest_points_segment_segment(pos12, s1, s2, max_dist)),
        (Shape::HalfSpace(_), Shape::HalfSpace(_)) => None,
        (
            Shape::HalfSpace(h), _,
        ) => Some(closest_points_halfspace_support_map(pos12, h, shape2, max_dist)),
        (
            _, Shape::HalfSpace(h),
        ) => Some(closest_points_support_map_halfspace(pos12, shape1, h, max_dist)),
        _ => Some(closest_points_support_map_support_map(pos12, shape1, shape2, max_dist)),
    }
}

/// Contact between `shape1` and `shape2` placed at `pos12` when their signed distance is at most
/// `prediction`, each side in its shape's frame (upstream `DefaultQueryDispatcher::contact`).
/// #### Panics
/// * The panics of the selected kernel.
#[inline(always)]
pub fn contact(
    pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed,
) -> Option<Option<Contact>> {
    match (shape1, shape2) {
        (Shape::Ball(b1), Shape::Ball(b2)) => Some(contact_ball_ball(pos12, b1, b2, prediction)),
        (Shape::HalfSpace(_), Shape::HalfSpace(_)) => None,
        (
            Shape::HalfSpace(h), _,
        ) => Some(contact_halfspace_support_map(pos12, h, shape2, prediction)),
        (
            _, Shape::HalfSpace(h),
        ) => Some(contact_support_map_halfspace(pos12, shape1, h, prediction)),
        (Shape::Ball(b1), _) => Some(contact_ball_convex_polyhedron(pos12, b1, shape2, prediction)),
        (_, Shape::Ball(b2)) => Some(contact_convex_polyhedron_ball(pos12, shape1, b2, prediction)),
        _ => Some(contact_support_map_support_map(pos12, shape1, shape2, prediction)),
    }
}

/// Parry's `QueryDispatcher` trait: the four pair queries behind one value, for code generic
/// over its dispatcher. `self` carries no state for [`DefaultQueryDispatcher`].
pub trait QueryDispatcher<T> {
    /// See `crate::dispatch::intersection_test`.
    fn intersection_test(self: @T, pos12: Pose2, shape1: Shape, shape2: Shape) -> Option<bool>;
    /// See [`distance`].
    fn distance(self: @T, pos12: Pose2, shape1: Shape, shape2: Shape) -> Option<Fixed>;
    /// See [`contact`].
    fn contact(
        self: @T, pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed,
    ) -> Option<Option<Contact>>;
    /// See [`closest_points`].
    fn closest_points(
        self: @T, pos12: Pose2, shape1: Shape, shape2: Shape, max_dist: Fixed,
    ) -> Option<ClosestPoints>;
}

/// Parry's `DefaultQueryDispatcher`: the tables of this module.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct DefaultQueryDispatcher {}

pub impl DefaultQueryDispatcherImpl of QueryDispatcher<DefaultQueryDispatcher> {
    #[inline(always)]
    fn intersection_test(
        self: @DefaultQueryDispatcher, pos12: Pose2, shape1: Shape, shape2: Shape,
    ) -> Option<bool> {
        crate::dispatch::intersection_test(pos12, shape1, shape2)
    }
    #[inline(always)]
    fn distance(
        self: @DefaultQueryDispatcher, pos12: Pose2, shape1: Shape, shape2: Shape,
    ) -> Option<Fixed> {
        distance(pos12, shape1, shape2)
    }
    #[inline(always)]
    fn contact(
        self: @DefaultQueryDispatcher,
        pos12: Pose2,
        shape1: Shape,
        shape2: Shape,
        prediction: Fixed,
    ) -> Option<Option<Contact>> {
        contact(pos12, shape1, shape2, prediction)
    }
    #[inline(always)]
    fn closest_points(
        self: @DefaultQueryDispatcher, pos12: Pose2, shape1: Shape, shape2: Shape, max_dist: Fixed,
    ) -> Option<ClosestPoints> {
        closest_points(pos12, shape1, shape2, max_dist)
    }
}

#[cfg(test)]
mod tests {
    use fixed::{FixedTrait, HALF, ONE, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::Pose2Trait;
    use rapier_math::rot2::Rot2;
    use crate::shape::{BallTrait, CuboidTrait, HalfSpaceTrait, Shape};
    use super::{DefaultQueryDispatcher, QueryDispatcher};

    /// The trait forwards to the tables; the half-space pair is unsupported everywhere.
    #[test]
    fn test_default_dispatcher_forwards() {
        let d: DefaultQueryDispatcher = Default::default();
        let pos12 = Pose2Trait::new(
            Vec2 { x: FixedTrait::from_int(3), y: ZERO }, Rot2 { re: ONE, im: ZERO },
        );
        let (ball, cuboid) = (
            Shape::Ball(BallTrait::new(HALF)),
            Shape::Cuboid(CuboidTrait::new(Vec2 { x: ONE, y: ONE })),
        );
        assert_eq!(d.distance(pos12, ball, cuboid), super::distance(pos12, ball, cuboid));
        assert_eq!(d.distance(pos12, ball, cuboid), Some(ONE + HALF));
        assert_eq!(d.intersection_test(pos12, ball, cuboid), Some(false));
        assert_eq!(d.contact(pos12, ball, cuboid, ONE), Some(None));
        assert_eq!(
            d.closest_points(pos12, ball, cuboid, ONE),
            super::closest_points(pos12, ball, cuboid, ONE),
        );
        let hs = Shape::HalfSpace(HalfSpaceTrait::new(Vec2 { x: ZERO, y: ONE }));
        assert!(d.distance(pos12, hs, hs).is_none());
        assert!(d.contact(pos12, hs, hs, ONE).is_none());
        assert!(d.closest_points(pos12, hs, hs, ONE).is_none());
        assert!(d.intersection_test(pos12, hs, hs).is_none());
    }
}
