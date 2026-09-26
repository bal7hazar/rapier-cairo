//! Half-space kernels of the shape-pair queries (Parry `distance_halfspace_support_map.rs`,
//! `closest_points_halfspace_support_map.rs`, `contact_halfspace_support_map.rs`).
//!
//! All three read the deepest point of the other shape along `-normal` (its support point).
//! That point is taken in the other shape's own frame and moved once, so the reported `point2`
//! is the exact support point instead of upstream's round trip
//! `pos12.inverse_transform_point(pos12 * support)`.

use fixed::wide::dot2;
use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::shape::{HalfSpace, Shape};
use super::support_map::local_support_point_toward;
use super::{ClosestPoints, ClosestPointsTrait, Contact, ContactTrait};

/// `(support point of other in its frame, the same point in the half-space frame)`.
#[inline(always)]
fn deepest(pos12: Pose2, halfspace: HalfSpace, other: Shape) -> (Vec2, Vec2) {
    let local = local_support_point_toward(other, pos12.rotation.inverse_rotate(-halfspace.normal));
    (local, pos12.transform_point(local))
}

#[inline(always)]
fn dot(a: Vec2, b: Vec2) -> Fixed {
    dot2(a.x, b.x, a.y, b.y)
}

/// Distance between a half-space and a support-map shape placed at `pos12`: `max(0, n .
/// deepest)` (upstream `distance_halfspace_support_map`).
/// #### Panics
/// * `'Query: not a support map'` when `other` is a half-space; the transform's overflow panics.
pub fn distance_halfspace_support_map(pos12: Pose2, halfspace: HalfSpace, other: Shape) -> Fixed {
    let (_, deepest) = deepest(pos12, halfspace, other);
    let d = dot(halfspace.normal, deepest);
    if d > ZERO {
        d
    } else {
        ZERO
    }
}

/// [`distance_halfspace_support_map`] with the shapes in the other order (upstream
/// `distance_support_map_halfspace`).
/// #### Panics
/// * See [`distance_halfspace_support_map`] and `Pose2::inverse`.
pub fn distance_support_map_halfspace(pos12: Pose2, other: Shape, halfspace: HalfSpace) -> Fixed {
    distance_halfspace_support_map(pos12.inverse(), halfspace, other)
}

/// Closest points of a half-space and a support-map shape within `margin` (upstream
/// `closest_points_halfspace_support_map`): `Intersecting` when the deepest point is on or below
/// the boundary, `WithinMargin(deepest projected on the boundary, deepest)` when it is within
/// `margin` above it, `Disjoint` otherwise.
/// #### Panics
/// * `'Query: negative margin'` when `margin < 0`; see [`distance_halfspace_support_map`].
pub fn closest_points_halfspace_support_map(
    pos12: Pose2, halfspace: HalfSpace, other: Shape, margin: Fixed,
) -> ClosestPoints {
    assert(margin >= ZERO, super::errors::NEGATIVE_MARGIN);
    let (local, deepest) = deepest(pos12, halfspace, other);
    let height = dot(halfspace.normal, deepest);
    if height > margin {
        ClosestPoints::Disjoint
    } else if height <= ZERO {
        ClosestPoints::Intersecting
    } else {
        ClosestPoints::WithinMargin((deepest - halfspace.normal.mul_scalar(height), local))
    }
}

/// [`closest_points_halfspace_support_map`] with the shapes in the other order, flipped
/// (upstream `closest_points_support_map_halfspace`).
/// #### Panics
/// * See [`closest_points_halfspace_support_map`] and `Pose2::inverse`.
pub fn closest_points_support_map_halfspace(
    pos12: Pose2, other: Shape, halfspace: HalfSpace, margin: Fixed,
) -> ClosestPoints {
    closest_points_halfspace_support_map(pos12.inverse(), halfspace, other, margin).flipped()
}

/// Contact between a half-space and a support-map shape when the deepest point is at most
/// `prediction` above the boundary (upstream `contact_halfspace_support_map`): `normal1` is the
/// half-space normal, `dist` the signed height of the deepest point.
/// #### Panics
/// * See [`distance_halfspace_support_map`].
pub fn contact_halfspace_support_map(
    pos12: Pose2, halfspace: HalfSpace, other: Shape, prediction: Fixed,
) -> Option<Contact> {
    let (local, deepest) = deepest(pos12, halfspace, other);
    let height = dot(halfspace.normal, deepest);
    if height > prediction {
        return None;
    }
    let normal2 = pos12.rotation.inverse_rotate(-halfspace.normal);
    Some(
        ContactTrait::new(
            deepest - halfspace.normal.mul_scalar(height), local, halfspace.normal, normal2, height,
        ),
    )
}

/// [`contact_halfspace_support_map`] with the shapes in the other order, flipped (upstream
/// `contact_support_map_halfspace`).
/// #### Panics
/// * See [`contact_halfspace_support_map`] and `Pose2::inverse`.
/// #### Deviations
/// * `pos12` is inverted before the half-space kernel runs; upstream hands it over as is (see
///   the module documentation of [`crate::query`]).
pub fn contact_support_map_halfspace(
    pos12: Pose2, other: Shape, halfspace: HalfSpace, prediction: Fixed,
) -> Option<Contact> {
    let c = contact_halfspace_support_map(pos12.inverse(), halfspace, other, prediction)?;
    Some(c.flipped())
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::query::ClosestPoints;
    use crate::shape::{BallTrait, CuboidTrait, HalfSpace, HalfSpaceTrait, Shape};
    use super::{
        closest_points_halfspace_support_map, closest_points_support_map_halfspace,
        contact_halfspace_support_map, contact_support_map_halfspace,
        distance_halfspace_support_map, distance_support_map_halfspace,
    };

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    fn at(x: Fixed, y: Fixed) -> Pose2 {
        Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
    }

    fn up() -> HalfSpace {
        HalfSpaceTrait::new(v(ZERO, ONE))
    }

    fn cuboid() -> Shape {
        Shape::Cuboid(CuboidTrait::new(v(ONE, HALF)))
    }

    /// `(height of the cuboid centre, distance, closest points at margin 1, contact dist)`.
    #[test]
    fn test_halfspace_cuboid_table() {
        let cases: Span<(Fixed, Fixed, ClosestPoints, Option<Fixed>)> = array![
            (int(3), TWO + HALF, ClosestPoints::Disjoint, None),
            (ONE, HALF, ClosestPoints::WithinMargin((v(ONE, ZERO), v(ONE, -HALF))), None),
            (HALF, ZERO, ClosestPoints::Intersecting, Some(ZERO)),
            (-ONE, ZERO, ClosestPoints::Intersecting, Some(-int(3) / int(2))),
        ]
            .span();
        for (y, distance, closest, dist) in cases {
            let pos12 = at(ZERO, *y);
            assert_eq!(distance_halfspace_support_map(pos12, up(), cuboid()), *distance);
            assert_eq!(distance_support_map_halfspace(pos12.inverse(), cuboid(), up()), *distance);
            assert_eq!(closest_points_halfspace_support_map(pos12, up(), cuboid(), ONE), *closest);
            let got = contact_halfspace_support_map(pos12, up(), cuboid(), ZERO);
            match dist {
                Some(d) => assert_eq!(got.unwrap().dist, *d),
                None => assert!(got.is_none()),
            }
        }
    }

    /// Regression for upstream's `contact_support_map_halfspace`, which does not invert `pos12`:
    /// a ball one unit above a half-space placed second has no contact (upstream answers -1.5).
    #[test]
    fn test_support_map_halfspace_inverts_the_pose() {
        let ball = Shape::Ball(BallTrait::new(HALF));
        let pos12 = at(ZERO, -ONE);
        assert!(contact_support_map_halfspace(pos12, ball, up(), ZERO).is_none());
        let c = contact_support_map_halfspace(pos12, ball, up(), ONE).unwrap();
        assert_eq!(
            (c.point1, c.point2, c.normal1, c.normal2, c.dist),
            (v(ZERO, -HALF), v(ZERO, ZERO), v(ZERO, -ONE), v(ZERO, ONE), HALF),
        );
        // Below the boundary: penetrating by 2.5.
        let c = contact_support_map_halfspace(at(ZERO, TWO), ball, up(), ZERO).unwrap();
        assert_eq!(c.dist, -int(5) / int(2));
        assert_eq!(
            closest_points_support_map_halfspace(pos12, ball, up(), ONE),
            ClosestPoints::WithinMargin((v(ZERO, -HALF), v(ZERO, ZERO))),
        );
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_distance_halfspace_support_map_cuboid() {
        let _ = distance_halfspace_support_map(
            opaque(at(ONE, TWO)), opaque(up()), opaque(cuboid()),
        );
    }

    #[test]
    fn gas_closest_points_halfspace_support_map_cuboid() {
        let _ = closest_points_halfspace_support_map(
            opaque(at(ONE, ONE)), opaque(up()), opaque(cuboid()), opaque(ONE),
        );
    }

    #[test]
    fn gas_contact_halfspace_support_map_cuboid() {
        let _ = contact_halfspace_support_map(
            opaque(at(ONE, ZERO)), opaque(up()), opaque(cuboid()), opaque(ZERO),
        );
    }

    #[test]
    fn gas_contact_support_map_halfspace_cuboid() {
        let _ = contact_support_map_halfspace(
            opaque(at(ONE, ZERO)), opaque(cuboid()), opaque(up()), opaque(ZERO),
        );
    }
}
