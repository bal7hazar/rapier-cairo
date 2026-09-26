//! Ball kernels of the shape-pair queries (Parry `distance_ball_ball.rs`,
//! `closest_points_ball_ball.rs`, `contact_ball_ball.rs` and the `*_ball_convex_polyhedron.rs`
//! files). `pos12` is the pose of shape 2 in the frame of shape 1.
//!
//! Every threshold is decided on exact wide squares (`rapier_math::math_ext::norm2`): the ball–
//! ball pair never takes a square root to decide, only to report a distance. The ball–convex
//! kernels project the centre with the exact per-shape projections of [`crate::point`].

use fixed::wide::norm2;
use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::consts::DEFAULT_EPSILON;
use rapier_math::math_ext::norm2::{is_norm2_le, is_norm2_lt};
use rapier_math::math_ext::vec2::try_normalize2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::feature_id::{FeatureId, FeatureIdTrait};
use crate::point::{PointProjection, PointQuery};
use crate::shape::{Ball, ConvexPolygon, ConvexPolygonTrait, CuboidTrait, SegmentTrait, Shape};
use super::{ClosestPoints, Contact, ContactTrait, X, Y, normalize_and_length};

/// Distance between two balls, the second centred at `center2` (frame of the first); zero when
/// they touch or overlap.
///
/// Mirrors `distance_ball_ball`: zero when `|center2|^2 <= (r1 + r2)^2` (compared wide),
/// `|center2| - r1 - r2` otherwise (floored norm).
/// #### Panics
/// * `'Fixed: overflow'` if `r1 + r2` or `|center2|` leaves the scalar range.
pub fn distance_ball_ball(b1: Ball, center2: Vec2, b2: Ball) -> Fixed {
    let sum = b1.radius + b2.radius;
    if is_norm2_le(center2.x, center2.y, sum) {
        ZERO
    } else {
        norm2(center2.x, center2.y) - sum
    }
}

/// Closest points of two balls within `margin`.
///
/// Mirrors `closest_points_ball_ball`: `Intersecting` when `|t| <= r1 + r2`, `WithinMargin`
/// when `|t| <= r1 + r2 + margin` (both compared wide), `Disjoint` otherwise; `p1 = n r1` and
/// `p2 = -R^-1 n r2` with `n = t / |t|`.
/// #### Panics
/// * `'Query: negative margin'` when `margin < 0` (upstream asserts it too).
/// * `'Fixed: overflow'` if a radius sum leaves the scalar range.
pub fn closest_points_ball_ball(pos12: Pose2, b1: Ball, b2: Ball, margin: Fixed) -> ClosestPoints {
    assert(margin >= ZERO, super::errors::NEGATIVE_MARGIN);
    let t = pos12.translation;
    let sum = b1.radius + b2.radius;
    if !is_norm2_le(t.x, t.y, sum + margin) {
        return ClosestPoints::Disjoint;
    }
    if is_norm2_le(t.x, t.y, sum) {
        return ClosestPoints::Intersecting;
    }
    // `|t| > r1 + r2 >= 0`: the direction exists.
    let (dir, _) = normalize_and_length(t);
    let n = dir.unwrap_or(X);
    let p2 = -pos12.rotation.inverse_rotate(n).mul_scalar(b2.radius);
    ClosestPoints::WithinMargin((n.mul_scalar(b1.radius), p2))
}

/// Contact between two balls when `|t| < r1 + r2 + prediction` (strict, compared wide).
///
/// Mirrors `contact_ball_ball`: `normal1 = t / |t|` (`+X` for concentric balls), `normal2 =
/// -R^-1 normal1`, the points on each surface, `dist = |t| - r1 - r2`.
/// #### Panics
/// * `'Fixed: overflow'` if `r1 + r2 + prediction` or `|t|` leaves the scalar range.
pub fn contact_ball_ball(pos12: Pose2, b1: Ball, b2: Ball, prediction: Fixed) -> Option<Contact> {
    let t = pos12.translation;
    let sum = b1.radius + b2.radius;
    let limit = sum + prediction;
    if limit < ZERO || !is_norm2_lt(t.x, t.y, limit) {
        return None;
    }
    let (dir, len) = normalize_and_length(t);
    let normal1 = dir.unwrap_or(X);
    let normal2 = -pos12.rotation.inverse_rotate(normal1);
    Some(
        ContactTrait::new(
            normal1.mul_scalar(b1.radius),
            normal2.mul_scalar(b2.radius),
            normal1,
            normal2,
            len - sum,
        ),
    )
}

/// Upstream `Shape::feature_normal_at_point` for the closed set: the ball's radial direction,
/// the cuboid / segment / polygon feature normal, `None` for a capsule or a half-space.
fn feature_normal_at_point(shape: Shape, feature: FeatureId, point: Vec2) -> Option<Vec2> {
    match shape {
        Shape::Ball(_) => match try_normalize2(point.x, point.y) {
            Some((x, y)) => Some(Vec2 { x, y }),
            None => None,
        },
        Shape::Cuboid(c) => c.feature_normal(feature),
        Shape::Segment(s) => s.feature_normal(feature),
        Shape::ConvexPolygon(p) => polygon_feature_normal(p.unbox(), feature),
        _ => None,
    }
}

/// `ConvexPolygon::feature_normal`: the face normal, or the normalised sum of the two normals
/// around a vertex.
fn polygon_feature_normal(polygon: ConvexPolygon, feature: FeatureId) -> Option<Vec2> {
    let code: u8 = feature.code().try_into().unwrap();
    if feature.is_face() {
        Some(polygon.normal(code))
    } else if feature.is_vertex() {
        let previous = if code == 0 {
            polygon.count - 1
        } else {
            code - 1
        };
        let sum = polygon.normal(previous) + polygon.normal(code);
        match try_normalize2(sum.x, sum.y) {
            Some((x, y)) => Some(Vec2 { x, y }),
            None => None,
        }
    } else {
        None
    }
}

/// The normal of a centre on the surface: the feature normal, else the projection's direction
/// from the origin, else `+Y` (upstream's chain). Cold path.
#[inline(never)]
fn surface_normal(shape: Shape, feature: FeatureId, point: Vec2) -> Vec2 {
    if let Some(n) = feature_normal_at_point(shape, feature, point) {
        return n;
    }
    match try_normalize2(point.x, point.y) {
        Some((x, y)) => Vec2 { x, y },
        None => Y,
    }
}

/// Contact between a convex `shape1` and `ball2` (centre at `pos12.translation`) when the signed
/// distance is at most `prediction`.
///
/// Mirrors `contact_convex_polyhedron_ball`: the centre is projected on the boundary of
/// `shape1`; `dist = ±|proj - centre| - r` (negative inside) and the normal points from the
/// surface to the centre, or is the feature normal when the centre is within `DEFAULT_EPSILON`
/// of the surface.
/// #### Panics
/// * The panics of the projection on `shape1` (see [`crate::point`]).
pub fn contact_convex_polyhedron_ball(
    pos12: Pose2, shape1: Shape, ball2: Ball, prediction: Fixed,
) -> Option<Contact> {
    let center = pos12.translation;
    let (proj, feature): (PointProjection, FeatureId) = shape1
        .project_local_point_and_get_feature(center);
    let (dir, len) = normalize_and_length(proj.point - center);
    let (dist, normal1) = match dir {
        Some(d) => if len < DEFAULT_EPSILON {
            (-ball2.radius, surface_normal(shape1, feature, proj.point))
        } else if proj.is_inside {
            (-len - ball2.radius, d)
        } else {
            (len - ball2.radius, -d)
        },
        None => (-ball2.radius, surface_normal(shape1, feature, proj.point)),
    };
    if dist > prediction {
        return None;
    }
    let normal2 = pos12.rotation.inverse_rotate(-normal1);
    Some(ContactTrait::new(proj.point, normal2.mul_scalar(ball2.radius), normal1, normal2, dist))
}

/// Contact between `ball1` and a convex `shape2`: [`contact_convex_polyhedron_ball`] on the
/// inverted pose, flipped (upstream `contact_ball_convex_polyhedron`).
/// #### Panics
/// * See [`contact_convex_polyhedron_ball`] and `Pose2::inverse`.
pub fn contact_ball_convex_polyhedron(
    pos12: Pose2, ball1: Ball, shape2: Shape, prediction: Fixed,
) -> Option<Contact> {
    let c = contact_convex_polyhedron_ball(pos12.inverse(), shape2, ball1, prediction)?;
    Some(c.flipped())
}

/// Closest points of a convex `shape1` and `ball2` within `margin`, from the contact at
/// prediction `margin` (upstream `closest_points_convex_polyhedron_ball`): `Intersecting` for
/// `dist <= 0`.
/// #### Panics
/// * See [`contact_convex_polyhedron_ball`].
pub fn closest_points_convex_polyhedron_ball(
    pos12: Pose2, shape1: Shape, ball2: Ball, margin: Fixed,
) -> ClosestPoints {
    from_contact(contact_convex_polyhedron_ball(pos12, shape1, ball2, margin))
}

/// Closest points of `ball1` and a convex `shape2` within `margin` (upstream
/// `closest_points_ball_convex_polyhedron`).
/// #### Panics
/// * See [`contact_ball_convex_polyhedron`].
pub fn closest_points_ball_convex_polyhedron(
    pos12: Pose2, ball1: Ball, shape2: Shape, margin: Fixed,
) -> ClosestPoints {
    from_contact(contact_ball_convex_polyhedron(pos12, ball1, shape2, margin))
}

#[inline(always)]
fn from_contact(contact: Option<Contact>) -> ClosestPoints {
    match contact {
        Some(c) => if c.dist <= ZERO {
            ClosestPoints::Intersecting
        } else {
            ClosestPoints::WithinMargin((c.point1, c.point2))
        },
        None => ClosestPoints::Disjoint,
    }
}

/// Distance between a convex `shape1` and `ball2`: `max(0, |solid projection - centre| - r)`
/// (upstream `distance_convex_polyhedron_ball`).
/// #### Panics
/// * The panics of the projection on `shape1`.
pub fn distance_convex_polyhedron_ball(pos12: Pose2, shape1: Shape, ball2: Ball) -> Fixed {
    let center = pos12.translation;
    let proj = shape1.project_local_point(center, true);
    let d = proj.point - center;
    let dist = norm2(d.x, d.y) - ball2.radius;
    if dist > ZERO {
        dist
    } else {
        ZERO
    }
}

/// Distance between `ball1` and a convex `shape2` (upstream `distance_ball_convex_polyhedron`).
/// #### Panics
/// * See [`distance_convex_polyhedron_ball`] and `Pose2::inverse`.
pub fn distance_ball_convex_polyhedron(pos12: Pose2, ball1: Ball, shape2: Shape) -> Fixed {
    distance_convex_polyhedron_ball(pos12.inverse(), shape2, ball1)
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::query::ClosestPoints;
    use crate::shape::{Ball, BallTrait, CuboidTrait, SegmentTrait, Shape};
    use super::{
        closest_points_ball_ball, closest_points_ball_convex_polyhedron,
        closest_points_convex_polyhedron_ball, contact_ball_ball, contact_ball_convex_polyhedron,
        contact_convex_polyhedron_ball, distance_ball_ball, distance_ball_convex_polyhedron,
        distance_convex_polyhedron_ball,
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

    fn quarter_at(x: Fixed, y: Fixed) -> Pose2 {
        Pose2Trait::new(v(x, y), Rot2 { re: ZERO, im: ONE })
    }

    fn ball() -> Ball {
        BallTrait::new(HALF)
    }

    /// `(center2, distance, closest points at margin 1, contact dist at prediction 0)`.
    #[test]
    fn test_ball_ball_table() {
        let cases: Span<(Fixed, Fixed, ClosestPoints, Option<Fixed>)> = array![
            (int(3), TWO, ClosestPoints::Disjoint, None),
            (TWO, ONE, ClosestPoints::WithinMargin((v(HALF, ZERO), v(-HALF, ZERO))), None),
            // Touching: intersecting, but no contact at prediction 0 (strict `<`, as upstream).
            (ONE, ZERO, ClosestPoints::Intersecting, None),
            (HALF, ZERO, ClosestPoints::Intersecting, Some(-HALF)),
        ]
            .span();
        for (x, distance, closest, dist) in cases {
            let pos12 = at(*x, ZERO);
            assert_eq!(distance_ball_ball(ball(), pos12.translation, ball()), *distance);
            assert_eq!(closest_points_ball_ball(pos12, ball(), ball(), ONE), *closest);
            let got = contact_ball_ball(pos12, ball(), ball(), ZERO);
            match dist {
                Some(d) => assert_eq!(got.unwrap().dist, *d),
                None => assert!(got.is_none()),
            }
        }
        // Concentric balls: the `+X` fallback normal.
        let c = contact_ball_ball(at(ZERO, ZERO), ball(), ball(), ZERO).unwrap();
        assert_eq!((c.normal1, c.normal2, c.dist), (v(ONE, ZERO), v(-ONE, ZERO), -ONE));
        // The second ball's side is in its own (rotated) frame.
        let c = contact_ball_ball(quarter_at(HALF, ZERO), ball(), ball(), ZERO).unwrap();
        assert_eq!((c.normal2, c.point2), (v(ZERO, ONE), v(ZERO, HALF)));
    }

    #[test]
    #[should_panic(expected: 'Query: negative margin')]
    fn test_negative_margin_panics() {
        let _ = closest_points_ball_ball(at(int(3), ZERO), ball(), ball(), -ONE);
    }

    /// A ball against a cuboid, both orders: distance, closest points and contact agree.
    #[test]
    fn test_ball_convex_both_orders() {
        let cuboid = Shape::Cuboid(CuboidTrait::new(v(ONE, HALF)));
        let pos12 = at(ZERO, TWO);
        assert_eq!(distance_convex_polyhedron_ball(pos12, cuboid, ball()), ONE);
        assert_eq!(distance_ball_convex_polyhedron(pos12.inverse(), ball(), cuboid), ONE);
        let c = contact_convex_polyhedron_ball(pos12, cuboid, ball(), ONE).unwrap();
        assert_eq!(
            (c.point1, c.point2, c.normal1, c.dist),
            (v(ZERO, HALF), v(ZERO, -HALF), v(ZERO, ONE), ONE),
        );
        let f = contact_ball_convex_polyhedron(pos12.inverse(), ball(), cuboid, ONE).unwrap();
        assert_eq!(
            (f.point1, f.point2, f.normal2, f.dist), (c.point2, c.point1, c.normal1, c.dist),
        );
        assert_eq!(
            closest_points_convex_polyhedron_ball(pos12, cuboid, ball(), HALF),
            ClosestPoints::Disjoint,
        );
        assert_eq!(
            closest_points_ball_convex_polyhedron(pos12.inverse(), ball(), cuboid, ONE),
            ClosestPoints::WithinMargin((v(ZERO, -HALF), v(ZERO, HALF))),
        );
        // Centre inside: negative distance, the normal points out through the nearest face.
        let c = contact_convex_polyhedron_ball(at(ZERO, HALF / int(2)), cuboid, ball(), ZERO)
            .unwrap();
        assert_eq!((c.normal1, c.dist), (v(ZERO, ONE), -HALF / int(2) - HALF));
        // Centre on a segment: the segment's face normal.
        let segment = Shape::Segment(SegmentTrait::new(v(-ONE, ZERO), v(ONE, ZERO)));
        let c = contact_convex_polyhedron_ball(at(ZERO, ZERO), segment, ball(), ZERO).unwrap();
        assert_eq!((c.normal1, c.dist), (v(ZERO, -ONE), -HALF));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_distance_ball_ball() {
        let _ = distance_ball_ball(opaque(ball()), opaque(v(TWO, ONE)), opaque(ball()));
    }

    #[test]
    fn gas_closest_points_ball_ball() {
        let _ = closest_points_ball_ball(
            opaque(at(TWO, ONE)), opaque(ball()), opaque(ball()), opaque(TWO),
        );
    }

    #[test]
    fn gas_contact_ball_ball() {
        let _ = contact_ball_ball(
            opaque(at(HALF, HALF)), opaque(ball()), opaque(ball()), opaque(ZERO),
        );
    }

    #[test]
    fn gas_distance_convex_polyhedron_ball_cuboid() {
        let _ = distance_convex_polyhedron_ball(
            opaque(at(TWO, ONE)),
            opaque(Shape::Cuboid(CuboidTrait::new(v(ONE, HALF)))),
            opaque(ball()),
        );
    }

    #[test]
    fn gas_contact_ball_convex_polyhedron_cuboid() {
        let _ = contact_ball_convex_polyhedron(
            opaque(at(TWO, ONE)),
            opaque(ball()),
            opaque(Shape::Cuboid(CuboidTrait::new(v(ONE, HALF)))),
            opaque(ONE),
        );
    }

    #[test]
    fn gas_closest_points_convex_polyhedron_ball_cuboid() {
        let _ = closest_points_convex_polyhedron_ball(
            opaque(at(TWO, ONE)),
            opaque(Shape::Cuboid(CuboidTrait::new(v(ONE, HALF)))),
            opaque(ball()),
            opaque(TWO),
        );
    }
}
