//! Parry's shape-pair queries (`parry::query`) for the closed 2D shape set: distance, closest
//! points, single contact, and the AABB point / ray queries (work package QY1,
//! `docs/briefs/qy1-shape-queries.md`).
//!
//! # Entry points
//!
//! * [`distance`], [`closest_points`], [`contact`] and [`intersection_test`] take the two shapes
//!   with their world poses, as upstream's free functions do: they compute
//!   `pos12 = pos1.inv_mul(pos2)`, call the local-frame table of [`dispatcher`] (upstream's
//!   `DefaultQueryDispatcher`) and move the answer back to world space.
//! * [`dispatcher`] holds the tables, in upstream's priority order, and the per-pair kernels
//!   live in [`ball`], [`halfspace`], [`cuboid`], [`segment`] and [`support_map`] under their
//!   upstream names (`distance_ball_ball`, `contact_halfspace_support_map`, …); `pos12` is the
//!   pose of shape 2 in the frame of shape 1 there.
//!
//! # Semantics
//!
//! * Unsupported pairs (half-space–half-space, upstream `Err(Unsupported)`) answer `None`, the
//!   convention of `crate::dispatch::intersection_test`; `contact` is therefore
//!   `Option<Option<Contact>>` (unsupported / no contact within `prediction` / the contact).
//! * `distance` is `max(0, …)`: zero for touching or penetrating shapes.
//! * `closest_points` answers `Intersecting` for touching or penetrating shapes, except for
//!   segment–segment where upstream never does (it answers the crossing point twice).
//! * A contact's `dist` is signed (negative when penetrating), its normals are unit and point
//!   outward from each shape, and `point2 - point1 = dist * normal1` in world space.
//!
//! # Deviations
//!
//! * Every pair upstream sends to GJK (and EPA for a penetrating contact) is answered exactly by
//!   the analytic kernel of [`support_map`] (polygonal core plus radius, SAT for the penetration
//!   depth); the answers agree with upstream's within GJK's tolerance and EPA's approximation of a
//!   rounded shape (see `tests/shape_queries_golden.cairo` for the bands).
//! * Upstream's `contact_support_map_halfspace` does not invert `pos12` before handing it to
//!   `contact_halfspace_support_map` (parry 0.30.2 and 0.31.1), so it answers a contact for a
//!   shape above a half-space placed second. The port inverts it, as the matching `distance` and
//!   `closest_points` kernels do upstream.
//! * Composite shapes and the shape casts are deferred (lots SH2 / CC).

pub mod ball;
pub mod cuboid;
pub mod dispatcher;
pub mod halfspace;
pub mod segment;
pub mod support_map;
use fixed::wide::{NormTrait, RecipTrait, norm2_wide};
use fixed::{Fixed, ONE, ZERO};
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2Trait;
use crate::shape::Shape;

pub mod errors {
    /// A `closest_points` margin below zero (upstream asserts it).
    pub const NEGATIVE_MARGIN: felt252 = 'Query: negative margin';
    /// A support-map kernel handed a half-space.
    pub const NOT_SUPPORT_MAP: felt252 = 'Query: not a support map';
}

/// Closest points of two shapes (Parry `ClosestPoints`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum ClosestPoints {
    /// The shapes touch or overlap.
    Intersecting,
    /// The closest points, one on each shape, when they are within the margin of each other.
    WithinMargin: (Vec2, Vec2),
    /// The shapes are further apart than the margin.
    Disjoint,
}

#[generate_trait]
pub impl ClosestPointsImpl of ClosestPointsTrait {
    /// Swaps the two points of a `WithinMargin` answer (upstream `flip`).
    #[inline(always)]
    fn flip(ref self: ClosestPoints) {
        self = Self::flipped(self);
    }

    /// The answer with the two points swapped (upstream `flipped`).
    #[inline(always)]
    fn flipped(self: ClosestPoints) -> ClosestPoints {
        match self {
            ClosestPoints::WithinMargin((p1, p2)) => ClosestPoints::WithinMargin((p2, p1)),
            other => other,
        }
    }

    /// The answer with its first point moved by `pos1` and its second by `pos2` (upstream
    /// `transform_by`).
    #[inline(always)]
    fn transform_by(self: ClosestPoints, pos1: Pose2, pos2: Pose2) -> ClosestPoints {
        match self {
            ClosestPoints::WithinMargin((
                p1, p2,
            )) => ClosestPoints::WithinMargin((pos1.transform_point(p1), pos2.transform_point(p2))),
            other => other,
        }
    }
}

/// A single contact between two shapes (Parry `query::Contact`), each point and normal in the
/// frame of its own shape until moved.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Contact {
    /// The contact point on shape 1.
    pub point1: Vec2,
    /// The contact point on shape 2.
    pub point2: Vec2,
    /// Unit outward normal of shape 1 at `point1`.
    pub normal1: Vec2,
    /// Unit outward normal of shape 2 at `point2`.
    pub normal2: Vec2,
    /// Signed distance along `normal1`: negative when penetrating.
    pub dist: Fixed,
}

#[generate_trait]
pub impl ContactImpl of ContactTrait {
    /// A contact (upstream `Contact::new`).
    #[inline(always)]
    fn new(point1: Vec2, point2: Vec2, normal1: Vec2, normal2: Vec2, dist: Fixed) -> Contact {
        Contact { point1, point2, normal1, normal2, dist }
    }

    /// Swaps the two sides (upstream `flip`).
    #[inline(always)]
    fn flip(ref self: Contact) {
        self = Self::flipped(self);
    }

    /// The contact with its two sides swapped (upstream `flipped`).
    #[inline(always)]
    fn flipped(self: Contact) -> Contact {
        Contact {
            point1: self.point2,
            point2: self.point1,
            normal1: self.normal2,
            normal2: self.normal1,
            dist: self.dist,
        }
    }

    /// Moves side 1 by `pos1` and side 2 by `pos2` (upstream `transform_by_mut`).
    #[inline(always)]
    fn transform_by_mut(ref self: Contact, pos1: Pose2, pos2: Pose2) {
        self.point1 = pos1.transform_point(self.point1);
        self.point2 = pos2.transform_point(self.point2);
        self.normal1 = pos1.rotation.rotate(self.normal1);
        self.normal2 = pos2.rotation.rotate(self.normal2);
    }

    /// Moves side 1 by `pos` (upstream `transform1_by_mut`).
    #[inline(always)]
    fn transform1_by_mut(ref self: Contact, pos: Pose2) {
        self.point1 = pos.transform_point(self.point1);
        self.normal1 = pos.rotation.rotate(self.normal1);
    }
}

/// `(d / |d|, |d|)`: the direction is `None` when `d` is zero; the length is the floored wide
/// norm and the direction `d * recip(|d|)`, exact for an axis-aligned `d`.
pub(crate) fn normalize_and_length(d: Vec2) -> (Option<Vec2>, Fixed) {
    let n = norm2_wide(d.x, d.y);
    match n.try_recip() {
        Some(r) => (Some(Vec2 { x: r.mul(d.x), y: r.mul(d.y) }), n.to_fixed()),
        None => (None, ZERO),
    }
}

/// `+X`, upstream's fallback normal of two concentric balls.
pub(crate) const X: Vec2 = Vec2 { x: ONE, y: ZERO };
/// `+Y`, upstream's last-resort fallback normal.
pub(crate) const Y: Vec2 = Vec2 { x: ZERO, y: ONE };

/// Distance between `g1` at `pos1` and `g2` at `pos2`, zero when they touch or overlap.
///
/// Mirrors `parry::query::distance`. `None` for an unsupported pair (half-space–half-space).
/// #### Panics
/// * `Pose2::inv_mul` needs a unit `pos1.rotation`; see the kernels of [`dispatcher::distance`].
pub fn distance(pos1: Pose2, g1: Shape, pos2: Pose2, g2: Shape) -> Option<Fixed> {
    dispatcher::distance(pos1.inv_mul(pos2), g1, g2)
}

/// Closest points between `g1` at `pos1` and `g2` at `pos2`, in world space, when they are within
/// `max_dist` of each other.
///
/// Mirrors `parry::query::closest_points`. `None` for an unsupported pair.
/// #### Panics
/// * See [`distance`].
pub fn closest_points(
    pos1: Pose2, g1: Shape, pos2: Pose2, g2: Shape, max_dist: Fixed,
) -> Option<ClosestPoints> {
    let answer = dispatcher::closest_points(pos1.inv_mul(pos2), g1, g2, max_dist)?;
    Some(answer.transform_by(pos1, pos2))
}

/// The contact between `g1` at `pos1` and `g2` at `pos2`, in world space, when their signed
/// distance is at most `prediction`.
///
/// Mirrors `parry::query::contact`: the outer `None` is upstream's `Err(Unsupported)`, the inner
/// one "no contact within `prediction`".
/// #### Panics
/// * See [`distance`].
pub fn contact(
    pos1: Pose2, g1: Shape, pos2: Pose2, g2: Shape, prediction: Fixed,
) -> Option<Option<Contact>> {
    match dispatcher::contact(pos1.inv_mul(pos2), g1, g2, prediction)? {
        Some(c) => {
            let mut c = c;
            c.transform_by_mut(pos1, pos2);
            Some(Some(c))
        },
        None => Some(None),
    }
}

/// Whether `g1` at `pos1` and `g2` at `pos2` intersect (touching included).
///
/// Mirrors `parry::query::intersection_test` on top of `crate::dispatch::intersection_test`.
/// `None` for an unsupported pair.
/// #### Panics
/// * See `crate::dispatch::intersection_test`.
pub fn intersection_test(pos1: Pose2, g1: Shape, pos2: Pose2, g2: Shape) -> Option<bool> {
    crate::dispatch::intersection_test(pos1.inv_mul(pos2), g1, g2)
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::shape::{
        BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait,
        Shape,
    };
    use super::{
        ClosestPoints, ClosestPointsTrait, Contact, ContactTrait, closest_points, contact,
        dispatcher, distance, intersection_test,
    };

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn int(x: i32) -> Fixed {
        FixedTrait::from_int(x)
    }

    /// Quarter turn around `(1, 2)`.
    fn quarter() -> Pose2 {
        Pose2Trait::new(v(ONE, TWO), Rot2 { re: ZERO, im: ONE })
    }

    fn at(x: Fixed, y: Fixed) -> Pose2 {
        Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
    }

    fn shapes() -> Span<Shape> {
        let triangle = ConvexPolygonTrait::from_convex_polyline(
            array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span(),
        )
            .unwrap();
        array![
            Shape::Ball(BallTrait::new(HALF)), Shape::Cuboid(CuboidTrait::new(v(HALF, ONE))),
            Shape::Capsule(CapsuleTrait::new_x(HALF, HALF)),
            Shape::Segment(SegmentTrait::new(v(ZERO, -ONE), v(ZERO, ONE))),
            Shape::HalfSpace(HalfSpaceTrait::new(v(ZERO, ONE))),
            Shape::ConvexPolygon(BoxTrait::new(triangle)),
        ]
            .span()
    }

    #[test]
    fn test_closest_points_methods() {
        let (a, b) = (v(ONE, ZERO), v(ZERO, TWO));
        let mut w = ClosestPoints::WithinMargin((a, b));
        assert_eq!(w.flipped(), ClosestPoints::WithinMargin((b, a)));
        w.flip();
        assert_eq!(w, ClosestPoints::WithinMargin((b, a)));
        assert_eq!(ClosestPoints::Intersecting.flipped(), ClosestPoints::Intersecting);
        assert_eq!(
            ClosestPoints::WithinMargin((a, a)).transform_by(quarter(), at(ONE, ONE)),
            ClosestPoints::WithinMargin((v(ONE, int(3)), v(TWO, ONE))),
        );
        assert_eq!(
            ClosestPoints::Disjoint.transform_by(quarter(), quarter()), ClosestPoints::Disjoint,
        );
    }

    #[test]
    fn test_contact_methods() {
        let c = ContactTrait::new(v(ONE, ZERO), v(TWO, ZERO), v(ONE, ZERO), v(-ONE, ZERO), ONE);
        let f = c.flipped();
        assert_eq!(
            f,
            Contact {
                point1: v(TWO, ZERO),
                point2: v(ONE, ZERO),
                normal1: v(-ONE, ZERO),
                normal2: v(ONE, ZERO),
                dist: ONE,
            },
        );
        let mut g = c;
        g.flip();
        assert_eq!(g, f);
        let mut t = c;
        t.transform_by_mut(quarter(), at(ONE, ONE));
        assert_eq!(
            (t.point1, t.point2, t.normal1, t.normal2),
            (v(ONE, int(3)), v(int(3), ONE), v(ZERO, ONE), v(-ONE, ZERO)),
        );
        let mut t1 = c;
        t1.transform1_by_mut(quarter());
        assert_eq!(
            (t1.point1, t1.point2, t1.normal1), (v(ONE, int(3)), v(TWO, ZERO), v(ZERO, ONE)),
        );
    }

    /// Every pair of the closed set, both world poses non-trivial: the world queries are the
    /// local tables on `pos1.inv_mul(pos2)` with the answers moved to world space; only the
    /// half-space pair is unsupported.
    #[test]
    fn test_world_queries_match_local_tables() {
        let pos1 = quarter();
        let pos2 = at(TWO, int(3));
        let pos12 = pos1.inv_mul(pos2);
        let mut unsupported = 0;
        for s1 in shapes() {
            for s2 in shapes() {
                let local = dispatcher::distance(pos12, *s1, *s2);
                assert_eq!(distance(pos1, *s1, pos2, *s2), local);
                assert_eq!(
                    intersection_test(pos1, *s1, pos2, *s2),
                    crate::dispatch::intersection_test(pos12, *s1, *s2),
                );
                match dispatcher::closest_points(pos12, *s1, *s2, int(10)) {
                    Some(cp) => assert_eq!(
                        closest_points(pos1, *s1, pos2, *s2, int(10)),
                        Some(cp.transform_by(pos1, pos2)),
                    ),
                    None => {
                        assert!(closest_points(pos1, *s1, pos2, *s2, int(10)).is_none());
                        unsupported += 1;
                    },
                }
                match dispatcher::contact(pos12, *s1, *s2, int(10)) {
                    Some(Some(c)) => {
                        let mut c = c;
                        c.transform_by_mut(pos1, pos2);
                        assert_eq!(contact(pos1, *s1, pos2, *s2, int(10)), Some(Some(c)));
                    },
                    Some(None) => assert_eq!(contact(pos1, *s1, pos2, *s2, int(10)), Some(None)),
                    None => assert!(contact(pos1, *s1, pos2, *s2, int(10)).is_none()),
                }
            }
        }
        assert_eq!(unsupported, 1);
    }

    #[test]
    fn gas_baseline() {}

    // World-space entry points, one pair per kernel family (the tables are inlined).
    #[test]
    fn gas_distance_ball_ball() {
        let _ = distance(
            opaque(quarter()),
            opaque(Shape::Ball(BallTrait::new(HALF))),
            opaque(at(int(3), ZERO)),
            opaque(Shape::Ball(BallTrait::new(HALF))),
        );
    }

    #[test]
    fn gas_distance_cuboid_cuboid() {
        let _ = distance(
            opaque(quarter()),
            opaque(Shape::Cuboid(CuboidTrait::new(v(HALF, ONE)))),
            opaque(at(int(4), ZERO)),
            opaque(Shape::Cuboid(CuboidTrait::new(v(HALF, ONE)))),
        );
    }

    #[test]
    fn gas_distance_capsule_segment() {
        let _ = distance(
            opaque(quarter()),
            opaque(Shape::Capsule(CapsuleTrait::new_x(HALF, HALF))),
            opaque(at(int(4), ZERO)),
            opaque(Shape::Segment(SegmentTrait::new(v(ZERO, -ONE), v(ZERO, ONE)))),
        );
    }

    #[test]
    fn gas_closest_points_ball_cuboid() {
        let _ = closest_points(
            opaque(quarter()),
            opaque(Shape::Ball(BallTrait::new(HALF))),
            opaque(at(int(4), ZERO)),
            opaque(Shape::Cuboid(CuboidTrait::new(v(HALF, ONE)))),
            opaque(int(10)),
        );
    }

    #[test]
    fn gas_contact_halfspace_capsule() {
        let _ = contact(
            opaque(quarter()),
            opaque(Shape::HalfSpace(HalfSpaceTrait::new(v(ZERO, ONE)))),
            opaque(at(ONE, TWO)),
            opaque(Shape::Capsule(CapsuleTrait::new_x(HALF, HALF))),
            opaque(ZERO),
        );
    }

    #[test]
    fn gas_contact_cuboid_polygon() {
        let triangle = ConvexPolygonTrait::from_convex_polyline(
            array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span(),
        )
            .unwrap();
        let _ = contact(
            opaque(quarter()),
            opaque(Shape::Cuboid(CuboidTrait::new(v(HALF, ONE)))),
            opaque(at(ONE, TWO)),
            opaque(Shape::ConvexPolygon(BoxTrait::new(triangle))),
            opaque(ZERO),
        );
    }

    #[test]
    fn gas_intersection_test_cuboid_cuboid() {
        let _ = intersection_test(
            opaque(quarter()),
            opaque(Shape::Cuboid(CuboidTrait::new(v(HALF, ONE)))),
            opaque(at(ONE, TWO)),
            opaque(Shape::Cuboid(CuboidTrait::new(v(HALF, ONE)))),
        );
    }
}
