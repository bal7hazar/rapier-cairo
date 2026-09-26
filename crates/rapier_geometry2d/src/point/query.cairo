//! Parry's `PointQuery` and `PointQueryWithLocation` traits over the closed shape set (work
//! package QY1).
//!
//! The per-shape free functions of [`crate::point`] (`project_local_point_ball`, …) stay the
//! kernels; this module only gives them upstream's trait surface, generic over the shape type,
//! with upstream's provided methods as default implementations: the `Pose2` wrappers
//! (`project_point`, `distance_to_point`, `contains_point`, …) move the point into the local
//! frame and the projection back, and the `*_with_max_dist` variants filter on the distance.
//! [`Shape`] dispatches by `match`, inlined so that the caller pays the arm it reaches.
//!
//! # Deviations
//!
//! * `self` is taken by value (value types, AGENTS §7) instead of `&self`.
//! * `*_with_max_dist` compare `|proj - pt|` with `max_dist` on the exact wide square
//!   (`max_dist < 0` never answers), where upstream compares the rounded length.
//! * `PointQueryWithLocation` carries its location type as a second generic parameter instead of
//!   an associated type; the implementors are [`Segment`] (`SegmentPointLocation`) and
//!   [`Triangle`] (`TrianglePointLocation`).
//! * The round shapes' impl is generic over the inner shape (`super::round_shape`).

use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_math::math_ext::norm2::is_norm2_gt;
use rapier_math::pose2::{Pose2, Pose2Trait};
use crate::aabb::{
    Aabb, contains_local_point_aabb, distance_to_local_point_aabb, project_local_point_aabb,
    project_local_point_and_get_feature_aabb,
};
use crate::feature_id::FeatureId;
use crate::shape::{
    Ball, Capsule, ConvexPolygon, Cuboid, HalfSpace, Segment, Shape, Triangle,
    TrianglePointLocation,
};
use super::convex_polygon::{
    contains_local_point_convex_polygon, distance_to_local_point_convex_polygon,
    project_local_point_and_get_feature_convex_polygon, project_local_point_convex_polygon,
};
use super::round_shape::{
    contains_local_point_round, distance_to_local_point_round,
    project_local_point_and_get_feature_round, project_local_point_round,
};
use super::triangle::{
    contains_local_point_triangle, distance_to_local_point_triangle,
    project_local_point_and_get_feature_triangle, project_local_point_and_get_location_triangle,
    project_local_point_triangle,
};
use super::{
    PointProjection, PointProjectionTrait, SegmentPointLocation, contains_local_point_ball,
    contains_local_point_capsule, contains_local_point_cuboid, contains_local_point_halfspace,
    contains_local_point_segment, distance_to_local_point_ball, distance_to_local_point_capsule,
    distance_to_local_point_cuboid, distance_to_local_point_halfspace,
    distance_to_local_point_segment, project_local_point_and_get_feature_ball,
    project_local_point_and_get_feature_capsule, project_local_point_and_get_feature_cuboid,
    project_local_point_and_get_feature_halfspace, project_local_point_and_get_feature_segment,
    project_local_point_and_get_location_segment, project_local_point_ball,
    project_local_point_capsule, project_local_point_cuboid, project_local_point_halfspace,
    project_local_point_segment,
};

/// `Some(proj)` when `|proj.point - pt| <= max_dist`, compared wide.
#[inline(always)]
fn within_max_dist(proj: PointProjection, pt: Vec2, max_dist: Fixed) -> Option<PointProjection> {
    let d = proj.point - pt;
    if max_dist < ZERO || is_norm2_gt(d.x, d.y, max_dist) {
        None
    } else {
        Some(proj)
    }
}

/// Point projection on a shape (Parry `PointQuery`). Every point is in the local frame of the
/// shape unless the method takes a pose `m`, in which case both the point and the answer are in
/// the frame `m` is expressed in.
pub trait PointQuery<T, +Drop<T>> {
    /// Projects `pt` on the shape: onto the filled shape if `solid`, onto its boundary otherwise.
    fn project_local_point(self: T, pt: Vec2, solid: bool) -> PointProjection;

    /// Projects `pt` on the boundary and returns the feature it lands on.
    fn project_local_point_and_get_feature(self: T, pt: Vec2) -> (PointProjection, FeatureId);

    /// Signed distance from `pt` to the shape: negative inside when `solid = false`, zero
    /// inside when `solid = true`.
    fn distance_to_local_point(self: T, pt: Vec2, solid: bool) -> Fixed;

    /// Whether `pt` is inside the shape, boundary included.
    fn contains_local_point(self: T, pt: Vec2) -> bool;

    /// [`PointQuery::project_local_point`], `None` when the projection is further than
    /// `max_dist` from `pt`.
    fn project_local_point_with_max_dist(
        self: T, pt: Vec2, solid: bool, max_dist: Fixed,
    ) -> Option<
        PointProjection,
    > {
        within_max_dist(Self::project_local_point(self, pt, solid), pt, max_dist)
    }

    /// [`PointQuery::project_local_point_with_max_dist`] for the shape placed at `m`.
    fn project_point_with_max_dist(
        self: T, m: Pose2, pt: Vec2, solid: bool, max_dist: Fixed,
    ) -> Option<
        PointProjection,
    > {
        let proj = Self::project_local_point_with_max_dist(
            self, m.inverse_transform_point(pt), solid, max_dist,
        )?;
        Some(proj.transform_by(m))
    }

    /// [`PointQuery::project_local_point`] for the shape placed at `m`.
    fn project_point(
        self: T, m: Pose2, pt: Vec2, solid: bool,
    ) -> PointProjection {
        Self::project_local_point(self, m.inverse_transform_point(pt), solid).transform_by(m)
    }

    /// [`PointQuery::distance_to_local_point`] for the shape placed at `m`.
    fn distance_to_point(
        self: T, m: Pose2, pt: Vec2, solid: bool,
    ) -> Fixed {
        Self::distance_to_local_point(self, m.inverse_transform_point(pt), solid)
    }

    /// [`PointQuery::project_local_point_and_get_feature`] for the shape placed at `m`.
    fn project_point_and_get_feature(
        self: T, m: Pose2, pt: Vec2,
    ) -> (
        PointProjection, FeatureId,
    ) {
        let (proj, feature) = Self::project_local_point_and_get_feature(
            self, m.inverse_transform_point(pt),
        );
        (proj.transform_by(m), feature)
    }

    /// [`PointQuery::contains_local_point`] for the shape placed at `m`.
    fn contains_point(
        self: T, m: Pose2, pt: Vec2,
    ) -> bool {
        Self::contains_local_point(self, m.inverse_transform_point(pt))
    }
}

/// Point projection that also reports where the projection lies (Parry
/// `PointQueryWithLocation`); `L` is upstream's associated `Location` type.
pub trait PointQueryWithLocation<T, L, +Drop<T>, +Drop<L>> {
    /// Projects `pt` on the shape and returns the location of the projection.
    fn project_local_point_and_get_location(self: T, pt: Vec2, solid: bool) -> (PointProjection, L);

    /// [`PointQueryWithLocation::project_local_point_and_get_location`] for the shape placed at
    /// `m`.
    fn project_point_and_get_location(
        self: T, m: Pose2, pt: Vec2, solid: bool,
    ) -> (
        PointProjection, L,
    ) {
        let (proj, location) = Self::project_local_point_and_get_location(
            self, m.inverse_transform_point(pt), solid,
        );
        (proj.transform_by(m), location)
    }

    /// [`PointQueryWithLocation::project_local_point_and_get_location`], `None` when the
    /// projection is further than `max_dist` from `pt`.
    fn project_local_point_and_get_location_with_max_dist(
        self: T, pt: Vec2, solid: bool, max_dist: Fixed,
    ) -> Option<
        (PointProjection, L),
    > {
        let (proj, location) = Self::project_local_point_and_get_location(self, pt, solid);
        match within_max_dist(proj, pt, max_dist) {
            Some(proj) => Some((proj, location)),
            None => None,
        }
    }

    /// [`PointQueryWithLocation::project_local_point_and_get_location_with_max_dist`] for the
    /// shape placed at `m`.
    fn project_point_and_get_location_with_max_dist(
        self: T, m: Pose2, pt: Vec2, solid: bool, max_dist: Fixed,
    ) -> Option<
        (PointProjection, L),
    > {
        let (proj, location) = Self::project_local_point_and_get_location_with_max_dist(
            self, m.inverse_transform_point(pt), solid, max_dist,
        )?;
        Some((proj.transform_by(m), location))
    }
}

pub impl BallPointQuery of PointQuery<Ball> {
    fn project_local_point(self: Ball, pt: Vec2, solid: bool) -> PointProjection {
        project_local_point_ball(self, pt, solid)
    }
    fn project_local_point_and_get_feature(self: Ball, pt: Vec2) -> (PointProjection, FeatureId) {
        project_local_point_and_get_feature_ball(self, pt)
    }
    fn distance_to_local_point(self: Ball, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_ball(self, pt, solid)
    }
    fn contains_local_point(self: Ball, pt: Vec2) -> bool {
        contains_local_point_ball(self, pt)
    }
}

pub impl CuboidPointQuery of PointQuery<Cuboid> {
    fn project_local_point(self: Cuboid, pt: Vec2, solid: bool) -> PointProjection {
        project_local_point_cuboid(self, pt, solid)
    }
    fn project_local_point_and_get_feature(self: Cuboid, pt: Vec2) -> (PointProjection, FeatureId) {
        project_local_point_and_get_feature_cuboid(self, pt)
    }
    fn distance_to_local_point(self: Cuboid, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_cuboid(self, pt, solid)
    }
    fn contains_local_point(self: Cuboid, pt: Vec2) -> bool {
        contains_local_point_cuboid(self, pt)
    }
}

pub impl CapsulePointQuery of PointQuery<Capsule> {
    fn project_local_point(self: Capsule, pt: Vec2, solid: bool) -> PointProjection {
        project_local_point_capsule(self, pt, solid)
    }
    fn project_local_point_and_get_feature(
        self: Capsule, pt: Vec2,
    ) -> (PointProjection, FeatureId) {
        project_local_point_and_get_feature_capsule(self, pt)
    }
    fn distance_to_local_point(self: Capsule, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_capsule(self, pt, solid)
    }
    fn contains_local_point(self: Capsule, pt: Vec2) -> bool {
        contains_local_point_capsule(self, pt)
    }
}

pub impl SegmentPointQuery of PointQuery<Segment> {
    fn project_local_point(self: Segment, pt: Vec2, solid: bool) -> PointProjection {
        project_local_point_segment(self, pt, solid)
    }
    fn project_local_point_and_get_feature(
        self: Segment, pt: Vec2,
    ) -> (PointProjection, FeatureId) {
        project_local_point_and_get_feature_segment(self, pt)
    }
    fn distance_to_local_point(self: Segment, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_segment(self, pt, solid)
    }
    fn contains_local_point(self: Segment, pt: Vec2) -> bool {
        contains_local_point_segment(self, pt)
    }
}

pub impl SegmentPointQueryWithLocation of PointQueryWithLocation<Segment, SegmentPointLocation> {
    fn project_local_point_and_get_location(
        self: Segment, pt: Vec2, solid: bool,
    ) -> (PointProjection, SegmentPointLocation) {
        project_local_point_and_get_location_segment(self, pt, solid)
    }
}

pub impl HalfSpacePointQuery of PointQuery<HalfSpace> {
    fn project_local_point(self: HalfSpace, pt: Vec2, solid: bool) -> PointProjection {
        project_local_point_halfspace(self, pt, solid)
    }
    fn project_local_point_and_get_feature(
        self: HalfSpace, pt: Vec2,
    ) -> (PointProjection, FeatureId) {
        project_local_point_and_get_feature_halfspace(self, pt)
    }
    fn distance_to_local_point(self: HalfSpace, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_halfspace(self, pt, solid)
    }
    fn contains_local_point(self: HalfSpace, pt: Vec2) -> bool {
        contains_local_point_halfspace(self, pt)
    }
}

pub impl ConvexPolygonPointQuery of PointQuery<ConvexPolygon> {
    fn project_local_point(self: ConvexPolygon, pt: Vec2, solid: bool) -> PointProjection {
        project_local_point_convex_polygon(self, pt, solid)
    }
    fn project_local_point_and_get_feature(
        self: ConvexPolygon, pt: Vec2,
    ) -> (PointProjection, FeatureId) {
        project_local_point_and_get_feature_convex_polygon(self, pt)
    }
    fn distance_to_local_point(self: ConvexPolygon, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_convex_polygon(self, pt, solid)
    }
    fn contains_local_point(self: ConvexPolygon, pt: Vec2) -> bool {
        contains_local_point_convex_polygon(self, pt)
    }
}

pub impl TrianglePointQuery of PointQuery<Triangle> {
    fn project_local_point(self: Triangle, pt: Vec2, solid: bool) -> PointProjection {
        project_local_point_triangle(self, pt, solid)
    }
    fn project_local_point_and_get_feature(
        self: Triangle, pt: Vec2,
    ) -> (PointProjection, FeatureId) {
        project_local_point_and_get_feature_triangle(self, pt)
    }
    fn distance_to_local_point(self: Triangle, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_triangle(self, pt, solid)
    }
    fn contains_local_point(self: Triangle, pt: Vec2) -> bool {
        contains_local_point_triangle(self, pt)
    }
}

pub impl TrianglePointQueryWithLocation of PointQueryWithLocation<Triangle, TrianglePointLocation> {
    fn project_local_point_and_get_location(
        self: Triangle, pt: Vec2, solid: bool,
    ) -> (PointProjection, TrianglePointLocation) {
        project_local_point_and_get_location_triangle(self, pt, solid)
    }
}

pub impl AabbPointQuery of PointQuery<Aabb> {
    fn project_local_point(self: Aabb, pt: Vec2, solid: bool) -> PointProjection {
        project_local_point_aabb(self, pt, solid)
    }
    fn project_local_point_and_get_feature(self: Aabb, pt: Vec2) -> (PointProjection, FeatureId) {
        project_local_point_and_get_feature_aabb(self, pt)
    }
    fn distance_to_local_point(self: Aabb, pt: Vec2, solid: bool) -> Fixed {
        distance_to_local_point_aabb(self, pt, solid)
    }
    fn contains_local_point(self: Aabb, pt: Vec2) -> bool {
        contains_local_point_aabb(self, pt)
    }
}

/// `match` dispatch over the closed set; every method is inlined so that the caller pays the arm
/// it reaches.
pub impl ShapePointQuery of PointQuery<Shape> {
    #[inline(always)]
    fn project_local_point(self: Shape, pt: Vec2, solid: bool) -> PointProjection {
        match self {
            Shape::Ball(s) => project_local_point_ball(s, pt, solid),
            Shape::Cuboid(s) => project_local_point_cuboid(s, pt, solid),
            Shape::Capsule(s) => project_local_point_capsule(s, pt, solid),
            Shape::Segment(s) => project_local_point_segment(s, pt, solid),
            Shape::HalfSpace(s) => project_local_point_halfspace(s, pt, solid),
            Shape::ConvexPolygon(s) => project_local_point_convex_polygon(s.unbox(), pt, solid),
            Shape::Triangle(s) => project_local_point_triangle(s.unbox(), pt, solid),
            Shape::RoundCuboid(s) => project_local_point_round(
                s.inner_shape, s.border_radius, pt, solid,
            ),
            Shape::RoundTriangle(s) => {
                let s = s.unbox();
                project_local_point_round(s.inner_shape, s.border_radius, pt, solid)
            },
            Shape::RoundConvexPolygon(s) => {
                let s = s.unbox();
                project_local_point_round(s.inner_shape, s.border_radius, pt, solid)
            },
        }
    }
    #[inline(always)]
    fn project_local_point_and_get_feature(self: Shape, pt: Vec2) -> (PointProjection, FeatureId) {
        match self {
            Shape::Ball(s) => project_local_point_and_get_feature_ball(s, pt),
            Shape::Cuboid(s) => project_local_point_and_get_feature_cuboid(s, pt),
            Shape::Capsule(s) => project_local_point_and_get_feature_capsule(s, pt),
            Shape::Segment(s) => project_local_point_and_get_feature_segment(s, pt),
            Shape::HalfSpace(s) => project_local_point_and_get_feature_halfspace(s, pt),
            Shape::ConvexPolygon(s) => project_local_point_and_get_feature_convex_polygon(
                s.unbox(), pt,
            ),
            Shape::Triangle(s) => project_local_point_and_get_feature_triangle(s.unbox(), pt),
            Shape::RoundCuboid(s) => project_local_point_and_get_feature_round(
                s.inner_shape, s.border_radius, pt,
            ),
            Shape::RoundTriangle(s) => {
                let s = s.unbox();
                project_local_point_and_get_feature_round(s.inner_shape, s.border_radius, pt)
            },
            Shape::RoundConvexPolygon(s) => {
                let s = s.unbox();
                project_local_point_and_get_feature_round(s.inner_shape, s.border_radius, pt)
            },
        }
    }
    #[inline(always)]
    fn distance_to_local_point(self: Shape, pt: Vec2, solid: bool) -> Fixed {
        match self {
            Shape::Ball(s) => distance_to_local_point_ball(s, pt, solid),
            Shape::Cuboid(s) => distance_to_local_point_cuboid(s, pt, solid),
            Shape::Capsule(s) => distance_to_local_point_capsule(s, pt, solid),
            Shape::Segment(s) => distance_to_local_point_segment(s, pt, solid),
            Shape::HalfSpace(s) => distance_to_local_point_halfspace(s, pt, solid),
            Shape::ConvexPolygon(s) => distance_to_local_point_convex_polygon(s.unbox(), pt, solid),
            Shape::Triangle(s) => distance_to_local_point_triangle(s.unbox(), pt, solid),
            Shape::RoundCuboid(s) => distance_to_local_point_round(
                s.inner_shape, s.border_radius, pt, solid,
            ),
            Shape::RoundTriangle(s) => {
                let s = s.unbox();
                distance_to_local_point_round(s.inner_shape, s.border_radius, pt, solid)
            },
            Shape::RoundConvexPolygon(s) => {
                let s = s.unbox();
                distance_to_local_point_round(s.inner_shape, s.border_radius, pt, solid)
            },
        }
    }
    #[inline(always)]
    fn contains_local_point(self: Shape, pt: Vec2) -> bool {
        match self {
            Shape::Ball(s) => contains_local_point_ball(s, pt),
            Shape::Cuboid(s) => contains_local_point_cuboid(s, pt),
            Shape::Capsule(s) => contains_local_point_capsule(s, pt),
            Shape::Segment(s) => contains_local_point_segment(s, pt),
            Shape::HalfSpace(s) => contains_local_point_halfspace(s, pt),
            Shape::ConvexPolygon(s) => contains_local_point_convex_polygon(s.unbox(), pt),
            _ => contains_local_point_sh1(self, pt) != 0,
        }
    }
}

/// The SH1 arms of `ShapePointQuery::contains_local_point`, metered (one-iteration loop): their
/// projections are statically dearer than every old arm, which would otherwise pay their gas
/// alignment. Answers `1` inside, `0` outside: a `bool` result would share its post-call block
/// with the old arms' calls, moving their code layout.
#[inline(never)]
fn contains_local_point_sh1(shape: Shape, pt: Vec2) -> u8 {
    let mut inside = false;
    let mut pending = true;
    while pending {
        inside = match shape {
            Shape::Triangle(s) => contains_local_point_triangle(s.unbox(), pt),
            Shape::RoundCuboid(s) => contains_local_point_round(s.inner_shape, s.border_radius, pt),
            Shape::RoundTriangle(s) => {
                let s = s.unbox();
                contains_local_point_round(s.inner_shape, s.border_radius, pt)
            },
            Shape::RoundConvexPolygon(s) => {
                let s = s.unbox();
                contains_local_point_round(s.inner_shape, s.border_radius, pt)
            },
            _ => false,
        };
        pending = false;
    }
    if inside {
        1
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_math::pose2::{Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use crate::aabb::AabbTrait;
    use crate::feature_id::{FeatureId, FeatureIdTrait};
    use crate::point::{PointProjection, SegmentPointLocation};
    use crate::shape::{
        BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, Segment,
        SegmentTrait, Shape,
    };
    use super::{PointQuery, PointQueryWithLocation, ShapePointQuery};

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

    fn shapes() -> Span<Shape> {
        let triangle = ConvexPolygonTrait::from_convex_polyline(
            array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span(),
        )
            .unwrap();
        array![
            Shape::Ball(BallTrait::new(HALF)), Shape::Cuboid(CuboidTrait::new(v(HALF, ONE))),
            Shape::Capsule(CapsuleTrait::new_x(HALF, HALF)),
            Shape::Segment(SegmentTrait::new(v(ZERO, -ONE), v(ZERO, ONE))),
            Shape::HalfSpace(HalfSpaceTrait::new(v(-ONE, ZERO))),
            Shape::ConvexPolygon(BoxTrait::new(triangle)),
        ]
            .span()
    }

    /// The posed methods are the local ones on the moved point, with the answer moved back.
    #[test]
    fn test_posed_methods_match_local_methods() {
        let m = quarter();
        let points = array![v(int(3), int(2)), v(ONE, TWO), v(ZERO, ZERO), v(int(-2), HALF)];
        for shape in shapes() {
            for pt in points.span() {
                let local = m.inverse_transform_point(*pt);
                let proj = (*shape).project_local_point(local, false);
                let posed = (*shape).project_point(m, *pt, false);
                assert_eq!(
                    posed,
                    PointProjection {
                        is_inside: proj.is_inside, point: m.transform_point(proj.point),
                    },
                );
                let (fproj, feature) = (*shape).project_local_point_and_get_feature(local);
                let (fposed, fposed_id) = (*shape).project_point_and_get_feature(m, *pt);
                assert_eq!(fposed_id, feature);
                assert_eq!(fposed.point, m.transform_point(fproj.point));
                assert_eq!(
                    (*shape).distance_to_point(m, *pt, true),
                    (*shape).distance_to_local_point(local, true),
                );
                assert_eq!((*shape).contains_point(m, *pt), (*shape).contains_local_point(local));
            }
        }
    }

    /// `(point, max_dist, answers)`: the filter keeps projections at exactly `max_dist`.
    #[test]
    fn test_max_dist_filters_table() {
        let ball = BallTrait::new(ONE);
        let cases: Span<(Vec2, Fixed, bool)> = array![
            (v(int(3), ZERO), TWO, true), (v(int(3), ZERO), TWO - Fixed { raw: 1 }, false),
            (v(HALF, ZERO), ZERO, true), (v(int(-5), ZERO), int(10), true),
            (v(int(3), ZERO), -ONE, false),
        ]
            .span();
        for (pt, max_dist, answers) in cases {
            assert_eq!(
                ball.project_local_point_with_max_dist(*pt, true, *max_dist).is_some(), *answers,
            );
            let world = quarter().transform_point(*pt);
            assert_eq!(
                ball.project_point_with_max_dist(quarter(), world, true, *max_dist).is_some(),
                *answers,
            );
        }
    }

    #[test]
    fn test_segment_location_queries() {
        let seg = SegmentTrait::new(v(-ONE, ZERO), v(ONE, ZERO));
        let (proj, loc) = seg.project_local_point_and_get_location(v(int(3), ONE), true);
        assert_eq!(proj.point, v(ONE, ZERO));
        assert_eq!(loc, SegmentPointLocation::OnVertex(1));
        assert!(
            seg
                .project_local_point_and_get_location_with_max_dist(v(int(3), ONE), true, ONE)
                .is_none(),
        );
        let (posed, posed_loc) = seg
            .project_point_and_get_location_with_max_dist(quarter(), v(ONE, TWO), true, ONE)
            .unwrap();
        assert_eq!(posed.point, v(ONE, TWO));
        assert_eq!(posed_loc, SegmentPointLocation::OnEdge((HALF, HALF)));
        let (_, loc2) = seg.project_point_and_get_location(quarter(), v(ONE, int(5)), false);
        assert_eq!(loc2, SegmentPointLocation::OnVertex(1));
    }

    /// The `Shape` dispatch answers what the typed implementations answer.
    #[test]
    fn test_shape_dispatch_matches_typed_impls() {
        let pt = v(ONE, HALF);
        let ball = BallTrait::new(HALF);
        assert_eq!(
            Shape::Ball(ball).project_local_point(pt, true), ball.project_local_point(pt, true),
        );
        let seg: Segment = SegmentTrait::new(v(ZERO, -ONE), v(ZERO, ONE));
        assert_eq!(
            Shape::Segment(seg).project_local_point_and_get_feature(pt),
            seg.project_local_point_and_get_feature(pt),
        );
        let cuboid = CuboidTrait::new(v(HALF, ONE));
        assert_eq!(
            Shape::Cuboid(cuboid).distance_to_local_point(pt, false),
            cuboid.distance_to_local_point(pt, false),
        );
        let capsule = CapsuleTrait::new_x(HALF, HALF);
        assert_eq!(
            Shape::Capsule(capsule).contains_local_point(pt), capsule.contains_local_point(pt),
        );
    }

    /// `(point, solid, projection, inside, feature)` on the box `[0, 2] x [0, 1]`.
    #[test]
    fn test_aabb_projection_table() {
        let aabb = AabbTrait::new(v(ZERO, ZERO), v(TWO, ONE));
        let cases: Span<(Vec2, bool, Vec2, bool, FeatureId)> = array![
            (v(int(3), HALF), true, v(TWO, HALF), false, FeatureIdTrait::face(0)),
            (v(int(-1), HALF), true, v(ZERO, HALF), false, FeatureIdTrait::face(2)),
            (v(int(3), int(3)), true, v(TWO, ONE), false, FeatureIdTrait::vertex(0)),
            (v(int(-1), int(-1)), true, v(ZERO, ZERO), false, FeatureIdTrait::vertex(3)),
            (v(ONE, HALF), true, v(ONE, HALF), true, FeatureIdTrait::face(1)),
            (v(HALF, HALF), false, v(ZERO, HALF), true, FeatureIdTrait::face(2)),
            (v(ONE + HALF, HALF), false, v(TWO, HALF), true, FeatureIdTrait::face(0)),
        ]
            .span();
        for (pt, solid, point, inside, feature) in cases {
            let proj = aabb.project_local_point(*pt, *solid);
            assert_eq!(proj, PointProjection { is_inside: *inside, point: *point });
            let (_, f) = aabb.project_local_point_and_get_feature(*pt);
            assert_eq!(f, *feature);
            assert_eq!(super::AabbPointQuery::contains_local_point(aabb, *pt), *inside);
        }
        assert_eq!(aabb.distance_to_local_point(v(int(5), int(5)), true), int(5));
        assert_eq!(aabb.distance_to_local_point(v(HALF, HALF), false), -HALF);
        assert_eq!(aabb.distance_to_local_point(v(HALF, HALF), true), ZERO);
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_aabb_project_local_point_and_get_feature() {
        let _ = opaque(AabbTrait::new(v(ZERO, ZERO), v(TWO, ONE)))
            .project_local_point_and_get_feature(opaque(v(ONE + HALF, HALF)));
    }

    #[test]
    fn gas_shape_project_point_cuboid() {
        let _ = ShapePointQuery::project_point(
            opaque(Shape::Cuboid(CuboidTrait::new(v(HALF, ONE)))),
            opaque(quarter()),
            opaque(v(int(3), int(2))),
            false,
        );
    }

    #[test]
    fn gas_ball_project_local_point_with_max_dist() {
        let _ = opaque(BallTrait::new(ONE))
            .project_local_point_with_max_dist(opaque(v(int(3), ZERO)), true, opaque(TWO));
    }

    #[test]
    fn gas_segment_project_point_and_get_location() {
        let _ = opaque(SegmentTrait::new(v(-ONE, ZERO), v(ONE, ZERO)))
            .project_point_and_get_location(opaque(quarter()), opaque(v(ONE, int(5))), false);
    }
}
