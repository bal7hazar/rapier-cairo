//! Point queries on a [`Segment`] (Parry `query/point/point_segment.rs` and
//! `PointQueryWithLocation for Segment`).
//!
//! The projection is one clamped ratio `u = (ab . ap) / |ab|^2`, i.e. the single division of the
//! point module. Everything the caller branches on afterwards — the Voronoi region, `is_inside`,
//! the `Face(0)` / `Face(1)` side — is decided on the **exact** `i128` products of
//! [`super::wide2`], never on the rounded projection:
//!
//! * the region is `ab . ap <= 0` / `>= |ab|^2`, both raw Q64.64 quantities of the same scale;
//! * `is_inside` on the interior is `ab x ap == 0` (the point is exactly on the line) instead of
//!   upstream's `relative_eq(proj, pt)`, which in Q32.32 would compare a value that was just
//!   rounded twice with the input it came from and answer "not on the segment" for points that
//!   demonstrably are;
//! * the side of the segment is `perp_dot(pt - proj, ab) = -(ab x ap)` exactly, because
//!   `perp_dot(ab, ab) = 0` removes the `u` term.

use fixed::wide::{distance2, mul_add};
use fixed::{Fixed, ONE, ZERO};
use glam::vec2::Vec2;
use rapier_math::math_ext::norm2::norm2_sq_wide;
use crate::feature_id::{FeatureId, FeatureIdTrait};
use super::ratio::clamped_ratio;
use super::shapes_shim::{Segment, segment_scaled_direction};
use super::wide2::{cross_wide, dot_wide};
use super::{PointProjection, SegmentPointLocation};

/// Projects `pt` on `seg` and returns where on the segment it landed, plus the exact
/// `ab x ap` the caller may need for the side test.
fn project_with_cross(seg: Segment, pt: Vec2) -> (PointProjection, SegmentPointLocation, i128) {
    let ab = segment_scaled_direction(seg);
    let ap = Vec2 { x: pt.x - seg.a.x, y: pt.y - seg.a.y };
    let ab_ap = dot_wide(ab.x, ab.y, ap.x, ap.y);
    let sq_ab = norm2_sq_wide(ab.x, ab.y);
    if ab_ap <= 0 {
        // Voronoi region of `a` (a zero-length segment lands here too).
        (
            PointProjection { is_inside: pt == seg.a, point: seg.a },
            SegmentPointLocation::OnVertex(0),
            0,
        )
    } else if ab_ap >= sq_ab {
        // Voronoi region of `b`.
        (
            PointProjection { is_inside: pt == seg.b, point: seg.b },
            SegmentPointLocation::OnVertex(1),
            0,
        )
    } else {
        let u = clamped_ratio(ab_ap, sq_ab);
        let point = Vec2 { x: mul_add(ab.x, u, seg.a.x), y: mul_add(ab.y, u, seg.a.y) };
        let cross = cross_wide(ab.x, ab.y, ap.x, ap.y);
        (
            PointProjection { is_inside: cross == 0, point },
            SegmentPointLocation::OnEdge((ONE - u, u)),
            cross,
        )
    }
}

/// Returns the point of `seg` named by `location`.
///
/// Mirrors `parry::shape::Segment::point_at`: `a` for `OnVertex(0)`, `b` for any other vertex
/// code, `a * u + b * v` for `OnEdge((u, v))`.
/// #### Panics
/// * `'Fixed: overflow'` if the interpolated point does not fit the scalar range.
/// #### Deviations
/// * The interpolation is a single fused `a * u + b * v` (one rescale), so it is the floor of the
///   exact combination instead of two rounded products.
pub fn segment_point_at(seg: Segment, location: SegmentPointLocation) -> Vec2 {
    match location {
        SegmentPointLocation::OnVertex(i) => if i == 0 {
            seg.a
        } else {
            seg.b
        },
        SegmentPointLocation::OnEdge((
            u, v,
        )) => Vec2 {
            x: fixed::wide::dot2(seg.a.x, u, seg.b.x, v),
            y: fixed::wide::dot2(seg.a.y, u, seg.b.y, v),
        },
    }
}

/// Projects `pt` on `seg` and returns the location of the projection.
///
/// Mirrors `PointQueryWithLocation::project_local_point_and_get_location` for `Segment`. `solid`
/// is ignored, as upstream does: a segment has no interior.
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `b - a` or `pt - a` leaves the scalar range.
/// * `'i128_add Overflow'` for raws at the very ends of the range (see [`super::wide2`]).
/// #### Deviations
/// * `is_inside` is the exact collinearity test described in the module documentation instead of
///   upstream's approximate comparison of the projected point with `pt`.
/// * `u` is the correctly rounded ratio of two exact wide products, so it is within half an ulp
///   of the exact value even for segments shorter than `2^-16`, whose `|ab|^2` is 0 as a `Fixed`.
#[inline(always)]
pub fn project_local_point_and_get_location_segment(
    seg: Segment, pt: Vec2, solid: bool,
) -> (PointProjection, SegmentPointLocation) {
    let _ = solid;
    let (proj, location, _) = project_with_cross(seg, pt);
    (proj, location)
}

/// Projects `pt` on `seg`.
///
/// Mirrors `PointQuery::project_local_point` for `Segment`; see
/// [`project_local_point_and_get_location_segment`].
/// #### Panics
/// * See [`project_local_point_and_get_location_segment`].
/// #### Deviations
/// * See [`project_local_point_and_get_location_segment`].
#[inline(always)]
pub fn project_local_point_segment(seg: Segment, pt: Vec2, solid: bool) -> PointProjection {
    let _ = solid;
    let (proj, _, _) = project_with_cross(seg, pt);
    proj
}

/// Projects `pt` on `seg` and names the feature it landed on.
///
/// Mirrors `PointQuery::project_local_point_and_get_feature` for `Segment`: `Vertex(0)` /
/// `Vertex(1)` in a vertex region, and on the interior `Face(0)` when `perp_dot(pt - proj, ab)`
/// is non-negative, `Face(1)` otherwise.
/// #### Panics
/// * See [`project_local_point_and_get_location_segment`].
/// #### Deviations
/// * The side is read off the exact `ab x ap` rather than off the rounded `pt - proj`, so it
///   never flips because the projection landed 1 ulp away.
pub fn project_local_point_and_get_feature_segment(
    seg: Segment, pt: Vec2,
) -> (PointProjection, FeatureId) {
    let (proj, location, cross) = project_with_cross(seg, pt);
    let feature = match location {
        SegmentPointLocation::OnVertex(i) => FeatureIdTrait::vertex(i),
        SegmentPointLocation::OnEdge(_) => if cross <= 0 {
            FeatureIdTrait::face(0)
        } else {
            FeatureIdTrait::face(1)
        },
    };
    (proj, feature)
}

/// Returns the distance from `pt` to `seg` (always non-negative: a segment has no interior).
///
/// Mirrors the default `PointQuery::distance_to_local_point` on top of the segment projection.
/// #### Panics
/// * `'Fixed: overflow'` if the distance does not fit the scalar range.
/// #### Deviations
/// * `fixed::wide::distance2` floors the length, so the result is at most 1 ulp below the exact
///   distance.
pub fn distance_to_local_point_segment(seg: Segment, pt: Vec2, solid: bool) -> Fixed {
    let (proj, _, _) = project_with_cross(seg, pt);
    let dist = distance2(pt.x, pt.y, proj.point.x, proj.point.y);
    if solid || !proj.is_inside {
        dist
    } else {
        ZERO - dist
    }
}

/// Returns `true` when `pt` lies exactly on `seg`.
///
/// Mirrors the default `PointQuery::contains_local_point`.
/// #### Panics
/// * See [`project_local_point_and_get_location_segment`].
/// #### Deviations
/// * Exact, by the collinearity test of the module documentation.
#[inline(always)]
pub fn contains_local_point_segment(seg: Segment, pt: Vec2) -> bool {
    let (proj, _, _) = project_with_cross(seg, pt);
    proj.is_inside
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, ONE, ZERO};
    use glam::vec2::Vec2;
    use rapier_testing::opaque;
    use crate::feature_id::{FeatureId, FeatureIdTrait};
    use super::super::shapes_shim::Segment;
    use super::super::{PointProjection, SegmentPointLocation};
    use super::{
        contains_local_point_segment, distance_to_local_point_segment,
        project_local_point_and_get_feature_segment, project_local_point_and_get_location_segment,
        project_local_point_segment, segment_point_at,
    };

    const UNIT: i64 = 0x1_0000_0000;
    const HALF: i64 = 0x8000_0000;

    fn v(x: i64, y: i64) -> Vec2 {
        Vec2 { x: Fixed { raw: x }, y: Fixed { raw: y } }
    }

    fn seg(ax: i64, ay: i64, bx: i64, by: i64) -> Segment {
        Segment { a: v(ax, ay), b: v(bx, by) }
    }

    #[test]
    fn test_location_and_projection_table() {
        let s = seg(-UNIT, 0, UNIT, 0);
        // (segment, point, expected location, expected point, expected is_inside)
        let cases: Span<(Segment, Vec2, SegmentPointLocation, Vec2, bool)> = array![
            // Interior, above and below.
            (
                s,
                v(0, UNIT),
                SegmentPointLocation::OnEdge((Fixed { raw: HALF }, Fixed { raw: HALF })),
                v(0, 0),
                false,
            ),
            (
                s,
                v(0, -UNIT),
                SegmentPointLocation::OnEdge((Fixed { raw: HALF }, Fixed { raw: HALF })),
                v(0, 0),
                false,
            ),
            // Exactly on the interior.
            (
                s,
                v(0, 0),
                SegmentPointLocation::OnEdge((Fixed { raw: HALF }, Fixed { raw: HALF })),
                v(0, 0),
                true,
            ),
            // Vertex regions, including the exact vertices and the edge extensions.
            (s, v(-2 * UNIT, UNIT), SegmentPointLocation::OnVertex(0), v(-UNIT, 0), false),
            (s, v(-UNIT, 0), SegmentPointLocation::OnVertex(0), v(-UNIT, 0), true),
            (s, v(2 * UNIT, 0), SegmentPointLocation::OnVertex(1), v(UNIT, 0), false),
            (s, v(UNIT, 0), SegmentPointLocation::OnVertex(1), v(UNIT, 0), true),
            // A zero-length segment is the vertex region of `a`.
            (
                seg(HALF, HALF, HALF, HALF),
                v(UNIT, UNIT),
                SegmentPointLocation::OnVertex(0),
                v(HALF, HALF),
                false,
            ),
            (
                seg(HALF, HALF, HALF, HALF),
                v(HALF, HALF),
                SegmentPointLocation::OnVertex(0),
                v(HALF, HALF),
                true,
            ),
        ]
            .span();
        for (s, pt, location, point, is_inside) in cases {
            let (proj, loc) = project_local_point_and_get_location_segment(*s, *pt, false);
            assert_eq!(loc, *location);
            assert_eq!(proj, PointProjection { is_inside: *is_inside, point: *point });
            assert_eq!(project_local_point_segment(*s, *pt, true), proj);
            assert_eq!(contains_local_point_segment(*s, *pt), *is_inside);
        }
    }

    #[test]
    fn test_feature_table() {
        let s = seg(-UNIT, 0, UNIT, 0);
        // (point, expected feature)
        let cases: Span<(Vec2, FeatureId)> = array![
            (v(0, UNIT), FeatureIdTrait::face(1)), (v(0, -UNIT), FeatureIdTrait::face(0)),
            (v(0, 0), FeatureIdTrait::face(0)), (v(-2 * UNIT, 0), FeatureIdTrait::vertex(0)),
            (v(2 * UNIT, 0), FeatureIdTrait::vertex(1)),
        ]
            .span();
        for (pt, expected) in cases {
            let (_, feature) = project_local_point_and_get_feature_segment(s, *pt);
            assert_eq!(feature, *expected);
        }
    }

    /// A segment far shorter than `2^-16`, whose `|ab|^2` is 0 once rescaled to `Fixed`: the
    /// wide ratio still splits it correctly.
    #[test]
    fn test_tiny_segment_keeps_its_interior() {
        let s = seg(0, 0, 1024, 0);
        let (proj, loc) = project_local_point_and_get_location_segment(s, v(256, 64), false);
        assert_eq!(
            loc,
            SegmentPointLocation::OnEdge(
                (Fixed { raw: 3 * 0x4000_0000 }, Fixed { raw: 0x4000_0000 }),
            ),
        );
        assert_eq!(proj.point, v(256, 0));
        assert!(!proj.is_inside);
        assert!(contains_local_point_segment(s, v(512, 0)));
        assert_eq!(distance_to_local_point_segment(s, v(256, 64), false), Fixed { raw: 64 });
    }

    /// Huge coordinates: the region test is an `i128` comparison, so nothing overflows.
    #[test]
    fn test_huge_segment() {
        let big = 0x4000_0000_0000_0000;
        let s = Segment { a: v(0, 0), b: v(big, 0) };
        let (proj, loc) = project_local_point_and_get_location_segment(
            s, v(big / 2, big / 4), false,
        );
        assert_eq!(loc, SegmentPointLocation::OnEdge((Fixed { raw: HALF }, Fixed { raw: HALF })));
        assert_eq!(proj.point, v(big / 2, 0));
        let (_, loc) = project_local_point_and_get_location_segment(s, v(big, big), false);
        assert_eq!(loc, SegmentPointLocation::OnVertex(1));
    }

    #[test]
    fn test_point_at_round_trips() {
        let s = seg(-UNIT, 0, UNIT, 0);
        assert_eq!(segment_point_at(s, SegmentPointLocation::OnVertex(0)), v(-UNIT, 0));
        assert_eq!(segment_point_at(s, SegmentPointLocation::OnVertex(1)), v(UNIT, 0));
        assert_eq!(
            segment_point_at(
                s, SegmentPointLocation::OnEdge((Fixed { raw: HALF }, Fixed { raw: HALF })),
            ),
            v(0, 0),
        );
        assert_eq!(segment_point_at(s, SegmentPointLocation::OnEdge((ZERO, ONE))), v(UNIT, 0));
    }

    #[test]
    fn test_distance_is_zero_on_the_segment_and_positive_outside() {
        let s = seg(0, 0, 2 * UNIT, UNIT);
        assert_eq!(distance_to_local_point_segment(s, v(UNIT, HALF), false), ZERO);
        assert_eq!(distance_to_local_point_segment(s, v(0, 0), false), ZERO);
        assert!(distance_to_local_point_segment(s, v(0, UNIT), false) > ZERO);
        assert!(distance_to_local_point_segment(s, v(4 * UNIT, 0), true) > ZERO);
    }

    /// The projection always lands between the end points and `u` reproduces it.
    #[test]
    #[fuzzer(runs: 64, seed: 20260920)]
    fn fuzz_projection_matches_its_location(x: i32, y: i32, bx: i16, by: i16) {
        let s = Segment { a: v(0, 0), b: v(bx.into() * 65536, by.into() * 65536) };
        let pt = v(x.into(), y.into());
        let (proj, loc) = project_local_point_and_get_location_segment(s, pt, false);
        let at = segment_point_at(s, loc);
        assert!(proj.point.x.abs_diff_eq(at.x, Fixed { raw: 4 }));
        assert!(proj.point.y.abs_diff_eq(at.y, Fixed { raw: 4 }));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_project_segment_interior() {
        let _ = project_local_point_segment(
            opaque(seg(-UNIT, 0, UNIT, 0)), opaque(v(0, UNIT)), false,
        );
    }

    #[test]
    fn gas_project_segment_vertex() {
        let _ = project_local_point_segment(
            opaque(seg(-UNIT, 0, UNIT, 0)), opaque(v(4 * UNIT, UNIT)), false,
        );
    }

    #[test]
    fn gas_project_and_get_location_segment() {
        let _ = project_local_point_and_get_location_segment(
            opaque(seg(-UNIT, 0, UNIT, 0)), opaque(v(0, UNIT)), false,
        );
    }

    #[test]
    fn gas_project_and_get_feature_segment() {
        let _ = project_local_point_and_get_feature_segment(
            opaque(seg(-UNIT, 0, UNIT, 0)), opaque(v(0, UNIT)),
        );
    }

    #[test]
    fn gas_distance_to_local_point_segment() {
        let _ = distance_to_local_point_segment(
            opaque(seg(-UNIT, 0, UNIT, 0)), opaque(v(0, UNIT)), false,
        );
    }

    #[test]
    fn gas_contains_local_point_segment() {
        let _ = contains_local_point_segment(opaque(seg(-UNIT, 0, UNIT, 0)), opaque(v(0, UNIT)));
    }

    #[test]
    fn gas_segment_point_at_edge() {
        let _ = segment_point_at(
            opaque(seg(-UNIT, 0, UNIT, 0)),
            opaque(SegmentPointLocation::OnEdge((Fixed { raw: HALF }, Fixed { raw: HALF }))),
        );
    }
}
