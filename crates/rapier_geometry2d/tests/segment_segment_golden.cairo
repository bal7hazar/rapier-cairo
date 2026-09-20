//! Every `rapier_golden::segment_segment` pair against
//! `rapier_geometry2d::closest_points`.
//!
//! Upstream takes `pos12` and transforms the second segment into the frame of the first; the
//! port's entry point expects both segments in one frame, so this file does that transform with
//! `rapier_math::pose2::Pose2` — exactly the call a narrow-phase generator will make. The
//! locations come back in the frame of each original segment, so `p2` is read off the
//! *untransformed* `seg2`, as the fixtures record it.
//!
//! Tolerances are the ones `tools/golden/README.md` documents: points and `u` 4 ulp, `dist_sq`
//! 16 ulp, locations exact unless the pair is `ambiguous` (parallel or collinear overlap, where
//! the closest pair is not unique and only the distance is comparable).

use fixed::wide::distance2_squared;
use fixed::{Fixed, ONE};
use glam::vec2::Vec2;
use rapier_geometry2d::closest_points::{
    closest_points_segment_segment, closest_points_segment_segment_with_locations,
};
use rapier_geometry2d::point::{SegmentPointLocation, segment_point_at};
use rapier_geometry2d::shape::Segment;
use rapier_golden::compare::{vec2_within, within};
use rapier_golden::segment_segment;
use rapier_golden::types::{PoseRaw, SegmentLocationRaw, SegmentPairCase, SegmentRaw, Vec2Raw};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

/// Points and barycentric coordinates: one division scaled by the segment length.
const POINT_TOLERANCE: u64 = 4;
/// `dist_sq`: two points of tolerance 4, at distances below 2.
const DIST_SQ_TOLERANCE: u64 = 16;

fn vector(v: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: v.x }, y: Fixed { raw: v.y } }
}

fn raw(v: Vec2) -> Vec2Raw {
    Vec2Raw { x: v.x.raw, y: v.y.raw }
}

fn pose(p: PoseRaw) -> Pose2 {
    Pose2Trait::new(
        vector(p.translation),
        Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    )
}

fn segment(s: SegmentRaw) -> Segment {
    Segment { a: vector(s.a), b: vector(s.b) }
}

/// `seg` expressed in the frame of segment 1.
fn transformed(s: SegmentRaw, p: Pose2) -> Segment {
    Segment { a: p.transform_point(vector(s.a)), b: p.transform_point(vector(s.b)) }
}

/// The locations and the two points of one case, `p2` in the frame of segment 2.
fn solve(case: @SegmentPairCase) -> (SegmentPointLocation, SegmentPointLocation, Vec2, Vec2) {
    let p = pose(*case.pos12);
    let seg1 = segment(*case.seg1);
    let seg2_in_1 = transformed(*case.seg2, p);
    let (loc1, loc2) = closest_points_segment_segment_with_locations(seg1, seg2_in_1);
    (loc1, loc2, segment_point_at(seg1, loc1), segment_point_at(segment(*case.seg2), loc2))
}

fn location_matches(got: SegmentPointLocation, expected: SegmentLocationRaw) -> bool {
    match expected {
        SegmentLocationRaw::NoLocation => false,
        SegmentLocationRaw::OnVertex(i) => got == SegmentPointLocation::OnVertex(i),
        SegmentLocationRaw::OnEdge(u) => match got {
            SegmentPointLocation::OnEdge((_, v)) => within(v.raw, u, POINT_TOLERANCE),
            _ => false,
        },
    }
}

#[test]
fn test_table_has_the_expected_size() {
    assert_eq!(segment_segment::cases().len(), 24);
}

/// The two closest points of every unambiguous pair, in their own frames.
#[test]
fn test_points_golden() {
    let mut checked = 0_u32;
    for case in segment_segment::cases() {
        if *case.ambiguous {
            continue;
        }
        let (_, _, p1, p2) = solve(case);
        assert!(vec2_within(raw(p1), *case.p1, POINT_TOLERANCE), "p1 {}", *case.id);
        assert!(vec2_within(raw(p2), *case.p2, POINT_TOLERANCE), "p2 {}", *case.id);
        checked += 1;
    }
    assert_eq!(checked, 17);
}

/// `|p1 - pos12 * p2|^2` of **every** pair, ambiguous ones included: the distance is the one
/// quantity a tie cannot change.
#[test]
fn test_distance_squared_golden() {
    for case in segment_segment::cases() {
        let p = pose(*case.pos12);
        let (_, _, p1, p2) = solve(case);
        let q2 = p.transform_point(p2);
        let dist_sq = distance2_squared(p1.x, p1.y, q2.x, q2.y);
        assert!(within(dist_sq.raw, *case.dist_sq, DIST_SQ_TOLERANCE), "dist_sq {}", *case.id);
    }
}

/// The locations of every unambiguous pair, and the barycentric invariant on the others.
#[test]
fn test_locations_golden() {
    for case in segment_segment::cases() {
        let (loc1, loc2, _, _) = solve(case);
        for loc in array![loc1, loc2].span() {
            match *loc {
                SegmentPointLocation::OnVertex(i) => assert!(i == 0 || i == 1, "{}", *case.id),
                SegmentPointLocation::OnEdge((
                    u, v,
                )) => { assert_eq!(u + v, ONE, "barycentric sum {}", *case.id); },
            }
        }
        if *case.ambiguous {
            continue;
        }
        assert!(location_matches(loc1, *case.loc1), "loc1 {}", *case.id);
        assert!(location_matches(loc2, *case.loc2), "loc2 {}", *case.id);
    }
}

/// The point-returning entry point agrees with the location-returning one, in frame 1.
#[test]
fn test_points_entry_point_matches_locations() {
    for case in segment_segment::cases() {
        let p = pose(*case.pos12);
        let seg1 = segment(*case.seg1);
        let seg2_in_1 = transformed(*case.seg2, p);
        let (p1, p2) = closest_points_segment_segment(seg1, seg2_in_1);
        let (loc1, loc2) = closest_points_segment_segment_with_locations(seg1, seg2_in_1);
        assert_eq!(p1, segment_point_at(seg1, loc1), "p1 {}", *case.id);
        assert_eq!(p2, segment_point_at(seg2_in_1, loc2), "p2 {}", *case.id);
    }
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_closest_points_golden_crossing() {
    let case = segment_segment::SEG_CROSSING_OBLIQUE;
    let _ = closest_points_segment_segment_with_locations(
        opaque(segment(case.seg1)), opaque(segment(case.seg2)),
    );
}

#[test]
fn gas_closest_points_golden_parallel() {
    let case = segment_segment::SEG_PARALLEL;
    let _ = closest_points_segment_segment_with_locations(
        opaque(segment(case.seg1)), opaque(segment(case.seg2)),
    );
}
