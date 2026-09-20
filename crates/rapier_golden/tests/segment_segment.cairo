//! Sanity checks of the `segment_segment` fixtures: the squared distance is the distance between
//! the two closest points, locations match the points, the answer is symmetric, and no closest
//! distance exceeds an end-point distance. They guard the harness (wrong frame, wrong point),
//! not the port.

use rapier_golden::compare::{vec2_within, within};
use rapier_golden::segment_segment;
use rapier_golden::types::{PoseRaw, SegmentLocationRaw, SegmentPairCase, SegmentRaw, Vec2Raw};

const ONE: i128 = 0x100000000;

fn mul(a: i128, b: i128) -> i128 {
    a * b / ONE
}

fn narrow(value: i128) -> i64 {
    value.try_into().unwrap()
}

/// `pose · p`.
fn apply(pose: PoseRaw, p: Vec2Raw) -> Vec2Raw {
    let (re, im): (i128, i128) = (pose.rotation.re.into(), pose.rotation.im.into());
    let (x, y): (i128, i128) = (p.x.into(), p.y.into());
    Vec2Raw {
        x: pose.translation.x + narrow(mul(re, x) - mul(im, y)),
        y: pose.translation.y + narrow(mul(im, x) + mul(re, y)),
    }
}

/// `|a - b|²`.
fn distance_squared(a: Vec2Raw, b: Vec2Raw) -> i64 {
    let (dx, dy): (i128, i128) = ((a.x - b.x).into(), (a.y - b.y).into());
    narrow(mul(dx, dx) + mul(dy, dy))
}

fn min(a: i64, b: i64) -> i64 {
    if a < b {
        a
    } else {
        b
    }
}

/// The point of `segment` a location designates.
fn point_at(segment: SegmentRaw, location: SegmentLocationRaw) -> Vec2Raw {
    match location {
        SegmentLocationRaw::NoLocation => panic!("a segment location is expected"),
        SegmentLocationRaw::OnVertex(0) => segment.a,
        SegmentLocationRaw::OnVertex(_) => segment.b,
        SegmentLocationRaw::OnEdge(u) => {
            let (ex, ey): (i128, i128) = (
                (segment.b.x - segment.a.x).into(), (segment.b.y - segment.a.y).into(),
            );
            let u: i128 = u.into();
            Vec2Raw { x: segment.a.x + narrow(mul(ex, u)), y: segment.a.y + narrow(mul(ey, u)) }
        },
    }
}

#[test]
fn test_table_has_the_expected_size() {
    // 15 pairs at the identity + 3 posed + 6 swapped copies.
    assert_eq!(segment_segment::cases().len(), 24);
}

#[test]
fn test_locations_designate_the_reported_points() {
    for case in segment_segment::cases() {
        assert!(vec2_within(point_at(*case.seg1, *case.loc1), *case.p1, 4), "p1 of {}", *case.id);
        assert!(vec2_within(point_at(*case.seg2, *case.loc2), *case.p2, 4), "p2 of {}", *case.id);
    }
}

#[test]
fn test_squared_distance_is_measured_between_p1_and_the_posed_p2() {
    for case in segment_segment::cases() {
        let expected = distance_squared(*case.p1, apply(*case.pos12, *case.p2));
        assert!(within(*case.dist_sq, expected, 16), "{}", *case.id);
    }
}

#[test]
fn test_closest_distance_never_exceeds_an_end_point_distance() {
    for case in segment_segment::cases() {
        let ends1 = array![*case.seg1.a, *case.seg1.b];
        let ends2 = array![apply(*case.pos12, *case.seg2.a), apply(*case.pos12, *case.seg2.b)];
        let mut shortest = distance_squared(*ends1.at(0), *ends2.at(0));
        for a in ends1.span() {
            for b in ends2.span() {
                shortest = min(shortest, distance_squared(*a, *b));
            }
        }
        assert!(*case.dist_sq <= shortest + 16, "{}", *case.id);
    }
}

fn assert_same_distance(a: SegmentPairCase, b: SegmentPairCase) {
    assert_eq!(a.seg1, b.seg2, "segments of {}", a.id);
    assert_eq!(a.seg2, b.seg1, "segments of {}", a.id);
    assert!(within(a.dist_sq, b.dist_sq, 2), "distance of {}", a.id);
    // The closest points are swapped too (no pose between the segments).
    assert!(vec2_within(a.p1, b.p2, 2), "points of {}", a.id);
    assert!(vec2_within(a.p2, b.p1, 2), "points of {}", a.id);
}

#[test]
fn test_distance_is_symmetric() {
    assert_same_distance(
        segment_segment::SEG_CROSSING_OBLIQUE, segment_segment::SEG_CROSSING_OBLIQUE_SWAP,
    );
    assert_same_distance(segment_segment::SEG_PARALLEL, segment_segment::SEG_PARALLEL_SWAP);
    assert_same_distance(
        segment_segment::SEG_COLLINEAR_OVERLAP, segment_segment::SEG_COLLINEAR_OVERLAP_SWAP,
    );
    assert_same_distance(
        segment_segment::SEG_ENDPOINT_INTERIOR, segment_segment::SEG_ENDPOINT_INTERIOR_SWAP,
    );
    assert_same_distance(
        segment_segment::SEG_ENDPOINT_ENDPOINT, segment_segment::SEG_ENDPOINT_ENDPOINT_SWAP,
    );
    assert_same_distance(segment_segment::SEG_SKEW_BEYOND, segment_segment::SEG_SKEW_BEYOND_SWAP);
}

#[test]
fn test_touching_and_crossing_segments_have_a_zero_distance() {
    let cases = [
        segment_segment::SEG_CROSSING, segment_segment::SEG_CROSSING_OBLIQUE,
        segment_segment::SEG_COLLINEAR_OVERLAP, segment_segment::SEG_COLLINEAR_TOUCHING,
        segment_segment::SEG_COLLINEAR_SAME, segment_segment::SEG_POSE_ROT30,
    ];
    for case in cases.span() {
        assert!(within(*case.dist_sq, 0, 2), "{}", *case.id);
    }
}

#[test]
fn test_closed_forms_of_simple_configurations() {
    // Parallel at height 1: distance 1.
    assert!(within(segment_segment::SEG_PARALLEL.dist_sq, 0x100000000, 1));
    // T configuration: the end point (0.3, 0.5) is 0.5 above the interior of segment 1.
    assert!(within(segment_segment::SEG_ENDPOINT_INTERIOR.dist_sq, 0x40000000, 1));
    // (1, 0) to (2, 1).
    assert!(within(segment_segment::SEG_ENDPOINT_ENDPOINT.dist_sq, 0x200000000, 1));
    // Zero-length second segment at (0.3, 0.7): 0.7 above the interior of segment 1.
    assert!(within(segment_segment::SEG_ZERO_LEN_SECOND.dist_sq, 0x7d70a3d7, 4));
    // Two points (0, 0) and (1, 1).
    assert!(within(segment_segment::SEG_BOTH_ZERO_LEN.dist_sq, 0x200000000, 1));
}

#[test]
fn test_quarter_turn_poses_are_exact() {
    // Segment 2 becomes horizontal at height 2: distance 2 from segment 1.
    assert!(within(segment_segment::SEG_POSE_QUARTER.dist_sq, 0x400000000, 1));
    // Segment 2 becomes vertical at x = 1.5: 0.5 from the end of segment 1.
    assert!(within(segment_segment::SEG_POSE_QUARTER_VERTICAL.dist_sq, 0x40000000, 1));
}

#[test]
fn test_parallel_ties_are_resolved_at_the_start_of_the_second_segment() {
    // Parallel segments: the closest pair is not unique. Upstream starts from s = 0, projects onto
    // segment 2 and clamps: it answers the point of segment 2 with the smallest x, whichever end
    // point of segment 2 that is, and its projection on segment 1.
    for case in [segment_segment::SEG_PARALLEL, segment_segment::SEG_PARALLEL_REVERSED].span() {
        assert_eq!(*case.p1, Vec2Raw { x: -0x80000000, y: 0 }, "p1 of {}", *case.id);
        assert_eq!(*case.p2, Vec2Raw { x: -0x80000000, y: 0x100000000 }, "p2 of {}", *case.id);
    }
    assert_eq!(segment_segment::SEG_PARALLEL.loc2, SegmentLocationRaw::OnVertex(0));
    assert_eq!(segment_segment::SEG_PARALLEL_REVERSED.loc2, SegmentLocationRaw::OnVertex(1));
}
