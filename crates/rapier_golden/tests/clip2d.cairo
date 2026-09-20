//! Sanity checks of the `clip2d` fixtures: clipped points lie on their segments, feature ids
//! agree with the points, order follows the projection direction. They guard the harness (wrong
//! segment, wrong feature convention), not the port.

use rapier_golden::clip2d;
use rapier_golden::compare::vec2_within;
use rapier_golden::types::{ClipCase, ClipPointRaw, ClipResultRaw, SegmentRaw, Vec2Raw};

const ONE: i128 = 0x100000000;

fn mul(a: i128, b: i128) -> i128 {
    a * b / ONE
}

fn narrow(value: i128) -> i64 {
    value.try_into().unwrap()
}

fn abs(x: i128) -> i128 {
    if x < 0 {
        -x
    } else {
        x
    }
}

/// `(p - a) x (b - a)`, in Q32.32.
fn cross(p: Vec2Raw, a: Vec2Raw, b: Vec2Raw) -> i128 {
    let (px, py): (i128, i128) = ((p.x - a.x).into(), (p.y - a.y).into());
    let (ex, ey): (i128, i128) = ((b.x - a.x).into(), (b.y - a.y).into());
    mul(px, ey) - mul(py, ex)
}

/// `(p - a) . (b - a)`, in Q32.32.
fn dot_along(p: Vec2Raw, a: Vec2Raw, b: Vec2Raw) -> i128 {
    let (px, py): (i128, i128) = ((p.x - a.x).into(), (p.y - a.y).into());
    let (ex, ey): (i128, i128) = ((b.x - a.x).into(), (b.y - a.y).into());
    mul(px, ex) + mul(py, ey)
}

/// Tolerance in ulps of a point computed as `a + (b - a) t`: a couple of ulps per unit length.
fn tolerance(segment: SegmentRaw) -> i128 {
    let (ex, ey): (i128, i128) = (
        (segment.b.x - segment.a.x).into(), (segment.b.y - segment.a.y).into(),
    );
    4 + 4 * (abs(ex) + abs(ey)) / ONE
}

/// `p` lies on the closed segment.
fn on_segment(p: Vec2Raw, segment: SegmentRaw) -> bool {
    let tol = tolerance(segment);
    let length_squared = dot_along(segment.b, segment.a, segment.b);
    let along = dot_along(p, segment.a, segment.b);
    abs(cross(p, segment.a, segment.b)) <= tol && along >= -tol && along <= length_squared + tol
}

fn check_on_segments(case: @ClipCase, result: ClipResultRaw, which: felt252) {
    if result.clipped {
        for point in result.points.span() {
            assert!(on_segment(*point.p1, *case.seg1), "p1 of {} ({})", *case.id, which);
            assert!(on_segment(*point.p2, *case.seg2), "p2 of {} ({})", *case.id, which);
        }
    }
}

fn check_features(case: @ClipCase, result: ClipResultRaw, which: felt252) {
    if result.clipped {
        for point in result.points.span() {
            let ClipPointRaw { p1, p2, f1, f2 } = *point;
            assert!(f1 <= 2 && f2 <= 2, "feature range of {} ({})", *case.id, which);
            // Feature 0 is the first vertex as passed, 2 the second one.
            if f1 == 0 {
                assert!(vec2_within(p1, *case.seg1.a, 2), "f1 = 0 of {} ({})", *case.id, which);
            }
            if f1 == 2 {
                assert!(vec2_within(p1, *case.seg1.b, 2), "f1 = 2 of {} ({})", *case.id, which);
            }
            if f2 == 0 {
                assert!(vec2_within(p2, *case.seg2.a, 2), "f2 = 0 of {} ({})", *case.id, which);
            }
            if f2 == 2 {
                assert!(vec2_within(p2, *case.seg2.b, 2), "f2 = 2 of {} ({})", *case.id, which);
            }
        }
    }
}

#[test]
fn test_table_has_the_expected_size() {
    assert_eq!(clip2d::cases().len(), 16);
}

#[test]
fn test_clipped_points_lie_on_their_segments() {
    for case in clip2d::cases() {
        check_on_segments(case, *case.plain, 'plain');
        check_on_segments(case, *case.with_normal, 'with_normal');
    }
}

#[test]
fn test_features_agree_with_the_points() {
    for case in clip2d::cases() {
        check_features(case, *case.plain, 'plain');
        check_features(case, *case.with_normal, 'with_normal');
    }
}

#[test]
fn test_unclipped_results_are_zeroed() {
    for case in clip2d::cases() {
        for result in array![*case.plain, *case.with_normal].span() {
            if !*result.clipped {
                let [a, b] = *result.points;
                assert_eq!(a.p1, Vec2Raw { x: 0, y: 0 });
                assert_eq!((a.f1, a.f2, b.f1, b.f2), (0, 0, 0, 0));
            }
        }
    }
}

#[test]
fn test_disjoint_projections_are_not_clipped() {
    for case in [clip2d::CLIP_COLLINEAR_DISJOINT, clip2d::CLIP_DISJOINT_PARALLEL].span() {
        assert!(!*case.plain.clipped, "plain {}", *case.id);
        assert!(!*case.with_normal.clipped, "with_normal {}", *case.id);
    }
}

#[test]
fn test_touching_segments_clip_to_a_single_point() {
    for case in [clip2d::CLIP_TOUCH_POINT, clip2d::CLIP_TOUCH_REVERSED].span() {
        for result in array![*case.plain, *case.with_normal].span() {
            assert!(*result.clipped, "{}", *case.id);
            let [a, b] = *result.points;
            assert!(vec2_within(a.p1, b.p1, 1), "single point on 1 ({})", *case.id);
            assert!(vec2_within(a.p2, b.p2, 1), "single point on 2 ({})", *case.id);
            assert!(vec2_within(a.p1, *case.seg1.b, 1), "at the end of segment 1 ({})", *case.id);
        }
    }
}

#[test]
fn test_plain_points_are_ordered_along_segment_one() {
    for case in clip2d::cases() {
        if *case.plain.clipped {
            let [a, b] = *case.plain.points;
            // (b - a) . (seg1.b - seg1.a) >= 0: the points come sorted along segment 1.
            let (dx, dy): (i128, i128) = ((b.p1.x - a.p1.x).into(), (b.p1.y - a.p1.y).into());
            let (ex, ey): (i128, i128) = (
                (*case.seg1.b.x - *case.seg1.a.x).into(), (*case.seg1.b.y - *case.seg1.a.y).into(),
            );
            assert!(mul(dx, ex) + mul(dy, ey) >= -4, "order of {}", *case.id);
        }
    }
}

#[test]
fn test_with_normal_points_are_ordered_along_the_tangent() {
    // The projection axis is the tangent (-n.y, n.x), not the segments: with a +y normal the first
    // point has the larger x.
    for case in clip2d::cases() {
        if *case.with_normal.clipped {
            let [a, b] = *case.with_normal.points;
            let tangent = Vec2Raw { x: -*case.normal.y, y: *case.normal.x };
            let (dx, dy): (i128, i128) = ((b.p1.x - a.p1.x).into(), (b.p1.y - a.p1.y).into());
            let (tx, ty): (i128, i128) = (tangent.x.into(), tangent.y.into());
            assert!(mul(dx, tx) + mul(dy, ty) >= -4, "order of {}", *case.id);
        }
    }
}

#[test]
fn test_partial_overlap_clips_to_the_common_range() {
    let case = clip2d::CLIP_PARALLEL_PARTIAL;
    let [a, b] = case.plain.points;
    // Segment 1 is (0,0)-(2,0), segment 2 is (1,0.25)-(3,0.25): the common range is x in [1, 2].
    assert_eq!(a.p1, Vec2Raw { x: ONE.try_into().unwrap(), y: 0 });
    assert_eq!(b.p1, Vec2Raw { x: (2 * ONE).try_into().unwrap(), y: 0 });
    assert_eq!((a.f1, a.f2, b.f1, b.f2), (1, 0, 2, 1));
    assert_eq!(a.p2.y, 0x40000000, "the incident points sit at height 0.25");
}
