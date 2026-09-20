//! Sanity checks of the `sat2d` fixtures: unit axes, the sign of the separation in each regime,
//! closed forms for axis-aligned cuboids and the consistency of the two directions. They guard
//! the harness (wrong pose, wrong direction, wrong sign), not the port.

use rapier_golden::compare::{vec2_within, within};
use rapier_golden::sat2d;
use rapier_golden::types::{PoseRaw, SatCase, SatOperandRaw, Vec2Raw};

const ONE: i128 = 0x100000000;

fn mul(a: i128, b: i128) -> i128 {
    a * b / ONE
}

fn narrow(value: i128) -> i64 {
    value.try_into().unwrap()
}

fn norm_squared(v: Vec2Raw) -> i64 {
    narrow(mul(v.x.into(), v.x.into()) + mul(v.y.into(), v.y.into()))
}

fn max(a: i64, b: i64) -> i64 {
    if a > b {
        a
    } else {
        b
    }
}

fn half_extents(shape: SatOperandRaw) -> Vec2Raw {
    match shape {
        SatOperandRaw::Cuboid(h) => h,
        _ => panic!("cuboid expected"),
    }
}

/// The separation of the pair along its best axis: SAT reports the larger of the two directions.
fn best(case: @SatCase) -> i64 {
    max(*case.sep1.separation, *case.sep2.separation)
}

#[test]
fn test_table_has_the_expected_size() {
    // 9 cuboid-cuboid + 6 cuboid-segment + 7 cuboid-triangle.
    assert_eq!(sat2d::cases().len(), 22);
}

#[test]
fn test_axes_are_unit_vectors() {
    for case in sat2d::cases() {
        assert!(within(norm_squared(*case.sep1.axis), 0x100000000, 8), "axis 1 of {}", *case.id);
        assert!(within(norm_squared(*case.sep2.axis), 0x100000000, 8), "axis 2 of {}", *case.id);
    }
}

#[test]
fn test_pos21_is_the_inverse_of_pos12() {
    for case in sat2d::cases() {
        let PoseRaw { translation: t12, rotation: r12 } = *case.pos12;
        let PoseRaw { translation: t21, rotation: r21 } = *case.pos21;
        // The rotation conjugate is exact.
        assert_eq!((r21.re, r21.im), (r12.re, -r12.im), "rotation of {}", *case.id);
        // t21 = -R21 t12, snapped: within a couple of ulps of the fixed-point evaluation.
        let (re, im): (i128, i128) = (r21.re.into(), r21.im.into());
        let (x, y): (i128, i128) = (t12.x.into(), t12.y.into());
        let expected = Vec2Raw {
            x: narrow(-(mul(re, x) - mul(im, y))), y: narrow(-(mul(im, x) + mul(re, y))),
        };
        assert!(vec2_within(t21, expected, 4), "translation of {}", *case.id);
    }
}

#[test]
fn test_separated_and_within_prediction_cases_have_a_positive_best_separation() {
    let cases = [
        sat2d::CUBOID_CUBOID_SEPARATED, sat2d::CUBOID_CUBOID_WITHIN_PRED,
        sat2d::CUBOID_CUBOID_SEP_DIAGONAL, sat2d::CUBOID_SEGMENT_SEPARATED,
        sat2d::CUBOID_SEGMENT_WITHIN_PRED, sat2d::CUBOID_TRIANGLE_SEPARATED,
        sat2d::CUBOID_TRIANGLE_WITHIN_PRED,
    ];
    for case in cases.span() {
        assert!(best(case) > 0, "{}", *case.id);
    }
}

#[test]
fn test_touching_cases_have_a_zero_best_separation() {
    let cases = [
        sat2d::CUBOID_CUBOID_TOUCHING, sat2d::CUBOID_CUBOID_DEGEN_CORNER,
        sat2d::CUBOID_SEGMENT_TOUCHING, sat2d::CUBOID_TRIANGLE_TOUCHING,
        sat2d::CUBOID_TRIANGLE_DEGEN_EDGE,
    ];
    for case in cases.span() {
        assert!(within(best(case), 0, 4), "{}", *case.id);
    }
}

#[test]
fn test_penetrating_cases_have_a_negative_best_separation() {
    let cases = [
        sat2d::CUBOID_CUBOID_SHALLOW, sat2d::CUBOID_CUBOID_DEEP, sat2d::CUBOID_CUBOID_DEGENERATE,
        sat2d::CUBOID_CUBOID_DEGEN_ROT90, sat2d::CUBOID_SEGMENT_SHALLOW, sat2d::CUBOID_SEGMENT_DEEP,
        sat2d::CUBOID_SEGMENT_DEGENERATE, sat2d::CUBOID_TRIANGLE_SHALLOW,
        sat2d::CUBOID_TRIANGLE_DEEP, sat2d::CUBOID_TRIANGLE_DEGENERATE,
    ];
    for case in cases.span() {
        assert!(best(case) < 0, "{}", *case.id);
    }
}

#[test]
fn test_axis_aligned_cuboids_report_the_gap_along_the_dominant_axis() {
    // One positive axis only: the separation is `|t| - h1 - h2` on that axis.
    let cases = [
        sat2d::CUBOID_CUBOID_SEPARATED, sat2d::CUBOID_CUBOID_WITHIN_PRED,
        sat2d::CUBOID_CUBOID_TOUCHING,
    ];
    for case in cases.span() {
        let h1 = half_extents(*case.shape1);
        let h2 = half_extents(*case.shape2);
        let expected = *case.pos12.translation.x - h1.x - h2.x;
        assert!(within(*case.sep1.separation, expected, 1), "sep1 of {}", *case.id);
        assert!(within(*case.sep2.separation, expected, 1), "sep2 of {}", *case.id);
        assert_eq!(*case.sep1.axis, Vec2Raw { x: 0x100000000, y: 0 }, "axis 1 of {}", *case.id);
        assert_eq!(*case.sep2.axis, Vec2Raw { x: -0x100000000, y: 0 }, "axis 2 of {}", *case.id);
    }
}

#[test]
fn test_axis_aligned_cuboids_agree_in_both_directions() {
    // Same four face normals seen from either box: the separations coincide.
    let cases = [
        sat2d::CUBOID_CUBOID_SEPARATED, sat2d::CUBOID_CUBOID_WITHIN_PRED,
        sat2d::CUBOID_CUBOID_TOUCHING, sat2d::CUBOID_CUBOID_DEGENERATE,
        sat2d::CUBOID_CUBOID_DEGEN_CORNER, sat2d::CUBOID_CUBOID_SEP_DIAGONAL,
    ];
    for case in cases.span() {
        assert!(within(*case.sep1.separation, *case.sep2.separation, 4), "{}", *case.id);
    }
}

#[test]
fn test_separation_along_two_positive_axes_is_the_corner_distance() {
    // `sep_diagonal`: gaps (1.0, 0.8) along x and y. Upstream does not return the larger gap but
    // the distance along the diagonal built from both, i.e. the corner-to-corner distance.
    let case = sat2d::CUBOID_CUBOID_SEP_DIAGONAL;
    let h1 = half_extents(case.shape1);
    let h2 = half_extents(case.shape2);
    let gap_x: i128 = (case.pos12.translation.x - h1.x - h2.x).into();
    let gap_y: i128 = (case.pos12.translation.y - h1.y - h2.y).into();
    assert!(gap_x > 0 && gap_y > 0, "both gaps are positive");
    let separation: i128 = case.sep1.separation.into();
    assert!(
        within(
            narrow(mul(separation, separation)), narrow(mul(gap_x, gap_x) + mul(gap_y, gap_y)), 8,
        ),
    );
    // The axis is parallel to (gap_x, gap_y): cross product ~ 0.
    let (ax, ay): (i128, i128) = (case.sep1.axis.x.into(), case.sep1.axis.y.into());
    assert!(within(narrow(mul(ax, gap_y) - mul(ay, gap_x)), 0, 4));
    assert!(case.sep1.separation > narrow(gap_x), "larger than the best single axis");
}

#[test]
fn test_ties_keep_the_first_axis() {
    // Cuboid vs a vertical segment through its centre: the -x and +x faces tie, the strict `>` of
    // upstream keeps the first one tested (-x). The segment normal is oriented towards the
    // cuboid, whose sign of an exact zero picks +x.
    let case = sat2d::CUBOID_SEGMENT_DEGENERATE;
    assert!(case.ambiguous);
    assert_eq!(case.sep1.axis, Vec2Raw { x: -0x100000000, y: 0 });
    assert_eq!(case.sep2.axis, Vec2Raw { x: 0x100000000, y: 0 });
    // Coincident cuboids: the tested axes are oriented by the sign of an exact zero (+).
    let case = sat2d::CUBOID_CUBOID_DEGENERATE;
    assert!(case.ambiguous);
    assert_eq!(case.sep1.axis, Vec2Raw { x: 0, y: 0x100000000 });
    assert_eq!(case.sep2.axis, Vec2Raw { x: 0, y: 0x100000000 });
}

#[test]
fn test_segment_normal_is_perpendicular_to_the_segment() {
    // sep2 of a cuboid-segment case tests the single normal of the segment (in the segment frame).
    for case in sat2d::cases() {
        if let SatOperandRaw::Segment(segment) = *case.shape2 {
            let (dx, dy): (i128, i128) = (
                (segment.b.x - segment.a.x).into(), (segment.b.y - segment.a.y).into(),
            );
            let (nx, ny): (i128, i128) = ((*case.sep2.axis.x).into(), (*case.sep2.axis.y).into());
            assert!(within(narrow(mul(dx, nx) + mul(dy, ny)), 0, 4), "{}", *case.id);
        }
    }
}
