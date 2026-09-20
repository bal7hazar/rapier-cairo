//! Sanity checks of the `point_projection` fixtures: a solid projection of an inside point is the
//! point, projections land on the boundary, distances match the projection, locations match the
//! projected point. They guard the harness (wrong flag, wrong frame), not the port.

use rapier_golden::compare::{vec2_within, within};
use rapier_golden::point_projection;
use rapier_golden::types::{
    PointFeatureRaw, ProjectionCase, SegmentLocationRaw, SegmentRaw, ShapeRaw, Vec2Raw,
};

const ONE: i128 = 0x100000000;

fn mul(a: i128, b: i128) -> i128 {
    a * b / ONE
}

fn narrow(value: i128) -> i64 {
    value.try_into().unwrap()
}

fn abs(x: i64) -> i64 {
    if x < 0 {
        -x
    } else {
        x
    }
}

/// `|a - b|²`.
fn distance_squared(a: Vec2Raw, b: Vec2Raw) -> i64 {
    let (dx, dy): (i128, i128) = ((a.x - b.x).into(), (a.y - b.y).into());
    narrow(mul(dx, dx) + mul(dy, dy))
}

fn is_segment(case: @ProjectionCase) -> bool {
    match *case.shape {
        ShapeRaw::Segment(_) => true,
        _ => false,
    }
}

#[test]
fn test_table_has_the_expected_size() {
    // 4 ball + 9 cuboid + 10 capsule + 10 segment.
    assert_eq!(point_projection::cases().len(), 33);
}

#[test]
fn test_solid_projection_of_an_inside_point_is_the_point() {
    let mut checked = 0_u32;
    for case in point_projection::cases() {
        if !is_segment(case) && *case.projection_solid.is_inside {
            assert_eq!(*case.projection_solid.point, *case.point, "{}", *case.id);
            checked += 1;
        }
    }
    assert_eq!(checked, 13);
}

#[test]
fn test_outside_points_project_the_same_way_solid_or_not() {
    for case in point_projection::cases() {
        if !*case.projection_solid.is_inside {
            assert_eq!(*case.projection, *case.projection_solid, "{}", *case.id);
        }
    }
}

#[test]
fn test_inside_flag_does_not_depend_on_the_solid_flag() {
    for case in point_projection::cases() {
        assert_eq!(*case.projection.is_inside, *case.projection_solid.is_inside, "{}", *case.id);
    }
}

#[test]
fn test_distance_is_the_distance_to_the_projection() {
    for case in point_projection::cases() {
        let distance = *case.distance;
        let expected = distance_squared(*case.point, *case.projection.point);
        let squared = narrow(mul(distance.into(), distance.into()));
        assert!(within(squared, expected, 16), "|distance| of {}", *case.id);
        // Negative inside (or zero), positive outside.
        if *case.projection.is_inside {
            assert!(distance <= 0, "sign of {}", *case.id);
        } else {
            assert!(distance >= 0, "sign of {}", *case.id);
        }
    }
}

#[test]
fn test_ball_projection_lies_on_the_sphere() {
    let mut checked = 0_u32;
    for case in point_projection::cases() {
        if let ShapeRaw::Ball(radius) = *case.shape {
            let r: i128 = radius.into();
            let squared = distance_squared(*case.projection.point, Vec2Raw { x: 0, y: 0 });
            assert!(within(squared, narrow(mul(r, r)), 4), "{}", *case.id);
            checked += 1;
        }
    }
    assert_eq!(checked, 4);
}

#[test]
fn test_cuboid_projection_lies_on_the_boundary() {
    let mut checked = 0_u32;
    for case in point_projection::cases() {
        if let ShapeRaw::Cuboid(half_extents) = *case.shape {
            let p = *case.projection.point;
            let on_x_face = within(abs(p.x), half_extents.x, 1) && abs(p.y) <= half_extents.y + 1;
            let on_y_face = within(abs(p.y), half_extents.y, 1) && abs(p.x) <= half_extents.x + 1;
            assert!(on_x_face || on_y_face, "{}", *case.id);
            checked += 1;
        }
    }
    assert_eq!(checked, 9);
}

#[test]
fn test_points_on_the_boundary_are_inside_and_project_to_themselves() {
    let cases = [
        point_projection::BALL_ON_BOUNDARY, point_projection::CUBOID_ON_FACE,
        point_projection::CUBOID_ON_VERTEX, point_projection::CAPSULE_ON_BOUNDARY,
        point_projection::SEGMENT_ON_INTERIOR, point_projection::SEGMENT_ON_VERTEX_A,
        point_projection::SEGMENT_ON_VERTEX_B,
    ];
    for case in cases.span() {
        assert!(*case.projection.is_inside, "{}", *case.id);
        assert!(vec2_within(*case.projection.point, *case.point, 1), "{}", *case.id);
        assert!(within(*case.distance, 0, 1), "{}", *case.id);
    }
}

#[test]
fn test_cuboid_tie_between_axes_picks_x() {
    // (0.5, 0) is 0.5 away from the +x face and from both y faces: `diff.x <= diff.y` picks x.
    let case = point_projection::CUBOID_INSIDE_TIE;
    assert_eq!(case.projection.point, Vec2Raw { x: 0x100000000, y: 0 });
    // The centre: y is nearer, and an exact zero has sign +1: the projection is on the +y face.
    let case = point_projection::CUBOID_CENTER;
    assert_eq!(case.projection.point, Vec2Raw { x: 0, y: 0x80000000 });
}

fn segment_of(case: @ProjectionCase) -> SegmentRaw {
    match *case.shape {
        ShapeRaw::Segment(segment) => segment,
        _ => panic!("segment expected"),
    }
}

#[test]
fn test_segment_location_matches_the_projected_point() {
    let mut checked = 0_u32;
    for case in point_projection::cases() {
        if is_segment(case) {
            let segment = segment_of(case);
            let p = *case.projection.point;
            match *case.location {
                SegmentLocationRaw::NoLocation => panic!("a segment reports a location"),
                SegmentLocationRaw::OnVertex(0) => assert_eq!(p, segment.a, "{}", *case.id),
                SegmentLocationRaw::OnVertex(_) => assert_eq!(p, segment.b, "{}", *case.id),
                SegmentLocationRaw::OnEdge(u) => {
                    let (ex, ey): (i128, i128) = (
                        (segment.b.x - segment.a.x).into(), (segment.b.y - segment.a.y).into(),
                    );
                    let u: i128 = u.into();
                    let expected = Vec2Raw {
                        x: segment.a.x + narrow(mul(ex, u)), y: segment.a.y + narrow(mul(ey, u)),
                    };
                    assert!(vec2_within(p, expected, 4), "{}", *case.id);
                },
            }
            checked += 1;
        } else {
            assert_eq!(*case.location, SegmentLocationRaw::NoLocation, "{}", *case.id);
        }
    }
    assert_eq!(checked, 10);
}

#[test]
fn test_segment_features_follow_the_location() {
    for case in point_projection::cases() {
        if is_segment(case) {
            match (*case.location, *case.feature) {
                (
                    SegmentLocationRaw::OnVertex(i), PointFeatureRaw::Vertex(j),
                ) => assert_eq!(i, j, "{}", *case.id),
                (
                    SegmentLocationRaw::OnEdge(_), PointFeatureRaw::Face(side),
                ) => assert!(side <= 1, "{}", *case.id),
                _ => panic!("feature and location disagree for {}", *case.id),
            }
        }
    }
}

#[test]
fn test_segment_face_id_is_the_side_of_the_point() {
    // (0.3, 0.5) is left of a segment running along +x (perp_dot(dp, dir) < 0 -> Face(1)).
    assert_eq!(point_projection::SEGMENT_ABOVE.feature, PointFeatureRaw::Face(1));
    assert_eq!(point_projection::SEGMENT_BELOW.feature, PointFeatureRaw::Face(0));
    // On the segment perp_dot == 0: Face(0).
    assert_eq!(point_projection::SEGMENT_ON_INTERIOR.feature, PointFeatureRaw::Face(0));
}

#[test]
fn test_ball_and_capsule_always_report_face_zero() {
    for case in point_projection::cases() {
        match *case.shape {
            ShapeRaw::Ball(_) |
            ShapeRaw::Capsule(_) => assert_eq!(
                *case.feature, PointFeatureRaw::Face(0), "{}", *case.id,
            ),
            _ => {},
        }
    }
}

#[test]
fn test_capsule_projection_is_a_radius_away_from_the_core_segment() {
    // The capsule (0,-0.5)-(0,0.5) of radius 0.25: every non-solid projection of the vertical
    // capsule is 0.25 away from the core, i.e. |x| = 0.25 on the side region.
    for case in [
        point_projection::CAPSULE_OUTSIDE_SIDE, point_projection::CAPSULE_INSIDE,
        point_projection::CAPSULE_ON_BOUNDARY, point_projection::CAPSULE_ON_SEGMENT,
    ]
        .span() {
        assert_eq!(*case.projection.point.x, 0x40000000, "{}", *case.id);
    }
}
