//! Every `rapier_golden::point_projection` case against `rapier_geometry2d::point`.
//!
//! Tolerances are the ones `tools/golden/README.md` documents for this family: the cuboid
//! projection is exact, the segment is 4 ulp (one division) with `u` at 2 ulp, the ball 4 ulp,
//! the capsule 8 ulp (it normalises), every `distance` 8 ulp, and every discrete output
//! (`is_inside`, feature, location kind) is exact.

use fixed::{Fixed, ZERO};
use glam::vec2::Vec2;
use rapier_geometry2d::feature_id::{FEATURE_UNKNOWN, FeatureId, FeatureIdTrait};
use rapier_geometry2d::point::{
    PointProjection, SegmentPointLocation, contains_local_point_ball, contains_local_point_capsule,
    contains_local_point_cuboid, contains_local_point_segment, distance_to_local_point_ball,
    distance_to_local_point_capsule, distance_to_local_point_cuboid,
    distance_to_local_point_segment, project_local_point_and_get_feature_ball,
    project_local_point_and_get_feature_capsule, project_local_point_and_get_feature_cuboid,
    project_local_point_and_get_feature_segment, project_local_point_and_get_location_segment,
    project_local_point_ball, project_local_point_capsule, project_local_point_cuboid,
    project_local_point_segment,
};
use rapier_geometry2d::shape::{Ball, Capsule, Cuboid, Segment};
use rapier_golden::compare::{vec2_within, within};
use rapier_golden::point_projection;
use rapier_golden::types::{PointFeatureRaw, ProjectionCase, SegmentLocationRaw, ShapeRaw, Vec2Raw};
use rapier_testing::opaque;

const POINT_TOLERANCE: u64 = 8;
const U_TOLERANCE: u64 = 2;
const DISTANCE_TOLERANCE: u64 = 8;

fn vector(v: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: v.x }, y: Fixed { raw: v.y } }
}

fn raw(v: Vec2) -> Vec2Raw {
    Vec2Raw { x: v.x.raw, y: v.y.raw }
}

/// The per-shape projection tolerance of `tools/golden/README.md`.
fn tolerance_of(shape: ShapeRaw) -> u64 {
    match shape {
        ShapeRaw::Ball(_) => 4,
        ShapeRaw::Cuboid(_) => 1,
        ShapeRaw::Capsule(_) => 8,
        ShapeRaw::HalfSpace(_) => 4,
        ShapeRaw::Segment(_) => 4,
    }
}

/// `project_local_point` of the shape of `case`.
fn project(case: @ProjectionCase, solid: bool) -> PointProjection {
    let pt = vector(*case.point);
    match *case.shape {
        ShapeRaw::Ball(r) => project_local_point_ball(Ball { radius: Fixed { raw: r } }, pt, solid),
        ShapeRaw::Cuboid(he) => project_local_point_cuboid(
            Cuboid { half_extents: vector(he) }, pt, solid,
        ),
        ShapeRaw::Capsule(c) => project_local_point_capsule(
            Capsule {
                segment: Segment { a: vector(c.a), b: vector(c.b) },
                radius: Fixed { raw: c.radius },
            },
            pt,
            solid,
        ),
        ShapeRaw::HalfSpace(_) => panic!("no half space in this family"),
        ShapeRaw::Segment(s) => project_local_point_segment(
            Segment { a: vector(s.a), b: vector(s.b) }, pt, solid,
        ),
    }
}

/// `distance_to_local_point` of the shape of `case`.
fn distance(case: @ProjectionCase, solid: bool) -> Fixed {
    let pt = vector(*case.point);
    match *case.shape {
        ShapeRaw::Ball(r) => distance_to_local_point_ball(
            Ball { radius: Fixed { raw: r } }, pt, solid,
        ),
        ShapeRaw::Cuboid(he) => distance_to_local_point_cuboid(
            Cuboid { half_extents: vector(he) }, pt, solid,
        ),
        ShapeRaw::Capsule(c) => distance_to_local_point_capsule(
            Capsule {
                segment: Segment { a: vector(c.a), b: vector(c.b) },
                radius: Fixed { raw: c.radius },
            },
            pt,
            solid,
        ),
        ShapeRaw::HalfSpace(_) => panic!("no half space in this family"),
        ShapeRaw::Segment(s) => distance_to_local_point_segment(
            Segment { a: vector(s.a), b: vector(s.b) }, pt, solid,
        ),
    }
}

/// `contains_local_point` of the shape of `case`.
fn contains(case: @ProjectionCase) -> bool {
    let pt = vector(*case.point);
    match *case.shape {
        ShapeRaw::Ball(r) => contains_local_point_ball(Ball { radius: Fixed { raw: r } }, pt),
        ShapeRaw::Cuboid(he) => contains_local_point_cuboid(
            Cuboid { half_extents: vector(he) }, pt,
        ),
        ShapeRaw::Capsule(c) => contains_local_point_capsule(
            Capsule {
                segment: Segment { a: vector(c.a), b: vector(c.b) },
                radius: Fixed { raw: c.radius },
            },
            pt,
        ),
        ShapeRaw::HalfSpace(_) => panic!("no half space in this family"),
        ShapeRaw::Segment(s) => contains_local_point_segment(
            Segment { a: vector(s.a), b: vector(s.b) }, pt,
        ),
    }
}

/// `project_local_point_and_get_feature` of the shape of `case`.
fn feature(case: @ProjectionCase) -> FeatureId {
    let pt = vector(*case.point);
    let (_, id) = match *case.shape {
        ShapeRaw::Ball(r) => project_local_point_and_get_feature_ball(
            Ball { radius: Fixed { raw: r } }, pt,
        ),
        ShapeRaw::Cuboid(he) => project_local_point_and_get_feature_cuboid(
            Cuboid { half_extents: vector(he) }, pt,
        ),
        ShapeRaw::Capsule(c) => project_local_point_and_get_feature_capsule(
            Capsule {
                segment: Segment { a: vector(c.a), b: vector(c.b) },
                radius: Fixed { raw: c.radius },
            },
            pt,
        ),
        ShapeRaw::HalfSpace(_) => panic!("no half space in this family"),
        ShapeRaw::Segment(s) => project_local_point_and_get_feature_segment(
            Segment { a: vector(s.a), b: vector(s.b) }, pt,
        ),
    };
    id
}

fn expected_feature(expected: PointFeatureRaw) -> FeatureId {
    match expected {
        PointFeatureRaw::Unknown => FEATURE_UNKNOWN,
        PointFeatureRaw::Vertex(code) => FeatureIdTrait::vertex(code),
        PointFeatureRaw::Face(code) => FeatureIdTrait::face(code),
    }
}

#[test]
fn test_table_has_the_expected_size() {
    assert_eq!(point_projection::cases().len(), 33);
}

/// Both projections of every case: the point within the per-shape tolerance, `is_inside` exact.
#[test]
fn test_projection_golden() {
    for case in point_projection::cases() {
        let tolerance = tolerance_of(*case.shape);
        let hollow = project(case, false);
        assert!(
            vec2_within(raw(hollow.point), *case.projection.point, tolerance),
            "hollow point {}",
            *case.id,
        );
        assert_eq!(hollow.is_inside, *case.projection.is_inside, "hollow inside {}", *case.id);
        let solid = project(case, true);
        assert!(
            vec2_within(raw(solid.point), *case.projection_solid.point, tolerance),
            "solid point {}",
            *case.id,
        );
        assert_eq!(solid.is_inside, *case.projection_solid.is_inside, "solid inside {}", *case.id);
    }
}

/// `distance_to_local_point(pt, false)` of every case, and the `solid = true` invariants.
#[test]
fn test_distance_golden() {
    for case in point_projection::cases() {
        let hollow = distance(case, false);
        assert!(within(hollow.raw, *case.distance, DISTANCE_TOLERANCE), "distance {}", *case.id);
        // `solid = true` never reports a negative distance, and agrees outside.
        let solid = distance(case, true);
        assert!(solid >= ZERO, "solid distance sign {}", *case.id);
        if !*case.projection.is_inside {
            assert!(within(solid.raw, *case.distance, DISTANCE_TOLERANCE), "solid {}", *case.id);
        }
        // `contains_local_point` is the `is_inside` of the solid projection.
        assert_eq!(contains(case), *case.projection_solid.is_inside, "contains {}", *case.id);
    }
}

/// The feature of every case, exactly.
#[test]
fn test_feature_golden() {
    for case in point_projection::cases() {
        assert_eq!(feature(case), expected_feature(*case.feature), "feature {}", *case.id);
    }
}

/// The segment location of every segment case, with `u` within 2 ulp.
#[test]
fn test_segment_location_golden() {
    let mut checked = 0_u32;
    for case in point_projection::cases() {
        let s = match *case.shape {
            ShapeRaw::Segment(s) => s,
            _ => { continue; },
        };
        let seg = Segment { a: vector(s.a), b: vector(s.b) };
        let (proj, location) = project_local_point_and_get_location_segment(
            seg, vector(*case.point), false,
        );
        assert!(
            vec2_within(raw(proj.point), *case.projection.point, POINT_TOLERANCE),
            "location point {}",
            *case.id,
        );
        match *case.location {
            SegmentLocationRaw::NoLocation => panic!("a segment case must carry a location"),
            SegmentLocationRaw::OnVertex(i) => assert_eq!(
                location, SegmentPointLocation::OnVertex(i), "vertex {}", *case.id,
            ),
            SegmentLocationRaw::OnEdge(u) => {
                match location {
                    SegmentPointLocation::OnEdge((
                        one_minus_u, v,
                    )) => {
                        assert!(within(v.raw, u, U_TOLERANCE), "u {}", *case.id);
                        // The two barycentric coordinates sum to exactly 1.
                        assert_eq!(one_minus_u.raw + v.raw, 0x1_0000_0000, "sum {}", *case.id);
                    },
                    _ => panic!("expected an edge location"),
                }
            },
        }
        checked += 1;
    }
    assert_eq!(checked, 10);
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_project_ball_golden() {
    let case = point_projection::BALL_OUTSIDE;
    let _ = project_local_point_ball(
        opaque(Ball { radius: Fixed { raw: 0x8000_0000 } }), opaque(vector(case.point)), false,
    );
}

#[test]
fn gas_project_cuboid_golden() {
    let _ = project_local_point_cuboid(
        opaque(Cuboid { half_extents: vector(Vec2Raw { x: 0x1_0000_0000, y: 0x8000_0000 }) }),
        opaque(vector(point_projection::CUBOID_OUTSIDE_VERTEX.point)),
        false,
    );
}
