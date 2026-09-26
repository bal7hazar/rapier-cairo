//! `rapier_geometry2d::query::{distance, closest_points, contact}` against Parry's top-level
//! queries (`rapier_golden::shape_queries`, 81 cases: every pair of the closed set over four
//! regimes, plus the unsupported half-space pair; 3 margins and 2 predictions each).
//!
//! # Bands
//!
//! * Scalars (`distance`, a contact's `dist`): `EXACT` raw where upstream is analytic, `ITER` where
//!   it runs GJK / EPA (largest measured gap: 69 raw, `polygon_capsule/separated`, GJK on a
//!   rounded shape).
//! * Every point this port answers must be a witness: on its shape's surface within `SURFACE`, and
//!   `point2 - point1` equal to `dist * normal1` (contact) or of length `distance` (closest points)
//!   within `SURFACE`. Upstream's points must match within `EXACT`, or, where the answer is not
//!   unique (parallel faces, symmetric penetrations, GJK's inexact witnesses on a rounded
//!   shape), lie on their surfaces within `ITER` (counted in `alternative_points`; EPA's
//!   witnesses are not always a consistent pair: `polygon_segment/touching` answers two points
//!   two units apart at `dist = 0`).
//! * Normals match within `NORMAL` (`ITER` for GJK / EPA) or, for a symmetric penetration,
//! upstream's
//!   normal must separate the shapes by the same `dist` (counted in `alternative_normals`).
//! * Kind boundaries: touching pairs that GJK answers `WithinMargin` at distance ~0 are
//!   `Intersecting` here, a pair at 1 ulp of the margin may land on either side, and GJK / EPA
//!   miss the touching contact of `segment_capsule/touching`; two shapes touching at one point
//!   have a cone of valid normals, where EPA's may be any (counted in `boundary`).
//!
//! Every case is also run with the shapes swapped: the distance must not change and the contact
//! must be the flipped one (within `SURFACE`).
use fixed::wide::norm2;
use fixed::{Fixed, FixedTrait, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_geometry2d::point::PointQuery;
use rapier_geometry2d::query::support_map::local_support_point_toward;
use rapier_geometry2d::query::{
    ClosestPoints, Contact, ContactTrait, closest_points, contact, distance,
};
use rapier_geometry2d::shape::{
    Ball, Capsule, ConvexPolygonTrait, Cuboid, HalfSpace, Segment, Shape,
};
use rapier_golden::generated::shape_queries;
use rapier_golden::types::{
    ClosestPointsRaw, ContactAnswerRaw, PolygonContactShapeRaw, PoseRaw, ShapeQueryCase, ShapeRaw,
    Vec2Raw,
};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::{Rot2, Rot2Trait};

const EXACT: i64 = 4;
const ITER: i64 = 128;
const SURFACE: i64 = 16;
/// Normals of the analytic kernels: a projection rounded by one ulp at distance `d` turns the
/// normal by `~1 ulp / d` (10 raw at `d = 0.1`, `ball_segment/contained`).
const NORMAL: i64 = 32;

fn v(p: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: p.x }, y: Fixed { raw: p.y } }
}

fn pose(p: PoseRaw) -> Pose2 {
    Pose2 {
        translation: v(p.translation),
        rotation: Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    }
}

fn shape(s: PolygonContactShapeRaw) -> Shape {
    match s {
        PolygonContactShapeRaw::Polygon(p) => {
            let mut vertices = array![];
            let mut i = 0;
            while i != p.count {
                vertices.append(v(*p.vertices.span().at(i.into())));
                i += 1;
            }
            Shape::ConvexPolygon(
                BoxTrait::new(ConvexPolygonTrait::from_convex_polyline(vertices.span()).unwrap()),
            )
        },
        PolygonContactShapeRaw::Other(s) => match s {
            ShapeRaw::Ball(r) => Shape::Ball(Ball { radius: Fixed { raw: r } }),
            ShapeRaw::Cuboid(h) => Shape::Cuboid(Cuboid { half_extents: v(h) }),
            ShapeRaw::Segment(s) => Shape::Segment(Segment { a: v(s.a), b: v(s.b) }),
            ShapeRaw::Capsule(c) => Shape::Capsule(
                Capsule {
                    segment: Segment { a: v(c.a), b: v(c.b) }, radius: Fixed { raw: c.radius },
                },
            ),
            ShapeRaw::HalfSpace(n) => Shape::HalfSpace(HalfSpace { normal: v(n) }),
        },
    }
}

fn gap(a: Fixed, b: Fixed) -> i64 {
    (a - b).abs().raw
}

fn close(a: Fixed, b: Fixed, tol: i64) -> bool {
    gap(a, b) <= tol
}

fn close2(a: Vec2, b: Vec2, tol: i64) -> bool {
    close(a.x, b.x, tol) && close(a.y, b.y, tol)
}

fn on_surface(s: Shape, p: Pose2, pt: Vec2, tol: i64) -> bool {
    close(s.distance_to_point(p, pt, false), ZERO, tol)
}

fn is_halfspace(s: Shape) -> bool {
    match s {
        Shape::HalfSpace(_) => true,
        _ => false,
    }
}

/// `min(n . shape2) - max(n . shape1)` in world space (support-map shapes only).
fn separation(s1: Shape, p1: Pose2, s2: Shape, p2: Pose2, n: Vec2) -> Fixed {
    let far1 = p1.transform_point(local_support_point_toward(s1, p1.rotation.inverse_rotate(n)));
    let near2 = p2.transform_point(local_support_point_toward(s2, p2.rotation.inverse_rotate(-n)));
    n.dot(near2) - n.dot(far1)
}

#[derive(Copy, Drop, Default, Debug, PartialEq)]
struct Counts {
    cases: u32,
    unsupported: u32,
    alternative_points: u32,
    alternative_normals: u32,
    boundary: u32,
}

fn check_closest(
    case: @ShapeQueryCase,
    s1: Shape,
    p1: Pose2,
    s2: Shape,
    p2: Pose2,
    dist: Fixed,
    expected: ClosestPointsRaw,
    ref counts: Counts,
) {
    let tol = if *case.iterative_closest_points {
        ITER
    } else {
        EXACT
    };
    let margin = Fixed { raw: expected.margin };
    let got = closest_points(p1, s1, p2, s2, margin).unwrap();
    let (e1, e2) = (v(expected.p1), v(expected.p2));
    match got {
        ClosestPoints::Intersecting => {
            if expected.kind != 0 {
                // GJK: touching answered as two (nearly) equal points.
                assert!(expected.kind == 1 && close2(e1, e2, tol), "{}: kind", *case.id);
                assert!(dist == ZERO, "{}: touching", *case.id);
                counts.boundary += 1;
            }
        },
        ClosestPoints::Disjoint => {
            if expected.kind != 2 {
                // Within an ulp of the margin.
                assert!(expected.kind == 1 && close(dist, margin, EXACT), "{}: kind", *case.id);
                counts.boundary += 1;
            }
        },
        ClosestPoints::WithinMargin((
            a, b,
        )) => {
            assert!(expected.kind == 1, "{}: kind", *case.id);
            // Ours: a witness pair at `distance`.
            assert!(on_surface(s1, p1, a, SURFACE), "{}: point1 off surface", *case.id);
            assert!(on_surface(s2, p2, b, SURFACE), "{}: point2 off surface", *case.id);
            let len = norm2(b.x - a.x, b.y - a.y);
            assert!(close(len, dist, SURFACE), "{}: witness gap", *case.id);
            if !(close2(a, e1, EXACT) && close2(b, e2, EXACT)) {
                assert!(on_surface(s1, p1, e1, tol), "{}: upstream point1", *case.id);
                assert!(on_surface(s2, p2, e2, tol), "{}: upstream point2", *case.id);
                counts.alternative_points += 1;
            }
        },
    }
}

fn check_contact(
    case: @ShapeQueryCase,
    s1: Shape,
    p1: Pose2,
    s2: Shape,
    p2: Pose2,
    expected: ContactAnswerRaw,
    ref counts: Counts,
) {
    let tol = if *case.iterative_contact {
        ITER
    } else {
        EXACT
    };
    let prediction = Fixed { raw: expected.prediction };
    let got: Option<Contact> = contact(p1, s1, p2, s2, prediction).unwrap();
    let swapped: Option<Contact> = contact(p2, s2, p1, s1, prediction).unwrap();
    let c = match got {
        Some(c) => c,
        None => {
            assert!(!expected.some, "{}: missing contact", *case.id);
            assert!(swapped.is_none(), "{}: swapped contact", *case.id);
            return;
        },
    };
    // Swapped: the same depth; the normal too, unless the penetration is symmetric.
    let f = swapped.unwrap().flipped();
    assert!(close(f.dist, c.dist, SURFACE), "{}: swapped dist", *case.id);
    if !close2(f.normal1, c.normal1, SURFACE) {
        let sep = separation(s1, p1, s2, p2, f.normal1);
        assert!(close(sep, c.dist, SURFACE), "{}: swapped normal", *case.id);
    }
    // Ours: a witness.
    assert!(on_surface(s1, p1, c.point1, SURFACE), "{}: point1 off surface", *case.id);
    assert!(on_surface(s2, p2, c.point2, SURFACE), "{}: point2 off surface", *case.id);
    assert!(
        close2(c.point2 - c.point1, c.normal1.mul_scalar(c.dist), SURFACE), "{}: gap", *case.id,
    );
    assert!(close2(c.normal2, -c.normal1, SURFACE), "{}: normal2", *case.id);
    if !expected.some {
        // GJK / EPA miss an exactly touching pair.
        assert!(*case.iterative_contact && c.dist == ZERO, "{}: extra contact", *case.id);
        counts.boundary += 1;
        return;
    }
    assert!(close(c.dist, Fixed { raw: expected.dist }, tol), "{}: dist", *case.id);
    let (n1, n2) = (v(expected.normal1), v(expected.normal2));
    let ntol = if *case.iterative_contact {
        ITER
    } else {
        NORMAL
    };
    if !close2(c.normal1, n1, ntol) && *case.iterative_contact && c.dist == ZERO {
        // Shapes touching at a single point: any normal of the cone is valid, EPA's included.
        counts.boundary += 1;
    } else if !close2(c.normal1, n1, ntol) {
        // A symmetric penetration: upstream's normal separates by the same depth.
        assert!(!is_halfspace(s1) && !is_halfspace(s2), "{}: normal", *case.id);
        let sep = separation(s1, p1, s2, p2, n1);
        assert!(close(sep, c.dist, tol), "{}: alternative normal", *case.id);
        counts.alternative_normals += 1;
    }
    let (e1, e2) = (v(expected.point1), v(expected.point2));
    if !(close2(c.point1, e1, EXACT) && close2(c.point2, e2, EXACT)) {
        assert!(on_surface(s1, p1, e1, tol), "{}: upstream point1", *case.id);
        assert!(on_surface(s2, p2, e2, tol), "{}: upstream point2", *case.id);
        counts.alternative_points += 1;
    }
    let _ = n2;
}

fn check_range(from: u32, to: u32) -> Counts {
    let mut counts: Counts = Default::default();
    let cases = shape_queries::cases();
    let mut i = from;
    while i != to {
        let case = cases.at(i);
        let (s1, s2, p1, p2) = (
            shape(*case.shape1), shape(*case.shape2), pose(*case.pos1), pose(*case.pos2),
        );
        counts.cases += 1;
        if !*case.supported {
            assert!(distance(p1, s1, p2, s2).is_none(), "{}", *case.id);
            assert!(closest_points(p1, s1, p2, s2, ZERO).is_none(), "{}", *case.id);
            assert!(contact(p1, s1, p2, s2, ZERO).is_none(), "{}", *case.id);
            counts.unsupported += 1;
            i += 1;
            continue;
        }
        let tol = if *case.iterative_distance {
            ITER
        } else {
            EXACT
        };
        let dist = distance(p1, s1, p2, s2).unwrap();
        assert!(close(dist, Fixed { raw: *case.distance }, tol), "{}: distance", *case.id);
        assert!(close(distance(p2, s2, p1, s1).unwrap(), dist, SURFACE), "{}: swapped", *case.id);
        for expected in case.closest_points.span() {
            check_closest(case, s1, p1, s2, p2, dist, *expected, ref counts);
        }
        for expected in case.contacts.span() {
            check_contact(case, s1, p1, s2, p2, *expected, ref counts);
        }
        i += 1;
    }
    counts
}

/// `Counts { cases, unsupported, alternative_points, alternative_normals, boundary }`, pinned so
/// that a change of any answer is noticed.
fn counts(cases: u32, unsupported: u32, points: u32, normals: u32, boundary: u32) -> Counts {
    Counts {
        cases, unsupported, alternative_points: points, alternative_normals: normals, boundary,
    }
}

#[test]
fn test_ball_pairs() {
    assert_eq!(check_range(0, 24), counts(24, 0, 2, 0, 0));
}

#[test]
fn test_cuboid_capsule_halfspace_pairs() {
    assert_eq!(check_range(24, 56), counts(32, 0, 23, 4, 3));
}

#[test]
fn test_polygon_segment_pairs() {
    assert_eq!(check_range(56, 81), counts(25, 1, 20, 4, 12));
    assert_eq!(shape_queries::cases().len(), 81);
}
