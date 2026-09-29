//! `rapier_golden::generated::geometry_utils` against the other utilities of PX3: line-line
//! closest points, the support-map projection, the polygon area and centre of mass, the
//! `Segment` splits, the scaled shapes, the bounding-sphere queries and the polygonal-feature
//! contacts. Each case carries its band in ulps (`tol`, `0` = exact); integers are exact.
//!
//! The `deviation` cases of the scaled shapes are answered by an upstream polygon that does not
//! fit the port's 8 vertices: the port answers `None`.

use fixed::Fixed;
use glam_core::Vec2;
use rapier_geometry2d::aabb::bounding_volume::BoundingSphere;
use rapier_geometry2d::closest_points::line_line::{
    closest_points_line_line, closest_points_line_line_parameters,
    closest_points_line_line_parameters_eps,
};
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait};
use rapier_geometry2d::feature_id::{FEATURE_UNKNOWN, FeatureIdTrait};
use rapier_geometry2d::mass::convex_polygon::convex_polygon_area_and_center_of_mass;
use rapier_geometry2d::point::PointQuery;
use rapier_geometry2d::point::bounding_sphere::BoundingSpherePointQuery;
use rapier_geometry2d::polygonal_feature::{PolygonalFeature, PolygonalFeatureTrait};
use rapier_geometry2d::query::split::{SegmentSplitTrait, SplitResult};
use rapier_geometry2d::query::support_map::local_point_projection_on_support_map;
use rapier_geometry2d::ray::bounding_sphere::BoundingSphereRayCast;
use rapier_geometry2d::ray::{Ray, RayCast};
use rapier_geometry2d::shape::scaled::Either;
use rapier_geometry2d::shape::{
    Ball, BallTrait, Capsule, CapsuleTrait, ConvexPolygon, ConvexPolygonTrait, Cuboid, Segment,
    SegmentTrait, Shape,
};
use rapier_golden::generated::geometry_utils as g;
use rapier_golden::generated::geometry_utils::UtilCase;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;

fn next(ref s: Span<i64>) -> i64 {
    *s.pop_front().unwrap()
}

fn q(ref s: Span<i64>) -> Fixed {
    Fixed { raw: next(ref s) }
}

fn vec(ref s: Span<i64>) -> Vec2 {
    let x = q(ref s);
    let y = q(ref s);
    Vec2 { x, y }
}

fn segment(ref s: Span<i64>) -> Segment {
    let a = vec(ref s);
    let b = vec(ref s);
    Segment { a, b }
}

fn points(ref s: Span<i64>) -> Array<Vec2> {
    let n = next(ref s);
    let mut out = array![];
    let mut i = 0;
    while i != n {
        out.append(vec(ref s));
        i += 1;
    }
    out
}

fn near(got: Fixed, ref exp: Span<i64>, tol: i64, id: felt252) {
    let d = got.raw - next(ref exp);
    assert(d <= tol && -d <= tol, id);
}

fn near_vec(got: Vec2, ref exp: Span<i64>, tol: i64, id: felt252) {
    near(got.x, ref exp, tol, id);
    near(got.y, ref exp, tol, id);
}

fn near_segment(got: Segment, ref exp: Span<i64>, tol: i64, id: felt252) {
    near_vec(got.a, ref exp, tol, id);
    near_vec(got.b, ref exp, tol, id);
}

#[test]
fn test_closest_points_line_line() {
    for c in g::closest_points_line_line_parameters().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let (o1, d1, o2, d2) = (vec(ref i), vec(ref i), vec(ref i), vec(ref i));
        let (s, t) = closest_points_line_line_parameters(o1, d1, o2, d2);
        near(s, ref o, tol, id);
        near(t, ref o, tol, id);
    }
    for c in g::closest_points_line_line().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let (o1, d1, o2, d2) = (vec(ref i), vec(ref i), vec(ref i), vec(ref i));
        let (p1, p2) = closest_points_line_line(o1, d1, o2, d2);
        near_vec(p1, ref o, tol, id);
        near_vec(p2, ref o, tol, id);
    }
    for c in g::closest_points_line_line_parameters_eps().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let (o1, d1, o2, d2) = (vec(ref i), vec(ref i), vec(ref i), vec(ref i));
        let eps = q(ref i);
        let (s, t, parallel) = closest_points_line_line_parameters_eps(o1, d1, o2, d2, eps);
        near(s, ref o, tol, id);
        near(t, ref o, tol, id);
        assert(if parallel {
            1
        } else {
            0
        } == next(ref o), id);
    }
}

#[test]
fn test_support_map_projection_and_polygon_mass() {
    for c in g::support_map_projection().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let shape = match next(ref i) {
            0 => Shape::Ball(Ball { radius: q(ref i) }),
            1 => Shape::Cuboid(Cuboid { half_extents: vec(ref i) }),
            2 => {
                let (a, b) = (vec(ref i), vec(ref i));
                Shape::Capsule(CapsuleTrait::new(a, b, q(ref i)))
            },
            3 => Shape::Segment(segment(ref i)),
            _ => {
                let pts = points(ref i);
                Shape::ConvexPolygon(
                    BoxTrait::new(ConvexPolygonTrait::from_convex_polyline(pts.span()).unwrap()),
                )
            },
        };
        let point = vec(ref i);
        let solid = next(ref i) == 1;
        let proj = local_point_projection_on_support_map(shape, point, solid);
        assert(if proj.is_inside {
            1
        } else {
            0
        } == next(ref o), id);
        near_vec(proj.point, ref o, tol, id);
    }
    for c in g::convex_polygon_area_and_center_of_mass().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let pts = points(ref i);
        let (area, com) = convex_polygon_area_and_center_of_mass(pts.span());
        near(area, ref o, tol, id);
        near_vec(com, ref o, tol, id);
    }
}

fn near_split(got: SplitResult<Segment>, ref exp: Span<i64>, tol: i64, id: felt252) {
    let tag = next(ref exp);
    match got {
        SplitResult::Pair((
            a, b,
        )) => {
            assert(tag == 0, id);
            near_segment(a, ref exp, tol, id);
            near_segment(b, ref exp, tol, id);
        },
        SplitResult::Negative => assert(tag == 1, id),
        SplitResult::Positive => assert(tag == 2, id),
    }
}

#[test]
fn test_segment_from_array_and_splits() {
    for c in g::segment_from_array().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let (a, b) = (vec(ref i), vec(ref i));
        near_segment(SegmentTrait::from_array([a, b]), ref o, tol, id);
    }
    for c in g::segment_canonical_split().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let s = segment(ref i);
        let axis: u32 = next(ref i).try_into().unwrap();
        let (bias, eps) = (q(ref i), q(ref i));
        near_split(s.canonical_split(axis, bias, eps), ref o, tol, id);
    }
    for c in g::segment_local_split().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let s = segment(ref i);
        let axis = vec(ref i);
        let (bias, eps) = (q(ref i), q(ref i));
        near_split(s.local_split(axis, bias, eps), ref o, tol, id);
    }
    for c in g::segment_local_split_and_get_intersection().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let s = segment(ref i);
        let axis = vec(ref i);
        let (bias, eps) = (q(ref i), q(ref i));
        let (result, hit) = s.local_split_and_get_intersection(axis, bias, eps);
        near_split(result, ref o, tol, id);
        let has = next(ref o);
        match hit {
            None => assert(has == 0, id),
            Some((
                p, t,
            )) => {
                assert(has == 1, id);
                near_vec(p, ref o, tol, id);
                near(t, ref o, tol, id);
            },
        }
    }
}

/// The `n`, points and normals a polygon must have.
fn near_polygon(got: ConvexPolygon, ref exp: Span<i64>, tol: i64, id: felt252) {
    let n = next(ref exp);
    assert(got.count.into() == n, id);
    let mut k: u8 = 0;
    while k.into() != n {
        near_vec(got.vertex(k), ref exp, tol, id);
        k += 1;
    }
    k = 0;
    while k.into() != n {
        near_vec(got.normal(k), ref exp, tol, id);
        k += 1;
    }
}

#[test]
fn test_scaled_shapes() {
    for c in g::ball_scaled().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let radius = q(ref i);
        let scale = vec(ref i);
        let n: u32 = next(ref i).try_into().unwrap();
        let kind = next(ref o);
        let ball = Ball { radius };
        match BallTrait::scaled(ball, scale, n) {
            None => assert(kind == 0 || kind == 3, id),
            Some(Either::Left(b)) => {
                assert(kind == 1, id);
                near(b.radius, ref o, tol, id);
            },
            Some(Either::Right(p)) => {
                assert(kind == 2, id);
                let _ = next(ref o);
                near_polygon(p, ref o, tol, id);
            },
        }
    }
    for c in g::capsule_scaled().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let (a, b) = (vec(ref i), vec(ref i));
        let radius = q(ref i);
        let scale = vec(ref i);
        let n: u32 = next(ref i).try_into().unwrap();
        let kind = next(ref o);
        let capsule = CapsuleTrait::new(a, b, radius);
        match CapsuleTrait::scaled(capsule, scale, n) {
            None => assert(kind == 0 || kind == 3, id),
            Some(Either::Left(cap)) => {
                assert(kind == 1, id);
                near(cap.radius, ref o, tol, id);
                let _ = next(ref o);
                near_segment(cap.segment, ref o, tol, id);
            },
            Some(Either::Right(p)) => {
                assert(kind == 2, id);
                let _ = next(ref o);
                near_polygon(p, ref o, tol, id);
            },
        }
    }
    for c in g::convex_polygon_scaled().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let pts = points(ref i);
        let scale = vec(ref i);
        let polygon = ConvexPolygonTrait::from_convex_polyline(pts.span()).unwrap();
        let some = next(ref o);
        match ConvexPolygonTrait::scaled(polygon, scale) {
            None => assert(some == 0, id),
            Some(p) => {
                assert(some == 1, id);
                near_polygon(p, ref o, tol, id);
            },
        }
    }
}

#[test]
fn test_bounding_sphere_queries() {
    for c in g::bounding_sphere_point().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let center = vec(ref i);
        let radius = q(ref i);
        let pt = vec(ref i);
        let solid = next(ref i) == 1;
        let s = BoundingSphere { center, radius };
        let proj = PointQuery::project_local_point(s, pt, solid);
        assert(if proj.is_inside {
            1
        } else {
            0
        } == next(ref o), id);
        near_vec(proj.point, ref o, tol, id);
        near(PointQuery::distance_to_local_point(s, pt, solid), ref o, tol, id);
        assert(if PointQuery::contains_local_point(s, pt) {
            1
        } else {
            0
        } == next(ref o), id);
    }
    for c in g::bounding_sphere_ray().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let center = vec(ref i);
        let radius = q(ref i);
        let (origin, dir) = (vec(ref i), vec(ref i));
        let max = q(ref i);
        let solid = next(ref i) == 1;
        let s = BoundingSphere { center, radius };
        let ray = Ray { origin, dir };
        let some_t = next(ref o);
        match RayCast::cast_local_ray(s, ray, max, solid) {
            None => {
                assert(some_t == 0, id);
                let _ = next(ref o);
            },
            Some(t) => {
                assert(some_t == 1, id);
                near(t, ref o, tol, id);
            },
        }
        let some_hit = next(ref o);
        match RayCast::cast_local_ray_and_get_normal(s, ray, max, solid) {
            None => assert(some_hit == 0, id),
            Some(h) => {
                assert(some_hit == 1, id);
                near(h.time_of_impact, ref o, tol, id);
                near_vec(h.normal, ref o, tol, id);
            },
        }
        let hits = RayCast::intersects_local_ray(s, ray, max);
        let mut rest = o;
        if some_hit == 0 {
            let _ = (next(ref rest), next(ref rest), next(ref rest));
        }
        assert(if hits {
            1
        } else {
            0
        } == next(ref rest), id);
    }
}

fn pose(ref s: Span<i64>) -> Pose2 {
    let t = vec(ref s);
    let re = q(ref s);
    let im = q(ref s);
    Pose2Trait::new(t, Rot2 { re, im })
}

fn near_manifold(m: ContactManifold, ref exp: Span<i64>, tol: i64, id: felt252) {
    assert(m.num_points.into() == next(ref exp), id);
    let [c0, c1] = m.points;
    let mut slot = 0;
    for c in array![c0, c1] {
        if slot < m.num_points {
            near_vec(c.local_p1, ref exp, tol, id);
            near_vec(c.local_p2, ref exp, tol, id);
            assert(c.fid1.packed.into() == next(ref exp), id);
            assert(c.fid2.packed.into() == next(ref exp), id);
            near(c.dist, ref exp, tol, id);
        } else {
            let _ = (vec(ref exp), vec(ref exp), next(ref exp), next(ref exp), q(ref exp));
        }
        slot += 1;
    }
}

#[test]
fn test_polygonal_feature_contacts() {
    for c in g::polygonal_feature_face_face_contacts().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let pos12 = pose(ref i);
        let s1 = segment(ref i);
        let normal = vec(ref i);
        let s2 = segment(ref i);
        let flipped = next(ref i) == 1;
        let (face1, face2): (PolygonalFeature, PolygonalFeature) = (s1.into(), s2.into());
        let mut m: ContactManifold = ContactManifoldTrait::new();
        PolygonalFeatureTrait::face_face_contacts(pos12, face1, normal, face2, ref m, flipped);
        near_manifold(m, ref o, tol, id);
        // The named entry point is the face-face arm of `contacts`.
        let mut via: ContactManifold = ContactManifoldTrait::new();
        let inv = pos12.inverse();
        PolygonalFeatureTrait::contacts(
            pos12, inv, normal, -normal, face1, face2, ref via, flipped,
        );
        assert(via == m, id);
    }
    for c in g::polygonal_feature_face_vertex_contacts().span() {
        let UtilCase { id, tol, input, output } = *c;
        let (mut i, mut o) = (input, output);
        let pos12 = pose(ref i);
        let s1 = segment(ref i);
        let normal = vec(ref i);
        let v = vec(ref i);
        let vid: u32 = next(ref i).try_into().unwrap();
        let flipped = next(ref i) == 1;
        let face1: PolygonalFeature = s1.into();
        let zero = Vec2 { x: Fixed { raw: 0 }, y: Fixed { raw: 0 } };
        let vertex2 = PolygonalFeature {
            vertices: [v, zero],
            vids: [FeatureIdTrait::vertex(vid), FEATURE_UNKNOWN],
            fid: FEATURE_UNKNOWN,
            num_vertices: 1,
        };
        let mut m: ContactManifold = ContactManifoldTrait::new();
        PolygonalFeatureTrait::face_vertex_contacts(pos12, face1, normal, vertex2, ref m, flipped);
        near_manifold(m, ref o, tol, id);
        let mut via: ContactManifold = ContactManifoldTrait::new();
        PolygonalFeatureTrait::contacts(
            pos12, pos12.inverse(), normal, -normal, face1, vertex2, ref via, flipped,
        );
        assert(via == m, id);
    }
}
