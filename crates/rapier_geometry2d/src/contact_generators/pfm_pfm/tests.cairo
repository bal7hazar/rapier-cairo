use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::contact::{ContactManifold, ContactManifoldTrait};
use crate::contact_generators::polygon_polygon::contact_manifold_polygon_polygon;
use crate::dispatch::contact_manifold;
use crate::feature_id::FeatureIdTrait;
use crate::point::round_shape::project_local_point_round;
use crate::point::triangle::project_local_point_and_get_location_triangle;
use crate::ray::round_shape::cast_local_ray_and_get_normal_round_cuboid;
use crate::ray::triangle::cast_local_ray_and_get_normal_triangle;
use crate::ray::{Ray, RayTrait};
use crate::shape::{
    BallTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, RoundShape, Shape, Triangle,
    TriangleTrait,
};
use super::contact_manifold_pfm_pfm;

fn v(x: i64, y: i64) -> Vec2 {
    Vec2 { x: Fixed { raw: x * 0x40000000 }, y: Fixed { raw: y * 0x40000000 } }
}

fn at(x: i64, y: i64) -> Pose2 {
    Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
}

/// Counter-clockwise `(-1, -0.5) (1, -0.5) (0, 0.75)` (quarter units).
fn ccw() -> Triangle {
    TriangleTrait::new(v(-4, -2), v(4, -2), v(0, 3))
}

fn cw() -> Triangle {
    TriangleTrait::new(v(-4, -2), v(0, 3), v(4, -2))
}

fn tri(t: Triangle) -> Shape {
    t.into()
}

fn cuboid() -> Shape {
    Shape::Cuboid(CuboidTrait::new(v(2, 1)))
}

fn rcub() -> Shape {
    RoundShape {
        inner_shape: CuboidTrait::new(v(2, 1)), border_radius: FixedTrait::from_ratio(1, 4),
    }
        .into()
}

fn poly() -> Shape {
    ConvexPolygonTrait::from_convex_polyline(array![v(-2, -2), v(2, -2), v(2, 2), v(-2, 2)].span())
        .unwrap()
        .into()
}

fn run(s1: Shape, s2: Shape, pos12: Pose2) -> ContactManifold {
    let mut m: ContactManifold = Default::default();
    assert!(contact_manifold(pos12, s1, s2, FixedTrait::from_ratio(1, 50), ref m));
    m
}

#[test]
fn test_orientation_does_not_change_the_manifold() {
    // A cuboid resting on the bottom edge, both triangle orientations: same points and ids.
    let p = at(0, -3);
    let a = run(tri(ccw()), cuboid(), p);
    let b = run(tri(cw()), cuboid(), p);
    assert_eq!(a.num_points, 2);
    assert_eq!(a.num_points, b.num_points);
    assert_eq!(a.local_n1, v(0, -4));
    assert_eq!(a.local_n1, b.local_n1);
    let mut i = 0;
    while i != a.num_points {
        assert_eq!(a.point(i).local_p1, b.point(i).local_p1);
        assert_eq!(a.point(i).dist, b.point(i).dist);
        // Face 0 (`ab`) of the triangle: the same edge in both orientations.
        assert!(a.point(i).fid1 == FeatureIdTrait::face(0) || a.point(i).fid1.is_vertex());
        i += 1;
    }
}

#[test]
fn test_border_radius_offsets_points_and_distance() {
    // Round cuboid over a plain cuboid: the plain pair's distance less the radius, points pushed.
    let p = at(0, -5);
    let plain = run(cuboid(), cuboid(), p);
    let round = run(rcub(), cuboid(), p);
    assert_eq!(plain.num_points, round.num_points);
    let r = FixedTrait::from_ratio(1, 4);
    let mut i = 0;
    while i != plain.num_points {
        assert_eq!(round.point(i).dist, plain.point(i).dist - r);
        assert_eq!(round.point(i).local_p1.y, plain.point(i).local_p1.y - r);
        i += 1;
    }
    // Separated beyond the grown prediction: no point.
    assert_eq!(run(rcub(), cuboid(), at(0, -9)).num_points, 0);
}

#[test]
fn test_unsupported_and_warm_start() {
    let mut m: ContactManifold = Default::default();
    let ball = Shape::Ball(BallTrait::new(ONE));
    assert!(!contact_manifold_pfm_pfm(at(0, 0), ball, tri(ccw()), ZERO, ref m));
    let hs = Shape::HalfSpace(HalfSpaceTrait::new(v(0, 4)));
    assert!(!contact_manifold_pfm_pfm(at(0, 0), tri(ccw()), hs, ZERO, ref m));
    // Warm start: the impulse follows the feature ids to the next call.
    let p = at(0, -3);
    let mut m = run(tri(ccw()), cuboid(), p);
    let [mut a, b] = m.points;
    a.data.impulse = HALF;
    m.points = [a, b];
    m.local_n2 = -m.local_n2; // defeat the fast path
    assert!(contact_manifold(p, tri(ccw()), cuboid(), FixedTrait::from_ratio(1, 50), ref m));
    assert_eq!(m.point(0).data.impulse, HALF);
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u8);
}

#[test]
fn gas_triangle_cuboid() {
    let _ = run(opaque(tri(ccw())), opaque(cuboid()), opaque(at(0, -3)));
}

#[test]
fn gas_triangle_triangle() {
    let _ = run(opaque(tri(ccw())), opaque(tri(cw())), opaque(at(1, -4)));
}

#[test]
fn gas_triangle_polygon() {
    let _ = run(opaque(tri(ccw())), opaque(poly()), opaque(at(0, -4)));
}

/// Rejected: the triangle as a validated `ConvexPolygon` through the polygon generator
/// (construction normalises three normals and checks convexity; polygon feature ids).
#[test]
fn gas_triangle_polygon_as_polygon_alternative() {
    let t = opaque(ccw());
    let p1 = ConvexPolygonTrait::from_convex_polyline(array![t.a, t.b, t.c].span()).unwrap();
    let p2 = ConvexPolygonTrait::from_convex_polyline(
        array![v(-2, -2), v(2, -2), v(2, 2), v(-2, 2)].span(),
    )
        .unwrap();
    let mut m: ContactManifold = Default::default();
    contact_manifold_polygon_polygon(
        opaque(at(0, -4)), p1, p2, FixedTrait::from_ratio(1, 50), ref m,
    );
}

#[test]
fn gas_round_cuboid_cuboid() {
    let _ = run(opaque(rcub()), opaque(cuboid()), opaque(at(0, -5)));
}

#[test]
fn gas_round_cuboid_round_cuboid() {
    let _ = run(opaque(rcub()), opaque(rcub()), opaque(at(1, -5)));
}

#[test]
fn gas_ball_triangle() {
    let _ = run(opaque(Shape::Ball(BallTrait::new(HALF))), opaque(tri(ccw())), opaque(at(0, -4)));
}

#[test]
fn gas_halfspace_round_cuboid() {
    let hs = Shape::HalfSpace(HalfSpaceTrait::new(v(0, 4)));
    let _ = run(opaque(hs), opaque(rcub()), opaque(at(0, 5)));
}

#[test]
fn gas_project_triangle() {
    let _ = project_local_point_and_get_location_triangle(opaque(ccw()), opaque(v(6, -4)), false);
}

#[test]
fn gas_project_round_cuboid() {
    let _ = project_local_point_round(
        opaque(CuboidTrait::new(v(2, 1))), opaque(HALF), opaque(v(6, 4)), false,
    );
}

#[test]
fn gas_ray_triangle() {
    let _ = cast_local_ray_and_get_normal_triangle(
        opaque(ccw()),
        opaque(RayTrait::new(v(-12, 0), v(4, 0))),
        opaque(FixedTrait::from_int(9)),
        true,
    );
}

#[test]
fn gas_ray_round_cuboid() {
    let ray: Ray = RayTrait::new(v(-12, 0), v(4, 0));
    let _ = cast_local_ray_and_get_normal_round_cuboid(
        opaque(CuboidTrait::new(v(2, 1))),
        opaque(HALF),
        opaque(ray),
        opaque(FixedTrait::from_int(9)),
        true,
    );
}

#[test]
fn test_cuboid_triangle_entry_points() {
    let p = at(0, 3);
    let t = cw();
    let c = CuboidTrait::new(v(2, 1));
    let mut a: ContactManifold = Default::default();
    super::contact_manifold_cuboid_triangle(p, c, t, FixedTrait::from_ratio(1, 50), ref a);
    let b = run(Shape::Cuboid(c), tri(t), p);
    assert_eq!(a, b);
    let mut m: ContactManifold = Default::default();
    assert!(!super::contact_manifold_cuboid_triangle_shapes(p, tri(t), tri(t), ZERO, ref m));
    assert!(
        crate::query::intersection::intersection_test_cuboid_triangle(
            p, c, t,
        ) == crate::query::intersection::intersection_test_triangle_cuboid(p.inverse(), t, c),
    );
    assert!(
        crate::query::intersection::intersection_test_aabb_triangle(
            crate::aabb::Aabb { mins: v(-2, -1), maxs: v(2, 1) }, t,
        ),
    );
    let _ = crate::query::cuboid::closest_points_cuboid_triangle(p, c, t, ONE);
    let _ = crate::query::cuboid::closest_points_triangle_cuboid(p.inverse(), t, c, ONE);
}
