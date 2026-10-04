use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam_core::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::feature_id::{FEATURE_UNKNOWN, FeatureIdTrait};
use crate::point::SegmentPointLocation;
use crate::point::composite::{
    HeightFieldPointQuery, PolylinePointQuery, project_local_point_and_get_feature_heightfield,
    project_local_point_and_get_feature_polyline, project_local_point_and_get_location_polyline,
    project_local_point_assuming_solid_interior_ccw,
};
use crate::query::dispatcher::{closest_points, contact, distance};
use crate::query::nonlinear_shape_cast::{NonlinearRigidMotionTrait, cast_shapes_nonlinear};
use crate::query::shape_cast::cast_shapes_local;
use crate::query::{ClosestPoints, sweep};
use crate::ray::composite::{
    cast_local_ray_and_get_normal_heightfield_part, cast_local_ray_and_get_normal_polyline_part,
};
use crate::ray::{Ray, cast_local_ray_and_get_normal};
use crate::shape::polyline::ORIENTED;
use crate::shape::{
    BallTrait, CuboidTrait, HalfSpaceTrait, HeightField, HeightFieldTrait, Polyline, PolylineTrait,
    Shape, ShapeTrait,
};

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
}

/// A "V": `(-2, 1) -> (0, 0) -> (2, 1)`.
fn vee() -> Polyline {
    PolylineTrait::new(array![v(int(-2), ONE), v(ZERO, ZERO), v(TWO, ONE)].span(), None)
}

/// Flat ground at `y = 0` over `x in [-4, 4]`, four cells.
fn flat() -> HeightField {
    HeightFieldTrait::new(array![ZERO, ZERO, ZERO, ZERO, ZERO].span(), v(int(8), ONE))
}

fn ccw_square() -> Polyline {
    let mut p = PolylineTrait::new(
        array![v(-ONE, -ONE), v(ONE, -ONE), v(ONE, ONE), v(-ONE, ONE)].span(),
        Some(array![[0, 1], [1, 2], [2, 3], [3, 0]].span()),
    );
    p.set_flags(ORIENTED);
    p
}

#[test]
fn test_point_queries() {
    let p = vee();
    let (proj, (seg, loc)) = project_local_point_and_get_location_polyline(
        @p, v(ONE, int(3)), false,
    );
    assert_eq!(seg, 1);
    assert!(!proj.is_inside);
    assert_eq!(loc, SegmentPointLocation::OnVertex(1));
    // The segment's own feature (parry 0.31): segment 0's side `Face(0)`, then segment 1's end
    // vertex `Vertex(1)` (0.30.2: `Face(1)`, the segment).
    let (_, feature) = project_local_point_and_get_feature_polyline(@p, v(-ONE, -ONE));
    assert_eq!(feature, FeatureIdTrait::face(0));
    let (_, feature) = project_local_point_and_get_feature_polyline(@p, v(int(3), int(3)));
    assert_eq!(feature, FeatureIdTrait::vertex(1));
    // An unoriented polyline has no interior.
    assert!(!p.contains_local_point(v(ZERO, HALF)));
    // An oriented square: inside, solid answers the point itself.
    let sq = ccw_square();
    let inside = v(HALF, -HALF);
    let solid = sq.project_local_point(inside, true);
    assert!(solid.is_inside);
    assert_eq!(solid.point, inside);
    let hollow = sq.project_local_point(inside, false);
    assert!(hollow.is_inside);
    assert!(hollow.point != inside);
    assert!(sq.contains_local_point(inside));
    assert!(!sq.contains_local_point(v(TWO, ZERO)));
    let (p2, _) = project_local_point_assuming_solid_interior_ccw(@sq, inside);
    assert!(p2.is_inside);
    // Heightfield: the closest enabled cell, `Unknown` feature, no interior.
    let h = flat();
    let proj = h.project_local_point(v(HALF, TWO), true);
    assert_eq!(proj.point, v(HALF, ZERO));
    assert!(!h.contains_local_point(v(HALF, -ONE)));
    assert_eq!(
        project_local_point_and_get_feature_heightfield(@h, v(ZERO, ONE)),
        (proj_at(v(ZERO, ZERO)), FEATURE_UNKNOWN),
    );
    assert_eq!(h.distance_to_local_point(v(ONE, -TWO), true), TWO);
}

fn proj_at(p: Vec2) -> crate::point::PointProjection {
    crate::point::PointProjection { is_inside: false, point: p }
}

#[test]
fn test_ray_casts() {
    let p = vee();
    // Straight down onto the right arm: segment 1.
    let ray = Ray { origin: v(ONE, int(5)), dir: v(ZERO, -ONE) };
    let (seg, hit) = cast_local_ray_and_get_normal_polyline_part(@p, ray, int(100), true).unwrap();
    assert_eq!(seg, 1);
    assert_eq!(hit.time_of_impact, int(5) - HALF);
    // Strictly below `max_time_of_impact`, as upstream's BVH search.
    assert!(cast_local_ray_and_get_normal_polyline_part(@p, ray, int(5) - HALF, true).is_none());
    let h = flat();
    let (cell, hit) = cast_local_ray_and_get_normal_heightfield_part(@h, ray, int(100)).unwrap();
    assert_eq!(cell, 2);
    assert_eq!(hit.time_of_impact, int(5));
    // The cell's `normal()` (clockwise of `a -> b`): `(0, -1)`; from above, the segment's own
    // `Face(1)` (parry 0.31; 0.30.2 answered `Face(cell + 4)`).
    assert_eq!(hit.normal, v(ZERO, -ONE));
    assert_eq!(hit.feature, FeatureIdTrait::face(1));
    // A slanted ray walks the cells.
    let slanted = Ray { origin: v(int(-3), ONE), dir: v(ONE, -HALF) };
    let (cell, hit) = cast_local_ray_and_get_normal_heightfield_part(@h, slanted, int(100))
        .unwrap();
    assert_eq!((cell, hit.time_of_impact), (1, TWO));
    // Missing above the box.
    assert!(
        cast_local_ray_and_get_normal(
            Shape::HeightField(BoxTrait::new(h)),
            Ray { origin: v(int(-5), ONE), dir: v(ZERO, ONE) },
            int(10),
            true,
        )
            .is_none(),
    );
}

#[test]
fn test_pair_queries() {
    let poly = Shape::Polyline(BoxTrait::new(vee()));
    let hf = Shape::HeightField(BoxTrait::new(flat()));
    let ball = Shape::Ball(BallTrait::new(HALF));
    let cuboid = Shape::Cuboid(CuboidTrait::new(v(HALF, HALF)));
    // Intersection: ball by point query, cuboid by parts; heightfield–cuboid unsupported.
    assert_eq!(crate::dispatch::intersection_test(at(ZERO, HALF), poly, ball), Some(true));
    assert_eq!(crate::dispatch::intersection_test(at(ZERO, int(3)), poly, ball), Some(false));
    assert_eq!(crate::dispatch::intersection_test(at(ONE, ONE), poly, cuboid), Some(true));
    assert_eq!(crate::dispatch::intersection_test(at(-ONE, -ONE), cuboid, poly), Some(true));
    assert_eq!(crate::dispatch::intersection_test(at(ZERO, HALF), hf, ball), Some(true));
    assert_eq!(crate::dispatch::intersection_test(at(ZERO, HALF), hf, cuboid), None);
    // Distance: the closest segment; unsupported for a heightfield.
    // The ball below the vertex of the "V".
    assert_eq!(distance(at(ZERO, -TWO), poly, ball), Some(ONE + HALF));
    assert_eq!(distance(at(ZERO, TWO), ball, poly), Some(ONE + HALF));
    assert_eq!(distance(at(ZERO, -TWO), hf, ball), None);
    // Closest points.
    assert_eq!(
        closest_points(at(ZERO, -TWO), poly, ball, int(5)),
        Some(ClosestPoints::WithinMargin((v(ZERO, ZERO), v(ZERO, HALF)))),
    );
    assert_eq!(
        closest_points(at(ZERO, TWO), ball, poly, int(5)),
        Some(ClosestPoints::WithinMargin((v(ZERO, HALF), v(ZERO, ZERO)))),
    );
    assert_eq!(closest_points(at(ZERO, -TWO), poly, ball, ONE), Some(ClosestPoints::Disjoint));
    // Contact: the deepest part, flipped back when the composite is second.
    let c = contact(at(HALF, HALF - FixedTrait::from_ratio(1, 8)), hf, cuboid, ZERO)
        .unwrap()
        .unwrap();
    assert_eq!(c.normal1, v(ZERO, ONE));
    assert_eq!(c.dist, -FixedTrait::from_ratio(1, 8));
    let c2 = contact(at(-HALF, -(HALF - FixedTrait::from_ratio(1, 8))), cuboid, hf, ZERO)
        .unwrap()
        .unwrap();
    assert_eq!(c2.normal2, v(ZERO, ONE));
    assert_eq!(c2.dist, c.dist);
    assert_eq!(contact(at(ZERO, int(5)), hf, cuboid, ONE), Some(None));
    // A half-space against a polyline goes through the segments.
    let hs = Shape::HalfSpace(HalfSpaceTrait::new(v(ZERO, ONE)));
    assert_eq!(distance(at(ZERO, int(3)), hs, poly), Some(int(3)));
}

#[test]
fn test_shape_casts() {
    let hf = Shape::HeightField(BoxTrait::new(flat()));
    let poly = Shape::Polyline(BoxTrait::new(vee()));
    let ball = Shape::Ball(BallTrait::new(HALF));
    let fall = v(ZERO, -ONE);
    let hit = cast_shapes_local(at(ONE, int(3)), fall, hf, ball, Default::default())
        .unwrap()
        .unwrap();
    assert_eq!(hit.time_of_impact, int(3) - HALF);
    // Composite second: swapped back.
    let hit2 = cast_shapes_local(at(-ONE, -int(3)), v(ZERO, ONE), ball, hf, Default::default())
        .unwrap()
        .unwrap();
    assert_eq!(hit2.time_of_impact, hit.time_of_impact);
    assert_eq!(hit2.normal2, hit.normal1);
    // The ball meets the arms of the "V" (slope 1/2) at a height of `sqrt(5) / 4`.
    let hit3 = cast_shapes_local(at(ZERO, int(3)), fall, poly, ball, Default::default())
        .unwrap()
        .unwrap();
    assert!(hit3.time_of_impact > FixedTrait::from_ratio(244, 100));
    assert!(hit3.time_of_impact < FixedTrait::from_ratio(245, 100));
    // Nonlinear: a polyline answers, a heightfield does not.
    let still = NonlinearRigidMotionTrait::identity();
    let moving = crate::query::NonlinearRigidMotion {
        start: at(ZERO, int(3)), local_center: v(ZERO, ZERO), linvel: fall, angvel: ZERO,
    };
    let nl = cast_shapes_nonlinear(still, poly, moving, ball, ZERO, int(10), true)
        .unwrap()
        .unwrap();
    assert!(nl.time_of_impact > int(2) && nl.time_of_impact < int(3));
    assert!(cast_shapes_nonlinear(still, hf, moving, ball, ZERO, int(10), true).is_none());
}

#[test]
fn test_shape_trait_on_composites() {
    let poly: Shape = vee().into();
    let hf: Shape = flat().into();
    assert!(poly.is_composite() && hf.is_composite());
    assert!(!poly.is_convex() && !hf.is_convex());
    assert!(poly.as_support_map().is_none() && hf.as_support_map().is_none());
    assert_eq!(poly.as_polyline(), Some(vee()));
    assert_eq!(hf.as_heightfield(), Some(flat()));
    assert!(poly.as_heightfield().is_none());
    assert_eq!(poly.shape_type(), crate::shape::ShapeType::Polyline);
    assert_eq!(poly.compute_local_aabb(), vee().local_aabb());
    assert_eq!(hf.compute_aabb(at(ONE, ONE)), flat().aabb(at(ONE, ONE)));
    // `Serde` tags 10 and 11.
    let mut out = array![];
    core::serde::Serde::serialize(@hf, ref out);
    assert_eq!(*out.at(0), 11);
    let mut span = out.span();
    let back: Shape = core::serde::Serde::deserialize(ref span).unwrap();
    assert_eq!(back, hf);
    let mut out = array![];
    core::serde::Serde::serialize(@poly, ref out);
    assert_eq!(*out.at(0), 10);
    let mut span = out.span();
    let back: Shape = core::serde::Serde::deserialize(ref span).unwrap();
    assert_eq!(back, poly);
}

#[test]
fn test_sweep_toi_composite() {
    let proxy = sweep::ToiProxyTrait::point(v(ZERO, ZERO), HALF);
    let fast = sweep::composite::SweepCompositeFastShape {
        proxy,
        sweep: sweep::SweepTrait::from_poses(at(ZERO, int(3)), at(ZERO, int(-3)), v(ZERO, ZERO)),
        local_centroid: v(ZERO, ZERO),
        min_extent: HALF,
    };
    let slop = FixedTrait::from_ratio(1, 200);
    let out = sweep::composite::sweep_time_of_impact_composite(
        Shape::HeightField(BoxTrait::new(flat())), at(ZERO, ZERO), fast, false, false, ONE, slop,
    )
        .unwrap();
    assert_eq!(out.status, sweep::SweepToiStatus::Hit);
    assert!(out.fraction > FixedTrait::from_ratio(4, 10) && out.fraction < HALF);
    let out = sweep::composite::sweep_time_of_impact_composite(
        Shape::Polyline(BoxTrait::new(vee())), at(ZERO, ZERO), fast, false, false, ONE, slop,
    )
        .unwrap();
    assert_eq!(out.status, sweep::SweepToiStatus::Hit);
    assert!(
        sweep::composite::sweep_time_of_impact_composite(
            Shape::Ball(BallTrait::new(ONE)), at(ZERO, ZERO), fast, false, false, ONE, slop,
        )
            .is_none(),
    );
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_contact_heightfield_cuboid() {
    let _ = contact(
        opaque(at(HALF, HALF)),
        opaque(Shape::HeightField(BoxTrait::new(flat()))),
        opaque(Shape::Cuboid(CuboidTrait::new(v(HALF, HALF)))),
        opaque(ZERO),
    );
}

#[test]
fn gas_distance_polyline_ball() {
    let _ = distance(
        opaque(at(ZERO, TWO)),
        opaque(Shape::Polyline(BoxTrait::new(vee()))),
        opaque(Shape::Ball(BallTrait::new(HALF))),
    );
}

#[test]
fn gas_ray_heightfield() {
    let _ = cast_local_ray_and_get_normal_heightfield_part(
        opaque(@flat()), opaque(Ray { origin: v(ONE, int(5)), dir: v(ZERO, -ONE) }), opaque(int(9)),
    );
}
