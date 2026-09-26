//! SH1 point projections, ray casts, mass properties and AABBs (triangles, round shapes)
//! against Parry f64 0.30.2.
//!
//! Bands (raw Q32.32 units): the triangle kernels are upstream's algorithms on exact decisions
//! (`EXACT` = 4); the round shapes answer in closed form where upstream runs GJK: distances and
//! ray answers within `GJK` = 4096 (`~1e-6`), projected points within `GJK_POINT` = `2^19`
//! (`~1.2e-4`: GJK stops on the rounded boundary with a projection up to `7e-5` away from the
//! exact one, `rcub/out_vertex`, while its distance agrees to `1e-8`); mass and inertia inverses
//! as the polygon family.
use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::feature_id::FeatureIdTrait;
use rapier_geometry2d::point::PointQuery;
use rapier_geometry2d::ray::{Ray, cast_ray, cast_ray_and_get_normal};
use rapier_geometry2d::shape::{
    RoundConvexPolygonTrait, ShapeTrait, TrianglePointLocation, TriangleTrait,
};
use rapier_golden::compare::abs_diff;
use rapier_golden::generated::{sh1_aabb, sh1_mass, sh1_points, sh1_rays};
use rapier_golden::types::{
    PointFeatureRaw, PoseRaw, RayAnswerRaw, Sh1ShapeRaw, TriangleLocationRaw, Vec2Raw,
};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use super::sh1_contacts_golden::shape;

const EXACT: u64 = 4;
const GJK: u64 = 4096;
const GJK_POINT: u64 = 0x80000;

fn v(p: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: p.x }, y: Fixed { raw: p.y } }
}
fn pose(p: PoseRaw) -> Pose2 {
    Pose2 {
        translation: v(p.translation),
        rotation: Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    }
}
fn near(id: felt252, a: Vec2, b: Vec2Raw, tol: u64) {
    assert!(
        abs_diff(a.x.raw, b.x) <= tol && abs_diff(a.y.raw, b.y) <= tol, "{} {:?} {:?}", id, a, b,
    );
}
fn is_triangle(s: Sh1ShapeRaw) -> bool {
    match s {
        Sh1ShapeRaw::Triangle(_) => true,
        _ => false,
    }
}
fn band(s: Sh1ShapeRaw) -> u64 {
    if is_triangle(s) {
        EXACT
    } else {
        GJK
    }
}
fn point_band(s: Sh1ShapeRaw) -> u64 {
    if is_triangle(s) {
        EXACT
    } else {
        GJK_POINT
    }
}

#[test]
fn test_sh1_point_projections() {
    let mut count = 0;
    for c in sh1_points::cases() {
        let s = shape(*c.shape);
        let tol = band(*c.shape);
        let point_tol = point_band(*c.shape);
        let pt = v(*c.point);
        let (projection, feature) = s.project_local_point_and_get_feature(pt);
        assert_eq!(projection.is_inside, *c.projection.is_inside, "{} inside", *c.id);
        near(*c.id, projection.point, *c.projection.point, point_tol);
        let solid = s.project_local_point(pt, true);
        assert_eq!(solid.is_inside, *c.projection_solid.is_inside, "{} solid", *c.id);
        near(*c.id, solid.point, *c.projection_solid.point, point_tol);
        let d = s.distance_to_local_point(pt, false);
        assert!(abs_diff(d.raw, *c.distance) <= tol, "{} distance {:?}", *c.id, d);
        match *c.feature {
            PointFeatureRaw::Face(i) => assert!(feature.is_face() && feature.code() == i),
            PointFeatureRaw::Vertex(i) => assert!(feature.is_vertex() && feature.code() == i),
            PointFeatureRaw::Unknown => assert!(feature.is_unknown()),
        }
        if let Sh1ShapeRaw::Triangle(_) = *c.shape {
            let t = s.as_triangle().unwrap();
            let (with_loc, loc) =
                rapier_geometry2d::point::project_local_point_and_get_location_triangle(
                t, pt, false,
            );
            assert_eq!(with_loc, projection);
            match (*c.location, loc) {
                (
                    TriangleLocationRaw::OnVertex(i), TrianglePointLocation::OnVertex(j),
                ) => assert_eq!(i, j),
                (
                    TriangleLocationRaw::OnEdge(e), TrianglePointLocation::OnEdge((j, (u, w))),
                ) => {
                    assert_eq!(e.edge, j);
                    assert!(abs_diff(u.raw, e.u) <= EXACT && abs_diff(w.raw, e.v) <= EXACT);
                },
                (TriangleLocationRaw::OnSolid, TrianglePointLocation::OnSolid) => {},
                _ => panic!("location"),
            }
            let _ = t.area();
        }
        count += 1;
    }
    assert_eq!(count, 30);
}

fn check_ray(
    id: felt252,
    s: rapier_geometry2d::shape::Shape,
    c_pose: Pose2,
    ray: Ray,
    max: Fixed,
    solid: bool,
    e: RayAnswerRaw,
    tol: u64,
) {
    let toi = cast_ray(s, c_pose, ray, max, solid);
    assert_eq!(toi.is_some(), e.has_toi, "{} has_toi {}", id, solid);
    if let Some(t) = toi {
        assert!(abs_diff(t.raw, e.toi) <= tol, "{} toi {:?} {}", id, t, e.toi);
    }
    let hit = cast_ray_and_get_normal(s, c_pose, ray, max, solid);
    assert_eq!(hit.is_some(), e.hit.hit, "{} hit", id);
    if let Some(h) = hit {
        assert!(abs_diff(h.time_of_impact.raw, e.hit.time_of_impact) <= tol, "{} hit toi", id);
        near(id, h.normal, e.hit.normal, tol);
        match e.hit.feature {
            PointFeatureRaw::Face(i) => assert!(h.feature.is_face() && h.feature.code() == i),
            PointFeatureRaw::Vertex(i) => assert!(h.feature.is_vertex() && h.feature.code() == i),
            PointFeatureRaw::Unknown => assert!(h.feature.is_unknown()),
        }
    }
}

#[test]
fn test_sh1_ray_casts() {
    let mut count = 0;
    for c in sh1_rays::cases() {
        let s = shape(*c.shape);
        let ray = Ray { origin: v(*c.origin), dir: v(*c.dir) };
        let max = Fixed { raw: *c.max_toi };
        let tol = band(*c.shape);
        check_ray(*c.id, s, pose(*c.pose), ray, max, true, *c.solid, tol);
        check_ray(*c.id, s, pose(*c.pose), ray, max, false, *c.hollow, tol);
        count += 1;
    }
    assert_eq!(count, 20);
}

#[test]
fn test_sh1_mass_and_aabb() {
    for c in sh1_mass::cases() {
        let p = shape(*c.shape).mass_properties(Fixed { raw: *c.density });
        near(*c.id, p.local_com, *c.expected.local_com, 16);
        assert!(abs_diff(p.inv_mass.raw, *c.expected.inv_mass) <= 1024, "{} inv_mass", *c.id);
        assert!(
            abs_diff(p.inv_principal_inertia.raw, *c.expected.inv_principal_inertia) <= 4096,
            "{} inertia {:?}",
            *c.id,
            p,
        );
    }
    for c in sh1_aabb::cases() {
        let s = shape(*c.shape);
        let b = s.compute_aabb(pose(*c.pose));
        let rotated = (*c.pose).rotation.im != 0;
        if let Sh1ShapeRaw::RoundPolygon(_) = *c.shape {
            if rotated {
                // The dispatch's round-polygon box is the transformed local box (conservative, see
                // `RoundConvexPolygonShape`): it contains upstream's tight box, which the method
                // `RoundConvexPolygonTrait::compute_aabb` reproduces.
                assert!(
                    b.mins.x.raw <= *c.mins.x
                        && b.mins.y.raw <= *c.mins.y
                        && b.maxs.x.raw >= *c.maxs.x
                        && b.maxs.y.raw >= *c.maxs.y,
                    "{} contains",
                    *c.id,
                );
                let tight = s.as_round_convex_polygon().unwrap().compute_aabb(pose(*c.pose));
                near(*c.id, tight.mins, *c.mins, EXACT);
                near(*c.id, tight.maxs, *c.maxs, EXACT);
                continue;
            }
        }
        near(*c.id, b.mins, *c.mins, EXACT);
        near(*c.id, b.maxs, *c.maxs, EXACT);
    }
}
