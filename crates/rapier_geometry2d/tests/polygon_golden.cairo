//! Convex polygons against Parry f64 0.30.2. GJK boundary ties compare distance, not coordinates.
use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::feature_id::FeatureIdTrait;
use rapier_geometry2d::mass::MassPropertiesTrait;
use rapier_geometry2d::point::convex_polygon::{
    distance_to_local_point_convex_polygon, project_local_point_and_get_feature_convex_polygon,
    project_local_point_convex_polygon,
};
use rapier_geometry2d::ray::{Ray, cast_ray_and_get_normal};
use rapier_geometry2d::shape::{ConvexPolygon, ConvexPolygonTrait, Shape};
use rapier_golden::compare::abs_diff;
use rapier_golden::generated::{polygon_aabb, polygon_mass, polygon_point, polygon_ray};
use rapier_golden::types::{ConvexPolygonRaw, PointFeatureRaw, PoseRaw, Vec2Raw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;

fn v(p: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: p.x }, y: Fixed { raw: p.y } }
}
fn polygon(p: ConvexPolygonRaw) -> ConvexPolygon {
    let mut vertices = array![];
    let points = p.vertices.span();
    let mut i: u32 = 0;
    while i != p.count.into() {
        vertices.append(v(*points.at(i)));
        i += 1;
    }
    ConvexPolygonTrait::from_convex_polyline(vertices.span()).unwrap()
}
fn pose(p: PoseRaw) -> Pose2 {
    Pose2 {
        translation: v(p.translation),
        rotation: Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    }
}
fn near(a: Vec2, b: Vec2Raw, tol: u64) {
    assert!(abs_diff(a.x.raw, b.x) <= tol);
    assert!(abs_diff(a.y.raw, b.y) <= tol);
}

#[test]
fn test_polygon_bounds() {
    for c in polygon_aabb::cases() {
        let result = polygon(*c.shape).compute_aabb(pose(*c.pose));
        near(result.mins, *c.mins, 2);
        near(result.maxs, *c.maxs, 2);
    }
}
#[test]
fn test_polygon_mass() {
    for c in polygon_mass::cases() {
        let p = polygon(*c.shape).mass_properties(Fixed { raw: *c.density });
        near(p.local_com, *c.expected.local_com, 16);
        assert!(abs_diff(p.inv_mass.raw, *c.expected.inv_mass) <= 1024);
        assert!(abs_diff(p.inv_principal_inertia.raw, *c.expected.inv_principal_inertia) <= 4096);
    }
}
#[test]
fn test_polygon_projection() {
    for c in polygon_point::cases() {
        let p = polygon(*c.shape);
        let pt = v(*c.point);
        let (projection, feature) = project_local_point_and_get_feature_convex_polygon(p, pt);
        assert_eq!(projection.is_inside, *c.projection.is_inside);
        // Upstream EPA/GJK may choose another equally close boundary point at the center.
        if !*c.ambiguous && !*c.gjk_degenerate {
            near(projection.point, *c.projection.point, 1024);
            match *c.feature {
                PointFeatureRaw::Face(i) => {
                    assert!(feature.is_face());
                    assert_eq!(feature.code(), i);
                },
                PointFeatureRaw::Vertex(i) => {
                    assert!(feature.is_vertex());
                    assert_eq!(feature.code(), i);
                },
                PointFeatureRaw::Unknown => {},
            }
        }
        let solid = project_local_point_convex_polygon(p, pt, true);
        near(solid.point, *c.projection_solid.point, 1024);
        let distance = distance_to_local_point_convex_polygon(p, pt, false);
        if *c.gjk_degenerate {
            assert_eq!(*c.id, 'poly_pent/tie');
            assert_eq!(*c.distance, -6442450944);
            assert_eq!(projection.point, Vec2 { x: fixed::ZERO, y: -fixed::ONE });
            assert_eq!(distance, -fixed::ONE);
        } else {
            assert!(abs_diff(distance.raw, *c.distance) <= 1024);
        }
    }
}
#[test]
fn test_polygon_ray() {
    for c in polygon_ray::cases() {
        let p = Shape::ConvexPolygon(BoxTrait::new(polygon(*c.shape)));
        let r = Ray { origin: v(*c.origin), dir: v(*c.dir) };
        for (solid, expected) in [(true, *c.solid), (false, *c.hollow)].span() {
            let hit = cast_ray_and_get_normal(
                p, pose(*c.pose), r, Fixed { raw: *c.max_toi }, *solid,
            );
            assert_eq!(hit.is_some(), *expected.hit.hit);
            if let Some(hit) = hit {
                assert!(abs_diff(hit.time_of_impact.raw, *expected.hit.time_of_impact) <= 1024);
                if hit.time_of_impact.raw != 0 {
                    near(hit.normal, *expected.hit.normal, 1024);
                }
            }
        }
    }
}
