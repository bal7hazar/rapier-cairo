use fixed::trig::TrigTrait;
use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2Trait;
use rapier_testing::opaque;
use crate::aabb::Aabb;
use crate::point::convex_polygon::{
    contains_local_point_convex_polygon, distance_to_local_point_convex_polygon,
    project_local_point_and_get_feature_convex_polygon, project_local_point_convex_polygon,
};
use crate::point::cuboid::project_local_point_cuboid;
use crate::ray::convex_polygon::{
    cast_local_ray_and_get_normal_convex_polygon, cast_local_ray_convex_polygon,
};
use crate::ray::{Ray, cast_local_ray_cuboid};
use crate::shape::{Cuboid, Shape, ShapeTrait};
use super::{ConvexPolygon, ConvexPolygonTrait};

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}
fn square() -> ConvexPolygon {
    ConvexPolygon {
        vertices: [
            v(-ONE, -ONE), v(ONE, -ONE), v(ONE, ONE), v(-ONE, ONE), Vec2Trait::ZERO,
            Vec2Trait::ZERO, Vec2Trait::ZERO, Vec2Trait::ZERO,
        ],
        normals: [
            v(ZERO, -ONE), v(ONE, ZERO), v(ZERO, ONE), v(-ONE, ZERO), Vec2Trait::ZERO,
            Vec2Trait::ZERO, Vec2Trait::ZERO, Vec2Trait::ZERO,
        ],
        count: 4,
    }
}
fn pose() -> Pose2 {
    rapier_math::pose2::IDENTITY
}
fn ray() -> Ray {
    Ray { origin: v(-TWO, HALF), dir: v(TWO, ZERO) }
}

#[test]
fn test_construction_table() {
    let p = square();
    let good = array![v(-ONE, -ONE), v(ONE, -ONE), v(ONE, ONE), v(-ONE, ONE)];
    assert_eq!(ConvexPolygonTrait::from_convex_polyline(good.span()), Some(p));
    let cases = array![
        array![], array![v(ZERO, ZERO)], array![v(ZERO, ZERO), v(ONE, ZERO)],
        array![v(ZERO, ZERO), v(ONE, ZERO), v(TWO, ZERO)],
        array![v(-ONE, -ONE), v(-ONE, ONE), v(ONE, ONE), v(ONE, -ONE)],
        array![v(-ONE, -ONE), v(ONE, -ONE), v(ONE, -ONE), v(-ONE, ONE)],
        array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ZERO), v(ONE, ONE), v(-ONE, ONE)],
        array![v(-ONE, -ONE), v(ONE, ONE), v(-ONE, ONE), v(ONE, -ONE)],
        array![
            v(ZERO, ZERO), v(ZERO, ZERO), v(ZERO, ZERO), v(ZERO, ZERO), v(ZERO, ZERO),
            v(ZERO, ZERO), v(ZERO, ZERO), v(ZERO, ZERO), v(ZERO, ZERO),
        ],
    ];
    for points in cases {
        assert!(ConvexPolygonTrait::from_convex_polyline(points.span()).is_none());
    }
}

#[test]
fn test_box_queries_and_mass() {
    let p = square();
    let cuboid = Cuboid { half_extents: v(ONE, ONE) };
    let bounds = Aabb { mins: v(-ONE, -ONE), maxs: v(ONE, ONE) };
    assert_eq!(p.compute_local_aabb(), bounds);
    assert_eq!(p.compute_aabb(pose()), bounds);
    assert_eq!(Shape::ConvexPolygon(BoxTrait::new(p)).compute_aabb(pose()), bounds);
    assert_eq!(p.local_support_point(v(ONE, ZERO)), v(ONE, -ONE));
    assert_eq!(p.support_point(v(ZERO, ZERO)), v(-ONE, -ONE));
    let feature = p.support_feature(v(ONE, ZERO));
    assert_eq!(feature.vertices, [v(ONE, -ONE), v(ONE, ONE)]);
    let mass = p.mass_properties(ONE);
    let reference = crate::mass::MassPropertiesTrait::from_cuboid(ONE, v(ONE, ONE));
    assert_eq!(mass.local_com, reference.local_com);
    assert_eq!(mass.inv_mass, reference.inv_mass);
    assert!(
        rapier_golden::compare::abs_diff(
            mass.inv_principal_inertia.raw, reference.inv_principal_inertia.raw,
        ) <= 2,
    );
    for pt in array![v(TWO, HALF), v(-TWO, TWO), v(ZERO, HALF), v(ONE, HALF), v(ONE, ONE)] {
        for solid in [false, true].span() {
            let got = project_local_point_convex_polygon(p, pt, *solid);
            let expected = project_local_point_cuboid(cuboid, pt, *solid);
            assert_eq!(got, expected);
        }
    }
    assert_eq!(p.count(), 4);
    assert_eq!(p.vertices(), p.vertices);
    assert_eq!(p.normals(), p.normals);
}

#[test]
fn test_ray_table() {
    let p = square();
    let cases = array![
        (ray(), false, TWO, Some(HALF)),
        (Ray { origin: v(ZERO, ZERO), dir: v(TWO, ZERO) }, false, TWO, Some(HALF)),
        (Ray { origin: v(ZERO, ZERO), dir: v(TWO, ZERO) }, true, TWO, Some(ZERO)),
        (Ray { origin: v(ZERO, ZERO), dir: v(ZERO, ZERO) }, false, TWO, None),
        (Ray { origin: v(-TWO, TWO), dir: v(TWO, ZERO) }, false, TWO, None),
        (Ray { origin: v(TWO, ZERO), dir: v(TWO, ZERO) }, false, TWO, None),
        (Ray { origin: v(-TWO, ZERO), dir: v(ONE, ZERO) }, false, HALF, None),
        (Ray { origin: v(-TWO, ZERO), dir: v(ONE, ZERO) }, false, ONE, Some(ONE)),
        (Ray { origin: v(ONE, ZERO), dir: v(ONE, ZERO) }, false, ONE, Some(ZERO)),
    ];
    for (r, solid, max, expected) in cases {
        assert_eq!(cast_local_ray_convex_polygon(p, r, max, solid), expected);
    }
    let hit = cast_local_ray_and_get_normal_convex_polygon(p, ray(), TWO, false).unwrap();
    assert_eq!(hit.normal, v(-ONE, ZERO));
}

#[test]
#[fuzzer(runs: 64, seed: 20260924)]
fn fuzz_projection_boundary(x: i16, y: i16) {
    let pt = v(Fixed { raw: x.into() * 1048576 }, Fixed { raw: y.into() * 1048576 });
    let p = square();
    let result = project_local_point_convex_polygon(p, pt, false);
    let d = result.point - pt;
    let best = crate::point::dot_wide(d.x, d.y, d.x, d.y);
    let mut i = 0;
    while i != 4 {
        let a = p.vertex(i);
        let b = p.vertex(p.next(i));
        let mut j: i32 = 0;
        while j != 9 {
            let t = FixedTrait::from_int(j) * Fixed { raw: 536870912 };
            let sample = a + (b - a).mul_scalar(t) - pt;
            assert!(best <= crate::point::dot_wide(sample.x, sample.y, sample.x, sample.y));
            j += 1;
        }
        i += 1;
    }
}

#[test]
#[fuzzer(runs: 64, seed: 20260924)]
fn fuzz_ray_box_slabs(y: i16, speed: u8) {
    let r = Ray {
        origin: v(-TWO, Fixed { raw: y.into() * 1048576 }),
        dir: v(Fixed { raw: (speed.into() + 1) * 16777216 }, ZERO),
    };
    let max = FixedTrait::from_int(1024);
    let a = cast_local_ray_convex_polygon(square(), r, max, false);
    let b = cast_local_ray_cuboid(Cuboid { half_extents: v(ONE, ONE) }, r, max, false);
    assert_eq!(a, b);
}

#[test]
fn gas_baseline() {
    let _ = opaque(square());
}
#[test]
fn gas_construct() {
    let p = opaque(square());
    let _ = ConvexPolygonTrait::from_convex_polyline(
        [p.vertex(0), p.vertex(1), p.vertex(2), p.vertex(3)].span(),
    );
}
#[test]
fn gas_support() {
    let _ = opaque(square()).support_point(v(ONE, HALF));
}
#[test]
fn gas_feature() {
    let _ = opaque(square()).support_feature(v(ONE, HALF));
}
#[test]
fn gas_aabb() {
    let _ = opaque(square()).compute_aabb(pose());
}
#[test]
fn gas_local_aabb() {
    let _ = opaque(square()).compute_local_aabb();
}
#[test]
fn gas_mass() {
    let _ = opaque(square()).mass_properties(ONE);
}
#[test]
fn gas_projection() {
    let _ = project_local_point_convex_polygon(opaque(square()), v(TWO, HALF), false);
}
#[test]
fn gas_projection_feature() {
    let _ = project_local_point_and_get_feature_convex_polygon(opaque(square()), v(TWO, HALF));
}
#[test]
fn gas_contains() {
    let _ = contains_local_point_convex_polygon(opaque(square()), v(TWO, HALF));
}
#[test]
fn gas_distance() {
    let _ = distance_to_local_point_convex_polygon(opaque(square()), v(TWO, HALF), false);
}
#[test]
fn gas_ray() {
    let _ = cast_local_ray_convex_polygon(opaque(square()), ray(), TWO, false);
}
#[test]
fn gas_aabb_loop() {
    let _ = super::alternatives::compute_aabb_loop(opaque(square()), pose());
}

#[test]
fn gas_aabb_support() {
    let _ = super::alternatives::compute_aabb_support(opaque(square()), pose());
}

#[test]
#[fuzzer(runs: 64, seed: 20260924)]
fn fuzz_aabb_variants(angle: u8, tx: i16, ty: i16) {
    let pose = Pose2 {
        translation: v(Fixed { raw: tx.into() * 65536 }, Fixed { raw: ty.into() * 65536 }),
        rotation: {
            let (sin, cos) = Fixed { raw: angle.into() * 16777216 }.sin_cos();
            Rot2Trait::from_cos_sin(cos, sin)
        },
    };
    assert_eq!(
        square().compute_aabb(pose), super::alternatives::compute_aabb_support(square(), pose),
    );
    assert_eq!(square().compute_aabb(pose), super::alternatives::compute_aabb_loop(square(), pose));
}

#[test]
fn test_polygon_contacts_dispatch() {
    use crate::contact::ContactManifold;
    use crate::shape::{Ball, HalfSpace};
    let polygon = Shape::ConvexPolygon(BoxTrait::new(square()));
    let mut m: ContactManifold = Default::default();
    assert!(
        crate::dispatch::contact_manifold(
            pose(), polygon, Shape::Ball(Ball { radius: HALF }), ZERO, ref m,
        ),
    );
    assert_eq!(m.num_points, 1);
    let halfspace = Shape::HalfSpace(HalfSpace { normal: v(ZERO, ONE) });
    assert!(crate::dispatch::contact_manifold(pose(), halfspace, polygon, ZERO, ref m));
    assert_eq!(m.num_points, 2);
    let mut other: ContactManifold = Default::default();
    assert!(
        crate::dispatch::alternatives::contact_manifold_plain(
            pose(), halfspace, polygon, ZERO, ref other,
        ),
    );
    assert_eq!(m, other);
    assert!(!crate::dispatch::contact_manifold(pose(), polygon, polygon, ZERO, ref m));
    assert_eq!(m.num_points, 0);
}

#[test]
fn test_tiny_edges_use_wide_products() {
    let tiny = Fixed { raw: 1 };
    let p = ConvexPolygonTrait::from_convex_polyline(
        [v(ZERO, ZERO), v(tiny, ZERO), v(tiny, tiny), v(ZERO, tiny)].span(),
    )
        .unwrap();
    assert_eq!(p.normal(0), v(ZERO, -ONE));
    assert!(contains_local_point_convex_polygon(p, v(tiny, tiny)));
    assert!(!contains_local_point_convex_polygon(p, v(Fixed { raw: 2 }, tiny)));
}

#[test]
#[should_panic(expected: 'Polygon: vertex index')]
fn test_vertex_index_panics() {
    let _ = square().vertex(4);
}

#[test]
fn test_entry_rounding_to_zero_keeps_entry_face() {
    let r = Ray {
        origin: v(Fixed { raw: -4294967297 }, ZERO), dir: v(FixedTrait::from_int(4), ZERO),
    };
    let hit = cast_local_ray_and_get_normal_convex_polygon(square(), r, ONE, false).unwrap();
    assert_eq!(hit.time_of_impact, ZERO);
    assert_eq!(hit.normal, v(-ONE, ZERO));
}

#[test]
fn gas_bounding_sphere() {
    let polygon = opaque(square());
    let _ = polygon.vertices();
    let _ = polygon.points();
    let _ = polygon.normals();
    let _ = polygon.count();
    let _ = polygon.vertex(0);
    let _ = polygon.normal(0);
    let _ = polygon.next(0);
    let _ = polygon.compute_local_bounding_sphere();
    let p = Shape::ConvexPolygon(BoxTrait::new(polygon));
    let _ = p.as_convex_polygon();
    let _ = p.shape_type();
}

#[test]
fn test_boxed_shape_serialization_and_equality() {
    let a = Shape::ConvexPolygon(BoxTrait::new(square()));
    let b = Shape::ConvexPolygon(BoxTrait::new(square()));
    assert_eq!(a, b);
    let mut data = array![];
    a.serialize(ref data);
    // Tag, 16 vertex components, 16 normal components, count; no pointer identity.
    assert_eq!(data.len(), 34);
    assert_eq!(*data.at(0), 5);
    let mut serialized = data.span();
    assert_eq!(Serde::<Shape>::deserialize(ref serialized), Some(a));
    assert!(serialized.is_empty());
}

#[test]
fn test_mass_density_zero_and_negative() {
    for density in [ZERO, -ONE, TWO].span() {
        let got = square().mass_properties(*density);
        let expected = crate::mass::MassPropertiesTrait::from_cuboid(*density, v(ONE, ONE));
        assert_eq!(got.local_com, expected.local_com);
        assert_eq!(got.inv_mass, expected.inv_mass);
        assert!(
            rapier_golden::compare::abs_diff(
                got.inv_principal_inertia.raw, expected.inv_principal_inertia.raw,
            ) <= 2,
        );
    }
}

#[test]
#[should_panic(expected: 'i64_sub Overflow')]
fn test_unrepresentable_edge_panics() {
    let _ = ConvexPolygonTrait::from_convex_polyline(
        [v(fixed::MIN, ZERO), v(fixed::MAX, ZERO), v(ZERO, ONE)].span(),
    );
}

#[test]
fn test_triangle_bounding_sphere_uses_vertex_mean() {
    let p = ConvexPolygonTrait::from_convex_polyline(
        [v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span(),
    )
        .unwrap();
    let (center, radius) = p.compute_local_bounding_sphere();
    assert_eq!(center, v(ZERO, Fixed { raw: -1431655765 }));
    assert_eq!(radius, Fixed { raw: 5726623061 });
}
