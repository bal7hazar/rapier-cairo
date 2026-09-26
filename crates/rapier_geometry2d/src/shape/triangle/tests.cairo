use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_math::pose2::Pose2Trait;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::aabb::Aabb;
use crate::feature_id::FeatureIdTrait;
use crate::mass::MassPropertiesTrait;
use crate::shape::{ConvexPolygonTrait, Segment};
use super::{
    Triangle, TriangleOrientation, TrianglePointLocation, TrianglePointLocationTrait, TriangleTrait,
    core_edge_id, core_vertex_id, feature_to_triangle, triangle_core,
};

fn v(x: i32, y: i32) -> Vec2 {
    Vec2 { x: FixedTrait::from_int(x.into()), y: FixedTrait::from_int(y.into()) }
}

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x.into())
}

/// Counter-clockwise right triangle `(0,0) (4,0) (0,3)`.
fn ccw() -> Triangle {
    TriangleTrait::new(v(0, 0), v(4, 0), v(0, 3))
}

/// The same triangle, clockwise.
fn cw() -> Triangle {
    TriangleTrait::new(v(0, 0), v(0, 3), v(4, 0))
}

fn flat() -> Triangle {
    TriangleTrait::new(v(0, 0), v(2, 0), v(5, 0))
}

#[test]
fn test_orientation_area_center_perimeter_table() {
    let cases: Span<(Triangle, TriangleOrientation, Fixed, Vec2, Fixed)> = array![
        (
            ccw(),
            TriangleOrientation::CounterClockwise,
            int(6),
            Vec2 { x: Fixed { raw: 5726623061 }, y: ONE },
            int(12),
        ),
        (
            cw(),
            TriangleOrientation::Clockwise,
            int(6),
            Vec2 { x: Fixed { raw: 5726623061 }, y: ONE },
            int(12),
        ),
        (
            flat(),
            TriangleOrientation::Degenerate,
            ZERO,
            Vec2 { x: Fixed { raw: 10021590357 }, y: ZERO },
            int(10),
        ),
    ]
        .span();
    for (t, o, area, center, perimeter) in cases {
        assert_eq!((*t).orientation(ZERO), *o);
        assert_eq!(TriangleTrait::orientation2d((*t).a, (*t).b, (*t).c, ZERO), *o);
        assert_eq!((*t).area(), *area);
        assert_eq!((*t).center(), *center);
        assert_eq!((*t).perimeter(), *perimeter);
    }
    // `epsilon` is compared with the doubled area (12 here).
    assert_eq!(ccw().orientation(int(12)), TriangleOrientation::Degenerate);
    assert_eq!(ccw().orientation(int(11)), TriangleOrientation::CounterClockwise);
    assert_eq!(cw().orientation(int(11)), TriangleOrientation::Clockwise);
}

#[test]
fn test_vertices_edges_reverse_and_from_array() {
    let t = ccw();
    assert_eq!(t.vertices(), [v(0, 0), v(4, 0), v(0, 3)]);
    let [e0, e1, e2] = t.edges();
    assert_eq!(e0, Segment { a: v(0, 0), b: v(4, 0) });
    assert_eq!(e1, Segment { a: v(4, 0), b: v(0, 3) });
    assert_eq!(e2, Segment { a: v(0, 3), b: v(0, 0) });
    assert_eq!(t.edges_scaled_directions(), [v(4, 0), v(-4, 3), v(0, -3)]);
    let mut r = t;
    r.reverse();
    assert_eq!(r, cw());
    let from: Triangle = [v(0, 0), v(4, 0), v(0, 3)].into();
    assert_eq!(from, t);
    assert_eq!(TriangleTrait::from_array([v(0, 0), v(4, 0), v(0, 3)]), t);
    assert_eq!(t.scaled(v(2, -1)), TriangleTrait::new(v(0, 0), v(8, 0), v(0, -3)));
}

#[test]
fn test_contains_point_both_orientations() {
    let cases: Span<(Vec2, bool)> = array![
        (v(1, 1), true), (v(0, 0), true), (v(2, 0), true), (v(4, 0), true), (v(5, 0), false),
        (v(-1, 1), false), (v(3, 3), false), (v(0, 3), true),
    ]
        .span();
    for (p, inside) in cases {
        assert_eq!(ccw().contains_point(*p), *inside);
        assert_eq!(cw().contains_point(*p), *inside);
    }
    // A degenerate triangle contains the points of its segment's line inside its sign pattern.
    assert!(flat().contains_point(v(1, 0)));
}

#[test]
fn test_support_point_edge_and_extents() {
    let t = ccw();
    // Ties: `c` wins against `a` (upstream's `>` order).
    let cases: Span<(Vec2, Vec2, (Fixed, Fixed))> = array![
        (v(1, 0), v(4, 0), (ZERO, int(4))), (v(0, 1), v(0, 3), (ZERO, int(3))),
        (v(-1, -1), v(0, 0), (int(-4), ZERO)), (v(-1, 0), v(0, 3), (int(-4), ZERO)),
    ]
        .span();
    for (dir, support, extents) in cases {
        assert_eq!(t.local_support_point(*dir), *support);
        assert_eq!(t.extents_on_dir(*dir), *extents);
    }
    // The edge opposite the vertex of smallest dot (`a` first, then `b`, on ties).
    let edges: Span<(Vec2, Segment)> = array![
        (v(1, 0), Segment { a: v(4, 0), b: v(0, 3) }),
        (v(-1, 0), Segment { a: v(0, 3), b: v(0, 0) }),
        (v(0, -1), Segment { a: v(0, 0), b: v(4, 0) }),
    ]
        .span();
    for (dir, edge) in edges {
        assert_eq!(t.local_support_edge_segment(*dir), *edge);
    }
}

#[test]
fn test_support_face_ids_and_core_mapping() {
    let t = ccw();
    let f = t.support_face(v(0, -1));
    assert_eq!(f.vertices, [v(0, 0), v(4, 0)]);
    assert_eq!(f.vids, [FeatureIdTrait::vertex(0), FeatureIdTrait::vertex(1)]);
    assert_eq!(f.fid, FeatureIdTrait::face(0));
    let f = t.support_face(v(1, 1));
    assert_eq!(f.fid, FeatureIdTrait::face(1));
    let f = t.support_face(v(-1, 0));
    assert_eq!(f.vids, [FeatureIdTrait::vertex(2), FeatureIdTrait::vertex(0)]);
    // The core of a counter-clockwise triangle is itself; its features map to the same ids.
    let (core, reversed) = triangle_core(t);
    assert!(!reversed);
    assert_eq!(core.count, 3);
    for dir in array![v(0, -1), v(1, 1), v(-1, 0)].span() {
        assert_eq!(feature_to_triangle(core.support_feature(*dir), false), t.support_face(*dir));
    }
    // Clockwise: the core runs a, c, b; ids follow the triangle's vertices.
    let (core, reversed) = triangle_core(cw());
    assert!(reversed);
    assert_eq!(core.vertex(1), v(4, 0));
    let f = feature_to_triangle(core.support_feature(v(0, -1)), true);
    assert_eq!(f.vertices, [v(0, 0), v(4, 0)]);
    assert_eq!(f.vids, [FeatureIdTrait::vertex(0), FeatureIdTrait::vertex(2)]);
    assert_eq!(f.fid, FeatureIdTrait::face(2));
    let table: Span<(u32, u32, u32)> = array![(0, 0, 2), (1, 2, 1), (2, 1, 0)].span();
    for (k, vid, eid) in table {
        assert_eq!(core_vertex_id(*k, true), *vid);
        assert_eq!(core_edge_id(*k, true), *eid);
        assert_eq!(core_vertex_id(*k, false), *k);
        assert_eq!(core_edge_id(*k, false), *k);
    }
}

#[test]
fn test_circumcircle_table() {
    // Right triangle: the centre is the hypotenuse's midpoint, radius 2.5.
    let (c, r) = ccw().circumcircle();
    assert_eq!(c, Vec2 { x: TWO, y: FixedTrait::from_ratio(3, 2) });
    assert_eq!(r, FixedTrait::from_ratio(5, 2));
    let (c, r) = cw().circumcircle();
    assert_eq!(c, Vec2 { x: TWO, y: FixedTrait::from_ratio(3, 2) });
    assert_eq!(r, FixedTrait::from_ratio(5, 2));
    // Degenerate: the longest edge `a c`.
    let (c, r) = flat().circumcircle();
    assert_eq!(c, Vec2 { x: FixedTrait::from_ratio(5, 2), y: ZERO });
    assert_eq!(r, FixedTrait::from_ratio(5, 2));
}

#[test]
fn test_angle_closest_to_90_and_barycentric() {
    // The right angle is at `a`, i.e. vertex `i + 1 = 0`: `i = 2`.
    assert_eq!(ccw().angle_closest_to_90(), 2);
    assert_eq!(TriangleTrait::new(v(4, 0), v(0, 0), v(0, 3)).angle_closest_to_90(), 0);
    let q = FixedTrait::from_ratio(1, 4);
    let cases: Span<(TrianglePointLocation, Option<(Fixed, Fixed, Fixed)>, bool)> = array![
        (TrianglePointLocation::OnVertex(0), Some((ONE, ZERO, ZERO)), false),
        (TrianglePointLocation::OnVertex(2), Some((ZERO, ZERO, ONE)), false),
        (TrianglePointLocation::OnEdge((0, (q, ONE - q))), Some((q, ONE - q, ZERO)), false),
        (TrianglePointLocation::OnEdge((1, (q, ONE - q))), Some((ZERO, q, ONE - q)), false),
        (TrianglePointLocation::OnEdge((2, (q, ONE - q))), Some((q, ZERO, ONE - q)), false),
        (TrianglePointLocation::OnFace((0, (q, q, HALF))), Some((q, q, HALF)), true),
        (TrianglePointLocation::OnSolid, None, false),
    ]
        .span();
    for (loc, bary, face) in cases {
        assert_eq!((*loc).barycentric_coordinates(), *bary);
        assert_eq!((*loc).is_on_face(), *face);
    }
}

#[test]
fn test_aabb_sphere_mass_and_feature_normal() {
    let t = ccw();
    assert_eq!(t.local_aabb(), Aabb { mins: v(0, 0), maxs: v(4, 3) });
    let quarter = Pose2Trait::new(v(1, 1), Rot2 { re: ZERO, im: ONE });
    assert_eq!(t.aabb(quarter), Aabb { mins: v(-2, 1), maxs: v(1, 5) });
    let s = t.local_bounding_sphere();
    assert_eq!(s.center, t.center());
    assert!(s.radius > int(2) && s.radius < int(3));
    assert_eq!(t.bounding_sphere(quarter).center, quarter.transform_point(t.center()));
    let m = t.mass_properties(TWO);
    assert!((m.mass() - int(12)).abs() < Fixed { raw: 1000 });
    assert_eq!(m.local_com, t.center());
    // `(16 + 9) / 6 * 6 * 2 = 50`, about `a`, as upstream.
    let inertia = m.principal_inertia();
    assert!(inertia > int(49) && inertia < int(51));
    assert_eq!(flat().mass_properties(TWO).mass(), ZERO);
    // Outward normals whatever the orientation.
    assert_eq!(t.feature_normal(FeatureIdTrait::face(0)), Some(v(0, -1)));
    assert_eq!(cw().feature_normal(FeatureIdTrait::face(2)), Some(v(0, -1)));
    assert_eq!(t.feature_normal(FeatureIdTrait::face(2)), Some(v(-1, 0)));
    let n = t.feature_normal(FeatureIdTrait::vertex(0)).unwrap();
    assert!(n.x < ZERO && n.y < ZERO);
    assert_eq!(flat().feature_normal(FeatureIdTrait::face(0)), None);
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_area() {
    let _ = opaque(ccw()).area();
}

#[test]
fn gas_local_support_point() {
    let _ = opaque(ccw()).local_support_point(opaque(v(1, 1)));
}

#[test]
fn gas_support_face() {
    let _ = opaque(ccw()).support_face(opaque(v(1, 1)));
}

#[test]
fn gas_triangle_core() {
    let _ = triangle_core(opaque(cw()));
}

#[test]
fn gas_compute_aabb() {
    let _ = opaque(ccw())
        .compute_aabb(opaque(Pose2Trait::new(v(1, 1), Rot2 { re: ZERO, im: ONE })));
}

#[test]
fn gas_mass_properties() {
    let _ = opaque(ccw()).mass_properties(opaque(ONE));
}

#[test]
fn gas_circumcircle() {
    let _ = opaque(ccw()).circumcircle();
}

#[test]
fn gas_contains_point() {
    let _ = opaque(ccw()).contains_point(opaque(v(1, 1)));
}
