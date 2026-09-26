//! Tests and gas probes of the MH1 `Aabb` helpers (the original `aabb::tests` stay unchanged).

use fixed::{Fixed, FixedTrait, MAX, TWO, ZERO};
use glam::Vec2;
use rapier_testing::opaque;
use super::bounding_volume::BoundingSphere;
use super::{Aabb, AabbTrait, alternatives};

fn i(n: i32) -> Fixed {
    FixedTrait::from_int(n)
}

fn v(x: i32, y: i32) -> Vec2 {
    Vec2 { x: i(x), y: i(y) }
}

fn aabb(x0: i32, y0: i32, x1: i32, y1: i32) -> Aabb {
    Aabb { mins: v(x0, y0), maxs: v(x1, y1) }
}

const A: Aabb = Aabb {
    mins: Vec2 { x: Fixed { raw: -4294967296 }, y: Fixed { raw: -8589934592 } },
    maxs: Vec2 { x: Fixed { raw: 12884901888 }, y: Fixed { raw: 8589934592 } },
};

#[test]
fn test_invalid_points_and_growth() {
    let invalid = AabbTrait::new_invalid();
    assert_eq!(invalid.mins, Vec2 { x: MAX, y: MAX });
    assert_eq!(invalid.maxs, Vec2 { x: -MAX, y: -MAX });
    assert_eq!(invalid.merged(A), A);
    let mut grown = invalid;
    grown.take_point(v(1, 2));
    assert_eq!(grown, aabb(1, 2, 1, 2));
    grown.take_point(v(-3, 5));
    assert_eq!(grown, aabb(-3, 2, 1, 5));
    assert_eq!(
        AabbTrait::from_points(array![v(1, 2), v(-3, 0), v(2, -1)].span()), aabb(-3, -1, 2, 2),
    );
    assert_eq!(AabbTrait::from_points(array![v(1, 2)].span()), aabb(1, 2, 1, 2));
}

#[test]
#[should_panic(expected: 'Bounding: empty point cloud')]
fn test_from_no_point_panics() {
    let _ = AabbTrait::from_points(array![].span());
}

#[test]
fn test_measures_and_moves_table() {
    // (box, half perimeter, translated by (1, 1), grown by (1, 2)).
    let cases: Span<(Aabb, Fixed, Aabb, Aabb)> = array![
        (A, i(8), aabb(0, -1, 4, 3), aabb(-2, -4, 4, 4)),
        (aabb(0, 0, 0, 0), ZERO, aabb(1, 1, 1, 1), aabb(-1, -2, 1, 2)),
        (aabb(2, -5, 3, -1), i(5), aabb(3, -4, 4, 0), aabb(1, -7, 4, 1)),
    ]
        .span();
    for (b, half_perimeter, translated, grown) in cases {
        assert_eq!((*b).half_perimeter(), *half_perimeter);
        assert_eq!((*b).half_area_or_perimeter(), *half_perimeter);
        assert_eq!((*b).translated(v(1, 1)), *translated);
        assert_eq!((*b).add_half_extents(v(1, 2)), *grown);
    }
}

#[test]
fn test_bounding_sphere() {
    // Centre (1, 0), radius |(4, 4)| / 2 = 2 sqrt(2): floor(isqrt) then floor(half).
    let sphere = A.bounding_sphere();
    assert_eq!(sphere, BoundingSphere { center: v(1, 0), radius: Fixed { raw: 12148001999 } });
    assert_eq!(alternatives::bounding_sphere_half_extents(A), sphere);
    // A flat box: the half length.
    assert_eq!(aabb(0, 0, 4, 0).bounding_sphere(), BoundingSphere { center: v(2, 0), radius: TWO });
}

#[test]
fn test_intersection_table() {
    let cases: Span<(Aabb, Option<Aabb>)> = array![
        (aabb(1, 2, 4, 5), Some(aabb(1, 2, 3, 2))), (aabb(0, -1, 1, 1), Some(aabb(0, -1, 1, 1))),
        (aabb(4, 0, 5, 1), None), (aabb(-5, -5, 5, 5), Some(A)), (aabb(0, 3, 1, 4), None),
    ]
        .span();
    for (other, expected) in cases {
        assert_eq!(A.intersection(*other), *expected);
        assert_eq!((*other).intersection(A), *expected);
    }
}

#[test]
fn test_vertices_faces_and_quadrants() {
    let [p0, p1, p2, p3] = A.vertices();
    assert_eq!((p0, p1, p2, p3), (v(-1, -2), v(3, -2), v(3, 2), v(-1, 2)));
    let vertices = array![p0, p1, p2, p3];
    let [f0, f1, f2, f3] = AabbTrait::FACES_VERTEX_IDS;
    // +x, -x, +y, -y faces.
    let (a, b) = f0;
    assert!(*vertices.at(a).x == A.maxs.x && *vertices.at(b).x == A.maxs.x);
    let (a, b) = f1;
    assert!(*vertices.at(a).x == A.mins.x && *vertices.at(b).x == A.mins.x);
    let (a, b) = f2;
    assert!(*vertices.at(a).y == A.maxs.y && *vertices.at(b).y == A.maxs.y);
    let (a, b) = f3;
    assert!(*vertices.at(a).y == A.mins.y && *vertices.at(b).y == A.mins.y);
    let [q0, q1, q2, q3] = A.split_at_center();
    assert_eq!(
        (q0, q1, q2, q3),
        (aabb(-1, -2, 1, 0), aabb(1, -2, 3, 0), aabb(1, 0, 3, 2), aabb(-1, 0, 1, 2)),
    );
}

#[test]
fn test_difference_table() {
    // Hole in the middle: four pieces, cut below / above on x then y.
    let (pieces, cuts) = A.difference_with_cut_sequence(aabb(0, -1, 1, 1));
    assert_eq!(
        pieces, array![aabb(-1, -2, 0, 2), aabb(1, -2, 3, 2), aabb(0, -2, 1, -1), aabb(0, 1, 1, 2)],
    );
    assert_eq!(cuts, array![(1, i(0)), (-1, i(-1)), (2, i(-1)), (-2, i(-1))]);
    // Touching boxes do not cut; a superset removes everything; a corner overlap cuts twice.
    let (pieces, cuts) = A.difference_with_cut_sequence(aabb(3, 0, 5, 1));
    assert_eq!((pieces, cuts.len()), (array![A], 0));
    assert_eq!(A.difference(aabb(-5, -5, 5, 5)).len(), 0);
    assert_eq!(A.difference(aabb(2, 1, 5, 5)), array![aabb(-1, -2, 2, 2), aabb(2, -2, 3, 1)]);
}

#[test]
fn gas_baseline() {}
#[test]
fn gas_new_invalid() {
    let _ = opaque(AabbTrait::new_invalid());
}
#[test]
fn gas_from_points_3() {
    let _ = AabbTrait::from_points(opaque(array![v(1, 2), v(-3, 0), v(2, -1)]).span());
}
#[test]
fn gas_half_perimeter() {
    let _ = opaque(A).half_perimeter();
}
#[test]
fn gas_take_point() {
    let mut a = opaque(A);
    a.take_point(opaque(v(5, 5)));
}
#[test]
fn gas_translated() {
    let _ = opaque(A).translated(opaque(v(1, 1)));
}
#[test]
fn gas_add_half_extents() {
    let _ = opaque(A).add_half_extents(opaque(v(1, 2)));
}
#[test]
fn gas_bounding_sphere() {
    let _ = opaque(A).bounding_sphere();
}
#[test]
fn gas_bounding_sphere_half_extents() {
    let _ = alternatives::bounding_sphere_half_extents(opaque(A));
}
#[test]
fn gas_intersection() {
    let _ = opaque(A).intersection(opaque(aabb(1, 2, 4, 5)));
}
#[test]
fn gas_vertices() {
    let _ = opaque(A).vertices();
}
#[test]
fn gas_split_at_center() {
    let _ = opaque(A).split_at_center();
}
#[test]
fn gas_difference_hole() {
    let _ = opaque(A).difference(opaque(aabb(0, -1, 1, 1)));
}
