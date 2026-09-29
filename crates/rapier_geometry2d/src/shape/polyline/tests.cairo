use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam_core::{Vec2, Vec2Trait};
use rapier_testing::opaque;
use crate::aabb::Aabb;
use crate::feature_id::FeatureIdTrait;
use crate::shape::segment::SegmentTrait;
use super::alternatives::segments_in_aabb_linear;
use super::{ORIENTED, Polyline, PolylineFlags, PolylineFlagsTrait, PolylineTrait};

fn v(x: i32, y: i32) -> Vec2 {
    Vec2 { x: FixedTrait::from_int(x), y: FixedTrait::from_int(y) }
}

fn ccw_square() -> Polyline {
    PolylineTrait::new(
        array![v(-1, -1), v(1, -1), v(1, 1), v(-1, 1)].span(),
        Some(array![[0, 1], [1, 2], [2, 3], [3, 0]].span()),
    )
}

/// A zigzag ground of `n` segments over `x = 0..n`.
fn zigzag(n: u32) -> Polyline {
    let mut vertices = array![];
    let mut i: u32 = 0;
    while i != n + 1 {
        let x: i32 = i.try_into().unwrap();
        vertices.append(v(x, if i % 2 == 0 {
            0
        } else {
            1
        }));
        i += 1;
    }
    PolylineTrait::new(vertices.span(), None)
}

fn near(a: Fixed, b: Fixed, ulps: i64) -> bool {
    let d = a.raw - b.raw;
    d <= ulps && -d <= ulps
}

#[test]
fn test_construction_and_accessors() {
    let p = PolylineTrait::new(array![v(0, 0), v(1, 0), v(2, 1)].span(), None);
    assert_eq!(p.num_segments(), 2);
    assert_eq!(p.indices(), array![[0, 1], [1, 2]].span());
    assert_eq!(p.flat_indices(), array![0, 1, 1, 2]);
    assert_eq!(p.segment(1), SegmentTrait::new(v(1, 0), v(2, 1)));
    assert_eq!(p.segments().len(), 2);
    assert_eq!(p.local_aabb(), Aabb { mins: v(0, 0), maxs: v(2, 1) });
    assert_eq!(p.flags(), PolylineFlagsTrait::empty());
    assert!(p.pseudo_normals().is_none());
    assert!(p.segment_normal_constraints(0).is_none());
    assert_eq!(
        p.segment_feature_to_polyline_feature(1, FeatureIdTrait::vertex(0)),
        FeatureIdTrait::face(1),
    );
    // Upstream's degenerate inputs: no segment.
    assert_eq!(PolylineTrait::new(array![].span(), None).num_segments(), 0);
    assert_eq!(PolylineTrait::new(array![v(1, 1)].span(), None).num_segments(), 0);
    assert_eq!(PolylineTrait::new(array![].span(), None).local_aabb(), Default::default());
    let zero: crate::mass::MassProperties = Default::default();
    assert_eq!(p.mass_properties(ONE), zero);
}

#[test]
#[should_panic(expected: 'Polyline: vertex index')]
fn test_index_out_of_range() {
    let _ = PolylineTrait::new(array![v(0, 0), v(1, 0)].span(), Some(array![[0, 2]].span()));
}

#[test]
fn test_flags_and_pseudo_normals() {
    let flags: PolylineFlags = PolylineFlagsTrait::empty() | ORIENTED;
    assert!(flags.contains(ORIENTED));
    assert!(!PolylineFlagsTrait::empty().contains(ORIENTED));
    let mut p = ccw_square();
    p.set_flags(ORIENTED);
    assert!(p.is_oriented());
    // Upstream `corner_pseudo_normal_bisects_its_two_faces`: vertex 1 bisects -Y and +X.
    let c = p.segment_normal_constraints(0).unwrap();
    assert_eq!(c.face, v(0, -1));
    let [_, e1] = c.edges;
    assert!(near(e1.x, FixedTrait::from_raw(3037000499), 2));
    assert!(near(e1.y, FixedTrait::from_raw(-3037000499), 2));
    // Every face normal points away from the centre (outward).
    let mut i: u32 = 0;
    while i != 4 {
        let c = p.segment_normal_constraints(i).unwrap();
        let s = p.segment(i);
        let mid = (s.a + s.b).mul_scalar(HALF);
        assert!(c.face.dot(mid) > ZERO);
        i += 1;
    }
    p.set_flags(PolylineFlagsTrait::empty());
    assert!(p.pseudo_normals().is_none());
}

#[test]
fn test_tree_prefilter_matches_linear_scan() {
    // Boxes of several sizes and positions against grounds of several sizes (padding leaves).
    let boxes = array![
        Aabb { mins: v(-5, -5), maxs: v(-1, -1) }, Aabb { mins: v(0, 0), maxs: v(0, 0) },
        Aabb { mins: v(2, 0), maxs: v(3, 2) }, Aabb { mins: v(-10, -10), maxs: v(100, 100) },
        Aabb { mins: v(6, 2), maxs: v(9, 3) },
        Aabb {
            mins: Vec2 { x: FixedTrait::from_ratio(9, 2), y: ZERO },
            maxs: Vec2 { x: FixedTrait::from_ratio(11, 2), y: HALF },
        },
    ];
    for n in array![1_u32, 2, 3, 7, 8, 9, 13] {
        let p = zigzag(n);
        for b in boxes.span() {
            assert_eq!(p.segments_in_aabb(*b), segments_in_aabb_linear(@p, *b));
        }
    }
    // The tree: `2 * leaf_base` nodes, the root box is the polyline's.
    let p = zigzag(5);
    assert_eq!(p.bvh().len(), 16);
    assert_eq!(*p.bvh().at(1), p.local_aabb());
}

#[test]
fn test_scaled_reverse_set_vertices_components() {
    let p = ccw_square();
    let s = p.scaled(v(2, 3));
    assert_eq!(s.segment(0), SegmentTrait::new(v(-2, -3), v(2, -3)));
    let mut r = ccw_square();
    r.reverse();
    assert_eq!(r.indices(), array![[0, 3], [3, 2], [2, 1], [1, 0]].span());
    assert_eq!(r.local_aabb(), p.local_aabb());
    let mut m = ccw_square();
    m.set_vertices(array![v(0, 0), v(2, 0), v(2, 2), v(0, 2)].span());
    assert_eq!(m.local_aabb(), Aabb { mins: v(0, 0), maxs: v(2, 2) });
    // Two closed loops chained: two components with their own vertices.
    let two = PolylineTrait::new(
        array![v(0, 0), v(1, 0), v(0, 1), v(5, 5), v(6, 5), v(5, 6)].span(),
        Some(array![[0, 1], [1, 2], [2, 0], [3, 4], [4, 5], [5, 3]].span()),
    );
    let components = two.extract_connected_components();
    assert_eq!(components.len(), 2);
    assert_eq!(components.at(1).vertices(), array![v(5, 5), v(6, 5), v(5, 6)].span());
    assert_eq!(components.at(1).indices(), array![[0, 1], [1, 2], [2, 0]].span());
}

#[test]
#[should_panic(expected: 'Polyline: vertex count')]
fn test_set_vertices_count() {
    let mut p = ccw_square();
    p.set_vertices(array![v(0, 0)].span());
}

#[test]
fn test_serde_round_trip() {
    let mut p = ccw_square();
    p.set_flags(ORIENTED);
    let mut out = array![];
    core::serde::Serde::serialize(@p, ref out);
    let mut span = out.span();
    let back: Polyline = core::serde::Serde::deserialize(ref span).unwrap();
    assert_eq!(back, p);
}

#[test]
fn gas_baseline() {}

/// A ball-sized box over the middle of the ground.
fn probe_box(n: u32) -> Aabb {
    let x: i32 = (n / 2).try_into().unwrap();
    Aabb {
        mins: Vec2 { x: FixedTrait::from_int(x) + HALF, y: ZERO },
        maxs: Vec2 { x: FixedTrait::from_int(x + 1), y: HALF },
    }
}

// The prefilter of the contact manifolds: implicit tree (shipped) against the linear scan.
#[test]
fn gas_segments_in_aabb_tree_10() {
    let p = zigzag(10);
    let _ = opaque(@p).segments_in_aabb(opaque(probe_box(10)));
}
#[test]
fn gas_segments_in_aabb_linear_10() {
    let p = zigzag(10);
    let _ = segments_in_aabb_linear(opaque(@p), opaque(probe_box(10)));
}
#[test]
fn gas_segments_in_aabb_tree_50() {
    let p = zigzag(50);
    let _ = opaque(@p).segments_in_aabb(opaque(probe_box(50)));
}
#[test]
fn gas_segments_in_aabb_linear_50() {
    let p = zigzag(50);
    let _ = segments_in_aabb_linear(opaque(@p), opaque(probe_box(50)));
}
#[test]
fn gas_segments_in_aabb_tree_200() {
    let p = zigzag(200);
    let _ = opaque(@p).segments_in_aabb(opaque(probe_box(200)));
}
#[test]
fn gas_segments_in_aabb_linear_200() {
    let p = zigzag(200);
    let _ = segments_in_aabb_linear(opaque(@p), opaque(probe_box(200)));
}
#[test]
fn gas_new_50() {
    let mut vertices = array![];
    let mut i: i32 = 0;
    while i != 51 {
        vertices.append(v(i, i % 2));
        i += 1;
    }
    let _ = PolylineTrait::new(opaque(vertices.span()), None);
}
