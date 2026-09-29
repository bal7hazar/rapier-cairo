use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam_core::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::aabb::{Aabb, AabbTrait};
use crate::contact::{ContactManifold, ContactManifoldTrait};
use crate::dispatch::composite::contact_manifolds_composite;
use crate::mass::{MassProperties, MassPropertiesTrait};
use crate::point::PointQuery;
use crate::ray::{Ray, cast_local_ray_and_get_normal};
use crate::shape::{
    BallTrait, CapsuleTrait, CuboidTrait, HalfSpaceTrait, HeightFieldTrait, PolylineTrait, Shape,
    ShapeTrait, ShapeType, TriangleTrait,
};
use super::alternatives::{parts_in_aabb_tree, tree};
use super::{Compound, CompoundTrait, part_mass_properties};

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: v(x, y), ..Default::default() }
}

fn cuboid(hx: Fixed, hy: Fixed) -> Shape {
    CuboidTrait::new(v(hx, hy)).into()
}

/// An L: a 2 x 0.5 bar and a 0.5 x 1.5 arm on its left end.
fn ell() -> Compound {
    CompoundTrait::new(
        array![
            (at(ZERO, ZERO), cuboid(ONE, HALF / TWO)),
            (at(-HALF - HALF / TWO, ONE), cuboid(HALF / TWO, HALF + HALF / TWO)),
        ]
            .span(),
    )
}

/// A ball, a turned cuboid and a capsule.
fn trio() -> Compound {
    let turned = Pose2 { translation: v(ZERO, ZERO), rotation: Rot2 { re: ZERO, im: ONE } };
    CompoundTrait::new(
        array![
            (at(-ONE, ZERO), BallTrait::new(HALF).into()), (turned, cuboid(HALF, HALF / TWO)),
            (at(ONE, ZERO), CapsuleTrait::new_y(HALF, HALF / TWO).into()),
        ]
            .span(),
    )
}

/// `n` unit boxes along `x = 0, 2, 4, …`.
fn row(n: u32) -> Compound {
    let mut parts = array![];
    let mut i: u32 = 0;
    while i != n {
        let x: i32 = (2 * i).try_into().unwrap();
        parts.append((at(int(x), ZERO), cuboid(HALF, HALF)));
        i += 1;
    }
    CompoundTrait::new(parts.span())
}

#[test]
fn test_new_boxes_and_accessors() {
    let c = ell();
    assert_eq!(c.num_parts(), 2);
    let aabbs = c.aabbs();
    let quarter = HALF / TWO;
    assert_eq!(*aabbs.at(0), Aabb { mins: v(-ONE, -quarter), maxs: v(ONE, quarter) });
    assert_eq!(*aabbs.at(1), Aabb { mins: v(-ONE, quarter), maxs: v(-HALF, int(2) - quarter) });
    assert_eq!(c.local_aabb(), Aabb { mins: v(-ONE, -quarter), maxs: v(ONE, int(2) - quarter) });
    let (pose, _) = c.part(1);
    assert_eq!(c.part_pose(1), pose);
    let s: Shape = c.into();
    assert_eq!(s.shape_type(), ShapeType::Compound);
    assert!(s.is_composite() && !s.is_convex());
    assert_eq!(s.as_compound(), Some(c));
    assert_eq!(s.as_support_map(), None);
    assert_eq!(s.compute_local_aabb(), c.local_aabb());
    let p = at(int(3), int(-2));
    assert_eq!(s.compute_aabb(p), c.local_aabb().transform_by(p));
    assert_eq!(s.compute_local_bounding_sphere(), c.local_aabb().bounding_sphere());
}

#[test]
fn test_serde_round_trip() {
    let s: Shape = trio().into();
    let mut out = array![];
    s.serialize(ref out);
    assert_eq!(*out.at(0), 12);
    let mut span = out.span();
    let back: Shape = Serde::deserialize(ref span).unwrap();
    assert_eq!(back, s);
    assert!(span.is_empty());
}

#[test]
#[should_panic(expected: 'Compound: no part')]
fn test_new_empty() {
    let _ = CompoundTrait::new(array![].span());
}

#[test]
#[should_panic(expected: 'Compound: nested composite')]
fn test_new_nested_polyline() {
    let line: Shape = PolylineTrait::new(array![v(ZERO, ZERO), v(ONE, ZERO)].span(), None).into();
    let _ = CompoundTrait::new(array![(at(ZERO, ZERO), line)].span());
}

#[test]
#[should_panic(expected: 'Compound: nested composite')]
fn test_new_nested_compound() {
    let inner: Shape = ell().into();
    let _ = CompoundTrait::new(
        array![(at(ZERO, ZERO), cuboid(ONE, ONE)), (at(ONE, ONE), inner)].span(),
    );
}

#[test]
#[should_panic(expected: 'Compound: nested composite')]
fn test_new_nested_heightfield() {
    let h: Shape = HeightFieldTrait::new(array![ZERO, ONE].span(), v(ONE, ONE)).into();
    let _ = CompoundTrait::new(array![(at(ZERO, ZERO), h)].span());
}

#[test]
#[should_panic(expected: 'Compound: part index')]
fn test_part_index() {
    let _ = ell().part(2);
}

#[test]
fn test_mass_properties_sum_of_parts() {
    let density = TWO;
    for c in array![ell(), trio(), row(3)] {
        let mut sum: MassProperties = Default::default();
        for part in c.shapes() {
            let (pose, shape) = *part;
            sum = sum + shape.mass_properties(density).transform_by(pose);
        }
        let s: Shape = c.into();
        assert_eq!(s.mass_properties(density), sum);
        assert_eq!(c.mass_properties(density), sum);
    }
    // A half-space part has no mass; a composite (rejected by `new`) none either.
    let h: Shape = HalfSpaceTrait::new(v(ZERO, ONE)).into();
    assert_eq!(part_mass_properties(h, ONE), Default::default());
}

#[test]
fn test_parts_in_aabb_scan_matches_tree() {
    let boxes = array![
        Aabb { mins: v(int(-1), int(-1)), maxs: v(int(1), int(1)) },
        Aabb { mins: v(int(3), ZERO), maxs: v(int(5), HALF) },
        Aabb { mins: v(int(100), ZERO), maxs: v(int(101), ONE) },
        Aabb { mins: v(int(-10), int(-10)), maxs: v(int(40), int(10)) },
        Aabb { mins: v(HALF, ZERO), maxs: v(int(1), ONE) },
    ];
    for n in array![1_u32, 2, 3, 5, 8, 13] {
        let c = row(n);
        let (nodes, leaf_base) = tree(@c);
        for b in boxes.span() {
            assert_eq!(c.parts_in_aabb(*b), parts_in_aabb_tree(nodes, leaf_base, n, *b));
        }
    }
    assert_eq!(row(8).parts_in_aabb(*boxes.at(1)), array![2]);
    assert_eq!(row(8).parts_in_aabb(*boxes.at(4)), array![0]);
}

#[test]
fn test_point_queries_through_parts() {
    let s: Shape = ell().into();
    // `(point, solid projection, inside, contains)`: above the bar, inside the arm, on the right.
    let quarter = HALF / TWO;
    let cases = array![
        (v(HALF, ONE), v(HALF, quarter), false, false),
        (v(-HALF - quarter, ONE), v(-HALF - quarter, ONE), true, true),
        (v(int(3), ZERO), v(ONE, ZERO), false, false),
    ];
    for (pt, proj, inside, contains) in cases {
        let p = s.project_local_point(pt, true);
        assert_eq!((p.point, p.is_inside), (proj, inside));
        assert_eq!(s.contains_local_point(pt), contains);
    }
    // Hollow from inside the arm: its nearest side, negative distance.
    let d = s.distance_to_local_point(v(-HALF - quarter, ONE), false);
    assert_eq!(d, -quarter);
}

#[test]
fn test_ray_hits_turned_part() {
    let s: Shape = trio().into();
    // Straight down onto the cuboid turned by a quarter: its half extents swap (0.25 wide, 0.5
    // tall), its normal comes back rotated into the compound's frame.
    let ray = Ray { origin: v(ZERO, int(3)), dir: v(ZERO, -ONE) };
    let hit = cast_local_ray_and_get_normal(s, ray, int(10), true).unwrap();
    assert_eq!(hit.time_of_impact, int(3) - HALF);
    assert_eq!(hit.normal, v(ZERO, ONE));
}

#[test]
fn test_manifolds_carry_part_ids_and_frames() {
    let c = ell();
    let s: Shape = c.into();
    let ball: Shape = BallTrait::new(HALF).into();
    // A ball resting on the bar's right end, in the compound's frame.
    let quarter = HALF / TWO;
    let pos12 = at(HALF, quarter + HALF);
    let prediction = HALF / int(10);
    let ms = contact_manifolds_composite(pos12, s, ball, prediction, array![].span()).unwrap();
    assert_eq!(ms.len(), 1);
    let m: ContactManifold = *ms.at(0);
    assert_eq!((m.subshape1, m.subshape2, m.num_points), (0, 0, 1));
    // Reversed: the part id goes to `subshape2`, points in the part's frame.
    let ms = contact_manifolds_composite(pos12.inverse(), ball, s, prediction, array![].span())
        .unwrap();
    let m: ContactManifold = *ms.at(0);
    assert_eq!((m.subshape1, m.subshape2, m.num_points), (0, 0, 1));
    // The arm's part frame: a ball against the arm's left side.
    let pos12 = at(-ONE - HALF, ONE);
    let ms = contact_manifolds_composite(pos12, s, ball, prediction, array![].span()).unwrap();
    let m: ContactManifold = *ms.at(0);
    assert_eq!((m.subshape1, m.num_points), (1, 1));
    // The arm is centred at (-0.75, 1): its left face is at x = -0.25 in its own frame.
    assert_eq!(m.point(0).local_p1, v(-quarter, ZERO));
}

#[test]
fn test_compound_pairs_support_matrix() {
    let s: Shape = ell().into();
    let t: Shape = trio().into();
    let line: Shape = PolylineTrait::new(array![v(int(-3), ZERO), v(int(3), ZERO)].span(), None)
        .into();
    let hf: Shape = HeightFieldTrait::new(array![ZERO, ZERO, ZERO].span(), v(int(6), ONE)).into();
    let above = at(ZERO, HALF);
    let prediction = HALF / int(10);
    for (a, b) in array![(line, s), (s, line), (hf, t), (t, hf), (s, t), (t, s)] {
        let got = contact_manifolds_composite(above, a, b, prediction, array![].span());
        assert!(got.is_some());
    }
    // Two composites without a compound stay unsupported (SH2a).
    assert!(contact_manifolds_composite(above, line, hf, prediction, array![].span()).is_none());
}

#[test]
fn gas_baseline() {}

/// The middle part's box of `row(n)`.
fn probe_box(n: u32) -> Aabb {
    let x: i32 = (n - 1).try_into().unwrap();
    Aabb { mins: v(int(x) - HALF / TWO, ZERO), maxs: v(int(x) + HALF / TWO, ONE) }
}

// The prefilter of the compound's manifolds: the scan of the part boxes (shipped) against the
// polyline's implicit tree (built outside the probe: `gas_tree_build_*` is its setup).
#[test]
fn gas_parts_in_aabb_scan_2() {
    let c = row(2);
    let _ = opaque(@c).parts_in_aabb(opaque(probe_box(2)));
}
#[test]
fn gas_parts_in_aabb_tree_2() {
    let c = row(2);
    let (nodes, leaf_base) = tree(@c);
    let _ = parts_in_aabb_tree(opaque(nodes), opaque(leaf_base), 2, opaque(probe_box(2)));
}
#[test]
fn gas_parts_in_aabb_scan_8() {
    let c = row(8);
    let _ = opaque(@c).parts_in_aabb(opaque(probe_box(8)));
}
#[test]
fn gas_parts_in_aabb_tree_8() {
    let c = row(8);
    let (nodes, leaf_base) = tree(@c);
    let _ = parts_in_aabb_tree(opaque(nodes), opaque(leaf_base), 8, opaque(probe_box(8)));
}
#[test]
fn gas_parts_in_aabb_scan_16() {
    let c = row(16);
    let _ = opaque(@c).parts_in_aabb(opaque(probe_box(16)));
}
#[test]
fn gas_parts_in_aabb_tree_16() {
    let c = row(16);
    let (nodes, leaf_base) = tree(@c);
    let _ = parts_in_aabb_tree(opaque(nodes), opaque(leaf_base), 16, opaque(probe_box(16)));
}
#[test]
fn gas_row_2() {
    let _ = opaque(row(2));
}
#[test]
fn gas_row_8() {
    let _ = opaque(row(8));
}
#[test]
fn gas_row_16() {
    let _ = opaque(row(16));
}
#[test]
fn gas_tree_build_2() {
    let c = row(2);
    let _ = opaque(tree(@c));
}
#[test]
fn gas_tree_build_8() {
    let c = row(8);
    let _ = opaque(tree(@c));
}
#[test]
fn gas_tree_build_16() {
    let c = row(16);
    let _ = opaque(tree(@c));
}
#[test]
fn gas_new_ell() {
    let _ = opaque(ell());
}
#[test]
fn gas_compute_aabb_ell() {
    let s: Shape = ell().into();
    let _ = opaque(s).compute_aabb(opaque(at(ONE, TWO)));
}
#[test]
fn gas_mass_properties_ell() {
    let s: Shape = ell().into();
    let _ = opaque(s).mass_properties(opaque(ONE));
}
#[test]
fn gas_project_local_point_ell() {
    let s: Shape = ell().into();
    let _ = opaque(s).project_local_point(opaque(v(HALF, ONE)), true);
}
#[test]
fn gas_cast_local_ray_ell() {
    let s: Shape = ell().into();
    let ray = Ray { origin: v(-HALF, int(3)), dir: v(ZERO, -ONE) };
    let _ = cast_local_ray_and_get_normal(opaque(s), opaque(ray), int(10), true);
}
#[test]
fn gas_manifolds_ell_ball() {
    let s: Shape = ell().into();
    let ball: Shape = BallTrait::new(HALF).into();
    let pos12 = at(HALF, HALF + HALF / TWO);
    let _ = contact_manifolds_composite(
        opaque(pos12), opaque(s), opaque(ball), HALF / int(10), array![].span(),
    );
}
#[test]
fn gas_manifolds_ell_cuboid() {
    let s: Shape = ell().into();
    let c = cuboid(HALF, HALF);
    let pos12 = at(HALF, HALF + HALF / TWO);
    let _ = contact_manifolds_composite(
        opaque(pos12), opaque(s), opaque(c), HALF / int(10), array![].span(),
    );
}
#[test]
fn gas_manifolds_ell_halfspace() {
    let s: Shape = ell().into();
    let h: Shape = HalfSpaceTrait::new(v(ZERO, ONE)).into();
    let pos12 = at(ZERO, -HALF / TWO);
    let _ = contact_manifolds_composite(
        opaque(pos12), opaque(s), opaque(h), HALF / int(10), array![].span(),
    );
}
#[test]
fn gas_manifolds_ell_polyline() {
    let s: Shape = ell().into();
    let line: Shape = PolylineTrait::new(array![v(int(-3), ZERO), v(int(3), ZERO)].span(), None)
        .into();
    let pos12 = at(ZERO, -HALF / TWO);
    let _ = contact_manifolds_composite(
        opaque(pos12), opaque(s), opaque(line), HALF / int(10), array![].span(),
    );
}
#[test]
fn gas_triangle_part_is_convex() {
    let t: Shape = TriangleTrait::new(v(ZERO, ZERO), v(ONE, ZERO), v(ZERO, ONE)).into();
    let _ = opaque(CompoundTrait::new(array![(at(ZERO, ZERO), t)].span()));
}
