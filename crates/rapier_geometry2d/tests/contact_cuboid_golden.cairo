//! Eight cuboid fixtures, including weighted-diagonal corner and quarter turn.
//! Numeric tolerance is the README's 64 raw units; f32 feature ids are exact.
use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait};
use rapier_geometry2d::contact_generators::cuboid_cuboid::{
    contact_manifold_cuboid_cuboid, contact_manifold_cuboid_cuboid_shapes,
};
use rapier_geometry2d::shape::{Cuboid, Shape};
use rapier_golden::compare::{vec2_within, within};
use rapier_golden::contact_manifolds;
use rapier_golden::types::{ManifoldCase, PoseRaw, ShapeRaw, Vec2Raw};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

fn v(p: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: p.x }, y: Fixed { raw: p.y } }
}
fn raw(p: Vec2) -> Vec2Raw {
    Vec2Raw { x: p.x.raw, y: p.y.raw }
}
fn pose(p: PoseRaw) -> Pose2 {
    Pose2 {
        translation: v(p.translation),
        rotation: Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    }
}
fn cuboid(s: ShapeRaw) -> Cuboid {
    let ShapeRaw::Cuboid(h) = s else {
        panic!("cuboid fixture");
    };
    Cuboid { half_extents: v(h) }
}
fn check(m: ContactManifold, c: ManifoldCase) {
    assert_eq!(m.num_points.into(), c.num_points, "{} count", c.id);
    if m.num_points == 0 {
        return;
    }
    assert!(vec2_within(raw(m.local_n1), c.local_n1, 64), "{} n1", c.id);
    assert!(vec2_within(raw(m.local_n2), c.local_n2, 64), "{} n2", c.id);
    let mut i = 0_u8;
    for point in c.points.span() {
        if i == m.num_points {
            break;
        }
        let actual = m.point(i);
        assert!(vec2_within(raw(actual.local_p1), *point.local_p1, 64), "{} p1 {}", c.id, i);
        assert!(vec2_within(raw(actual.local_p2), *point.local_p2, 64), "{} p2 {}", c.id, i);
        assert!(within(actual.dist.raw, *point.dist, 64), "{} dist {}", c.id, i);
        assert_eq!(actual.fid1.packed, *point.fid1, "{} fid1 {}", c.id, i);
        assert_eq!(actual.fid2.packed, *point.fid2, "{} fid2 {}", c.id, i);
        i += 1;
    }
}
#[test]
fn test_every_cuboid_golden() {
    let mut count = 0;
    for c in contact_manifolds::cases() {
        if let (ShapeRaw::Cuboid(_), ShapeRaw::Cuboid(_)) = (*c.shape1, *c.shape2) {
            let mut m: ContactManifold = Default::default();
            assert!(
                contact_manifold_cuboid_cuboid_shapes(
                    pose(*c.pos12),
                    Shape::Cuboid(cuboid(*c.shape1)),
                    Shape::Cuboid(cuboid(*c.shape2)),
                    Fixed { raw: contact_manifolds::PREDICTION },
                    ref m,
                ),
            );
            check(m, *c);
            if *c.id == 'cuboid_cuboid/shallow' {
                assert!(m.point(0).dist.raw > contact_manifolds::PREDICTION);
                assert!(m.point(1).dist.raw < 0);
            }
            count += 1;
        }
    }
    assert_eq!(count, 8);
}
#[test]
fn test_swapped_golden_geometry() {
    // The fixture table has no separately exported cuboid/cuboid swap entries.
    // Reverse nondegenerate pairs and compare their geometry to swapped anchors.
    // Clipping follows the reversed tangent, so the two points reverse order.
    // At coincident endpoints the first shape supplies the vertex id: those ids
    // are covered independently by the explicit touching-order test below.
    for c in array![
        contact_manifolds::CUBOID_CUBOID_SEPARATED, contact_manifolds::CUBOID_CUBOID_WITHIN_PRED,
        contact_manifolds::CUBOID_CUBOID_TOUCHING, contact_manifolds::CUBOID_CUBOID_SHALLOW,
        contact_manifolds::CUBOID_CUBOID_DEEP, contact_manifolds::CUBOID_CUBOID_DEGEN_ROT90,
    ]
        .span() {
        let p = pose(*c.pos12).inverse();
        let mut m: ContactManifold = Default::default();
        contact_manifold_cuboid_cuboid(
            p,
            cuboid(*c.shape2),
            cuboid(*c.shape1),
            Fixed { raw: contact_manifolds::PREDICTION },
            ref m,
        );
        assert_eq!(m.num_points.into(), *c.num_points);
        if m.num_points != 0 {
            assert!(vec2_within(raw(m.local_n1), *c.local_n2, 64), "{} swapped n1", *c.id);
            assert!(vec2_within(raw(m.local_n2), *c.local_n1, 64), "{} swapped n2", *c.id);
            let [e0, e1] = *c.points;
            for (i, expected) in array![(0_u8, e1), (1_u8, e0)].span() {
                let actual = m.point(*i);
                assert!(
                    vec2_within(raw(actual.local_p1), *expected.local_p2, 64),
                    "{} swapped p1",
                    *c.id,
                );
                assert!(
                    vec2_within(raw(actual.local_p2), *expected.local_p1, 64),
                    "{} swapped p2",
                    *c.id,
                );
                assert!(within(actual.dist.raw, *expected.dist, 64), "{} swapped dist", *c.id);
                if *c.id != 'cuboid_cuboid/touching' {
                    assert_eq!(actual.fid1.packed, *expected.fid2, "{} swapped fid1", *c.id);
                    assert_eq!(actual.fid2.packed, *expected.fid1, "{} swapped fid2", *c.id);
                }
            }
        }
    }
}
#[test]
fn test_touching_reversed_feature_ties() {
    let c = contact_manifolds::CUBOID_CUBOID_TOUCHING;
    let mut m: ContactManifold = Default::default();
    contact_manifold_cuboid_cuboid(
        pose(c.pos12).inverse(),
        cuboid(c.shape2),
        cuboid(c.shape1),
        Fixed { raw: contact_manifolds::PREDICTION },
        ref m,
    );
    assert_eq!(m.num_points, 2);
    assert_eq!(m.point(0).fid1.packed, 0x40000001);
    assert_eq!(m.point(1).fid1.packed, 0x40000003);
    assert_eq!(m.point(0).fid2.packed, 0xc0000038);
    assert_eq!(m.point(1).fid2.packed, 0xc0000038);
}
#[test]
fn gas_baseline() {
    let _ = opaque(contact_manifolds::CUBOID_CUBOID_SHALLOW);
}
#[test]
fn gas_golden_cuboid() {
    let c = opaque(contact_manifolds::CUBOID_CUBOID_SHALLOW);
    let mut m = Default::default();
    contact_manifold_cuboid_cuboid(
        pose(c.pos12),
        cuboid(c.shape1),
        cuboid(c.shape2),
        Fixed { raw: contact_manifolds::PREDICTION },
        ref m,
    );
    let _ = opaque(m);
}

#[test]
fn test_swapped_ambiguous_cases_follow_first_axis_and_endpoint_ties() {
    // Swapping coincident centres cannot negate the normal: SAT's zero sign is
    // positive in either order. These expected contacts follow support-face codes.
    let c = contact_manifolds::CUBOID_CUBOID_DEGENERATE;
    let mut m: ContactManifold = Default::default();
    contact_manifold_cuboid_cuboid(
        pose(c.pos12).inverse(),
        cuboid(c.shape2),
        cuboid(c.shape1),
        Fixed { raw: contact_manifolds::PREDICTION },
        ref m,
    );
    assert_eq!(m.num_points, 2);
    assert_eq!(raw(m.local_n1), c.local_n1);
    assert_eq!(raw(m.local_n2), c.local_n2);
    for (i, x, fid) in array![
        (0_u8, 2147483648_i64, 0x40000000_u32), (1_u8, -2147483648, 0x40000001),
    ]
        .span() {
        let pt = m.point(*i);
        assert_eq!(raw(pt.local_p1), Vec2Raw { x: *x, y: 2147483648 });
        assert_eq!(raw(pt.local_p2), Vec2Raw { x: *x, y: -2147483648 });
        assert_eq!(pt.dist.raw, -4294967296);
        assert_eq!(pt.fid1.packed, *fid);
        assert_eq!(pt.fid2.packed, 0xc000003e);
    }
    let c = contact_manifolds::CUBOID_CUBOID_DEGEN_CORNER;
    m = Default::default();
    contact_manifold_cuboid_cuboid(
        pose(c.pos12).inverse(),
        cuboid(c.shape2),
        cuboid(c.shape1),
        Fixed { raw: contact_manifolds::PREDICTION },
        ref m,
    );
    assert_eq!(m.num_points, 2);
    assert!(vec2_within(raw(m.local_n1), c.local_n2, 64));
    assert!(vec2_within(raw(m.local_n2), c.local_n1, 64));
    for i in array![0_u8, 1].span() {
        let pt = m.point(*i);
        assert_eq!(raw(pt.local_p1), Vec2Raw { x: -2147483648, y: -2147483648 });
        assert_eq!(raw(pt.local_p2), Vec2Raw { x: 4294967296, y: 2147483648 });
        assert_eq!(pt.dist.raw, 0);
    }
    assert_eq!(m.point(0).fid1.packed, 0x40000003);
    assert_eq!(m.point(0).fid2.packed, 0xc0000034);
    assert_eq!(m.point(1).fid1.packed, 0xc000003e);
    assert_eq!(m.point(1).fid2.packed, 0x40000000);
}
