//! `dispatch::intersection_test` against upstream's `DefaultQueryDispatcher::intersection_test`
//! (`rapier_golden::intersection_tests`, 81 cases: every supported pair over four regimes, plus
//! the unsupported half-space pair), in the recorded order and with the shapes swapped
//! (`pos12.inverse()`). Answers are booleans: compared exactly. The `gjk_touching` cases (upstream
//! answers them with GJK, within its tolerance) must answer `true` here, touching included; every
//! one of them is `true` upstream too.
use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::dispatch::intersection_test;
use rapier_geometry2d::shape::{
    Ball, Capsule, ConvexPolygonTrait, Cuboid, HalfSpace, Segment, Shape,
};
use rapier_golden::generated::intersection_tests;
use rapier_golden::types::{PolygonContactShapeRaw, PoseRaw, ShapeRaw, Vec2Raw};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;

fn v(p: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: p.x }, y: Fixed { raw: p.y } }
}

fn pose(p: PoseRaw) -> Pose2 {
    Pose2 {
        translation: v(p.translation),
        rotation: Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    }
}

fn shape(s: PolygonContactShapeRaw) -> Shape {
    match s {
        PolygonContactShapeRaw::Polygon(p) => {
            let mut vertices = array![];
            let mut i = 0;
            while i != p.count {
                vertices.append(v(*p.vertices.span().at(i.into())));
                i += 1;
            }
            Shape::ConvexPolygon(
                BoxTrait::new(ConvexPolygonTrait::from_convex_polyline(vertices.span()).unwrap()),
            )
        },
        PolygonContactShapeRaw::Other(s) => match s {
            ShapeRaw::Ball(r) => Shape::Ball(Ball { radius: Fixed { raw: r } }),
            ShapeRaw::Cuboid(h) => Shape::Cuboid(Cuboid { half_extents: v(h) }),
            ShapeRaw::Segment(s) => Shape::Segment(Segment { a: v(s.a), b: v(s.b) }),
            ShapeRaw::Capsule(c) => Shape::Capsule(
                Capsule {
                    segment: Segment { a: v(c.a), b: v(c.b) }, radius: Fixed { raw: c.radius },
                },
            ),
            ShapeRaw::HalfSpace(n) => Shape::HalfSpace(HalfSpace { normal: v(n) }),
        },
    }
}

#[test]
fn test_every_case_both_orders() {
    let mut touching = 0;
    let mut unsupported = 0;
    for case in intersection_tests::cases() {
        let (s1, s2, pos12) = (shape(*case.shape1), shape(*case.shape2), pose(*case.pos12));
        let expected = if *case.supported {
            Some(*case.intersecting)
        } else {
            unsupported += 1;
            None
        };
        if *case.gjk_touching {
            // Exact kernels: touching is intersecting (upstream agrees on every recorded case).
            assert!(*case.intersecting, "{}: upstream GJK answer changed", *case.id);
            touching += 1;
        }
        assert!(intersection_test(pos12, s1, s2) == expected, "{}", *case.id);
        assert!(intersection_test(pos12.inverse(), s2, s1) == expected, "{} swapped", *case.id);
    }
    assert_eq!(intersection_tests::cases().len(), 81);
    assert_eq!((touching, unsupported), (9, 1));
}
