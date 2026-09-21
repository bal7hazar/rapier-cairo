//! Half-space contact-manifold golden checks.
//!
//! The committed upstream set contains halfspace-cuboid cases. Halfspace-segment/capsule are
//! covered analytically in the generator module because this fixture family has no such pairs.

use fixed::Fixed;
use glam::Vec2;
use rapier_geometry2d::contact::{ContactManifold, ContactManifoldTrait, TrackedContact};
use rapier_geometry2d::contact_generators::halfspace_pfm::contact_manifold_halfspace_pfm_shapes;
use rapier_geometry2d::feature_id::FeatureId;
use rapier_geometry2d::shape::{Ball, Capsule, Cuboid, HalfSpace, Segment, Shape};
use rapier_golden::compare::{vec2_within, within};
use rapier_golden::contact_manifolds;
use rapier_golden::types::{ContactPointRaw, ManifoldCase, PoseRaw, ShapeRaw, Vec2Raw};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;

const POINT_TOLERANCE: u64 = 64;
const DIST_TOLERANCE: u64 = 64;
const NORMAL_TOLERANCE: u64 = 16;

fn vector(v: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: v.x }, y: Fixed { raw: v.y } }
}

fn raw(v: Vec2) -> Vec2Raw {
    Vec2Raw { x: v.x.raw, y: v.y.raw }
}

fn pose(p: PoseRaw) -> Pose2 {
    Pose2Trait::new(
        vector(p.translation),
        Rot2 { re: Fixed { raw: p.rotation.re }, im: Fixed { raw: p.rotation.im } },
    )
}

fn shape(s: ShapeRaw) -> Shape {
    match s {
        ShapeRaw::Ball(r) => Shape::Ball(Ball { radius: Fixed { raw: r } }),
        ShapeRaw::Cuboid(h) => Shape::Cuboid(Cuboid { half_extents: vector(h) }),
        ShapeRaw::Capsule(c) => Shape::Capsule(
            Capsule {
                segment: Segment { a: vector(c.a), b: vector(c.b) },
                radius: Fixed { raw: c.radius },
            },
        ),
        ShapeRaw::HalfSpace(n) => Shape::HalfSpace(HalfSpace { normal: vector(n) }),
        ShapeRaw::Segment(s) => Shape::Segment(Segment { a: vector(s.a), b: vector(s.b) }),
    }
}

fn is_halfspace_case(case: ManifoldCase) -> bool {
    match (case.shape1, case.shape2) {
        (ShapeRaw::HalfSpace(_), ShapeRaw::Cuboid(_)) => true,
        _ => false,
    }
}

fn check_point(actual: TrackedContact, expected: ContactPointRaw, id: felt252, i: u8) {
    assert!(
        vec2_within(raw(actual.local_p1), expected.local_p1, POINT_TOLERANCE), "{} p1 {}", id, i,
    );
    assert!(
        vec2_within(raw(actual.local_p2), expected.local_p2, POINT_TOLERANCE), "{} p2 {}", id, i,
    );
    assert!(within(actual.dist.raw, expected.dist, DIST_TOLERANCE), "{} dist {}", id, i);
    assert_eq!(actual.fid1, FeatureId { packed: expected.fid1 }, "{} fid1 {}", id, i);
    assert_eq!(actual.fid2, FeatureId { packed: expected.fid2 }, "{} fid2 {}", id, i);
}

fn check_case(case: ManifoldCase) {
    let mut manifold: ContactManifold = Default::default();
    assert!(
        contact_manifold_halfspace_pfm_shapes(
            pose(case.pos12),
            shape(case.shape1),
            shape(case.shape2),
            Fixed { raw: contact_manifolds::PREDICTION },
            ref manifold,
        ),
        "{} supported",
        case.id,
    );
    assert_eq!(manifold.num_points, case.num_points.try_into().unwrap(), "{}", case.id);
    assert!(
        vec2_within(raw(manifold.local_n1), case.local_n1, NORMAL_TOLERANCE),
        "{} local_n1",
        case.id,
    );
    assert!(
        vec2_within(raw(manifold.local_n2), case.local_n2, NORMAL_TOLERANCE),
        "{} local_n2",
        case.id,
    );
    let points = case.points.span();
    let mut i = 0_u8;
    while i != manifold.num_points {
        check_point(manifold.point(i), *points.at(i.into()), case.id, i);
        i += 1;
    }
}

#[test]
fn test_halfspace_cuboid_golden_cases() {
    let mut count = 0;
    for case in contact_manifolds::cases() {
        if is_halfspace_case(*case) {
            check_case(*case);
            count += 1;
        }
    }
    assert_eq!(count, 6);
}

#[test]
fn gas_baseline() {
    let _ = opaque(contact_manifolds::HALFSPACE_CUBOID_TOUCHING);
}

#[test]
fn gas_halfspace_cuboid_golden() {
    check_case(opaque(contact_manifolds::HALFSPACE_CUBOID_TOUCHING));
}
