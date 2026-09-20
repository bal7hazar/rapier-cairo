//! All 15 in-scope cases in both directions. Seven triangle cases are explicitly
//! Counted as deferred: implementing their three edge axes is outside GD's scope.
use fixed::{Fixed, ONE, ZERO};
use glam::Vec2;
use rapier_geometry2d::sat::{
    cuboid_cuboid_find_local_separating_normal_oneway,
    cuboid_segment_find_local_separating_normal_oneway,
    segment_cuboid_find_local_separating_normal_oneway,
};
use rapier_geometry2d::shape::{Cuboid, Segment};
use rapier_golden::compare::within;
use rapier_golden::sat2d;
use rapier_golden::types::{PoseRaw, SatAxisRaw, SatOperandRaw, Vec2Raw};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2Trait;
use rapier_testing::opaque;
fn v(p: Vec2Raw) -> Vec2 {
    Vec2 { x: Fixed { raw: p.x }, y: Fixed { raw: p.y } }
}
fn pose(p: PoseRaw) -> Pose2 {
    Pose2 {
        translation: v(p.translation),
        rotation: Rot2Trait::from_cos_sin(
            Fixed { raw: p.rotation.re }, Fixed { raw: p.rotation.im },
        ),
    }
}
fn check(actual: (Fixed, Vec2), expected: SatAxisRaw, p: Pose2, id: felt252) {
    let (s, n) = actual;
    let face = n.x == ZERO || n.y == ZERO;
    let aligned = (p.rotation.re == ZERO || p.rotation.im == ZERO) && face;
    let tol = if aligned {
        1
    } else {
        4
    };
    assert!(
        within(s.raw, expected.separation, tol), "{} sep {} vs {}", id, s.raw, expected.separation,
    );
    let ntol = if face {
        4
    } else {
        8
    };
    assert!(within(n.x.raw, expected.axis.x, ntol), "{} nx {} vs {}", id, n.x.raw, expected.axis.x);
    assert!(within(n.y.raw, expected.axis.y, ntol), "{} ny {} vs {}", id, n.y.raw, expected.axis.y);
}
#[test]
fn test_all_in_scope_sat_vectors_both_directions() {
    let mut tested = 0;
    let mut deferred = 0;
    for c in sat2d::cases() {
        let c1 = match *c.shape1 {
            SatOperandRaw::Cuboid(h) => Cuboid { half_extents: v(h) },
            _ => panic!("expected cuboid"),
        };
        let p = pose(*c.pos12);
        let q = pose(*c.pos21);
        match *c.shape2 {
            SatOperandRaw::Cuboid(h) => {
                let c2 = Cuboid { half_extents: v(h) };
                check(
                    cuboid_cuboid_find_local_separating_normal_oneway(c1, c2, p), *c.sep1, p, *c.id,
                );
                check(
                    cuboid_cuboid_find_local_separating_normal_oneway(c2, c1, q), *c.sep2, q, *c.id,
                );
                tested += 1;
            },
            SatOperandRaw::Segment(s) => {
                let s = Segment { a: v(s.a), b: v(s.b) };
                check(
                    cuboid_segment_find_local_separating_normal_oneway(c1, s, p), *c.sep1, p, *c.id,
                );
                check(
                    segment_cuboid_find_local_separating_normal_oneway(s, c1, q), *c.sep2, q, *c.id,
                );
                tested += 1;
            },
            SatOperandRaw::Triangle(_) => { deferred += 1; },
        }
    }
    assert_eq!(tested, 15);
    assert_eq!(deferred, 7);
}
#[test]
fn gas_baseline() {
    let _ = opaque(sat2d::CUBOID_CUBOID_SHALLOW.pos12);
}
#[test]
fn gas_golden_sat() {
    let p = pose(opaque(sat2d::CUBOID_CUBOID_SHALLOW.pos12));
    let c = Cuboid { half_extents: Vec2 { x: ONE, y: ONE } };
    let _ = cuboid_cuboid_find_local_separating_normal_oneway(c, c, p);
}
