//! Tests and `gas_*` probes of `crate::dispatch`.

use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::contact::{ContactData, ContactManifold, ContactManifoldTrait};
use crate::contact_generators::ball_ball::contact_manifold_ball_ball;
use crate::contact_generators::capsule_capsule::contact_manifold_capsule_capsule;
use crate::contact_generators::convex_ball::{
    contact_manifold_ball_convex, contact_manifold_convex_ball,
};
use crate::contact_generators::cuboid_capsule::{
    contact_manifold_cuboid_capsule, contact_manifold_cuboid_capsule_shapes,
};
use crate::contact_generators::cuboid_cuboid::contact_manifold_cuboid_cuboid;
use crate::contact_generators::cuboid_segment::{
    contact_manifold_cuboid_segment, contact_manifold_cuboid_segment_shapes,
};
use crate::contact_generators::halfspace_pfm::contact_manifold_halfspace_pfm;
use crate::shape::{Ball, Capsule, Cuboid, HalfSpace, Segment, Shape};
use super::alternatives::{
    contact_manifold_helpers, contact_manifold_outlined, contact_manifold_plain,
    contact_manifold_plain_outlined, contact_manifold_shapes_chain,
};
use super::{contact_manifold, contact_manifold_step};

const PREDICTION: Fixed = Fixed { raw: 85899346 };
/// 30 degrees.
const R_30: Rot2 = Rot2 { re: Fixed { raw: 3719550787 }, im: HALF };
const QUARTER: Fixed = Fixed { raw: 1073741824 };

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn ball_s() -> Ball {
    Ball { radius: HALF }
}
fn cuboid_s() -> Cuboid {
    Cuboid { half_extents: v(ONE, HALF) }
}
fn capsule_s() -> Capsule {
    Capsule { segment: segment_s(), radius: QUARTER }
}
fn segment_s() -> Segment {
    Segment { a: v(-HALF, ZERO), b: v(HALF, ZERO) }
}
fn halfspace_s() -> HalfSpace {
    HalfSpace { normal: v(ZERO, ONE) }
}

/// Shape 2 slightly below-right of shape 1 (half-space normal +Y): every pair overlaps.
fn pose_direct() -> Pose2 {
    Pose2 { translation: v(HALF, FixedTrait::from_ratio(1, 8)), rotation: R_30 }
}
/// The same scene, seen from the other side (used when shape 1 is the half-space's partner).
fn pose_reversed() -> Pose2 {
    Pose2 { translation: v(HALF, FixedTrait::from_ratio(-1, 8)), rotation: R_30 }
}

/// Every ordered pair of the closed enum: `(shape1, shape2, supported)`.
fn all_pairs() -> Array<(Shape, Shape, bool)> {
    let shapes = array![
        Shape::Ball(ball_s()), Shape::Cuboid(cuboid_s()), Shape::Capsule(capsule_s()),
        Shape::Segment(segment_s()), Shape::HalfSpace(halfspace_s()),
    ];
    let mut pairs = array![];
    for s1 in shapes.span() {
        for s2 in shapes.span() {
            let unsupported = match (*s1, *s2) {
                (Shape::Segment(_), Shape::Segment(_)) => true,
                (Shape::Segment(_), Shape::Capsule(_)) => true,
                (Shape::Capsule(_), Shape::Segment(_)) => true,
                (Shape::HalfSpace(_), Shape::HalfSpace(_)) => true,
                _ => false,
            };
            pairs.append((*s1, *s2, !unsupported));
        }
    }
    pairs
}

/// Sets the solver impulse of both point slots.
fn with_impulse(m: ContactManifold, raw: i64) -> ContactManifold {
    let mut out = m;
    let [mut p0, mut p1] = m.points;
    let data = ContactData { impulse: FixedTrait::from_raw(raw), ..Default::default() };
    p0.data = data;
    p1.data = data;
    out.points = [p0, p1];
    out
}

/// Dispatcher vs chain on one pair and one pose, on a fresh then a warm manifold; `true`
/// when the pair produced contacts.
#[inline(never)]
fn check_pair(pose: Pose2, shape1: Shape, shape2: Shape, supported: bool) -> bool {
    let mut a: ContactManifold = Default::default();
    let mut b: ContactManifold = Default::default();
    assert_eq!(contact_manifold(pose, shape1, shape2, PREDICTION, ref a), supported);
    assert_eq!(contact_manifold_shapes_chain(pose, shape1, shape2, PREDICTION, ref b), supported);
    assert_eq!(a, b);
    if a.num_points == 0 {
        return false;
    }
    // Second pass over the warm manifold, with a solver impulse to carry over.
    a = with_impulse(a, 7);
    b = with_impulse(b, 7);
    contact_manifold(pose, shape1, shape2, PREDICTION, ref a);
    contact_manifold_shapes_chain(pose, shape1, shape2, PREDICTION, ref b);
    assert_eq!(a, b);
    assert_eq!(a.point(0).data.impulse.raw, 7);
    true
}

fn same_metered_plain(pose: Pose2, shape1: Shape, shape2: Shape, supported: bool) -> bool {
    let mut metered: ContactManifold = Default::default();
    let mut plain: ContactManifold = Default::default();
    assert_eq!(contact_manifold(pose, shape1, shape2, PREDICTION, ref metered), supported);
    assert_eq!(contact_manifold_plain(pose, shape1, shape2, PREDICTION, ref plain), supported);
    assert_eq!(metered, plain);
    metered = with_impulse(metered, 11);
    plain = with_impulse(plain, 11);
    assert_eq!(contact_manifold(pose, shape1, shape2, PREDICTION, ref metered), supported);
    assert_eq!(contact_manifold_plain(pose, shape1, shape2, PREDICTION, ref plain), supported);
    metered == plain
}

/// The dispatcher gives, for the 25 ordered pairs, the same result and the same manifold as
/// upstream's chain of `*_shapes` wrappers, on a fresh manifold and on a warm one; every
/// supported pair produces contacts under one of the two poses.
#[test]
fn test_dispatch_matches_shapes_chain_on_every_pair() {
    let mut supported = 0_u32;
    for (s1, s2, ok) in all_pairs().span() {
        let hit = check_pair(pose_direct(), *s1, *s2, *ok)
            | check_pair(pose_reversed(), *s1, *s2, *ok);
        assert_eq!(hit, *ok);
        if *ok {
            supported += 1;
        }
    }
    assert_eq!(supported, 21);
}

/// Metering is a gas-only wrapper around GG's plain typed match: same support result and
/// identical manifold for the 25 ordered pairs, fresh and warm.
#[test]
fn test_metered_equals_plain() {
    for (s1, s2, ok) in all_pairs().span() {
        assert!(same_metered_plain(pose_direct(), *s1, *s2, *ok));
    }
}

/// Ball–ball goes to the ball–ball generator, not to the convex–ball one (which accepts
/// it too), and the flipped ball-first pair mirrors the ball-second one.
#[test]
fn test_ball_ball_is_matched_first() {
    let p = pose_direct();
    let mut m: ContactManifold = Default::default();
    let mut e: ContactManifold = Default::default();
    assert!(contact_manifold(p, Shape::Ball(ball_s()), Shape::Ball(ball_s()), PREDICTION, ref m));
    contact_manifold_ball_ball(p, ball_s(), ball_s(), PREDICTION, ref e);
    assert_eq!(m, e);
    assert_eq!(m.num_points, 1);
}

/// Unsupported pairs return `false` and clear a manifold that held points.
#[test]
fn test_unsupported_pairs_clear_the_manifold() {
    let mut live: ContactManifold = Default::default();
    contact_manifold(
        pose_direct(), Shape::Ball(ball_s()), Shape::Ball(ball_s()), PREDICTION, ref live,
    );
    assert_eq!(live.num_points, 1);
    let unsupported = array![
        (Shape::Segment(segment_s()), Shape::Segment(segment_s())),
        (Shape::Segment(segment_s()), Shape::Capsule(capsule_s())),
        (Shape::Capsule(capsule_s()), Shape::Segment(segment_s())),
        (Shape::HalfSpace(halfspace_s()), Shape::HalfSpace(halfspace_s())),
    ];
    for (s1, s2) in unsupported.span() {
        let mut m = live;
        assert!(!contact_manifold(pose_direct(), *s1, *s2, PREDICTION, ref m));
        assert_eq!(m.num_points, 0);
    }
}

/// `pos12` moved by `(dx, dy)` raw.
fn moved(pos12: Pose2, dx: i64, dy: i64) -> Pose2 {
    Pose2 {
        translation: pos12.translation + v(Fixed { raw: dx }, Fixed { raw: dy }),
        rotation: pos12.rotation,
    }
}

/// `contact_manifold_step` and `contact_manifold` from the same manifolds: same result, same
/// manifold.
fn same_step(
    pos12: Pose2, shape1: Shape, shape2: Shape, ref a: ContactManifold, ref b: ContactManifold,
) -> bool {
    let got = contact_manifold_step(pos12, shape1, shape2, PREDICTION, ref a);
    let expected = contact_manifold(pos12, shape1, shape2, PREDICTION, ref b);
    got == expected && a == b
}

/// Every ordered pair at a separated, a resting, a shallow-rotated and a deep pose, three calls
/// on the same manifold: cold, warm and barely moved (persistence fast path taken), and moved
/// beyond the persistence tolerance (fast path fails, the generator runs): the step's variant is
/// bit-identical to the metered dispatcher on every call.
#[test]
fn test_step_matches_metered_cold_warm_and_moved() {
    let level = Rot2 { re: ONE, im: ZERO };
    let poses = array![
        Pose2 { translation: v(ZERO, FixedTrait::from_int(3)), rotation: level },
        Pose2 { translation: v(ZERO, ONE), rotation: level }, pose_direct(), pose_reversed(),
    ];
    for pos12 in poses.span() {
        for (s1, s2, _) in all_pairs().span() {
            let mut a: ContactManifold = Default::default();
            let mut b: ContactManifold = Default::default();
            assert!(same_step(*pos12, *s1, *s2, ref a, ref b));
            assert!(same_step(moved(*pos12, 4294, 0), *s1, *s2, ref a, ref b));
            assert!(same_step(moved(*pos12, 42949672, -4294967), *s1, *s2, ref a, ref b));
        }
    }
}

// Opaque inputs of the probes.
fn pd() -> Pose2 {
    opaque(pose_direct())
}
fn pr() -> Pose2 {
    opaque(pose_reversed())
}
fn ball() -> Shape {
    opaque(Shape::Ball(ball_s()))
}
fn cuboid() -> Shape {
    opaque(Shape::Cuboid(cuboid_s()))
}
fn capsule() -> Shape {
    opaque(Shape::Capsule(capsule_s()))
}
fn segment() -> Shape {
    opaque(Shape::Segment(segment_s()))
}
fn halfspace() -> Shape {
    opaque(Shape::HalfSpace(halfspace_s()))
}

#[test]
fn gas_baseline() {
    let _ = pd();
}

#[test]
fn gas_dispatch_ball_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), ball(), ball(), PREDICTION, ref m);
}

#[test]
fn gas_generator_ball_ball() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_ball_ball(pd(), opaque(ball_s()), opaque(ball_s()), PREDICTION, ref m);
}

#[test]
fn gas_outlined_ball_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_outlined(pd(), ball(), ball(), PREDICTION, ref m);
}

#[test]
fn gas_plain_ball_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_plain(pd(), ball(), ball(), PREDICTION, ref m);
}

#[test]
fn gas_plain_outlined_ball_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_plain_outlined(pd(), ball(), ball(), PREDICTION, ref m);
}

#[test]
fn gas_helpers_ball_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_helpers(pd(), ball(), ball(), PREDICTION, ref m);
}

#[test]
fn gas_chain_ball_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_shapes_chain(pd(), ball(), ball(), PREDICTION, ref m);
}

#[test]
fn gas_dispatch_cuboid_cuboid() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), cuboid(), cuboid(), PREDICTION, ref m);
}

#[test]
fn gas_generator_cuboid_cuboid() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_cuboid_cuboid(pd(), opaque(cuboid_s()), opaque(cuboid_s()), PREDICTION, ref m);
}

#[test]
fn gas_dispatch_capsule_capsule() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), capsule(), capsule(), PREDICTION, ref m);
}

#[test]
fn gas_generator_capsule_capsule() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_capsule_capsule(
        pd(), opaque(capsule_s()), opaque(capsule_s()), PREDICTION, ref m,
    );
}

#[test]
fn gas_dispatch_ball_cuboid() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), ball(), cuboid(), PREDICTION, ref m);
}

#[test]
fn gas_generator_ball_cuboid() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_ball_convex(pd(), opaque(ball_s()), cuboid(), PREDICTION, ref m);
}

#[test]
fn gas_dispatch_cuboid_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), cuboid(), ball(), PREDICTION, ref m);
}

#[test]
fn gas_generator_cuboid_ball() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_convex_ball(pd(), cuboid(), opaque(ball_s()), PREDICTION, ref m);
}

#[test]
fn gas_dispatch_cuboid_capsule() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), cuboid(), capsule(), PREDICTION, ref m);
}

#[test]
fn gas_generator_cuboid_capsule() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_cuboid_capsule(
        pd(), opaque(cuboid_s()), opaque(capsule_s()), PREDICTION, ref m,
    );
}

#[test]
fn gas_dispatch_capsule_cuboid() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), capsule(), cuboid(), PREDICTION, ref m);
}

#[test]
fn gas_generator_capsule_cuboid() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_cuboid_capsule_shapes(pd(), capsule(), cuboid(), PREDICTION, ref m);
}

#[test]
fn gas_dispatch_cuboid_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), cuboid(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_generator_cuboid_segment() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_cuboid_segment(
        pd(), opaque(cuboid_s()), opaque(segment_s()), PREDICTION, ref m,
    );
}

#[test]
fn gas_outlined_cuboid_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_outlined(pd(), cuboid(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_plain_cuboid_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_plain(pd(), cuboid(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_plain_outlined_cuboid_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_plain_outlined(pd(), cuboid(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_helpers_cuboid_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_helpers(pd(), cuboid(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_chain_cuboid_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_shapes_chain(pd(), cuboid(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_dispatch_segment_cuboid() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), segment(), cuboid(), PREDICTION, ref m);
}

#[test]
fn gas_generator_segment_cuboid() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_cuboid_segment_shapes(pd(), segment(), cuboid(), PREDICTION, ref m);
}

#[test]
fn gas_dispatch_halfspace_cuboid() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), halfspace(), cuboid(), PREDICTION, ref m);
}

#[test]
fn gas_generator_halfspace_cuboid() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_halfspace_pfm(pd(), opaque(halfspace_s()), cuboid(), PREDICTION, ref m, false);
}

#[test]
fn gas_dispatch_halfspace_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), halfspace(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_generator_halfspace_segment() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_halfspace_pfm(
        pd(), opaque(halfspace_s()), segment(), PREDICTION, ref m, false,
    );
}

#[test]
fn gas_dispatch_halfspace_capsule() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), halfspace(), capsule(), PREDICTION, ref m);
}

#[test]
fn gas_generator_halfspace_capsule() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_halfspace_pfm(
        pd(), opaque(halfspace_s()), capsule(), PREDICTION, ref m, false,
    );
}

#[test]
fn gas_dispatch_cuboid_halfspace() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pr(), cuboid(), halfspace(), PREDICTION, ref m);
}

#[test]
fn gas_generator_cuboid_halfspace() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_halfspace_pfm(
        pr().inverse(), opaque(halfspace_s()), cuboid(), PREDICTION, ref m, true,
    );
}

#[test]
fn gas_dispatch_segment_halfspace() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pr(), segment(), halfspace(), PREDICTION, ref m);
}

#[test]
fn gas_generator_segment_halfspace() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_halfspace_pfm(
        pr().inverse(), opaque(halfspace_s()), segment(), PREDICTION, ref m, true,
    );
}

#[test]
fn gas_dispatch_capsule_halfspace() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pr(), capsule(), halfspace(), PREDICTION, ref m);
}

#[test]
fn gas_generator_capsule_halfspace() {
    let mut m: ContactManifold = Default::default();
    contact_manifold_halfspace_pfm(
        pr().inverse(), opaque(halfspace_s()), capsule(), PREDICTION, ref m, true,
    );
}

#[test]
fn gas_outlined_capsule_halfspace() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_outlined(pr(), capsule(), halfspace(), PREDICTION, ref m);
}

#[test]
fn gas_plain_capsule_halfspace() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_plain(pr(), capsule(), halfspace(), PREDICTION, ref m);
}

#[test]
fn gas_plain_outlined_capsule_halfspace() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_plain_outlined(pr(), capsule(), halfspace(), PREDICTION, ref m);
}

#[test]
fn gas_helpers_capsule_halfspace() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_helpers(pr(), capsule(), halfspace(), PREDICTION, ref m);
}

#[test]
fn gas_chain_capsule_halfspace() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_shapes_chain(pr(), capsule(), halfspace(), PREDICTION, ref m);
}

#[test]
fn gas_dispatch_segment_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), segment(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_outlined_segment_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_outlined(pd(), segment(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_plain_segment_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_plain(pd(), segment(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_plain_outlined_segment_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_plain_outlined(pd(), segment(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_helpers_segment_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_helpers(pd(), segment(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_chain_segment_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_shapes_chain(pd(), segment(), segment(), PREDICTION, ref m);
}

/// The step's variant behind a call: a loop-free outlined caller pays its costliest arm.
#[inline(never)]
fn step_outlined(pos12: Pose2, shape1: Shape, shape2: Shape, ref m: ContactManifold) -> bool {
    contact_manifold_step(pos12, shape1, shape2, PREDICTION, ref m)
}

/// A cuboid–cuboid manifold computed at `pd()`: the warm probes start from it.
fn warm_cuboid_cuboid() -> ContactManifold {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold(pd(), cuboid(), cuboid(), PREDICTION, ref m);
    m
}

#[test]
fn gas_step_ball_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_step(pd(), ball(), ball(), PREDICTION, ref m);
}

#[test]
fn gas_step_outlined_ball_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = step_outlined(pd(), ball(), ball(), ref m);
}

#[test]
fn gas_step_cuboid_cuboid() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_step(pd(), cuboid(), cuboid(), PREDICTION, ref m);
}

/// Warm probes: the setup (`warm_cuboid_cuboid`) is common, compare them with each other.
#[test]
fn gas_dispatch_cuboid_cuboid_warm() {
    let mut m = warm_cuboid_cuboid();
    let _ = contact_manifold(pd(), cuboid(), cuboid(), PREDICTION, ref m);
}

#[test]
fn gas_step_cuboid_cuboid_warm() {
    let mut m = warm_cuboid_cuboid();
    let _ = contact_manifold_step(pd(), cuboid(), cuboid(), PREDICTION, ref m);
}

#[test]
fn gas_step_cuboid_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_step(pd(), cuboid(), segment(), PREDICTION, ref m);
}

#[test]
fn gas_step_segment_segment() {
    let mut m: ContactManifold = Default::default();
    let _ = contact_manifold_step(pd(), segment(), segment(), PREDICTION, ref m);
}


#[test]
fn gas_step_fallback_ball_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = super::alternatives::contact_manifold_step_fallback(
        pd(), ball(), ball(), PREDICTION, ref m,
    );
}
#[test]
fn gas_step_fallback_cuboid_cuboid_warm() {
    let mut m = warm_cuboid_cuboid();
    let _ = super::alternatives::contact_manifold_step_fallback(
        pd(), cuboid(), cuboid(), PREDICTION, ref m,
    );
}

#[test]
fn test_polygon_dispatch_candidate_equivalence() {
    let polygon = Shape::ConvexPolygon(
        BoxTrait::new(crate::contact_generators::polygon_polygon::cuboid_core(cuboid_s())),
    );
    for other in array![polygon, cuboid(), segment(), capsule()].span() {
        for (s1, s2) in array![(polygon, *other), (*other, polygon)].span() {
            let mut a: ContactManifold = Default::default();
            let mut b: ContactManifold = Default::default();
            let mut c: ContactManifold = Default::default();
            for p in array![pose_direct(), pose_direct(), pose_reversed()].span() {
                assert!(contact_manifold_step(*p, *s1, *s2, PREDICTION, ref a));
                assert!(
                    super::alternatives::contact_manifold_step_fallback(
                        *p, *s1, *s2, PREDICTION, ref b,
                    ),
                );
                assert!(
                    super::alternatives::boxed::contact_manifold_step_boxed(
                        *p, *s1, *s2, PREDICTION, ref c,
                    ),
                );
                assert_eq!(a, b);
                assert_eq!(a, c);
            }
        }
    }
}

#[test]
fn gas_step_boxed_ball_ball() {
    let mut m: ContactManifold = Default::default();
    let _ = super::alternatives::boxed::contact_manifold_step_boxed(
        pd(), ball(), ball(), PREDICTION, ref m,
    );
}
