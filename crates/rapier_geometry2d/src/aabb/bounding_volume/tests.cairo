use fixed::{Fixed, FixedTrait, HALF, MAX, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::aabb::Aabb;
use crate::shape::{BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, SegmentTrait};
use super::{
    BoundingSphere, BoundingSphereTrait, BoundingVolume, alternatives, ball_aabb, local_ball_aabb,
    local_point_cloud_aabb, local_support_map_aabb, point_cloud_aabb, point_cloud_bounding_sphere,
    point_cloud_bounding_sphere_with_center,
};

fn i(n: i32) -> Fixed {
    FixedTrait::from_int(n)
}

fn v(x: i32, y: i32) -> Vec2 {
    Vec2 { x: i(x), y: i(y) }
}

fn s(x: i32, y: i32, r: Fixed) -> BoundingSphere {
    BoundingSphereTrait::new(v(x, y), r)
}

fn aabb(x0: i32, y0: i32, x1: i32, y1: i32) -> Aabb {
    Aabb { mins: v(x0, y0), maxs: v(x1, y1) }
}

/// Quarter turn, then `(10, 0)`: literals, no runtime normalisation inside a probe.
const POSE: Pose2 = Pose2 {
    translation: Vec2 { x: Fixed { raw: 42949672960 }, y: Fixed { raw: 0 } },
    rotation: Rot2 { re: Fixed { raw: 0 }, im: Fixed { raw: 4294967296 } },
};

fn close(a: Fixed, b: Fixed, ulps: i64) -> bool {
    a.abs_diff_eq(b, Fixed { raw: ulps })
}

#[test]
fn test_sphere_accessors_and_moves() {
    let a = s(3, 0, ONE);
    assert_eq!((BoundingSphereTrait::center(a), a.radius()), (v(3, 0), ONE));
    assert_eq!(BoundingVolume::center(a), v(3, 0));
    assert_eq!(a.translated(v(1, -1)), s(4, -1, ONE));
    // (3, 0) turned a quarter is (0, 3), then + (10, 0).
    assert_eq!(a.transform_by(POSE), s(10, 3, ONE));
}

#[test]
fn test_sphere_intersects_contains_table() {
    let big = s(0, 0, TWO);
    // (other, intersects, big contains other): touching counts on both tests.
    let cases: Span<(BoundingSphere, bool, bool)> = array![
        (s(3, 0, ONE), true, false), (s(5, 0, ONE), false, false), (s(1, 0, ONE), true, true),
        (BoundingSphere { center: Vec2 { x: HALF, y: ZERO }, radius: ONE }, true, true),
        (s(0, 0, TWO), true, true), (s(0, 0, i(3)), true, false),
        (BoundingSphere { center: v(3, 0), radius: ONE - Fixed { raw: 1 } }, false, false),
        // Unbounded radii never overflow.
        (BoundingSphere { center: v(1000, -1000), radius: MAX }, true, false),
    ]
        .span();
    for (other, hit, inside) in cases {
        assert_eq!(BoundingVolume::intersects(big, *other), *hit);
        assert_eq!(BoundingVolume::intersects(*other, big), *hit);
        assert_eq!(BoundingVolume::contains(big, *other), *inside);
    }
    let unbounded = BoundingSphere { center: v(0, 0), radius: MAX };
    assert!(BoundingVolume::contains(unbounded, big));
}

#[test]
fn test_sphere_merge() {
    let a = s(0, 0, TWO);
    // Disjoint enough: R = (3 + 2 + 1) / 2 = 3, centre (1, 0) (t = 1/3 floors 1 ulp low).
    let m = BoundingVolume::merged(a, s(3, 0, ONE));
    assert_eq!(m.radius, i(3));
    assert!(close(m.center.x, ONE, 2) && m.center.y == ZERO);
    let alt = alternatives::merged_extremes(a, s(3, 0, ONE));
    assert!(close(alt.radius, i(3), 4) && close(alt.center.x, ONE, 4));
    // Containment either way returns the container unchanged; so does a coincident pair.
    assert_eq!(BoundingVolume::merged(a, s(1, 0, ONE)), a);
    assert_eq!(BoundingVolume::merged(s(1, 0, ONE), a), a);
    assert_eq!(BoundingVolume::merged(s(0, 0, ONE), a), a);
    let mut grown = s(1, 0, ONE);
    BoundingVolume::merge(ref grown, s(-5, 0, ONE));
    assert_eq!(grown.radius, i(4));
    assert!(close(grown.center.x, i(-2), 2));
}

#[test]
fn test_margins() {
    let a = s(0, 0, TWO);
    assert_eq!(BoundingVolume::loosened(a, ONE).radius, i(3));
    assert_eq!(BoundingVolume::tightened(a, TWO).radius, ZERO);
    let mut b = a;
    BoundingVolume::loosen(ref b, HALF);
    BoundingVolume::tighten(ref b, ONE);
    assert_eq!(b.radius, ONE + HALF);
    let box_ = aabb(-1, -2, 3, 2);
    assert_eq!(BoundingVolume::loosened(box_, ONE), aabb(-2, -3, 4, 3));
    assert_eq!(BoundingVolume::tightened(box_, ONE), aabb(0, -1, 2, 1));
    // `tightened` may invert an `Aabb` (as upstream); `tighten` may not.
    assert_eq!(BoundingVolume::tightened(box_, i(3)), aabb(2, 1, 0, -1));
    let mut c = box_;
    BoundingVolume::tighten(ref c, TWO);
    assert_eq!(c, aabb(1, 0, 1, 0));
    BoundingVolume::loosen(ref c, ONE);
    BoundingVolume::merge(ref c, aabb(5, 5, 6, 6));
    assert_eq!(c, aabb(0, -1, 6, 6));
    assert!(BoundingVolume::intersects(c, box_) && !BoundingVolume::contains(box_, c));
    assert_eq!(BoundingVolume::center(box_), v(1, 0));
}

#[test]
#[should_panic(expected: 'Bounding: negative margin')]
fn test_negative_sphere_margin_panics() {
    let _ = BoundingVolume::loosened(s(0, 0, ONE), -ONE);
}

#[test]
#[should_panic(expected: 'Bounding: margin too large')]
fn test_sphere_overtightening_panics() {
    let _ = BoundingVolume::tightened(s(0, 0, ONE), TWO);
}

#[test]
#[should_panic(expected: 'Bounding: negative margin')]
fn test_negative_aabb_margin_panics() {
    let _ = BoundingVolume::tightened(aabb(0, 0, 1, 1), -ONE);
}

#[test]
#[should_panic(expected: 'Bounding: margin too large')]
fn test_aabb_overtightening_panics() {
    let mut a = aabb(0, 0, 1, 1);
    BoundingVolume::tighten(ref a, ONE);
}

#[test]
fn test_ball_and_point_cloud_boxes() {
    assert_eq!(ball_aabb(v(1, 2), ONE), aabb(0, 1, 2, 3));
    assert_eq!(local_ball_aabb(ONE), aabb(-1, -1, 1, 1));
    let pts = array![v(1, 0), v(0, 2), v(-1, -1)].span();
    assert_eq!(local_point_cloud_aabb(pts), aabb(-1, -1, 1, 2));
    // Quarter turn: (1, 0) -> (0, 1), (0, 2) -> (-2, 0), (-1, -1) -> (1, -1); then + (10, 0).
    assert_eq!(point_cloud_aabb(POSE, pts), aabb(8, -1, 11, 1));
}

#[test]
fn test_support_map_boxes_match_the_shapes() {
    let ball = BallTrait::new(HALF);
    assert_eq!(local_support_map_aabb(ball), ball.compute_local_aabb());
    let cuboid = CuboidTrait::new(v(2, 1));
    assert_eq!(local_support_map_aabb(cuboid), cuboid.compute_local_aabb());
    let capsule = CapsuleTrait::new(v(-2, -1), v(4, 3), HALF);
    assert_eq!(local_support_map_aabb(capsule), capsule.compute_local_aabb());
    let segment = SegmentTrait::new(v(1, 2), v(3, 0));
    assert_eq!(local_support_map_aabb(segment), segment.compute_local_aabb());
    let triangle = ConvexPolygonTrait::from_convex_polyline(
        array![v(0, 0), v(4, 0), v(1, 3)].span(),
    )
        .unwrap();
    assert_eq!(local_support_map_aabb(triangle), triangle.compute_local_aabb());
}

#[test]
fn test_point_cloud_spheres() {
    let square = array![v(0, 0), v(2, 0), v(2, 2), v(0, 2)].span();
    let sphere = point_cloud_bounding_sphere(square);
    // sqrt(2), floored.
    assert_eq!(sphere, BoundingSphere { center: v(1, 1), radius: Fixed { raw: 6074000999 } });
    let polygon = ConvexPolygonTrait::from_convex_polyline(square).unwrap();
    assert_eq!(polygon.local_bounding_sphere(), sphere);
    assert_eq!(
        point_cloud_bounding_sphere_with_center(square, v(0, 0)).radius, Fixed { raw: 12148001999 },
    );
    assert_eq!(point_cloud_bounding_sphere_with_center(array![].span(), v(1, 1)).radius, ZERO);
    // The mean truncates toward zero: (-3 ulp + 0) / 2 = -1 ulp.
    let tiny = array![Vec2 { x: Fixed { raw: -3 }, y: ZERO }, Vec2 { x: ZERO, y: ZERO }].span();
    assert_eq!(point_cloud_bounding_sphere(tiny).center.x, Fixed { raw: -1 });
}

#[test]
#[should_panic(expected: 'Bounding: empty point cloud')]
fn test_empty_point_cloud_sphere_panics() {
    let _ = point_cloud_bounding_sphere(array![].span());
}

#[test]
#[fuzzer(runs: 128, seed: 20260926)]
fn fuzz_merged_contains_both(x0: i16, y0: i16, r0: u16, x1: i16, y1: i16, r1: u16) {
    // Raws scaled by 2^16: coordinates up to +-2^31 ulp, radii up to 2^32 ulp.
    let a = BoundingSphere {
        center: Vec2 {
            x: Fixed { raw: x0.into() * 0x10000 }, y: Fixed { raw: y0.into() * 0x10000 },
        },
        radius: Fixed { raw: r0.into() * 0x10000 },
    };
    let b = BoundingSphere {
        center: Vec2 {
            x: Fixed { raw: x1.into() * 0x10000 }, y: Fixed { raw: y1.into() * 0x10000 },
        },
        radius: Fixed { raw: r1.into() * 0x10000 },
    };
    let m = BoundingVolume::merged(a, b);
    let loose = BoundingVolume::loosened(m, Fixed { raw: 8 });
    assert!(BoundingVolume::contains(loose, a) && BoundingVolume::contains(loose, b));
    // Never larger than the two-extreme candidate beyond their rounding.
    let alt = alternatives::merged_extremes(a, b);
    assert!(m.radius <= alt.radius + Fixed { raw: 16 });
}

#[test]
#[fuzzer(runs: 128, seed: 20260926)]
fn fuzz_exact_tests_agree_with_the_candidates(x: i16, y: i16, r0: u16, r1: u16) {
    let a = BoundingSphere {
        center: Vec2 { x: ZERO, y: ZERO }, radius: Fixed { raw: r0.into() * 0x10000 },
    };
    let b = BoundingSphere {
        center: Vec2 { x: Fixed { raw: x.into() * 0x10000 }, y: Fixed { raw: y.into() * 0x10000 } },
        radius: Fixed { raw: r1.into() * 0x10000 },
    };
    assert_eq!(BoundingVolume::contains(a, b), alternatives::contains_u128(a, b));
    assert_eq!(BoundingVolume::intersects(a, b), alternatives::intersects_u128(a, b));
    // The floored length is at most the exact one: exact containment implies the candidate's.
    if BoundingVolume::contains(a, b) {
        assert!(alternatives::contains_sqrt(a, b));
    }
    // Both intersection tests compare squares; the `Fixed` one floors them, which can only add
    // hits.
    if BoundingVolume::intersects(a, b) {
        assert!(alternatives::intersects_fixed(a, b));
    }
}

#[test]
fn gas_baseline() {}
#[test]
fn gas_sphere_new() {
    let _ = BoundingSphereTrait::new(opaque(v(1, 2)), opaque(ONE));
}
#[test]
fn gas_sphere_transform_by() {
    let _ = opaque(s(3, 0, ONE)).transform_by(opaque(POSE));
}
#[test]
fn gas_sphere_translated() {
    let _ = opaque(s(3, 0, ONE)).translated(opaque(v(1, 1)));
}
#[test]
fn gas_sphere_intersects_exact() {
    let _ = BoundingVolume::intersects(opaque(s(0, 0, TWO)), opaque(s(3, 1, ONE)));
}
#[test]
fn gas_sphere_intersects_fixed() {
    let _ = alternatives::intersects_fixed(opaque(s(0, 0, TWO)), opaque(s(3, 1, ONE)));
}
#[test]
fn gas_sphere_contains_exact() {
    let _ = BoundingVolume::contains(opaque(s(0, 0, TWO)), opaque(s(1, 0, HALF)));
}
#[test]
fn gas_sphere_contains_sqrt() {
    let _ = alternatives::contains_sqrt(opaque(s(0, 0, TWO)), opaque(s(1, 0, HALF)));
}
#[test]
fn gas_sphere_merged_closed_form() {
    let _ = BoundingVolume::merged(opaque(s(0, 0, TWO)), opaque(s(3, 1, ONE)));
}
#[test]
fn gas_sphere_intersects_u128() {
    let _ = alternatives::intersects_u128(opaque(s(0, 0, TWO)), opaque(s(3, 1, ONE)));
}
#[test]
fn gas_sphere_contains_u128() {
    let _ = alternatives::contains_u128(opaque(s(0, 0, TWO)), opaque(s(1, 0, HALF)));
}
#[test]
fn gas_sphere_merged_exact_containment() {
    let _ = alternatives::merged_exact_containment(opaque(s(0, 0, TWO)), opaque(s(3, 1, ONE)));
}
#[test]
fn gas_sphere_merged_extremes() {
    let _ = alternatives::merged_extremes(opaque(s(0, 0, TWO)), opaque(s(3, 1, ONE)));
}
#[test]
fn gas_sphere_merged_contained() {
    let _ = BoundingVolume::merged(opaque(s(0, 0, TWO)), opaque(s(1, 0, HALF)));
}
#[test]
fn gas_sphere_loosened() {
    let _ = BoundingVolume::loosened(opaque(s(0, 0, TWO)), opaque(ONE));
}
#[test]
fn gas_sphere_tightened() {
    let _ = BoundingVolume::tightened(opaque(s(0, 0, TWO)), opaque(ONE));
}
#[test]
fn gas_aabb_loosened_checked() {
    let _ = BoundingVolume::loosened(opaque(aabb(-1, -2, 3, 2)), opaque(ONE));
}
#[test]
fn gas_ball_aabb() {
    let _ = ball_aabb(opaque(v(1, 2)), opaque(ONE));
}
#[test]
fn gas_local_point_cloud_aabb_4() {
    let _ = local_point_cloud_aabb(opaque(array![v(0, 0), v(2, 0), v(2, 2), v(0, 2)]).span());
}
#[test]
fn gas_point_cloud_aabb_4() {
    let _ = point_cloud_aabb(
        opaque(POSE), opaque(array![v(0, 0), v(2, 0), v(2, 2), v(0, 2)]).span(),
    );
}
#[test]
fn gas_local_support_map_aabb_cuboid() {
    let _ = local_support_map_aabb(opaque(CuboidTrait::new(v(2, 1))));
}
#[test]
fn gas_point_cloud_bounding_sphere_4() {
    let _ = point_cloud_bounding_sphere(opaque(array![v(0, 0), v(2, 0), v(2, 2), v(0, 2)]).span());
}
