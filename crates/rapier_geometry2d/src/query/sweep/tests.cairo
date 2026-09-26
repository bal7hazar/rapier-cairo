//! Tests and `gas_*` probes of the swept time of impact (`super`): upstream's own unit tests
//! (`sweep_toi.rs`), the proxies of every shape, the sweeps and the distance kernel.

use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_golden::compare::abs_diff;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::shape::{
    BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, RoundShapeTrait,
    SegmentTrait, Shape, TriangleTrait,
};
use super::{
    SimplexCache, Sweep, SweepToiStatus, SweepTrait, ToiProxy, ToiProxyTrait, proxy_distance,
    sweep_time_of_impact,
};

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

fn ratio(n: i64, d: i64) -> Fixed {
    FixedTrait::from_ratio(n, d)
}

fn pose(x: Fixed, y: Fixed) -> Pose2 {
    Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
}

/// Upstream's `slop()`: 0.005 (nearest raw).
fn slop() -> Fixed {
    Fixed { raw: 21474836 }
}

fn ball_proxy(r: Fixed) -> ToiProxy {
    ToiProxyTrait::point(v(ZERO, ZERO), r)
}

fn cuboid_proxy(h: Fixed) -> ToiProxy {
    ToiProxyTrait::from_array(array![v(-h, -h), v(h, -h), v(h, h), v(-h, h)].span(), ZERO)
}

fn near(a: Fixed, b: Fixed, tol: u64) -> bool {
    abs_diff(a.raw, b.raw) <= tol
}

/// Upstream's `ball_hits_static_box`, `ball_misses_box`,
/// `overlapped_at_start_returns_fraction_zero`, `box_vs_box_face_impact` (fractions within 0.001 of
/// the expected ones, as upstream).
#[test]
fn test_upstream_cases() {
    let wall = cuboid_proxy(HALF);
    let wall_sweep = SweepTrait::constant(pose(ZERO, ZERO), v(ZERO, ZERO));
    let ball = ball_proxy(ratio(1, 10));
    let milli = Fixed { raw: 4294967 };
    // Ball hits the static box: (3 - 0.5 - (0.1 - slop)) / 6.
    let sweep = SweepTrait::from_poses(pose(-int(3), ZERO), pose(int(3), ZERO), v(ZERO, ZERO));
    let r = sweep_time_of_impact(wall, wall_sweep, ball, sweep, ONE, slop());
    assert_eq!(r.status, SweepToiStatus::Hit);
    let expected = (int(3) - HALF - (ratio(1, 10) - slop())) / int(6);
    assert!(near(r.fraction, expected, milli.raw.try_into().unwrap()), "{:?}", r);
    // Ball misses the box.
    let sweep = SweepTrait::from_poses(pose(-int(3), int(5)), pose(int(3), int(5)), v(ZERO, ZERO));
    let r = sweep_time_of_impact(wall, wall_sweep, ball, sweep, ONE, slop());
    assert_eq!((r.status, r.fraction), (SweepToiStatus::Separated, ONE));
    // Overlapped at the start.
    let sweep = SweepTrait::from_poses(pose(ZERO, ZERO), pose(int(3), ZERO), v(ZERO, ZERO));
    let r = sweep_time_of_impact(wall, wall_sweep, ball, sweep, ONE, slop());
    assert_eq!((r.status, r.fraction), (SweepToiStatus::Overlapped, ZERO));
    // Box against box, face on face: (4 - 1 - slop) / 8.
    let sweep = SweepTrait::from_poses(pose(-int(4), ZERO), pose(int(4), ZERO), v(ZERO, ZERO));
    let r = sweep_time_of_impact(wall, wall_sweep, cuboid_proxy(HALF), sweep, ONE, slop());
    assert_eq!(r.status, SweepToiStatus::Hit);
    let expected = (int(3) - slop()) / int(8);
    assert!(near(r.fraction, expected, milli.raw.try_into().unwrap()), "{:?}", r);
    assert!(near(r.normal.x, -ONE, 16) && near(r.normal.y, ZERO, 16), "{:?}", r);
}

/// Upstream's `rotating_bar_hits_ball`: a bar of half length 2 turning a quarter turn hits a ball
/// at distance 1.5 on the diagonal between fractions 0.3 and 0.5.
#[test]
fn test_rotating_bar_hits_ball() {
    let bar = ToiProxyTrait::from_array(array![v(-TWO, ZERO), v(TWO, ZERO)].span(), ratio(1, 20));
    let quarter = Pose2Trait::new(v(ZERO, ZERO), Rot2 { re: ZERO, im: ONE });
    let bar_sweep = SweepTrait::from_poses(pose(ZERO, ZERO), quarter, v(ZERO, ZERO));
    let d = Fixed { raw: 4555500750 }; // 1.5 / sqrt(2)
    let ball_sweep = SweepTrait::constant(pose(d, d), v(ZERO, ZERO));
    let r = sweep_time_of_impact(bar, bar_sweep, ball_proxy(ratio(1, 10)), ball_sweep, ONE, slop());
    assert_eq!(r.status, SweepToiStatus::Hit);
    assert!(r.fraction > ratio(3, 10) && r.fraction < HALF, "{:?}", r);
}

#[test]
fn test_proxies_of_shapes() {
    let triangle: Shape = TriangleTrait::new(v(ZERO, ZERO), v(ONE, ZERO), v(ZERO, ONE)).into();
    let polygon: Shape = ConvexPolygonTrait::from_convex_polyline(
        array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span(),
    )
        .unwrap()
        .into();
    let cases: Array<(Shape, u32, Fixed)> = array![
        (BallTrait::new(HALF).into(), 1, HALF), (CuboidTrait::new(v(ONE, HALF)).into(), 4, ZERO),
        (CapsuleTrait::new_x(ONE, HALF).into(), 2, HALF),
        (SegmentTrait::new(v(ZERO, ZERO), v(ONE, ONE)).into(), 2, ZERO), (triangle, 3, ZERO),
        (polygon, 3, ZERO),
        (Shape::RoundCuboid(RoundShapeTrait::new(CuboidTrait::new(v(ONE, HALF)), HALF)), 4, HALF),
    ];
    for (shape, n, r) in cases {
        let p = ToiProxyTrait::from_shape(shape).unwrap();
        assert_eq!((p.points().len(), p.radius), (n, r));
    }
    assert!(ToiProxyTrait::from_shape(HalfSpaceTrait::new(v(ZERO, ONE)).into()).is_none());
    // Support and box.
    let c = cuboid_proxy(HALF);
    assert_eq!(c.support(v(ONE, ONE)), 2);
    assert_eq!(c.support(v(-ONE, -ONE)), 0);
    let aabb = ToiProxyTrait::point(v(ONE, ZERO), HALF).compute_aabb(pose(ONE, ONE));
    assert_eq!((aabb.mins, aabb.maxs), (v(TWO - HALF, HALF), v(TWO + HALF, ONE + HALF)));
}

#[test]
fn test_sweep_methods() {
    let quarter = Pose2Trait::new(v(TWO, ZERO), Rot2 { re: ZERO, im: ONE });
    let s = SweepTrait::from_poses(pose(ZERO, ZERO), quarter, v(ONE, ZERO));
    assert_eq!((s.c1, s.c2), (v(ONE, ZERO), v(TWO, ONE)));
    assert_eq!(s.transform_at(ZERO), pose(ZERO, ZERO));
    assert_eq!(s.transform_at(ONE), quarter);
    assert_eq!(s.final_transform(), quarter);
    let mid = s.transform_at(HALF);
    // Half way: rotated by 45 degrees, centre at (1.5, 0.5).
    assert!(near(mid.rotation.re, mid.rotation.im, 2));
    assert!(near(mid.transform_point(v(ONE, ZERO)).x, ONE + HALF, 4));
    assert_eq!(s.shifted(v(ONE, ONE)).c1, v(ZERO, -ONE));
    // A half turn keeps the start rotation at its midpoint (no direction to normalise).
    let half_turn = Pose2Trait::new(v(ZERO, ZERO), Rot2 { re: -ONE, im: ZERO });
    let h = SweepTrait::from_poses(pose(ZERO, ZERO), half_turn, v(ZERO, ZERO));
    assert_eq!(h.transform_at(HALF).rotation, Rot2 { re: ONE, im: ZERO });
}

/// The distance kernel: features, distances, overlap and radii.
#[test]
fn test_proxy_distance() {
    let c = cuboid_proxy(HALF);
    let mut cache: SimplexCache = Default::default();
    // Face of B against the corner... here face against face: a vertex of B on A's right face.
    let out = proxy_distance(pose(TWO, ZERO), c, c, false, ref cache);
    assert!(out.distance == ONE && out.normal == v(ONE, ZERO));
    assert_eq!(cache.count, 1);
    // A ball above the middle of the top face: vertex against edge.
    let out = proxy_distance(pose(ZERO, TWO), c, ball_proxy(HALF), false, ref cache);
    assert!(
        out.distance == ONE + HALF && out.normal == v(ZERO, ONE) && out.point_a == v(ZERO, HALF),
    );
    assert_eq!((cache.count, cache.index_a, cache.index_b), (2, [2, 3], [0, 0]));
    let with_radii = proxy_distance(pose(ZERO, TWO), c, ball_proxy(HALF), true, ref cache);
    assert!(with_radii.distance == ONE && with_radii.point_b == v(ZERO, TWO - HALF));
    // Overlapping (a point inside the box, crossing segments): zero.
    assert_eq!(
        proxy_distance(pose(ZERO, ZERO), c, ball_proxy(ONE), false, ref cache).distance, ZERO,
    );
    let seg = ToiProxyTrait::from_array(array![v(-ONE, ZERO), v(ONE, ZERO)].span(), ZERO);
    let turned = Pose2Trait::new(v(ZERO, ZERO), Rot2 { re: ZERO, im: ONE });
    assert_eq!(proxy_distance(turned, seg, seg, false, ref cache).distance, ZERO);
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_sweep_time_of_impact_ball_box() {
    let sweep = SweepTrait::from_poses(pose(-int(3), ZERO), pose(int(3), ZERO), v(ZERO, ZERO));
    let _ = sweep_time_of_impact(
        opaque(cuboid_proxy(HALF)),
        opaque(SweepTrait::constant(pose(ZERO, ZERO), v(ZERO, ZERO))),
        opaque(ball_proxy(ratio(1, 10))),
        opaque(sweep),
        opaque(ONE),
        opaque(slop()),
    );
}

#[test]
fn gas_sweep_time_of_impact_box_box() {
    let sweep = SweepTrait::from_poses(pose(-int(4), ZERO), pose(int(4), ZERO), v(ZERO, ZERO));
    let _ = sweep_time_of_impact(
        opaque(cuboid_proxy(HALF)),
        opaque(SweepTrait::constant(pose(ZERO, ZERO), v(ZERO, ZERO))),
        opaque(cuboid_proxy(HALF)),
        opaque(sweep),
        opaque(ONE),
        opaque(slop()),
    );
}

#[test]
fn gas_sweep_time_of_impact_rotating_bar_ball() {
    let bar = ToiProxyTrait::from_array(array![v(-TWO, ZERO), v(TWO, ZERO)].span(), ratio(1, 20));
    let quarter = Pose2Trait::new(v(ZERO, ZERO), Rot2 { re: ZERO, im: ONE });
    let d = Fixed { raw: 4555500750 };
    let _ = sweep_time_of_impact(
        opaque(bar),
        opaque(SweepTrait::from_poses(pose(ZERO, ZERO), quarter, v(ZERO, ZERO))),
        opaque(ball_proxy(ratio(1, 10))),
        opaque(SweepTrait::constant(pose(d, d), v(ZERO, ZERO))),
        opaque(ONE),
        opaque(slop()),
    );
}

#[test]
fn gas_proxy_distance_box_box() {
    let mut cache: SimplexCache = Default::default();
    let _ = proxy_distance(
        opaque(Pose2Trait::new(v(TWO, HALF), Rot2 { re: Fixed { raw: 3719550787 }, im: HALF })),
        opaque(cuboid_proxy(HALF)),
        opaque(cuboid_proxy(HALF)),
        false,
        ref cache,
    );
}

#[test]
fn gas_proxy_distance_exhaustive_box_box() {
    let mut cache: SimplexCache = Default::default();
    let _ = super::proxy::proxy_distance_exhaustive(
        opaque(Pose2Trait::new(v(TWO, HALF), Rot2 { re: Fixed { raw: 3719550787 }, im: HALF })),
        opaque(cuboid_proxy(HALF)),
        opaque(cuboid_proxy(HALF)),
        false,
        ref cache,
    );
}

/// The pruned and the exhaustive kernels agree (points, distance, features) over poses around a
/// box, for every proxy kind.
#[test]
fn test_pruning_agrees() {
    let proxies = array![
        cuboid_proxy(HALF), ball_proxy(HALF),
        ToiProxyTrait::from_array(array![v(-ONE, ZERO), v(ONE, ZERO)].span(), ZERO),
        ToiProxyTrait::from_array(array![v(ZERO, ZERO), v(ZERO, ONE), v(ONE, ZERO)].span(), ZERO),
    ];
    let poses = array![
        Pose2Trait::new(v(TWO, HALF), Rot2 { re: Fixed { raw: 3719550787 }, im: HALF }),
        pose(-TWO, ONE), pose(ZERO, -int(3)), pose(int(3), int(3)), pose(HALF, ZERO),
    ];
    for a in proxies.span() {
        for b in proxies.span() {
            for p in poses.span() {
                let (mut c1, mut c2): (SimplexCache, SimplexCache) = (
                    Default::default(), Default::default(),
                );
                let x = proxy_distance(*p, *a, *b, false, ref c1);
                let y = super::proxy::proxy_distance_exhaustive(*p, *a, *b, false, ref c2);
                assert_eq!(x, y);
                assert_eq!(c1, c2);
            }
        }
    }
}

#[test]
fn gas_sweep_transform_at() {
    let quarter = Pose2Trait::new(v(TWO, ZERO), Rot2 { re: ZERO, im: ONE });
    let s: Sweep = SweepTrait::from_poses(pose(ZERO, ZERO), quarter, v(ONE, ZERO));
    let _ = opaque(s).transform_at(opaque(ratio(1, 3)));
}
