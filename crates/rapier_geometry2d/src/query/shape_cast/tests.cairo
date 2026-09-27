//! Table-driven tests and `gas_*` probes of the linear shape casts (`super`).

use fixed::{Fixed, FixedTrait, HALF, MAX, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_golden::compare::abs_diff;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::{Rot2, Rot2Trait};
use rapier_testing::opaque;
use crate::point::PointQuery;
use crate::shape::{
    BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, RoundShapeTrait,
    SegmentTrait, Shape, TriangleTrait,
};
use super::alternatives::{cast_conservative_advancement, cso_cast_clipped, on_face_every_face};
use super::support_map::{cso_cast, on_face};
use super::super::support_map::{core_witness, local_core, transformed};
use super::{
    ShapeCastHit, ShapeCastHitTrait, ShapeCastOptions, ShapeCastOptionsTrait, ShapeCastStatus,
    cast_shapes, cast_shapes_local, cast_shapes_support_map_support_map,
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

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2Trait::new(v(x, y), Rot2 { re: ONE, im: ZERO })
}

/// 30 degrees (`im = 0.5`, `re` the Q32.32 cosine).
fn r30() -> Rot2 {
    Rot2 { re: Fixed { raw: 3719550787 }, im: HALF }
}

fn cuboid() -> Shape {
    CuboidTrait::new(v(HALF, HALF)).into()
}

fn ball() -> Shape {
    BallTrait::new(HALF).into()
}

fn capsule() -> Shape {
    CapsuleTrait::new_x(HALF, ratio(1, 4)).into()
}

fn segment() -> Shape {
    SegmentTrait::new(v(ZERO, -ONE), v(ZERO, ONE)).into()
}

fn flat_segment() -> Shape {
    SegmentTrait::new(v(-ONE, ZERO), v(ONE, ZERO)).into()
}

fn polygon() -> Shape {
    ConvexPolygonTrait::from_convex_polyline(
        array![v(-ONE, -ONE), v(ONE, -ONE), v(ZERO, ONE)].span(),
    )
        .unwrap()
        .into()
}

fn triangle() -> Shape {
    TriangleTrait::new(v(-ONE, -HALF), v(ONE, -HALF), v(ratio(1, 4), ratio(3, 4))).into()
}

fn round_cuboid() -> Shape {
    Shape::RoundCuboid(
        RoundShapeTrait::new(CuboidTrait::new(v(ratio(2, 5), ratio(1, 4))), ratio(1, 10)),
    )
}

fn options(max: Fixed, target: Fixed, stop: bool, geometry: bool) -> ShapeCastOptions {
    ShapeCastOptions {
        max_time_of_impact: max,
        target_distance: target,
        stop_at_penetration: stop,
        compute_impact_geometry_on_penetration: geometry,
    }
}

fn near(a: Fixed, b: Fixed, tol: u64) -> bool {
    abs_diff(a.raw, b.raw) <= tol
}

fn near_v(a: Vec2, b: Vec2, tol: u64) -> bool {
    near(a.x, b.x, tol) && near(a.y, b.y, tol)
}

/// `|signed distance of p to the surface of shape|` is within `tol` raws.
fn on_surface(shape: Shape, p: Vec2, tol: u64) -> bool {
    let proj = shape.project_local_point(p, false);
    near_v(proj.point, p, tol)
}

/// The support-map pairs: `(shape1, shape2, pos12, vel12, options) -> (toi, normal1)` or a miss.
#[test]
fn test_support_map_table() {
    let corner_toi = ONE + HALF - HALF * Fixed { raw: 3037000500 };
    let diag = v(Fixed { raw: 3037000500 }, Fixed { raw: 3037000500 });
    let dflt: ShapeCastOptions = Default::default();
    let cases: Array<
        (felt252, Shape, Shape, Pose2, Vec2, ShapeCastOptions, Option<(Fixed, Vec2)>),
    > =
        array![
        // Face against face, head-on.
        (
            'cub-cub',
            cuboid(),
            cuboid(),
            at(int(3), ZERO),
            v(-ONE, ZERO),
            dflt,
            Some((TWO, v(ONE, ZERO))),
        ),
        // Sliding exactly along the face line, then hitting the side face.
        (
            'graze',
            cuboid(),
            cuboid(),
            at(int(3), ONE),
            v(-ONE, ZERO),
            dflt,
            Some((TWO, v(ONE, ZERO))),
        ),
        // Passing above.
        ('miss', cuboid(), cuboid(), at(int(3), TWO), v(-ONE, ZERO), dflt, None),
        // Moving away.
        ('away', cuboid(), cuboid(), at(int(3), ZERO), v(ONE, ZERO), dflt, None),
        // Corner of a box on a ball, along the diagonal: the circle of the corner.
        (
            'ball-corner',
            ball(),
            cuboid(),
            at(TWO, TWO),
            v(-ONE, -ONE),
            dflt,
            Some((corner_toi, diag)),
        ),
        // Capsules end to end.
        (
            'cap-cap',
            capsule(),
            capsule(),
            at(int(4), ZERO),
            v(-ONE, ZERO),
            dflt,
            Some((int(4) - ONE - HALF, v(ONE, ZERO))),
        ),
        // A flat segment falling on a polygon's apex.
        (
            'poly-seg',
            polygon(),
            flat_segment(),
            at(ZERO, int(3)),
            v(ZERO, -ONE),
            dflt,
            Some((TWO, v(ZERO, ONE))),
        ),
        // Target distance and max time of impact.
        (
            'target',
            cuboid(),
            cuboid(),
            at(int(3), ZERO),
            v(-ONE, ZERO),
            options(MAX, HALF, true, true),
            Some((ONE + HALF, v(ONE, ZERO))),
        ),
        (
            'max-cut',
            cuboid(),
            cuboid(),
            at(int(3), ZERO),
            v(-ONE, ZERO),
            options(ONE + HALF, ZERO, true, true),
            None,
        ),
        (
            'max-in',
            cuboid(),
            cuboid(),
            at(int(3), ZERO),
            v(-ONE, ZERO),
            options(TWO, ZERO, true, true),
            Some((TWO, v(ONE, ZERO))),
        ),
        // Zero velocity never hits, even overlapping.
        ('still', cuboid(), cuboid(), at(HALF, ZERO), v(ZERO, ZERO), dflt, None),
        // Triangle and round cuboid.
        (
            'tri-rcub',
            triangle(),
            round_cuboid(),
            at(ZERO, int(3)),
            v(ZERO, -TWO),
            dflt,
            Some((ratio(1, 2) * (int(3) - ratio(3, 4) - ratio(35, 100)), v(ZERO, ONE))),
        ),
    ];
    for (id, g1, g2, pos12, vel12, opts, expected) in cases {
        let answer = cast_shapes_support_map_support_map(pos12, vel12, g1, g2, opts);
        match expected {
            None => assert!(answer.is_none(), "{} hit", id),
            Some((
                toi, normal,
            )) => {
                let h = answer.expect(id);
                assert!(
                    near(h.time_of_impact, toi, 8), "{} toi {:?} {:?}", id, h.time_of_impact, toi,
                );
                assert!(near_v(h.normal1, normal, 8), "{} normal {:?}", id, h.normal1);
                assert_eq!(h.status, ShapeCastStatus::Converged, "{}", id);
                assert!(on_surface(g1, h.witness1, 16), "{} witness1 {:?}", id, h.witness1);
                assert!(on_surface(g2, h.witness2, 16), "{} witness2 {:?}", id, h.witness2);
            },
        }
    }
}

/// Starts in contact: `t = 0`, the status, the contact geometry and the penetration flags.
#[test]
fn test_start_in_contact() {
    let (g, left) = (cuboid(), v(-ONE, ZERO));
    // Touching, approaching: converged at 0, the contact normal.
    let h = cast_shapes_support_map_support_map(at(ONE, ZERO), left, g, g, Default::default())
        .unwrap();
    assert_eq!((h.time_of_impact, h.status), (ZERO, ShapeCastStatus::Converged));
    assert_eq!(h.normal1, v(ONE, ZERO));
    // Touching, separating: a hit when stopping at penetration, none otherwise.
    let away = v(ONE, ZERO);
    assert!(
        cast_shapes_support_map_support_map(at(ONE, ZERO), away, g, g, Default::default())
            .is_some(),
    );
    assert!(
        cast_shapes_support_map_support_map(
            at(ONE, ZERO), away, g, g, options(MAX, ZERO, false, true),
        )
            .is_none(),
    );
    // Penetrating: t = 0, penetrating, contact geometry.
    let h = cast_shapes_support_map_support_map(at(HALF, ZERO), left, g, g, Default::default())
        .unwrap();
    assert_eq!(
        (h.time_of_impact, h.status), (ZERO, ShapeCastStatus::PenetratingOrWithinTargetDist),
    );
    assert_eq!((h.normal1, h.normal2), (v(ONE, ZERO), v(-ONE, ZERO)));
    // Without contact geometry: upstream's `-vel12 / |vel12|` normal.
    let h = cast_shapes_support_map_support_map(
        at(HALF, ZERO), v(ZERO, -TWO), g, g, options(MAX, ZERO, true, false),
    )
        .unwrap();
    assert_eq!((h.time_of_impact, h.normal1), (ZERO, v(ZERO, ONE)));
    // Within the target distance only.
    let h = cast_shapes_support_map_support_map(
        at(ONE + ratio(1, 4), ZERO), left, g, g, options(MAX, HALF, true, true),
    )
        .unwrap();
    assert_eq!(
        (h.time_of_impact, h.status), (ZERO, ShapeCastStatus::PenetratingOrWithinTargetDist),
    );
    // A ball in the corner region of a box (inside the clipped polygon, outside the shape):
    // its circle decides.
    let h = cast_shapes_support_map_support_map(
        at(ONE, ONE), v(-ONE, ZERO), cuboid(), ball(), Default::default(),
    )
        .unwrap();
    // Grazing the corner's circle: the ray `y = 0` touches the circle of radius 0.5 around
    // `(-0.5, -0.5)` at its top.
    assert_eq!((h.time_of_impact, h.normal1), (HALF, v(ZERO, ONE)));
    assert!(on_surface(cuboid(), h.witness1, 16) && on_surface(ball(), h.witness2, 16));
}

/// Swapping the shapes swaps the hit: same time, witnesses and normals exchanged.
#[test]
fn test_symmetry() {
    let shapes = array![
        cuboid(), ball(), capsule(), segment(), polygon(), triangle(), round_cuboid(),
    ];
    let pos12 = Pose2Trait::new(v(ratio(1, 3), int(3)), r30());
    let vel12 = v(ratio(-1, 10), -ONE);
    for g1 in shapes.span() {
        for g2 in shapes.span() {
            let a = cast_shapes_local(pos12, vel12, *g1, *g2, Default::default()).unwrap();
            let b = cast_shapes_local(
                pos12.inverse(),
                -pos12.rotation.inverse_rotate(vel12),
                *g2,
                *g1,
                Default::default(),
            )
                .unwrap();
            assert_eq!(a.is_some(), b.is_some());
            if let (Some(a), Some(b)) = (a, b) {
                let b = b.swapped();
                assert!(near(a.time_of_impact, b.time_of_impact, 64), "{:?}", a);
                assert!(near_v(a.normal1, b.normal1, 64), "{:?} {:?}", a, b);
                assert!(near_v(a.witness1, b.witness1, 1024), "{:?} {:?}", a, b);
            }
        }
    }
}

/// The table and the world wrapper: unsupported half-space pair, forwarding.
#[test]
fn test_table_and_world() {
    let hs: Shape = HalfSpaceTrait::new(v(ZERO, ONE)).into();
    assert!(cast_shapes_local(at(ZERO, ONE), v(ZERO, -ONE), hs, hs, Default::default()).is_none());
    let pos1 = Pose2Trait::new(v(ONE, TWO), r30());
    let pos2 = at(int(4), int(3));
    let (vel1, vel2) = (v(ONE, ZERO), v(-ONE, -ONE));
    let pos12 = pos1.inv_mul(pos2);
    let vel12 = pos1.rotation.inverse_rotate(vel2 - vel1);
    for g in array![ball(), cuboid(), hs, capsule()] {
        assert_eq!(
            cast_shapes(pos1, vel1, cuboid(), pos2, vel2, g, Default::default()),
            cast_shapes_local(pos12, vel12, cuboid(), g, Default::default()),
        );
    }
    let hit = cast_shapes(
        at(ZERO, ZERO),
        ZERO_V,
        ball(),
        at(int(3), ZERO),
        v(-ONE, ZERO),
        ball(),
        ShapeCastOptionsTrait::with_max_time_of_impact(TWO),
    )
        .unwrap()
        .unwrap();
    assert_eq!(hit.time_of_impact, TWO);
    let moved: ShapeCastHit = hit.transform1_by(at(ONE, ZERO));
    assert_eq!(moved.witness1, v(ONE + HALF, ZERO));
}

const ZERO_V: Vec2 = Vec2 { x: ZERO, y: ZERO };

/// Conservative advancement agrees with the winner on its hits.
#[test]
fn test_alternative_agrees() {
    let pos12 = Pose2Trait::new(v(ratio(1, 3), int(3)), r30());
    let vel12 = v(ratio(-1, 10), -ONE);
    for g in array![cuboid(), ball(), capsule(), polygon(), round_cuboid()] {
        let a = cast_shapes_support_map_support_map(pos12, vel12, cuboid(), g, Default::default())
            .unwrap();
        let b = cast_conservative_advancement(pos12, vel12, cuboid(), g, Default::default())
            .unwrap();
        assert!(near(a.time_of_impact, b.time_of_impact, 0x10000), "{:?} {:?}", a, b);
    }
}

/// The winner's lazy entry and full clipping agree, hits and misses, over a sweep of directions.
#[test]
fn test_clipped_agrees() {
    let pos12 = Pose2Trait::new(v(ratio(1, 3), int(3)), r30());
    let dirs = array![
        v(ZERO, -ONE), v(ratio(-1, 10), -ONE), v(ONE, -ONE), v(-ONE, -ONE), v(ONE, ZERO),
        v(ZERO, ONE), v(ratio(-1, 2), -ONE), v(ratio(3, 10), -ONE),
    ];
    let shapes = array![cuboid(), ball(), capsule(), segment(), polygon(), round_cuboid()];
    for g1 in shapes.span() {
        for g2 in shapes.span() {
            let core1 = local_core(*g1);
            let core2 = transformed(local_core(*g2), pos12);
            let radius = core1.radius + core2.radius;
            for d in dirs.span() {
                assert_eq!(
                    cso_cast(core1, core2, radius, *d, MAX),
                    cso_cast_clipped(core1, core2, radius, *d, MAX),
                );
            }
        }
    }
}

#[test]
fn gas_baseline() {}

fn probe_cso(g1: Shape, g2: Shape, clipped: bool) {
    let pos12 = opaque(Pose2Trait::new(v(ratio(1, 3), int(3)), r30()));
    let core1 = local_core(opaque(g1));
    let core2 = transformed(local_core(opaque(g2)), pos12);
    let dir = opaque(v(ratio(-1, 10), -ONE));
    let _ = if clipped {
        cso_cast_clipped(core1, core2, core1.radius + core2.radius, dir, MAX)
    } else {
        cso_cast(core1, core2, core1.radius + core2.radius, dir, MAX)
    };
}

#[test]
fn gas_cso_cast_cuboid_cuboid() {
    probe_cso(cuboid(), cuboid(), false);
}

#[test]
fn gas_cso_cast_clipped_cuboid_cuboid() {
    probe_cso(cuboid(), cuboid(), true);
}

#[test]
fn gas_cso_cast_polygon_capsule() {
    probe_cso(polygon(), capsule(), false);
}

#[test]
fn gas_cso_cast_clipped_polygon_capsule() {
    probe_cso(polygon(), capsule(), true);
}

fn probe(g1: Shape, g2: Shape) {
    let _ = cast_shapes_support_map_support_map(
        opaque(Pose2Trait::new(v(ratio(1, 3), int(3)), r30())),
        opaque(v(ratio(-1, 10), -ONE)),
        opaque(g1),
        opaque(g2),
        opaque(Default::default()),
    );
}

fn probe_ca(g1: Shape, g2: Shape) {
    let _ = cast_conservative_advancement(
        opaque(Pose2Trait::new(v(ratio(1, 3), int(3)), r30())),
        opaque(v(ratio(-1, 10), -ONE)),
        opaque(g1),
        opaque(g2),
        opaque(Default::default()),
    );
}

#[test]
fn gas_cast_support_map_ball_cuboid() {
    probe(ball(), cuboid());
}

#[test]
fn gas_cast_support_map_cuboid_cuboid() {
    probe(cuboid(), cuboid());
}

#[test]
fn gas_cast_support_map_capsule_capsule() {
    probe(capsule(), capsule());
}

#[test]
fn gas_cast_support_map_polygon_triangle() {
    probe(polygon(), triangle());
}

#[test]
fn gas_cast_support_map_round_cuboid_cuboid() {
    probe(round_cuboid(), cuboid());
}

#[test]
fn gas_cast_conservative_advancement_ball_cuboid() {
    probe_ca(ball(), cuboid());
}

#[test]
fn gas_cast_conservative_advancement_cuboid_cuboid() {
    probe_ca(cuboid(), cuboid());
}

#[test]
fn gas_cast_conservative_advancement_round_cuboid_cuboid() {
    probe_ca(round_cuboid(), cuboid());
}

#[test]
fn gas_cast_shapes_world_cuboid_cuboid() {
    let _ = cast_shapes(
        opaque(Pose2Trait::new(v(ONE, TWO), r30())),
        opaque(v(ONE, ZERO)),
        opaque(cuboid()),
        opaque(at(int(4), int(3))),
        opaque(v(-ONE, -ONE)),
        opaque(cuboid()),
        opaque(Default::default()),
    );
}

/// A wide floor (20 x 1): long faces make the rounding of a projection on them visible.
fn floor() -> Shape {
    CuboidTrait::new(v(int(10), HALF)).into()
}

/// The character-shaped start pairs: `(id, shape1, shape2, pos12)`, shape 2 resting on (or within
/// `START_TD` of) a face of shape 1, at an `x` that rounds.
fn start_pairs() -> Array<(felt252, Shape, Shape, Pose2)> {
    let x = ratio(1, 3);
    array![
        ('ball-floor', floor(), ball(), at(x, ONE)),
        ('cub-floor', floor(), cuboid(), at(x, ONE + ratio(1, 400))),
        ('floor-ball', ball(), floor(), at(-x, -ONE)),
        ('cap-floor', floor(), capsule(), at(x, ratio(3, 4) + ratio(1, 400))),
        ('rcub-floor', floor(), round_cuboid(), at(x, ratio(17, 20))),
        ('tri-floor', floor(), triangle(), at(x, ONE + ratio(1, 400))),
        ('ball-wall', floor(), ball(), at(int(10) + HALF + ratio(1, 400), ratio(1, 7))),
        ('ball-turned', floor(), ball(), Pose2Trait::new(v(ratio(-1, 3), ONE), r30())),
    ]
}

/// The target distance of the start probes (KC1's offset: `0.01` of a 0.5 half-height).
const START_TD: Fixed = Fixed { raw: 21474836 };

/// A start on a face: the start witness normal is the face's exact normal, on both
/// `stop_at_penetration` paths, and a move exactly along the face is not a hit without stopping
/// at penetration (upstream's `normal · vel >= 0`).
#[test]
fn test_start_face_normal_exact() {
    let up = v(ZERO, ONE);
    let expected = array![up, up, -up, up, up, up, v(ONE, ZERO), up];
    let mut expected = expected.span();
    for (id, g1, g2, pos12) in start_pairs() {
        let n = *expected.pop_front().unwrap();
        let into = v(ratio(1, 3), ZERO) - n;
        let along = v(-n.y, n.x);
        let stop = options(MAX, START_TD, true, true);
        let h = cast_shapes_support_map_support_map(pos12, into, g1, g2, stop).expect(id);
        assert_eq!(h.time_of_impact, ZERO, "{}", id);
        assert_eq!(h.normal1, n, "{} normal1", id);
        let n2 = -pos12.rotation.inverse_rotate(n);
        assert_eq!(h.normal2, n2, "{} normal2", id);
        let go = options(MAX, START_TD, false, true);
        let h = cast_shapes_support_map_support_map(pos12, into, g1, g2, go).expect(id);
        assert_eq!(h.normal1, n, "{} normal1 (no stop)", id);
        let h = cast_shapes_support_map_support_map(pos12, along, g1, g2, go);
        assert!(h.is_none(), "{} parallel move hit {:?}", id, h);
    }
}

fn probe_start(index: u32, touching: bool) {
    let (_, g1, g2, pos12) = *start_pairs().span()[index];
    let pos12 = if touching {
        pos12
    } else {
        Pose2Trait::new(pos12.translation + v(ZERO, TWO), pos12.rotation)
    };
    let _ = cast_shapes_support_map_support_map(
        opaque(pos12),
        opaque(v(ratio(1, 3), -ONE)),
        opaque(g1),
        opaque(g2),
        opaque(options(MAX, START_TD, false, true)),
    );
}

#[test]
fn gas_cast_start_touching_ball_floor() {
    probe_start(0, true);
}

#[test]
fn gas_cast_start_touching_cuboid_floor() {
    probe_start(1, true);
}

#[test]
fn gas_cast_start_touching_capsule_floor() {
    probe_start(3, true);
}

#[test]
fn gas_cast_start_touching_triangle_floor() {
    probe_start(5, true);
}

#[test]
fn gas_cast_start_apart_ball_floor() {
    probe_start(0, false);
}

#[test]
fn gas_cast_start_apart_cuboid_floor() {
    probe_start(1, false);
}

#[test]
fn gas_cast_start_apart_capsule_floor() {
    probe_start(3, false);
}

#[test]
fn gas_cast_start_apart_triangle_floor() {
    probe_start(5, false);
}

/// The two start snaps agree on every start pair (and its apart variant).
#[test]
fn test_on_face_alternative_agrees() {
    for (id, g1, g2, pos12) in start_pairs() {
        let core1 = local_core(g1);
        for dy in array![ZERO, TWO] {
            let pos = Pose2Trait::new(pos12.translation + v(ZERO, dy), pos12.rotation);
            let core2 = transformed(local_core(g2), pos);
            let w = core_witness(core1, core2);
            assert_eq!(on_face(core1, core2, w), on_face_every_face(core1, core2, w), "{}", id);
        }
    }
}

fn probe_on_face(index: u32, every: bool) {
    let (_, g1, g2, pos12) = *start_pairs().span()[index];
    let core1 = local_core(opaque(g1));
    let core2 = transformed(local_core(opaque(g2)), opaque(pos12));
    let w = core_witness(core1, core2);
    let _ = if every {
        on_face_every_face(core1, core2, w)
    } else {
        on_face(core1, core2, w)
    };
}

#[test]
fn gas_on_face_cuboid_floor() {
    probe_on_face(1, false);
}

#[test]
fn gas_on_face_every_face_cuboid_floor() {
    probe_on_face(1, true);
}

#[test]
fn gas_on_face_floor_ball() {
    probe_on_face(2, false);
}

#[test]
fn gas_on_face_every_face_floor_ball() {
    probe_on_face(2, true);
}
