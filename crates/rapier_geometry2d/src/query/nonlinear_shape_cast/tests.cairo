//! Table-driven tests and `gas_*` probes of the nonlinear shape casts (`super`).

use fixed::{FRAC_PI_2, FRAC_PI_4, Fixed, FixedTrait, HALF, MAX, ONE, PI, TWO, ZERO};
use glam_core::Vec2;
use rapier_golden::compare::abs_diff;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::shape::{
    BallTrait, CapsuleTrait, ConvexPolygonTrait, CuboidTrait, HalfSpaceTrait, RoundShapeTrait,
    SegmentTrait, Shape, TriangleTrait,
};
use super::alternatives::position_at_time_branch;
use super::support_map::compute_toi_with_tolerance;
use super::super::shape_cast::{ShapeCastStatus, cast_shapes};
use super::{
    NonlinearRigidMotion, NonlinearRigidMotionTrait, NonlinearShapeCastMode,
    NonlinearShapeCastModeTrait, cast_shapes_nonlinear, ccd_angular_thickness, ccd_thickness,
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

fn near(a: Fixed, b: Fixed, tol: u64) -> bool {
    abs_diff(a.raw, b.raw) <= tol
}

fn near_v(a: Vec2, b: Vec2, tol: u64) -> bool {
    near(a.x, b.x, tol) && near(a.y, b.y, tol)
}

fn cuboid() -> Shape {
    CuboidTrait::new(v(HALF, HALF)).into()
}

fn bar() -> Shape {
    CapsuleTrait::new_x(TWO, ratio(1, 20)).into()
}

fn small_ball() -> Shape {
    BallTrait::new(ratio(1, 10)).into()
}

fn still(pos: Pose2) -> NonlinearRigidMotion {
    NonlinearRigidMotionTrait::constant_position(pos)
}

#[test]
fn test_position_at_time() {
    // Pure translation.
    let m = NonlinearRigidMotionTrait::new(at(ONE, TWO), v(ZERO, ZERO), v(TWO, -ONE), ZERO);
    assert_eq!(m.position_at_time(HALF), at(TWO, ONE + HALF));
    // A quarter turn about the local centre (1, 0) of a body at the origin: the centre stays
    // put, the origin goes to (1, -1).
    let m = NonlinearRigidMotionTrait::new(at(ZERO, ZERO), v(ONE, ZERO), v(ZERO, ZERO), FRAC_PI_2);
    let p = m.position_at_time(ONE);
    assert!(near_v(p.translation, v(ONE, -ONE), 8), "{:?}", p);
    assert!(near(p.rotation.re, ZERO, 8) && near(p.rotation.im, ONE, 8), "{:?}", p);
    assert!(near_v(p.transform_point(v(ONE, ZERO)), v(ONE, ZERO), 8));
    // Identity and constant motions.
    assert_eq!(NonlinearRigidMotionTrait::identity().position_at_time(TWO), Default::default());
    assert_eq!(still(at(ONE, ONE)).position_at_time(int(5)), at(ONE, ONE));
}

/// `append*` / `prepend*` keep the world rotation centre; `freeze` stops at the given time.
#[test]
fn test_motion_edits() {
    let m = NonlinearRigidMotionTrait::new(at(ONE, ZERO), v(ONE, ZERO), v(ONE, ZERO), FRAC_PI_2);
    let world_center = |m: NonlinearRigidMotion| m.start.transform_point(m.local_center);
    let edits = array![
        m.append_translation(v(ZERO, TWO)), m.prepend_translation(v(ZERO, TWO)),
        m.append(at(ONE, ONE)), m.prepend(at(ONE, ONE)),
    ];
    let expected_starts = array![at(ONE, TWO), at(ONE, TWO), at(TWO, ONE), at(TWO, ONE)];
    let mut i = 0;
    for e in edits.span() {
        assert!(*e.start == *expected_starts[i]);
        assert_eq!(world_center(*e), world_center(m));
        assert!(*e.linvel == m.linvel && *e.angvel == m.angvel);
        i += 1;
    }
    let mut f = m;
    f.freeze(HALF);
    assert_eq!(f.start, m.position_at_time(HALF));
    assert_eq!((f.linvel, f.angvel), (v(ZERO, ZERO), ZERO));
}

#[test]
fn test_ccd_thickness_table() {
    let polygon: Shape = ConvexPolygonTrait::from_convex_polyline(
        array![v(-ONE, -HALF), v(ONE, -HALF), v(ZERO, ONE)].span(),
    )
        .unwrap()
        .into();
    let triangle: Shape = TriangleTrait::new(v(-ONE, ZERO), v(ONE, ZERO), v(ZERO, ONE)).into();
    let rcub = Shape::RoundCuboid(RoundShapeTrait::new(CuboidTrait::new(v(ONE, HALF)), HALF));
    let cases = array![
        (small_ball(), ratio(1, 10), PI), (cuboid(), HALF, FRAC_PI_2),
        (bar(), ratio(1, 20), FRAC_PI_2),
        (SegmentTrait::new(v(ZERO, ZERO), v(ONE, ZERO)).into(), ZERO, FRAC_PI_2),
        (HalfSpaceTrait::new(v(ZERO, ONE)).into(), MAX, PI), (polygon, ratio(3, 4), FRAC_PI_4),
        (triangle, ZERO, FRAC_PI_2), (rcub, ONE, FRAC_PI_2),
    ];
    for (shape, thickness, angular) in cases {
        assert_eq!((ccd_thickness(shape), ccd_angular_thickness(shape)), (thickness, angular));
    }
    assert_eq!(
        NonlinearShapeCastModeTrait::directional_toi(small_ball(), cuboid()),
        NonlinearShapeCastMode::Directional((ratio(1, 10) + HALF, PI)),
    );
}

/// Without rotation, the nonlinear cast finds the linear time of impact (within its tolerance).
#[test]
fn test_translation_matches_linear_cast() {
    let shapes = array![cuboid(), small_ball(), bar()];
    let pos2 = Pose2Trait::new(
        v(ratio(1, 3), int(3)), Rot2 { re: Fixed { raw: 3719550787 }, im: HALF },
    );
    let vel2 = v(ratio(-1, 10), -ONE);
    for g1 in shapes.span() {
        for g2 in shapes.span() {
            let linear = cast_shapes(
                at(ZERO, ZERO), v(ZERO, ZERO), *g1, pos2, vel2, *g2, Default::default(),
            )
                .unwrap()
                .unwrap();
            let m2 = NonlinearRigidMotionTrait::new(pos2, v(ZERO, ZERO), vel2, ZERO);
            let nonlinear = cast_shapes_nonlinear(
                still(at(ZERO, ZERO)), *g1, m2, *g2, ZERO, int(4), true,
            )
                .unwrap()
                .unwrap();
            assert_eq!(nonlinear.status, ShapeCastStatus::Converged);
            assert!(
                near(nonlinear.time_of_impact, linear.time_of_impact, 0x4000),
                "{:?} {:?}",
                nonlinear,
                linear,
            );
        }
    }
}

/// A bar of half length 2 (radius 0.05) turning a quarter turn about its centre hits a ball of
/// radius 0.1 at distance 1.5 on the diagonal: at angle `45deg - asin(0.15 / 1.5)`.
#[test]
fn test_rotating_bar() {
    let d = Fixed { raw: 4555500750 }; // 1.5 / sqrt(2)
    let bar_motion = NonlinearRigidMotionTrait::new(
        at(ZERO, ZERO), v(ZERO, ZERO), v(ZERO, ZERO), FRAC_PI_2,
    );
    let hit = cast_shapes_nonlinear(
        bar_motion, bar(), still(at(d, d)), small_ball(), ZERO, ONE, true,
    )
        .unwrap()
        .unwrap();
    // asin(0.1) = 0.100167421; (pi/4 - 0.100167421) / (pi/2) = 0.436230...
    let expected = Fixed { raw: 1873599765 };
    assert!(near(hit.time_of_impact, expected, 0x4000), "{:?}", hit);
    // The bisection range collapsed onto a slightly penetrating time: upstream's `Failed`.
    assert!(hit.status == ShapeCastStatus::Converged || hit.status == ShapeCastStatus::Failed);
    // Too short a time interval: no impact.
    assert!(
        cast_shapes_nonlinear(
            bar_motion, bar(), still(at(d, d)), small_ball(), ZERO, ratio(2, 5), true,
        )
            .unwrap()
            .is_none(),
    );
    // The swapped order answers the same time.
    let swapped = cast_shapes_nonlinear(
        still(at(d, d)), small_ball(), bar_motion, bar(), ZERO, ONE, true,
    )
        .unwrap()
        .unwrap();
    assert!(near(swapped.time_of_impact, hit.time_of_impact, 16));
}

/// Starts in contact: `StopAtPenetration` answers the start; the directional mode ignores a
/// separating start and keeps an approaching one.
#[test]
fn test_start_in_contact() {
    let ground = still(at(ZERO, ZERO));
    let up = NonlinearRigidMotionTrait::new(at(ZERO, ONE), v(ZERO, ZERO), v(ZERO, ONE), ZERO);
    let down = NonlinearRigidMotionTrait::new(at(ZERO, ONE), v(ZERO, ZERO), v(ZERO, -ONE), ZERO);
    let hit = cast_shapes_nonlinear(ground, cuboid(), up, cuboid(), ZERO, ONE, true)
        .unwrap()
        .unwrap();
    assert_eq!(
        (hit.time_of_impact, hit.status), (ZERO, ShapeCastStatus::PenetratingOrWithinTargetDist),
    );
    assert!(
        cast_shapes_nonlinear(ground, cuboid(), up, cuboid(), ZERO, ONE, false).unwrap().is_none(),
    );
    // Approaching: a hit only when the travel before the end exceeds the thickness (1).
    assert!(
        cast_shapes_nonlinear(ground, cuboid(), down, cuboid(), ZERO, ONE, false)
            .unwrap()
            .is_none(),
    );
    let hit = cast_shapes_nonlinear(ground, cuboid(), down, cuboid(), ZERO, TWO, false)
        .unwrap()
        .unwrap();
    assert_eq!(hit.time_of_impact, ZERO);
    // Overlapping: penetrating status.
    let inside = NonlinearRigidMotionTrait::new(at(ZERO, HALF), v(ZERO, ZERO), v(ZERO, ONE), ZERO);
    let hit = cast_shapes_nonlinear(ground, cuboid(), inside, cuboid(), ZERO, ONE, true)
        .unwrap()
        .unwrap();
    assert_eq!(hit.status, ShapeCastStatus::PenetratingOrWithinTargetDist);
    // Half-spaces are unsupported.
    let hs: Shape = HalfSpaceTrait::new(v(ZERO, ONE)).into();
    assert!(cast_shapes_nonlinear(ground, hs, up, cuboid(), ZERO, ONE, true).is_none());
    assert!(cast_shapes_nonlinear(ground, cuboid(), up, hs, ZERO, ONE, true).is_none());
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_position_at_time_translation() {
    let m = NonlinearRigidMotionTrait::new(at(ONE, TWO), v(ZERO, ZERO), v(TWO, -ONE), ZERO);
    let _ = opaque(m).position_at_time(opaque(HALF));
}

#[test]
fn gas_position_at_time_rotation() {
    let m = NonlinearRigidMotionTrait::new(at(ONE, TWO), v(HALF, ZERO), v(TWO, -ONE), FRAC_PI_2);
    let _ = opaque(m).position_at_time(opaque(HALF));
}

#[test]
fn gas_position_at_time_branch_translation() {
    let m = NonlinearRigidMotionTrait::new(at(ONE, TWO), v(ZERO, ZERO), v(TWO, -ONE), ZERO);
    let _ = position_at_time_branch(opaque(m), opaque(HALF));
}

#[test]
fn gas_position_at_time_branch_rotation() {
    let m = NonlinearRigidMotionTrait::new(at(ONE, TWO), v(HALF, ZERO), v(TWO, -ONE), FRAC_PI_2);
    let _ = position_at_time_branch(opaque(m), opaque(HALF));
}

/// The metered and the branching `position_at_time` agree.
#[test]
fn test_position_at_time_candidates_agree() {
    let motions = array![
        NonlinearRigidMotionTrait::new(at(ONE, TWO), v(HALF, ZERO), v(TWO, -ONE), FRAC_PI_2),
        NonlinearRigidMotionTrait::new(at(ONE, TWO), v(ZERO, ZERO), v(TWO, -ONE), ZERO),
        NonlinearRigidMotionTrait::new(
            Pose2Trait::new(v(-ONE, HALF), Rot2 { re: ZERO, im: ONE }),
            v(ONE, ONE),
            v(ZERO, ONE),
            -PI,
        ),
    ];
    for m in motions {
        for t in array![ZERO, HALF, ONE, int(3)] {
            assert_eq!(m.position_at_time(t), position_at_time_branch(m, t));
        }
    }
}

#[test]
fn gas_cast_shapes_nonlinear_translation_cuboid_cuboid() {
    let m2 = NonlinearRigidMotionTrait::new(
        at(ratio(1, 3), int(3)), v(ZERO, ZERO), v(ratio(-1, 10), -ONE), ZERO,
    );
    let _ = cast_shapes_nonlinear(
        opaque(still(at(ZERO, ZERO))),
        opaque(cuboid()),
        opaque(m2),
        opaque(cuboid()),
        opaque(ZERO),
        opaque(int(4)),
        opaque(true),
    );
}

#[test]
fn gas_cast_shapes_nonlinear_rotating_bar_ball() {
    let d = Fixed { raw: 4555500750 };
    let bar_motion = NonlinearRigidMotionTrait::new(
        at(ZERO, ZERO), v(ZERO, ZERO), v(ZERO, ZERO), FRAC_PI_2,
    );
    let _ = cast_shapes_nonlinear(
        opaque(bar_motion),
        opaque(bar()),
        opaque(still(at(d, d))),
        opaque(small_ball()),
        opaque(ZERO),
        opaque(ONE),
        opaque(true),
    );
}

#[test]
fn gas_cast_shapes_nonlinear_directional_start_in_contact() {
    let down = NonlinearRigidMotionTrait::new(
        at(ZERO, ONE), v(ZERO, ZERO), v(ZERO, -ONE), ratio(1, 2),
    );
    let _ = cast_shapes_nonlinear(
        opaque(still(at(ZERO, ZERO))),
        opaque(cuboid()),
        opaque(down),
        opaque(cuboid()),
        opaque(ZERO),
        opaque(ONE),
        opaque(false),
    );
}

/// The finer tolerance (64 ulp instead of `TOLERANCE`): same impact, more bisection steps.
#[test]
fn gas_compute_toi_rotating_bar_ball_tolerance_64() {
    let d = Fixed { raw: 4555500750 };
    let bar_motion = NonlinearRigidMotionTrait::new(
        at(ZERO, ZERO), v(ZERO, ZERO), v(ZERO, ZERO), FRAC_PI_2,
    );
    let _ = compute_toi_with_tolerance(
        opaque(bar_motion),
        opaque(bar()),
        opaque(still(at(d, d))),
        opaque(small_ball()),
        opaque(ZERO),
        opaque(ONE),
        NonlinearShapeCastMode::StopAtPenetration,
        opaque(Fixed { raw: 64 }),
    );
}

#[test]
fn gas_compute_toi_translation_cuboid_cuboid_tolerance_64() {
    let m2 = NonlinearRigidMotionTrait::new(
        at(ratio(1, 3), int(3)), v(ZERO, ZERO), v(ratio(-1, 10), -ONE), ZERO,
    );
    let _ = compute_toi_with_tolerance(
        opaque(still(at(ZERO, ZERO))),
        opaque(cuboid()),
        opaque(m2),
        opaque(cuboid()),
        opaque(ZERO),
        opaque(int(4)),
        NonlinearShapeCastMode::StopAtPenetration,
        opaque(Fixed { raw: 64 }),
    );
}
