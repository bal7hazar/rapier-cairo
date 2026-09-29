//! Tests and `gas_*` probes of [`RigidBodyCcd`]: defaults, the point-velocity bound and the two
//! fast-body tests at and around their threshold (strict: `motion > thickness / 2`).

use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam_core::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::rigid_body::{RigidBodyForces, RigidBodyPosition, RigidBodyVelocity};
use super::{FAST_BODY_SAFETY_FACTOR, NO_THICKNESS, RigidBodyCcd, RigidBodyCcdTrait};

fn ratio(n: i64, d: i64) -> Fixed {
    FixedTrait::from_ratio(n, d)
}

fn vels(x: Fixed, y: Fixed, w: Fixed) -> RigidBodyVelocity {
    RigidBodyVelocity { linvel: Vec2 { x, y }, angvel: w }
}

fn thick(t: Fixed) -> RigidBodyCcd {
    RigidBodyCcd { ccd_thickness: t, ..Default::default() }
}

fn at(x: Fixed, y: Fixed, rotation: Rot2) -> Pose2 {
    Pose2Trait::new(Vec2 { x, y }, rotation)
}

const IDENTITY: Rot2 = Rot2 { re: ONE, im: ZERO };

#[test]
fn test_defaults_match_upstream() {
    let ccd: RigidBodyCcd = Default::default();
    assert_eq!(ccd.ccd_thickness, NO_THICKNESS);
    assert!(!ccd.ccd_active && !ccd.ccd_enabled);
    assert_eq!(ccd.soft_ccd_prediction, ZERO);
    assert_eq!(FAST_BODY_SAFETY_FACTOR, HALF);
    assert_eq!(RigidBodyCcdTrait::FAST_BODY_SAFETY_FACTOR, HALF);
    // A body without swept collider is never fast.
    assert!(!ccd.is_moving_fast(ONE, vels(ratio(1000, 1), ZERO, ZERO), None, ONE));
}

#[test]
fn test_max_point_velocity() {
    // (linvel x, linvel y, angvel, max_extent, expected)
    let cases = array![
        (ratio(3, 1), ratio(4, 1), ZERO, ONE, ratio(5, 1)), (ZERO, ZERO, -TWO, HALF, ONE),
        (ratio(3, 1), ratio(-4, 1), TWO, TWO, ratio(9, 1)), (ZERO, ZERO, ratio(100, 1), ZERO, ZERO),
    ];
    for (x, y, w, extent, expected) in cases {
        assert_eq!(thick(ONE).max_point_velocity(vels(x, y, w), extent), expected);
    }
}

/// `(speed, forces force x, thickness, fast)` at `dt = 1/8` (exact), no rotation: fast iff
/// `(speed + force · dt) · dt > thickness / 2`.
#[test]
fn test_is_moving_fast_threshold() {
    let dt = ratio(1, 8);
    let cases = array![
        (ratio(4, 1), None, ONE, false), // 0.5 == 0.5: not strictly above
        (ratio(41, 10), None, ONE, true), (ratio(39, 10), None, ONE, false),
        (ZERO, Some(ratio(41, 1)), ONE, true), // force · dt = 5.125 m/s
        (ratio(4, 1), Some(-ratio(10, 1)), ONE, false), (ONE, None, ratio(1, 10), true),
    ];
    for (speed, force, thickness, fast) in cases {
        let forces = match force {
            Some(fx) => Some(
                RigidBodyForces { force: Vec2 { x: fx, y: ZERO }, ..Default::default() },
            ),
            None => None,
        };
        assert_eq!(thick(thickness).is_moving_fast(dt, vels(speed, ZERO, ZERO), forces, ONE), fast);
    }
}

/// The post-solve test takes the larger of the actual motion and the velocity bound.
#[test]
fn test_is_moving_fast_with_next_position() {
    let dt = ratio(1, 10);
    let quarter = Rot2 { re: ZERO, im: ONE };
    // (next x, next rotation, speed, fast) for a thickness of 1 and a max extent of 1
    let cases = array![
        (ratio(6, 10), IDENTITY, ZERO, true), // moved 0.6 > 0.5
        (ratio(5, 10), IDENTITY, ZERO, false), // moved 0.5, not above
        (ZERO, quarter, ZERO, true), // |sin(pi/2)| · 1 = 1
        (ZERO, IDENTITY, ratio(6, 1), true), // velocity bound 0.6
        (ratio(1, 10), IDENTITY, ratio(1, 1), false),
    ];
    for (x, rotation, speed, fast) in cases {
        let pos = RigidBodyPosition {
            position: at(ZERO, ZERO, IDENTITY), next_position: at(x, ZERO, rotation),
        };
        assert_eq!(
            thick(ONE)
                .is_moving_fast_with_next_position(
                    dt, vels(speed, ZERO, ZERO), pos, Vec2 { x: ZERO, y: ZERO }, ONE,
                ),
            fast,
        );
    }
}

#[test]
fn gas_baseline() {}

#[test]
fn gas_is_moving_fast() {
    let ccd = thick(opaque(HALF));
    let _ = ccd
        .is_moving_fast(
            opaque(ratio(1, 60)), vels(opaque(ratio(18, 1)), opaque(ZERO), opaque(ONE)), None, ONE,
        );
}

#[test]
fn gas_is_moving_fast_with_forces() {
    let ccd = thick(opaque(HALF));
    let forces = RigidBodyForces {
        force: Vec2 { x: opaque(ONE), y: opaque(-ONE) }, ..Default::default(),
    };
    let _ = ccd
        .is_moving_fast(
            opaque(ratio(1, 60)),
            vels(opaque(ratio(18, 1)), opaque(ZERO), opaque(ONE)),
            Some(forces),
            ONE,
        );
}

#[test]
fn gas_is_moving_fast_with_next_position() {
    let ccd = thick(opaque(HALF));
    let pos = RigidBodyPosition {
        position: at(opaque(ZERO), opaque(ZERO), IDENTITY),
        next_position: at(opaque(ratio(3, 10)), opaque(ZERO), IDENTITY),
    };
    let _ = ccd
        .is_moving_fast_with_next_position(
            opaque(ratio(1, 60)),
            vels(opaque(ratio(18, 1)), opaque(ZERO), opaque(ONE)),
            pos,
            Vec2 { x: opaque(ZERO), y: opaque(ZERO) },
            opaque(ONE),
        );
}
