//! Kinematic interpolation uses centre-of-mass displacement and the shortest rotation.
use fixed::trig::TrigTrait;
use fixed::{Fixed, FixedTrait, HALF, ONE, PI, TWO, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::rigid_body::RigidBodyVelocity;
use super::{RigidBodyPosition, RigidBodyPositionTrait};

#[cfg(test)]
mod alternatives {
    use super::*;
    // Literal upstream shift^-1 * next * current^-1 * shift. More products and
    // cancellation than the algebraically equivalent COM displacement; floors differ.
    pub fn conjugated(p: RigidBodyPosition, inv_dt: Fixed, com: Vec2) -> RigidBodyVelocity {
        let shift = Pose2 { translation: p.position.transform_point(com), ..Default::default() };
        let error = shift.inverse() * p.next_position * p.position.inverse() * shift;
        RigidBodyVelocity {
            linvel: error.translation.mul_scalar(inv_dt),
            angvel: error.rotation.im.atan2(error.rotation.re) * inv_dt,
        }
    }
}

#[test]
fn test_interpolation_translation_rotation_and_com() {
    let p = RigidBodyPosition {
        position: Default::default(),
        next_position: Pose2 {
            translation: Vec2 { x: TWO, y: -ONE }, rotation: Rot2 { re: ZERO, im: ONE },
        },
    };
    let v = p.interpolate_velocity(TWO, Vec2 { x: ONE, y: ZERO });
    assert_eq!(v.linvel, Vec2 { x: TWO, y: ZERO });
    assert!((v.angvel - PI).abs() <= Fixed { raw: 2 });
    assert_eq!(p.interpolate_velocity(ZERO, Default::default()), Default::default());
    let same = RigidBodyPositionTrait::from_position(p.next_position);
    assert_eq!(same.interpolate_velocity(TWO, Vec2 { x: ONE, y: -ONE }), Default::default());
    let back = RigidBodyPosition { position: p.next_position, next_position: p.position };
    assert!(back.interpolate_velocity(ONE, Default::default()).angvel < ZERO);
}

#[test]
#[fuzzer(runs: 32, seed: 20260924)]
fn fuzz_com_displacement_matches_conjugation(x: i16, y: i16, angle: i16) {
    let theta = Fixed { raw: angle.into() * 65536 };
    let (s, c) = theta.sin_cos();
    let p = RigidBodyPosition {
        position: Pose2 {
            translation: Vec2 { x: HALF, y: -HALF }, rotation: Rot2 { re: c, im: s },
        },
        next_position: Pose2 {
            translation: Vec2 {
                x: Fixed { raw: x.into() * 65536 }, y: Fixed { raw: y.into() * 65536 },
            },
            ..Default::default(),
        },
    };
    let com = Vec2 { x: HALF, y: ONE };
    let a = p.interpolate_velocity(TWO, com);
    let b = alternatives::conjugated(p, TWO, com);
    assert!((a.linvel.x - b.linvel.x).abs() <= Fixed { raw: 16 });
    assert!((a.linvel.y - b.linvel.y).abs() <= Fixed { raw: 16 });
    assert_eq!(a.angvel, b.angvel);
}
fn input() -> (RigidBodyPosition, Fixed, Vec2) {
    opaque(
        (
            RigidBodyPosition {
                position: Default::default(),
                next_position: Pose2 {
                    translation: Vec2 { x: HALF, y: ONE }, rotation: Rot2 { re: ZERO, im: ONE },
                },
            },
            TWO,
            Vec2 { x: HALF, y: -HALF },
        ),
    )
}
#[test]
fn gas_baseline() {
    let _ = input();
}
#[test]
fn gas_interpolate_velocity_com() {
    let (p, inv_dt, com) = input();
    let _ = opaque(p.interpolate_velocity(inv_dt, com));
}
#[test]
fn gas_interpolate_velocity_conjugated() {
    let (p, inv_dt, com) = input();
    let _ = opaque(alternatives::conjugated(p, inv_dt, com));
}

#[test]
#[should_panic(expected: ('Fixed: overflow',))]
fn test_interpolation_velocity_overflow() {
    let p = RigidBodyPosition {
        position: Default::default(),
        next_position: Pose2 { translation: Vec2 { x: TWO, y: ZERO }, ..Default::default() },
    };
    let _ = p.interpolate_velocity(fixed::MAX, Default::default());
}
