//! Tests and `gas_*` probes of the PD / PID controllers. Golden comparisons against rapier2d-f64
//! live in `tests/control_golden.cairo`.

use fixed::{FRAC_PI_2, Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
use glam::vec2::Vec2;
use rapier_core::rigid_body::axes_mask::{ANG_Z, LIN_X, LIN_Y};
use rapier_core::rigid_body::{AxesMask, AxesMaskTrait};
use rapier_dynamics2d::rigid_body::velocity::RigidBodyVelocity;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodyBuilderTrait};
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use super::alternatives::{correction_mask_mul, linear_rigid_body_correction_full};
use super::{
    DEFAULT_KD, DEFAULT_KP, PdController, PdControllerTrait, PdErrors, PidController,
    PidControllerTrait,
};

fn int(x: i32) -> Fixed {
    FixedTrait::from_int(x)
}

fn ratio(n: i64, d: i64) -> Fixed {
    FixedTrait::from_ratio(n, d)
}

fn v(x: Fixed, y: Fixed) -> Vec2 {
    Vec2 { x, y }
}

/// cos / sin of 30 degrees (the golden harness' exact pair).
fn rot30() -> Rot2 {
    Rot2 { re: Fixed { raw: 3719550787 }, im: HALF }
}

/// A dynamic body at `(1, 2)` turned by 30 degrees, moving at `(0.5, -0.25)` and `0.75` rad/s,
/// its centre of mass at `(0.25, 0.5)` in its frame.
fn body() -> RigidBody {
    let mut rb = RigidBodyBuilderTrait::dynamic()
        .position(Pose2Trait::new(v(ONE, TWO), rot30()))
        .linvel(v(HALF, ratio(-1, 4)))
        .angvel(ratio(3, 4))
        .build();
    rb.mprops.local_mprops.local_com = v(ratio(1, 4), HALF);
    rb
}

fn errors(lx: Fixed, ly: Fixed, a: Fixed) -> PdErrors {
    PdErrors { linear: v(lx, ly), angular: a }
}

fn pd(kp: Fixed, kd: Fixed, bits: u8) -> PdController {
    PdControllerTrait::new(kp, kd, AxesMaskTrait::from_bits(bits).unwrap())
}

#[test]
fn test_defaults() {
    let pd: PdController = Default::default();
    assert_eq!(pd, PdControllerTrait::new(DEFAULT_KP, DEFAULT_KD, AxesMaskTrait::all()));
    assert_eq!(pd.lin_kp, v(int(60), int(60)));
    let pid: PidController = Default::default();
    assert_eq!(pid.pd, pd);
    assert_eq!((pid.lin_ki, pid.ang_ki), (v(ONE, ONE), ONE));
    assert_eq!((pid.lin_integral, pid.ang_integral), (v(ZERO, ZERO), ZERO));
    assert_eq!(pid.axes(), AxesMaskTrait::all());
}

/// `correction` against closed forms, one row per axis mask: `(bits, linvel, angvel)` for pose
/// errors `(2, -1, 0.5)`, velocity errors `(0.5, 1, -1)`, kp 2 and kd 0.5.
#[test]
fn test_correction_masks() {
    let cases: Array<(u8, Vec2, Fixed)> = array![
        (35, v(ratio(17, 4), ratio(-3, 2)), HALF), (0, v(ZERO, ZERO), ZERO),
        (1, v(ratio(17, 4), ZERO), ZERO), (2, v(ZERO, ratio(-3, 2)), ZERO),
        (32, v(ZERO, ZERO), HALF), (33, v(ratio(17, 4), ZERO), HALF),
    ];
    let pose = errors(TWO, -ONE, HALF);
    let vel = errors(HALF, ONE, -ONE);
    for (bits, linvel, angvel) in cases {
        let c = pd(TWO, HALF, bits);
        let got = c.correction(pose, vel);
        assert_eq!(got, RigidBodyVelocity { linvel, angvel });
        assert_eq!(correction_mask_mul(c, pose, vel), got);
    }
}

/// The body-level corrections: the linear / angular shortcuts equal the halves of the full
/// correction, and the full correction equals `correction` of the hand-computed errors.
#[test]
fn test_rigid_body_corrections() {
    let rb = body();
    let c = pd(int(3), HALF, 35);
    let targets: Array<(Vec2, Rot2, Vec2, Fixed)> = array![
        (v(int(2), ONE), Rot2 { re: ONE, im: ZERO }, v(ONE, ZERO), ZERO),
        (v(-ONE, int(4)), Rot2 { re: ZERO, im: ONE }, v(ZERO, -ONE), TWO),
        (v(ONE, TWO), rot30(), v(HALF, ratio(-1, 4)), ratio(3, 4)),
    ];
    for (tpos, trot, tlin, tang) in targets {
        let lin = c.linear_rigid_body_correction(@rb, tpos, tlin);
        assert_eq!(lin, linear_rigid_body_correction_full(c, @rb, tpos, tlin));
        let full = c
            .rigid_body_correction(
                @rb,
                Pose2Trait::new(v(ZERO, ZERO), trot),
                RigidBodyVelocity { linvel: rb.vels.linvel, angvel: tang },
            );
        assert_eq!(c.angular_rigid_body_correction(@rb, trot, tang), full.angvel);
    }
    // At the target: no correction.
    let still = c.rigid_body_correction(@rb, Pose2Trait::new(v(ONE, TWO), rot30()), rb.vels);
    assert_eq!(still, RigidBodyVelocity { linvel: v(ZERO, ZERO), angvel: ZERO });
    // A quarter turn ahead, same COM-shifted translation: angular error pi/2 (within 4 ulps).
    let turn = c
        .angular_rigid_body_correction(
            @rb, Rot2 { re: -HALF, im: Fixed { raw: 3719550787 } }, rb.vels.angvel,
        );
    assert!((turn - FRAC_PI_2 * int(3)).abs() <= Fixed { raw: 64 }, "turn {:?}", turn);
}

/// The PID integrates `pose * dt` before correcting; `reset_integrals` and `set_axes` touch only
/// their fields.
#[test]
fn test_pid_integrals() {
    let mut pid = PidControllerTrait::new(TWO, HALF, ONE, AxesMaskTrait::all());
    let pose = errors(TWO, -ONE, HALF);
    let vel = errors(ZERO, ZERO, ZERO);
    let dt = ratio(1, 4);
    let first = pid.correction(dt, pose, vel);
    // integral (0.5, -0.25, 0.125) * ki 0.5 added to pose * kp 2.
    assert_eq!(first.linvel, v(ratio(17, 4), ratio(-17, 8)));
    assert_eq!(first.angvel, ratio(17, 16));
    let second = pid.correction(dt, pose, vel);
    assert_eq!(pid.lin_integral, v(ONE, -HALF));
    assert_eq!(second.linvel, v(ratio(9, 2), ratio(-9, 4)));
    pid.set_axes(LIN_Y);
    assert_eq!(pid.axes(), LIN_Y);
    let masked = pid.correction(dt, pose, vel);
    assert_eq!((masked.linvel.x, masked.angvel), (ZERO, ZERO));
    pid.reset_integrals();
    assert_eq!((pid.lin_integral, pid.ang_integral), (v(ZERO, ZERO), ZERO));
    assert_eq!(pid.pd.lin_kp, v(TWO, TWO));
    // The body-level PID updates both integrals even for a linear correction.
    let rb = body();
    let mut pid = PidControllerTrait::new(TWO, ONE, HALF, AxesMaskTrait::all());
    let _ = pid.linear_rigid_body_correction(dt, @rb, v(int(3), TWO), v(ZERO, ZERO));
    assert!(pid.ang_integral != ZERO);
    let mut pid2 = PidControllerTrait::new(TWO, ONE, HALF, AxesMaskTrait::all());
    let full = pid2
        .rigid_body_correction(
            dt,
            @rb,
            Pose2Trait::new(v(int(3), TWO), Rot2 { re: ONE, im: ZERO }),
            RigidBodyVelocity { linvel: v(ZERO, ZERO), angvel: rb.vels.angvel },
        );
    assert_eq!(pid, pid2);
    let mut pid3 = PidControllerTrait::new(TWO, ONE, HALF, AxesMaskTrait::all());
    let lin = pid3.linear_rigid_body_correction(dt, @rb, v(int(3), TWO), v(ZERO, ZERO));
    assert_eq!(lin, full.linvel);
    let ang = pid3.angular_rigid_body_correction(dt, @rb, rot30(), ZERO);
    assert!(ang != ZERO || pid3.ang_integral == ZERO);
}

#[test]
fn test_masks_are_upstream_bits() {
    assert_eq!((LIN_X.bits, LIN_Y.bits, ANG_Z.bits), (1, 2, 32));
    let none: AxesMask = AxesMaskTrait::empty();
    let c = PdControllerTrait::new(TWO, TWO, none);
    let rb = body();
    assert_eq!(c.linear_rigid_body_correction(@rb, v(int(9), int(9)), v(ONE, ONE)), v(ZERO, ZERO));
}

// ---------------------------------------------------------------------------------------------
// Gas probes (inputs through `opaque`; subtract `gas_baseline`).

#[test]
fn gas_baseline() {
    let _ = opaque(body());
}

#[test]
fn gas_pd_correction() {
    let c = opaque(pd(TWO, HALF, 35));
    let _ = opaque(c.correction(opaque(errors(TWO, -ONE, HALF)), opaque(errors(HALF, ONE, -ONE))));
    let _ = opaque(body());
}

#[test]
fn gas_pd_correction_mask_mul() {
    let c = opaque(pd(TWO, HALF, 35));
    let _ = opaque(
        correction_mask_mul(c, opaque(errors(TWO, -ONE, HALF)), opaque(errors(HALF, ONE, -ONE))),
    );
    let _ = opaque(body());
}

#[test]
fn gas_pd_linear_rigid_body_correction() {
    let rb = opaque(body());
    let c = opaque(pd(int(3), HALF, 35));
    let _ = opaque(
        c.linear_rigid_body_correction(@rb, opaque(v(int(2), ONE)), opaque(v(ONE, ZERO))),
    );
}

#[test]
fn gas_pd_linear_rigid_body_correction_full() {
    let rb = opaque(body());
    let c = opaque(pd(int(3), HALF, 35));
    let _ = opaque(
        linear_rigid_body_correction_full(c, @rb, opaque(v(int(2), ONE)), opaque(v(ONE, ZERO))),
    );
}

#[test]
fn gas_pd_angular_rigid_body_correction() {
    let rb = opaque(body());
    let c = opaque(pd(int(3), HALF, 35));
    let _ = opaque(
        c.angular_rigid_body_correction(@rb, opaque(Rot2 { re: ZERO, im: ONE }), opaque(TWO)),
    );
}

#[test]
fn gas_pd_rigid_body_correction() {
    let rb = opaque(body());
    let c = opaque(pd(int(3), HALF, 35));
    let target = opaque(Pose2Trait::new(v(int(2), ONE), Rot2 { re: ZERO, im: ONE }));
    let vels = opaque(RigidBodyVelocity { linvel: v(ONE, ZERO), angvel: TWO });
    let _ = opaque(c.rigid_body_correction(@rb, target, vels));
}

#[test]
fn gas_pid_correction() {
    let _ = opaque(body());
    let mut pid = opaque(PidControllerTrait::new(TWO, HALF, ONE, AxesMaskTrait::all()));
    let _ = opaque(
        pid
            .correction(
                opaque(ratio(1, 60)),
                opaque(errors(TWO, -ONE, HALF)),
                opaque(errors(HALF, ONE, -ONE)),
            ),
    );
    let _ = opaque(pid);
}

#[test]
fn gas_pid_rigid_body_correction() {
    let rb = opaque(body());
    let mut pid = opaque(PidControllerTrait::new(TWO, HALF, ONE, AxesMaskTrait::all()));
    let target: Pose2 = opaque(Pose2Trait::new(v(int(2), ONE), Rot2 { re: ZERO, im: ONE }));
    let vels = opaque(RigidBodyVelocity { linvel: v(ONE, ZERO), angvel: TWO });
    let _ = opaque(pid.rigid_body_correction(opaque(ratio(1, 60)), @rb, target, vels));
    let _ = opaque(pid);
}
