//! PD and PID controllers (upstream `control/pid_controller.rs`), 2D: velocity corrections that
//! drive a rigid-body towards a target pose and velocity. Pure arithmetic on the body's pose and
//! velocities: nothing here touches a world or the step.
//!
//! Every correction is `(pose_error * kp + vel_error * kd [+ integral * ki]) * mask` per axis, the
//! mask being 1 or 0 as the axis is in [`PdController::axes`]. Products floor (Q32.32); the pose
//! errors are `rapier_dynamics2d`'s `RigidBodyPosition::pose_errors` (the angle through the fixed
//! `atan2`), which is upstream's COM-shifted `pose_errors` simplified algebraically.
//!
//! # Candidates
//!
//! Ranked by the `gas_*` probes of `tests` (Sierra gas net of `gas_baseline`, Cairo steps in
//! parentheses); the losers live in `alternatives`.
//!
//! * The axis mask: **select** (an axis outside the mask gives zero, inside it the unmasked value,
//!   bit-identical to a multiplication by `1.0`): `correction` 24,619 (204), against upstream's
//!   multiplication by a `0 / 1` mask (`alternatives::correction_mask_mul`): 29,839 (240).
//! * [`PdControllerTrait::linear_rigid_body_correction`] computes only the linear half of the pose
//!   error (same result): 39,556 (324), against upstream's path through the full
//!   [`PdControllerTrait::rigid_body_correction`]
//!   (`alternatives::linear_rigid_body_correction_full`), which pays the `atan2` of the angle:
//!   82,689 (653). The angular one likewise skips the linear half: 50,083 (396). The PID keeps the
//!   full path: its integrals accumulate both halves.
//!
//! Other probes: `rigid_body_correction` 83,489 (665); `PidController::correction` 47,689 (406),
//! `PidController::rigid_body_correction` 107,259 (874).

use fixed::{Fixed, ONE, TrigTrait, ZERO};
use glam::vec2::Vec2;
use rapier_core::rigid_body::axes_mask::{ANG_Z, LIN_X, LIN_Y};
use rapier_core::rigid_body::{AxesMask, AxesMaskTrait};
use rapier_dynamics2d::rigid_body::position::{RigidBodyPosition, RigidBodyPositionTrait};
use rapier_dynamics2d::rigid_body::velocity::RigidBodyVelocity;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodyTrait};
use rapier_math::pose2::{IDENTITY as POSE_IDENTITY, Pose2, Pose2Trait};
use rapier_math::rot2::{Rot2, Rot2Trait};

#[cfg(test)]
mod alternatives;
#[cfg(test)]
mod tests;

/// `0.8`, upstream's default derivative gain (nearest Q32.32).
pub const DEFAULT_KD: Fixed = Fixed { raw: 3435973837 };
/// `60`, upstream's default proportional gain.
pub const DEFAULT_KP: Fixed = Fixed { raw: 257698037760 };

/// A Proportional-Derivative controller (upstream `PdController`): corrects a rigid-body at the
/// velocity level so that it matches a target pose. Default: `new(60, 0.8, AxesMask::all())`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PdController {
    /// Proportional gain of the linear position errors (usually a multiple of `1 / dt`).
    pub lin_kp: Vec2,
    /// Derivative gain of the linear velocity errors (usually in `[0, 1]`).
    pub lin_kd: Vec2,
    /// Proportional gain of the angular position error.
    pub ang_kp: Fixed,
    /// Derivative gain of the angular velocity error.
    pub ang_kd: Fixed,
    /// The axes the controller acts on: the others get a zero correction.
    pub axes: AxesMask,
}

/// A Proportional-Integral-Derivative controller (upstream `PidController`): the [`PdController`]
/// plus the accumulated position errors. Default: `new(60, 1, 0.8, AxesMask::all())`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PidController {
    /// The Proportional-Derivative part.
    pub pd: PdController,
    /// The linear error accumulated through time (the integral term).
    pub lin_integral: Vec2,
    /// The angular error accumulated through time.
    pub ang_integral: Fixed,
    /// Gain of the linear integral term.
    pub lin_ki: Vec2,
    /// Gain of the angular integral term.
    pub ang_ki: Fixed,
}

/// Position or velocity errors measured for PID control (upstream `PdErrors`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PdErrors {
    /// The linear (translational) part of the error.
    pub linear: Vec2,
    /// The angular (rotational) part of the error, radians.
    pub angular: Fixed,
}

/// Upstream `impl From<RigidBodyVelocity> for PdErrors`: the linear and angular velocities.
pub impl RigidBodyVelocityIntoPdErrors of Into<RigidBodyVelocity, PdErrors> {
    #[inline(always)]
    fn into(self: RigidBodyVelocity) -> PdErrors {
        PdErrors { linear: self.linvel, angular: self.angvel }
    }
}

pub impl PdControllerDefault of Default<PdController> {
    #[inline(always)]
    fn default() -> PdController {
        PdControllerTrait::new(DEFAULT_KP, DEFAULT_KD, AxesMaskTrait::all())
    }
}

pub impl PidControllerDefault of Default<PidController> {
    #[inline(always)]
    fn default() -> PidController {
        PidControllerTrait::new(DEFAULT_KP, ONE, DEFAULT_KD, AxesMaskTrait::all())
    }
}

/// `value` when `on`, zero otherwise: the product by upstream's `0 / 1` mask, exactly.
#[inline(always)]
fn masked(value: Fixed, on: bool) -> Fixed {
    if on {
        value
    } else {
        ZERO
    }
}

/// The COM-shifted pose errors of `rb` towards `target` (upstream `RigidBodyPosition {
/// position: rb.pos.position, next_position: target }.pose_errors(rb.local_center_of_mass())`).
#[inline(always)]
fn body_pose_errors(rb: @RigidBody, target: Pose2) -> PdErrors {
    let errors = RigidBodyPosition { position: rb.position(), next_position: target }
        .pose_errors(rb.local_center_of_mass());
    errors.into()
}

/// The velocity errors `target - rb.vels`.
#[inline(always)]
fn body_vel_errors(rb: @RigidBody, target: RigidBodyVelocity) -> PdErrors {
    let vels = *rb.vels;
    PdErrors { linear: target.linvel - vels.linvel, angular: target.angvel - vels.angvel }
}

#[generate_trait]
pub impl PdControllerImpl of PdControllerTrait {
    /// The controller with the gains `kp` / `kd` on every axis, acting on `axes` only (the gains
    /// are set on all axes regardless).
    #[inline(always)]
    fn new(kp: Fixed, kd: Fixed, axes: AxesMask) -> PdController {
        PdController {
            lin_kp: Vec2 { x: kp, y: kp },
            lin_kd: Vec2 { x: kd, y: kd },
            ang_kp: kp,
            ang_kd: kd,
            axes,
        }
    }

    /// The linear correction (a velocity change with the usual gains) of `rb` towards the
    /// position `target_pos` and velocity `target_linvel`. Same value as
    /// `rigid_body_correction(rb, from_translation(target_pos), (target_linvel,
    /// rb.angvel)).linvel`;
    /// only the linear pose error is computed.
    ///
    /// # Panics
    /// * `'Fixed: overflow'` when a product leaves the Q32.32 range.
    fn linear_rigid_body_correction(
        self: PdController, rb: @RigidBody, target_pos: Vec2, target_linvel: Vec2,
    ) -> Vec2 {
        let pos = rb.position();
        let com = rb.local_center_of_mass();
        let target = Pose2Trait::new(target_pos, POSE_IDENTITY.rotation);
        let pose = target.transform_point(com) - pos.transform_point(com);
        let vel = target_linvel - *rb.vels.linvel;
        let (on_x, on_y) = (self.axes.contains(LIN_X), self.axes.contains(LIN_Y));
        Vec2 {
            x: masked(pose.x * self.lin_kp.x + vel.x * self.lin_kd.x, on_x),
            y: masked(pose.y * self.lin_kp.y + vel.y * self.lin_kd.y, on_y),
        }
    }

    /// The angular correction of `rb` towards the rotation `target_rot` and angular velocity
    /// `target_angvel` (only the angular pose error is computed: one `atan2`).
    ///
    /// # Panics
    /// * `'Fixed: overflow'` when a product leaves the Q32.32 range.
    fn angular_rigid_body_correction(
        self: PdController, rb: @RigidBody, target_rot: Rot2, target_angvel: Fixed,
    ) -> Fixed {
        let dr = target_rot * rb.position().rotation.inverse();
        let pose = dr.im.atan2(dr.re);
        let vel = target_angvel - *rb.vels.angvel;
        masked(pose * self.ang_kp + vel * self.ang_kd, self.axes.contains(ANG_Z))
    }

    /// The linear and angular correction of `rb` towards `target_pose` and `target_vels`
    /// ([`Self::correction`] of the body's pose and velocity errors).
    fn rigid_body_correction(
        self: PdController, rb: @RigidBody, target_pose: Pose2, target_vels: RigidBodyVelocity,
    ) -> RigidBodyVelocity {
        self.correction(body_pose_errors(rb, target_pose), body_vel_errors(rb, target_vels))
    }

    /// The correction of the given pose and velocity errors: `(pose * kp + vel * kd) * mask` per
    /// axis, products floored.
    fn correction(
        self: PdController, pose_errors: PdErrors, vel_errors: PdErrors,
    ) -> RigidBodyVelocity {
        let (on_x, on_y) = (self.axes.contains(LIN_X), self.axes.contains(LIN_Y));
        let pl = pose_errors.linear;
        let vl = vel_errors.linear;
        RigidBodyVelocity {
            linvel: Vec2 {
                x: masked(pl.x * self.lin_kp.x + vl.x * self.lin_kd.x, on_x),
                y: masked(pl.y * self.lin_kp.y + vl.y * self.lin_kd.y, on_y),
            },
            angvel: masked(
                pose_errors.angular * self.ang_kp + vel_errors.angular * self.ang_kd,
                self.axes.contains(ANG_Z),
            ),
        }
    }
}

#[generate_trait]
pub impl PidControllerImpl of PidControllerTrait {
    /// The controller with the gains `kp` / `ki` / `kd` on every axis, zero integrals, acting on
    /// `axes` only.
    #[inline(always)]
    fn new(kp: Fixed, ki: Fixed, kd: Fixed, axes: AxesMask) -> PidController {
        PidController {
            pd: PdControllerTrait::new(kp, kd, axes),
            lin_integral: Vec2 { x: ZERO, y: ZERO },
            ang_integral: ZERO,
            lin_ki: Vec2 { x: ki, y: ki },
            ang_ki: ki,
        }
    }

    /// Sets the axes the errors and corrections are computed for (the gains are unchanged).
    #[inline(always)]
    fn set_axes(ref self: PidController, axes: AxesMask) {
        self.pd.axes = axes;
    }

    /// The axes the errors and corrections are computed for.
    #[inline(always)]
    fn axes(self: @PidController) -> AxesMask {
        *self.pd.axes
    }

    /// Resets both accumulated errors to zero.
    #[inline(always)]
    fn reset_integrals(ref self: PidController) {
        self.lin_integral = Vec2 { x: ZERO, y: ZERO };
        self.ang_integral = ZERO;
    }

    /// The linear correction of `rb` towards `target_pos` / `target_linvel`; the integrals
    /// accumulate both halves of the pose error over `dt`, as upstream (the angular half is the
    /// error towards the identity rotation).
    fn linear_rigid_body_correction(
        ref self: PidController, dt: Fixed, rb: @RigidBody, target_pos: Vec2, target_linvel: Vec2,
    ) -> Vec2 {
        let target = Pose2Trait::new(target_pos, POSE_IDENTITY.rotation);
        let vels = RigidBodyVelocity { linvel: target_linvel, angvel: *rb.vels.angvel };
        self.rigid_body_correction(dt, rb, target, vels).linvel
    }

    /// The angular correction of `rb` towards `target_rot` / `target_angvel` (the integrals
    /// accumulate both halves, the linear one towards the origin, as upstream).
    fn angular_rigid_body_correction(
        ref self: PidController, dt: Fixed, rb: @RigidBody, target_rot: Rot2, target_angvel: Fixed,
    ) -> Fixed {
        let target = Pose2Trait::new(Vec2 { x: ZERO, y: ZERO }, target_rot);
        let vels = RigidBodyVelocity { linvel: *rb.vels.linvel, angvel: target_angvel };
        self.rigid_body_correction(dt, rb, target, vels).angvel
    }

    /// The linear and angular correction of `rb` towards `target_pose` / `target_vels`.
    fn rigid_body_correction(
        ref self: PidController,
        dt: Fixed,
        rb: @RigidBody,
        target_pose: Pose2,
        target_vels: RigidBodyVelocity,
    ) -> RigidBodyVelocity {
        self.correction(dt, body_pose_errors(rb, target_pose), body_vel_errors(rb, target_vels))
    }

    /// Accumulates `pose_errors * dt` into the integrals, then returns
    /// `(pose * kp + vel * kd + integral * ki) * mask` per axis, products floored.
    ///
    /// # Panics
    /// * `'Fixed: overflow'` when a product or an integral leaves the Q32.32 range.
    fn correction(
        ref self: PidController, dt: Fixed, pose_errors: PdErrors, vel_errors: PdErrors,
    ) -> RigidBodyVelocity {
        let pl = pose_errors.linear;
        let vl = vel_errors.linear;
        self
            .lin_integral =
                Vec2 { x: self.lin_integral.x + pl.x * dt, y: self.lin_integral.y + pl.y * dt };
        self.ang_integral = self.ang_integral + pose_errors.angular * dt;
        let pd = self.pd;
        let li = self.lin_integral;
        let (on_x, on_y) = (pd.axes.contains(LIN_X), pd.axes.contains(LIN_Y));
        RigidBodyVelocity {
            linvel: Vec2 {
                x: masked(pl.x * pd.lin_kp.x + vl.x * pd.lin_kd.x + li.x * self.lin_ki.x, on_x),
                y: masked(pl.y * pd.lin_kp.y + vl.y * pd.lin_kd.y + li.y * self.lin_ki.y, on_y),
            },
            angvel: masked(
                pose_errors.angular * pd.ang_kp
                    + vel_errors.angular * pd.ang_kd
                    + self.ang_integral * self.ang_ki,
                pd.axes.contains(ANG_Z),
            ),
        }
    }
}
