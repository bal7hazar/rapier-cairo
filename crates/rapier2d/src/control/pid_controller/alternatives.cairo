//! Rejected formulations of `super` (see its candidates): upstream's multiplication by a `0 / 1`
//! axis mask, and the PD's linear correction through the full rigid-body correction.

use fixed::{Fixed, ONE, ZERO};
use glam_core::vec2::Vec2;
use rapier_core::rigid_body::AxesMaskTrait;
use rapier_core::rigid_body::axes_mask::{ANG_Z, LIN_X, LIN_Y};
use rapier_dynamics2d::rigid_body::velocity::RigidBodyVelocity;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodyTrait};
use rapier_math::pose2::{IDENTITY as POSE_IDENTITY, Pose2Trait};
use super::{PdController, PdControllerTrait, PdErrors};

/// `1` when `on`, `0` otherwise (upstream `contains(..) as u32 as Real`).
#[inline(always)]
fn bit(on: bool) -> Fixed {
    if on {
        ONE
    } else {
        ZERO
    }
}

/// Upstream's `correction`: the unmasked sums multiplied by the `0 / 1` mask.
pub fn correction_mask_mul(
    pd: PdController, pose_errors: PdErrors, vel_errors: PdErrors,
) -> RigidBodyVelocity {
    let mx = bit(pd.axes.contains(LIN_X));
    let my = bit(pd.axes.contains(LIN_Y));
    let ma = bit(pd.axes.contains(ANG_Z));
    let pl = pose_errors.linear;
    let vl = vel_errors.linear;
    RigidBodyVelocity {
        linvel: Vec2 {
            x: (pl.x * pd.lin_kp.x + vl.x * pd.lin_kd.x) * mx,
            y: (pl.y * pd.lin_kp.y + vl.y * pd.lin_kd.y) * my,
        },
        angvel: (pose_errors.angular * pd.ang_kp + vel_errors.angular * pd.ang_kd) * ma,
    }
}

/// Upstream's `linear_rigid_body_correction`: the full correction (both pose-error halves, one
/// `atan2`), linear part kept.
pub fn linear_rigid_body_correction_full(
    pd: PdController, rb: @RigidBody, target_pos: Vec2, target_linvel: Vec2,
) -> Vec2 {
    let target = Pose2Trait::new(target_pos, POSE_IDENTITY.rotation);
    let vels = RigidBodyVelocity { linvel: target_linvel, angvel: rb.angvel() };
    pd.rigid_body_correction(rb, target, vels).linvel
}
