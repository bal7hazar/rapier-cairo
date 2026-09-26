//! Rejected candidates of `super`, kept for the `gas_*` ranking of `super::tests`.

use fixed::trig::TrigTrait;
use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::{Rot2, Rot2Trait};
use super::NonlinearRigidMotion;

/// `position_at_time` with a plain `if` on a zero angular velocity: Sierra gas charges the
/// rotation branch to every call, so a translating motion pays for the `sin_cos` too (the winner
/// meters it behind a one-iteration `while`).
pub fn position_at_time_branch(motion: NonlinearRigidMotion, t: Fixed) -> Pose2 {
    let start = motion.start;
    let shift = Vec2 { x: motion.linvel.x * t, y: motion.linvel.y * t };
    if motion.angvel == ZERO {
        return Pose2 { translation: start.translation + shift, rotation: start.rotation };
    }
    let center = start.transform_point(motion.local_center);
    let (sin, cos) = (motion.angvel * t).sin_cos();
    let rot = Rot2 { re: cos, im: sin };
    Pose2 {
        translation: center + shift + rot.rotate(start.translation - center),
        rotation: rot.mul(start.rotation),
    }
}
