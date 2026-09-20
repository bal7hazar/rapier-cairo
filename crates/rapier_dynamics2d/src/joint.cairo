//! Phase-1 impulse joints. Limits, motors and coupling are stored but not solved.
//! Local frames are body-local; the solver adapter requires centre-of-mass-local frames.
mod builders;
mod set;
pub use builders::{
    FixedJointBuilder, FixedJointBuilderTrait, GenericJointBuilder, GenericJointBuilderTrait,
    PrismaticJointBuilder, PrismaticJointBuilderTrait, RevoluteJointBuilder,
    RevoluteJointBuilderTrait,
};
use core::num::traits::DivRem;
use fixed::{Fixed, MAX, MIN, ZERO};
use rapier_core::integration_parameters::spring::{JOINT_DEFAULTS, SpringCoefficients};
use rapier_math::pose2::Pose2;
pub use set::{ImpulseJoint, ImpulseJointSet, ImpulseJointSetTrait};

/// 2D axis bits: X=1, Y=2, AngX=4. Only values 0..7 are valid.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)]
pub struct JointAxesMask {
    pub bits: u8,
}
/// Linear X axis.
pub const LIN_X: JointAxesMask = JointAxesMask { bits: 1 };
/// Linear Y axis.
pub const LIN_Y: JointAxesMask = JointAxesMask { bits: 2 };
/// Angular axis (upstream calls it AngX in 2D).
pub const ANG_X: JointAxesMask = JointAxesMask { bits: 4 };
/// Revolute locks translation only.
pub const LOCKED_REVOLUTE_AXES: JointAxesMask = JointAxesMask { bits: 3 };
/// Prismatic leaves local X free.
pub const LOCKED_PRISMATIC_AXES: JointAxesMask = JointAxesMask { bits: 6 };
/// Fixed locks all three axes.
pub const LOCKED_FIXED_AXES: JointAxesMask = JointAxesMask { bits: 7 };
/// Invalid construction inputs.
pub mod errors {
    pub const AXIS: felt252 = 'Joint: invalid axis';
    pub const MASK: felt252 = 'Joint: invalid mask';
    pub const UNIT: felt252 = 'Joint: nonunit axis';
}
#[generate_trait]
pub impl JointAxesMaskImpl of JointAxesMaskTrait {
    /// Tests axis 0, 1 or 2 without bitwise builtins; invalid indices panic.
    fn contains_axis(self: JointAxesMask, axis: u8) -> bool {
        assert(self.bits <= 7, errors::MASK);
        let n = match axis {
            0 => self.bits,
            1 => {
                let (q, _) = DivRem::div_rem(self.bits, 2);
                q
            },
            2 => {
                let (q, _) = DivRem::div_rem(self.bits, 4);
                q
            },
            _ => core::panic_with_felt252(errors::AXIS),
        };
        let (_, r) = DivRem::div_rem(n, 2);
        r != 0
    }
}
/// Inactive limit data. Finite Fixed extrema stand in for upstream's floating extrema.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct JointLimits {
    pub min: Fixed,
    pub max: Fixed,
    pub impulse: Fixed,
}
pub impl JointLimitsDefault of Default<JointLimits> {
    fn default() -> JointLimits {
        JointLimits { min: MIN, max: MAX, impulse: ZERO }
    }
}
/// Motor model retained for future motor rows.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum MotorModel {
    AccelerationBased,
    ForceBased,
}
/// Inactive motor data; units follow the corresponding linear/angular axis.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct JointMotor {
    pub target_vel: Fixed,
    pub target_pos: Fixed,
    pub stiffness: Fixed,
    pub damping: Fixed,
    pub max_force: Fixed,
    pub impulse: Fixed,
    pub model: MotorModel,
}
pub impl JointMotorDefault of Default<JointMotor> {
    fn default() -> JointMotor {
        JointMotor {
            target_vel: ZERO,
            target_pos: ZERO,
            stiffness: ZERO,
            damping: ZERO,
            max_force: MAX,
            impulse: ZERO,
            model: MotorModel::AccelerationBased,
        }
    }
}
/// Explicit/user and attached-body disable states, matching upstream.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum JointEnabled {
    Enabled,
    DisabledByAttachedBody,
    Disabled,
}
/// Joint frames, lock masks and reserved limit/motor state. Rotations must be unit;
/// positions and numeric intermediates must fit Q32.32. Softness is evaluated per substep.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct GenericJoint {
    pub local_frame1: Pose2,
    pub local_frame2: Pose2,
    pub locked_axes: JointAxesMask,
    pub coupled_axes: JointAxesMask,
    pub limit_axes: JointAxesMask,
    pub motor_axes: JointAxesMask,
    pub limits: [JointLimits; 3],
    pub motors: [JointMotor; 3],
    pub softness: SpringCoefficients,
    pub contacts_enabled: bool,
    pub enabled: JointEnabled,
}
pub impl GenericJointDefault of Default<GenericJoint> {
    fn default() -> GenericJoint {
        let l = Default::default();
        let m = Default::default();
        GenericJoint {
            local_frame1: Default::default(),
            local_frame2: Default::default(),
            locked_axes: Default::default(),
            coupled_axes: Default::default(),
            limit_axes: Default::default(),
            motor_axes: Default::default(),
            limits: [l, l, l],
            motors: [m, m, m],
            softness: JOINT_DEFAULTS,
            contacts_enabled: true,
            enabled: JointEnabled::Enabled,
        }
    }
}
#[cfg(test)]
mod tests {
    use rapier_testing::opaque;
    use super::*;
    #[test]
    fn test_masks_and_defaults() {
        let mut bits = 0;
        while bits != 8 {
            let mask = JointAxesMask { bits };
            assert_eq!(mask.contains_axis(0), bits == 1 || bits == 3 || bits == 5 || bits == 7);
            assert_eq!(mask.contains_axis(1), bits == 2 || bits == 3 || bits == 6 || bits == 7);
            assert_eq!(mask.contains_axis(2), bits >= 4);
            bits += 1;
        }
        let j: GenericJoint = Default::default();
        assert!(j.contacts_enabled);
        assert_eq!(j.enabled, JointEnabled::Enabled);
        let [l, _, _] = j.limits;
        assert_eq!(l.min, MIN);
        assert_eq!(l.max, MAX);
        let [m, _, _] = j.motors;
        assert_eq!(m.max_force, MAX);
    }
    #[test]
    #[should_panic(expected: 'Joint: invalid axis')]
    fn test_invalid_axis() {
        let _ = LIN_X.contains_axis(3);
    }
    #[test]
    #[should_panic(expected: 'Joint: invalid mask')]
    fn test_invalid_mask() {
        let _ = JointAxesMask { bits: 8 }.contains_axis(0);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(ZERO);
    }
    #[test]
    fn gas_contains_axis() {
        let _ = opaque(LOCKED_FIXED_AXES).contains_axis(opaque(1));
    }
    #[test]
    fn gas_defaults() {
        let _ = opaque(Default::<GenericJoint>::default());
    }
}
