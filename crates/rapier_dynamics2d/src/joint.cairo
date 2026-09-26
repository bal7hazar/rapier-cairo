//! Impulse joints with free-axis limits and motors, and coupled linear limits/motors (rope,
//! spring; RJ).
//! Local frames are body-local; the solver adapter requires centre-of-mass-local frames.
mod builders;
mod config;
pub use config::GenericJointTrait;
#[cfg(test)]
mod builder_controls;
mod fixed_joint;
mod pin_slot_joint;
mod prismatic_joint;
mod revolute_joint;
mod rope_joint;
mod set;
mod spring_joint;
#[cfg(test)]
mod typed_tests;
pub use builders::{
    FixedJointBuilder, FixedJointBuilderTrait, GenericJointBuilder, GenericJointBuilderIntoGeneric,
    GenericJointBuilderTrait, PrismaticJointBuilder, PrismaticJointBuilderTrait,
    RevoluteJointBuilder, RevoluteJointBuilderTrait,
};
use core::num::traits::DivRem;
use fixed::{Fixed, MAX, MIN, ZERO};
pub use fixed_joint::{
    FixedJoint, FixedJointBuilderIntoGeneric, FixedJointIntoGeneric, FixedJointTrait,
};
pub use pin_slot_joint::{
    PinSlotJoint, PinSlotJointBuilder, PinSlotJointBuilderIntoGeneric, PinSlotJointBuilderTrait,
    PinSlotJointIntoGeneric, PinSlotJointTrait,
};
pub use prismatic_joint::{
    PrismaticJoint, PrismaticJointBuilderIntoGeneric, PrismaticJointIntoGeneric,
    PrismaticJointTrait,
};
use rapier_core::integration_parameters::spring::{JOINT_DEFAULTS, SpringCoefficients};
use rapier_math::pose2::Pose2;
pub use revolute_joint::{
    RevoluteJoint, RevoluteJointBuilderIntoGeneric, RevoluteJointIntoGeneric, RevoluteJointTrait,
};
pub use rope_joint::{
    RopeJoint, RopeJointBuilder, RopeJointBuilderIntoGeneric, RopeJointBuilderTrait,
    RopeJointIntoGeneric, RopeJointTrait,
};
pub use set::{ImpulseJoint, ImpulseJointSet, ImpulseJointSetTrait, ImpulseJointTrait};
pub use spring_joint::{
    SpringJoint, SpringJointBuilder, SpringJointBuilderIntoGeneric, SpringJointBuilderTrait,
    SpringJointIntoGeneric, SpringJointTrait,
};

/// 2D axis bits: X=1, Y=2, AngX=4. Only values 0..7 are valid.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct JointAxesMask {
    pub bits: u8,
}
/// The empty mask (upstream `JointAxesMask::empty()`, its `Default`).
pub impl JointAxesMaskDefault of Default<JointAxesMask> {
    #[inline(always)]
    fn default() -> JointAxesMask {
        JointAxesMask { bits: 0 }
    }
}
/// One axis of a joint (upstream `JointAxis`, 2D: `LinX`, `LinY`, `AngX`). The setters of the
/// joints take the axis index (`0`, `1`, `2`), [`JointAxisTrait::index`] gives it.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum JointAxis {
    LinX,
    LinY,
    AngX,
}
#[generate_trait]
pub impl JointAxisImpl of JointAxisTrait {
    /// Index of the axis in the `limits` / `motors` arrays: `0`, `1`, `2`.
    #[inline(always)]
    fn index(self: JointAxis) -> u8 {
        match self {
            JointAxis::LinX => 0,
            JointAxis::LinY => 1,
            JointAxis::AngX => 2,
        }
    }
    /// The single-bit mask of the axis (upstream `From<JointAxis> for JointAxesMask`).
    #[inline(always)]
    fn mask(self: JointAxis) -> JointAxesMask {
        match self {
            JointAxis::LinX => LIN_X,
            JointAxis::LinY => LIN_Y,
            JointAxis::AngX => ANG_X,
        }
    }
}
/// Upstream `From<JointAxis> for JointAxesMask`.
pub impl JointAxisIntoMask of Into<JointAxis, JointAxesMask> {
    #[inline(always)]
    fn into(self: JointAxis) -> JointAxesMask {
        self.mask()
    }
}
/// Linear X axis.
pub const LIN_X: JointAxesMask = JointAxesMask { bits: 1 };
/// Linear Y axis.
pub const LIN_Y: JointAxesMask = JointAxesMask { bits: 2 };
/// Angular axis (upstream calls it AngX in 2D).
pub const ANG_X: JointAxesMask = JointAxesMask { bits: 4 };
/// Both linear axes (upstream `JointAxesMask::LIN_AXES`), coupled by rope and spring joints.
pub const LIN_AXES: JointAxesMask = JointAxesMask { bits: 3 };
/// Revolute locks translation only.
pub const LOCKED_REVOLUTE_AXES: JointAxesMask = JointAxesMask { bits: 3 };
/// Prismatic leaves local X free.
pub const LOCKED_PRISMATIC_AXES: JointAxesMask = JointAxesMask { bits: 6 };
/// Pin-slot locks the local Y translation only (upstream `LOCKED_PIN_SLOT_AXES`, 2D).
pub const LOCKED_PIN_SLOT_AXES: JointAxesMask = JointAxesMask { bits: 2 };
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
/// Limit data. Finite Fixed extrema stand in for upstream's floating extrema.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct JointLimits {
    pub min: Fixed,
    pub max: Fixed,
    pub impulse: Fixed,
}
/// Upstream `From<[N; 2]> for JointLimits`: `[min, max]` with a zero impulse.
pub impl JointLimitsFromArray of Into<[Fixed; 2], JointLimits> {
    #[inline(always)]
    fn into(self: [Fixed; 2]) -> JointLimits {
        let [min, max] = self;
        JointLimits { min, max, impulse: ZERO }
    }
}
pub impl JointLimitsDefault of Default<JointLimits> {
    fn default() -> JointLimits {
        JointLimits { min: MIN, max: MAX, impulse: ZERO }
    }
}
/// Motor stiffness/damping interpretation.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum MotorModel {
    AccelerationBased,
    ForceBased,
}
/// Motor data; units follow the corresponding linear/angular axis.
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
/// Joint frames, lock masks and limit/motor state. Rotations must be unit;
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
