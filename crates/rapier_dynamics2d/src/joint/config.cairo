//! Upstream limit/motor configuration. Setters copy exactly; validation occurs in the solver.
use fixed::{Fixed, ZERO};
use super::{GenericJoint, JointAxesMask, JointMotor, MotorModel, errors};

#[generate_trait]
pub impl GenericJointImpl of GenericJointTrait {
    /// Store `[min, max]` and enable limits on axis 0=X, 1=Y, 2=AngX; preserve its impulse.
    /// All Fixed values are copied exactly (even unordered limits); invalid axis/mask panics
    /// with Joint: invalid axis / Joint: invalid mask.
    fn set_limits(ref self: GenericJoint, axis: u8, limits: [Fixed; 2]) {
        enable(ref self.limit_axes, axis);
        let [min, max] = limits;
        let [mut x, mut y, mut w] = self.limits;
        match axis {
            0 => {
                x.min = min;
                x.max = max;
            },
            1 => {
                y.min = min;
                y.max = max;
            },
            _ => {
                w.min = min;
                w.max = max;
            },
        }
        self.limits = [x, y, w];
    }
    /// Store a model without enabling the motor; preserve all other state.
    /// Axis 0..2 required (Joint: invalid axis otherwise); no rounding.
    fn set_motor_model(ref self: GenericJoint, axis: u8, model: MotorModel) {
        let mut motor = read_motor(self, axis);
        motor.model = model;
        write_motor(ref self, axis, motor);
    }
    /// Enable velocity control, preserving target_pos, max_force, model and impulse.
    /// Set stiffness=0 and damping=factor; exact copies of any Fixed value, no validation.
    /// Axis 0..2/mask 0..7 required (Joint errors otherwise).
    fn set_motor_velocity(ref self: GenericJoint, axis: u8, target_vel: Fixed, factor: Fixed) {
        let target_pos = read_motor(self, axis).target_pos;
        self.set_motor(axis, target_pos, target_vel, ZERO, factor);
    }
    /// Enable position control with target_vel=0; preserve max_force, model and impulse.
    /// Exact copies of any Fixed value. Axis 0..2/mask 0..7 required (Joint errors otherwise).
    ///
    fn set_motor_position(
        ref self: GenericJoint, axis: u8, target_pos: Fixed, stiffness: Fixed, damping: Fixed,
    ) {
        self.set_motor(axis, target_pos, ZERO, stiffness, damping);
    }
    /// Enable combined control; preserve max_force, model and impulse.
    /// Exact copies of any Fixed value. Axis 0..2/mask 0..7 required (Joint errors otherwise).
    ///
    fn set_motor(
        ref self: GenericJoint,
        axis: u8,
        target_pos: Fixed,
        target_vel: Fixed,
        stiffness: Fixed,
        damping: Fixed,
    ) {
        enable(ref self.motor_axes, axis);
        let mut motor = read_motor(self, axis);
        motor.target_pos = target_pos;
        motor.target_vel = target_vel;
        motor.stiffness = stiffness;
        motor.damping = damping;
        write_motor(ref self, axis, motor);
    }
    /// Store the force cap without enabling the motor; preserve all other state.
    /// Exact copy of any Fixed value. Axis 0..2 required (Joint: invalid axis otherwise).
    ///
    fn set_motor_max_force(ref self: GenericJoint, axis: u8, max_force: Fixed) {
        let mut motor = read_motor(self, axis);
        motor.max_force = max_force;
        write_motor(ref self, axis, motor);
    }
}

fn enable(ref mask: JointAxesMask, axis: u8) {
    assert(mask.bits <= 7, errors::MASK);
    let bit: u8 = match axis {
        0 => 1,
        1 => 2,
        2 => 4,
        _ => core::panic_with_felt252(errors::AXIS),
    };
    mask.bits = mask.bits | bit;
}
fn read_motor(joint: GenericJoint, axis: u8) -> JointMotor {
    let [x, y, w] = joint.motors;
    match axis {
        0 => x,
        1 => y,
        2 => w,
        _ => core::panic_with_felt252(errors::AXIS),
    }
}
fn write_motor(ref joint: GenericJoint, axis: u8, motor: JointMotor) {
    let [mut x, mut y, mut w] = joint.motors;
    match axis {
        0 => x = motor,
        1 => y = motor,
        _ => w = motor,
    }
    joint.motors = [x, y, w];
}

#[cfg(test)]
mod alternatives {
    use super::*;
    use super::super::JointAxesMaskTrait;
    pub fn enable_arithmetic(ref mask: JointAxesMask, axis: u8) {
        if !mask.contains_axis(axis) {
            mask.bits += match axis {
                0 => 1,
                1 => 2,
                _ => 4,
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{HALF, MAX, MIN, ONE};
    use rapier_testing::opaque;
    use super::*;
    use super::super::JointAxesMaskTrait;

    #[test]
    fn test_setters_preserve_other_axes_and_impulses() {
        for axis in [0_u8, 1, 2].span() {
            let axis = *axis;
            let mut j: GenericJoint = Default::default();
            let motor = JointMotor { impulse: HALF, ..Default::default() };
            j.motors = [motor, motor, motor];
            let limit = super::super::JointLimits { impulse: -HALF, ..Default::default() };
            j.limits = [limit, limit, limit];
            let original = j;
            j.set_motor_model(axis, MotorModel::ForceBased);
            j.set_motor_max_force(axis, MAX);
            assert_eq!(j.motor_axes.bits, 0);
            j.set_motor(axis, HALF, ONE, ONE, HALF);
            let m = read_motor(j, axis);
            assert_eq!(
                m,
                JointMotor {
                    target_pos: HALF,
                    target_vel: ONE,
                    stiffness: ONE,
                    damping: HALF,
                    max_force: MAX,
                    impulse: HALF,
                    model: MotorModel::ForceBased,
                },
            );
            j.set_motor_velocity(axis, -ONE, HALF);
            assert_eq!(read_motor(j, axis), JointMotor { target_vel: -ONE, stiffness: ZERO, ..m });
            j.set_motor_position(axis, -HALF, ONE, ONE);
            assert_eq!(
                read_motor(j, axis),
                JointMotor {
                    target_pos: -HALF, target_vel: ZERO, stiffness: ONE, damping: ONE, ..m,
                },
            );
            j.set_limits(axis, [MIN, MAX]);
            j.set_limits(axis, [HALF, -HALF]); // Upstream does not reorder or validate.
            assert!(j.limit_axes.contains_axis(axis));
            assert!(j.motor_axes.contains_axis(axis));
            let expected_bit = match axis {
                0 => 1,
                1 => 2,
                _ => 4,
            };
            assert_eq!(j.limit_axes.bits, expected_bit);
            assert_eq!(j.motor_axes.bits, expected_bit);
            let mut i = 0_u8;
            for l in j.limits.span() {
                if i == axis {
                    assert_eq!(
                        *l, super::super::JointLimits { min: HALF, max: -HALF, impulse: -HALF },
                    );
                } else {
                    assert_eq!(*l, limit);
                    assert_eq!(read_motor(j, i), motor);
                }
                i += 1;
            }
            assert_eq!(j.locked_axes, original.locked_axes);
            assert_eq!(j.coupled_axes, original.coupled_axes);
            assert_eq!(j.local_frame1, original.local_frame1);
            assert_eq!(j.local_frame2, original.local_frame2);
            assert_eq!(j.softness, original.softness);
            assert_eq!(j.enabled, original.enabled);
            assert_eq!(j.contacts_enabled, original.contacts_enabled);
        }
    }
    #[test]
    fn test_enable_candidates_exhaustive_and_idempotent() {
        let mut bits = 0;
        while bits != 8 {
            for axis in [0_u8, 1, 2].span() {
                let mut a = JointAxesMask { bits };
                let mut b = a;
                enable(ref a, *axis);
                alternatives::enable_arithmetic(ref b, *axis);
                assert_eq!(a, b);
                assert!(a.contains_axis(*axis));
                enable(ref a, *axis);
                assert_eq!(a, b);
            }
            bits += 1;
        }
    }
    #[test]
    #[fuzzer(runs: 32, seed: 7)]
    fn fuzz_enable_candidates(bits: u8, axis: u8) {
        let mut a = JointAxesMask { bits: bits % 8 };
        let mut b = a;
        enable(ref a, axis % 3);
        alternatives::enable_arithmetic(ref b, axis % 3);
        assert_eq!(a, b);
    }
    #[test]
    fn test_setters_copy_extremes_without_numeric_validation() {
        let mut j: GenericJoint = Default::default();
        j.set_motor(0, MIN, MAX, MIN, MAX);
        j.set_motor_max_force(0, MIN);
        assert_eq!(
            read_motor(j, 0),
            JointMotor {
                target_pos: MIN,
                target_vel: MAX,
                stiffness: MIN,
                damping: MAX,
                max_force: MIN,
                ..Default::default(),
            },
        );
        j.set_motor_velocity(0, MIN, ZERO);
        assert_eq!(read_motor(j, 0).target_pos, MIN);
        assert_eq!(read_motor(j, 0).stiffness, ZERO);
        assert_eq!(read_motor(j, 0).damping, ZERO);
        j.set_motor_position(0, MAX, ZERO, MIN);
        assert_eq!(read_motor(j, 0).target_vel, ZERO);
        assert_eq!(read_motor(j, 0).target_pos, MAX);
        assert_eq!(read_motor(j, 0).damping, MIN);
    }
    #[test]
    #[should_panic(expected: 'Joint: invalid axis')]
    fn test_invalid_limit_axis() {
        let mut j: GenericJoint = Default::default();
        j.set_limits(3, [ZERO, ONE]);
    }
    #[test]
    #[should_panic(expected: 'Joint: invalid axis')]
    fn test_invalid_motor_axis() {
        let mut j: GenericJoint = Default::default();
        j.set_motor_max_force(3, ONE);
    }
    #[test]
    #[should_panic(expected: 'Joint: invalid mask')]
    fn test_invalid_limit_mask() {
        let mut j = GenericJoint { limit_axes: JointAxesMask { bits: 8 }, ..Default::default() };
        j.set_limits(0, [ZERO, ONE]);
    }
    #[test]
    #[should_panic(expected: 'Joint: invalid mask')]
    fn test_invalid_motor_mask() {
        let mut j = GenericJoint { motor_axes: JointAxesMask { bits: 8 }, ..Default::default() };
        j.set_motor(0, ZERO, ONE, ZERO, ONE);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }
    #[test]
    fn gas_enable_bitwise() {
        let mut m = opaque(JointAxesMask { bits: 1 });
        enable(ref m, opaque(2));
        let _ = opaque(m);
    }
    #[test]
    fn gas_enable_arithmetic() {
        let mut m = opaque(JointAxesMask { bits: 1 });
        alternatives::enable_arithmetic(ref m, opaque(2));
        let _ = opaque(m);
    }
    #[test]
    fn gas_set_limits() {
        let mut j = opaque(Default::<GenericJoint>::default());
        j.set_limits(opaque(2), opaque([-HALF, HALF]));
        let _ = opaque(j);
    }
    #[test]
    fn gas_set_motor_model() {
        let mut j = opaque(Default::<GenericJoint>::default());
        j.set_motor_model(opaque(2), opaque(MotorModel::ForceBased));
        let _ = opaque(j);
    }
    #[test]
    fn gas_set_motor_max_force() {
        let mut j = opaque(Default::<GenericJoint>::default());
        j.set_motor_max_force(opaque(2), opaque(ONE));
        let _ = opaque(j);
    }
    #[test]
    fn gas_set_motor_velocity() {
        let mut j = opaque(Default::<GenericJoint>::default());
        j.set_motor_velocity(opaque(2), opaque(ONE), opaque(HALF));
        let _ = opaque(j);
    }
    #[test]
    fn gas_set_motor_position() {
        let mut j = opaque(Default::<GenericJoint>::default());
        j.set_motor_position(opaque(2), opaque(HALF), opaque(ONE), opaque(ONE));
        let _ = opaque(j);
    }
    #[test]
    fn gas_set_motor() {
        let mut j = opaque(Default::<GenericJoint>::default());
        j.set_motor(opaque(2), opaque(HALF), opaque(ONE), opaque(ONE), opaque(HALF));
        let _ = opaque(j);
    }
}
