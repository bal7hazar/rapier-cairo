//! Upstream limit/motor configuration. Setters copy exactly; validation occurs in the solver.
use fixed::{Fixed, MAX, MIN, ONE, ZERO};
use glam::Vec2;
use rapier_core::integration_parameters::spring::SpringCoefficients;
use rapier_math::pose2::{Pose2, Pose2Trait};
use rapier_math::rot2::{Rot2, Rot2Trait};
use super::{
    FixedJoint, GenericJoint, JointAxesMask, JointAxesMaskTrait, JointEnabled, JointLimits,
    JointMotor, LOCKED_FIXED_AXES, LOCKED_PRISMATIC_AXES, LOCKED_REVOLUTE_AXES, MotorModel,
    PrismaticJoint, RevoluteJoint, RopeJoint, errors,
};

#[generate_trait]
pub impl GenericJointImpl of GenericJointTrait {
    /// Upstream `GenericJoint::new`: the default joint with `locked_axes` locked. Mask 0..7
    /// required (Joint: invalid mask otherwise).
    fn new(locked_axes: JointAxesMask) -> GenericJoint {
        let mut joint: GenericJoint = Default::default();
        joint.lock_axes(locked_axes);
        joint
    }
    /// Adds `axes` to the locked axes (upstream `lock_axes`, a bitwise or). Masks 0..7 required
    /// (Joint: invalid mask otherwise).
    fn lock_axes(ref self: GenericJoint, axes: JointAxesMask) {
        assert(self.locked_axes.bits <= 7, errors::MASK);
        assert(axes.bits <= 7, errors::MASK);
        self.locked_axes.bits = self.locked_axes.bits | axes.bits;
    }
    /// Upstream `GenericJoint::complete_ang_frame`, 2D: the rotation whose first column is
    /// `axis`. Unlike upstream (which takes the columns as they are) the axis must be a unit
    /// vector, since every frame of a joint is a unit rotation (Joint: nonunit axis otherwise).
    fn complete_ang_frame(axis: Vec2) -> Rot2 {
        let rotation = Rot2 { re: axis.x, im: axis.y };
        assert(rotation.is_unit(), errors::UNIT);
        rotation
    }
    /// Whether the joint is enabled (upstream `is_enabled`): neither disabled by hand nor by an
    /// attached body.
    #[inline(always)]
    fn is_enabled(self: GenericJoint) -> bool {
        self.enabled == JointEnabled::Enabled
    }
    /// Upstream `set_enabled`: `false` disables an enabled joint (or one disabled by its bodies),
    /// `true` re-enables a joint that was disabled by hand only; a joint disabled by an attached
    /// body stays so until the body is enabled again.
    fn set_enabled(ref self: GenericJoint, enabled: bool) {
        self.enabled = match self.enabled {
            JointEnabled::Disabled => if enabled {
                JointEnabled::Enabled
            } else {
                JointEnabled::Disabled
            },
            other => if enabled {
                other
            } else {
                JointEnabled::Disabled
            },
        };
    }
    /// Sets the frame of the joint in the first body (upstream `set_local_frame1`); exact copy.
    #[inline(always)]
    fn set_local_frame1(ref self: GenericJoint, local_frame: Pose2) {
        self.local_frame1 = local_frame;
    }
    /// Sets the frame of the joint in the second body (upstream `set_local_frame2`); exact copy.
    #[inline(always)]
    fn set_local_frame2(ref self: GenericJoint, local_frame: Pose2) {
        self.local_frame2 = local_frame;
    }
    /// Upstream `local_axis1`: `local_frame1 * X`. As in upstream 0.35 (`Pose * Vec2` is
    /// `transform_point`) the translation of the frame is part of the result: read
    /// `local_frame1.rotation.rotate(X)` for the bare axis.
    fn local_axis1(self: GenericJoint) -> Vec2 {
        self.local_frame1.transform_point(Vec2 { x: ONE, y: ZERO })
    }
    /// Upstream `set_local_axis1`: the rotation of the first frame that maps X to `axis`
    /// ([`complete_ang_frame`](Self::complete_ang_frame); a nonunit `axis` panics).
    fn set_local_axis1(ref self: GenericJoint, local_axis: Vec2) {
        self.local_frame1.rotation = Self::complete_ang_frame(local_axis);
    }
    /// Upstream `local_axis2`, see [`local_axis1`](Self::local_axis1).
    fn local_axis2(self: GenericJoint) -> Vec2 {
        self.local_frame2.transform_point(Vec2 { x: ONE, y: ZERO })
    }
    /// Upstream `set_local_axis2`, see [`set_local_axis1`](Self::set_local_axis1).
    fn set_local_axis2(ref self: GenericJoint, local_axis: Vec2) {
        self.local_frame2.rotation = Self::complete_ang_frame(local_axis);
    }
    /// The anchor of the joint in the first body (upstream `local_anchor1`).
    #[inline(always)]
    fn local_anchor1(self: GenericJoint) -> Vec2 {
        self.local_frame1.translation
    }
    /// Sets the anchor of the joint in the first body (upstream `set_local_anchor1`).
    #[inline(always)]
    fn set_local_anchor1(ref self: GenericJoint, anchor: Vec2) {
        self.local_frame1.translation = anchor;
    }
    /// The anchor of the joint in the second body (upstream `local_anchor2`).
    #[inline(always)]
    fn local_anchor2(self: GenericJoint) -> Vec2 {
        self.local_frame2.translation
    }
    /// Sets the anchor of the joint in the second body (upstream `set_local_anchor2`).
    #[inline(always)]
    fn set_local_anchor2(ref self: GenericJoint, anchor: Vec2) {
        self.local_frame2.translation = anchor;
    }
    /// Whether the two attached bodies collide (upstream `contacts_enabled`).
    #[inline(always)]
    fn contacts_enabled(self: GenericJoint) -> bool {
        self.contacts_enabled
    }
    /// Upstream `set_contacts_enabled`.
    #[inline(always)]
    fn set_contacts_enabled(ref self: GenericJoint, enabled: bool) {
        self.contacts_enabled = enabled;
    }
    /// Sets the constraint softness (upstream `set_softness`); exact copy.
    #[inline(always)]
    fn set_softness(ref self: GenericJoint, softness: SpringCoefficients) {
        self.softness = softness;
    }
    /// The limits of `axis` (0=X, 1=Y, 2=AngX), `None` when the axis has no limit enabled
    /// (upstream `limits`). Invalid axis/mask panics with Joint: invalid axis / mask.
    fn limits(self: GenericJoint, axis: u8) -> Option<JointLimits> {
        if !self.limit_axes.contains_axis(axis) {
            return None;
        }
        let [x, y, w] = self.limits;
        Some(match axis {
            0 => x,
            1 => y,
            _ => w,
        })
    }
    /// The motor of `axis` (0=X, 1=Y, 2=AngX), `None` when the axis has no motor enabled
    /// (upstream `motor`). Invalid axis/mask panics with Joint: invalid axis / mask.
    fn motor(self: GenericJoint, axis: u8) -> Option<JointMotor> {
        if !self.motor_axes.contains_axis(axis) {
            return None;
        }
        Some(read_motor(self, axis))
    }
    /// The motor model of `axis`, `None` when the axis has no motor enabled (upstream
    /// `motor_model`).
    fn motor_model(self: GenericJoint, axis: u8) -> Option<MotorModel> {
        Some(self.motor(axis)?.model)
    }
    /// The typed view of a revolute joint, `None` unless the locked axes are exactly
    /// `LOCKED_REVOLUTE_AXES` (upstream `as_revolute`, by value: write a change back with
    /// `joint = view.into()`).
    fn as_revolute(self: GenericJoint) -> Option<RevoluteJoint> {
        if self.locked_axes == LOCKED_REVOLUTE_AXES {
            Some(RevoluteJoint { data: self })
        } else {
            None
        }
    }
    /// The typed view of a fixed joint, `None` unless the locked axes are exactly
    /// `LOCKED_FIXED_AXES` (upstream `as_fixed`).
    fn as_fixed(self: GenericJoint) -> Option<FixedJoint> {
        if self.locked_axes == LOCKED_FIXED_AXES {
            Some(FixedJoint { data: self })
        } else {
            None
        }
    }
    /// The typed view of a prismatic joint, `None` unless the locked axes are exactly
    /// `LOCKED_PRISMATIC_AXES` (upstream `as_prismatic`).
    fn as_prismatic(self: GenericJoint) -> Option<PrismaticJoint> {
        if self.locked_axes == LOCKED_PRISMATIC_AXES {
            Some(PrismaticJoint { data: self })
        } else {
            None
        }
    }
    /// The typed view of a rope joint, `None` unless no axis is locked (upstream `as_rope`).
    fn as_rope(self: GenericJoint) -> Option<RopeJoint> {
        if self.locked_axes.bits == 0 {
            Some(RopeJoint { data: self })
        } else {
            None
        }
    }
    /// Upstream `flip`: swaps the two frames and mirrors what is expressed in them: the limits
    /// of the axes that are not coupled become `[-max, -min]`, the motor target velocity and
    /// position are negated. The extrema exchange (`-MIN` is `MAX` and `-MAX` is `MIN`), so
    /// the unbounded limits (`MIN`, `MAX`) stay unbounded, as upstream's `±f32::MAX`.
    fn flip(ref self: GenericJoint) {
        let frame1 = self.local_frame1;
        self.local_frame1 = self.local_frame2;
        self.local_frame2 = frame1;
        let coupled = self.coupled_axes;
        let [mut x, mut y, mut w] = self.limits;
        if !coupled.contains_axis(0) {
            flip_limits(ref x);
        }
        if !coupled.contains_axis(1) {
            flip_limits(ref y);
        }
        if !coupled.contains_axis(2) {
            flip_limits(ref w);
        }
        self.limits = [x, y, w];
        let [mut mx, mut my, mut mw] = self.motors;
        flip_motor(ref mx);
        flip_motor(ref my);
        flip_motor(ref mw);
        self.motors = [mx, my, mw];
    }
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

/// `-value`, except that the extrema exchange (`MIN` <-> `MAX`): Q32.32 is asymmetric, upstream's
/// unbounded `±f32::MAX` are symmetric.
fn mirror(value: Fixed) -> Fixed {
    if value == MIN {
        MAX
    } else if value == MAX {
        MIN
    } else {
        -value
    }
}
fn flip_limits(ref limits: JointLimits) {
    let min = limits.min;
    limits.min = mirror(limits.max);
    limits.max = mirror(min);
}
fn flip_motor(ref motor: JointMotor) {
    motor.target_vel = mirror(motor.target_vel);
    motor.target_pos = mirror(motor.target_pos);
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
