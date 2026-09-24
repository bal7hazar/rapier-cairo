//! Value builders; setters copy without rounding. Frames must contain unit rotations.
use fixed::Fixed;
use glam::Vec2;
use rapier_core::integration_parameters::spring::SpringCoefficients;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::{Rot2, Rot2Trait};
use super::{
    GenericJoint, GenericJointTrait, JointAxesMask, JointEnabled, LOCKED_FIXED_AXES,
    LOCKED_PRISMATIC_AXES, LOCKED_REVOLUTE_AXES, MotorModel, errors,
};

/// Generic joint builder; build returns the shared GenericJoint representation.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct GenericJointBuilder {
    pub data: GenericJoint,
}
#[generate_trait]
pub impl GenericJointBuilderImpl of GenericJointBuilderTrait {
    /// Create with upstream locks; prismatic axis must be unit within 8 ulp squared norm.
    fn new(locked_axes: JointAxesMask) -> GenericJointBuilder {
        let data = GenericJoint { locked_axes: locked_axes, ..Default::default() };
        assert(locked_axes.bits <= 7, errors::MASK);
        let result = GenericJointBuilder { data };
        result
    }
    /// Set local_frame1; exact copy, no arithmetic or panics.
    fn local_frame1(mut self: GenericJointBuilder, value: Pose2) -> GenericJointBuilder {
        self.data.local_frame1 = value;
        self
    }
    /// Set local_frame2; exact copy, no arithmetic or panics.
    fn local_frame2(mut self: GenericJointBuilder, value: Pose2) -> GenericJointBuilder {
        self.data.local_frame2 = value;
        self
    }
    /// Set local_anchor1; exact copy, no arithmetic or panics.
    fn local_anchor1(mut self: GenericJointBuilder, value: Vec2) -> GenericJointBuilder {
        self.data.local_frame1.translation = value;
        self
    }
    /// Set local_anchor2; exact copy, no arithmetic or panics.
    fn local_anchor2(mut self: GenericJointBuilder, value: Vec2) -> GenericJointBuilder {
        self.data.local_frame2.translation = value;
        self
    }
    /// Set contacts_enabled; exact copy, no arithmetic or panics.
    fn contacts_enabled(mut self: GenericJointBuilder, value: bool) -> GenericJointBuilder {
        self.data.contacts_enabled = value;
        self
    }
    /// Set enabled; exact copy, no arithmetic or panics.
    fn enabled(mut self: GenericJointBuilder, value: bool) -> GenericJointBuilder {
        self.data.enabled = if value {
            JointEnabled::Enabled
        } else {
            JointEnabled::Disabled
        };
        self
    }
    /// Set softness; exact copy, no arithmetic or panics.
    fn softness(mut self: GenericJointBuilder, value: SpringCoefficients) -> GenericJointBuilder {
        self.data.softness = value;
        self
    }
    /// Store limits and enable their axis; values are copied exactly.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Axis must be 0=X, 1=Y, 2=AngX; invalid axis/mask panics use Joint errors.
    fn limits(mut self: GenericJointBuilder, axis: u8, limits: [Fixed; 2]) -> GenericJointBuilder {
        self.data.set_limits(axis, limits);
        self
    }
    /// Store the model without enabling the motor; exact copy.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Axis must be 0=X, 1=Y, 2=AngX; invalid axis/mask panics use Joint errors.
    fn motor_model(
        mut self: GenericJointBuilder, axis: u8, model: MotorModel,
    ) -> GenericJointBuilder {
        self.data.set_motor_model(axis, model);
        self
    }
    /// Enable velocity control; preserve target position and clear stiffness.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Axis must be 0=X, 1=Y, 2=AngX; invalid axis/mask panics use Joint errors.
    fn motor_velocity(
        mut self: GenericJointBuilder, axis: u8, target_vel: Fixed, factor: Fixed,
    ) -> GenericJointBuilder {
        self.data.set_motor_velocity(axis, target_vel, factor);
        self
    }
    /// Enable position control and clear target velocity; exact copies.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Axis must be 0=X, 1=Y, 2=AngX; invalid axis/mask panics use Joint errors.
    fn motor_position(
        mut self: GenericJointBuilder,
        axis: u8,
        target_pos: Fixed,
        stiffness: Fixed,
        damping: Fixed,
    ) -> GenericJointBuilder {
        self.data.set_motor_position(axis, target_pos, stiffness, damping);
        self
    }
    /// Enable combined position/velocity control; exact copies.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Axis must be 0=X, 1=Y, 2=AngX; invalid axis/mask panics use Joint errors.
    fn set_motor(
        mut self: GenericJointBuilder,
        axis: u8,
        target_pos: Fixed,
        target_vel: Fixed,
        stiffness: Fixed,
        damping: Fixed,
    ) -> GenericJointBuilder {
        self.data.set_motor(axis, target_pos, target_vel, stiffness, damping);
        self
    }
    /// Store the force cap without enabling the motor; exact copy.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Axis must be 0=X, 1=Y, 2=AngX; invalid axis/mask panics use Joint errors.
    fn motor_max_force(
        mut self: GenericJointBuilder, axis: u8, max_force: Fixed,
    ) -> GenericJointBuilder {
        self.data.set_motor_max_force(axis, max_force);
        self
    }
    /// Return the joint; all copies are exact.
    fn build(self: GenericJointBuilder) -> GenericJoint {
        self.data
    }
}

/// Fixed joint builder; build returns the shared GenericJoint representation.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct FixedJointBuilder {
    pub data: GenericJoint,
}
#[generate_trait]
pub impl FixedJointBuilderImpl of FixedJointBuilderTrait {
    /// Create with upstream locks; prismatic axis must be unit within 8 ulp squared norm.
    fn new() -> FixedJointBuilder {
        let data = GenericJoint { locked_axes: LOCKED_FIXED_AXES, ..Default::default() };
        let result = FixedJointBuilder { data };
        result
    }
    /// Set local_frame1; exact copy, no arithmetic or panics.
    fn local_frame1(mut self: FixedJointBuilder, value: Pose2) -> FixedJointBuilder {
        self.data.local_frame1 = value;
        self
    }
    /// Set local_frame2; exact copy, no arithmetic or panics.
    fn local_frame2(mut self: FixedJointBuilder, value: Pose2) -> FixedJointBuilder {
        self.data.local_frame2 = value;
        self
    }
    /// Set local_anchor1; exact copy, no arithmetic or panics.
    fn local_anchor1(mut self: FixedJointBuilder, value: Vec2) -> FixedJointBuilder {
        self.data.local_frame1.translation = value;
        self
    }
    /// Set local_anchor2; exact copy, no arithmetic or panics.
    fn local_anchor2(mut self: FixedJointBuilder, value: Vec2) -> FixedJointBuilder {
        self.data.local_frame2.translation = value;
        self
    }
    /// Set contacts_enabled; exact copy, no arithmetic or panics.
    fn contacts_enabled(mut self: FixedJointBuilder, value: bool) -> FixedJointBuilder {
        self.data.contacts_enabled = value;
        self
    }
    /// Set enabled; exact copy, no arithmetic or panics.
    fn enabled(mut self: FixedJointBuilder, value: bool) -> FixedJointBuilder {
        self.data.enabled = if value {
            JointEnabled::Enabled
        } else {
            JointEnabled::Disabled
        };
        self
    }
    /// Set softness; exact copy, no arithmetic or panics.
    fn softness(mut self: FixedJointBuilder, value: SpringCoefficients) -> FixedJointBuilder {
        self.data.softness = value;
        self
    }
    /// Return the joint; all copies are exact.
    fn build(self: FixedJointBuilder) -> GenericJoint {
        self.data
    }
}

/// Revolute joint builder; build returns the shared GenericJoint representation.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RevoluteJointBuilder {
    pub data: GenericJoint,
}
#[generate_trait]
pub impl RevoluteJointBuilderImpl of RevoluteJointBuilderTrait {
    /// Create with upstream locks; prismatic axis must be unit within 8 ulp squared norm.
    fn new() -> RevoluteJointBuilder {
        let data = GenericJoint { locked_axes: LOCKED_REVOLUTE_AXES, ..Default::default() };
        let result = RevoluteJointBuilder { data };
        result
    }
    /// Set local_frame1; exact copy, no arithmetic or panics.
    fn local_frame1(mut self: RevoluteJointBuilder, value: Pose2) -> RevoluteJointBuilder {
        self.data.local_frame1 = value;
        self
    }
    /// Set local_frame2; exact copy, no arithmetic or panics.
    fn local_frame2(mut self: RevoluteJointBuilder, value: Pose2) -> RevoluteJointBuilder {
        self.data.local_frame2 = value;
        self
    }
    /// Set local_anchor1; exact copy, no arithmetic or panics.
    fn local_anchor1(mut self: RevoluteJointBuilder, value: Vec2) -> RevoluteJointBuilder {
        self.data.local_frame1.translation = value;
        self
    }
    /// Set local_anchor2; exact copy, no arithmetic or panics.
    fn local_anchor2(mut self: RevoluteJointBuilder, value: Vec2) -> RevoluteJointBuilder {
        self.data.local_frame2.translation = value;
        self
    }
    /// Set contacts_enabled; exact copy, no arithmetic or panics.
    fn contacts_enabled(mut self: RevoluteJointBuilder, value: bool) -> RevoluteJointBuilder {
        self.data.contacts_enabled = value;
        self
    }
    /// Set enabled; exact copy, no arithmetic or panics.
    fn enabled(mut self: RevoluteJointBuilder, value: bool) -> RevoluteJointBuilder {
        self.data.enabled = if value {
            JointEnabled::Enabled
        } else {
            JointEnabled::Disabled
        };
        self
    }
    /// Set softness; exact copy, no arithmetic or panics.
    fn softness(mut self: RevoluteJointBuilder, value: SpringCoefficients) -> RevoluteJointBuilder {
        self.data.softness = value;
        self
    }
    /// Store limits and enable their axis; values are copied exactly.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Uses the free joint axis; valid builder state does not panic.
    fn limits(mut self: RevoluteJointBuilder, limits: [Fixed; 2]) -> RevoluteJointBuilder {
        self.data.set_limits(2, limits);
        self
    }
    /// Store the model without enabling the motor; exact copy.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Uses the free joint axis; valid builder state does not panic.
    fn motor_model(mut self: RevoluteJointBuilder, model: MotorModel) -> RevoluteJointBuilder {
        self.data.set_motor_model(2, model);
        self
    }
    /// Enable velocity control; preserve target position and clear stiffness.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Uses the free joint axis; valid builder state does not panic.
    fn motor_velocity(
        mut self: RevoluteJointBuilder, target_vel: Fixed, factor: Fixed,
    ) -> RevoluteJointBuilder {
        self.data.set_motor_velocity(2, target_vel, factor);
        self
    }
    /// Enable position control and clear target velocity; exact copies.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Uses the free joint axis; valid builder state does not panic.
    fn motor_position(
        mut self: RevoluteJointBuilder, target_pos: Fixed, stiffness: Fixed, damping: Fixed,
    ) -> RevoluteJointBuilder {
        self.data.set_motor_position(2, target_pos, stiffness, damping);
        self
    }
    /// Enable combined position/velocity control; exact copies.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Uses the free joint axis; valid builder state does not panic.
    fn motor(
        mut self: RevoluteJointBuilder,
        target_pos: Fixed,
        target_vel: Fixed,
        stiffness: Fixed,
        damping: Fixed,
    ) -> RevoluteJointBuilder {
        self.data.set_motor(2, target_pos, target_vel, stiffness, damping);
        self
    }
    /// Store the force cap without enabling the motor; exact copy.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Uses the free joint axis; valid builder state does not panic.
    fn motor_max_force(mut self: RevoluteJointBuilder, max_force: Fixed) -> RevoluteJointBuilder {
        self.data.set_motor_max_force(2, max_force);
        self
    }
    /// Return the joint; all copies are exact.
    fn build(self: RevoluteJointBuilder) -> GenericJoint {
        self.data
    }
}

/// Prismatic joint builder; build returns the shared GenericJoint representation.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PrismaticJointBuilder {
    pub data: GenericJoint,
}
#[generate_trait]
pub impl PrismaticJointBuilderImpl of PrismaticJointBuilderTrait {
    /// Create with upstream locks; prismatic axis must be unit within 8 ulp squared norm.
    fn new(axis: Vec2) -> PrismaticJointBuilder {
        let data = GenericJoint { locked_axes: LOCKED_PRISMATIC_AXES, ..Default::default() };
        let result = PrismaticJointBuilder { data };
        result.axis(axis)
    }
    /// Set local_frame1; exact copy, no arithmetic or panics.
    fn local_frame1(mut self: PrismaticJointBuilder, value: Pose2) -> PrismaticJointBuilder {
        self.data.local_frame1 = value;
        self
    }
    /// Set local_frame2; exact copy, no arithmetic or panics.
    fn local_frame2(mut self: PrismaticJointBuilder, value: Pose2) -> PrismaticJointBuilder {
        self.data.local_frame2 = value;
        self
    }
    /// Set local_anchor1; exact copy, no arithmetic or panics.
    fn local_anchor1(mut self: PrismaticJointBuilder, value: Vec2) -> PrismaticJointBuilder {
        self.data.local_frame1.translation = value;
        self
    }
    /// Set local_anchor2; exact copy, no arithmetic or panics.
    fn local_anchor2(mut self: PrismaticJointBuilder, value: Vec2) -> PrismaticJointBuilder {
        self.data.local_frame2.translation = value;
        self
    }
    /// Set contacts_enabled; exact copy, no arithmetic or panics.
    fn contacts_enabled(mut self: PrismaticJointBuilder, value: bool) -> PrismaticJointBuilder {
        self.data.contacts_enabled = value;
        self
    }
    /// Set enabled; exact copy, no arithmetic or panics.
    fn enabled(mut self: PrismaticJointBuilder, value: bool) -> PrismaticJointBuilder {
        self.data.enabled = if value {
            JointEnabled::Enabled
        } else {
            JointEnabled::Disabled
        };
        self
    }
    /// Set softness; exact copy, no arithmetic or panics.
    fn softness(
        mut self: PrismaticJointBuilder, value: SpringCoefficients,
    ) -> PrismaticJointBuilder {
        self.data.softness = value;
        self
    }
    /// Set both local X axes; nonunit/zero input panics with Joint: nonunit axis.
    fn axis(mut self: PrismaticJointBuilder, axis: Vec2) -> PrismaticJointBuilder {
        let r = Rot2 { re: axis.x, im: axis.y };
        assert(r.is_unit(), errors::UNIT);
        self.data.local_frame1.rotation = r;
        self.data.local_frame2.rotation = r;
        self
    }
    /// Store limits and enable their axis; values are copied exactly.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Uses the free joint axis; valid builder state does not panic.
    fn limits(mut self: PrismaticJointBuilder, limits: [Fixed; 2]) -> PrismaticJointBuilder {
        self.data.set_limits(0, limits);
        self
    }
    /// Store the model without enabling the motor; exact copy.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Uses the free joint axis; valid builder state does not panic.
    fn motor_model(mut self: PrismaticJointBuilder, model: MotorModel) -> PrismaticJointBuilder {
        self.data.set_motor_model(0, model);
        self
    }
    /// Enable velocity control; preserve target position and clear stiffness.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Uses the free joint axis; valid builder state does not panic.
    fn motor_velocity(
        mut self: PrismaticJointBuilder, target_vel: Fixed, factor: Fixed,
    ) -> PrismaticJointBuilder {
        self.data.set_motor_velocity(0, target_vel, factor);
        self
    }
    /// Enable position control and clear target velocity; exact copies.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Uses the free joint axis; valid builder state does not panic.
    fn motor_position(
        mut self: PrismaticJointBuilder, target_pos: Fixed, stiffness: Fixed, damping: Fixed,
    ) -> PrismaticJointBuilder {
        self.data.set_motor_position(0, target_pos, stiffness, damping);
        self
    }
    /// Enable combined position/velocity control; exact copies.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Uses the free joint axis; valid builder state does not panic.
    fn set_motor(
        mut self: PrismaticJointBuilder,
        target_pos: Fixed,
        target_vel: Fixed,
        stiffness: Fixed,
        damping: Fixed,
    ) -> PrismaticJointBuilder {
        self.data.set_motor(0, target_pos, target_vel, stiffness, damping);
        self
    }
    /// Store the force cap without enabling the motor; exact copy.
    /// No rounding or numeric validation; the solver validates physical inputs.
    /// Uses the free joint axis; valid builder state does not panic.
    fn motor_max_force(mut self: PrismaticJointBuilder, max_force: Fixed) -> PrismaticJointBuilder {
        self.data.set_motor_max_force(0, max_force);
        self
    }
    /// Return the joint; all copies are exact.
    fn build(self: PrismaticJointBuilder) -> GenericJoint {
        self.data
    }
}

#[cfg(test)]
mod tests {
    use fixed::{ONE, ZERO};
    use rapier_testing::opaque;
    use super::*;
    #[test]
    fn test_builders() {
        let a = Vec2 { x: ONE, y: ZERO };
        assert_eq!(
            FixedJointBuilderTrait::new().local_anchor1(a).build().local_frame1.translation, a,
        );
        assert_eq!(RevoluteJointBuilderTrait::new().build().locked_axes, LOCKED_REVOLUTE_AXES);
        let j = PrismaticJointBuilderTrait::new(a).contacts_enabled(false).enabled(false).build();
        assert_eq!(j.locked_axes, LOCKED_PRISMATIC_AXES);
        assert!(!j.contacts_enabled);
        assert_eq!(j.enabled, JointEnabled::Disabled);
    }
    #[test]
    #[should_panic(expected: 'Joint: nonunit axis')]
    fn test_zero_axis() {
        let _ = PrismaticJointBuilderTrait::new(Default::default());
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }
    #[test]
    fn gas_generic_new() {
        let _ = opaque(GenericJointBuilderTrait::new(opaque(LOCKED_FIXED_AXES)));
    }
    #[test]
    fn gas_generic_build() {
        let _ = opaque(opaque(GenericJointBuilderTrait::new(opaque(LOCKED_FIXED_AXES))).build());
    }
    #[test]
    fn gas_generic_local_frame1() {
        let _ = opaque(
            opaque(GenericJointBuilderTrait::new(opaque(LOCKED_FIXED_AXES)))
                .local_frame1(opaque(Default::default())),
        );
    }
    #[test]
    fn gas_generic_local_frame2() {
        let _ = opaque(
            opaque(GenericJointBuilderTrait::new(opaque(LOCKED_FIXED_AXES)))
                .local_frame2(opaque(Default::default())),
        );
    }
    #[test]
    fn gas_generic_local_anchor1() {
        let _ = opaque(
            opaque(GenericJointBuilderTrait::new(opaque(LOCKED_FIXED_AXES)))
                .local_anchor1(opaque(Vec2 { x: ONE, y: ZERO })),
        );
    }
    #[test]
    fn gas_generic_local_anchor2() {
        let _ = opaque(
            opaque(GenericJointBuilderTrait::new(opaque(LOCKED_FIXED_AXES)))
                .local_anchor2(opaque(Vec2 { x: ONE, y: ZERO })),
        );
    }
    #[test]
    fn gas_generic_contacts_enabled() {
        let _ = opaque(
            opaque(GenericJointBuilderTrait::new(opaque(LOCKED_FIXED_AXES)))
                .contacts_enabled(opaque(false)),
        );
    }
    #[test]
    fn gas_generic_enabled() {
        let _ = opaque(
            opaque(GenericJointBuilderTrait::new(opaque(LOCKED_FIXED_AXES))).enabled(opaque(false)),
        );
    }
    #[test]
    fn gas_generic_softness() {
        let _ = opaque(
            opaque(GenericJointBuilderTrait::new(opaque(LOCKED_FIXED_AXES)))
                .softness(opaque(super::super::JOINT_DEFAULTS)),
        );
    }
    #[test]
    fn gas_fixed_new() {
        let _ = opaque(FixedJointBuilderTrait::new());
    }
    #[test]
    fn gas_fixed_build() {
        let _ = opaque(opaque(FixedJointBuilderTrait::new()).build());
    }
    #[test]
    fn gas_fixed_local_frame1() {
        let _ = opaque(
            opaque(FixedJointBuilderTrait::new()).local_frame1(opaque(Default::default())),
        );
    }
    #[test]
    fn gas_fixed_local_frame2() {
        let _ = opaque(
            opaque(FixedJointBuilderTrait::new()).local_frame2(opaque(Default::default())),
        );
    }
    #[test]
    fn gas_fixed_local_anchor1() {
        let _ = opaque(
            opaque(FixedJointBuilderTrait::new()).local_anchor1(opaque(Vec2 { x: ONE, y: ZERO })),
        );
    }
    #[test]
    fn gas_fixed_local_anchor2() {
        let _ = opaque(
            opaque(FixedJointBuilderTrait::new()).local_anchor2(opaque(Vec2 { x: ONE, y: ZERO })),
        );
    }
    #[test]
    fn gas_fixed_contacts_enabled() {
        let _ = opaque(opaque(FixedJointBuilderTrait::new()).contacts_enabled(opaque(false)));
    }
    #[test]
    fn gas_fixed_enabled() {
        let _ = opaque(opaque(FixedJointBuilderTrait::new()).enabled(opaque(false)));
    }
    #[test]
    fn gas_fixed_softness() {
        let _ = opaque(
            opaque(FixedJointBuilderTrait::new()).softness(opaque(super::super::JOINT_DEFAULTS)),
        );
    }
    #[test]
    fn gas_revolute_new() {
        let _ = opaque(RevoluteJointBuilderTrait::new());
    }
    #[test]
    fn gas_revolute_build() {
        let _ = opaque(opaque(RevoluteJointBuilderTrait::new()).build());
    }
    #[test]
    fn gas_revolute_local_frame1() {
        let _ = opaque(
            opaque(RevoluteJointBuilderTrait::new()).local_frame1(opaque(Default::default())),
        );
    }
    #[test]
    fn gas_revolute_local_frame2() {
        let _ = opaque(
            opaque(RevoluteJointBuilderTrait::new()).local_frame2(opaque(Default::default())),
        );
    }
    #[test]
    fn gas_revolute_local_anchor1() {
        let _ = opaque(
            opaque(RevoluteJointBuilderTrait::new())
                .local_anchor1(opaque(Vec2 { x: ONE, y: ZERO })),
        );
    }
    #[test]
    fn gas_revolute_local_anchor2() {
        let _ = opaque(
            opaque(RevoluteJointBuilderTrait::new())
                .local_anchor2(opaque(Vec2 { x: ONE, y: ZERO })),
        );
    }
    #[test]
    fn gas_revolute_contacts_enabled() {
        let _ = opaque(opaque(RevoluteJointBuilderTrait::new()).contacts_enabled(opaque(false)));
    }
    #[test]
    fn gas_revolute_enabled() {
        let _ = opaque(opaque(RevoluteJointBuilderTrait::new()).enabled(opaque(false)));
    }
    #[test]
    fn gas_revolute_softness() {
        let _ = opaque(
            opaque(RevoluteJointBuilderTrait::new()).softness(opaque(super::super::JOINT_DEFAULTS)),
        );
    }
    #[test]
    fn gas_prismatic_new() {
        let _ = opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })));
    }
    #[test]
    fn gas_prismatic_build() {
        let _ = opaque(
            opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO }))).build(),
        );
    }
    #[test]
    fn gas_prismatic_local_frame1() {
        let _ = opaque(
            opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
                .local_frame1(opaque(Default::default())),
        );
    }
    #[test]
    fn gas_prismatic_local_frame2() {
        let _ = opaque(
            opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
                .local_frame2(opaque(Default::default())),
        );
    }
    #[test]
    fn gas_prismatic_local_anchor1() {
        let _ = opaque(
            opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
                .local_anchor1(opaque(Vec2 { x: ONE, y: ZERO })),
        );
    }
    #[test]
    fn gas_prismatic_local_anchor2() {
        let _ = opaque(
            opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
                .local_anchor2(opaque(Vec2 { x: ONE, y: ZERO })),
        );
    }
    #[test]
    fn gas_prismatic_contacts_enabled() {
        let _ = opaque(
            opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
                .contacts_enabled(opaque(false)),
        );
    }
    #[test]
    fn gas_prismatic_enabled() {
        let _ = opaque(
            opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
                .enabled(opaque(false)),
        );
    }
    #[test]
    fn gas_prismatic_softness() {
        let _ = opaque(
            opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
                .softness(opaque(super::super::JOINT_DEFAULTS)),
        );
    }
    #[test]
    fn gas_prismatic_axis() {
        let _ = opaque(
            opaque(PrismaticJointBuilderTrait::new(opaque(Vec2 { x: ONE, y: ZERO })))
                .axis(opaque(Vec2 { x: ZERO, y: ONE })),
        );
    }
}
