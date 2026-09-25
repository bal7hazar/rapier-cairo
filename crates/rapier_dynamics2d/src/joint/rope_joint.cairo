//! Rope joint builder (upstream `RopeJointBuilder`): no locked axis, coupled linear axes and a
//! `[0, max_dist]` limit on the first one, i.e. a maximum anchor distance. Setters copy exactly.
use fixed::{Fixed, ZERO};
use glam::Vec2;
use rapier_core::integration_parameters::spring::SpringCoefficients;
use super::{GenericJoint, GenericJointTrait, LIN_AXES, MotorModel};

/// Rope joint builder; build returns the shared GenericJoint representation.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RopeJointBuilder {
    pub data: GenericJoint,
}
#[generate_trait]
pub impl RopeJointBuilderImpl of RopeJointBuilderTrait {
    /// Create a rope of maximum anchor distance `max_dist` (upstream requires it > 0; the value
    /// is copied without validation). No locked axis, coupled X/Y, limit `[0, max_dist]` on X.
    fn new(max_dist: Fixed) -> RopeJointBuilder {
        let mut data = GenericJoint { coupled_axes: LIN_AXES, ..Default::default() };
        data.set_limits(0, [ZERO, max_dist]);
        RopeJointBuilder { data }
    }
    /// Set contacts_enabled; exact copy, no arithmetic or panics.
    fn contacts_enabled(mut self: RopeJointBuilder, value: bool) -> RopeJointBuilder {
        self.data.contacts_enabled = value;
        self
    }
    /// Set local_anchor1; exact copy, no arithmetic or panics.
    fn local_anchor1(mut self: RopeJointBuilder, value: Vec2) -> RopeJointBuilder {
        self.data.local_frame1.translation = value;
        self
    }
    /// Set local_anchor2; exact copy, no arithmetic or panics.
    fn local_anchor2(mut self: RopeJointBuilder, value: Vec2) -> RopeJointBuilder {
        self.data.local_frame2.translation = value;
        self
    }
    /// Store the coupled motor's model without enabling it; exact copy.
    fn motor_model(mut self: RopeJointBuilder, model: MotorModel) -> RopeJointBuilder {
        self.data.set_motor_model(0, model);
        self
    }
    /// Enable the coupled (distance) velocity motor; preserve target position, clear stiffness.
    /// No rounding or numeric validation; the solver validates physical inputs.
    fn motor_velocity(
        mut self: RopeJointBuilder, target_vel: Fixed, factor: Fixed,
    ) -> RopeJointBuilder {
        self.data.set_motor_velocity(0, target_vel, factor);
        self
    }
    /// Enable the coupled (distance) position motor and clear target velocity; exact copies.
    fn motor_position(
        mut self: RopeJointBuilder, target_pos: Fixed, stiffness: Fixed, damping: Fixed,
    ) -> RopeJointBuilder {
        self.data.set_motor_position(0, target_pos, stiffness, damping);
        self
    }
    /// Enable combined position/velocity control of the coupled motor; exact copies.
    fn set_motor(
        mut self: RopeJointBuilder,
        target_pos: Fixed,
        target_vel: Fixed,
        stiffness: Fixed,
        damping: Fixed,
    ) -> RopeJointBuilder {
        self.data.set_motor(0, target_pos, target_vel, stiffness, damping);
        self
    }
    /// Store the coupled motor's force cap without enabling it; exact copy.
    fn motor_max_force(mut self: RopeJointBuilder, max_force: Fixed) -> RopeJointBuilder {
        self.data.set_motor_max_force(0, max_force);
        self
    }
    /// Replace the maximum distance: limit `[0, max_dist]` on X; exact copy.
    fn max_distance(mut self: RopeJointBuilder, max_dist: Fixed) -> RopeJointBuilder {
        self.data.set_limits(0, [ZERO, max_dist]);
        self
    }
    /// Set softness; exact copy, no arithmetic or panics.
    fn softness(mut self: RopeJointBuilder, value: SpringCoefficients) -> RopeJointBuilder {
        self.data.softness = value;
        self
    }
    /// Return the joint; all copies are exact.
    fn build(self: RopeJointBuilder) -> GenericJoint {
        self.data
    }
}

#[cfg(test)]
mod tests {
    use fixed::{HALF, MAX, MIN, ONE};
    use rapier_testing::opaque;
    use super::*;
    use super::super::{
        GenericJointBuilderTrait, JOINT_DEFAULTS, JointAxesMask, JointEnabled, JointLimits,
    };

    #[test]
    fn test_rope_matches_upstream_generic_construction() {
        let v = Vec2 { x: HALF, y: -ONE };
        let mut expected = GenericJointBuilderTrait::new(JointAxesMask { bits: 0 })
            .coupled_axes(LIN_AXES)
            .local_anchor1(v)
            .local_anchor2(-v)
            .contacts_enabled(false)
            .softness(JOINT_DEFAULTS)
            .build();
        expected.set_limits(0, [ZERO, ONE]);
        let rope = RopeJointBuilderTrait::new(HALF)
            .local_anchor1(v)
            .local_anchor2(-v)
            .contacts_enabled(false)
            .max_distance(ONE)
            .build();
        assert_eq!(rope, expected);
        assert_eq!(rope.locked_axes.bits, 0);
        assert_eq!(rope.limit_axes.bits, 1);
        assert_eq!(rope.motor_axes.bits, 0);
        assert_eq!(rope.enabled, JointEnabled::Enabled);
        let [x, y, w] = rope.limits;
        assert_eq!(x, JointLimits { min: ZERO, max: ONE, impulse: ZERO });
        assert_eq!((y.min, y.max, w.min, w.max), (MIN, MAX, MIN, MAX));
        // Motor setters target the first coupled axis, exactly as the generic setters.
        let mut m = expected;
        m.set_motor_model(0, MotorModel::ForceBased);
        m.set_motor(0, ONE, HALF, ONE, HALF);
        m.set_motor_max_force(0, HALF);
        let r = RopeJointBuilder { data: rope }
            .motor_model(MotorModel::ForceBased)
            .set_motor(ONE, HALF, ONE, HALF)
            .motor_max_force(HALF)
            .build();
        assert_eq!(r, m);
        m.set_motor_velocity(0, -ONE, ONE);
        assert_eq!(RopeJointBuilder { data: r }.motor_velocity(-ONE, ONE).build(), m);
        m.set_motor_position(0, HALF, ONE, ZERO);
        assert_eq!(RopeJointBuilder { data: m }.motor_position(HALF, ONE, ZERO).build(), m);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }
    #[test]
    fn gas_rope_new() {
        let _ = opaque(RopeJointBuilderTrait::new(opaque(ONE)));
    }
    #[test]
    fn gas_rope_max_distance() {
        let _ = opaque(opaque(RopeJointBuilderTrait::new(ONE)).max_distance(opaque(HALF)));
    }
}
