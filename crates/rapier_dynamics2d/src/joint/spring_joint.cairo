//! Spring joint builder (upstream `SpringJointBuilder`): no locked axis, coupled linear axes and
//! a force-based position motor on the first one toward the rest length. Setters copy exactly.
use fixed::Fixed;
use glam::Vec2;
use super::{GenericJoint, GenericJointTrait, LIN_AXES, MotorModel};

/// Spring joint builder; build returns the shared GenericJoint representation.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SpringJointBuilder {
    pub data: GenericJoint,
}
#[generate_trait]
pub impl SpringJointBuilderImpl of SpringJointBuilderTrait {
    /// Create a spring pulling the anchor distance toward `rest_length`, with `stiffness` and
    /// `damping` (must be nonnegative; checked by the solver). Model `ForceBased`, as upstream.
    fn new(rest_length: Fixed, stiffness: Fixed, damping: Fixed) -> SpringJointBuilder {
        let mut data = GenericJoint { coupled_axes: LIN_AXES, ..Default::default() };
        data.set_motor_position(0, rest_length, stiffness, damping);
        data.set_motor_model(0, MotorModel::ForceBased);
        SpringJointBuilder { data }
    }
    /// Set contacts_enabled; exact copy, no arithmetic or panics.
    fn contacts_enabled(mut self: SpringJointBuilder, value: bool) -> SpringJointBuilder {
        self.data.contacts_enabled = value;
        self
    }
    /// Set local_anchor1; exact copy, no arithmetic or panics.
    fn local_anchor1(mut self: SpringJointBuilder, value: Vec2) -> SpringJointBuilder {
        self.data.local_frame1.translation = value;
        self
    }
    /// Set local_anchor2; exact copy, no arithmetic or panics.
    fn local_anchor2(mut self: SpringJointBuilder, value: Vec2) -> SpringJointBuilder {
        self.data.local_frame2.translation = value;
        self
    }
    /// Mass-dependent (`ForceBased`, default) or mass-independent (`AccelerationBased`)
    /// spring constants; exact copy.
    fn spring_model(mut self: SpringJointBuilder, model: MotorModel) -> SpringJointBuilder {
        self.data.set_motor_model(0, model);
        self
    }
    /// Return the joint; all copies are exact.
    fn build(self: SpringJointBuilder) -> GenericJoint {
        self.data
    }
}

#[cfg(test)]
mod tests {
    use fixed::{HALF, ONE};
    use rapier_testing::opaque;
    use super::*;
    use super::super::{GenericJointBuilderTrait, JointAxesMask, JointMotor};

    #[test]
    fn test_spring_matches_upstream_generic_construction() {
        let v = Vec2 { x: HALF, y: -ONE };
        let mut expected = GenericJointBuilderTrait::new(JointAxesMask { bits: 0 })
            .coupled_axes(LIN_AXES)
            .motor_position(0, ONE, HALF + HALF + HALF, HALF)
            .motor_model(0, MotorModel::ForceBased)
            .local_anchor1(v)
            .local_anchor2(-v)
            .contacts_enabled(false)
            .build();
        let spring = SpringJointBuilderTrait::new(ONE, HALF + HALF + HALF, HALF)
            .local_anchor1(v)
            .local_anchor2(-v)
            .contacts_enabled(false)
            .build();
        assert_eq!(spring, expected);
        assert_eq!(
            (spring.locked_axes.bits, spring.limit_axes.bits, spring.motor_axes.bits), (0, 0, 1),
        );
        let [m, _, _] = spring.motors;
        assert_eq!(
            m,
            JointMotor {
                target_pos: ONE,
                stiffness: HALF + HALF + HALF,
                damping: HALF,
                model: MotorModel::ForceBased,
                ..Default::default(),
            },
        );
        expected.set_motor_model(0, MotorModel::AccelerationBased);
        let s = SpringJointBuilder { data: spring }.spring_model(MotorModel::AccelerationBased);
        assert_eq!(s.build(), expected);
    }
    #[test]
    fn gas_baseline() {
        let _ = opaque(ONE);
    }
    #[test]
    fn gas_spring_new() {
        let _ = opaque(SpringJointBuilderTrait::new(opaque(ONE), opaque(ONE), opaque(HALF)));
    }
}
