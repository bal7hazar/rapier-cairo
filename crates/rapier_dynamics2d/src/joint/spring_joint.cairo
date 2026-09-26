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


/// A spring between the two anchors (upstream `SpringJoint`): a `GenericJoint` with coupled linear
/// axes and a force-based position motor on the first one toward the rest length.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct SpringJoint {
    pub data: GenericJoint,
}
#[generate_trait]
pub impl SpringJointImpl of SpringJointTrait {
    /// A spring pulling the anchor distance toward `rest_length` with `stiffness` and `damping`
    /// (upstream `SpringJoint::new`); model `ForceBased`.
    fn new(rest_length: Fixed, stiffness: Fixed, damping: Fixed) -> SpringJoint {
        SpringJoint { data: SpringJointBuilderTrait::new(rest_length, stiffness, damping).data }
    }
    /// The underlying generic joint (upstream `data`).
    #[inline(always)]
    fn data(self: SpringJoint) -> GenericJoint {
        self.data
    }
    /// Whether the two attached bodies collide (upstream `contacts_enabled`).
    #[inline(always)]
    fn contacts_enabled(self: SpringJoint) -> bool {
        self.data.contacts_enabled
    }
    /// Sets whether the two attached bodies collide (upstream `set_contacts_enabled`).
    #[inline(always)]
    fn set_contacts_enabled(ref self: SpringJoint, enabled: bool) {
        self.data.contacts_enabled = enabled;
    }
    /// The anchor of the joint in the first body (upstream `local_anchor1`).
    #[inline(always)]
    fn local_anchor1(self: SpringJoint) -> Vec2 {
        self.data.local_frame1.translation
    }
    /// Sets the anchor of the joint in the first body (upstream `set_local_anchor1`); exact copy.
    #[inline(always)]
    fn set_local_anchor1(ref self: SpringJoint, anchor: Vec2) {
        self.data.local_frame1.translation = anchor;
    }
    /// The anchor of the joint in the second body (upstream `local_anchor2`).
    #[inline(always)]
    fn local_anchor2(self: SpringJoint) -> Vec2 {
        self.data.local_frame2.translation
    }
    /// Sets the anchor of the joint in the second body (upstream `set_local_anchor2`); exact copy.
    #[inline(always)]
    fn set_local_anchor2(ref self: SpringJoint, anchor: Vec2) {
        self.data.local_frame2.translation = anchor;
    }
    /// Mass-dependent (`ForceBased`, default) or mass-independent (`AccelerationBased`) spring
    /// constants (upstream `set_spring_model`); exact copy.
    #[inline(always)]
    fn set_spring_model(ref self: SpringJoint, model: MotorModel) {
        self.data.set_motor_model(0, model);
    }
}

/// Upstream `From<SpringJoint> for GenericJoint`.
pub impl SpringJointIntoGeneric of Into<SpringJoint, GenericJoint> {
    #[inline(always)]
    fn into(self: SpringJoint) -> GenericJoint {
        self.data
    }
}

/// Upstream `From<SpringJointBuilder> for GenericJoint`.
pub impl SpringJointBuilderIntoGeneric of Into<SpringJointBuilder, GenericJoint> {
    #[inline(always)]
    fn into(self: SpringJointBuilder) -> GenericJoint {
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
