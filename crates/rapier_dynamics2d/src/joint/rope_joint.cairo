//! Rope joint builder (upstream `RopeJointBuilder`): no locked axis, coupled linear axes and a
//! `[0, max_dist]` limit on the first one, i.e. a maximum anchor distance. Setters copy exactly.
use fixed::{Fixed, MAX, ZERO};
use glam::Vec2;
use rapier_core::integration_parameters::spring::SpringCoefficients;
use super::{GenericJoint, GenericJointTrait, JointMotor, LIN_AXES, MotorModel};

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


/// A joint that keeps the two anchors within a maximum distance (upstream `RopeJoint`): a
/// `GenericJoint` with coupled linear axes and a `[0, max_dist]` limit on the first one.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RopeJoint {
    pub data: GenericJoint,
}
#[generate_trait]
pub impl RopeJointImpl of RopeJointTrait {
    /// A rope of maximum anchor distance `max_dist` (upstream `RopeJoint::new`; the value is
    /// copied without validation).
    fn new(max_dist: Fixed) -> RopeJoint {
        let mut data = GenericJoint { coupled_axes: LIN_AXES, ..Default::default() };
        data.set_limits(0, [ZERO, max_dist]);
        RopeJoint { data }
    }
    /// The underlying generic joint (upstream `data`).
    #[inline(always)]
    fn data(self: RopeJoint) -> GenericJoint {
        self.data
    }
    /// Whether the two attached bodies collide (upstream `contacts_enabled`).
    #[inline(always)]
    fn contacts_enabled(self: RopeJoint) -> bool {
        self.data.contacts_enabled
    }
    /// Sets whether the two attached bodies collide (upstream `set_contacts_enabled`).
    #[inline(always)]
    fn set_contacts_enabled(ref self: RopeJoint, enabled: bool) {
        self.data.contacts_enabled = enabled;
    }
    /// The anchor of the joint in the first body (upstream `local_anchor1`).
    #[inline(always)]
    fn local_anchor1(self: RopeJoint) -> Vec2 {
        self.data.local_frame1.translation
    }
    /// Sets the anchor of the joint in the first body (upstream `set_local_anchor1`); exact copy.
    #[inline(always)]
    fn set_local_anchor1(ref self: RopeJoint, anchor: Vec2) {
        self.data.local_frame1.translation = anchor;
    }
    /// The anchor of the joint in the second body (upstream `local_anchor2`).
    #[inline(always)]
    fn local_anchor2(self: RopeJoint) -> Vec2 {
        self.data.local_frame2.translation
    }
    /// Sets the anchor of the joint in the second body (upstream `set_local_anchor2`); exact copy.
    #[inline(always)]
    fn set_local_anchor2(ref self: RopeJoint, anchor: Vec2) {
        self.data.local_frame2.translation = anchor;
    }
    /// The constraint softness (upstream `softness`).
    #[inline(always)]
    fn softness(self: RopeJoint) -> SpringCoefficients {
        self.data.softness
    }
    /// Sets the constraint softness (upstream `set_softness`); exact copy.
    #[inline(always)]
    fn set_softness(ref self: RopeJoint, softness: SpringCoefficients) {
        self.data.softness = softness;
    }
    /// The motor of `axis` (upstream `motor`, which takes the axis: the coupled distance motor
    /// is on `0`), `None` while it is not enabled.
    #[inline(always)]
    fn motor(self: RopeJoint, axis: u8) -> Option<JointMotor> {
        self.data.motor(axis)
    }
    /// The maximum distance between the anchors (upstream `max_distance`); `MAX` when no limit
    /// is set.
    fn max_distance(self: RopeJoint) -> Fixed {
        match self.data.limits(0) {
            Some(limits) => limits.max,
            None => MAX,
        }
    }
    /// Sets the maximum distance between the anchors: limit `[0, max_dist]` (upstream
    /// `set_max_distance`); exact copy.
    #[inline(always)]
    fn set_max_distance(ref self: RopeJoint, max_dist: Fixed) {
        self.data.set_limits(0, [ZERO, max_dist]);
    }
    /// Sets the motor model of the coupled distance axis without enabling the motor (upstream
    /// `set_motor_model`).
    #[inline(always)]
    fn set_motor_model(ref self: RopeJoint, model: MotorModel) {
        self.data.set_motor_model(0, model);
    }
    /// Enables the velocity motor of the coupled distance axis (upstream `set_motor_velocity`);
    /// exact copies, the target position is kept and the stiffness cleared.
    #[inline(always)]
    fn set_motor_velocity(ref self: RopeJoint, target_vel: Fixed, factor: Fixed) {
        self.data.set_motor_velocity(0, target_vel, factor);
    }
    /// Enables the position motor of the coupled distance axis (upstream `set_motor_position`);
    /// exact copies, the target velocity is cleared.
    #[inline(always)]
    fn set_motor_position(
        ref self: RopeJoint, target_pos: Fixed, stiffness: Fixed, damping: Fixed,
    ) {
        self.data.set_motor_position(0, target_pos, stiffness, damping);
    }
    /// Enables the motor of the coupled distance axis (upstream `set_motor`); exact copies.
    #[inline(always)]
    fn set_motor(
        ref self: RopeJoint, target_pos: Fixed, target_vel: Fixed, stiffness: Fixed, damping: Fixed,
    ) {
        self.data.set_motor(0, target_pos, target_vel, stiffness, damping);
    }
    /// Sets the force cap of the coupled distance motor without enabling it (upstream
    /// `set_motor_max_force`); exact copy.
    #[inline(always)]
    fn set_motor_max_force(ref self: RopeJoint, max_force: Fixed) {
        self.data.set_motor_max_force(0, max_force);
    }
}

/// Upstream `From<RopeJoint> for GenericJoint`.
pub impl RopeJointIntoGeneric of Into<RopeJoint, GenericJoint> {
    #[inline(always)]
    fn into(self: RopeJoint) -> GenericJoint {
        self.data
    }
}

/// Upstream `From<RopeJointBuilder> for GenericJoint`.
pub impl RopeJointBuilderIntoGeneric of Into<RopeJointBuilder, GenericJoint> {
    #[inline(always)]
    fn into(self: RopeJointBuilder) -> GenericJoint {
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
