//! Pin-slot joint (upstream `PinSlotJoint`, 2D only; Godot's groove joint): the bodies keep
//! their relative rotation free and slide along the joint's X axis. A `GenericJoint` that locks the
//! local Y axis only, wrapped by a typed view and a builder that build it.
use fixed::Fixed;
use glam::Vec2;
use rapier_core::integration_parameters::spring::SpringCoefficients;
use super::{
    GenericJoint, GenericJointTrait, JointLimits, JointMotor, LOCKED_PIN_SLOT_AXES, MotorModel,
};

/// A joint that locks every relative motion but the translation along the joint's local X axis
/// and the relative rotation (upstream `PinSlotJoint`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PinSlotJoint {
    pub data: GenericJoint,
}
#[generate_trait]
pub impl PinSlotJointImpl of PinSlotJointTrait {
    /// A pin-slot joint sliding along `axis`, expressed in the local space of both bodies
    /// (upstream `PinSlotJoint::new`). Panics with Joint: nonunit axis when `axis` is not a unit
    /// vector.
    fn new(axis: Vec2) -> PinSlotJoint {
        let mut data = GenericJoint { locked_axes: LOCKED_PIN_SLOT_AXES, ..Default::default() };
        data.set_local_axis1(axis);
        data.set_local_axis2(axis);
        PinSlotJoint { data }
    }
    /// The underlying generic joint (upstream `data`).
    #[inline(always)]
    fn data(self: PinSlotJoint) -> GenericJoint {
        self.data
    }
    /// Whether the two attached bodies collide (upstream `contacts_enabled`).
    #[inline(always)]
    fn contacts_enabled(self: PinSlotJoint) -> bool {
        self.data.contacts_enabled
    }
    /// Sets whether the two attached bodies collide (upstream `set_contacts_enabled`).
    #[inline(always)]
    fn set_contacts_enabled(ref self: PinSlotJoint, enabled: bool) {
        self.data.contacts_enabled = enabled;
    }
    /// The anchor of the joint in the first body (upstream `local_anchor1`).
    #[inline(always)]
    fn local_anchor1(self: PinSlotJoint) -> Vec2 {
        self.data.local_frame1.translation
    }
    /// Sets the anchor of the joint in the first body (upstream `set_local_anchor1`); exact copy.
    #[inline(always)]
    fn set_local_anchor1(ref self: PinSlotJoint, anchor: Vec2) {
        self.data.local_frame1.translation = anchor;
    }
    /// The anchor of the joint in the second body (upstream `local_anchor2`).
    #[inline(always)]
    fn local_anchor2(self: PinSlotJoint) -> Vec2 {
        self.data.local_frame2.translation
    }
    /// Sets the anchor of the joint in the second body (upstream `set_local_anchor2`); exact copy.
    #[inline(always)]
    fn set_local_anchor2(ref self: PinSlotJoint, anchor: Vec2) {
        self.data.local_frame2.translation = anchor;
    }
    /// The constraint softness (upstream `softness`).
    #[inline(always)]
    fn softness(self: PinSlotJoint) -> SpringCoefficients {
        self.data.softness
    }
    /// Sets the constraint softness (upstream `set_softness`); exact copy.
    #[inline(always)]
    fn set_softness(ref self: PinSlotJoint, softness: SpringCoefficients) {
        self.data.softness = softness;
    }
    /// `local_frame1 * X` (upstream `local_axis1`; the translation is part of it, see
    /// [`GenericJointTrait::local_axis1`]).
    #[inline(always)]
    fn local_axis1(self: PinSlotJoint) -> Vec2 {
        self.data.local_axis1()
    }
    /// Sets the rotation of the first frame from its X axis (upstream `set_local_axis1`). Panics
    /// with Joint: nonunit axis when `axis` is not a unit vector.
    #[inline(always)]
    fn set_local_axis1(ref self: PinSlotJoint, axis: Vec2) {
        self.data.set_local_axis1(axis);
    }
    /// `local_frame2 * X` (upstream `local_axis2`, see [`local_axis1`](Self::local_axis1)).
    #[inline(always)]
    fn local_axis2(self: PinSlotJoint) -> Vec2 {
        self.data.local_axis2()
    }
    /// Sets the rotation of the second frame from its X axis (upstream `set_local_axis2`).
    #[inline(always)]
    fn set_local_axis2(ref self: PinSlotJoint, axis: Vec2) {
        self.data.set_local_axis2(axis);
    }
    /// The motor of the linear (local X) axis, `None` while it is not enabled (upstream `motor`).
    #[inline(always)]
    fn motor(self: PinSlotJoint) -> Option<JointMotor> {
        self.data.motor(0)
    }
    /// Sets the motor model of the linear (local X) axis without enabling the motor (upstream
    /// `set_motor_model`).
    #[inline(always)]
    fn set_motor_model(ref self: PinSlotJoint, model: MotorModel) {
        self.data.set_motor_model(0, model);
    }
    /// Enables the velocity motor of the linear (local X) axis (upstream `set_motor_velocity`);
    /// exact copies, the target position is kept and the stiffness cleared.
    #[inline(always)]
    fn set_motor_velocity(ref self: PinSlotJoint, target_vel: Fixed, factor: Fixed) {
        self.data.set_motor_velocity(0, target_vel, factor);
    }
    /// Enables the position motor of the linear (local X) axis (upstream `set_motor_position`);
    /// exact copies, the target velocity is cleared.
    #[inline(always)]
    fn set_motor_position(
        ref self: PinSlotJoint, target_pos: Fixed, stiffness: Fixed, damping: Fixed,
    ) {
        self.data.set_motor_position(0, target_pos, stiffness, damping);
    }
    /// Enables the motor of the linear (local X) axis (upstream `set_motor`); exact copies.
    #[inline(always)]
    fn set_motor(
        ref self: PinSlotJoint,
        target_pos: Fixed,
        target_vel: Fixed,
        stiffness: Fixed,
        damping: Fixed,
    ) {
        self.data.set_motor(0, target_pos, target_vel, stiffness, damping);
    }
    /// Sets the force cap of the linear (local X) motor without enabling it (upstream
    /// `set_motor_max_force`); exact copy.
    #[inline(always)]
    fn set_motor_max_force(ref self: PinSlotJoint, max_force: Fixed) {
        self.data.set_motor_max_force(0, max_force);
    }
    /// The limits of the linear (local X) axis, `None` while none is set (upstream `limits`).
    #[inline(always)]
    fn limits(self: PinSlotJoint) -> Option<JointLimits> {
        self.data.limits(0)
    }
    /// Limits the linear (local X) axis to `[min, max]` (upstream `set_limits`); exact copies, no
    /// check of the order.
    #[inline(always)]
    fn set_limits(ref self: PinSlotJoint, limits: [Fixed; 2]) {
        self.data.set_limits(0, limits);
    }
}

/// Upstream `From<PinSlotJoint> for GenericJoint`.
pub impl PinSlotJointIntoGeneric of Into<PinSlotJoint, GenericJoint> {
    #[inline(always)]
    fn into(self: PinSlotJoint) -> GenericJoint {
        self.data
    }
}

/// Pin-slot joint builder; build returns the shared `GenericJoint` representation, as the other
/// builders (upstream builds a `PinSlotJoint`: wrap it with `PinSlotJoint { data }`).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PinSlotJointBuilder {
    pub data: GenericJoint,
}
#[generate_trait]
pub impl PinSlotJointBuilderImpl of PinSlotJointBuilderTrait {
    /// A builder of a pin-slot joint sliding along `axis` (upstream `PinSlotJointBuilder::new`);
    /// a nonunit axis panics with Joint: nonunit axis.
    fn new(axis: Vec2) -> PinSlotJointBuilder {
        PinSlotJointBuilder { data: PinSlotJointTrait::new(axis).data }
    }
    /// Set contacts_enabled; exact copy.
    fn contacts_enabled(mut self: PinSlotJointBuilder, enabled: bool) -> PinSlotJointBuilder {
        self.data.contacts_enabled = enabled;
        self
    }
    /// Set local_anchor1; exact copy.
    fn local_anchor1(mut self: PinSlotJointBuilder, anchor: Vec2) -> PinSlotJointBuilder {
        self.data.local_frame1.translation = anchor;
        self
    }
    /// Set local_anchor2; exact copy.
    fn local_anchor2(mut self: PinSlotJointBuilder, anchor: Vec2) -> PinSlotJointBuilder {
        self.data.local_frame2.translation = anchor;
        self
    }
    /// Set the rotation of the first frame from its X axis; a nonunit axis panics with Joint:
    /// nonunit axis.
    fn local_axis1(mut self: PinSlotJointBuilder, axis: Vec2) -> PinSlotJointBuilder {
        self.data.set_local_axis1(axis);
        self
    }
    /// Set the rotation of the second frame from its X axis; a nonunit axis panics with Joint:
    /// nonunit axis.
    fn local_axis2(mut self: PinSlotJointBuilder, axis: Vec2) -> PinSlotJointBuilder {
        self.data.set_local_axis2(axis);
        self
    }
    /// Store the model of the sliding motor without enabling it; exact copy.
    fn motor_model(mut self: PinSlotJointBuilder, model: MotorModel) -> PinSlotJointBuilder {
        self.data.set_motor_model(0, model);
        self
    }
    /// Enable the sliding velocity motor; exact copies.
    fn motor_velocity(
        mut self: PinSlotJointBuilder, target_vel: Fixed, factor: Fixed,
    ) -> PinSlotJointBuilder {
        self.data.set_motor_velocity(0, target_vel, factor);
        self
    }
    /// Enable the sliding position motor; exact copies.
    fn motor_position(
        mut self: PinSlotJointBuilder, target_pos: Fixed, stiffness: Fixed, damping: Fixed,
    ) -> PinSlotJointBuilder {
        self.data.set_motor_position(0, target_pos, stiffness, damping);
        self
    }
    /// Enable the sliding motor with a position and a velocity target; exact copies.
    fn set_motor(
        mut self: PinSlotJointBuilder,
        target_pos: Fixed,
        target_vel: Fixed,
        stiffness: Fixed,
        damping: Fixed,
    ) -> PinSlotJointBuilder {
        self.data.set_motor(0, target_pos, target_vel, stiffness, damping);
        self
    }
    /// Store the force cap of the sliding motor without enabling it; exact copy.
    fn motor_max_force(mut self: PinSlotJointBuilder, max_force: Fixed) -> PinSlotJointBuilder {
        self.data.set_motor_max_force(0, max_force);
        self
    }
    /// Limit the slide to `[min, max]`; exact copies.
    fn limits(mut self: PinSlotJointBuilder, limits: [Fixed; 2]) -> PinSlotJointBuilder {
        self.data.set_limits(0, limits);
        self
    }
    /// Set softness; exact copy.
    fn softness(
        mut self: PinSlotJointBuilder, softness: SpringCoefficients,
    ) -> PinSlotJointBuilder {
        self.data.softness = softness;
        self
    }
    /// Return the joint; all copies are exact.
    fn build(self: PinSlotJointBuilder) -> GenericJoint {
        self.data
    }
}
/// Upstream `From<PinSlotJointBuilder> for GenericJoint`.
pub impl PinSlotJointBuilderIntoGeneric of Into<PinSlotJointBuilder, GenericJoint> {
    #[inline(always)]
    fn into(self: PinSlotJointBuilder) -> GenericJoint {
        self.data
    }
}
