//! Typed view of a prismatic joint (upstream `PrismaticJoint`): a `GenericJoint` whose free axes
//! are the ones listed below. Nothing is added to the generic joint: the view wraps it.
use fixed::Fixed;
use glam::Vec2;
use rapier_core::integration_parameters::spring::SpringCoefficients;
use super::{
    GenericJoint, GenericJointTrait, JointLimits, JointMotor, LOCKED_PRISMATIC_AXES, MotorModel,
    PrismaticJointBuilder,
};

/// A joint that locks every relative motion but the translation along the local X axis of two
/// bodies (upstream `PrismaticJoint`, 2D).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PrismaticJoint {
    pub data: GenericJoint,
}
#[generate_trait]
pub impl PrismaticJointImpl of PrismaticJointTrait {
    /// A prismatic joint along `axis`, expressed in the local space of both bodies (upstream
    /// `PrismaticJoint::new`). Panics with Joint: nonunit axis when `axis` is not a unit vector.
    fn new(axis: Vec2) -> PrismaticJoint {
        let mut data = GenericJoint { locked_axes: LOCKED_PRISMATIC_AXES, ..Default::default() };
        data.set_local_axis1(axis);
        data.set_local_axis2(axis);
        PrismaticJoint { data }
    }
    /// The underlying generic joint (upstream `data`).
    #[inline(always)]
    fn data(self: PrismaticJoint) -> GenericJoint {
        self.data
    }
    /// Whether the two attached bodies collide (upstream `contacts_enabled`).
    #[inline(always)]
    fn contacts_enabled(self: PrismaticJoint) -> bool {
        self.data.contacts_enabled
    }
    /// Sets whether the two attached bodies collide (upstream `set_contacts_enabled`).
    #[inline(always)]
    fn set_contacts_enabled(ref self: PrismaticJoint, enabled: bool) {
        self.data.contacts_enabled = enabled;
    }
    /// The anchor of the joint in the first body (upstream `local_anchor1`).
    #[inline(always)]
    fn local_anchor1(self: PrismaticJoint) -> Vec2 {
        self.data.local_frame1.translation
    }
    /// Sets the anchor of the joint in the first body (upstream `set_local_anchor1`); exact copy.
    #[inline(always)]
    fn set_local_anchor1(ref self: PrismaticJoint, anchor: Vec2) {
        self.data.local_frame1.translation = anchor;
    }
    /// The anchor of the joint in the second body (upstream `local_anchor2`).
    #[inline(always)]
    fn local_anchor2(self: PrismaticJoint) -> Vec2 {
        self.data.local_frame2.translation
    }
    /// Sets the anchor of the joint in the second body (upstream `set_local_anchor2`); exact copy.
    #[inline(always)]
    fn set_local_anchor2(ref self: PrismaticJoint, anchor: Vec2) {
        self.data.local_frame2.translation = anchor;
    }
    /// The constraint softness (upstream `softness`).
    #[inline(always)]
    fn softness(self: PrismaticJoint) -> SpringCoefficients {
        self.data.softness
    }
    /// Sets the constraint softness (upstream `set_softness`); exact copy.
    #[inline(always)]
    fn set_softness(ref self: PrismaticJoint, softness: SpringCoefficients) {
        self.data.softness = softness;
    }
    /// `local_frame1 * X` (upstream `local_axis1`; the translation is part of it, see
    /// [`GenericJointTrait::local_axis1`]).
    #[inline(always)]
    fn local_axis1(self: PrismaticJoint) -> Vec2 {
        self.data.local_axis1()
    }
    /// Sets the rotation of the first frame from its X axis (upstream `set_local_axis1`). Panics
    /// with Joint: nonunit axis when `axis` is not a unit vector.
    #[inline(always)]
    fn set_local_axis1(ref self: PrismaticJoint, axis: Vec2) {
        self.data.set_local_axis1(axis);
    }
    /// `local_frame2 * X` (upstream `local_axis2`, see [`local_axis1`](Self::local_axis1)).
    #[inline(always)]
    fn local_axis2(self: PrismaticJoint) -> Vec2 {
        self.data.local_axis2()
    }
    /// Sets the rotation of the second frame from its X axis (upstream `set_local_axis2`).
    #[inline(always)]
    fn set_local_axis2(ref self: PrismaticJoint, axis: Vec2) {
        self.data.set_local_axis2(axis);
    }
    /// The motor of the linear (local X) axis, `None` while it is not enabled (upstream `motor`).
    #[inline(always)]
    fn motor(self: PrismaticJoint) -> Option<JointMotor> {
        self.data.motor(0)
    }
    /// Sets the motor model of the linear (local X) axis without enabling the motor (upstream
    /// `set_motor_model`).
    #[inline(always)]
    fn set_motor_model(ref self: PrismaticJoint, model: MotorModel) {
        self.data.set_motor_model(0, model);
    }
    /// Enables the velocity motor of the linear (local X) axis (upstream `set_motor_velocity`);
    /// exact copies, the target position is kept and the stiffness cleared.
    #[inline(always)]
    fn set_motor_velocity(ref self: PrismaticJoint, target_vel: Fixed, factor: Fixed) {
        self.data.set_motor_velocity(0, target_vel, factor);
    }
    /// Enables the position motor of the linear (local X) axis (upstream `set_motor_position`);
    /// exact copies, the target velocity is cleared.
    #[inline(always)]
    fn set_motor_position(
        ref self: PrismaticJoint, target_pos: Fixed, stiffness: Fixed, damping: Fixed,
    ) {
        self.data.set_motor_position(0, target_pos, stiffness, damping);
    }
    /// Enables the motor of the linear (local X) axis (upstream `set_motor`); exact copies.
    #[inline(always)]
    fn set_motor(
        ref self: PrismaticJoint,
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
    fn set_motor_max_force(ref self: PrismaticJoint, max_force: Fixed) {
        self.data.set_motor_max_force(0, max_force);
    }
    /// The limits of the linear (local X) axis, `None` while none is set (upstream `limits`).
    #[inline(always)]
    fn limits(self: PrismaticJoint) -> Option<JointLimits> {
        self.data.limits(0)
    }
    /// Limits the linear (local X) axis to `[min, max]` (upstream `set_limits`); exact copies, no
    /// check of the order.
    #[inline(always)]
    fn set_limits(ref self: PrismaticJoint, limits: [Fixed; 2]) {
        self.data.set_limits(0, limits);
    }
}

/// Upstream `From<PrismaticJoint> for GenericJoint`.
pub impl PrismaticJointIntoGeneric of Into<PrismaticJoint, GenericJoint> {
    #[inline(always)]
    fn into(self: PrismaticJoint) -> GenericJoint {
        self.data
    }
}

/// Upstream `From<PrismaticJointBuilder> for GenericJoint`.
pub impl PrismaticJointBuilderIntoGeneric of Into<PrismaticJointBuilder, GenericJoint> {
    #[inline(always)]
    fn into(self: PrismaticJointBuilder) -> GenericJoint {
        self.data
    }
}
