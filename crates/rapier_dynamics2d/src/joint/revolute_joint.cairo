//! Typed view of a revolute joint (upstream `RevoluteJoint`): a `GenericJoint` that locks the
//! translation and leaves the angular axis free. Nothing is added to the generic joint.
use fixed::Fixed;
use fixed::trig::TrigTrait;
use glam::Vec2;
use rapier_core::integration_parameters::spring::SpringCoefficients;
use rapier_math::rot2::{Rot2, Rot2Trait};
use super::{
    GenericJoint, GenericJointTrait, JointLimits, JointMotor, LOCKED_REVOLUTE_AXES, MotorModel,
    RevoluteJointBuilder,
};

/// A joint that locks the translation of two bodies and leaves their relative rotation free
/// (upstream `RevoluteJoint`, 2D).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RevoluteJoint {
    pub data: GenericJoint,
}
#[generate_trait]
pub impl RevoluteJointImpl of RevoluteJointTrait {
    /// A revolute joint with identity frames (upstream `RevoluteJoint::new`, 2D).
    fn new() -> RevoluteJoint {
        RevoluteJoint {
            data: GenericJoint { locked_axes: LOCKED_REVOLUTE_AXES, ..Default::default() },
        }
    }
    /// The underlying generic joint (upstream `data`).
    #[inline(always)]
    fn data(self: RevoluteJoint) -> GenericJoint {
        self.data
    }
    /// Whether the two attached bodies collide (upstream `contacts_enabled`).
    #[inline(always)]
    fn contacts_enabled(self: RevoluteJoint) -> bool {
        self.data.contacts_enabled
    }
    /// Sets whether the two attached bodies collide (upstream `set_contacts_enabled`).
    #[inline(always)]
    fn set_contacts_enabled(ref self: RevoluteJoint, enabled: bool) {
        self.data.contacts_enabled = enabled;
    }
    /// The anchor of the joint in the first body (upstream `local_anchor1`).
    #[inline(always)]
    fn local_anchor1(self: RevoluteJoint) -> Vec2 {
        self.data.local_frame1.translation
    }
    /// Sets the anchor of the joint in the first body (upstream `set_local_anchor1`); exact copy.
    #[inline(always)]
    fn set_local_anchor1(ref self: RevoluteJoint, anchor: Vec2) {
        self.data.local_frame1.translation = anchor;
    }
    /// The anchor of the joint in the second body (upstream `local_anchor2`).
    #[inline(always)]
    fn local_anchor2(self: RevoluteJoint) -> Vec2 {
        self.data.local_frame2.translation
    }
    /// Sets the anchor of the joint in the second body (upstream `set_local_anchor2`); exact copy.
    #[inline(always)]
    fn set_local_anchor2(ref self: RevoluteJoint, anchor: Vec2) {
        self.data.local_frame2.translation = anchor;
    }
    /// The constraint softness (upstream `softness`).
    #[inline(always)]
    fn softness(self: RevoluteJoint) -> SpringCoefficients {
        self.data.softness
    }
    /// Sets the constraint softness (upstream `set_softness`); exact copy.
    #[inline(always)]
    fn set_softness(ref self: RevoluteJoint, softness: SpringCoefficients) {
        self.data.softness = softness;
    }
    /// The current relative angle of the two frames (upstream `angle`), from the rotations of the
    /// two bodies: `atan2` of the rotation `frame1⁻¹ * frame2` in world space, in `[-pi, pi]`.
    /// Panics with `Fixed: overflow` if the product leaves Q32.32 (unit rotations cannot).
    fn angle(self: RevoluteJoint, rb_rot1: Rot2, rb_rot2: Rot2) -> Fixed {
        let joint_rot1 = rb_rot1.mul(self.data.local_frame1.rotation);
        let joint_rot2 = rb_rot2.mul(self.data.local_frame2.rotation);
        let err = joint_rot1.inverse().mul(joint_rot2);
        err.im.atan2(err.re)
    }
    /// The motor of the angular axis, `None` while it is not enabled (upstream `motor`).
    #[inline(always)]
    fn motor(self: RevoluteJoint) -> Option<JointMotor> {
        self.data.motor(2)
    }
    /// Sets the motor model of the angular axis without enabling the motor (upstream
    /// `set_motor_model`).
    #[inline(always)]
    fn set_motor_model(ref self: RevoluteJoint, model: MotorModel) {
        self.data.set_motor_model(2, model);
    }
    /// Enables the velocity motor of the angular axis (upstream `set_motor_velocity`); exact
    /// copies, the target position is kept and the stiffness cleared.
    #[inline(always)]
    fn set_motor_velocity(ref self: RevoluteJoint, target_vel: Fixed, factor: Fixed) {
        self.data.set_motor_velocity(2, target_vel, factor);
    }
    /// Enables the position motor of the angular axis (upstream `set_motor_position`); exact
    /// copies, the target velocity is cleared.
    #[inline(always)]
    fn set_motor_position(
        ref self: RevoluteJoint, target_pos: Fixed, stiffness: Fixed, damping: Fixed,
    ) {
        self.data.set_motor_position(2, target_pos, stiffness, damping);
    }
    /// Enables the motor of the angular axis (upstream `set_motor`); exact copies.
    #[inline(always)]
    fn set_motor(
        ref self: RevoluteJoint,
        target_pos: Fixed,
        target_vel: Fixed,
        stiffness: Fixed,
        damping: Fixed,
    ) {
        self.data.set_motor(2, target_pos, target_vel, stiffness, damping);
    }
    /// Sets the force cap of the angular motor without enabling it (upstream
    /// `set_motor_max_force`); exact copy.
    #[inline(always)]
    fn set_motor_max_force(ref self: RevoluteJoint, max_force: Fixed) {
        self.data.set_motor_max_force(2, max_force);
    }
    /// The limits of the angular axis, `None` while none is set (upstream `limits`).
    #[inline(always)]
    fn limits(self: RevoluteJoint) -> Option<JointLimits> {
        self.data.limits(2)
    }
    /// Limits the angular axis to `[min, max]` (upstream `set_limits`); exact copies, no check
    /// of the order.
    #[inline(always)]
    fn set_limits(ref self: RevoluteJoint, limits: [Fixed; 2]) {
        self.data.set_limits(2, limits);
    }
}

/// Upstream `From<RevoluteJoint> for GenericJoint`.
pub impl RevoluteJointIntoGeneric of Into<RevoluteJoint, GenericJoint> {
    #[inline(always)]
    fn into(self: RevoluteJoint) -> GenericJoint {
        self.data
    }
}

/// Upstream `From<RevoluteJointBuilder> for GenericJoint`.
pub impl RevoluteJointBuilderIntoGeneric of Into<RevoluteJointBuilder, GenericJoint> {
    #[inline(always)]
    fn into(self: RevoluteJointBuilder) -> GenericJoint {
        self.data
    }
}
