//! Typed view of a fixed joint (upstream `FixedJoint`): a `GenericJoint` with every axis locked.
//! Nothing is added to the generic joint: the view wraps it and its methods are field accesses.
use glam::Vec2;
use rapier_core::integration_parameters::spring::SpringCoefficients;
use rapier_math::pose2::Pose2;
use super::{FixedJointBuilder, GenericJoint, LOCKED_FIXED_AXES};

/// A joint that locks every relative motion of two bodies (upstream `FixedJoint`); the view of a
/// [`GenericJoint`] built with `LOCKED_FIXED_AXES`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct FixedJoint {
    pub data: GenericJoint,
}

pub impl FixedJointDefault of Default<FixedJoint> {
    fn default() -> FixedJoint {
        FixedJointTrait::new()
    }
}
#[generate_trait]
pub impl FixedJointImpl of FixedJointTrait {
    /// A fixed joint with identity frames, contacts enabled (upstream `FixedJoint::new`).
    fn new() -> FixedJoint {
        FixedJoint { data: GenericJoint { locked_axes: LOCKED_FIXED_AXES, ..Default::default() } }
    }
    /// The underlying generic joint (upstream `data`).
    #[inline(always)]
    fn data(self: FixedJoint) -> GenericJoint {
        self.data
    }
    /// Whether the two attached bodies collide (upstream `contacts_enabled`).
    #[inline(always)]
    fn contacts_enabled(self: FixedJoint) -> bool {
        self.data.contacts_enabled
    }
    /// Sets whether the two attached bodies collide (upstream `set_contacts_enabled`).
    #[inline(always)]
    fn set_contacts_enabled(ref self: FixedJoint, enabled: bool) {
        self.data.contacts_enabled = enabled;
    }
    /// The anchor of the joint in the first body (upstream `local_anchor1`).
    #[inline(always)]
    fn local_anchor1(self: FixedJoint) -> Vec2 {
        self.data.local_frame1.translation
    }
    /// Sets the anchor of the joint in the first body (upstream `set_local_anchor1`); exact copy.
    #[inline(always)]
    fn set_local_anchor1(ref self: FixedJoint, anchor: Vec2) {
        self.data.local_frame1.translation = anchor;
    }
    /// The anchor of the joint in the second body (upstream `local_anchor2`).
    #[inline(always)]
    fn local_anchor2(self: FixedJoint) -> Vec2 {
        self.data.local_frame2.translation
    }
    /// Sets the anchor of the joint in the second body (upstream `set_local_anchor2`); exact copy.
    #[inline(always)]
    fn set_local_anchor2(ref self: FixedJoint, anchor: Vec2) {
        self.data.local_frame2.translation = anchor;
    }
    /// The constraint softness (upstream `softness`).
    #[inline(always)]
    fn softness(self: FixedJoint) -> SpringCoefficients {
        self.data.softness
    }
    /// Sets the constraint softness (upstream `set_softness`); exact copy.
    #[inline(always)]
    fn set_softness(ref self: FixedJoint, softness: SpringCoefficients) {
        self.data.softness = softness;
    }
    /// The frame of the joint in the first body (upstream `local_frame1`).
    #[inline(always)]
    fn local_frame1(self: FixedJoint) -> Pose2 {
        self.data.local_frame1
    }
    /// Sets the frame of the joint in the first body (upstream `set_local_frame1`); exact copy.
    #[inline(always)]
    fn set_local_frame1(ref self: FixedJoint, local_frame: Pose2) {
        self.data.local_frame1 = local_frame;
    }
    /// The frame of the joint in the second body (upstream `local_frame2`).
    #[inline(always)]
    fn local_frame2(self: FixedJoint) -> Pose2 {
        self.data.local_frame2
    }
    /// Sets the frame of the joint in the second body (upstream `set_local_frame2`); exact copy.
    #[inline(always)]
    fn set_local_frame2(ref self: FixedJoint, local_frame: Pose2) {
        self.data.local_frame2 = local_frame;
    }
}

/// Upstream `From<FixedJoint> for GenericJoint`.
pub impl FixedJointIntoGeneric of Into<FixedJoint, GenericJoint> {
    #[inline(always)]
    fn into(self: FixedJoint) -> GenericJoint {
        self.data
    }
}

/// Upstream `From<FixedJointBuilder> for GenericJoint`.
pub impl FixedJointBuilderIntoGeneric of Into<FixedJointBuilder, GenericJoint> {
    #[inline(always)]
    fn into(self: FixedJointBuilder) -> GenericJoint {
        self.data
    }
}
