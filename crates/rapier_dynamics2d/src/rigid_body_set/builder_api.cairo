//! RigidBodyBuilder API split out of `rigid_body_set.cairo`.

use fixed::Fixed;
use glam::Vec2;
use rapier_core::rigid_body::{RigidBodyActivationTrait, RigidBodyChangesTrait, RigidBodyType};
use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use crate::rigid_body::{LockedAxes, RigidBodyMassPropsTrait, RigidBodyPositionTrait};
use super::body_api::RigidBodyTrait;
use super::{
    RigidBody, set_extra_additional_is_mass, set_extra_allow_fast_rotation,
    set_extra_pgs_iterations, set_extra_solver_iterations,
};

/// Upstream-named body builder. Unexposed builder options remain configurable on `build()`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct RigidBodyBuilder {
    body: RigidBody,
}

/// Upstream default builder: dynamic body.
pub impl RigidBodyBuilderDefault of Default<RigidBodyBuilder> {
    #[inline(always)]
    fn default() -> RigidBodyBuilder {
        RigidBodyBuilderTrait::dynamic()
    }
}

/// Builds a rigid body from its builder.
pub impl RigidBodyFromBuilder of Into<RigidBodyBuilder, RigidBody> {
    #[inline(always)]
    fn into(self: RigidBodyBuilder) -> RigidBody {
        self.build()
    }
}

#[generate_trait]
pub impl RigidBodyBuilderImpl of RigidBodyBuilderTrait {
    /// Default body at identity with the supplied type; exact initialization.
    fn new(body_type: RigidBodyType) -> RigidBodyBuilder {
        RigidBodyBuilder { body: RigidBodyTrait::new(body_type, Default::default()) }
    }
    /// Dynamic body at identity, upstream defaults.
    fn dynamic() -> RigidBodyBuilder {
        Self::new(RigidBodyType::Dynamic)
    }
    /// Fixed body at identity, upstream defaults.
    fn fixed() -> RigidBodyBuilder {
        Self::new(RigidBodyType::Fixed)
    }
    /// Deprecated upstream alias for [`fixed`].
    #[inline(always)]
    fn new_static() -> RigidBodyBuilder {
        Self::fixed()
    }
    /// Position-controlled body at identity, upstream defaults.
    fn kinematic_position_based() -> RigidBodyBuilder {
        Self::new(RigidBodyType::KinematicPositionBased)
    }
    /// Deprecated upstream alias for [`kinematic_position_based`].
    #[inline(always)]
    fn new_kinematic_position_based() -> RigidBodyBuilder {
        Self::kinematic_position_based()
    }
    /// Velocity-controlled body at identity, upstream defaults.
    fn kinematic_velocity_based() -> RigidBodyBuilder {
        Self::new(RigidBodyType::KinematicVelocityBased)
    }
    /// Deprecated upstream alias for [`kinematic_velocity_based`].
    #[inline(always)]
    fn new_kinematic_velocity_based() -> RigidBodyBuilder {
        Self::kinematic_velocity_based()
    }
    /// Initial pose, unit rotation required. Refreshes COM; fixed arithmetic overflow panics.
    fn position(mut self: RigidBodyBuilder, position: Pose2) -> RigidBodyBuilder {
        self.body.pos = RigidBodyPositionTrait::from_position(position);
        self
            .body
            .mprops = self
            .body
            .mprops
            .update_world_mass_properties(self.body.body_type, position);
        self
    }
    /// Initial pose alias.
    #[inline(always)]
    fn pose(self: RigidBodyBuilder, position: Pose2) -> RigidBodyBuilder {
        self.position(position)
    }
    /// Initial translation.
    fn translation(mut self: RigidBodyBuilder, translation: Vec2) -> RigidBodyBuilder {
        self.body.set_translation(translation, false);
        self.body.changes = RigidBodyChangesTrait::empty();
        self
    }
    /// Initial rotation.
    fn rotation(mut self: RigidBodyBuilder, rotation: Rot2) -> RigidBodyBuilder {
        self.body.set_rotation(rotation, false);
        self.body.changes = RigidBodyChangesTrait::empty();
        self
    }
    /// Initial linear velocity.
    fn linvel(mut self: RigidBodyBuilder, linvel: Vec2) -> RigidBodyBuilder {
        self.body.vels.linvel = linvel;
        self
    }
    /// Initial angular velocity.
    fn angvel(mut self: RigidBodyBuilder, angvel: Fixed) -> RigidBodyBuilder {
        self.body.vels.angvel = angvel;
        self
    }
    /// Initial linear damping.
    fn linear_damping(mut self: RigidBodyBuilder, factor: Fixed) -> RigidBodyBuilder {
        self.body.damping.linear_damping = factor;
        self
    }
    /// Initial angular damping.
    fn angular_damping(mut self: RigidBodyBuilder, factor: Fixed) -> RigidBodyBuilder {
        self.body.damping.angular_damping = factor;
        self
    }
    /// Initial gravity scale.
    fn gravity_scale(mut self: RigidBodyBuilder, scale: Fixed) -> RigidBodyBuilder {
        self.body.forces.gravity_scale = scale;
        self
    }
    /// Initial configured dominance, exactly in [-128,127]; no rounding or panic.
    fn dominance_group(mut self: RigidBodyBuilder, group: i8) -> RigidBodyBuilder {
        self.body.dominance.group = group;
        self
    }
    /// Initial enabled flag.
    fn enabled(mut self: RigidBodyBuilder, enabled: bool) -> RigidBodyBuilder {
        self.body.enabled = enabled;
        self
    }
    /// Initial user data.
    fn user_data(mut self: RigidBodyBuilder, data: u128) -> RigidBodyBuilder {
        self.body.user_data = data;
        self
    }
    /// Initial additional solver iterations.
    fn additional_solver_iterations(
        mut self: RigidBodyBuilder, additional_iterations: u32,
    ) -> RigidBodyBuilder {
        self
            .body
            .solver_flags =
                set_extra_solver_iterations(self.body.solver_flags, additional_iterations);
        self
    }
    /// Initial additional PGS iterations.
    fn additional_pgs_iterations(
        mut self: RigidBodyBuilder, additional_iterations: u32,
    ) -> RigidBodyBuilder {
        self
            .body
            .solver_flags = set_extra_pgs_iterations(self.body.solver_flags, additional_iterations);
        self
    }
    /// Initial locked axes.
    fn locked_axes(mut self: RigidBodyBuilder, locked_axes: LockedAxes) -> RigidBodyBuilder {
        self.body.set_locked_axes(locked_axes, false);
        self
    }
    /// Locks all translations.
    fn lock_translations(mut self: RigidBodyBuilder) -> RigidBodyBuilder {
        self.body.lock_translations(true, false);
        self
    }
    /// Enables translations per axis.
    fn enabled_translations(
        mut self: RigidBodyBuilder, allow_translation_x: bool, allow_translation_y: bool,
    ) -> RigidBodyBuilder {
        self.body.set_enabled_translations(allow_translation_x, allow_translation_y, false);
        self
    }
    /// Deprecated upstream alias.
    #[inline(always)]
    fn restrict_translations(
        self: RigidBodyBuilder, allow_translation_x: bool, allow_translation_y: bool,
    ) -> RigidBodyBuilder {
        self.enabled_translations(allow_translation_x, allow_translation_y)
    }
    /// Locks the 2D rotation.
    fn lock_rotations(mut self: RigidBodyBuilder) -> RigidBodyBuilder {
        self.body.lock_rotations(true, false);
        self
    }
    /// 2D rotation enable switch.
    fn enabled_rotations(mut self: RigidBodyBuilder, allow_rotations: bool) -> RigidBodyBuilder {
        self.body.set_enabled_rotations(allow_rotations, false);
        self
    }
    /// Deprecated upstream alias.
    #[inline(always)]
    fn restrict_rotations(self: RigidBodyBuilder, allow_rotations: bool) -> RigidBodyBuilder {
        self.enabled_rotations(allow_rotations)
    }
    /// Initial additional mass properties.
    fn additional_mass_properties(
        mut self: RigidBodyBuilder, mprops: MassProperties,
    ) -> RigidBodyBuilder {
        self.body.mprops.additional_local_mprops = mprops;
        self.body.solver_flags = set_extra_additional_is_mass(self.body.solver_flags, false);
        self
    }
    /// Initial additional mass.
    fn additional_mass(mut self: RigidBodyBuilder, mass: Fixed) -> RigidBodyBuilder {
        let mut mprops: MassProperties = Default::default();
        mprops.set_mass(mass, true);
        self.body.mprops.additional_local_mprops = mprops;
        self.body.solver_flags = set_extra_additional_is_mass(self.body.solver_flags, true);
        self
    }
    /// Can the body sleep?
    fn can_sleep(mut self: RigidBodyBuilder, can_sleep: bool) -> RigidBodyBuilder {
        if can_sleep {
            self.body.activation = RigidBodyActivationTrait::active();
        } else {
            self.body.activation = RigidBodyActivationTrait::cannot_sleep();
        }
        self
    }
    /// Initial sleeping state.
    fn sleeping(mut self: RigidBodyBuilder, sleeping: bool) -> RigidBodyBuilder {
        if sleeping {
            self.body.sleep();
        } else {
            self.body.activation.sleeping = false;
        }
        self
    }
    /// Initial fast-rotation switch.
    fn allow_fast_rotation(mut self: RigidBodyBuilder, allow: bool) -> RigidBodyBuilder {
        self.body.solver_flags = set_extra_allow_fast_rotation(self.body.solver_flags, allow);
        self
    }
    /// 3D-only upstream switch, ignored in 2D.
    #[inline(always)]
    fn gyroscopic_forces_enabled(self: RigidBodyBuilder, enabled: bool) -> RigidBodyBuilder {
        self
    }
    /// Builds the configured body, an exact copy.
    fn build(self: RigidBodyBuilder) -> RigidBody {
        self.body
    }
}
