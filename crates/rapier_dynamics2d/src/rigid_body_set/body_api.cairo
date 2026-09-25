use fixed::{Fixed, ZERO};
use glam::{Vec2, Vec2Trait};
use rapier_core::Handle;
use rapier_core::rigid_body::changes::{
    DOMINANCE, ENABLED_OR_DISABLED, LOCAL_MASS_PROPERTIES, POSITION, SLEEP, TYPE,
};
use rapier_core::rigid_body::{
    RigidBodyActivation, RigidBodyActivationTrait, RigidBodyChangesTrait, RigidBodyDominanceTrait,
    RigidBodyType, RigidBodyTypeTrait,
};
use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
use rapier_math::math_ext::vec2::gcross_vv;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use crate::collider_set::ColliderSet;
use crate::rigid_body::{
    LockedAxes, LockedAxesTrait, ROTATION_LOCKED, RigidBodyForcesTrait, RigidBodyMassProps,
    RigidBodyMassPropsTrait, RigidBodyPositionTrait, RigidBodyVelocity, RigidBodyVelocityTrait,
    TRANSLATION_LOCKED, TRANSLATION_LOCKED_X, TRANSLATION_LOCKED_Y,
};
use super::{
    RigidBody, extra_additional_is_mass, extra_allow_fast_rotation, extra_pgs_iterations,
    extra_solver_iterations, recompute_body_mass_properties, set_extra_additional_is_mass,
    set_extra_allow_fast_rotation, set_extra_pgs_iterations, set_extra_solver_iterations,
};

#[generate_trait]
pub impl RigidBodyImpl of RigidBodyTrait {
    /// A body of type `body_type` at `position`, at rest, with no collider, no mass (the
    /// colliders bring it), zero damping, unit gravity scale, awake, dominance group 0, enabled
    /// and no change flag (the set raises them all on `insert`). Upstream `RigidBodyBuilder`
    /// defaults.
    fn new(body_type: RigidBodyType, position: Pose2) -> RigidBody {
        RigidBody {
            pos: RigidBodyPositionTrait::from_position(position),
            mprops: RigidBodyMassPropsTrait::from_local(
                Default::default(), LockedAxesTrait::empty(),
            )
                .update_world_mass_properties(body_type, position),
            vels: Default::default(),
            damping: Default::default(),
            forces: Default::default(),
            colliders: array![].span(),
            activation: RigidBodyActivationTrait::active(),
            changes: RigidBodyChangesTrait::empty(),
            body_type,
            dominance: Default::default(),
            enabled: true,
            solver_flags: 0,
            user_data: 0,
        }
    }

    /// `new(Dynamic, position)`.
    #[inline(always)]
    fn dynamic(position: Pose2) -> RigidBody {
        Self::new(RigidBodyType::Dynamic, position)
    }

    /// `new(Fixed, position)`.
    #[inline(always)]
    fn fixed(position: Pose2) -> RigidBody {
        Self::new(RigidBodyType::Fixed, position)
    }

    /// `new(KinematicPositionBased, position)`.
    #[inline(always)]
    fn kinematic_position_based(position: Pose2) -> RigidBody {
        Self::new(RigidBodyType::KinematicPositionBased, position)
    }

    /// `new(KinematicVelocityBased, position)`: velocities drive motion, forces and
    /// contact impulses cannot change it. Exact initialization; no rounding or panic.
    fn kinematic_velocity_based(position: Pose2) -> RigidBody {
        Self::new(RigidBodyType::KinematicVelocityBased, position)
    }

    /// Target pose (upstream `next_position`), exact copy.
    fn next_position(self: @RigidBody) -> Pose2 {
        *self.pos.next_position
    }

    /// Sets a kinematic body's target, waking it strongly iff different from the current
    /// pose. Other body types are untouched. Exact copy; rotation must be unit.
    /// As upstream, accepts both kinematic types; velocity-based integration replaces it.
    fn set_next_kinematic_position(ref self: RigidBody, position: Pose2) {
        if self.is_kinematic() {
            self.pos.next_position = position;
            if self.pos.position != position {
                self.wake_up(true);
            }
        }
    }

    /// Replaces only target translation; same type and wake rules as the pose setter.
    /// Values must fit Q32.32; exact copy, no arithmetic or panic.
    fn set_next_kinematic_translation(ref self: RigidBody, translation: Vec2) {
        if self.is_kinematic() {
            self.pos.next_position.translation = translation;
            if self.pos.position.translation != translation {
                self.wake_up(true);
            }
        }
    }

    /// Replaces only target rotation (unit complex); same type and wake rules as upstream.
    /// Exact copy, no normalization, arithmetic or panic.
    fn set_next_kinematic_rotation(ref self: RigidBody, rotation: Rot2) {
        if self.is_kinematic() {
            self.pos.next_position.rotation = rotation;
            if self.pos.position.rotation != rotation {
                self.wake_up(true);
            }
        }
    }

    /// Configured dominance group, exactly in [-128, 127]. Fixed effective dominance is 128.
    fn dominance_group(self: @RigidBody) -> i8 {
        *self.dominance.group
    }

    /// Replaces the signed group, raising DOMINANCE iff changed. No direct wake-up, as
    /// upstream; the pipeline handles affected contacts. Exact, no rounding or panic.
    fn set_dominance_group(ref self: RigidBody, group: i8) {
        if self.dominance.group != group {
            self.dominance.group = group;
            self.changes.insert(DOMINANCE);
        }
    }

    /// Effective dominance group after fixed-body promotion.
    #[inline(always)]
    fn effective_dominance_group(self: @RigidBody) -> i16 {
        (*self.dominance).effective_group(*self.body_type)
    }

    /// Solver active-set offset. Persistent active sets are rebuilt in this port.
    #[inline(always)]
    fn effective_active_set_offset(self: @RigidBody) -> u32 {
        0xffffffff
    }

    /// Extra solver substeps requested by this body.
    #[inline(always)]
    fn additional_solver_iterations(self: @RigidBody) -> u32 {
        extra_solver_iterations(*self.solver_flags)
    }

    /// Sets extra solver substeps.
    #[inline(always)]
    fn set_additional_solver_iterations(ref self: RigidBody, additional_iterations: u32) {
        self.solver_flags = set_extra_solver_iterations(self.solver_flags, additional_iterations);
    }

    /// Extra PGS iterations requested by this body.
    #[inline(always)]
    fn additional_pgs_iterations(self: @RigidBody) -> u32 {
        extra_pgs_iterations(*self.solver_flags)
    }

    /// Sets extra PGS iterations.
    #[inline(always)]
    fn set_additional_pgs_iterations(ref self: RigidBody, additional_iterations: u32) {
        self.solver_flags = set_extra_pgs_iterations(self.solver_flags, additional_iterations);
    }

    /// World pose of the body frame.
    #[inline(always)]
    fn position(self: @RigidBody) -> Pose2 {
        *self.pos.position
    }

    /// World centre of mass (refreshed by `update_world_mass_properties`).
    #[inline(always)]
    fn world_com(self: @RigidBody) -> Vec2 {
        *self.mprops.world_com
    }

    /// World centre of mass.
    #[inline(always)]
    fn center_of_mass(self: @RigidBody) -> Vec2 {
        *self.mprops.world_com
    }

    /// Local centre of mass.
    #[inline(always)]
    fn local_center_of_mass(self: @RigidBody) -> Vec2 {
        *self.mprops.local_mprops.local_com
    }

    /// Mass properties component.
    #[inline(always)]
    fn mass_properties(self: @RigidBody) -> RigidBodyMassProps {
        *self.mprops
    }

    /// Local mass.
    #[inline(always)]
    fn mass(self: @RigidBody) -> Fixed {
        (*self.mprops).mass()
    }

    /// Locked movement axes.
    #[inline(always)]
    fn locked_axes(self: @RigidBody) -> LockedAxes {
        *self.mprops.flags
    }

    /// Are all translations locked?
    #[inline(always)]
    fn is_translation_locked(self: @RigidBody) -> bool {
        self.mprops.flags.contains(TRANSLATION_LOCKED)
    }

    /// Is the 2D rotation locked?
    #[inline(always)]
    fn is_rotation_locked(self: @RigidBody) -> bool {
        self.mprops.flags.contains(ROTATION_LOCKED)
    }

    /// Sets the locked axes and refreshes effective mass properties.
    fn set_locked_axes(ref self: RigidBody, locked_axes: LockedAxes, wake_up: bool) {
        if self.mprops.flags != locked_axes {
            if self.is_dynamic_or_kinematic() && wake_up {
                self.wake_up(true);
            }
            self.changes.insert(LOCAL_MASS_PROPERTIES);
            self.mprops.flags = locked_axes;
            self
                .mprops = self
                .mprops
                .update_world_mass_properties(self.body_type, self.pos.position);
        }
    }

    /// Locks or unlocks all translations.
    fn lock_translations(ref self: RigidBody, locked: bool, wake_up: bool) {
        let mut flags = self.mprops.flags;
        if locked {
            flags.insert(TRANSLATION_LOCKED);
        } else {
            flags.remove(TRANSLATION_LOCKED);
        }
        self.set_locked_axes(flags, wake_up);
    }

    /// Locks or unlocks the 2D rotation.
    fn lock_rotations(ref self: RigidBody, locked: bool, wake_up: bool) {
        let mut flags = self.mprops.flags;
        if locked {
            flags.insert(ROTATION_LOCKED);
        } else {
            flags.remove(ROTATION_LOCKED);
        }
        self.set_locked_axes(flags, wake_up);
    }

    /// Enables translations per 2D axis.
    fn set_enabled_translations(
        ref self: RigidBody, allow_translation_x: bool, allow_translation_y: bool, wake_up: bool,
    ) {
        let mut flags = self.mprops.flags;
        if allow_translation_x {
            flags.remove(TRANSLATION_LOCKED_X);
        } else {
            flags.insert(TRANSLATION_LOCKED_X);
        }
        if allow_translation_y {
            flags.remove(TRANSLATION_LOCKED_Y);
        } else {
            flags.insert(TRANSLATION_LOCKED_Y);
        }
        self.set_locked_axes(flags, wake_up);
    }

    /// Deprecated upstream alias for [`set_enabled_translations`].
    #[inline(always)]
    fn restrict_translations(
        ref self: RigidBody, allow_translation_x: bool, allow_translation_y: bool, wake_up: bool,
    ) {
        self.set_enabled_translations(allow_translation_x, allow_translation_y, wake_up);
    }

    /// Enables or disables the 2D rotation.
    fn set_enabled_rotations(ref self: RigidBody, allow_rotations: bool, wake_up: bool) {
        self.lock_rotations(!allow_rotations, wake_up);
    }

    /// Deprecated upstream alias for [`set_enabled_rotations`].
    #[inline(always)]
    fn restrict_rotations(ref self: RigidBody, allow_rotations: bool, wake_up: bool) {
        self.set_enabled_rotations(allow_rotations, wake_up);
    }

    #[inline(always)]
    fn is_dynamic(self: @RigidBody) -> bool {
        (*self.body_type).is_dynamic()
    }

    #[inline(always)]
    fn is_fixed(self: @RigidBody) -> bool {
        (*self.body_type).is_fixed()
    }

    #[inline(always)]
    fn is_kinematic(self: @RigidBody) -> bool {
        (*self.body_type).is_kinematic()
    }

    /// Is this body dynamic or kinematic?
    #[inline(always)]
    fn is_dynamic_or_kinematic(self: @RigidBody) -> bool {
        (*self.body_type).is_dynamic_or_kinematic()
    }

    /// Body type.
    #[inline(always)]
    fn body_type(self: @RigidBody) -> RigidBodyType {
        *self.body_type
    }

    /// Sets the body type and refreshes effective mass properties.
    fn set_body_type(ref self: RigidBody, body_type: RigidBodyType, wake_up: bool) {
        if self.body_type != body_type {
            self.changes.insert(TYPE);
            self.body_type = body_type;
            if body_type == RigidBodyType::Fixed {
                self.vels = RigidBodyVelocityTrait::zero();
            }
            self
                .mprops = self
                .mprops
                .update_world_mass_properties(self.body_type, self.pos.position);
            if self.is_dynamic_or_kinematic() && wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Moves the body (and, after `propagate_modified_body_positions_to_colliders`, its
    /// colliders). Raises `POSITION` when the pose changes; sets both the current and the next
    /// pose, refreshes the world centre of mass and wakes the body up (upstream
    /// `set_position(pos, wake_up = true)`).
    fn set_position(ref self: RigidBody, position: Pose2) {
        if self.pos.position != position {
            self.changes.insert(POSITION);
        }
        self.pos = RigidBodyPositionTrait::from_position(position);
        self.mprops = self.mprops.update_world_mass_properties(self.body_type, position);
        self.wake_up(true);
    }

    /// Translation of the body frame.
    #[inline(always)]
    fn translation(self: @RigidBody) -> Vec2 {
        *self.pos.position.translation
    }

    /// Rotation of the body frame.
    #[inline(always)]
    fn rotation(self: @RigidBody) -> Rot2 {
        *self.pos.position.rotation
    }

    /// Sets only the translation, waking when it changes.
    fn set_translation(ref self: RigidBody, translation: Vec2, wake_up: bool) {
        if self.pos.position.translation != translation
            || self.pos.next_position.translation != translation {
            self.changes.insert(POSITION);
            self.pos.position.translation = translation;
            self.pos.next_position.translation = translation;
            self
                .mprops = self
                .mprops
                .update_world_mass_properties(self.body_type, self.pos.position);
            if self.is_dynamic_or_kinematic() && wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Sets only the rotation, waking when it changes.
    fn set_rotation(ref self: RigidBody, rotation: Rot2, wake_up: bool) {
        if self.pos.position.rotation != rotation || self.pos.next_position.rotation != rotation {
            self.changes.insert(POSITION);
            self.pos.position.rotation = rotation;
            self.pos.next_position.rotation = rotation;
            self
                .mprops = self
                .mprops
                .update_world_mass_properties(self.body_type, self.pos.position);
            if self.is_dynamic_or_kinematic() && wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Linear velocity.
    #[inline(always)]
    fn linvel(self: @RigidBody) -> Vec2 {
        *self.vels.linvel
    }

    /// Angular velocity.
    #[inline(always)]
    fn angvel(self: @RigidBody) -> Fixed {
        *self.vels.angvel
    }

    /// Linear and angular velocity component.
    #[inline(always)]
    fn vels(self: @RigidBody) -> RigidBodyVelocity {
        *self.vels
    }

    /// Sets both velocities through upstream's per-component guards.
    #[inline(always)]
    fn set_vels(ref self: RigidBody, vels: RigidBodyVelocity, wake_up: bool) {
        self.set_linvel(vels.linvel);
        if !wake_up && self.activation.sleeping {
            self.activation.sleeping = true;
        }
        self.set_angvel(vels.angvel);
    }

    /// Replaces the linear velocity and wakes the body up (upstream `set_linvel(linvel, wake_up
    /// = true)`, dynamic and velocity-based kinematic bodies only).
    #[inline(always)]
    fn set_linvel(ref self: RigidBody, linvel: Vec2) {
        if (self.body_type == RigidBodyType::Dynamic
            || self.body_type == RigidBodyType::KinematicVelocityBased)
            && self.vels.linvel != linvel {
            self.vels.linvel = linvel;
            self.wake_up(true);
        }
    }

    /// Replaces the angular velocity and wakes the body up (upstream `set_angvel(angvel, wake_up
    /// = true)`, dynamic and velocity-based kinematic bodies only).
    #[inline(always)]
    fn set_angvel(ref self: RigidBody, angvel: Fixed) {
        if (self.body_type == RigidBodyType::Dynamic
            || self.body_type == RigidBodyType::KinematicVelocityBased)
            && self.vels.angvel != angvel {
            self.vels.angvel = angvel;
            self.wake_up(true);
        }
    }

    /// Is the body asleep (upstream `is_sleeping`)? A sleeping body keeps its pose, has zero
    /// velocities and is skipped by the step until something wakes it up.
    #[inline(always)]
    fn is_sleeping(self: @RigidBody) -> bool {
        *self.activation.sleeping
    }

    /// Returns true when either velocity component is non-zero.
    #[inline(always)]
    fn is_moving(self: @RigidBody) -> bool {
        !(*self.vels).is_zero()
    }

    /// Activation component, copied.
    #[inline(always)]
    fn activation(self: @RigidBody) -> RigidBodyActivation {
        *self.activation
    }

    /// Activation component for mutation; marks sleep state dirty.
    #[inline(always)]
    fn activation_mut(ref self: RigidBody) -> RigidBodyActivation {
        self.changes.insert(SLEEP);
        self.activation
    }

    /// Is this body enabled?
    #[inline(always)]
    fn is_enabled(self: @RigidBody) -> bool {
        *self.enabled
    }

    /// Enables or disables this body, raising the upstream change flag.
    fn set_enabled(ref self: RigidBody, enabled: bool) {
        if self.enabled != enabled {
            if enabled {
                self.changes = RigidBodyChangesTrait::all();
            } else {
                self.changes.insert(ENABLED_OR_DISABLED);
            }
            self.enabled = enabled;
        }
    }

    /// Wakes the body up (upstream `RigidBody::wake_up`): raises `SLEEP` when it was asleep and
    /// clears the sleeping flag; a `strong` wake-up also resets the still-time counter, so that
    /// the body cannot fall asleep again for `time_until_sleep`. The step then wakes the whole
    /// island of the body (touching bodies, jointed bodies), as upstream's island manager does.
    #[inline(always)]
    fn wake_up(ref self: RigidBody, strong: bool) {
        if self.activation.sleeping {
            self.changes.insert(SLEEP);
        }
        self.activation.wake_up(strong);
    }

    /// Puts the body to sleep (upstream `RigidBody::sleep`): sleeping flag set, still-time
    /// counter filled, both velocities zeroed. The step wakes it again at once when its island
    /// has an awake body (upstream keeps such a body in the awake island).
    #[inline(always)]
    fn sleep(ref self: RigidBody) {
        self.activation.sleep();
        self.vels = RigidBodyVelocityTrait::zero();
    }

    /// Adds `force` to the user force of a dynamic body (upstream `add_force`): kept across
    /// steps until `reset_forces`. Nothing happens for a zero force or a non-dynamic body; the
    /// body is woken up (strongly) when `wake_up` and the force was added.
    fn add_force(ref self: RigidBody, force: Vec2, wake_up: bool) {
        if force != Vec2Trait::ZERO && self.body_type.is_dynamic() {
            self.forces = self.forces.add_force(force);
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Adds `torque` to the user torque of a dynamic body (upstream `add_torque`); as
    /// [`add_force`](RigidBodyTrait::add_force) for the zero and body-type conditions.
    fn add_torque(ref self: RigidBody, torque: Fixed, wake_up: bool) {
        if torque != ZERO && self.body_type.is_dynamic() {
            self.forces = self.forces.add_torque(torque);
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Adds `force` applied at the world `point` (upstream `add_force_at_point`): the force and
    /// its torque about the world centre of mass; as [`add_force`](RigidBodyTrait::add_force)
    /// otherwise.
    fn add_force_at_point(ref self: RigidBody, force: Vec2, point: Vec2, wake_up: bool) {
        if force != Vec2Trait::ZERO && self.body_type.is_dynamic() {
            self.forces = self.forces.add_force_at_point(self.mprops, force, point);
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Clears the user force (upstream `reset_forces`); wakes the body up when `wake_up` and
    /// the force was not zero already.
    fn reset_forces(ref self: RigidBody, wake_up: bool) {
        if self.forces.user_force != Vec2Trait::ZERO {
            self.forces = self.forces.reset_forces();
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Clears the user torque (upstream `reset_torques`); wakes the body up when `wake_up` and
    /// the torque was not zero already.
    fn reset_torques(ref self: RigidBody, wake_up: bool) {
        if self.forces.user_torque != ZERO {
            self.forces = self.forces.reset_torques();
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// User force, zero for non-dynamic bodies.
    #[inline(always)]
    fn user_force(self: @RigidBody) -> Vec2 {
        if (*self.body_type).is_dynamic() {
            *self.forces.user_force
        } else {
            Vec2Trait::ZERO
        }
    }

    /// User torque, zero for non-dynamic bodies.
    #[inline(always)]
    fn user_torque(self: @RigidBody) -> Fixed {
        if (*self.body_type).is_dynamic() {
            *self.forces.user_torque
        } else {
            ZERO
        }
    }

    /// Applies a linear impulse at the centre of mass of a dynamic body (upstream
    /// `apply_impulse`): `linvel += impulse * effective_inv_mass`, each component floored once.
    /// Nothing happens for a zero impulse or a non-dynamic body; the body is woken up (strongly)
    /// when `wake_up` and the impulse was applied.
    fn apply_impulse(ref self: RigidBody, impulse: Vec2, wake_up: bool) {
        if impulse != Vec2Trait::ZERO && self.body_type.is_dynamic() {
            self.vels.linvel = self.vels.linvel + impulse * self.mprops.effective_inv_mass;
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies an angular impulse to a dynamic body (upstream `apply_torque_impulse`): `angvel
    /// += effective_world_inv_inertia * torque_impulse`, floored once; as
    /// [`apply_impulse`](RigidBodyTrait::apply_impulse) for the conditions.
    fn apply_torque_impulse(ref self: RigidBody, torque_impulse: Fixed, wake_up: bool) {
        if torque_impulse != ZERO && self.body_type.is_dynamic() {
            self.vels.angvel += self.mprops.effective_world_inv_inertia * torque_impulse;
            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies `impulse` at the world `point` (upstream `apply_impulse_at_point`): the linear
    /// impulse, then the angular impulse `(point - world_com) x impulse`, each through the
    /// helpers above (and their conditions).
    fn apply_impulse_at_point(ref self: RigidBody, impulse: Vec2, point: Vec2, wake_up: bool) {
        let dpt = point - self.mprops.world_com;
        let torque_impulse = gcross_vv(dpt.x, dpt.y, impulse.x, impulse.y);
        self.apply_impulse(impulse, wake_up);
        self.apply_torque_impulse(torque_impulse, wake_up);
    }

    /// Velocity at a world-space point.
    #[inline(always)]
    fn velocity_at_point(self: @RigidBody, point: Vec2) -> Vec2 {
        (*self.vels).velocity_at_point(point, *self.mprops.world_com)
    }

    /// Kinetic energy.
    #[inline(always)]
    fn kinetic_energy(self: @RigidBody) -> Fixed {
        (*self.vels).kinetic_energy(*self.mprops)
    }

    /// Gravitational potential energy, leap-frog aligned with kinetic energy.
    fn gravitational_potential_energy(self: @RigidBody, dt: Fixed, gravity: Vec2) -> Fixed {
        let world_com = (*self.mprops.local_mprops).world_com(*self.pos.position)
            - self.vels.linvel.mul_scalar(dt * fixed::HALF);
        -(*self.mprops).mass()
            * *self.forces.gravity_scale
            * (gravity.x * world_com.x + gravity.y * world_com.y)
    }

    /// Predicts the pose after `dt` using velocities only.
    #[inline(always)]
    fn predict_position_using_velocity(self: @RigidBody, dt: Fixed) -> Pose2 {
        (*self.pos).predict_position_using_velocity(dt, *self.vels, *self.mprops)
    }

    /// Predicts the pose after `dt` using forces then velocities.
    #[inline(always)]
    fn predict_position_using_velocity_and_forces(self: @RigidBody, dt: Fixed) -> Pose2 {
        (*self.pos)
            .predict_position_using_velocity_and_forces(dt, *self.forces, *self.vels, *self.mprops)
    }

    /// Linear damping.
    #[inline(always)]
    fn linear_damping(self: @RigidBody) -> Fixed {
        *self.damping.linear_damping
    }

    /// Sets linear damping.
    #[inline(always)]
    fn set_linear_damping(ref self: RigidBody, damping: Fixed) {
        self.damping.linear_damping = damping;
    }

    /// Angular damping.
    #[inline(always)]
    fn angular_damping(self: @RigidBody) -> Fixed {
        *self.damping.angular_damping
    }

    /// Sets angular damping.
    #[inline(always)]
    fn set_angular_damping(ref self: RigidBody, damping: Fixed) {
        self.damping.angular_damping = damping;
    }

    /// Gravity multiplier.
    #[inline(always)]
    fn gravity_scale(self: @RigidBody) -> Fixed {
        *self.forces.gravity_scale
    }

    /// Sets gravity multiplier and optionally wakes a sleeping body.
    fn set_gravity_scale(ref self: RigidBody, scale: Fixed, wake_up: bool) {
        if self.forces.gravity_scale != scale {
            if wake_up && self.activation.sleeping {
                self.changes.insert(SLEEP);
                self.activation.sleeping = false;
            }
            self.forces.gravity_scale = scale;
        }
    }

    /// Returns whether fast rotation is allowed.
    #[inline(always)]
    fn is_fast_rotation_allowed(self: @RigidBody) -> bool {
        extra_allow_fast_rotation(*self.solver_flags)
    }

    /// Allows or disallows fast rotation.
    #[inline(always)]
    fn set_allow_fast_rotation(ref self: RigidBody, allow: bool) {
        self.solver_flags = set_extra_allow_fast_rotation(self.solver_flags, allow);
    }

    /// Gyroscopic forces are 3D-only; always disabled in 2D.
    #[inline(always)]
    fn gyroscopic_forces_enabled(self: @RigidBody) -> bool {
        false
    }

    /// Gyroscopic forces are 3D-only; ignored in 2D.
    #[inline(always)]
    fn enable_gyroscopic_forces(ref self: RigidBody, enabled: bool) {}

    /// 2D angular velocity is unaffected by 3D gyroscopic correction.
    #[inline(always)]
    fn angvel_with_gyroscopic_forces(self: @RigidBody, dt: Fixed) -> Fixed {
        *self.vels.angvel
    }

    /// Soft bodies are out of scope; regular rigid bodies have no cluster.
    #[inline(always)]
    fn soft_cluster(self: @RigidBody) -> Option<u32> {
        None
    }

    /// Soft-frame bodies are out of scope.
    #[inline(always)]
    fn is_soft_frame(self: @RigidBody) -> bool {
        false
    }

    /// Attached colliders in attachment order.
    #[inline(always)]
    fn colliders(self: @RigidBody) -> Span<Handle> {
        *self.colliders
    }

    /// Copies public state except the collider list, raising all change flags.
    fn copy_from(ref self: RigidBody, other: RigidBody) {
        let colliders = self.colliders;
        self.pos = other.pos;
        self.mprops = other.mprops;
        self.vels = other.vels;
        self.damping = other.damping;
        self.forces = other.forces;
        self.activation = other.activation;
        self.body_type = other.body_type;
        self.dominance = other.dominance;
        self.enabled = other.enabled;
        self.solver_flags = other.solver_flags;
        self.user_data = other.user_data;
        self.colliders = colliders;
        self.changes = RigidBodyChangesTrait::all();
    }

    /// Sets additional mass and marks local mass properties for recomputation.
    fn set_additional_mass(ref self: RigidBody, additional_mass: Fixed, wake_up: bool) {
        let mut props: MassProperties = Default::default();
        props.set_mass(additional_mass, true);
        self.do_set_additional_mass_properties(props, true, wake_up);
    }

    /// Sets additional mass properties and marks local mass properties for recomputation.
    fn set_additional_mass_properties(ref self: RigidBody, props: MassProperties, wake_up: bool) {
        self.do_set_additional_mass_properties(props, false, wake_up);
    }

    /// Internal additional-mass setter.
    fn do_set_additional_mass_properties(
        ref self: RigidBody, props: MassProperties, is_mass: bool, wake_up: bool,
    ) {
        if self.mprops.additional_local_mprops != props
            || extra_additional_is_mass(self.solver_flags) != is_mass {
            self.changes.insert(LOCAL_MASS_PROPERTIES);
            self.mprops.additional_local_mprops = props;
            self.solver_flags = set_extra_additional_is_mass(self.solver_flags, is_mass);
            if self.is_dynamic_or_kinematic() && wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Recomputes local mass properties from attached colliders and additional mass.
    fn recompute_mass_properties_from_colliders(ref self: RigidBody, ref colliders: ColliderSet) {
        recompute_body_mass_properties(ref self, ref colliders);
    }
}
