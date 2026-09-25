//! The collider (upstream `geometry/collider.rs`, `Collider`): a shape, a pose, a mass
//! specification and the scalar components of `rapier_core::collider`.
//!
//! Setters mirror upstream's change tracking: each raises its `ColliderChanges` bit, and those
//! that upstream guards with an equality test (sensor flag, groups, mass specification, enabled
//! flag) raise it only when the value actually changes. Getters and setters are cold paths:
//! plain field accesses, no candidates.

use fixed::{Fixed, HALF, ONE};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::changes::{
    ENABLED_OR_DISABLED, GROUPS, LOCAL_MASS_PROPERTIES, PARENT, POSITION, SHAPE, TYPE,
};
use rapier_core::collider::{
    ActiveCollisionTypes, ActiveEvents, ActiveHooks, CoefficientCombineRule, ColliderChanges,
    ColliderChangesTrait, ColliderEnabled, ColliderFlags, ColliderMaterial, ColliderType,
    ColliderTypeTrait,
};
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_core::interaction_groups::InteractionGroups;
use rapier_geometry2d::aabb::{Aabb, AabbTrait};
use rapier_geometry2d::mass::{MassProperties, MassPropertiesTrait};
use rapier_geometry2d::shape::{Shape, ShapeTrait};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use crate::collider::components::{BoxedOneWayPlatformPartialEq, BoxedOneWayPlatformSerde};
use super::components::{
    ColliderMassProps, ColliderMassPropsTrait, ColliderParent, ColliderPosition,
};

/// A collision shape with its pose, mass, material and filtering flags.
///
/// `pos` is the world pose; while the collider is attached to a body the collider set keeps it
/// equal to `body pose * parent.pos_wrt_parent`. `ColliderBuilderTrait::build` produces it with
/// `changes = ColliderChanges::all()` and no parent.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct Collider {
    pub co_type: ColliderType,
    pub shape: Shape,
    pub mprops: ColliderMassProps,
    pub changes: ColliderChanges,
    pub parent: Option<ColliderParent>,
    pub pos: ColliderPosition,
    pub material: ColliderMaterial,
    pub flags: ColliderFlags,
    /// Total contact force beyond which a contact force event may be emitted.
    pub contact_force_event_threshold: Fixed,
    /// Optional one-way cone; absent by default. Configured with the builder.
    pub one_way: Box<Option<super::components::OneWayPlatform>>,
    pub user_data: u128,
}

/// Replaces the mass specification, raising `LOCAL_MASS_PROPERTIES` when it differs
/// (upstream `do_set_mass_properties`).
fn set_mprops(ref collider: Collider, mprops: ColliderMassProps) {
    if mprops != collider.mprops {
        collider.changes = collider.changes | LOCAL_MASS_PROPERTIES;
        collider.mprops = mprops;
    }
}

/// `mass / area` of `shape`. Out of line, as `mass_of`, `mprops_mass`, `density_of_mprops` and
/// `components::mass_variant`: inlined in the `match` of its caller, the cost of a computing arm
/// is charged to every arm.
#[inline(never)]
fn density_of(mass: Fixed, shape: Shape) -> Fixed {
    mass * shape.mass_properties(ONE).inv_mass
}

/// `mprops.mass() / area` of `shape`.
#[inline(never)]
fn density_of_mprops(mprops: MassProperties, shape: Shape) -> Fixed {
    mprops.mass() * shape.mass_properties(ONE).inv_mass
}

/// The mass of explicit mass properties: `1 / inv_mass`.
#[inline(never)]
fn mprops_mass(mprops: MassProperties) -> Fixed {
    mprops.mass()
}

/// `density * area` of `shape`, read back from the stored inverse.
#[inline(never)]
fn mass_of(density: Fixed, shape: Shape) -> Fixed {
    shape.mass_properties(density).mass()
}

#[generate_trait]
pub impl ColliderImpl of ColliderTrait {
    /// The handle of the parent body, `None` for a standalone collider.
    #[inline(always)]
    fn parent(self: Collider) -> Option<Handle> {
        match self.parent {
            Some(parent) => Some(parent.handle),
            None => None,
        }
    }

    /// Whether the collider only reports overlaps and produces no contact force.
    #[inline(always)]
    fn is_sensor(self: Collider) -> bool {
        self.co_type.is_sensor()
    }

    /// Turns the collider into a sensor or back into a solid; raises `TYPE` on a change.
    fn set_sensor(ref self: Collider, is_sensor: bool) {
        if is_sensor != self.co_type.is_sensor() {
            self.changes = self.changes | TYPE;
            self.co_type = if is_sensor {
                ColliderType::Sensor
            } else {
                ColliderType::Solid
            };
        }
    }

    /// `true` unless the collider is disabled, by the user or by its parent.
    #[inline(always)]
    fn is_enabled(self: Collider) -> bool {
        self.flags.enabled == ColliderEnabled::Enabled
    }

    /// Enables or disables the collider; raises `ENABLED_OR_DISABLED` on a change. As upstream,
    /// disabling an `Enabled` or `DisabledByParent` collider makes it `Disabled`, enabling a
    /// `Disabled` one makes it `Enabled`, and every other call is a no-op (a collider
    /// `DisabledByParent` stays so until its parent is enabled).
    fn set_enabled(ref self: Collider, enabled: bool) {
        match self.flags.enabled {
            ColliderEnabled::Enabled |
            ColliderEnabled::DisabledByParent => {
                if !enabled {
                    self.changes = self.changes | ENABLED_OR_DISABLED;
                    self.flags.enabled = ColliderEnabled::Disabled;
                }
            },
            ColliderEnabled::Disabled => {
                if enabled {
                    self.changes = self.changes | ENABLED_OR_DISABLED;
                    self.flags.enabled = ColliderEnabled::Enabled;
                }
            },
        }
    }

    /// World pose.
    #[inline(always)]
    fn position(self: Collider) -> Pose2 {
        self.pos.pose
    }

    /// World translation.
    #[inline(always)]
    fn translation(self: Collider) -> Vec2 {
        self.pos.pose.translation
    }

    /// World rotation.
    #[inline(always)]
    fn rotation(self: Collider) -> Rot2 {
        self.pos.pose.rotation
    }

    /// Sets the world pose (standalone colliders); raises `POSITION`.
    #[inline(always)]
    fn set_position(ref self: Collider, position: Pose2) {
        self.changes = self.changes | POSITION;
        self.pos = ColliderPosition { pose: position };
    }

    /// Sets the world translation, keeping the rotation; raises `POSITION`.
    #[inline(always)]
    fn set_translation(ref self: Collider, translation: Vec2) {
        self.changes = self.changes | POSITION;
        self.pos = ColliderPosition { pose: Pose2 { translation, ..self.pos.pose } };
    }

    /// Sets the world rotation (unit), keeping the translation; raises `POSITION`.
    #[inline(always)]
    fn set_rotation(ref self: Collider, rotation: Rot2) {
        self.changes = self.changes | POSITION;
        self.pos = ColliderPosition { pose: Pose2 { rotation, ..self.pos.pose } };
    }

    /// The pose relative to the parent body, `None` for a standalone collider.
    #[inline(always)]
    fn position_wrt_parent(self: Collider) -> Option<Pose2> {
        match self.parent {
            Some(parent) => Some(parent.pos_wrt_parent),
            None => None,
        }
    }

    /// Sets the pose relative to the parent body and raises `PARENT`; does nothing without a
    /// parent.
    fn set_position_wrt_parent(ref self: Collider, pos_wrt_parent: Pose2) {
        if let Some(parent) = self.parent {
            self.changes = self.changes | PARENT;
            self.parent = Some(ColliderParent { pos_wrt_parent, ..parent });
        }
    }

    /// Sets the translation relative to the parent body; as [`Self::set_position_wrt_parent`].
    fn set_translation_wrt_parent(ref self: Collider, translation: Vec2) {
        if let Some(parent) = self.parent {
            self.changes = self.changes | PARENT;
            self
                .parent =
                    Some(
                        ColliderParent {
                            pos_wrt_parent: Pose2 { translation, ..parent.pos_wrt_parent },
                            ..parent,
                        },
                    );
        }
    }

    /// Sets the rotation (unit) relative to the parent body; as [`Self::set_position_wrt_parent`].
    /// Takes a `Rot2` where upstream takes an angle (no trigonometry yet).
    fn set_rotation_wrt_parent(ref self: Collider, rotation: Rot2) {
        if let Some(parent) = self.parent {
            self.changes = self.changes | PARENT;
            self
                .parent =
                    Some(
                        ColliderParent {
                            pos_wrt_parent: Pose2 { rotation, ..parent.pos_wrt_parent }, ..parent,
                        },
                    );
        }
    }

    /// The shape.
    #[inline(always)]
    fn shape(self: Collider) -> Shape {
        self.shape
    }

    /// Replaces the shape; raises `SHAPE`.
    #[inline(always)]
    fn set_shape(ref self: Collider, shape: Shape) {
        self.changes = self.changes | SHAPE;
        self.shape = shape;
    }

    /// The area of the shape: the mass of a unit density.
    /// #### Panics
    /// * `'Fixed: overflow'` as [`ColliderMassPropsTrait::mass_properties`].
    fn volume(self: Collider) -> Fixed {
        self.shape.mass_properties(ONE).mass()
    }

    /// The density: the stored one, or the mass (of the mass specification) over the area. A
    /// shape without area (segment, half-space) has density `0` unless one was stored.
    /// #### Panics
    /// * `'Fixed: overflow'` if the product leaves the scalar range.
    fn density(self: Collider) -> Fixed {
        match self.mprops {
            ColliderMassProps::Density(density) => density,
            ColliderMassProps::Mass(mass) => density_of(mass, self.shape),
            ColliderMassProps::MassProperties(mprops) => density_of_mprops(mprops, self.shape),
        }
    }

    /// The mass: the stored one, or `density * area`.
    /// #### Panics
    /// * `'Fixed: overflow'` as [`ColliderMassPropsTrait::mass_properties`].
    fn mass(self: Collider) -> Fixed {
        match self.mprops {
            ColliderMassProps::Density(density) => mass_of(density, self.shape),
            ColliderMassProps::Mass(mass) => mass,
            ColliderMassProps::MassProperties(mprops) => mprops_mass(mprops),
        }
    }

    /// Specifies the mass by a density, replacing any previous specification; raises
    /// `LOCAL_MASS_PROPERTIES` when it changes.
    fn set_density(ref self: Collider, density: Fixed) {
        set_mprops(ref self, ColliderMassProps::Density(density));
    }

    /// Specifies the mass by a mass (the inertia follows from the shape); as `set_density`.
    fn set_mass(ref self: Collider, mass: Fixed) {
        set_mprops(ref self, ColliderMassProps::Mass(mass));
    }

    /// Specifies explicit mass properties; as `set_density`.
    fn set_mass_properties(ref self: Collider, mass_properties: MassProperties) {
        set_mprops(ref self, ColliderMassProps::MassProperties(mass_properties));
    }

    /// The local mass properties, resolved from the mass specification and the shape.
    /// #### Panics
    /// * `'Fixed: overflow'` as [`ColliderMassPropsTrait::mass_properties`].
    fn mass_properties(self: Collider) -> MassProperties {
        self.mprops.mass_properties(self.shape)
    }

    /// The world AABB of the shape at the collider's pose (no contact skin).
    /// #### Panics
    /// * As `Shape::compute_aabb`.
    #[inline(always)]
    fn compute_aabb(self: Collider) -> Aabb {
        self.shape.compute_aabb(self.pos.pose)
    }

    /// The world AABB loosened by `prediction` on every side (upstream
    /// `compute_collision_aabb`; the port has no contact skin, so the margin is `prediction`).
    /// #### Panics
    /// * As `Shape::compute_aabb`; `'Fixed: overflow'` when a loosened bound leaves the range.
    #[inline(always)]
    fn compute_collision_aabb(self: Collider, prediction: Fixed) -> Aabb {
        self.shape.compute_aabb(self.pos.pose).loosened(prediction)
    }

    /// The AABB the broad phase uses (upstream `compute_broad_phase_aabb`): the collision AABB
    /// for half the prediction distance of `params` (`prediction * 1/2`, rounded as `Fixed`
    /// products, the margin of `ColliderSetTrait::broad_phase_proxies`). Upstream also reads the
    /// parent body for soft CCD, which the port does not have, hence no body-set argument.
    /// #### Panics
    /// * As [`Self::compute_collision_aabb`].
    #[inline(always)]
    fn compute_broad_phase_aabb(self: Collider, params: IntegrationParameters) -> Aabb {
        self.compute_collision_aabb(params.prediction_distance() * HALF)
    }

    /// The smallest AABB containing the shape at the current pose and at `next_position`
    /// (upstream `compute_swept_aabb`, parry's `Shape::compute_swept_aabb`: the merge of the two
    /// world AABBs; the motion in between is not swept). Inlined: outlined, the two shape
    /// `match`es are charged their costliest arm (`alternatives`, 154k against 32k–42k gas).
    /// #### Panics
    /// * As `Shape::compute_aabb`.
    #[inline(always)]
    fn compute_swept_aabb(self: Collider, next_position: Pose2) -> Aabb {
        self.shape.compute_aabb(self.pos.pose).merged(self.shape.compute_aabb(next_position))
    }

    /// Copies every property of `other` into `self` except the parent link (upstream
    /// `copy_from`: it cannot re-parent), and raises every change flag. The world pose is copied
    /// only when `self` has no parent (a parent body drives it otherwise).
    fn copy_from(ref self: Collider, other: Collider) {
        if self.parent.is_none() {
            self.pos = other.pos;
        }
        self.co_type = other.co_type;
        self.shape = other.shape;
        self.mprops = other.mprops;
        self.material = other.material;
        self.contact_force_event_threshold = other.contact_force_event_threshold;
        self.user_data = other.user_data;
        self.flags = other.flags;
        self.one_way = other.one_way;
        self.changes = ColliderChangesTrait::all();
    }

    /// Material of the collider.
    #[inline(always)]
    fn material(self: Collider) -> ColliderMaterial {
        self.material
    }

    #[inline(always)]
    fn friction(self: Collider) -> Fixed {
        self.material.friction
    }

    #[inline(always)]
    fn set_friction(ref self: Collider, coefficient: Fixed) {
        self.material.friction = coefficient;
    }

    #[inline(always)]
    fn friction_combine_rule(self: Collider) -> CoefficientCombineRule {
        self.material.friction_combine_rule
    }

    #[inline(always)]
    fn set_friction_combine_rule(ref self: Collider, rule: CoefficientCombineRule) {
        self.material.friction_combine_rule = rule;
    }

    #[inline(always)]
    fn restitution(self: Collider) -> Fixed {
        self.material.restitution
    }

    #[inline(always)]
    fn set_restitution(ref self: Collider, coefficient: Fixed) {
        self.material.restitution = coefficient;
    }

    #[inline(always)]
    fn restitution_combine_rule(self: Collider) -> CoefficientCombineRule {
        self.material.restitution_combine_rule
    }

    #[inline(always)]
    fn set_restitution_combine_rule(ref self: Collider, rule: CoefficientCombineRule) {
        self.material.restitution_combine_rule = rule;
    }

    #[inline(always)]
    fn collision_groups(self: Collider) -> InteractionGroups {
        self.flags.collision_groups
    }

    /// Replaces the collision groups; raises `GROUPS` when they differ.
    fn set_collision_groups(ref self: Collider, groups: InteractionGroups) {
        if self.flags.collision_groups != groups {
            self.changes = self.changes | GROUPS;
            self.flags.collision_groups = groups;
        }
    }

    #[inline(always)]
    fn solver_groups(self: Collider) -> InteractionGroups {
        self.flags.solver_groups
    }

    /// Replaces the solver groups; raises `GROUPS` when they differ.
    fn set_solver_groups(ref self: Collider, groups: InteractionGroups) {
        if self.flags.solver_groups != groups {
            self.changes = self.changes | GROUPS;
            self.flags.solver_groups = groups;
        }
    }

    #[inline(always)]
    fn active_hooks(self: Collider) -> ActiveHooks {
        self.flags.active_hooks
    }

    #[inline(always)]
    fn set_active_hooks(ref self: Collider, active_hooks: ActiveHooks) {
        self.flags.active_hooks = active_hooks;
    }

    #[inline(always)]
    fn active_events(self: Collider) -> ActiveEvents {
        self.flags.active_events
    }

    #[inline(always)]
    fn set_active_events(ref self: Collider, active_events: ActiveEvents) {
        self.flags.active_events = active_events;
    }

    #[inline(always)]
    fn active_collision_types(self: Collider) -> ActiveCollisionTypes {
        self.flags.active_collision_types
    }

    #[inline(always)]
    fn set_active_collision_types(
        ref self: Collider, active_collision_types: ActiveCollisionTypes,
    ) {
        self.flags.active_collision_types = active_collision_types;
    }

    #[inline(always)]
    fn contact_force_event_threshold(self: Collider) -> Fixed {
        self.contact_force_event_threshold
    }

    #[inline(always)]
    fn set_contact_force_event_threshold(ref self: Collider, threshold: Fixed) {
        self.contact_force_event_threshold = threshold;
    }
}

#[cfg(test)]
pub mod alternatives {
    use rapier_geometry2d::aabb::{Aabb, AabbTrait};
    use rapier_geometry2d::shape::ShapeTrait;
    use rapier_math::pose2::Pose2;
    use super::Collider;

    /// Rejected: `compute_swept_aabb` out of line (every shape arm of both `match`es charged).
    #[inline(never)]
    pub fn compute_swept_aabb_outlined(collider: Collider, next_position: Pose2) -> Aabb {
        collider
            .shape
            .compute_aabb(collider.pos.pose)
            .merged(collider.shape.compute_aabb(next_position))
    }
}

#[cfg(test)]
mod tests;
