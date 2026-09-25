//! The collider (upstream `geometry/collider.rs`, `Collider`): a shape, a pose, a mass
//! specification and the scalar components of `rapier_core::collider`.
//!
//! Setters mirror upstream's change tracking: each raises its `ColliderChanges` bit, and those
//! that upstream guards with an equality test (sensor flag, groups, mass specification, enabled
//! flag) raise it only when the value actually changes. Getters and setters are cold paths:
//! plain field accesses, no candidates.

use fixed::{Fixed, ONE};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::collider::changes::{
    ENABLED_OR_DISABLED, GROUPS, LOCAL_MASS_PROPERTIES, PARENT, POSITION, SHAPE, TYPE,
};
use rapier_core::collider::{
    ActiveCollisionTypes, ActiveEvents, ActiveHooks, CoefficientCombineRule, ColliderChanges,
    ColliderEnabled, ColliderFlags, ColliderMaterial, ColliderType, ColliderTypeTrait,
};
use rapier_core::interaction_groups::InteractionGroups;
use rapier_geometry2d::aabb::Aabb;
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
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, PI, TWO, ZERO};
    use glam::Vec2;
    use rapier_core::Handle;
    use rapier_core::collider::changes::{
        ENABLED_OR_DISABLED, GROUPS, LOCAL_MASS_PROPERTIES, PARENT, POSITION, SHAPE, TYPE,
    };
    use rapier_core::collider::{
        ActiveEventsTrait, ActiveHooksTrait, CoefficientCombineRule, ColliderChangesTrait,
        ColliderEnabled, ColliderType,
    };
    use rapier_core::interaction_groups::{GROUP_1, GROUP_2, InteractionGroupsTrait};
    use rapier_geometry2d::mass::MassPropertiesTrait;
    use rapier_geometry2d::shape::{CuboidTrait, Shape};
    use rapier_math::pose2::{IDENTITY, Pose2, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use super::super::builder::ColliderBuilderTrait;
    use super::super::components::{ColliderMassProps, ColliderParent};
    use super::{Collider, ColliderTrait};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    fn quarter_turn() -> Rot2 {
        Rot2 { re: ZERO, im: ONE }
    }

    /// A fresh unit-density ball, with the change flags cleared.
    fn quiet() -> Collider {
        Collider {
            changes: ColliderChangesTrait::empty(), ..ColliderBuilderTrait::ball(ONE).build(),
        }
    }

    fn attached() -> Collider {
        let parent = ColliderParent {
            handle: Handle { index: 3, generation: 1 },
            pos_wrt_parent: Pose2Trait::new(v(ONE, TWO), quarter_turn()),
        };
        Collider { parent: Some(parent), ..quiet() }
    }

    fn near(a: Fixed, b: Fixed, ulps: i64) {
        let d = if a.raw > b.raw {
            a.raw - b.raw
        } else {
            b.raw - a.raw
        };
        assert!(d <= ulps, "{} vs {}", a.raw, b.raw);
    }

    #[test]
    fn test_sensor_and_enabled_transitions() {
        let mut c = quiet();
        assert!(!c.is_sensor() && c.is_enabled());
        // No-op calls raise nothing.
        c.set_sensor(false);
        c.set_enabled(true);
        assert!(c.changes.is_empty());
        c.set_sensor(true);
        assert!(c.is_sensor() && c.co_type == ColliderType::Sensor);
        assert_eq!(c.changes, TYPE);
        c.set_enabled(false);
        assert!(!c.is_enabled() && c.flags.enabled == ColliderEnabled::Disabled);
        assert_eq!(c.changes, TYPE | ENABLED_OR_DISABLED);
        // A collider disabled by its parent is not enabled, and disabling it makes it
        // `Disabled`; enabling it does nothing (upstream only leaves `Disabled`).
        let mut c = quiet();
        c.flags.enabled = ColliderEnabled::DisabledByParent;
        assert!(!c.is_enabled());
        c.set_enabled(true);
        assert!(c.changes.is_empty() && c.flags.enabled == ColliderEnabled::DisabledByParent);
        c.set_enabled(false);
        assert_eq!(c.flags.enabled, ColliderEnabled::Disabled);
        assert_eq!(c.changes, ENABLED_OR_DISABLED);
        c.set_enabled(true);
        assert!(c.is_enabled());
        c.set_sensor(true);
        c.set_sensor(false);
        assert!(!c.is_sensor());
    }

    #[test]
    fn test_pose_setters_and_getters() {
        let mut c = quiet();
        assert_eq!(c.position(), IDENTITY);
        let t = v(ONE, -TWO);
        c.set_translation(t);
        assert_eq!(c.translation(), t);
        assert_eq!(c.rotation(), Rot2 { re: ONE, im: ZERO });
        assert_eq!(c.changes, POSITION);
        c.set_rotation(quarter_turn());
        assert_eq!(c.position(), Pose2Trait::new(t, quarter_turn()));
        let pose = Pose2Trait::new(v(TWO, TWO), Rot2 { re: -ONE, im: ZERO });
        c.set_position(pose);
        assert_eq!(c.position(), pose);
        assert_eq!(c.pos.pose, pose);
    }

    #[test]
    fn test_parent_accessors_and_setters() {
        // Standalone: no parent, and every `_wrt_parent` setter is a no-op.
        let mut c = quiet();
        assert!(c.parent().is_none() && c.position_wrt_parent().is_none());
        c.set_position_wrt_parent(Pose2Trait::new(v(ONE, ONE), quarter_turn()));
        c.set_translation_wrt_parent(v(ONE, ONE));
        c.set_rotation_wrt_parent(quarter_turn());
        assert_eq!(c, quiet());
        // Attached.
        let mut c = attached();
        assert_eq!(c.parent(), Some(Handle { index: 3, generation: 1 }));
        assert_eq!(c.position_wrt_parent(), Some(Pose2Trait::new(v(ONE, TWO), quarter_turn())));
        c.set_translation_wrt_parent(v(-ONE, ZERO));
        assert_eq!(c.position_wrt_parent(), Some(Pose2Trait::new(v(-ONE, ZERO), quarter_turn())));
        c.set_rotation_wrt_parent(Rot2 { re: ONE, im: ZERO });
        assert_eq!(
            c.position_wrt_parent(), Some(Pose2Trait::new(v(-ONE, ZERO), IDENTITY.rotation)),
        );
        let pose = Pose2Trait::new(v(TWO, ONE), quarter_turn());
        c.set_position_wrt_parent(pose);
        assert_eq!(c.position_wrt_parent(), Some(pose));
        assert_eq!(c.parent(), Some(Handle { index: 3, generation: 1 }));
        assert_eq!(c.changes, PARENT);
        // The world pose is not touched.
        assert_eq!(c.position(), IDENTITY);
    }

    /// Guarded setters raise their bit on a change only; the others always raise or never do.
    #[test]
    fn test_change_tracking_of_setters() {
        let mut c = quiet();
        let all = InteractionGroupsTrait::all();
        c.set_collision_groups(all);
        c.set_solver_groups(all);
        c.set_density(ONE);
        assert!(c.changes.is_empty());
        let groups = InteractionGroupsTrait::new(GROUP_1, GROUP_2, Default::default());
        c.set_collision_groups(groups);
        assert_eq!((c.changes, c.collision_groups()), (GROUPS, groups));
        let mut c = quiet();
        c.set_solver_groups(groups);
        assert_eq!((c.changes, c.solver_groups()), (GROUPS, groups));
        let mut c = quiet();
        c.set_shape(Shape::Cuboid(CuboidTrait::new(v(ONE, HALF))));
        assert_eq!(c.changes, SHAPE);
        assert_eq!(c.shape(), Shape::Cuboid(CuboidTrait::new(v(ONE, HALF))));
        // Material and flags never raise a bit.
        let mut c = quiet();
        c.set_friction(TWO);
        c.set_restitution(HALF);
        c.set_friction_combine_rule(CoefficientCombineRule::Max);
        c.set_restitution_combine_rule(CoefficientCombineRule::Min);
        c.set_active_hooks(ActiveHooksTrait::all());
        c.set_active_events(ActiveEventsTrait::all());
        c.set_contact_force_event_threshold(ONE);
        assert!(c.changes.is_empty());
        assert_eq!((c.friction(), c.restitution()), (TWO, HALF));
        assert_eq!(c.friction_combine_rule(), CoefficientCombineRule::Max);
        assert_eq!(c.restitution_combine_rule(), CoefficientCombineRule::Min);
        assert_eq!(c.material(), c.material);
        assert_eq!(c.active_hooks(), ActiveHooksTrait::all());
        assert_eq!(c.active_events(), ActiveEventsTrait::all());
        assert_eq!(c.contact_force_event_threshold(), ONE);
    }

    /// Density, mass and explicit mass properties, with the change bit on a change only.
    #[test]
    fn test_mass_variants() {
        let mut c = quiet();
        let volume = c.volume();
        near(volume, PI, 4);
        // `Density`: density is stored, mass is `density * area`.
        assert_eq!(c.density(), ONE);
        near(c.mass(), volume, 64);
        c.set_density(TWO);
        assert_eq!((c.changes, c.mprops), (LOCAL_MASS_PROPERTIES, ColliderMassProps::Density(TWO)));
        near(c.mass(), volume * TWO, 64);
        // `Mass`: mass is stored, density is `mass / area`.
        let mut c = quiet();
        c.set_mass(volume * TWO);
        assert_eq!(c.changes, LOCAL_MASS_PROPERTIES);
        assert_eq!(c.mass(), volume * TWO);
        near(c.density(), TWO, 64);
        near(c.mass_properties().mass(), volume * TWO, 64);
        // `MassProperties`: returned as given, whatever the shape.
        let mut c = quiet();
        let props = MassPropertiesTrait::new(v(HALF, ZERO), TWO, ONE);
        c.set_mass_properties(props);
        assert_eq!(c.mass_properties(), props);
        near(c.mass(), TWO, 4);
        near(c.density(), TWO / volume, 64);
        c.changes = ColliderChangesTrait::empty();
        c.set_mass_properties(props);
        assert!(c.changes.is_empty());
        // A shape without area has no density to derive.
        let segment = ColliderBuilderTrait::segment(v(ZERO, ZERO), v(ONE, ZERO)).mass(TWO).build();
        assert_eq!(segment.density(), ZERO);
        assert_eq!(segment.mass(), TWO);
        assert_eq!(segment.volume(), ZERO);
    }

    #[test]
    fn test_aabb_follows_the_pose() {
        let mut c = quiet();
        let aabb = c.compute_aabb();
        assert_eq!((aabb.mins, aabb.maxs), (v(-ONE, -ONE), v(ONE, ONE)));
        c.set_translation(v(TWO, -ONE));
        let aabb = c.compute_aabb();
        assert_eq!((aabb.mins, aabb.maxs), (v(ONE, -TWO), v(FixedTrait::from_int(3), ZERO)));
        // A quarter turn swaps the extents of a box, exactly.
        let mut c = ColliderBuilderTrait::cuboid(TWO, ONE).build();
        c.set_rotation(quarter_turn());
        let aabb = c.compute_aabb();
        assert_eq!((aabb.mins, aabb.maxs), (v(-ONE, -TWO), v(ONE, TWO)));
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_set_sensor() {
        let mut c = opaque(quiet());
        c.set_sensor(opaque(true));
        assert!(c.is_sensor());
    }

    #[test]
    fn gas_set_enabled() {
        let mut c = opaque(quiet());
        c.set_enabled(opaque(false));
        assert!(!c.is_enabled());
    }

    #[test]
    fn gas_set_position() {
        let mut c = opaque(quiet());
        c.set_position(opaque(IDENTITY));
        assert!(c.changes.intersects(POSITION));
    }

    #[test]
    fn gas_set_position_wrt_parent() {
        let mut c = opaque(attached());
        c.set_position_wrt_parent(opaque(IDENTITY));
        assert!(c.changes.intersects(PARENT));
    }

    #[test]
    fn gas_set_collision_groups() {
        let mut c = opaque(quiet());
        c.set_collision_groups(opaque(InteractionGroupsTrait::none()));
        assert!(c.changes.intersects(GROUPS));
    }

    #[test]
    fn gas_position() {
        let pose: Pose2 = opaque(quiet()).position();
        assert!(pose.rotation.re == ONE);
    }

    #[test]
    fn gas_friction() {
        assert!(opaque(quiet()).friction() == HALF);
    }

    #[test]
    fn gas_mass_from_density() {
        assert!(opaque(quiet()).mass() != ZERO);
    }

    #[test]
    fn gas_mass_stored() {
        let c = Collider { mprops: ColliderMassProps::Mass(opaque(TWO)), ..quiet() };
        assert!(opaque(c).mass() != ZERO);
    }

    #[test]
    fn gas_density_stored() {
        assert!(opaque(quiet()).density() != ZERO);
    }

    #[test]
    fn gas_density_from_mass() {
        let c = Collider { mprops: ColliderMassProps::Mass(opaque(TWO)), ..quiet() };
        assert!(opaque(c).density() != ZERO);
    }

    #[test]
    fn gas_mass_from_mass_properties() {
        let props = MassPropertiesTrait::new(v(ZERO, ZERO), opaque(TWO), opaque(ONE));
        let c = Collider { mprops: ColliderMassProps::MassProperties(props), ..quiet() };
        assert!(opaque(c).mass() != ZERO);
    }

    #[test]
    fn gas_mass_properties_density() {
        assert!(opaque(quiet()).mass_properties().inv_mass != ZERO);
    }

    #[test]
    fn gas_mass_properties_cuboid() {
        assert!(
            opaque(ColliderBuilderTrait::cuboid(TWO, ONE).build())
                .mass_properties()
                .inv_mass != ZERO,
        );
    }

    #[test]
    fn gas_mass_properties_capsule() {
        assert!(
            opaque(ColliderBuilderTrait::capsule_y(HALF, HALF).build())
                .mass_properties()
                .inv_mass != ZERO,
        );
    }

    #[test]
    fn gas_compute_aabb_ball() {
        assert!(opaque(quiet()).compute_aabb().maxs.x != ZERO);
    }

    #[test]
    fn gas_compute_aabb_cuboid_rotated() {
        let mut c = ColliderBuilderTrait::cuboid(TWO, ONE).build();
        c.set_rotation(opaque(quarter_turn()));
        assert!(opaque(c).compute_aabb().maxs.x != ZERO);
    }

    /// The cost of `opaque(collider)` itself, to subtract from the probes above.
    #[test]
    fn gas_opaque_collider() {
        let _ = opaque(quiet());
    }
}
