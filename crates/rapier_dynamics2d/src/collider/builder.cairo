//! `ColliderBuilder` (upstream `geometry/collider.rs`): a fluent description of a collider.
//!
//! Every method takes and returns the builder by value; `build` never fails. Defaults are
//! upstream's: density 1, friction 0.5, restitution 0, `Average` combine rules, identity pose,
//! solid, enabled, all-pass groups, no hooks, no events, no force threshold, `user_data` 0.
//!
//! Deviations: `rotation` takes a unit [`Rot2`] where upstream takes an angle (no trigonometry in
//! `rapier_math` yet); `halfspace` takes a plain `Vec2` outward normal (expected unit); the
//! deprecated `position_wrt_parent` and `delta`, the shapes that are not in the closed `Shape`
//! enum and `contact_skin` are not ported.

use fixed::{Fixed, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::collider::{
    ActiveCollisionTypes, ActiveEvents, ActiveEventsTrait, ActiveHooks, ActiveHooksTrait,
    CoefficientCombineRule, ColliderChangesTrait, ColliderEnabled, ColliderFlags, ColliderMaterial,
    ColliderType,
};
use rapier_core::interaction_groups::{InteractionGroups, InteractionGroupsTrait};
use rapier_geometry2d::mass::MassProperties;
use rapier_geometry2d::shape::{
    BallTrait, CapsuleTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait, Shape,
};
use rapier_math::pose2::{IDENTITY, Pose2};
use rapier_math::rot2::Rot2;
use super::components::{ColliderMassProps, ColliderPosition};
use super::object::Collider;

/// The settings of a collider to be built.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ColliderBuilder {
    /// The shape of the collider.
    pub shape: Shape,
    /// How the mass is specified.
    pub mass_properties: ColliderMassProps,
    pub friction: Fixed,
    pub friction_combine_rule: CoefficientCombineRule,
    pub restitution: Fixed,
    pub restitution_combine_rule: CoefficientCombineRule,
    /// Pose of the collider: relative to its parent body, or in the world when standalone.
    pub position: Pose2,
    pub is_sensor: bool,
    pub active_collision_types: ActiveCollisionTypes,
    pub active_hooks: ActiveHooks,
    pub active_events: ActiveEvents,
    pub user_data: u128,
    pub collision_groups: InteractionGroups,
    pub solver_groups: InteractionGroups,
    pub enabled: bool,
    pub contact_force_event_threshold: Fixed,
}

/// Upstream default: a ball of radius `0.5`.
pub impl ColliderBuilderDefault of Default<ColliderBuilder> {
    fn default() -> ColliderBuilder {
        ColliderBuilderTrait::ball(HALF)
    }
}

#[generate_trait]
pub impl ColliderBuilderImpl of ColliderBuilderTrait {
    /// A builder for `shape` with the default settings.
    fn new(shape: Shape) -> ColliderBuilder {
        ColliderBuilder {
            shape,
            mass_properties: ColliderMassProps::Density(ONE),
            friction: HALF,
            friction_combine_rule: CoefficientCombineRule::Average,
            restitution: ZERO,
            restitution_combine_rule: CoefficientCombineRule::Average,
            position: IDENTITY,
            is_sensor: false,
            active_collision_types: Default::default(),
            active_hooks: ActiveHooksTrait::empty(),
            active_events: ActiveEventsTrait::empty(),
            user_data: 0,
            collision_groups: InteractionGroupsTrait::all(),
            solver_groups: InteractionGroupsTrait::all(),
            enabled: true,
            contact_force_event_threshold: ZERO,
        }
    }

    /// A strictly convex CCW polygon with 3..=8 vertices. Returns `None` for invalid order,
    /// duplicates, collinearity, concavity or count. Normalisation rounds to nearest; coordinate
    /// differences/wide products must fit Q32.32/i128, otherwise construction panics.
    fn convex_polygon(points: Span<Vec2>) -> Option<ColliderBuilder> {
        let polygon = rapier_geometry2d::shape::ConvexPolygonTrait::from_convex_polyline(points)?;
        Some(Self::new(Shape::ConvexPolygon(BoxTrait::new(polygon))))
    }

    /// A disc of radius `radius`.
    fn ball(radius: Fixed) -> ColliderBuilder {
        Self::new(Shape::Ball(BallTrait::new(radius)))
    }

    /// A box of half extents `(hx, hy)`.
    fn cuboid(hx: Fixed, hy: Fixed) -> ColliderBuilder {
        Self::new(Shape::Cuboid(CuboidTrait::new(Vec2 { x: hx, y: hy })))
    }

    /// The capsule of core segment `a`–`b` and radius `radius`.
    fn capsule_from_endpoints(a: Vec2, b: Vec2, radius: Fixed) -> ColliderBuilder {
        Self::new(Shape::Capsule(CapsuleTrait::new(a, b, radius)))
    }

    /// A capsule along the `x` axis, its core segment `2 * half_height` long.
    fn capsule_x(half_height: Fixed, radius: Fixed) -> ColliderBuilder {
        Self::new(Shape::Capsule(CapsuleTrait::new_x(half_height, radius)))
    }

    /// A capsule along the `y` axis, its core segment `2 * half_height` long.
    fn capsule_y(half_height: Fixed, radius: Fixed) -> ColliderBuilder {
        Self::new(Shape::Capsule(CapsuleTrait::new_y(half_height, radius)))
    }

    /// The segment `a`–`b`.
    fn segment(a: Vec2, b: Vec2) -> ColliderBuilder {
        Self::new(Shape::Segment(SegmentTrait::new(a, b)))
    }

    /// The half-space behind the plane through the origin of outward unit normal
    /// `outward_normal`.
    fn halfspace(outward_normal: Vec2) -> ColliderBuilder {
        Self::new(Shape::HalfSpace(HalfSpaceTrait::new(outward_normal)))
    }

    /// Stores `data` in the collider (upstream `user_data`).
    fn user_data(self: ColliderBuilder, data: u128) -> ColliderBuilder {
        ColliderBuilder { user_data: data, ..self }
    }

    fn collision_groups(self: ColliderBuilder, groups: InteractionGroups) -> ColliderBuilder {
        ColliderBuilder { collision_groups: groups, ..self }
    }

    fn solver_groups(self: ColliderBuilder, groups: InteractionGroups) -> ColliderBuilder {
        ColliderBuilder { solver_groups: groups, ..self }
    }

    fn sensor(self: ColliderBuilder, is_sensor: bool) -> ColliderBuilder {
        ColliderBuilder { is_sensor, ..self }
    }

    fn active_hooks(self: ColliderBuilder, active_hooks: ActiveHooks) -> ColliderBuilder {
        ColliderBuilder { active_hooks, ..self }
    }

    fn active_events(self: ColliderBuilder, active_events: ActiveEvents) -> ColliderBuilder {
        ColliderBuilder { active_events, ..self }
    }

    fn active_collision_types(
        self: ColliderBuilder, active_collision_types: ActiveCollisionTypes,
    ) -> ColliderBuilder {
        ColliderBuilder { active_collision_types, ..self }
    }

    fn friction(self: ColliderBuilder, friction: Fixed) -> ColliderBuilder {
        ColliderBuilder { friction, ..self }
    }

    fn friction_combine_rule(
        self: ColliderBuilder, rule: CoefficientCombineRule,
    ) -> ColliderBuilder {
        ColliderBuilder { friction_combine_rule: rule, ..self }
    }

    fn restitution(self: ColliderBuilder, restitution: Fixed) -> ColliderBuilder {
        ColliderBuilder { restitution, ..self }
    }

    fn restitution_combine_rule(
        self: ColliderBuilder, rule: CoefficientCombineRule,
    ) -> ColliderBuilder {
        ColliderBuilder { restitution_combine_rule: rule, ..self }
    }

    /// Specifies the mass by a density, replacing any previous specification (last call wins).
    fn density(self: ColliderBuilder, density: Fixed) -> ColliderBuilder {
        ColliderBuilder { mass_properties: ColliderMassProps::Density(density), ..self }
    }

    /// Specifies the mass by a mass; the inertia follows from the shape.
    fn mass(self: ColliderBuilder, mass: Fixed) -> ColliderBuilder {
        ColliderBuilder { mass_properties: ColliderMassProps::Mass(mass), ..self }
    }

    /// Specifies explicit mass properties.
    fn mass_properties(self: ColliderBuilder, mass_properties: MassProperties) -> ColliderBuilder {
        ColliderBuilder {
            mass_properties: ColliderMassProps::MassProperties(mass_properties), ..self,
        }
    }

    /// The force beyond which a contact force event may be emitted.
    fn contact_force_event_threshold(self: ColliderBuilder, threshold: Fixed) -> ColliderBuilder {
        ColliderBuilder { contact_force_event_threshold: threshold, ..self }
    }

    /// Sets the translation of the pose, keeping its rotation.
    fn translation(self: ColliderBuilder, translation: Vec2) -> ColliderBuilder {
        ColliderBuilder { position: Pose2 { translation, ..self.position }, ..self }
    }

    /// Sets the rotation of the pose (unit), keeping its translation.
    fn rotation(self: ColliderBuilder, rotation: Rot2) -> ColliderBuilder {
        ColliderBuilder { position: Pose2 { rotation, ..self.position }, ..self }
    }

    /// Sets the whole pose: relative to the parent body, or in the world when standalone.
    fn position(self: ColliderBuilder, position: Pose2) -> ColliderBuilder {
        ColliderBuilder { position, ..self }
    }

    /// Whether the collider starts enabled.
    fn enabled(self: ColliderBuilder, enabled: bool) -> ColliderBuilder {
        ColliderBuilder { enabled, ..self }
    }

    /// The collider: no parent, `changes = ColliderChanges::all()`, world pose = `position`.
    fn build(self: ColliderBuilder) -> Collider {
        Collider {
            co_type: if self.is_sensor {
                ColliderType::Sensor
            } else {
                ColliderType::Solid
            },
            shape: self.shape,
            mprops: self.mass_properties,
            changes: ColliderChangesTrait::all(),
            parent: None,
            pos: ColliderPosition { pose: self.position },
            material: ColliderMaterial {
                friction: self.friction,
                restitution: self.restitution,
                friction_combine_rule: self.friction_combine_rule,
                restitution_combine_rule: self.restitution_combine_rule,
            },
            flags: ColliderFlags {
                active_collision_types: self.active_collision_types,
                collision_groups: self.collision_groups,
                solver_groups: self.solver_groups,
                active_hooks: self.active_hooks,
                active_events: self.active_events,
                enabled: if self.enabled {
                    ColliderEnabled::Enabled
                } else {
                    ColliderEnabled::Disabled
                },
            },
            contact_force_event_threshold: self.contact_force_event_threshold,
            user_data: self.user_data,
        }
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, TWO, ZERO};
    use glam::Vec2;
    use rapier_core::collider::changes::ColliderChanges;
    use rapier_core::collider::events::COLLISION_EVENTS;
    use rapier_core::collider::hooks::FILTER_CONTACT_PAIRS;
    use rapier_core::collider::{
        ActiveCollisionTypesTrait, ActiveEventsTrait, ActiveHooksTrait, CoefficientCombineRule,
        ColliderChangesTrait, ColliderEnabled, ColliderType,
    };
    use rapier_core::interaction_groups::{GROUP_1, GROUP_2, InteractionGroupsTrait};
    use rapier_geometry2d::mass::MassPropertiesTrait;
    use rapier_geometry2d::shape::{
        BallTrait, CapsuleTrait, CuboidTrait, HalfSpaceTrait, SegmentTrait, Shape,
    };
    use rapier_math::pose2::{IDENTITY, Pose2Trait};
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use super::super::components::ColliderMassProps;
    use super::super::object::ColliderTrait;
    use super::{ColliderBuilder, ColliderBuilderTrait};

    fn v(x: Fixed, y: Fixed) -> Vec2 {
        Vec2 { x, y }
    }

    /// Every constructor gives the expected shape, with the upstream defaults everywhere else.
    #[test]
    fn test_shape_constructors_and_defaults() {
        let shapes = array![
            (ColliderBuilderTrait::ball(TWO), Shape::Ball(BallTrait::new(TWO))),
            (ColliderBuilderTrait::cuboid(TWO, ONE), Shape::Cuboid(CuboidTrait::new(v(TWO, ONE)))),
            (
                ColliderBuilderTrait::capsule_y(TWO, HALF),
                Shape::Capsule(CapsuleTrait::new(v(ZERO, -TWO), v(ZERO, TWO), HALF)),
            ),
            (
                ColliderBuilderTrait::capsule_x(TWO, HALF),
                Shape::Capsule(CapsuleTrait::new(v(-TWO, ZERO), v(TWO, ZERO), HALF)),
            ),
            (
                ColliderBuilderTrait::capsule_from_endpoints(v(ZERO, ONE), v(ONE, ONE), HALF),
                Shape::Capsule(CapsuleTrait::new(v(ZERO, ONE), v(ONE, ONE), HALF)),
            ),
            (
                ColliderBuilderTrait::segment(v(ZERO, ONE), v(ONE, ONE)),
                Shape::Segment(SegmentTrait::new(v(ZERO, ONE), v(ONE, ONE))),
            ),
            (
                ColliderBuilderTrait::halfspace(v(ZERO, ONE)),
                Shape::HalfSpace(HalfSpaceTrait::new(v(ZERO, ONE))),
            ),
        ];
        for entry in shapes.span() {
            let (builder, shape) = *entry;
            assert_eq!(builder, ColliderBuilderTrait::new(shape));
            assert_eq!(builder.shape, shape);
            assert_eq!(builder.mass_properties, ColliderMassProps::Density(ONE));
            assert_eq!((builder.friction, builder.restitution), (HALF, ZERO));
            assert_eq!(builder.friction_combine_rule, CoefficientCombineRule::Average);
            assert_eq!(builder.restitution_combine_rule, CoefficientCombineRule::Average);
            assert_eq!(builder.position, IDENTITY);
            assert!(!builder.is_sensor && builder.enabled);
            assert_eq!(builder.user_data, 0_u128);
            assert_eq!(builder.contact_force_event_threshold, ZERO);
        }
        let default: ColliderBuilder = Default::default();
        assert_eq!(default, ColliderBuilderTrait::ball(HALF));
    }

    #[test]
    fn test_build_defaults() {
        let c = ColliderBuilderTrait::cuboid(TWO, ONE).build();
        assert_eq!(c.co_type, ColliderType::Solid);
        assert_eq!(c.shape, Shape::Cuboid(CuboidTrait::new(v(TWO, ONE))));
        assert_eq!(c.mprops, ColliderMassProps::Density(ONE));
        assert_eq!(c.changes, ColliderChangesTrait::all());
        assert!(c.parent.is_none());
        assert_eq!(c.pos.pose, IDENTITY);
        assert_eq!((c.friction(), c.restitution()), (HALF, ZERO));
        assert_eq!(c.friction_combine_rule(), CoefficientCombineRule::Average);
        assert_eq!(c.restitution_combine_rule(), CoefficientCombineRule::Average);
        assert_eq!(c.collision_groups(), InteractionGroupsTrait::all());
        assert_eq!(c.solver_groups(), InteractionGroupsTrait::all());
        assert_eq!(c.active_hooks(), ActiveHooksTrait::empty());
        assert_eq!(c.active_events(), ActiveEventsTrait::empty());
        let types: rapier_core::collider::ActiveCollisionTypes = Default::default();
        assert_eq!(c.active_collision_types(), types);
        assert!(c.is_enabled() && !c.is_sensor());
        assert_eq!(c.contact_force_event_threshold(), ZERO);
        assert_eq!(c.user_data, 0_u128);
        // The default mass is the unit-density one: an area of 8 for a `4 x 2` box.
        assert_eq!(c.mass_properties(), CuboidTrait::new(v(TWO, ONE)).mass_properties(ONE));
    }

    /// Each setter reaches the built collider, and only the field it names.
    #[test]
    fn test_setters_reach_the_collider() {
        let groups = InteractionGroupsTrait::new(GROUP_1, GROUP_2, Default::default());
        let pose = Pose2Trait::new(v(ONE, TWO), Rot2 { re: ZERO, im: ONE });
        let base = ColliderBuilderTrait::ball(ONE);
        let c = base
            .friction(TWO)
            .restitution(HALF)
            .friction_combine_rule(CoefficientCombineRule::Max)
            .restitution_combine_rule(CoefficientCombineRule::Min)
            .sensor(true)
            .enabled(false)
            .collision_groups(groups)
            .solver_groups(groups)
            .active_events(COLLISION_EVENTS)
            .active_hooks(FILTER_CONTACT_PAIRS)
            .active_collision_types(ActiveCollisionTypesTrait::empty())
            .contact_force_event_threshold(TWO)
            .user_data(0x1234_u128)
            .position(pose)
            .build();
        assert_eq!((c.friction(), c.restitution()), (TWO, HALF));
        assert_eq!(c.friction_combine_rule(), CoefficientCombineRule::Max);
        assert_eq!(c.restitution_combine_rule(), CoefficientCombineRule::Min);
        assert!(c.is_sensor() && !c.is_enabled());
        assert_eq!(c.flags.enabled, ColliderEnabled::Disabled);
        assert_eq!((c.collision_groups(), c.solver_groups()), (groups, groups));
        assert_eq!(c.active_events(), COLLISION_EVENTS);
        assert_eq!(c.active_hooks(), FILTER_CONTACT_PAIRS);
        assert!(c.active_collision_types().is_empty());
        assert_eq!(c.contact_force_event_threshold(), TWO);
        assert_eq!(c.user_data, 0x1234_u128);
        assert_eq!(c.position(), pose);
        // Pose setters compose: translation and rotation keep each other.
        let moved = base.translation(v(ONE, TWO)).rotation(Rot2 { re: ZERO, im: ONE }).build();
        assert_eq!(moved.position(), pose);
        let turned = base.rotation(Rot2 { re: ZERO, im: ONE }).translation(v(ONE, TWO)).build();
        assert_eq!(turned.position(), pose);
        // The builder is a value: `base` is unchanged.
        assert_eq!(base, ColliderBuilderTrait::ball(ONE));
        assert_eq!(base.build().changes, ColliderChanges { bits: 0x1ff });
    }

    /// The last of `density`, `mass` and `mass_properties` wins.
    #[test]
    fn test_mass_specification_last_call_wins() {
        let props = MassPropertiesTrait::new(v(HALF, ZERO), TWO, ONE);
        let base = ColliderBuilderTrait::ball(ONE);
        assert_eq!(base.density(TWO).mass_properties, ColliderMassProps::Density(TWO));
        assert_eq!(base.mass(TWO).mass_properties, ColliderMassProps::Mass(TWO));
        assert_eq!(
            base.mass_properties(props).mass_properties, ColliderMassProps::MassProperties(props),
        );
        assert_eq!(base.mass(TWO).density(HALF).mass_properties, ColliderMassProps::Density(HALF));
        assert_eq!(base.density(HALF).mass(TWO).mass_properties, ColliderMassProps::Mass(TWO));
        assert_eq!(base.mass(TWO).mass_properties(props).build().mass_properties(), props);
        assert_eq!(base.density(ZERO).build().mass_properties(), Default::default());
        assert_eq!(base.build().density(), ONE);
        assert_eq!(FixedTrait::from_int(3) - TWO, ONE);
    }

    #[test]
    fn test_convex_polygon_constructor() {
        let points = [v(-ONE, -ONE), v(ONE, -ONE), v(ONE, ONE), v(-ONE, ONE)];
        let polygon = ColliderBuilderTrait::convex_polygon(points.span()).unwrap().build();
        assert_eq!(polygon.mass_properties().inv_mass, Fixed { raw: 1073741824 });
        assert!(
            ColliderBuilderTrait::convex_polygon([v(ZERO, ZERO), v(ONE, ZERO)].span()).is_none(),
        );
    }

    #[test]
    fn gas_convex_polygon() {
        let points = opaque([v(-ONE, -ONE), v(ONE, -ONE), v(ONE, ONE), v(-ONE, ONE)]);
        let _ = ColliderBuilderTrait::convex_polygon(points.span());
    }

    #[test]
    fn gas_baseline() {}

    #[test]
    fn gas_new() {
        let _ = ColliderBuilderTrait::new(opaque(Shape::Ball(BallTrait::new(ONE))));
    }

    #[test]
    fn gas_ball() {
        let _ = ColliderBuilderTrait::ball(opaque(ONE));
    }

    #[test]
    fn gas_setters() {
        let _ = opaque(ColliderBuilderTrait::ball(ONE)).friction(opaque(TWO)).sensor(opaque(true));
    }

    #[test]
    fn gas_build() {
        let _ = opaque(ColliderBuilderTrait::ball(ONE)).build();
    }

    #[test]
    fn gas_ball_and_build() {
        let _ = ColliderBuilderTrait::ball(opaque(ONE)).density(opaque(TWO)).build();
    }
}
