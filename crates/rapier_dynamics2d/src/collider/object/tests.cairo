//! Tests and gas probes of `Collider` (`super`).

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
    Collider { changes: ColliderChangesTrait::empty(), ..ColliderBuilderTrait::ball(ONE).build() }
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
    assert_eq!(c.position_wrt_parent(), Some(Pose2Trait::new(v(-ONE, ZERO), IDENTITY.rotation)));
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
        opaque(ColliderBuilderTrait::cuboid(TWO, ONE).build()).mass_properties().inv_mass != ZERO,
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

/// Collision, broad-phase and swept AABBs: (collider pose, next pose, prediction) against the
/// exact expected boxes.
#[test]
fn test_collision_broad_phase_and_swept_aabbs() {
    let three = FixedTrait::from_int(3);
    let quarter = FixedTrait::from_raw(HALF.raw / 2);
    // (collider, next pose, prediction, collision mins/maxs, swept mins/maxs)
    let cases = array![
        (
            quiet(),
            Pose2Trait::new(v(TWO, ZERO), IDENTITY.rotation),
            HALF,
            v(-ONE - HALF, -ONE - HALF),
            v(ONE + HALF, ONE + HALF),
            v(-ONE, -ONE),
            v(three, ONE),
        ),
        (
            ColliderBuilderTrait::cuboid(TWO, ONE).build(),
            Pose2Trait::new(v(ZERO, -three), quarter_turn()),
            ZERO,
            v(-TWO, -ONE),
            v(TWO, ONE),
            v(-TWO, -three - TWO),
            v(TWO, ONE),
        ),
    ];
    for (c, next, prediction, cmin, cmax, smin, smax) in cases {
        let aabb = c.compute_collision_aabb(prediction);
        assert_eq!((aabb.mins, aabb.maxs), (cmin, cmax));
        let swept = c.compute_swept_aabb(next);
        assert_eq!((swept.mins, swept.maxs), (smin, smax));
        // A swept AABB to the current pose is the plain AABB.
        assert_eq!(c.compute_swept_aabb(c.position()), c.compute_aabb());
    }
    // The broad-phase AABB is loosened by half the prediction distance (default 0.002 m).
    let params: rapier_core::integration_parameters::IntegrationParameters = Default::default();
    let c = quiet();
    let margin =
        rapier_core::integration_parameters::IntegrationParametersTrait::prediction_distance(
        params,
    )
        * HALF;
    assert_eq!(c.compute_broad_phase_aabb(params), c.compute_collision_aabb(margin));
    assert_eq!(c.compute_collision_aabb(quarter).maxs, v(ONE + quarter, ONE + quarter));
}

/// `copy_from` copies everything but the parent link, the pose only without a parent, and
/// raises every change flag.
#[test]
fn test_copy_from() {
    let other = ColliderBuilderTrait::cuboid(TWO, ONE)
        .sensor(true)
        .friction(TWO)
        .density(HALF)
        .user_data(7)
        .translation(v(ONE, ONE))
        .build();
    // Standalone: becomes `other` (with every flag raised).
    let mut c = quiet();
    c.copy_from(other);
    assert_eq!(c, Collider { changes: ColliderChangesTrait::all(), ..other });
    // Attached: keeps its parent and its world pose.
    let mut c = attached();
    let before = c;
    c.copy_from(other);
    assert_eq!(c.parent, before.parent);
    assert_eq!(c.pos, before.pos);
    assert_eq!(
        Collider { parent: other.parent, pos: other.pos, ..c },
        Collider { changes: ColliderChangesTrait::all(), ..other },
    );
}

#[test]
fn gas_compute_collision_aabb_ball() {
    assert!(opaque(quiet()).compute_collision_aabb(opaque(HALF)).maxs.x != ZERO);
}

#[test]
fn gas_compute_broad_phase_aabb_ball() {
    assert!(opaque(quiet()).compute_broad_phase_aabb(opaque(Default::default())).maxs.x != ZERO);
}

#[test]
fn gas_compute_swept_aabb_ball() {
    let next = Pose2Trait::new(v(TWO, ZERO), IDENTITY.rotation);
    assert!(opaque(quiet()).compute_swept_aabb(opaque(next)).maxs.x != ZERO);
}

#[test]
fn gas_compute_swept_aabb_cuboid_rotated() {
    let next = Pose2Trait::new(v(TWO, ZERO), quarter_turn());
    let c = ColliderBuilderTrait::cuboid(TWO, ONE).build();
    assert!(opaque(c).compute_swept_aabb(opaque(next)).maxs.x != ZERO);
}

#[test]
fn gas_copy_from() {
    let mut c = opaque(attached());
    c.copy_from(opaque(quiet()));
    assert!(c.parent.is_some());
}
