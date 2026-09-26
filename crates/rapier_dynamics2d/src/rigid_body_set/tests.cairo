//! Unit tests and gas probes of the rigid-body set.

use fixed::{Fixed, HALF, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_core::rigid_body::changes::{ENABLED_OR_DISABLED, LOCAL_MASS_PROPERTIES, TYPE};
use rapier_core::rigid_body::{RigidBodyChangesTrait, RigidBodyType};
use rapier_geometry2d::mass::MassProperties;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::collider::ColliderBuilderTrait;
use crate::collider_set::{ColliderSet, ColliderSetTrait};
use crate::rigid_body::{LockedAxesTrait, ROTATION_LOCKED};
use super::{
    RigidBody, RigidBodyBuilderTrait, RigidBodySet, RigidBodySetTrait, RigidBodyTrait,
    cold_or_default, extra_additional_is_mass, recompute_body_mass_properties,
};

fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: Vec2 { x, y }, rotation: Rot2 { re: ONE, im: ZERO } }
}

#[test]
fn test_set_lifecycle() {
    let mut bodies = RigidBodySetTrait::new();
    let mut colliders: ColliderSet = ColliderSetTrait::new();
    let mut body = RigidBodyTrait::dynamic(at(ZERO, ZERO));
    body.colliders = array![Handle { index: 9, generation: 9 }].span();
    let h0 = bodies.insert(body);
    let h1 = bodies.insert(RigidBodyTrait::fixed(at(ONE, ZERO)));
    assert_eq!(h0, Handle { index: 0, generation: 0 });
    assert_eq!(h1, Handle { index: 1, generation: 0 });
    // `insert` resets the collider list and raises every change flag.
    let stored = bodies.get(h0).unwrap();
    assert_eq!(stored.colliders.len(), 0);
    assert_eq!(stored.changes, RigidBodyChangesTrait::all());
    assert_eq!(bodies.len(), 2);
    let mut moved = stored;
    moved.set_linvel(Vec2 { x: ONE, y: ONE });
    assert!(bodies.set(h0, moved));
    assert_eq!(bodies.get(h0).unwrap().linvel(), Vec2 { x: ONE, y: ONE });
    assert!(bodies.remove(h0, ref colliders, true).is_some());
    assert!(!bodies.contains(h0));
    assert!(!bodies.set(h0, moved));
    assert_eq!(bodies.get(h0), None);
    // Slot 0 is reused with the bumped generation; iteration is by slot index.
    let h2 = bodies.insert(RigidBodyTrait::dynamic(at(TWO, ZERO)));
    assert_eq!(h2, Handle { index: 0, generation: 1 });
    let all = bodies.iter();
    assert_eq!(all.len(), 2);
    let (first, _) = *all.at(0);
    let (second, _) = *all.at(1);
    assert_eq!(first, h2);
    assert_eq!(second, h1);
    assert!(!bodies.is_empty());
}

#[test]
fn test_set_compatibility_accessors_are_value_copies() {
    let mut bodies = RigidBodySetTrait::with_capacity(4);
    let h0 = bodies.insert(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
    let h1 = bodies.insert(RigidBodyTrait::fixed(at(ONE, ZERO)));
    assert_eq!(bodies.get_mut(h0), bodies.get(h0));
    let (_, unknown_h1) = bodies.get_unknown_gen(1).unwrap();
    let (_, unknown_h0) = bodies.get_unknown_gen_mut(0).unwrap();
    assert_eq!(unknown_h1, h1);
    assert_eq!(unknown_h0, h0);
    let (first, second) = bodies.get_pair_mut(h0, h1);
    assert_eq!(first.unwrap().body_type(), RigidBodyType::Dynamic);
    assert_eq!(second.unwrap().body_type(), RigidBodyType::Fixed);
    let (same, none) = bodies.get_pair_mut(h0, h0);
    assert!(same.is_some());
    assert_eq!(none, None);
    assert_eq!(bodies.iter_mut().len(), 2);
}

#[test]
fn test_builder_round_trip() {
    let extra = MassProperties {
        local_com: Vec2 { x: ONE, y: ZERO }, inv_mass: ONE, inv_principal_inertia: HALF,
    };
    let body = RigidBodyBuilderTrait::dynamic()
        .translation(Vec2 { x: ONE, y: TWO })
        .rotation(Rot2 { re: ZERO, im: ONE })
        .linvel(Vec2 { x: TWO, y: -ONE })
        .angvel(HALF)
        .linear_damping(ONE)
        .angular_damping(TWO)
        .gravity_scale(HALF)
        .dominance_group(-3)
        .enabled(false)
        .user_data(99)
        .additional_solver_iterations(7)
        .additional_pgs_iterations(5)
        .locked_axes(ROTATION_LOCKED)
        .additional_mass_properties(extra)
        .allow_fast_rotation(true)
        .build();
    assert_eq!(body.translation(), Vec2 { x: ONE, y: TWO });
    assert_eq!(body.rotation(), Rot2 { re: ZERO, im: ONE });
    assert_eq!(body.linvel(), Vec2 { x: TWO, y: -ONE });
    assert_eq!(body.angvel(), HALF);
    assert_eq!(body.gravity_scale(), HALF);
    assert_eq!(body.dominance_group(), -3);
    assert!(!body.is_enabled());
    let cold = cold_or_default(body.cold);
    assert_eq!(cold.user_data, 99);
    assert_eq!(body.additional_solver_iterations(), 7);
    assert_eq!(body.additional_pgs_iterations(), 5);
    assert!(body.locked_axes().contains(ROTATION_LOCKED));
    assert!(body.is_fast_rotation_allowed());
    assert_eq!(cold.additional_local_mprops, extra);
    assert!(!extra_additional_is_mass(cold.solver_flags));
}

#[test]
fn test_setters_flags_and_additional_mass_recompute() {
    let mut colliders: ColliderSet = ColliderSetTrait::new();
    let mut body = RigidBodyTrait::dynamic(at(ZERO, ZERO));
    body.set_enabled(false);
    assert!(body.changes.contains(ENABLED_OR_DISABLED));
    body.set_body_type(RigidBodyType::Fixed, true);
    assert!(body.changes.contains(TYPE));
    assert_eq!(body.angvel(), ZERO);
    body.set_body_type(RigidBodyType::Dynamic, true);
    body.set_locked_axes(ROTATION_LOCKED, true);
    assert!(body.changes.contains(LOCAL_MASS_PROPERTIES));
    body.set_additional_mass(TWO, true);
    recompute_body_mass_properties(ref body, ref colliders);
    assert_eq!(body.mass(), TWO);
}

#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}

#[test]
fn gas_body_new() {
    let _ = RigidBodyTrait::dynamic(opaque(at(ONE, ZERO)));
}

#[test]
fn gas_insert() {
    let mut bodies = RigidBodySetTrait::new();
    let _ = bodies.insert(opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO))));
}

#[test]
fn gas_get_set() {
    let mut bodies = RigidBodySetTrait::new();
    let h = bodies.insert(opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO))));
    let body = bodies.get(opaque(h)).unwrap();
    let _ = bodies.set(h, body);
}

#[test]
fn gas_remove() {
    let mut bodies = RigidBodySetTrait::new();
    let mut colliders = ColliderSetTrait::new();
    let h = bodies.insert(opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO))));
    let _ = bodies.remove(opaque(h), ref colliders, true);
}

#[test]
fn gas_iter_8() {
    let mut bodies = RigidBodySetTrait::new();
    let body = opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
    let mut i: u32 = 0;
    while i != 8 {
        let _ = bodies.insert(body);
        i += 1;
    }
    let _ = bodies.iter();
}

/// Eight moved bodies with one collider each.
#[test]
fn gas_propagate_modified_body_positions_8() {
    let mut bodies: RigidBodySet = RigidBodySetTrait::new();
    let mut colliders = ColliderSetTrait::new();
    let body = opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
    let collider = opaque(ColliderBuilderTrait::ball(ONE).build());
    let mut i: u32 = 0;
    while i != 8 {
        let h = bodies.insert(body);
        let _ = colliders.insert_with_parent(collider, h, ref bodies);
        i += 1;
    }
    bodies.propagate_modified_body_positions_to_colliders(ref colliders);
}

/// Save / restore of eight bodies, one of them removed: `gas_to_state` − `gas_state_setup`,
/// `gas_from_state` − `gas_to_state`.
fn state_setup() -> RigidBodySet {
    let mut bodies = RigidBodySetTrait::new();
    let mut colliders = ColliderSetTrait::new();
    let body = opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
    let mut i: u32 = 0;
    while i != 8 {
        let _ = bodies.insert(body);
        i += 1;
    }
    let _ = bodies.remove(Handle { index: 3, generation: 0 }, ref colliders, true);
    bodies
}

#[test]
fn gas_state_setup() {
    let _ = state_setup();
}

#[test]
fn gas_to_state() {
    let mut bodies = state_setup();
    let _ = bodies.to_state();
}

#[test]
fn gas_from_state() {
    let mut bodies = state_setup();
    let restored = RigidBodySetTrait::from_state(bodies.to_state());
    assert_eq!(restored.len(), 7);
}

/// Sleep helpers on a sleeping dynamic body (file budget: one probe per family;
/// `gas_<family>` − `gas_sleeping_body`).
#[inline(never)]
fn sleeping_body() -> RigidBody {
    let mut body = RigidBodyTrait::dynamic(opaque(at(ZERO, ZERO)));
    body.sleep();
    body
}

#[test]
fn gas_sleeping_body() {
    let _ = sleeping_body();
}

/// `is_sleeping` then a strong `wake_up`.
#[test]
fn gas_wake_up() {
    let mut body = sleeping_body();
    let _ = opaque(body.is_sleeping());
    body.wake_up(opaque(true));
}

/// `add_force`, `add_torque`, `add_force_at_point`, `reset_forces`, `reset_torques`.
#[test]
fn gas_forces() {
    let mut body = sleeping_body();
    let f = opaque(Vec2 { x: ONE, y: ZERO });
    body.add_force(f, true);
    body.add_torque(ONE, true);
    body.add_force_at_point(f, Vec2 { x: ZERO, y: ONE }, true);
    body.reset_forces(true);
    body.reset_torques(true);
}

/// `apply_impulse`, `apply_torque_impulse`, `apply_impulse_at_point`.
#[test]
fn gas_impulses() {
    let mut body = sleeping_body();
    let i = opaque(Vec2 { x: ONE, y: ZERO });
    body.apply_impulse(i, true);
    body.apply_torque_impulse(ONE, true);
    body.apply_impulse_at_point(i, Vec2 { x: ZERO, y: ONE }, true);
}
