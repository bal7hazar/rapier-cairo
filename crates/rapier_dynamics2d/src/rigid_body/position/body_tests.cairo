//! Body mutation API checks, separated to keep rigid_body_set below the source budget.
use fixed::{Fixed, ONE, TWO, ZERO};
use glam::Vec2;
use rapier_core::rigid_body::changes::{DOMINANCE, POSITION, SLEEP};
use rapier_core::rigid_body::{RigidBodyActivationTrait, RigidBodyChangesTrait, RigidBodyType};
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use crate::rigid_body::{RigidBodyMassPropsTrait, RigidBodyVelocityTrait};
use crate::rigid_body_set::{RigidBody, RigidBodyBuilderTrait, RigidBodyTrait};
fn at(x: Fixed, y: Fixed) -> Pose2 {
    Pose2 { translation: Vec2 { x, y }, ..Default::default() }
}
#[test]
fn test_constructors() {
    // (body, type, dynamic, fixed, kinematic)
    let cases: Array<(RigidBody, RigidBodyType, bool, bool, bool)> = array![
        (RigidBodyTrait::dynamic(at(ONE, TWO)), RigidBodyType::Dynamic, true, false, false),
        (RigidBodyTrait::fixed(at(ONE, TWO)), RigidBodyType::Fixed, false, true, false),
        (
            RigidBodyTrait::kinematic_velocity_based(at(ONE, TWO)),
            RigidBodyType::KinematicVelocityBased,
            false,
            false,
            true,
        ),
        (
            RigidBodyTrait::kinematic_position_based(at(ONE, TWO)),
            RigidBodyType::KinematicPositionBased,
            false,
            false,
            true,
        ),
    ];
    for (body, body_type, dynamic, fixed, kinematic) in cases {
        assert_eq!(body.body_type, body_type);
        assert_eq!(body.is_dynamic(), dynamic);
        assert_eq!(body.is_fixed(), fixed);
        assert_eq!(body.is_kinematic(), kinematic);
        assert_eq!(body.position(), at(ONE, TWO));
        assert_eq!(body.world_com(), Vec2 { x: ONE, y: TWO });
        assert!(body.changes.is_empty());
        assert_eq!(body.colliders.len(), 0);
        assert!(body.enabled);
    }
}

#[test]
fn test_setters() {
    let mut body = RigidBodyTrait::dynamic(at(ZERO, ZERO));
    body.set_position(at(ZERO, ZERO));
    assert!(!body.changes.contains(POSITION));
    body.set_position(at(ONE, ZERO));
    assert!(body.changes.contains(POSITION));
    assert_eq!(body.pos.next_position, at(ONE, ZERO));
    assert_eq!(body.world_com(), Vec2 { x: ONE, y: ZERO });
    body.set_linvel(Vec2 { x: TWO, y: ONE });
    body.set_angvel(ONE);
    assert_eq!(body.linvel(), Vec2 { x: TWO, y: ONE });
    assert_eq!(body.vels.angvel, ONE);
}

/// `sleep` zeroes the velocities and fills the timer; every setter wakes the body up
/// (strongly: timer reset, `SLEEP` raised when it was asleep), as upstream with `wake_up =
/// true`; a weak wake-up keeps the timer.
#[test]
fn test_sleep_and_wake_up() {
    let mut body = RigidBodyTrait::dynamic(at(ZERO, ZERO));
    body.set_linvel(Vec2 { x: ONE, y: ONE });
    body.changes = RigidBodyChangesTrait::empty();
    body.sleep();
    assert!(body.is_sleeping());
    assert!(body.activation.is_eligible_for_sleep());
    assert_eq!(body.vels, RigidBodyVelocityTrait::zero());
    let mut weak = body;
    weak.wake_up(false);
    assert!(!weak.is_sleeping() && weak.changes.contains(SLEEP));
    assert!(weak.activation.is_eligible_for_sleep(), "weak: timer kept");
    // (setter, expected linvel.x after it) on a sleeping body.
    let mut n = 0;
    while n != 3 {
        let mut b = body;
        if n == 0 {
            b.set_position(at(ONE, ZERO));
        } else if n == 1 {
            b.set_linvel(Vec2 { x: ONE, y: ZERO });
        } else {
            b.set_angvel(ONE);
        }
        assert!(!b.is_sleeping() && b.changes.contains(SLEEP), "setter {} wakes", n);
        assert_eq!(b.activation.time_since_can_sleep, ZERO, "setter {} resets", n);
        n += 1;
    }
    // Waking an awake body raises nothing.
    let mut awake = RigidBodyTrait::dynamic(at(ZERO, ZERO));
    awake.wake_up(true);
    assert!(awake.changes.is_empty());
}

/// Force and impulse helpers: dynamic bodies only, zero inputs do nothing, the flag decides
/// the wake-up. Rows: (helper, dynamic body, zero input); a row applies iff dynamic and not
/// zero, and then wakes the sleeping body.
#[test]
fn test_forces_and_impulses_wake_dynamic_bodies() {
    let mut dynamic = RigidBodyTrait::dynamic(at(ZERO, ZERO));
    // Unit mass and inertia at the origin: impulses map one to one.
    dynamic.mprops.local_mprops.inv_mass = ONE;
    dynamic.mprops.local_mprops.inv_principal_inertia = ONE;
    dynamic
        .mprops = dynamic
        .mprops
        .update_world_mass_properties(RigidBodyType::Dynamic, at(ZERO, ZERO));
    dynamic.sleep();
    let mut fixed = RigidBodyTrait::fixed(at(ZERO, ZERO));
    fixed.sleep();
    let rows: Array<(u8, bool, bool)> = array![
        (0, true, false), (0, true, true), (0, false, false), (1, true, false), (1, true, true),
        (1, false, false), (2, true, false), (3, true, false), (3, true, true), (3, false, false),
        (4, true, false), (4, false, false), (5, true, false),
    ];
    let point = Vec2 { x: ONE, y: ZERO };
    for (helper, is_dynamic, zero) in rows {
        let mut b = if is_dynamic {
            dynamic
        } else {
            fixed
        };
        let v = if zero {
            Vec2 { x: ZERO, y: ZERO }
        } else {
            Vec2 { x: ONE, y: TWO }
        };
        let t = if zero {
            ZERO
        } else {
            TWO
        };
        let applied = is_dynamic && !zero;
        let (force, torque, linvel, angvel) = if helper == 0 {
            b.add_force(v, true);
            (v, ZERO, ZERO, ZERO)
        } else if helper == 1 {
            b.add_torque(t, true);
            (Vec2 { x: ZERO, y: ZERO }, t, ZERO, ZERO)
        } else if helper == 2 {
            // Torque of (1, 2) at (1, 0) about the origin: 1·2 − 0·1 = 2.
            b.add_force_at_point(v, point, true);
            (v, TWO, ZERO, ZERO)
        } else if helper == 3 {
            b.apply_impulse(v, true);
            (Vec2 { x: ZERO, y: ZERO }, ZERO, v.x, ZERO)
        } else if helper == 4 {
            b.apply_torque_impulse(t, true);
            (Vec2 { x: ZERO, y: ZERO }, ZERO, ZERO, t)
        } else {
            b.apply_impulse_at_point(v, point, true);
            (Vec2 { x: ZERO, y: ZERO }, ZERO, v.x, TWO)
        };
        let (force, torque, linvel, angvel) = if applied {
            (force, torque, linvel, angvel)
        } else {
            (Vec2 { x: ZERO, y: ZERO }, ZERO, ZERO, ZERO)
        };
        assert_eq!(b.forces.user_force, force, "force of {} {} {}", helper, is_dynamic, zero);
        assert_eq!(b.forces.user_torque, torque, "torque of {}", helper);
        assert_eq!(b.vels.linvel.x, linvel, "linvel of {}", helper);
        assert_eq!(b.vels.angvel, angvel, "angvel of {}", helper);
        assert_eq!(!b.is_sleeping(), applied, "wake of {} {} {}", helper, is_dynamic, zero);
    }
    // `wake_up = false` leaves the body asleep; the resets wake only when something was set.
    let mut quiet = dynamic;
    quiet.add_force(point, false);
    quiet.apply_impulse(point, false);
    quiet.reset_torques(true);
    assert!(quiet.is_sleeping(), "nothing to reset");
    quiet.reset_forces(true);
    assert!(!quiet.is_sleeping() && quiet.forces.user_force == Vec2 { x: ZERO, y: ZERO });
    let mut spun = dynamic;
    spun.add_torque(ONE, false);
    spun.reset_torques(true);
    assert!(!spun.is_sleeping() && spun.forces.user_torque == ZERO);
}


#[test]
fn test_kinematic_setter_types_and_wake_rules() {
    for kind in array![
        RigidBodyType::Fixed, RigidBodyType::Dynamic, RigidBodyType::KinematicPositionBased,
        RigidBodyType::KinematicVelocityBased,
    ] {
        let mut b = RigidBodyTrait::new(kind, at(ZERO, ZERO));
        b.sleep();
        b.set_next_kinematic_position(at(ZERO, ZERO));
        assert!(b.is_sleeping());
        b.set_next_kinematic_translation(Vec2 { x: -ONE, y: TWO });
        let kine = b.is_kinematic();
        assert_eq!(!b.is_sleeping(), kine);
        let target = b.next_position();
        assert_eq!(target.translation.x, if kine {
            -ONE
        } else {
            ZERO
        });
        assert_eq!(b.position(), at(ZERO, ZERO));
        let rotation = Rot2 { re: ZERO, im: ONE };
        b.set_next_kinematic_rotation(rotation);
        assert_eq!(b.next_position().translation, target.translation);
        assert_eq!(b.next_position().rotation, if kine {
            rotation
        } else {
            Default::default()
        });
        b.set_next_kinematic_position(at(TWO, ONE));
        assert_eq!(b.next_position(), if kine {
            at(TWO, ONE)
        } else {
            at(ZERO, ZERO)
        });
        b.sleep();
        b.set_linvel(Vec2 { x: ONE, y: ZERO });
        b.set_angvel(-ONE);
        let velocity = kind == RigidBodyType::Dynamic
            || kind == RigidBodyType::KinematicVelocityBased;
        assert_eq!(!b.is_sleeping(), velocity);
        assert_eq!(b.vels.angvel, if velocity {
            -ONE
        } else {
            ZERO
        });
    }
}
#[test]
fn test_dominance_changes_and_builder() {
    for group in array![-128_i8, -1_i8, 0_i8, 127_i8] {
        let mut body = RigidBodyBuilderTrait::dynamic().dominance_group(group).build();
        assert_eq!(body.dominance_group(), group);
        body.changes = RigidBodyChangesTrait::empty();
        body.set_dominance_group(group);
        assert!(body.changes.is_empty());
        body.set_dominance_group(if group == 0 {
            1
        } else {
            0
        });
        assert!(body.changes.contains(DOMINANCE));
    }
}
#[test]
fn gas_baseline() {
    let _ = opaque(ONE);
}
#[test]
fn gas_kinematic_velocity_based() {
    let _ = opaque(RigidBodyTrait::kinematic_velocity_based(opaque(at(ONE, ZERO))));
}
#[test]
fn gas_next_kinematic_position() {
    let mut b = opaque(RigidBodyTrait::kinematic_position_based(at(ZERO, ZERO)));
    b.set_next_kinematic_position(opaque(at(ONE, TWO)));
    let _ = opaque(b.next_position());
}
#[test]
fn gas_next_kinematic_translation() {
    let mut b = opaque(RigidBodyTrait::kinematic_position_based(at(ZERO, ZERO)));
    b.set_next_kinematic_translation(opaque(Vec2 { x: ONE, y: TWO }));
    let _ = opaque(b);
}
#[test]
fn gas_next_kinematic_rotation() {
    let mut b = opaque(RigidBodyTrait::kinematic_position_based(at(ZERO, ZERO)));
    b.set_next_kinematic_rotation(opaque(Rot2 { re: ZERO, im: ONE }));
    let _ = opaque(b);
}
#[test]
fn gas_dominance_group() {
    let mut b = opaque(RigidBodyTrait::dynamic(at(ZERO, ZERO)));
    b.set_dominance_group(opaque(127));
    let _ = opaque(b.dominance_group());
}
#[test]
fn gas_velocity_setters() {
    let mut b = opaque(RigidBodyTrait::kinematic_velocity_based(at(ZERO, ZERO)));
    b.set_linvel(opaque(Vec2 { x: ONE, y: TWO }));
    b.set_angvel(opaque(ONE));
    let _ = opaque(b);
}
#[test]
fn gas_builder() {
    for b in array![
        RigidBodyBuilderTrait::new(opaque(RigidBodyType::Dynamic)),
        RigidBodyBuilderTrait::dynamic(), RigidBodyBuilderTrait::fixed(),
        RigidBodyBuilderTrait::kinematic_position_based(),
        RigidBodyBuilderTrait::kinematic_velocity_based(),
    ] {
        let _ = opaque(b.position(opaque(at(ONE, TWO))).dominance_group(opaque(-128)).build());
    }
}
