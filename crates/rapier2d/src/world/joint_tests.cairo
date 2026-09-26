//! Joint accessors of `World` (JA1): iteration, per-body queries, setters with their wake-ups.

use fixed::{HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::Handle;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::joint::{
    FixedJointBuilderTrait, GenericJointTrait, ImpulseJointTrait, RevoluteJointBuilderTrait,
};
use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
use rapier_math::pose2::Pose2;
use rapier_math::rot2::Rot2;
use rapier_testing::opaque;
use super::{World, WorldTrait};

fn at(x: fixed::Fixed) -> Pose2 {
    Pose2 { translation: Vec2 { x, y: ZERO }, rotation: Rot2 { re: ONE, im: ZERO } }
}

/// Three bodies a, b, c on a line, joined a-b (revolute) and b-c (fixed).
fn chain() -> (World, Handle, Handle, Handle, Handle, Handle) {
    let mut world = WorldTrait::new(Vec2 { x: ZERO, y: ZERO }, Default::default());
    let (a, _) = world
        .insert(RigidBodyTrait::dynamic(at(ZERO)), ColliderBuilderTrait::ball(HALF).build());
    let (b, _) = world
        .insert(RigidBodyTrait::dynamic(at(ONE + ONE)), ColliderBuilderTrait::ball(HALF).build());
    let (c, _) = world
        .insert(
            RigidBodyTrait::dynamic(at(ONE + ONE + ONE + ONE)),
            ColliderBuilderTrait::ball(HALF).build(),
        );
    let ab = world.insert_impulse_joint(a, b, RevoluteJointBuilderTrait::new().build());
    let bc = world.insert_impulse_joint(b, c, FixedJointBuilderTrait::new().build());
    (world, a, b, c, ab, bc)
}

fn put_to_sleep(ref world: World, handles: Span<Handle>) {
    for handle in handles {
        let mut body = world.body(*handle).unwrap();
        body.sleep();
        assert!(world.set_body(*handle, body));
    }
}

fn awake(ref world: World, handle: Handle) -> bool {
    !world.body(handle).unwrap().is_sleeping()
}

#[test]
fn test_impulse_joints_and_impulse_joints_with() {
    let (mut world, a, b, c, ab, bc) = chain();
    let all = world.impulse_joints();
    assert_eq!(all.len(), 2);
    let (first, joint) = *all.at(0);
    assert_eq!((first, joint.body1(), joint.body2()), (ab, a, b));
    let with_b = world.impulse_joints_with(b);
    assert_eq!(with_b.len(), 2);
    let (b1, b2, handle, _) = *with_b.at(1);
    assert_eq!((b1, b2, handle), (b, c, bc));
    assert_eq!(world.impulse_joints_with(a).len(), 1);
    // A removal drops the joint from both views.
    assert!(world.remove_impulse_joint(ab).is_some());
    assert_eq!(world.impulse_joints().len(), 1);
    assert_eq!(world.impulse_joints_with(a).len(), 0);
    assert_eq!(world.impulse_joints_with(b).len(), 1);
}

/// `set_impulse_joint` writes the data and the impulses back, keeps the bodies, and wakes the
/// attached bodies only when asked (`wake_up_connected_bodies`); a stale handle changes nothing.
#[test]
fn test_set_impulse_joint_and_wake_ups() {
    let (mut world, a, b, c, ab, bc) = chain();
    put_to_sleep(ref world, array![a, b, c].span());
    let mut joint = world.impulse_joint(ab).unwrap();
    joint.data.set_contacts_enabled(false);
    joint.impulses = [HALF, ZERO, ONE];
    // Without wake-up: written, bodies stay asleep.
    assert!(world.set_impulse_joint(ab, joint, false));
    assert_eq!(world.impulse_joint(ab), Some(joint));
    assert!(!awake(ref world, a) && !awake(ref world, b) && !awake(ref world, c));
    // The bodies of a joint cannot be rebound through the value: only data and impulses are used.
    let mut moved = joint;
    moved.body2 = c;
    assert!(world.set_impulse_joint(ab, moved, false));
    assert_eq!(world.impulse_joint(ab).unwrap().body2, b);
    // With wake-up: both bodies of the joint, not the third one.
    assert!(world.set_impulse_joint(ab, joint, true));
    assert!(awake(ref world, a) && awake(ref world, b) && !awake(ref world, c));
    // A stale handle: nothing written, nobody woken.
    assert!(world.remove_impulse_joint(bc).is_some());
    put_to_sleep(ref world, array![a, b].span());
    assert!(!world.set_impulse_joint(bc, joint, true));
    assert!(!awake(ref world, a) && !awake(ref world, b));
}

/// `set_impulse_joint_bodies` rebinds the joint (handle kept) and wakes the old and the new
/// bodies on request; a stale handle gives `None`.
#[test]
fn test_set_impulse_joint_bodies_wakes_old_and_new() {
    let (mut world, a, b, c, ab, bc) = chain();
    put_to_sleep(ref world, array![a, b, c].span());
    let rebound = world.set_impulse_joint_bodies(ab, a, c, false).unwrap();
    assert_eq!((rebound.body1, rebound.body2), (a, c));
    assert_eq!(world.impulse_joint(ab), Some(rebound));
    assert!(!awake(ref world, a) && !awake(ref world, b) && !awake(ref world, c));
    assert_eq!(world.impulse_joints_with(c).len(), 2);
    assert_eq!(world.impulse_joints_with(b).len(), 1);
    let _ = world.set_impulse_joint_bodies(bc, a, b, true).unwrap();
    assert!(awake(ref world, a) && awake(ref world, b) && awake(ref world, c));
    assert!(world.remove_impulse_joint(ab).is_some());
    assert!(world.set_impulse_joint_bodies(ab, a, b, true).is_none());
}

/// Removing a body removes its joints, and the accessors follow.
#[test]
fn test_accessors_after_body_removal() {
    let (mut world, a, b, _, ab, bc) = chain();
    assert!(world.remove_body(b).is_some());
    assert!(world.impulse_joint(ab).is_none() && world.impulse_joint(bc).is_none());
    assert_eq!(world.impulse_joints().len(), 0);
    assert_eq!(world.impulse_joints_with(a).len(), 0);
}

fn probe(op: u8) {
    let (mut world, a, b, c, ab, _) = chain();
    let joint = world.impulse_joint(ab).unwrap();
    match op {
        0 => { let _ = opaque(world.impulse_joints()); },
        1 => { let _ = opaque(world.impulse_joints_with(opaque(b))); },
        2 => { let _ = opaque(world.set_impulse_joint(opaque(ab), opaque(joint), opaque(true))); },
        3 => {
            let _ = opaque(
                world.set_impulse_joint_bodies(opaque(ab), opaque(a), opaque(c), opaque(true)),
            );
        },
        _ => {},
    }
}

#[test]
fn gas_joint_setup() {
    probe(4);
}

#[test]
fn gas_impulse_joints() {
    probe(0);
}

#[test]
fn gas_impulse_joints_with() {
    probe(1);
}

#[test]
fn gas_set_impulse_joint() {
    probe(2);
}

#[test]
fn gas_set_impulse_joint_bodies() {
    probe(3);
}
