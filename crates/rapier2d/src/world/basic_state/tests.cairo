//! The basic codec writes and reads `WorldState`'s felts, and rejects what it does not compile.

use fixed::{HALF, ONE, ZERO};
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::joint::RevoluteJointBuilderTrait;
use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
use rapier_testing::opaque;
use crate::pipeline::config::BasicStepConfig;
use crate::pipeline::config::tests::{at, ball_over, basic_level, v};
use crate::world::{World, WorldTrait};
use super::{BasicWorldState, from_basic_state, into_basic_state};

fn felts<T, +Serde<T>, +Drop<T>>(value: @T) -> Array<felt252> {
    let mut out = array![];
    value.serialize(ref out);
    out
}

/// `basic_level(seed)` after `steps` basic steps (pairs, sleep, the active set).
fn stepped(seed: u32, steps: u32) -> World {
    let mut world = basic_level(seed);
    let mut k = 0;
    while k != steps {
        let _ = world.step_with_force_events_with::<BasicStepConfig>();
        k += 1;
    }
    world
}

/// Same felts as `WorldState` both ways, and `from_basic_state` restores what `from_state` does,
/// on levels with the four basic shapes, before the first step, in flight and asleep.
#[test]
fn test_basic_codec_same_felts() {
    for (seed, steps) in array![(0_u32, 0_u32), (1, 3), (2, 12), (5, 40)] {
        let mut world = stepped(seed, steps);
        let expected = felts(@world.to_state());
        let basic = into_basic_state(stepped(seed, steps));
        assert!(felts(@basic) == expected, "felts, level {} step {}", seed, steps);
        let mut span = expected.span();
        let read: BasicWorldState = Serde::deserialize(ref span).unwrap();
        assert!(span.is_empty());
        assert!(read.state == world.to_state(), "read, level {} step {}", seed, steps);
        let mut restored = from_basic_state(read);
        assert!(felts(@restored.to_state()) == expected, "restored, level {} step {}", seed, steps);
    }
}

/// A restored world steps as the original.
#[test]
fn test_basic_codec_steps_the_same() {
    let mut world = stepped(3, 5);
    let mut restored = from_basic_state(into_basic_state(stepped(3, 5)));
    let mut k = 0;
    while k != 20 {
        let expected = world.step_with_force_events_with::<BasicStepConfig>();
        assert!(restored.step_with_force_events_with::<BasicStepConfig>() == expected);
        k += 1;
    }
    assert!(felts(@restored.to_state()) == felts(@world.to_state()));
}

#[test]
#[should_panic(expected: 'State: not a basic shape')]
fn test_basic_codec_rejects_writing_other_shapes() {
    let world = ball_over(ColliderBuilderTrait::capsule_y(HALF, HALF));
    let _ = felts(@into_basic_state(world));
}

#[test]
#[should_panic(expected: 'State: not a basic shape')]
fn test_basic_codec_rejects_reading_other_shapes() {
    let mut world = ball_over(
        ColliderBuilderTrait::triangle(v(ZERO, ZERO), v(ONE, ZERO), v(ZERO, ONE)),
    );
    let state = felts(@world.to_state());
    let mut span = state.span();
    let _: Option<BasicWorldState> = Serde::deserialize(ref span);
}

fn with_joint() -> World {
    let mut world = basic_level(0);
    let a = world.insert_body(RigidBodyTrait::fixed(at(ZERO, ONE)));
    let b = world.insert_body(RigidBodyTrait::dynamic(at(ONE, ONE)));
    let _ = world.insert_impulse_joint(a, b, RevoluteJointBuilderTrait::new().build());
    world
}

#[test]
#[should_panic(expected: 'State: joints disabled')]
fn test_basic_codec_rejects_writing_joints() {
    let _ = felts(@into_basic_state(with_joint()));
}

#[test]
#[should_panic(expected: 'State: joints disabled')]
fn test_basic_codec_rejects_reading_joints() {
    let mut world = with_joint();
    let state = felts(@world.to_state());
    let mut span = state.span();
    let _: Option<BasicWorldState> = Serde::deserialize(ref span);
}

/// A joint arena emptied by a removal reads (same felts) but is not restored.
#[test]
#[should_panic(expected: 'State: joints disabled')]
fn test_basic_codec_rejects_restoring_used_joint_arena() {
    let mut world = basic_level(0);
    let a = world.insert_body(RigidBodyTrait::fixed(at(ZERO, ONE)));
    let b = world.insert_body(RigidBodyTrait::dynamic(at(ONE, ONE)));
    let joint = world.insert_impulse_joint(a, b, RevoluteJointBuilderTrait::new().build());
    let _ = world.remove_impulse_joint(joint);
    let state = felts(@world.to_state());
    let mut span = state.span();
    let read: BasicWorldState = Serde::deserialize(ref span).unwrap();
    assert!(felts(@read) == state);
    let _ = from_basic_state(read);
}

#[test]
fn gas_baseline() {
    let _ = opaque(1_u32);
}

/// The basic codec against `WorldState`'s on the same level (run with
/// `--tracked-resource cairo-steps`).
#[test]
fn gas_basic_state_round_trip() {
    let world = stepped(opaque(2), 12);
    let state = felts(@into_basic_state(world));
    let mut span = state.span();
    let read: BasicWorldState = Serde::deserialize(ref span).unwrap();
    let _ = from_basic_state(read);
}

#[test]
fn gas_world_state_round_trip() {
    let world = stepped(opaque(2), 12);
    let state = felts(@world.into_state());
    let mut span = state.span();
    let read: crate::world::state::WorldState = Serde::deserialize(ref span).unwrap();
    let _ = WorldTrait::from_state(read);
}
