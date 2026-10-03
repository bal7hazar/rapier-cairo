//! The basic codec writes and reads `WorldState`'s felts, and rejects what it does not compile.

use fixed::{FixedTrait, HALF, ONE, ZERO};
use rapier_core::interaction_groups::{Group, InteractionGroupsTrait, InteractionTestMode};
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::joint::RevoluteJointBuilderTrait;
use rapier_dynamics2d::rigid_body::RigidBodyMassProps;
use rapier_dynamics2d::rigid_body_set::{RigidBodyBuilderTrait, RigidBodyTrait};
use rapier_geometry2d::mass::MassPropertiesTrait;
use rapier_testing::opaque;
use crate::pipeline::active_set::ActiveSet;
use crate::pipeline::config::BasicStepConfig;
use crate::pipeline::config::tests::{at, ball_over, basic_level, v};
use crate::world::{World, WorldTrait};
use super::decode::{read_active_set, read_body_mass_props};
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

/// CS7: `basic_level(1)` with what the levels leave at their defaults: a body with additional
/// mass (cold data) and CCD enabled (its extra slot), locked rotations, colliders given a mass
/// and mass properties, a one-way platform, interaction groups in `Or` mode, a parentless
/// collider and a removed body (free slots), stepped so that pairs and the active set exist.
fn rare_layouts() -> World {
    let mut world = basic_level(1);
    let body = RigidBodyBuilderTrait::dynamic()
        .position(at(ONE, FixedTrait::from_int(3)))
        .additional_mass(HALF)
        .ccd_enabled(true)
        .lock_rotations()
        .user_data(7)
        .build();
    let groups = InteractionGroupsTrait::new(
        Group { bits: 3 }, Group { bits: 5 }, InteractionTestMode::Or,
    );
    let _ = world
        .insert(
            body,
            ColliderBuilderTrait::cuboid(HALF, HALF).mass(ONE).collision_groups(groups).build(),
        );
    let heavy = RigidBodyBuilderTrait::dynamic()
        .position(at(FixedTrait::from_int(3), FixedTrait::from_int(3)))
        .build();
    let props = MassPropertiesTrait::new(v(ZERO, ZERO), FixedTrait::from_int(2), ONE);
    let _ = world.insert(heavy, ColliderBuilderTrait::ball(HALF).mass_properties(props).build());
    let platform = RigidBodyBuilderTrait::fixed().position(at(ZERO, ZERO)).build();
    let _ = world
        .insert(
            platform,
            ColliderBuilderTrait::cuboid(FixedTrait::from_int(2), HALF)
                .one_way(v(ZERO, ONE), HALF)
                .build(),
        );
    let _ = world.insert_collider(ColliderBuilderTrait::ball(HALF).build(), None);
    let gone = RigidBodyBuilderTrait::dynamic()
        .position(at(FixedTrait::from_int(5), FixedTrait::from_int(5)))
        .build();
    let (handle, _) = world.insert(gone, ColliderBuilderTrait::ball(HALF).build());
    let _ = world.remove_body(handle);
    let mut k = 0;
    while k != 4 {
        let _ = world.step_with_force_events_with::<BasicStepConfig>();
        k += 1;
    }
    world
}

/// The reader (`decode`) gives the values `WorldState`'s derived `Serde` gives on the layouts the
/// levels leave out, and `None` on every truncation of a state (as the derived `Serde`).
#[test]
fn test_basic_codec_reads_rare_layouts() {
    let mut world = rare_layouts();
    let expected = felts(@world.to_state());
    let mut span = expected.span();
    let read: BasicWorldState = Serde::deserialize(ref span).unwrap();
    assert!(span.is_empty());
    assert!(read.state == world.to_state());
    for cut in array![1_u32, 2, 30, 31, expected.len() / 2, expected.len() - 1] {
        let mut short = expected.span().slice(0, cut);
        let got: Option<BasicWorldState> = Serde::deserialize(ref short);
        assert!(got.is_none(), "cut {}", cut);
    }
}

/// The active set and a body's mass properties of a stepped level, as felts.
fn reader_felts() -> (Array<felt252>, Array<felt252>) {
    let mut world = stepped(opaque(2), 12);
    let state = world.to_state();
    let (_, body) = *state.bodies.entries[1];
    (felts(@state.active_set), felts(@body.mprops))
}

#[test]
fn test_readers_as_derived() {
    let (set, mprops) = reader_felts();
    let (mut a, mut b) = (set.span(), set.span());
    let derived: ActiveSet = Serde::deserialize(ref a).unwrap();
    assert!(read_active_set(ref b).unwrap() == derived);
    let (mut a, mut b) = (mprops.span(), mprops.span());
    let derived: RigidBodyMassProps = Serde::deserialize(ref a).unwrap();
    assert!(read_body_mass_props(ref b).unwrap() == derived);
}

#[test]
fn gas_read_active_set() {
    let (set, _) = reader_felts();
    let mut span = set.span();
    let _ = opaque(read_active_set(ref span).unwrap().sleeping);
}

#[test]
fn gas_read_active_set_derived() {
    let (set, _) = reader_felts();
    let mut span = set.span();
    let read: ActiveSet = Serde::deserialize(ref span).unwrap();
    let _ = opaque(read.sleeping);
}

#[test]
fn gas_read_body_mass_props() {
    let (_, mprops) = reader_felts();
    let mut span = mprops.span();
    let _ = opaque(read_body_mass_props(ref span).unwrap().max_extent);
}

#[test]
fn gas_read_body_mass_props_derived() {
    let (_, mprops) = reader_felts();
    let mut span = mprops.span();
    let read: RigidBodyMassProps = Serde::deserialize(ref span).unwrap();
    let _ = opaque(read.max_extent);
}

/// WS3: the basic codec rejects version-3 felts (its caller classes compile no migration); the
/// same felts read by `WorldState`'s `Serde` migrate, and the basic codec reads their re-encoding.
#[test]
#[should_panic(expected: 'world state: version')]
fn test_basic_codec_rejects_version_3() {
    let mut world = stepped(2, 12);
    let v3 = felts(@crate::world::state::v3::downgrade(@world.to_state()));
    let mut span = v3.span();
    let migrated: crate::world::state::WorldState = Serde::deserialize(ref span).unwrap();
    let current = felts(@migrated);
    let mut span = current.span();
    let basic: BasicWorldState = Serde::deserialize(ref span).unwrap();
    assert!(basic.state == migrated);
    let mut span = v3.span();
    let _: Option<BasicWorldState> = Serde::deserialize(ref span);
}
