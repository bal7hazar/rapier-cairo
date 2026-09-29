//! The World edits a game applies between steps, in a declared class (CS7): `WorldEditClass`
//! inserts a body with its collider and initial velocities (a projectile), removes bodies and puts
//! bodies to sleep. Compiled next to the step, these edits cost a caller class 16.9k (an
//! insertion), 9.1k (a removal) and 10.3k (a sleep) CASM felts (slingfall's S36a); here the world
//! crosses in and out with the basic codec (`rapier2d::world::basic_state`), whose reader and
//! writer the caller class of `SlimSplitStages` already compiles.
//!
//! A caller calls [`edit_world`] on the steps that edit, never on the others. The edits are applied
//! in order by the same `World` methods as in process ([`apply_edits`]), so the world that comes
//! back is the one the caller would have (`tests/edits.cairo`: the pile10 shot's launch,
//! destructions and sleeps, every tick).

use rapier2d::prelude::{Fixed, Handle, Pose2, Shape, Vec2};
use rapier2d::world::basic_state::{
    BasicWorldState, deserialize_basic_shape, from_basic_state, into_basic_state,
    serialize_basic_shape,
};
use rapier2d::world::{World, WorldTrait};
use rapier_core::collider::ActiveEvents;
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::collider::ColliderBuilderTrait;
use rapier_dynamics2d::rigid_body_set::{RigidBodyBuilderTrait, RigidBodyTrait};
use starknet::syscalls::library_call_syscall;
use starknet::{ClassHash, SyscallResultTrait};
use crate::hashes::errors;

/// A body to insert with one collider: `RigidBodyBuilderTrait::new(body_type)` at `position` with
/// `linvel` and `angvel`, and a collider of `shape` (a basic shape) with the given density,
/// friction, restitution, contact-force event threshold, active events and user data, every other
/// field at the builders' defaults.
#[derive(Copy, Drop, PartialEq, Debug)]
pub struct BodyInsert {
    pub body_type: RigidBodyType,
    pub position: Pose2,
    pub linvel: Vec2,
    pub angvel: Fixed,
    pub shape: Shape,
    pub density: Fixed,
    pub friction: Fixed,
    pub restitution: Fixed,
    pub contact_force_event_threshold: Fixed,
    pub active_events: ActiveEvents,
    pub user_data: u128,
}

/// The crossing of a [`BodyInsert`], its fields in order, the shape by the basic codec.
///
/// # Panics
/// `'State: not a basic shape'` on another shape.
pub impl BodyInsertSerde of Serde<BodyInsert> {
    fn serialize(self: @BodyInsert, ref output: Array<felt252>) {
        self.body_type.serialize(ref output);
        self.position.serialize(ref output);
        self.linvel.serialize(ref output);
        self.angvel.serialize(ref output);
        serialize_basic_shape(self.shape, ref output);
        self.density.serialize(ref output);
        self.friction.serialize(ref output);
        self.restitution.serialize(ref output);
        self.contact_force_event_threshold.serialize(ref output);
        self.active_events.serialize(ref output);
        self.user_data.serialize(ref output);
    }

    fn deserialize(ref serialized: Span<felt252>) -> Option<BodyInsert> {
        Some(
            BodyInsert {
                body_type: Serde::deserialize(ref serialized)?,
                position: Serde::deserialize(ref serialized)?,
                linvel: Serde::deserialize(ref serialized)?,
                angvel: Serde::deserialize(ref serialized)?,
                shape: deserialize_basic_shape(ref serialized)?,
                density: Serde::deserialize(ref serialized)?,
                friction: Serde::deserialize(ref serialized)?,
                restitution: Serde::deserialize(ref serialized)?,
                contact_force_event_threshold: Serde::deserialize(ref serialized)?,
                active_events: Serde::deserialize(ref serialized)?,
                user_data: Serde::deserialize(ref serialized)?,
            },
        )
    }
}

/// One World edit.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum WorldEdit {
    /// `WorldTrait::insert` of the body and collider of a [`BodyInsert`].
    Insert: BodyInsert,
    /// `WorldTrait::remove_body` (nothing when the handle does not resolve).
    Remove: Handle,
    /// `RigidBodyTrait::sleep` then `WorldTrait::set_body`, unless the body already sleeps
    /// (nothing when the handle does not resolve).
    Sleep: Handle,
}

/// Applies `edits` to `world` in order, in process; returns the handles of the inserted bodies,
/// in the order of their edits.
pub fn apply_edits(ref world: World, edits: Span<WorldEdit>) -> Array<Handle> {
    let mut inserted = array![];
    for edit in edits {
        match *edit {
            WorldEdit::Insert(insert) => { inserted.append(insert_body(ref world, insert)); },
            WorldEdit::Remove(handle) => { let _ = world.remove_body(handle); },
            WorldEdit::Sleep(handle) => {
                if let Some(sleeping) = world.is_sleeping(handle) {
                    if !sleeping {
                        let mut body = world.body(handle).unwrap();
                        body.sleep();
                        let _ = world.set_body(handle, body);
                    }
                }
            },
        }
    }
    inserted
}

/// The insertion of [`WorldEdit::Insert`].
fn insert_body(ref world: World, insert: BodyInsert) -> Handle {
    let body = RigidBodyBuilderTrait::new(insert.body_type)
        .position(insert.position)
        .linvel(insert.linvel)
        .angvel(insert.angvel)
        .build();
    let collider = ColliderBuilderTrait::new(insert.shape)
        .density(insert.density)
        .friction(insert.friction)
        .restitution(insert.restitution)
        .contact_force_event_threshold(insert.contact_force_event_threshold)
        .active_events(insert.active_events)
        .user_data(insert.user_data)
        .build();
    let (handle, _) = world.insert(body, collider);
    handle
}

/// `apply_edits` on `world` in `WorldEditClass` at `class_hash` (the caller's side): the world
/// crosses in and out with the basic codec, the edits as the felts of their `Span<WorldEdit>`
/// (serialized where they are decided, e.g. in a rules class, and forwarded as they are: the
/// caller class compiles no code of the edits). Returns the edited world and the inserted handles.
///
/// # Panics
/// `errors::DECODE` when the class returns something else than its result; as the basic codec
/// (another shape, a joint) and as the class (felts that are not a `Span<WorldEdit>`).
pub fn edit_world(
    class_hash: ClassHash, world: World, edits: Span<felt252>,
) -> (World, Array<Handle>) {
    let mut calldata = array![];
    into_basic_state(world).serialize(ref calldata);
    calldata.append_span(edits);
    let mut ret = library_call_syscall(class_hash, selector!("edit"), calldata.span())
        .unwrap_syscall();
    let state: BasicWorldState = Serde::deserialize(ref ret).expect(errors::DECODE);
    let inserted: Array<Handle> = Serde::deserialize(ref ret).expect(errors::DECODE);
    (from_basic_state(state), inserted)
}

/// The World edits of a basic world ([`apply_edits`]), the world in and out with the basic
/// codec.
#[starknet::contract]
pub mod WorldEditClass {
    use rapier2d::prelude::Handle;
    use rapier2d::world::basic_state::{BasicWorldState, from_basic_state, into_basic_state};
    use super::WorldEdit;

    #[storage]
    struct Storage {}

    /// [`super::apply_edits`] on `world`: the edited world and the inserted handles.
    #[external(v0)]
    fn edit(
        self: @ContractState, world: BasicWorldState, edits: Span<WorldEdit>,
    ) -> (BasicWorldState, Array<Handle>) {
        let mut world = from_basic_state(world);
        let inserted = super::apply_edits(ref world, edits);
        (into_basic_state(world), inserted)
    }
}
