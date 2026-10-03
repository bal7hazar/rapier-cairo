//! The [`WorldState`] codec of a basic world (work package CS6): balls, cuboids, convex polygons
//! and half-spaces, no impulse joint, the worlds `BasicStepConfig` steps.
//!
//! [`BasicWorldState`] is a [`WorldState`] whose `Serde` writes and reads **the same felts** as
//! `WorldState`'s (version 4, same layout, no new version; a version-3 state is read and migrated
//! as `WorldState`'s `Serde` does): a contract that takes and returns a `BasicWorldState` has the
//! calldata of one that takes a `WorldState`. What it does not compile
//! is the rest of the format: the `Serde` of the other shapes (triangles, rounded shapes,
//! polylines, height fields, compounds, capsules, segments) and of the impulse joints, and the
//! restore of a joint arena. That is what a caller class under the declared-class limit leaves out
//! (`docs/research/class-split.md`, CS6).
//!
//! A state the codec does not support is rejected, never altered: a collider of another shape
//! ([`errors::NOT_BASIC`], when it is read or written), a joint arena that was ever used (a joint,
//! a free slot or a removal: [`errors::JOINTS`], when it is read or restored; a joint when it is
//! written).

use rapier_core::data::arena::ArenaState;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::collider::components::BoxedOneWayPlatformSerde;
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::collider_set::access::ColliderSetChangesTrait;
use rapier_dynamics2d::joint::ImpulseJoint;
use rapier_dynamics2d::rigid_body_set::RigidBodySetTrait;
use rapier_geometry2d::shape::{ConvexPolygon, Shape};
use crate::pipeline::active_set::DormantPairs;
use super::World;
use super::state::v3::{self, WorldStateV3};
use super::state::{WORLD_STATE_VERSION, WorldState, into_state};

/// Panics of the basic codec.
pub mod errors {
    /// A collider's shape is not a ball, a cuboid, a convex polygon or a half-space.
    pub const NOT_BASIC: felt252 = 'State: not a basic shape';
    /// The joint arena is not the empty, never used one.
    pub const JOINTS: felt252 = 'State: joints disabled';
}

/// A [`WorldState`] of a basic world, serialized by the basic codec (see the module
/// documentation): the felts of `WorldState`'s `Serde`.
#[derive(Drop)]
pub struct BasicWorldState {
    pub state: WorldState,
}

/// `WorldState`'s `Serde`, field by field, with the basic shapes and no joint.
///
/// # Panics
/// [`errors::NOT_BASIC`] on another shape; [`errors::JOINTS`] when writing a joint.
pub impl BasicWorldStateSerde of Serde<BasicWorldState> {
    fn serialize(self: @BasicWorldState, ref output: Array<felt252>) {
        let state = self.state;
        state.version.serialize(ref output);
        state.gravity.serialize(ref output);
        state.integration_parameters.serialize(ref output);
        state.bodies.serialize(ref output);
        serialize_colliders(state.colliders, ref output);
        state.removed_colliders.serialize(ref output);
        serialize_joints(state.impulse_joints, ref output);
        state.narrow_phase.serialize(ref output);
        state.active_set.serialize(ref output);
        state.dormant_apart.serialize(ref output);
        state.dormant_pairs.serialize(ref output);
    }

    fn deserialize(ref serialized: Span<felt252>) -> Option<BasicWorldState> {
        let version = decode::read_version(ref serialized)?;
        let gravity = decode::read_gravity(ref serialized)?;
        let integration_parameters = decode::read_parameters(ref serialized)?;
        let bodies = decode::read_bodies(ref serialized)?;
        let colliders = decode::read_colliders(ref serialized)?;
        let removed_colliders = if version == v3::VERSION {
            array![]
        } else {
            decode::read_handles(ref serialized)?
        };
        let impulse_joints = deserialize_joints(ref serialized)?;
        let narrow_phase = Serde::deserialize(ref serialized)?;
        let active_set = decode::read_active_set(ref serialized)?;
        if version == v3::VERSION {
            // WS3: a version-3 state, migrated (no dormant pair apart).
            return Some(
                BasicWorldState {
                    state: v3::migrate(
                        WorldStateV3 {
                            version,
                            gravity,
                            integration_parameters,
                            bodies,
                            colliders,
                            impulse_joints: v3::joints_v3(impulse_joints),
                            narrow_phase,
                            active_set,
                        },
                    ),
                },
            );
        }
        Some(
            BasicWorldState {
                state: WorldState {
                    version,
                    gravity,
                    integration_parameters,
                    bodies,
                    colliders,
                    removed_colliders,
                    impulse_joints,
                    narrow_phase,
                    active_set,
                    dormant_apart: decode::read_bool(ref serialized)?,
                    dormant_pairs: Serde::deserialize(ref serialized)?,
                },
            },
        )
    }
}

/// Rebuilds the world of `state` (`WorldTrait::from_state` without the joint arena, which is the
/// empty one).
///
/// # Panics
/// As `WorldTrait::from_state`; [`errors::JOINTS`] when the joint arena was ever used.
pub fn from_basic_state(state: BasicWorldState) -> World {
    let WorldState {
        version,
        gravity,
        integration_parameters,
        bodies,
        colliders,
        removed_colliders,
        impulse_joints,
        narrow_phase,
        active_set,
        dormant_apart,
        dormant_pairs,
    } = state.state;
    assert(version == WORLD_STATE_VERSION, super::state::errors::VERSION);
    assert(
        impulse_joints.generation == 0
            && impulse_joints.capacity == 0
            && impulse_joints.free_list.is_empty()
            && impulse_joints.entries.is_empty(),
        errors::JOINTS,
    );
    let mut colliders = ColliderSetTrait::from_state(colliders);
    colliders.restore_removed(removed_colliders);
    World {
        gravity,
        integration_parameters,
        bodies: RigidBodySetTrait::from_state(bodies),
        colliders,
        impulse_joints: Default::default(),
        narrow_phase,
        active_set: BoxTrait::new(active_set),
        dormant: BoxTrait::new(DormantPairs { apart: dormant_apart, pairs: dormant_pairs }),
    }
}

/// Saves and consumes `world` (`WorldTrait::into_state`).
pub fn into_basic_state(world: World) -> BasicWorldState {
    BasicWorldState { state: into_state(world) }
}

fn serialize_colliders(colliders: @ArenaState<Collider>, ref output: Array<felt252>) {
    colliders.generation.serialize(ref output);
    colliders.capacity.serialize(ref output);
    colliders.free_list.serialize(ref output);
    let entries = *colliders.entries;
    output.append(entries.len().into());
    for (handle, collider) in entries {
        handle.serialize(ref output);
        serialize_collider(collider, ref output);
    }
}

/// The derived `Serde` of `Collider`, field by field, the shape by [`serialize_basic_shape`]: the
/// same felts for a basic shape (also the crossing of `rapier2d_classes`' `MassClass`, CS7).
///
/// # Panics
/// [`errors::NOT_BASIC`] on another shape.
pub fn serialize_collider(collider: @Collider, ref output: Array<felt252>) {
    collider.co_type.serialize(ref output);
    serialize_basic_shape(collider.shape, ref output);
    collider.mprops.serialize(ref output);
    collider.changes.serialize(ref output);
    collider.parent.serialize(ref output);
    collider.pos.serialize(ref output);
    collider.material.serialize(ref output);
    collider.flags.serialize(ref output);
    collider.contact_force_event_threshold.serialize(ref output);
    collider.one_way.serialize(ref output);
    collider.user_data.serialize(ref output);
}

/// `ShapeSerde`'s tags and payloads of the four basic shapes: ball `0`, cuboid `1`, half-space
/// `4`, convex polygon `5` (also the crossing of the classes of `rapier2d_classes`).
///
/// # Panics
/// [`errors::NOT_BASIC`] on another shape.
pub fn serialize_basic_shape(shape: @Shape, ref output: Array<felt252>) {
    match shape {
        Shape::Ball(x) => {
            output.append(0);
            x.serialize(ref output);
        },
        Shape::Cuboid(x) => {
            output.append(1);
            x.serialize(ref output);
        },
        Shape::HalfSpace(x) => {
            output.append(4);
            x.serialize(ref output);
        },
        Shape::ConvexPolygon(x) => {
            output.append(5);
            let polygon: ConvexPolygon = (*x).unbox();
            polygon.serialize(ref output);
        },
        _ => core::panic_with_felt252(errors::NOT_BASIC),
    }
}

/// The basic shapes of [`serialize_basic_shape`].
///
/// # Panics
/// [`errors::NOT_BASIC`] on another tag.
pub fn deserialize_basic_shape(ref serialized: Span<felt252>) -> Option<Shape> {
    let tag: felt252 = Serde::deserialize(ref serialized)?;
    Some(
        match tag {
            0 => Shape::Ball(Serde::deserialize(ref serialized)?),
            1 => Shape::Cuboid(Serde::deserialize(ref serialized)?),
            4 => Shape::HalfSpace(Serde::deserialize(ref serialized)?),
            5 => {
                let polygon: ConvexPolygon = Serde::deserialize(ref serialized)?;
                Shape::ConvexPolygon(BoxTrait::new(polygon))
            },
            _ => core::panic_with_felt252(errors::NOT_BASIC),
        },
    )
}

/// The joint arena's felts: its bookkeeping and no entry.
fn serialize_joints(joints: @ArenaState<ImpulseJoint>, ref output: Array<felt252>) {
    assert(joints.entries.is_empty(), errors::JOINTS);
    joints.generation.serialize(ref output);
    joints.capacity.serialize(ref output);
    joints.free_list.serialize(ref output);
    output.append(0);
}

fn deserialize_joints(ref serialized: Span<felt252>) -> Option<ArenaState<ImpulseJoint>> {
    let (generation, capacity, free_list, len) = decode::read_joint_bookkeeping(ref serialized)?;
    assert(len == 0, errors::JOINTS);
    Some(ArenaState { generation, capacity, free_list, entries: array![].span() })
}

/// The reader of the codec (CS7), also used by the crossings of `rapier2d_classes`.
pub mod decode;
#[cfg(test)]
mod tests;
