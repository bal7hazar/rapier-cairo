//! Version 3 of the world state (CC2 to WS3), kept so that its felts still decode (WS3).
//!
//! [`WorldStateV3`] has the version-3 layout: its derived `Serde` reads and writes the felts a
//! version-3 world state wrote. [`migrate`] turns it into the current [`WorldState`] and
//! [`downgrade`] does the converse. Version 3 had no dormant pairs apart: a migrated state keeps
//! every pair in its list and does not keep them apart (`DormantPairs::apart` is `false`), the
//! state a version-4 world built and stepped the same way saves; [`downgrade`] merges the dormant
//! pairs kept apart back into the list (the active set's positions are those of the whole list in
//! both versions). Version 3 kept no removed colliders (`ColliderSetTrait::take_removed`): a
//! migrated state has none, and [`downgrade`] leaves them out. Version 3's joints had no user data
//! ([`GenericJointV3`]): a migrated joint's is `0`, and [`downgrade`] leaves it out.
//!
//! The migrated world steps to the same bits and events as the uninterrupted one
//! (`crates/rapier2d/tests/world_state.cairo`, `test_migrated_*`).

use fixed::Fixed;
use glam_core::Vec2;
use rapier_core::Handle;
use rapier_core::data::arena::ArenaState;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_core::integration_parameters::spring::SpringCoefficients;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::joint::{
    GenericJoint, ImpulseJoint, JointAxesMask, JointEnabled, JointLimits, JointMotor,
};
use rapier_dynamics2d::narrow_phase::NarrowPhase;
use rapier_dynamics2d::rigid_body_set::RigidBody;
use rapier_math::pose2::Pose2;
use crate::pipeline::active_set::ActiveSet;
use crate::pipeline::merge_pairs;
use super::WorldState;

/// The layout version of [`WorldStateV3`].
pub const VERSION: u32 = 3;

/// A joint's data in version 3: [`GenericJoint`] without `user_data`.
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct GenericJointV3 {
    pub local_frame1: Pose2,
    pub local_frame2: Pose2,
    pub locked_axes: JointAxesMask,
    pub coupled_axes: JointAxesMask,
    pub limit_axes: JointAxesMask,
    pub motor_axes: JointAxesMask,
    pub limits: [JointLimits; 3],
    pub motors: [JointMotor; 3],
    pub softness: SpringCoefficients,
    pub contacts_enabled: bool,
    pub enabled: JointEnabled,
}

/// An impulse joint in version 3 (its data a [`GenericJointV3`]).
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct ImpulseJointV3 {
    pub body1: Handle,
    pub body2: Handle,
    pub data: GenericJointV3,
    pub impulses: [Fixed; 3],
}

/// A world state of version 3, field for field (see the module documentation).
#[derive(Drop, Serde, PartialEq, Debug)]
pub struct WorldStateV3 {
    /// [`VERSION`] when written by a version-3 codec.
    pub version: u32,
    pub gravity: Vec2,
    pub integration_parameters: IntegrationParameters,
    pub bodies: ArenaState<RigidBody>,
    pub colliders: ArenaState<Collider>,
    pub impulse_joints: ArenaState<ImpulseJointV3>,
    /// Every contact and intersection pair, dormant ones included, ascending key.
    pub narrow_phase: NarrowPhase,
    pub active_set: ActiveSet,
}

/// Panic messages of the migration.
pub mod errors {
    /// [`super::migrate`] received a state of another version than 3.
    pub const VERSION: felt252 = 'world state v3: version';
}

/// The current [`WorldState`] of a version-3 state (see the module documentation).
///
/// # Panics
/// [`errors::VERSION`] when `state.version` is not 3.
pub fn migrate(state: WorldStateV3) -> WorldState {
    let WorldStateV3 {
        version,
        gravity,
        integration_parameters,
        bodies,
        colliders,
        impulse_joints,
        narrow_phase,
        active_set,
    } = state;
    assert(version == VERSION, errors::VERSION);
    WorldState {
        version: super::WORLD_STATE_VERSION,
        gravity,
        integration_parameters,
        bodies,
        colliders,
        removed_colliders: array![],
        impulse_joints: joints_v4(impulse_joints),
        narrow_phase,
        active_set,
        dormant_apart: false,
        dormant_pairs: array![],
    }
}

/// The version-3 state of `state` (see the module documentation): its felts are those a
/// version-3 codec wrote for the same world. The switch of the dormant pairs apart is left out.
pub fn downgrade(state: @WorldState) -> WorldStateV3 {
    let mut pairs = array![];
    if state.dormant_pairs.is_empty() {
        pairs.append_span(state.narrow_phase.pairs.span());
    } else {
        pairs = merge_pairs(state.narrow_phase.pairs.span(), state.dormant_pairs.span());
    }
    WorldStateV3 {
        version: VERSION,
        gravity: *state.gravity,
        integration_parameters: *state.integration_parameters,
        bodies: *state.bodies,
        colliders: *state.colliders,
        impulse_joints: joints_v3(*state.impulse_joints),
        narrow_phase: NarrowPhase { pairs },
        active_set: state.active_set.clone(),
    }
}

/// The joint arena of version 3 in the current layout (user data `0`).
fn joints_v4(joints: ArenaState<ImpulseJointV3>) -> ArenaState<ImpulseJoint> {
    let mut entries = array![];
    for (handle, joint) in joints.entries {
        let d = *joint.data;
        let data = GenericJoint {
            local_frame1: d.local_frame1,
            local_frame2: d.local_frame2,
            locked_axes: d.locked_axes,
            coupled_axes: d.coupled_axes,
            limit_axes: d.limit_axes,
            motor_axes: d.motor_axes,
            limits: d.limits,
            motors: d.motors,
            softness: d.softness,
            contacts_enabled: d.contacts_enabled,
            enabled: d.enabled,
            user_data: 0,
        };
        let joint = ImpulseJoint {
            body1: *joint.body1, body2: *joint.body2, data, impulses: *joint.impulses,
        };
        entries.append((*handle, joint));
    }
    ArenaState {
        generation: joints.generation,
        capacity: joints.capacity,
        free_list: joints.free_list,
        entries: entries.span(),
    }
}

/// The joint arena in the version-3 layout (user data left out).
pub fn joints_v3(joints: ArenaState<ImpulseJoint>) -> ArenaState<ImpulseJointV3> {
    let mut entries = array![];
    for (handle, joint) in joints.entries {
        let d = *joint.data;
        let data = GenericJointV3 {
            local_frame1: d.local_frame1,
            local_frame2: d.local_frame2,
            locked_axes: d.locked_axes,
            coupled_axes: d.coupled_axes,
            limit_axes: d.limit_axes,
            motor_axes: d.motor_axes,
            limits: d.limits,
            motors: d.motors,
            softness: d.softness,
            contacts_enabled: d.contacts_enabled,
            enabled: d.enabled,
        };
        let joint = ImpulseJointV3 {
            body1: *joint.body1, body2: *joint.body2, data, impulses: *joint.impulses,
        };
        entries.append((*handle, joint));
    }
    ArenaState {
        generation: joints.generation,
        capacity: joints.capacity,
        free_list: joints.free_list,
        entries: entries.span(),
    }
}
