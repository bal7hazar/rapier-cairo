//! Version 3 of the world state (CC2 to WS3), kept so that its felts still decode (WS3).
//!
//! [`WorldStateV3`] has the version-3 layout: its derived `Serde` reads and writes the felts a
//! version-3 world state wrote. [`migrate`] turns it into the current [`WorldState`] and
//! [`downgrade`] does the converse. Version 3 had no dormant pairs apart: a migrated state keeps
//! every pair in its list and does not keep them apart (`DormantPairs::apart` is `false`), the
//! state a version-4 world built and stepped the same way saves; [`downgrade`] merges the dormant
//! pairs kept apart back into the list (the active set's positions are those of the whole list in
//! both versions).
//!
//! The migrated world steps to the same bits and events as the uninterrupted one
//! (`crates/rapier2d/tests/world_state.cairo`, `test_migrated_*`).

use glam_core::Vec2;
use rapier_core::data::arena::ArenaState;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::joint::ImpulseJoint;
use rapier_dynamics2d::narrow_phase::NarrowPhase;
use rapier_dynamics2d::rigid_body_set::RigidBody;
use crate::pipeline::active_set::ActiveSet;
use crate::pipeline::merge_pairs;
use super::WorldState;

/// The layout version of [`WorldStateV3`].
pub const VERSION: u32 = 3;

/// A world state of version 3, field for field (see the module documentation).
#[derive(Drop, Serde, PartialEq, Debug)]
pub struct WorldStateV3 {
    /// [`VERSION`] when written by a version-3 codec.
    pub version: u32,
    pub gravity: Vec2,
    pub integration_parameters: IntegrationParameters,
    pub bodies: ArenaState<RigidBody>,
    pub colliders: ArenaState<Collider>,
    pub impulse_joints: ArenaState<ImpulseJoint>,
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
        impulse_joints,
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
        impulse_joints: *state.impulse_joints,
        narrow_phase: NarrowPhase { pairs },
        active_set: state.active_set.clone(),
    }
}
