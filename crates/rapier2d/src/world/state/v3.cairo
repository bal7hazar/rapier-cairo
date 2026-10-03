//! Version 3 of the world state (CC2 to WS3), kept so that its felts still decode (WS3).
//!
//! [`WorldStateV3`] has the version-3 layout: its derived `Serde` reads and writes the felts a
//! version-3 world state wrote. [`migrate`] turns it into the current [`WorldState`] and
//! [`downgrade`] does the converse:
//!
//! * version 3 kept the dormant pairs of a valid active set in the pair list; version 4 keeps
//!   them in [`ActiveSet::dormant`]. Both keep the positions of the other pairs in the whole
//!   list (`ActiveSet::pairs`), so the split is exact: [`migrate`] takes the pairs that are not
//!   at those positions out of a valid set's list, [`downgrade`] merges them back;
//! * an invalid set holds no dormant pair in either version.
//!
//! A migrated state is the state a version-4 world built and stepped the same way saves, and
//! steps to the same bits and events (`tests`, `crates/rapier2d/tests/world_state.cairo`).

use fixed::Fixed;
use glam_core::Vec2;
use rapier_core::Handle;
use rapier_core::data::arena::ArenaState;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::joint::ImpulseJoint;
use rapier_dynamics2d::narrow_phase::NarrowPhase;
use rapier_dynamics2d::rigid_body_set::RigidBody;
use rapier_geometry2d::broad_phase::BroadPhaseProxy;
use crate::pipeline::active_set::ActiveSet;
use crate::pipeline::merge_pairs;
use super::WorldState;

/// The layout version of [`WorldStateV3`].
pub const VERSION: u32 = 3;

/// The active set of version 3: [`ActiveSet`] without `dormant` (its dormant pairs were in the
/// pair list).
#[derive(Drop, Clone, Serde, PartialEq, Debug)]
pub struct ActiveSetV3 {
    pub valid: bool,
    pub bodies: Array<Handle>,
    pub colliders: Array<(Handle, u32)>,
    pub statics: Array<BroadPhaseProxy>,
    /// Positions, ascending, of the pairs of the pair list that are not dormant.
    pub pairs: Array<u32>,
    pub sleeping: u32,
    pub force_events: bool,
    pub prediction: Fixed,
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
    pub impulse_joints: ArenaState<ImpulseJoint>,
    /// Every contact and intersection pair, dormant ones included, ascending key.
    pub narrow_phase: NarrowPhase,
    pub active_set: ActiveSetV3,
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
    let ActiveSetV3 {
        valid,
        bodies: members,
        colliders: active,
        statics,
        pairs,
        sleeping,
        force_events,
        prediction,
    } = active_set;
    let (narrow_phase, dormant) = if valid {
        let (live, dormant) = crate::pipeline::active_set::split_at_positions(
            narrow_phase.pairs.span(), pairs.span(),
        );
        (NarrowPhase { pairs: live }, dormant)
    } else {
        (narrow_phase, array![])
    };
    WorldState {
        version: super::WORLD_STATE_VERSION,
        gravity,
        integration_parameters,
        bodies,
        colliders,
        impulse_joints,
        narrow_phase,
        active_set: ActiveSet {
            valid,
            bodies: members,
            colliders: active,
            statics,
            pairs,
            sleeping,
            force_events,
            prediction,
            dormant,
        },
    }
}

/// The version-3 state of `state` (see the module documentation): its felts are those a
/// version-3 codec wrote for the same world.
pub fn downgrade(state: @WorldState) -> WorldStateV3 {
    let set = state.active_set;
    let mut pairs = array![];
    if set.dormant.is_empty() {
        pairs.append_span(state.narrow_phase.pairs.span());
    } else {
        pairs = merge_pairs(state.narrow_phase.pairs.span(), set.dormant.span());
    }
    WorldStateV3 {
        version: VERSION,
        gravity: *state.gravity,
        integration_parameters: *state.integration_parameters,
        bodies: *state.bodies,
        colliders: *state.colliders,
        impulse_joints: *state.impulse_joints,
        narrow_phase: NarrowPhase { pairs },
        active_set: ActiveSetV3 {
            valid: *set.valid,
            bodies: set.bodies.clone(),
            colliders: set.colliders.clone(),
            statics: set.statics.clone(),
            pairs: set.pairs.clone(),
            sleeping: *set.sleeping,
            force_events: *set.force_events,
            prediction: *set.prediction,
        },
    }
}
