//! Versioned save / restore of a [`World`] (requested by the programme for chunked execution:
//! state in, `K` steps, state out).
//!
//! [`WorldState`] owns every field of [`World`], which is exactly the persistent state of
//! `docs/PLAN.md` D9: gravity, integration parameters, the three sets as `ArenaState`s
//! (generation counter, capacity, free list and live entries: bodies with their activation, sleep
//! state and change flags; colliders; joints with their accumulated impulses) and the
//! narrow-phase pairs (manifolds with their warm-start impulses, event status, sensor
//! `intersecting` bits), and since version 2 the step's active set (BT2,
//! `crate::pipeline::active_set`), saved invalid when a set was written since the step that
//! filled it (the restored sets start unmodified). A persistent piece added to [`World`] later
//! (island manager, CCD) gets its field here, and [`WORLD_STATE_VERSION`] is bumped.
//!
//! [`to_state`] leaves the world as is; [`into_state`] consumes it and moves the pair list
//! instead of copying it (the end of a chunk).
//!
//! Guarantee: `from_state(to_state(w))` equals `w` field for field (the sets' `modified` flags
//! aside: they start cleared, and the active set is saved invalid when they were raised), so
//! stepping it produces the same bits and events as stepping `w`, and the sets issue the same
//! handles, removals included.
//!
//! Version policy: `version` is the first serialized felt; [`from_state`] rejects any other
//! version than [`WORLD_STATE_VERSION`]. The layout is the `Serde` of [`WorldState`], i.e. the
//! field order below; any change to it, or to the `Serde` of a stored type, bumps the version.

use glam::Vec2;
use rapier_core::data::arena::ArenaState;
use rapier_core::integration_parameters::IntegrationParameters;
use rapier_dynamics2d::collider::Collider;
use rapier_dynamics2d::collider_set::ColliderSetTrait;
use rapier_dynamics2d::joint::{ImpulseJoint, ImpulseJointSetTrait};
use rapier_dynamics2d::narrow_phase::NarrowPhase;
use rapier_dynamics2d::rigid_body_set::{RigidBody, RigidBodySetTrait};
use crate::pipeline::active_set::ActiveSet;
use super::World;

/// Layout version written by [`to_state`] and required by [`from_state`].
pub const WORLD_STATE_VERSION: u32 = 2;

/// Panic messages of the world state.
pub mod errors {
    /// `from_state` received a state of another layout version.
    pub const VERSION: felt252 = 'world state: version';
}

/// Flat, serialisable image of a [`World`]: one field per field of the world, in the same
/// order, behind a layout version.
#[derive(Drop, Serde, PartialEq, Debug)]
pub struct WorldState {
    /// Layout version, [`WORLD_STATE_VERSION`] when written by this crate.
    pub version: u32,
    pub gravity: Vec2,
    pub integration_parameters: IntegrationParameters,
    pub bodies: ArenaState<RigidBody>,
    pub colliders: ArenaState<Collider>,
    pub impulse_joints: ArenaState<ImpulseJoint>,
    /// Last step's contact and intersection pairs, ascending key.
    pub narrow_phase: NarrowPhase,
    /// The step's active set (BT2, version 2), marked invalid when a set was written since the
    /// step that filled it.
    pub active_set: ActiveSet,
}

/// Saves `world`, which is left unchanged. Cost: one dict read per allocated and per free slot
/// of each set, and a copy of the pair list.
pub fn to_state(ref world: World) -> WorldState {
    let mut pairs = array![];
    pairs.append_span(world.narrow_phase.pairs.span());
    WorldState {
        version: WORLD_STATE_VERSION,
        gravity: world.gravity,
        integration_parameters: world.integration_parameters,
        bodies: world.bodies.to_state(),
        colliders: world.colliders.to_state(),
        impulse_joints: world.impulse_joints.to_state(),
        narrow_phase: NarrowPhase { pairs },
        active_set: saved_active_set(
            world.active_set.as_snapshot().unbox().clone(),
            world.bodies.is_modified() || world.colliders.is_modified(),
        ),
    }
}

/// Saves `world` and consumes it: [`to_state`] without the copy of the pair list, for a world
/// that is dropped right after (the end of a chunk). Destructuring `World` here makes a field
/// added to the world a compile error until [`WorldState`] carries it.
pub fn into_state(world: World) -> WorldState {
    let World {
        gravity,
        integration_parameters,
        mut bodies,
        mut colliders,
        mut impulse_joints,
        narrow_phase,
        active_set,
    } = world;
    let modified = bodies.is_modified() || colliders.is_modified();
    WorldState {
        version: WORLD_STATE_VERSION,
        gravity,
        integration_parameters,
        bodies: bodies.to_state(),
        colliders: colliders.to_state(),
        impulse_joints: impulse_joints.to_state(),
        narrow_phase,
        active_set: saved_active_set(active_set.unbox(), modified),
    }
}

/// The active set as saved: invalid when a set was written since the step that filled it (the
/// restored sets start unmodified).
fn saved_active_set(active_set: ActiveSet, modified: bool) -> ActiveSet {
    let mut active_set = active_set;
    if modified {
        active_set.valid = false;
    }
    active_set
}

/// Rebuilds the world saved by [`to_state`]. Cost: one dict write per allocated slot of each set.
///
/// # Panics
/// `world state: version` when `state.version != WORLD_STATE_VERSION`; `Arena: state ...` when a
/// set image is invalid.
pub fn from_state(state: WorldState) -> World {
    let WorldState {
        version,
        gravity,
        integration_parameters,
        bodies,
        colliders,
        impulse_joints,
        narrow_phase,
        active_set,
    } = state;
    assert(version == WORLD_STATE_VERSION, errors::VERSION);
    World {
        gravity,
        integration_parameters,
        bodies: RigidBodySetTrait::from_state(bodies),
        colliders: ColliderSetTrait::from_state(colliders),
        impulse_joints: ImpulseJointSetTrait::from_state(impulse_joints),
        narrow_phase,
        active_set: BoxTrait::new(active_set),
    }
}

/// Rejected candidate (brief §3.4, kept for re-ranking): the compact form. It stores the
/// dynamic fields only (per body: poses, velocities, activation, change flags, world centre of
/// mass and inertia; per collider: world pose and change flags; per joint: impulses) plus the
/// arena bookkeeping and the whole pair list, and overlays them on a `level` state holding the
/// same live handles, rebuilt by the level code (not counted in its cost). Measured on the
/// G0-like piles against the full form: pile10 1 864 → 1 583 felts (−15 %), pile20
/// 4 896 → 3 793 (−23 %); round-trip Cairo steps 51 974 → 44 053 (−15 %) and 114 814 → 98
/// 653 (−14 %) (`gas_pile*_compact_*`): the narrow-phase pairs, which must stay whole (warm
/// start, event status, dormant pairs of sleeping bodies), are 69 % of the full pile10 state. Below
/// the 40 % bar: not shipped.
#[cfg(test)]
pub mod alternatives {
    use fixed::Fixed;
    use rapier_core::Handle;
    use rapier_core::collider::ColliderChanges;
    use rapier_core::rigid_body::{RigidBodyActivation, RigidBodyChanges};
    use rapier_dynamics2d::rigid_body::{RigidBodyPosition, RigidBodyVelocity};
    use rapier_math::pose2::Pose2;
    use super::{
        ArenaState, ColliderSetTrait, ImpulseJointSetTrait, NarrowPhase, RigidBodySetTrait, World,
        WorldState, from_state,
    };

    #[derive(Copy, Drop, Serde, PartialEq, Debug)]
    pub struct BodyDyn {
        pub pos: RigidBodyPosition,
        pub vels: RigidBodyVelocity,
        pub activation: RigidBodyActivation,
        pub changes: RigidBodyChanges,
        pub world_com: glam::Vec2,
        pub effective_world_inv_inertia: Fixed,
    }

    #[derive(Copy, Drop, Serde, PartialEq, Debug)]
    pub struct ColliderDyn {
        pub pose: Pose2,
        pub changes: ColliderChanges,
    }

    #[derive(Copy, Drop, Serde, PartialEq, Debug)]
    pub struct CompactState {
        pub bodies: ArenaState<BodyDyn>,
        pub colliders: ArenaState<ColliderDyn>,
        pub joints: ArenaState<[Fixed; 3]>,
        pub pairs: Span<rapier_dynamics2d::narrow_phase::ContactPair>,
    }

    fn arena<T, U, +Copy<T>, +Drop<T>, +Drop<U>>(
        state: ArenaState<T>, entries: Array<(Handle, U)>,
    ) -> ArenaState<U> {
        ArenaState {
            generation: state.generation,
            capacity: state.capacity,
            free_list: state.free_list,
            entries: entries.span(),
        }
    }

    pub fn to_compact(ref world: World) -> CompactState {
        let bodies = world.bodies.to_state();
        let mut b = array![];
        for (handle, body) in bodies.entries {
            b
                .append(
                    (
                        *handle,
                        BodyDyn {
                            pos: *body.pos,
                            vels: *body.vels,
                            activation: *body.activation,
                            changes: *body.changes,
                            world_com: *body.mprops.world_com,
                            effective_world_inv_inertia: *body.mprops.effective_world_inv_inertia,
                        },
                    ),
                );
        }
        let colliders = world.colliders.to_state();
        let mut c = array![];
        for (handle, collider) in colliders.entries {
            c
                .append(
                    (*handle, ColliderDyn { pose: *collider.pos.pose, changes: *collider.changes }),
                );
        }
        let joints = world.impulse_joints.to_state();
        let mut j = array![];
        for (handle, joint) in joints.entries {
            j.append((*handle, *joint.impulses));
        }
        CompactState {
            bodies: arena(bodies, b),
            colliders: arena(colliders, c),
            joints: arena(joints, j),
            pairs: world.narrow_phase.pairs.span(),
        }
    }

    /// Overlays `compact` on `level` (same live handles, entry for entry).
    pub fn from_compact(level: WorldState, compact: CompactState) -> World {
        let mut bodies = array![];
        let mut dyns = compact.bodies.entries;
        for (handle, body) in level.bodies.entries {
            let (_, d) = *dyns.pop_front().unwrap();
            let mut body = *body;
            body.pos = d.pos;
            body.vels = d.vels;
            body.activation = d.activation;
            body.changes = d.changes;
            body.mprops.world_com = d.world_com;
            body.mprops.effective_world_inv_inertia = d.effective_world_inv_inertia;
            bodies.append((*handle, body));
        }
        let mut colliders = array![];
        let mut dyns = compact.colliders.entries;
        for (handle, collider) in level.colliders.entries {
            let (_, d) = *dyns.pop_front().unwrap();
            let mut collider = *collider;
            collider.pos.pose = d.pose;
            collider.changes = d.changes;
            colliders.append((*handle, collider));
        }
        let mut joints = array![];
        let mut dyns = compact.joints.entries;
        for (handle, joint) in level.impulse_joints.entries {
            let (_, impulses) = *dyns.pop_front().unwrap();
            let mut joint = *joint;
            joint.impulses = impulses;
            joints.append((*handle, joint));
        }
        let mut pairs = array![];
        pairs.append_span(compact.pairs);
        from_state(
            WorldState {
                version: level.version,
                gravity: level.gravity,
                integration_parameters: level.integration_parameters,
                bodies: arena(compact.bodies, bodies),
                colliders: arena(compact.colliders, colliders),
                impulse_joints: arena(compact.joints, joints),
                narrow_phase: NarrowPhase { pairs },
                active_set: Default::default(),
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use fixed::{Fixed, FixedTrait, HALF, ONE, ZERO};
    use rapier_dynamics2d::collider::ColliderBuilderTrait;
    use rapier_dynamics2d::joint::RevoluteJointBuilderTrait;
    use rapier_dynamics2d::rigid_body_set::RigidBodyTrait;
    use rapier_math::pose2::Pose2;
    use rapier_math::rot2::Rot2;
    use rapier_testing::opaque;
    use super::super::WorldTrait;
    use super::{Vec2, WORLD_STATE_VERSION, World, WorldState};

    fn at(x: Fixed, y: Fixed) -> Pose2 {
        Pose2 { translation: Vec2 { x, y }, rotation: Rot2 { re: ONE, im: ZERO } }
    }

    fn i(n: u32) -> Fixed {
        FixedTrait::from_int(n.try_into().unwrap())
    }

    /// G0-like pile of `n` bodies after one step: a fixed ground cuboid, `n - 2` unit cuboids
    /// resting in three columns, and a ball fired at them.
    fn pile(n: u32) -> World {
        let mut world = WorldTrait::new(
            Vec2 { x: ZERO, y: Fixed { raw: -42133629174 } }, Default::default(),
        );
        let _ = world
            .insert(
                RigidBodyTrait::fixed(at(ZERO, -ONE)),
                ColliderBuilderTrait::cuboid(i(20), ONE).build(),
            );
        let mut k = 0;
        while k != n - 2 {
            let (column, row) = DivRem::div_rem(k, 3);
            let x = i(4) + i(row);
            let y = HALF + i(column);
            let _ = world
                .insert(
                    RigidBodyTrait::dynamic(at(x, y)),
                    ColliderBuilderTrait::cuboid(HALF, HALF).build(),
                );
            k += 1;
        }
        let mut ball = RigidBodyTrait::dynamic(at(-i(4), i(2)));
        ball.set_linvel(Vec2 { x: i(12), y: i(3) });
        let _ = world.insert(ball, ColliderBuilderTrait::ball(HALF).build());
        let _ = world.step();
        world
    }

    fn serialized(state: @WorldState) -> Array<felt252> {
        let mut out = array![];
        state.serialize(ref out);
        out
    }

    fn deserialized(felts: Span<felt252>) -> WorldState {
        let mut felts = felts;
        Serde::deserialize(ref felts).unwrap()
    }

    #[test]
    fn test_round_trip_is_identity() {
        let mut world = pile(10);
        let _ = world
            .insert_impulse_joint(
                super::super::Handle { index: 1, generation: 0 },
                super::super::Handle { index: 2, generation: 0 },
                RevoluteJointBuilderTrait::new().build(),
            );
        let _ = world.remove_body(super::super::Handle { index: 3, generation: 0 });
        let _ = world.step();
        let state = world.to_state();
        assert_eq!(state.version, WORLD_STATE_VERSION);
        assert!(state.narrow_phase.pairs.len() != 0);
        assert_eq!(state.bodies.free_list, array![3].span());
        let felts = serialized(@state);
        assert_eq!(*felts.at(0), WORLD_STATE_VERSION.into());
        let mut restored = WorldTrait::from_state(deserialized(felts.span()));
        assert_eq!(restored.to_state(), state);
        let mut original = world.into_state();
        assert_eq!(original, state);
        original.version = 0;
        assert!(original != state);
        println!("pile10: {} felts", felts.len());
        println!("pile20: {} felts", serialized(@pile(20).to_state()).len());
    }

    #[test]
    #[should_panic(expected: 'world state: version')]
    fn test_version_mismatch_panics() {
        let mut state = WorldTrait::new(Default::default(), Default::default()).to_state();
        state.version = WORLD_STATE_VERSION + 1;
        let _ = WorldTrait::from_state(state);
    }

    // Gas probes, one chain per size: `setup` (the pile after one step), then `to_state`,
    // `serialize`, `deserialize`, `from_state` added one at a time; each stage is the difference
    // with the previous probe. `into_state` compares with `to_state`.

    #[test]
    fn gas_baseline() {
        let _ = opaque(1_u32);
    }

    #[test]
    fn gas_pile10_setup() {
        let _ = pile(opaque(10));
    }

    #[test]
    fn gas_pile10_to_state() {
        let mut world = pile(opaque(10));
        let _ = world.to_state();
    }

    #[test]
    fn gas_pile10_into_state() {
        let world = pile(opaque(10));
        let _ = world.into_state();
    }

    #[test]
    fn gas_pile10_serialize() {
        let mut world = pile(opaque(10));
        let _ = serialized(@world.to_state());
    }

    #[test]
    fn gas_pile10_deserialize() {
        let mut world = pile(opaque(10));
        let _ = deserialized(serialized(@world.to_state()).span());
    }

    #[test]
    fn gas_pile10_from_state() {
        let mut world = pile(opaque(10));
        let _ = WorldTrait::from_state(deserialized(serialized(@world.to_state()).span()));
    }

    #[test]
    fn gas_pile20_setup() {
        let _ = pile(opaque(20));
    }

    #[test]
    fn gas_pile20_to_state() {
        let mut world = pile(opaque(20));
        let _ = world.to_state();
    }

    #[test]
    fn gas_pile20_into_state() {
        let world = pile(opaque(20));
        let _ = world.into_state();
    }

    #[test]
    fn gas_pile20_serialize() {
        let mut world = pile(opaque(20));
        let _ = serialized(@world.to_state());
    }

    #[test]
    fn gas_pile20_deserialize() {
        let mut world = pile(opaque(20));
        let _ = deserialized(serialized(@world.to_state()).span());
    }

    #[test]
    fn gas_pile20_from_state() {
        let mut world = pile(opaque(20));
        let _ = WorldTrait::from_state(deserialized(serialized(@world.to_state()).span()));
    }

    // Compact form (rejected, `super::alternatives`): same chain; `level` is the full state of the
    // pile before the chunk, rebuilt by the level code and not counted.

    fn compact_felts(ref world: World) -> Array<felt252> {
        let mut out = array![];
        super::alternatives::to_compact(ref world).serialize(ref out);
        out
    }

    fn compact_back(felts: Span<felt252>) -> super::alternatives::CompactState {
        let mut felts = felts;
        Serde::deserialize(ref felts).unwrap()
    }

    #[test]
    fn test_compact_round_trip_is_exact() {
        let mut world = pile(10);
        let level = world.to_state();
        let _ = world.step();
        let full = world.to_state();
        let felts = compact_felts(ref world);
        let mut restored = super::alternatives::from_compact(level, compact_back(felts.span()));
        assert_eq!(restored.to_state(), full);
        println!("pile10 compact: {} felts", felts.len());
        let mut world = pile(20);
        println!("pile20 compact: {} felts", compact_felts(ref world).len());
    }

    #[test]
    fn gas_pile10_compact_to() {
        let mut world = pile(opaque(10));
        let _ = super::alternatives::to_compact(ref world);
    }

    #[test]
    fn gas_pile10_compact_serialize() {
        let mut world = pile(opaque(10));
        let _ = compact_felts(ref world);
    }

    #[test]
    fn gas_pile10_compact_deserialize() {
        let mut world = pile(opaque(10));
        let _ = compact_back(compact_felts(ref world).span());
    }

    #[test]
    fn gas_pile10_compact_from() {
        let mut world = pile(opaque(10));
        let level = world.to_state();
        let compact = compact_back(compact_felts(ref world).span());
        let _ = super::alternatives::from_compact(level, compact);
    }

    #[test]
    fn gas_pile10_level() {
        let mut world = pile(opaque(10));
        let _ = world.to_state();
        let _ = compact_back(compact_felts(ref world).span());
    }

    #[test]
    fn gas_pile20_compact_to() {
        let mut world = pile(opaque(20));
        let _ = super::alternatives::to_compact(ref world);
    }

    #[test]
    fn gas_pile20_compact_serialize() {
        let mut world = pile(opaque(20));
        let _ = compact_felts(ref world);
    }

    #[test]
    fn gas_pile20_compact_deserialize() {
        let mut world = pile(opaque(20));
        let _ = compact_back(compact_felts(ref world).span());
    }

    #[test]
    fn gas_pile20_compact_from() {
        let mut world = pile(opaque(20));
        let level = world.to_state();
        let compact = compact_back(compact_felts(ref world).span());
        let _ = super::alternatives::from_compact(level, compact);
    }

    #[test]
    fn gas_pile20_level() {
        let mut world = pile(opaque(20));
        let _ = world.to_state();
        let _ = compact_back(compact_felts(ref world).span());
    }
}
