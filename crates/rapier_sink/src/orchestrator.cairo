//! Route (b) of CS6, measured and not shipped (`docs/research/class-split.md`, CS6; moved here
//! from `rapier2d_classes` by CS7: no game declares it, so the published crate does not carry it.
//! Its whole-shot tests stayed behind at `9c7372b`, `rapier2d_classes/tests/route_b.cairo`: they
//! declare the stage classes, which only that package's tests can): a second
//! orchestration class. The caller keeps the world between steps (the game's rules run there) and
//! hands it to `OrchestratorClass` once per step, which runs the step (the stages of
//! `SlimSplitStages`, each in its class) and returns the world and the force events. The world
//! crosses twice per step with the basic codec (`rapier2d::world::basic_state`).
//!
//! A declared class cannot be compiled with the game's constant class hashes of the stage classes
//! it calls: it reads them from the calling contract's storage ([`StoredClassHashes`], at the
//! slots of [`slots`], ≈ 200 Cairo steps per read).

use rapier2d::prelude::ContactForceEvent;
use rapier2d::world::basic_state::BasicWorldState;
use rapier2d_classes::hashes::{ClassHashes, errors};
use starknet::syscalls::{library_call_syscall, storage_read_syscall};
use starknet::{ClassHash, SyscallResultTrait};

/// The storage slots of the calling contract where [`StoredClassHashes`] reads each class hash.
pub mod slots {
    pub const CONTACT_BALL: felt252 = selector!("contact_ball_class");
    pub const CONTACT_POLYGON: felt252 = selector!("contact_polygon_class");
    pub const SOLVER: felt252 = selector!("solver_class");
    pub const SOLVE_ADVANCE: felt252 = selector!("solve_advance_class");
    pub const ISLANDS: felt252 = selector!("islands_class");
    pub const BROAD_PHASE: felt252 = selector!("broad_phase_class");
    pub const MASS: felt252 = selector!("mass_class");
    pub const NARROW_PHASE: felt252 = selector!("narrow_phase_class");
    pub const ACTIVE_SET: felt252 = selector!("active_set_class");
    pub const FORCE_EVENTS: felt252 = selector!("force_events_class");
}

fn stored(slot: felt252) -> ClassHash {
    let value = storage_read_syscall(0, slot.try_into().unwrap()).unwrap_syscall();
    value.try_into().unwrap()
}

/// Each class hash read from a storage slot of the calling contract ([`slots`]).
pub impl StoredClassHashes of ClassHashes {
    fn contact_ball() -> ClassHash {
        stored(slots::CONTACT_BALL)
    }

    fn contact_polygon() -> ClassHash {
        stored(slots::CONTACT_POLYGON)
    }

    fn solver() -> ClassHash {
        stored(slots::SOLVER)
    }

    fn solve_advance() -> ClassHash {
        stored(slots::SOLVE_ADVANCE)
    }

    fn islands() -> ClassHash {
        stored(slots::ISLANDS)
    }

    fn broad_phase() -> ClassHash {
        stored(slots::BROAD_PHASE)
    }

    fn mass() -> ClassHash {
        stored(slots::MASS)
    }

    fn narrow_phase() -> ClassHash {
        stored(slots::NARROW_PHASE)
    }

    fn active_set() -> ClassHash {
        stored(slots::ACTIVE_SET)
    }

    fn force_events() -> ClassHash {
        stored(slots::FORCE_EVENTS)
    }
}

/// One step of `state` in `OrchestratorClass` at `class_hash`: the
/// world after the step and the force events (the caller's side of route (b)).
///
/// # Panics
/// `errors::DECODE` when the class returns something else than its result.
pub fn orchestrated_step(
    class_hash: ClassHash, state: BasicWorldState,
) -> (BasicWorldState, Array<ContactForceEvent>) {
    let mut calldata = array![];
    state.serialize(ref calldata);
    let mut ret = library_call_syscall(class_hash, selector!("step"), calldata.span())
        .unwrap_syscall();
    Serde::deserialize(ref ret).expect(errors::DECODE)
}

/// The step of a basic world (route (b)): the world crosses in and out; the stages of
/// `rapier2d_classes::SlimSplitStages` (route (a)'s levers: without the pair loop and the rebuild
/// of the active set in their classes, the orchestrator is 84,291 CASM felts, over the limit).
#[starknet::contract]
pub mod OrchestratorClass {
    use rapier2d::prelude::{BasicStepConfig, ContactForceEvent, WorldTrait};
    use rapier2d::world::basic_state::{BasicWorldState, from_basic_state, into_basic_state};
    use rapier2d_classes::SlimSplitStages;
    use super::StoredClassHashes;

    #[storage]
    struct Storage {}

    /// `step_with_force_events_with_stages::<BasicStepConfig, SlimSplitStages>` on `state`.
    #[external(v0)]
    fn step(
        self: @ContractState, state: BasicWorldState,
    ) -> (BasicWorldState, Array<ContactForceEvent>) {
        let mut world = from_basic_state(state);
        let (_, forces) = world
            .step_with_force_events_with_stages::<
                BasicStepConfig, SlimSplitStages<StoredClassHashes>,
            >();
        (into_basic_state(world), forces)
    }

    /// `felts` back (the crossing's transfer alone: the floor of any layout of the same felts).
    #[external(v0)]
    fn echo(self: @ContractState, felts: Span<felt252>) -> Span<felt252> {
        felts
    }
}
