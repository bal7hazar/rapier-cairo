//! The caller classes of the split step (work package CS4): `BasicGameStep` with the contact
//! generation and the island solve library-called in the declared classes of `rapier2d_classes`
//! (`ContactBallClass`, `ContactPolygonClass`, `SolverClass`, measured there). Size fixtures: the
//! class hashes are constants of [`FixtureHashes`] (a game supplies its declared ones).
//!
//! * `Split4Step`: `rapier2d_classes::ContactSolveStepConfig`, CS4's layout (4 classes: the
//!   caller, 2 contact families, the solve).
//! * `StagesSplitStep` (CS5): `rapier2d_classes::SplitStepConfig`, every stage out (the caller,
//!   2 contact families called once per pair, the broad phase, the islands, the solve and position
//!   update, the mass properties: 7 classes);
//! * `StagesBatchedStep`: `SplitBatchedStepConfig` (the contact generation batched per family,
//!   one call of each family class per step);
//! * `StagesHybridStep`: `SplitHybridStepConfig` (the free bodies of a step without a touching
//!   pair moved in the caller).
//! * `Split3Step`: the contact generation in `crate::split`'s `ContactClass` (one class, hash read
//!   from storage) and the solve in `SolverClass` (3 classes).

use rapier2d::pipeline::config::{
    InProcessBroadPhase, InProcessIslands, InProcessMass, InProcessSolveAdvance, NoComposites,
    NoSensors, PairLoopNarrowPhase, StepConfig,
};
use rapier2d_classes::{ClassHashes, LibraryCallSolver};
use starknet::ClassHash;
use crate::split::LibraryCallDispatcher;

/// Arbitrary constant class hashes (the classes' sizes do not depend on the values).
pub impl FixtureHashes of ClassHashes {
    fn contact_ball() -> ClassHash {
        const H: ClassHash = 0x1ba1_felt252.try_into().unwrap();
        H
    }

    fn contact_polygon() -> ClassHash {
        const H: ClassHash = 0x2b01_felt252.try_into().unwrap();
        H
    }

    fn solver() -> ClassHash {
        const H: ClassHash = 0x3501_felt252.try_into().unwrap();
        H
    }

    fn solve_advance() -> ClassHash {
        const H: ClassHash = 0x4a0_felt252.try_into().unwrap();
        H
    }

    fn islands() -> ClassHash {
        const H: ClassHash = 0x5a1_felt252.try_into().unwrap();
        H
    }

    fn broad_phase() -> ClassHash {
        const H: ClassHash = 0x6b0_felt252.try_into().unwrap();
        H
    }

    fn mass() -> ClassHash {
        const H: ClassHash = 0x7a55_felt252.try_into().unwrap();
        H
    }
}

/// `BasicStepConfig` with the contact generation in `ContactClass` and the solve in
/// `SolverClass`.
pub impl Split3StepConfig of StepConfig {
    impl Dispatcher = LibraryCallDispatcher;
    impl Sensors = NoSensors;
    impl Composites = NoComposites;
    impl Joints = LibraryCallSolver<FixtureHashes>;
    impl Narrow = PairLoopNarrowPhase<LibraryCallDispatcher, NoSensors, NoComposites>;
    impl Broad = InProcessBroadPhase;
    impl Islands = InProcessIslands;
    impl Advance = InProcessSolveAdvance<LibraryCallSolver<FixtureHashes>>;
    impl Mass = InProcessMass;
}

#[starknet::contract]
pub mod Split4Step {
    use rapier2d::prelude::{WorldState, WorldTrait};
    use rapier2d_classes::ContactSolveStepConfig;
    use super::FixtureHashes;

    #[storage]
    struct Storage {}

    /// `BasicGameStep::step_state` with `ContactSolveStepConfig<FixtureHashes>`.
    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world.step_with_force_events_with::<ContactSolveStepConfig<FixtureHashes>>();
            i += 1;
        }
        world.into_state()
    }
}

#[starknet::contract]
pub mod Split3Step {
    use rapier2d::prelude::{WorldState, WorldTrait};
    use starknet::ClassHash;
    use starknet::storage::StoragePointerWriteAccess;
    use super::Split3StepConfig;

    #[storage]
    struct Storage {
        /// Read raw at `crate::split::CONTACT_CLASS_SLOT` by the contact dispatcher.
        contact_class: ClassHash,
    }

    #[constructor]
    fn constructor(ref self: ContractState, contact_class: ClassHash) {
        self.contact_class.write(contact_class);
    }

    /// `BasicGameStep::step_state` with [`Split3StepConfig`].
    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world.step_with_force_events_with::<Split3StepConfig>();
            i += 1;
        }
        world.into_state()
    }
}

#[starknet::contract]
pub mod StagesSplitStep {
    use rapier2d::prelude::{WorldState, WorldTrait};
    use rapier2d_classes::SplitStepConfig;
    use super::FixtureHashes;

    #[storage]
    struct Storage {}

    /// `BasicGameStep::step_state` with `SplitStepConfig<FixtureHashes>`.
    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world.step_with_force_events_with::<SplitStepConfig<FixtureHashes>>();
            i += 1;
        }
        world.into_state()
    }
}

#[starknet::contract]
pub mod StagesBatchedStep {
    use rapier2d::prelude::{WorldState, WorldTrait};
    use rapier2d_classes::SplitBatchedStepConfig;
    use super::FixtureHashes;

    #[storage]
    struct Storage {}

    /// `BasicGameStep::step_state` with `SplitBatchedStepConfig<FixtureHashes>`.
    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world.step_with_force_events_with::<SplitBatchedStepConfig<FixtureHashes>>();
            i += 1;
        }
        world.into_state()
    }
}

#[starknet::contract]
pub mod StagesHybridStep {
    use rapier2d::prelude::{WorldState, WorldTrait};
    use rapier2d_classes::SplitHybridStepConfig;
    use super::FixtureHashes;

    #[storage]
    struct Storage {}

    /// `BasicGameStep::step_state` with `SplitHybridStepConfig<FixtureHashes>`.
    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world.step_with_force_events_with::<SplitHybridStepConfig<FixtureHashes>>();
            i += 1;
        }
        world.into_state()
    }
}
