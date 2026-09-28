//! The caller classes of the split step (work package CS4): `BasicGameStep` with the contact
//! generation and the island solve library-called in the declared classes of `rapier2d_classes`
//! (`ContactBallClass`, `ContactPolygonClass`, `SolverClass`, measured there). Size fixtures: the
//! class hashes are constants of [`FixtureHashes`] (a game supplies its declared ones).
//!
//! * `Split4Step`: `rapier2d_classes::ContactSolveStepConfig`, CS4's layout (4 classes: the
//!   caller, 2 contact families, the solve).
//! * `StagesSplitStep` (CS5): `rapier2d_classes::SplitStages`, every stage out (the caller,
//!   2 contact families called once per pair, the broad phase, the islands, the solve and position
//!   update, the mass properties: 7 classes);
//! * `StagesBatchedStep`: `SplitBatchedStages` (the contact generation batched per family,
//!   one call of each family class per step);
//! * `StagesHybridStep`: `SplitHybridStages` (the free bodies of a step without a touching
//!   pair moved in the caller).
//! * `Split3Step`: the contact generation in `crate::split`'s `ContactClass` (one class, hash read
//!   from storage) and the solve in `SolverClass` (3 classes).

use rapier2d::pipeline::config::{NoComposites, NoSensors, PairLoopNarrowPhase, StepConfig};
use rapier2d::pipeline::stages::{
    BasicShapeKernels, InProcessActiveSet, InProcessForceEvents, NoFreePath, StageConfig,
};
use rapier2d_classes::{
    ClassHashes, FamilyDispatcher, LibraryCallBroadPhase, LibraryCallIslands, LibraryCallMass,
    LibraryCallSolveAdvance, LibraryCallSolver,
};
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

    fn narrow_phase() -> ClassHash {
        const H: ClassHash = 0x8a77_felt252.try_into().unwrap();
        H
    }

    fn active_set() -> ClassHash {
        const H: ClassHash = 0x9ac7_felt252.try_into().unwrap();
        H
    }

    fn force_events() -> ClassHash {
        const H: ClassHash = 0xf0ce_felt252.try_into().unwrap();
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
}

/// CS6's levers 1 and 2 alone: `SlimSplitStages` with the pair loop in the caller (the contact
/// generation of each pair in the family classes, as `SplitStages`).
pub impl Levers12Stages of StageConfig {
    impl Narrow = PairLoopNarrowPhase<FamilyDispatcher<FixtureHashes>, NoSensors, NoComposites>;
    impl Broad = LibraryCallBroadPhase<FixtureHashes>;
    impl Islands = LibraryCallIslands<FixtureHashes>;
    impl Advance = LibraryCallSolveAdvance<FixtureHashes>;
    impl Mass = LibraryCallMass<FixtureHashes>;
    impl Shapes = BasicShapeKernels;
    impl Forces = InProcessForceEvents;
    impl Active = InProcessActiveSet<BasicShapeKernels>;
    impl Free = NoFreePath;
    const KINEMATIC: bool = false;
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
    use rapier2d::prelude::{BasicStepConfig, WorldState, WorldTrait};
    use rapier2d_classes::SplitStages;
    use super::FixtureHashes;

    #[storage]
    struct Storage {}

    /// `BasicGameStep::step_state` with `SplitStages<FixtureHashes>`.
    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world
                .step_with_force_events_with_stages::<
                    BasicStepConfig, SplitStages<FixtureHashes>,
                >();
            i += 1;
        }
        world.into_state()
    }
}

#[starknet::contract]
pub mod StagesBatchedStep {
    use rapier2d::prelude::{BasicStepConfig, WorldState, WorldTrait};
    use rapier2d_classes::SplitBatchedStages;
    use super::FixtureHashes;

    #[storage]
    struct Storage {}

    /// `BasicGameStep::step_state` with `SplitBatchedStages<FixtureHashes>`.
    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world
                .step_with_force_events_with_stages::<
                    BasicStepConfig, SplitBatchedStages<FixtureHashes>,
                >();
            i += 1;
        }
        world.into_state()
    }
}

#[starknet::contract]
pub mod StagesHybridStep {
    use rapier2d::prelude::{BasicStepConfig, WorldState, WorldTrait};
    use rapier2d_classes::SplitHybridStages;
    use super::FixtureHashes;

    #[storage]
    struct Storage {}

    /// `BasicGameStep::step_state` with `SplitHybridStages<FixtureHashes>`.
    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world
                .step_with_force_events_with_stages::<
                    BasicStepConfig, SplitHybridStages<FixtureHashes>,
                >();
            i += 1;
        }
        world.into_state()
    }
}

#[starknet::contract]
pub mod SlimSplitStep {
    use rapier2d::prelude::BasicStepConfig;
    use rapier2d::world::WorldTrait;
    use rapier2d::world::basic_state::{BasicWorldState, from_basic_state, into_basic_state};
    use rapier2d_classes::SlimSplitStages;
    use super::FixtureHashes;

    #[storage]
    struct Storage {}

    /// `StagesSplitStep::step_state` with `SlimSplitStages<FixtureHashes>` and the basic codec
    /// (CS6: the same calldata).
    #[external(v0)]
    fn step_state(self: @ContractState, state: BasicWorldState, steps: u32) -> BasicWorldState {
        let mut world = from_basic_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world
                .step_with_force_events_with_stages::<
                    BasicStepConfig, SlimSplitStages<FixtureHashes>,
                >();
            i += 1;
        }
        into_basic_state(world)
    }
}

#[starknet::contract]
pub mod Levers12Step {
    use rapier2d::prelude::BasicStepConfig;
    use rapier2d::world::WorldTrait;
    use rapier2d::world::basic_state::{BasicWorldState, from_basic_state, into_basic_state};
    use super::Levers12Stages;

    #[storage]
    struct Storage {}

    /// `SlimSplitStep::step_state` with [`Levers12Stages`] (CS6's levers 1 and 2 alone).
    #[external(v0)]
    fn step_state(self: @ContractState, state: BasicWorldState, steps: u32) -> BasicWorldState {
        let mut world = from_basic_state(state);
        let mut i = 0;
        while i != steps {
            let _ = world.step_with_force_events_with_stages::<BasicStepConfig, Levers12Stages>();
            i += 1;
        }
        into_basic_state(world)
    }
}

/// Route (b) of CS6 (measured, not shipped): the caller of `rapier2d_classes`'
/// `OrchestratorClass`, the world decoded here between the steps (where a game's rules run) and
/// crossing to the orchestrator and back at every step.
#[starknet::contract]
pub mod OrchestratedStep {
    use rapier2d::world::basic_state::{BasicWorldState, from_basic_state, into_basic_state};
    use rapier2d_classes::orchestrator::orchestrated_step;
    use starknet::ClassHash;

    #[storage]
    struct Storage {}

    /// `SlimSplitStep::step_state` with each step in the orchestrator at `orchestrator`.
    #[external(v0)]
    fn step_state(
        self: @ContractState, state: BasicWorldState, steps: u32, orchestrator: ClassHash,
    ) -> BasicWorldState {
        let mut world = from_basic_state(state);
        let mut i = 0;
        while i != steps {
            let (next, _) = orchestrated_step(orchestrator, into_basic_state(world));
            world = from_basic_state(next);
            i += 1;
        }
        into_basic_state(world)
    }
}
