//! CS7: the slim caller with the World edits a game applies between its steps (size fixtures;
//! `rapier2d_classes::edits`). Both take the edits of a chunk and apply them before its steps:
//!
//! * `SlimEditStep`: `SlimSplitStep` with the edits in `WorldEditClass` (`edit_world`: the world
//!   crosses with the basic codec the caller already compiles);
//! * `SlimInCallerEditStep`: the same edits in process (`apply_edits`), the measured loser.

#[starknet::contract]
pub mod SlimEditStep {
    use rapier2d::prelude::BasicStepConfig;
    use rapier2d::world::WorldTrait;
    use rapier2d::world::basic_state::{BasicWorldState, from_basic_state, into_basic_state};
    use rapier2d_classes::{SlimSplitStages, edit_world};
    use starknet::ClassHash;
    use crate::classes::FixtureHashes;

    #[storage]
    struct Storage {}

    /// `SlimSplitStep::step_state` after `edits` (the felts of a `Span<WorldEdit>`, forwarded) in
    /// `WorldEditClass` at `edit_class` (no call when `edits` is empty).
    #[external(v0)]
    fn step_state(
        self: @ContractState,
        state: BasicWorldState,
        steps: u32,
        edits: Span<felt252>,
        edit_class: ClassHash,
    ) -> BasicWorldState {
        let mut world = from_basic_state(state);
        if !edits.is_empty() {
            let (edited, _) = edit_world(edit_class, world, edits);
            world = edited;
        }
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
pub mod SlimInCallerEditStep {
    use rapier2d::prelude::BasicStepConfig;
    use rapier2d::world::WorldTrait;
    use rapier2d::world::basic_state::{BasicWorldState, from_basic_state, into_basic_state};
    use rapier2d_classes::{SlimSplitStages, WorldEdit, apply_edits};
    use crate::classes::FixtureHashes;

    #[storage]
    struct Storage {}

    /// `SlimEditStep::step_state` with the edits in process.
    #[external(v0)]
    fn step_state(
        self: @ContractState, state: BasicWorldState, steps: u32, edits: Span<WorldEdit>,
    ) -> BasicWorldState {
        let mut world = from_basic_state(state);
        let _ = apply_edits(ref world, edits);
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
