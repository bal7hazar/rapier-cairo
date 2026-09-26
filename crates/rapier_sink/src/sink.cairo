//! The fixture contracts. Each class is measured alone: a Starknet class holds every function
//! its entry points reach, so the difference between two classes is the cost of what one reaches
//! and the other does not.
//!
//! * `WorldOnly`: builds the game's world, no step (the floor under every step).
//! * `GameStep`: the game's configuration stepped with `step_with_force_events` (ball, cuboid,
//!   convex polygon, half-space; no joint, no sensor): the class CS2 must bring under the limits.
//! * `Sink`: the game's configuration and the full one (every shape, a sensor, every joint),
//!   through `step_with_force_events` and `step` (both monomorphisations of the step).
//! * `StateStep`: one chunk of a multi-class layout: a `WorldState` in calldata, `steps` steps,
//!   the `WorldState` out (the world crossing a class boundary).
//! * `StateRoundTrip`: the crossing alone (`WorldState` in, `from_state`, `into_state`, out): what
//!   every class of a multi-class layout pays before any physics.

#[starknet::contract]
pub mod WorldOnly {
    use rapier2d::prelude::{Handle, RigidBodyTrait, WorldTrait};
    use crate::scene;

    #[storage]
    struct Storage {}

    /// The `x` translation (raw Q32.32) of the game world's cuboid after the world is built.
    #[external(v0)]
    fn build(self: @ContractState, dx: i64, dy: i64) -> i64 {
        let mut world = scene::game_world(dx, dy);
        world.body(Handle { index: 0, generation: 0 }).unwrap().position().translation.x.raw
    }
}

#[starknet::contract]
pub mod GameStep {
    use crate::scene;

    #[storage]
    struct Storage {}

    /// `(collision events, contact-force events)` of `steps` steps of the game world.
    #[external(v0)]
    fn simulate(self: @ContractState, dx: i64, dy: i64, steps: u32) -> (u32, u32) {
        let mut world = scene::game_world(dx, dy);
        scene::run_with_forces(ref world, steps)
    }
}

#[starknet::contract]
pub mod Sink {
    use crate::scene;

    #[storage]
    struct Storage {}

    /// `(collision events, contact-force events)` of `steps` steps of the game world.
    #[external(v0)]
    fn simulate_game(self: @ContractState, dx: i64, dy: i64, steps: u32) -> (u32, u32) {
        let mut world = scene::game_world(dx, dy);
        scene::run_with_forces(ref world, steps)
    }

    /// `(collision events, contact-force events)` of `steps` steps of the full world.
    #[external(v0)]
    fn simulate_full(self: @ContractState, dx: i64, dy: i64, steps: u32) -> (u32, u32) {
        let mut world = scene::full_world(dx, dy);
        scene::run_with_forces(ref world, steps)
    }

    /// Collision events of `steps` plain `World::step` calls of the full world.
    #[external(v0)]
    fn simulate_full_plain(self: @ContractState, dx: i64, dy: i64, steps: u32) -> u32 {
        let mut world = scene::full_world(dx, dy);
        scene::run(ref world, steps)
    }
}

#[starknet::contract]
pub mod StateStep {
    use rapier2d::prelude::{WorldState, WorldTrait};
    use crate::scene;

    #[storage]
    struct Storage {}

    /// `state` after `steps` calls of `step_with_force_events` (the events are dropped).
    #[external(v0)]
    fn step_state(self: @ContractState, state: WorldState, steps: u32) -> WorldState {
        let mut world = WorldTrait::from_state(state);
        let _ = scene::run_with_forces(ref world, steps);
        world.into_state()
    }
}

#[starknet::contract]
pub mod StateRoundTrip {
    use rapier2d::prelude::{WorldState, WorldTrait};

    #[storage]
    struct Storage {}

    /// `state` restored into a `World` and saved again.
    #[external(v0)]
    fn round_trip(self: @ContractState, state: WorldState) -> WorldState {
        WorldTrait::from_state(state).into_state()
    }
}
