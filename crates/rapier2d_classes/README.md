# rapier2d_classes

The `rapier2d` step split across declared Starknet classes (work packages CS4–CS6, CX1, CX2), so that no
class of a game's step exceeds the size limit of a declared class (73,728 Sierra and CASM felts). The
analysis is in `docs/research/class-split.md`.

```toml
[dependencies]
rapier2d = "0.1.0-alpha.7"
rapier2d_classes = "0.1.0-alpha.7"
```

`StepConfig` (`rapier2d::pipeline::config`) chooses what a step supports (contact dispatcher,
sensor / composite / joint strategies); `StageConfig` (`rapier2d::pipeline::stages`, CS5 / CS6)
chooses where each stage runs and what the caller class compiles. This crate declares the classes
that run the stages and the strategies that library-call them:

| class | runs | called (`SlimSplitStages`) |
|---|---|---|
| `NarrowPhaseClass` | the narrow phase's pair loop (filters, one-way platforms, solver data, events) and the contact generation of the pairs without a ball | once per step with a pair |
| `ContactBallClass` | contact generation of the pairs with a ball | once per step with such a pair (per pair with `SplitStages`) |
| `ContactPolygonClass` | contact generation of cuboid, convex polygon and half-space pairs | not called (in `NarrowPhaseClass`; once per step with such a pair with `SplitBatchedStages`) |
| `SolveAdvanceClass` | constraints, island solve, free bodies, position update, sleep timers | once per step with a moving body |
| `IslandsClass` | union-find, sleep and wake-up rules | on the steps whose islands can change |
| `BroadPhaseClass` | candidate pairs | once per step |
| `MassClass` | mass properties from the colliders | once per body whose colliders or local mass changed |
| `ActiveSetClass` | the rebuild of the active set | once per step that fills it |
| `ForceEventsClass` | the contact-force events (optional: not in `SlimSplitStages`) | once per step with force events |
| `SolverClass` | the island solve alone (CS4's layout, `ContactSolveStepConfig`) | once per step with a touching manifold |

A game declares the classes, then compiles its contract's step with `SlimSplitStages<H>`, `H` an
impl of `ClassHashes` returning the declared hashes as constants, and takes and returns the world
with the basic `WorldState` codec (`rapier2d::world::basic_state`: the same felts as `WorldState`,
without the code of the other shapes and of the joints):

```cairo
use rapier2d::prelude::{BasicStepConfig, WorldTrait};
use rapier2d::world::basic_state::{BasicWorldState, from_basic_state, into_basic_state};
use rapier2d_classes::{ClassHashes, SlimSplitStages};
use starknet::ClassHash;

impl GameClasses of ClassHashes {
    fn contact_ball() -> ClassHash {
        const H: ClassHash = 0x..._felt252.try_into().unwrap();
        H
    }
    // contact_polygon(), solver(), solve_advance(), islands(), broad_phase(), mass(),
    // narrow_phase(), active_set(), force_events(): the same
}

fn step_state(state: BasicWorldState, steps: u32) -> BasicWorldState {
    let mut world = from_basic_state(state);
    let (_, events) = world
        .step_with_force_events_with_stages::<BasicStepConfig, SlimSplitStages<GameClasses>>();
    // ...
    into_basic_state(world)
}
```

The caller class compiled this way (`rapier_sink`'s `SlimSplitStep`) is 73,204 CASM felts; it
supports the worlds of `BasicStepConfig` (balls, cuboids, convex polygons, half-spaces; no sensor,
composite or impulse joint) without position-based kinematic bodies (rejected with
`'Step: kinematic disabled'`), and the basic codec rejects any other shape and any joint arena ever
used. On slingfall's pile10 reference shot it costs 30.77M Cairo steps (+37.2 % over the in-process
step), 4 transactions of ≤ 10M. `SplitStages` (every stage out, the pair loop in the caller: a
caller over the limit), `SplitBatchedStages`, `SplitHybridStages`, `ContactSolveStepConfig`
(CS4's layout) and `orchestrator::OrchestratorClass` (CS6's route (b)) are the measured
alternatives.

Results are bit-identical to `BasicStepConfig` (`tests/split.cairo`, `tests/slim.cairo`,
`tests/route_b.cairo`: every tick of slingfall's pile10 reference shot, and with user changes in
the middle of it); `tests/steps.cairo`, `tests/slim.cairo` and `tests/route_b.cairo` measure the
Cairo steps of each layout. The class sizes are tracked in `gas/bytecode.size`
(`scripts/bytecode_size.py`, which fails when a declared class exceeds 73,728 Sierra or CASM
felts).

The class hashes change whenever the code a class compiles changes (this crate, or the engine
code it reaches): a game re-declares the classes and updates its constants with each release.
