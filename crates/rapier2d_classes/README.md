# rapier2d_classes

The `rapier2d` step split across declared Starknet classes (work packages CS4–CS7, CX1, CX2), so that no
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
| `WorldEditClass` | the World edits between steps: insert a body with its collider and velocities, remove bodies, put bodies to sleep (`edits`, CS7) | by the game, once per tick that edits (`edit_world`) |

The classes a game declares:

* **`SlimSplitStages`** (the caller class under the limit): `NarrowPhaseClass`, `ContactBallClass`,
  `SolveAdvanceClass`, `IslandsClass`, `BroadPhaseClass`, `MassClass`, `ActiveSetClass`; with the
  force events out, `ForceEventsClass`; with the World edits out of its own class, `WorldEditClass`.
  Its `ClassHashes` impl must still return a hash for `contact_polygon()` and `solver()`, which it
  never calls (any value).
* **CS4's layout** (`ContactSolveStepConfig`, a caller over the limit): `ContactBallClass`,
  `ContactPolygonClass`, `SolverClass`.

Every class of this crate is one of them (CS7 moved route (b)'s `OrchestratorClass` to the unpublished
`rapier_sink` fixtures). Sizes (felts, release profile; identical in the dev profile), each gated at 73,728 with at
least 1,000 felts of margin by `scripts/bytecode_size.py check`:

| class | Sierra | CASM | CASM margin | declared for |
|---|--:|--:|--:|---|
| `NarrowPhaseClass` | 22,008 | 68,372 | 5,356 | `SlimSplitStages` |
| `ContactBallClass` | 14,950 | 41,560 | 32,168 | `SlimSplitStages`, CS4 |
| `SolveAdvanceClass` | 37,044 | 58,547 | 15,181 | `SlimSplitStages` |
| `IslandsClass` | 7,952 | 19,083 | 54,645 | `SlimSplitStages` |
| `BroadPhaseClass` | 4,525 | 9,841 | 63,887 | `SlimSplitStages` |
| `MassClass` | 13,297 | 35,686 | 38,042 | `SlimSplitStages` |
| `ActiveSetClass` | 4,381 | 10,930 | 62,798 | `SlimSplitStages` |
| `ForceEventsClass` | 2,832 | 6,202 | 67,526 | `SlimSplitStages` with the force events out |
| `WorldEditClass` | 17,610 | 51,865 | 21,863 | the World edits between steps (`edit_world`) |
| `ContactPolygonClass` | 14,515 | 57,299 | 16,429 | CS4's layout |
| `SolverClass` | 29,500 | 43,726 | 30,002 | CS4's layout |

The caller classes of the fixtures, for reference: `SlimSplitStep` 26,834 / 67,076 (margin 6,652), `SlimEditStep` (the
same with the edits forwarded to `WorldEditClass`) 27,135 / 68,818 (margin 4,910).

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

The caller class compiled this way (`rapier_sink`'s `SlimSplitStep`) is 67,076 CASM felts (CS7: the
basic codec reads the world through outlined readers, `rapier2d::world::basic_state::decode`); it
supports the worlds of `BasicStepConfig` (balls, cuboids, convex polygons, half-spaces; no sensor,
composite or impulse joint) without position-based kinematic bodies (rejected with
`'Step: kinematic disabled'`), and the basic codec rejects any other shape and any joint arena ever
used. On slingfall's pile10 reference shot it costs 30.71M Cairo steps (+36.9 % over the in-process
step), 4 transactions of ≤ 10M. `SplitStages` (every stage out, the pair loop in the caller: a
caller over the limit), `SplitBatchedStages`, `SplitHybridStages` and `ContactSolveStepConfig`
(CS4's layout) are the measured alternatives (stage configurations: they compile no class).

The World edits a game applies between steps go to `WorldEditClass` with `edit_world(class_hash,
world, edits)`: the world crosses in and out with the basic codec, the edits as the felts of a
`Span<WorldEdit>` forwarded as they are. Next to `SlimSplitStep`'s step the call costs the caller
1,742 CASM felts (the same edits in process: +29,581, over the limit) and ≈ 105k to 111k Cairo steps
per call on pile10 (a world of 1,839 felts).

Results are bit-identical to `BasicStepConfig` (`tests/split.cairo`, `tests/slim.cairo`,
`tests/edits.cairo`: every tick of slingfall's pile10 reference shot, and with user changes or the
World edits in the middle of it); `tests/steps.cairo` and `tests/slim.cairo` measure the Cairo steps
of each layout, `tests/edits.cairo` those of each edit call. The class sizes are tracked in `gas/bytecode.size`
(`scripts/bytecode_size.py`, which fails when a declared class comes within 1,000 felts of 73,728 Sierra or CASM
felts).

The class hashes change whenever the code a class compiles changes (this crate, or the engine
code it reaches): a game re-declares the classes and updates its constants with each release.
