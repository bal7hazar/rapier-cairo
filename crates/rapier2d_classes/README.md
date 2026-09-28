# rapier2d_classes

The `rapier2d` step split across declared Starknet classes (work package CS4), so that a game's
step is not one class over the size limit of a declared class. The analysis is in
`docs/research/class-split.md`.

`StepConfig` (`rapier2d::pipeline::config`) hands the contact generation, the island solve and,
since CS5, every other stage of the step (the narrow phase's pair loop, the broad phase, the
islands, the fused solve and position update, the mass properties) to strategies. This crate
declares the classes that run them and the strategies that library-call them:

| class | runs | called (`SplitStepConfig`) |
|---|---|---|
| `ContactBallClass` | contact generation of the pairs with a ball | once per ball pair whose AABBs overlap (`contact_batch`: once per step with all of them) |
| `ContactPolygonClass` | contact generation of cuboid, convex polygon and half-space pairs | once per such pair (`contact_batch`: once per step) |
| `SolveAdvanceClass` | constraints, island solve, free bodies, position update, sleep timers | once per step with a moving body |
| `IslandsClass` | union-find, sleep and wake-up rules | on the steps whose islands can change |
| `BroadPhaseClass` | candidate pairs | once per step |
| `MassClass` | mass properties from the colliders | once per body whose colliders or local mass changed |
| `SolverClass` | the island solve alone (CS4's layout, `ContactSolveStepConfig`) | once per step with a touching manifold |

A game declares the classes, then compiles its contract's step with `SplitStepConfig<H>`, `H` an
impl of `ClassHashes` returning the declared hashes as constants:

```cairo
use rapier2d_classes::{ClassHashes, SplitStepConfig};
use starknet::ClassHash;

impl GameClasses of ClassHashes {
    fn contact_ball() -> ClassHash {
        const H: ClassHash = 0x..._felt252.try_into().unwrap();
        H
    }
    // contact_polygon(), solver(), solve_advance(), islands(), broad_phase(), mass(): the same
}

let (_, events) = world.step_with_force_events_with::<SplitStepConfig<GameClasses>>();
```

`SplitBatchedStepConfig` (contact generation batched per family), `SplitHybridStepConfig` (no
solve call on a step without a touching pair) and `ContactSolveStepConfig` (CS4's layout) are the
measured alternatives.

Results are bit-identical to `BasicStepConfig` (`tests/split.cairo`, every tick of slingfall's
pile10 reference shot, and with user changes in the middle of it); `tests/steps.cairo` measures the
Cairo steps of each layout. The class sizes are tracked in `gas/bytecode.size`
(`scripts/bytecode_size.py`, which fails when a class exceeds 73,728 Sierra or CASM felts).

The class hashes change whenever the code a class compiles changes (this crate, or the engine
code it reaches): a game re-declares the classes and updates its constants with each release.
