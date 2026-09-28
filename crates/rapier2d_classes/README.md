# rapier2d_classes

The `rapier2d` step split across declared Starknet classes (work package CS4), so that a game's
step is not one class over the size limit of a declared class. The analysis is in
`docs/research/class-split.md`.

`StepConfig` (`rapier2d::pipeline::config`) already hands the contact generation and the island
solve to strategies. This crate declares the classes that run them and the strategies that
library-call them:

| class | runs | called |
|---|---|---|
| `ContactBallClass` | contact generation of the pairs with a ball | once per ball pair whose AABBs overlap |
| `ContactPolygonClass` | contact generation of cuboid, convex polygon and half-space pairs | once per such pair |
| `SolverClass` | the island solve (constraints, sweeps, integration) | once per step with a touching manifold |

A game declares the three classes, then compiles its contract's step with
`SplitStepConfig<H>`, `H` an impl of `ClassHashes` returning the declared hashes as constants:

```cairo
use rapier2d_classes::{ClassHashes, SplitStepConfig};
use starknet::ClassHash;

impl GameClasses of ClassHashes {
    fn contact_ball() -> ClassHash {
        const H: ClassHash = 0x..._felt252.try_into().unwrap();
        H
    }
    // contact_polygon(), solver(): the same, with their hashes
}

let (_, events) = world.step_with_force_events_with::<SplitStepConfig<GameClasses>>();
```

Results are bit-identical to `BasicStepConfig` (`tests/split.cairo`, every tick of slingfall's
pile10 reference shot). The class sizes are tracked in `gas/bytecode.size`
(`scripts/bytecode_size.py`, which fails when a class exceeds 73,728 Sierra or CASM felts).

The class hashes change whenever the code a class compiles changes (this crate, or the engine
code it reaches): a game re-declares the classes and updates its constants with each release.
