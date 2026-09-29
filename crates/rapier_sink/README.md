# rapier_sink

Starknet contract fixtures that link the `rapier2d` step into deployable classes, so that the
compiled class size of a game-like consumer is tracked against the network limits (work package
CS1). **Not published** (`scripts/release.sh` lists the published crates).

`python3 scripts/bytecode_size.py [table|snapshot|check|attribution]` builds this package in the
release profile and measures every class; `gas/bytecode.size` is the committed snapshot (checked by
CI's `bytecode` job). The analysis is in `docs/research/class-size.md`.

`programs/lib.cairo` holds the `#[executable]` fixtures (CS2): the game world stepped under each
step configuration (`rapier2d::pipeline::config`). It is not part of this package:
`scripts/bytecode_size.py` builds it as a temporary executable package and records each
program's felts (`program.*` in `gas/bytecode.size`), what a proof of the game hashes.

`src/split.cairo` holds the multi-class fixtures of CS3 (the game's step split across declared
classes called through `library_call_syscall`): `BasicGameStep` (the game's step, `WorldState` in
and out), the contact-generation class `ContactClass`, its caller `SplitNarrowStep` and `Echo` (the
cost of a bare library call). The supported declared classes (`ContactBallClass`,
`ContactPolygonClass`, `SolverClass`) live in `crates/rapier2d_classes` (CS4); `src/classes.cairo`
holds their callers, size fixtures: `Split4Step` (`rapier2d_classes::ContactSolveStepConfig`),
`Split3Step` (`ContactClass` and `SolverClass`), CS5's `Stages*Step` (every stage out), and CS6's
`SlimSplitStep` (`SlimSplitStages` with the basic `WorldState` codec: the caller class under the
declared-class limit, in `DECLARED`), `Levers12Step` (its levers 1 and 2 alone) and
`OrchestratedStep` (route (b)'s caller). `src/orchestrator.cairo` holds route (b)'s
`OrchestratorClass` (moved from `rapier2d_classes` by CS7: no game declares it); `src/edits.cairo`
CS7's `SlimEditStep` (the slim caller with the World edits in `rapier2d_classes`' `WorldEditClass`)
and `SlimInCallerEditStep` (the same edits in process, over the limit). `tests/` reproduces slingfall's pile10 level and reference shot and
measures each layout in exact Cairo steps (`snforge test -p rapier_sink --tracked-resource
cairo-steps --detailed-resources`); the analysis is in `docs/research/class-split.md`.
