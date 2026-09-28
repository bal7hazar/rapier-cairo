# CS4 — the contact-family and solver classes, library-called through `StepConfig`

## 1. Read first
`AGENTS.md`; `docs/research/class-split.md` (CS3: §3 the classes and their crossings, §4 costs, §6 the prototype, the
recommendation); `docs/adr/0001-upstream-divergences.md` entry 37; the prototype branch `origin/proto/cs3-phase-dispatch`
(`git diff origin/main...origin/proto/cs3-phase-dispatch`: `rapier_dynamics2d/src/solver/{body_store,island}.cairo`
`Serde` / `pub` on the solver's I/O, `crates/rapier_sink/src/split_solve.cairo` — `SolverClass`, the 3- / 4-class
callers, the compact solver crossing — and `tests/split_solve.cairo`); on `main`: `crates/rapier_sink/src/{split,family}.cairo`
(`LibraryCallDispatcher`, the family classes, the storage-slot class hashes), `tests/{pile10,split}.cairo`,
`crates/rapier2d/src/pipeline/config.cairo` (`StepConfig`, `BasicStepConfig`), `scripts/bytecode_size.py`.

**Programme go (2026-09-28)** for CS4 with these conditions: results bit-identical on every step probe and on the
151-tick pile10 shot; `program.basic` and the Cairo steps of the `game_path` probes unchanged for in-process users; the
`Serde` / `pub` additions on the solver I/O documented as a Cairo-only extension (propose the ADR text under
Escalations; the orchestrator adds it); the declared classes' sizes in `gas/bytecode.size` with CI failing when one
exceeds 73,728 Sierra or CASM felts.

## 2. Scope (file allowlist)
- `crates/rapier_dynamics2d/src/solver/**`: only `Serde` derives / `pub` on the solver's input and output types and the
  compact solver view CS3 used (no logic change, no field change).
- **New crate `crates/rapier2d_classes/`** (orchestrator exception: you create its `Scarb.toml` and `src/lib.cairo`;
  publishable metadata as `crates/rapier2d/Scarb.toml`, `starknet = "2.19.4"`, `[[target.starknet-contract]]` with
  `sierra` and `casm`): the declared classes `ContactBallClass`, `ContactPolygonClass`, `SolverClass`; the
  library-calling strategies (a `ContactDispatcher` that routes each pair to its family class, the solve strategy that
  library-calls `SolverClass`); a `ClassHashes` provider trait with **constant** class hashes (the game supplies an impl
  of consts after declaring the classes; tests use the hashes snforge declares) replacing CS3's storage-slot reads; a
  `StepConfig` impl generic over it (e.g. `SplitStepConfig<H>`) for contracts. Its tests: bit-identity of the split step
  vs `BasicStepConfig` over the pile10 151-tick shot (reuse or move `rapier_sink/tests/pile10.cairo`'s world), the
  per-call steps.
- `crates/rapier_sink/**`: re-point the fixtures at `rapier2d_classes` (the callers `Split3Step` / `Split4Step` become
  fixtures of it; drop what the new crate supersedes), `scripts/bytecode_size.py` (build the new crate's classes,
  a `DECLARED` list checked against 73,728 in both Sierra and CASM — `check` fails above it), `gas/bytecode.size`.
- Forbidden: the step's logic, `crates/rapier2d/src/**` (the slots exist; a missing one goes under Escalations), any other crate's
  `Scarb.toml` / `lib.cairo`, `.github/**` (CI already runs `snforge test -p rapier2d_classes` in job `sink` once the
  crate exists).

## 3. Expected result
- `ContactBallClass`, `ContactPolygonClass`, `SolverClass` each ≤ 73,728 Sierra and CASM felts (CS3: 39,561 / 54,634 /
  45,552 CASM), in `gas/bytecode.size`, guarded by `bytecode_size.py check`.
- A contract stepping with `SplitStepConfig<H>` is bit-identical to `BasicStepConfig` on the pile10 shot (every tick:
  bodies, pairs, events), with its steps reported (CS3: +24.2 % with storage hashes; expect ≈ −0.31M from constant
  hashes).
- In-process users unchanged: `steps_game_basic_*` and every step probe identical in exact Cairo steps; `program.basic`
  and every existing line of `gas/bytecode.size` identical.
- The caller fixture's size reported (CS3: 162,902 CASM; not a target of this lot).

## 4. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`): `scarb
fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier_dynamics2d, rapier2d_classes,
rapier_sink (one at a time); snapshots with `--from-log`; `python3 scripts/bytecode_size.py snapshot` then `check`;
`python3 scripts/api_parity.py --check` (regenerate if needed). Never a workspace-wide run. Rebase on `origin/main`
before the PR. Commit wip states early. Conventional commits + trailer; push; `gh pr create --base main --title "<what
ships>" --body-file …`; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API · Class sizes ·
Pile10 bit-identity and steps (before / after, per call) · In-process unchanged proof · Proposed ADR text · Requested
changes to release / CI · Escalations · PR URL). Memory rules apply.

## 5. Work autonomously, do not ask questions, do not widen the scope.
