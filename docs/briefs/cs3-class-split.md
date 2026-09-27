# CS3 — split the game's step across declared classes (measurement and design, SNIP-36 path)

## 1. Read first
`AGENTS.md` (§6 crate-scoped gate, §7 incl. the game-shaped probes); `docs/research/class-size.md` (CS1: decomposition
per Sierra function, lever estimates, §5 multi-class layout — ≈ 27k CASM of fixed cost per class for a `WorldState`
round trip, 76,942 steps to cross the level-10 world once, ≈ 6.8k steps per `library_call`, a 4–5-class chain at
+310–385k steps per tick; §7 CS2 outcome); `docs/BUDGETS.md` ("The game on 0.1.0-alpha.6": `SlingfallSim` 140,568
Sierra felts, pile10 impact step 391,314 steps, flight step 11,367, pile10 reference shot 22.0M steps over 151 ticks);
`docs/adr/0001-upstream-divergences.md` entry 37 (`StepConfig`); on `main`: `crates/rapier_sink/**` (fixtures,
`programs/`), `scripts/bytecode_size.py` (`table`, `attribution --class … --cut LABEL=REGEX`),
`crates/rapier2d/src/{pipeline.cairo,pipeline/**,world.cairo}` (the step and `StepConfig`),
`crates/rapier2d/tests/game_path.cairo` (`steps_game_basic_*`). The game, read-only: `/home/claude/projects/slingfall`
(`crates/slingfall_contract/src/simulate/class.cairo` — how the game calls the step today;
`crates/slingfall_replay/tests/golden.cairo` and `fixtures/levels/pile10.json` — the pile10 world and its reference shot).

**Programme decision (2026-09-27).** The game's contract is live on Sepolia but a settled record takes 73 min through
SHARP. The only path under 5 min is SNIP-36: the physics in **declared classes, each ≤ 81,920 felts in Sierra AND in
CASM**, called through `library_call`, with **≤ 10M Cairo steps per transaction**. Target: every class ≤ 73,728 felts
(10 % margin) in both, and a step overhead ≤ +25 % on the game-shaped path.

## 2. Scope (file allowlist)
- Merged to `main` (one PR): `docs/research/class-split.md` (new, the deliverable); `crates/rapier_sink/**` (new
  **test-only** fixture contracts / programs: e.g. a `BasicStepConfig` game-step class, per-phase classes, a
  `library_call` chain driven from snforge tests); `scripts/bytecode_size.py` (new fixtures, per-phase attribution
  cuts); `gas/bytecode.size` only if a tracked fixture is added (existing lines unchanged).
- Prototype only, **never merged** (push it as branch `proto/cs3-phase-dispatch`, no PR): any change the measurements
  need in `crates/rapier2d/src/**`, `crates/rapier_dynamics2d/src/**`, `crates/rapier_geometry2d/src/**` (e.g. making
  phases callable separately, a phase-dispatch generic). Its results must stay bit-identical to `main`.
- Forbidden on the merged PR: any change to the engine crates' `src/**`, their `Scarb.toml` / `lib.cairo`, the step's
  results or Cairo steps. Nothing enters the step path without the programme's go.

## 3. Questions (each answered with measurements from real builds)
1. **Size by phase** of the `BasicStepConfig` step as the game builds it (`step_with_force_events_with::<BasicStepConfig>`
   + `WorldState` in / out): user changes, broad phase, narrow phase per shape pair (ball / cuboid / convex polygon /
   half-space pairs), islands / sleep, constraint build, solve, integrate, force events, `WorldState` encode / decode —
   Sierra felts and CASM felts per phase (attribution on a class that reaches exactly the game's code; state the
   attribution rule for shared code: maths, arena, corelib).
2. **Candidate cuts** into 2, 3 or 4 classes, each ≤ 73,728 Sierra and CASM felts (count each class's shared code in it),
   and for each boundary: the data that must cross (types, felts on a pile10 impact step and on a flight step). Look
   beyond CS1's "whole `WorldState` per crossing": e.g. the caller class keeps the world and passes each phase only its
   inputs (pairs, poses, shapes → manifolds; constraints → impulses); phases skipped when empty (a flight tick with no
   contact pair should not call the narrow-phase / solver classes); generator classes per shape-pair family called
   only for pairs that exist.
3. **Measured cost of each cut** on the game-shaped path: extra Cairo steps per engine step for `Serde` + `library_call`
   (flight step, pile10 impact step), and the resulting total of the pile10 reference shot (151 ticks; a local
   reproduction of the pile10 world and shot is fine — state how close it is to slingfall's 22.0M). Target ≤ +25 %;
   report whatever you measure, and the per-transaction steps against the 10M limit (how many ticks per transaction).
4. **API**: can `step_with::<C>` (or `StepConfig`) take the phase dispatch as a generic — in-process by default,
   `library_call` in a contract — so that results stay bit-identical and non-contract users pay nothing (0 steps,
   same `program.basic`)? Show it on the prototype branch with the exact Cairo steps of `steps_game_basic_*` equal.
5. **Recommendation**: the cut to build, its expected sizes and steps, the risks (e.g. whether the SNIP-36 virtual OS
   executes `library_call_syscall`: say what is verified and what is not), and the lots it implies (IDs, scope,
   order), with what each would change in the step path.

## 4. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`): `scarb
fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p rapier_sink`; `python3 scripts/bytecode_size.py
check` (or `snapshot` if a tracked fixture is added); for the prototype, the `steps_game_basic_*` probes before / after.
Never a workspace-wide run. Rebase on `origin/main` before the PR. Commit wip states early (the prototype branch too).
Conventional commits + trailer; push; `gh pr create --base main --title "<what ships>" --body-file …`; `gh pr checks
--watch` until green; never merge; `REPORT.md` (Summary · Size by phase · Cuts table · Crossing costs and pile10 shot ·
API finding · Recommendation and implied lots · Verified vs open · Prototype branch · PR URL). Memory rules apply.

## 5. Work autonomously, do not ask questions, do not widen the scope.
