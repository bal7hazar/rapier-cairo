# CS2 — a step that compiles only what the game reaches (program size at zero Cairo-step cost)

## 1. Read first
`AGENTS.md` (§7 incl. the SH1 lesson on `match` arms); `docs/research/class-size.md` (CS1: decomposition, lever
estimates — the generic step was measured at −49 % CASM for 0 steps before CC / SH2 added code); `docs/PLAN.md` (the CS
row; programme decision 2026-09-27: CS2 comes back because the settled proof path bootloads the game's program and
Pedersen-hashes all of it — slingfall's `c1main` is 582,399 felts ≈ 4.6M steps on top of an 8.9M-step level, pushing
its reference shot from Atlantic's S tier into M); RG1's report (the game-shaped probe); on `main`:
`crates/rapier2d/src/{pipeline.cairo,pipeline/**,world.cairo,dispatcher.cairo}`, `crates/rapier_geometry2d/src/{dispatch.cairo,dispatch/**}`,
`crates/rapier_dynamics2d/src/{narrow_phase.cairo,narrow_phase/**}` (the `ContactDispatcher` hook), `crates/rapier_sink/**`
and `scripts/bytecode_size.py`. Upstream (`UP=/home/claude/git/refs`): parry's pluggable `QueryDispatcher`, rapier's
`PhysicsPipeline::step` arguments (hooks, events, optional CCD).

## 2. Scope (file allowlist)
`crates/rapier2d/src/**` (the configurable step and its strategies), `crates/rapier_geometry2d/src/{dispatch.cairo,dispatch/**}`
(dispatcher composition), `crates/rapier_dynamics2d/src/{narrow_phase.cairo,narrow_phase/**,solver/island.cairo}` (only
the seams the strategies need), `crates/rapier_sink/**` (a game-shaped **executable** fixture next to the contract
ones), `scripts/bytecode_size.py` (track executable program size too), `gas/bytecode.size`, their tests, and the
snapshots that move. Forbidden: kernels, generators' bodies, solver sweeps, `Scarb.toml`/`lib.cairo` (re-exports →
Escalations).

## 3. Expected result
- A configurable step, e.g. `WorldTrait::step_with::<C>()` / `step_with_force_events_with::<C>()` (names to pick,
  upstream-faithful where possible), where `C` supplies: the `ContactDispatcher` (a game dispatcher that knows only the
  game's shape pairs), and no-op or real strategies for joints, sensors / intersection pairs, CCD and composite shapes.
  Monomorphised with the no-op strategies, the joint solver, the sensor tests, the CCD solver, the composite hooks and
  the unused generators must not be compiled into the program.
- `World::step` / `step_with_force_events` / `step_with_ccd` stay exactly as they are (full strategies): same results,
  same Cairo steps.
- A game-shaped configuration (ball / cuboid / convex polygon / half-space, force events, no joints / sensors / CCD /
  composites) with **bit-identical results and Cairo steps** to `World::step_with_force_events` on the same world
  (P3 contact scenes, level windows, RG1's game-shaped probe).
- Program size: `scripts/bytecode_size.py` also reports an `#[executable]` fixture's program felts (full step vs
  game-shaped step), plus the contract classes as today.

## 4. Targets and constraints
Zero Cairo-step cost (before / after table on every probe above) and results bit-identical. Size target: the
game-shaped executable's program at least −40 % against the full one; report every strategy's share. Configurations
that disable a feature must reject worlds that use it (a documented panic at insertion or at step), never silently
ignore it. ≤ 800 lines per file; ≤ 4 fuzz per module.

## 5. Tests
Equivalence tests: each configuration vs the full step on worlds it supports (raw equality of bodies, pairs, events);
rejection tests for unsupported worlds; existing tests unchanged; `python3 scripts/bytecode_size.py check`.

## 6. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`): `scarb fmt
--workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on the crates you touch (one at a time); snapshots
with `--from-log`; `python3 scripts/bytecode_size.py snapshot`; `python3 scripts/api_parity.py --check`. Never a
workspace-wide run: CI is the full gate. Rebase on `origin/main` before the PR (RG1 and SH2b land before you). Commit
wip states early. Conventional commits + trailer; push; `gh pr create --base main --title "<what ships>" --body-file
…`; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API · Program-size table (full vs game-shaped,
per strategy) · Steps-unchanged proof · Results-unchanged proof · Deviations · Requested re-exports · Escalations · PR
URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
