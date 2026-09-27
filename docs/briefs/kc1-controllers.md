# KC1 — controllers: PID controller, then the kinematic character controller

## 1. Read first
`AGENTS.md` (§7: steps-unchanged proofs include the game-shaped probes); `docs/PLAN.md` (KC row; programme decision
2026-09-27: KC after the closeout, with the constraint below); `docs/API_PARITY.md` (the `control` module: 47 items —
`PidController`, `PdController`, `PdErrors`, `AxesMask`, `KinematicCharacterController`, `CharacterLength`,
`CharacterAutostep`, `CharacterCollision`, `EffectiveCharacterMovement`, … — the exact list); CC1's shape casts
(`rapier_geometry2d::query::{cast_shapes, ShapeCastOptions}`, `World::cast_shape`, QY2's `QueryPipeline`) which the
character controller sweeps with; the pre-declared module `crates/rapier2d/src/control.cairo`. Upstream
(`UP=/home/claude/git/refs`): `$UP/rapier/src/control/{pid_controller.rs,character_controller.rs,mod.rs}`.

## 2. Scope (file allowlist)
`crates/rapier2d/src/control.cairo` + `control/**`, their tests, the harness `tools/golden/src/**` (new families:
PID corrections, character-controller moves — slopes, steps, snapping, sliding along walls; existing vectors
byte-identical), vectors / fixtures (additions only), `tools/golden/README.md`, `docs/API_PARITY.md` (regenerated),
and the snapshots that move. Forbidden: the step, the pipeline, the solver, geometry kernels, `Scarb.toml`/`lib.cairo`
(re-exports → Escalations).

## 3. Expected result
Upstream names and semantics, 2D: (1) **PID / PD controllers** (`PidController`, `PdController`, `PdErrors`, gains,
axes mask, `linear_correction`, `angular_correction`, `rigid_body_correction`, integral terms, …), pure math on body
poses / velocities; (2) the **kinematic character controller** (`KinematicCharacterController` with `up`, `offset`,
`slide`, `autostep`, `max_slope_climb_angle`, `min_slope_slide_angle`, `snap_to_ground`, `normal_nudge_factor`;
`move_shape` returning `EffectiveCharacterMovement` and the `CharacterCollision`s; `solve_character_collision_impulses`)
built on CC1's shape casts and the scene queries. Every item: implemented, excluded (closed reason), or `missing` with
a reason.

## 4. Constraints and golden
**Nothing of KC may enter the `BasicStepConfig` program or move a step:** `gas/bytecode.size`'s `program.*` entries
and the game-shaped / P3 / level probes' exact Cairo steps unchanged (before / after table). Golden families from
rapier2d-f64 0.35.3 (the vendored copy) within documented bands. Measure a PID correction and a character move (flat
ground, slope, step, wall) in gas and exact steps. ≤ 800 lines per file; ≤ 4 fuzz per module.

## 5. Definition of done
Harness twice → zero diff. Crate-scoped local gate only (AGENTS §6; foreground, through `scripts/build-shims/`):
`scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier2d (and rapier_golden);
snapshots with `--from-log`; `python3 scripts/bytecode_size.py snapshot` (then prove `program.*` unchanged);
`python3 scripts/api_parity.py` then `--check`. Never a workspace-wide run: CI is the full gate. Rebase on `origin/main`
before the PR (PX1 may land first: regenerate `docs/API_PARITY.md`). Conventional commits + trailer; push; `gh pr create
--base main --title "<what ships>" --body-file …`; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary ·
Items · API · Golden results and bands · Costs · Steps and program-size unchanged proof · Deviations · Requested
re-exports · Escalations · PR URL). Memory rules apply.

## 6. Work autonomously, do not ask questions, do not widen the scope.
