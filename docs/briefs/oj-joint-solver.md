# OJ — optimise the joint path (≈ 4.3M Sierra gas per joint per step), bit-identical

## 1. Read first
`AGENTS.md` (§7 gas cost model); `docs/PLAN.md` (DE, DF, OS, OI findings); `docs/BUDGETS.md` (current:
`pendulum_chain1` 4.82M gas | 39.1k steps, `pendulum_chain3` 13.35M | 107.2k — ≈ 4.3M | 34k per joint,
only −3 to −5 % since 09-22 while contact scenes lost 30 %). On `main`: `crates/rapier_dynamics2d/src/solver/joint.cairo`
+ `solver/joint/{helper,row,alternatives}.cairo` (DE rows, Gram–Schmidt, per-substep rebuild),
`solver/island/sweeps.cairo` (`prepare_joints`, `rebuild_joints` every substep, `joints(…)` sweeps,
`write_joints`), `solver/island.cairo` (OS: separate joint stages; OI: free bodies solved alone — a
jointed body goes through the dict store), `crates/rapier_dynamics2d/src/joint/{builders,set}.cairo`,
`crates/rapier2d/tests/gas_scenes.cairo` (`gas_step_pendulum_chain{1,3}` + `steps_step_*` twins) and
`crates/rapier2d/src/pipeline.cairo` (`joint_values`, `write_joints` glue). OS's report (PR #68 body) lists
the joint candidates it already rejected (direct bodies, dict storage, metered, specialised rows) — do not
redo them without a new reason. Upstream (`UP=/home/claude/git/refs`):
`$UP/rapier/src/dynamics/solver/joint_constraint/`, `$UP/rapier/src/dynamics/solver/staged_island_solver/worker.rs`.

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/solver/joint.cairo`, `solver/joint/**`, `solver/island/sweeps.cairo`
(joint functions only), `crates/rapier_dynamics2d/src/joint/**` (internal representation only — the public
builder API and `ImpulseJoint` fields stay), `crates/rapier_dynamics2d/tests/joint_scenes.cairo` (only to
follow an internal change), `crates/rapier2d/src/pipeline.cairo` joint glue only, and the snapshots that
move (module filters after `gas.py check`). Forbidden: contact solver, body store (unless a joint-only
path needs a read helper — escalate), narrow/broad phase, frozen interface types, `Scarb.toml`/`lib.cairo`,
`tools/golden/**` and `crates/rapier2d/tests/golden_scenes*` (lot SO works there in parallel).

## 3. Expected result — pure optimisation, bit-identical
Measure first, per joint per step, in **Sierra gas AND exact Cairo steps**: `prepare_joints`, the 4
per-substep `rebuild_joints` (anchors, frames, Gram–Schmidt), warm start, biased and relaxed sweeps,
writeback, and the pipeline glue (`joint_values` / `write_joints` array rebuilds). Report the per-joint
split for a revolute joint (pendulum) and a prismatic and a fixed joint (build them in a probe). Then fix
what dominates. Every result bit-identical: `joint_scenes`, the substep and golden pendulum replays
unchanged; add an old-vs-new fuzz on random joint chains (raw compare of velocities and impulses).
Realistic inputs only (chains of 1, 3, 8 joints built through the public builders), no benchmark-shaped
shortcuts.

## 4. Efficiency and variants
Target: ≥ 25 % less Sierra gas on `gas_step_pendulum_chain3`, no Cairo-steps regression anywhere, contact
scenes unchanged or better. Losers under `mod alternatives` with probes; ≤ 4 fuzz per module; ≤ 800 lines
per file; P3 ceilings only move down.

## 5. Tests
Existing ones unchanged; the equivalence fuzz; `gas_*` per candidate and per joint type.

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms, never background); `gas.py check` then module-filtered
snapshots; if another lot merged meanwhile, rebase and regenerate; conventional commits + trailer (subject
must describe what ships); push; `gh pr create` per template; `gh pr checks --watch` until green; never
merge; `REPORT.md` (Summary · Per-joint breakdown before/after, gas | exact steps · Winners and losers ·
Scene budgets · Deviations (none numerically) · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
