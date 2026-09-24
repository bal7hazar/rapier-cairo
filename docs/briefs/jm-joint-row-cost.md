# JM — joint limit/motor rows: pay only when enabled, and give plain joints their cost back

## 1. Read first
`AGENTS.md` (§7 gas cost model: an outlined loop-free function pays its costliest path; inlined `match`
pays the reached arm; one-iteration `while` meters an arm; loops pay per iteration run); `docs/PLAN.md` (OJ,
JL findings); JL's REPORT in PR #96's body: one-joint world step 4.34M plain, **7.07M with a limit whether it
is active or not**, 6.89M with a motor; plain pendulum frames +1.6–1.8 % since JL; the inline dispatch won
over early-return and metered-loop candidates *for the row dispatch* (but the limit/motor row construction
itself is always charged). On `main`: `crates/rapier_dynamics2d/src/solver/joint.cairo`,
`solver/joint/{bounded.cairo,bounded/*,kernels.cairo,helper.cairo,row.cairo,alternatives*}`,
`solver/island/sweeps.cairo` (joint stages), `crates/rapier_dynamics2d/src/joint/{config,builder_controls}.cairo`,
`crates/rapier2d/tests/gas_scenes.cairo` (pendulum probes + `steps_step_*` twins).

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/solver/joint.cairo`, `solver/joint/**`, `solver/island/sweeps.cairo` (joint
functions only), `crates/rapier_dynamics2d/tests/joint_scenes.cairo` (only to follow an internal change),
`crates/rapier2d/tests/gas_scenes.cairo` (ceilings only, downward; you may add probes `gas_step_pendulum_limited`
and `gas_step_wheel_motor` built like the existing pendulum ones, with `steps_step_*` twins), and the
snapshots that move. Forbidden: everything else (lot CP1 changes geometry and `Shape` matches in parallel).

## 3. Expected result — pure optimisation, bit-identical
Measure first (Sierra gas and exact Cairo steps) per joint per step for: plain revolute / prismatic / fixed,
revolute with an inactive limit, with an active limit, with a velocity motor, with a position motor. Then
make the limit and motor rows cost only when enabled and only their own arm: e.g. build them behind
metered calls (`while pending`), keep the plain path free of the bounded machinery, specialise the joint
kind once at `prepare_joints` (the kind does not change during a step), avoid copying the bounded state
when it is absent. Every result bit-identical: `joint_scenes`, the substep and golden replays (pendulum,
pendulum_limited, wheel_motor, slider_limited, servo) unchanged; add an old-vs-new fuzz on random chains
with random limits/motors (raw compare of velocities and impulses).

## 4. Efficiency
Targets: plain pendulum chains back to their pre-JL cost (≤ +0.3 % vs 3.77M / 10.15M net frame, see JL's
report), an inactive limit ≤ +10 % over a plain joint, an active limit or a motor ≤ +40 %. No Cairo-steps
regression. Losers under `mod alternatives` with probes; ≤ 4 fuzz per module; ≤ 800 lines per file; P3
ceilings only move down.

## 5. Tests
Existing ones unchanged; the equivalence fuzz; `gas_*` per candidate and per row kind.

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms; crate-scoped runs via `scripts/build-shims/snforge -p …` while
iterating); `gas.py check` then module-filtered snapshots; if CP1 merged meanwhile, rebase and regenerate;
conventional commits + trailer; push; `gh pr create` per template; wait for the checks to be registered, then
`gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Per-joint breakdown before/after, gas |
exact steps · Winners and losers · Scene budgets · Deviations (none) · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
