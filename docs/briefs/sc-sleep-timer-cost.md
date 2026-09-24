# SC — cut the awake cost of the sleep timer (free fall +7 % → ≤ +3 %), bit-identical

## 1. Read first
`AGENTS.md` (§7 gas cost model: a loop-free body pays its costliest path; inlined `match` pays the reached
arm; one-iteration `while` meters an arm); `docs/PLAN.md` (SL findings); SL's REPORT in PR #95's body (free
fall +6.8 to +7.7 % since SL: ≈ 29k gas per moving body, of which `update_sleep_timer` 17.6k — drift 6.0k,
810 when `cannot_sleep`; option C: hoist `threshold · length_unit · dt`-style per-step constants ≈ 2.4k/body);
on `main`: `crates/rapier2d/src/pipeline/islands.cairo` (`update_sleep_timer`, `relative_pose_drift`,
`update_islands`), `crates/rapier2d/src/pipeline/sleeping.cairo`, `crates/rapier_core/src/rigid_body/activation.cairo`
(`dynamic_gate`, `update_timer`), `crates/rapier2d/tests/gas_scenes.cairo` (P3 probes, `steps_step_*` twins).

## 2. Scope (file allowlist)
`crates/rapier2d/src/pipeline/islands.cairo` (+ `pipeline/islands/*.cairo`), `crates/rapier2d/src/pipeline/sleeping.cairo`,
`crates/rapier_core/src/rigid_body/activation.cairo` (internal helpers only; public API and fields unchanged),
`crates/rapier2d/tests/gas_scenes.cairo` (ceilings only, downward), and the snapshots that move. Forbidden:
everything else (lot JL is working on joints and the golden harness in parallel).

## 3. Expected result — pure optimisation, bit-identical
Measure per moving body (Sierra gas and exact Cairo steps): the gate's arms, `relative_pose_drift` (square
roots, the chord division), the timer update, the per-body reads/writes. Candidates: hoist per-step
constants out of the per-body path (option C); skip the drift when the velocity test already decides (only
if upstream's decision is provably identical — prove it with the fuzz); avoid the full `RigidBody` copy for
bodies whose activation does not change; inline or meter the `body_type` match. Every result
bit-identical: `box_stack3_sleep`, `ball_drop_sleep`, all world/golden tests unchanged; add an old-vs-new fuzz
on random bodies (pose deltas, velocities, thresholds incl. negative) comparing the activation raw.

## 4. Efficiency
Target: `gas_step_free_fall{8,32}` within +3 % of the pre-SL values recorded in PR #95 (0.62M / 3.59M / 13.99M
net for 1 / 8 / 32), no Cairo-steps regression, sleeping-stack step not worse. Losers under `mod alternatives`;
≤ 4 fuzz per module; ≤ 800 lines per file; P3 ceilings only move down.

## 5. Tests
Existing ones unchanged; the equivalence fuzz; `gas_*` per candidate.

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms; crate-scoped runs while iterating); `gas.py check` then
module-filtered snapshots; conventional commits + trailer; push; `gh pr create` per template; wait for the
checks to be registered, then `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary ·
Breakdown per body before/after (gas | steps) · Winners and losers · Scene budgets · Deviations (none) ·
Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
