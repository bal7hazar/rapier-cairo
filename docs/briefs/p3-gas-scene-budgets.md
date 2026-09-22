# P3 — `gas_scene_*` budgets per step, in Sierra gas and Cairo steps

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (§4 wave-4/5 measured costs, the path-insensitivity finding, P1's stage
split: BOX_STACK3 = user changes 262k · broad phase 338k · narrow phase 2.42M · solver 14.4M ·
position update 418k Sierra gas); `scripts/gas.py` (`rank`); on `main`: `crates/rapier2d/src/pipeline/benches.cairo`
(P1's probes and its `#[inline(always)]` probe pattern), `crates/rapier2d/tests/world_step.cairo`,
`crates/rapier_testing` (`opaque`), `crates/rapier_dynamics2d/src/solver/island/benches.cairo` (DF's
per-stage probes). Upstream is not needed.

## 2. Scope (file allowlist)
`crates/rapier2d/tests/gas_scenes.cairo` (+ `crates/rapier2d/tests/gas_scenes/*.cairo`),
`gas/rapier2d_integrationtest/gas_scenes.snap`, and a new `docs/BUDGETS.md`. Nothing else.

## 3. Expected content
A benchmark matrix of **one `World::step`** (settled state, i.e. after N warm-up steps run outside
the probe via `opaque`) by scene family × size: free fall (1, 8, 32 bodies, no pairs), balls on a
half-space (1, 8, 32), cuboid stack (1, 3, 5, 10), mixed ball/cuboid/capsule pile (8), pendulum
chain (1, 3 joints). Each probe reports Sierra gas (`gas_*` test, snapshot) and Cairo steps (`snforge
test <name> --detailed-resources --tracked-resource cairo-steps`, table in `docs/BUDGETS.md`). Fit
and report a per-body / per-pair / per-manifold-point marginal cost for each resource
(`scripts/gas.py rank rapier2d_integrationtest::gas_scenes` for the ranking). Add
`#[available_gas]` ceilings on the hot-path probes at +10 % of the measured value so that a
regression fails loudly (state the value in the attribute, not a constant).

## 4. Efficiency
This package measures, it does not optimise — but it names the top-3 optimisation targets with
their share of a stack step and the expected gain (solver sweeps, narrow phase per pair, proxies),
in `docs/BUDGETS.md`, so that the orchestrator can brief them. Compile budget: table-driven, ≤ 800
lines per file, no fuzz; keep the total number of probes ≤ 24 (CI `test` job is at ~4 min).

## 5. Tests
The probes themselves; a sanity assertion inside each that the step produced the expected number
of contact pairs (so a probe cannot silently measure an empty world).

## 6. Definition of done
Foreground gate; `python3 scripts/gas.py snapshot --filter rapier2d_integrationtest::gas_scenes`;
`gas.py check`; conventional commits + trailer; push; `gh pr create` per template; `gh pr checks
--watch` until green; never merge; `REPORT.md` (Summary · Budget matrix (gas | steps) · Marginal
costs · Top-3 targets · Deviations · Deferred · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
