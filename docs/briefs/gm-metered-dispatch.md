# GM — move the metered dispatcher into `rapier_geometry2d::dispatch` (P1 escalation A)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (wave-4 GG finding, wave-5 P1 finding); on `main`:
`crates/rapier_geometry2d/src/dispatch.cairo` (GG's `#[inline(always)]` typed `match`),
`crates/rapier2d/src/dispatcher.cairo` (P1's `DefaultDispatcher`: the same arms, each generator call
wrapped in a one-iteration `while pending { …; pending = false; }` so that an outlined caller is not
charged every generator — read its module doc, it is the specification), `crates/rapier2d/src/pipeline/benches.cairo`
(the ball-vs-cuboid narrow-phase probes that prove it: 1 486 460 vs 3 197 940 Sierra gas for 4 pairs).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/dispatch.cairo`, `crates/rapier_geometry2d/src/dispatch/alternatives.cairo`,
`crates/rapier_geometry2d/tests/dispatch_golden.cairo`, `crates/rapier2d/src/dispatcher.cairo`,
`gas/rapier_geometry2d/dispatch.snap`, `gas/rapier_geometry2d_integrationtest/dispatch_golden.snap`,
`gas/rapier2d/dispatcher.snap`, and — only if their numbers move — `gas/rapier2d/{pipeline,world}.snap`,
`gas/rapier2d_integrationtest/world_step.snap` (regenerate with the module filters, never by hand).

## 3. Expected result
One source of truth: `dispatch::contact_manifold` becomes the metered `match` (P1's arms, verbatim
order), still `#[inline(always)]`; `DefaultDispatcher::contact_manifold` becomes a one-line
`#[inline(always)]` call to it. GG's plain match moves to `dispatch/alternatives.cairo` with a
probe pair (inlined caller vs outlined caller) that documents why metering exists. No behaviour
change: every golden and equivalence test stays as is.

## 4. Efficiency
Acceptance = the numbers P1 measured hold through the real pipeline: `pipeline::benches` narrow
phase for 4 ball pairs ≤ 1.5M and for 4 cuboid pairs ≈ 3.2M Sierra gas, and `dispatch` unit probes
per pair within 1 % of GG's table (`docs/PLAN.md`); Cairo steps +≈265 per pair at most. Report both.

## 5. Tests
Existing ones; add `test_metered_equals_plain` over the 25-pair shape matrix P1 generated (reuse its
generator if reachable, else copy it) — `fuzz_*` ≤ 4, ≤ 800 lines/file.

## 6. Definition of done
Foreground gate; snapshot filters `rapier_geometry2d::dispatch`, `rapier_geometry2d_integrationtest::dispatch_golden`,
`rapier2d::dispatcher` (+ the others above only if `gas.py check` says they drifted); `gas.py check`;
conventional commits + trailer; push; `gh pr create` per template; `gh pr checks --watch` until green;
never merge; `REPORT.md` (Summary · API · Gas table · Deviations · Deferred · Requested re-exports ·
Escalations · PR URL). Memory rules of the system prompt apply (one build at a time, foreground).

## 7. Work autonomously, do not ask questions, do not widen the scope.
