# SO — why `box_stack3` diverges from upstream (strict per-sample comparison)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D8 sequential solver in pair order, D11; the SD/DM/GS story: SD found two
causes for the slopes — per-surface lever arms (fixed by DM) and upstream's f64 feature-id bug (fixed in
the references by GS) — and the slide's remaining failure is an exact zero-gap tie on upstream's `dist <= 0`
soft/rigid switch); `tools/golden/README.md` (scenes, the SD diagnostics section and how SD instrumented
the harness: per-step manifolds, points, `dist`, feature ids, impulses, `solver_dp1/2`); on `main`:
`crates/rapier2d/tests/golden_scenes.cairo` + `golden_scenes/{builder,slope_diagnostics}.cairo` (P2/SD/GS:
`box_stack3` passes its rest invariants but fails the strict comparison — GS measured 12 violations in
window 0–60, maxima tx 7.5M, ty 4.4M, vy 218M raw, the gap opening at landing around steps 3–6, and 3 in
window 60–120), `tools/golden/src/{scenes.rs,main.rs}`, `tools/golden/vendor/` (patched parry).
Engine, read-only: `crates/rapier2d/src/pipeline.cairo`, `crates/rapier_dynamics2d/src/{narrow_phase.cairo,solver/**}`,
`crates/rapier_geometry2d/src/{contact_generators/cuboid_cuboid.cairo,sat.cairo,polygonal_feature.cairo,manifold.cairo}`.
Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/dynamics/solver/` (how the staged island solver
orders contact constraints: interaction-graph edge order, colouring or none in 2D with enhanced
determinism?), `$UP/rapier/src/geometry/narrow_phase/`, `$UP/rapier/src/dynamics/island_manager*`.

## 2. Scope (file allowlist)
Rust harness diagnostics for `box_stack3` only (`tools/golden/src/scenes.rs`, `tools/golden/vectors/scenes.json`
additions, `crates/rapier_golden/src/types.cairo` new types only, `crates/rapier_golden/src/generated/scenes.cairo`
generated, `tools/golden/README.md` scenes section), Cairo diagnostics in
`crates/rapier2d/tests/golden_scenes/stack_diagnostics.cairo` (new, declared from `golden_scenes.cairo`),
`crates/rapier2d/tests/golden_scenes.cairo`, and the snapshots `gas/rapier2d_integrationtest/golden_scenes.snap`,
`gas/rapier_golden_integrationtest/sanity.snap`. **Engine files are read-only**: when you find the cause,
describe the exact fix (file, function, before/after, numbers) under Escalations; exception: a fix of
≤ 10 lines in one engine file whose crate's tests all still pass may go in a SEPARATE commit, flagged.

## 3. Expected result
The first quantity that differs, quantity by quantity, at the first diverging step, among: (a) broad-phase
pair set and order; (b) per-pair manifolds (normal, point count and order, feature ids, `dist`) — watch
exact SAT ties between faces of aligned equal boxes; (c) solver contacts and the NEW/matched status; (d)
**the order in which upstream solves the stack's constraints** vs the port's ascending pair order (D8) —
if upstream's order differs, report it precisely and what matching it would cost (the order is part of
the state transition: changing it is a decision for the orchestrator, give the evidence); (e) exact-zero
`dist` ties on the soft/rigid switch like the slide (resting boxes in Q32.32 may sit at exactly 0); (f)
anything else. For each candidate cause, a counterfactual test that shows how much of the gap it closes
(as SD did). Then the recommendation.

## 4. Efficiency
Not a gas package; do not regress any snapshot outside your allowlist.

## 5. Tests
Counterfactual diagnostics are tests (they pass, assert the measured recovery); regeneration idempotent;
≤ 800 lines per file, no fuzz.

## 6. Definition of done
`cargo run --release --locked` twice in `tools/golden` (wrapped in `flock $HOME/orchestrator/heavy-build.lock`),
second run zero diff; foreground Cairo gate (tool timeout 3600000 ms); module-filtered snapshots; `gas.py
check`; conventional commits + trailer; push; `gh pr create` per template; `gh pr checks --watch` until green;
never merge; `REPORT.md` (Summary · First divergence · Causes with counterfactual recovery · Upstream solve
order vs port · Recommendation · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
