# DO — solve pairs with a fixed body last (D8 amended after SO), strict `box_stack3`

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D8 as amended in this PR's base: dynamic–dynamic pairs first, pairs with a
fixed/world body last, each group in ascending pair order); SO's report in PR #76's body and
`crates/rapier2d/tests/golden_scenes/stack_diagnostics.cairo` (the colour-order counterfactual, the
`seed_impulses` helper, the measured recoveries: 0 violations in window 0–60 at max 404 / 909 ulp, and 0
in window 60–120 once the re-seed also restores upstream's impulses); `tools/golden/README.md` (SO's
section: upstream gives a touching pair a persistent colour — non-fixed pairs the lowest free colour
< 120, pairs with a fixed body the highest free colour < 128 — and solves colours in ascending order);
on `main`: `crates/rapier2d/src/pipeline.cairo` (`solve_and_advance` ≈ l.571–582 builds the touching
manifold list; `scatter_touching`; `solve`; `touching_manifolds`). Upstream (`UP=/home/claude/git/refs`):
the colour assignment SO cites — check how KINEMATIC bodies are classified there (fixed-like or not) and
follow upstream.

## 2. Scope (file allowlist)
`crates/rapier2d/src/pipeline.cairo` (+ `pipeline/*.cairo`) — only the functions that build, order and
scatter the touching manifolds (lot OJ edits the joint glue in the same file in parallel: stay out of
`joint_values` / `write_joints`), `crates/rapier2d/tests/golden_scenes.cairo`,
`crates/rapier2d/tests/golden_scenes/*.cairo`, `crates/rapier2d/tests/world_step.cairo` (only if an
expectation encodes the old order — say which), and the snapshots that move (module filters after
`gas.py check`). Forbidden: engine crates other than `rapier2d`, `tools/golden/**`, docs.

## 3. Expected result
The solver receives the touching manifolds as a **stable partition** of the ascending pair list: first the
pairs whose two bodies are both non-fixed (per upstream's classification), then the pairs involving a
fixed body or no body; `scatter_touching` writes the solved impulses back to the right pairs. Nothing else
changes (manifold contents, warm start, events, determinism: no dict iteration). Then: `box_stack3` moves
from rest invariants to the **strict per-sample comparison** in both windows (the second window re-seeds
upstream's impulses with SO's `seed_impulses`); `stack_diagnostics.cairo` keeps its counterfactuals but
the "colour order" one now equals the engine — assert that, and drop duplicates. Record the 4-box case SO
described (upstream order `(1,2),(3,4),(2,3)` ≠ partition) as a documented limitation in a test comment.

## 4. Efficiency
SO measured +27k Sierra gas per step for the partition; report yours (gas | exact steps) on
`cuboid_stack{3,10}`, `mixed_pile8`, `balls_halfspace8`. No other regression.

## 5. Tests
All existing tests pass (the slide stays `#[ignore]`d on its zero-gap tie); `box_stack3` strict; an
order test on a hand-built world (mixed fixed/dynamic pairs) asserting the solve order.

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms, never background); `gas.py check` then module-filtered snapshots;
if lot OJ merged meanwhile, rebase and regenerate; conventional commits + trailer; push; `gh pr create`
per template; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Order rule and
kinematic classification (with the upstream line) · box_stack3 strict results (max ulp per quantity) ·
Gas | steps · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
