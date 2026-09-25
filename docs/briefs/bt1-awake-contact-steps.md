# BT1 — Cairo steps of an awake contact tick (solver + narrow phase), the level's critical path

## 1. Read first
`AGENTS.md` (§7 gas cost model: path-insensitive Sierra gas, inlining, hot value structs); `docs/PLAN.md` ("Cost of a
level", "Client-side execution", BT row); `docs/BUDGETS.md`; G0's probes `crates/rapier2d/tests/level_budget.cairo`
and replays `crates/rapier2d/tests/golden_scenes/levels.cairo`; the earlier optimisation briefs
`docs/briefs/{os-solver-sweeps,oj-joint-solver,on-narrow-phase,op-pipeline-per-body,oi-idle-body-solver}.md` (what
was already tried, measured and kept); on `main`: `crates/rapier_dynamics2d/src/{solver.cairo,solver/**,narrow_phase.cairo,narrow_phase/**}`,
`crates/rapier_geometry2d/src/{contact_generators.cairo,contact_generators/**,manifold.cairo,dispatch.cairo,dispatch/**}`,
`crates/rapier2d/src/pipeline.cairo` (read only: how the stages are called).

**Why (programme target, owner's guideline):** a 10-block level at 60 Hz × 4 substeps costs 204M Cairo steps over
300 ticks; the target is ≤ 3e7 steps per shot (client: 10 s in the browser; proof: 1.1B gas ≈ 9M steps per
transaction). In G0's impact window the solver is 77 % and the narrow phase 16.5 % of the step (level 10), ≈ 5.8M gas
per awake contacting body-tick at 4 substeps. **Cairo steps are the measure** (the prover pays the executed path);
report Sierra gas next to them.

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/solver.cairo` + `solver/**`, `crates/rapier_dynamics2d/src/narrow_phase.cairo` +
`narrow_phase/**`, `crates/rapier_geometry2d/src/{contact_generators.cairo,contact_generators/**,manifold.cairo,clip.cairo,sat.cairo}`,
their tests and benches, new probes in `crates/rapier2d/tests/level_budget.cairo` (stage probes on the level states),
and the snapshots that move. Forbidden: `crates/rapier2d/src/**` and the collider / world / set files (CW is
changing them in parallel: list any pipeline-side lever under Escalations with its measured gain), frozen interface
types, `Scarb.toml`/`lib.cairo`.

## 3. Method
1. **Profile first.** On the level-10 and level-20 impact states (G0's `*_impact_*` windows) and on P3's
   `cuboid_stack10` / `mixed_pile8`, measure exact Cairo steps per stage and per function family of the awake
   contact path: contact generation and manifold update, constraint build, warm start, each solver sweep (per
   substep), velocity integration / position update, restitution / friction parts, solver body copies. Put the table
   in REPORT.md before optimising.
2. **Rank the levers by steps saved** and implement the top ones, one measured A/B each (winner in the library, losers
   under `#[cfg(test)] mod alternatives` with their numbers). Look in particular at: rebuilding immutable arrays in
   every sweep or substep (copy-on-write of `Array` / `Span` of structs), per-substep recomputation of per-step
   invariants, oversized structs copied in the hot loops, redundant Q32.32 products / divisions, `Option` / `Box`
   unwraps inside loops, dictionary traffic, and loop shapes that defeat inlining. These are hints, not answers: the
   profile decides.
3. **Results:** bit-identical is the default (every golden vector and scene unchanged). A lever that changes results is
   allowed only if every golden check stays within its tolerance band; list it with the per-golden deltas and keep it
   separable (the orchestrator decides and registers it in ADR 0001).

## 4. Targets and anti-overfitting
Goal: **−30 % Cairo steps on the level-10 and level-20 impact windows** at 4 substeps, with the per-unit costs (per
contact point per substep, per awake body per substep) falling by the same order. Measure on shuffled / level-shaped
inputs, never only on a probe built in a favourable order; no fast path that only fires on a benchmark's shape (a
review lesson of BP and RB). P3 scenes may only go down (lower their `#[available_gas]` ceilings to +10 % of the new
gross values). Do not run G0's `#[ignore]`d whole-level replays locally (≈ 15 GB each): use the windows.

## 5. Tests
Existing tests unchanged (golden, scenes, solver unit tests); new stage probes table-driven; ≤ 800 lines per file;
≤ 4 fuzz per module.

## 6. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`):
`scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier_geometry2d,
rapier_dynamics2d and rapier2d; snapshots with `snforge test -p <crate> --tracked-resource sierra-gas > gas-<crate>.log`
and `python3 scripts/gas.py snapshot --filter <crate>::<module> --from-log gas-<crate>.log`; exact steps with
`snforge test -p rapier2d <filter> --detailed-resources --tracked-resource cairo-steps`. Never a workspace-wide run:
CI is the full gate. Commit each kept lever separately (conventional commits + trailer); push; `gh pr create --base
main --title "<what ships>" --body-file …`; wait for the checks to be registered, then `gh pr checks --watch` until
green; never merge; `REPORT.md` (Summary · Profile table · Levers kept / rejected with steps and gas · Before / after:
impact L10 / L20, flight, load, P3 scenes, per-unit costs · Result changes (none, or per-golden deltas) · Pipeline-side
levers for later · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
