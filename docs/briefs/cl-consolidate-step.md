# CL — consolidate after the optimisation wave: one dispatch table, inlined combine, file budget, ceilings

## 1. Read first
`AGENTS.md` (§7 gas cost model, compile budget ≤ 800 lines per file); `docs/PLAN.md` (GG/GM metered
dispatcher; ON's findings); ON's REPORT in PR #84's body (escalations 1–4); on `main`:
`crates/rapier_geometry2d/src/dispatch.cairo` (+ `dispatch/alternatives.cairo`: the metered table
`contact_manifold`), `crates/rapier2d/src/pipeline/step_dispatcher.cairo` (ON's `StepDispatcher`: the same
table without metering and with `try_update_contacts` hoisted — a second copy), `crates/rapier2d/src/dispatcher.cairo`
(`DefaultDispatcher`), `crates/rapier_dynamics2d/src/narrow_phase.cairo` (`compute_contacts_from_scratch::<D>`,
and `combine` at ≈ l.632, a copy of `rapier_core`'s rule), `crates/rapier_core/src/collider/combine_rule.cairo`
(`CoefficientCombineRuleTrait::apply` is outlined, so its costliest arm — the `sqrt` of `GeometricMean` —
is charged every call), `crates/rapier2d/src/pipeline.cairo` (843 lines), `crates/rapier2d/tests/gas_scenes.cairo`.

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/dispatch.cairo` (+ `dispatch/*.cairo`), `crates/rapier_geometry2d/tests/dispatch_golden.cairo`,
`crates/rapier2d/src/**`, `crates/rapier_dynamics2d/src/narrow_phase.cairo` (+ `narrow_phase/*.cairo`),
`crates/rapier_core/src/collider/combine_rule.cairo`, `crates/rapier2d/tests/gas_scenes.cairo` (ceilings only),
and the snapshots that move (module filters after `gas.py check`). Forbidden: `broad_phase*` (lot BS in
parallel), generators, solver, frozen interface types, `Scarb.toml`; `lib.cairo` re-exports go to Escalations.

## 3. Expected result — bit-identical, never more expensive
1. **One dispatch table.** The step's dispatcher lives in `rapier_geometry2d::dispatch` (e.g. a
   `contact_manifold_step` with `try_update_contacts` hoisted, next to the metered `contact_manifold`, or
   one function if one formulation wins everywhere — measure); `DefaultDispatcher` (and
   `StepDispatcher`, if kept as a name) delegate to it; the copy in `step_dispatcher.cairo` is deleted.
   The dispatch golden test covers the step variant on all 87 cases in both orders.
2. **Combine rule**: make `CoefficientCombineRuleTrait::apply` (and `combine`) inlinable so the reached arm
   only is paid, delete `narrow_phase::combine`; measure `apply` per rule before/after.
3. **File budget**: `pipeline.cairo` ≤ 800 lines by moving cohesive groups into `pipeline/*.cairo`.
4. **Ceilings**: after your changes, reset every `#[available_gas]` in `gas_scenes.cairo` to +10 % of the new
   gross `gas_step_*` value (never above the current ceiling unless a change you ship is a measured,
   documented trade-off).
Every scene and golden test unchanged; no snapshot entry may increase except where you justify it.

## 4. Efficiency
Report per item, Sierra gas and exact Cairo steps: the narrow-phase stage of `cuboid_stack3`,
`balls_halfspace8`, `mixed_pile8`, and the P3 net step for all 13 scenes before/after. ≤ 4 fuzz per module,
≤ 800 lines per file, losers under `mod alternatives`.

## 5. Tests
Existing ones unchanged; the dispatch golden extended to the step variant; `gas_*` per candidate.

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms, never background); `gas.py check` then module-filtered snapshots;
if lot BS merged meanwhile, rebase and regenerate; conventional commits + trailer (subjects = what ships);
push; `gh pr create` per template; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary ·
Items 1–4 with numbers · Scene table · Deviations (none) · Requested re-exports · Escalations · PR URL).
Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
