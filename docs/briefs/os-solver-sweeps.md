# OS — optimise the solver sweeps (81 % of a stack step), bit-identical results

## 1. Read first
`AGENTS.md` (§2 measure-never-guess, §7 the gas cost model: Sierra gas charges a loop-free function its
most expensive path; an enum `match` inlined into its caller pays only the reached arm; a computing arm
inside an outlined function is "metered" with a one-iteration `while pending { …; pending = false; }`;
loops are charged per iteration run); `docs/PLAN.md` (D8, D9, DC/DF/P1/P3 findings); `docs/BUDGETS.md`
(targets: solver contact sweeps = 80.7 % of `BOX_STACK3`'s 17.9M-gas step; a 20–25 % solver reduction
saves 2.9–3.6M). On `main`: `crates/rapier_dynamics2d/src/solver/island.cairo` and `island/sweeps.cairo`
(DF: `contacts(ref cs, ref bodies, ms, p, stage)` and `joints(…)`), `solver/contact.cairo` +
`solver/contact/{set,element,checks,benches,alternatives,fixtures}.cairo` (DC + DM),
`solver/joint.cairo` + `solver/joint/*` (DE), `solver/body.cairo`, `solver/body_store.cairo` (DF's
`Felt252Dict` dense store, `DenseBodiesTrait::{get, set_pair}`), `crates/rapier2d/tests/gas_scenes.cairo`
(P3 budget probes — your acceptance numbers) and `crates/rapier2d/src/pipeline/benches.cairo` (P1 stage
split). Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/dynamics/solver/staged_island_solver/worker.rs`,
`$UP/rapier/src/dynamics/solver/contact_constraint/two_body_constraint.rs` (element solve).

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/solver/**` (every file, including the frozen-by-convention DC/DE internal
APIs: you may change how constraints receive their two bodies), `crates/rapier_dynamics2d/tests/{contact_solver_scenes,substep_scenes,joint_scenes}.cairo`
(only to follow an internal API change — expectations must NOT change), and the snapshots that move:
`gas/rapier_dynamics2d/**`, `gas/rapier_dynamics2d_integrationtest/**`, `gas/rapier2d/**`,
`gas/rapier2d_integrationtest/**` (regenerate with module filters after `gas.py check` names them).
Forbidden: `rapier2d` sources, `narrow_phase`, geometry, the frozen types of `docs/interfaces/**`
(`SolverContact`, `ContactManifold`, `ContactData`), any `Scarb.toml`/`lib.cairo`.

## 3. Expected result — a pure optimisation
**Results must be bit-identical**: every existing test (golden replays, substep scenes, joint scenes,
P2 golden scenes, P3 scene probes' sanity asserts) passes unchanged; add a fuzz (≤ 4 per module) that
runs the old and new sweep on the same random stack and compares every body velocity and impulse
raw-for-raw. Suspects already visible in `island/sweeps.cairo::contacts` (measure each before fixing):
1. `match stage { 0 => update + warmstart, 1 => solve(bias), 2 => rhs + solve, 3 => solve, _ => restitution }`
   inside the per-manifold loop body: per the cost model every biased/relaxed pass may pay the
   update+warm-start arm. Candidates: one loop per stage (monomorphised helpers), or metered arms.
2. A fresh `Array` of the two `SolverBody` per manifold per pass (`array![bodies.get(i), bodies.get(j)]`)
   then `set_pair` back: candidates: a by-value two-body struct / tuple through the constraint API,
   skipping the dict write for `WORLD` / static bodies.
3. The whole constraint array rebuilt every pass (`pop_front` + `append` of a large struct with two
   elements): candidates: split hot (impulses, accumulators) from cold (coefficients computed at
   `update`) data so passes iterate a `Span` of cold data and rewrite only the hot part, or keep
   constraints in the dict store.
4. The same three questions for `joints(…)`.
Upstream's order of operations and every arithmetic expression stay exactly the same.

## 4. Efficiency and variants
Report, in **Sierra gas and Cairo steps**, before/after: one substep contact sweep per manifold (1 and 2
points), per joint row, and the P3 scene budgets (`gas_step_cuboid_stack{1,3,5,10}`, `balls_halfspace8`,
`mixed_pile8`, `pendulum_chain3`). Target: ≥ 20 % less Sierra gas on `gas_step_cuboid_stack3` with no
Cairo-steps regression anywhere. Every candidate you try is measured and the losers stay under `mod
alternatives` with their probes (≤ 4 fuzz per module, ≤ 800 lines per file — split files if needed).
Update P3's `#[available_gas]` ceilings only downward, never upward.

## 5. Tests
All existing ones unchanged; the old-vs-new equivalence fuzz; `gas_*` for every candidate.

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms, never background); `gas.py check` then module-filtered
snapshots; conventional commits + trailer; push; `gh pr create` per template (before/after table in the
body); `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Suspects measured · Winners
and losers (gas | steps) · Scene budgets before/after · Deviations (must be none numerically) · Deferred
· Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
