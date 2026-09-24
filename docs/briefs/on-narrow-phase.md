# ON — narrow phase: one implementation, cheaper per pair

## 1. Read first
`AGENTS.md` (§7 gas cost model: loop-free bodies are charged their costliest path, inlined `match` pays
the reached arm, metered calls via one-iteration `while`); `docs/PLAN.md` (DD, GG/GM metered dispatcher,
P1, OP findings); `docs/BUDGETS.md`. On `main`: `crates/rapier_dynamics2d/src/narrow_phase.cairo` (+
`narrow_phase/*`: `ContactDispatcher`, `PairCollider`, `pair_collider(s)`, `CarryOver`/`SortedMerge`,
`compute_contacts_with`, `dropped_events`, `process_pair`, `pair_filtered`, `update_manifold`,
`solver_contact`), `crates/rapier2d/src/pipeline.cairo` (OP's `collision_inputs` builds the
`PairCollider` scratch in the fused walk, and `contacts_from_scratch` is a **copy** of
`compute_contacts_with` because `narrow_phase.cairo` was out of OP's scope), `crates/rapier2d/src/dispatcher.cairo`,
`crates/rapier_geometry2d/src/{dispatch.cairo,manifold.cairo}` (GM metered dispatcher; GE
`try_update_contacts`/`match_contacts`), `crates/rapier2d/tests/gas_scenes.cairo` (P3 probes + `steps_step_*` twins).
Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/geometry/narrow_phase/`.

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/narrow_phase.cairo` (+ `narrow_phase/*.cairo`),
`crates/rapier_dynamics2d/tests/narrow_phase_scenes.cairo`, `crates/rapier2d/src/pipeline.cairo` (+
`pipeline/*.cairo`) only for the narrow-phase glue (`collision_inputs`, `contacts_from_scratch` and their
tests/probes), and the snapshots that move (module filters after `gas.py check`). Forbidden: geometry
(generators, dispatch, manifold, broad phase — lot BG is on `broad_phase.cairo`), solver, frozen interface
types (`ContactManifold`, `ContactManifoldData`, `SolverContact`, `ContactData`), `Scarb.toml`/`lib.cairo`.

## 3. Expected result — dedupe, then optimise, bit-identical
1. **One implementation.** Move the scratch-driven loop into `narrow_phase` (e.g. `compute_contacts_from_scratch`
   next to `compute_contacts_with`, or make the latter take the scratch) and delete the copy in
   `rapier2d::pipeline`; the pipeline calls the narrow-phase function. Behaviour and gas unchanged by this step.
2. **Measure per pair**, Sierra gas and exact Cairo steps, for ball–ball, ball–halfspace, cuboid–cuboid
   (resting, cached), cuboid–halfspace: `carry.take`, `pair_filtered`, the pose composition `pos12`,
   the dispatcher call, `match_contacts`, the `SolverContact` build (`solver_contact` per point),
   friction/restitution combine, event status, the `current.append`. Then cut what dominates. Suspects:
   whole-`ContactPair` copies (a manifold is large) in carry-over, `append` and events; recomputing
   `pos12` from two poses per pair; `solver_contact` per point doing work that depends only on the pair;
   outlined loop-free functions paying their costliest path (the dispatcher is metered, `process_pair`
   and `update_manifold` are not).
3. Every result bit-identical: narrow-phase scenes, world_step, golden scenes, P3 sanity asserts.

## 4. Efficiency and variants
Targets: ≥ 25 % less Sierra gas for the narrow-phase stage of `cuboid_stack3` and `balls_halfspace8`, no
Cairo-steps regression, no other scene worse. Realistic inputs only (P3 scenes and pairs built through
`World`). Losers under `mod alternatives` with probes; ≤ 4 fuzz per module (old vs new narrow phase on
random worlds, raw compare of pairs, manifolds and events); ≤ 800 lines per file; P3 ceilings only go down.

## 5. Tests
Existing ones unchanged; the equivalence fuzz; `gas_*` per candidate and per pair kind.

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms, never background); `gas.py check` then module-filtered snapshots;
if lot BG merged meanwhile, rebase and regenerate; conventional commits + trailer (subject = what ships);
push; `gh pr create` per template; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary ·
Dedupe · Per-pair breakdown before/after, gas | exact steps · Winners and losers · Scene budgets ·
Deviations (none numerically) · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
