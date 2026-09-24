# OI — the solver's per-body cost when a body has no constraint (430k gas per free body)

## 1. Read first
`AGENTS.md` (§7 gas cost model); `docs/PLAN.md` (DF, OS, OP findings; D9 persisted state); `docs/BUDGETS.md`;
OP's measurements (PR #69 body): at 32 free-falling bodies the solver stage still costs 13.6M gas, ≈ 430k
per body with no constraint at all; the rest per body = position update 115k, user changes 59k,
`find_pairs` 97k (lot BP is on it). On `main`: `crates/rapier_dynamics2d/src/solver/body_store.cairo`
(`from_bodies` gathers every body into the `Felt252Dict` dense store and builds `BodyStep`s,
`to_bodies` writes velocities and `next_position` back), `solver/island.cairo` (`solve_island`: per
substep `add_forces`, joint rebuild, contact sweeps, `integrate`; after the substeps restitution,
writeback, `damp`; `snapshot`), `solver/body.cairo`, `crates/rapier2d/src/pipeline.cairo` (`solve`,
`advance_with_snapshot` / `advance_body_with_snapshot`: position ← next_position, world mass, colliders),
`crates/rapier2d/tests/gas_scenes.cairo` (P3 probes and their `#[available_gas]` ceilings).
Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/dynamics/solver/staged_island_solver/worker.rs`,
`$UP/rapier/src/dynamics/solver/velocity_solver.rs`, `$UP/rapier/src/pipeline/physics_pipeline/substep.rs`
(`advance_to_final_positions`).

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/solver/{body_store.cairo,island.cairo,island/*.cairo,body.cairo}`,
`crates/rapier2d/src/pipeline.cairo` (+ `pipeline/*.cairo`), `crates/rapier2d/tests/gas_scenes.cairo`,
the existing tests only to follow an internal change (expectations unchanged), and the snapshots that
move (`gas/rapier_dynamics2d*/**`, `gas/rapier2d*/**`, module filters after `gas.py check`). Forbidden:
`solver/contact*`, `solver/joint*` (just optimised by OS), `broad_phase.cairo` (lot BP in parallel),
`narrow_phase.cairo`, frozen interface types, `Scarb.toml`/`lib.cairo`.

## 3. Expected result — pure optimisation, bit-identical
Measure first where the 430k per idle body go (gather into the dict, `BodyStep` build, per-substep
`add_forces` + `integrate` over every body, `damp`, `to_bodies`, then the separate position-update walk),
in Sierra gas and Cairo steps. Candidates (OP's escalation, plus what the numbers show):
A. fuse `to_bodies` with the position update (`position ← next_position`, world mass, collider poses) so a
   body is read and written once after the solver;
B. carry the world mass properties computed by the previous advance instead of recomputing them in
   `from_bodies`, if D9 allows it without new persisted state (it is already on the body);
C. bodies touched by no manifold and no joint: integrate their 4 substeps in one closed-form pass that is
   **bit-identical** to the substep loop (same floors in the same order — prove it with the fuzz) or keep
   the loop but skip the dict round-trips for them;
D. fixed / sleeping-free static bodies never enter the dict.
Upstream order and every arithmetic expression stay the same.

Also (OS's escalation): P3's `#[available_gas(l2_gas: …)]` ceilings make `snforge … --tracked-resource
cairo-steps` fail on some scene probes, so the budgets have no step counts. Make every P3 probe measurable
in both resources (e.g. keep the ceiling on the Sierra-gas probe and add an uncapped `steps_*` companion,
or another mechanism you measure and justify), then set every ceiling to the new measured value + 10 %.
`docs/**` is not yours: put the full gas | steps matrix in REPORT.md; the orchestrator updates
`docs/BUDGETS.md`.

## 4. Efficiency and variants
Targets: `gas_step_free_fall32` −30 % vs `main`, no Cairo-steps regression anywhere, contact and joint
scenes not worse. Losers under `mod alternatives` with probes; ≤ 4 fuzz per module (one old-vs-new step on
random worlds with free, constrained and fixed bodies, raw compare); ≤ 800 lines per file.

## 5. Tests
All existing tests unchanged; the equivalence fuzz; `gas_*` per candidate.

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms, never background); `gas.py check` then module-filtered
snapshots; if lot BP merged meanwhile, rebase and regenerate; conventional commits + trailer; push;
`gh pr create` per template (before/after table); `gh pr checks --watch` until green; never merge;
`REPORT.md` (Summary · Breakdown per idle body before/after (gas | steps) · Winners and losers · P3 matrix
gas | steps with new ceilings · Deviations (none numerically) · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
