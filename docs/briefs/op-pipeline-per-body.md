# OP — cut the per-body overhead of `World::step` outside the solver

## 1. Read first
`AGENTS.md` (§7 gas cost model: an outlined loop-free function is charged its most expensive path; an
inlined `match` pays only the reached arm; metered arms = one-iteration `while pending { …; pending =
false; }`; loops pay per iteration run); `docs/PLAN.md` (P1, GM, P3 findings); `docs/BUDGETS.md` (free fall
costs **812 838 Sierra gas | 7 301 steps per dynamic body per step with no contact at all**; P1 split of
`BOX_STACK3`: user changes 262k, broad phase 338k, narrow phase 2.42M, position update 418k).
On `main`: `crates/rapier2d/src/pipeline.cairo` (+ `pipeline/benches.cairo`: `step`,
`handle_user_changes`, `detect_collisions`, `solve`, `touching_manifolds`, `scatter_touching`,
`advance_to_final_positions`, `recompute_mass_properties_from_colliders`), `crates/rapier2d/src/world.cairo`,
`crates/rapier_dynamics2d/src/{collider_set.cairo,rigid_body_set.cairo}` (`broad_phase_proxies`:
one `collider.compute_aabb()` per collider per step, DD measured 290k per collider),
`crates/rapier_dynamics2d/src/collider/object.cairo`, `crates/rapier_geometry2d/src/{shape.cairo,aabb.cairo}`
(`ShapeTrait::compute_aabb`: a `match` over the closed shape enum),
`crates/rapier_core/src/data/arena.cairo` (the `Felt252Dict` arena behind the sets; `to_array`, `iter`).

## 2. Scope (file allowlist)
`crates/rapier2d/src/**`, `crates/rapier_dynamics2d/src/{collider_set,rigid_body_set}.cairo`,
`crates/rapier_dynamics2d/src/collider/*.cairo`, `crates/rapier_dynamics2d/src/rigid_body/*.cairo`,
`crates/rapier_geometry2d/src/{shape.cairo,shape/*.cairo,aabb.cairo}`, their `tests/` files only to follow
an internal change (expectations unchanged), and the snapshots that move (`gas/rapier2d*/**`,
`gas/rapier_dynamics2d*/**`, `gas/rapier_geometry2d*/**`, module filters after `gas.py check`).
**Forbidden:** `crates/rapier_dynamics2d/src/solver/**` (lot OS is optimising the solver in parallel,
including `body_store.cairo` — `from_bodies` / `to_bodies` are theirs), `narrow_phase.cairo`,
`rapier_core` (escalate if the arena is the bottleneck), frozen interface types, any `Scarb.toml`/`lib.cairo`.

## 3. Expected result — a pure optimisation, bit-identical
Measure first, per stage, for `free_fall1/8/32` and `cuboid_stack3` (Sierra gas AND Cairo steps): where do
the 812k per body go? Then fix what the numbers point at. Suspects: (1) `compute_aabb` charged the most
expensive shape arm for every collider (outlined `match`) → inline or meter; (2) the arena walked several
times per step (`to_array()` in proxies, user changes, position update, `pair_colliders`) → one pass,
reuse the snapshot; (3) mass-property recompute or world-mass update done for bodies that did not change;
(4) the per-collider pose composition done twice (proxies and narrow phase). Keep upstream's semantics
and the D9 persistent-state rule (no new persisted cache unless measured cheaper **including** its
storage, cf. P1's rejected proxy cache). Every result bit-identical: all golden/scene tests unchanged.

## 4. Efficiency and variants
Target: ≥ 30 % less Sierra gas per body on `gas_step_free_fall{8,32}`, no Cairo-steps regression, no
regression on the contact scenes. Every candidate measured; losers under `mod alternatives` with their
probes; ≤ 4 fuzz per module; ≤ 800 lines per file. P3's `#[available_gas]` ceilings only move downward.

## 5. Tests
Existing ones unchanged; an equivalence test (old vs new stage functions on random worlds, raw compare).

## 6. Definition of done
Foreground gate (tool timeout 3600000 ms, never background); `gas.py check` then module-filtered
snapshots; if lot OS merged first, rebase and regenerate; conventional commits + trailer; push; `gh pr
create` per template (before/after table); `gh pr checks --watch` until green; never merge; `REPORT.md`
(Summary · Per-stage breakdown before/after (gas | steps) · Winners and losers · Scene budgets · Deviations
(none numerically) · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
