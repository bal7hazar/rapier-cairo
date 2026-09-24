# SL — sleeping: per-step islands, sleep timers, wake-ups (phase 2, wave 6)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D8 solve order, **D9 persisted state = poses, velocities, warm-start
impulses and sleep timers**, D11 golden validation, phase 2); `docs/BUDGETS.md` (a resting stack still costs
the full step every frame: sleeping is the largest gas win left for games); `tools/golden/README.md`
(settings table: sleeping is OFF in every trace today via `can_sleep(false)`); on `main`:
`crates/rapier_core/src/rigid_body/activation.cairo` (C3: `RigidBodyActivation` with upstream's fields and
defaults), `crates/rapier_core/src/data/union_find.cairo` (C1), `crates/rapier2d/src/{world.cairo,pipeline.cairo,pipeline/*}`
(the step: user changes, broad/narrow phase, D8 ordering, solver, advance), `crates/rapier_dynamics2d/src/{rigid_body_set.cairo,narrow_phase.cairo,solver/**}`.
Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/dynamics/island_manager/{sleep.rs,manager.rs,island.rs,persistent.rs}`,
`$UP/rapier/src/dynamics/rigid_body_components.rs` (`RigidBodyActivation`: `update_energy`-like threshold test,
`time_since_can_sleep`, `wake_up(strong)`), `$UP/rapier/src/pipeline/physics_pipeline/` (where islands are
built, where sleeping bodies are skipped, where wake-ups happen: contact start with an awake body, joint,
user change, force/impulse application).

## 2. Scope (file allowlist)
`crates/rapier2d/src/**`, `crates/rapier2d/tests/{world_step.cairo,golden_scenes.cairo,golden_scenes/*.cairo}`,
`crates/rapier_dynamics2d/src/{rigid_body_set.cairo,rigid_body/*.cairo}` (wake/sleep helpers only),
`crates/rapier_dynamics2d/src/solver/island.cairo` (+ `island/*`) only to take an "awake bodies" input,
new `crates/rapier_dynamics2d/src/islands.cairo` is NOT allowed (lib.cairo is orchestrator-owned) — put the
island builder in `crates/rapier2d/src/pipeline/islands.cairo`; Rust harness: `tools/golden/src/scenes.rs`,
`tools/golden/vectors/scenes.json` (additions only), `crates/rapier_golden/src/types.cairo` (new types only),
`crates/rapier_golden/src/generated/scenes.cairo` (generated), `tools/golden/README.md` (scenes section);
the snapshots that move. Forbidden: geometry, contact/joint solver internals, frozen interface types,
`Scarb.toml`/`lib.cairo` (list needed re-exports under Escalations).

## 3. Expected semantics (upstream's, observable behaviour)
Per step, bodies are grouped into islands (connected components of dynamic bodies through touching contact
pairs and enabled joints; fixed bodies do not connect islands) with `rapier_core`'s union-find — per-step,
no persisted island structure (D9: only the per-body activation state persists). A body's sleep timer
advances when its motion is below upstream's thresholds (port the exact test: normalised linear threshold ×
`length_unit`, the angular rule, the farthest-point displacement rate) and resets otherwise; an island
sleeps when every member has been below threshold for `time_until_sleep`; sleeping bodies keep their pose,
get zero velocity as upstream does, and are skipped by the solver and by pair generation between two
sleeping/fixed bodies; an island wakes (whole island, strong reset) when an awake dynamic body starts
touching one of its bodies, when a joint links it to an awake body, or on user changes (`set_body`,
forces/impulses, pose/velocity edits) — mirror upstream's wake-up points. Determinism: ascending order
everywhere, no dict iteration. Document every divergence from upstream's persistent-island manager.

## 4. Golden and efficiency
Harness: add scenes with sleeping ON (at least `box_stack3_sleep`: settles and sleeps; `ball_drop_sleep` with
a second ball hitting the sleeping one at a later step to wake it), recording per sample the `is_sleeping`
flag of every dynamic body; existing traces unchanged (they keep `can_sleep(false)`). Report, Sierra gas and
exact Cairo steps: a step of a fully sleeping `cuboid_stack10` (target ≤ 15 % of today's awake step), the
awake overhead of the island/timer bookkeeping on the P3 scenes (target ≤ +3 %), and a wake-up step.

## 5. Tests
Golden sleep scenes (sleep step and wake step exact, states within the README tolerances); table-driven
world tests (never sleeps with `normalized_linear_threshold < 0`, kinematic/fixed never sleep, force wakes,
joint-linked islands sleep together); ≤ 4 fuzz per module; ≤ 800 lines per file.

## 6. Definition of done
`cargo run --release --locked` twice in `tools/golden` (under `flock $HOME/orchestrator/heavy-build.lock`),
second run zero diff; foreground Cairo gate (tool timeout 3600000 ms, never background); `gas.py check` then
module-filtered snapshots; conventional commits + trailer; push; `gh pr create` per template; wait for the
checks to be registered, then `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API ·
Semantics vs upstream (divergences) · Golden results · Gas | steps · Requested re-exports · Escalations · PR URL).
Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
