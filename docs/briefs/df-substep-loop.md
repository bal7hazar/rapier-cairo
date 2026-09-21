# DF — `rapier_dynamics2d::solver::island`: the substep loop over dense solver bodies

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D8, D9, §4 hot loops); `docs/research/01-rapier-analysis.md` §2.5
(exact substep order); on `main`: `crates/rapier_dynamics2d/src/solver/{body.cairo,contact.cairo,joint.cairo}`
(DC contact constraints and `SolverBody`, DE joint constraints — DC's report warns that scattering
into an immutable `Array<SolverBody>` costs O(bodies) per manifold), `crates/rapier_dynamics2d/src/rigid_body.cairo`
(DA `RigidBodyVelocity::integrate`, `RigidBodyForces::integrate`, `apply_damping`),
`crates/rapier_core/src/integration_parameters.cairo`, `crates/rapier_core/src/data/arena.cairo`;
DD (merged, PR #43): `crates/rapier_dynamics2d/src/{rigid_body_set.cairo,narrow_phase.cairo}` —
`RigidBody`, `RigidBodySet` (`iter` in ascending index order), `ContactPair` and its manifolds with
`ContactManifoldData.solver_contacts`; `src/narrow_phase/mock.cairo` is a test-only dispatcher you
may reuse in tests.
Upstream (`UP=/home/claude/git/refs`):
`$UP/rapier/src/dynamics/solver/staged_island_solver/worker.rs` (`run_worker`: per substep —
add force increment, rebuild joint rows, update + warm-start contacts, biased sweep joints then
contacts (friction skipped), clamp velocities, `integrate_linearized`, relax sweep with friction;
after substeps — restitution pass, impulse writeback, damping, `next_position`),
`$UP/rapier/src/dynamics/solver/velocity_solver.rs`, `$UP/rapier/src/dynamics/solver/solver_body.rs`.
Golden: `rapier_golden::scenes` ball_drop, ball_bounce, box_slope_stick/slide, box_stack3 —
replayable with hand-built manifolds until GG lands (the tests may construct manifolds
analytically as DC did, or run DD's `compute_contacts` with its mock dispatcher).

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/solver/island.cairo` (+ `src/solver/island/*.cairo`),
`crates/rapier_dynamics2d/src/solver/body_store.cairo`, `crates/rapier_dynamics2d/tests/substep_scenes.cairo`,
`gas/rapier_dynamics2d/solver.snap` (filters `…::solver::island`, `…::solver::body_store` only),
`gas/rapier_dynamics2d_integrationtest/substep_scenes.snap`. Modules pre-declared. Do not modify
DC's or DE's files: if their API blocks you, use it as-is and escalate.

## 3. Expected API and semantics
`SolverBodyStore`: the dense per-step body store the sweeps read and write — implement two
candidates behind one trait: (a) `Felt252Dict<Nullable<SolverBody>>` keyed by dense index (O(1)
gather/scatter), (b) `Array<SolverBody>` rebuilt on scatter (DC's current shape); `from_bodies(bodies:
@RigidBodySet or Span<RigidBody>, gravity, params) -> store + handle→index map`, `to_bodies` writing
back velocities/positions. `solve_island(params, ref store, ref contact_set, ref joint_set)`
running upstream's substep loop exactly in the order above with `num_solver_iterations`
substeps, `num_internal_pgs_iterations` and `num_internal_stabilization_iterations` honoured,
velocity clamping by `max_linear_velocity`/`max_corrective_velocity` (guarded `fixed::MAX`
sentinels as in C2), restitution pass, writeback, damping, final positions. Iteration order =
manifold order then joint order as given (D8). DEFER: islands/sleeping (whole world = one island),
CCD, parallel colouring.

## 4. Efficiency and variants
Ship the cheaper body store (expected: the dict). Bench `solve_island` for the four golden scenes
and for a 5-box stack with hand-built manifolds; report cost per step and per substep, split
between contact sweeps, joint sweeps and integration; report the store crossover if any.
Sierra gas is path-insensitive for loop-free code (`docs/PLAN.md`, wave-4 finding): give every
table in both Sierra gas and Cairo steps (`snforge test <name> --detailed-resources
--tracked-resource cairo-steps`), and decide the store on gas with steps as the tie-breaker.

## 5. Tests
Golden replays: ball_drop (contact phase included: build the ball–halfspace manifold analytically
each step from the current pose), ball_bounce, box_slope_stick/slide, box_stack3, pendulum (with
DE joints) — all within `tools/golden/README.md` tolerances over the sampled steps; energy never
increases without restitution (`fuzz_*` ≤ 4); `gas_*` per public function and candidate. ≤ 800
lines/file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); snapshot filters above; conventional commits + trailer;
push; `gh pr create` per template; `gh pr checks --watch` until green; never merge; `REPORT.md`
(Summary · API · Gas table incl. per-step split · Deviations · Deferred · Requested re-exports ·
Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
