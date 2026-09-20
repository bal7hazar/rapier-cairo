# DC — `rapier_dynamics2d::solver::contact`: the scalar soft-contact constraint on mock manifolds

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D8: sequential Gauss–Seidel in pair-slot order, gather/scatter per
manifold; §4 hot loops); `docs/research/01-rapier-analysis.md` §2.5 (substep order), §3.7, §9.4;
`docs/interfaces/geometry-dynamics.md` §3 and `crates/rapier_geometry2d/src/contact.cairo`
(`ContactManifold`, `SolverContact`, `NEW_CONTACT_BIT` — frozen);
`crates/rapier_core/src/integration_parameters/spring.cairo` (`SoftnessCoefficients`: `erp_inv_dt`,
`cfm_factor`, `cfm_coeff`), `crates/rapier_math/src/math_ext/vec2.cairo` (`gcross_*`).
Upstream (`UP=/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs`):
`$UP/rapier/src/dynamics/solver/contact_constraint/{contact_constraint_element.rs,
contact_with_coulomb_friction.rs,contact_constraints_set.rs}` (the scalar math: `generate`,
`update`, `warmstart`, `solve` biased/unbiased, `update_rhs_wo_bias`, `apply_restitution`,
`writeback_impulses`, `remove_bias`), `$UP/rapier/src/dynamics/solver/solver_body.rs`
(`SolverBody`/`SolverVel`), `$UP/rapier/src/dynamics/solver/staged_island_solver/worker.rs`
(`run_worker`: the order of operations inside a substep — port the *contact* part only),
`$UP/rapier/src/dynamics/solver/velocity_solver.rs`. Ignore SIMD/parallel/twist-friction/block
solver code paths.

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/solver.cairo`, `crates/rapier_dynamics2d/src/solver/contact.cairo`
(+ `src/solver/contact/*.cairo`), `crates/rapier_dynamics2d/src/solver/body.cairo`,
`crates/rapier_dynamics2d/tests/contact_solver_scenes.cairo`,
`gas/rapier_dynamics2d/solver.snap`, `gas/rapier_dynamics2d_integrationtest/contact_solver_scenes.snap`.
`lib.cairo` already declares `pub mod solver;`. Rigid bodies (DA) are developed in parallel:
define here the dense `SolverBody { position: Pose2, linvel: Vec2, angvel: Fixed, im: Vec2,
ii: Fixed, ... }` the solver reads/writes (upstream `SolverBody`), independent of DA's components;
DF will bridge them.

## 3. Expected API and semantics
`ContactConstraintElement { normal_part: ContactConstraintNormalPart, tangent_part: ContactConstraintTangentPart }`
with upstream fields (`gcross1/2`, `rhs`, `rhs_wo_bias`, `impulse`, `impulse_accumulator`,
`total_impulse`, `r` (inverse effective mass), `local_p1/2`, `dist`); `ContactConstraint` for one
manifold (`solver_vel1/2` indices, `dir1`, `im1/im2`, `cfm_factor`, `limit` (friction), `elements: [..; 2]`,
`num_elements`, `manifold_id`); `ContactConstraintsSet` over `Span<ContactManifold>` and
`Array<SolverBody>`; functions with upstream names: `generate(manifold, bodies, params, dt)`,
`update(params, bodies, manifold)` (per substep), `warmstart(ref bodies)`, `solve(ref bodies,
solve_restitution, solve_friction)` (biased then unbiased/relax pass as upstream), `remove_bias`,
`update_rhs_wo_bias`, `writeback_impulses(ref manifolds)`. Soft-contact coefficients from
`SoftnessCoefficients` (contact vs static-contact chosen per upstream rule), friction
`limit = friction · normal_impulse`, restitution pass after all substeps, `NEW_CONTACT_BIT`
gating of warm start and restitution. Gauss–Seidel order = manifold order, then element order.
DEFER: joints, islands, block solver, twist friction, `tangent_velocity` (keep the field, treat as
zero).

## 4. Efficiency and variants
This is the #1 gas consumer (D8): gather the two bodies' velocities once per manifold, solve all
elements, scatter once; hoist `cfm_factor`, `erp_inv_dt`, `inv_dt`; zero divisions inside `solve`
(all `r` precomputed in `generate`); fused `J·v` rows via `fixed::wide`. Bench `solve` per manifold
per substep as (a) direct port (per-element gather/scatter) and (b) gathered variant; report the
cost per manifold and per element for 1 and 2 points.

## 5. Tests
Scenes on mock manifolds (build manifolds by hand, no narrow phase): (1) ball resting on ground
under gravity for 60 steps × 4 substeps — the normal impulse converges to `m·g·dt` and the
velocity to 0 within tolerance; (2) ball bouncing with restitution 1 → velocity reverses; (3) box
on slope with friction 0.5: sticks at 20°, slides at 45° (compare with `rapier_golden::scenes`
box_slope samples within their tolerance — the manifold for a box on a plane is 2 points at the
bottom corners with `dist` from geometry, which you construct analytically); (4) stack of two
balls: impulses sum correctly. Table-driven, ≤ 800 lines per file, `fuzz_*` ≤ 4 (e.g. energy
never increases without restitution), `gas_*` for every function and candidate.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); `python3 scripts/gas.py snapshot --filter
rapier_dynamics2d::solver`, `… rapier_dynamics2d_integrationtest::contact_solver_scenes`; conventional
commits + trailer; push; `gh pr create` per template; `gh pr checks --watch` until green; never
merge; `REPORT.md` (Summary · API · Gas table incl. cost per manifold/element · Deviations ·
Deferred · Requested re-exports · Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
