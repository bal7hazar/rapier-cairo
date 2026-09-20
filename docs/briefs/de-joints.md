# DE — `rapier_dynamics2d::{joint, solver::joint}`: fixed, revolute and prismatic joints and their solver rows

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D4: rigid joints special-cased because upstream's `cfm ≈ 1.5e-9`
underflows; phase-1 scope = fixed, revolute, prismatic without limits/motors);
`docs/research/01-rapier-analysis.md` §4.8, §9.4 (joint rows rebuilt every substep, Gram–Schmidt);
on `main`: `crates/rapier_core/src/integration_parameters/spring.cairo` (`joint_softness_coefficients`),
`crates/rapier_dynamics2d/src/solver/{body.cairo,contact.cairo}` (DC's `SolverBody`, the
generate/warmstart/solve/writeback shape to mirror), `crates/rapier_dynamics2d/src/rigid_body.cairo`
(DA), `crates/rapier_math/src/{rot2,pose2}.cairo`. Upstream
(`UP=/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs`):
`$UP/rapier/src/dynamics/joint/{generic_joint.rs,fixed_joint.rs,revolute_joint.rs,prismatic_joint.rs,impulse_joint/}`
(`GenericJoint`, `JointAxesMask`, `JointLimits`/`JointMotor` data kept but unused), and
`$UP/rapier/src/dynamics/solver/joint_constraint/{joint_constraint_helper.rs,joint_generic_constraint.rs,joint_constraint_builder.rs}`
(2D: `lock_linear`, `lock_angular`, `finalize` with Gram–Schmidt orthogonalisation, `solve_generic`,
`warmstart`, `writeback`, `remove_bias`). Golden: `rapier_golden::scenes` pendulum (revolute joint,
22 samples).

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/joint.cairo` (+ `src/joint/*.cairo`),
`crates/rapier_dynamics2d/src/solver/joint.cairo` (+ `src/solver/joint/*.cairo`),
`crates/rapier_dynamics2d/tests/joint_scenes.cairo`, `gas/rapier_dynamics2d/{joint,solver}.snap`
(for `solver.snap` regenerate with `--filter rapier_dynamics2d::solver::joint` only),
`gas/rapier_dynamics2d_integrationtest/joint_scenes.snap`. Modules pre-declared. Do not touch DC's
files.

## 3. Expected API and semantics
`GenericJoint { local_frame1: Pose2, local_frame2: Pose2, locked_axes: JointAxesMask, coupled_axes,
limit_axes, motor_axes, limits: [JointLimits; 3], motors: [JointMotor; 3], contacts_enabled: bool,
enabled: JointEnabled }` (2D: axes X, Y, AngX), `GenericJointBuilder` and the typed builders
`FixedJointBuilder`, `RevoluteJointBuilder` (`local_anchor1/2`), `PrismaticJointBuilder`
(`axis`, `local_anchor1/2`) producing `GenericJoint`; `ImpulseJoint { body1: Handle, body2: Handle,
data: GenericJoint, impulses: [Fixed; 3] }`, `ImpulseJointSet` (Arena, ascending order).
Solver: `JointConstraintHelper` (2D) with `lock_linear`, `lock_angular`, `finalize`
(Gram–Schmidt over the rows, upstream's exact order), `JointGenericConstraint`-style rows
{`lin_jac`, `ang_jac1/2`, `inv_lhs`, `rhs`, `rhs_wo_bias`, `impulse`, `cfm_coeff`, `cfm_gain`,
`erp_inv_dt`}; `generate(joint, bodies, params)` per substep as upstream (rows rebuilt every
substep), `warmstart`, `solve` (biased/relaxed), `remove_bias`, `writeback_impulses`. Softness
from `joint_softness_coefficients`: apply D4 — when `cfm_coeff` underflows to < 8 ulp treat the
joint as rigid (`cfm = 0`, `erp` as computed) and document the threshold. DEFER: limits, motors,
rope/spring joints, multibody, `JointAxesMask` coupling beyond locks.

## 4. Efficiency and variants
Rows are rebuilt every substep: fused Jacobian dots, no division in `solve` (`inv_lhs`
precomputed in `finalize`). Bench `generate + solve` per joint type per substep; compare (a)
upstream's generic 3-row path and (b) specialised 2-row revolute / 1+1-row prismatic
constructions; ship the winner, keep the loser under `mod alternatives` if results are within
1 ulp.

## 5. Tests
Golden: pendulum scene from `rapier_golden::scenes` replayed with DA integration + DC-style
substep loop (mock, in the test file) within the README tolerance over the 22 samples; fixed
joint holds two bodies rigid under gravity; prismatic constrains motion to its axis; revolute
conserves the anchor distance; `fuzz_*` ≤ 4; `gas_*` per public function and candidate. ≤ 800
lines/file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); snapshot filters above; conventional commits + trailer;
push; `gh pr create` per template; `gh pr checks --watch` until green; never merge; `REPORT.md`
(Summary · API · Gas table · Deviations (incl. the rigid-joint threshold) · Deferred · Requested
re-exports · Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
