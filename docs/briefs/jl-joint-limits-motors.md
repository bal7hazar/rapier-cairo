# JL — joint limits and motors (phase 2, wave 7)

## 1. Read first
`AGENTS.md` (§7 gas cost model; OS/OJ lessons: bit-identical refactors, realistic inputs, exact steps);
`docs/PLAN.md` (DE: limits/motors stored but not solved; OJ: current joint rows; phase 2 table);
`tools/golden/README.md` (scenes, pendulum); on `main`: `crates/rapier_dynamics2d/src/joint.cairo`
(`JointLimits`, `JointMotor`, `MotorModel`, `limit_axes` / `motor_axes` masks — stored today, unsolved),
`joint/builders.cairo` (public builders), `solver/joint.cairo` + `solver/joint/{helper,row}.cairo` (OJ's
two-/three-row kernels, Gram–Schmidt), `solver/island/sweeps.cairo` (joint stages), `rapier_core` spring
coefficients (`IntegrationParameters`, `SpringCoefficients`), glam.cairo's `fixed::trig::TrigTrait::atan2`
(fixed 0.3.0, used for the revolute angle). Upstream (`UP=/home/claude/git/refs`):
`$UP/rapier/src/dynamics/joint/{generic_joint.rs,revolute_joint.rs,prismatic_joint.rs,motor_model.rs}`
(`set_limits`, `set_motor_velocity/position/max_force/model`, `JointLimits`, `JointMotor`),
`$UP/rapier/src/dynamics/solver/joint_constraint/{joint_velocity_constraint.rs,joint_constraint_builder.rs,joint_constraint_helper.rs}`
(`limit_linear`, `limit_angular`, `motor_linear`, `motor_angular`: row construction, bounds, the motor spring
model, how limit rows are active only past the bound, impulse clamping).

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/joint.cairo`, `joint/*.cairo` (public builder setters mirroring upstream names),
`crates/rapier_dynamics2d/src/solver/joint.cairo`, `solver/joint/**`, `solver/island/sweeps.cairo` (joint
functions only), `crates/rapier_dynamics2d/tests/joint_scenes.cairo`, the Rust harness `tools/golden/src/scenes.rs`
(+ `tools/golden/vectors/scenes.json` additions only, `crates/rapier_golden/src/types.cairo` new fields/types only,
`crates/rapier_golden/src/generated/scenes.cairo` generated, `tools/golden/README.md` scenes section),
`crates/rapier2d/tests/golden_scenes.cairo` + `golden_scenes/*.cairo` (new scene replays), and the snapshots that
move. Forbidden: contact solver, narrow/broad phase, `rapier2d` sources other than tests (a builder re-export in
the prelude goes to Escalations), frozen interface types, `Scarb.toml`/`lib.cairo`.

## 3. Expected semantics
Upstream's, exactly: revolute angular limit (the relative angle from the two frames via `atan2`, wrapped as
upstream does), prismatic linear limit along the free axis, velocity and position motors (target velocity,
target position with stiffness/damping, `max_force` clamping, `MotorModel::{AccelerationBased, ForceBased}`),
limit rows only active beyond the bound (as upstream builds them), impulses persisted for warm start like
the existing rows, both stages (biased / relaxed) as upstream. Existing joints without limits/motors must stay
**bit-identical** (all current joint, substep and golden pendulum tests unchanged, the P3 pendulum budgets not
worse). Document every divergence (e.g. `atan2` precision vs f64).

## 4. Golden and efficiency
Harness scenes (upstream, sleeping off like the others): `pendulum_limited` (revolute limit hit and held),
`wheel_motor` (revolute velocity motor driving a wheel on the ground or a free body), `slider_limited`
(prismatic limit), `servo` (revolute position motor converging). Replays within the README tolerances.
Report Sierra gas and exact Cairo steps per joint per step with/without an active limit/motor.
Losers under `mod alternatives`; ≤ 4 fuzz per module; ≤ 800 lines per file.

## 5. Tests
The golden scenes; table-driven unit tests per row kind (inactive limit, active lower/upper, motor velocity,
motor position, max-force clamp); unchanged existing tests.

## 6. Definition of done
Harness twice → zero diff (cargo takes no lock); foreground Cairo gate (tool timeout 3600000 ms; crate-scoped
runs while iterating); `gas.py check` then module-filtered snapshots; conventional commits + trailer; push;
`gh pr create` per template; wait for the checks to be registered, then `gh pr checks --watch` until green;
never merge; `REPORT.md` (Summary · API · Semantics and divergences · Golden results · Gas | steps · Requested
re-exports · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
