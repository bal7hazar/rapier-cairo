# RJ — coupled limits/motors, rope and spring joints (phase 2, wave 8)

## 1. Read first
`AGENTS.md` (§7 gas cost model, incl. the "gas wallet" found by JM); `docs/PLAN.md` (DE, OJ, JL, JM findings);
`docs/adr/0001-upstream-divergences.md`; on `main`: `crates/rapier_dynamics2d/src/joint.cairo` + `joint/*`
(`GenericJoint`, `coupled_axes` mask stored by DE, builders), `crates/rapier_dynamics2d/src/solver/joint.cairo`
+ `solver/joint/{step,bounded,kernels,row,helper}.cairo` (JL rows, JM specialisation `Plain` / `Controlled` /
`Legacy` and the gas wallet), `solver/island/sweeps.cairo`, `tools/golden/src/{scenes.rs,cairo.rs,cairo/scenes.rs}`
(JL added joint limits/motors to the scene exporter), `crates/rapier2d/tests/golden_scenes/joint_controls.cairo`.
Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/dynamics/joint/{rope_joint.rs,spring_joint.rs,generic_joint.rs}`
(`RopeJoint::new(max_dist)` = empty locked axes, `coupled_axes(LIN_AXES)`, limit `[0, max_dist]` on `LinX`;
`SpringJoint::new(rest_length, stiffness, damping)` = coupled `LIN_AXES`, position motor on `LinX`,
`MotorModel::ForceBased`), `$UP/rapier/src/dynamics/solver/joint_constraint/joint_constraint_helper.rs`
(`limit_linear_coupled` ≈ l.210, `motor_linear_coupled` ≈ l.333, `limit_angular_coupled` ≈ l.725) and
`joint_velocity_constraint.rs` (where coupled rows are emitted, ≈ l.239 / 321 / 339).

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/joint.cairo`, `joint/*.cairo` (rope/spring builders mirroring upstream names),
`crates/rapier_dynamics2d/src/solver/joint.cairo`, `solver/joint/**`, `solver/island/sweeps.cairo` (joint
functions only), `crates/rapier_dynamics2d/tests/joint_scenes.cairo`, the harness `tools/golden/src/**` (rope/spring
scenes; existing scenes byte-identical), `tools/golden/vectors/scenes.json` (additions only), generated fixtures,
`crates/rapier_golden/src/types.cairo` (additions only), `tools/golden/README.md`,
`crates/rapier2d/tests/golden_scenes.cairo` + `golden_scenes/*`, `crates/rapier2d/tests/gas_scenes.cairo` (new
probes + twins only), and the snapshots that move. Forbidden: contact solver and `solver/island.cairo` (lot KD
is fixing kinematic contact endpoints there in parallel), geometry, `Scarb.toml`/`lib.cairo` (re-exports →
Escalations).

## 3. Expected semantics (upstream's)
Coupled linear limit (distance between anchors limited to `[min, max]`, one row along the anchor direction,
active only past the bound as upstream builds it), coupled linear motor (spring toward a rest length with
stiffness/damping, both motor models), coupled angular limit if upstream emits it in 2D (check; implement if
yes). `RopeJointBuilder::new(max_dist)` and `SpringJointBuilder::new(rest_length, stiffness, damping)` with
upstream's setters (`local_anchor1/2`, `contacts_enabled`, `spring_model`, …). Zero-length anchor separation:
follow upstream's handling (direction fallback). Existing joints bit-identical; JM's specialisation extended
(a coupled joint is a new `Controlled` kind or its own) without making `Plain` dearer.

## 4. Golden and efficiency
Harness scenes: `rope_pendulum` (a ball on a slack rope that becomes taut), `spring_mass` (a body oscillating on a
damped spring, both motor models if cheap to add). Replays within the README tolerances. Report Sierra gas and
exact Cairo steps per joint per step (rope slack / taut, spring); add P3 probes `gas_step_rope`, `gas_step_spring`
with `steps_step_*` twins (+10 % ceilings). No regression on existing pendulum/joint probes. Losers under
`mod alternatives`; ≤ 4 fuzz per module; ≤ 800 lines per file.

## 5. Tests
Golden scenes; table-driven row tests (slack rope emits no row, taut rope clamps, spring force sign/magnitude);
existing tests unchanged.

## 6. Definition of done
Harness twice → zero diff (cargo takes no lock); foreground Cairo gate (tool timeout 3600000 ms; crate-scoped runs
via `scripts/build-shims/snforge -p …`); `gas.py check` then module-filtered snapshots; if KD merged meanwhile,
rebase and regenerate; conventional commits + trailer; push; `gh pr create` per template; wait for the checks to be
registered, then `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API · Semantics and
divergences · Golden results · Gas | steps · Requested re-exports · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
