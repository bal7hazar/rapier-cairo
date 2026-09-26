# JA1 — joint API completion, user-facing part (AP "Joint API completion", 239 items)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (programme decisions: free parity items welcome, nothing that costs Cairo steps; JL, RJ,
JM); `docs/API_PARITY.md` (section "WP: Joint API completion" and the owner tables `FixedJoint`, `RevoluteJoint`,
`PrismaticJoint`, `RopeJoint`, `SpringJoint`, `GenericJoint`, `GenericJointBuilder`, `ImpulseJoint`,
`ImpulseJointSet`, `JointLimits`, `JointMotor`, `MotorModel`, `JointAxis`, … — the exact items); `scripts/api_parity.py`
(`METHOD_RENAMES`, `OWNER_ALIASES`, closed exclusion reasons); on `main`: `crates/rapier_dynamics2d/src/{joint.cairo,joint/**}`,
`crates/rapier2d/src/world.cairo` (the joint entry points), the joint golden replays and P3 joint scenes
(`pendulum_*`, `wheel_motor`, `rope`, `spring`). Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/dynamics/joint/**`.

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/joint.cairo` + `joint/**` (API only: typed joint views, accessors, setters, builders,
set methods), `crates/rapier2d/src/world.cairo` + `world/**` (joint accessors on `World` only), their tests,
`scripts/api_parity.py` (`METHOD_RENAMES` / `OWNER_ALIASES` entries you justify), `docs/API_PARITY.md` (regenerated),
and the snapshots that move. Forbidden: the joint solver (`solver/**`), the pipeline, geometry (QY1 works there in
parallel), `Scarb.toml`/`lib.cairo` (re-exports → Escalations).

## 3. Expected result
Upstream names and semantics: the typed joints (`FixedJoint`, `RevoluteJoint`, `PrismaticJoint`, `RopeJoint`,
`SpringJoint`) as thin views over `GenericJoint` with their accessors / setters (`local_anchor1/2`, `local_frame1/2`,
`limits`, `set_limits`, `motor`, `set_motor_velocity`, `set_motor_position`, `set_motor_model`, `angle`, …), their
builders' remaining options, `GenericJoint` / `JointLimits` / `JointMotor` / `MotorModel` members, `ImpulseJointSet`
members (`get`, `contains`, `len`, `iter`, `attached_joints`, `joints_between`, `remove_joints_attached_to_rigid_body`,
…) and the matching `World` accessors, with upstream's wake-up semantics on setters. Solver internals
(`AnyJointConstraintMut`, `AngularLimitParams`, constraint builders) stay `missing` with the reason "solver internals",
multibody items are already excluded. Every item: `ported`, `excluded` (closed reason), or `missing` with a reason.

## 4. Constraints
**No step cost:** `GenericJoint` and the other structs the solver copies keep their layout (a typed view wraps a
`GenericJoint`, nothing is added to it); every P3 joint scene, the joint golden replays and the level probes stay
identical in Sierra gas and exact Cairo steps; `gas/bytecode.size` unchanged unless a step-reachable function had to
change (then explain). `#[inline(always)]` trivial accessors. ≤ 800 lines per file (split the trait impls into
sibling files); ≤ 4 fuzz per module.

## 5. Tests
Table-driven per group (typed views round-trip to `GenericJoint`, setters + wake-ups, set methods after removals,
`World` accessors); existing tests unchanged; `python3 scripts/api_parity.py --check` passes.

## 6. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`):
`scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier_dynamics2d and rapier2d;
snapshots with `snforge test -p <crate> --tracked-resource sierra-gas > gas-<crate>.log` and
`python3 scripts/gas.py snapshot --filter <crate>::<module> --from-log gas-<crate>.log`; `python3 scripts/api_parity.py`
then `--check`; `python3 scripts/bytecode_size.py check`. Never a workspace-wide run: CI is the full gate.
Conventional commits + trailer; push; `gh pr create --base main --title "<what ships>" --body-file …`; wait for the
checks to be registered, then `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Items closed /
excluded / left with reasons · API · Gas | steps (unchanged scenes) · Deviations · Requested re-exports · Escalations ·
PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
