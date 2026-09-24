# KD — kinematic bodies (position- and velocity-based) and dominance, end to end (phase 2, wave 8)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D8, D9, phase 2 table; SL sleeping: kinematic bodies are island members that
sleep when both velocities are exactly zero); `docs/adr/0001-upstream-divergences.md`; `tools/golden/README.md`
(scenes); on `main`: `crates/rapier_dynamics2d/src/{rigid_body_set.cairo,rigid_body/*}` (`RigidBodyTrait`,
`kinematic_position_based` constructor, `next_position`), `crates/rapier_dynamics2d/src/solver/{body_store.cairo,contact.cairo}`
(kinematic handling in the store; `relative_dominance` already makes the dominant side immovable in contact
rows), `crates/rapier_dynamics2d/src/narrow_phase.cairo` (`PairCollider.dominance`, effective group),
`crates/rapier2d/src/{world.cairo,pipeline.cairo,pipeline/*}`, `tools/golden/src/{scenes.rs,cairo.rs,cairo/*}`.
Upstream (`UP=/home/claude/git/refs`): `$UP/rapier/src/dynamics/rigid_body.rs` (`set_next_kinematic_position`
/ `_translation` / `_rotation` ≈ l.1204–1240, `next_position`, `set_dominance_group` ≈ l.867, `set_linvel` on
kinematic velocity-based bodies), `$UP/rapier/src/dynamics/rigid_body_components.rs` (`effective_group`
≈ l.1297), `$UP/rapier/src/pipeline/physics_pipeline/substep.rs` (`interpolate_kinematic_velocities` ≈ l.244,
and how the final kinematic position is committed).

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/{rigid_body_set.cairo,rigid_body/*}` (public API additions mirroring upstream's
names), `crates/rapier_dynamics2d/src/solver/body_store.cairo` (kinematic velocities only),
`crates/rapier2d/src/{world.cairo,pipeline.cairo,pipeline/*}`, tests (`crates/rapier2d/tests/world_step.cairo`,
`crates/rapier2d/tests/golden_scenes.cairo` + `golden_scenes/*`), the harness `tools/golden/src/**` (new scenes;
existing scenes byte-identical), `tools/golden/vectors/scenes.json` (additions only), generated fixtures,
`crates/rapier_golden/src/types.cairo` (additions only), `tools/golden/README.md`, and the snapshots that move.
Forbidden: contact/joint solver internals (dominance is already there — escalate if it is wrong), geometry,
frozen interface types, `Scarb.toml`/`lib.cairo` (re-exports → Escalations).

## 3. Expected semantics (upstream's)
Kinematic position-based: `set_next_kinematic_position/translation/rotation`; at the start of the step the
velocity is interpolated from the pose error (`(next − current) · inv_dt`, angular through the relative
rotation as upstream) so that contacts and joints see it; the body reaches exactly `next` at the end of the
step; it is not affected by forces or contacts. Kinematic velocity-based: moves with its set velocity, not
affected by contacts. Dominance: `set_dominance_group(i8)` / builder, effective group as upstream (non-dynamic
bodies are infinitely dominant), contact rows as already implemented — verify end to end. Wake-ups: a moving
kinematic body wakes the sleeping bodies it touches (SL semantics; say what upstream does and match it).

## 4. Golden and efficiency
Harness scenes: `kinematic_platform` (a position-based platform moving sideways and up carrying a dynamic box
with friction), `kinematic_pusher` (velocity-based body pushing a dynamic box), `dominance_stack` (two dynamic
boxes, the upper one more dominant — upstream semantics). Replays within the README tolerances. Report Sierra
gas and exact Cairo steps of one step of each scene, and the overhead on the 13 P3 scenes (target ≤ +1 %).
≤ 4 fuzz per module; ≤ 800 lines per file.

## 5. Tests
Golden scenes; world tests (next position reached exactly, kinematic unaffected by contacts, dominance
direction, sleeping partner woken by a moving kinematic body).

## 6. Definition of done
Harness twice → zero diff (cargo takes no lock); foreground Cairo gate (tool timeout 3600000 ms; crate-scoped
runs via `scripts/build-shims/snforge -p …`); `gas.py check` then module-filtered snapshots; conventional commits
+ trailer; push; `gh pr create` per template; wait for the checks to be registered, then `gh pr checks --watch`
until green; never merge; `REPORT.md` (Summary · API · Semantics and divergences · Golden results · Gas | steps ·
Requested re-exports · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
