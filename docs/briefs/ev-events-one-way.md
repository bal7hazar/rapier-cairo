# EV — contact-force events and one-way platforms (phase 2, wave 8)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D9, D10 — no hooks: a built-in flag instead; DD events, SL, KD findings);
`docs/adr/0001-upstream-divergences.md`; on `main`: `crates/rapier_core/src/collider/events.cairo`
(`ActiveEvents`, `CONTACT_FORCE_EVENTS` already defined), `crates/rapier_dynamics2d/src/{events.cairo,narrow_phase.cairo,collider/*}`
(`CollisionEvent`, `PairCollider`, collider components and builder), `crates/rapier_dynamics2d/src/solver/contact*`
(impulses written back per point), `crates/rapier2d/src/{world.cairo,pipeline.cairo,pipeline/*}`,
`tools/golden/src/{scenes.rs,cairo.rs,cairo/*}`. Upstream (`UP=/home/claude/git/refs`):
`$UP/rapier/src/geometry/mod.rs` (`ContactForceEvent` ≈ l.198: `collider1/2`, `total_force`,
`total_force_magnitude`, `max_force_direction`, `max_force_magnitude`, `started`, `from_contact_pair(dt, pair, …)`),
`$UP/rapier/src/pipeline/event_handler.rs` (`handle_contact_force_event`, threshold semantics),
`$UP/rapier/src/geometry/collider.rs` (`contact_force_event_threshold`), where the pipeline emits them after
the solver; `$UP/rapier/examples2d/one_way_platforms2.rs` and `$UP/rapier/crates/rapier2d/tests/issue_752_oneway_platform_normal.rs`
(upstream's one-way platforms through `PhysicsHooks::modify_solver_contacts`).

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/{events.cairo,narrow_phase.cairo,narrow_phase/*,collider/*}` (new component field /
builder setters mirroring upstream names), `crates/rapier2d/src/{world.cairo,pipeline.cairo,pipeline/*}`, tests
(`crates/rapier2d/tests/world_step.cairo`, `golden_scenes.cairo` + `golden_scenes/*`, narrow-phase scenes), the
harness `tools/golden/src/**` (new scenes; existing byte-identical — lot RJ also adds scenes there in parallel:
rebase and regenerate if it merges first), `tools/golden/vectors/scenes.json` (additions), generated fixtures,
`crates/rapier_golden/src/types.cairo` (additions), `tools/golden/README.md`, and the snapshots that move.
Forbidden: solver internals (read impulses through the manifold data), geometry, joint files, `Scarb.toml`/`lib.cairo`
(re-exports → Escalations). `World::step`'s signature stays: add `WorldTrait::step_with_force_events(ref self) ->
(Array<CollisionEvent>, Array<ContactForceEvent>)` and make `step` return the first array of the same computation
without paying for force events when no collider enables them.

## 3. Expected semantics
**Contact-force events (upstream's):** for each contact pair where one collider has `CONTACT_FORCE_EVENTS`, after
the solver: total force = Σ impulses / dt along the normal(s) (and tangent? follow `from_contact_pair` exactly),
magnitude compared with the colliders' `contact_force_event_threshold` (upstream's rule when both set one), event
fields as upstream, deterministic order (ascending pair). **One-way platforms (port feature, no hooks):** a collider
flag `one_way(local_up: Vec2, allowed_angle?)` — mirror the upstream example's `modify_solver_contacts` rule: solver
contacts are removed when the contact normal (from the platform to the other body) is not within the allowed cone
around the platform's world up, and **a body that started passing through keeps passing** as the example/issue-752
test do (read the example: it tracks the direction at contact start). Document the exact rule in the ADR entry you
propose (Escalations). No cost when no collider uses the flag.

## 4. Golden and efficiency
Harness scenes (upstream with the example's hook for one-way): `one_way_jump` (a body jumping up through a platform
and landing on it), `force_event_drop` (a box dropped on a sensor-less ground with a force threshold: the step and
magnitude of the events). Replays within tolerances; events compared exactly on steps and within tolerance on
magnitudes. Report the overhead on the 13 P3 scenes (target ≤ +1 %, none when unused) and the cost of a step with
the features on (gas | exact steps). ≤ 4 fuzz per module; ≤ 800 lines per file.

## 5. Tests
Golden scenes; world tests (no events below threshold, both colliders' thresholds, one-way from above collides /
from below passes / sideways per the cone).

## 6. Definition of done
Harness twice → zero diff (cargo takes no lock); foreground Cairo gate (tool timeout 3600000 ms; crate-scoped runs
via `scripts/build-shims/snforge -p …` / `scripts/build-shims/scarb`); `gas.py check` then module-filtered snapshots;
conventional commits + trailer; push; `gh pr create` per template; wait for the checks to be registered, then
`gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API · Semantics and divergences (proposed ADR
entries) · Golden results · Gas | steps · Requested re-exports · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
