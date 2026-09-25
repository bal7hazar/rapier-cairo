# SE — sensors: intersection pairs and intersection events (parity with rapier-rs)

## 1. Read first
`AGENTS.md` (§7 gas cost model); `docs/PLAN.md` (DD deferred "sensors' intersection events (keep the filter)";
D7, D8, D9; SL sleeping; EV events); `docs/adr/0001-upstream-divergences.md`; on `main`:
`crates/rapier_dynamics2d/src/{narrow_phase.cairo,narrow_phase/*,events.cairo,collider/*}` (sensor flag, the
filter that skips sensors today, `CollisionEvent` with its flags), `crates/rapier_core/src/collider/events.cairo`
(`ActiveEvents`, `CollisionEventFlags` incl. `SENSOR`), `crates/rapier_geometry2d/src/{dispatch.cairo,shape.cairo,point.cairo}`,
`crates/rapier2d/src/{world.cairo,pipeline.cairo,pipeline/*,queries.cairo}`. Upstream (`UP=/home/claude/git/refs`):
`$UP/rapier/src/geometry/narrow_phase/` (intersection graph, `compute_intersections`, `IntersectionPair`, events,
`intersection_pair`, `intersection_pairs_with`, `intersection_pairs`), `$UP/rapier/src/geometry/contact_pair.rs`,
`$UP/parry/src/query/intersection_test/` and `$UP/parry/src/query/default_query_dispatcher.rs` (`intersection_test`
per shape pair), `$UP/rapier/src/pipeline/event_handler.rs`.

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/{narrow_phase.cairo,narrow_phase/*,events.cairo}`, `crates/rapier_geometry2d/src/dispatch.cairo`
+ `dispatch/*` (an `intersection_test` table next to the contact tables), `crates/rapier2d/src/{world.cairo,pipeline.cairo,pipeline/*}`,
tests (`crates/rapier_dynamics2d/tests/narrow_phase_scenes.cairo`, `crates/rapier2d/tests/world_step.cairo`, a new
`crates/rapier_geometry2d/tests/intersection_golden.cairo`, golden scene files), the harness `tools/golden/src/**`
(a new `intersection_tests` family and a `sensor_trigger` scene; existing vectors byte-identical), vectors/generated
fixtures/types (additions only), `tools/golden/README.md`, and the snapshots that move. Forbidden: solver, joints,
frozen interface types, `Scarb.toml`/`lib.cairo` (re-exports → Escalations).

## 3. Expected semantics (upstream's)
Pairs involving a sensor (and not rejected by the existing filters: collision groups, active collision types,
disabled colliders) become **intersection pairs**: a boolean `intersecting` updated each step from
`intersection_test(pos12, shape1, shape2)` (exact, no prediction distance), never reaching the solver.
`CollisionEvent::Started/Stopped` with the `SENSOR` flag when `intersecting` changes, if either collider has
`COLLISION_EVENTS` (upstream's rule), in the deterministic event order (ascending pair). `World` queries mirroring
upstream: `intersection_pair(c1, c2) -> Option<bool>`, `intersection_pairs_with(c) -> Array<…>`. Sleeping: sensors
attached to sleeping bodies follow upstream (read `compute_intersections`: which pairs are updated). Removing a
collider emits `Stopped` with `REMOVED` as upstream. `intersection_test` per shape pair: implement with the
cheapest exact method per pair (analytic distance ≤ 0 or SAT without clipping) — measure against deriving it from
the contact generators; every pair of the closed shape set that has a contact generator must be supported.

## 4. Golden and efficiency
Harness: `intersection_tests` family (every supported pair × {separated, touching, overlapping, contained}) and a
`sensor_trigger` scene (a body falling through a sensor: Started/Stopped steps exact). Report Sierra gas and exact
Cairo steps per `intersection_test` kind and per sensor pair per step; P3 scenes ≤ +1 % (no sensors in them). ≤ 4
fuzz per module; ≤ 800 lines per file.

## 5. Tests
Golden family and scene; world tests (events with/without `COLLISION_EVENTS`, groups exclusion, sensor on a sleeping
body, removal emits `Stopped | REMOVED`).

## 6. Definition of done
Harness twice → zero diff (cargo takes no lock); foreground Cairo gate (tool timeout 3600000 ms; crate-scoped runs
via `scripts/build-shims/snforge -p …`); `gas.py check` then module-filtered snapshots; conventional commits +
trailer; push; `gh pr create --base main --title "<what ships>" --body-file …` per template; wait for the checks to
be registered, then `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API · Semantics and
divergences · Golden results · Gas | steps · Requested re-exports · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
