# QY2 — world-level scene queries: `QueryPipeline` and `QueryFilter` completion (AP "Query completion", part 2)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (the QY row: QY1 done); `docs/API_PARITY.md` (the owner tables `QueryPipeline`,
`QueryFilter`, `QueryFilterFlags`, `PhysicsWorld` — the exact items); `scripts/api_parity.py` (`OWNER_ALIASES`:
`QueryPipeline` maps to `World` + `queries`; `METHOD_RENAMES`; closed exclusion reasons); on `main`:
`crates/rapier2d/src/{queries.cairo,queries/**,world.cairo}` (the queries already ported: `cast_ray`,
`cast_ray_and_get_normal`, `intersect_ray`, `intersect_point`, `project_point`, `intersect_aabb`, `QueryFilter`),
`crates/rapier_geometry2d/src/query.cairo` (QY1: `intersection_test`, `distance`, `contact`, `closest_points`) and
`crates/rapier_geometry2d/src/point/**` (`project_point_and_get_feature`). Upstream (`UP=/home/claude/git/refs`):
`$UP/rapier/src/pipeline/query_pipeline/**`, `$UP/rapier/src/pipeline/physics_world.rs`.

## 2. Scope (file allowlist)
`crates/rapier2d/src/{queries.cairo,queries/**,world.cairo,world/**}` (query entry points only), their tests,
`scripts/api_parity.py` (`METHOD_RENAMES` / `OWNER_ALIASES` entries you justify), `docs/API_PARITY.md` (regenerated),
and the snapshots that move. Forbidden: the step (pipeline, narrow phase, solver), geometry (MH1 works there in
parallel: use QY1's `rapier_geometry2d::query` as it is), `Scarb.toml`/`lib.cairo`.

## 3. Expected result
Upstream names and semantics: `intersect_shape(shape_pos, shape, filter)` (the colliders whose shape intersects the
given shape, through `rapier_geometry2d::query::intersection_test`, broad-phase-pruned like the other queries),
`project_point_and_get_feature`, `intersect_aabb_conservative`, `with_filter` / a `QueryPipeline` view (a value that
bundles `@World` and a `QueryFilter`, or a documented `OWNER_ALIASES` mapping if a view adds nothing), `QueryFilterFlags`
(its constants and `test`), `QueryFilter::exclude_solids`, `From<InteractionGroups>` / `From<QueryFilterFlags>` for
`QueryFilter`. `QueryFilter::predicate` is a closure: excluded as `dyn hooks`. `cast_shape` / `cast_shape_nonlinear`
stay `missing` (lot CC). Results ordered deterministically (ascending collider handle) and documented.

## 4. Constraints
No step cost: nothing on the step's path changes (P3, level probes and `gas/bytecode.size` identical). Measure each new
query on a 20-collider world (`gas_*` probes with `rapier_testing::opaque`, exact Cairo steps). ≤ 800 lines per file;
≤ 4 fuzz per module.

## 5. Tests
Table-driven per query (filters, sensors, disabled colliders, removed colliders, empty world); results checked against
brute force over all colliders; existing tests unchanged; `python3 scripts/api_parity.py --check` passes.

## 6. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`):
`scarb fmt --workspace`; `scarb lint -p rapier2d --deny-warnings`, `scarb build -p rapier2d`, `snforge test -p
rapier2d`; snapshots with `snforge test -p rapier2d --tracked-resource sierra-gas > gas-rapier2d.log` and
`python3 scripts/gas.py snapshot --filter rapier2d::<module> --from-log gas-rapier2d.log` (and the integration-test
modules you touch); `python3 scripts/api_parity.py` then `--check`; `python3 scripts/bytecode_size.py check`. Never a
workspace-wide run: CI is the full gate. Conventional commits + trailer; push; `gh pr create --base main --title
"<what ships>" --body-file …`; wait for the checks to be registered, then `gh pr checks --watch` until green; never
merge; `REPORT.md` (Summary · Items closed / excluded / left with reasons · API · Gas | steps · Deviations · Requested
re-exports · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
