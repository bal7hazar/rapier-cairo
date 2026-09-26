# QY1 — parry shape-pair queries for the closed 2D shape set (AP "Query completion", part 1)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (programme decisions: parity items that cost no Cairo steps are welcome; the QY row);
`docs/API_PARITY.md` (section "WP: Query completion" and the owner tables of the `parry::query` module: `Aabb`,
`ClosestPoints`, `Contact`, `ContactManifold`, `ContactManifoldData`, `Ball`, `Capsule`, … — the exact items you
close); `scripts/api_parity.py` (how an item is judged `ported`; `METHOD_RENAMES`, `OWNER_ALIASES`, the closed
exclusion reasons); `docs/adr/0001-upstream-divergences.md` (entries 14, 17, 24: analytic kernels instead of GJK / EPA);
on `main`: `crates/rapier_geometry2d/src/{point.cairo,point/**,ray.cairo,ray/**,closest_points.cairo,contact.cairo,manifold.cairo,aabb.cairo,sat.cairo,dispatch.cairo,dispatch/**,shape.cairo,shape/**}`
and the pre-declared `crates/rapier_geometry2d/src/query.cairo`. Upstream (`UP=/home/claude/git/refs`):
`$UP/parry/src/query/{distance,closest_points,contact,point,ray,intersection_test}/`, `$UP/parry/src/query/default_query_dispatcher.rs`.

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/query.cairo` + `query/**` (new), `crates/rapier_geometry2d/src/{point.cairo,point/**,ray.cairo,ray/**,closest_points.cairo,aabb.cairo}`
(additions only: the per-shape `PointQuery` / `RayCast` variants that are missing, the `Aabb` ones),
`crates/rapier_geometry2d/src/{contact.cairo,manifold.cairo}` (manifold utilities of the list, no change to what the
step uses), their tests, a new golden family in the harness `tools/golden/src/**` (`shape_queries`; existing vectors
byte-identical) with its vectors / generated fixtures / types (additions only), `tools/golden/README.md`,
`scripts/api_parity.py` only for `METHOD_RENAMES` / `OWNER_ALIASES` entries you justify, `docs/API_PARITY.md`
(regenerated), and the snapshots that move. Forbidden: anything the step reaches (contact generators, dispatch tables
used by the narrow phase, the narrow phase, the solver, the pipeline, `World`: BT4 works in `rapier2d` in parallel —
list `World`-level query wrappers under Escalations), `Scarb.toml`/`lib.cairo`.

## 3. Expected result
Parry's top-level queries for every pair of the closed set (ball, cuboid, capsule, segment, half-space, convex
polygon), upstream names and semantics, in `rapier_geometry2d::query`: `distance(pos1, g1, pos2, g2) -> Option<Fixed>`,
`closest_points(pos1, g1, pos2, g2, max_dist) -> ClosestPoints` (`Intersecting` / `WithinMargin(p1, p2)` /
`Disjoint`), `contact(pos1, g1, pos2, g2, prediction) -> Option<Contact>` (`point1`, `point2`, `normal1`, `normal2`,
`dist`), with `intersection_test` re-used from `dispatch`; `None` where upstream's dispatcher has no algorithm for the
pair (half-space–half-space, …) exactly as upstream. The missing `PointQuery` / `RayCast` members (e.g.
`project_point_and_get_feature`, `contains_point`, `distance_to_point`, `cast_ray_and_get_normal` where missing) and
the `Aabb` point / ray queries. The `ContactManifold` / `ContactManifoldData` utilities of the list where they are
meaningful without Parry's workspaces. Composite-shape and shape-cast items stay `missing` (lots SH2 / CC); GJK / EPA
internals are excluded through the closed reason. Every item: `ported`, `excluded` (closed reason), or `missing` with
a one-line reason in REPORT.md.

## 4. Golden and efficiency
Harness family `shape_queries` from parry2d-f64 0.30.2 (the vendored copy): every supported pair × {separated,
touching, overlapping, contained} × {distance, closest_points (with and without margin), contact (prediction 0 and
0.1)}, strict or within documented bands (analytic kernels vs GJK: say which cases differ and by how much). Choose the
cheapest exact formulation per pair and measure it (Sierra gas and exact Cairo steps per kernel, `gas_*` probes with
`rapier_testing::opaque`; losers under `#[cfg(test)] mod alternatives`). Nothing on the step's path changes: every P3,
level and scene probe identical, `gas/bytecode.size` unchanged (the step does not reach the new code).

## 5. Tests
The golden family; table-driven unit tests per kernel family; ≤ 800 lines per file; ≤ 4 fuzz per module.

## 6. Definition of done
Harness twice → zero diff (cargo takes no lock). Crate-scoped local gate only (AGENTS §6; foreground, tool timeout
3600000 ms, through `scripts/build-shims/`): `scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` /
`snforge test -p` on rapier_geometry2d and rapier_golden; snapshots with `snforge test -p <crate> --tracked-resource
sierra-gas > gas-<crate>.log` and `python3 scripts/gas.py snapshot --filter <crate>::<module> --from-log
gas-<crate>.log`; `python3 scripts/api_parity.py` then `--check`; `python3 scripts/bytecode_size.py check` (must pass
unchanged). Never a workspace-wide run: CI is the full gate. Conventional commits + trailer; push; `gh pr create --base
main --title "<what ships>" --body-file …`; wait for the checks to be registered, then `gh pr checks --watch` until
green; never merge; `REPORT.md` (Summary · Items closed / excluded / left with reasons · API · Golden results and
bands · Gas | steps per kernel · Deviations · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
