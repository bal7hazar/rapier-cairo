# MH1 — mass, AABB, bounding-volume and shape helpers (AP "Mass, AABB, and shape helpers", 101 items)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (programme decisions: free parity items welcome, nothing that costs Cairo steps; the MH
row); `docs/API_PARITY.md` (section "WP: Mass, AABB, and shape helpers" and the owner tables `Aabb`,
`BoundingVolume`, `BoundingSphere`, `PackedFeatureId`, `FeatureId`, `Ball`, `Cuboid`, `Capsule`, `Segment`,
`HalfSpace`, `ConvexPolygon`, `PolygonalFeature(Map)`, `Shape`, `MassProperties` — the exact items);
`scripts/api_parity.py` (`METHOD_RENAMES`, `OWNER_ALIASES`, closed exclusion reasons); on `main`:
`crates/rapier_geometry2d/src/{aabb.cairo,mass.cairo,feature_id.cairo,polygonal_feature.cairo,shape.cairo,shape/**}`.
Upstream (`UP=/home/claude/git/refs`): `$UP/parry/src/{bounding_volume,mass_properties,shape}/`.

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/{aabb.cairo,mass.cairo,feature_id.cairo,polygonal_feature.cairo,shape.cairo,shape/**}`
and a new `crates/rapier_geometry2d/src/bounding_volume.cairo` + `bounding_volume/**` declared from `aabb.cairo` or
`shape.cairo` (not from `lib.cairo`: list crate-root re-exports under Escalations), their tests, a harness family in
`tools/golden/src/**` if a helper has numeric content worth a golden check (additions only; existing vectors
byte-identical), `scripts/api_parity.py` (`METHOD_RENAMES` / `OWNER_ALIASES` entries you justify),
`docs/API_PARITY.md` (regenerated), and the snapshots that move. Forbidden: anything the step reaches beyond adding
new functions (the contact generators, dispatch, the narrow phase, solver, pipeline; the layout of `Shape`, `Aabb`,
`ConvexPolygon`, `MassProperties`), `Scarb.toml`/`lib.cairo`. JA1 works in `rapier_dynamics2d` in parallel.

## 3. Expected result
Upstream names and semantics: the `Aabb` members (`center`, `half_extents`, `extents`, `volume`, `vertices`,
`split_at_center`, `merged`, `loosened` / `tightened`, `intersects`, `contains`, `intersection`, `transform_by`, …),
the `BoundingVolume` operations on `Aabb` and `BoundingSphere` (a `BoundingSphere` type with `center`, `radius`,
`transform_by`, and the per-shape `compute_local_bounding_sphere` / `compute_bounding_sphere`), `PackedFeatureId`
and `FeatureId` helpers, the per-shape helpers of the list (`Ball`, `Cuboid`, `Capsule`, `Segment`, `HalfSpace`,
`ConvexPolygon`: `aabb`, `local_aabb`, `bounding_sphere`, `scaled`, `feature_normal`, …), `PolygonalFeature(Map)` and
`MassProperties` members. Every item: `ported`, `excluded` (closed reason), or `missing` with a one-line reason.

## 4. Constraints and efficiency
**No step cost:** the structs the step copies keep their layout; every P3, level and scene probe identical in Sierra
gas and exact Cairo steps; `gas/bytecode.size` unchanged unless a step-reachable function had to change (then
explain). Choose the cheapest exact formulation for the helpers with real arithmetic (bounding spheres, merged /
transformed AABBs) and measure them (`gas_*` probes with `rapier_testing::opaque`); `#[inline(always)]` trivial
accessors. ≤ 800 lines per file; ≤ 4 fuzz per module.

## 5. Tests
Table-driven per owner (upstream values from the source or a golden family); existing tests unchanged;
`python3 scripts/api_parity.py --check` passes.

## 6. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`):
`scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier_geometry2d (and
rapier_golden if touched); snapshots with `snforge test -p <crate> --tracked-resource sierra-gas > gas-<crate>.log`
and `python3 scripts/gas.py snapshot --filter <crate>::<module> --from-log gas-<crate>.log`; `python3
scripts/api_parity.py` then `--check`; `python3 scripts/bytecode_size.py check`. Never a workspace-wide run: CI is the
full gate. Conventional commits + trailer; push; `gh pr create --base main --title "<what ships>" --body-file …`;
wait for the checks to be registered, then `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary ·
Items closed / excluded / left with reasons · API · Gas | steps · Deviations · Requested re-exports · Escalations · PR
URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
