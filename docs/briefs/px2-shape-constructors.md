# PX2 — `Shape` constructors (upstream `SharedShape::*`), a free parity item

## 1. Read first
`AGENTS.md`; PX1's report (#199): `SharedShape` maps to `Shape`, but its constructors stay `missing` because Cairo's
`Shape` has no constructors of those names; `docs/API_PARITY.md` (the `SharedShape` table: `ball`, `capsule`,
`capsule_x`, `capsule_y`, `cuboid`, `halfspace`, `segment`, `triangle`, `round_cuboid`, `round_triangle`,
`convex_hull`, `round_convex_hull`, `convex_polyline`, `round_convex_polyline`, `convex_polyline_unmodified`,
`polyline`, `heightfield`, `compound`, `new`, `make_mut`, …); `scripts/api_parity.py` (`OWNER_ALIASES`,
`METHOD_RENAMES`, `MISSING_REASONS`); on `main`: `crates/rapier_geometry2d/src/{shape.cairo,shape/**}` and
`crates/rapier_dynamics2d/src/collider/builder.cairo` (the `ColliderBuilder` constructors build the same shapes — reuse
their validation). Upstream (`UP=/home/claude/git/refs`): `$UP/parry/src/shape/shared_shape.rs`.

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/{shape.cairo,shape/**}` (constructors only: `ShapeTrait::{ball, cuboid, capsule,
capsule_x, capsule_y, segment, halfspace, triangle, round_cuboid, round_triangle, convex_hull, round_convex_hull,
convex_polyline, round_convex_polyline, convex_polyline_unmodified, polyline, heightfield, compound}` returning `Shape`
or `Option<Shape>` as upstream returns `SharedShape` or `Option<SharedShape>`), their tests, `scripts/api_parity.py`
(remove the `MISSING_REASONS` entries you close; `new` / `make_mut` → a `METHOD_RENAMES` only if the semantics match,
otherwise keep their reasons), `docs/API_PARITY.md` (regenerated), and the snapshots that move. Forbidden: anything the
step reaches, `ColliderBuilder` (KC1 and future lots), `Scarb.toml`/`lib.cairo`.

## 3. Expected result
Each constructor builds exactly the `Shape` the matching `ColliderBuilder` constructor builds (same validation, same
`None` cases), upstream argument order. `convex_decomposition*`, `voxelized_*`, `round_convex_decomposition*` stay
`missing` (V-HACD / voxels: no port). Report raw and in-scope parity before / after.

## 4. Constraints
No step cost: P3, level and game-shaped probes identical; `gas/bytecode.size` unchanged (`program.*` included). ≤ 800
lines per file.

## 5. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, through `scripts/build-shims/`): `scarb fmt --workspace`; `scarb lint
-p` / `scarb build -p` / `snforge test -p` on rapier_geometry2d; snapshots with `--from-log`; `python3
scripts/api_parity.py --self-test`, then generate and `--check`; `python3 scripts/bytecode_size.py check`. Never a
workspace-wide run. Rebase on `origin/main` before the PR. Conventional commits + trailer; push; `gh pr create --base main
--title "<what ships>" --body-file …`; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Items ·
Parity raw / in scope before / after · Escalations · PR URL). Memory rules apply.

## 6. Work autonomously, do not ask questions, do not widen the scope.
