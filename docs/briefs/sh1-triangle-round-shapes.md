# SH1 — additional 2D shapes, part 1: triangle and round shapes (parity with parry / rapier)

## 1. Read first
`AGENTS.md` (§7: hot value structs stay compact); `docs/PLAN.md` (programme decisions: parity is welcome **unless it
costs Cairo steps** — here: nothing may change for the existing shapes); `docs/adr/0001-upstream-divergences.md`
(entries 13–17: convex polygon construction, analytic projection, SAT contacts, `Shape::ConvexPolygon` boxed so `Shape`
stays six felts); `docs/API_PARITY.md` (owner tables `Triangle`, `TriangleOrientation`, `TrianglePointLocation`,
`RoundShape`, `RoundTriangle`, `ColliderBuilder` (`triangle`, `round_cuboid`, `round_triangle`, `round_convex_hull`,
`round_convex_polyline`, …), `Shape` (`as_triangle`, `as_round_*`, …) — the exact items); on `main`:
`crates/rapier_geometry2d/src/{shape.cairo,shape/**,dispatch.cairo,dispatch/**,contact_generators.cairo,contact_generators/**,point/**,ray/**,query.cairo,query/**,aabb.cairo,mass.cairo}`,
`crates/rapier_dynamics2d/src/collider/builder.cairo`; CP1 / CP2 (#105, #107) as the precedent for adding a shape.
Upstream (`UP=/home/claude/git/refs`): `$UP/parry/src/shape/{triangle.rs,round_shape.rs}`, the triangle and round-shape
cases of `$UP/parry/src/query/**` (contact manifolds: PFM–PFM with border radii; point / ray; distance / contact),
`$UP/rapier/src/geometry/collider.rs` (builders).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/{shape.cairo,shape/**,dispatch.cairo,dispatch/**,contact_generators.cairo,contact_generators/**,point.cairo,point/**,ray.cairo,ray/**,query.cairo,query/**,aabb.cairo,aabb/**,mass.cairo}`,
`crates/rapier_dynamics2d/src/collider/builder.cairo` (+ sibling files if it passes 800 lines), their tests, the
harness `tools/golden/src/**` (new families `triangle_contacts`, `round_shape_contacts`, point / ray / mass cases;
**existing vectors byte-identical**, and new cases in NEW generated files so that no existing table grows), vectors /
generated fixtures / types (additions only), `tools/golden/README.md`, `scripts/api_parity.py` (entries you justify),
`docs/API_PARITY.md` (regenerated), and the snapshots that move. Forbidden: solver, narrow phase, pipeline, `World`,
`Scarb.toml`/`lib.cairo`.

## 3. Expected result
- **`Triangle`** (`a`, `b`, `c`, orientation, `normal`, point location, …) and **round shapes** (`RoundShape` with a
  `border_radius` over a cuboid, a triangle or a convex polygon), as new `Shape` variants **boxed** like
  `ConvexPolygon`, so that `Shape` keeps its six-felt representation.
- Every query the other shapes have, for the new ones: AABB, mass properties (round shapes: parry's formula), point
  projection / ray cast, `intersection_test`, contact manifolds against every shape of the closed set (triangle: the
  convex-polygon path is acceptable if it gives parry's manifolds within the documented bands; round shapes: the inner
  shape's manifold with the border radii added to the prediction and the contact offsets, as parry does), `distance` /
  `contact` / `closest_points` (QY1's API).
- `ColliderBuilder::{triangle, round_cuboid, round_triangle, round_convex_hull, round_convex_polyline}` and the
  `Shape::as_*` accessors; everything else of the tables: implemented, excluded through a closed reason, or `missing`
  with a reason.

## 4. Constraints, golden and efficiency
**Existing shapes unchanged in Cairo steps:** every P3 scene, level window, golden replay and contact-generator probe of
the existing shapes keeps its exact Cairo steps (measure before / after with `--tracked-resource cairo-steps`); Sierra
gas may move by the new match arms (path-insensitive), ≤ +1 %, explained; `gas/bytecode.size` regenerated. Golden:
new families from parry2d-f64 0.30.2 (triangle and round-shape pairs × {separated, touching, overlapping} for
manifolds; point / ray / mass cases), strict or within documented bands (the analytic / SAT divergences of ADR 14 / 17
apply). Measure the new generators (gas and exact steps per pair, `gas_*` probes, losers under `alternatives`).
≤ 800 lines per file; ≤ 4 fuzz per module.

## 5. Tests
The golden families; table-driven unit tests; existing tests unchanged; `python3 scripts/api_parity.py --check`.

## 6. Definition of done
Harness twice → zero diff. Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through
`scripts/build-shims/`): `scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on
rapier_geometry2d, rapier_golden, rapier_dynamics2d and rapier2d (one at a time); snapshots with `snforge test -p
<crate> --tracked-resource sierra-gas > gas-<crate>.log` and `python3 scripts/gas.py snapshot --filter
<crate>::<module> --from-log gas-<crate>.log`; `python3 scripts/bytecode_size.py snapshot`; `python3
scripts/api_parity.py` then `--check`. Never a workspace-wide run: CI is the full gate. Rebase on `origin/main` before
the PR. Conventional commits + trailer; push; `gh pr create --base main --title "<what ships>" --body-file …`; wait
for the checks to be registered, then `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Items
closed / excluded / left · API · Golden results and bands · Steps unchanged proof (before / after table) · Gas | steps
of the new generators · Deviations · Requested re-exports · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
