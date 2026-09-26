# SH2a — additional 2D shapes, part 2a: polyline and 2D heightfield (composite shapes, multi-manifold pairs)

## 1. Read first
`AGENTS.md` (§7, incl. the SH1 lesson: new `match` arms can move old paths' Cairo steps; box large payloads);
`docs/PLAN.md` (programme order: SH2 after CC — level geometry variety; parity unless it costs Cairo steps);
SH1's report and code (#174) as the precedent for adding `Shape` variants; CC1 (#180) and CC2 (#182) for the casts /
CCD the new shapes must also answer; `docs/adr/0001-upstream-divergences.md`; `docs/API_PARITY.md` (owner tables
`Polyline`, `PolylineFlags`, `HeightField`, `Heightfield`, `HeightFieldCellStatus`, `ColliderBuilder`
(`polyline`, `heightfield`, …), `Shape::as_polyline*` / `as_heightfield*`, the composite query items — the exact list);
on `main`: `crates/rapier_geometry2d/src/{shape.cairo,shape/**,dispatch.cairo,dispatch/**,contact_generators.cairo,contact_generators/**,point/**,ray/**,query.cairo,query/**,aabb.cairo,aabb/**,mass.cairo}`,
`crates/rapier_dynamics2d/src/{narrow_phase.cairo,narrow_phase/**,collider/builder.cairo}`, `crates/rapier2d/src/pipeline/**`
(how pairs reach the solver and events). Upstream (`UP=/home/claude/git/refs`): `$UP/parry/src/shape/{polyline.rs,heightfield2.rs}`,
the composite cases of `$UP/parry/src/query/**` (contact manifolds `composite_shape_shape`, `heightfield_shape`, point /
ray / intersection / distance / casts on composites), `$UP/rapier/src/geometry/narrow_phase/**` (one `ContactPair`
holds several manifolds, one per sub-shape pair).

## 2. Scope (file allowlist)
The geometry files above (new shapes under `shape/**`, new generators, dispatch and query arms), `crates/rapier_dynamics2d/src/{narrow_phase.cairo,narrow_phase/**}`
(multi-manifold pairs), `crates/rapier_dynamics2d/src/collider/builder.cairo` (+ siblings), `crates/rapier2d/src/{pipeline.cairo,pipeline/**,world/state.cairo}`
only where pairs / events / state must follow, their tests, the harness `tools/golden/src/**` (new families; existing
vectors byte-identical, new cases in new generated files), vectors / fixtures / types (additions only),
`tools/golden/README.md`, `scripts/api_parity.py` (entries you justify), `docs/API_PARITY.md` (regenerated), and the
snapshots that move. Forbidden: the contact solver, the joint solver, `Scarb.toml`/`lib.cairo`.

## 3. Expected result
- **`Polyline`** (vertices, segment indices, flags) and the **2D `HeightField`** (heights, scale, cell status) as new
  **boxed** `Shape` variants (`Shape` keeps its six-felt width).
- Every query for them, parry's semantics, with sub-shape ids where upstream returns them: AABB, mass properties
  (upstream: zero-mass for these shapes as colliders of dynamic bodies? read it), point projection (with feature / sub-shape),
  ray cast (`RayIntersection::with_subshape` becomes meaningful: implement it if the field can be added without
  widening the existing results' Cairo steps; otherwise keep the id out-of-band and say how), `intersection_test`,
  `distance` / `contact` / `closest_points`, shape casts and the swept TOI against convex shapes.
- **Contacts against every convex shape of the closed set:** one manifold per touching sub-shape (segment / cell), as
  upstream. Preferred design, to be measured against alternatives: one narrow-phase entry per (collider pair, sub-shape
  pair) so that `ContactPair`, the solver and existing pairs are untouched, with collision / force events aggregated
  per collider pair exactly as upstream (one `Started` / `Stopped` per collider pair). Upstream prunes sub-shapes
  with a BVH: a deterministic AABB prefilter over the parts is acceptable; measure it for 10, 50, 200 segments.
- `ColliderBuilder::{polyline, heightfield, …}`, `Shape::as_polyline`, `as_heightfield`, and every other item of the
  tables: implemented, excluded through a closed reason, or `missing` with a reason (compound → SH2b).

## 4. Constraints, golden and efficiency
**Existing shapes and pairs unchanged in Cairo steps** (P3, level windows, golden replays, generator probes: before /
after table); Sierra gas ≤ +1 %, explained; `gas/bytecode.size` regenerated; `WorldState` version bumped only if its
layout changes (explain). Golden families from parry2d-f64 0.30.2 / rapier2d-f64 0.35.3 (the vendored copies):
polyline and heightfield vs every convex shape (manifolds per sub-shape, strict or within bands), queries, and a scene
(a box sliding / resting on a heightfield ground, a ball rolling along a polyline) with event traces. Measure the
per-step cost of a ball and of a box on a 10- and a 50-segment ground. ≤ 800 lines per file; ≤ 4 fuzz per module.

## 5. Tests
The golden families and scenes; table-driven unit tests; existing tests unchanged; `python3 scripts/api_parity.py --check`.

## 6. Definition of done
Harness twice → zero diff. Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through
`scripts/build-shims/`): `scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on
rapier_geometry2d, rapier_golden, rapier_dynamics2d and rapier2d (one at a time); snapshots with `--from-log`;
`python3 scripts/bytecode_size.py snapshot`; `python3 scripts/api_parity.py` then `--check`. Never a workspace-wide run:
CI is the full gate. Rebase on `origin/main` before the PR. Conventional commits + trailer; push; `gh pr create --base
main --title "<what ships>" --body-file …`; wait for the checks to be registered, then `gh pr checks --watch` until
green; never merge; `REPORT.md` (Summary · Items · API · Multi-manifold design and alternatives measured · Golden
results and bands · Steps unchanged proof · Costs of composite pairs · `WorldState` · Deviations · Requested
re-exports · Escalations · PR URL). Memory rules apply. Budget your turns: commit wip states early.

## 7. Work autonomously, do not ask questions, do not widen the scope.
