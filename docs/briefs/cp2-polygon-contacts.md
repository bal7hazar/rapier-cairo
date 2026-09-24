# CP2 — contact generators for convex polygons (phase 2, wave 7)

## 1. Read first
`AGENTS.md` (§7 gas cost model); `docs/PLAN.md` (GF1–GF4, GG, GM/CL dispatch — `dispatch::contact_manifold`
(metered) and `contact_manifold_step` (inlined in the pair loop) share one table; CP1 findings);
`docs/adr/0001-upstream-divergences.md`; `docs/interfaces/geometry-dynamics.md` §2 (manifold invariants);
on `main`: `crates/rapier_geometry2d/src/shape/convex_polygon.cairo` (CP1: `ConvexPolygon`, `support_feature`,
normals, `Shape::ConvexPolygon(Box<…>)`), `contact_generators/{cuboid_cuboid,cuboid_segment,cuboid_capsule,capsule_capsule,convex_ball,halfspace_pfm}.cairo`
(SAT + `PolygonalFeatureTrait::contacts` clipping precedents — CP1 already routes polygon–ball and
polygon–half-space through `convex_ball` / `halfspace_pfm`), `sat.cairo`, `polygonal_feature.cairo`, `clip.cairo`,
`dispatch.cairo` (+ `dispatch/*`), `tests/{dispatch_golden,contact_*_golden}.cairo`, `tools/golden/src/{manifolds.rs,shapes.rs,cairo.rs,cairo/*}`.
Upstream (`UP=/home/claude/git/refs`): `$UP/parry/src/query/contact_manifolds/contact_manifolds_pfm_pfm.rs`
(upstream's route for polygon pairs: GJK/EPA + polygonal-feature clipping), `$UP/parry/src/query/sat/`,
`$UP/parry/src/query/default_query_dispatcher.rs`.

## 2. Scope (file allowlist)
New generator modules are declared by the orchestrator: `crates/rapier_geometry2d/src/contact_generators/polygon_polygon.cairo`
(polygon–polygon and polygon–cuboid) and `polygon_segment.cairo` (polygon–segment and polygon–capsule) are
pre-declared (empty); `crates/rapier_geometry2d/src/{dispatch.cairo,dispatch/*}` (new arms in both tables),
`crates/rapier_geometry2d/src/sat.cairo` (new helpers only), a new test file `crates/rapier_geometry2d/tests/contact_polygon_golden.cairo`,
`crates/rapier_geometry2d/tests/dispatch_golden.cairo`, the harness `tools/golden/src/**` (polygon pairs in the
`contact_manifolds` family; existing cases byte-identical), `tools/golden/vectors/contact_manifolds.json` (additions
only), generated fixtures, `crates/rapier_golden/src/types.cairo` (additions only), `crates/rapier_golden/tests/sanity.cairo`
(case counts), `tools/golden/README.md`, and the snapshots that move. Forbidden: everything else.

## 3. Expected semantics
Every pair of the closed shape set involving a polygon produces a manifold (≤ 2 points, 2D), with upstream's
observable behaviour: normal from shape 1 to shape 2, points within prediction kept like upstream, feature ids
of the polygon = CP1's face/vertex ids, `try_update_contacts` fast path and `match_contacts` like the other
generators. Algorithm: SAT over both polygons' face normals (the cuboid and segment as polygons of 4 / 2
vertices) + `PolygonalFeatureTrait::contacts` clipping — no GJK/EPA (ADR entry 5 pattern); document where the
result differs from upstream's GJK/EPA path (ties, deep penetration where EPA is approximate — CP1 found an EPA
distance of −1.5 where the exact value is −1). Both argument orders (flipped pairs as the existing generators do).

## 4. Golden and efficiency
Harness: polygon pairs (polygon–polygon, polygon–cuboid, polygon–segment, polygon–capsule, and the flipped
orders) × the six regimes of the family (separated, within prediction, touching, shallow, deep, degenerate),
`ambiguous` tags on exact ties; compare within the README tolerances. **No regression on existing scenes**:
the 13 P3 scenes ≤ +1 % Sierra gas and exact steps (both dispatch tables gain arms — if an outlined match got
dearer, fix it). Report per-pair costs (gas | steps). Losers under `mod alternatives`; ≤ 4 fuzz per module;
≤ 16 probes; ≤ 800 lines per file.

## 5. Tests
Golden manifolds; table-driven regime tests per pair; the dispatch golden covers the new pairs in both orders.

## 6. Definition of done
Harness twice → zero diff (cargo takes no lock); foreground Cairo gate (tool timeout 3600000 ms; crate-scoped runs
via `scripts/build-shims/snforge -p …` while iterating); `gas.py check` then module-filtered snapshots; conventional
commits + trailer; push; `gh pr create` per template; wait for the checks to be registered, then `gh pr checks
--watch` until green; never merge; `REPORT.md` (Summary · API · Divergences (SAT vs GJK/EPA) · Golden results · Gas |
steps incl. the 13-scene table · Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
