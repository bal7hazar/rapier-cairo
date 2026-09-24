# CP1 — convex polygon shape: data, AABB, mass, point projection, ray cast (phase 2, wave 7)

## 1. Read first
`AGENTS.md` (§7 gas cost model — adding an enum variant adds an arm to every `match` on `Shape`: an
outlined match is charged its costliest arm, so measure every scene); `docs/PLAN.md` (D6 closed shape enum,
phase 2 table; OP/ON/CL findings on inlined dispatch); `docs/adr/0001-upstream-divergences.md`;
`docs/interfaces/geometry-dynamics.md`; on `main`: `crates/rapier_geometry2d/src/{shape.cairo,shape/*,aabb.cairo,mass.cairo,point.cairo,ray.cairo,ray/*,polygonal_feature.cairo,dispatch.cairo}`,
every `match` on `Shape` (≈ 50 sites: geometry generators and dispatch, `rapier_dynamics2d::collider::{builder,components}`,
`rapier2d::{queries,dispatcher,pipeline/islands}`), `tools/golden/src/{shapes.rs,point_projection.rs,mass.rs,ray_casts.rs,cairo.rs,cairo/*}`.
Upstream (`UP=/home/claude/git/refs`): `$UP/parry/src/shape/convex_polygon.rs` (construction from points —
`from_convex_polyline` / `from_convex_hull`, normals, `support_feature`/`polygonal_feature`),
`$UP/parry/src/mass_properties/mass_properties_convex_polygon.rs`, `$UP/parry/src/query/point/point_support_map.rs`
and `query/ray/ray_support_map.rs` (upstream uses GJK for both — the port does it analytically, see §3).

## 2. Scope (file allowlist)
**Interface change approved by the orchestrator:** `Shape` and `ShapeType` gain a `ConvexPolygon` variant
(D6 stays a closed enum). Allowlist: `crates/rapier_geometry2d/src/**`, `crates/rapier_geometry2d/tests/**`,
`crates/rapier_dynamics2d/src/collider/**` and `crates/rapier2d/src/**` **only** to add the new arm to existing
`match`es (no other change there), the Rust harness `tools/golden/src/**` (new shape spec + polygon cases in the
existing `point_projection`, `mass_properties`, `aabb`, `ray_casts` families; existing cases byte-identical),
`tools/golden/vectors/*.json` (additions only), `crates/rapier_golden/src/{types.cairo,generated.cairo,generated/*}`
(types: additions only; `ShapeRaw` may gain the polygon form), `tools/golden/README.md`, and the snapshots that move.
`crates/rapier_geometry2d/src/lib.cairo` and every `Scarb.toml` are the orchestrator's (list re-exports under
Escalations). Contact generation for polygons is lot CP2: in `dispatch`, every pair involving a polygon returns
`false` (unsupported) for now, except `convex_ball` / `halfspace_pfm` only if they work unchanged through the new
point projection / support feature — say which.

## 3. Expected API and semantics
`ConvexPolygon { vertices: [Vec2; 8], normals: [Vec2; 8], count: u8 }` (3 ≤ count ≤ 8, counter-clockwise),
`ConvexPolygonTrait::{from_convex_polyline(points) -> Option<ConvexPolygon>, vertices, normals, count,
support_point, support_feature / polygonal_feature (edge = face ids like the cuboid's, vertex ids), scaled?}`
with upstream's names; `compute_aabb`, `MassProperties::from_convex_polygon(density, …)` (upstream's
triangle-fan formula, same order of accumulation), point projection with features (analytic: nearest edge /
vertex, `is_inside` by all edge half-planes — document where it can differ from upstream's GJK: ties, the
reported feature), ray cast (analytic: clip the ray against the edge half-planes, entry normal, hollow exit —
avoid upstream's unit-mixing defect #4 of the ADR). `ColliderBuilder::convex_polygon(points)` in
`rapier_dynamics2d` if the builder has per-shape constructors (else escalate). Degenerate inputs (collinear,
duplicates, > 8 points, clockwise) → `None` as upstream's `from_convex_polyline` does or document.

## 4. Golden and efficiency
Harness: polygon cases (triangle, quad, pentagon, octagon; rotated; thin) in `point_projection`, `mass_properties`,
`aabb`, `ray_casts`, from parry f64 0.30.2 (GJK results: compare points/distances within tolerance, features
where meaningful, tag `ambiguous` ties). **No regression on existing scenes**: report the P3 net step for all 13
scenes before/after (Sierra gas and exact steps) — target ≤ +1 % everywhere; if an outlined match got dearer,
fix it (inline or meter the polygon arm). Report per-query costs for the polygon (gas | steps). ≤ 4 fuzz per
module (projection vs brute-force sampling of the boundary; ray vs half-plane clipping reference); ≤ 16 probes;
≤ 800 lines per file.

## 5. Tests
Golden comparisons; table-driven construction/degenerate cases; mass properties of polygon == cuboid when the
polygon is the same box; existing tests unchanged.

## 6. Definition of done
Harness twice → zero diff (cargo takes no lock); foreground Cairo gate (tool timeout 3600000 ms; crate-scoped
runs via `scripts/build-shims/snforge -p …` while iterating); `gas.py check` then module-filtered snapshots;
conventional commits + trailer; push; `gh pr create` per template; wait for the checks to be registered, then
`gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API · Divergences from upstream (GJK vs
analytic) · Golden results · Gas | steps incl. the 13-scene table · Requested re-exports · Escalations · PR URL).
Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
