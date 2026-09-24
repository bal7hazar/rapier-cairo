# QP — scene queries: ray casts, point and AABB queries (phase 2, wave 6)

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D6 closed shape enum, D7 stateless broad phase, phase 2); `tools/golden/README.md`
(families, quantisation, tolerances); on `main`: `crates/rapier_geometry2d/src/{shape.cairo,shape/*,point.cairo,aabb.cairo,broad_phase.cairo}`
(GB shapes, GC point projection with features, GA/BG AABB and grid), `crates/rapier_math` (`Pose2`
kernels, `math_ext`), `crates/rapier2d/src/world.cairo`, `crates/rapier_dynamics2d/src/collider_set.cairo`.
Upstream (`UP=/home/claude/git/refs`): `$UP/parry/src/query/ray/{ray.rs,ray_ball.rs,ray_cuboid.rs,ray_aabb.rs,ray_capsule.rs,ray_halfspace.rs,ray_support_map.rs}`
(segment rays go through the support-map/segment path), `$UP/parry/src/query/point/`,
`$UP/rapier/src/pipeline/query_pipeline.rs` (`cast_ray`, `cast_ray_and_get_normal`, `intersect_ray`,
`project_point`, `intersect_point`, `intersect_aabb_conservative`, `QueryFilter`: exclude fixed/dynamic/
sensors, exclude a collider or body, interaction-group filter).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/ray.cairo` (+ `src/ray/*.cairo`; the module is pre-declared in `lib.cairo`),
`crates/rapier2d/src/queries.cairo` (+ `src/queries/*.cairo`; pre-declared), `crates/rapier2d/src/world.cairo`
(only to expose the query methods), new integration test files `crates/rapier_geometry2d/tests/ray_golden.cairo`
and `crates/rapier2d/tests/world_queries.cairo` (test files are auto-discovered), the Rust harness
`tools/golden/src/**` (new family `ray_casts`, wiring in `main.rs`/`cairo.rs`), `tools/golden/vectors/ray_casts.json`,
`crates/rapier_golden/src/types.cairo` (new types only), `crates/rapier_golden/src/generated.cairo` and
`generated/ray_casts.cairo` (both written by the harness), `tools/golden/README.md` (families table + a
`ray_casts` section), and the snapshots that move. `crates/rapier_golden/src/lib.cairo` re-exports are the
orchestrator's: use `rapier_golden::generated::ray_casts` and list the re-export under Escalations.
Forbidden: every other file.

## 3. Expected API and semantics
Geometry: `Ray { origin: Vec2, dir: Vec2 }`, `RayIntersection { time_of_impact: Fixed, normal: Vec2, feature: FeatureId }`,
per shape `cast_local_ray(shape, ray, max_time_of_impact, solid) -> Option<Fixed>` and
`cast_local_ray_and_get_normal(...) -> Option<RayIntersection>`, and the `Pose2` wrappers (`cast_ray`,
`cast_ray_and_get_normal`), all five shapes — upstream semantics exactly (solid vs hollow, rays starting
inside, parallel rays, zero-length direction, `max_time_of_impact` inclusive/exclusive as upstream).
World (rapier2d): `QueryFilter` (subset: flags exclude_fixed/kinematic/dynamic/sensors, exclude collider,
exclude rigid body, collision groups), `WorldTrait::{cast_ray, cast_ray_and_get_normal, intersect_ray
(returns all hits as an array, ascending collider handle), project_point, intersect_point, intersect_aabb}`
over the colliders, brute force first; measure a broad-phase-assisted variant (reuse BG's grid) for n ≥ 64.
Deterministic ties: equal time of impact → lowest collider handle.

## 4. Golden and efficiency
Harness family `ray_casts`: every shape × regimes (hit outside, grazing, from inside solid/hollow, parallel
miss, max-toi cut, degenerate direction), f64 parry `0.30.2` (the vendored patched copy is fine), quantised
inputs, raw outputs; tolerance in the README. Report Sierra gas and exact Cairo steps per shape ray cast and for
`World::cast_ray` at 8 / 32 / 128 colliders. Losers under `mod alternatives`; ≤ 4 fuzz per module; ≤ 16 probes;
≤ 800 lines per file.

## 5. Tests
Golden ray casts (time of impact within tolerance, normals, feature ids exact where upstream's are
meaningful); world query tests (filters, ties, sensors); fuzz: ray vs brute-force sampling where meaningful.

## 6. Definition of done
Harness twice → zero diff (under the heavy-build lock); foreground Cairo gate (tool timeout 3600000 ms);
`gas.py check` then module-filtered snapshots; conventional commits + trailer; push; `gh pr create` per
template; wait for checks, `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API ·
Golden results · Gas | steps · Deviations · Requested re-exports / module declarations · Escalations · PR URL).
Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
