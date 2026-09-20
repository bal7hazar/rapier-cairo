# GH — consolidate the wave-3 shims of `rapier_geometry2d`

## 1. Read first
`AGENTS.md`; the "Escalations" sections quoted here (from the GB, GC and GD reports):
GB shipped `shape::aabb_shim::{Aabb, AabbTrait, absolute_transform_vector, segment_aabb}` because
GA was not merged; GD shipped `sat::shims` (private copies of `Ball/Cuboid/Capsule/Segment/HalfSpace`)
and its own `PolygonalFeature` while GB shipped `SupportFeature` with the same fields; GC may
have shipped `point::shapes_shim`. All of GA, GB, GC, GD are now merged on `main`. Read on
`main`: `crates/rapier_geometry2d/src/{aabb.cairo,shape.cairo,shape/*.cairo,sat.cairo,sat/shims.cairo,polygonal_feature.cairo,point.cairo,closest_points.cairo}`
and the tests under `crates/rapier_geometry2d/tests/`.

## 2. Scope (file allowlist)
Everything under `crates/rapier_geometry2d/src/` **except** `lib.cairo`, `contact.cairo`,
`feature_id.cairo`, `mass.cairo` (frozen struct; you may edit the impls below it if a shim is
referenced there), `manifold.cairo`, `broad_phase.cairo`, `contact_generators/**`, `dispatch.cairo`;
every file under `crates/rapier_geometry2d/tests/`; the corresponding `gas/rapier_geometry2d/*.snap`
and `gas/rapier_geometry2d_integrationtest/*.snap` of the modules you touch.

## 3. Expected outcome
- `shape/aabb_shim.cairo`, `sat/shims.cairo` and any `point/shapes_shim.cairo` are deleted;
  every use points to `crate::aabb::{Aabb, AabbTrait}` and `crate::shape::*`.
- GB's `absolute_transform_vector` logic lives in `Aabb::transform_by` (GA already ships the
  absolute-rotation formula: keep GA's, delete GB's copy) and `segment_aabb` moves into
  `Segment`'s `compute_local_aabb`/`compute_aabb` if not already equivalent.
- One polygonal feature type: `polygonal_feature::PolygonalFeature` (GD). `Cuboid::support_feature`
  returns it; `SupportFeature` is removed (or kept only as a type alias if a test needs the name).
- No semantic change: every golden test keeps passing with identical numbers; the gas of the
  affected functions must not increase (compare `gas.py diff` before/after — a decrease from
  removing duplicated conversions is welcome, an increase must be explained in the report).
- Re-export needs (GB asked `pub use shape::{Shape, ShapeTrait, ShapeType}; pub use mass::MassPropertiesTrait;`)
  go under "Requested re-exports": `lib.cairo` is orchestrator-owned.

## 4. Efficiency
No new candidates; this is a refactor. Report the gas delta table (expected: zero or negative).

## 5. Tests
No new tests required; all existing tests must pass unchanged. Compile budget unchanged (≤ 800
lines per file).

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); regenerate the snapshots of the touched modules only
(`python3 scripts/gas.py snapshot --filter rapier_geometry2d::<module>` per module, plus the
integration-test modules whose files you touched); conventional commits + trailer; push;
`gh pr create` per template; `gh pr checks --watch` until green; never merge; `REPORT.md`
(Summary · API changes · Gas delta table · Deviations · Deferred · Requested re-exports ·
Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
