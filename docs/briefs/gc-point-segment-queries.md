# GC — `rapier_geometry2d::{point, closest_points}`: point projection and segment–segment

## 1. Read first
`AGENTS.md`; `docs/PLAN.md`; `docs/research/02-parry-analysis.md` §5.6, §6 (numeric hazards);
`crates/rapier_math/src/consts.cairo` and `math_ext/norm2.cairo` (wide comparisons — use them for
every squared-length test); style precedents: `crates/rapier_math/src/math_ext/vec2.cairo`.
Upstream (`UP=/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs`):
`$UP/parry/src/query/point/{point_ball,point_cuboid,point_capsule,point_segment,point_halfspace,point_query.rs}`,
`$UP/parry/src/query/closest_points/closest_points_segment_segment.rs`,
`$UP/parry/src/shape/segment.rs` (`SegmentPointLocation`), `$UP/parry/src/query/point/point_query.rs`
(`PointProjection`). Golden vectors: `rapier_golden::point_projection` (33 cases),
`rapier_golden::segment_segment` (24 pairs, `ambiguous` flag for parallel cases).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/point.cairo` (+ `src/point/*.cairo`),
`crates/rapier_geometry2d/src/closest_points.cairo`,
`crates/rapier_geometry2d/tests/{point_golden,segment_segment_golden}.cairo`,
`gas/rapier_geometry2d/{point,closest_points}.snap`,
`gas/rapier_geometry2d_integrationtest/{point_golden,segment_segment_golden}.snap`.
`lib.cairo` already declares both modules. Shape structs come from GB (`rapier_geometry2d::shape`);
if GB is not merged when you start, define minimal private copies of `Ball/Cuboid/Capsule/Segment/HalfSpace`
in `point/shapes_shim.cairo` with the exact field names of the GB brief and list the swap under
"Escalations".

## 3. Expected API and semantics
```cairo
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct PointProjection { pub is_inside: bool, pub point: Vec2 }
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub enum SegmentPointLocation { OnVertex: u32, OnEdge: (Fixed, Fixed) }   // barycentric as upstream
```
`project_local_point_ball/cuboid/capsule/segment/halfspace(shape, pt, solid: bool) -> PointProjection`,
`project_local_point_and_get_location_segment(seg, pt, solid) -> (PointProjection, SegmentPointLocation)`,
`project_local_point_and_get_feature_cuboid(...) -> (PointProjection, FeatureId)` (f32 feature
semantics), `distance_to_local_point`, `contains_local_point` per shape;
`closest_points_segment_segment(seg1, seg2) -> (Vec2, Vec2)` and
`closest_points_segment_segment_with_locations(seg1, seg2) -> (SegmentPointLocation, SegmentPointLocation)`
(upstream's Ericson-style algorithm with its degenerate branches: zero-length segments, parallel
segments — deterministic tie-break documented). DEFER: support-map projection (GJK), ray casts.

## 4. Efficiency and variants
Every `length_squared` comparison goes through `rapier_math::math_ext::norm2` wide helpers; every
division is deferred to one `Recip`/`normalize2` at the end. For the cuboid projection and the
segment–segment core, implement the straightforward port and a branch-reduced variant
(clamp-based, `min/max` instead of `if`), measure, ship the winner.

## 5. Tests
Golden: every `rapier_golden::point_projection` case (point, `is_inside`, location/feature where
present) and every non-`ambiguous` `rapier_golden::segment_segment` pair (both points and the
squared distance) within the README tolerances; for `ambiguous` pairs check only the distance.
Table-driven edge cases: point exactly on a vertex/edge, inside with `solid = false`, tiny and
huge coordinates (wide comparisons must not overflow). `fuzz_*` ≤ 4 per module; `gas_*` for
every public function and candidate. ≤ 800 lines per file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); `python3 scripts/gas.py snapshot --filter
rapier_geometry2d::point`, `… ::closest_points`, `… rapier_geometry2d_integrationtest::point_golden`,
`… ::segment_segment_golden`; conventional commits + trailer; push; `gh pr create` per template;
`gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · API · Gas table ·
Deviations · Deferred · Requested re-exports · Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
