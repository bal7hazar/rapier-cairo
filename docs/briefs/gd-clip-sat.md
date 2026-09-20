# GD — `rapier_geometry2d::{clip, sat, polygonal_feature}`: clipping and 2D SAT

## 1. Read first
`AGENTS.md`; `docs/PLAN.md`; `docs/research/02-parry-analysis.md` §5.3–5.4 and §6;
`crates/rapier_math/src/consts.cairo` + `math_ext/norm2.cairo` (wide comparisons); style precedents:
`crates/rapier_math/src/math_ext/vec2.cairo`. Upstream
(`UP=/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs`):
`$UP/parry/src/query/clip/clip_segment_segment.rs`, `$UP/parry/src/query/sat/{sat_cuboid_cuboid,sat_cuboid_segment,sat_cuboid_point,sat_support_map_support_map}.rs`,
`$UP/parry/src/shape/polygonal_feature2d.rs` (`PolygonalFeature`: 2 vertices, feature ids,
`clip`, `contacts`), `$UP/parry/src/shape/cuboid.rs` (`support_feature`, `support_face`).
Golden vectors: `rapier_golden::sat2d` (22 cases, both directions, weighted-diagonal and tie
behaviours documented in `tools/golden/README.md`), `rapier_golden::clip2d` (16 cases incl. the
point-order quirk of `clip_segment_segment_with_normal`).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/clip.cairo`, `crates/rapier_geometry2d/src/sat.cairo` (+ `src/sat/*.cairo`),
`crates/rapier_geometry2d/src/polygonal_feature.cairo`,
`crates/rapier_geometry2d/tests/{clip_golden,sat_golden}.cairo`,
`gas/rapier_geometry2d/{clip,sat,polygonal_feature}.snap`,
`gas/rapier_geometry2d_integrationtest/{clip_golden,sat_golden}.snap`. `lib.cairo` already declares
the three modules. Shapes come from GB; if not merged when you start, use private shims with the
GB field names and list the swap under "Escalations".

## 3. Expected API and semantics
`clip_segment_segment(a1, b1, a2, b2) -> Option<((Vec2, Vec2), (Vec2, Vec2))>` and
`clip_segment_segment_with_normal(seg1, seg2, normal)` with upstream's exact outputs and point
order; `PolygonalFeature { vertices: [Vec2; 2], vids: [FeatureId; 2], fid: FeatureId, num_vertices: u8 }`
with `transform_by(pose)`, `clip(self, other, normal, prediction, ref manifold: ContactManifold)`
producing `TrackedContact`s (dist, fids) exactly as `polygonal_feature2d.rs` does;
`cuboid_cuboid_find_local_separating_normal_oneway(c1, c2, pos12) -> (Fixed, Vec2)`,
`cuboid_cuboid_compute_separation_wrt_local_line`, `cuboid_segment_find_local_separating_normal_oneway`,
`cuboid_support_map_find_local_separating_normal_oneway` (for capsules), `point_cuboid_find_local_separating_normal_oneway`
— same names, same axis order and tie-breaking as upstream (first axis tested wins), the
weighted-diagonal behaviour reproduced, not "fixed". DEFER: GJK/EPA, 3D SAT, triangle cases.

## 4. Efficiency and variants
Support scans are the hot loop: unroll over the 4 cuboid vertices/2 axes (no loops); squared
comparisons wide; one `normalize2` at most per call. Bench the separation search as (a) direct
port and (b) branch-reduced with `min/max`; ship the winner.

## 5. Tests
Golden: every `rapier_golden::sat2d` case (axis and separation, both directions, tolerances per
README) and every `rapier_golden::clip2d` case (points + feature ids, exact order); table-driven
degenerate inputs (parallel edges, corner–corner, zero-length segment must not panic — return
`None`/upstream's result). `fuzz_*` ≤ 4 per module; `gas_*` per public function and candidate.
≤ 800 lines per file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); `python3 scripts/gas.py snapshot --filter
rapier_geometry2d::clip`, `… ::sat`, `… ::polygonal_feature`, `… rapier_geometry2d_integrationtest::clip_golden`,
`… ::sat_golden`; conventional commits + trailer; push; `gh pr create` per template; `gh pr checks
--watch` until green; never merge; `REPORT.md` (Summary · API · Gas table · Deviations · Deferred ·
Requested re-exports · Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
