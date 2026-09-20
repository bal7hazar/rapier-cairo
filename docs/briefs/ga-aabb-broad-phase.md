# GA — `rapier_geometry2d::{aabb, broad_phase}`

## 1. Read first
`AGENTS.md`; `docs/PLAN.md` (D7 stateless broad phase, D8 pair order, wave-1 outcomes on arena vs
arrays); `docs/interfaces/geometry-dynamics.md` §5; style precedents on `main`:
`crates/rapier_math/src/math_ext/norm2.cairo` (candidates + `gas_*`), `crates/rapier_core/src/data/arena.cairo`.
Upstream (read-only clone `UP=/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs`):
`$UP/parry/src/bounding_volume/aabb.rs` (`Aabb`, `intersects`, `merged`, `loosened`, `center`,
`half_extents`, `transform_by`, `contains_local_point`), `$UP/rapier/src/geometry/broad_phase_bvh/`
(only for the pair semantics: which pairs are reported, static–static skipped, `BroadPhasePairEvent`).
Golden vectors: `rapier_golden::aabb` (shape AABBs under poses — used by GB, here only for
`intersects`/`merged` sanity) and `rapier_golden::aabb_overlap` (sets → sorted overlapping pairs,
closed-interval convention documented in `tools/golden/README.md`).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/aabb.cairo`, `crates/rapier_geometry2d/src/broad_phase.cairo`
(+ submodules under `src/broad_phase/`), `crates/rapier_geometry2d/tests/aabb_overlap_golden.cairo`,
`gas/rapier_geometry2d/aabb.snap`, `gas/rapier_geometry2d/broad_phase.snap`,
`gas/rapier_geometry2d_integrationtest/aabb_overlap_golden.snap`. `lib.cairo` already declares
`pub mod aabb; pub mod broad_phase;`. Everything else is forbidden; needs go under "Escalations".

## 3. Expected API and semantics
Frozen inputs: `fixed::Fixed`, `glam::vec2::Vec2` (`min`, `max`, `abs`, operators, `mul_scalar`),
`rapier_math::pose2::Pose2` (M2: `transform_point`, `rotation.rotate`), `rapier_core::data::handle::Handle`.
```cairo
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default)] pub struct Aabb { pub mins: Vec2, pub maxs: Vec2 }
```
`AabbTrait`: `new(mins, maxs)`, `from_half_extents(center, half_extents)`, `center`, `half_extents`,
`extents`, `intersects(other) -> bool` (closed intervals, as upstream), `contains(other)`,
`contains_local_point(p)`, `merged(other)`, `loosened(margin)`, `tightened(margin)`,
`transform_by(pose)` (rotate the 4 corners; upstream formula with `abs` of the rotation matrix is
cheaper — implement that one, keep the corner version as the checked alternative),
`volume()` (area), `scaled`.
`broad_phase`: stateless, called every step with `Span<BroadPhaseProxy>` where
```cairo
#[derive(Copy, Drop, Serde, PartialEq, Debug)]
pub struct BroadPhaseProxy { pub collider: Handle, pub aabb: Aabb, pub is_static: bool }
```
`find_pairs(proxies: Span<BroadPhaseProxy>) -> Array<(u32, u32)>` returning index pairs
`(i, j)` with `i < j`, static–static pairs skipped, sorted ascending (D8: this order is part of the
state-transition function). Two candidates behind the same signature: (a) brute force O(n²) with
a static/dynamic split (dynamic×all only); (b) sort-and-prune on the x axis (sort proxies by
`mins.x`, sweep, test y only) — sort must be deterministic (stable, index tie-break). DEFER:
persistent BVH, pair events (add/remove diffing is DD's job), collision-group filtering (done by
the narrow phase with `rapier_core::interaction_groups`).

## 4. Efficiency and variants
Measure `find_pairs` for both candidates at n = 8, 16, 32, 64 with (i) all dynamic, (ii) 75 %
static, (iii) sparse (few overlaps) and (iv) dense (stack) layouts; report the crossover and
ship the cheaper one as `find_pairs`, the other under `mod alternatives` with its probes. Inner
overlap test: 4 comparisons on raw `i64`, no `Fixed` arithmetic. Sorting: implement an insertion
sort on a `Felt252Dict`-free `Array` (n ≤ 64) and note its cost; do not pull a generic sort crate.

## 5. Tests
Table-driven `test_*` for every `Aabb` method (edge-touching intervals count as overlap; empty
AABB), golden: every `rapier_golden::aabb_overlap` set → exact pair list from both candidates;
`fuzz_*` (fixed seed, ≤ 4 per module) equivalence brute-force vs sort-and-prune on random sets;
`gas_*` for every public function and both candidates at each size/layout (one `gas_baseline`
per test module, `rapier_testing::opaque` inputs). Compile budget: ≤ 800 lines per file.

## 6. Definition of done
Foreground gate from the worktree root: `scarb fmt --workspace && scarb lint --workspace
--deny-warnings && scarb build --workspace && snforge test --workspace`; then
`python3 scripts/gas.py snapshot --filter rapier_geometry2d::aabb`, `… --filter
rapier_geometry2d::broad_phase`, `… --filter rapier_geometry2d_integrationtest::aabb_overlap_golden`;
conventional commits with the trailer; push; `gh pr create` following
`.github/PULL_REQUEST_TEMPLATE.md`; `gh pr checks --watch` until green; never merge; `REPORT.md`
(Summary · API · Gas table incl. the crossover table · Deviations · Deferred · Requested re-exports ·
Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
