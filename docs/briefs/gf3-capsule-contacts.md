# GF3 — contact generators: capsule–capsule and cuboid–capsule

## 1. Read first
`AGENTS.md`; `docs/interfaces/geometry-dynamics.md` §2 and `crates/rapier_geometry2d/src/contact.cairo`;
`docs/research/02-parry-analysis.md` §4.1 (the analytic cuboid–capsule generator exists upstream
but is commented out of the dispatcher — this project re-enables it); `tools/golden/README.md`
(capsule quirks: crossing segments give a noisy normal or a `+Y` fallback); on `main`:
`crates/rapier_geometry2d/src/{closest_points.cairo,sat.cairo,polygonal_feature.cairo,shape.cairo,manifold.cairo}`
(GC `closest_points_segment_segment_with_locations`, GD `cuboid_support_map_find_local_separating_normal_oneway`
and `PolygonalFeature`, GB `Capsule`, GE persistence). Upstream
(`UP=/home/claude/git/refs`):
`$UP/parry/src/query/contact_manifolds/{contact_manifolds_capsule_capsule.rs,contact_manifolds_cuboid_capsule.rs}`
(read the 2D `cfg(feature = "dim2")` branches), `$UP/parry/src/shape/capsule.rs` (`to_polyline`?
no — `local_support_point`, `segment`). Golden: `rapier_golden::contact_manifolds` capsule–capsule,
cuboid–capsule (+ flipped) cases.

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/contact_generators/capsule_capsule.cairo`,
`crates/rapier_geometry2d/src/contact_generators/cuboid_capsule.cairo`,
`crates/rapier_geometry2d/tests/contact_capsule_golden.cairo`,
`gas/rapier_geometry2d/contact_generators.capsule_capsule.snap` and
`…/contact_generators.cuboid_capsule.snap` (filters `…::capsule_capsule`, `…::cuboid_capsule` only), `gas/rapier_geometry2d_integrationtest/contact_capsule_golden.snap`. Modules pre-declared.

## 3. Expected API and semantics
```cairo
pub fn contact_manifold_capsule_capsule(pos12: Pose2, capsule1: Capsule, capsule2: Capsule, prediction: Fixed, ref manifold: ContactManifold);
pub fn contact_manifold_cuboid_capsule(pos12: Pose2, cuboid1: Cuboid, capsule2: Capsule, prediction: Fixed, ref manifold: ContactManifold);
```
plus the `_shapes` wrappers (`-> bool`, both argument orders handled as upstream: cuboid–capsule
and capsule–cuboid with the flip). Capsule–capsule: upstream 2D algorithm (closest points of the
two core segments, then the two-point manifold when the segments are near-parallel: reproduce
upstream's parallelism test and the second point construction, radii subtracted from `dist`,
feature ids as upstream). Cuboid–capsule: upstream's analytic generator (SAT of cuboid vs the
capsule support map, then `PolygonalFeature` clip of the cuboid face against the capsule
segment, radius subtracted), normals/points per upstream. Fast path `try_update_contacts` first.
DEFER: compound handling, `ContactManifoldsWorkspace`, the pfm–pfm/GJK fallback.

## 4. Efficiency and variants
One `normalize2` per call; wide comparisons for the parallelism test. Bench each generator over
the 6 regimes; compare the near-parallel second-point construction as (a) upstream and (b) a
clip-based construction reusing GD's `clip_segment_segment` (ship the cheaper one if goldens are
bit-identical, else keep it under `mod alternatives`).

## 5. Tests
Golden: every capsule–capsule and cuboid–capsule case (num_points, points, dist, normals within
tolerance; fids exact; `ambiguous` only num_points and dist). Table-driven: parallel stacked
capsules (2 points), crossing capsules (1 point), capsule lying on a cuboid face (2 points),
capsule end on a cuboid corner, separated within prediction. `fuzz_*` ≤ 4; `gas_*` per
function/candidate. ≤ 800 lines/file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); snapshot filters above; conventional commits + trailer;
push; `gh pr create` per template; `gh pr checks --watch` until green; never merge; `REPORT.md`
(Summary · API · Gas table · Deviations · Deferred · Requested re-exports · Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
