# GF2 — contact generator: cuboid–cuboid

## 1. Read first
`AGENTS.md`; `docs/interfaces/geometry-dynamics.md` §2 and `crates/rapier_geometry2d/src/contact.cairo`;
`docs/PLAN.md` wave-1 outcomes and `tools/golden/README.md` (SAT weighted-diagonal quirk, tie-break
on first axis, clipped points beyond prediction are kept, f32 feature ids); on `main`:
`crates/rapier_geometry2d/src/{sat.cairo,polygonal_feature.cairo,clip.cairo,shape.cairo,manifold.cairo}`
(GD's `cuboid_cuboid_find_local_separating_normal_oneway`, `PolygonalFeatureTrait::{transform_by,clip,contacts}`,
GB's `Cuboid::support_feature`/`support_face`, GE's `try_update_contacts`). Upstream
(`UP=/private/tmp/claude-501/-Users-bal7hazar-git-rapier-cairo--claude-worktrees-rapier-physics-cairo-benchmark-2d3317/239b7c62-97d7-47ea-9d79-74f81363555f/scratchpad/refs`):
`$UP/parry/src/query/contact_manifolds/contact_manifolds_cuboid_cuboid.rs` (100 lines: the
`try_update_contacts` fast path, the two-way SAT, the `support_feature` + `clip` + `contacts`
sequence, the `normal.flip` convention). Golden: `rapier_golden::contact_manifolds` cuboid–cuboid
cases (6 regimes + flipped order).

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/contact_generators/cuboid_cuboid.cairo`,
`crates/rapier_geometry2d/tests/contact_cuboid_golden.cairo`,
`gas/rapier_geometry2d/contact_generators.snap` (regenerate with `--filter
rapier_geometry2d::contact_generators::cuboid_cuboid` only), `gas/rapier_geometry2d_integrationtest/contact_cuboid_golden.snap`.
Modules are pre-declared. Everything else is forbidden.

## 3. Expected API and semantics
```cairo
pub fn contact_manifold_cuboid_cuboid(pos12: Pose2, cuboid1: Cuboid, cuboid2: Cuboid, prediction: Fixed, ref manifold: ContactManifold);
pub fn contact_manifold_cuboid_cuboid_shapes(pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold) -> bool;
```
Exact upstream sequence: (1) `if manifold.try_update_contacts(pos12) { return }`; (2) SAT in both
directions (`pos21 = pos12.inverse()`), pick the axis with the larger separation, early exit with
`manifold.clear()` when `sep > prediction`; (3) `support_feature` of both cuboids along ±normal,
transform feature 2 by `pos12`, `clip` then `contacts` into the manifold with the correct
`local_n1`/`local_n2`; (4) points beyond prediction are kept (upstream); feature ids as upstream
(f32 semantics from GB). DEFER: compound handling, `ContactManifoldsWorkspace`.

## 4. Efficiency and variants
Hot path of every stacking scene: keep the `try_update_contacts` fast path first; the SAT calls
are GD's. Bench the full generator on the 6 regimes; report the split between fast path, SAT and
clipping. Variant to compare: (a) upstream order, (b) skipping the second SAT direction when the
first already exceeds `prediction` (upstream does not; keep only if bit-identical results on all
goldens, else keep under `mod alternatives` with the reason).

## 5. Tests
Golden: every cuboid–cuboid case (num_points, points, dist, normals within README tolerance;
feature ids exact — the flipped-order cases too). Table-driven: face–face resting, corner on
face, deep penetration, separated within prediction (points present with dist > 0), fast-path hit
on a second call with a tiny motion (assert the manifold is updated, not regenerated, via
`num_points` and unchanged fids). `fuzz_*` ≤ 4; `gas_*` per function/candidate. ≤ 800 lines/file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); the snapshot filters above; conventional commits +
trailer; push; `gh pr create` per template; `gh pr checks --watch` until green; never merge;
`REPORT.md` (Summary · API · Gas table with the fast-path/SAT/clip split · Deviations · Deferred ·
Requested re-exports · Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
