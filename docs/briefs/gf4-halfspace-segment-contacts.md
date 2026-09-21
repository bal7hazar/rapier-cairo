# GF4 — contact generators: halfspace–polygonal-feature-map and cuboid–segment

## 1. Read first
`AGENTS.md`; `docs/interfaces/geometry-dynamics.md` §2 and `crates/rapier_geometry2d/src/contact.cairo`;
`tools/golden/README.md` (halfspace degenerate normal pointing into the solid when a ball centre is
exactly on the boundary — that case belongs to GF1's convex–ball; here: halfspace vs cuboid/segment/capsule);
on `main`: `crates/rapier_geometry2d/src/{polygonal_feature.cairo,sat.cairo,clip.cairo,shape.cairo,manifold.cairo}`
(GD `PolygonalFeature`, `cuboid_segment_find_local_separating_normal_oneway`,
`segment_cuboid_find_local_separating_normal_oneway`; GB `Cuboid::support_feature`,
`Segment`, `HalfSpace`). Upstream
(`UP=/home/claude/git/refs`):
`$UP/parry/src/query/contact_manifolds/contact_manifolds_halfspace_pfm.rs` (`local_support_feature`
of the pfm toward `-normal`, one `TrackedContact` per vertex of the feature with `dist = normal·p`,
`fid1 = UNKNOWN`, `fid2` = the feature's vertex id), `$UP/parry/src/shape/polygonal_feature_map.rs`
(`local_support_feature` for cuboid, segment, capsule), and for cuboid–segment the pfm–pfm path
specialised to 2D via SAT + clip (the `contact_manifolds_pfm_pfm.rs` sequence with GD's
cuboid–segment SAT instead of GJK). Golden: `rapier_golden::contact_manifolds` halfspace–cuboid,
halfspace–ball is GF1's, plus any cuboid–segment cases present; if a needed pair has no golden
case, derive expected values analytically in the test and say so in the report.

## 2. Scope (file allowlist)
`crates/rapier_geometry2d/src/contact_generators/halfspace_pfm.cairo`,
`crates/rapier_geometry2d/src/contact_generators/cuboid_segment.cairo`,
`crates/rapier_geometry2d/tests/contact_halfspace_golden.cairo`,
`gas/rapier_geometry2d/contact_generators.halfspace_pfm.snap` and
`…/contact_generators.cuboid_segment.snap` (filters `…::halfspace_pfm`, `…::cuboid_segment` only), `gas/rapier_geometry2d_integrationtest/contact_halfspace_golden.snap`. Modules pre-declared.

## 3. Expected API and semantics
```cairo
pub fn contact_manifold_halfspace_pfm(pos12: Pose2, halfspace1: HalfSpace, pfm2: Shape, prediction: Fixed, ref manifold: ContactManifold, flipped: bool);
pub fn contact_manifold_halfspace_pfm_shapes(pos12: Pose2, shape1: Shape, shape2: Shape, prediction: Fixed, ref manifold: ContactManifold) -> bool;
pub fn contact_manifold_cuboid_segment(pos12: Pose2, cuboid1: Cuboid, segment2: Segment, prediction: Fixed, ref manifold: ContactManifold);
pub fn contact_manifold_cuboid_segment_shapes(...) -> bool;
```
`pfm2 ∈ {Cuboid, Segment, Capsule}` (capsule = segment feature with the radius subtracted from
`dist`, as upstream's `PolygonalFeatureMap for Capsule`); handle both argument orders through the
`flipped` convention upstream uses (swap normals/points/fids). Keep points whose `dist <=
prediction`? — mirror upstream exactly and state which comparison it uses. Fast path
`try_update_contacts` first where upstream does it. DEFER: `ConvexPolygon` pfm, GJK fallback,
compound handling.

## 4. Efficiency and variants
Halfspace contacts are the floor of every game: zero divisions, no `normalize2` (the halfspace
normal is unit by construction — assert it in debug tests only), two `dot2_add` per point. Bench
per pfm shape; compare (a) upstream's generic `local_support_feature` dispatch with (b) a direct
per-shape path; ship the winner.

## 5. Tests
Golden where cases exist; analytic otherwise (box resting flat on a halfspace: 2 points, dist =
-penetration, normal = halfspace normal; box tilted: 1 point; capsule flat: 2 points with radius
subtracted; segment crossing the plane: 1 point below, the other above with dist > 0). `fuzz_*`
≤ 4; `gas_*` per function/candidate. ≤ 800 lines/file.

## 6. Definition of done
Foreground gate (`scarb fmt --workspace && scarb lint --workspace --deny-warnings && scarb build
--workspace && snforge test --workspace`); snapshot filters above; conventional commits + trailer;
push; `gh pr create` per template; `gh pr checks --watch` until green; never merge; `REPORT.md`
(Summary · API · Gas table · Deviations · Deferred · Requested re-exports · Escalations · PR URL).

## 7. Work autonomously, do not ask questions, do not widen the scope.
