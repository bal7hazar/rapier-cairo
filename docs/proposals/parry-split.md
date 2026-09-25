# Proposal — split the Parry port out of rapier-cairo into bal7hazar/parry-cairo

Status: **proposal, awaiting the owner's decision** (2026-09-25). Nothing is split before that decision.
Question relayed by the glam-cairo orchestrator: the Rust reference keeps parry (`parry2d`, `parry3d`)
separate from rapier; does rapier-cairo contain parry's role, and how would it split?

## 1. Where parry lives today

Yes: `crates/rapier_geometry2d` is the Parry port (PLAN §3 already says it "can be extracted as parry-cairo
later at no cost", and the DAG `testing ← math ← core ← geometry2d ← dynamics2d ← rapier2d` never lets
geometry depend on dynamics). About 15 k lines of source (tests excluded), 30+ gas snapshot modules, and
the golden families generated from `parry2d-f64`.

| Module (today, `rapier_geometry2d::…`) | Upstream home | After the split |
|---|---|---|
| `shape` (+ `shape/convex_polygon`), `aabb`, `mass` | parry `shape`, `bounding_volume`, `mass_properties` | **parry-cairo** |
| `point`, `ray`, `closest_points`, `clip`, `sat`, `polygonal_feature`, `feature_id` | parry `query::{point,ray,closest_points,clip,sat}`, `shape::{polygonal_feature_map,feature_id}` | **parry-cairo** |
| `manifold` (`try_update_contacts`, `match_contacts`), `contact_generators/*`, `dispatch` | parry `query::contact_manifolds`, `DefaultQueryDispatcher` | **parry-cairo** |
| `contact` — `TrackedContact`, `ContactManifold` | parry `ContactManifold<ManifoldData, ContactData>` (generic) | **parry-cairo**, made generic (see §3) |
| `contact` — `ContactManifoldData`, `ContactData`, `SolverContact`, `SolverFlags`, `NEW_CONTACT_BIT` | rapier `geometry/contact_pair.rs` | **rapier-cairo** (`rapier_dynamics2d`) |
| `broad_phase` (tail scan, strips, grid) | rapier `geometry/broad_phase_*` (parry only provides the BVH) | **rapier-cairo** (it uses rapier's collider `Handle`) |

Golden vectors: the parry families (`aabb`, `aabb_overlap`, `sat2d`, `clip2d`, `point_projection`,
`segment_segment`, `mass_properties`, `contact_manifolds`, `ray_casts`, `polygon_*`) and the vendored patched
`parry2d-f64` move with a parry-only harness; `scenes`, `integration_parameters` and the sleep/kinematic
diagnostics stay in rapier's harness. `rapier_testing::opaque` is needed on both sides (a copy, or a tiny
shared `cairo-testing` package later). `scripts/gas.py`, the build shims, the executor tooling and the CI
layout are copied.

## 2. What rapier would pin afterwards

`parry2d = "0.1.0"` from scarbs.xyz (like `fixed` / `glam`: registry versions only, a numeric change is a
MINOR bump, so pinning the version pins the golden vectors), plus `fixed`, `glam`, and `glamx` once it has
`Pose2`. `rapier_math` shrinks to what is rapier's own (`consts` for the solver, a few `math_ext` helpers) or
disappears.

## 3. Prerequisites, in order

1. **`glamx` gets `Pose2`** (glam-cairo's orchestrator; rapier's measured M2 kernels — `Rot2`/`Pose2`, fused,
   PR #21 — can be contributed). Without it parry-cairo would carry `Pose2` itself and the chain
   `fixed → glam → glamx → parry → rapier` would be broken on day one.
2. **Generic manifold, inside rapier-cairo first**: `ContactManifold<M, C>` (manifold payload, per-point
   payload) with rapier's `ContactManifoldData` / `ContactData` as the instantiation — an F3 interface change,
   landed and measured (monomorphisation: expected zero gas change, to be proven on the 16 P3 scenes) before
   anything moves.
3. **Move `rapier_geometry2d`'s broad phase into `rapier_dynamics2d`** (or keep a thin `rapier_geometry2d`
   for rapier-only geometry), so that the crate that leaves contains parry and only parry.
4. **Create bal7hazar/parry-cairo** with crate `parry2d` (history preserved with `git filter-repo` on the
   moved paths), its harness, CI and tooling; publish `parry2d 0.1.0`; rapier switches to the registry
   version in one PR whose gas delta must be zero (module paths in snapshot names change, values do not).

Estimated effort: 4 orchestrator PRs + 2 executor lots (generic manifold, broad-phase move); risk: snapshot
path churn and the golden harness split. Benefit: mirrors upstream, lets nalgebra-cairo / a future 3D port
reuse the geometry without rapier, and gives parry its own semver.

## 4. Recommendation

Split, but after phase 2 wave 8 and in the order of §3 — prerequisite 1 is on glam-cairo's side and should be
requested now. Until the owner decides, rapier-cairo keeps the geometry in-tree and new geometry work keeps
respecting the DAG (no dynamics import in `rapier_geometry2d`).
